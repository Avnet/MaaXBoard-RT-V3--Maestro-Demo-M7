/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stddef.h>
#include <string.h>
#include <math.h>
#include <main.h>

#include "fsl_debug_console.h"
#include "osa_memory.h"
#include "vit_proc.h"

#include "led_control.h"
#include "http_callbacks.h"
/* header for low power mode */
#include "lpm.h"
#include "streamer_pcm.h"

#define MODEL_LOCATION     VIT_MODEL_IN_ROM

#if (NUM_OF_CH==1)
#define VIT_OPERATING_MODE VIT_WAKEWORD_ENABLE | VIT_VOICECMD_ENABLE
#define NUMBER_OF_CHANNELS _1CHAN		// PDM0 - Mic1, Mic2,   PDM1 - Mic3, mic4
#define VIT_MIC1_MIC2_DISTANCE 0	// distance in mm between silkscreen MIC0[U11] and MIC2[U2]
#define VIT_MIC1_MIC3_DISTANCE 0	// distance in mm between silkscreen MIC0[U11] and MIC1[U1]
#elif (NUM_OF_CH==2)
#define VIT_OPERATING_MODE VIT_ALL_MODULE_ENABLE
#define NUMBER_OF_CHANNELS _2CHAN
#define VIT_MIC1_MIC2_DISTANCE 70
#define VIT_MIC1_MIC3_DISTANCE 0
#else
#define VIT_OPERATING_MODE VIT_ALL_MODULE_ENABLE
#define NUMBER_OF_CHANNELS _3CHAN
#define VIT_MIC1_MIC2_DISTANCE 70
#define VIT_MIC1_MIC3_DISTANCE 39
#endif



#ifdef PLATFORM_RT1060
#define DEVICE_ID VIT_IMXRT1060

#elif defined PLATFORM_RT1170
#define DEVICE_ID VIT_IMXRT1170

#else
#error "No platform selected"

#endif

#include "vit_proc.h"
#include "VIT_Model_en.h"
#include "VIT_Model_cn.h"
#include "VIT.h"
#include "PL_platformTypes_CortexM7.h"

#define MEMORY_ALIGNMENT 8 // in bytes

static VIT_Handle_t VITHandle = PL_NULL;      // VIT handle pointer
static VIT_InstanceParams_st VITInstParams;   // VIT instance parameters structure
static VIT_ControlParams_st VITControlParams; // VIT control parameters structure
static PL_MemoryTable_st VITMemoryTable;      // VIT memory table descriptor
static PL_BOOL InitPhase_Error        = PL_FALSE;
static VIT_DataIn_st VIT_InputBuffers = {PL_NULL, PL_NULL,
                                         PL_NULL}; // Resetting Input Buffer addresses provided to VIT_process() API
static PL_INT8 *pMemory[PL_NR_MEMORY_REGIONS];

/* Freertos task queue */
static QueueHandle_t *mp3_commandQ;
static QueueHandle_t *wifi_controlQ;
static queue_command_t voice_command = {.taskId=0};
static uint8_t wifi_command = 0;

extern void APP_SetWakeupConfig(void);

VIT_ReturnStatus_en VIT_ModelInfo(void)
{
    VIT_ReturnStatus_en VIT_Status;
    VIT_ModelInfo_st Model_Info;
    VIT_Status = VIT_GetModelInfo(&Model_Info);
    if (VIT_Status != VIT_SUCCESS)
    {
        PRINTF("VIT_GetModelInfo error: %d\r\n", VIT_Status);
        return VIT_INVALID_MODEL;
    }
    PRINTF("\n   \t*** VIT config BEGIN ***\r\n");

    PRINTF("\n   \tVIT Model info\r\n");
    PRINTF("   \tVIT Model Release: 0x%04x\r\n", Model_Info.VIT_Model_Release);
    if (Model_Info.pLanguage != PL_NULL)
    {
        PRINTF("   \tLanguage supported: %s \r\n", Model_Info.pLanguage);
    }
    PRINTF("   \tNumber of Commands supported: %d \r\n", Model_Info.NbOfVoiceCmds);

    if (!Model_Info.WW_VoiceCmds_Strings) // Check here if Model is containing WW and CMDs strings
    {
        PRINTF("   \tVIT_Model integrating WakeWord and Voice Commands strings: NO\r\n");
    }
    else
    {
        const char *ptr;

        PRINTF("   \tVIT_Model integrating WakeWord and Voice Commands strings: YES\r\n");
        ptr = Model_Info.pWakeWord;
        if (ptr != PL_NULL)
        {
            PRINTF("   \tWakeWord supported: %s \r\n", ptr);
        }
        PRINTF("   \tVoice commands supported: \r\n");
        ptr = Model_Info.pVoiceCmds_List;
        if (ptr != PL_NULL)
        {
            for (PL_UINT16 i = 0; i < Model_Info.NbOfVoiceCmds; i++)
            {
                PRINTF("    \t'%s' \r\n", ptr);
                ptr += strlen(ptr) + 1; // to consider NULL char
            }
        }
    }
    /*
     *   VIT Get Library information
     */
    VIT_LibInfo_st Lib_Info;
    VIT_Status = VIT_GetLibInfo(&Lib_Info);
    if (VIT_Status != VIT_SUCCESS)
    {
        PRINTF("VIT_GetLibInfo error: %d\r\n", VIT_Status);
        return VIT_INVALID_STATE;
    }
    PRINTF("\n   \tVIT Lib Info\r\n");
    PRINTF("   \tVIT LIB Release: 0x%04x\r\n", Lib_Info.VIT_LIB_Release);
    PRINTF("   \tVIT Features supported by the lib: 0x%04x\r\n", Lib_Info.VIT_Features_Supported);
    PRINTF("   \tNumber of channels supported by VIT lib: %d\r\n", Lib_Info.NumberOfChannels_Supported);
    if (Lib_Info.WakeWord_In_Text2Model)
    {
        PRINTF("   \tVIT WakeWord in Text2Model\r\n");
    }
    else
    {
        PRINTF("   \tVIT WakeWord in Audio2Model\r\n");
    }
    PRINTF("   \t*** VIT config END ***\r\n\r\n");
    /*
     *   Configure VIT Instance Parameters
     */
    // Check that NUMBER_OF_CHANNELS is supported by VIT
    // Retrieve from VIT_GetLibInfo API the number of channel supported by the VIT lib
    PL_UINT16 max_nb_of_Channels = Lib_Info.NumberOfChannels_Supported;
    if (NUMBER_OF_CHANNELS > max_nb_of_Channels)
    {
        PRINTF("VIT lib is supporting only: %d channels\r\n", max_nb_of_Channels);
        return VIT_INVALID_PARAMETER_OUTOFRANGE;
    }
    return VIT_SUCCESS;
}

int VIT_Initialize(void *arg)
{
    VIT_ReturnStatus_en VIT_Status;

    switch (Vit_Language)
    {
        case CN:
            VIT_Status = VIT_SetModel(VIT_Model_cn, VIT_MODEL_IN_ROM);
            break;
        default:
            VIT_Status = VIT_SetModel(VIT_Model_en, VIT_MODEL_IN_ROM);
    }
    if (VIT_Status != VIT_SUCCESS)
    {
        return VIT_Status;
    }

    VIT_Status = VIT_ModelInfo();
    if (VIT_Status != VIT_SUCCESS)
    {
        return VIT_Status;
    }
    /*
     *   Configure VIT Instance Parameters
     */
    VITInstParams.SampleRate_Hz   = VIT_SAMPLE_RATE;
    VITInstParams.SamplesPerFrame = VIT_SAMPLES_PER_FRAME;
    VITInstParams.NumberOfChannel = NUMBER_OF_CHANNELS;
    VITInstParams.DeviceId        = DEVICE_ID;

    /*
     *   VIT get memory table: Get size info per memory type
     */
    VIT_Status = VIT_GetMemoryTable(PL_NULL, // VITHandle param should be NULL
                                    &VITMemoryTable, &VITInstParams);
    if (VIT_Status != VIT_SUCCESS)
    {
        PRINTF("VIT_GetMemoryTable error: %d\r\n", VIT_Status);
        return VIT_Status;
    }

    /*
     *   Reserve memory space: Malloc for each memory type
     */
    for (int i = 0; i < PL_NR_MEMORY_REGIONS; i++)
    {
        /* Log the memory size */
        if (VITMemoryTable.Region[i].Size != 0)
        {
            // reserve memory space
            // NB: VITMemoryTable.Region[PL_MEMREGION_PERSISTENT_FAST_DATA] should be allocated
            //      in the fastest memory of the platform (when possible) - this is not the case in this example.
            pMemory[i]                            = osa_malloc(VITMemoryTable.Region[i].Size + MEMORY_ALIGNMENT);
            VITMemoryTable.Region[i].pBaseAddress = (void *)pMemory[i];
        }
    }

    /*
     *    Create VIT Instance
     */
    VITHandle  = PL_NULL; // force to null address for correct memory initialization
    VIT_Status = VIT_GetInstanceHandle(&VITHandle, &VITMemoryTable, &VITInstParams);
    if (VIT_Status != VIT_SUCCESS)
    {
        InitPhase_Error = PL_TRUE;
        PRINTF("VIT_GetInstanceHandle error: %d\r\n", VIT_Status);
    }

    /*
     *    Test the reset (OPTIONAL)
     */
    if (!InitPhase_Error)
    {
        VIT_Status = VIT_ResetInstance(VITHandle);
        if (VIT_Status != VIT_SUCCESS)
        {
            InitPhase_Error = PL_TRUE;
            PRINTF("VIT_ResetInstance error: %d\r\n", VIT_Status);
        }
    }

    /*
     *   Set and Apply VIT control parameters
     */
    VITControlParams.OperatingMode = VIT_OPERATING_MODE;
    VITControlParams.MIC1_MIC2_Distance = VIT_MIC1_MIC2_DISTANCE;
    VITControlParams.MIC1_MIC3_Distance = VIT_MIC1_MIC3_DISTANCE;
    if (!InitPhase_Error)
    {
        VIT_Status = VIT_SetControlParameters(VITHandle, &VITControlParams);
        if (VIT_Status != VIT_SUCCESS)
        {
            InitPhase_Error = PL_TRUE;
            PRINTF("VIT_SetControlParameters error: %d\r\n", VIT_Status);
        }
    }
    /*
        //Public call to VIT_GetStatusParameters
        VIT_StatusParams_st* pVIT_StatusParam_Buffer = (VIT_StatusParams_st*)&VIT_StatusParams_Buffer;

        VIT_GetStatusParameters(VITHandle, pVIT_StatusParam_Buffer, sizeof(VIT_StatusParams_Buffer));
        PRINTF("\nVIT Status Params\n");
        PRINTF(" VIT LIB Release   = 0x%04x\n", pVIT_StatusParam_Buffer->VIT_LIB_Release);
        PRINTF(" VIT Model Release = 0x%04x\n", pVIT_StatusParam_Buffer->VIT_MODEL_Release);
        PRINTF(" VIT Features supported by the lib = 0x%04x\n", pVIT_StatusParam_Buffer->VIT_Features_Supported);
        PRINTF(" VIT Features Selected             = 0x%04x\n", pVIT_StatusParam_Buffer->VIT_Features_Selected);
        PRINTF(" Number of channels supported by VIT lib = %d\n", pVIT_StatusParam_Buffer->NumberOfChannels_Supported);
        PRINTF(" Number of channels selected             = %d\n", pVIT_StatusParam_Buffer->NumberOfChannels_Selected);
        PRINTF(" Device Selected: device id = %d\n", pVIT_StatusParam_Buffer->Device_Selected);
        if (pVIT_StatusParam_Buffer->WakeWord_In_Text2Model)
        {
            PRINTF(" VIT WakeWord in Text2Model\n ");
        }
        else
        {
            PRINTF(" VIT WakeWord in Audio2Model\n ");
        }
    */
    return VIT_Status;
}

void set_task_handle(void *t_handle)
{
	mp3_commandQ = (QueueHandle_t *)t_handle;
}

void set_task_handle_wifi(void *t_handle)
{
	wifi_controlQ = (QueueHandle_t *)t_handle;
}

int VIT_Execute(void *arg, PL_INT16 *inputBuffer, int size)
{
    VIT_ReturnStatus_en VIT_Status;
    VIT_VoiceCommand_st VoiceCommand;                               // Voice Command id
    VIT_DetectionStatus_en VIT_DetectionResults = VIT_NO_DETECTION; // VIT detection result

    if (size != VIT_SAMPLES_PER_FRAME)
    {
        PRINTF("Input buffer format issue\r\n");
        return VIT_INVALID_FRAME_SIZE;
    }
    /*
     *   VIT Process
     */
    // Current VIT library is supporting only one channel
    // VIT_InputBuffers.pBuffer_Chan1 should be set to the input buffer address
    // VIT_InputBuffers.pBuffer_Chan1 setting can be done out of the while loop
    // Application should take care of the ping pong buffers (when present) handling - no pingpong buffer in this
    // example app.
    if (VITInstParams.NumberOfChannel == _1CHAN)
    {
        VIT_InputBuffers.pBuffer_Chan1 = inputBuffer; // PCM buffer: 16-bit - 16kHz - mono
        VIT_InputBuffers.pBuffer_Chan2 = PL_NULL;
        VIT_InputBuffers.pBuffer_Chan3 = PL_NULL;
    }
    else if (VITInstParams.NumberOfChannel == _2CHAN)
    {
    	VIT_InputBuffers.pBuffer_Chan1 = inputBuffer; // PCM buffer: 16-bit - 16kHz - mono
    	VIT_InputBuffers.pBuffer_Chan2 = &inputBuffer[VIT_SAMPLES_PER_FRAME];
    	VIT_InputBuffers.pBuffer_Chan3 = PL_NULL;
    }
    else if (VITInstParams.NumberOfChannel == _3CHAN)
    {
    	VIT_InputBuffers.pBuffer_Chan1 = inputBuffer; // PCM buffer: 16-bit - 16kHz - mono
    	VIT_InputBuffers.pBuffer_Chan2 = &inputBuffer[VIT_SAMPLES_PER_FRAME];
    	VIT_InputBuffers.pBuffer_Chan3 = &inputBuffer[VIT_SAMPLES_PER_FRAME*2];
    }
    else
    {
    	PRINTF("Input buffer format issue\r\n"); // could be implemented if needed, see VIT_ExApp.c
    	return VIT_INVALID_PARAMETER_OUTOFRANGE;
    }

    VIT_Status = VIT_Process(VITHandle,
                             &VIT_InputBuffers, // temporal audio input data
                             &VIT_DetectionResults);

    if (VIT_Status != VIT_SUCCESS)
    {
        PRINTF("VIT_Process error: %d\r\n", VIT_Status);
        return VIT_Status; // will stop processing VIT and go directly to MEM free
    }

    if (VIT_DetectionResults == VIT_WW_DETECTED)
    {
        PRINTF("[VIT]  WakeWord detected \r\n");
    }
    else if (VIT_DetectionResults == VIT_VC_DETECTED)
    {
        // Retrieve id of the Voice Command detected
        // String of the Command can also be retrieved (when WW and CMDs strings are integrated in Model)
        VIT_Status = VIT_GetVoiceCommandFound(VITHandle, &VoiceCommand);
        if (VIT_Status != VIT_SUCCESS)
        {
            PRINTF("[VIT!] VIT_GetVoiceCommandFound error: %d\r\n", VIT_Status);
            return VIT_Status; // will stop processing VIT and go directly to MEM free
        }
        else
        {
            PRINTF("[VIT]  Voice Command detected %d :", VoiceCommand.Cmd_Id);
            // Retrieve CMD Name: OPTIONAL
			// Check first if CMD string is present
			if (VoiceCommand.pCmd_Name != PL_NULL)
			{
				PRINTF(" %s\r\n", VoiceCommand.pCmd_Name);
			}
			else
			{
				PRINTF("\r\n");
			}

            switch(VoiceCommand.Cmd_Id)
            {
            	case PLAY:		// PLAY
            		voice_command.command_type = CMD_START;
            		xQueueSend(*mp3_commandQ, (void *) &voice_command, 10);
            		break;
            	case NEXT:		// NEXT
            		voice_command.command_type = CMD_NEXT;
            		xQueueSend(*mp3_commandQ, (void *) &voice_command, 10);
            		break;
            	case PREVIOUS:		// PREVIOUS
            		voice_command.command_type = CMD_PREVIOUS;
            		xQueueSend(*mp3_commandQ, (void *) &voice_command, 10);
            		break;
            	case STOP:		// STOP
            		voice_command.command_type = CMD_PAUSE;
            		xQueueSend(*mp3_commandQ, (void *) &voice_command, 10);
            		break;
            	case VOLUME_UP:			// VOLUME_UP
            		voice_command.command_type = CMD_VOL_UP;
            		xQueueSend(*mp3_commandQ, (void *) &voice_command, 10);
            		break;
            	case VOLUME_DOWN:		// VOLUME_DOWN
            		voice_command.command_type = CMD_VOL_DWN;
            		xQueueSend(*mp3_commandQ, (void *) &voice_command, 10);
            		break;
            	case LED_RED: 		// LED RED
            		set_led(RED);
            		break;
            	case LED_GREEN: 	// LED GREEN
					set_led(GREEN);
					break;
            	case LED_BLUE: 		// LED BLUE
					set_led(BLUE);
					break;
            	case SENSOR_PAGE: 	// LED STATUS
            		change_page();
					break;
            	case WIRELESS_OFF: 	// WIFI OFF
            		wifi_command = 1;
            		xQueueSend(*wifi_controlQ, (void *) &wifi_command, 10);
					break;
            	case RESET:
            		/* turn off wifi */
            		PRINTF("*** Reset initiated, Stopping All tasks\r\n");
            		wifi_command = 1;
            		xQueueSend(*wifi_controlQ, (void *) &wifi_command, 10);
            		/* stop mp3 player */
            		voice_command.command_type = CMD_STOP;
            		xQueueSend(*mp3_commandQ, (void *) &voice_command, 10);
            		USB_HostFatUnmount();
            		vTaskDelay(10000 / portTICK_PERIOD_MS);
            		PRINTF("*** All done. Rebooting\r\n");
            		vTaskDelay(1000 / portTICK_PERIOD_MS);
            		NVIC_SystemReset();
            		break;
            	default:
            		break;
            }
        }
    }
    return VIT_Status;
}

int VIT_Deinit(void)
{
    VIT_ReturnStatus_en VIT_Status; /* Function call status */
                                    // retrieve size of the different MEM tables allocated
    VIT_Status =
        VIT_GetMemoryTable(VITHandle, // Should provide VIT_Handle to retrieve the size of the different MemTabs
                           &VITMemoryTable, &VITInstParams);
    if (VIT_Status != VIT_SUCCESS)
    {
        PRINTF("VIT_GetMemoryTable error: %d\r\n", VIT_Status);
        return VIT_Status;
    }

    // Free the MEM tables
    for (int i = 0; i < PL_NR_MEMORY_REGIONS; i++)
    {
        if (VITMemoryTable.Region[i].Size != 0)
        {
            osa_free((PL_INT8 *)pMemory[i]);
            pMemory[i] = NULL;
        }
    }
    return VIT_Status;
}

VIT_Initialize_T VIT_Initialize_func = VIT_Initialize;
VIT_Execute_T VIT_Execute_func       = VIT_Execute;
VIT_Deinit_T VIT_Deinit_func         = VIT_Deinit;
VIT_Language_T Vit_Language;
