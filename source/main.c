/*
 * Copyright 2020-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Board includes */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "main.h"

#include "ff.h"
#include "diskio.h"
#include "fsl_usb_disk.h"
#include "sdmmc_config.h"

#include "fsl_debug_console.h"

#include "fsl_codec_common.h"
#include "fsl_sgtl5000.h"
#include "fsl_codec_adapter.h"
#include "fsl_dmamux.h"
#include "app_definitions.h"

/* Freertos headers add */
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"

/* usb flash */
#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"
#include "usb_host_msd.h"
#include "host_msd_command.h"
#include "usb_phy.h"

/* voice task */
#include "voice_task.h"
/* mp3 player task */
#include "mp3_player.h"

#include "http_callbacks.h"

/* Low power */
#include "lpm.h"

/* multi core header */
#include "multi_core_app.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
extern uint32_t __Vectors[];
#define IMAGE_ENTRY_ADDRESS ((uint32_t)__Vectors)

#define CPU_NAME "iMXRT1176"

#define APP_WAKEUP_BUTTON_GPIO        BOARD_USER_BUTTON_GPIO
#define APP_WAKEUP_BUTTON_GPIO_PIN    BOARD_USER_BUTTON_GPIO_PIN
#define APP_WAKEUP_BUTTON_IRQ         BOARD_USER_BUTTON_IRQ
#define APP_WAKEUP_BUTTON_IRQ_HANDLER BOARD_USER_BUTTON_IRQ_HANDLER
#define APP_WAKEUP_BUTTON_NAME        BOARD_USER_BUTTON_NAME

/* Flag indicates Core Boot Up*/
#define BOOT_FLAG 0x01U

/* Channel transmit and receive register */
#define CHN_MU_REG_NUM 0U

AT_QUICKACCESS_SECTION_CODE(void CLOCK_SetClockRoot(clock_root_t root, const clock_root_config_t *config));
AT_QUICKACCESS_SECTION_CODE(void SwitchFlexspiRootClock(bool pllLdoDisabled));

#define TASK_STACK_SIZE  (1024)

extern struct board_state_variables g_BoardState;
/*******************************************************************************
 * Definitions
 ******************************************************************************/
static int BOARD_CODEC_Init(void);
/*!
 * @brief host callback function.
 *
 * device attach/detach callback function.
 *
 * @param deviceHandle        device handle.
 * @param configurationHandle attached device's configuration descriptor information.
 * @param eventCode           callback event code, please reference to enumeration host_event_t.
 *
 * @retval kStatus_USB_Success              The host is initialized successfully.
 * @retval kStatus_USB_NotSupported         The application don't support the configuration.
 */
static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode);

/*!
 * @brief application initialization.
 */
static void USB_HostApplicationInit(void);
static void USB_HostApplicationDeInit(void);
static void USB_HostTask(void *param);
static void USB_HostApplicationTask(void *param);
extern void USB_HostClockInit(void);
extern void USB_HostIsrEnable(void);
extern void USB_HostTaskFn(void *param);
void BOARD_InitHardware(void);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* counter for CPU load measurement */
static uint32_t perfCounter = 0;

/* Allocate the memory for the heap. */
#if defined(configAPPLICATION_ALLOCATED_HEAP) && (configAPPLICATION_ALLOCATED_HEAP)
	#if defined(FRTOS_HEAP) && FRTOS_HEAP==_MEM_REGION_OCRAM
		__attribute__ ((section(".heapOcram"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];
	#elif defined(FRTOS_HEAP) && FRTOS_HEAP==_MEM_REGION_SDRAM_
		__attribute__ ((section(".heapSdram"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];
	#else
		#error "Choose memory region either _USE_OCRAM or _USE_SDRAM_"
	#endif
#endif
/*! @brief USB host msd command instance global variable */
extern usb_host_msd_command_instance_t g_MsdCommandInstance;

/*! @brief USB host msd fatfs instance global variable */
extern usb_host_msd_fatfs_instance_t g_MsdFatfsInstance;

usb_host_handle g_HostHandle;

/*******************************************************************************
 * Variables
 ******************************************************************************/
codec_handle_t codecHandle   = {0};
/* codec config */
sgtl_config_t sgtlConfig = {
	.i2cConfig        = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE, .codecI2CSourceClock = BOARD_CODEC_I2C_CLOCK_FREQ},
	.route            = kSGTL_RoutePlaybackandRecord,
	.bus              = kSGTL_BusI2S,
	.slaveAddress     = SGTL5000_I2C_ADDR,
	.format           = {
			.mclk_HZ       = 24576000U,
			.sampleRate    = 44100U,
			.bitWidth      = 16U
	},
	.master_slave     = false,
};

codec_config_t boardCodecConfig = {.codecDevType = kCODEC_SGTL5000, .codecDevConfig = &sgtlConfig};

/*
 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM) / (2^POST)
 *                              = 24 * (32 + 77/100)  / 2
 *                              = 393.24MHZ
 */
const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,   /* Divider after the PLL, should only be 0, 1, 2, 3, 4, 5 */
    .numerator   = 77,  /* 30 bit numerator of fractional loop divider. */
    .denominator = 100, /* 30 bit denominator of fractional loop divider */
};

static app_handle_t app;

static custom_app_handle_t custom_app;
static voice_task_param_t voice_params;
static wifi_task_param_t wifi_params;
static TaskHandle_t xVoiceTaskHandle = NULL;
static TaskHandle_t xMusicTaskHandle = NULL;

/* Freertos queue */
static QueueHandle_t mp3_commandQ = NULL;
static QueueHandle_t wifi_controlQ = NULL;

/* Function Prototypes */
static void music_task(void *param);
static void BOARD_EnableSaiMclkOutput(bool enable);
static void Voice_Task(void *param);
void APP_WAKEUP_BUTTON_IRQ_HANDLER(void);

int main(void)
{
	gpc_cpu_mode_t preCpuMode;
    int ret;
    BOARD_ConfigMPU();
    BOARD_InitPins();

#if defined(WIFI_BOARD_AW_CM358)
    /* Init SDIO_RST */
    BOARD_InitM2WifiResetPins();
#endif
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    // set start address after waking up from SUSPEND mode
    IOMUXC_LPSR_GPR->GPR26 &= ~IOMUXC_LPSR_GPR_GPR26_CM7_INIT_VTOR_MASK;
    IOMUXC_LPSR_GPR->GPR26 |= IOMUXC_LPSR_GPR_GPR26_CM7_INIT_VTOR(IMAGE_ENTRY_ADDRESS >> 7);

#if defined(WIFI_BOARD_AW_CM358)
    /* Set SDIO_RST to 1 */
    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    GPIO_PinInit(GPIO9, 13, &gpio_config);
    GPIO_WritePinOutput(GPIO9, 13, 1);
    SDK_DelayAtLeastUs(30000, CLOCK_GetFreq(kCLOCK_CpuClk));
    GPIO_WritePinOutput(GPIO9, 13, 0);
    SDK_DelayAtLeastUs(30000, CLOCK_GetFreq(kCLOCK_CpuClk));
    GPIO_WritePinOutput(GPIO9, 13, 1);
    SDK_DelayAtLeastUs(30000, CLOCK_GetFreq(kCLOCK_CpuClk));
#endif

    CLOCK_InitAudioPll(&audioPllConfig);
    USB_HostApplicationInit();

    /*Clock setting for LPI2C*/
    CLOCK_SetRootClockMux(kCLOCK_Root_Lpi2c5, 1);

    /*Clock setting for SAI2*/
    CLOCK_SetRootClockMux(kCLOCK_Root_Sai2, kCLOCK_SAI2_ClockRoot_MuxAudioPllOut);
    CLOCK_SetRootClockDiv(kCLOCK_Root_Sai2, 16);

    /* 24.576m mic root clock */
    CLOCK_SetRootClockMux(kCLOCK_Root_Mic, kCLOCK_MIC_ClockRoot_MuxAudioPllOut);
    CLOCK_SetRootClockDiv(kCLOCK_Root_Mic, 16);

    clock_root_config_t rootCfg = {0};
    rootCfg.mux = kCLOCK_LPI2C5_ClockRoot_MuxOscRc16M;
    rootCfg.div = 1;
    CLOCK_SetRootClock(kCLOCK_Root_Lpi2c5, &rootCfg);

    /*Enable MCLK clock*/
    BOARD_EnableSaiMclkOutput(true);

    /* Init DMAMUX */
    DMAMUX_Init(DEMO_DMAMUX);
	DMAMUX_SetSource(DEMO_DMAMUX, DEMO_TX_CHANNEL, (uint8_t)DEMO_SAI_TX_SOURCE);
	DMAMUX_EnableChannel(DEMO_DMAMUX, DEMO_TX_CHANNEL);

	/* Init DMA for PDM peripheral */
    DMAMUX_SetSource(DEMO_DMAMUX, DEMO_PDM_EDMA_CHANNEL_1, DEMO_PDM_REQUEST_SOURCE);
    DMAMUX_EnableChannel(DEMO_DMAMUX, DEMO_PDM_EDMA_CHANNEL_1);

    edma_config_t dmaConfig;
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(DEMO_DMA, &dmaConfig);

    PRINTF("\r\n");
    PRINTF("**********************************\r\n");
    PRINTF("MaaXBoard Voice Demo start\r\n");
    PRINTF("**********************************\r\n");

    ret = BOARD_CODEC_Init();
    if (ret)
    {
        PRINTF("CODEC_Init failed\r\n");
        return -1;
    }

    /* Freertos queue for mp3 commands. 2 senders (webserver, voice control), 1 receiver(mp3) */
    mp3_commandQ = xQueueCreate(10, sizeof(queue_command_t));
	if (mp3_commandQ != NULL)
	{
		vQueueAddToRegistry(mp3_commandQ, "mp3CommandQ");
	}

    /* Freertos queue for mp3 commands. 2 senders (webserver, voice control), 1 receiver(mp3) */
    wifi_controlQ = xQueueCreate(10, sizeof(uint8_t));
	if (wifi_controlQ != NULL)
	{
		vQueueAddToRegistry(wifi_controlQ, "wifiControlQ");
	}

    /* USB host task */
    if (xTaskCreate(USB_HostTask, "usb host task", 2000L / sizeof(portSTACK_TYPE), g_HostHandle, 4, NULL) != pdPASS)
    {
        PRINTF("create host task error\r\n");
    }

    /* create semaphore for USB thumb-drive plugging */
    app.usbDiskSem = xSemaphoreCreateBinary();
    app.cmd_queue = &mp3_commandQ;
    custom_app.app_handle_ptr = &app;
    custom_app.usb_host_msd_fatfs_inst_ptr = &g_MsdFatfsInstance;
    if (xTaskCreate(USB_HostApplicationTask, "usb app task", 2000L / sizeof(portSTACK_TYPE), &custom_app, 3,
                    NULL) != pdPASS)
    {
        PRINTF("create app task error\r\n");
    }

    /* mp3 player task */
	if (xTaskCreate(music_task, "Music Task", TASK_STACK_SIZE, &app, configMAX_PRIORITIES - 5,
			&xMusicTaskHandle) != pdPASS)
	{
		PRINTF("\r\nFailed to create application task\r\n");
		while (1);
	}

	/* voice control task */
	voice_params.cmd_queue = &mp3_commandQ;
	voice_params.cntrl_queue = &wifi_controlQ;
	if (xTaskCreate(Voice_Task, "Voice Task", TASK_STACK_SIZE, &voice_params, configMAX_PRIORITIES - 5,
			&xVoiceTaskHandle) != pdPASS)
	{
		PRINTF("\r\nFailed to create application task\r\n");
		while (1);
	}

	/* wifi task
	 *  - wifi manager
	 *  - httpserver
	 *  */
	wifi_params.cmd_queue = &mp3_commandQ;
	wifi_params.cntrl_queue = &wifi_controlQ;
	if (xTaskCreate(wifi_task, "wifi_task", 2048, &wifi_params, configMAX_PRIORITIES - 4, &g_BoardState.wifiTask) != pdPASS)
	{
		PRINTF("\r\nFailed to create wifi task\r\n");
		while (1);
	}

	/* Comm task
	 *  Communication between M7 and M4 cores.
	 *  */
    if (xTaskCreate(SlaveToMasterComm, "Comm_TASK", 2048, NULL, configMAX_PRIORITIES - 4, NULL)!= pdPASS)
    {
    	PRINTF("\r\nFailed to create multi core Comm task\r\n");
    	while (1);
    }

    /* Run RTOS */
    vTaskStartScheduler();

    /* Should not reach this statement */
    return 0;
}

/*******************************************************************************
 * Code
 ******************************************************************************/

void APP_WAKEUP_BUTTON_IRQ_HANDLER(void)
{
    if ((1U << APP_WAKEUP_BUTTON_GPIO_PIN) & GPIO_GetPinsInterruptFlags(APP_WAKEUP_BUTTON_GPIO))
    {
        /* Disable interrupt. */
        GPIO_DisableInterrupts(APP_WAKEUP_BUTTON_GPIO, 1U << APP_WAKEUP_BUTTON_GPIO_PIN);
        GPIO_ClearPinsInterruptFlags(APP_WAKEUP_BUTTON_GPIO, 1U << APP_WAKEUP_BUTTON_GPIO_PIN);
        GPC_DisableWakeupSource(APP_WAKEUP_BUTTON_IRQ);
#ifdef CORE1_GET_INPUT_FROM_CORE0
        MU_TriggerInterrupts(MU_BASE, kMU_GenInt0InterruptTrigger);
#endif
    }
    __DSB();
}

void APP_SetWakeupConfig(void)
{
    PRINTF("Press button %s to wake up system.\r\n", APP_WAKEUP_BUTTON_NAME);
    GPIO_ClearPinsInterruptFlags(APP_WAKEUP_BUTTON_GPIO, 1U << APP_WAKEUP_BUTTON_GPIO_PIN);
    /* Enable GPIO pin interrupt */
    GPIO_EnableInterrupts(APP_WAKEUP_BUTTON_GPIO, 1U << APP_WAKEUP_BUTTON_GPIO_PIN);

    NVIC_ClearPendingIRQ(APP_WAKEUP_BUTTON_IRQ);
    NVIC_SetPriority(APP_WAKEUP_BUTTON_IRQ, 5);

    /* Enable the Interrupt */
    EnableIRQ(APP_WAKEUP_BUTTON_IRQ);
    /* Mask all interrupt first */
    GPC_DisableAllWakeupSource(GPC_CPU_MODE_CTRL);
    /* Enable GPC interrupt */
    GPC_EnableWakeupSource(APP_WAKEUP_BUTTON_IRQ);
}



static void BOARD_EnableSaiMclkOutput(bool enable)
{
    if (enable)
    {
        IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_SAI2_MCLK_DIR_MASK;
    }
    else
    {
        IOMUXC_GPR->GPR1 &= (~IOMUXC_GPR_GPR1_SAI2_MCLK_DIR_MASK);
    }
}

static int BOARD_CODEC_Init(void)
{
    CODEC_Init(&codecHandle, &boardCodecConfig);

    /* Initial volume kept low for hearing safety. */
    CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight, 90);

    return 0;
}

/*******************************************************************************
 * Code
 ******************************************************************************/

void USB_OTG1_IRQHandler(void)
{
    USB_HostEhciIsrFunction(g_HostHandle);
}

void USB_OTG2_IRQHandler(void)
{
    USB_HostEhciIsrFunction(g_HostHandle);
}

void USB_HostClockInit(void)
{
    uint32_t usbClockFreq;
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };
    usbClockFreq = 24000000;
    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, usbClockFreq);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, usbClockFreq);
    }
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
}

void USB_HostIsrEnable(void)
{
    uint8_t irqNumber;

    uint8_t usbHOSTEhciIrq[] = USBHS_IRQS;
    irqNumber                = usbHOSTEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
/* USB_HOST_CONFIG_EHCI */

/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#endif
    EnableIRQ((IRQn_Type)irqNumber);
}

static void USB_HostIsrDisable(void)
{
    uint8_t irqNumber;

    uint8_t usbHOSTEhciIrq[] = USBHS_IRQS;
    irqNumber                = usbHOSTEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];

    DisableIRQ((IRQn_Type)irqNumber);
}


void USB_HostTaskFn(void *param)
{
    USB_HostEhciTaskFunction(param);
}

/*!
 * @brief USB isr function.
 */

static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode)
{
    usb_status_t status = kStatus_USB_Success;
    switch (eventCode & 0x0000FFFFU)
    {
        case kUSB_HostEventAttach:
            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            break;

        case kUSB_HostEventNotSupported:
            PRINTF("device not supported.\r\n");
            break;

        case kUSB_HostEventEnumerationDone:
            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            break;

        case kUSB_HostEventDetach:
            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            break;

        case kUSB_HostEventEnumerationFail:
            PRINTF("enumeration failed\r\n");
            break;

        default:
            break;
    }
    return status;
}

static void USB_HostApplicationInit(void)
{
    usb_status_t status = kStatus_USB_Success;

    USB_HostClockInit();

#if ((defined FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    status = USB_HostInit(CONTROLLER_ID, &g_HostHandle, USB_HostEvent);
    if (status != kStatus_USB_Success)
    {
        PRINTF("[USB] host init error\r\n");
        return;
    }
    USB_HostIsrEnable();

    PRINTF("[USB] host init done\r\n");
}

static void USB_HostApplicationDeInit(void)
{
	usb_status_t status = kStatus_USB_Success;
	status = USB_HostDeinit(g_HostHandle);
    USB_HostIsrDisable();
}


static void USB_HostTask(void *param)
{
    while (1)
    {
        USB_HostTaskFn(param);
//        vTaskDelay(5/portTICK_PERIOD_MS);
    }
}

static void USB_HostApplicationTask(void *param)
{
    while (1)
    {
        USB_HostMsdTask(param);
        vTaskDelay(5/portTICK_PERIOD_MS);
    }
}

static void Voice_Task(void *param)
{
	if (!start_vit(param))
	{
		PRINTF("[VIT]  voice task error\r\n");
		vTaskDelete(NULL);
		return;
	}
	while(1)
	{
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

static void music_task(void *param)
{
	audio_playback_task(param);
	vTaskDelete(NULL);
}

/*!
 * @brief PIT1 Timer interrupt service ISR
 */
void PIT1_IRQHANDLER(void)
{
	PIT_ClearStatusFlags(PIT1_PERIPHERAL, PIT1_CHANNEL_0, kPIT_TimerFlag);
	perfCounter++;
	__DSB();
}

/*!
 * @brief Configures the PIT timer, it will be called by Freertos
 */
void AppConfigureTimerForRuntimeStats(void) {
	pit_config_t config;

	PIT_GetDefaultConfig(&config);
	config.enableRunInDebug = false;
	PIT_Init(PIT1_PERIPHERAL, &config);
	PIT_SetTimerPeriod(PIT1_PERIPHERAL, PIT1_CHANNEL_0, PIT1_CHANNEL_0_TICKS);
	PIT_EnableInterrupts(PIT1_PERIPHERAL, PIT1_CHANNEL_0, kPIT_TimerInterruptEnable);
	EnableIRQ(PIT1_IRQN);
	PIT_StartTimer(PIT1_PERIPHERAL, PIT1_CHANNEL_0);
}

/*!
 * @brief Returns 32bit counter value. Used for freertos runtime analysis
 */
uint32_t AppGetRuntimeCounterValueFromISR(void) {
	return perfCounter;
}

/**
 * @brief Loop forever if stack overflow is detected.
 *
 * If configCHECK_FOR_STACK_OVERFLOW is set to 1,
 * this hook provides a location for applications to
 * define a response to a stack overflow.
 *
 * Use this hook to help identify that a stack overflow
 * has occurred.
 *
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    portDISABLE_INTERRUPTS();

    /* Loop forever */
    for (;;)
        ;
}

/**
 * @brief Warn user if pvPortMalloc fails.
 *
 * Called if a call to pvPortMalloc() fails because there is insufficient
 * free memory available in the FreeRTOS heap.  pvPortMalloc() is called
 * internally by FreeRTOS API functions that create tasks, queues, software
 * timers, and semaphores.  The size of the FreeRTOS heap is set by the
 * configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
 *
 */
void vApplicationMallocFailedHook()
{
    PRINTF(("\r\nERROR: Malloc failed to allocate memory\r\n"));
}
