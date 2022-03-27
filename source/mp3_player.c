/*
 * mp3_player.c
 *
 *  Created on: Dec 3, 2021
 *      Author: gulziibayar
 */
#include "mp3_player.h"

#include <string.h>
#include <stdint.h>
#include "fsl_debug_console.h"
#include "fsl_shell.h"

#include "app_streamer.h"
#include "fsl_usb_disk.h"

#ifdef VIT_PROC
#include "PL_platformTypes_CortexM7.h"
#include "VIT.h"
#include "vit_proc.h"
#endif

#include "main.h"
#include "app_definitions.h"
#include "fsl_codec_common.h"

/* helix mp3 decoder */
#include "mp3dec.h"

#include "lpm.h"
/* MP3 Variables */
#define FILE_READ_BUFFER_SIZE 8192
MP3FrameInfo			mp3FrameInfo;
HMP3Decoder				hMP3Decoder;
FIL						file;
char					file_read_buffer[FILE_READ_BUFFER_SIZE];
volatile int			bytes_left;
char					*read_ptr;

extern codec_handle_t codecHandle;

static uint32_t Mp3ReadId3V2Text(FIL* pInFile, uint32_t unDataLen, char* pszBuffer, uint32_t unBufferSize);
static uint32_t Mp3ReadId3V2Tag(FIL* pInFile, char* pszArtist,
		uint32_t unArtistSize, char* pszTitle, uint32_t unTitleSize);

static int play_mp3(char* filename);
static void mp3_player_task(void *param);

static app_handle_t *app_ptr=NULL;

/* Freertos mp3 command queue*/
static QueueHandle_t *mp3_commandQ;
static queue_command_t mp3_recvd_cmd;

static streamer_handle_t myStreamerHandle;
static bool file_playing = false;
static bool stop_command = false;
static bool pause_command = false;
static bool player_stopped = true;
static bool playlist_clear = true;
static struct track_node *head_track = NULL;
static struct track_node *curr_track = NULL;
static struct track_node *last_track = NULL;
static struct track_node *temp_track = NULL;

AT_NONCACHEABLE_SECTION_INIT(static mp3_rtos_t mp3Handle) = {0};

static int16_t audio_buffer0[4096];
static int16_t audio_buffer1[4096];

static int available_buff = 0;
static int32_t curr_volume = 80;

static void waveToBuffer(int buffer)
{
	int offset, err;
	int outOfData = 0;
	int16_t *samples;
	if (buffer)
	{
		samples = audio_buffer0;
	}
	else
	{
		samples = audio_buffer1;
	}
	memcpy((uint8_t *)samples, read_ptr, sizeof(audio_buffer0));
	bytes_left -= sizeof(audio_buffer0);

	mp3Handle.saiTx.data     = (uint8_t *)samples;
	mp3Handle.saiTx.dataSize = sizeof(audio_buffer0);
	status_t sai_status;
	sai_status = SAI_TransferSendEDMA(DEMO_SAI, &mp3Handle.saiTxHandle, &mp3Handle.saiTx);
	if (sai_status == kStatus_Success)
	{
	}
	else if (sai_status == kStatus_SAI_QueueFull)
	{
#ifdef DEBUG_MP3_PLAYBACK
    	/* wait for SAI to finish sending buffer; */
    	PRINTF("sai full\r\n");
#endif
        /* Wait for transfer to finish */
        if (xSemaphoreTake(mp3Handle.semaphoreTX, portMAX_DELAY) != pdTRUE)
        {
            return;
        }
    	SAI_TransferSendEDMA(DEMO_SAI, &mp3Handle.saiTxHandle, &mp3Handle.saiTx);
	}
	else
	{
		PRINTF("sai error\r\n");
	}
}

/*!
 * @brief convert MP3 Frame to PCM
 *
 * Called by the audio driver when it is time to provide data to
 * one of the audio buffers (while the other buffer is sent to the
 * CODEC using DMA). One mp3 frame is decoded at a time and
 * provided to the audio driver.
 *
 * @param int   	it is either 0, 1
 * @status          returns nothing.
 */
static void AudioConvertTransfer(int buffer) {

	int offset, err;
	int outOfData = 0;

	int16_t *samples;
	if (buffer)
	{
		samples = audio_buffer0;
	}
	else
	{
		samples = audio_buffer1;
	}

	offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
	bytes_left -= offset;
	read_ptr += offset;

	err = MP3Decode(hMP3Decoder, (unsigned char**)&read_ptr, (int*)&bytes_left, samples, 0);

	if (err)
	{
		/* error occurred */
		switch (err)
		{
			case ERR_MP3_INDATA_UNDERFLOW:
				outOfData = 1;
				break;
			case ERR_MP3_MAINDATA_UNDERFLOW:
				/* do nothing - next call to decode will provide more mainData */
				break;
			case ERR_MP3_FREE_BITRATE_SYNC:
			default:
				outOfData = 1;
				break;
		}
	}
	else
	{
		/* no error */
		MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);

		/* Duplicate data in case of mono to maintain playback speed */
		if (mp3FrameInfo.nChans == 1)
		{
			for(int i = mp3FrameInfo.outputSamps;i >= 0;i--)
			{
				samples[2 * i]=samples[i];
				samples[2 * i + 1]=samples[i];
			}
			mp3FrameInfo.outputSamps *= 2;
		}
	}
	if (err)
	{
		return;
	}


	mp3Handle.saiTx.data     = (uint8_t *)samples;
	mp3Handle.saiTx.dataSize = (mp3FrameInfo.bitsPerSample / 8) * mp3FrameInfo.outputSamps;//sizeof(audio_buffer0);
    status_t sai_status;
    sai_status = SAI_TransferSendEDMA(DEMO_SAI, &mp3Handle.saiTxHandle, &mp3Handle.saiTx);
    if (sai_status == kStatus_Success)
    {
    }
    else if (sai_status == kStatus_SAI_QueueFull)
    {
#ifdef DEBUG_MP3_PLAYBACK
    	/* wait for SAI to finish sending buffer; */
    	PRINTF("sai full\r\n");
#endif
        /* Wait for transfer to finish */
        if (xSemaphoreTake(mp3Handle.semaphoreTX, 4000) != pdTRUE)
        {
        	PRINTF("transfer never finished\r\n");
            return;
        }
    	SAI_TransferSendEDMA(DEMO_SAI, &mp3Handle.saiTxHandle, &mp3Handle.saiTx);
    }
    else
    {
    	PRINTF("sai error\r\n");
    }
}

/*!
 * @brief SAI call back when data transfer complete
 *
 * available_buff tells the mp3 decoder, it finished
 * sending PCM data. ready for more.
 */
static void saiTxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_SAI_TxError == status)
    {
        /* Handle the error. */
    }
    else
    {
    	if (available_buff)
		{
			available_buff--;
		}
    	if (available_buff)
    	{
    		mp3_rtos_t *tempHandle = (mp3_rtos_t *)userData;
			BaseType_t reschedule;
			xSemaphoreGiveFromISR(tempHandle->semaphoreTX, &reschedule);
			portYIELD_FROM_ISR(reschedule);
    	}
    }
}

/*!
 * @brief prints all tracks in the linked list
 */
static void print_tracks()
{
	struct track_node *track_ptr;

	track_ptr = head_track;
	int32_t track_index = 1;
	while(track_ptr!=NULL)
	{
		PRINTF("%d. %s\r\n", track_index, track_ptr->fileName);
		track_ptr = track_ptr->next;
		track_index++;
	}
}

/*!
 * @brief set the pointer to 1st track
 */
int set_head_track()
{
	temp_track = head_track;
}

/*!
 * @brief get next track
 */
int get_next_track(char *str_ptr)
{
	if (temp_track!=NULL)
	{
		strcpy(str_ptr, temp_track->fileName);
		temp_track = temp_track->next;
		return 1;
	}
	else
	{
		return 0;
	}
}

/*!
 * @brief get current track
 */
int get_playing_track(char *str_ptr)
{
	if (curr_track!=NULL)
	{
		strcpy(str_ptr, curr_track->fileName);
		return 1;
	}
	else
	{
		return 0;
	}
}

/*!
 * @brief returns current volume
 *
 */
int32_t getCurrVolume()
{
	return curr_volume;
}

/*!
 * @brief returns whether player is running or not.
 *
 * This function used for informing webserver to whether mp3 player is running
 */
int isPlaying()
{
	if (pause_command) {
		return 2;
	}
	return player_stopped?0:1;
}

/*!
 * @brief returns whether usb thumbdrive is plugged or not
 *
 */
int isUsbPlugged()
{
	if (app_ptr==NULL)
	{
		return 0;
	}
	else
	{
		return (app_ptr->usbDiskInserted)?1:0;
	}
}

static uint32_t get_sai2_clock()
{
	return 786432000U;
}

/*!
 * @brief configure Audio
 *
 * This function used for configuring SAI(Serial Audio Interface), DMA handle
 * And the associated interrupts.
 */
static void configure_audio(uint32_t s_rate)
{
	sai_transceiver_t config;
	uint32_t sample_rate = DEMO_AUDIO_SAMPLE_RATE_44100;
	/* set up NVIC priorities */
	NVIC_SetPriority(LPI2C5_IRQn, 5);
	NVIC_SetPriority(DEMO_SAI_TX_IRQ, 5U);
	NVIC_SetPriority(DMA0_DMA16_IRQn, 4U);

	/* Create DMA handle. */
	EDMA_CreateHandle(&mp3Handle.dmaTxHandle, DEMO_DMA, DEMO_TX_CHANNEL);

	/* SAI init */
	SAI_Init(DEMO_SAI);

	SAI_TransferTxCreateHandleEDMA(DEMO_SAI, &mp3Handle.saiTxHandle, saiTxCallback, (void *)&mp3Handle,
								   &mp3Handle.dmaTxHandle);

	/* I2S mode configurations */
	SAI_GetClassicI2SConfig(&config, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, kSAI_Channel0Mask);

    config.bitClock.bclkSource = (sai_bclk_source_t)DEMO_SAI_CLOCK_SOURCE;
    SAI_TransferTxSetConfigEDMA(DEMO_SAI, &mp3Handle.saiTxHandle, &config);

    /* set bit clock divider */

    if (s_rate == 48000)
    {
    	sample_rate = DEMO_AUDIO_SAMPLE_RATE_48000;
    }

    CLOCK_ControlMode(0);
    SAI_TxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, sample_rate, DEMO_AUDIO_BIT_WIDTH,
                          DEMO_AUDIO_DATA_CHANNEL);
    CLOCK_ControlMode(1);
    mp3Handle.semaphoreTX = xSemaphoreCreateBinary();

    /* Enable SAI transmit and FIFO error interrupts. */
    SAI_TxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
}

/*!
 * @brief configure Audio
 *
 * This function used for configuring SAI(Serial Audio Interface), DMA handle
 * And the associated interrupts.
 */
static void create_player_task()
{
	if (xTaskCreate(mp3_player_task, "mp3_player", TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 5,
			NULL) != pdPASS)
	{
		PRINTF("\r\nFailed to create MP3 task\r\n");
		while (1);
	}
}

/*!
 * @brief stops the current mp3 playback
 *
 * This function used for stopping current mp3 playback when STOP, NEXT, PREVIOUS command is received
 * And it waits until mp3 player task is stopped. It should stop at most within 1 mp3 Frame conversion time.
 */
static void wait_player_task()
{
	stop_command = true;
	pause_command = false;
	while(!player_stopped)
	{
		vTaskDelay(20/portTICK_PERIOD_MS);
	}
	stop_command = false;
}

/*!
 * @brief creates the playlist
 *
 * This function reads all the mp3 files in the root directory. And store the filenames in linked list;
 */
static status_t create_playlist()
{
	FILINFO fileInformation;
	DIR directory;
	uint32_t count = 0;
	FRESULT error;
	char *dot;
	struct track_node *track_ptr;
	head_track = NULL;
	curr_track = NULL;
	last_track = NULL;
	temp_track = NULL;

	/* list directory */
	error = f_opendir(&directory, "/");
	if (error)
	{
		PRINTF("Failed to open root directory of USB disk\r\n");
		return kStatus_Fail;
	}

	while (1)
	{
		error = f_readdir(&directory, &fileInformation);

		/* When dir end or error detected, break out */
		if ((error != FR_OK) || (fileInformation.fname[0U] == 0U))
		{
			break;
		}
		/* Skip root directory */
		if (fileInformation.fname[0] == '.')
		{
			continue;
		}
		if (!(fileInformation.fattrib & AM_DIR))
		{
			/* Check file for supported audio extension */
			dot = strrchr(fileInformation.fname, '.');
			if ((dot && strncmp(dot + 1, "opus", 4) == 0) || (dot && strncmp(dot + 1, "ogg", 3) == 0) ||
				(dot && strncmp(dot + 1, "mp3", 3) == 0))
			{
				track_ptr = pvPortMalloc(sizeof(struct track_node));
				track_ptr->prev = NULL;
				track_ptr->next = NULL;
				last_track = track_ptr;
				strcpy(track_ptr->fileName, fileInformation.fname);
				if (head_track==NULL)
				{
					head_track = track_ptr;
					curr_track = head_track;
				}
				else
				{
					track_ptr->prev = curr_track;
					curr_track->next = track_ptr;
					curr_track = track_ptr;
				}
				count++;
			}
		}
	}
	playlist_clear = false;
	curr_track = head_track;
	return kStatus_Success;
}

/*!
 * @brief free the playlist
 *
 * This function frees the memory allocated for mp3 playlist.
 */
static void free_playlist()
{
	struct track_node *track_ptr;
	track_ptr = head_track;
	while (track_ptr!=NULL)
	{
		if (track_ptr!=NULL) {
			curr_track = track_ptr->next;
			vPortFree((void *)track_ptr);
		}
		track_ptr = curr_track;
	}
	head_track = NULL;
	curr_track = NULL;
	last_track = NULL;
	temp_track = NULL;
	playlist_clear = true;
}

/*!
 * @brief FREERTOS task: "Audio playback task"
 *
 * This task handles initiating mp3_player_task based on the Voice command.
 * The voice command comes from Voice task through freertos notification object.
 */
int audio_playback_task(void *param)
{
	app_ptr = (app_handle_t *)param;
	int play_error = 0;
	mp3_commandQ = app_ptr->cmd_queue;
	DIR directory;
	FILINFO fileInformation;
	char *filename, *dot;
	uint32_t count = 0;
	FRESULT error;
	int eap_par = 0;

	CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight, curr_volume);
	/* Wait until USB disk is inserted. */
	if (xSemaphoreTake(app_ptr->usbDiskSem, portMAX_DELAY) != pdTRUE)
	{
		PRINTF("[Mp3!] Please insert an USB disk. mp3_player timed out\r\n");
		return kStatus_Fail;
	}

	if (!app_ptr->usbDiskInserted)
	{
    	PRINTF("[Mp3] Please insert an USB disk with audio files and retry this command\r\n");
	}

	if (create_playlist() == kStatus_Fail)
	{
		PRINTF("[Mp3] Create playlist failed\r\n");
	}
	/* Track list is complete now. */

	/* start the track pointer from the beginning by default */
	BaseType_t xResult;
	uint32_t ulNotifiedValue;

	while(1)
	{
		xResult = xQueueReceive(*mp3_commandQ, &(mp3_recvd_cmd), 100/portTICK_PERIOD_MS);

		if (!app_ptr->usbDiskInserted)
		{
			if (!playlist_clear)
			{
				free_playlist();
			}
			PRINTF("[Mp3] USB unplugged, please plug in\r\n");
			if (xSemaphoreTake(app_ptr->usbDiskSem, portMAX_DELAY) != pdTRUE)
			{
				PRINTF("[Mp3] Please insert an USB disk. mp3_player timed out\r\n");
				return kStatus_Fail;
			}
			if (create_playlist()==kStatus_Fail)
			{
				PRINTF("[Mp3] Create playlist failed\r\n");
			}
		}
		if (xResult == pdTRUE)
		{
			switch(mp3_recvd_cmd.command_type)
			{
				case CMD_START:
					if (pause_command)
					{
						pause_command = false;
						break;
					}
					if (!player_stopped)
					{
						break;
					}
					create_player_task();
					PRINTF("[Mp3]  Playing track:\"%s\" ", curr_track->fileName);
					file_playing = true;
					break;
				case CMD_STOP:
					PRINTF("[Mp3]  Stop track:\"%s\"\r\n", curr_track->fileName);
					wait_player_task();
					file_playing = false;
					break;
				case CMD_NEXT:
					if (curr_track->next == NULL)
					{
						/* last track reached, play from beginning */
						curr_track = head_track;
					}
					else
					{
						curr_track = curr_track->next;
					}
					if (!player_stopped)
					{
						wait_player_task();
					}
					create_player_task();
					file_playing = true;
					PRINTF("[Mp3] Playing next track:\"%s\"\r\n", curr_track->fileName);
					break;
				case CMD_PREVIOUS:
					if (curr_track->prev == NULL)
					{
						/* 1st track reached, play from last */
						curr_track = last_track;
					}
					else
					{
						curr_track = curr_track->prev;
					}
					if (!player_stopped)
					{
						wait_player_task();
					}
					create_player_task();
					file_playing = true;

					PRINTF("[Mp3] Playing previous track:\"%s\"\r\n", curr_track->fileName);
					break;
				case CMD_VOL_UP:
					curr_volume+=VOL_STEPS;
					if (curr_volume>100)
					{
						curr_volume = 100;
					}
					CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight, curr_volume);
					break;
				case CMD_VOL_DWN:
					curr_volume-=VOL_STEPS;
					if (curr_volume<0)
					{
						curr_volume = 0;
					}
					CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight, curr_volume);
					break;
				case CMD_PAUSE:
					PRINTF("[Mp3] Recvd: Pause\r\n");
					if (!player_stopped)
					{
						pause_command = true;
					}
					break;
				case CMD_VOL_VAL:
					if (mp3_recvd_cmd.buffer[0] >= 0 && mp3_recvd_cmd.buffer[0] <= 100)
					{
						curr_volume = mp3_recvd_cmd.buffer[0];
						CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneLeft | kCODEC_PlayChannelHeadphoneRight, curr_volume);
					}
					break;
				default:
					PRINTF("[Mp3] Unknown command received from voice task");
					break;
			}
		}
	}
	return kStatus_Success;
}

/*!
 * @brief FREERTOS task: "Mp3 player task"
 *
 * This task handles mp3 decoding, and transfer decoded PCM data to audio interface
 * The task will stop on following conditions:
 * 			- file is finished playing
 * 			- stop_command is true
 */
static void mp3_player_task(void *param)
{
	int res;
	player_stopped = false;
	res = play_mp3(curr_track->fileName);
	if (res<0) /* file read error */
	{
		/* clear the playlist */
		free_playlist();
	}
	PRINTF("[Mp3]  *** Player stopped ***\r\n");
	player_stopped = true;
    pause_command = false;
	vTaskDelete(NULL);
}


/*!
 * @brief get frame info for configuring sample_rate
 *
 * Before calling this function, hMP3Decoder = MP3InitDecoder(); must be called.
 *
 * @param *int   	pointer for sample_rate variable
 * @status int      0 when successful. other values are error.
 */
static int get_mp3_frameInfo(int32_t *s_rate)
{
	int offset, err;
	while(bytes_left > FILE_READ_BUFFER_SIZE/4)
	{
		offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
		bytes_left -= offset;
		read_ptr += offset;
		err = MP3Decode(hMP3Decoder, (unsigned char**)&read_ptr, (int*)&bytes_left, audio_buffer0, 0);
		if (err==0)
		{
			/* no error */
			MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
			PRINTF("@(bitrate:%d, ", mp3FrameInfo.bitrate);
			PRINTF("samp_rate:%d, ", mp3FrameInfo.samprate);
			PRINTF("chan:%d)\r\n", mp3FrameInfo.nChans);

			if (mp3FrameInfo.samprate == 48000)
			{
				*s_rate = 480000;
			}
			else
			{
				*s_rate = 441000;
			}
			/* restore the pointer to initial position */
			bytes_left = FILE_READ_BUFFER_SIZE;
			read_ptr = file_read_buffer;
			/* success */
			return 0;
		}
		else
		{
			/* error, possibly decoder not working(memory allocation issue) */
			if (err==ERR_MP3_NULL_POINTER)
			{
				return 1;
			}
			/* otherwise keep searching for valid frame */
		}
	}
	/* error */
	return 1;
}

/*!
 * @brief function for playing mp3 file
 */
static int play_mp3(char* filename) {
	unsigned int br, btr;
	FRESULT res;
	int buffer_select = 1;
	bytes_left = FILE_READ_BUFFER_SIZE;
	read_ptr = file_read_buffer;
	available_buff = 0;
	int32_t sample_rate = 0;
	if (FR_OK == f_open(&file, filename, FA_OPEN_EXISTING | FA_READ))
	{
		/* Fill buffer */
		res = f_read(&file, file_read_buffer, FILE_READ_BUFFER_SIZE, &br);

		if (res != FR_OK) {
			/* Close currently open file */
			f_close(&file);
			return -1;
		}
		// ignore the wave header.
//		read_ptr = read_ptr+44;
		hMP3Decoder = MP3InitDecoder();
		if (hMP3Decoder==NULL)
		{
			/* Couldn't allocate memory for mp3 decoder */
			f_close(&file);
			return -1;
		}
		if (get_mp3_frameInfo(&sample_rate)==0)
		{
			configure_audio(sample_rate);
		}
		else
		{
			/* free the resource for decoder */
			MP3FreeDecoder(hMP3Decoder);
			/* Close currently open file */
			f_close(&file);
			PRINTF("[Mp3] Couldn't read mp3 sample_rate\r\n");
			return 1;
		}
		while(1)
		{
			if (pause_command)
			{
				vTaskDelay(50/portTICK_PERIOD_MS);
			}
			else
			{
				if (available_buff < 2)
				{
					buffer_select ^= 1;
	//				waveToBuffer(buffer_select);
	#ifdef DEBUG_MP3_PLAYBACK
					long start = xTaskGetTickCount();
	#endif
					AudioConvertTransfer(buffer_select);

					available_buff++;
					/*
					 * If past half of buffer, refill...
					 *
					 * When bytes_left changes, the audio callback has just been executed. This
					 * means that there should be enough time to copy the end of the buffer
					 * to the beginning and update the pointer before the next audio callback.
					 * Getting audio callbacks while the next part of the file is read from the
					 * file system should not cause problems.
					 */
					if (bytes_left < (FILE_READ_BUFFER_SIZE / 2))
					{
						/* Copy rest of data to beginning of read buffer */
						memcpy(file_read_buffer, read_ptr, bytes_left);

						/* Update read pointer for audio sampling */
						read_ptr = file_read_buffer;

						/* Read next part of file */
						btr = FILE_READ_BUFFER_SIZE - bytes_left;

						res = f_read(&file, file_read_buffer + bytes_left, btr, &br);

						/* Update the bytes left variable */
						bytes_left = FILE_READ_BUFFER_SIZE;

						/* Out of data or error or STOP_command issued from Voice control... Stop playback! */
						if (br < btr || res != FR_OK || stop_command==true)
						{
							MP3FreeDecoder(hMP3Decoder);
							SAI_TransferTerminateSendEDMA(DEMO_SAI, &mp3Handle.saiTxHandle);
							vSemaphoreDelete(mp3Handle.semaphoreTX);
							/* Close currently open file */
							f_close(&file);
							/* Return to previous function */
							if (res != FR_OK)
								return -1;
							return 0;
						}
					}
	#ifdef DEBUG_MP3_PLAYBACK
					long stop = xTaskGetTickCount() - start;
					PRINTF("%d\r\n", stop);
	#endif
				}
				else
				{
					if (xSemaphoreTake(mp3Handle.semaphoreTX, 4000) != pdTRUE)
					{
						MP3FreeDecoder(hMP3Decoder);
						SAI_TransferTerminateSendEDMA(DEMO_SAI, &mp3Handle.saiTxHandle);
						vSemaphoreDelete(mp3Handle.semaphoreTX);
						/* Close currently open file */
						f_close(&file);
						PRINTF("[Mp3] Sai not responding\r\n");
						return 1;
					}
				}
			}
		}
	}
	return -1;
}

/*!
 * @brief parses tag on the MP3 file
 */
static uint32_t Mp3ReadId3V2Text(FIL* pInFile, uint32_t unDataLen, char* pszBuffer, uint32_t unBufferSize)
{
	UINT unRead = 0;
	BYTE byEncoding = 0;
	if((f_read(pInFile, &byEncoding, 1, &unRead) == FR_OK) && (unRead == 1))
	{
		unDataLen--;
		if(unDataLen <= (unBufferSize - 1))
		{
			if((f_read(pInFile, pszBuffer, unDataLen, &unRead) == FR_OK) ||
					(unRead == unDataLen))
			{
				if(byEncoding == 0)
				{
					/* ISO-8859-1 multibyte */
					/* just add a terminating zero */
					pszBuffer[unDataLen] = 0;
				}
				else if(byEncoding == 1)
				{
					/* UTF16LE unicode */
					uint32_t r = 0;
					uint32_t w = 0;
					if((unDataLen > 2) && (pszBuffer[0] == 0xFF) && (pszBuffer[1] == 0xFE))
					{
						/* ignore BOM, assume LE */
						r = 2;
					}
					for(; r < unDataLen; r += 2, w += 1)
					{
						/* should be acceptable for 7 bit ascii */
						pszBuffer[w] = pszBuffer[r];
					}
					pszBuffer[w] = 0;
				}
			}
			else
			{
				return 1;
			}
		}
		else
		{
			/* we won't read a partial text */
			if(f_lseek(pInFile, f_tell(pInFile) + unDataLen) != FR_OK)
			{
				return 1;
			}
		}
	}
	else
	{
		return 1;
	}
	return 0;
}

/*!
 * @brief parses tag on the MP3 file
 */
static uint32_t Mp3ReadId3V2Tag(FIL* pInFile, char* pszArtist, uint32_t unArtistSize, char* pszTitle, uint32_t unTitleSize)
{
	pszArtist[0] = 0;
	pszTitle[0] = 0;

	BYTE id3hd[10];
	UINT unRead = 0;
	if((f_read(pInFile, id3hd, 10, &unRead) != FR_OK) || (unRead != 10))
	{
		return 1;
	}
	else
	{
		uint32_t unSkip = 0;
		if((unRead == 10) &&
				(id3hd[0] == 'I') &&
				(id3hd[1] == 'D') &&
				(id3hd[2] == '3'))
		{
			unSkip += 10;
			unSkip = ((id3hd[6] & 0x7f) << 21) | ((id3hd[7] & 0x7f) << 14) | ((id3hd[8] & 0x7f) << 7) | (id3hd[9] & 0x7f);

			/* try to get some information from the tag */
			/* skip the extended header, if present */
			uint8_t unVersion = id3hd[3];
			if(id3hd[5] & 0x40)
			{
				BYTE exhd[4];
				f_read(pInFile, exhd, 4, &unRead);
				size_t unExHdrSkip = ((exhd[0] & 0x7f) << 21) | ((exhd[1] & 0x7f) << 14) | ((exhd[2] & 0x7f) << 7) | (exhd[3] & 0x7f);
				unExHdrSkip -= 4;
				if(f_lseek(pInFile, f_tell(pInFile) + unExHdrSkip) != FR_OK)
				{
					return 1;
				}
			}
			uint32_t nFramesToRead = 2;
			while(nFramesToRead > 0)
			{
				char frhd[10];
				if((f_read(pInFile, frhd, 10, &unRead) != FR_OK) || (unRead != 10))
				{
					return 1;
				}
				if((frhd[0] == 0) || (strncmp(frhd, "3DI", 3) == 0))
				{
					break;
				}
				char szFrameId[5] = {0, 0, 0, 0, 0};
				memcpy(szFrameId, frhd, 4);
				uint32_t unFrameSize = 0;
				uint32_t i = 0;
				for(; i < 4; i++)
				{
					if(unVersion == 3)
					{
						/* ID3v2.3 */
						unFrameSize <<= 8;
						unFrameSize += frhd[i + 4];
					}
					if(unVersion == 4)
					{
						/* ID3v2.4 */
						unFrameSize <<= 7;
						unFrameSize += frhd[i + 4] & 0x7F;
					}
				}

				if(strcmp(szFrameId, "TPE1") == 0)
				{
					/* artist */
					if(Mp3ReadId3V2Text(pInFile, unFrameSize, pszArtist, unArtistSize) != 0)
					{
						break;
					}
					nFramesToRead--;
				}
				else if(strcmp(szFrameId, "TIT2") == 0)
				{
					/* title */
					if(Mp3ReadId3V2Text(pInFile, unFrameSize, pszTitle, unTitleSize) != 0)
					{
						break;
					}
					nFramesToRead--;
				}
				else
				{
					if(f_lseek(pInFile, f_tell(pInFile) + unFrameSize) != FR_OK)
					{
						return 1;
					}
				}
			}
		}
		if(f_lseek(pInFile, unSkip) != FR_OK)
		{
			return 1;
		}
	}

	return 0;
}

