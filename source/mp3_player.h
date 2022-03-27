/*
 * mp3_player.h
 *
 *  Created on: Dec 3, 2021
 *      Author: gulziibayar
 */

#ifndef MP3_PLAYER_H_
#define MP3_PLAYER_H_

#include "fsl_dmamux.h"
#include "fsl_sai_edma.h"
#include "FreeRTOS.h"
#include "portable.h"
#include "semphr.h"

struct track_node{
	char fileName[40];
	struct track_node *next;
	struct track_node *prev;
};

/*! @brief PCM interface structure */
typedef struct _mp3_rtos_t
{
    sai_transfer_t saiTx;
    sai_edma_handle_t saiTxHandle;
    edma_handle_t dmaTxHandle;
    SemaphoreHandle_t semaphoreTX;
} mp3_rtos_t;

int set_head_track();
int get_next_track(char *str_ptr);
int get_playing_track(char *str_ptr);
int32_t getCurrVolume();
int isPlaying();
int isUsbPlugged();
int audio_playback_task(void *param);

#endif /* MP3_PLAYER_H_ */
