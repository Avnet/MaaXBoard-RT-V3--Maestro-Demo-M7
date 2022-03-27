/*
 * voice_task.c
 *
 *  Created on: Dec 2, 2021
 *      Author: gulziibayar
 */
#include "voice_task.h"

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

static streamer_handle_t streamerHandle;
int start_vit(void *param)
{
	voice_task_param_t *tempPtr = (voice_task_param_t *)param;
	status_t ret;
	out_sink_t out_sink;
	out_sink = VIT_SINK;

	STREAMER_Init();

	set_task_handle(tempPtr->cmd_queue);
	set_task_handle_wifi(tempPtr->cntrl_queue);
	ret = STREAMER_mic_Create(&streamerHandle, out_sink);

	if (ret != kStatus_Success)
    {
        PRINTF("STREAMER_Create failed\r\n");
        goto error;
    }

#ifdef VIT_PROC
    if (out_sink == VIT_SINK)
    {
        PRINTF("[VIT]  To see VIT functionality say wake-word and command.\r\n");
    }
#endif

	STREAMER_Start(&streamerHandle);
	return 1;
error:
	PRINTF("Cleanup\r\n");
	STREAMER_Destroy(&streamerHandle);
	return 0;
}
