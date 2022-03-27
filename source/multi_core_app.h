/*
 * multi_core_app.h
 *
 *  Created on: Mar 21, 2022
 *      Author: gulziibayar
 */

#ifndef MULTI_CORE_APP_H_
#define MULTI_CORE_APP_H_
#include "stdint.h"
#include "FreeRTOS.h"

/*
 * configSUPPORT_STATIC_ALLOCATION is set to 1, requiring this callback to
 * provide statically allocated data for use by the idle task, which is a task
 * created by the scheduler when it starts.
 */

extern int16_t fxos_pitch;
extern int16_t fxos_roll;
extern int16_t fxos_yaw;
extern int16_t light_distance;

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize);

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize);
void SlaveToMasterComm(void *param);

#endif /* MULTI_CORE_APP_H_ */
