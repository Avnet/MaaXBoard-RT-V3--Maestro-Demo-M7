/*
 * Copyright 2020-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "ff.h"
//usb stuff added
#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"
#include "host_msd_command.h"

/* PIT timer header */
#include "fsl_pit.h"
#include <stdbool.h>

#define _MEM_REGION_OCRAM_	0	/* High performance with limited size ~750Kb */
#define _MEM_REGION_SDRAM_	1	/* Slow performance with abundant size 32MB */

#define FRTOS_HEAP	_MEM_REGION_SDRAM_

#if defined(FRTOS_HEAP) && FRTOS_HEAP==_MEM_REGION_OCRAM_
	#if NUM_OF_CH>1
		#error "NUM_OF_CH must be 1, if OCRAM used"
	#endif
#elif defined(FRTOS_HEAP) && FRTOS_HEAP==_MEM_REGION_SDRAM_
	#if NUM_OF_CH>3
		#error "NUM_OF_CH must be less than 3"
	#endif
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* PIT Timer used for runtime analysis on Freertos */
#define PIT1_PERIPHERAL PIT1
/* Definition of clock source frequency. */
#define PIT1_CLK_FREQ 240000000UL
/* Definition of ticks count for channel 0 - deprecated. */
#define PIT1_0_TICKS 23999U
/* PIT1 interrupt vector ID (number) - deprecated. */
#define PIT1_0_IRQN PIT0_IRQn
/* PIT1 interrupt handler identifier - deprecated. */
#define PIT1_0_IRQHANDLER PIT0_IRQHandler
/* Definition of channel number for channel 0. */
#define PIT1_CHANNEL_0 kPIT_Chnl_0
/* Definition of ticks count for channel 0. */
#define PIT1_CHANNEL_0_TICKS 23999U
/* PIT1 interrupt vector ID (number). */
#define PIT1_IRQN PIT1_IRQn
/* PIT1 interrupt handler identifier. */
#define PIT1_IRQHANDLER PIT1_IRQHandler

/* Command list from VIT */
#define CMD_START 		0
#define CMD_STOP  		1
#define CMD_NEXT		2
#define CMD_PREVIOUS 	3
#define CMD_VOL_UP 		4
#define CMD_VOL_DWN 	5
#define CMD_PAUSE	 	6
#define CMD_VOL_VAL		7

#define VOL_STEPS	20

typedef struct _queue_command
{
	uint8_t command_type;
	uint8_t taskId;
	uint8_t buffer[24];
}queue_command_t;

typedef struct _app_handle
{
    TaskHandle_t shell_task_handle;
    QueueHandle_t *cmd_queue;
    /* SD card management */
    SemaphoreHandle_t usbDiskSem;
    volatile bool sdcardInserted;
    volatile bool sdcardInsertedPrev;
    volatile bool usbDiskInserted;
    FATFS fileSystem;
    FIL fileObject;
} app_handle_t;

#if ((defined USB_HOST_CONFIG_KHCI) && (USB_HOST_CONFIG_KHCI))
#define CONTROLLER_ID kUSB_ControllerKhci0
#endif /* USB_HOST_CONFIG_KHCI */
#if ((defined USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI))
#define CONTROLLER_ID kUSB_ControllerEhci1
#endif /* USB_HOST_CONFIG_EHCI */
#if ((defined USB_HOST_CONFIG_OHCI) && (USB_HOST_CONFIG_OHCI))
#define CONTROLLER_ID kUSB_ControllerOhci0
#endif /* USB_HOST_CONFIG_OHCI */
#if ((defined USB_HOST_CONFIG_IP3516HS) && (USB_HOST_CONFIG_IP3516HS))
#define CONTROLLER_ID kUSB_ControllerIp3516Hs0
#endif /* USB_HOST_CONFIG_IP3516HS */

#if defined(__GIC_PRIO_BITS)
#define USB_HOST_INTERRUPT_PRIORITY (25U)
#elif defined(__NVIC_PRIO_BITS) && (__NVIC_PRIO_BITS >= 3)
#define USB_HOST_INTERRUPT_PRIORITY (6U)
#else
#define USB_HOST_INTERRUPT_PRIORITY (3U)
#endif

/*! @brief host app device attach/detach status */
typedef enum _usb_host_app_state
{
    kStatus_DEV_Idle = 0, /*!< there is no device attach/detach */
    kStatus_DEV_Attached, /*!< device is attached */
    kStatus_DEV_Detached, /*!< device is detached */
} usb_host_app_state_t;

typedef struct
{
	app_handle_t *app_handle_ptr;
	usb_host_msd_fatfs_instance_t *usb_host_msd_fatfs_inst_ptr;
}custom_app_handle_t;


/* struct wrapper for freertos task */
typedef struct
{
	QueueHandle_t *cmd_queue;
	QueueHandle_t *cntrl_queue;
}voice_task_param_t;

/* struct wrapper for freertos task */
typedef struct
{
	QueueHandle_t *cmd_queue;
	QueueHandle_t *cntrl_queue;
}wifi_task_param_t;


#endif /* __MAIN_H__ */
