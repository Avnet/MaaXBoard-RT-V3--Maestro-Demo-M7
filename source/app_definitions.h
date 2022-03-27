/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_DEFINITIONS_H_
#define _APP_DEFINITIONS_H_

/*${header:start}*/
#include "fsl_sgtl5000.h"
/*${header:end}*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
/* SAI instance and clock */
#define DEMO_CODEC_WM8960
#define DEMO_SAI              SAI2
#define DEMO_SAI_CHANNEL      (0)
#define DEMO_SAI_BITWIDTH     (kSAI_WordWidth16bits)
#define DEMO_SAI_IRQ          SAI2_IRQn
#define DEMO_SAITxIRQHandler  SAI2_IRQHandler
#define DEMO_SAI_TX_SYNC_MODE kSAI_ModeAsync
#define DEMO_SAI_RX_SYNC_MODE kSAI_ModeSync
#define DEMO_SAI_MASTER_SLAVE kSAI_Master
#define DEMO_SAI_CLOCK_SOURCE         (kSAI_BclkSourceMclkDiv)


#define DEMO_AUDIO_DATA_CHANNEL (2U)
#define DEMO_AUDIO_BIT_WIDTH    kSAI_WordWidth16bits
#define DEMO_AUDIO_SAMPLE_RATE_44100  (kSAI_SampleRate44100Hz)
#define DEMO_AUDIO_SAMPLE_RATE_48000  (kSAI_SampleRate48KHz)
#define DEMO_AUDIO_MASTER_CLOCK DEMO_SAI_CLK_FREQ
#define DEMO_PDM_SAMPLE_RATE  (16000U)


/* IRQ */
#define DEMO_SAI_TX_IRQ SAI2_IRQn
#define DEMO_SAI_RX_IRQ SAI2_IRQn

#define DEMO_PDM_IRQ PDM_EVENT_IRQn


/* DMA */
#define DEMO_PDM		   PDM
#define DEMO_DMA           DMA0
#define DEMO_DMAMUX        DMAMUX0
#define DEMO_PDM_EDMA_CHANNEL_1 1
#define DEMO_PDM_REQUEST_SOURCE kDmaRequestMuxPdm

#define DEMO_TX_CHANNEL    (0U)
#define DEMO_RX_CHANNEL    (1U)
#define DEMO_SAI_TX_SOURCE kDmaRequestMuxSai2Tx
#define DEMO_SAI_RX_SOURCE kDmaRequestMuxSai2Rx

/* Select Audio/Video PLL (786.48 MHz) as sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_SELECT (2U)
/* Clock pre divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER (0U)
/* Clock divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_DIVIDER (63U)
/* Get frequency of sai1 clock */
#define DEMO_SAI_CLK_FREQ CLOCK_GetRootClockFreq(kCLOCK_Root_Sai2)

#define DEMO_CHANNEL_NUM 2

#define BOARD_MASTER_CLOCK_CONFIG()

// added
#define DEMO_PDM_CLK_FREQ             24576000
#define DEMO_PDM_FIFO_WATERMARK       (4)
#define DEMO_PDM_QUALITY_MODE         kPDM_QualityModeHigh
#define DEMO_PDM_CIC_OVERSAMPLE_RATE  (0U)
#define DEMO_PDM_ENABLE_CHANNEL_MIC0  (0U)
#define DEMO_PDM_ENABLE_CHANNEL_MIC1  (1U)
#define DEMO_PDM_ENABLE_CHANNEL_MIC2  (2U)
/*${macro:end}*/

#endif /* _APP_DEFINITIONS_H_ */

