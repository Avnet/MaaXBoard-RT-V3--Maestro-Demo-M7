/*
 * Copyright 2018-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "osa_common.h"
#include "main.h"
#include "board.h"
#include "streamer_pcm.h"
#include "fsl_codec_common.h"
#include "fsl_sgtl5000.h"
#include "app_definitions.h"
#include "osa_memory.h"
#include "fsl_cache.h"
#include "fsl_debug_console.h"

//#define DEBUG_AUDIO_SAMPLE 1

#define BUFFER_SIZE   (160*2*2*NUM_OF_CH)
#define BUFFER_NUMBER (2)
AT_NONCACHEABLE_SECTION_INIT(static pcm_rtos_t pcmHandle) = {0};
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t s_edmaTcd_0[2], 32U);
/* current buffer index for PDM (it is either 0 or 1) */
static uint32_t buffer_index = 0;

/* create twice the size of VIT sample chunk. 160(vit)*2 */
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_buffer[BUFFER_SIZE * BUFFER_NUMBER], 4);
pdm_edma_transfer_t pdmXfer[2] = {
    {
        .data         = s_buffer,
        .dataSize     = BUFFER_SIZE,
        .linkTransfer = &pdmXfer[1],
    },

    {.data = &s_buffer[BUFFER_SIZE], .dataSize = BUFFER_SIZE, .linkTransfer = &pdmXfer[0]},
};
static const pdm_config_t pdmConfig         = {
    .enableDoze        = false,
    .fifoWatermark     = DEMO_PDM_FIFO_WATERMARK,
    .qualityMode       = DEMO_PDM_QUALITY_MODE,
    .cicOverSampleRate = DEMO_PDM_CIC_OVERSAMPLE_RATE,
};
static const pdm_channel_config_t channelConfig = {
    .cutOffFreq = kPDM_DcRemoverCutOff152Hz,
    .gain       = kPDM_DfOutputGain4,
};

extern codec_handle_t codecHandle;

/*
 * Function prototypes
 * */

static void DeInterleave(uint8_t *dataInput, uint8_t *dataOutput, uint16_t FrameSize, uint8_t ChannelNumber);


/*! @brief SAI Transmit IRQ handler.
 *
 * This function is used to handle or clear error state.
 */
static void SAI_UserTxIRQHandler(void)
{
    /* Clear the FEF (Tx FIFO underrun) flag. */
    SAI_TxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
    SAI_TxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);

    SDK_ISR_EXIT_BARRIER;
}

/*! @brief SAI Receive IRQ handler.
 *
 * This function is used to handle or clear error state.
 */
static void SAI_UserRxIRQHandler(void)
{
    /* Clear the FEF (Rx FIFO overflow) flag. */
    SAI_RxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
    SAI_RxSoftwareReset(DEMO_SAI, kSAI_ResetTypeFIFO);

    SDK_ISR_EXIT_BARRIER;
}

/*! @brief SAI IRQ handler.
 *
 * This function checks FIFO overrun/underrun errors and clears error state.
 */
void SAI1_IRQHandler(void)
{
    if (DEMO_SAI->TCSR & kSAI_FIFOErrorFlag)
        SAI_UserTxIRQHandler();

    if (DEMO_SAI->RCSR & kSAI_FIFOErrorFlag)
        SAI_UserRxIRQHandler();
}

/*! @brief SAI EDMA transmit callback
 *
 * This function is called by the EDMA interface after a block of data has been
 * successfully written to the SAI.
 */
static void saiTxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    pcm_rtos_t *pcm = (pcm_rtos_t *)userData;
    BaseType_t reschedule;
    xSemaphoreGiveFromISR(pcm->semaphoreTX, &reschedule);
    portYIELD_FROM_ISR(reschedule);
}

static void pdmCallback(PDM_Type *base, pdm_edma_handle_t *handle, status_t status, void *userData)
{
    pcm_rtos_t *pcm = (pcm_rtos_t *)userData;
	BaseType_t reschedule;
	/* if value is 0, 0th buffer has the PDM data, 1st buffer is being copied by DMA */
	buffer_index ^= 1;
	xSemaphoreGiveFromISR(pcm->semaphoreRX, &reschedule);
	portYIELD_FROM_ISR(reschedule);
}

static void saiRxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    pcm_rtos_t *pcm = (pcm_rtos_t *)userData;
    BaseType_t reschedule;
    xSemaphoreGiveFromISR(pcm->semaphoreRX, &reschedule);
    portYIELD_FROM_ISR(reschedule);
}

void streamer_pcm_init(void)
{
	NVIC_SetPriority(DEMO_PDM_IRQ, 5U);
	NVIC_SetPriority(DMA1_DMA17_IRQn, 4U);
    /* Create DMA handle. */
    EDMA_CreateHandle(&pcmHandle.dmaRxHandle, DEMO_DMA, DEMO_PDM_EDMA_CHANNEL_1);
    /* Setup pdm */
    PDM_Init(DEMO_PDM, &pdmConfig);
    pcmHandle.isFirstRx = 1;
    EnableIRQ(DEMO_PDM_IRQ);
}

pcm_rtos_t *streamer_pcm_open(uint32_t num_buffers)
{
    pcmHandle.semaphoreTX = xSemaphoreCreateBinary();
//    SAI_TransferTxCreateHandleEDMA(DEMO_SAI, &pcmHandle.saiTxHandle, saiTxCallback, (void *)&pcmHandle,
//                                   &pcmHandle.dmaTxHandle);
    return &pcmHandle;
}

pcm_rtos_t *streamer_pcm_rx_open(uint32_t num_buffers)
{
    pcmHandle.semaphoreRX = xSemaphoreCreateBinary();
    PDM_TransferCreateHandleEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, pdmCallback, (void *)&pcmHandle, &pcmHandle.dmaRxHandle);
	PDM_TransferInstallEDMATCDMemory(&pcmHandle.pdmRxHandle, s_edmaTcd_0, 2);
	PDM_TransferSetChannelConfigEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, DEMO_PDM_ENABLE_CHANNEL_MIC0, &channelConfig);
#if (NUM_OF_CH==2)
	PDM_TransferSetChannelConfigEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, DEMO_PDM_ENABLE_CHANNEL_MIC1, &channelConfig);
#elif (NUM_OF_CH==3)
	PDM_TransferSetChannelConfigEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, DEMO_PDM_ENABLE_CHANNEL_MIC1, &channelConfig);
	PDM_TransferSetChannelConfigEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, DEMO_PDM_ENABLE_CHANNEL_MIC2, &channelConfig);
#endif
	if (PDM_SetSampleRateConfig(DEMO_PDM, DEMO_PDM_CLK_FREQ, DEMO_PDM_SAMPLE_RATE) != kStatus_Success)
	{
	   PRINTF("PDM configure sample rate failed.\r\n");
	   return NULL;
	}
	PDM_Reset(DEMO_PDM);
    return &pcmHandle;
}

void streamer_pcm_start(pcm_rtos_t *pcm)
{
    /* Interrupts already enabled - nothing to do.
     * App/streamer can begin writing data to SAI. */
}

void streamer_pcm_close(pcm_rtos_t *pcm)
{
    /* Stop playback.  This will flush the SAI transmit buffers. */
    SAI_TransferTerminateSendEDMA(DEMO_SAI, &pcm->saiTxHandle);
    vSemaphoreDelete(pcmHandle.semaphoreTX);
}

void streamer_pcm_rx_close(pcm_rtos_t *pcm)
{
    /* Stop playback.  This will flush the SAI transmit buffers. */
    PDM_TransferTerminateReceiveEDMA(DEMO_PDM, &pcm->pdmRxHandle);
    vSemaphoreDelete(pcmHandle.semaphoreRX);
}

int streamer_pcm_write(pcm_rtos_t *pcm, uint8_t *data, uint32_t size)
{
    pcm->saiTx.dataSize = size;
    pcm->saiTx.data     = data;

    /* Ensure write size is a multiple of 32, otherwise EDMA will assert
     * failure.  Round down for the last chunk of a file/stream. */
    if (size % 32)
    {
        pcm->saiTx.dataSize = size - (size % 32);
    }

    /* Start transfer */
    DCACHE_CleanByRange((uint32_t)pcm->saiTx.data, pcm->saiTx.dataSize);
    if (SAI_TransferSendEDMA(DEMO_SAI, &pcm->saiTxHandle, &pcm->saiTx) == kStatus_SAI_QueueFull)
    {
        /* Wait for transfer to finish */
        if (xSemaphoreTake(pcm->semaphoreTX, portMAX_DELAY) != pdTRUE)
        {
            return -1;
        }
        SAI_TransferSendEDMA(DEMO_SAI, &pcm->saiTxHandle, &pcm->saiTx);
    }

    return 0;
}

void streamer_read_stop()
{
	PDM_TransferTerminateReceiveEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle);
	PDM_Reset(DEMO_PDM);
	pcmHandle.pdmRxHandle.receivedBytes = 0;
	buffer_index = 0;
}

void streamer_read_start()
{
	pcmHandle.pdmXfer_pt = pdmXfer;
	PDM_TransferReceiveEDMA(DEMO_PDM, &pcmHandle.pdmRxHandle, pcmHandle.pdmXfer_pt);
	pcmHandle.isFirstRx = 0;
}

int streamer_pcm_read(pcm_rtos_t *pcm, uint8_t *data, uint8_t *next_buffer, uint32_t size)
{
    /* Start the first transfer */
    if (pcm->isFirstRx)
    {
    	/* pdmXfer uses circular linked list buffer for storing Audio samples continuously */
    	pcm->pdmXfer_pt = pdmXfer;
    	PDM_TransferReceiveEDMA(DEMO_PDM, &pcm->pdmRxHandle, pcm->pdmXfer_pt);
        pcm->isFirstRx = 0;
    }

    /* Wait for transfer to finish */
//    if (xSemaphoreTake(pcm->semaphoreRX, portMAX_DELAY) != pdTRUE)
    if (xSemaphoreTake(pcm->semaphoreRX, 3000) != pdTRUE)
    {
    	PRINTF("PDM transfer never finishes");
        return -1;
    }

    /*
     * convert to 16bit samples and de-interleave the multi-channel samples like
     * A1, B1, C1, A2, B2 ... => A1, A2, A3, .. B1, B2,...
     */
    DeInterleave((uint8_t *)&(pdmXfer[buffer_index ^ 1].data[0]), data, 160, NUM_OF_CH);

#ifdef DEBUG_AUDIO_SAMPLE
    /* used for visualizing raw audio samples */
    static uint8_t msb;
    msb = *(data+BUFFER_SIZE/4);
    static uint8_t lsb;
    lsb = *(data+BUFFER_SIZE/4+1);
    int16_t channel = lsb<<8 | msb;
	static int count = 0;
	count++;
	if (count == 2) {
		count = 0;
		PRINTF("%d\n", channel+32767);
	}
#endif
    if (pcm->dummy_tx_enable)
        SAI_TxEnable(DEMO_SAI, true);

    return 0;
}

/*! @brief Map an integer sample rate (Hz) to internal SAI enum */
static sai_sample_rate_t _pcm_map_sample_rate(uint32_t sample_rate)
{
    switch (sample_rate)
    {
        case 8000:
            return kSAI_SampleRate8KHz;
        case 11025:
            return kSAI_SampleRate11025Hz;
        case 12000:
            return kSAI_SampleRate12KHz;
        case 16000:
            return kSAI_SampleRate16KHz;
        case 24000:
            return kSAI_SampleRate24KHz;
        case 22050:
            return kSAI_SampleRate22050Hz;
        case 32000:
            return kSAI_SampleRate32KHz;
        case 44100:
            return kSAI_SampleRate44100Hz;
        case 48000:
        default:
            return kSAI_SampleRate48KHz;
    }
}

/*! @brief Map an integer bit width (bits) to internal SAI enum */
static sai_word_width_t _pcm_map_word_width(uint32_t bit_width)
{
    switch (bit_width)
    {
        case 8:
            return kSAI_WordWidth8bits;
        case 16:
            return kSAI_WordWidth16bits;
        case 24:
            return kSAI_WordWidth24bits;
        case 32:
            return kSAI_WordWidth32bits;
        default:
            return kSAI_WordWidth16bits;
    }
}

/*! @brief Map an integer number of channels to internal SAI enum */
static sai_mono_stereo_t _pcm_map_channels(uint8_t num_channels)
{
    if (num_channels >= 2)
        return kSAI_Stereo;
    else
        return kSAI_MonoRight;
}

int streamer_pcm_setparams(
    pcm_rtos_t *pcm, uint32_t sample_rate, uint32_t bit_width, uint8_t num_channels, bool transfer, bool dummy_tx)
{
    sai_transfer_format_t format = {0};
    sai_transceiver_t saiConfig, saiConfig2;
    uint32_t masterClockHz = 0U;

    pcm->sample_rate     = sample_rate;
    pcm->bit_width       = bit_width;
    pcm->num_channels    = num_channels;
    pcm->dummy_tx_enable = dummy_tx;

    masterClockHz = streamer_set_master_clock(sample_rate);

    format.channel       = 0U;
    format.bitWidth      = _pcm_map_word_width(bit_width);
    format.sampleRate_Hz = _pcm_map_sample_rate(sample_rate);
    format.stereo        = _pcm_map_channels(num_channels);
#if (defined FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER && FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER)
    format.masterClockHz = masterClockHz;
#endif

    /* I2S transfer mode configurations */
    if (transfer)
    {
        SAI_GetClassicI2SConfig(&saiConfig, _pcm_map_word_width(bit_width), format.stereo, 1U << DEMO_SAI_CHANNEL);
        /* If there wasn't first RX transfer, we need to set SAI mode to Async */
        if (pcm->isFirstRx)
        {
            saiConfig.syncMode = kSAI_ModeAsync;
        }
        else
        /* Otherwise we need to sync the SAI for the loopback */
        {
            saiConfig.syncMode = kSAI_ModeSync;
        }
        saiConfig.masterSlave = kSAI_Master;

        SAI_TransferTxSetConfigEDMA(DEMO_SAI, &pcmHandle.saiTxHandle, &saiConfig);
        /* set bit clock divider */
        SAI_TxSetBitClockRate(DEMO_SAI, masterClockHz, _pcm_map_sample_rate(sample_rate),
                              _pcm_map_word_width(bit_width), DEMO_CHANNEL_NUM);
        /* Enable SAI transmit and FIFO error interrupts. */
        SAI_TxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
    }
    else
    {
        /* I2S receive mode configurations */
        SAI_GetClassicI2SConfig(&saiConfig, _pcm_map_word_width(bit_width), format.stereo, 1U << DEMO_SAI_CHANNEL);
        if (dummy_tx)
            SAI_GetClassicI2SConfig(&saiConfig2, _pcm_map_word_width(bit_width), format.stereo, 1U << DEMO_SAI_CHANNEL);
        saiConfig.syncMode    = kSAI_ModeAsync;
        saiConfig.masterSlave = kSAI_Master;

//        SAI_TransferRxSetConfigEDMA(DEMO_SAI, &pcmHandle.saiRxHandle, &saiConfig);
        /* set bit clock divider */
        SAI_RxSetBitClockRate(DEMO_SAI, masterClockHz, _pcm_map_sample_rate(sample_rate),
                              _pcm_map_word_width(bit_width), DEMO_CHANNEL_NUM);

        if (dummy_tx)
        {
            saiConfig2.syncMode    = kSAI_ModeSync;
            saiConfig2.masterSlave = kSAI_Master;
            SAI_TransferTxSetConfigEDMA(DEMO_SAI, &pcmHandle.saiTxHandle, &saiConfig2);
            /* set bit clock divider */
            SAI_TxSetBitClockRate(DEMO_SAI, masterClockHz, _pcm_map_sample_rate(sample_rate),
                                  _pcm_map_word_width(bit_width), DEMO_CHANNEL_NUM);
            /* Enable SAI transmit and FIFO error interrupts. */
            SAI_TxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
        }

        /* Enable SAI transmit and FIFO error interrupts. */
        SAI_RxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
    }

    CODEC_SetMute(&codecHandle, kCODEC_PlayChannelHeadphoneRight | kCODEC_PlayChannelHeadphoneLeft, true);

    CODEC_SetFormat(&codecHandle, masterClockHz, format.sampleRate_Hz, format.bitWidth);

    CODEC_SetMute(&codecHandle, kCODEC_PlayChannelHeadphoneRight | kCODEC_PlayChannelHeadphoneLeft, false);

    return 0;
}

void streamer_pcm_getparams(pcm_rtos_t *pcm, uint32_t *sample_rate, uint32_t *bit_width, uint8_t *num_channels)
{
    *sample_rate  = pcm->sample_rate;
    *bit_width    = pcm->bit_width;
    *num_channels = pcm->num_channels;
}

int streamer_pcm_mute(pcm_rtos_t *pcm, bool mute)
{
    CODEC_SetMute(&codecHandle, kCODEC_PlayChannelHeadphoneRight | kCODEC_PlayChannelHeadphoneLeft, mute);

    return 0;
}

int streamer_pcm_set_volume(uint32_t volume)
{
    uint32_t adjvol;

    /* Normalize 0-100 volume to 0-255 scale used by WM8960. */
    //    adjvol = (uint32_t)(((float)volume / (float)100) * (float)256);
    adjvol = (uint32_t)((((float)volume / (float)100) * (float)79) + 48);

    CODEC_SetVolume(&codecHandle, kCODEC_PlayChannelHeadphoneRight | kCODEC_PlayChannelHeadphoneLeft, adjvol);

    return 0;
}

int streamer_set_master_clock(int sample_rate)
{
    int master_clock;
#if DEMO_CODEC_CS42448
    int divider    = DEMO_SAI1_CLOCK_SOURCE_DIVIDER;
    int predivider = DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER;
#endif

//    if (sample_rate % 8000 == 0 || sample_rate % 6000 == 0)
//    {
//        /* Configure Audio PLL clock to 786.432 MHz to  to be divisible by 48000 Hz */
//        const clock_audio_pll_config_t audioPllConfig48 = {
//            .loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
//            .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
//            .numerator   = 96,  /* 30 bit numerator of fractional loop divider. */
//            .denominator = 125, /* 30 bit denominator of fractional loop divider */
//        };
//        CLOCK_InitAudioPll(&audioPllConfig48);
//    }
//    else
//    {
//        /* Configure Audio PLL clock to 722.5344 MHz to be divisible by 44100 Hz */
//        const clock_audio_pll_config_t audioPllConfig = {
//            .loopDivider = 30,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
//            .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
//            .numerator   = 66,  /* 30 bit numerator of fractional loop divider. */
//            .denominator = 625, /* 30 bit denominator of fractional loop divider */
//        };
//        CLOCK_InitAudioPll(&audioPllConfig);
//    }
#if DEMO_CODEC_CS42448
    switch (sample_rate)
    {
        case 11025:
        case 12000:
        case 24000:
        {
            divider = 63;
            break;
        }
        case 8000:
        {
            predivider = 1;
        }
        case 16000:
        {
            divider = 47;
            break;
        }
        case 32000:
        {
            divider = 23;
            break;
        }
        case 22050:
        case 44100:
        case 48000:
        default:
            divider = 31;
            break;
    }

    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, predivider);
    CLOCK_SetDiv(kCLOCK_Sai1Div, divider);
    master_clock = CLOCK_GetFreq(kCLOCK_AudioPllClk) / (divider + 1U) / (predivider + 1U);
#else
    //master_clock = DEMO_SAI_CLK_FREQ;
#endif
    //return master_clock;
    return 786432000U;
}

/****************************************************************************************/
/*                                                                                      */
/*                          Functions                                                   */
/*                                                                                      */
/****************************************************************************************/
//  de-Interleave Multichannel signal
//   example:  A1.B1.C1.A2.B2.C2.A3.B3.C3....An.Bn.Cn   (3 Channels case : A, B, C)
//             will become
//             A1.A2.A3....An.B1.B2.B3....Bn.C1.C2.C3....Cn

// Simple helper function for de-interleaving Multichannel stream
// The caller function shall ensure that all arguments are correct.
// In place processing not supported
static void DeInterleave(uint8_t *dataInput, uint8_t *dataOutput, uint16_t FrameSize, uint8_t ChannelNumber)
{
	int index_o=0;
    int index_i;
    int16_t sample;
    for (uint8_t ichan = 0; ichan < ChannelNumber; ichan++)
    {
        for (uint16_t i = 0; i < FrameSize; i++)
        {
            index_i = ChannelNumber*i*4+ichan*4+2;
            sample = (dataInput[index_i+1]<<8) | dataInput[index_i];
        	dataOutput[index_o] = sample & 0xFF;
        	dataOutput[index_o+1] = (sample >> 8) & 0xFF;
            index_o = index_o+2;
        }
    }
    return;
}


