/**
 * @file tkl_aud_adc.h
 * @brief
 *
 * @copyright Copyright (c) 2025 Tuya Inc. All Rights Reserved.
 *
 * Permission is hereby granted, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), Under the premise of complying
 * with the license of the third-party open source software contained in the software,
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software.
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 */

#ifndef __TKL_AUD_ADC_H__
#define __TKL_AUD_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Public macros -------------------------------------------------------------*/

/* Public typedefs -----------------------------------------------------------*/
#define AUDIA_ADC_INVALID_GAIN_VALUE   0xFFFFFFFF

typedef struct {
    TUYA_AUDIO_ADC_CHAN_E chan;
    UINT8_T sample_bits;            // T5 SMP, only support 16bits
    UINT32_T sample_rate;
    UINT32_T frame_time_ms;
    TKL_AUD_INPUT_CB upper_cb;
    VOID_T *args;
} TKL_AUD_ADC_CFG_T;

/**
 * @brief audio dac channel
 *
 */
typedef enum {
    TUYA_AUDIO_ADC_CHANNEL_0,
    TUYA_AUDIO_ADC_CHANNEL_1,
    TUYA_AUDIO_ADC_CHANNEL_2,
    TUYA_AUDIO_ADC_CHANNEL_3,
    TUYA_AUDIO_ADC_CHANNEL_4,
    TUYA_AUDIO_ADC_CHANNEL_5,
    TUYA_AUDIO_ADC_CHANNEL_6,
    TUYA_AUDIO_ADC_CHANNEL_7,
    TUYA_AUDIO_ADC_CHANNEL_8,
    TUYA_AUDIO_ADC_CHANNEL_9,
    TUYA_AUDIO_ADC_MIC_CHANNEL_MAX
} TUYA_AUDIO_ADC_CHANNEL_E;

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/
/**
* @brief Initialize audio ADC
*
* @param[in] config: ADC configuration including chan, sample_rate, sample_bits,
*                    frame_time_ms, upper_cb. Gain is set via tkl_aud_adc_set_gain().
*
* @note This API allocates context, configures ADC hardware, DMA ring buffer,
*       and registers flash operation notify handler. On BK7259 V2 the onboard
*       3ch frame layout is [mic1, mic2, mic3]: CHANNEL_L/R/LR select mic1/mic2/both,
*       CHANNEL_MIC3 selects the AEC-reference mic (index 2). Must be called
*       before tkl_aud_adc_start.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_init(TUYA_AUDIO_ADC_PORT_E port, TKL_AUD_ADC_CFG_T *config);

/**
* @brief Start audio ADC capture
*
* @note Starts DMA transfer and ADC hardware. After starting, DMA finish ISR
*       will be triggered periodically, calling upper_aud_adc_cb with PCM data.
*       Safe to call multiple times (idempotent).
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_start(TUYA_AUDIO_ADC_PORT_E port);

/**
* @brief Set audio ADC digital/analog gain for one MIC channel
*
* @param[in] port: ADC port
* @param[in] channel: physical MIC, TUYA_AUDIO_ADC_CHANNEL_0/1/2 -> MIC1/MIC2/MIC3
* @param[in] gain: gain[0]=dig dB, gain[1]=ana dB (when num>=2). Pass
*                  AUDIA_ADC_INVALID_GAIN_VALUE to leave dig or ana unchanged.
* @param[in] num: 1 (dig only) or 2 (dig then ana)
*
* @note Dig and ana can be updated independently. At least one of dig/ana must
*       be valid. No volume remapping is performed.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_set_gain(TUYA_AUDIO_ADC_PORT_E port, TUYA_AUDIO_ADC_CHANNEL_E channel, INT32_T gain[], UINT32_T num);


/**
* @brief Stop audio ADC capture
*
* @note Stops DMA transfer and ADC hardware. After stopping, DMA finish ISR
*       will no longer fire. Safe to call multiple times (idempotent).
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_stop(TUYA_AUDIO_ADC_PORT_E port);

/**
* @brief Deinitialize audio ADC
*
* @note Stops capture if running, releases DMA channel and ring buffer,
*       unregisters flash operation notify, frees context memory,
*       and restores CPU frequency.
*/
VOID_T tkl_aud_adc_deinit(TUYA_AUDIO_ADC_PORT_E port);


#ifdef __cplusplus
}
#endif

#endif /*__TKL_AUD_ADC_H__ */

