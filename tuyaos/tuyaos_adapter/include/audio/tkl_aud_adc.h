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
typedef struct {
    TUYA_AUDIO_ADC_CHAN_E chan;
    UINT8_T sample_bits;            // T5 SMP, only support 16bits
    UINT32_T sample_rate;
    UINT32_T frame_time_ms;
    TKL_AUD_INPUT_CB upper_cb;
    VOID_T *args;
} TKL_AUD_ADC_CFG_T;

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
* @brief Set audio ADC digital/analog gain
*
* @param[in] gain: gain array in dB (platform-dependent count)
* @param[in] num:  number of gain values. On this platform (dig+ana): num=2,
*                  gain[0]=digital dB, gain[1]=analog dB. num=1 sets dig only.
*
* @note No volume remapping is performed; caller passes hardware gain directly.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_set_gain(TUYA_AUDIO_ADC_PORT_E port, INT32_T gain[], UINT32_T num);

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

