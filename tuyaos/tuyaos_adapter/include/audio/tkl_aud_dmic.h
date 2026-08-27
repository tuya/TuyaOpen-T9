/**
 * @file tkl_aud_dmic.h
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

#ifndef __TKL_AUD_DMIC_H__
#define __TKL_AUD_DMIC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Public macros -------------------------------------------------------------*/
#define AUDIA_DMIC_INVALID_GAIN_VALUE   0xFFFFFFFF

/* Public typedefs -----------------------------------------------------------*/
typedef struct {
    TUYA_AUDIO_DMIC_CHAN_E chan;
    UINT8_T sample_bits;            // T5 SMP, only support 16bits
    UINT32_T sample_rate;
    UINT32_T frame_time_ms;
    TKL_AUD_INPUT_CB upper_cb;
    VOID_T *args;
} TKL_AUD_DMIC_CFG_T;

/**
 * @brief DMIC physical channel used by set_gain
 * @note On this platform maps to ADC dig path CH0/1/2 (DMIC shares ADC dig gain).
 */
typedef enum {
    TUYA_AUDIO_DMIC_CHANNEL_0,
    TUYA_AUDIO_DMIC_CHANNEL_1,
    TUYA_AUDIO_DMIC_CHANNEL_2,
    TUYA_AUDIO_DMIC_CHANNEL_3,
    TUYA_AUDIO_DMIC_CHANNEL_4,
    TUYA_AUDIO_DMIC_CHANNEL_5,
    TUYA_AUDIO_DMIC_CHANNEL_6,
    TUYA_AUDIO_DMIC_CHANNEL_7,
    TUYA_AUDIO_DMIC_CHANNEL_8,
    TUYA_AUDIO_DMIC_CHANNEL_9,
    TUYA_AUDIO_DMIC_MIC_CHANNEL_MAX
} TUYA_AUDIO_DMIC_CHANNEL_E;

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief Configure DMIC pinmux before init
 * @param[in] clk_gpio DMIC clock gpio id
 * @param[in] data_gpio DMIC data gpio id
 * @return OPRT_OK on success, error code on failure
 * @note Call this API before tkl_aud_dmic_init(). If not called, default
 *       board pin definition is used.
 */
OPERATE_RET tkl_aud_dmic_pinmux_config(UINT32_T clk_gpio, UINT32_T data_gpio);

/**
 * @brief Initialize audio digital (DMIC) input
 * @param[in] port Digital audio port number
 * @param[in] config DMIC configuration
 * @return OPRT_OK on success, error code on failure
 * @note Gain is set via tkl_aud_dmic_set_gain().
 */
OPERATE_RET tkl_aud_dmic_init(TUYA_AUDIO_DMIC_PORT_E port, TKL_AUD_DMIC_CFG_T *config);

/**
 * @brief Start audio digital (DMIC) capture
 * @param[in] port Digital audio port number
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_aud_dmic_start(TUYA_AUDIO_DMIC_PORT_E port);

/**
 * @brief Set DMIC dig/ana gain for one physical channel
 * @param[in] port Digital audio port number
 * @param[in] channel TUYA_AUDIO_DMIC_CHANNEL_0/1/2 -> ADC dig path CH0/1/2
 * @param[in] gain gain[0]=dig dB, gain[1]=ana dB (when num>=2). Pass
 *                 AUDIA_DMIC_INVALID_GAIN_VALUE to leave dig or ana unchanged.
 * @param[in] num 1 (dig only) or 2 (dig then ana)
 * @return OPRT_OK on success, error code on failure
 * @note Dig is the effective gain for DMIC. Ana maps to analog MIC PGA and
 *       has little/no effect on digital-mic amplitude on this platform.
 * @note Dig and ana can be updated independently. At least one must be valid.
 */
OPERATE_RET tkl_aud_dmic_set_gain(TUYA_AUDIO_DMIC_PORT_E port, TUYA_AUDIO_DMIC_CHANNEL_E channel,
                                  INT32_T gain[], UINT32_T num);

/**
 * @brief Stop audio digital (DMIC) capture
 * @param[in] port Digital audio port number
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_aud_dmic_stop(TUYA_AUDIO_DMIC_PORT_E port);

/**
 * @brief Deinitialize audio digital (DMIC) input
 * @param[in] port Digital audio port number
 * @return none
 */
VOID_T tkl_aud_dmic_deinit(TUYA_AUDIO_DMIC_PORT_E port);

#ifdef __cplusplus
}
#endif

#endif /*__TKL_AUD_DMIC_H__ */
