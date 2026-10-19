/**
 * @file tkl_aud_dac.h
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

#ifndef __TKL_AUD_DAC_H__
#define __TKL_AUD_DAC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Public macros -------------------------------------------------------------*/

/* Public typedefs -----------------------------------------------------------*/
#define AUDIA_DAC_INVALID_GAIN_VALUE   0xFFFFFFFF

typedef VOID (*TKL_AUD_DAC_FRAME_CB)(TUYA_AUDIO_DAC_FRAME_EVT_E event, VOID_T *args);

/**
 * @brief DAC digital input source selection (maps to OEM aud_dac_source_t on V2).
 * @note AUTO picks CALL for 8k/16k voice rates, A2DP for media rates (44.1k+ / 48k).
 */
typedef enum {
    TKL_AUD_DAC_SRC_AUTO = 0,   /**< Infer from sample_rate (default, backward compatible) */
    TKL_AUD_DAC_SRC_MEDIA,      /**< Music/media path -> AUD_DAC_SOURCE_A2DP */
    TKL_AUD_DAC_SRC_VOICE,      /**< Voice/call path -> AUD_DAC_SOURCE_CALL */
    TKL_AUD_DAC_SRC_HINT,       /**< Hint/prompt path -> AUD_DAC_SOURCE_HINT */
} TKL_AUD_DAC_SRC_E;

typedef struct {
    UINT8_T  chan_num;      // now, only one speaker
    UINT8_T sample_bits;    // T5 SMP, only support 16bits
    UINT32_T sample_rate;
    UINT32_T frame_time_ms;
    TKL_AUD_DAC_FRAME_CB frame_cb;
    VOID_T              *args;
    TKL_AUD_DAC_SRC_E src;  /**< Digital source; 0=AUTO */
}TKL_AUD_DAC_CFG_T;

/**
 * @brief audio dac channel
 *
 */
typedef enum {
    TUYA_AUDIO_DAC_CHANNEL_0,
    TUYA_AUDIO_DAC_CHANNEL_1,
    TUYA_AUDIO_DAC_CHANNEL_2,
    TUYA_AUDIO_DAC_CHANNEL_3,
    TUYA_AUDIO_DAC_CHANNEL_4,
    TUYA_AUDIO_DAC_CHANNEL_5,
    TUYA_AUDIO_DAC_CHANNEL_6,
    TUYA_AUDIO_DAC_CHANNEL_7,
    TUYA_AUDIO_DAC_CHANNEL_8,
    TUYA_AUDIO_DAC_CHANNEL_9,
    TUYA_AUDIO_DAC_CHANNEL_MAX
} TUYA_AUDIO_DAC_CHANNEL_E;

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/
/**
 * @brief Initialize the DAC playback context and hardware resources.
 * @param[in] config DAC configuration used to set sample rate, frame size,
 *                   and frame callback. Gain is set via tkl_aud_dac_set_gain().
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @return OPRT_OK on success, other error codes on failure.
 * @note The current platform only supports 16-bit PCM output and forces the
 *       playback channel count to one onboard speaker.
 * @note This function must be called before `tkl_aud_dac_start()`.
 */
OPERATE_RET tkl_aud_dac_init(TUYA_AUDIO_DAC_PORT_E port, TKL_AUD_DAC_CFG_T *config);

/**
 * @brief Deinitialize the DAC driver instance.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @return OPRT_OK on success, other error codes on failure.
 * @note If playback is still running, this function stops it first and then
 *       releases DMA, DAC, callback, and power-management resources.
 */
OPERATE_RET tkl_aud_dac_deinit(TUYA_AUDIO_DAC_PORT_E port);

/**
 * @brief Start DAC playback.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @return OPRT_OK on success, other error codes on failure.
 * @note The driver pre-fills the DMA ring buffer with silence before enabling
 *       DMA and DAC output to reduce pop noise on startup.
 * @note Calling this function after playback has already started returns
 *       success without reinitializing the hardware.
 */
OPERATE_RET tkl_aud_dac_start(TUYA_AUDIO_DAC_PORT_E port);

/**
 * @brief Stop DAC playback.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @return OPRT_OK on success, other error codes on failure.
 * @note This function stops DMA transfer first, mutes the DAC output, and
 *       then stops the DAC hardware.
 * @note Calling this function after playback has already stopped returns
 *       success without additional hardware operations.
 */
OPERATE_RET tkl_aud_dac_stop(TUYA_AUDIO_DAC_PORT_E port);

/**
 * @brief Set the DAC hardware gain.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @param[in] channel TUYA_AUDIO_DAC_CHANNEL_0/1 -> L/R. On this platform OEM dig
 *                    API always programs L+R together; ana is global.
 * @param[in] gain gain[0]=dig dB, gain[1]=ana dB (when num>=2). Pass
 *                 AUDIA_DAC_INVALID_GAIN_VALUE to leave dig or ana unchanged.
 * @param[in] num 1 (dig only) or 2 (dig then ana)
 * @return OPRT_OK on success, other error codes on failure.
 * @note Dig and ana can be updated independently. At least one must be valid.
 * @note Dig <= silence threshold mutes; otherwise unmutes when dig is applied.
 */
OPERATE_RET tkl_aud_dac_set_gain(TUYA_AUDIO_DAC_PORT_E port, TUYA_AUDIO_DAC_CHANNEL_E channel, INT32_T gain[], UINT32_T num);

/**
 * @brief Write PCM data into the DAC DMA ring buffer.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @param[in] buffer Pointer to the PCM data buffer.
 * @param[in] len PCM data length in bytes.
 * @return OPRT_OK on success, `OPRT_OS_ADAPTER_DAC_BUSY` when the ring buffer
 *         does not have enough free space, or other error codes on failure.
 * @note The DAC must already be in the started state before this function is
 *       called.
 * @note Data is copied into the internal DMA ring buffer, so the caller must
 *       ensure that enough free space is available for the requested length.
 */
OPERATE_RET tkl_aud_dac_write(TUYA_AUDIO_DAC_PORT_E port, UINT8_T *buffer, UINT32_T len);

/**
 * @brief Get the configured DAC frame size in bytes.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @return Frame size in bytes, or `0` if the DAC driver is not initialized.
 */
UINT32_T tkl_aud_dac_get_frame_size(TUYA_AUDIO_DAC_PORT_E port);


#ifdef __cplusplus
}
#endif

#endif /*__TKL_AUD_DAC_H__ */

