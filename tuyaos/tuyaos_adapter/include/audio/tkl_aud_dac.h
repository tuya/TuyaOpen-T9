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
    INT32_T gain[2];    /**< Hardware gain in dB: [0]=dig, [1]=ana when gain_num=2 */
    UINT32_T gain_num;  /**< Gain count: 2=dig+ana, 1=dig only */
    UINT8_T  chan_num;      // now, only one speaker
    UINT8_T sample_bits;    // T5 SMP, only support 16bits
    UINT32_T sample_rate;
    UINT32_T frame_time_ms;
    TKL_AUD_DAC_FRAME_CB frame_cb;
    VOID_T              *args;
    TKL_AUD_DAC_SRC_E src;  /**< Digital source; 0=AUTO */
    UINT32_T pa_pin; /**< Speaker PA enable GPIO number (active-high). */
}TKL_AUD_DAC_CFG_T;

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/
/**
 * @brief Initialize the DAC playback context and hardware resources.
 * @param[in] config DAC configuration used to set sample rate, frame size,
 *                   initial gain, and frame callback.
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
 * @param[in] gain Gain array in dB (platform-dependent count).
 * @param[in] num Number of gain values. On this platform (dig+ana): num=2,
 *                gain[0]=digital dB, gain[1]=analog dB. num=1 sets dig only.
 * @return OPRT_OK on success, other error codes on failure.
 * @note Dig gain <= -100 (silence) mutes the DAC; otherwise unmutes and applies.
 * @note No volume remapping is performed; caller passes hardware gain directly.
 */
OPERATE_RET tkl_aud_dac_set_gain(TUYA_AUDIO_DAC_PORT_E port, INT32_T gain[], UINT32_T num);

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

