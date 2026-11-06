/**
 * @file tkl_aud_dac.h
 * @brief Audio DAC (playback) interface.
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
 * @note On the reference implementation (T5/SMP) the @p port argument is ignored
 *       by every function below - the board exposes a single DAC path.
 */

#ifndef __TKL_AUD_DAC_H__
#define __TKL_AUD_DAC_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Public macros -------------------------------------------------------------*/
/**
 * Sentinel for a gain entry that carries no gain. How a port treats such an
 * entry is port specific.
 */
#define AUDIO_DAC_INVALID_GAIN_VALUE   0xFFFFFFFF

/* Public typedefs -----------------------------------------------------------*/
/**
 * @brief Configuration for one DAC playback instance.
 *
 * @note The initial gain is **not** part of this configuration; set it with
 *       @ref tkl_aud_dac_set_gain after @ref tkl_aud_dac_init.
 * @note The reference implementation accepts 16-bit samples only. @p dac_chan
 *       is a channel bitmap and one or two channels are supported: a single
 *       enabled channel drives L, two enabled channels drive L/R.
 */
typedef struct {
    TUYA_AUDIO_DAC_CHANNEL_E dac_chan;          /**< DAC channel bitmap; see @ref TUYA_AUDIO_DAC_CHANNEL_E */
    TUYA_AUDIO_SAMPLE_BITS_E sample_bits;   /**< Sample width in bits; see @ref TUYA_AUDIO_SAMPLE_BITS_E */
    UINT32_T sample_rate;                   /**< Sample rate in Hz */
    UINT32_T frame_time_ms;                 /**< Length of one frame in milliseconds */
    TUYA_AUDIO_OUTPUT_CB frame_cb;            /**< Frame callback; see @ref TUYA_AUDIO_OUTPUT_CB */
    VOID_T *args;                           /**< Opaque pointer handed back to @p frame_cb */
} TUYA_AUDIO_DAC_BASE_CFG_T;

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/
/**
 * @brief Initialize the DAC playback instance for a port.
 *
 * @param[in] port   DAC port to configure.
 * @param[in] config Playback configuration. Must not be NULL.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note Must be called before @ref tkl_aud_dac_start.
 * @note Calling this again on an already initialized port reconfigures it and
 *       returns OPRT_OK without leaking the previous resources.
 * @note Whether @p config->frame_cb may be NULL is not settled by this header;
 *       a port must document its own answer. See the module documentation.
 */
OPERATE_RET tkl_aud_dac_init(TUYA_AUDIO_DAC_PORT_E port, CONST TUYA_AUDIO_DAC_BASE_CFG_T *config);

/**
 * @brief Deinitialize the DAC playback instance.
 *
 * @param[in] port DAC port.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note If playback is still running it is stopped first, then the DMA channel,
 *       the DAC hardware, the callback and the ring buffer are released.
 */
OPERATE_RET tkl_aud_dac_deinit(TUYA_AUDIO_DAC_PORT_E port);

/**
 * @brief Start playback.
 *
 * @param[in] port DAC port.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note The reference implementation pre-fills the ring buffer with silence
 *       before enabling DMA, to suppress the pop that an abrupt start causes.
 * @note Idempotent: calling this while already playing returns OPRT_OK without
 *       reconfiguring the hardware.
 */
OPERATE_RET tkl_aud_dac_start(TUYA_AUDIO_DAC_PORT_E port);

/**
 * @brief Stop playback.
 *
 * @param[in] port DAC port.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note After this returns the frame callback no longer fires.
 * @note Idempotent: calling this while already stopped returns OPRT_OK.
 */
OPERATE_RET tkl_aud_dac_stop(TUYA_AUDIO_DAC_PORT_E port);

/**
 * @brief Set the DAC output gain.
 *
 * @param[in] port    DAC port.
 * @param[in] channel Channel whose gain is set; one bit of @ref TUYA_AUDIO_DAC_CHANNEL_E.
 * @param[in] gain    Gain values; @p num entries.
 * @param[in] num     Number of entries in @p gain.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note Unlike @c audio_adc and @c audio_dmic the playback path has no
 *       per-channel gain axis; a port may ignore @p channel and apply the gain
 *       to the whole output. How the entries in @p gain map to the hardware
 *       gain registers is not specified by this header.
 * @note An entry equal to AUDIO_DAC_INVALID_GAIN_VALUE carries no gain; how a
 *       port treats such an entry is port specific.
 * @note The value is remapped through a non-linear hardware gain curve before it
 *       reaches the DAC gain register, so a given input value does not scale the
 *       output linearly.
 * @note A gain of `0` mutes the DAC output and a non-zero value unmutes it.
 *       Muting is sticky: a later non-zero gain is what cancels it.
 * @note The valid gain range and its mapping to the hardware gain register are
 *       platform defined. The test spec's `0x00`..`0x3F` range described the
 *       removed `tkl_aud_dac_set_vol()` and does not necessarily apply here.
 * @note Must be called after @ref tkl_aud_dac_init.
 */
OPERATE_RET tkl_aud_dac_set_gain(TUYA_AUDIO_DAC_PORT_E port, TUYA_AUDIO_DAC_CHANNEL_E channel, INT32_T gain[], UINT32_T num);

/**
 * @brief Write PCM data into the playback ring buffer.
 *
 * @param[in] port   DAC port.
 * @param[in] buffer PCM data to play. Must not be NULL.
 * @param[in] len    Length of @p buffer in bytes.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *         OPRT_OS_ADAPTER_DAC_BUSY is returned when the ring buffer has no room
 *         for @p len bytes.
 *
 * @note Playback must already be started; writing before @ref tkl_aud_dac_start
 *       fails.
 * @note The data is copied into the internal ring buffer, so the caller may
 *       reuse @p buffer as soon as this returns.
 */
OPERATE_RET tkl_aud_dac_write(TUYA_AUDIO_DAC_PORT_E port, CONST UINT8_T *buffer, UINT32_T len);

/**
 * @brief Get the frame size in bytes for the configured playback format.
 *
 * @param[in] port DAC port.
 *
 * @return Frame size in bytes, or 0 if the port is not initialized.
 */
UINT32_T tkl_aud_dac_get_frame_size(TUYA_AUDIO_DAC_PORT_E port);


#ifdef __cplusplus
}
#endif

#endif /*__TKL_AUD_DAC_H__ */

