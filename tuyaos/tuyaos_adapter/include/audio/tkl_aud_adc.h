/**
 * @file tkl_aud_adc.h
 * @brief Analog audio ADC (microphone capture) interface.
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
#define AUDIO_ADC_INVALID_GAIN_VALUE   0xFFFFFFFF

/* Public typedefs -----------------------------------------------------------*/
/**
 * @brief Configuration for one analog ADC capture instance.
 *
 * @note The initial gain is **not** part of this configuration; set it with
 *       @ref tkl_aud_adc_set_gain after @ref tkl_aud_adc_init.
 * @note The reference implementation accepts 16-bit samples only.
 * @note On BK7259 (CONFIG_AUD_DRIVER_V2) the port argument is ignored: there is
 *       one capture path shared with the DMIC interface. @p chan is a bitmap of
 *       @ref TUYA_AUDIO_MIC_CHANNEL_E ADC bits (BIT(0)..BIT(2) map to lanes
 *       0..2 of the 3-lane frame); the callback receives only the selected
 *       lanes, interleaved in ascending lane order, and @c buffer->chan[]
 *       names them.
 */
typedef struct {
    TUYA_AUDIO_MIC_CHANNEL_E chan;
    TUYA_AUDIO_SAMPLE_BITS_E sample_bits;   /**< Sample width in bits; see @ref TUYA_AUDIO_SAMPLE_BITS_E */
    UINT32_T sample_rate;                   /**< Sample rate in Hz */
    UINT32_T frame_time_ms;                 /**< Length of one frame in milliseconds */
    TUYA_AUDIO_INPUT_CB upper_cb;           /**< Frame callback; must not be NULL */
    VOID_T *args;                           /**< Opaque pointer handed back to @p upper_cb */
} TUYA_AUDIO_ADC_BASE_CFG_T;

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/
/**
 * @brief Initialize the analog ADC capture instance for a port.
 *
 * @param[in] port   ADC port to configure.
 * @param[in] config Capture configuration. Must not be NULL, and
 *                   @p config->upper_cb must not be NULL.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note Must be called before @ref tkl_aud_adc_start.
 * @note Calling this again on an already initialized port reconfigures it and
 *       returns OPRT_OK without leaking the previous resources. On this platform
 *       a different sample rate / bits / frame time can only be applied while
 *       the ADC is the sole user of the shared capture path; otherwise
 *       OPRT_COM_ERROR is returned and the running stream is left untouched.
 */
OPERATE_RET tkl_aud_adc_init(TUYA_AUDIO_ADC_PORT_E port, CONST TUYA_AUDIO_ADC_BASE_CFG_T *config);

/**
 * @brief Start capturing.
 *
 * @param[in] port ADC port.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note The first frame callback arrives roughly @p frame_time_ms after this
 *       call; subsequent frames follow at the same interval.
 * @note Idempotent: calling this while already capturing returns OPRT_OK
 *       without reconfiguring the hardware.
 */
OPERATE_RET tkl_aud_adc_start(TUYA_AUDIO_ADC_PORT_E port);

/**
 * @brief Set the ADC gain for one audio channel.
 *
 * @param[in] port    ADC port.
 * @param[in] channel Audio channel whose gain to set; one bit of
 *                    @ref TUYA_AUDIO_MIC_CHANNEL_E (ADC bits BIT(0)..BIT(2)).
 * @param[in] gain    Gain values; the first entry is the digital gain in dB and
 *                    the optional second entry the analog gain in dB.
 * @param[in] num     Number of entries in @p gain (1 or 2).
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note An entry equal to AUDIO_ADC_INVALID_GAIN_VALUE carries no gain; on this
 *       platform that entry is skipped, so dig and ana can be updated
 *       independently.
 * @note The valid gain range and its mapping to the hardware gain register are
 *       platform defined. The test spec's `0x00`..`0x3F` range described the
 *       removed `tkl_aud_adc_set_vol()` and does not necessarily apply here.
 * @note Must be called after @ref tkl_aud_adc_init.
 */
OPERATE_RET tkl_aud_adc_set_gain(TUYA_AUDIO_ADC_PORT_E port, TUYA_AUDIO_MIC_CHANNEL_E channel, INT32_T gain[], UINT32_T num);

/**
 * @brief Stop capturing.
 *
 * @param[in] port ADC port.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note After this returns the frame callback no longer fires.
 * @note Idempotent: calling this while already stopped returns OPRT_OK.
 */
OPERATE_RET tkl_aud_adc_stop(TUYA_AUDIO_ADC_PORT_E port);

/**
 * @brief Deinitialize the analog ADC capture instance.
 *
 * @param[in] port ADC port.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note If capture is still running it is stopped first, then the DMA channel,
 *       the ring buffer and the context are released.
 */
OPERATE_RET tkl_aud_adc_deinit(TUYA_AUDIO_ADC_PORT_E port);


#ifdef __cplusplus
}
#endif

#endif /*__TKL_AUD_ADC_H__ */
