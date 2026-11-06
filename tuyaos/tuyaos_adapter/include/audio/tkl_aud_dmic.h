/**
 * @file tkl_aud_dmic.h
 * @brief Digital microphone (DMIC) capture interface.
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
#define AUDIO_DMIC_INVALID_GAIN_VALUE   0xFFFFFFFF

/* Public typedefs -----------------------------------------------------------*/
/**
 * @brief Configuration for one DMIC capture instance.
 *
 * @note The initial gain is **not** part of this configuration; set it with
 *       @ref tkl_aud_dmic_set_gain after @ref tkl_aud_dmic_init.
 * @note The reference implementation accepts 16-bit samples only.
 * @note On BK7259 (CONFIG_AUD_DRIVER_V2) the digital mic is injected into the
 *       same 3-lane capture path the analog ADC uses, so the two interfaces can
 *       be open at the same time as long as their lane sets do not overlap.
 *       @p chan is a bitmap of @ref TUYA_AUDIO_MIC_CHANNEL_E DMIC bits:
 *       port 0 owns DMIC channel 0 (BIT(8), lane 0) and port 1 owns DMIC
 *       channels 1 and 2 (BIT(9)/BIT(10), lanes 1 and 2). At least one bit of
 *       the port's own channels must be selected.
 */
typedef struct {
    TUYA_AUDIO_MIC_CHANNEL_E chan;
    TUYA_AUDIO_SAMPLE_BITS_E sample_bits;   /**< Sample width in bits; see @ref TUYA_AUDIO_SAMPLE_BITS_E */
    UINT32_T sample_rate;                   /**< Sample rate in Hz */
    UINT32_T frame_time_ms;                 /**< Length of one frame in milliseconds */
    TUYA_AUDIO_INPUT_CB upper_cb;           /**< Frame callback; must not be NULL */
    VOID_T *args;                           /**< Opaque pointer handed back to @p upper_cb */
} TUYA_AUDIO_DMIC_BASE_CFG_T;

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/
/**
 * @brief Initialize the DMIC capture instance for a port.
 *
 * @param[in] port   DMIC port to configure.
 * @param[in] config Capture configuration. Must not be NULL, and
 *                   @p config->upper_cb must not be NULL.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note Must be called before @ref tkl_aud_dmic_start.
 * @note Calling this again on an already initialized port reconfigures it and
 *       returns OPRT_OK without leaking the previous resources. On this platform
 *       a different sample rate / bits / frame time can only be applied while
 *       this port is the sole user of the shared capture path; otherwise
 *       OPRT_COM_ERROR is returned and the running stream is left untouched.
 */
OPERATE_RET tkl_aud_dmic_init(TUYA_AUDIO_DMIC_PORT_E port, CONST TUYA_AUDIO_DMIC_BASE_CFG_T *config);

/**
 * @brief Start capturing.
 *
 * @param[in] port DMIC port.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note The first frame callback arrives roughly @p frame_time_ms after this
 *       call; subsequent frames follow at the same interval.
 * @note Idempotent: calling this while already capturing returns OPRT_OK
 *       without reconfiguring the hardware.
 */
OPERATE_RET tkl_aud_dmic_start(TUYA_AUDIO_DMIC_PORT_E port);

/**
 * @brief Set the DMIC gain for one audio channel.
 *
 * @param[in] port    DMIC port.
 * @param[in] channel Audio channel whose gain to set; one bit of
 *                    @ref TUYA_AUDIO_MIC_CHANNEL_E (DMIC bits BIT(8)..BIT(10)).
 * @param[in] gain    Gain values; the first entry is the digital gain in dB and
 *                    the optional second entry the analog gain in dB.
 * @param[in] num     Number of entries in @p gain (1 or 2).
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note The gain channel axis is the audio channel axis - the same
 *       TUYA_AUDIO_MIC_CHANNEL_E that @p config->chan uses.
 * @note An entry equal to AUDIO_DMIC_INVALID_GAIN_VALUE carries no gain; on this
 *       platform that entry is skipped, so dig and ana can be updated
 *       independently.
 * @note The valid gain range and its mapping to the hardware gain register are
 *       platform defined. The test spec's `0x00`..`0x3F` range described the
 *       removed `tkl_aud_dmic_set_vol()` and does not necessarily apply here.
 * @note Must be called after @ref tkl_aud_dmic_init.
 */
OPERATE_RET tkl_aud_dmic_set_gain(TUYA_AUDIO_DMIC_PORT_E port, TUYA_AUDIO_MIC_CHANNEL_E channel, INT32_T gain[], UINT32_T num);

/**
 * @brief Stop capturing.
 *
 * @param[in] port DMIC port.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note After this returns the frame callback no longer fires.
 * @note Idempotent: calling this while already stopped returns OPRT_OK.
 */
OPERATE_RET tkl_aud_dmic_stop(TUYA_AUDIO_DMIC_PORT_E port);

/**
 * @brief Deinitialize the DMIC capture instance.
 *
 * @param[in] port DMIC port.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 *
 * @note If capture is still running it is stopped first, then the DMA channel,
 *       the ring buffer and the context are released.
 */
OPERATE_RET tkl_aud_dmic_deinit(TUYA_AUDIO_DMIC_PORT_E port);

/**
 * @brief Platform extension: configure DMIC pinmux before init.
 *
 * @param[in] port      DMIC port whose pads are being configured.
 * @param[in] clk_gpio  DMIC clock gpio id.
 * @param[in] data_gpio DMIC data gpio id.
 *
 * @return OPRT_OK on success, error code on failure.
 *
 * @note This function is NOT part of the TKL audio contract; it is a
 *       board-level extension for platforms whose DMIC pads are not fixed.
 *       Call it before @ref tkl_aud_dmic_init for the same port. Each port has
 *       its own pad pair because the two ports use different pad functions; the
 *       two pairs must not share a pin. If not called, the board default pair
 *       is used for that port.
 */
OPERATE_RET tkl_aud_dmic_pinmux_config(TUYA_AUDIO_DMIC_PORT_E port,
                                       UINT32_T clk_gpio, UINT32_T data_gpio);

#ifdef __cplusplus
}
#endif

#endif /*__TKL_AUD_DMIC_H__ */
