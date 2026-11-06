/**
 * @file tkl_aud_capture.h
 * @brief Private audio-capture core shared by tkl_aud_adc.c and tkl_aud_dmic.c
 * @version 2.0
 * @date 2026-11-03
 * @copyright Copyright (c) Tuya Inc. All Rights Reserved.
 *
 * On BK7259 there is exactly ONE capture path: ADC lanes 0..2 are interleaved
 * onto mic0_data_bus and drained by a single GDMA channel. The digital mic is
 * not a separate peripheral - it is injected into those very ADC lanes through
 * sys_cfg.digmic_en (MODE_0 -> lane 0, MODE_1 -> lanes 1+2).
 *
 * T5 has two independent FIFOs/DMAs, so its adapters each own a DMA. T9 cannot
 * do that. This module owns the single DMA, ring buffer and ISR, keeps one
 * per-lane gain store, and lets tkl_aud_adc.c / tkl_aud_dmic.c stay as two
 * independent public interfaces on top of it:
 *
 *   - the FIRST interface to open creates the core, later ones only attach;
 *   - the core is destroyed when the LAST interface closes;
 *   - the DMA geometry is fixed (always 3 lanes), so switching between an
 *     analog-only, MODE_0, MODE_1 or MODE_0|MODE_1 configuration rewrites
 *     digmic_en only;
 *   - every started user gets its own frame, containing exactly the lanes its
 *     channel bitmap selected, ascending lane order;
 *   - the lane sets of attached users must not overlap.
 *
 * This header is INTERNAL to tuyaos_adapter/src/misc.
 */

#ifndef __TKL_AUD_CAPTURE_H__
#define __TKL_AUD_CAPTURE_H__

#include "tuya_cloud_types.h"
#include "tkl_aud_adc.h"
#include "tkl_aud_dmic.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TKL_AUD_CAP_USER_ADC = 0,
    /** DMIC port 0: digmic_en MODE_0 (lane 0 digital), pads DMIC0_* */
    TKL_AUD_CAP_USER_DMIC0,
    /** DMIC port 1: digmic_en MODE_1 (lanes 1+2 digital), pads DMIC1_* */
    TKL_AUD_CAP_USER_DMIC1,
    TKL_AUD_CAP_USER_MAX
} TKL_AUD_CAP_USER_E;

/**
 * @brief Whether @p u is one of the two digital-mic users
 * @param[in] u user index
 * @return non-zero for DMIC0 / DMIC1
 */
#define TKL_AUD_CAP_USER_IS_DMIC(u) \
    (((u) == TKL_AUD_CAP_USER_DMIC0) || ((u) == TKL_AUD_CAP_USER_DMIC1))

/**
 * @brief One interface's request handed to the core at open time
 * @note Only the fields relevant to @p user are read.
 */
typedef struct {
    UINT32_T            sample_rate;     /**< must match across attached users */
    UINT8_T             sample_bits;     /**< only 16 is supported */
    UINT32_T            frame_time_ms;   /**< must match across attached users */
    TUYA_AUDIO_INPUT_CB upper_cb;
    VOID_T             *args;
    TUYA_AUDIO_MIC_CHANNEL_E chan;       /**< channel bitmap, see TUYA_AUDIO_MIC_CHANNEL_E */
    /* TKL_AUD_CAP_USER_DMIC0 / _DMIC1 */
    TUYA_AUDIO_DMIC_PORT_E dmic_port;
    UINT32_T            dmic_clk_gpio;
    UINT32_T            dmic_data_gpio;
} TKL_AUD_CAP_REQ_T;

/**
 * @brief Open (or attach to) the shared capture core
 * @param[in] user which public interface is opening
 * @param[in] req  that interface's request
 * @return OPRT_OK on success; OPRT_INVALID_PARM on a parameter/lane conflict;
 *         OPRT_COM_ERROR if the user is already open
 * @note The first caller creates the hardware; later callers attach. A second
 *       caller whose sample_rate / sample_bits / frame_time_ms differ, whose
 *       lanes collide with the ones already configured, or whose DMIC pads are
 *       already taken by the other digital port, is rejected and leaves the
 *       core untouched.
 */
OPERATE_RET __t9_aud_cap_open(TKL_AUD_CAP_USER_E user, const TKL_AUD_CAP_REQ_T *req);

/**
 * @brief Re-open an attached user with a new request (re-init)
 * @param[in] user which public interface is re-initializing
 * @param[in] req  the new request
 * @return OPRT_OK on success
 * @note A request with the same geometry only swaps the callback and args.
 *       A different sample_rate / bits / frame time / channel bitmap tears the
 *       hardware down and rebuilds it, preserving "started", but is only
 *       allowed while @p user is the sole attached user. Otherwise
 *       OPRT_COM_ERROR is returned and nothing is changed.
 * @note If @p user is not attached this is equivalent to __t9_aud_cap_open().
 */
OPERATE_RET __t9_aud_cap_reopen(TKL_AUD_CAP_USER_E user, const TKL_AUD_CAP_REQ_T *req);

/**
 * @brief Close (detach from) the shared capture core
 * @param[in] user which public interface is closing
 * @return OPRT_OK on success
 * @note The last user to close releases the DMA, ring buffer and ADC bring-up.
 */
OPERATE_RET __t9_aud_cap_close(TKL_AUD_CAP_USER_E user);

/**
 * @brief Start capture for one interface
 * @param[in] user which public interface is starting
 * @return OPRT_OK on success
 * @note Reference counted: only the first start touches the hardware, a second
 *       start while another interface is already streaming is a no-op.
 */
OPERATE_RET __t9_aud_cap_start(TKL_AUD_CAP_USER_E user);

/**
 * @brief Stop capture for one interface
 * @param[in] user which public interface is stopping
 * @return OPRT_OK on success
 * @note Only the last stop parks the capture hardware; the shared stream keeps
 *       running while any other interface is still started.
 */
OPERATE_RET __t9_aud_cap_stop(TKL_AUD_CAP_USER_E user);

/**
 * @brief Set dig/ana gain on one ADC lane owned by @p user
 * @param[in] user   which interface is writing
 * @param[in] lane   ADC lane index (aud_adc_chl_t)
 * @param[in] set_dig TRUE to write the digital gain
 * @param[in] dig_db  digital gain in dB
 * @param[in] set_ana TRUE to write the analog gain
 * @param[in] ana_db  analog gain in dB
 * @return OPRT_OK on success; OPRT_INVALID_PARM if the lane is not this
 *         interface's (a DMIC must not move the analog mic's PGA, and vice
 *         versa); OPRT_COM_ERROR if the hardware write failed
 */
OPERATE_RET __t9_aud_cap_set_gain(TKL_AUD_CAP_USER_E user, UINT32_T lane,
                                 BOOL_T set_dig, FLOAT_T dig_db,
                                 BOOL_T set_ana, INT32_T ana_db);

/**
 * @brief Whether @p user is currently open on the core
 * @param[in] user interface to query
 * @return TRUE if attached
 */
BOOL_T __t9_aud_cap_user_attached(TKL_AUD_CAP_USER_E user);

#ifdef __cplusplus
}
#endif

#endif /*__TKL_AUD_CAPTURE_H__ */
