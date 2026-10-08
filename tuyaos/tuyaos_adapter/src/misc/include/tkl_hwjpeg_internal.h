/**
 * @file tkl_hwjpeg_internal.h
 * @brief One hardware JPEG decoder, one session at a time.
 *
 * BK7259 has a single JPEG decode core, and the vcdec layer under it routes its
 * one interrupt through a single "active instance" pointer (see
 * modules/vcdec/vcdec_common.h: "dispatches to the active vcdec instance (JPEG
 * or H.264) via the common active-instance pointer"). Two live decode sessions
 * therefore do not queue behind each other - the second one takes the interrupt
 * routing and BOTH stall permanently.
 *
 * Board-verified 2026-10-08: creating a second JPEG decoder while a UVC MJPEG
 * stream runs logs "dec isr codec mismatch: hw=2 active=1" one millisecond
 * later, both sessions then time out, and the streaming one never recovers -
 * not even after the second one is closed and deleted. The display froze and
 * the decode task retried every second for the rest of the log
 * (irq_status=0x00000401).
 *
 * So every hardware JPEG decode in this adapter holds this gate for the whole
 * life of its session. Anything that cannot get it must not touch the
 * hardware: fall back to software decode, or fail.
 *
 * @note This is a software convention inside the adapter. The vendor layers do
 *       not enforce it - hw_decoder_register() happily accepts a second
 *       decoder of the same type (it only serialises its registration list,
 *       not the decoding), which is why the failure is silent and destructive
 *       rather than a clean error.
 */
#ifndef __TKL_HWJPEG_INTERNAL_H__
#define __TKL_HWJPEG_INTERNAL_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Claim the hardware JPEG decoder.
 *
 * @param[in] timeout_ms  how long to wait for it; 0 does not wait at all
 *
 * @return OPRT_OK when the caller now owns the gate; OPRT_TIMEOUT when it is
 *         held by someone else and was not released within @p timeout_ms
 */
OPERATE_RET tkl_hwjpeg_acquire(UINT32_T timeout_ms);

/**
 * @brief Release the gate.
 *
 * @note Only the owner may call this, and exactly once per acquire().
 */
VOID_T tkl_hwjpeg_release(VOID_T);

/**
 * @brief Whether some session currently holds the gate.
 *
 * @return TRUE when held, FALSE otherwise
 */
BOOL_T tkl_hwjpeg_is_busy(VOID_T);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_HWJPEG_INTERNAL_H__ */
