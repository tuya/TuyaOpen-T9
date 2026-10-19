/**
 * @file tkl_codec_gate_internal.h
 * @brief One hardware codec session at a time, per side.
 *
 * @details
 * BK7259 gives each codec direction exactly one interrupt and one "active
 * instance" pointer, and both directions host two codecs behind it:
 *
 *   - decode (vcdec): one ISR, one active-instance pointer, JPEG and H.264
 *     instances behind it. See modules/vcdec/vcdec_common.h.
 *   - encode (vcenc): one ISR (INT_SRC_H26E), one active vcenc_common_t
 *     pointer plus a HWIF_ENC_MODE sanity check, H.264 and JPEG instances
 *     behind it. See components/bk_vpu/bk_encoder/common/hw_encoder_ctlr.c.
 *
 * Two live sessions on the same side therefore do not queue behind each
 * other - the second one takes the interrupt routing and BOTH stall
 * permanently. Board-verified on the decode side, 2026-10-08: creating a
 * second JPEG decoder while a UVC MJPEG stream runs logs "dec isr codec
 * mismatch: hw=2 active=1" one millisecond later, both sessions then time
 * out, and the streaming one never recovers - not even after the second one
 * is closed and deleted. The display froze and the decode task retried every
 * second for the rest of the log (irq_status=0x00000401).
 *
 * The encode side is the same structure but has NOT been reproduced on a
 * board - it is recorded here as a hazard, not as a verified failure. No
 * adapter path takes TKL_CODEC_GATE_ENC yet.
 *
 * So every hardware codec session in this adapter holds its side's gate for
 * the whole life of that session. Anything that cannot get it must not touch
 * the hardware: fall back to a software path, or fail.
 *
 * @note This is a software convention inside the adapter. The vendor layers do
 *       not enforce it - hw_decoder_register() happily accepts a second
 *       decoder of the same type (it only serialises its registration list,
 *       not the decoding), which is why the failure is silent and destructive
 *       rather than a clean error.
 *
 * @note The two sides are separate gates on purpose: decode and encode are
 *       different hardware blocks, so one gate for both would serialise
 *       unrelated work.
 */
#ifndef __TKL_CODEC_GATE_INTERNAL_H__
#define __TKL_CODEC_GATE_INTERNAL_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TKL_CODEC_GATE_DEC = 0,   /**< vcdec: the JPEG / H.264 decode core */
    TKL_CODEC_GATE_ENC,       /**< vcenc: the H.264 / JPEG encode core */
    TKL_CODEC_GATE_MAX,
} TKL_CODEC_GATE_E;

/**
 * @brief Claim one codec core.
 *
 * @param[in] which       which side's core
 * @param[in] timeout_ms  how long to wait for it; 0 does not wait at all
 *
 * @return OPRT_OK when the caller now owns the gate; OPRT_TIMEOUT when it is
 *         held by someone else and was not released within @p timeout_ms;
 *         OPRT_INVALID_PARM for a @p which outside the enum
 */
OPERATE_RET tkl_codec_gate_acquire(TKL_CODEC_GATE_E which, UINT32_T timeout_ms);

/**
 * @brief Release one codec core.
 *
 * @note Only the owner may call this, and exactly once per acquire().
 *       A @p which outside the enum is ignored.
 */
VOID_T tkl_codec_gate_release(TKL_CODEC_GATE_E which);

/**
 * @brief Whether some session currently holds a codec core.
 *
 * @return TRUE when held, FALSE otherwise (including an out-of-range @p which)
 */
BOOL_T tkl_codec_gate_is_busy(TKL_CODEC_GATE_E which);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_CODEC_GATE_INTERNAL_H__ */
