/**
 * @file tkl_audio_frontend.h
 * @brief T9 audio front-end: 3-lane capture -> BF -> AEC/NS -> 20 ms mono
 * @version 1.0
 * @date 2026-11-06
 * @copyright Copyright (c) 2025-2026 Tuya Inc. All Rights Reserved.
 *
 * BK7259 captures ADC lanes 0..2 on one FIFO/DMA, 16 kHz / 16 bit / 8 ms.
 * This module turns that 3-lane frame into the 20 ms mono stream the TDD/TDL
 * and ai_audio layers expect, optionally running the tal_audio_subsys
 * DAS BF + FDAF AEC+NS pipeline on the way. The AEC input is 2 mic + 1 ref:
 * both mics are delay-and-sum steered into a mono beam by the BF pre-stage,
 * and the capture reference is delayed by the BF latency so AEC stays aligned.
 *
 * Lane roles and beamformer geometry are compile-time constants (see the
 * TKL_AF_* defaults); they are board wiring/geometry, not something the
 * algorithm can infer.
 */

#ifndef __TKL_AUDIO_FRONTEND_H__
#define __TKL_AUDIO_FRONTEND_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Capture side: one frame of three interleaved ADC lanes. */
#define TKL_AF_SAMPLE_RATE      (16000U)
#define TKL_AF_CAPTURE_FRAME_MS (8U)
#define TKL_AF_CAPTURE_LANES    (3U)
#define TKL_AF_CAPTURE_SAMPLES  ((TKL_AF_SAMPLE_RATE / 1000U) * TKL_AF_CAPTURE_FRAME_MS) /* 128 */
#define TKL_AF_CAPTURE_BYTES    (TKL_AF_CAPTURE_SAMPLES * TKL_AF_CAPTURE_LANES * 2U)     /* 768 */

/* Output side: mono 20 ms, matching tdd_audio's sample_tm_ms. */
#define TKL_AF_OUT_FRAME_MS (20U)
#define TKL_AF_OUT_SAMPLES  ((TKL_AF_SAMPLE_RATE / 1000U) * TKL_AF_OUT_FRAME_MS) /* 320 */
#define TKL_AF_OUT_BYTES    (TKL_AF_OUT_SAMPLES * 2U)                            /* 640 */

/**
 * @brief Create the AEC/NS stage (no-op without TKL_AF_WITH_TAL_LIB)
 * @return OPRT_OK on success; OPRT_COM_ERROR if the algorithm instance failed
 * @note Falls back to passthrough when the algorithm library cannot allocate.
 */
OPERATE_RET tkl_af_init(void);

/**
 * @brief Destroy the AEC/NS stage and drop buffered samples
 * @return OPRT_OK
 */
OPERATE_RET tkl_af_deinit(void);

/**
 * @brief Whether the AEC/NS stage is actually running
 * @return TRUE when AEC+NS processes the stream, FALSE for passthrough
 */
BOOL_T tkl_af_aec_enabled(void);

/**
 * @brief Process one 3-lane 8 ms capture frame
 * @param[in]  in      768 bytes, lanes interleaved per sample, native endian
 * @param[in]  inlen   must be at least TKL_AF_CAPTURE_BYTES
 * @param[out] out     buffer for one 20 ms mono block, at least TKL_AF_OUT_BYTES
 * @param[in]  outlen  must be at least TKL_AF_OUT_BYTES
 * @return TKL_AF_OUT_BYTES when a full block is ready, 0 when more capture
 *         frames are needed, -1 on a bad argument
 * @note Every returned block is fed to the VAD/KWS stages before it returns.
 *       Must be called from one task only, in capture order.
 */
int tkl_af_process(const uint8_t *in, uint32_t inlen, uint8_t *out, uint32_t outlen);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_AUDIO_FRONTEND_H__ */
