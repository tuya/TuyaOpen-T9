/**
 * @file tkl_frame_buffer.h
 * @brief Common process - adapter the media frame buffer allocator
 * @version 0.1
 * @date 2026-08-13
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_FRAME_BUFFER_H__
#define __TKL_FRAME_BUFFER_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Media frame buffer kind.
 *
 * Hardware media blocks (ISP / GPU / VENC / VDEC) cannot take memory from the
 * ordinary heap: raw frames are DMA targets that must live in non-cacheable
 * memory, and encoded bitstreams come from a separate pool. The sink callbacks
 * of tkl_gpu_set_sink() / tkl_venc_set_sink() are supplied by the caller, so the
 * caller needs a way to allocate that memory - this is it.
 */
typedef enum {
    TKL_FRAME_BUFFER_CODED = 0, /**< encoded bitstream, e.g. tkl_venc_set_sink output */
    TKL_FRAME_BUFFER_UNCODED,   /**< raw frame (non-cacheable), e.g. tkl_gpu_set_sink output */
    TKL_FRAME_BUFFER_TYPE_MAX,
} TKL_FRAME_BUFFER_TYPE_E;

/**
 * @brief Bring up the media frame buffer pools.
 *
 * @note Idempotent. Call this before starting any media hardware: the ISP, the
 *       H264 encoder and the GPU allocate from these pools internally, without
 *       going through the caller-supplied sink callbacks, so waiting for the
 *       first tkl_frame_buffer_malloc() is too late and those internal
 *       allocations fail with "not init".
 */
VOID_T tkl_frame_buffer_init(VOID_T);

/**
 * @brief Allocate a media frame buffer.
 *
 * @param[in] type buffer kind, see TKL_FRAME_BUFFER_TYPE_E
 * @param[in] size byte count to allocate
 *
 * @note The backing pool is brought up on first use, so no explicit init call
 *       is required. Release with tkl_frame_buffer_free().
 *
 * @return buffer pointer on success, NULL on error
 */
VOID_T *tkl_frame_buffer_malloc(TKL_FRAME_BUFFER_TYPE_E type, UINT32_T size);

/**
 * @brief Release a buffer obtained from tkl_frame_buffer_malloc().
 *
 * @param[in] frame buffer pointer; NULL is ignored
 */
VOID_T tkl_frame_buffer_free(VOID_T *frame);

/**
 * @brief Mark a frame buffer as write-through.
 *
 * @param[in] frame buffer obtained from tkl_frame_buffer_malloc()
 *
 * @note Useful when the CPU writes into a buffer that hardware reads back
 *       (for example an OSD overlay), so stores reach memory without an
 *       explicit cache clean. Raw frames written only by hardware do not
 *       need this.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_frame_buffer_set_write_through(VOID_T *frame);

#ifdef __cplusplus
}
#endif

#endif
