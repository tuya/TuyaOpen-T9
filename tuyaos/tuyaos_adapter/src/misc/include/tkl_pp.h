/**
 * @file tkl_pp.h
 * @brief TKL-internal frame-mode image processor (scale + format convert).
 *
 * @details
 * The only memory-in / memory-out image engine on this part. `bk_pp_process()`
 * blocks until the hardware is done, which suits a TKL that owns no threads.
 *
 * **Capability limits - these are the hardware's, not this wrapper's:**
 *   - input pixel format: **NV12 only**
 *   - output pixel format: NV12 / RGB565 / RGB888
 *   - scaling: yes
 *   - **rotation / mirroring / compressed output: none**
 *
 * The GPU can rotate, but only in flexa mode, so callers needing rotation have
 * no frame-mode path today.
 *
 * TKL-internal, never exported to business.
 *
 * @version 0.1
 * @date 2026-08-27
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 */

#ifndef __TKL_PP_H__
#define __TKL_PP_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque processor handle; one handle processes one request at a time. */
typedef VOID_T *TKL_PP_HANDLE;

/** Source picture. Always NV12. */
typedef struct {
    UINT8_T *y;         /**< luminance plane */
    UINT8_T *uv;        /**< interleaved CbCr plane */
    UINT16_T width;     /**< source width in pixels */
    UINT16_T height;    /**< source height in pixels */
} TKL_PP_IN_T;

/** Destination picture. */
typedef struct {
    UINT8_T *buffer;    /**< destination, must be hardware-reachable (tkl_media_buf) */
    UINT32_T size;      /**< destination capacity in bytes, see tkl_pp_output_size() */
    UINT16_T width;     /**< 0 = same as source */
    UINT16_T height;    /**< 0 = same as source */
    UINT32_T format;    /**< bk_pixel_format_t: NV12 / RGB565 / RGB888 */
} TKL_PP_OUT_T;

/**
 * @brief Create and bring up a processor.
 *
 * @param[in]  timeout_ms hardware wait timeout; 0 selects the vendor default (1000)
 * @param[out] handle     receives the new handle
 *
 * @note Bring-up registers with the shared decoder/PP power domain, so prefer
 *       keeping one handle open over opening per frame.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_pp_open(UINT32_T timeout_ms, TKL_PP_HANDLE *handle);

/**
 * @brief Tear down a processor and release the handle.
 *
 * @param[in] handle processor handle; NULL is ignored
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_pp_close(TKL_PP_HANDLE handle);

/**
 * @brief Run one scale and/or format conversion. Blocks until complete.
 *
 * @param[in] handle processor handle
 * @param[in] in     NV12 source
 * @param[in] out    destination; @p out->buffer must hold at least
 *                   tkl_pp_output_size(out->format, w, h) bytes
 *
 * @note Serialised per handle by an internal mutex, so several threads may share
 *       one handle. Blocks the calling thread - never call from an ISR, from the
 *       GPU flexa thread, or from the UVC stream task.
 *
 * @return OPRT_OK on success, OPRT_NOT_SUPPORTED for a format the hardware
 *         cannot do. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_pp_process(TKL_PP_HANDLE handle, CONST TKL_PP_IN_T *in, CONST TKL_PP_OUT_T *out);

/**
 * @brief Bytes the hardware needs for one output picture.
 *
 * @param[in] format bk_pixel_format_t: NV12 / RGB565 / RGB888
 * @param[in] width  output width in pixels
 * @param[in] height output height in pixels
 *
 * @note Not width*height*bpp: the hardware rounds the stored height up to 2
 *       rows for NV12 and 16 for RGB, and its RGB888 is 4 bytes per pixel.
 *
 * @return required byte count, 0 for an unsupported format or zero geometry
 */
UINT32_T tkl_pp_output_size(UINT32_T format, UINT16_T width, UINT16_T height);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_PP_H__ */
