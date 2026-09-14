/**
 * @file tkl_osd_engine.h
 * @brief TKL-internal OSD compositor, shared by every multimedia peripheral.
 *
 * @details
 * Hands caller-owned, pre-composed ARGB8888 bitmaps to the overlay backend:
 *
 *   TKL_OSD_BACKEND_GPU   -> bk_gpu_blit_set()          (display channel)
 *   TKL_OSD_BACKEND_VCENC -> bk_h264_encode_set_osd()   (encode channel)
 *
 * TKL does not compose, allocate or release overlay bitmap data. The context
 * maps each destination coordinate to a backend slot internally.
 *
 * @version 0.1
 * @date 2026-08-27
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 */

#ifndef __TKL_OSD_ENGINE_H__
#define __TKL_OSD_ENGINE_H__

#include "tuya_cloud_types.h"
#include "tkl_media_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque compositor handle; one per peripheral channel. */
typedef VOID_T *TKL_OSD_ENGINE_HANDLE;

/** Overlay hardware behind a channel. */
typedef enum {
    TKL_OSD_BACKEND_GPU = 0, /**< display path: 4 slots, rotate 0/90/270 */
    TKL_OSD_BACKEND_VCENC,   /**< encode path: 8 slots, no rotation, H264 only */
} TKL_OSD_BACKEND_E;

typedef struct {
    UINT16_T target_w;   /**< GPU: panel buffer width. VCENC: encoded frame width  */
    UINT16_T target_h;   /**< GPU: panel buffer height. VCENC: encoded frame height */
    UINT32_T src_format; /**< GPU source format (bk_pixel_format_t); 0 = ARGB8888 */
} TKL_OSD_ENGINE_CFG_T;

/**
 * @brief Bind a compositor to one channel's overlay backend.
 *
 * @param[in]  backend   which overlay hardware
 * @param[in]  bk_handle the backend's own handle: bk_gpu_ctlr_handle_t for GPU,
 *                       bk_h264_encode_ctlr_handle_t for VCENC
 * @param[in]  cfg       target geometry and source format
 * @param[out] handle    receives the compositor handle
 *
 * @note Allocates nothing on the hardware - the backend must already be open.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_osd_engine_open(TKL_OSD_BACKEND_E backend,
                                VOID *bk_handle,
                                CONST TKL_OSD_ENGINE_CFG_T *cfg,
                                TKL_OSD_ENGINE_HANDLE *handle);

/**
 * @brief Clear every slot and release the compositor.
 *
 * @param[in] handle compositor handle; NULL is ignored
 *
 * @note Clearing reclaims sprites submitted but never consumed, so this must
 *       run **before** the stream it decorates is stopped.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_osd_engine_close(TKL_OSD_ENGINE_HANDLE handle);

/**
 * @brief Submit or clear one pre-composed overlay bitmap.
 *
 * @param[in] handle compositor handle
 * @param[in] osd    bitmap request; NULL data clears its destination coordinate
 *
 * @note The destination coordinate identifies an overlay. Re-submitting the
 *       same coordinate updates its existing backend slot.
 *
 * @return OPRT_OK on success, OPRT_NOT_SUPPORTED when no backend slot remains
 *         or the requested rotation is unsupported. Others on error, please
 *         refer to tuya_error_code.h
 */
OPERATE_RET tkl_osd_engine_submit(TKL_OSD_ENGINE_HANDLE handle, CONST TUYA_OSD_T *osd);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_OSD_ENGINE_H__ */
