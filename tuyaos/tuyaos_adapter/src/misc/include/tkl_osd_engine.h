/**
 * @file tkl_osd_engine.h
 * @brief TKL-internal OSD compositor, shared by every multimedia peripheral.
 *
 * @details
 * Turns a TKL_OSD_T (icons + UTF-8 text) into one ARGB8888 sprite and hands it
 * to whichever overlay backend the channel uses. Composition and rasterization
 * are identical for both backends; only the submit call differs:
 *
 *   TKL_OSD_BACKEND_GPU   -> bk_gpu_blit_set()          (display channel)
 *   TKL_OSD_BACKEND_VCENC -> bk_h264_encode_set_osd()   (encode channel)
 *
 * Sprites come from tkl_media_buf (UNCODED); ownership transfers to the backend
 * on a successful submit.
 *
 * TKL-internal: business reaches this only through the per-peripheral
 * tkl_xxx_set_osd() / tkl_xxx_get_osd_caps() pair.
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
    UINT16_T rotate;     /**< GPU only: 0 / 90 / 270, must match the pipeline rotation */
    UINT32_T src_format; /**< GPU only: sprite format (bk_pixel_format_t); 0 = auto */
} TKL_OSD_ENGINE_CFG_T;

/**
 * @brief Bind a compositor to one channel's overlay backend.
 *
 * @param[in]  backend   which overlay hardware
 * @param[in]  bk_handle the backend's own handle: bk_gpu_ctlr_handle_t for GPU,
 *                       bk_h264_encode_ctlr_handle_t for VCENC
 * @param[in]  cfg       target geometry and rotation
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
 * @brief Compose one slot and hand it to the backend.
 *
 * @param[in] handle compositor handle
 * @param[in] osd    slot contents; count == 0 or elems == NULL clears the slot
 *
 * @note Takes effect on the next composed / encoded frame. Re-submitting a slot
 *       replaces its previous sprite, which the backend releases on its own -
 *       a periodic clock overlay does not leak.
 *
 * @return OPRT_OK on success, OPRT_NOT_SUPPORTED when the slot index or a
 *         requested feature exceeds what the backend offers. Others on error,
 *         please refer to tuya_error_code.h
 */
OPERATE_RET tkl_osd_engine_submit(TKL_OSD_ENGINE_HANDLE handle, CONST TKL_OSD_T *osd);

/**
 * @brief Report what this channel's overlay backend can do.
 *
 * @param[in]  handle compositor handle
 * @param[out] caps   receives the capabilities
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_osd_engine_caps(TKL_OSD_ENGINE_HANDLE handle, TKL_OSD_CAPS_T *caps);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_OSD_ENGINE_H__ */
