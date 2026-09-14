/**
 * @file tkl_venc_internal.h
 * @brief Common process - adapter the video encoder api
 * @version 0.1
 * @date 2026-04-15
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_VENC_INTERNAL_H__
#define __TKL_VENC_INTERNAL_H__

#include "tuya_cloud_types.h"
#include "tkl_media_types.h"
#include "tkl_vcodec_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef VOID *TUYA_VENC_HANDLE;

typedef VOID (*TUYA_VENC_STREAM_CB)(const TUYA_VCODEC_STREAM_T *stream, VOID *arg);

typedef enum {
    TUYA_VENC_SOURCE_NONE = 0,
    TUYA_VENC_SOURCE_ISP_FLEXA,
    TUYA_VENC_SOURCE_VDEC_FLEXA,
} TUYA_VENC_SOURCE_TYPE_E;

typedef struct {
    UINT16_T width;
    UINT16_T height;
    UINT8_T fps;
    UINT32_T gop;
    UINT32_T codec;                 /* TUYA_VCODEC_TYPE_E, default H264 */
    UINT32_T input_format;          /* bk_pixel_format_t, default NV12 */
    /* JPEG quality as the raw vcenc qLevel, 0..10, HIGHER is better - it indexes
     * eleven built-in quantisation tables running coarse (0) to all-ones (10).
     * qLevel N is the same table libjpeg produces at quality N*10, so 10 means
     * quantisation off. Default 2, i.e. libjpeg quality ~30. Ignored for H264.
     * Note this is NOT the 1..100 scale TUYA_IMAGE_CODEC_ENC_PARAM_T uses. */
    UINT8_T quality;
} TUYA_VENC_CFG_T;

typedef struct {
    UINT32_T bitrate;               /* target bitrate in bps; 0 selects fixed-QP mode */
    UINT8_T qp_min_i;               /* 0 uses adapter default when bitrate > 0 */
    UINT8_T qp_max_i;
    UINT8_T qp_min_p;
    UINT8_T qp_max_p;
    BOOL_T force_idr;               /* request IDR after rate ctrl update */
} TUYA_VENC_RATE_CTRL_T;

/**
 * @brief Where encoded/decoded output goes, and optionally who owns its memory.
 *
 * @note alloc_cb/free_cb are **optional and all-or-nothing**. Both NULL is the
 *       normal case: TKL picks the pool, and the buffer must not be freed or
 *       kept past the callback. Install both when the buffer has to outlive the
 *       callback or come from a pool of yours. Setting just one is rejected.
 */
typedef struct {
    TUYA_VENC_STREAM_CB stream_cb;
    TUYA_VCODEC_ALLOC_CB alloc_cb;
    TUYA_VCODEC_FREE_CB free_cb;
    VOID *cb_arg;
} TUYA_VENC_SINK_T;

typedef struct {
    TUYA_VENC_SOURCE_TYPE_E type;
    union {
        struct {
            VOID *isp_handle;
            TUYA_VCODEC_FLEXA_BUFFER_T flexa;
        } isp;

        struct {
            VOID *vdec_handle;
            TUYA_VCODEC_FLEXA_BUFFER_T flexa;
        } vdec;
    };
} TUYA_VENC_SOURCE_T;

OPERATE_RET tkl_venc_init(TUYA_VENC_HANDLE *handle, const TUYA_VENC_CFG_T *config);
OPERATE_RET tkl_venc_deinit(TUYA_VENC_HANDLE handle);
OPERATE_RET tkl_venc_set_source(TUYA_VENC_HANDLE handle, const TUYA_VENC_SOURCE_T *source);
OPERATE_RET tkl_venc_set_sink(TUYA_VENC_HANDLE handle, const TUYA_VENC_SINK_T *sink);
OPERATE_RET tkl_venc_start(TUYA_VENC_HANDLE handle);
OPERATE_RET tkl_venc_stop(TUYA_VENC_HANDLE handle);
OPERATE_RET tkl_venc_set_rate_ctrl(TUYA_VENC_HANDLE handle, const TUYA_VENC_RATE_CTRL_T *rate_ctrl);
OPERATE_RET tkl_venc_request_idr(TUYA_VENC_HANDLE handle);
/**
 * @brief Set or clear one encode-path pre-composed ARGB8888 overlay.
 * @param[in] handle venc handle
 * @param[in] osd bitmap in un-rotated encoded-frame coordinates
 * @return OPRT_OK on success, OPRT_NOT_SUPPORTED for JPEG/non-H264, no slot,
 *         or non-zero rotation
 * @note The caller owns the bitmap data. Same coordinates update one of the
 *       eight backend slots; NULL data clears that coordinate.
 */
OPERATE_RET tkl_venc_set_osd(TUYA_VENC_HANDLE handle, CONST TUYA_OSD_T *osd);
OPERATE_RET tkl_venc_get_handle(TUYA_VENC_HANDLE handle, VOID **backend_handle);

/* JPEG still-image encode and the NV12->YUYV helper moved to tkl_image.h:
 * they are one-shot, host-less operations with no encoder state, which is the
 * whole point of that module. */

#ifdef __cplusplus
}
#endif

#endif
