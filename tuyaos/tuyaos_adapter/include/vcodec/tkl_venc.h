/**
 * @file tkl_venc.h
 * @brief Common process - adapter the video encoder api
 * @version 0.1
 * @date 2026-04-15
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_VENC_H__
#define __TKL_VENC_H__

#include "tuya_cloud_types.h"
#include "tkl_vcodec_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef VOID *TUYA_VENC_HANDLE;

typedef VOID (*TUYA_VENC_STREAM_CB)(const TUYA_VCODEC_STREAM_T *stream, VOID *arg);

typedef enum {
    TUYA_VENC_SOURCE_NONE = 0,
    TUYA_VENC_SOURCE_USER_FRAME,
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
    UINT8_T quality;                /* JPEG encode quality 0..10 (lower=higher quality), default 2; ignored for H264 */
} TUYA_VENC_CFG_T;

typedef struct {
    UINT32_T bitrate;               /* target bitrate in bps; 0 selects fixed-QP mode */
    UINT8_T qp_min_i;               /* 0 uses adapter default when bitrate > 0 */
    UINT8_T qp_max_i;
    UINT8_T qp_min_p;
    UINT8_T qp_max_p;
    BOOL_T force_idr;               /* request IDR after rate ctrl update */
} TUYA_VENC_RATE_CTRL_T;

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
OPERATE_RET tkl_venc_submit_frame(TUYA_VENC_HANDLE handle, const TUYA_VCODEC_FRAME_T *frame);
OPERATE_RET tkl_venc_get_handle(TUYA_VENC_HANDLE handle, VOID **backend_handle);

#ifdef __cplusplus
}
#endif

#endif
