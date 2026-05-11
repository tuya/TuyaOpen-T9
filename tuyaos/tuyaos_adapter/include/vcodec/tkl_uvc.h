/**
 * @file tkl_uvc.h
 * @brief Common process - adapter the USB video camera api
 * @version 0.1
 * @date 2026-05-09
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_UVC_H__
#define __TKL_UVC_H__

#include "tuya_cloud_types.h"
#include "tkl_vcodec_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef VOID *TUYA_UVC_HANDLE;

typedef VOID (*TUYA_UVC_STREAM_CB)(const TUYA_VCODEC_STREAM_T *stream, VOID *arg);
typedef VOID (*TUYA_UVC_STATE_CB)(BOOL_T connected, VOID *arg);

typedef struct {
    UINT8_T port;                   /* USB hub port, default 1 */
    UINT16_T width;
    UINT16_T height;
    UINT8_T fps;                    /* default 30 */
    UINT8_T drop_num;
    UINT32_T codec;                 /* TUYA_VCODEC_TYPE_MJPEG/H264, default MJPEG */
    UINT32_T connect_timeout_ms;    /* default 4000 */
} TUYA_UVC_CFG_T;

typedef struct {
    TUYA_UVC_STREAM_CB stream_cb;
    TUYA_UVC_STATE_CB state_cb;
    TUYA_VCODEC_ALLOC_CB alloc_cb;
    TUYA_VCODEC_FREE_CB free_cb;
    VOID *cb_arg;
} TUYA_UVC_SINK_T;

OPERATE_RET tkl_uvc_init(TUYA_UVC_HANDLE *handle, const TUYA_UVC_CFG_T *config);
OPERATE_RET tkl_uvc_deinit(TUYA_UVC_HANDLE handle);
OPERATE_RET tkl_uvc_set_sink(TUYA_UVC_HANDLE handle, const TUYA_UVC_SINK_T *sink);
OPERATE_RET tkl_uvc_start(TUYA_UVC_HANDLE handle);
OPERATE_RET tkl_uvc_stop(TUYA_UVC_HANDLE handle);
OPERATE_RET tkl_uvc_get_handle(TUYA_UVC_HANDLE handle, VOID **backend_handle);

#ifdef __cplusplus
}
#endif

#endif
