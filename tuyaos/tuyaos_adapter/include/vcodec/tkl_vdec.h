/**
 * @file tkl_vdec.h
 * @brief Common process - adapter the video decoder api
 * @version 0.1
 * @date 2026-04-15
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_VDEC_H__
#define __TKL_VDEC_H__

#include "tuya_cloud_types.h"
#include "tkl_vcodec_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef VOID *TUYA_VDEC_HANDLE;

typedef VOID (*TUYA_VDEC_FRAME_CB)(const TUYA_VCODEC_FRAME_T *frame, VOID *arg);
typedef VOID (*TUYA_VDEC_FLEXA_CB)(UINT32_T wr_cnt, VOID *arg);

typedef enum {
    TUYA_VDEC_SINK_NONE = 0,
    TUYA_VDEC_SINK_FRAME,
    TUYA_VDEC_SINK_FLEXA,
} TUYA_VDEC_SINK_TYPE_E;

typedef struct {
    UINT16_T out_width;
    UINT16_T out_height;
    UINT32_T output_format;         /* bk_pixel_format_t */
    UINT16_T rotate_degree;         /* 0/90/180/270 */
} TUYA_VDEC_PP_T;

typedef struct {
    UINT32_T codec;                 /* TUYA_VCODEC_TYPE_E */
    UINT16_T width;
    UINT16_T height;
    UINT32_T output_format;         /* bk_pixel_format_t */
    BOOL_T low_latency;
    UINT8_T flexa_lines;
} TUYA_VDEC_CFG_T;

typedef struct {
    TUYA_VDEC_SINK_TYPE_E type;
    TUYA_VCODEC_ALLOC_CB alloc_cb;
    TUYA_VCODEC_FREE_CB free_cb;
    VOID *cb_arg;
    union {
        struct {
            TUYA_VDEC_FRAME_CB frame_cb;
        } frame;

        struct {
            TUYA_VCODEC_FLEXA_BUFFER_T flexa;
            TUYA_VDEC_FLEXA_CB flexa_cb;
        } flexa;
    };
} TUYA_VDEC_SINK_T;

OPERATE_RET tkl_vdec_init(TUYA_VDEC_HANDLE *handle, const TUYA_VDEC_CFG_T *config);
OPERATE_RET tkl_vdec_deinit(TUYA_VDEC_HANDLE handle);
OPERATE_RET tkl_vdec_set_sink(TUYA_VDEC_HANDLE handle, const TUYA_VDEC_SINK_T *sink);
OPERATE_RET tkl_vdec_set_pp(TUYA_VDEC_HANDLE handle, const TUYA_VDEC_PP_T *pp);
OPERATE_RET tkl_vdec_start(TUYA_VDEC_HANDLE handle);
OPERATE_RET tkl_vdec_stop(TUYA_VDEC_HANDLE handle);
OPERATE_RET tkl_vdec_reset(TUYA_VDEC_HANDLE handle);
OPERATE_RET tkl_vdec_submit_stream(TUYA_VDEC_HANDLE handle, const TUYA_VCODEC_STREAM_T *stream);
OPERATE_RET tkl_vdec_get_handle(TUYA_VDEC_HANDLE handle, VOID **backend_handle);
OPERATE_RET tkl_vdec_get_flexa_source(TUYA_VDEC_HANDLE handle, TUYA_VCODEC_FLEXA_BUFFER_T *flexa);

#ifdef __cplusplus
}
#endif

#endif
