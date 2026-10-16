/**
 * @file tkl_vcodec_types.h
 * @brief Common process - adapter the video codec common types
 * @version 0.1
 * @date 2026-04-15
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_VCODEC_TYPES_H__
#define __TKL_VCODEC_TYPES_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TUYA_VCODEC_TYPE_NONE = 0,
    TUYA_VCODEC_TYPE_H264,
    TUYA_VCODEC_TYPE_JPEG,
    TUYA_VCODEC_TYPE_MJPEG,
} TUYA_VCODEC_TYPE_E;

typedef enum {
    TUYA_VCODEC_FRAME_UNKNOWN = 0,
    TUYA_VCODEC_FRAME_I,
    TUYA_VCODEC_FRAME_P,
    TUYA_VCODEC_FRAME_B,
    TUYA_VCODEC_FRAME_HEADER,
    TUYA_VCODEC_FRAME_END,
} TUYA_VCODEC_FRAME_TYPE_E;

typedef enum {
    TUYA_VCODEC_ROTATE_0 = 0,
    TUYA_VCODEC_ROTATE_90 = 90,
    TUYA_VCODEC_ROTATE_180 = 180,
    TUYA_VCODEC_ROTATE_270 = 270,
} TUYA_VCODEC_ROTATE_E;

typedef VOID *(*TUYA_VCODEC_ALLOC_CB)(UINT32_T size, VOID *arg);
typedef OPERATE_RET (*TUYA_VCODEC_FREE_CB)(VOID *ptr, VOID *arg);

typedef struct {
    UINT8_T *data;
    UINT32_T size;
    UINT64_T pts;
    UINT32_T codec;                 /* TUYA_VCODEC_TYPE_E */
    UINT32_T frame_type;            /* TUYA_VCODEC_FRAME_TYPE_E */
    VOID *priv;
} TUYA_VCODEC_STREAM_T;

typedef struct {
    UINT8_T *buffer;
    UINT32_T size;
    UINT16_T width;
    UINT16_T height;
    UINT16_T stride;
    UINT16_T slice_height;
    UINT32_T format;                /* bk_pixel_format_t */
    UINT32_T y_addr;
    UINT32_T u_addr;
    UINT32_T v_addr;
    UINT64_T pts;
    VOID *priv;
} TUYA_VCODEC_FRAME_T;

typedef struct {
    VOID *owner_handle;
    UINT8_T *src_buffer;
    UINT8_T buf_cnt;
    UINT16_T width;
    UINT16_T height;
    UINT32_T format;                /* bk_pixel_format_t */
    UINT32_T y_addr;
    UINT32_T u_addr;
    UINT32_T v_addr;
} TUYA_VCODEC_FLEXA_BUFFER_T;

#ifdef __cplusplus
}
#endif

#endif
