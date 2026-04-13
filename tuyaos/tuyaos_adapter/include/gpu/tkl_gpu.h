/**
 * @file tkl_gpu.h
 * @brief Common process - adapter the GPU pipeline api
 * @version 0.1
 * @date 2026-04-13
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_GPU_H__
#define __TKL_GPU_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef VOID *(*TUYA_GPU_ALLOC_CB)(UINT32_T size, VOID *arg);
typedef OPERATE_RET (*TUYA_GPU_FREE_CB)(VOID *ptr, VOID *arg);
typedef VOID (*TUYA_GPU_FRAME_DONE_CB)(VOID *frame, UINT32_T frame_size, VOID *arg);

typedef struct {
    UINT8_T *src_buffer;
    UINT8_T buf_cnt;
    UINT16_T width;
    UINT16_T height;
    UINT32_T format;                /* bk_pixel_format_t */
} TUYA_GPU_FLEXA_BUFFER_T;

typedef enum {
    TUYA_GPU_SOURCE_NONE = 0,
    TUYA_GPU_SOURCE_ISP_FLEXA,
    TUYA_GPU_SOURCE_MJPEG_FLEXA,
} TUYA_GPU_SOURCE_TYPE_E;

typedef struct {
    UINT16_T dst_width;
    UINT16_T dst_height;
    UINT16_T rotate_degree;         /* 0/90/180/270 */
    UINT32_T dst_format;            /* bk_pixel_format_t, default ARGB8888 */
    BOOL_T scale;
    BOOL_T dst_compress;
    UINT8_T flexa_lines;            /* default 16 */
    TUYA_GPU_ALLOC_CB alloc_cb;
    TUYA_GPU_FREE_CB free_cb;
    TUYA_GPU_FRAME_DONE_CB frame_done_cb;
    VOID *cb_arg;
} TUYA_GPU_CFG_T;

typedef struct {
    TUYA_GPU_SOURCE_TYPE_E type;
    union {
        struct {
            VOID *isp_handle;
            TUYA_GPU_FLEXA_BUFFER_T flexa;
        } isp;

        struct {
            VOID *jpeg_handle;
            TUYA_GPU_FLEXA_BUFFER_T flexa;
        } mjpeg;
    };
} TUYA_GPU_SOURCE_T;

OPERATE_RET tkl_gpu_init(TUYA_GPU_CFG_T *config);
OPERATE_RET tkl_gpu_deinit(VOID);
OPERATE_RET tkl_gpu_set_source(TUYA_GPU_SOURCE_T *source);
OPERATE_RET tkl_gpu_start(VOID);
OPERATE_RET tkl_gpu_stop(VOID);
OPERATE_RET tkl_gpu_get_handle(VOID **handle);

#ifdef __cplusplus
}
#endif

#endif
