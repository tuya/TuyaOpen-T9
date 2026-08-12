/**
 * @file tkl_gpu.h
 * @brief Common process - adapter the GPU pipeline api
 * @version 0.2
 * @date 2026-04-13
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 */

#ifndef __TKL_GPU_H__
#define __TKL_GPU_H__

#include "tuya_cloud_types.h"
#include <components/bk_draw_osd_types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef VOID *(*TUYA_GPU_ALLOC_CB)(UINT32_T size, VOID *arg);
typedef OPERATE_RET (*TUYA_GPU_FREE_CB)(VOID *ptr, VOID *arg);
typedef VOID (*TUYA_GPU_FRAME_DONE_CB)(VOID *frame, UINT32_T frame_size, VOID *arg);

typedef struct {
    TUYA_GPU_ALLOC_CB alloc_cb;
    TUYA_GPU_FREE_CB free_cb;
    TUYA_GPU_FRAME_DONE_CB frame_done_cb;
    VOID *cb_arg;
} TUYA_GPU_SINK_T;

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
    /* Legacy sink callbacks kept for compatibility; prefer tkl_gpu_set_sink(). */
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

/* ---------------------------------------------------------------------------
 * Display GPU OSD (bk_draw_osd pipeline submit model)
 * Encode-path OSD remains tkl_venc_set_osd.
 * --------------------------------------------------------------------------- */

typedef enum {
    TUYA_GPU_OSD_OP_UPDATE = 0, /* add/update one element by name+content, then draw */
    TUYA_GPU_OSD_OP_REMOVE,       /* remove one element by name, then redraw remainder */
    TUYA_GPU_OSD_OP_CLEAR,        /* clear all GPU blits; keep instance and dynamic list */
} TUYA_GPU_OSD_OP_E;

typedef struct {
    CONST blend_info_t *blend_assets; /* asset table for UPDATE; {.addr=NULL} ends; required for add */
    CONST blend_info_t *blend_info;   /* optional preload list (not drawn at open); NULL or {.addr=NULL} ok */
    UINT32_T src_format;              /* 0=auto (ABGR if compress else ARGB) */
    BOOL_T by_flexa;                  /* false=frame-end, true=per-flexa block */
    /* OSD 内容旋转角度，仅支持 0 / 90 / 270（GPU 引擎不支持 180），默认为 0。*/
    UINT16_T rotate_degree;
} TUYA_GPU_OSD_OPEN_T;

OPERATE_RET tkl_gpu_init(TUYA_GPU_CFG_T *config);
OPERATE_RET tkl_gpu_deinit(VOID);
OPERATE_RET tkl_gpu_set_sink(const TUYA_GPU_SINK_T *sink);
OPERATE_RET tkl_gpu_set_source(TUYA_GPU_SOURCE_T *source);
OPERATE_RET tkl_gpu_start(VOID);
OPERATE_RET tkl_gpu_stop(VOID);
OPERATE_RET tkl_gpu_get_handle(VOID **handle);

/**
 * @brief Create bk_draw_osd instance on started tkl_gpu (does not draw)
 * @param[in] cfg assets + optional preload list + blend mode + OSD 旋转角度
 * @return OPRT_OK on success; OPRT_INVALID_PARM if rotate_degree is not 0/90/270
 * @note Requires tkl_gpu_start(). No blit until tkl_gpu_osd_ctrl(UPDATE, ...).
 *       blend_assets/blend_info pointers are not deep-copied.
 */
OPERATE_RET tkl_gpu_osd_open(CONST TUYA_GPU_OSD_OPEN_T *cfg);

/**
 * @brief Runtime OSD list / blit control
 * @param[in] op UPDATE / REMOVE / CLEAR
 * @param[in] name element name (UPDATE/REMOVE required; CLEAR ignored)
 * @param[in] content text or image variant key (UPDATE required; others ignored)
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_gpu_osd_ctrl(TUYA_GPU_OSD_OP_E op, CONST CHAR_T *name, CONST CHAR_T *content);

/**
 * @brief Destroy OSD instance and clear blits
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_gpu_osd_close(VOID);

/**
 * @brief Set OSD blend timing on bound GPU
 * @param[in] by_flexa TRUE=per-flexa, FALSE=frame-end
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_gpu_osd_set_mode(BOOL_T by_flexa);

#ifdef __cplusplus
}
#endif

#endif
