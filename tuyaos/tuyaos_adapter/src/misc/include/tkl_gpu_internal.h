/**
 * @file tkl_gpu_internal.h
 * @brief Common process - adapter the GPU pipeline api
 * @version 0.3
 * @date 2026-08-27
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 */

#ifndef __TKL_GPU_INTERNAL_H__
#define __TKL_GPU_INTERNAL_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque per-pipeline handle.
 *
 * @note One handle == one bk_gpu controller instance. See tkl_gpu_init() for
 *       the (vendor-imposed) limit on how many may be live at once.
 */
typedef VOID_T *TKL_GPU_HANDLE;

typedef VOID *(*TUYA_GPU_ALLOC_CB)(UINT32_T size, VOID *arg);
typedef OPERATE_RET (*TUYA_GPU_FREE_CB)(VOID *ptr, VOID *arg);
typedef VOID (*TUYA_GPU_FRAME_DONE_CB)(VOID *frame, UINT32_T frame_size, VOID *arg);

/**
 * @brief Where output goes, and optionally who owns its memory.
 *
 * @note alloc_cb/free_cb are **optional and all-or-nothing**. Both NULL is the
 *       normal case: TKL picks the pool, and the buffer must not be freed or
 *       kept past the callback. Install both when the buffer has to outlive the
 *       callback or come from a pool of yours. Setting just one is rejected.
 */
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

/**
 * @brief Create a GPU pipeline context.
 *
 * @param[in]  config pipeline geometry / rotation / compression
 * @param[out] handle receives the new pipeline handle
 *
 * @note No vendor resource is touched here - the bk_gpu controller is created
 *       by tkl_gpu_set_source(), which is where the source geometry becomes
 *       known.
 * @warning Only **one** pipeline may hold a live bk_gpu controller at a time -
 *          GPU bring-up reaches global state that is not reference counted, so
 *          a second instance's teardown powers the domain down under the first.
 *          tkl_gpu_set_source() refuses with OPRT_RESOURCE_BUSY.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpu_init(CONST TUYA_GPU_CFG_T *config, TKL_GPU_HANDLE *handle);

/**
 * @brief Stop, destroy the bk_gpu controller and release the context.
 *
 * @param[in] handle pipeline handle; NULL is ignored
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpu_deinit(TKL_GPU_HANDLE handle);

/**
 * @brief Install the frame alloc / free / done callbacks.
 *
 * @param[in] handle pipeline handle
 * @param[in] sink   all three callbacks are required
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpu_set_sink(TKL_GPU_HANDLE handle, CONST TUYA_GPU_SINK_T *sink);

/**
 * @brief Bind the flexa source and build the bk_gpu controller.
 *
 * @param[in] handle pipeline handle
 * @param[in] source ISP or MJPEG-decoder flexa source
 *
 * @note The vendor controller copies its config at creation and cannot change
 *       geometry, so a source with different geometry forces a rebuild; an
 *       unchanged config keeps the existing controller.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpu_set_source(TKL_GPU_HANDLE handle, CONST TUYA_GPU_SOURCE_T *source);

/**
 * @brief Open the controller and start the flexa bond.
 *
 * @param[in] handle pipeline handle
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpu_start(TKL_GPU_HANDLE handle);

/**
 * @brief Stop the flexa bond and close the controller. The instance survives.
 *
 * @param[in] handle pipeline handle
 *
 * @note Close only, never destroy: tkl_gpu_start() can be called again on the
 *       same handle as long as the source geometry has not changed.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpu_stop(TKL_GPU_HANDLE handle);

/**
 * @brief Get the underlying bk_gpu controller handle, for bonding by peers.
 *
 * @param[in]  handle     pipeline handle
 * @param[out] raw_handle receives the bk_gpu_ctlr_handle_t
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpu_get_handle(TKL_GPU_HANDLE handle, VOID **raw_handle);

#ifdef __cplusplus
}
#endif

#endif
