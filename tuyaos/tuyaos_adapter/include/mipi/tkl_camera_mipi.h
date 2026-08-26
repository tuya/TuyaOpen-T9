/**
 * @file tkl_camera_mipi.h
 * @brief Common process - adapter the MIPI camera api
 * @version 0.1
 * @date 2026-04-10
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_CAMERA_MIPI_H__
#define __TKL_CAMERA_MIPI_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TUYA_CAMERA_MIPI_CHANNEL_MP 0U
#define TUYA_CAMERA_MIPI_CHANNEL_SP 1U

typedef struct {
    INT_T pin_scl;
    INT_T pin_sda;
    UINT8_T i2c_id;
    INT_T pin_reset;
    INT_T pin_pwdn;
    INT_T pin_xclk;

    UINT16_T sensor_width;
    UINT16_T sensor_height;
    UINT8_T sensor_fps;
    UINT32_T sensor_input_format;   /* bk_pixel_format_t, default RAW10 */
    UINT8_T hmirror;                /* Horizontal mirror: 0=off, 1=on */
    UINT8_T vflip;                  /* Vertical flip: 0=off, 1=on */

    UINT16_T output_width;
    UINT16_T output_height;
    UINT32_T output_format;         /* bk_pixel_format_t, default NV12 */
    UINT8_T output_buf_cnt;
    BOOL_T enable_flexa;
    UINT8_T work_mode;              /* 0: frame mode, 1: flexa mode */
    UINT32_T read_timeout_ms;
    UINT8_T skip_frames;    /* MP AE 预热：通道打开后丢弃前 N 帧，0=关闭 */

    BOOL_T sp_enable;
    UINT16_T sp_width;
    UINT16_T sp_height;
    UINT32_T sp_format;             /* bk_pixel_format_t, default output_format */
    UINT8_T sp_buf_cnt;
    BOOL_T sp_enable_flexa;
    UINT8_T sp_work_mode;           /* 0: frame mode, 1: flexa mode */
} TUYA_CAMERA_MIPI_CFG_T;

typedef struct {
    UINT8_T *buffer;
    UINT32_T size;
    UINT32_T timeout_ms;
    UINT32_T out_len;
    UINT16_T width;
    UINT16_T height;
    UINT32_T format;                /* bk_pixel_format_t */
    UINT8_T channel_id;             /* TUYA_CAMERA_MIPI_CHANNEL_*, default MP */
    UINT32_T y_stride;              /* bytes per Y/RGB row in buffer after get_frame */
    UINT32_T uv_stride;             /* bytes per UV row for NV12; 0 if N/A */
} TUYA_CAMERA_MIPI_FRAME_T;

typedef struct {
    VOID *isp_handle;
    UINT8_T *src_buffer;
    UINT8_T buf_cnt;
    UINT16_T width;
    UINT16_T height;
    UINT32_T format;                /* bk_pixel_format_t */
    UINT32_T y_addr;
    UINT32_T u_addr;
    UINT32_T v_addr;
} TUYA_CAMERA_MIPI_FLEXA_SOURCE_T;

/**
 * @brief ISP CPROC manual attributes (color processing)
 */
typedef struct {
    UINT8_T enable;       /* 0=disable, 1=enable */
    UINT8_T op_type;      /* 0=auto, 1=manual */
    INT8_T brightness;    /* [-127, 127] */
    UINT8_T contrast;     /* [0, 255] */
    UINT8_T saturation;   /* [0, 255] */
    INT8_T hue;           /* [-90, 90] */
} TKL_CAMERA_MIPI_CPROC_T;

OPERATE_RET tkl_camera_mipi_init(TUYA_CAMERA_MIPI_CFG_T *config);
OPERATE_RET tkl_camera_mipi_deinit(void);
OPERATE_RET tkl_camera_mipi_start(void);
OPERATE_RET tkl_camera_mipi_stop(void);
/**
 * @brief Set ISP output resolution of the given channel
 * @param[in] channel_id TUYA_CAMERA_MIPI_CHANNEL_MP or TUYA_CAMERA_MIPI_CHANNEL_SP
 * @param[in] width new output width in pixels
 * @param[in] height new output height in pixels
 * @return OPRT_OK on success
 * @note MP-only streaming uses stop()+start(); MP+SP concurrent uses live channel switch.
 */
OPERATE_RET tkl_camera_mipi_set_output(UINT8_T channel_id, UINT16_T width, UINT16_T height);
/**
 * @brief Set sensor horizontal mirror and vertical flip
 * @param[in] hmirror 0=off, non-zero=on
 * @param[in] vflip 0=off, non-zero=on
 * @return OPRT_OK on success
 * @note If camera has started, applies immediately via sensor registers.
 *       If not started yet, values are saved and applied on next start.
 */
OPERATE_RET tkl_camera_mipi_set_mirror_flip(UINT8_T hmirror, UINT8_T vflip);
/**
 * @brief Get current mirror/flip config
 * @param[out] hmirror current hmirror (0/1), may be NULL
 * @param[out] vflip current vflip (0/1), may be NULL
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_camera_mipi_get_mirror_flip(UINT8_T *hmirror, UINT8_T *vflip);
/**
 * @brief Get ISP exposure-normalized luminance
 * @param[out] luminance (meanLum * 1e6) / (exposure[0] * 39)
 * @return OPRT_OK on success
 * @note Requires camera started. Wraps BK_CAM_IOCTL_GET_EXPOSURE_LUMINANCE.
 */
OPERATE_RET tkl_camera_mipi_get_exposure_luminance(UINT32_T *luminance);
/**
 * @brief Switch ISP output between gray and color via CPROC saturation
 * @param[in] enable TRUE=gray (saturation 0, manual), FALSE=restore sensor default CPROC
 * @return OPRT_OK on success
 * @note Requires camera started. Gray keeps brightness/contrast/hue of the current CPROC.
 */
OPERATE_RET tkl_camera_mipi_set_color_to_gray(BOOL_T enable);
/**
 * @brief Switch ISP gamma-out between night curve and backed-up day curve
 * @param[in] enable TRUE=apply night gamma-out (backup current first), FALSE=restore backup
 * @return OPRT_OK on success
 * @note Requires camera started. Night curve fields are left for product fill-in.
 */
OPERATE_RET tkl_camera_mipi_set_night_gamma(BOOL_T enable);
/**
 * @brief Get current ISP CPROC attributes
 * @param[out] cproc CPROC snapshot (manual fields only)
 * @return OPRT_OK on success
 * @note Requires camera started. Wraps BK_CAM_IOCTL_GET_CPROC.
 */
OPERATE_RET tkl_camera_mipi_get_cproc(TKL_CAMERA_MIPI_CPROC_T *cproc);
OPERATE_RET tkl_camera_mipi_get_frame(TUYA_CAMERA_MIPI_FRAME_T *frame);
OPERATE_RET tkl_camera_mipi_get_isp_handle(VOID **isp_handle);
OPERATE_RET tkl_camera_mipi_get_flexa_source(TUYA_CAMERA_MIPI_FLEXA_SOURCE_T *source);

#ifdef __cplusplus
}
#endif

#endif
