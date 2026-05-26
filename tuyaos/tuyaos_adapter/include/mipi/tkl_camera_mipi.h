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

    UINT16_T output_width;
    UINT16_T output_height;
    UINT32_T output_format;         /* bk_pixel_format_t, default NV12 */
    UINT8_T output_buf_cnt;
    BOOL_T enable_flexa;
    UINT8_T work_mode;              /* 0: frame mode, 1: flexa mode */
    UINT32_T read_timeout_ms;

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

OPERATE_RET tkl_camera_mipi_init(TUYA_CAMERA_MIPI_CFG_T *config);
OPERATE_RET tkl_camera_mipi_deinit(void);
OPERATE_RET tkl_camera_mipi_start(void);
OPERATE_RET tkl_camera_mipi_stop(void);
OPERATE_RET tkl_camera_mipi_get_frame(TUYA_CAMERA_MIPI_FRAME_T *frame);
OPERATE_RET tkl_camera_mipi_get_isp_handle(VOID **isp_handle);
OPERATE_RET tkl_camera_mipi_get_flexa_source(TUYA_CAMERA_MIPI_FLEXA_SOURCE_T *source);

#ifdef __cplusplus
}
#endif

#endif
