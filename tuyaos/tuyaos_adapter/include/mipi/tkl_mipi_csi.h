/**
 * @file tkl_mipi_csi.h
 * @brief Common process - MIPI CSI camera adapter API.
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 */

#ifndef __TKL_MIPI_CSI_H__
#define __TKL_MIPI_CSI_H__

#include "tuya_cloud_types.h"
#include "tkl_media_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** T9 primary/secondary paths; other platforms may map these to equivalent paths. */
typedef enum {
    TUYA_MIPI_CSI_CHANNEL_PRIMARY   = 0U,
    TUYA_MIPI_CSI_CHANNEL_SECONDARY = 1U,
} TUYA_MIPI_CSI_CHANNEL_E;

/** Output sinks selected on a path. Values are stable bit flags: 0, 1, 2, 4. */
typedef enum {
    TUYA_MIPI_CSI_OUTPUT_NONE   = 0U,
    TUYA_MIPI_CSI_OUTPUT_FRAME  = (1U << 0), /**< pull with tkl_mipi_csi_get_frame() */
    TUYA_MIPI_CSI_OUTPUT_PIXEL  = (1U << 1), /**< processed pixels via pixel_cb */
    TUYA_MIPI_CSI_OUTPUT_STREAM = (1U << 2), /**< encoded stream via stream_cb */
} TUYA_MIPI_CSI_OUTPUT_KIND_E;

/** Output to which an OSD request applies. */
typedef enum {
    TUYA_MIPI_CSI_OSD_PIXEL  = 0U, /**< target/display coordinates */
    TUYA_MIPI_CSI_OSD_STREAM = 1U, /**< unrotated encoded-frame coordinates */
} TUYA_MIPI_CSI_OSD_CHANNEL_E;

/** Board-specific sensor wiring, kept separate from stream policy. */
typedef struct {
    INT_T   pin_scl;
    INT_T   pin_sda;
    INT_T   pin_reset;
    INT_T   pin_pwdn;
    INT_T   pin_xclk;
    UINT8_T i2c_id;
} TUYA_MIPI_CSI_BOARD_CFG_T;

/** One ISP path and its requested consumers. */
typedef struct {
    TUYA_MIPI_CSI_OUTPUT_KIND_E output_kinds;
    UINT16_T                    width;
    UINT16_T                    height;
    TUYA_FRAME_FMT_E            format;
} TUYA_MIPI_CSI_PATH_CFG_T;

/** Pixel output geometry and delivered pixel format. */
typedef struct {
    UINT16_T         rotate;
    UINT16_T         target_width;
    UINT16_T         target_height;
    TUYA_FRAME_FMT_E format;
} TUYA_MIPI_CSI_PIXEL_CFG_T;

/** Encoder initialization controls; zero values use backend defaults. */
typedef struct {
    UINT8_T  fps;
    UINT32_T gop;
    UINT32_T bitrate;
} TUYA_MIPI_CSI_ENCODER_CFG_T;

/** Sensor description and requested path consumers. */
typedef struct {
    TUYA_MIPI_CSI_BOARD_CFG_T board;

    UINT16_T         sensor_width;
    UINT16_T         sensor_height;
    UINT8_T          sensor_fps;
    TUYA_FRAME_FMT_E sensor_input_format;

    TUYA_MIPI_CSI_PATH_CFG_T   primary;
    TUYA_MIPI_CSI_PATH_CFG_T   secondary;
    TUYA_MIPI_CSI_PIXEL_CFG_T  pixel;
    TUYA_MIPI_CSI_ENCODER_CFG_T encoder;

    TUYA_MEDIA_BUF_CB pixel_cb;
    VOID_T           *pixel_arg;
    TUYA_MEDIA_BUF_CB stream_cb;
    VOID_T           *stream_arg;
} TUYA_MIPI_CSI_CFG_T;

/** Initialize the sensor and paths; no output is produced until start(). */
OPERATE_RET tkl_mipi_csi_init(CONST TUYA_MIPI_CSI_CFG_T *cfg);

/** Start all configured paths. Idempotent after init(). */
OPERATE_RET tkl_mipi_csi_start(VOID_T);

/** Stop streaming while keeping the sensor initialized. Idempotent. */
OPERATE_RET tkl_mipi_csi_stop(VOID_T);

/** Stop, power down, and release all camera resources. */
OPERATE_RET tkl_mipi_csi_deinit(VOID_T);

/** Change a primary/secondary path resolution while it is running. */
OPERATE_RET tkl_mipi_csi_set_output(UINT8_T channel_id, UINT16_T width, UINT16_T height);

/** Adjust encoder bitrate and/or request an IDR while streaming. */
OPERATE_RET tkl_mipi_csi_set_encode(CONST TUYA_MEDIA_ENC_CTRL_T *ctrl);

/** Submit or clear one business-composed ARGB8888 overlay bitmap. */
OPERATE_RET tkl_mipi_csi_set_osd(TUYA_MIPI_CSI_OSD_CHANNEL_E channel,
                                 CONST TUYA_OSD_T *osd);

/** Pull one frame from a path configured with TUYA_MIPI_CSI_OUTPUT_FRAME. */
OPERATE_RET tkl_mipi_csi_get_frame(TUYA_MEDIA_BUF_T *out, UINT8_T channel_id,
                                   UINT32_T timeout_ms);

/** Set sensor horizontal mirror and vertical flip at runtime. */
OPERATE_RET tkl_mipi_csi_set_mirror_flip(UINT8_T hmirror, UINT8_T vflip);

/** Get current runtime mirror/flip state; either output may be NULL. */
OPERATE_RET tkl_mipi_csi_get_mirror_flip(UINT8_T *hmirror, UINT8_T *vflip);

/** Get ISP exposure-normalized luminance. */
OPERATE_RET tkl_mipi_csi_get_exposure_luminance(UINT32_T *luminance);

/** Switch ISP output between gray and color via CPROC saturation. */
OPERATE_RET tkl_mipi_csi_set_color_to_gray(BOOL_T enable);

/** Switch ISP gamma-out between night curve and backed-up day curve. */
OPERATE_RET tkl_mipi_csi_set_night_gamma(BOOL_T enable);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_MIPI_CSI_H__ */
