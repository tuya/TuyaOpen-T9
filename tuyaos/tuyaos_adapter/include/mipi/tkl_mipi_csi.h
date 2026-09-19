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

/** Board-specific sensor wiring, kept separate from stream policy. */
typedef struct {
    TUYA_I2C_NUM_E    port;
    TUYA_GPIO_NUM_E   scl;
    TUYA_GPIO_NUM_E   sda;
    TUYA_GPIO_CTRL_T   reset;
    TUYA_GPIO_CTRL_T   power_ctrl;
    TUYA_GPIO_NUM_E   clk;
} TUYA_MIPI_CSI_BOARD_CFG_T;

/** Sensor description and requested outputs; the adapter derives its internal
 *  pipelines from the frame/pixel/stream enables. */
typedef struct {
    TUYA_MIPI_CSI_BOARD_CFG_T board_cfg;
    TUYA_CAMERA_FRAME_CFG_T sensor_cfg;

    TUYA_FRAME_OUTPUT_CFG_T frame;    /* enable -> raw frames via get_frame() */
    TUYA_PIXEL_OUTPUT_CFG_T  pixel;
    TUYA_STREAM_OUTPUT_CFG_T stream;
} TUYA_MIPI_CSI_CFG_T;

/** Initialize the sensor and pipelines; no output is produced until start(). */
OPERATE_RET tkl_mipi_csi_init(CONST TUYA_MIPI_CSI_CFG_T *cfg);

/** Start all enabled outputs. Idempotent after init(). */
OPERATE_RET tkl_mipi_csi_start(VOID_T);

/** Stop streaming while keeping the sensor initialized. Idempotent. */
OPERATE_RET tkl_mipi_csi_stop(VOID_T);

/** Stop, power down, and release all camera resources. */
OPERATE_RET tkl_mipi_csi_deinit(VOID_T);

/**
 * @brief Change one output's resolution while it is running.
 *
 * FRAME changes the raw-frame (SP) resolution and switches live. PIXEL and
 * STREAM are equivalent - both address the shared high-throughput pipeline
 * (MP), whose resolution lives in cfg.stream.width/height; the rebuild
 * briefly interrupts the stream. In STOPPED state the value is recorded and
 * applied at the next start().
 */
OPERATE_RET tkl_mipi_csi_set_pp(TUYA_CAMERA_OUTPUT_TYPE_E type, UINT16_T width, UINT16_T height);

/** Adjust the encoder bitrate; requires stream.enable. */
OPERATE_RET tkl_mipi_csi_set_bitrate(UINT32_T bitrate);

/** Force an IDR frame; requires stream.enable. */
OPERATE_RET tkl_mipi_csi_force_idr(VOID_T);

/** Submit or clear one business-composed overlay bitmap in osd->format
 *  (0 keeps the default ARGB8888; the GPU path also takes NV12 - see
 *  TUYA_OSD_T). PIXEL
 *  addresses the pixel (GPU) pipeline, STREAM the encoded stream. */
OPERATE_RET tkl_mipi_csi_set_osd(TUYA_CAMERA_OUTPUT_TYPE_E type,
                                 CONST TUYA_OSD_T *osd);

/** Pull one raw frame; requires cfg.frame.enable. The adapter allocates the
 *  returned buf - release both frame and struct via buf->free_cb(buf->data). */
OPERATE_RET tkl_mipi_csi_get_frame(TUYA_MEDIA_BUF_T **out, UINT32_T timeout_ms);

/** Set sensor horizontal mirror and vertical flip at runtime. */
OPERATE_RET tkl_mipi_csi_set_mirror_flip(BOOL_T hmirror, BOOL_T vflip);

/** Get current runtime mirror/flip state; either output may be NULL. */
OPERATE_RET tkl_mipi_csi_get_mirror_flip(BOOL_T *hmirror, BOOL_T *vflip);

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
