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

/* ---------------------------------------------------------------------------
 * Channel topology (T9 / BK7259) - what each channel is and where it comes
 * from.
 *
 *   sensor -> ISP
 *   |
 *   +-- SP (frame mode) -------> ch1   complete frames         get_frame / cb
 *   |
 *   \-- MP (NV12, flexa ring) -> ch0   complete MP frames      get_frame / cb
 *       |
 *       +-- GPU ---------------> ch2   display frames          cb required
 *       |
 *       \-- vcenc -------------> ch3   H264 bitstream          cb required
 *
 * ------------------------------------------------------------------------- */

/** Board-specific sensor wiring, kept separate from stream policy. */
typedef struct {
    TUYA_I2C_NUM_E    port;
    TUYA_GPIO_NUM_E   scl;
    TUYA_GPIO_NUM_E   sda;
    TUYA_GPIO_CTRL_T   reset;
    TUYA_GPIO_CTRL_T   power_ctrl;
    TUYA_GPIO_NUM_E   clk;
} TUYA_MIPI_CSI_BOARD_CFG_T;

/** Sensor description and requested outputs. outputs is a caller-allocated
 *  array of TUYA_MEDIA_OUTPUT_CFG_T (one entry per wanted channel) - init()
 *  deep-copies it, so the caller may release the cfg after init() returns.
 *  event_cb reports lifecycle and error events (TUYA_CAMERA_EVENT_E). */
typedef struct {
    TUYA_MIPI_CSI_BOARD_CFG_T board_cfg;
    TUYA_CAMERA_FRAME_CFG_T sensor_cfg;

    UINT8_T                   output_num;
    TUYA_MEDIA_OUTPUT_CFG_T     *outputs;
    TUYA_CAMERA_EVENT_CB event_cb;
    VOID_T           *event_arg;
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
 * @brief Change one output channel's resolution while it is running.
 *
 * ch1 (SP): switches live. ch2: rebuilds the GPU pipeline briefly. ch3:
 * rebuilds MP - this is the authoritative MP geometry, and since one MP feeds
 * both ch2 and ch3, ch2 is briefly interrupted too. In STOPPED state the
 * value is recorded and applied at the next start(). ch0 is rejected until
 * the vendor tap lands.
 */
OPERATE_RET tkl_mipi_csi_set_pp(TUYA_CAMERA_FRAME_CHANNEL_E channel, UINT16_T width, UINT16_T height);

/** Adjust the encoder bitrate (ch3); requires ch3 configured and streaming. */
OPERATE_RET tkl_mipi_csi_set_bitrate(UINT32_T bitrate);

/** Set the I-frame interval for ch3. The encoder takes gop only at build
 *  time, so a running stream refuses it: call while STOPPED - the value is
 *  recorded and applied at the next start(). gop 0 restores the default 40. */
OPERATE_RET tkl_mipi_csi_set_gop(UINT32_T gop);

/** Force an IDR frame; requires ch3 running. */
OPERATE_RET tkl_mipi_csi_force_idr(VOID_T);

/** Submit or clear one business-composed overlay bitmap on a pipeline
 *  channel: ch2 addresses the GPU (display) path, ch3 the encoder path;
 *  ch0/ch1 carry no overlay and are rejected. See TUYA_OSD_T for the bitmap
 *  formats each backend takes. */
OPERATE_RET tkl_mipi_csi_set_osd(TUYA_CAMERA_FRAME_CHANNEL_E channel,
                                 CONST TUYA_OSD_T *osd);

/** Pull one complete frame from a pull channel: ch1 (SP) in this version.
 *  ch0 stays rejected until the vendor ships the MP tap, and the push
 *  channels ch2/ch3 never park frames. A channel configured with cb is
 *  push-only and rejects get_frame(). The adapter allocates the returned
 *  buffer; release both frame and struct via buf->free_cb(buf). */
OPERATE_RET tkl_mipi_csi_get_frame(TUYA_CAMERA_FRAME_CHANNEL_E channel, TUYA_MEDIA_BUF_T **out, UINT32_T timeout_ms);

/**
 * @brief Resume one configured channel's output after cancel_channel().
 *
 * Rebuilds from the init-time configuration. Idempotent when the channel is
 * already running. ch2/ch3 re-claim the shared GPU / encoder and fail with
 * OPRT_RESOURCE_NOT_READY when the other peripheral holds the hardware.
 */
OPERATE_RET tkl_mipi_csi_resume_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel);

/**
 * @brief Cancel one configured channel's output while the camera keeps
 *        running.
 *
 * ch1: SP closes, get_frame() returns OPRT_RESOURCE_NOT_READY. ch2: the GPU
 * pipeline is torn down and ch3 keeps streaming. ch3: the encoder is torn
 * down and ch2 keeps displaying. ch0 is rejected. Idempotent. Deinit/stop
 * cancel everything anyway.
 */
OPERATE_RET tkl_mipi_csi_cancel_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel);

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
