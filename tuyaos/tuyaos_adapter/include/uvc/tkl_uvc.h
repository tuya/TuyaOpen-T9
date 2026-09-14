/**
 * @file tkl_uvc.h
 * @brief Common process - USB video camera adapter API.
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 */

#ifndef __TKL_UVC_H__
#define __TKL_UVC_H__

#include "tuya_cloud_types.h"
#include "tkl_media_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Device plug/unplug notification. Runs on a vendor USB thread. */
typedef VOID (*TUYA_UVC_STATE_CB)(BOOL_T connected, VOID_T *arg);

/** Output sinks produced from one selected UVC input stream. */
typedef enum {
    TUYA_UVC_OUTPUT_NONE   = 0U,
    TUYA_UVC_OUTPUT_FRAME  = (1U << 0), /**< original complete UVC frame */
    TUYA_UVC_OUTPUT_PIXEL  = (1U << 1), /**< decoded and processed pixels */
    TUYA_UVC_OUTPUT_STREAM = (1U << 2), /**< re-encoded H.264 stream */
} TUYA_UVC_OUTPUT_KIND_E;

/** Which output an OSD or encoder setting applies to. */
typedef enum {
    TUYA_UVC_OSD_PIXEL  = 0U, /**< pixel output coordinates */
    TUYA_UVC_OSD_STREAM = 1U, /**< encoded stream coordinates */
} TUYA_UVC_OSD_CHANNEL_E;

/** Pixel output geometry and delivered pixel format. */
typedef struct {
    UINT16_T         rotate;
    UINT16_T         target_width;
    UINT16_T         target_height;
    TUYA_FRAME_FMT_E format;
} TUYA_UVC_PIXEL_CFG_T;

/** Encoder initialization controls; zero values use backend defaults. */
typedef struct {
    UINT8_T  fps;
    UINT32_T gop;
    UINT32_T bitrate;
} TUYA_UVC_ENCODER_CFG_T;

/**
 * @brief One UVC input stream and its requested TKL outputs.
 *
 * @note This version intentionally models one input stream. Dual-stream UVC
 *       devices are not part of this API yet.
 */
typedef struct {
    UINT8_T  port;   /**< USB hub port; 0 means the adapter default */
    UINT16_T width;
    UINT16_T height;
    UINT8_T  fps;    /**< 0 means the adapter default */

    /** Format delivered by the UVC device, not a decoder selection. */
    TUYA_FRAME_FMT_E input_format; /**< 0 uses the adapter default (JPEG) */

    /** Independent output requests; FRAME, PIXEL and STREAM may be combined. */
    UINT32_T output_kinds;

    /** Applies to TUYA_UVC_OUTPUT_PIXEL. */
    TUYA_UVC_PIXEL_CFG_T pixel;

    /** Applies to TUYA_UVC_OUTPUT_STREAM. */
    TUYA_UVC_ENCODER_CFG_T encoder;

    /** Original complete UVC frame, e.g. JPEG/MJPEG or H.264. */
    TUYA_MEDIA_BUF_CB frame_cb;
    VOID_T          *frame_arg;

    /** Decoded, scaled and rotated pixel output. */
    TUYA_MEDIA_BUF_CB pixel_cb;
    VOID_T          *pixel_arg;

    /** TKL-generated H.264 output. */
    TUYA_MEDIA_BUF_CB stream_cb;
    VOID_T          *stream_arg;

    /** Plug/unplug notification; runs on a vendor USB thread. */
    TUYA_UVC_STATE_CB state_cb;
    VOID_T           *state_arg;
} TUYA_UVC_CFG_T;

/** Enumerate the camera and allocate adapter resources; no stream yet. */
OPERATE_RET tkl_uvc_init(CONST TUYA_UVC_CFG_T *cfg);

/** Start all configured outputs. Idempotent after init(). */
OPERATE_RET tkl_uvc_start(VOID_T);

/** Stop streaming while keeping the camera initialized. Idempotent. */
OPERATE_RET tkl_uvc_stop(VOID_T);

/** Stop and release all camera resources. */
OPERATE_RET tkl_uvc_deinit(VOID_T);

/**
 * @brief Adjust encoder bitrate and/or request an IDR while streaming.
 *
 * @note STREAM output must be configured and running; otherwise the encoder
 *       does not exist and the call fails with OPRT_RESOURCE_NOT_READY.
 */
OPERATE_RET tkl_uvc_set_encode(CONST TUYA_MEDIA_ENC_CTRL_T *ctrl);

/** Submit or clear one business-composed ARGB8888 overlay bitmap. */
OPERATE_RET tkl_uvc_set_osd(TUYA_UVC_OSD_CHANNEL_E channel, CONST TUYA_OSD_T *osd);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_UVC_H__ */
