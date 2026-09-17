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

/** One (width,height) frame descriptor and its advertised frame rates. */
typedef struct {
    UINT16_T width;
    UINT16_T height;
    UINT8_T  fps_num;                /* valid entries in fps[] */
    UINT16_T fps[8];                 /* devices listing more are truncated to 8 */
} TUYA_UVC_FRAME_INFO_T;

/** One streaming format and its frame list, as advertised by the device. */
typedef struct {
    TUYA_FRAME_FMT_E format;         /* MJPEG / H264, mapped from the UVC GUID */
    UINT8_T  frame_num;              /* valid entries in frames[] */
    TUYA_UVC_FRAME_INFO_T frames[8];
} TUYA_UVC_FORMAT_INFO_T;

/**
 * @brief USB enumeration snapshot, read with tkl_uvc_get_info() before init().
 *
 * Filled from the USB device/config descriptors and the UVC class descriptors
 * (format + frame lists) during one enumeration; no stream is opened. String
 * descriptors are read over control transfers; unavailable ones come back as
 * "".
 */
typedef struct {
    /* USB device */
    UINT16_T vid;                    /* idVendor */
    UINT16_T pid;                    /* idProduct */
    UINT16_T bcd_usb;                /* USB spec in BCD, e.g. 0x0210 */
    UINT16_T bcd_device;             /* device release number in BCD */
    UINT8_T  speed;                  /* 0=LS 1=FS 2=HS 3=SS */
    CHAR_T   manufacturer[64];       /* string descriptors; "" when unavailable */
    CHAR_T   product[64];
    CHAR_T   serial[64];
    /* USB transport */
    UINT8_T  ctrl_intf;              /* VideoControl interface number */
    UINT8_T  data_intf;              /* VideoStreaming interface number */
    BOOL_T   is_bulk;                /* FALSE = ISO IN, TRUE = Bulk IN */
    UINT16_T ep_mps;                 /* streaming endpoint max packet size */
    /* UVC */
    UINT16_T bcd_uvc;                /* UVC spec (bcdVDC) */
    UINT8_T  format_num;             /* valid entries in formats[] */
    TUYA_UVC_FORMAT_INFO_T formats[3];
} TUYA_UVC_INFO_T;

/**
 * @brief One UVC input stream and its requested outputs.
 *
 * @note This version intentionally models one input stream. Dual-stream UVC
 *       devices are not part of this API yet.
 */
typedef struct {
    UINT8_T  port;                   /* USB hub port; 0 = adapter default */
    TUYA_CAMERA_FRAME_CFG_T uvc_cfg; /* input frame: w/h/fps/format (MJPEG/H264) */

    TUYA_FRAME_OUTPUT_CFG_T frame;   /* raw frames via tkl_uvc_get_frame();
                                         MJPEG sources only this version */

    /** Applies to TUYA_CAMERA_OUTPUT_PIXEL. */
    TUYA_PIXEL_OUTPUT_CFG_T pixel;

    /** Applies to TUYA_CAMERA_OUTPUT_STREAM. */
    TUYA_STREAM_OUTPUT_CFG_T stream;

    /** Plug/unplug notification; runs on a vendor USB thread. */
    TUYA_UVC_STATE_CB connect_state_cb;
    VOID_T           *connect_state_arg;
} TUYA_UVC_CFG_T;

/** Enumerate the camera and allocate adapter resources; no stream yet. */
OPERATE_RET tkl_uvc_init(CONST TUYA_UVC_CFG_T *cfg);

/** Start all configured outputs. Idempotent after init(). */
OPERATE_RET tkl_uvc_start(VOID_T);

/** Stop streaming while keeping the camera initialized. Idempotent. */
OPERATE_RET tkl_uvc_stop(VOID_T);

/** Stop and release all camera resources. */
OPERATE_RET tkl_uvc_deinit(VOID_T);

/** Submit or clear one business-composed ARGB8888 overlay bitmap. */
OPERATE_RET tkl_uvc_set_osd(TUYA_CAMERA_OUTPUT_TYPE_E type, CONST TUYA_OSD_T *osd);

/**
 * @brief Read the USB/UVC enumeration snapshot before tkl_uvc_init().
 *
 * Enumerates the bus once (no stream, no pipelines) and fills @p info. Safe
 * to call repeatedly; the device must be plugged in.
 *
 * @return OPRT_OK on success; OPRT_RESOURCE_NOT_READY when no camera is
 *         present on @c port
 */
OPERATE_RET tkl_uvc_get_info(TUYA_UVC_INFO_T *info);

/** Pull one raw frame; requires cfg.frame.enable. Caller releases via the
 *  returned buf's free_cb - it returns both the frame and the buf struct. */
OPERATE_RET tkl_uvc_get_frame(TUYA_MEDIA_BUF_T **out, UINT32_T timeout_ms);

/**
 * @brief Adjust the encoder bitrate while streaming.
 * @note STREAM output (MJPEG source, internal transcoder) must be configured
 *       and running; otherwise OPRT_RESOURCE_NOT_READY.
 */
OPERATE_RET tkl_uvc_set_bitrate(UINT32_T bitrate);

/** Force an IDR frame; requires STREAM (MJPEG source) to be running. */
OPERATE_RET tkl_uvc_force_idr(VOID_T);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_UVC_H__ */
