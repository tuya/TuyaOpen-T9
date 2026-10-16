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

/* ---------------------------------------------------------------------------
 * Channel topology (UVC source, T9 / BK7259) - what each channel is and where
 * it comes from.
 *
 *   USB MJPEG
 *   |
 *   +-- parked pool -----------> ch0   the MJPEG as captured   get_frame / cb
 *   |
 *   \-- vdec (MJPEG -> NV12) --> ch1   the decoded NV12        get_frame / cb
 *       |
 *       +-- GPU ---------------> ch2   display frames          cb required
 *       |
 *       \-- vcenc -------------> ch3   H264 bitstream          cb required
 *
 * ------------------------------------------------------------------------- */

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

    TUYA_MEDIA_OUTPUT_CFG_T     *outputs;
    UINT8_T                   output_num;

    /** Lifecycle and error events (TUYA_CAMERA_EVENT_E); replaces the old
     *  plug/unplug state callback. */
    TUYA_CAMERA_EVENT_CB event_cb;
    VOID_T           *event_arg;
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
 * @brief Change one output channel's resolution while it is running.
 *
 * ch1: FRAME-sink mode restarts the decoder at the new size; pull mode takes
 * effect on the next get_frame(). ch2: GPU pipeline rebuild around the new
 * size, brief hiccup, ch3 keeps streaming. ch3 is rejected - the encoder has
 * no scaler and encodes the USB-negotiated source size. ch0 is rejected (the
 * USB format was negotiated at open). INIT-state requests are recorded for
 * start().
 */
OPERATE_RET tkl_uvc_set_pp(TUYA_CAMERA_FRAME_CHANNEL_E channel, UINT16_T width, UINT16_T height);

/** Set the I-frame interval for ch3. The encoder takes gop only at build
 *  time, so a running stream refuses it: call while STOPPED - the value is
 *  recorded and applied at the next start(). gop 0 restores the default 40. */
OPERATE_RET tkl_uvc_set_gop(UINT32_T gop);

/** Submit or clear one business-composed overlay bitmap on a pipeline
 *  channel: ch2 addresses the GPU (display) path, ch3 the encoder path;
 *  ch0/ch1 carry no overlay and are rejected. See TUYA_OSD_T for the bitmap
 *  formats each backend takes. */
OPERATE_RET tkl_uvc_set_osd(TUYA_CAMERA_FRAME_CHANNEL_E channel, CONST TUYA_OSD_T *osd);

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

/** Pull one complete frame from a pull channel: ch0 (parked JPEG) or ch1
 *  (decoded NV12). get_frame(ch1) always decodes one parked MJPEG on demand -
 *  the decoder serves ch2/ch3 first and takes this job in between (see the
 *  topology note). A channel configured with cb is push-only and rejects
 *  get_frame(). The adapter allocates the returned buffer; release both frame
 *  and struct via buf->free_cb(buf). */
OPERATE_RET tkl_uvc_get_frame(TUYA_CAMERA_FRAME_CHANNEL_E channel, TUYA_MEDIA_BUF_T **out, UINT32_T timeout_ms);

/**
 * @brief Resume one configured channel's output after cancel_channel().
 *
 * Rebuilds from the init-time configuration. Idempotent when the channel is
 * already running. ch2/ch3 re-claim the shared GPU / encoder and fail with
 * OPRT_RESOURCE_NOT_READY when the other peripheral holds the hardware.
 */
OPERATE_RET tkl_uvc_resume_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel);

/**
 * @brief Cancel one configured channel's output while the camera keeps
 *        running.
 *
 * ch1: delivery detaches (FRAME-sink mode parks the decode task; pull mode
 * only gates get_frame()). ch2: the GPU pipeline is torn
 * down and ch3 keeps streaming. ch3: the encoder is torn down and ch2 keeps
 * displaying. ch0 is rejected (it is the parking pool itself). Idempotent.
 * Deinit/stop cancel everything anyway.
 */
OPERATE_RET tkl_uvc_cancel_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel);

/** Adjust the encoder bitrate (ch3); requires ch3 configured and streaming. */
OPERATE_RET tkl_uvc_set_bitrate(UINT32_T bitrate);

/** Force an IDR frame; requires ch3 running. */
OPERATE_RET tkl_uvc_force_idr(VOID_T);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_UVC_H__ */
