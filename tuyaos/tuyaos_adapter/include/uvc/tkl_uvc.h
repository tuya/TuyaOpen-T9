/**
 * @file tkl_uvc.h
 * @brief TuyaOS Kernel Layer (TKL) - USB video camera (UVC) abstraction
 *        interface
 * @version 0.1
 * @date 2026-04-09
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 *
 * This header defines the UVC camera TKL interface that every platform
 * adaptor must implement. A new platform porter provides only a tkl_uvc.c
 * that satisfies the contracts below; no changes to upper layers are
 * required.
 *
 * ============================================================
 * Porting guide for a new platform
 * ============================================================
 *
 * 1. Output channels
 *    A camera exposes logical output channels addressed by
 *    TUYA_CAMERA_FRAME_CHANNEL_E. What each channel carries is part of the
 *    interface contract; which peripheral implements it is not:
 *
 *      ch0  the frame as captured, undecoded          pull or push
 *      ch1  the decoded frame (NV12)                  pull or push
 *      ch2  display-bound frames                      push only, cb required
 *      ch3  encoded H.264 bitstream                   push only, cb required
 *
 *    ch2 and ch3 are fed by the same decode output, so a change to the
 *    decoded geometry (set_pp on ch1) reaches them as well.
 *
 *    The pipeline shape below is the reference topology. The peripheral
 *    names are descriptive, not contractual - a port maps them onto its own
 *    hardware:
 *
 *      USB source (MJPEG / H.264)
 *        |
 *        +-- parking pool ----------> ch0  frame as captured   get_frame / cb
 *        |
 *        \-- decoder (MJPEG->NV12) -> ch1  decoded NV12        get_frame / cb
 *            |
 *            +-- display pipeline --> ch2  display frames      cb required
 *            |
 *            \-- H.264 encoder -----> ch3  H.264 bitstream     cb required
 *
 *    ch0 is the parking pool itself: it holds the frames as they arrive off
 *    the bus, and it is what every other channel is fed from.
 *
 * 2. Input streams
 *    This version intentionally models one input stream. Dual-stream UVC
 *    devices are not part of this API yet; a port that cannot open the
 *    requested stream must fail init() rather than pick a different one.
 *
 * 3. Delivery modes
 *    A channel without cb is a pull channel: the caller reads frames with
 *    tkl_uvc_get_frame(). A channel with cb is a push channel: the adaptor
 *    calls cb as frames become available and get_frame() on that channel is
 *    rejected. ch2 and ch3 are always push.
 *
 *    ch1 additionally distinguishes two pull-side behaviours:
 *      - sink mode (cb configured): a decode task pushes frames into the
 *        registered callback as they are produced;
 *      - on-demand mode (no cb): get_frame() decodes one parked frame from
 *        ch0 per call. The decoder serves ch2/ch3 first and takes these
 *        jobs in between, so latency is not bounded by the frame interval.
 *    Document which behaviour your adaptor provides.
 *
 * 4. Buffer ownership
 *    - get_frame(): the adaptor allocates the returned TUYA_MEDIA_BUF_T. The
 *      caller releases both frame and struct with buf->free_cb(buf), and must
 *      never call free() on either.
 *    - Push channels: the buffer handed to cb is valid only for the duration
 *      of the call. To keep it, the callee takes ownership and releases it
 *      later through buf->free_cb.
 *    - init() deep-copies the outputs array, so the caller may release its
 *      cfg as soon as init() returns.
 *
 * 5. Callbacks and threading
 *    event_cb and every TUYA_MEDIA_BUF_CB run on a vendor or delivery thread.
 *    They must not block, allocate memory, print, or re-enter this module.
 *    Channel identity travels in the event value (TUYA_CAMERA_EVENT_E), so
 *    the event callback takes no channel argument.
 *
 * 6. Unsupported behaviour
 *    A channel or feature a port does not implement must be rejected with
 *    OPRT_NOT_SUPPORTED rather than silently ignored. ch2 and ch3 share one
 *    display / encode peripheral: when the other peripheral already holds it,
 *    resume fails with OPRT_RESOURCE_NOT_READY and the caller is expected to
 *    retry.
 */

#ifndef __TKL_UVC_H__
#define __TKL_UVC_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** One (width,height) frame descriptor and its advertised frame rates.
 *
 *  @note @ref fps_num gives the number of valid entries in @ref fps. A device
 *        advertising more than 8 rates is truncated to 8.
 */
typedef struct {
    UINT16_T width;
    UINT16_T height;
    UINT8_T  fps_num;                /* valid entries in fps[] */
    UINT16_T fps[8];                 /* devices listing more are truncated to 8 */
} TUYA_UVC_FRAME_INFO_T;

/** One streaming format and its frame list, as advertised by the device.
 *
 *  @note @ref frame_num gives the number of valid entries in @ref frames. A
 *        device advertising more than 8 frames is truncated to 8.
 */
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
 *
 * @note @ref format_num gives the number of valid entries in @ref formats. A
 *       device advertising more than 3 formats is truncated to 3.
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
 * @note @ref outputs is a caller-allocated array of TUYA_MEDIA_OUTPUT_CFG_T,
 *       one entry per wanted channel. init() deep-copies it, so the caller
 *       may release the cfg after init() returns.
 * @note @ref event_cb reports lifecycle and error events
 *       (TUYA_CAMERA_EVENT_E); @ref event_arg is passed back to it unchanged.
 *       It replaces the old plug/unplug state callback.
 */
typedef struct {
    UINT8_T                  port;     /* USB hub port; 0 = adapter default */
    TUYA_CAMERA_FRAME_CFG_T  uvc_cfg;  /* input frame: w/h/fps/format (MJPEG/H264) */

    UINT8_T                  output_num;
    TUYA_MEDIA_OUTPUT_CFG_T *outputs;

    TUYA_CAMERA_EVENT_CB     event_cb;
    VOID_T                  *event_arg;
} TUYA_UVC_BASE_CFG_T;

/**
 * @brief Enumerate the camera and allocate adapter resources.
 *
 * No stream is opened until tkl_uvc_start() is called.
 *
 * @param[in] cfg input stream and requested output channels; must not be NULL
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_init(CONST TUYA_UVC_BASE_CFG_T *cfg);

/**
 * @brief Start all configured outputs.
 *
 * Idempotent after init().
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_start(VOID_T);

/**
 * @brief Stop streaming while keeping the camera initialized.
 *
 * Idempotent. Use tkl_uvc_deinit() to also release the camera.
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_stop(VOID_T);

/**
 * @brief Stop and release all camera resources.
 *
 * Cancels every channel.
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_deinit(VOID_T);

/**
 * @brief Change one output channel's resolution while it is running.
 *
 * Per-channel behaviour:
 *  - ch1: in sink mode the decoder restarts at the new size; in on-demand
 *    mode the new size takes effect on the next get_frame().
 *  - ch2: rebuilds the display pipeline around the new size, a brief hiccup;
 *    ch3 keeps streaming.
 *  - ch3: rejected - the encoder has no scaler and encodes the source size
 *    negotiated over USB.
 *  - ch0: rejected - the USB format was negotiated at open.
 *
 * Requests made while STOPPED are recorded and applied at the next start().
 *
 * @param[in] channel output channel to reconfigure
 * @param[in] width   target width in pixels
 * @param[in] height  target height in pixels
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_set_pp(TUYA_CAMERA_FRAME_CHANNEL_E channel, UINT16_T width, UINT16_T height);

/**
 * @brief Set the I-frame interval for ch3.
 *
 * The encoder accepts a new GOP only at build time, so a running stream
 * refuses the call: invoke it while STOPPED - the value is recorded and
 * applied at the next start(). gop 0 restores the encoder default.
 *
 * @param[in] gop I-frame interval; 0 = encoder default
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_set_gop(UINT32_T gop);

/**
 * @brief Submit or clear one business-composed overlay bitmap.
 *
 * ch2 addresses the display path, ch3 the encoder path; ch0/ch1 carry no
 * overlay and are rejected. See TUYA_OSD_T for the bitmap formats each
 * backend accepts.
 *
 * @param[in] channel ch2 or ch3
 * @param[in] osd     overlay description; a NULL @ref TUYA_OSD_T.data clears
 *                    the position
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_set_osd(TUYA_CAMERA_FRAME_CHANNEL_E channel, CONST TUYA_OSD_T *osd);

/**
 * @brief Read the USB/UVC enumeration snapshot.
 *
 * Enumerates the bus once (no stream, no pipelines) and fills @p info. Safe
 * to call repeatedly; the device must be plugged in. Call it before
 * tkl_uvc_init() to discover which formats and frame sizes the device
 * supports.
 *
 * @param[out] info receives the enumeration snapshot; must not be NULL
 *
 * @return OPRT_OK on success; OPRT_RESOURCE_NOT_READY when no camera is
 *         present on @c port
 */
OPERATE_RET tkl_uvc_get_info(TUYA_UVC_INFO_T *info);

/**
 * @brief Pull one complete frame from a pull channel.
 *
 * ch0 returns the frame as captured (undecoded, e.g. MJPEG); ch1 returns the
 * decoded NV12. get_frame(ch1) always decodes one parked frame from ch0 on
 * demand - the decoder serves ch2/ch3 first and takes this job in between
 * (see the topology note). A channel configured with cb is push-only and
 * rejects get_frame(). The adaptor allocates the returned buffer; release
 * both frame and struct via buf->free_cb(buf).
 *
 * @param[in]  channel    pull channel to read from (ch0 or ch1)
 * @param[out] out        receives the frame; must not be NULL
 * @param[in]  timeout_ms wait budget in milliseconds; 0 = do not wait
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_get_frame(TUYA_CAMERA_FRAME_CHANNEL_E channel, TUYA_MEDIA_BUF_T **out, UINT32_T timeout_ms);

/**
 * @brief Resume one configured channel's output after cancel_channel().
 *
 * Rebuilds from the init-time configuration. Idempotent when the channel is
 * already running. ch2/ch3 re-claim the shared display / encode peripheral
 * and fail with OPRT_RESOURCE_NOT_READY when the other one holds it.
 *
 * @param[in] channel channel to resume
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_resume_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel);

/**
 * @brief Cancel one configured channel's output while the camera keeps
 *        running.
 *
 * Per-channel behaviour:
 *  - ch1: delivery detaches. In sink mode the decode task parks; in
 *    on-demand mode only get_frame() is gated.
 *  - ch2: the display pipeline is torn down and ch3 keeps streaming.
 *  - ch3: the encoder is torn down and ch2 keeps displaying.
 *  - ch0: rejected - it is the parking pool itself.
 *
 * Idempotent. deinit() / stop() cancel everything anyway.
 *
 * @param[in] channel channel to cancel
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_cancel_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel);

/**
 * @brief Adjust the encoder bitrate.
 *
 * Requires ch3 configured and streaming.
 *
 * @param[in] bitrate target bitrate in bps
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_set_bitrate(UINT32_T bitrate);

/**
 * @brief Force an IDR frame.
 *
 * Requires ch3 running.
 *
 * @return OPRT_OK on success. Others on error, please refer to
 *         tuya_error_code.h
 */
OPERATE_RET tkl_uvc_force_idr(VOID_T);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_UVC_H__ */
