/**
 * @file tkl_media_types.h
 * @brief Common process - shared media types for the multimedia peripherals
 *
 * @details
 * Types used by more than one peripheral (camera / uvc / dvp). Kept free of any
 * platform header so the public API never drags Beken types into business code.
 *
 * @version 0.1
 * @date 2026-08-27
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 */

#ifndef __TKL_MEDIA_TYPES_H__
#define __TKL_MEDIA_TYPES_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Data formats and buffers
 *
 * TUYA_FRAME_FMT_E (in tuya_cloud_types.h) describes the content of data:
 * raw pixel layouts (NV12/YUYV/RGB.../Bayer) and coded payloads (JPEG/H264)
 * use the same field. The business layer interprets the format it receives.
 * --------------------------------------------------------------------------- */

/**
 * @brief One media buffer, plus everything needed to release it.
 *
 * @note **The release function travels with the buffer.** Frames come from
 *       different pools and only the producing peripheral knows which, so call
 *       @ref free_cb on the struct when done - it releases @ref buffer and the
 *       struct itself when TKL owns them. The caller only ever calls free_cb,
 *       never free().
 * @note @ref free_cb may be NULL for a buffer the caller already owns, such as
 *       the destination of tkl_image_codec_convert().
 * @note @ref channel names the output channel that produced this buffer
 *       (TUYA_CAMERA_FRAME_CHANNEL_E; what each channel carries is defined by
 *       the per-peripheral topology tables in tkl_mipi_csi.h / tkl_uvc.h).
 *       Business demuxes on it when two channels deliver the same format.
 * @note @ref format describes the content in @ref buffer. For pixel formats,
 *       width/height describe the image layout. For coded H.264, format tells
 *       the frame type directly: TUYA_FRAME_FMT_H264_I / H264_P / H264_B.
 */
/** Output channel address - one address space across configuration,
 *  get_frame()/set_pp()/set_osd()/set_gop()/resume/cancel and
 *  TUYA_MEDIA_BUF_T.channel. What each channel carries is defined by the
 *  per-peripheral topology tables in tkl_mipi_csi.h and tkl_uvc.h. This
 *  version accepts channels 0..3; 4..9 are reserved. */
typedef enum {
    TUYA_CAMERA_FRAME_CHANNEL_0     = 0,
    TUYA_CAMERA_FRAME_CHANNEL_1     = 1,
    TUYA_CAMERA_FRAME_CHANNEL_2     = 2,
    TUYA_CAMERA_FRAME_CHANNEL_3     = 3,
    TUYA_CAMERA_FRAME_CHANNEL_4     = 4,
    TUYA_CAMERA_FRAME_CHANNEL_5     = 5,
    TUYA_CAMERA_FRAME_CHANNEL_6     = 6,
    TUYA_CAMERA_FRAME_CHANNEL_7     = 7,
    TUYA_CAMERA_FRAME_CHANNEL_8     = 8,
    TUYA_CAMERA_FRAME_CHANNEL_9     = 9,
    TUYA_CAMERA_FRAME_CHANNEL_MAX   = 10,
} TUYA_CAMERA_FRAME_CHANNEL_E;

/* forward typedef: free_cb below takes the buffer's own type, which is not
 * complete until the struct closes - C forbids the naked self-reference */
typedef struct TUYA_MEDIA_BUF_S TUYA_MEDIA_BUF_T;

struct TUYA_MEDIA_BUF_S {
    UINT8_T  *buffer;
    UINT32_T  length;         /**< bytes of valid content */
    VOID_T  (*free_cb)(TUYA_MEDIA_BUF_T *frame);  /**< release @ref buffer (and the struct
                                          itself when TKL owns it); NULL = caller owns it */

    TUYA_CAMERA_FRAME_CHANNEL_E channel;
    TUYA_FRAME_FMT_E format; /**< content pixel layout or coded format */
    UINT64_T  timestamp;          /**< milliseconds since boot */

    /* pixel formats */
    UINT16_T  width;
    UINT16_T  height;

    VOID_T *priv;           /**< peripheral-private; never touch */
    VOID_T *user_args;      /**< the arg registered with the callback that
                                 delivered this buffer */
};

/**
 * @brief Header stride of a one-block frame (struct + payload, one allocation).
 *
 * TKL puts the struct at the block base and the payload right behind it,
 * 64-aligned - payload = base + TUYA_MEDIA_BUF_HDR. A consumer that can only
 * release through a payload pointer, such as tkl_mipi_dsi_flush_frame_t's
 * UINT8_T free_cb, recovers the struct as
 * `(TUYA_MEDIA_BUF_T *)(VOID_T *)((UINT8_T *)payload - TUYA_MEDIA_BUF_HDR)`
 * and calls its free_cb from there. */
#define TUYA_MEDIA_BUF_HDR  (((UINT32_T)sizeof(TUYA_MEDIA_BUF_T) + 63U) & ~(UINT32_T)63U)

/**
 * @brief Camera lifecycle and error events, reported via TUYA_CAMERA_EVENT_CB.
 *
 * Channel-scoped faults name the channel in the event value itself (what each
 * channel carries is defined by the per-peripheral topology tables in
 * tkl_mipi_csi.h / tkl_uvc.h); hardware-scoped faults name the hardware, whose
 * channel the topology pins. Handlers run on a vendor or delivery thread -
 * keep them short.
 */
typedef enum {
    /* lifecycle */
    TUYA_CAMERA_EVENT_NONE = 0,
    TUYA_CAMERA_EVENT_CONNECT = 1,           /**< UVC: device plugged */
    TUYA_CAMERA_EVENT_DISCONNECT = 2,        /**< UVC: device unplugged */
    TUYA_CAMERA_EVENT_INITIALIZED = 3,
    TUYA_CAMERA_EVENT_STARTED = 4,
    TUYA_CAMERA_EVENT_STOPPED = 5,
    TUYA_CAMERA_EVENT_DEINITIALIZED = 6,
    /* channel-scoped flow faults; values 7-15 reserved */
    TUYA_CAMERA_EVENT_ERR_OVERFLOW_CH1 = 16, /**< ch1 delivery/parking consumer too slow */
    TUYA_CAMERA_EVENT_ERR_OVERFLOW_CH2 = 17, /**< ch2 display path dropped frames */
    TUYA_CAMERA_EVENT_ERR_OVERFLOW_CH3 = 18, /**< ch3 encode path dropped frames */
    TUYA_CAMERA_EVENT_ERR_TIMEOUT_CH1  = 19, /**< ch1 no frame for >3 s */
    TUYA_CAMERA_EVENT_ERR_TIMEOUT_CH2  = 20, /**< ch2 no frame for >3 s */
    TUYA_CAMERA_EVENT_ERR_TIMEOUT_CH3  = 21, /**< ch3 no frame for >3 s */
    /* hardware-scoped faults; the topology pins each to one channel */
    TUYA_CAMERA_EVENT_ERR_GPU   = 22,        /**< ch2 display pipeline failure */
    TUYA_CAMERA_EVENT_ERR_VENC  = 23,        /**< ch3 encode failure */
    TUYA_CAMERA_EVENT_ERR_VDEC  = 24,        /**< UVC ch1 decode failure */
    /* peripheral-scoped faults */
    TUYA_CAMERA_EVENT_ERR_SENSOR     = 25,   /**< CSI: I2C / power / detect failure */
    TUYA_CAMERA_EVENT_ERR_CSI        = 26,   /**< CSI: MIPI bus / ISP error */
    TUYA_CAMERA_EVENT_ERR_USB        = 27,   /**< UVC: transfer / enumeration error (reserved) */
    TUYA_CAMERA_EVENT_ERR_ALLOC      = 28,   /**< media buffer allocation failure */
    TUYA_CAMERA_EVENT_ERR_BAD_FORMAT = 29,   /**< UVC: device advertised an unsupported format (reserved) */
} TUYA_CAMERA_EVENT_E;

/** Camera event notification; channel info travels in the event value, so the
 *  callback carries no channel argument. Runs on a vendor or delivery thread. */
typedef VOID (*TUYA_CAMERA_EVENT_CB)(TUYA_CAMERA_EVENT_E event, VOID_T *arg);

/**
 * @brief Delivery callback for a produced buffer.
 * @param[in] buf produced buffer; valid only for the duration of the call
 *                unless the callee takes ownership via buf->free_cb. The
 *                context registered with this callback arrives in
 *                buf->user_args.
 */
typedef VOID_T (*TUYA_MEDIA_BUF_CB)(CONST TUYA_MEDIA_BUF_T *buf);

/** Encoder bitrate presets for TUYA_MEDIA_OUTPUT_CFG_T.level (ch3 only).
 *  T9 maps the levels to fixed bitrates - LOW 0.75 / MEDIUM 1.5 / HIGH 3 Mbps -
 *  via tkl_media_bitrate_from_level(); tkl_xxx_set_bitrate() fine-tunes from
 *  there and the last writer wins. */
typedef enum {
    TUYA_ENCODE_BITRATE_LEVEL_LOW     = 1, // low bitrate
    TUYA_ENCODE_BITRATE_LEVEL_MEDIUM  = 2, // medium bitrate
    TUYA_ENCODE_BITRATE_LEVEL_HIGH    = 3, // high bitrate
    TUYA_ENCODE_BITRATE_LEVEL_MAX = 4,
} TUYA_ENCODE_BITRATE_LEVEL_E;

/** Camera input frame: sensor (MIPI CSI) or USB device (UVC) geometry. */
typedef struct {
    UINT16_T         width;
    UINT16_T         height;
    UINT8_T          fps;
    TUYA_FRAME_FMT_E format;
} TUYA_CAMERA_FRAME_CFG_T;

/**
 * @brief One output channel request, addressed by @ref channel.
 *
 * One struct serves every channel. The per-peripheral topology tables in
 * tkl_mipi_csi.h / tkl_uvc.h define what each channel is and which fields it
 * honors; fields a channel does not honor must be 0 - init() rejects the
 * request otherwise. Rules common to both peripherals:
 *  - one entry per channel; channels 0..3 are accepted in this version;
 *  - ch2 (display) / ch3 (encode) require @ref cb - they are push outputs;
 *  - a channel configured with @ref cb is push-only: get_frame() on it fails;
 *  - @ref gop / @ref level are encode-only (ch3); gop 0 = encoder default (40),
 *    level maps to a bitrate via tkl_media_bitrate_from_level();
 *  - @ref rotate is honored on ch2 only (GPU 0/90/270);
 *  - ch0/ch1 geometry: width/height 0 on a pull channel means the MP output
 *    geometry, not the sensor's native size (a ch1-only camera has no MP
 *    consumer, and there the MP falls back to the full sensor frame);
 *    an NV12 pull channel additionally needs even dimensions.
 * init() deep-copies the outputs array, so the caller may release its cfg
 * after init() returns.
 */
typedef struct {
    TUYA_CAMERA_FRAME_CHANNEL_E channel;
    UINT16_T         rotate;
    UINT16_T         width;
    UINT16_T         height;
    TUYA_FRAME_FMT_E format;
    UINT32_T        gop; // H.264/H.265 (ch3); 0 = encoder default
    TUYA_ENCODE_BITRATE_LEVEL_E     level; // H.264/H.265 bitrate level (ch3)
    TUYA_MEDIA_BUF_CB cb;
    VOID_T          *arg;
} TUYA_MEDIA_OUTPUT_CFG_T;

/* ---------------------------------------------------------------------------
 * OSD overlay
 * --------------------------------------------------------------------------- */

/**
 * @brief One pre-composed overlay bitmap.
 *
 * @note The caller composes text, icons and masks before submitting. @ref data
 *       is a contiguous bitmap in @ref format with no stride padding (row
 *       stride equals the packed row width of the format).
 * @note @ref format carries a TUYA_FRAME_FMT_E value. 0 keeps the facade
 *       default ARGB8888. The GPU path additionally accepts
 *       TUYA_FRAME_FMT_NV12 - the vendor blit consumes NV12 natively (the
 *       vendor PIP demo feeds it NV12 frames straight from the ISP pool).
 *       The encode path stays ARGB8888 for now.
 * @note TKL neither allocates, copies nor releases @ref data. The buffer must
 *       remain hardware-readable for as long as the backend retains it.
 * @note A non-NULL @ref data updates the overlay identified by (@ref dst_x,
 *       @ref dst_y), or consumes one hardware overlay slot for a new position.
 *       A NULL @ref data clears that position; w/h/rotate are ignored then.
 */
typedef struct {
    UINT16_T dst_x;  /**< overlay origin in the channel coordinate system */
    UINT16_T dst_y;
    UINT16_T w;      /**< bitmap width in pixels */
    UINT16_T h;      /**< bitmap height in pixels */
    UINT16_T rotate; /**< GPU: 0/90/270; VCENC: 0 only */
    UINT32_T format; /**< TUYA_FRAME_FMT_E of @p data; 0 = default ARGB8888 */
    CONST VOID *data; /**< contiguous pixels in @ref format, owned by the caller */
} TUYA_OSD_T;

#ifdef __cplusplus
}
#endif

#endif /* __TKL_MEDIA_TYPES_H__ */
