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
 *       @ref free_cb rather than freeing, and pass the whole struct on to keep
 *       the pairing intact. For buffers the peripheral allocated as one block
 *       (struct + frame), free_cb releases both - the caller only ever calls
 *       free_cb, never free().
 * @note @ref free_cb may be NULL for a buffer the caller already owns, such as
 *       the destination of tkl_image_codec_decode().
 * @note @ref format describes the content in @ref data. For pixel formats,
 *       width/height describe the image layout. For coded H.264, format tells
 *       the frame type directly: TUYA_FRAME_FMT_H264_I / H264_P / H264_B.
 */
typedef struct {
    UINT8_T  *data;
    UINT32_T  size;         /**< bytes of valid content */
    VOID_T  (*free_cb)(void *data);  /**< release @ref data (and the buffer
                                          itself when TKL owns it); NULL = caller owns it */

    TUYA_FRAME_FMT_E format; /**< content pixel layout or coded format */
    UINT64_T  timestamp;          /**< milliseconds since boot */

    /* pixel formats */
    UINT16_T  width;
    UINT16_T  height;

    VOID_T *priv;           /**< peripheral-private; never touch */
    VOID_T *user_args;      /**< the arg registered with the callback that
                                 delivered this buffer */
} TUYA_MEDIA_BUF_T;

/**
 * @brief Delivery callback for a produced buffer.
 * @param[in] buf produced buffer; valid only for the duration of the call
 *                unless the callee takes ownership via buf->free_cb. The
 *                context registered with this callback arrives in
 *                buf->user_args.
 */
typedef VOID_T (*TUYA_MEDIA_BUF_CB)(CONST TUYA_MEDIA_BUF_T *buf);

/** Outputs requested from a camera (MIPI CSI / DVP / UVC). Stable bit flags:
 *  0, 1, 2, 4. Also used as the type selector on set_pp()/set_osd(). */
typedef enum {
    TUYA_CAMERA_OUTPUT_NONE   = 0U,
    TUYA_CAMERA_OUTPUT_FRAME  = (1U << 0),
    TUYA_CAMERA_OUTPUT_PIXEL  = (1U << 1),
    TUYA_CAMERA_OUTPUT_STREAM = (1U << 2),
} TUYA_CAMERA_OUTPUT_TYPE_E;

/** Camera input frame: sensor (MIPI CSI) or USB device (UVC) geometry. */
typedef struct {
    UINT16_T         width;
    UINT16_T         height;
    UINT8_T          fps;
    TUYA_FRAME_FMT_E format;
} TUYA_CAMERA_FRAME_CFG_T;

/** Raw frame output (SP on T9): enable pulls frames via get_frame(). */
typedef struct {
    BOOL_T         enable;
    UINT16_T         width;
    UINT16_T         height;
    TUYA_FRAME_FMT_E format;
} TUYA_FRAME_OUTPUT_CFG_T;

/** Pixel output: enable routes the decoded/scaled/rotated result to pixel_cb. */
typedef struct {
    BOOL_T         enable;
    UINT16_T         rotate;
    UINT16_T         width;
    UINT16_T         height;
    TUYA_FRAME_FMT_E format;
    TUYA_MEDIA_BUF_CB pixel_cb;       /** Decoded, scaled and rotated pixel output. */
    VOID_T          *pixel_arg;
} TUYA_PIXEL_OUTPUT_CFG_T;

/** Encoded stream output; zero values use backend defaults. rotate is not
 *  implemented yet (TODO). width/height is the encoder input geometry and the
 *  authoritative MP resolution when both pixel and stream are enabled. */
typedef struct {
    BOOL_T          enable;
    UINT16_T         rotate; // TODO: 添加旋转角度
    UINT8_T         fps;
    UINT32_T        gop;
    UINT32_T        bitrate;
    UINT16_T         width;
    UINT16_T         height;
    TUYA_FRAME_FMT_E format;
    TUYA_MEDIA_BUF_CB stream_cb;    /** TKL-generated H.264/H.265 output. */
    VOID_T          *stream_arg;
} TUYA_STREAM_OUTPUT_CFG_T;

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
