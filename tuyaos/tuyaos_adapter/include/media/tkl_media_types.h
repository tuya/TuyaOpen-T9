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

/** Encoded video frame classification, meaningful when the payload has one. */
typedef enum {
    TUYA_VIDEO_FRAME_UNKNOWN = 0,
    TUYA_VIDEO_FRAME_I,
    TUYA_VIDEO_FRAME_P,
} TUYA_VIDEO_FRAME_TYPE_E;

/**
 * @brief One media buffer, plus everything needed to release it.
 *
 * @note **The release function travels with the buffer.** Frames come from
 *       different pools and only the producing peripheral knows which, so call
 *       @ref free_cb rather than freeing, and pass the whole struct on to keep
 *       the pairing intact.
 * @note @ref free_cb may be NULL for a buffer the caller already owns, such as
 *       the destination of tkl_image_codec_decode().
 * @note @ref format describes the content in @ref data. For pixel formats,
 *       width/height describe the image layout. For coded
 *       video formats, @ref frame_type may classify the encoded frame.
 */
typedef struct {
    UINT8_T  *data;
    UINT32_T  size;         /**< bytes of valid content */
    VOID_T  (*free_cb)(UINT8_T *buffer);  /**< release @ref data; NULL = caller owns it */

    TUYA_FRAME_FMT_E format; /**< content pixel layout or coded format */
    UINT64_T  pts;          /**< milliseconds since boot */

    /* pixel formats */
    UINT16_T  width;
    UINT16_T  height;

    /* stream formats (JPEG/H264) */
    TUYA_VIDEO_FRAME_TYPE_E frame_type;
} TUYA_MEDIA_BUF_T;

/**
 * @brief Delivery callback for a produced buffer.
 * @param[in] buf produced buffer; valid only for the duration of the call
 *                unless the callee takes ownership via buf->free_cb
 * @param[in] arg caller context registered with the peripheral
 */
typedef VOID_T (*TUYA_MEDIA_BUF_CB)(CONST TUYA_MEDIA_BUF_T *buf, VOID_T *arg);

/**
 * @brief Runtime encoder controls.
 *
 * @note Shared by every peripheral that can encode; it was born inside
 *       tkl_mipi_csi.h, which is why the name is TUYA_MEDIA_ rather than
 *       TKL_CAM_.
 */
typedef struct {
    UINT32_T bitrate;   /**< target bps; 0 leaves it unchanged */
    BOOL_T   force_idr; /**< request an IDR frame */
} TUYA_MEDIA_ENC_CTRL_T;

/* ---------------------------------------------------------------------------
 * OSD overlay
 * --------------------------------------------------------------------------- */

/**
 * @brief One pre-composed ARGB8888 overlay bitmap.
 *
 * @note The caller composes text, icons and masks before submitting. @ref data
 *       is a contiguous bitmap with a row stride of @ref w * 4 bytes.
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
    CONST VOID *data; /**< contiguous ARGB8888 pixels, owned by the caller */
} TUYA_OSD_T;

#ifdef __cplusplus
}
#endif

#endif /* __TKL_MEDIA_TYPES_H__ */
