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
 * Pixel formats and buffers
 * --------------------------------------------------------------------------- */

/** Pixel layouts the media hardware understands. */
typedef enum {
    TKL_PIXEL_FMT_UNKNOWN = 0,
    TKL_PIXEL_FMT_NV12,      /**< YCbCr 4:2:0 semiplanar, Y plane then interleaved CbCr */
    TKL_PIXEL_FMT_YUYV,      /**< packed 4:2:2 */
    TKL_PIXEL_FMT_RGB565,
    TKL_PIXEL_FMT_RGB888,
    TKL_PIXEL_FMT_ARGB8888,
    TKL_PIXEL_FMT_ABGR8888,

    /* Raw Bayer, as it leaves the sensor and enters the ISP. The name gives the
     * 2x2 cell reading left-to-right, top-to-bottom, so BGGR is B G / G R. Get
     * the phase wrong and demosaic swaps two colours - RGGB against a BGGR
     * sensor comes out with red and blue exchanged. */
    TKL_PIXEL_FMT_BGGR10,    /**< B G / G R, 10-bit */
    TKL_PIXEL_FMT_GBRG10,    /**< G B / R G, 10-bit */
    TKL_PIXEL_FMT_GRBG10,    /**< G R / B G, 10-bit */
    TKL_PIXEL_FMT_RGGB10,    /**< R G / G B, 10-bit */
} TKL_PIXEL_FMT_E;

/** Compressed formats. */
typedef enum {
    TKL_MEDIA_CODEC_NONE = 0, /**< raw pixels */
    TKL_MEDIA_CODEC_JPEG,
    TKL_MEDIA_CODEC_H264,
} TKL_MEDIA_CODEC_E;

/** Coded picture type, meaningful for H264 only. */
typedef enum {
    TKL_MEDIA_FRAME_UNKNOWN = 0,
    TKL_MEDIA_FRAME_I,
    TKL_MEDIA_FRAME_P,
} TKL_MEDIA_FRAME_TYPE_E;

/** What a buffer carries. */
typedef enum {
    TKL_MEDIA_BUF_PIXEL = 0, /**< raw picture; width/height/format apply */
    TKL_MEDIA_BUF_STREAM,    /**< encoded bitstream; codec/frame_type apply */
} TKL_MEDIA_BUF_KIND_E;

/**
 * @brief One media buffer, plus everything needed to release it.
 *
 * @note **The release function travels with the buffer.** Frames come from
 *       different pools and only the producing peripheral knows which, so call
 *       @ref free_cb rather than freeing, and pass the whole struct on to keep
 *       the pairing intact.
 * @note @ref free_cb may be NULL for a buffer the caller already owns, such as
 *       the destination of tkl_image_decode().
 */
typedef struct {
    UINT8_T  *data;
    UINT32_T  size;         /**< bytes of valid content */
    VOID_T  (*free_cb)(UINT8_T *buffer);  /**< release @ref data; NULL = caller owns it */

    UINT8_T   kind;         /**< TKL_MEDIA_BUF_KIND_E */
    UINT64_T  pts;          /**< milliseconds since boot */

    /* TKL_MEDIA_BUF_PIXEL */
    UINT16_T  width;
    UINT16_T  height;
    UINT32_T  format;       /**< TKL_PIXEL_FMT_E */
    UINT32_T  y_stride;     /**< 0 = tightly packed (== width) */
    UINT32_T  uv_stride;    /**< 0 = tightly packed */

    /* TKL_MEDIA_BUF_STREAM */
    UINT32_T  codec;        /**< TKL_MEDIA_CODEC_E */
    UINT32_T  frame_type;   /**< TKL_MEDIA_FRAME_TYPE_E */
} TKL_MEDIA_BUF_T;

/**
 * @brief Delivery callback for a produced buffer.
 * @param[in] buf produced buffer; valid only for the duration of the call
 *                unless the callee takes ownership via buf->free_cb
 * @param[in] arg caller context registered with the peripheral
 */
typedef VOID_T (*TKL_MEDIA_BUF_CB)(CONST TKL_MEDIA_BUF_T *buf, VOID_T *arg);

/**
 * @brief Runtime encoder controls.
 *
 * @note Shared by every peripheral that can encode; it was born inside
 *       tkl_camera_mipi.h, which is why the name is TKL_MEDIA_ rather than
 *       TKL_CAM_.
 */
typedef struct {
    UINT32_T bitrate;   /**< target bps; 0 leaves it unchanged */
    BOOL_T   force_idr; /**< request an IDR frame */
} TKL_MEDIA_ENC_CTRL_T;

/* ---------------------------------------------------------------------------
 * OSD overlay
 *
 * One call sets one whole slot. There is no named-element list inside TKL:
 * the caller owns the element array and re-submits it when anything changes.
 * --------------------------------------------------------------------------- */

/** What one overlay element carries. */
typedef enum {
    TKL_OSD_ELEM_IMAGE = 0, /**< pre-rendered ARGB8888 bitmap */
    TKL_OSD_ELEM_TEXT,      /**< UTF-8 string, rasterized inside TKL */
} TKL_OSD_ELEM_TYPE_E;

/** Which rasterizer understands TKL_OSD_ELEM_T::font. */
typedef enum {
    TKL_OSD_FONT_BITMAP = 0, /**< emWin glyph table (gui_font_digit_struct) */
    TKL_OSD_FONT_LVGL,       /**< LVGL font object (lv_font_t) */
} TKL_OSD_FONT_KIND_E;

/**
 * @brief One element inside an overlay slot.
 *
 * @note @ref font is deliberately untyped. Font assets are platform artefacts
 *       that TKL only forwards to the matching rasterizer; typing it would drag
 *       the platform's font headers into the public API. @ref font_kind says
 *       which rasterizer to hand it to.
 */
typedef struct {
    UINT8_T   type;         /**< TKL_OSD_ELEM_TYPE_E */
    UINT16_T  x;            /**< x inside the slot's sprite */
    UINT16_T  y;            /**< y inside the slot's sprite */

    /* TKL_OSD_ELEM_IMAGE */
    CONST VOID *data;       /**< pixel data, row stride = w */
    UINT16_T  w;            /**< bitmap width */
    UINT16_T  h;            /**< bitmap height */
    UINT32_T  format;       /**< TKL_PIXEL_FMT_E; only TKL_PIXEL_FMT_ARGB8888 today */

    /* TKL_OSD_ELEM_TEXT */
    CONST VOID *font;       /**< font asset, see the note above */
    UINT8_T   font_kind;    /**< TKL_OSD_FONT_KIND_E */
    CONST CHAR_T *text;     /**< UTF-8, NUL terminated */
    UINT32_T  argb;         /**< 0x00RRGGBB; alpha comes from glyph coverage */
    UINT8_T   scale;        /**< integer upscale for LVGL fonts; 0/1 = 1x, ignored for bitmap */
} TKL_OSD_ELEM_T;

/**
 * @brief One overlay slot.
 *
 * @note @ref count == 0 or @ref elems == NULL clears the slot.
 * @note @ref rotate must match the channel's display rotation. The two channels
 *       of one peripheral do **not** share a coordinate system: the encode
 *       channel works in un-rotated sensor output coordinates, the display
 *       channel in rotated panel coordinates. The same element array normally
 *       cannot be used for both.
 */
typedef struct {
    UINT8_T   slot;         /**< beyond the channel's max_slots -> OPRT_NOT_SUPPORTED */
    UINT16_T  dst_x;        /**< slot origin in the channel's coordinate system */
    UINT16_T  dst_y;
    UINT16_T  w;            /**< sprite size; 0 = derive from the elements */
    UINT16_T  h;
    UINT16_T  rotate;       /**< 0 / 90 / 270; must match the channel's rotation */
    CONST TKL_OSD_ELEM_T *elems;
    UINT8_T   count;
} TKL_OSD_T;

/** What one channel's overlay hardware can do. */
typedef struct {
    UINT8_T  max_slots;     /**< independent overlay regions */
    BOOL_T   support_text;  /**< TKL_OSD_ELEM_TEXT accepted */
    BOOL_T   support_rotate;/**< TKL_OSD_T::rotate may be non-zero */
} TKL_OSD_CAPS_T;

#ifdef __cplusplus
}
#endif

#endif /* __TKL_MEDIA_TYPES_H__ */
