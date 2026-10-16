/**
 * @file tkl_image.h
 * @brief Common process - one-shot image operations
 *
 * @details
 * Memory in, memory out. Every call blocks until the hardware is done; there is
 * no handle, no stream state and no callback, so anything stateful (H264)
 * belongs to a peripheral or to tkl_stream instead.
 *
 * The caller supplies the destination buffer, so none of these transfer
 * ownership.
 *
 * @version 0.1
 * @date 2026-08-27
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 */

#ifndef __TKL_IMAGE_H__
#define __TKL_IMAGE_H__

#include "tuya_cloud_types.h"
#include "tkl_media_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** What a bitstream turned out to contain. */
typedef struct {
    UINT16_T width;
    UINT16_T height;
    UINT32_T format;    /**< TKL_PIXEL_FMT_E, the stream's own sampling */
    UINT32_T codec;     /**< TKL_MEDIA_CODEC_E */
} TKL_IMAGE_INFO_T;

/** Pixel-to-pixel transform request. */
typedef struct {
    UINT16_T width;     /**< 0 = keep the input width */
    UINT16_T height;    /**< 0 = keep the input height */
    UINT32_T format;    /**< TKL_PIXEL_FMT_E of the result */
    UINT16_T rotate;    /**< 0/90/180/270 - see the note below */
    BOOL_T   mirror;    /**< horizontal mirror - see the note below */
    BOOL_T   compress;  /**< compressed output for the display path - see below */
} TKL_IMAGE_XFORM_T;

/** Encode request. */
typedef struct {
    UINT8_T  quality;   /**< JPEG quality 1..100, higher is better (libjpeg
                             scale); 0 lets the adapter choose, and the adapter
                             chooses 100 - anything lower was judged visibly
                             worse on this sensor. Size the output buffer for
                             ~4.5 bits/pixel (~1.3-1.6 MB at 2304x1296); lower
                             the value yourself if you need smaller files, 90
                             costs about a third of the bytes. */
    UINT32_T codec;     /**< TKL_MEDIA_CODEC_E; only TKL_MEDIA_CODEC_JPEG today */
    BOOL_T   prefer_sw; /**< force the software encoder; see tkl_image_encode() */
} TKL_IMAGE_ENC_PARAM_T;

/**
 * @brief Read a bitstream's header without decoding it.
 *
 * @param[in]  in   TKL_MEDIA_BUF_STREAM buffer
 * @param[out] info receives geometry and codec
 *
 * @note JPEG only; H264 needs a live decoder instance, so it returns
 *       OPRT_NOT_SUPPORTED here.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_image_probe(CONST TKL_MEDIA_BUF_T *in, TKL_IMAGE_INFO_T *info);

/**
 * @brief Decode one still image.
 *
 * @param[in]     in  TKL_MEDIA_BUF_STREAM buffer, JPEG
 * @param[in,out] out caller-owned destination: fill in @c data, @c size and the
 *                    wanted @c width / @c height / @c format before calling;
 *                    @c size is overwritten with the bytes actually produced
 *
 * @note Output format must be NV12, RGB565 or RGB888 - the decoder feeds the
 *       same post-processor that tkl_image_transform() uses. Size the buffer
 *       with the same rounding rules: the hardware stores an even row count for
 *       NV12 and a 16-row multiple for RGB, and its RGB888 is 4 bytes/pixel.
 * @note H264 returns OPRT_NOT_SUPPORTED: it is stateful, so it belongs to
 *       tkl_stream.
 * @note Asking for the stream's own geometry in NV12 decodes straight into
 *       @p out. Any other size or format needs the full-size picture first, so
 *       the adapter takes one extra @c width*height*3/2 buffer from the pixel
 *       pool for the duration of the call - budget for it, and expect
 *       OPRT_MALLOC_FAILED when the pool cannot spare it.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_image_decode(CONST TKL_MEDIA_BUF_T *in, TKL_MEDIA_BUF_T *out);

/**
 * @brief Encode one still image.
 *
 * @param[in]     in    TKL_MEDIA_BUF_PIXEL buffer, NV12 or YUYV
 * @param[in]     param quality and target codec
 * @param[in,out] out   caller-owned destination; @c size is overwritten with the
 *                      bytes actually produced
 *
 * @note JPEG only; H264 returns OPRT_NOT_SUPPORTED because it is stateful.
 * @note NV12 goes to the hardware encoder by default; setting
 *       @ref TKL_IMAGE_ENC_PARAM_T::prefer_sw converts to YUYV on the CPU and
 *       uses the software encoder instead. YUYV input always uses the software
 *       encoder; the hardware takes NV12 only.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_image_encode(CONST TKL_MEDIA_BUF_T *in,
                             CONST TKL_IMAGE_ENC_PARAM_T *param,
                             TKL_MEDIA_BUF_T *out);

/**
 * @brief Scale and/or convert one picture.
 *
 * @param[in]     in    TKL_MEDIA_BUF_PIXEL buffer, NV12
 * @param[in]     xform target geometry and format
 * @param[in,out] out   caller-owned destination; @c size is overwritten
 *
 * @warning **This part has no frame-mode rotation.** A non-zero @c rotate,
 *          @c mirror or @c compress returns OPRT_NOT_SUPPORTED rather than
 *          silently handing back an untransformed picture.
 * @note Input must be NV12. Output may be NV12, RGB565, RGB888 (hardware) or
 *       YUYV (CPU fallback, no scaling).
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_image_transform(CONST TKL_MEDIA_BUF_T *in,
                                CONST TKL_IMAGE_XFORM_T *xform,
                                TKL_MEDIA_BUF_T *out);

/**
 * @brief Bytes the hardware needs for one output picture.
 *
 * @param[in] format TKL_PIXEL_FMT_E
 * @param[in] width  output width in pixels
 * @param[in] height output height in pixels
 *
 * @note Not width*height*bpp. See tkl_image_decode() for the rounding rules.
 *
 * @return required byte count, 0 for an unsupported format or zero geometry
 */
UINT32_T tkl_image_buffer_size(UINT32_T format, UINT16_T width, UINT16_T height);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_IMAGE_H__ */
