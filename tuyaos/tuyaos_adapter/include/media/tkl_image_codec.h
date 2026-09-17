/**
 * @file tkl_image.h
 * @brief Common process - one-shot image operations
 *
 * @details
 * Memory in, memory out. Every call blocks until the hardware is done; there is
 * no handle, no stream state and no callback, so anything stateful (H264)
 * belongs to a peripheral or to tkl_stream instead.
 *
 * Three entry points, nothing else:
 *   - tkl_image_codec_get_info()  - what a bitstream is, and how big the native
 *                             decode buffer must be
 *   - tkl_image_codec_convert()   - decode / encode / scale, one call each
 *   - tkl_image_codec_memcpy()    - same-format region copy, CPU or general-DMA
 *
 * All three speak TUYA_IMAGE_CODEC_BUF_T: a buffer the caller owns before and after
 * the call. It is TUYA_MEDIA_BUF_T minus the fields that only matter while a
 * frame travels between peripherals (free_cb / timestamp / priv) - nothing
 * here is ever released or kept by TKL. See tkl_media_types.h for the
 * streaming counterpart; a camera frame is handed to these ops by copying
 * its five payload fields over.
 *
 * The caller supplies the destination buffer, so none of these transfers
 * ownership. See tkl_image_codec_convert() for how an output request is expressed.
 *
 * @version 0.3
 * @date 2026-09-10
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 */

#ifndef __TUYA_IMAGE_CODEC_H__
#define __TUYA_IMAGE_CODEC_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The rectangle tkl_image_codec_memcpy() moves: the read window on the src
 *  buffer, the paste window on the dst buffer. Read by memcpy only - the
 *  other entry points ignore it. */
typedef struct {
    UINT16_T x;          /**< window origin, pixels */
    UINT16_T y;
    UINT16_T width;      /**< rectangle size, pixels; 0 = whole frame */
    UINT16_T height;
} TUYA_IMAGE_CODEC_REGION_T;

/**
 * @brief One image buffer for the one-shot image ops.
 *
 * The caller owns the memory before and after every call - nothing here
 * travels with a release function. That is the difference from
 * TUYA_MEDIA_BUF_T, whose frames cross peripheral boundaries and carry their
 * own free_cb; to run such a frame through these ops, copy the payload
 * fields over and leave free_cb with the media buffer.
 */
typedef struct {
    UINT8_T           *buffer;     /**< pixel / bitstream base address */
    UINT32_T           length;     /**< in: capacity; out: bytes produced */
    TUYA_FRAME_FMT_E   format;   /**< content pixel layout or coded format */
    UINT16_T           width;
    UINT16_T           height;
} TUYA_IMAGE_CODEC_BUF_T;

/** One tkl_image_codec_memcpy() operand: an image buffer plus the rectangle
 *  it moves. @ref region is the read window on the src operand and the paste
 *  window on the dst operand; an all-zero region means the whole frame. */
typedef struct {
    TUYA_IMAGE_CODEC_BUF_T buf;
    TUYA_IMAGE_CODEC_REGION_T region;    /**< memcpy only: src = read window,
                                       dst = paste window */
} TUYA_IMAGE_CODEC_COPY_BUF_T;

/** The bitstream tkl_image_codec_get_info() probes: pointer and length, nothing
 *  else. A raw pixel buffer has no header to read - its geometry lives with
 *  whoever produced it - so probing only makes sense for coded streams. */
typedef struct {
    UINT8_T  *buffer;    /**< buffer base address */
    UINT32_T  length;    /**< buffer length, bytes */
} TUYA_IMAGE_BUF_T;

typedef struct {
    TUYA_FRAME_FMT_E   format;   /**< content pixel layout or coded format */
    UINT16_T           width;
    UINT16_T           height;
} TUYA_IMAGE_INFO_T;

/**
 * @brief Read a bitstream's header without decoding it.
 *
 * @param[in]  in   the stream to probe: data + length. No format field on
 *                  purpose - the probe is JPEG-only by construction, so a
 *                  caller that does not know the format just hands the bytes
 *                  over and lets it try
 * @param[out] info  filled in from the stream's own header:
 *                  - width / height: the stream's geometry
 *                  - format: the stream's chroma sampling (NV12 for YUV420,
 *                    YUYV for YUV422) - informational. Size the native decode
 *                    buffer yourself from these: width x ((height+1) & ~1)
 *                    x 3/2 as NV12, and hand it to tkl_image_codec_convert()
 *                    via out.buffer / out.length
 *
 * @note JPEG only, by construction: a raw pixel buffer has no header (its
 *       geometry is the producer's knowledge, not something this call can
 *       read back), and a stream that is not a decodable JPEG fails with
 *       OPRT_COM_ERROR.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_image_codec_get_info(CONST TUYA_IMAGE_BUF_T *in, TUYA_IMAGE_INFO_T *info);

/**
 * @brief One-shot image operation: decode, encode or scale, chosen by the
 *        formats on the two buffers.
 *
 * @param[in]     in  media buffer. format = JPEG / NV12 / YUYV
 * @param[in,out] out caller-owned destination. Fill in @c data, @c size
 *                    (capacity) and the wanted @c width / @c height / @c format
 *                    before calling; @c size is overwritten with the bytes
 *                    actually produced
 *
 * The operation is the pair (in->format, out->format):
 *
 * | in->format    | out->format              | operation            |
 * |---------------|--------------------------|----------------------|
 * | JPEG          | NV12 / YUYV / RGB565 / RGB888 | decode          |
 * | NV12 / YUYV   | JPEG                     | encode               |
 * | NV12          | NV12 / RGB565 / RGB888 / YUYV | scale / convert |
 *
 * Anything else - JPEG to JPEG included - returns OPRT_NOT_SUPPORTED. The
 * JPEG encoder always uses the best quality and picks its backend internally;
 * no quality or engine choice is exposed.
 *
 * @note Buffer sizing is the caller's job:
 *       - pixel outputs: NV12 rows round to even and RGB rows to a multiple of
 *         16, and the hardware RGB888 is 4 bytes/pixel - see the formulas in
 *         the API doc. For a native-size NV12 decode, tkl_image_codec_get_info()
 *         returns the geometry in @ref TUYA_IMAGE_INFO_T - size the buffer from
 *         it and pass that back as the capacity and no sizing formula reaches
 *         you; convert recomputes the requirement itself and checks what you
 *         hand it.
 *       - JPEG output: no exact formula; budget about 4.5 bits/pixel.
 *       An undersized buffer returns OPRT_INVALID_PARM.
 * @note Encode: geometry comes from @p in (must be even), @c out->width /
 *       @c out->height are ignored. A YUYV input is packed to the software
 *       encoder's own byte order on an internal pixel-pool copy
 *       (width*height*2) - OPRT_MALLOC_FAILED when the pool cannot spare it.
 * @note Asking for anything but the stream's own geometry in NV12 needs the
 *       full-size picture first, so the adapter takes one extra buffer from
 *       the pixel pool for the duration of the call - width*height*3/2 for
 *       4:2:0 streams, width*height*2 for 4:2:2 - expect OPRT_MALLOC_FAILED
 *       when the pool cannot spare it.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_image_codec_convert(CONST TUYA_IMAGE_CODEC_BUF_T *in, TUYA_IMAGE_CODEC_BUF_T *out);

/**
 * @brief Copy one rectangle between two buffers of the same pixel format.
 *
 * @param[in] src source frame; @c region is the read window, an all-zero
 *                region means the whole frame
 * @param[in] dst destination frame; @c region is the paste window. Its size
 *                must equal the src window's - same format means no scaling
 *
 * Same format only: a format mismatch returns OPRT_NOT_SUPPORTED, converting
 * while copying is not this call's job. Out-of-range windows are rejected
 * with OPRT_INVALID_PARM, never clipped. Overlapping windows inside one
 * buffer are handled with memmove semantics.
 *
 * The backend is picked per call: small or badly aligned copies run on the
 * CPU, larger ones on the general DMA controller (one descriptor per pixel
 * row, a single interrupt per rectangle). Either way the call blocks until
 * the bytes have landed and the result is identical.
 *
 * Window alignment (both buffers): NV12 - x, y, width and height must be
 * even (the chroma plane addresses in halves); YUYV and RGB565 - x and
 * width must be even (4-byte pixel groups); RGB888 - no constraint.
 *
 * @return OPRT_OK on success; OPRT_INVALID_PARM on NULL pointers, an empty
 *         or out-of-range or misaligned window, an undersized dst or a
 *         size mismatch between the two windows; OPRT_NOT_SUPPORTED on a
 *         format mismatch or a non-pixel format. Others on error, please
 *         refer to tuya_error_code.h
 */
OPERATE_RET tkl_image_codec_memcpy(CONST TUYA_IMAGE_CODEC_COPY_BUF_T *src, CONST TUYA_IMAGE_CODEC_COPY_BUF_T *dst);

#ifdef __cplusplus
}
#endif

#endif /* __TUYA_IMAGE_CODEC_H__ */
