/**
 * @file tkl_image.c
 * @brief TKL adapter - one-shot image operations for T9 (BK7259).
 *
 * @details
 * Every entry point is synchronous and handle-free: any vendor controller is
 * created, used and destroyed inside the call.
 *
 * Frame-mode limits on this part: the post-processor scales and converts (NV12
 * in only) but cannot rotate or mirror, and H264 is stateful so it belongs to
 * tkl_stream.
 *
 * @version 0.3
 * @date 2026-09-10
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_image_codec.h"
#include "tkl_media_buf.h"
#include "tkl_media_fmt.h"
#include "tkl_pp.h"
#include "tkl_memory.h"
#include "tkl_output.h"
#include "tkl_semaphore.h"

#include <stdint.h>
#include <os/mem.h>
#include <common/bk_err.h>
#include <common/avdk_pixel_types.h>
#include <components/avdk_utils/avdk_error.h>
#include <components/bk_decode/bk_jpeg_decode_ctlr.h>
#include <components/bk_decode/bk_jpeg_decode_types.h>
#include <components/bk_encode/bk_jpeg_encode_ctlr.h>
#include <components/bk_encode/bk_jpeg_encode_types.h>
#include <driver/dma.h>

/**
 * Default when the caller leaves quality at 0: the all-ones quantisation table.
 *
 * Size the output buffer for ~4.5 bits/pixel at this setting (~1.3-1.6 MB at
 * 2304x1296); quality 90 costs about a third of that. Encode time is almost
 * independent of the setting on both backends.
 */
#define TUYA_IMAGE_CODEC_JPEG_DEFAULT_QUALITY  100U
#define TUYA_IMAGE_CODEC_DECODE_TIMEOUT_MS     1000U

/**
 * Public quality is 1..100, higher is better, matching libjpeg.
 *
 * The two vendor encoders read their `quality` field on different scales -
 * software 1..100, hardware a 0..10 preset index - but both tables come from the
 * libjpeg formula, so software takes the public value unchanged and hardware
 * needs a lookup. The presets sit at 10,20..90 then 97 and 100, so the spacing is
 * not uniform and no divide picks the right one; the thresholds below are the
 * exact inverse of that preset list.
 */
#define TUYA_IMAGE_CODEC_QLEVEL_MAX            10U

/**
 * Payload bytes at or below which tkl_image_codec_memcpy() stays on the CPU row
 * loop; bigger copies go to the general DMA controller.
 *
 * Measured on the bench (BK7259 AP, 2026-09-11, DWT cycle profile of every
 * phase): the CPU leg runs at ~40 MB/s with no fixed cost; the DMA leg runs at
 * ~50 MB/s plus ~140 us of per-call setup (channel borrow, descriptor table,
 * interrupt arming, one interrupt round trip - the cache maintenance is a
 * no-op here because the pixel pools are not D-cacheable). Break-even is
 * therefore near 26 KB and the cutoff is placed above it, so a 64 KB copy
 * comes out ~13% ahead over DMA and larger frames more so.
 */
#define TUYA_IMAGE_CODEC_COPY_CPU_THRESHOLD    (48U * 1024U)

/*
 * Test-only backend override for tkl_image_codec_memcpy(): 0 = auto (the adapter
 * picks), 1 = force the CPU leg, 2 = force the general-DMA leg. Declared by
 * test_image.c for board bring-up and benching; production code never sets it.
 */
INT_T g_tkl_image_memcpy_force = 0;

STATIC OPERATE_RET tkl_image_codec_map_err(avdk_err_t ret)
{
    if (ret == AVDK_ERR_OK) {
        return OPRT_OK;
    }
    if (ret == AVDK_ERR_INVAL) {
        return OPRT_INVALID_PARM;
    }
    if (ret == AVDK_ERR_NOMEM) {
        return OPRT_MALLOC_FAILED;
    }
    if (ret == AVDK_ERR_UNSUPPORTED) {
        return OPRT_NOT_SUPPORTED;
    }
    if (ret == AVDK_ERR_TIMEOUT) {
        return OPRT_TIMEOUT;
    }
    return OPRT_COM_ERROR;
}

/**
 * @brief Bytes the hardware needs for one output picture.
 *
 * @note Not width*height*bpp: NV12 rows round to even, RGB rows to a multiple
 *       of 16, and the hardware RGB888 is 4 bytes/pixel. This is the module's
 *       only authoritative size rule - convert() validates the caller's
 *       destination against it, and decode_pixels() sizes its intermediates
 *       with it.
 *
 * @return required byte count, 0 for an unsupported format or zero geometry
 */
STATIC UINT32_T tkl_image_codec_need_size(TUYA_FRAME_FMT_E format, UINT16_T width, UINT16_T height)
{
    if (width == 0U || height == 0U) {
        return 0U;
    }

    if (format == (UINT32_T)TUYA_FRAME_FMT_YUYV) {
        /* packed 4:2:2, no hardware rounding: the CPU fallback writes it */
        return (UINT32_T)width * height * 2U;
    }

    return tkl_pp_output_size(tkl_media_fmt_to_bk(format), width, height);
}

/* ---------------------------------------------------------------------------
 * Probe
 * ------------------------------------------------------------------------- */

/**
 * @brief Map the decoder's chroma sampling report to a public pixel format.
 * @param[in] fmt bk_jpeg_decode_img_fmt_t
 * @return TUYA_FRAME_FMT_E
 */
STATIC UINT32_T tkl_image_codec_jpeg_sampling_to_fmt(bk_jpeg_decode_img_fmt_t fmt)
{
    /* How the *stream* is sampled, not a layout the decoder can output - this is
     * informational. 4:4:4 and grayscale have no public counterpart. */
    switch (fmt) {
    case BK_JPEG_DECODE_IMG_FMT_YUV420:
        return (UINT32_T)TUYA_FRAME_FMT_NV12;
    case BK_JPEG_DECODE_IMG_FMT_YUV422:
        return (UINT32_T)TUYA_FRAME_FMT_YUYV;
    default:
        return (UINT32_T)TUYA_FRAME_FMT_UNKNOWN;
    }
}

/*
 * The probe itself - reading the JPEG header - is shared: the public
 * tkl_image_codec_get_info() wraps it for callers, and decode_pixels() calls
 * it directly before picking the direct-decode path.
 */
STATIC OPERATE_RET tkl_image_codec_probe(CONST UINT8_T *stream, UINT32_T length,
                                   TUYA_IMAGE_INFO_T *info)
{
    bk_jpeg_decode_img_info_t img_info;
    avdk_err_t ret;

    if (stream == NULL || length == 0U || info == NULL) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&img_info, 0, sizeof(img_info));
    img_info.input_stream = stream;
    img_info.input_stream_length = length;

    ret = bk_jpeg_decode_get_img_info(&img_info);
    if (ret != AVDK_ERR_OK) {
        return tkl_image_codec_map_err(ret);
    }

    if (img_info.format == BK_JPEG_DECODE_IMG_FMT_ERR ||
        img_info.width == 0U || img_info.height == 0U) {
        return OPRT_COM_ERROR;
    }

    info->format = tkl_image_codec_jpeg_sampling_to_fmt(img_info.format);
    info->width = (UINT16_T)img_info.width;
    info->height = (UINT16_T)img_info.height;
    return OPRT_OK;
}

OPERATE_RET tkl_image_codec_get_info(CONST TUYA_IMAGE_BUF_T *in, TUYA_IMAGE_INFO_T *info)
{
    if (in == NULL) {
        return OPRT_INVALID_PARM;
    }
    return tkl_image_codec_probe(in->buffer, in->length, info);
}

/* ---------------------------------------------------------------------------
 * Decode
 * ------------------------------------------------------------------------- */

STATIC OPERATE_RET tkl_image_codec_scale_convert(CONST TUYA_IMAGE_CODEC_BUF_T *in,
                                           TUYA_IMAGE_CODEC_BUF_T *out);

/**
 * @brief One decode, straight into the caller's buffer.
 *
 * @param[in]     in  JPEG stream
 * @param[in,out] out destination; geometry, format and capacity already checked
 *
 * @note Only valid when the request matches the stream: same size, NV12 out.
 *       Anything else needs the two-step path in tkl_image_codec_decode_pixels().
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_codec_decode_one(CONST TUYA_IMAGE_CODEC_BUF_T *in, TUYA_IMAGE_CODEC_BUF_T *out)
{
    bk_jpeg_decode_frame_config_t cfg;
    bk_jpeg_decode_input_t input;
    bk_jpeg_decode_ctlr_handle_t handle = NULL;
    UINT32_T need_size;
    UINT32_T bk_fmt;
    avdk_err_t ret;

    bk_fmt = tkl_media_fmt_to_bk(out->format);
    need_size = tkl_image_codec_need_size(out->format, out->width, out->height);

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.timeout_ms = TUYA_IMAGE_CODEC_DECODE_TIMEOUT_MS;
    cfg.out_width = out->width;
    cfg.out_height = out->height;
    cfg.out_format = bk_fmt;

    os_memset(&input, 0, sizeof(input));
    input.stream = in->buffer;
    input.stream_len = in->length;
    input.out_buffer = out->buffer;
    input.out_buffer_size = out->length;

    ret = bk_jpeg_decode_frame_ctlr_new(&handle, &cfg);
    if (ret == AVDK_ERR_OK) {
        ret = bk_jpeg_decode_init(handle);
        if (ret == AVDK_ERR_OK) {
            ret = bk_jpeg_decode_open(handle);
            if (ret == AVDK_ERR_OK) {
                ret = bk_jpeg_decode_frame(handle, &input);
                (void)bk_jpeg_decode_close(handle);
            }
            (void)bk_jpeg_decode_deinit(handle);
        }
        (void)bk_jpeg_decode_delete(handle);
    }

    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_image: jpeg decode failed: %d\r\n", ret);
        return tkl_image_codec_map_err(ret);
    }

    out->length = need_size;
    return OPRT_OK;
}

/**
 * @brief Convert the hardware JPEG decoder's 4:2:2 output to NV12.
 *
 * @param[in]  width  frame width, must be even
 * @param[in]  height frame height, must be even
 * @param[in]  nv16   source, width*height*2 bytes: Y plane then @p height
 *                    interleaved Cb/Cr rows of width bytes each (NV16-shaped)
 * @param[out] nv12   destination, width*height*3/2 bytes
 *
 * @note The decoder writes 4:2:2 semi-planar - not packed YUYV, not planar
 *       YUV422. Three independent board proofs (2026-09-09/10): a buffer sized
 *       as NV12 overruns by exactly one chroma plane (width*height/2); the Y
 *       plane sits intact at offset 0 under every reinterpretation (luminance
 *       always fine); and reading the bytes as planar YUV422 yields a correct
 *       picture with grey/striped chroma - exactly what interleaved rows misread
 *       as two separate planes produce. NV12 keeps half the chroma rows; 4:2:2
 *       chroma is vertically full-rate, so each even interleaved row is copied
 *       as-is (pair order Cb-first, NV12's own) and odd rows dropped. If decoded
 *       colours ever come out red/blue swapped, the hardware pair order is
 *       Cr-first and this copy - and only this copy - needs a swap.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_codec_nv16_to_nv12(UINT16_T width,
                                          UINT16_T height,
                                          CONST UINT8_T *nv16,
                                          UINT8_T *nv12)
{
    CONST UINT32_T luma = (UINT32_T)width * height;
    CONST UINT8_T *src_uv = nv16 + luma;
    UINT8_T *dst_uv = nv12 + luma;
    UINT32_T y;

    if (nv16 == NULL || nv12 == NULL ||
        (width & 1U) != 0U || (height & 1U) != 0U) {
        return OPRT_INVALID_PARM;
    }

    os_memcpy(nv12, nv16, luma);
    for (y = 0U; y < (UINT32_T)height; y += 2U) {
        os_memcpy(dst_uv + (y >> 1U) * width, src_uv + y * width, width);
    }

    return OPRT_OK;
}

/**
 * @brief Pack the hardware JPEG decoder's 4:2:2 output as YUYV.
 *
 * @param[in]  width  frame width, must be even
 * @param[in]  height frame height, must be even
 * @param[in]  nv16   semi-planar 4:2:2 source, see tkl_image_codec_nv16_to_nv12()
 * @param[out] yuyv   packed output, width*height*2 bytes, Y0 U Y1 V per pair
 *
 * @note The public YUYV name means this packed order; the decoder's own bytes
 *       are semi-planar, so this is a real conversion, not a rename of the
 *       buffer. Full 4:2:2 chroma is kept - one interleaved row serves its own
 *       Y row, nothing is subsampled.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_codec_nv16_to_yuyv(UINT16_T width,
                                          UINT16_T height,
                                          CONST UINT8_T *nv16,
                                          UINT8_T *yuyv)
{
    CONST UINT32_T luma = (UINT32_T)width * height;
    UINT32_T y;
    UINT32_T x;

    if (nv16 == NULL || yuyv == NULL ||
        (width & 1U) != 0U || (height & 1U) != 0U) {
        return OPRT_INVALID_PARM;
    }

    for (y = 0U; y < (UINT32_T)height; y++) {
        CONST UINT8_T *y_row = nv16 + y * width;
        CONST UINT8_T *uv_row = nv16 + luma + y * width;
        UINT8_T *out_row = yuyv + y * width * 2U;

        for (x = 0U; x < (UINT32_T)width; x += 2U) {
            out_row[x * 2U + 0U] = y_row[x];
            out_row[x * 2U + 1U] = uv_row[x];       /* Cb */
            out_row[x * 2U + 2U] = y_row[x + 1U];
            out_row[x * 2U + 3U] = uv_row[x + 1U];  /* Cr */
        }
    }

    return OPRT_OK;
}

/**
 * @brief Decode a JPEG into pixel output - the JPEG leg of tkl_image_codec_convert().
 *
 * @param[in]     in  JPEG stream
 * @param[in,out] out pixel destination; width/height/format/capacity filled by
 *                    the caller, length receives the bytes actually produced
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_codec_decode_pixels(CONST TUYA_IMAGE_CODEC_BUF_T *in,
                                           TUYA_IMAGE_CODEC_BUF_T *out)
{
    TUYA_IMAGE_INFO_T info;
    TUYA_IMAGE_CODEC_BUF_T native;
    UINT8_T *tmp = NULL;
    UINT8_T *tmp_nv12 = NULL;
    UINT32_T need_size;
    UINT32_T tmp_size;
    BOOL_T yuv_src;
    OPERATE_RET ret;

    if (in == NULL || out == NULL || in->buffer == NULL || in->length == 0U ||
        out->buffer == NULL || out->length == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (in->format == (UINT32_T)TUYA_FRAME_FMT_H264) {
        return OPRT_NOT_SUPPORTED;      /* stateful - see tkl_stream */
    }

    if (out->width == 0U || out->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (out->format != (UINT32_T)TUYA_FRAME_FMT_NV12 &&
        out->format != (UINT32_T)TUYA_FRAME_FMT_RGB565 &&
        out->format != (UINT32_T)TUYA_FRAME_FMT_RGB888 &&
        out->format != (UINT32_T)TUYA_FRAME_FMT_YUYV) {
        tkl_log_output("tkl_image: decode out format %u unsupported "
                       "(NV12/RGB565/RGB888/YUYV)\r\n",
                       (unsigned int)out->format);
        return OPRT_NOT_SUPPORTED;
    }

    need_size = tkl_image_codec_need_size(out->format, out->width, out->height);
    if (need_size == 0U || out->length < need_size) {
        tkl_log_output("tkl_image: decode out buffer %u < %u needed for %ux%u fmt=%u\r\n",
                       (unsigned int)out->length, (unsigned int)need_size,
                       (unsigned int)out->width, (unsigned int)out->height,
                       (unsigned int)out->format);
        return OPRT_INVALID_PARM;
    }

    ret = tkl_image_codec_probe(in->buffer, in->length, &info);
    if (ret != OPRT_OK) {
        return ret;
    }

    /* YUV420 and a native-size NV12 request: the decoder writes the caller's
     * buffer directly - the only request it can serve on its own. A 4:2:2
     * source never qualifies: the decoder outputs its own sampling layout. */
    yuv_src = (info.format == (UINT32_T)TUYA_FRAME_FMT_YUYV) ? TRUE : FALSE;
    if (yuv_src == FALSE &&
        out->format == (UINT32_T)TUYA_FRAME_FMT_NV12 &&
        out->width == info.width && out->height == info.height) {
        return tkl_image_codec_decode_one(in, out);
    }

    /* A different size or RGB out makes the vendor decoder rebuild the whole
     * frame before post-processing, and that buffer comes from vcdec's default
     * allocator - the JPEG controllers never call vcdec_register_memalloc(),
     * unlike the H264 ones - which cannot hand out a full frame. Do the same
     * steps here instead, taking the buffer from the slab it belongs in.
     * Same peak memory, minus the broken allocation.
     *
     * The intermediate must be sized for the SOURCE's own sampling: a 4:2:2
     * stream decodes to the decoder's own semi-planar 4:2:2 layout - Y (w*h)
     * then height interleaved Cb/Cr rows of width bytes, total width*height*2
     * (see tkl_image_codec_nv16_to_nv12()); the ~153 KB overrun when sized as NV12
     * was exactly one chroma plane (board-verified). A 4:2:0 stream decodes to
     * NV12. */
    tmp_size = (yuv_src == TRUE)
                   ? tkl_image_codec_need_size((UINT32_T)TUYA_FRAME_FMT_YUYV,
                                         info.width, info.height)
                   : tkl_image_codec_need_size((UINT32_T)TUYA_FRAME_FMT_NV12,
                                         info.width, info.height);
    if (tmp_size == 0U) {
        return OPRT_COM_ERROR;
    }

    tmp = (UINT8_T *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL, tmp_size,
                                         TUYA_MEDIA_BUF_OWNER_IMAGE);
    if (tmp == NULL) {
        tkl_log_output("tkl_image: decode needs %u bytes for the %ux%u native "
                       "picture before scaling to %ux%u\r\n",
                       (unsigned int)tmp_size, info.width, info.height,
                       out->width, out->height);
        return OPRT_MALLOC_FAILED;
    }

    os_memset(&native, 0, sizeof(native));
    native.buffer = tmp;
    native.length = tmp_size;
    native.width = info.width;
    native.height = info.height;
    native.format = (yuv_src == TRUE) ? (UINT32_T)TUYA_FRAME_FMT_YUYV
                                      : (UINT32_T)TUYA_FRAME_FMT_NV12;

    ret = tkl_image_codec_decode_one(in, &native);
    if (ret != OPRT_OK) {
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_IMAGE, tmp);
        return ret;
    }

    if (yuv_src == FALSE) {
        /* 4:2:0 native NV12 in hand - straight to the scale/convert leg. */
        ret = tkl_image_codec_scale_convert(&native, out);
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_IMAGE, tmp);
        return ret;
    }

    /* 4:2:2 source: the intermediate holds the decoder's semi-planar layout.
     * A native-size NV12 request converts straight into the caller's buffer; a
     * YUYV request packs the same bytes; anything else needs an NV12
     * intermediate before the PP scales it (the PP takes NV12 only). */
    if (out->format == (UINT32_T)TUYA_FRAME_FMT_YUYV) {
        /* YUYV cannot be scaled (the PP takes NV12 only and the CPU fallback
         * does not resize) - native geometry only. */
        if (out->width != info.width || out->height != info.height) {
            tkl_log_output("tkl_image: YUYV decode output cannot be scaled\r\n");
            tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_IMAGE, tmp);
            return OPRT_NOT_SUPPORTED;
        }
        ret = tkl_image_codec_nv16_to_yuyv(info.width, info.height,
                                     native.buffer, out->buffer);
        if (ret == OPRT_OK) {
            out->length = need_size;
        }
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_IMAGE, tmp);
        return ret;
    }
    if (out->format == (UINT32_T)TUYA_FRAME_FMT_NV12 &&
        out->width == info.width && out->height == info.height) {
        ret = tkl_image_codec_nv16_to_nv12(info.width, info.height,
                                     native.buffer, out->buffer);
        if (ret == OPRT_OK) {
            out->length = need_size;
        }
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_IMAGE, tmp);
        return ret;
    }

    tmp_size = tkl_image_codec_need_size((UINT32_T)TUYA_FRAME_FMT_NV12,
                                   info.width, info.height);
    tmp_nv12 = (UINT8_T *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL, tmp_size,
                                              TUYA_MEDIA_BUF_OWNER_IMAGE);
    if (tmp_nv12 == NULL) {
        tkl_log_output("tkl_image: 4:2:2 decode needs %u bytes for the %ux%u "
                       "NV12 picture before scaling\r\n",
                       (unsigned int)tmp_size, info.width, info.height);
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_IMAGE, tmp);
        return OPRT_MALLOC_FAILED;
    }

    ret = tkl_image_codec_nv16_to_nv12(info.width, info.height,
                                 native.buffer, tmp_nv12);
    if (ret == OPRT_OK) {
        os_memset(&native, 0, sizeof(native));
        native.buffer = tmp_nv12;
        native.length = tmp_size;
        native.width = info.width;
        native.height = info.height;
        native.format = (UINT32_T)TUYA_FRAME_FMT_NV12;
        ret = tkl_image_codec_scale_convert(&native, out);
    }

    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_IMAGE, tmp_nv12);
    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_IMAGE, tmp);
    return ret;
}

/* ---------------------------------------------------------------------------
 * NV12 -> YUYV, the public packed 4:2:2 layout
 * ------------------------------------------------------------------------- */

/**
 * @brief Convert NV12 to packed YUYV.
 *
 * @param[in]  width    frame width, must be even
 * @param[in]  height   frame height, must be even
 * @param[in]  y_plane  Y plane base
 * @param[in]  uv_plane interleaved CbCr plane base
 * @param[out] yuyv     packed output, width*height*2 bytes, Y0 U Y1 V per pair
 *
 * @note Emits the standard YUYV byte order - the one thing the public name
 *       YUYV means everywhere (BK_PIXEL_FORMAT_YUYV, ffplay yuyv422). The
 *       software JPEG encoder reads a different order; that swap lives only at
 *       the encoder's door, see tkl_image_codec_yuyv_to_yvyu().
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_codec_nv12_to_yuyv(UINT16_T width,
                                          UINT16_T height,
                                          CONST UINT8_T *y_plane,
                                          CONST UINT8_T *uv_plane,
                                          UINT8_T *yuyv)
{
    UINT32_T y;
    UINT32_T y_stride = (UINT32_T)width;
    UINT32_T yuyv_stride = (UINT32_T)width * 2U;

    if (y_plane == NULL || uv_plane == NULL || yuyv == NULL ||
        (width & 1U) != 0U || (height & 1U) != 0U) {
        return OPRT_INVALID_PARM;
    }

    for (y = 0U; y < (UINT32_T)height; y += 2U) {
        CONST UINT8_T *y_row0 = y_plane + y * y_stride;
        CONST UINT8_T *y_row1 = y_row0 + y_stride;
        CONST UINT8_T *uv_row = uv_plane + (y >> 1U) * y_stride;
        UINT8_T *out_row0 = yuyv + y * yuyv_stride;
        UINT8_T *out_row1 = out_row0 + yuyv_stride;
        UINT32_T x = 0U;

        for (; x + 8U <= (UINT32_T)width; x += 8U) {
            CONST UINT8_T *uv = uv_row + x;
            UINT8_T *d0 = out_row0 + (x * 2U);
            UINT8_T *d1 = out_row1 + (x * 2U);
            UINT32_T pack0;
            UINT32_T pack1;

            pack0 = ((UINT32_T)uv[1] << 24) | ((UINT32_T)y_row0[x + 1U] << 16) |
                    ((UINT32_T)uv[0] << 8) | y_row0[x];
            pack1 = ((UINT32_T)uv[1] << 24) | ((UINT32_T)y_row1[x + 1U] << 16) |
                    ((UINT32_T)uv[0] << 8) | y_row1[x];
            *(UINT32_T *)d0 = pack0;
            *(UINT32_T *)d1 = pack1;

            pack0 = ((UINT32_T)uv[3] << 24) | ((UINT32_T)y_row0[x + 3U] << 16) |
                    ((UINT32_T)uv[2] << 8) | y_row0[x + 2U];
            pack1 = ((UINT32_T)uv[3] << 24) | ((UINT32_T)y_row1[x + 3U] << 16) |
                    ((UINT32_T)uv[2] << 8) | y_row1[x + 2U];
            *(UINT32_T *)(d0 + 4U) = pack0;
            *(UINT32_T *)(d1 + 4U) = pack1;

            pack0 = ((UINT32_T)uv[5] << 24) | ((UINT32_T)y_row0[x + 5U] << 16) |
                    ((UINT32_T)uv[4] << 8) | y_row0[x + 4U];
            pack1 = ((UINT32_T)uv[5] << 24) | ((UINT32_T)y_row1[x + 5U] << 16) |
                    ((UINT32_T)uv[4] << 8) | y_row1[x + 4U];
            *(UINT32_T *)(d0 + 8U) = pack0;
            *(UINT32_T *)(d1 + 8U) = pack1;

            pack0 = ((UINT32_T)uv[7] << 24) | ((UINT32_T)y_row0[x + 7U] << 16) |
                    ((UINT32_T)uv[6] << 8) | y_row0[x + 6U];
            pack1 = ((UINT32_T)uv[7] << 24) | ((UINT32_T)y_row1[x + 7U] << 16) |
                    ((UINT32_T)uv[6] << 8) | y_row1[x + 6U];
            *(UINT32_T *)(d0 + 12U) = pack0;
            *(UINT32_T *)(d1 + 12U) = pack1;
        }

        for (; x < (UINT32_T)width; x += 2U) {
            UINT8_T u = uv_row[x];
            UINT8_T v = uv_row[x + 1U];
            UINT32_T pack0 = ((UINT32_T)v << 24) | ((UINT32_T)y_row0[x + 1U] << 16) |
                             ((UINT32_T)u << 8) | y_row0[x];
            UINT32_T pack1 = ((UINT32_T)v << 24) | ((UINT32_T)y_row1[x + 1U] << 16) |
                             ((UINT32_T)u << 8) | y_row1[x];

            *(UINT32_T *)(out_row0 + (x * 2U)) = pack0;
            *(UINT32_T *)(out_row1 + (x * 2U)) = pack1;
        }
    }

    return OPRT_OK;
}

/**
 * @brief Swap a packed YUYV picture's chroma bytes into the encoder's order.
 *
 * @param[in]  width  frame width, must be even
 * @param[in]  height frame height, must be even
 * @param[in]  yuyv   standard packed source, Y0 U Y1 V per pair
 * @param[out] yvyu   destination, width*height*2 bytes, Y0 V Y1 U per pair
 *
 * @note The software JPEG encoder reads V at byte +1 - verified on board
 *       2026-09-10: its JPEG comes out with correct colours from a YVYU feed.
 *       The public YUYV stays standard everywhere else, so this swap exists
 *       only at the encoder's door and runs on a copy; the caller's buffer is
 *       never modified.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_codec_yuyv_to_yvyu(UINT16_T width,
                                          UINT16_T height,
                                          CONST UINT8_T *yuyv,
                                          UINT8_T *yvyu)
{
    UINT32_T pairs;
    UINT32_T i;

    if (yuyv == NULL || yvyu == NULL ||
        (width & 1U) != 0U || (height & 1U) != 0U) {
        return OPRT_INVALID_PARM;
    }

    pairs = (UINT32_T)width * height / 2U;
    for (i = 0U; i < pairs; i++) {
        CONST UINT8_T *src = yuyv + i * 4U;
        UINT8_T *dst = yvyu + i * 4U;

        dst[0] = src[0];
        dst[1] = src[3];    /* V */
        dst[2] = src[2];
        dst[3] = src[1];    /* U */
    }

    return OPRT_OK;
}

/* ---------------------------------------------------------------------------
 * Encode
 * ------------------------------------------------------------------------- */

/**
 * @brief JPEG encoder output-completion hook; records the produced length.
 * @param[in] info vendor completion info, args points at a UINT32_T
 * @return BK_OK
 */
STATIC uint32_t tkl_image_codec_jpeg_outbuf_complete(bk_jpeg_encode_outbuf_info_t *info)
{
    if (info != NULL && info->args != NULL && info->status == BK_OK) {
        *(UINT32_T *)info->args = info->length;
    }

    return (uint32_t)BK_OK;
}

/**
 * @brief Map public quality 1..100 onto a vcenc preset index 0..10.
 * @param[in] quality public quality, 1..100
 * @return qLevel whose built-in table is closest to what libjpeg would produce
 */
STATIC UINT8_T tkl_image_codec_quality_to_qlevel(UINT8_T quality)
{
    /* k_min_q[i] is the lowest public quality that selects qLevel i+1 */
    STATIC CONST UINT8_T k_min_q[TUYA_IMAGE_CODEC_QLEVEL_MAX] = {
        15U, 24U, 34U, 44U, 55U, 65U, 75U, 85U, 94U, 98U
    };
    UINT8_T level = 0U;

    while (level < TUYA_IMAGE_CODEC_QLEVEL_MAX && quality >= k_min_q[level]) {
        level++;
    }

    return level;
}

/**
 * @brief Run one JPEG encode through a throwaway controller.
 *
 * @param[in]  use_sw  TRUE selects the software encoder, FALSE the NV12
 *                     hardware encoder
 * @param[in]  width   picture width
 * @param[in]  height  picture height
 * @param[in]  src     input base in the backend's own byte order: YVYU-packed
 *                     when @p use_sw (see tkl_image_codec_yuyv_to_yvyu()), NV12
 *                     luma otherwise
 * @param[in]  quality public quality, 1..100; converted per backend here, the
 *                     two encoders read the field on different scales
 * @param[out] out     destination buffer; @c length receives the encoded length
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_codec_jpeg_encode_run(BOOL_T use_sw,
                                             UINT16_T width,
                                             UINT16_T height,
                                             CONST UINT8_T *src,
                                             UINT8_T quality,
                                             TUYA_IMAGE_CODEC_BUF_T *out)
{
    bk_jpeg_encode_ctlr_handle_t handle = NULL;
    bk_jpeg_encode_input_t input;
    UINT32_T encoded = 0U;
    avdk_err_t ret;

    os_memset(&input, 0, sizeof(input));
    input.pic_buf = (UINT32_T)(uintptr_t)src;
    input.out_buf = (UINT32_T)(uintptr_t)out->buffer;
    input.out_size = out->length;

    if (use_sw == TRUE) {
        bk_jpeg_encode_sw_frame_config_t cfg;

        os_memset(&cfg, 0, sizeof(cfg));
        cfg.width = width;
        cfg.height = height;
        cfg.input_format = (UINT32_T)BK_PIXEL_FORMAT_YUYV;
        /* tjpg takes the public 1..100 unchanged */
        cfg.quality = quality;
        cfg.outbuf_complete = tkl_image_codec_jpeg_outbuf_complete;
        cfg.outbuf_complete_args = &encoded;

        ret = bk_jpeg_encode_sw_frame_new(&handle, &cfg);
    } else {
        bk_jpeg_encode_frame_config_t cfg;

        os_memset(&cfg, 0, sizeof(cfg));
        cfg.width = width;
        cfg.height = height;
        cfg.input_format = (UINT32_T)BK_PIXEL_FORMAT_NV12;
        cfg.input_buf = (UINT32_T)(uintptr_t)src;
        /* vcenc indexes one of eleven built-in tables */
        cfg.quality = tkl_image_codec_quality_to_qlevel(quality);
        cfg.outbuf_complete = tkl_image_codec_jpeg_outbuf_complete;
        cfg.outbuf_complete_args = &encoded;

        ret = bk_jpeg_encode_frame_new(&handle, &cfg);
    }

    if (ret == AVDK_ERR_OK) {
        ret = bk_jpeg_encode_init(handle);
        if (ret == AVDK_ERR_OK) {
            ret = bk_jpeg_encode_open(handle);
            if (ret == AVDK_ERR_OK) {
                ret = bk_jpeg_encode_frame(handle, &input);
                (void)bk_jpeg_encode_close(handle);
            }
            (void)bk_jpeg_encode_deinit(handle);
        }
        (void)bk_jpeg_encode_delete(handle);
    }

    if (ret != AVDK_ERR_OK || encoded == 0U) {
        tkl_log_output("tkl_image: jpeg %s encode failed ret=%d size=%u\r\n",
                       (use_sw == TRUE) ? "sw" : "hw", (INT_T)ret, (unsigned int)encoded);
        out->length = 0U;
        return (ret != AVDK_ERR_OK) ? tkl_image_codec_map_err(ret) : OPRT_COM_ERROR;
    }

    out->length = encoded;
    return OPRT_OK;
}

/**
 * @brief Encode a pixel picture to JPEG - the JPEG leg of tkl_image_codec_convert().
 *
 * @param[in]     in  NV12 or YUYV picture, geometry from the buffer fields
 * @param[in,out] out JPEG destination; capacity in @c size, which receives the
 *                    encoded length
 *
 * @note Always the best quality, backend chosen from the input format: NV12 to
 *       the hardware encoder, YUYV to the software one - through an internal
 *       encoder-order copy from the pixel pool, since the public YUYV is
 *       standard packed order. No quality or engine choice is exposed on
 *       purpose.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_codec_encode_jpeg(CONST TUYA_IMAGE_CODEC_BUF_T *in,
                                         TUYA_IMAGE_CODEC_BUF_T *out)
{
    UINT32_T y_plane_size;
    UINT8_T *packed;
    UINT8_T quality;
    OPERATE_RET ret;

    if (in == NULL || out == NULL || in->buffer == NULL || in->length == 0U ||
        out->buffer == NULL || out->length == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (in->width == 0U || in->height == 0U ||
        (in->width & 1U) != 0U || (in->height & 1U) != 0U) {
        return OPRT_INVALID_PARM;
    }

    quality = (UINT8_T)TUYA_IMAGE_CODEC_JPEG_DEFAULT_QUALITY;
    y_plane_size = (UINT32_T)in->width * in->height;

    if (in->format == (UINT32_T)TUYA_FRAME_FMT_YUYV) {
        if (in->length < (y_plane_size * 2U)) {
            return OPRT_INVALID_PARM;
        }
        /* The software encoder is the only backend that takes YUYV, and it
         * reads YVYU byte order - hand it a swapped copy, the caller's buffer
         * stays untouched. */
        packed = (UINT8_T *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL,
                                                y_plane_size * 2U,
                                                TUYA_MEDIA_BUF_OWNER_IMAGE);
        if (packed == NULL) {
            tkl_log_output("tkl_image: yuyv encode needs %u bytes from the "
                           "pixel pool for the encoder-order copy\r\n",
                           (unsigned int)(y_plane_size * 2U));
            return OPRT_MALLOC_FAILED;
        }
        ret = tkl_image_codec_yuyv_to_yvyu(in->width, in->height, in->buffer, packed);
        if (ret == OPRT_OK) {
            ret = tkl_image_codec_jpeg_encode_run(TRUE, in->width, in->height,
                                            packed, quality, out);
        }
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_IMAGE, packed);
        return ret;
    }

    if (in->format != (UINT32_T)TUYA_FRAME_FMT_NV12) {
        return OPRT_NOT_SUPPORTED;
    }

    if (in->length < (y_plane_size + (y_plane_size / 2U))) {
        return OPRT_INVALID_PARM;
    }

    return tkl_image_codec_jpeg_encode_run(FALSE, in->width, in->height,
                                     in->buffer, quality, out);
}

/* ---------------------------------------------------------------------------
 * Transform
 * ------------------------------------------------------------------------- */

/**
 * @brief Scale and/or convert one NV12 picture - the pixel-to-pixel leg of
 *        tkl_image_codec_convert(). The target geometry and format are read from the
 *        caller's @p out fields.
 *
 * @param[in]     in  NV12 picture - the only input format this part has
 * @param[in,out] out destination; width/height/format/capacity filled by the
 *                    caller, size and strides written back
 *
 * @note Output may be NV12 / RGB565 / RGB888 (hardware PP) or YUYV (CPU
 *       fallback, same size only).
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_codec_scale_convert(CONST TUYA_IMAGE_CODEC_BUF_T *in,
                                           TUYA_IMAGE_CODEC_BUF_T *out)
{
    TKL_PP_HANDLE pp = NULL;
    TKL_PP_IN_T pp_in;
    TKL_PP_OUT_T pp_out;
    UINT32_T y_plane_size;
    UINT32_T need_size;
    OPERATE_RET ret;

    if (in == NULL || out == NULL ||
        in->buffer == NULL || in->length == 0U ||
        out->buffer == NULL || out->length == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (in->format != (UINT32_T)TUYA_FRAME_FMT_NV12) {
        tkl_log_output("tkl_image: pixel input must be NV12 (got %u)\r\n",
                       (unsigned int)in->format);
        return OPRT_NOT_SUPPORTED;
    }

    if (in->width == 0U || in->height == 0U ||
        out->width == 0U || out->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    y_plane_size = (UINT32_T)in->width * in->height;
    if (in->length < (y_plane_size + (y_plane_size / 2U))) {
        return OPRT_INVALID_PARM;
    }

    need_size = tkl_image_codec_need_size(out->format, out->width, out->height);
    if (need_size == 0U) {
        tkl_log_output("tkl_image: output format %u unsupported\r\n",
                       (unsigned int)out->format);
        return OPRT_NOT_SUPPORTED;
    }
    if (out->length < need_size) {
        tkl_log_output("tkl_image: out buffer %u < %u needed\r\n",
                       (unsigned int)out->length, (unsigned int)need_size);
        return OPRT_INVALID_PARM;
    }

    if (out->format == (UINT32_T)TUYA_FRAME_FMT_YUYV) {
        /* the PP has no YUYV output; the CPU fallback cannot scale either */
        if (out->width != in->width || out->height != in->height) {
            tkl_log_output("tkl_image: YUYV output cannot be scaled\r\n");
            return OPRT_NOT_SUPPORTED;
        }

        ret = tkl_image_codec_nv12_to_yuyv(in->width, in->height,
                                     in->buffer, in->buffer + y_plane_size, out->buffer);
        if (ret != OPRT_OK) {
            return ret;
        }
    } else {
        ret = tkl_pp_open(0U, &pp);
        if (ret != OPRT_OK) {
            return ret;
        }

        os_memset(&pp_in, 0, sizeof(pp_in));
        pp_in.y = in->buffer;
        pp_in.uv = in->buffer + y_plane_size;
        pp_in.width = in->width;
        pp_in.height = in->height;

        os_memset(&pp_out, 0, sizeof(pp_out));
        pp_out.buffer = out->buffer;
        pp_out.size = out->length;
        pp_out.width = out->width;
        pp_out.height = out->height;
        pp_out.format = tkl_media_fmt_to_bk(out->format);

        ret = tkl_pp_process(pp, &pp_in, &pp_out);
        (void)tkl_pp_close(pp);
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    out->length = need_size;
    return OPRT_OK;
}

/* ---------------------------------------------------------------------------
 * Convert - the one public operation. The direction is the format pair on
 * the input and output buffers; see tkl_image.h for the table.
 * ------------------------------------------------------------------------- */

OPERATE_RET tkl_image_codec_convert(CONST TUYA_IMAGE_CODEC_BUF_T *in, TUYA_IMAGE_CODEC_BUF_T *out)
{
    if (in == NULL || out == NULL || in->buffer == NULL || in->length == 0U ||
        out->buffer == NULL || out->length == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (in->format == (UINT32_T)TUYA_FRAME_FMT_H264) {
        return OPRT_NOT_SUPPORTED;      /* stateful - see tkl_stream */
    }

    if (in->format == (UINT32_T)TUYA_FRAME_FMT_NV12 ||
        in->format == (UINT32_T)TUYA_FRAME_FMT_YUYV) {
        if (out->format == (UINT32_T)TUYA_FRAME_FMT_JPEG) {
            return tkl_image_codec_encode_jpeg(in, out);
        }
        /* YUYV-to-pixel is refused inside scale_convert() */
        return tkl_image_codec_scale_convert(in, out);
    }

    /* Anything else (JPEG or UNKNOWN) is treated as a JPEG bitstream. */
    if (out->format == (UINT32_T)TUYA_FRAME_FMT_JPEG) {
        /* JPEG to JPEG would be a transcode - not a one-shot operation here */
        return OPRT_NOT_SUPPORTED;
    }
    return tkl_image_codec_decode_pixels(in, out);
}

/* ---------------------------------------------------------------------------
 * Memcpy - same-format region copy. The backend (CPU row loop or the general
 * DMA controller) is picked inside; tkl_image.h holds the window contract.
 * ------------------------------------------------------------------------- */

/* One plane of a resolved copy: the window's first row and its shape. */
typedef struct {
    UINT8_T  *base;       /**< first row of the window */
    UINT32_T  pitch;      /**< row pitch, bytes */
    UINT32_T  row_bytes;  /**< window payload per row, bytes */
    UINT32_T  rows;       /**< window height in plane rows */
} TUYA_IMAGE_CODEC_COPY_PLANE_T;

typedef struct {
    TUYA_IMAGE_CODEC_COPY_PLANE_T y;
    TUYA_IMAGE_CODEC_COPY_PLANE_T uv;    /**< rows == 0 when there is no chroma plane */
} TUYA_IMAGE_CODEC_COPY_VIEW_T;

/**
 * Validate one memcpy operand and resolve its window into plane views.
 *
 * @param[in]    f      the operand's image buffer
 * @param[in]    region the window to move on @p f; an all-zero region means
 *                      the whole frame
 * @param[out]   v      resolved plane views
 *
 * OPRT_NOT_SUPPORTED for anything but the pixel formats; OPRT_INVALID_PARM
 * for a NULL/empty payload, a half-specified or out-of-range window (never
 * clipped), a window misaligned for the format, a stride smaller than the
 * row payload (rows would overlap) and a buffer smaller than its own
 * geometry. Everything else fills @p v and returns OPRT_OK.
 */
STATIC OPERATE_RET tkl_image_codec_copy_view(CONST TUYA_IMAGE_CODEC_BUF_T *f,
                                       CONST TUYA_IMAGE_CODEC_REGION_T *region,
                                       TUYA_IMAGE_CODEC_COPY_VIEW_T *v)
{
    UINT32_T y_bpp;
    UINT32_T ys;
    UINT32_T uvs;
    UINT32_T rx;
    UINT32_T ry;
    UINT32_T rw;
    UINT32_T rh;
    UINT32_T need;

    if (f->format == (UINT32_T)TUYA_FRAME_FMT_NV12) {
        y_bpp = 1U;
    } else if (f->format == (UINT32_T)TUYA_FRAME_FMT_YUYV ||
               f->format == (UINT32_T)TUYA_FRAME_FMT_RGB565) {
        y_bpp = 2U;
    } else if (f->format == (UINT32_T)TUYA_FRAME_FMT_RGB888) {
        y_bpp = 4U;
    } else {
        return OPRT_NOT_SUPPORTED;      /* coded formats are not memcpy-able */
    }

    if (f->buffer == NULL || f->width == 0U || f->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    /* an all-zero region is the whole frame; a half-zero one is a mistake */
    if (region->width == 0U || region->height == 0U) {
        if (region->width != 0U || region->height != 0U ||
            region->x != 0U || region->y != 0U) {
            return OPRT_INVALID_PARM;
        }
        rx = 0U;
        ry = 0U;
        rw = f->width;
        rh = f->height;
    } else {
        rx = region->x;
        ry = region->y;
        rw = region->width;
        rh = region->height;
        if (rx + rw > (UINT32_T)f->width || ry + rh > (UINT32_T)f->height) {
            return OPRT_INVALID_PARM;
        }
    }

    /* the alignment contract, per format family (tkl_image.h) */
    if (f->format == (UINT32_T)TUYA_FRAME_FMT_NV12) {
        if (((rx | ry | rw | rh) & 1U) != 0U) {
            return OPRT_INVALID_PARM;   /* the chroma plane steps in halves */
        }
    } else if (f->format != (UINT32_T)TUYA_FRAME_FMT_RGB888 &&
               ((rx | rw) & 1U) != 0U) {
        return OPRT_INVALID_PARM;       /* 4-byte pixel groups */
    }

    /* tightly packed by contract: the row pitch is the buffer's own width */
    ys = (UINT32_T)f->width * y_bpp;
    need = ys * (UINT32_T)f->height;

    v->uv.rows = 0U;
    if (f->format == (UINT32_T)TUYA_FRAME_FMT_NV12) {
        /* the chroma row is rw/2 interleaved Cb/Cr pairs = rw bytes */
        uvs = (UINT32_T)f->width;
        need += uvs * (UINT32_T)(f->height / 2U);
        v->uv.base = f->buffer + ys * (UINT32_T)f->height +
                     (ry / 2U) * uvs + rx;
        v->uv.pitch = uvs;
        v->uv.row_bytes = rw;
        v->uv.rows = rh / 2U;
    }

    /* the buffer must hold its own geometry before any window is discussed */
    if (need > f->length) {
        return OPRT_INVALID_PARM;
    }

    v->y.base = f->buffer + ry * ys + rx * y_bpp;
    v->y.pitch = ys;
    v->y.row_bytes = rw * y_bpp;
    v->y.rows = rh;
    return OPRT_OK;
}

/*
 * Move one plane row by row. os_memmove keeps a window that overlaps itself
 * inside one buffer correct within the row; the caller picks the row order
 * that keeps it correct across rows.
 */
STATIC VOID_T tkl_image_codec_copy_plane(CONST TUYA_IMAGE_CODEC_COPY_PLANE_T *sp,
                                   CONST TUYA_IMAGE_CODEC_COPY_PLANE_T *dp,
                                   BOOL_T descend)
{
    UINT32_T i;
    UINT32_T r;

    for (i = 0U; i < sp->rows; i++) {
        r = (descend == TRUE) ? (sp->rows - 1U - i) : i;
        os_memmove(dp->base + r * dp->pitch, sp->base + r * sp->pitch,
                   sp->row_bytes);
    }
}

#if CONFIG_GENERAL_DMA_LINK_MODE

/*
 * The general-DMA leg. One descriptor per pixel row, one completion interrupt
 * per rectangle: only the last descriptor in the chain enables int_finish_en,
 * so the whole copy is a single wait.
 *
 * The vendor coherency helpers (ap/middleware/arch/cortex-m/include/cache.h)
 * are declared here rather than #included - that tree's include directory is
 * not this file's to reach for, and the two signatures are stable vendor ABI.
 * They are the direction-correct entry points for handing a buffer to / taking
 * it back from a DMA master: clean-only before, invalidate-only after, and
 * both are no-ops on non-cacheable memory (the uncoded slabs most of this
 * module's frames live in).
 */
extern void bk_dcache_clean_for_producer(void *addr, size_t len);
extern void bk_dcache_invalidate_for_consumer(void *addr, size_t len);

#define TUYA_IMAGE_CODEC_COPY_DMA_TIMEOUT_MS   500U

/* Module-lifetime completion semaphore: created on the first DMA copy, never
 * released - there is no init-time hook in a one-shot module. The media path
 * calls this from a single producer thread, which is also what keeps two
 * callers from racing on the shared channel and semaphore. */
STATIC TKL_SEM_HANDLE s_tkl_image_copy_sem = NULL;

/*
 * Set when a round times out: that round may still post its completion after
 * the caller has given up, so the next round has to absorb it before arming
 * its own. The success path consumes the single post it waits for and leaves
 * nothing behind, so it skips the drain entirely.
 *
 * That skip is what makes this cheap. A 0-timeout tkl_semaphore_wait() on an
 * EMPTY semaphore does not return at once: it burns up to a whole RTOS tick
 * waiting for the timeout to expire. Draining unconditionally therefore cost a
 * fraction of a tick on every single copy - measured at 108..596 us per call
 * on the BK7259 AP, i.e. the dominant part of this leg's per-call overhead.
 */
STATIC BOOL_T s_tkl_image_copy_stale = FALSE;

STATIC VOID_T tkl_image_codec_copy_dma_isr(dma_id_t dma_id)
{
    (void)dma_id;
    tkl_semaphore_post(s_tkl_image_copy_sem);
}

/*
 * On-chip SRAM has two aliases on this part: the CPU sees it at 0x2Cxxxxxx
 * (0x3C for the non-secure view), the DMA engine only at 0x28xxxxxx (0x38) -
 * one bit of difference. Everything the vendor driver programs on its own goes
 * through SOC_SRAM_PERI_ADDR() and is fine, but the addresses the engine reads
 * back out of OUR descriptors are ours to convert:
 *
 *   - an unconverted chain entry is something the engine cannot fetch, so the
 *     chain stalls after one node (dma_en stuck at 1, no finish interrupt);
 *   - an unconverted data address stalls the transfer the same way.
 *
 * Doing it here rather than in the driver keeps this module self-contained, and
 * it is idempotent and range-guarded, so it is harmless if the driver converts
 * too, and passes PSRAM/QSPI/register addresses through untouched. The contract
 * is documented in the vendor tree's hpdma.h; the macro is spelled out locally
 * because that header's include path is not this file's to reach for, same
 * pattern as the cache helpers above.
 */
#define TUYA_IMAGE_CODEC_DMA_SRAM_ALIAS_BIT     0x04000000U
#define TUYA_IMAGE_CODEC_DMA_SRAM_ALIAS_MASK    0xFF000000U
#define TUYA_IMAGE_CODEC_DMA_SRAM_ALIAS_BASE    0x2C000000U
#define TUYA_IMAGE_CODEC_DMA_SRAM_ALIAS_BASE_NS 0x3C000000U

STATIC UINT32_T tkl_image_codec_dma_engine_addr(UINT32_T addr)
{
    UINT32_T top = addr & TUYA_IMAGE_CODEC_DMA_SRAM_ALIAS_MASK;

    if (top == TUYA_IMAGE_CODEC_DMA_SRAM_ALIAS_BASE ||
        top == TUYA_IMAGE_CODEC_DMA_SRAM_ALIAS_BASE_NS) {
        return addr & ~TUYA_IMAGE_CODEC_DMA_SRAM_ALIAS_BIT;
    }
    return addr;
}

/*
 * Row cursor for the copy: the two planes walked with running addresses.
 * Luma rows first, then chroma rows. Advancing is an add of the pitch, so no
 * row index is ever multiplied by a pitch, the luma/chroma handover costs one
 * branch per plane instead of one per row, and the merge predicate reads the
 * same two numbers either way.
 *
 * The run count and the descriptor fill both walk through this one cursor,
 * which is what keeps the two passes from disagreeing about where a run breaks.
 */
typedef struct {
    CONST TUYA_IMAGE_CODEC_COPY_PLANE_T *sp;      /**< plane being walked */
    CONST TUYA_IMAGE_CODEC_COPY_PLANE_T *dp;
    UINT32_T                      next_s;  /**< address of the row not yet handed out */
    UINT32_T                      next_d;
    UINT32_T                      left;    /**< rows left in this plane */
    UINT32_T                      s;       /**< address of the row handed out last */
    UINT32_T                      d;
    UINT32_T                      bytes;   /**< payload of the row handed out last */
} TUYA_IMAGE_CODEC_COPY_CURSOR_T;

STATIC VOID_T tkl_image_codec_copy_cursor_init(CONST TUYA_IMAGE_CODEC_COPY_VIEW_T *sv,
                                         CONST TUYA_IMAGE_CODEC_COPY_VIEW_T *dv,
                                         TUYA_IMAGE_CODEC_COPY_CURSOR_T *c)
{
    c->sp = &sv->y;
    c->dp = &dv->y;
    c->next_s = (UINT32_T)sv->y.base;
    c->next_d = (UINT32_T)dv->y.base;
    c->left = sv->y.rows;
    c->s = 0U;
    c->d = 0U;
    c->bytes = 0U;
}

/*
 * Hand out the next row of the copy. FALSE once both planes are exhausted, and
 * then s/d/bytes carry nothing meaningful. Rows with a zero payload are handed
 * out like any other - the geometry above has already rejected the degenerate
 * cases, and a zero-length descriptor would be the engine's problem, not ours.
 */
STATIC BOOL_T tkl_image_codec_copy_cursor_next(CONST TUYA_IMAGE_CODEC_COPY_VIEW_T *sv,
                                         CONST TUYA_IMAGE_CODEC_COPY_VIEW_T *dv,
                                         TUYA_IMAGE_CODEC_COPY_CURSOR_T *c)
{
    if (c->left == 0U) {
        /* luma drained - hand over to chroma, at most once */
        if (c->sp == &sv->uv || sv->uv.rows == 0U) {
            return FALSE;
        }
        c->sp = &sv->uv;
        c->dp = &dv->uv;
        c->next_s = (UINT32_T)sv->uv.base;
        c->next_d = (UINT32_T)dv->uv.base;
        c->left = sv->uv.rows;
    }

    c->s = c->next_s;
    c->d = c->next_d;
    c->bytes = c->sp->row_bytes;
    c->next_s += c->sp->pitch;
    c->next_d += c->dp->pitch;
    c->left--;

    return TRUE;
}

/*
 * Descriptor count for the transfer: consecutive rows collapse into one
 * descriptor whenever both the source and the destination continue exactly
 * where the previous row ended. A tightly packed plane (pitch == row payload,
 * the camera and encoder case) therefore costs ONE descriptor instead of one
 * per row - identical bytes over the bus, but a chain the engine walks in one
 * step rather than 96..720. Strided frames keep one descriptor per row, which
 * is what their per-row geometry genuinely requires.
 *
 * The length a merged run can carry is bounded by the 30-bit descriptor length
 * field (1 GiB), far above any single plane, so runs are never split on size.
 * Never returns 0: a copy always has at least one row.
 */
STATIC UINT32_T tkl_image_codec_copy_run_count(CONST TUYA_IMAGE_CODEC_COPY_VIEW_T *sv,
                                         CONST TUYA_IMAGE_CODEC_COPY_VIEW_T *dv)
{
    TUYA_IMAGE_CODEC_COPY_CURSOR_T cur;
    UINT32_T runs = 0U;
    UINT32_T s_end = 0U;
    UINT32_T d_end = 0U;

    tkl_image_codec_copy_cursor_init(sv, dv, &cur);
    while (tkl_image_codec_copy_cursor_next(sv, dv, &cur) == TRUE) {
        if (runs == 0U || cur.s != s_end || cur.d != d_end) {
            runs++;
        }
        s_end = cur.s + cur.bytes;
        d_end = cur.d + cur.bytes;
    }
    return (runs != 0U) ? runs : 1U;
}

/*
 * The DMA moves whole rows only, so every row start it is given must be
 * 16-byte aligned (the linked-list descriptor contract) - a 16-byte-aligned
 * base plus a 16-byte-aligned pitch delivers that for all rows at once - and
 * the row payload must be a whole number of 32-bit bus words so nothing is
 * left in the FIFO. Overlap is excluded by the caller: DMA has no
 * within-window move semantics.
 */
STATIC BOOL_T tkl_image_codec_copy_dma_fits(CONST TUYA_IMAGE_CODEC_COPY_VIEW_T *sv,
                                      CONST TUYA_IMAGE_CODEC_COPY_VIEW_T *dv)
{
    UINT32_T i;

    for (i = 0U; i < 2U; i++) {
        CONST TUYA_IMAGE_CODEC_COPY_PLANE_T *sp = (i == 0U) ? &sv->y : &sv->uv;
        CONST TUYA_IMAGE_CODEC_COPY_PLANE_T *dp = (i == 0U) ? &dv->y : &dv->uv;

        if (sp->rows == 0U) {
            continue;
        }
        if (((UINT32_T)sp->base & 0xFU) != 0U || (sp->pitch & 0xFU) != 0U ||
            ((UINT32_T)dp->base & 0xFU) != 0U || (dp->pitch & 0xFU) != 0U ||
            (sp->row_bytes & 3U) != 0U) {
            return FALSE;
        }
    }
    return TRUE;
}

STATIC OPERATE_RET tkl_image_codec_copy_dma(CONST TUYA_IMAGE_CODEC_COPY_VIEW_T *sv,
                                      CONST TUYA_IMAGE_CODEC_COPY_VIEW_T *dv)
{
    dma_link_config_t cfg;
    VOID_T *table = NULL;
    UINT32_T rows = 0U;
    UINT32_T runs = 0U;
    UINT32_T desc;
    UINT32_T s_end;
    UINT32_T d_end;
    UINT32_T run_src = 0U;
    UINT32_T run_dst = 0U;
    UINT32_T k;
    TUYA_IMAGE_CODEC_COPY_CURSOR_T cur;
    dma_id_t id = DMA_ID_MAX;
    OPERATE_RET ret = OPRT_COM_ERROR;

    if (tkl_image_codec_copy_dma_fits(sv, dv) != TRUE) {
        return OPRT_NOT_SUPPORTED;
    }
    rows = sv->y.rows + sv->uv.rows;
    if (rows == 0U) {
        /* Nothing to hand the engine. Refuse here rather than build a
         * zero-entry chain - bk_dma_link_init(0) walks its link loop with an
         * unsigned (0 - 1) bound and would run away over the heap. */
        return OPRT_INVALID_PARM;
    }
    if (bk_dma_driver_init() != BK_OK) {
        return OPRT_COM_ERROR;
    }

    if (s_tkl_image_copy_sem == NULL &&
        tkl_semaphore_create_init(&s_tkl_image_copy_sem, 0, 1) != OPRT_OK) {
        return OPRT_COM_ERROR;
    }

    id = bk_dma_alloc((UINT16_T)DMA_DEV_DTCM);
    if (id >= DMA_ID_MAX) {
        return OPRT_COM_ERROR;          /* no free channel */
    }


    if (bk_dma_register_isr(id, NULL, tkl_image_codec_copy_dma_isr) == BK_OK &&
        bk_dma_enable_finish_interrupt(id) == BK_OK) {
        /* Absorb a late completion post from an earlier timed-out round; the
         * channel is not started here yet, so nothing can post after this.
         * Only a timed-out round can leave a post behind, so the normal path
         * skips this - see s_tkl_image_copy_stale. */
        if (s_tkl_image_copy_stale == TRUE) {
            while (tkl_semaphore_wait(s_tkl_image_copy_sem, 0) == OPRT_OK) {
            }
            s_tkl_image_copy_stale = FALSE;
        }

        runs = tkl_image_codec_copy_run_count(sv, dv);

        table = bk_dma_link_init(runs);
        if (table != NULL) {
            /* bk_dma_link_init() chains the entries with CPU-side pointers,
             * which the engine cannot fetch - rewrite the chain in
             * engine-visible addresses. The last entry keeps the 0 terminator
             * the driver put there, and a single-run table has nothing to
             * rewrite. See tkl_image_codec_dma_engine_addr(). */
            dma_descriptor_t *d = (dma_descriptor_t *)table;

            for (k = 0U; k + 1U < runs; k++) {
                d[k].next_desc_addr =
                    tkl_image_codec_dma_engine_addr((UINT32_T)&d[k + 1U]);
            }
        }
    }
    if (table == NULL) {
        goto exit;
    }

    /* One descriptor per contiguous run - see tkl_image_codec_copy_run_count(). A
     * tightly packed plane lands as a single entry, a strided one falls back
     * to one entry per row. Only the last descriptor raises the finish
     * interrupt, so the whole rectangle is still a single wait. */
    tkl_image_codec_copy_cursor_init(sv, dv, &cur);
    s_end = 0U;
    d_end = 0U;
    desc = 0U;
    while (tkl_image_codec_copy_cursor_next(sv, dv, &cur) == TRUE) {
        if (desc != 0U && (cur.s != s_end || cur.d != d_end)) {
            /* the run before this row is closed, and it is not the last one */
            os_memset(&cfg, 0, sizeof(cfg));
            cfg.src_addr = tkl_image_codec_dma_engine_addr(run_src);
            cfg.dst_addr = tkl_image_codec_dma_engine_addr(run_dst);
            cfg.ctrl.bits.length = s_end - run_src;
            cfg.ctrl.bits.int_finish_en = 0U;
            if (bk_dma_link_set_desc(table, desc - 1U, &cfg) != BK_OK) {
                goto exit;
            }
        }
        if (desc == 0U || cur.s != s_end || cur.d != d_end) {
            run_src = cur.s;
            run_dst = cur.d;
            desc++;
        }
        s_end = cur.s + cur.bytes;
        d_end = cur.d + cur.bytes;
    }

    /* The final run carries the completion interrupt. desc and runs come from
     * the same predicate, so a mismatch would mean the descriptor table is
     * short - refuse the transfer rather than hand the engine a bad chain. */
    os_memset(&cfg, 0, sizeof(cfg));
    cfg.src_addr = tkl_image_codec_dma_engine_addr(run_src);
    cfg.dst_addr = tkl_image_codec_dma_engine_addr(run_dst);
    cfg.ctrl.bits.length = s_end - run_src;
    cfg.ctrl.bits.int_finish_en = 1U;
    if (desc != runs || bk_dma_link_set_desc(table, desc - 1U, &cfg) != BK_OK) {
        goto exit;
    }

    /* Producer side: push every byte the DMA will touch out to RAM - the
     * spans include the inter-row padding, which is harmless to clean. The
     * helper ends with a DSB (cache.h), so the data is visible before the
     * channel starts. */
    bk_dcache_clean_for_producer(sv->y.base,
        sv->y.row_bytes + (sv->y.rows - 1U) * sv->y.pitch);
    bk_dcache_clean_for_producer(dv->y.base,
        dv->y.row_bytes + (dv->y.rows - 1U) * dv->y.pitch);
    if (sv->uv.rows != 0U) {
        bk_dcache_clean_for_producer(sv->uv.base,
            sv->uv.row_bytes + (sv->uv.rows - 1U) * sv->uv.pitch);
        bk_dcache_clean_for_producer(dv->uv.base,
            dv->uv.row_bytes + (dv->uv.rows - 1U) * dv->uv.pitch);
    }

    if (bk_dma_link_transfer(id, table) == BK_OK) {
        ret = OPRT_OK;
        if (tkl_semaphore_wait(s_tkl_image_copy_sem,
                               TUYA_IMAGE_CODEC_COPY_DMA_TIMEOUT_MS) != OPRT_OK) {
            /* TEMP DEBUG PROBE (root cause of the missing finish interrupt) -
             * read the channel's live registers BEFORE bk_dma_stop() clears
             * them, first three timeouts only so the bench stays readable.
             * Decision table:
             *   en=1, rd/wr at row 0     -> channel stalled, chain never ran
             *   en=0, fin_cnt>0          -> transfer completed, IRQ not routed
             *   en=0, fin_cnt=0, no move -> channel self-cleared, never ran
             */
            /* The three read-side getters have no public header - declare the
             * stable vendor ABI locally, same pattern as the cache helpers. */
            extern UINT32_T bk_dma_get_finish_interrupt_cnt(dma_id_t id);
            extern UINT32_T dma_get_src_read_addr(dma_id_t id);
            extern UINT32_T dma_get_dest_write_addr(dma_id_t id);

            static UINT8_T s_probe_left = 3U;
            if (s_probe_left != 0U) {
                s_probe_left--;
                tkl_log_output("imgdma probe: en=%u fin_cnt=%u ll=0x%08x tbl=0x%08x "
                               "rd=0x%08x wr=0x%08x rem=%u rows=%u desc=%u\r\n",
                               (unsigned int)bk_dma_get_enable_status(id),
                               (unsigned int)bk_dma_get_finish_interrupt_cnt(id),
                               (unsigned int)bk_dma_get_next_ll_addr(id),
                               (unsigned int)(UINT32_T)table,
                               (unsigned int)dma_get_src_read_addr(id),
                               (unsigned int)dma_get_dest_write_addr(id),
                               (unsigned int)bk_dma_get_remain_len(id),
                               (unsigned int)rows,
                               (unsigned int)runs);
            }
            (void)bk_dma_stop(id);
            s_tkl_image_copy_stale = TRUE;
            ret = OPRT_TIMEOUT;
        }
    }

    /* Consumer side: drop this core's stale view of the destination -
     * invalidate only, never clean+invalidate: the clean phase could write a
     * stale line back over the DMA's fresh data. Ordered against the caller's
     * reads with a DMB, matching the vendor dma_memcpy paradigm. */
    bk_dcache_invalidate_for_consumer(dv->y.base,
        dv->y.row_bytes + (dv->y.rows - 1U) * dv->y.pitch);
    if (dv->uv.rows != 0U) {
        bk_dcache_invalidate_for_consumer(dv->uv.base,
            dv->uv.row_bytes + (dv->uv.rows - 1U) * dv->uv.pitch);
    }
    __asm__ __volatile__ ("dmb sy" : : : "memory");

exit:
    if (id < DMA_ID_MAX) {
        (void)bk_dma_disable_finish_interrupt(id);
        (void)bk_dma_register_isr(id, NULL, NULL);
        (void)bk_dma_free((UINT16_T)DMA_DEV_DTCM, id);
    }
    if (table != NULL) {
        bk_dma_link_deinit(table);
    }

    return ret;
}

#endif /* CONFIG_GENERAL_DMA_LINK_MODE */

OPERATE_RET tkl_image_codec_memcpy(CONST TUYA_IMAGE_CODEC_COPY_BUF_T *src, CONST TUYA_IMAGE_CODEC_COPY_BUF_T *dst)
{
    TUYA_IMAGE_CODEC_COPY_VIEW_T sv;
    TUYA_IMAGE_CODEC_COPY_VIEW_T dv;
    BOOL_T overlap;
    BOOL_T descend;
    OPERATE_RET ret;

    if (src == NULL || dst == NULL) {
        return OPRT_INVALID_PARM;
    }
    if (src->buf.format != dst->buf.format) {
        return OPRT_NOT_SUPPORTED;      /* converting is convert()'s job */
    }

    ret = tkl_image_codec_copy_view(&src->buf, &src->region, &sv);
    if (ret != OPRT_OK) {
        return ret;
    }
    ret = tkl_image_codec_copy_view(&dst->buf, &dst->region, &dv);
    if (ret != OPRT_OK) {
        return ret;
    }

    /* same format on both sides: equal windows reduce to the luma shape */
    if (sv.y.rows != dv.y.rows || sv.y.row_bytes != dv.y.row_bytes) {
        return OPRT_INVALID_PARM;
    }

    /* one buffer overlapping itself: far side first, memmove within a row */
    overlap = (src->buf.buffer == dst->buf.buffer) ? TRUE : FALSE;
    descend = (overlap == TRUE && dv.y.base > sv.y.base) ? TRUE : FALSE;

#if CONFIG_GENERAL_DMA_LINK_MODE
    /* Backend pick: big, well-aligned, non-overlapping windows go over the
     * general DMA; everything else - and any DMA run that fails - stays on
     * the identical-result CPU row loop. An overlap always takes the CPU
     * (DMA has no move semantics), even forced; a forced non-overlap reports
     * its error instead of silently falling back. */
    if (overlap != TRUE) {
        UINT32_T total = sv.y.rows * sv.y.row_bytes + sv.uv.rows * sv.uv.row_bytes;

        if (g_tkl_image_memcpy_force == 2 ||
            (g_tkl_image_memcpy_force == 0 &&
             total > TUYA_IMAGE_CODEC_COPY_CPU_THRESHOLD)) {
            ret = tkl_image_codec_copy_dma(&sv, &dv);
            if (ret == OPRT_OK) {
                return OPRT_OK;
            }
            if (g_tkl_image_memcpy_force == 2) {
                return ret;
            }
        }
    }
#else
    if (g_tkl_image_memcpy_force == 2) {
        return OPRT_NOT_SUPPORTED;      /* this build has no DMA leg */
    }
#endif

    tkl_image_codec_copy_plane(&sv.y, &dv.y, descend);
    if (sv.uv.rows != 0U) {
        tkl_image_codec_copy_plane(&sv.uv, &dv.uv, descend);
    }

    return OPRT_OK;
}
