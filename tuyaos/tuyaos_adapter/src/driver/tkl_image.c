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
 * @version 0.1
 * @date 2026-08-27
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_image.h"
#include "tkl_media_buf.h"
#include "tkl_media_fmt.h"
#include "tkl_pp.h"
#include "tkl_memory.h"
#include "tkl_output.h"

#include <stdint.h>
#include <os/mem.h>
#include <common/bk_err.h>
#include <common/avdk_pixel_types.h>
#include <components/avdk_utils/avdk_error.h>
#include <components/bk_decode/bk_jpeg_decode_ctlr.h>
#include <components/bk_decode/bk_jpeg_decode_types.h>
#include <components/bk_encode/bk_jpeg_encode_ctlr.h>
#include <components/bk_encode/bk_jpeg_encode_types.h>

/**
 * Default when the caller leaves quality at 0: the all-ones quantisation table.
 *
 * Size the output buffer for ~4.5 bits/pixel at this setting (~1.3-1.6 MB at
 * 2304x1296); quality 90 costs about a third of that. Encode time is almost
 * independent of the setting on both backends.
 */
#define TKL_IMAGE_JPEG_DEFAULT_QUALITY  100U
#define TKL_IMAGE_DECODE_TIMEOUT_MS     1000U

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
#define TKL_IMAGE_QLEVEL_MAX            10U

STATIC OPERATE_RET tkl_image_map_err(avdk_err_t ret)
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

UINT32_T tkl_image_buffer_size(UINT32_T format, UINT16_T width, UINT16_T height)
{
    if (width == 0U || height == 0U) {
        return 0U;
    }

    if (format == (UINT32_T)TKL_PIXEL_FMT_YUYV) {
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
 * @return TKL_PIXEL_FMT_E
 */
STATIC UINT32_T tkl_image_jpeg_sampling_to_fmt(bk_jpeg_decode_img_fmt_t fmt)
{
    /* How the *stream* is sampled, not a layout the decoder can output - this is
     * informational. 4:4:4 and grayscale have no public counterpart. */
    switch (fmt) {
    case BK_JPEG_DECODE_IMG_FMT_YUV420:
        return (UINT32_T)TKL_PIXEL_FMT_NV12;
    case BK_JPEG_DECODE_IMG_FMT_YUV422:
        return (UINT32_T)TKL_PIXEL_FMT_YUYV;
    default:
        return (UINT32_T)TKL_PIXEL_FMT_UNKNOWN;
    }
}

OPERATE_RET tkl_image_probe(CONST TKL_MEDIA_BUF_T *in, TKL_IMAGE_INFO_T *info)
{
    bk_jpeg_decode_img_info_t img_info;
    avdk_err_t ret;

    if (in == NULL || info == NULL || in->data == NULL || in->size == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (in->codec == (UINT32_T)TKL_MEDIA_CODEC_H264) {
        /* bk_h264_decode_get_info() takes a decoder handle - parsing H264 needs
         * a live instance, which makes it stateful by definition */
        return OPRT_NOT_SUPPORTED;
    }

    if (in->codec != (UINT32_T)TKL_MEDIA_CODEC_JPEG &&
        in->codec != (UINT32_T)TKL_MEDIA_CODEC_NONE) {
        return OPRT_NOT_SUPPORTED;
    }

    os_memset(&img_info, 0, sizeof(img_info));
    img_info.input_stream = in->data;
    img_info.input_stream_length = in->size;

    ret = bk_jpeg_decode_get_img_info(&img_info);
    if (ret != AVDK_ERR_OK) {
        return tkl_image_map_err(ret);
    }

    if (img_info.format == BK_JPEG_DECODE_IMG_FMT_ERR ||
        img_info.width == 0U || img_info.height == 0U) {
        return OPRT_COM_ERROR;
    }

    os_memset(info, 0, sizeof(*info));
    info->width = (UINT16_T)img_info.width;
    info->height = (UINT16_T)img_info.height;
    info->format = tkl_image_jpeg_sampling_to_fmt(img_info.format);
    info->codec = (UINT32_T)TKL_MEDIA_CODEC_JPEG;
    return OPRT_OK;
}

/* ---------------------------------------------------------------------------
 * Decode
 * ------------------------------------------------------------------------- */

/**
 * @brief One decode, straight into the caller's buffer.
 *
 * @param[in]     in  JPEG stream
 * @param[in,out] out destination; geometry, format and capacity already checked
 *
 * @note Only valid when the request matches the stream: same size, NV12 out.
 *       Anything else needs the two-step path in tkl_image_decode().
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_decode_one(CONST TKL_MEDIA_BUF_T *in, TKL_MEDIA_BUF_T *out)
{
    bk_jpeg_decode_frame_config_t cfg;
    bk_jpeg_decode_input_t input;
    bk_jpeg_decode_ctlr_handle_t handle = NULL;
    UINT32_T need_size;
    UINT32_T bk_fmt;
    avdk_err_t ret;

    bk_fmt = tkl_media_fmt_to_bk(out->format);
    need_size = tkl_image_buffer_size(out->format, out->width, out->height);

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.timeout_ms = TKL_IMAGE_DECODE_TIMEOUT_MS;
    cfg.out_width = out->width;
    cfg.out_height = out->height;
    cfg.out_format = bk_fmt;

    os_memset(&input, 0, sizeof(input));
    input.stream = in->data;
    input.stream_len = in->size;
    input.out_buffer = out->data;
    input.out_buffer_size = out->size;

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
        return tkl_image_map_err(ret);
    }

    out->kind = (UINT8_T)TKL_MEDIA_BUF_PIXEL;
    out->size = need_size;
    out->codec = (UINT32_T)TKL_MEDIA_CODEC_NONE;
    return OPRT_OK;
}

OPERATE_RET tkl_image_decode(CONST TKL_MEDIA_BUF_T *in, TKL_MEDIA_BUF_T *out)
{
    TKL_IMAGE_INFO_T info;
    TKL_MEDIA_BUF_T native;
    TKL_IMAGE_XFORM_T xform;
    UINT8_T *tmp = NULL;
    UINT32_T need_size;
    UINT32_T tmp_size;
    OPERATE_RET ret;

    if (in == NULL || out == NULL || in->data == NULL || in->size == 0U ||
        out->data == NULL || out->size == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (in->codec == (UINT32_T)TKL_MEDIA_CODEC_H264) {
        return OPRT_NOT_SUPPORTED;      /* stateful - see tkl_stream */
    }

    if (out->width == 0U || out->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (out->format != (UINT32_T)TKL_PIXEL_FMT_NV12 &&
        out->format != (UINT32_T)TKL_PIXEL_FMT_RGB565 &&
        out->format != (UINT32_T)TKL_PIXEL_FMT_RGB888) {
        tkl_log_output("tkl_image: decode out format %u unsupported (NV12/RGB565/RGB888)\r\n",
                       (unsigned int)out->format);
        return OPRT_NOT_SUPPORTED;
    }

    need_size = tkl_image_buffer_size(out->format, out->width, out->height);
    if (need_size == 0U || out->size < need_size) {
        tkl_log_output("tkl_image: decode out buffer %u < %u needed for %ux%u fmt=%u\r\n",
                       (unsigned int)out->size, (unsigned int)need_size,
                       (unsigned int)out->width, (unsigned int)out->height,
                       (unsigned int)out->format);
        return OPRT_INVALID_PARM;
    }

    ret = tkl_image_probe(in, &info);
    if (ret != OPRT_OK) {
        return ret;
    }

    /* Native geometry and NV12: the decoder writes the caller's buffer directly,
     * which is the only request it can serve on its own. */
    if (out->format == (UINT32_T)TKL_PIXEL_FMT_NV12 &&
        out->width == info.width && out->height == info.height) {
        return tkl_image_decode_one(in, out);
    }

    /* A different size or RGB out makes the vendor decoder rebuild the whole
     * frame as NV12 before post-processing, and that buffer comes from vcdec's
     * default allocator - the JPEG controllers never call
     * vcdec_register_memalloc(), unlike the H264 ones - which cannot hand out a
     * full frame. Do the same two steps here instead, taking the buffer from the
     * slab it belongs in. Same peak memory, minus the broken allocation. */
    tmp_size = tkl_image_buffer_size((UINT32_T)TKL_PIXEL_FMT_NV12,
                                     info.width, info.height);
    if (tmp_size == 0U) {
        return OPRT_COM_ERROR;
    }

    tmp = (UINT8_T *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL, tmp_size,
                                         TKL_MEDIA_BUF_OWNER_IMAGE);
    if (tmp == NULL) {
        tkl_log_output("tkl_image: decode needs %u bytes for the %ux%u native "
                       "picture before scaling to %ux%u\r\n",
                       (unsigned int)tmp_size, info.width, info.height,
                       out->width, out->height);
        return OPRT_MALLOC_FAILED;
    }

    os_memset(&native, 0, sizeof(native));
    native.data = tmp;
    native.size = tmp_size;
    native.kind = (UINT8_T)TKL_MEDIA_BUF_PIXEL;
    native.width = info.width;
    native.height = info.height;
    native.format = (UINT32_T)TKL_PIXEL_FMT_NV12;

    ret = tkl_image_decode_one(in, &native);
    if (ret == OPRT_OK) {
        os_memset(&xform, 0, sizeof(xform));
        xform.width = out->width;
        xform.height = out->height;
        xform.format = out->format;

        ret = tkl_image_transform(&native, &xform, out);
        if (ret == OPRT_OK) {
            out->codec = (UINT32_T)TKL_MEDIA_CODEC_NONE;
        }
    }

    tkl_media_buf_free(TKL_MEDIA_BUF_OWNER_IMAGE, tmp);
    return ret;
}

/* ---------------------------------------------------------------------------
 * NV12 -> YUYV, the software encoder's input layout
 * ------------------------------------------------------------------------- */

/**
 * @brief Convert NV12 to packed YUYV.
 *
 * @param[in]  width    frame width, must be even
 * @param[in]  height   frame height, must be even
 * @param[in]  y_plane  Y plane base
 * @param[in]  uv_plane interleaved CbCr plane base
 * @param[out] yuyv     packed output, width*height*2 bytes
 *
 * @note Emits YVYU byte order (Y0 V Y1 U) - that is what tjpg expects, while
 *       NV12 chroma arrives as U then V.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_nv12_to_yuyv(UINT16_T width,
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

            pack0 = ((UINT32_T)uv[0] << 24) | ((UINT32_T)y_row0[x + 1U] << 16) |
                    ((UINT32_T)uv[1] << 8) | y_row0[x];
            pack1 = ((UINT32_T)uv[0] << 24) | ((UINT32_T)y_row1[x + 1U] << 16) |
                    ((UINT32_T)uv[1] << 8) | y_row1[x];
            *(UINT32_T *)d0 = pack0;
            *(UINT32_T *)d1 = pack1;

            pack0 = ((UINT32_T)uv[2] << 24) | ((UINT32_T)y_row0[x + 3U] << 16) |
                    ((UINT32_T)uv[3] << 8) | y_row0[x + 2U];
            pack1 = ((UINT32_T)uv[2] << 24) | ((UINT32_T)y_row1[x + 3U] << 16) |
                    ((UINT32_T)uv[3] << 8) | y_row1[x + 2U];
            *(UINT32_T *)(d0 + 4U) = pack0;
            *(UINT32_T *)(d1 + 4U) = pack1;

            pack0 = ((UINT32_T)uv[4] << 24) | ((UINT32_T)y_row0[x + 5U] << 16) |
                    ((UINT32_T)uv[5] << 8) | y_row0[x + 4U];
            pack1 = ((UINT32_T)uv[4] << 24) | ((UINT32_T)y_row1[x + 5U] << 16) |
                    ((UINT32_T)uv[5] << 8) | y_row1[x + 4U];
            *(UINT32_T *)(d0 + 8U) = pack0;
            *(UINT32_T *)(d1 + 8U) = pack1;

            pack0 = ((UINT32_T)uv[6] << 24) | ((UINT32_T)y_row0[x + 7U] << 16) |
                    ((UINT32_T)uv[7] << 8) | y_row0[x + 6U];
            pack1 = ((UINT32_T)uv[6] << 24) | ((UINT32_T)y_row1[x + 7U] << 16) |
                    ((UINT32_T)uv[7] << 8) | y_row1[x + 6U];
            *(UINT32_T *)(d0 + 12U) = pack0;
            *(UINT32_T *)(d1 + 12U) = pack1;
        }

        for (; x < (UINT32_T)width; x += 2U) {
            UINT8_T u = uv_row[x];
            UINT8_T v = uv_row[x + 1U];
            UINT32_T pack0 = ((UINT32_T)u << 24) | ((UINT32_T)y_row0[x + 1U] << 16) |
                             ((UINT32_T)v << 8) | y_row0[x];
            UINT32_T pack1 = ((UINT32_T)u << 24) | ((UINT32_T)y_row1[x + 1U] << 16) |
                             ((UINT32_T)v << 8) | y_row1[x];

            *(UINT32_T *)(out_row0 + (x * 2U)) = pack0;
            *(UINT32_T *)(out_row1 + (x * 2U)) = pack1;
        }
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
STATIC uint32_t tkl_image_jpeg_outbuf_complete(bk_jpeg_encode_outbuf_info_t *info)
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
STATIC UINT8_T tkl_image_quality_to_qlevel(UINT8_T quality)
{
    /* k_min_q[i] is the lowest public quality that selects qLevel i+1 */
    STATIC CONST UINT8_T k_min_q[TKL_IMAGE_QLEVEL_MAX] = {
        15U, 24U, 34U, 44U, 55U, 65U, 75U, 85U, 94U, 98U
    };
    UINT8_T level = 0U;

    while (level < TKL_IMAGE_QLEVEL_MAX && quality >= k_min_q[level]) {
        level++;
    }

    return level;
}

STATIC UINT8_T tkl_image_quality_resolve(UINT8_T quality)
{
    if (quality == 0U) {
        return (UINT8_T)TKL_IMAGE_JPEG_DEFAULT_QUALITY;
    }
    if (quality > 100U) {
        return 100U;
    }
    return quality;
}

/**
 * @brief Run one JPEG encode through a throwaway controller.
 *
 * @param[in]  use_sw  TRUE selects the YUYV software encoder, FALSE the NV12
 *                     hardware encoder
 * @param[in]  width   picture width
 * @param[in]  height  picture height
 * @param[in]  src     input base; YUYV when @p use_sw, NV12 luma otherwise
 * @param[in]  quality public quality, 1..100; converted per backend here, the
 *                     two encoders read the field on different scales
 * @param[out] out     destination buffer; @c size receives the encoded length
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_image_jpeg_encode_run(BOOL_T use_sw,
                                             UINT16_T width,
                                             UINT16_T height,
                                             CONST UINT8_T *src,
                                             UINT8_T quality,
                                             TKL_MEDIA_BUF_T *out)
{
    bk_jpeg_encode_ctlr_handle_t handle = NULL;
    bk_jpeg_encode_input_t input;
    UINT32_T encoded = 0U;
    avdk_err_t ret;

    os_memset(&input, 0, sizeof(input));
    input.pic_buf = (UINT32_T)(uintptr_t)src;
    input.out_buf = (UINT32_T)(uintptr_t)out->data;
    input.out_size = out->size;

    if (use_sw == TRUE) {
        bk_jpeg_encode_sw_frame_config_t cfg;

        os_memset(&cfg, 0, sizeof(cfg));
        cfg.width = width;
        cfg.height = height;
        cfg.input_format = (UINT32_T)BK_PIXEL_FORMAT_YUYV;
        /* tjpg takes the public 1..100 unchanged */
        cfg.quality = quality;
        cfg.outbuf_complete = tkl_image_jpeg_outbuf_complete;
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
        cfg.quality = tkl_image_quality_to_qlevel(quality);
        cfg.outbuf_complete = tkl_image_jpeg_outbuf_complete;
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
        out->size = 0U;
        return (ret != AVDK_ERR_OK) ? tkl_image_map_err(ret) : OPRT_COM_ERROR;
    }

    out->size = encoded;
    return OPRT_OK;
}

OPERATE_RET tkl_image_encode(CONST TKL_MEDIA_BUF_T *in,
                             CONST TKL_IMAGE_ENC_PARAM_T *param,
                             TKL_MEDIA_BUF_T *out)
{
    UINT32_T y_plane_size;
    CONST UINT8_T *y_plane;
    CONST UINT8_T *uv_plane;
    UINT8_T *yuyv_tmp = NULL;
    OPERATE_RET op_ret;
    UINT8_T quality;
    BOOL_T use_sw;

    if (in == NULL || param == NULL || out == NULL ||
        in->data == NULL || in->size == 0U ||
        out->data == NULL || out->size == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (param->codec != (UINT32_T)TKL_MEDIA_CODEC_JPEG &&
        param->codec != (UINT32_T)TKL_MEDIA_CODEC_NONE) {
        /* H264 keeps a GOP and reference frames across calls - it cannot be a
         * one-shot operation */
        return OPRT_NOT_SUPPORTED;
    }

    if (in->width == 0U || in->height == 0U ||
        (in->width & 1U) != 0U || (in->height & 1U) != 0U) {
        return OPRT_INVALID_PARM;
    }

    /* stays on the public 1..100 scale; tkl_image_jpeg_encode_run() converts */
    quality = tkl_image_quality_resolve(param->quality);
    y_plane_size = (UINT32_T)in->width * in->height;

    if (in->format == (UINT32_T)TKL_PIXEL_FMT_YUYV) {
        if (in->size < (y_plane_size * 2U)) {
            return OPRT_INVALID_PARM;
        }
        /* the hardware encoder takes NV12 only */
        return tkl_image_jpeg_encode_run(TRUE, in->width, in->height,
                                         in->data, quality, out);
    }

    if (in->format != (UINT32_T)TKL_PIXEL_FMT_NV12) {
        return OPRT_NOT_SUPPORTED;
    }

    if (in->size < (y_plane_size + (y_plane_size / 2U))) {
        return OPRT_INVALID_PARM;
    }

    y_plane = in->data;
    uv_plane = (in->uv_stride != 0U) ? (in->data + in->y_stride * in->height)
                                     : (in->data + y_plane_size);

    use_sw = (param->prefer_sw == TRUE) ? TRUE : FALSE;
    if (use_sw != TRUE) {
        return tkl_image_jpeg_encode_run(FALSE, in->width, in->height,
                                         y_plane, quality, out);
    }

    /* software path: tjpg takes packed YUYV, so convert first */
    yuyv_tmp = (UINT8_T *)tkl_system_psram_malloc(y_plane_size * 2U);
    if (yuyv_tmp == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    op_ret = tkl_image_nv12_to_yuyv(in->width, in->height, y_plane, uv_plane, yuyv_tmp);
    if (op_ret == OPRT_OK) {
        op_ret = tkl_image_jpeg_encode_run(TRUE, in->width, in->height,
                                           yuyv_tmp, quality, out);
    }

    tkl_system_psram_free(yuyv_tmp);
    return op_ret;
}

/* ---------------------------------------------------------------------------
 * Transform
 * ------------------------------------------------------------------------- */

OPERATE_RET tkl_image_transform(CONST TKL_MEDIA_BUF_T *in,
                                CONST TKL_IMAGE_XFORM_T *xform,
                                TKL_MEDIA_BUF_T *out)
{
    TKL_PP_HANDLE pp = NULL;
    TKL_PP_IN_T pp_in;
    TKL_PP_OUT_T pp_out;
    UINT16_T dst_w;
    UINT16_T dst_h;
    UINT32_T y_plane_size;
    UINT32_T need_size;
    OPERATE_RET ret;

    if (in == NULL || xform == NULL || out == NULL ||
        in->data == NULL || in->size == 0U ||
        out->data == NULL || out->size == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (in->format != (UINT32_T)TKL_PIXEL_FMT_NV12) {
        tkl_log_output("tkl_image: transform input must be NV12 (got %u)\r\n",
                       (unsigned int)in->format);
        return OPRT_NOT_SUPPORTED;
    }

    if (in->width == 0U || in->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    /* No frame-mode engine on this part can do any of these: the PP scales and
     * converts only, the GPU rotates only under a flexa peripheral, and ROTT has
     * no driver. Refusing beats returning an untransformed picture. */
    if (xform->rotate != 0U || xform->mirror == TRUE || xform->compress == TRUE) {
        tkl_log_output("tkl_image: transform rotate/mirror/compress unsupported "
                       "(rotate=%u mirror=%u compress=%u)\r\n",
                       (unsigned int)xform->rotate,
                       (unsigned int)xform->mirror,
                       (unsigned int)xform->compress);
        return OPRT_NOT_SUPPORTED;
    }

    dst_w = (xform->width != 0U) ? xform->width : in->width;
    dst_h = (xform->height != 0U) ? xform->height : in->height;

    y_plane_size = (UINT32_T)in->width * in->height;
    if (in->size < (y_plane_size + (y_plane_size / 2U))) {
        return OPRT_INVALID_PARM;
    }

    need_size = tkl_image_buffer_size(xform->format, dst_w, dst_h);
    if (need_size == 0U) {
        return OPRT_NOT_SUPPORTED;
    }
    if (out->size < need_size) {
        tkl_log_output("tkl_image: transform out buffer %u < %u needed\r\n",
                       (unsigned int)out->size, (unsigned int)need_size);
        return OPRT_INVALID_PARM;
    }

    if (xform->format == (UINT32_T)TKL_PIXEL_FMT_YUYV) {
        /* the PP has no YUYV output; the CPU fallback cannot scale either */
        if (dst_w != in->width || dst_h != in->height) {
            tkl_log_output("tkl_image: YUYV output cannot be scaled\r\n");
            return OPRT_NOT_SUPPORTED;
        }

        ret = tkl_image_nv12_to_yuyv(in->width, in->height,
                                     in->data, in->data + y_plane_size, out->data);
        if (ret != OPRT_OK) {
            return ret;
        }
    } else {
        ret = tkl_pp_open(0U, &pp);
        if (ret != OPRT_OK) {
            return ret;
        }

        os_memset(&pp_in, 0, sizeof(pp_in));
        pp_in.y = in->data;
        pp_in.uv = in->data + y_plane_size;
        pp_in.width = in->width;
        pp_in.height = in->height;

        os_memset(&pp_out, 0, sizeof(pp_out));
        pp_out.buffer = out->data;
        pp_out.size = out->size;
        pp_out.width = dst_w;
        pp_out.height = dst_h;
        pp_out.format = tkl_media_fmt_to_bk(xform->format);

        ret = tkl_pp_process(pp, &pp_in, &pp_out);
        (void)tkl_pp_close(pp);
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    out->kind = (UINT8_T)TKL_MEDIA_BUF_PIXEL;
    out->width = dst_w;
    out->height = dst_h;
    out->format = xform->format;
    out->size = need_size;
    out->y_stride = 0U;
    out->uv_stride = 0U;
    return OPRT_OK;
}
