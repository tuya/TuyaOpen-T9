/**
 * @file tkl_venc.c
 * @brief Video encoder TKL adapter
 * @version 0.2
 * @date 2026-04-15
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include <stdint.h>
#include <common/avdk_pixel_types.h>
#include <common/bk_err.h>
#include <components/avdk_utils/avdk_error.h>
#include <components/bk_encode/bk_h264_encode_ctlr.h>
#include <components/bk_encode/bk_h264_encode_types.h>
#include <components/bk_encode/bk_jpeg_encode_ctlr.h>
#include <components/bk_flexa_bond.h>
#include "tkl_memory.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_venc.h"

#define TKL_VENC_STOP_DRAIN_MS              300U
#define TKL_VENC_JPEG_DEFAULT_QUALITY       2U
#define TKL_VENC_H264_DEFAULT_GOP           40U
#define TKL_VENC_H264_DEFAULT_BITRATE       1500000U
#define TKL_VENC_H264_QP_MIN_I              20U
#define TKL_VENC_H264_QP_MAX_I              51U
#define TKL_VENC_H264_QP_MIN_P              26U
#define TKL_VENC_H264_QP_MAX_P              51U
#define TKL_VENC_ISP_FLEXA_LINES            16U
#define TKL_VENC_JPEG_SW_DEFAULT_QUALITY    80U

typedef enum {
    TKL_VENC_BACKEND_NONE = 0,
    TKL_VENC_BACKEND_H264,
    TKL_VENC_BACKEND_JPEG,
} TKL_VENC_BACKEND_E;

typedef struct {
    BOOL_T started;
    BOOL_T backend_opened;
    TKL_VENC_BACKEND_E backend_type;
    TUYA_VENC_CFG_T cfg;
    TUYA_VENC_RATE_CTRL_T rate_ctrl;
    TUYA_VENC_SOURCE_T source;
    TUYA_VENC_SINK_T sink;
    union {
        bk_h264_encode_ctlr_handle_t h264;
        bk_jpeg_encode_ctlr_handle_t jpeg;
    } backend;
    VOID *bond_handle;
} TUYA_VENC_CTX_T;

static TUYA_VENC_CTX_T *tkl_venc_ctx_get(TUYA_VENC_HANDLE handle)
{
    return (TUYA_VENC_CTX_T *)handle;
}

static BOOL_T tkl_venc_is_h264_codec(UINT32_T codec)
{
    return (codec == TUYA_VCODEC_TYPE_H264);
}

static BOOL_T tkl_venc_is_jpeg_codec(UINT32_T codec)
{
    return (codec == TUYA_VCODEC_TYPE_JPEG || codec == TUYA_VCODEC_TYPE_MJPEG);
}

static UINT32_T tkl_venc_gop_resolve(const TUYA_VENC_CTX_T *ctx)
{
    if (ctx->cfg.gop != 0U) {
        return ctx->cfg.gop;
    }

    return TKL_VENC_H264_DEFAULT_GOP;
}

static VOID tkl_venc_h264_rate_ctrl_default(TUYA_VENC_RATE_CTRL_T *rate_ctrl)
{
    if (rate_ctrl == NULL) {
        return;
    }

    tkl_system_memset(rate_ctrl, 0, sizeof(*rate_ctrl));
    rate_ctrl->bitrate = TKL_VENC_H264_DEFAULT_BITRATE;
    rate_ctrl->qp_min_i = TKL_VENC_H264_QP_MIN_I;
    rate_ctrl->qp_max_i = TKL_VENC_H264_QP_MAX_I;
    rate_ctrl->qp_min_p = TKL_VENC_H264_QP_MIN_P;
    rate_ctrl->qp_max_p = TKL_VENC_H264_QP_MAX_P;
}

static VOID tkl_venc_h264_rate_ctrl_build(const TUYA_VENC_RATE_CTRL_T *src,
                                          bk_h264_encode_rate_ctrl_t *rate_ctrl)
{
    if (src == NULL || rate_ctrl == NULL) {
        return;
    }

    tkl_system_memset(rate_ctrl, 0, sizeof(*rate_ctrl));
    rate_ctrl->bitrate = src->bitrate;
    rate_ctrl->qp_min_i = src->qp_min_i;
    rate_ctrl->qp_max_i = src->qp_max_i;
    rate_ctrl->qp_min_p = src->qp_min_p;
    rate_ctrl->qp_max_p = src->qp_max_p;

    if (rate_ctrl->bitrate > 0U) {
        if (rate_ctrl->qp_min_i == 0U) {
            rate_ctrl->qp_min_i = TKL_VENC_H264_QP_MIN_I;
        }
        if (rate_ctrl->qp_max_i == 0U) {
            rate_ctrl->qp_max_i = TKL_VENC_H264_QP_MAX_I;
        }
        if (rate_ctrl->qp_min_p == 0U) {
            rate_ctrl->qp_min_p = TKL_VENC_H264_QP_MIN_P;
        }
        if (rate_ctrl->qp_max_p == 0U) {
            rate_ctrl->qp_max_p = TKL_VENC_H264_QP_MAX_P;
        }
    }
}

static VOID tkl_venc_h264_rate_ctrl_store(TUYA_VENC_CTX_T *ctx,
                                          const bk_h264_encode_rate_ctrl_t *rate_ctrl)
{
    if (ctx == NULL || rate_ctrl == NULL) {
        return;
    }

    ctx->rate_ctrl.bitrate = rate_ctrl->bitrate;
    ctx->rate_ctrl.qp_min_i = rate_ctrl->qp_min_i;
    ctx->rate_ctrl.qp_max_i = rate_ctrl->qp_max_i;
    ctx->rate_ctrl.qp_min_p = rate_ctrl->qp_min_p;
    ctx->rate_ctrl.qp_max_p = rate_ctrl->qp_max_p;
}

static OPERATE_RET tkl_venc_h264_apply_rate_ctrl(TUYA_VENC_CTX_T *ctx,
                                                const TUYA_VENC_RATE_CTRL_T *req,
                                                BOOL_T log_result)
{
    bk_h264_encode_rate_ctrl_t rate_ctrl;
    TUYA_VENC_RATE_CTRL_T local;
    const TUYA_VENC_RATE_CTRL_T *src = req;
    avdk_err_t ret;
    BOOL_T force_idr = FALSE;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (req != NULL) {
        tkl_system_memcpy(&local, req, sizeof(local));
        if (local.bitrate == 0U &&
            local.qp_min_i == 0U && local.qp_max_i == 0U &&
            local.qp_min_p == 0U && local.qp_max_p == 0U) {
            return OPRT_INVALID_PARM;
        }
        force_idr = local.force_idr;
        src = &local;
    } else {
        src = &ctx->rate_ctrl;
    }

    tkl_venc_h264_rate_ctrl_build(src, &rate_ctrl);
    if (rate_ctrl.bitrate == 0U &&
        (rate_ctrl.qp_min_i == 0U || rate_ctrl.qp_max_i == 0U ||
         rate_ctrl.qp_min_p == 0U || rate_ctrl.qp_max_p == 0U)) {
        return OPRT_INVALID_PARM;
    }

    tkl_venc_h264_rate_ctrl_store(ctx, &rate_ctrl);

    if (ctx->backend.h264 == NULL || ctx->backend_opened != TRUE) {
        return OPRT_OK;
    }

    ret = bk_h264_encode_set_rate_ctrl(ctx->backend.h264, &rate_ctrl);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_venc: bk_h264_encode_set_rate_ctrl failed (%d), bitrate=%u\r\n",
                ret, (unsigned int)rate_ctrl.bitrate);
        return OPRT_COM_ERROR;
    }

    if (log_result == TRUE) {
        tkl_log_output("tkl_venc: h264 rate ctrl bitrate=%u qp_i=%u..%u qp_p=%u..%u\r\n",
                (unsigned int)rate_ctrl.bitrate,
                (unsigned int)rate_ctrl.qp_min_i,
                (unsigned int)rate_ctrl.qp_max_i,
                (unsigned int)rate_ctrl.qp_min_p,
                (unsigned int)rate_ctrl.qp_max_p);
    }

    if (force_idr == TRUE) {
        ret = bk_h264_encode_force_idr(ctx->backend.h264);
        if (ret != AVDK_ERR_OK) {
            tkl_log_output("tkl_venc: force idr after rate ctrl failed (%d)\r\n", ret);
            return OPRT_COM_ERROR;
        }
    }

    return OPRT_OK;
}

static UINT8_T tkl_venc_jpeg_quality_resolve(const TUYA_VENC_CTX_T *ctx)
{
    if (ctx != NULL && ctx->cfg.quality <= 10U && ctx->cfg.quality > 0U) {
        return ctx->cfg.quality;
    }

    return TKL_VENC_JPEG_DEFAULT_QUALITY;
}

static UINT32_T tkl_venc_input_format_resolve(const TUYA_VENC_CTX_T *ctx)
{
    if (ctx->cfg.input_format != 0U) {
        return ctx->cfg.input_format;
    }

    switch (ctx->source.type) {
    case TUYA_VENC_SOURCE_ISP_FLEXA:
        return ctx->source.isp.flexa.format;
    case TUYA_VENC_SOURCE_VDEC_FLEXA:
        return ctx->source.vdec.flexa.format;
    default:
        return 0U;
    }
}

static UINT32_T tkl_venc_input_buf_resolve(const TUYA_VENC_CTX_T *ctx)
{
    switch (ctx->source.type) {
    case TUYA_VENC_SOURCE_ISP_FLEXA:
        if (ctx->source.isp.flexa.y_addr != 0U) {
            return ctx->source.isp.flexa.y_addr;
        }
        return (UINT32_T)(uintptr_t)ctx->source.isp.flexa.src_buffer;

    case TUYA_VENC_SOURCE_VDEC_FLEXA:
        if (ctx->source.vdec.flexa.y_addr != 0U) {
            return ctx->source.vdec.flexa.y_addr;
        }
        return (UINT32_T)(uintptr_t)ctx->source.vdec.flexa.src_buffer;

    default:
        return 0U;
    }
}

static UINT32_T tkl_venc_input_cnt_resolve(const TUYA_VENC_CTX_T *ctx)
{
    switch (ctx->source.type) {
    case TUYA_VENC_SOURCE_ISP_FLEXA:
        return ctx->source.isp.flexa.buf_cnt;
    case TUYA_VENC_SOURCE_VDEC_FLEXA:
        return ctx->source.vdec.flexa.buf_cnt;
    default:
        return 0U;
    }
}

/**
 * @brief Resolve ISP flexa ring buffer byte size for H264 HW encoder
 * @param[in] ctx encoder context
 * @return ring buffer size in bytes, 0 if unavailable
 * @note Matches doorbell app_codec_isp_channel_buffer_size() semantics.
 */
static UINT32_T tkl_venc_isp_flexa_buffer_size_resolve(const TUYA_VENC_CTX_T *ctx)
{
    UINT32_T y_size;
    UINT32_T width;
    UINT32_T buf_cnt;

    if (ctx == NULL || ctx->source.type != TUYA_VENC_SOURCE_ISP_FLEXA) {
        return 0U;
    }

    if (ctx->source.isp.flexa.u_addr > ctx->source.isp.flexa.y_addr) {
        y_size = ctx->source.isp.flexa.u_addr - ctx->source.isp.flexa.y_addr;
        return y_size + y_size / 2U;
    }

    width = ctx->source.isp.flexa.width;
    buf_cnt = ctx->source.isp.flexa.buf_cnt;
    if (width == 0U || buf_cnt == 0U) {
        return 0U;
    }

    return width * TKL_VENC_ISP_FLEXA_LINES * buf_cnt * 3U / 2U;
}

static VOID tkl_venc_size_resolve(const TUYA_VENC_CTX_T *ctx, UINT16_T *width, UINT16_T *height)
{
    UINT16_T resolved_width = 0U;
    UINT16_T resolved_height = 0U;

    if (ctx == NULL || width == NULL || height == NULL) {
        return;
    }

    resolved_width = ctx->cfg.width;
    resolved_height = ctx->cfg.height;
    if (resolved_width == 0U || resolved_height == 0U) {
        switch (ctx->source.type) {
        case TUYA_VENC_SOURCE_ISP_FLEXA:
            resolved_width = ctx->source.isp.flexa.width;
            resolved_height = ctx->source.isp.flexa.height;
            break;

        case TUYA_VENC_SOURCE_VDEC_FLEXA:
            resolved_width = ctx->source.vdec.flexa.width;
            resolved_height = ctx->source.vdec.flexa.height;
            break;

        default:
            break;
        }
    }

    *width = resolved_width;
    *height = resolved_height;
}

static TUYA_VCODEC_FRAME_TYPE_E tkl_venc_frame_type_convert(UINT32_T h264_type)
{
    switch (h264_type) {
    case 0:
        return TUYA_VCODEC_FRAME_I;
    case 1:
        return TUYA_VCODEC_FRAME_P;
    case 2:
        return TUYA_VCODEC_FRAME_B;
    case 3:
        return TUYA_VCODEC_FRAME_HEADER;
    case 4:
        return TUYA_VCODEC_FRAME_END;
    default:
        return TUYA_VCODEC_FRAME_UNKNOWN;
    }
}

static OPERATE_RET tkl_venc_source_validate(const TUYA_VENC_SOURCE_T *source)
{
    if (source == NULL) {
        return OPRT_INVALID_PARM;
    }

    switch (source->type) {
    case TUYA_VENC_SOURCE_ISP_FLEXA:
        if (source->isp.isp_handle == NULL ||
            (source->isp.flexa.src_buffer == NULL && source->isp.flexa.y_addr == 0U) ||
            source->isp.flexa.buf_cnt == 0U ||
            source->isp.flexa.width == 0U ||
            source->isp.flexa.height == 0U ||
            source->isp.flexa.format == BK_PIXEL_FORMAT_UNKNOW ||
            source->isp.flexa.format == BK_PIXEL_FORMAT_INVALID) {
            return OPRT_INVALID_PARM;
        }
        return OPRT_OK;

    case TUYA_VENC_SOURCE_VDEC_FLEXA:
        if (source->vdec.vdec_handle == NULL ||
            (source->vdec.flexa.src_buffer == NULL && source->vdec.flexa.y_addr == 0U) ||
            source->vdec.flexa.buf_cnt == 0U ||
            source->vdec.flexa.width == 0U ||
            source->vdec.flexa.height == 0U ||
            source->vdec.flexa.format == BK_PIXEL_FORMAT_UNKNOW ||
            source->vdec.flexa.format == BK_PIXEL_FORMAT_INVALID) {
            return OPRT_INVALID_PARM;
        }
        return OPRT_OK;

    case TUYA_VENC_SOURCE_USER_FRAME:
        return OPRT_OK;

    default:
        return OPRT_NOT_SUPPORTED;
    }
}

static OPERATE_RET tkl_venc_sink_validate(const TUYA_VENC_SINK_T *sink)
{
    if (sink == NULL || sink->stream_cb == NULL) {
        return OPRT_INVALID_PARM;
    }

    if ((sink->alloc_cb == NULL) != (sink->free_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static void *tkl_venc_outbuf_malloc(uint32_t outbuf_size, void *args)
{
    TUYA_VENC_CTX_T *ctx = (TUYA_VENC_CTX_T *)args;
    VOID *payload = NULL;

    if (ctx == NULL) {
        return NULL;
    }

    if (ctx->sink.alloc_cb != NULL) {
        payload = ctx->sink.alloc_cb(outbuf_size, ctx->sink.cb_arg);
    } else {
        payload = tkl_system_malloc(outbuf_size);
    }

    return payload;
}

static VOID tkl_venc_emit_stream(TUYA_VENC_CTX_T *ctx, VOID *payload, UINT32_T length,
                                 TUYA_VCODEC_FRAME_TYPE_E frame_type)
{
    TUYA_VCODEC_STREAM_T stream;

    if (ctx == NULL || payload == NULL) {
        return;
    }

    tkl_system_memset(&stream, 0, sizeof(stream));
    stream.data = (UINT8_T *)payload;
    stream.size = length;
    stream.pts = 0;
    stream.codec = ctx->cfg.codec;
    if (tkl_venc_is_jpeg_codec(ctx->cfg.codec)) {
        stream.codec = TUYA_VCODEC_TYPE_JPEG;
    }
    stream.frame_type = frame_type;
    stream.priv = payload;
    ctx->sink.stream_cb(&stream, ctx->sink.cb_arg);
}

static VOID tkl_venc_release_outbuf(TUYA_VENC_CTX_T *ctx, VOID *payload)
{
    if (ctx == NULL || payload == NULL) {
        return;
    }

    if (ctx->sink.free_cb != NULL) {
        (void)ctx->sink.free_cb(payload, ctx->sink.cb_arg);
    } else {
        tkl_system_free(payload);
    }
}

static uint32_t tkl_venc_h264_outbuf_complete(bk_h264_encode_outbuf_info_t *info)
{
    TUYA_VENC_CTX_T *ctx = NULL;
    VOID *payload = NULL;

    if (info == NULL) {
        return BK_FAIL;
    }

    ctx = (TUYA_VENC_CTX_T *)info->args;
    payload = info->outbuf;
    if (ctx == NULL || payload == NULL) {
        return BK_FAIL;
    }

    if (info->status == BK_OK) {
        tkl_venc_emit_stream(ctx, payload, info->length,
                             tkl_venc_frame_type_convert(info->type));
    }

    tkl_venc_release_outbuf(ctx, payload);
    return BK_OK;
}

static uint32_t tkl_venc_jpeg_outbuf_complete(bk_jpeg_encode_outbuf_info_t *info)
{
    TUYA_VENC_CTX_T *ctx = NULL;
    VOID *payload = NULL;

    if (info == NULL) {
        return BK_FAIL;
    }

    ctx = (TUYA_VENC_CTX_T *)info->args;
    payload = info->outbuf;
    if (ctx == NULL || payload == NULL) {
        return BK_FAIL;
    }

    if (info->status == BK_OK) {
        tkl_venc_emit_stream(ctx, payload, info->length, TUYA_VCODEC_FRAME_I);
    }

    tkl_venc_release_outbuf(ctx, payload);
    return BK_OK;
}

static OPERATE_RET tkl_venc_h264_backend_create(TUYA_VENC_CTX_T *ctx)
{
    avdk_err_t ret;
    UINT16_T width = 0U;
    UINT16_T height = 0U;

    if (ctx->source.type != TUYA_VENC_SOURCE_ISP_FLEXA &&
        ctx->source.type != TUYA_VENC_SOURCE_VDEC_FLEXA) {
        return OPRT_NOT_SUPPORTED;
    }

    tkl_venc_size_resolve(ctx, &width, &height);
    if (width == 0U || height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (tkl_venc_input_format_resolve(ctx) == BK_PIXEL_FORMAT_UNKNOW ||
        tkl_venc_input_format_resolve(ctx) == BK_PIXEL_FORMAT_INVALID) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->source.type == TUYA_VENC_SOURCE_VDEC_FLEXA) {
        bk_h264_encode_sw_flexa_config_t sw_cfg;

        tkl_system_memset(&sw_cfg, 0, sizeof(sw_cfg));
        sw_cfg.width = width;
        sw_cfg.height = height;
        sw_cfg.gop_frame_count = tkl_venc_gop_resolve(ctx);
        sw_cfg.input_format = tkl_venc_input_format_resolve(ctx);
        sw_cfg.input_flexa_cnt = tkl_venc_input_cnt_resolve(ctx);
        sw_cfg.input_buf = tkl_venc_input_buf_resolve(ctx);
        sw_cfg.input_size = height;
        sw_cfg.outbuf_malloc = tkl_venc_outbuf_malloc;
        sw_cfg.outbuf_malloc_args = ctx;
        sw_cfg.outbuf_complete = tkl_venc_h264_outbuf_complete;
        sw_cfg.outbuf_complete_args = ctx;

        ret = bk_h264_encode_sw_flexa_new(&ctx->backend.h264, &sw_cfg);
    } else {
        bk_h264_encode_hw_flexa_config_t hw_cfg;
        UINT32_T input_buffer_size;

        tkl_system_memset(&hw_cfg, 0, sizeof(hw_cfg));
        hw_cfg.width = width;
        hw_cfg.height = height;
        hw_cfg.gop_frame_count = tkl_venc_gop_resolve(ctx);
        hw_cfg.input_format = tkl_venc_input_format_resolve(ctx);
        hw_cfg.input_flexa_cnt = tkl_venc_input_cnt_resolve(ctx);
        hw_cfg.input_buf = tkl_venc_input_buf_resolve(ctx);
        input_buffer_size = tkl_venc_isp_flexa_buffer_size_resolve(ctx);
        if (input_buffer_size == 0U) {
            return OPRT_INVALID_PARM;
        }
        hw_cfg.input_size = input_buffer_size;
        hw_cfg.outbuf_malloc = tkl_venc_outbuf_malloc;
        hw_cfg.outbuf_malloc_args = ctx;
        hw_cfg.outbuf_complete = tkl_venc_h264_outbuf_complete;
        hw_cfg.outbuf_complete_args = ctx;

        ret = bk_h264_encode_hw_flexa_new(&ctx->backend.h264, &hw_cfg);
    }
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_venc: h264 flexa new failed (%d)\r\n", ret);
        if (ret == AVDK_ERR_NOMEM) {
            return OPRT_MALLOC_FAILED;
        }
        if (ret == AVDK_ERR_INVAL) {
            return OPRT_INVALID_PARM;
        }
        return OPRT_COM_ERROR;
    }

    ret = bk_h264_encode_init(ctx->backend.h264);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_venc: bk_h264_encode_init failed (%d)\r\n", ret);
        (void)bk_h264_encode_delete(ctx->backend.h264);
        ctx->backend.h264 = NULL;
        return OPRT_COM_ERROR;
    }

    ctx->backend_type = TKL_VENC_BACKEND_H264;
    ctx->backend_opened = FALSE;
    return OPRT_OK;
}

static OPERATE_RET tkl_venc_jpeg_backend_create(TUYA_VENC_CTX_T *ctx)
{
    avdk_err_t ret;
    UINT16_T width = 0U;
    UINT16_T height = 0U;
    bk_jpeg_encode_hw_flexa_config_t hw_cfg;

    if (ctx->source.type != TUYA_VENC_SOURCE_ISP_FLEXA) {
        return OPRT_NOT_SUPPORTED;
    }

    tkl_venc_size_resolve(ctx, &width, &height);
    if (width == 0U || height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (tkl_venc_input_format_resolve(ctx) == BK_PIXEL_FORMAT_UNKNOW ||
        tkl_venc_input_format_resolve(ctx) == BK_PIXEL_FORMAT_INVALID) {
        return OPRT_INVALID_PARM;
    }

    tkl_system_memset(&hw_cfg, 0, sizeof(hw_cfg));
    hw_cfg.width = width;
    hw_cfg.height = height;
    hw_cfg.input_format = tkl_venc_input_format_resolve(ctx);
    hw_cfg.input_flexa_cnt = tkl_venc_input_cnt_resolve(ctx);
    hw_cfg.input_buf = tkl_venc_input_buf_resolve(ctx);
    hw_cfg.input_size = tkl_venc_input_cnt_resolve(ctx);
    hw_cfg.quality = tkl_venc_jpeg_quality_resolve(ctx);
    hw_cfg.outbuf_malloc = tkl_venc_outbuf_malloc;
    hw_cfg.outbuf_malloc_args = ctx;
    hw_cfg.outbuf_complete = tkl_venc_jpeg_outbuf_complete;
    hw_cfg.outbuf_complete_args = ctx;

    ret = bk_jpeg_encode_hw_flexa_new(&ctx->backend.jpeg, &hw_cfg);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_venc: jpeg flexa new failed (%d)\r\n", ret);
        if (ret == AVDK_ERR_NOMEM) {
            return OPRT_MALLOC_FAILED;
        }
        if (ret == AVDK_ERR_INVAL) {
            return OPRT_INVALID_PARM;
        }
        return OPRT_COM_ERROR;
    }

    ret = bk_jpeg_encode_init(ctx->backend.jpeg);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_venc: bk_jpeg_encode_init failed (%d)\r\n", ret);
        (void)bk_jpeg_encode_delete(ctx->backend.jpeg);
        ctx->backend.jpeg = NULL;
        return OPRT_COM_ERROR;
    }

    ctx->backend_type = TKL_VENC_BACKEND_JPEG;
    ctx->backend_opened = FALSE;
    return OPRT_OK;
}

static OPERATE_RET tkl_venc_backend_create(TUYA_VENC_CTX_T *ctx)
{
    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (tkl_venc_is_h264_codec(ctx->cfg.codec)) {
        return tkl_venc_h264_backend_create(ctx);
    }

    if (tkl_venc_is_jpeg_codec(ctx->cfg.codec)) {
        return tkl_venc_jpeg_backend_create(ctx);
    }

    return OPRT_NOT_SUPPORTED;
}

static OPERATE_RET tkl_venc_backend_open(TUYA_VENC_CTX_T *ctx)
{
    avdk_err_t ret;
    OPERATE_RET op_ret;

    if (ctx == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (ctx->backend_type == TKL_VENC_BACKEND_H264) {
        if (ctx->backend.h264 == NULL) {
            return OPRT_RESOURCE_NOT_READY;
        }

        ret = bk_h264_encode_open(ctx->backend.h264);
        if (ret != AVDK_ERR_OK) {
            tkl_log_output("tkl_venc: bk_h264_encode_open failed (%d)\r\n", ret);
            return OPRT_COM_ERROR;
        }

        ctx->backend_opened = TRUE;
        op_ret = tkl_venc_h264_apply_rate_ctrl(ctx, NULL, TRUE);
        if (op_ret != OPRT_OK) {
            ctx->backend_opened = FALSE;
            (void)bk_h264_encode_close(ctx->backend.h264);
            return op_ret;
        }
    } else if (ctx->backend_type == TKL_VENC_BACKEND_JPEG) {
        if (ctx->backend.jpeg == NULL) {
            return OPRT_RESOURCE_NOT_READY;
        }

        ret = bk_jpeg_encode_open(ctx->backend.jpeg);
        if (ret != AVDK_ERR_OK) {
            tkl_log_output("tkl_venc: bk_jpeg_encode_open failed (%d)\r\n", ret);
            return OPRT_COM_ERROR;
        }
        ctx->backend_opened = TRUE;
    } else {
        return OPRT_RESOURCE_NOT_READY;
    }
    return OPRT_OK;
}

static OPERATE_RET tkl_venc_bond_start(TUYA_VENC_CTX_T *ctx)
{
    avdk_err_t ret;

    if (ctx->backend_type == TKL_VENC_BACKEND_H264) {
        switch (ctx->source.type) {
        case TUYA_VENC_SOURCE_ISP_FLEXA:
            ret = bk_flexa_isp_h264e_bond_start(&ctx->bond_handle,
                                                ctx->source.isp.isp_handle,
                                                ctx->backend.h264);
            break;

        case TUYA_VENC_SOURCE_VDEC_FLEXA:
            ret = bk_flexa_mjpegd_h264e_bond_start(&ctx->bond_handle,
                                                   ctx->source.vdec.vdec_handle,
                                                   ctx->backend.h264);
            break;

        default:
            return OPRT_NOT_SUPPORTED;
        }
    } else if (ctx->backend_type == TKL_VENC_BACKEND_JPEG) {
        if (ctx->source.type != TUYA_VENC_SOURCE_ISP_FLEXA) {
            return OPRT_NOT_SUPPORTED;
        }

        ret = bk_flexa_isp_jpege_bond_start(&ctx->bond_handle,
                                            ctx->source.isp.isp_handle,
                                            ctx->backend.jpeg);
    } else {
        return OPRT_NOT_SUPPORTED;
    }

    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_venc: bond start failed (%d)\r\n", ret);
        if (ret == AVDK_ERR_INVAL) {
            return OPRT_INVALID_PARM;
        }
        if (ret == AVDK_ERR_NOMEM) {
            return OPRT_MALLOC_FAILED;
        }
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

static VOID tkl_venc_bond_stop(TUYA_VENC_CTX_T *ctx)
{
    if (ctx->bond_handle == NULL) {
        return;
    }

    if (ctx->backend_type == TKL_VENC_BACKEND_H264) {
        switch (ctx->source.type) {
        case TUYA_VENC_SOURCE_ISP_FLEXA:
            bk_flexa_isp_h264e_bond_stop(ctx->bond_handle);
            break;
        case TUYA_VENC_SOURCE_VDEC_FLEXA:
            bk_flexa_mjpegd_h264e_bond_stop(ctx->bond_handle);
            break;
        default:
            break;
        }
    } else if (ctx->backend_type == TKL_VENC_BACKEND_JPEG) {
        bk_flexa_isp_jpege_bond_stop(ctx->bond_handle);
    }

    ctx->bond_handle = NULL;
}

static OPERATE_RET tkl_venc_backend_close(TUYA_VENC_CTX_T *ctx)
{
    avdk_err_t ret;

    if (ctx == NULL) {
        return OPRT_OK;
    }

    if (ctx->backend_opened == FALSE) {
        return OPRT_OK;
    }

    if (ctx->backend_type == TKL_VENC_BACKEND_H264) {
        if (ctx->backend.h264 == NULL) {
            return OPRT_OK;
        }

        ret = bk_h264_encode_close(ctx->backend.h264);
        if (ret != AVDK_ERR_OK) {
            tkl_log_output("tkl_venc: bk_h264_encode_close failed (%d)\r\n", ret);
            return OPRT_COM_ERROR;
        }
    } else if (ctx->backend_type == TKL_VENC_BACKEND_JPEG) {
        if (ctx->backend.jpeg == NULL) {
            return OPRT_OK;
        }

        ret = bk_jpeg_encode_close(ctx->backend.jpeg);
        if (ret != AVDK_ERR_OK) {
            tkl_log_output("tkl_venc: bk_jpeg_encode_close failed (%d)\r\n", ret);
            return OPRT_COM_ERROR;
        }
    }

    ctx->backend_opened = FALSE;
    return OPRT_OK;
}

static VOID tkl_venc_backend_destroy(TUYA_VENC_CTX_T *ctx)
{
    if (ctx == NULL) {
        return;
    }

    if (ctx->backend_type == TKL_VENC_BACKEND_H264 && ctx->backend.h264 != NULL) {
        (void)bk_h264_encode_deinit(ctx->backend.h264);
        (void)bk_h264_encode_delete(ctx->backend.h264);
        ctx->backend.h264 = NULL;
    } else if (ctx->backend_type == TKL_VENC_BACKEND_JPEG && ctx->backend.jpeg != NULL) {
        (void)bk_jpeg_encode_deinit(ctx->backend.jpeg);
        (void)bk_jpeg_encode_delete(ctx->backend.jpeg);
        ctx->backend.jpeg = NULL;
    }

    ctx->backend_type = TKL_VENC_BACKEND_NONE;
    ctx->backend_opened = FALSE;
}

static BOOL_T tkl_venc_backend_ready(const TUYA_VENC_CTX_T *ctx)
{
    if (ctx == NULL) {
        return FALSE;
    }

    if (ctx->backend_type == TKL_VENC_BACKEND_H264) {
        return (ctx->backend.h264 != NULL) ? TRUE : FALSE;
    }

    if (ctx->backend_type == TKL_VENC_BACKEND_JPEG) {
        return (ctx->backend.jpeg != NULL) ? TRUE : FALSE;
    }

    return FALSE;
}

OPERATE_RET tkl_venc_init(TUYA_VENC_HANDLE *handle, const TUYA_VENC_CFG_T *config)
{
    TUYA_VENC_CTX_T *ctx = NULL;

    if ((handle == NULL) || (config == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (*handle != NULL) {
        return OPRT_INVALID_PARM;
    }

    ctx = (TUYA_VENC_CTX_T *)tkl_system_malloc(sizeof(TUYA_VENC_CTX_T));
    if (ctx == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    tkl_system_memset(ctx, 0, sizeof(TUYA_VENC_CTX_T));
    tkl_system_memcpy(&ctx->cfg, config, sizeof(TUYA_VENC_CFG_T));
    if (ctx->cfg.codec == TUYA_VCODEC_TYPE_NONE) {
        ctx->cfg.codec = TUYA_VCODEC_TYPE_H264;
    }
    if (tkl_venc_is_h264_codec(ctx->cfg.codec) == TRUE) {
        tkl_venc_h264_rate_ctrl_default(&ctx->rate_ctrl);
    }
    *handle = (TUYA_VENC_HANDLE)ctx;

    return OPRT_OK;
}

OPERATE_RET tkl_venc_deinit(TUYA_VENC_HANDLE handle)
{
    TUYA_VENC_CTX_T *ctx = tkl_venc_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_venc_stop(handle);
    if (ret != OPRT_OK && ret != OPRT_NOT_SUPPORTED) {
        return ret;
    }

    tkl_venc_backend_destroy(ctx);
    tkl_system_free(ctx);
    return OPRT_OK;
}

OPERATE_RET tkl_venc_set_source(TUYA_VENC_HANDLE handle, const TUYA_VENC_SOURCE_T *source)
{
    TUYA_VENC_CTX_T *ctx = tkl_venc_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == TRUE) {
        return OPRT_COM_ERROR;
    }

    ret = tkl_venc_source_validate(source);
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_venc_bond_stop(ctx);
    tkl_venc_backend_destroy(ctx);

    tkl_system_memcpy(&ctx->source, source, sizeof(TUYA_VENC_SOURCE_T));

    ret = tkl_venc_backend_create(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_venc_set_sink(TUYA_VENC_HANDLE handle, const TUYA_VENC_SINK_T *sink)
{
    TUYA_VENC_CTX_T *ctx = tkl_venc_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == TRUE) {
        return OPRT_COM_ERROR;
    }

    ret = tkl_venc_sink_validate(sink);
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_system_memcpy(&ctx->sink, sink, sizeof(TUYA_VENC_SINK_T));
    return OPRT_OK;
}

OPERATE_RET tkl_venc_start(TUYA_VENC_HANDLE handle)
{
    TUYA_VENC_CTX_T *ctx = tkl_venc_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == TRUE) {
        return OPRT_OK;
    }

    ret = tkl_venc_source_validate(&ctx->source);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_venc_sink_validate(&ctx->sink);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (tkl_venc_backend_ready(ctx) != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_venc_backend_open(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_venc_bond_start(ctx);
    if (ret != OPRT_OK) {
        (void)tkl_venc_backend_close(ctx);
        return ret;
    }

    ctx->started = TRUE;
    return OPRT_OK;
}

OPERATE_RET tkl_venc_stop(TUYA_VENC_HANDLE handle)
{
    TUYA_VENC_CTX_T *ctx = tkl_venc_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == FALSE &&
        tkl_venc_backend_ready(ctx) != TRUE &&
        ctx->bond_handle == NULL) {
        return OPRT_OK;
    }

    if (ctx->started == FALSE) {
        tkl_venc_bond_stop(ctx);
        return OPRT_OK;
    }

    if (ctx->backend_type == TKL_VENC_BACKEND_H264 && ctx->backend.h264 != NULL) {
        (void)bk_h264_encode_ioctl(ctx->backend.h264, BK_H264_ENCODE_IOCTL_STOP_ENCODE, NULL);
    } else if (ctx->backend_type == TKL_VENC_BACKEND_JPEG && ctx->backend.jpeg != NULL) {
        (void)bk_jpeg_encode_ioctl(ctx->backend.jpeg, BK_JPEG_ENCODE_IOCTL_STOP_ENCODE, NULL);
    }

    tkl_venc_bond_stop(ctx);

    if (tkl_venc_backend_ready(ctx) == TRUE) {
        tkl_system_sleep(TKL_VENC_STOP_DRAIN_MS);
    }

    ret = tkl_venc_backend_close(ctx);
    ctx->started = FALSE;
    return ret;
}

/**
 * @brief Update H264 rate control
 * @param[in] handle encoder handle
 * @param[in] rate_ctrl rate control parameters
 * @return OPRT_OK on success
 * @note May be called before start to preset RC; applied to hardware at backend open.
 */
OPERATE_RET tkl_venc_set_rate_ctrl(TUYA_VENC_HANDLE handle, const TUYA_VENC_RATE_CTRL_T *rate_ctrl)
{
    TUYA_VENC_CTX_T *ctx = tkl_venc_ctx_get(handle);

    if (ctx == NULL || rate_ctrl == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->backend_type != TKL_VENC_BACKEND_H264 &&
        tkl_venc_is_h264_codec(ctx->cfg.codec) != TRUE) {
        return OPRT_NOT_SUPPORTED;
    }

    return tkl_venc_h264_apply_rate_ctrl(ctx, rate_ctrl, TRUE);
}

OPERATE_RET tkl_venc_request_idr(TUYA_VENC_HANDLE handle)
{
    TUYA_VENC_CTX_T *ctx = tkl_venc_ctx_get(handle);
    avdk_err_t ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->backend_type != TKL_VENC_BACKEND_H264 || ctx->backend.h264 == NULL) {
        return OPRT_NOT_SUPPORTED;
    }

    ret = bk_h264_encode_force_idr(ctx->backend.h264);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_venc: force idr failed (%d)\r\n", ret);
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_venc_submit_frame(TUYA_VENC_HANDLE handle, const TUYA_VCODEC_FRAME_T *frame)
{
    TUYA_VENC_CTX_T *ctx = tkl_venc_ctx_get(handle);

    if ((ctx == NULL) || (frame == NULL)) {
        return OPRT_INVALID_PARM;
    }

    (void)frame;

    if (ctx->source.type != TUYA_VENC_SOURCE_USER_FRAME) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_venc_get_handle(TUYA_VENC_HANDLE handle, VOID **backend_handle)
{
    TUYA_VENC_CTX_T *ctx = tkl_venc_ctx_get(handle);

    if ((ctx == NULL) || (backend_handle == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (tkl_venc_backend_ready(ctx) != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (ctx->backend_type == TKL_VENC_BACKEND_H264) {
        *backend_handle = (VOID *)ctx->backend.h264;
    } else {
        *backend_handle = (VOID *)ctx->backend.jpeg;
    }

    return OPRT_OK;
}

/**
 * @brief Convert NV12 to packed YUYV
 * @param[in] width frame width, must be even
 * @param[in] height frame height, must be even
 * @param[in] y_plane Y plane base
 * @param[in] uv_plane interleaved UV plane base
 * @param[out] yuyv packed YUYV output buffer
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_venc_jpeg_sw_nv12_to_yuyv(UINT16_T width,
                                                   UINT16_T height,
                                                   const UINT8_T *y_plane,
                                                   const UINT8_T *uv_plane,
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
        const UINT8_T *y_row0 = y_plane + y * y_stride;
        const UINT8_T *y_row1 = y_row0 + y_stride;
        const UINT8_T *uv_row = uv_plane + (y >> 1U) * y_stride;
        UINT8_T *out_row0 = yuyv + y * yuyv_stride;
        UINT8_T *out_row1 = out_row0 + yuyv_stride;
        UINT32_T x = 0U;

        for (; x + 8U <= (UINT32_T)width; x += 8U) {
            const UINT8_T *uv = uv_row + x;
            UINT8_T *d0 = out_row0 + (x * 2U);
            UINT8_T *d1 = out_row1 + (x * 2U);
            UINT32_T pack0;
            UINT32_T pack1;

            /* tjpg expects YVYU (Y0 V Y1 U); NV12 chroma is U V. */
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

/**
 * @brief Resolve software JPEG quality factor
 * @param[in] quality input quality, 0 selects default
 * @return resolved quality in 1..100
 */
STATIC UINT8_T __tkl_venc_jpeg_sw_quality_resolve(UINT8_T quality)
{
    if (quality == 0U) {
        return (UINT8_T)TKL_VENC_JPEG_SW_DEFAULT_QUALITY;
    }

    if (quality > 100U) {
        return 100U;
    }

    return quality;
}

/**
 * @brief JPEG software encode outbuf completion callback
 * @param[in] info encode output buffer info
 * @return BK_OK
 */
STATIC UINT32_T __tkl_venc_jpeg_sw_outbuf_complete(bk_jpeg_encode_outbuf_info_t *info)
{
    if (info != NULL && info->args != NULL && info->status == BK_OK) {
        *(UINT32_T *)info->args = info->length;
    }

    return BK_OK;
}

/**
 * @brief Encode packed YUYV to JPEG using bk_jpeg_encode_sw_frame
 * @param[in] width frame width
 * @param[in] height frame height
 * @param[in] yuyv packed YUYV input buffer
 * @param[in] quality quality factor 1..100
 * @param[out] out caller-owned JPEG output descriptor
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_venc_jpeg_sw_encode_yuyv(UINT16_T width,
                                                  UINT16_T height,
                                                  UINT8_T *yuyv,
                                                  UINT8_T quality,
                                                  TUYA_VENC_JPEG_SW_OUT_T *out)
{
    bk_jpeg_encode_sw_frame_config_t enc_cfg;
    bk_jpeg_encode_input_t enc_input;
    bk_jpeg_encode_ctlr_handle_t enc_handle = NULL;
    UINT32_T encoded_size = 0U;
    avdk_err_t ret;

    if (yuyv == NULL || out == NULL || out->buffer == NULL || out->buf_size == 0U) {
        return OPRT_INVALID_PARM;
    }

    enc_cfg = (bk_jpeg_encode_sw_frame_config_t) {
        .width = width,
        .height = height,
        .input_format = BK_PIXEL_FORMAT_YUYV,
        .quality = quality,
        .outbuf_complete = __tkl_venc_jpeg_sw_outbuf_complete,
        .outbuf_complete_args = &encoded_size,
    };
    enc_input = (bk_jpeg_encode_input_t) {
        .pic_buf = (UINT32_T)(uintptr_t)yuyv,
        .out_buf = (UINT32_T)(uintptr_t)out->buffer,
        .out_size = out->buf_size,
    };

    ret = bk_jpeg_encode_sw_frame_new(&enc_handle, &enc_cfg);
    if (ret == AVDK_ERR_OK) {
        ret = bk_jpeg_encode_init(enc_handle);
    }
    if (ret == AVDK_ERR_OK) {
        ret = bk_jpeg_encode_open(enc_handle);
    }
    if (ret == AVDK_ERR_OK) {
        ret = bk_jpeg_encode_frame(enc_handle, &enc_input);
    }
    if (enc_handle != NULL) {
        (void)bk_jpeg_encode_close(enc_handle);
        (void)bk_jpeg_encode_deinit(enc_handle);
        (void)bk_jpeg_encode_delete(enc_handle);
    }

    if (ret != AVDK_ERR_OK || encoded_size == 0U) {
        tkl_log_output("tkl_venc: jpeg sw encode failed ret=%d size=%u\r\n",
                       (INT_T)ret, encoded_size);
        return OPRT_COM_ERROR;
    }

    out->size = encoded_size;
    return OPRT_OK;
}

/**
 * @brief Validate JPEG software encode input parameters
 * @param[in] in input descriptor
 * @param[out] y_plane resolved Y plane pointer
 * @param[out] uv_plane resolved UV plane pointer for NV12
 * @param[out] yuyv_direct direct YUYV pointer when format is YUYV
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_venc_jpeg_sw_validate_in(const TUYA_VENC_JPEG_SW_IN_T *in,
                                                  const UINT8_T **y_plane,
                                                  const UINT8_T **uv_plane,
                                                  const UINT8_T **yuyv_direct)
{
    UINT32_T y_plane_size;
    UINT32_T nv12_min_size;
    UINT32_T yuyv_min_size;
    const UINT8_T *y = NULL;
    const UINT8_T *uv = NULL;

    if (in == NULL || y_plane == NULL || uv_plane == NULL || yuyv_direct == NULL) {
        return OPRT_INVALID_PARM;
    }

    *y_plane = NULL;
    *uv_plane = NULL;
    *yuyv_direct = NULL;

    if (in->buffer == NULL || in->size == 0U || in->width == 0U || in->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if ((in->width & 1U) != 0U || (in->height & 1U) != 0U) {
        return OPRT_INVALID_PARM;
    }

    y_plane_size = (UINT32_T)in->width * (UINT32_T)in->height;
    if (y_plane_size == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (in->format == (UINT32_T)BK_PIXEL_FORMAT_YUYV) {
        yuyv_min_size = y_plane_size * 2U;
        if (in->size < yuyv_min_size) {
            return OPRT_INVALID_PARM;
        }
        *yuyv_direct = in->buffer;
        return OPRT_OK;
    }

    if (in->format != (UINT32_T)BK_PIXEL_FORMAT_NV12) {
        return OPRT_NOT_SUPPORTED;
    }

    nv12_min_size = y_plane_size + (y_plane_size / 2U);
    if (in->size < nv12_min_size) {
        return OPRT_INVALID_PARM;
    }

    if (in->y_addr != 0U) {
        y = (const UINT8_T *)(uintptr_t)in->y_addr;
    } else {
        y = in->buffer;
    }

    if (in->u_addr != 0U) {
        uv = (const UINT8_T *)(uintptr_t)in->u_addr;
    } else {
        uv = y + y_plane_size;
    }

    if (uv < y) {
        return OPRT_INVALID_PARM;
    }

    *y_plane = y;
    *uv_plane = uv;
    return OPRT_OK;
}

/**
 * @brief One-shot JPEG software encode
 * @param[in]  in   input pixel frame
 * @param[out] out  caller-owned JPEG buffer; out->size filled on success
 * @return OPRT_OK on success
 * @note NV12 is converted to YUYV internally, then encoded via bk_jpeg_encode_sw_frame.
 */
OPERATE_RET tkl_venc_jpeg_sw_encode(const TUYA_VENC_JPEG_SW_IN_T *in,
                                    TUYA_VENC_JPEG_SW_OUT_T *out)
{
    OPERATE_RET op_ret;
    const UINT8_T *y_plane = NULL;
    const UINT8_T *uv_plane = NULL;
    const UINT8_T *yuyv_direct = NULL;
    UINT8_T *yuyv_tmp = NULL;
    UINT8_T *yuyv_encode = NULL;
    UINT32_T yuyv_size;
    UINT8_T quality;

    if (out == NULL) {
        return OPRT_INVALID_PARM;
    }

    out->size = 0U;
    if (out->buffer == NULL || out->buf_size == 0U) {
        return OPRT_INVALID_PARM;
    }

    op_ret = __tkl_venc_jpeg_sw_validate_in(in, &y_plane, &uv_plane, &yuyv_direct);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    quality = __tkl_venc_jpeg_sw_quality_resolve(in->quality);

    if (yuyv_direct != NULL) {
        yuyv_encode = (UINT8_T *)yuyv_direct;
    } else {
        yuyv_size = (UINT32_T)in->width * (UINT32_T)in->height * 2U;
        yuyv_tmp = (UINT8_T *)tkl_system_psram_malloc(yuyv_size);
        if (yuyv_tmp == NULL) {
            return OPRT_MALLOC_FAILED;
        }

        op_ret = __tkl_venc_jpeg_sw_nv12_to_yuyv(in->width, in->height,
                                                 y_plane, uv_plane, yuyv_tmp);
        if (op_ret != OPRT_OK) {
            tkl_system_psram_free(yuyv_tmp);
            return op_ret;
        }

        yuyv_encode = yuyv_tmp;
    }

    op_ret = __tkl_venc_jpeg_sw_encode_yuyv(in->width, in->height,
                                            yuyv_encode, quality, out);
    if (yuyv_tmp != NULL) {
        tkl_system_psram_free(yuyv_tmp);
        yuyv_tmp = NULL;
    }

    if (op_ret != OPRT_OK) {
        out->size = 0U;
    }

    return op_ret;
}

/**
 * @brief Convert tightly packed NV12 to packed YUYV used by soft JPEG
 * @param[in] width frame width, must be even
 * @param[in] height frame height, must be even
 * @param[in] nv12 tightly packed NV12 (stride == width)
 * @param[in] nv12_size NV12 buffer size
 * @param[out] yuyv packed output buffer
 * @param[in] yuyv_size YUYV buffer capacity
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_venc_nv12_to_yuyv(UINT16_T width,
                                  UINT16_T height,
                                  const UINT8_T *nv12,
                                  UINT32_T nv12_size,
                                  UINT8_T *yuyv,
                                  UINT32_T yuyv_size)
{
    UINT32_T y_plane_size;
    UINT32_T need_nv12;
    UINT32_T need_yuyv;

    if (nv12 == NULL || yuyv == NULL ||
        (width & 1U) != 0U || (height & 1U) != 0U) {
        return OPRT_INVALID_PARM;
    }

    y_plane_size = (UINT32_T)width * (UINT32_T)height;
    need_nv12 = y_plane_size + (y_plane_size / 2U);
    need_yuyv = y_plane_size * 2U;
    if (nv12_size < need_nv12 || yuyv_size < need_yuyv) {
        return OPRT_INVALID_PARM;
    }

    return __tkl_venc_jpeg_sw_nv12_to_yuyv(width, height,
                                           nv12,
                                           nv12 + y_plane_size,
                                           yuyv);
}
