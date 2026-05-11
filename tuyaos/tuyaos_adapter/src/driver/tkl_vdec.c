/**
 * @file tkl_vdec.c
 * @brief Video decoder TKL adapter
 * @version 0.1
 * @date 2026-04-15
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include <stdint.h>
#include <common/avdk_pixel_types.h>
#include <components/avdk_utils/avdk_error.h>
#include <components/bk_decode/bk_jpeg_decode_ctlr.h>
#include "tkl_memory.h"
#include "tkl_vdec.h"

#define TKL_VDEC_DEFAULT_TIMEOUT_MS 1000U
#define TKL_VDEC_DEFAULT_FLEXA_LINES 16U
#define TKL_VDEC_DEFAULT_FLEXA_CNT 3U
#define TKL_VDEC_ALIGN_HEIGHT(height) (((height) + 15U) & ~15U)

typedef struct {
    TUYA_VCODEC_FRAME_T frame;
    VOID *payload;
} TUYA_VDEC_FRAME_NODE_T;

typedef struct {
    BOOL_T started;
    BOOL_T sink_ready;
    BOOL_T backend_inited;
    BOOL_T backend_opened;
    TUYA_VDEC_CFG_T cfg;
    TUYA_VDEC_PP_T pp;
    TUYA_VDEC_SINK_T sink;
    bk_jpeg_decode_ctlr_handle_t backend_handle;
    UINT16_T out_width;
    UINT16_T out_height;
    UINT16_T aligned_height;
    UINT32_T out_format;
    UINT16_T flexa_lines;
} TUYA_VDEC_CTX_T;

static TUYA_VDEC_CTX_T *tkl_vdec_ctx_get(TUYA_VDEC_HANDLE handle)
{
    return (TUYA_VDEC_CTX_T *)handle;
}

static OPERATE_RET tkl_vdec_ret_convert(avdk_err_t ret)
{
    switch (ret) {
    case AVDK_ERR_OK:
        return OPRT_OK;
    case AVDK_ERR_INVAL:
        return OPRT_INVALID_PARM;
    case AVDK_ERR_NOMEM:
        return OPRT_MALLOC_FAILED;
    case AVDK_ERR_UNSUPPORTED:
        return OPRT_NOT_SUPPORTED;
    case AVDK_ERR_TIMEOUT:
        return OPRT_TIMEOUT;
    default:
        return OPRT_COM_ERROR;
    }
}

static BOOL_T tkl_vdec_codec_is_jpeg(UINT32_T codec)
{
    return (codec == TUYA_VCODEC_TYPE_MJPEG || codec == TUYA_VCODEC_TYPE_JPEG);
}

static UINT32_T tkl_vdec_output_format_resolve(const TUYA_VDEC_CTX_T *ctx)
{
    if (ctx->pp.output_format != 0U &&
        ctx->pp.output_format != BK_PIXEL_FORMAT_UNKNOW &&
        ctx->pp.output_format != BK_PIXEL_FORMAT_INVALID) {
        return ctx->pp.output_format;
    }

    if (ctx->cfg.output_format != 0U &&
        ctx->cfg.output_format != BK_PIXEL_FORMAT_UNKNOW &&
        ctx->cfg.output_format != BK_PIXEL_FORMAT_INVALID) {
        return ctx->cfg.output_format;
    }

    return BK_PIXEL_FORMAT_NV12;
}

static OPERATE_RET tkl_vdec_size_resolve(TUYA_VDEC_CTX_T *ctx)
{
    UINT16_T width;
    UINT16_T height;
    UINT16_T flexa_lines;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    width = (ctx->pp.out_width != 0U) ? ctx->pp.out_width : ctx->cfg.width;
    height = (ctx->pp.out_height != 0U) ? ctx->pp.out_height : ctx->cfg.height;
    if (width == 0U || height == 0U) {
        return OPRT_INVALID_PARM;
    }

    ctx->out_width = width;
    ctx->out_height = height;
    ctx->aligned_height = (UINT16_T)TKL_VDEC_ALIGN_HEIGHT(height);
    ctx->out_format = tkl_vdec_output_format_resolve(ctx);
    flexa_lines = (ctx->cfg.flexa_lines != 0U) ?
                  ctx->cfg.flexa_lines :
                  TKL_VDEC_DEFAULT_FLEXA_LINES;

    if (flexa_lines < 16U) {
        flexa_lines = 16U;
    }
    ctx->flexa_lines = (UINT16_T)TKL_VDEC_ALIGN_HEIGHT(flexa_lines);

    if (ctx->out_format != BK_PIXEL_FORMAT_NV12) {
        return OPRT_NOT_SUPPORTED;
    }

    if (ctx->pp.rotate_degree != 0U) {
        return OPRT_NOT_SUPPORTED;
    }

    return OPRT_OK;
}

static UINT32_T tkl_vdec_nv12_frame_size(const TUYA_VDEC_CTX_T *ctx)
{
    return (UINT32_T)ctx->out_width * (UINT32_T)ctx->aligned_height * 3U / 2U;
}

static UINT32_T tkl_vdec_frame_output_size(const TUYA_VDEC_CTX_T *ctx,
                                           const TUYA_VCODEC_STREAM_T *stream)
{
    bk_jpeg_decode_img_info_t info;
    UINT32_T width;
    UINT32_T height;
    UINT32_T size;
    UINT32_T min_size;
    avdk_err_t ret;

    if (ctx == NULL || stream == NULL || ctx->backend_handle == NULL) {
        return 0U;
    }

    min_size = tkl_vdec_nv12_frame_size(ctx);

    tkl_system_memset(&info, 0, sizeof(info));
    info.input_stream = stream->data;
    info.input_stream_length = stream->size;

    ret = bk_jpeg_decode_ioctl(ctx->backend_handle, BK_JPEG_DECODE_IOCTL_GET_INFO, &info);
    if (ret != AVDK_ERR_OK || info.width == 0U || info.height == 0U) {
        return (UINT32_T)ctx->out_width * (UINT32_T)ctx->aligned_height * 3U;
    }

    width = info.width;
    height = TKL_VDEC_ALIGN_HEIGHT(info.height);

    switch (info.format) {
    case BK_JPEG_DECODE_IMG_FMT_YUV400:
        size = width * height;
        break;
    case BK_JPEG_DECODE_IMG_FMT_YUV420:
        size = width * height * 3U / 2U;
        break;
    case BK_JPEG_DECODE_IMG_FMT_YUV422:
        size = width * height * 2U;
        break;
    case BK_JPEG_DECODE_IMG_FMT_YUV444:
    case BK_JPEG_DECODE_IMG_FMT_ERR:
    default:
        size = width * height * 3U;
        break;
    }

    return (size > min_size) ? size : min_size;
}

static UINT32_T tkl_vdec_flexa_size(const TUYA_VDEC_CTX_T *ctx, UINT8_T buf_cnt)
{
    return (UINT32_T)ctx->out_width * (UINT32_T)ctx->flexa_lines * 3U / 2U * (UINT32_T)buf_cnt;
}

static VOID tkl_vdec_flexa_fill(TUYA_VDEC_CTX_T *ctx, TUYA_VCODEC_FLEXA_BUFFER_T *flexa)
{
    UINT32_T base_addr;

    if (ctx == NULL || flexa == NULL) {
        return;
    }

    base_addr = flexa->y_addr;
    if (base_addr == 0U && flexa->src_buffer != NULL) {
        base_addr = (UINT32_T)(uintptr_t)flexa->src_buffer;
    }

    flexa->owner_handle = (VOID *)ctx->backend_handle;
    flexa->src_buffer = (UINT8_T *)(uintptr_t)base_addr;
    flexa->width = ctx->out_width;
    flexa->height = ctx->aligned_height;
    flexa->format = ctx->out_format;
    flexa->y_addr = base_addr;
    flexa->u_addr = base_addr +
                    ((UINT32_T)ctx->out_width *
                     (UINT32_T)ctx->flexa_lines *
                     (UINT32_T)flexa->buf_cnt);
    flexa->v_addr = 0U;
}

static OPERATE_RET tkl_vdec_config_validate(const TUYA_VDEC_CFG_T *config)
{
    if (config == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (!tkl_vdec_codec_is_jpeg(config->codec)) {
        return OPRT_NOT_SUPPORTED;
    }

    if (config->width == 0U || config->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static OPERATE_RET tkl_vdec_sink_validate(const TUYA_VDEC_CTX_T *ctx, const TUYA_VDEC_SINK_T *sink)
{
    if (ctx == NULL || sink == NULL) {
        return OPRT_INVALID_PARM;
    }

    if ((sink->alloc_cb == NULL) != (sink->free_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    switch (sink->type) {
    case TUYA_VDEC_SINK_FRAME:
        if (sink->frame.frame_cb == NULL) {
            return OPRT_INVALID_PARM;
        }
        return OPRT_OK;

    case TUYA_VDEC_SINK_FLEXA:
        if (sink->flexa.flexa.src_buffer == NULL && sink->flexa.flexa.y_addr == 0U) {
            return OPRT_INVALID_PARM;
        }
        if (sink->flexa.flexa.buf_cnt == 0U) {
            return OPRT_INVALID_PARM;
        }
        return OPRT_OK;

    default:
        return OPRT_INVALID_PARM;
    }
}

static void tkl_vdec_jpeg_frame_done_cb(int status, void *args)
{
    (void)status;
    (void)args;
}

static void tkl_vdec_jpeg_flexa_done_cb(uint32_t wr_cnt, void *args)
{
    TUYA_VDEC_CTX_T *ctx = (TUYA_VDEC_CTX_T *)args;

    if (ctx != NULL &&
        ctx->sink.type == TUYA_VDEC_SINK_FLEXA &&
        ctx->sink.flexa.flexa_cb != NULL) {
        ctx->sink.flexa.flexa_cb(wr_cnt, ctx->sink.cb_arg);
    }
}

static OPERATE_RET tkl_vdec_backend_create(TUYA_VDEC_CTX_T *ctx)
{
    avdk_err_t ret;
    OPERATE_RET op_ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->backend_handle != NULL) {
        return OPRT_OK;
    }

    op_ret = tkl_vdec_size_resolve(ctx);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    op_ret = tkl_vdec_sink_validate(ctx, &ctx->sink);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    if (ctx->sink.type == TUYA_VDEC_SINK_FRAME) {
        bk_jpeg_decode_frame_config_t cfg;

        tkl_system_memset(&cfg, 0, sizeof(cfg));
        cfg.timeout_ms = TKL_VDEC_DEFAULT_TIMEOUT_MS;
        cfg.out_width = ctx->out_width;
        cfg.out_height = ctx->out_height;
        cfg.out_format = ctx->out_format;
        cfg.frame_done_cb = tkl_vdec_jpeg_frame_done_cb;
        cfg.frame_done_args = ctx;

        ret = bk_jpeg_decode_frame_ctlr_new(&ctx->backend_handle, &cfg);
    } else {
        bk_jpeg_decode_flexa_config_t cfg;
        UINT8_T buf_cnt = ctx->sink.flexa.flexa.buf_cnt;

        if (buf_cnt == 0U) {
            buf_cnt = TKL_VDEC_DEFAULT_FLEXA_CNT;
        }

        tkl_system_memset(&cfg, 0, sizeof(cfg));
        cfg.timeout_ms = TKL_VDEC_DEFAULT_TIMEOUT_MS;
        cfg.out_width = ctx->out_width;
        cfg.out_height = ctx->out_height;
        cfg.out_format = ctx->out_format;
        cfg.segment_height = (UINT16_T)(ctx->flexa_lines / 16U);
        cfg.segment_number = buf_cnt;
        cfg.frame_done_cb = tkl_vdec_jpeg_frame_done_cb;
        cfg.frame_done_args = ctx;
        cfg.flexa_done_cb = tkl_vdec_jpeg_flexa_done_cb;
        cfg.flexa_done_args = ctx;

        ret = bk_jpeg_decode_flexa_ctlr_new(&ctx->backend_handle, &cfg);
    }

    if (ret != AVDK_ERR_OK) {
        return tkl_vdec_ret_convert(ret);
    }

    ret = bk_jpeg_decode_init(ctx->backend_handle);
    if (ret != AVDK_ERR_OK) {
        (void)bk_jpeg_decode_delete(ctx->backend_handle);
        ctx->backend_handle = NULL;
        return tkl_vdec_ret_convert(ret);
    }

    ctx->backend_inited = TRUE;
    if (ctx->sink.type == TUYA_VDEC_SINK_FLEXA) {
        tkl_vdec_flexa_fill(ctx, &ctx->sink.flexa.flexa);
    }
    return OPRT_OK;
}

static OPERATE_RET tkl_vdec_backend_open(TUYA_VDEC_CTX_T *ctx)
{
    avdk_err_t ret;

    if (ctx == NULL || ctx->backend_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (ctx->backend_opened == TRUE) {
        return OPRT_OK;
    }

    ret = bk_jpeg_decode_open(ctx->backend_handle);
    if (ret != AVDK_ERR_OK) {
        return tkl_vdec_ret_convert(ret);
    }

    ctx->backend_opened = TRUE;
    return OPRT_OK;
}

static OPERATE_RET tkl_vdec_backend_close(TUYA_VDEC_CTX_T *ctx)
{
    avdk_err_t ret;

    if (ctx == NULL || ctx->backend_handle == NULL) {
        return OPRT_OK;
    }

    if (ctx->backend_opened == TRUE) {
        ret = bk_jpeg_decode_close(ctx->backend_handle);
        if (ret != AVDK_ERR_OK) {
            return tkl_vdec_ret_convert(ret);
        }
        ctx->backend_opened = FALSE;
    }

    return OPRT_OK;
}

static OPERATE_RET tkl_vdec_backend_destroy(TUYA_VDEC_CTX_T *ctx)
{
    OPERATE_RET op_ret = OPRT_OK;
    OPERATE_RET close_ret;
    avdk_err_t ret;

    if (ctx == NULL || ctx->backend_handle == NULL) {
        return OPRT_OK;
    }

    close_ret = tkl_vdec_backend_close(ctx);
    if (close_ret != OPRT_OK) {
        op_ret = close_ret;
    }

    if (ctx->backend_inited == TRUE) {
        ret = bk_jpeg_decode_deinit(ctx->backend_handle);
        if (ret != AVDK_ERR_OK && op_ret == OPRT_OK) {
            op_ret = tkl_vdec_ret_convert(ret);
        }
        ctx->backend_inited = FALSE;
    }

    ret = bk_jpeg_decode_delete(ctx->backend_handle);
    if (ret != AVDK_ERR_OK && op_ret == OPRT_OK) {
        op_ret = tkl_vdec_ret_convert(ret);
    }
    ctx->backend_handle = NULL;

    return op_ret;
}

static VOID *tkl_vdec_frame_alloc(TUYA_VDEC_CTX_T *ctx, UINT32_T size)
{
    if (ctx->sink.alloc_cb != NULL) {
        return ctx->sink.alloc_cb(size, ctx->sink.cb_arg);
    }

    return tkl_system_malloc(size);
}

static VOID tkl_vdec_frame_free(TUYA_VDEC_CTX_T *ctx, VOID *ptr)
{
    if (ptr == NULL) {
        return;
    }

    if (ctx->sink.free_cb != NULL) {
        (void)ctx->sink.free_cb(ptr, ctx->sink.cb_arg);
    } else {
        tkl_system_free(ptr);
    }
}

OPERATE_RET tkl_vdec_init(TUYA_VDEC_HANDLE *handle, const TUYA_VDEC_CFG_T *config)
{
    TUYA_VDEC_CTX_T *ctx = NULL;
    OPERATE_RET ret;

    if ((handle == NULL) || (config == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (*handle != NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_vdec_config_validate(config);
    if (ret != OPRT_OK) {
        return ret;
    }

    ctx = (TUYA_VDEC_CTX_T *)tkl_system_malloc(sizeof(TUYA_VDEC_CTX_T));
    if (ctx == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    tkl_system_memset(ctx, 0, sizeof(TUYA_VDEC_CTX_T));
    tkl_system_memcpy(&ctx->cfg, config, sizeof(TUYA_VDEC_CFG_T));
    *handle = (TUYA_VDEC_HANDLE)ctx;

    return OPRT_OK;
}

OPERATE_RET tkl_vdec_deinit(TUYA_VDEC_HANDLE handle)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);
    OPERATE_RET ret;
    OPERATE_RET destroy_ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_vdec_stop(handle);
    destroy_ret = tkl_vdec_backend_destroy(ctx);

    tkl_system_free(ctx);
    return (ret != OPRT_OK) ? ret : destroy_ret;
}

OPERATE_RET tkl_vdec_set_sink(TUYA_VDEC_HANDLE handle, const TUYA_VDEC_SINK_T *sink)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);
    OPERATE_RET ret;

    if ((ctx == NULL) || (sink == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == TRUE) {
        return OPRT_COM_ERROR;
    }

    ret = tkl_vdec_size_resolve(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_vdec_sink_validate(ctx, sink);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_vdec_backend_destroy(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_system_memcpy(&ctx->sink, sink, sizeof(TUYA_VDEC_SINK_T));
    ctx->sink_ready = TRUE;

    ret = tkl_vdec_backend_create(ctx);
    if (ret != OPRT_OK) {
        ctx->sink_ready = FALSE;
        return ret;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_vdec_set_pp(TUYA_VDEC_HANDLE handle, const TUYA_VDEC_PP_T *pp)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);
    TUYA_VDEC_PP_T old_pp;
    OPERATE_RET ret;

    if ((ctx == NULL) || (pp == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == TRUE) {
        return OPRT_COM_ERROR;
    }

    tkl_system_memcpy(&old_pp, &ctx->pp, sizeof(old_pp));
    tkl_system_memcpy(&ctx->pp, pp, sizeof(TUYA_VDEC_PP_T));

    ret = tkl_vdec_size_resolve(ctx);
    if (ret != OPRT_OK) {
        tkl_system_memcpy(&ctx->pp, &old_pp, sizeof(ctx->pp));
        (void)tkl_vdec_size_resolve(ctx);
        return ret;
    }

    if (ctx->sink_ready == TRUE) {
        ret = tkl_vdec_backend_destroy(ctx);
        if (ret != OPRT_OK) {
            return ret;
        }
        ret = tkl_vdec_backend_create(ctx);
        if (ret != OPRT_OK) {
            ctx->sink_ready = FALSE;
            return ret;
        }
    }

    return OPRT_OK;
}

OPERATE_RET tkl_vdec_start(TUYA_VDEC_HANDLE handle)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == TRUE) {
        return OPRT_OK;
    }

    if (ctx->sink_ready != TRUE || ctx->backend_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_vdec_backend_open(ctx);
    if (ret != OPRT_OK) {
        (void)tkl_vdec_backend_close(ctx);
        return ret;
    }

    ctx->started = TRUE;
    return OPRT_OK;
}

OPERATE_RET tkl_vdec_stop(TUYA_VDEC_HANDLE handle)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == FALSE) {
        return OPRT_OK;
    }

    if (ctx->backend_handle != NULL) {
        (void)bk_jpeg_decode_ioctl(ctx->backend_handle, BK_JPEG_DECODE_IOCTL_ABORT, NULL);
    }
    ctx->started = FALSE;
    return tkl_vdec_backend_close(ctx);
}

OPERATE_RET tkl_vdec_reset(TUYA_VDEC_HANDLE handle)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->backend_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    return tkl_vdec_ret_convert(bk_jpeg_decode_ioctl(ctx->backend_handle,
                                                     BK_JPEG_DECODE_IOCTL_ABORT,
                                                     NULL));
}

OPERATE_RET tkl_vdec_submit_stream(TUYA_VDEC_HANDLE handle, const TUYA_VCODEC_STREAM_T *stream)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);
    bk_jpeg_decode_input_t input;
    avdk_err_t ret = AVDK_ERR_INVAL;

    if ((ctx == NULL) || (stream == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == FALSE || ctx->backend_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (stream->data == NULL || stream->size == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (stream->codec != TUYA_VCODEC_TYPE_NONE &&
        stream->codec != TUYA_VCODEC_TYPE_MJPEG &&
        stream->codec != TUYA_VCODEC_TYPE_JPEG) {
        return OPRT_INVALID_PARM;
    }

    tkl_system_memset(&input, 0, sizeof(input));
    input.stream = stream->data;
    input.stream_len = stream->size;

    if (ctx->sink.type == TUYA_VDEC_SINK_FRAME) {
        TUYA_VDEC_FRAME_NODE_T node;
        UINT32_T frame_size = tkl_vdec_nv12_frame_size(ctx);
        UINT32_T output_size = tkl_vdec_frame_output_size(ctx, stream);

        tkl_system_memset(&node, 0, sizeof(node));
        node.payload = tkl_vdec_frame_alloc(ctx, output_size);
        if (node.payload == NULL) {
            return OPRT_MALLOC_FAILED;
        }

        input.out_buffer = (UINT8_T *)node.payload;
        input.out_buffer_size = output_size;

        ret = bk_jpeg_decode_frame(ctx->backend_handle, &input);
        if (ret == AVDK_ERR_OK) {
            node.frame.buffer = (UINT8_T *)node.payload;
            node.frame.size = frame_size;
            node.frame.width = ctx->out_width;
            node.frame.height = ctx->out_height;
            node.frame.stride = ctx->out_width;
            node.frame.slice_height = ctx->aligned_height;
            node.frame.format = ctx->out_format;
            node.frame.y_addr = (UINT32_T)(uintptr_t)node.payload;
            node.frame.u_addr = node.frame.y_addr +
                                ((UINT32_T)ctx->out_width * (UINT32_T)ctx->aligned_height);
            node.frame.v_addr = 0U;
            node.frame.pts = stream->pts;
            node.frame.priv = &node;

            ctx->sink.frame.frame_cb(&node.frame, ctx->sink.cb_arg);
        }

        tkl_vdec_frame_free(ctx, node.payload);
    } else {
        UINT8_T buf_cnt = ctx->sink.flexa.flexa.buf_cnt;

        input.out_buffer = (ctx->sink.flexa.flexa.y_addr != 0U) ?
                           (UINT8_T *)(uintptr_t)ctx->sink.flexa.flexa.y_addr :
                           ctx->sink.flexa.flexa.src_buffer;
        input.out_buffer_size = tkl_vdec_flexa_size(ctx, buf_cnt);

        ret = bk_jpeg_decode_frame(ctx->backend_handle, &input);
    }

    return tkl_vdec_ret_convert(ret);
}

OPERATE_RET tkl_vdec_get_handle(TUYA_VDEC_HANDLE handle, VOID **backend_handle)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);

    if ((ctx == NULL) || (backend_handle == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->backend_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    *backend_handle = (VOID *)ctx->backend_handle;
    return OPRT_OK;
}

OPERATE_RET tkl_vdec_get_flexa_source(TUYA_VDEC_HANDLE handle, TUYA_VCODEC_FLEXA_BUFFER_T *flexa)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);

    if (ctx == NULL || flexa == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == FALSE || ctx->backend_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (ctx->sink.type != TUYA_VDEC_SINK_FLEXA) {
        return OPRT_NOT_SUPPORTED;
    }

    tkl_vdec_flexa_fill(ctx, &ctx->sink.flexa.flexa);
    tkl_system_memcpy(flexa, &ctx->sink.flexa.flexa, sizeof(*flexa));
    return OPRT_OK;
}
