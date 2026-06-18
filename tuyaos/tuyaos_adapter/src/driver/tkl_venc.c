/**
 * @file tkl_venc.c
 * @brief Video encoder TKL adapter skeleton
 * @version 0.1
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
#include <components/bk_flexa_bond.h>
#include "tkl_memory.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_venc.h"

#define TKL_VENC_STOP_DRAIN_MS 300U

typedef struct {
    BOOL_T started;
    BOOL_T backend_opened;
    TUYA_VENC_CFG_T cfg;
    TUYA_VENC_SOURCE_T source;
    TUYA_VENC_SINK_T sink;
    bk_h264_encode_ctlr_handle_t backend_handle;
    VOID *bond_handle;
} TUYA_VENC_CTX_T;

static TUYA_VENC_CTX_T *tkl_venc_ctx_get(TUYA_VENC_HANDLE handle)
{
    return (TUYA_VENC_CTX_T *)handle;
}

static UINT32_T tkl_venc_gop_resolve(const TUYA_VENC_CTX_T *ctx)
{
    if (ctx->cfg.gop != 0U) {
        return ctx->cfg.gop;
    }

    return 30U;
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

static uint32_t tkl_venc_outbuf_complete(bk_h264_encode_outbuf_info_t *info)
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
        TUYA_VCODEC_STREAM_T stream;

        tkl_system_memset(&stream, 0, sizeof(stream));
        stream.data = (UINT8_T *)payload;
        stream.size = info->length;
        stream.pts = 0;
        stream.codec = ctx->cfg.codec;
        stream.frame_type = tkl_venc_frame_type_convert(info->type);
        stream.priv = payload;

        ctx->sink.stream_cb(&stream, ctx->sink.cb_arg);
    }

    if (ctx->sink.free_cb != NULL) {
        (void)ctx->sink.free_cb(payload, ctx->sink.cb_arg);
    } else {
        tkl_system_free(payload);
    }

    return BK_OK;
}

static OPERATE_RET tkl_venc_backend_create(TUYA_VENC_CTX_T *ctx)
{
    avdk_err_t ret;
    UINT16_T width = 0U;
    UINT16_T height = 0U;

    if (ctx->cfg.codec != TUYA_VCODEC_TYPE_H264) {
        return OPRT_NOT_SUPPORTED;
    }

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
        sw_cfg.outbuf_complete = tkl_venc_outbuf_complete;
        sw_cfg.outbuf_complete_args = ctx;

        ret = bk_h264_encode_sw_flexa_new(&ctx->backend_handle, &sw_cfg);
    } else {
        bk_h264_encode_hw_flexa_config_t hw_cfg;

        tkl_system_memset(&hw_cfg, 0, sizeof(hw_cfg));
        hw_cfg.width = width;
        hw_cfg.height = height;
        hw_cfg.gop_frame_count = tkl_venc_gop_resolve(ctx);
        hw_cfg.input_format = tkl_venc_input_format_resolve(ctx);
        hw_cfg.input_flexa_cnt = tkl_venc_input_cnt_resolve(ctx);
        hw_cfg.input_buf = tkl_venc_input_buf_resolve(ctx);
        hw_cfg.input_size = tkl_venc_input_cnt_resolve(ctx);
        hw_cfg.outbuf_malloc = tkl_venc_outbuf_malloc;
        hw_cfg.outbuf_malloc_args = ctx;
        hw_cfg.outbuf_complete = tkl_venc_outbuf_complete;
        hw_cfg.outbuf_complete_args = ctx;

        ret = bk_h264_encode_hw_flexa_new(&ctx->backend_handle, &hw_cfg);
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

    ret = bk_h264_encode_init(ctx->backend_handle);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_venc: bk_h264_encode_init failed (%d)\r\n", ret);
        (void)bk_h264_encode_delete(ctx->backend_handle);
        ctx->backend_handle = NULL;
        return OPRT_COM_ERROR;
    }

    ctx->backend_opened = FALSE;
    return OPRT_OK;
}

static OPERATE_RET tkl_venc_backend_open(TUYA_VENC_CTX_T *ctx)
{
    avdk_err_t ret;

    if (ctx == NULL || ctx->backend_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = bk_h264_encode_open(ctx->backend_handle);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_venc: bk_h264_encode_open failed (%d)\r\n", ret);
        return OPRT_COM_ERROR;
    }

    ctx->backend_opened = TRUE;
    return OPRT_OK;
}

static OPERATE_RET tkl_venc_bond_start(TUYA_VENC_CTX_T *ctx)
{
    avdk_err_t ret;

    switch (ctx->source.type) {
    case TUYA_VENC_SOURCE_ISP_FLEXA:
        ret = bk_flexa_isp_h264e_bond_start(&ctx->bond_handle,
                                            ctx->source.isp.isp_handle,
                                            ctx->backend_handle);
        break;

    case TUYA_VENC_SOURCE_VDEC_FLEXA:
        ret = bk_flexa_mjpegd_h264e_bond_start(&ctx->bond_handle,
                                               ctx->source.vdec.vdec_handle,
                                               ctx->backend_handle);
        break;

    default:
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

    ctx->bond_handle = NULL;
}

static OPERATE_RET tkl_venc_backend_close(TUYA_VENC_CTX_T *ctx)
{
    avdk_err_t ret;

    if (ctx == NULL || ctx->backend_handle == NULL) {
        return OPRT_OK;
    }

    if (ctx->backend_opened == FALSE) {
        return OPRT_OK;
    }

    ret = bk_h264_encode_close(ctx->backend_handle);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_venc: bk_h264_encode_close failed (%d)\r\n", ret);
        return OPRT_COM_ERROR;
    }

    ctx->backend_opened = FALSE;
    return OPRT_OK;
}

static VOID tkl_venc_backend_destroy(TUYA_VENC_CTX_T *ctx)
{
    if (ctx->backend_handle == NULL) {
        return;
    }

    (void)bk_h264_encode_deinit(ctx->backend_handle);
    (void)bk_h264_encode_delete(ctx->backend_handle);
    ctx->backend_handle = NULL;
    ctx->backend_opened = FALSE;
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

    if (ctx->backend_handle == NULL) {
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

    if (ctx->started == FALSE && ctx->backend_handle == NULL && ctx->bond_handle == NULL) {
        return OPRT_OK;
    }

    if (ctx->started == FALSE) {
        tkl_venc_bond_stop(ctx);
        return OPRT_OK;
    }

    tkl_venc_bond_stop(ctx);
    /*
     * Bond stop must run before backend close because the BK ISP->H264E bond stop
     * path waits for one final frame-end handshake (driven by the still-live encoder
     * frame_done callback) to disable ISP flexa SBI and release its semaphore. If the
     * encoder is closed first, that handshake never completes -> ISP floods
     * "isp_isr_callback error state" until wait_sbi_disabled times out and the next
     * start crashes. After the bond is detached, leave a short drain window for any
     * in-flight encode callback before closing the backend handle.
     */
    if (ctx->backend_handle != NULL) {
        tkl_system_sleep(TKL_VENC_STOP_DRAIN_MS);
    }
    ret = tkl_venc_backend_close(ctx);
    ctx->started = FALSE;
    return ret;
}

OPERATE_RET tkl_venc_request_idr(TUYA_VENC_HANDLE handle)
{
    TUYA_VENC_CTX_T *ctx = tkl_venc_ctx_get(handle);
    avdk_err_t ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->backend_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = bk_h264_encode_force_idr(ctx->backend_handle);
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

    if (ctx->backend_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    *backend_handle = (VOID *)ctx->backend_handle;
    return OPRT_OK;
}
