/**
 * @file tkl_vdec.c
 * @brief Video decoder TKL adapter skeleton
 * @version 0.1
 * @date 2026-04-15
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_memory.h"
#include "tkl_vdec.h"

typedef struct {
    BOOL_T started;
    TUYA_VDEC_CFG_T cfg;
    TUYA_VDEC_PP_T pp;
    TUYA_VDEC_SINK_T sink;
    VOID *backend_handle;
} TUYA_VDEC_CTX_T;

static TUYA_VDEC_CTX_T *tkl_vdec_ctx_get(TUYA_VDEC_HANDLE handle)
{
    return (TUYA_VDEC_CTX_T *)handle;
}

OPERATE_RET tkl_vdec_init(TUYA_VDEC_HANDLE *handle, const TUYA_VDEC_CFG_T *config)
{
    TUYA_VDEC_CTX_T *ctx = NULL;

    if ((handle == NULL) || (config == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (*handle != NULL) {
        return OPRT_INVALID_PARM;
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

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    tkl_system_free(ctx);
    return OPRT_OK;
}

OPERATE_RET tkl_vdec_set_sink(TUYA_VDEC_HANDLE handle, const TUYA_VDEC_SINK_T *sink)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);

    if ((ctx == NULL) || (sink == NULL)) {
        return OPRT_INVALID_PARM;
    }

    tkl_system_memcpy(&ctx->sink, sink, sizeof(TUYA_VDEC_SINK_T));
    return OPRT_OK;
}

OPERATE_RET tkl_vdec_set_pp(TUYA_VDEC_HANDLE handle, const TUYA_VDEC_PP_T *pp)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);

    if ((ctx == NULL) || (pp == NULL)) {
        return OPRT_INVALID_PARM;
    }

    tkl_system_memcpy(&ctx->pp, pp, sizeof(TUYA_VDEC_PP_T));
    return OPRT_OK;
}

OPERATE_RET tkl_vdec_start(TUYA_VDEC_HANDLE handle)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    /* TODO: map decoder type/sink/pp to bk_jpeg_decode_* or h264_decoder_* backend. */
    return OPRT_NOT_SUPPORTED;
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

    /* TODO: stop and close backend decoder. */
    ctx->started = FALSE;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_vdec_reset(TUYA_VDEC_HANDLE handle)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    /* TODO: call jpeg_decoder_reset / matching backend reset api. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_vdec_submit_stream(TUYA_VDEC_HANDLE handle, const TUYA_VCODEC_STREAM_T *stream)
{
    TUYA_VDEC_CTX_T *ctx = tkl_vdec_ctx_get(handle);

    if ((ctx == NULL) || (stream == NULL)) {
        return OPRT_INVALID_PARM;
    }

    /* TODO: feed bitstream to decoder backend. */
    return OPRT_NOT_SUPPORTED;
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

    *backend_handle = ctx->backend_handle;
    return OPRT_OK;
}
