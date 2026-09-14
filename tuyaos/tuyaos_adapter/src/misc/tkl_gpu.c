/**
 * @file tkl_gpu.c
 * @brief TKL adapter - GPU (vg_lite) flexa pipeline for T9 (BK7259).
 *
 * @details
 * One TKL_GPU_HANDLE == one bk_gpu controller instance. The context is heap
 * allocated so a peripheral facade can own its own pipeline instead of sharing
 * a file-level singleton.
 *
 * **At most one controller may be live**, enforced by s_backend_live. Although
 * bk_gpu_ctlr_new() mallocs a fresh controller per call, bring-up is not
 * instance-local: bk_gpu_driver_init/deinit guard on a plain bool rather than a
 * refcount, so a second instance's teardown unregisters the GPU ISR and votes
 * the power domain off underneath the first. vg_lite's global context has the
 * same problem.
 *
 * @version 0.3
 * @date 2026-08-27
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tkl_gpu_internal.h"

#include <stdbool.h>
#include <stdint.h>
#include <common/avdk_pixel_types.h>
#include <components/avdk_utils/avdk_error.h>
#include <components/bk_flexa_bond.h>
#include <components/bk_gpu.h>
#include <components/bk_gpu_ctlr.h>
#include <os/mem.h>
#include <os/str.h>
#include "tkl_output.h"
#include "tkl_media_buf.h"

typedef struct {
    BOOL_T started;
    BOOL_T sink_ready;
    BOOL_T source_ready;
    BOOL_T backend_opened;
    TUYA_GPU_CFG_T cfg;
    TUYA_GPU_SINK_T sink;
    TUYA_GPU_SOURCE_T source;
    bk_gpu_ctlr_config_t gpu_cfg;       /* what the live controller was built with */
    bk_gpu_ctlr_handle_t gpu_handle;
    VOID *bond_handle;
} tkl_gpu_ctx_t;

/**
 * The one context that currently owns a bk_gpu controller.
 *
 * Needed because bk_gpu_ctlr_config_t::frame_malloc / frame_free are
 * `void *(*)(uint32_t)` / `avdk_err_t (*)(void *)` - no context argument - so
 * the sink they dispatch to cannot be reached from the callback alone. Safe
 * only because at most one controller is live at a time; see the file header.
 */
static tkl_gpu_ctx_t *s_active_ctx = NULL;
static UINT8_T s_backend_live = 0U;

static tkl_gpu_ctx_t *tkl_gpu_ctx_get(TKL_GPU_HANDLE handle)
{
    return (tkl_gpu_ctx_t *)handle;
}

static void tkl_gpu_cfg_normalize(TUYA_GPU_CFG_T *cfg)
{
    if (cfg->dst_format == BK_PIXEL_FORMAT_INVALID ||
        cfg->dst_format == BK_PIXEL_FORMAT_UNKNOW) {
        cfg->dst_format = BK_PIXEL_FORMAT_ARGB8888;
    }

    if (cfg->flexa_lines == 0U) {
        cfg->flexa_lines = 16U;
    }
}

static OPERATE_RET tkl_gpu_cfg_validate(const TUYA_GPU_CFG_T *cfg)
{
    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->rotate_degree != 0U &&
        cfg->rotate_degree != 90U &&
        cfg->rotate_degree != 180U &&
        cfg->rotate_degree != 270U) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->dst_width == 0U || cfg->dst_height == 0U) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static OPERATE_RET tkl_gpu_sink_validate(const TUYA_GPU_SINK_T *sink)
{
    if (sink == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (sink->frame_done_cb == NULL) {
        return OPRT_INVALID_PARM;
    }

    /* alloc_cb/free_cb are optional and all-or-nothing. Leave both NULL and TKL
     * takes the output frame from the write-through pixel pool itself; install
     * both to place the frame in your own bucket (what the camera/uvc facades
     * do, so close() can drain per peripheral). Half a pair is a bug: the frame
     * would come from one allocator and go back to another. */
    if ((sink->alloc_cb == NULL) != (sink->free_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static OPERATE_RET tkl_gpu_flexa_validate(const TUYA_GPU_FLEXA_BUFFER_T *flexa)
{
    if (flexa == NULL ||
        flexa->src_buffer == NULL ||
        flexa->buf_cnt == 0U ||
        flexa->width == 0U ||
        flexa->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (flexa->format == BK_PIXEL_FORMAT_INVALID ||
        flexa->format == BK_PIXEL_FORMAT_UNKNOW) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static OPERATE_RET tkl_gpu_source_validate(const TUYA_GPU_SOURCE_T *source)
{
    OPERATE_RET ret;

    if (source == NULL) {
        return OPRT_INVALID_PARM;
    }

    switch (source->type) {
    case TUYA_GPU_SOURCE_ISP_FLEXA:
        if (source->isp.isp_handle == NULL) {
            return OPRT_INVALID_PARM;
        }
        ret = tkl_gpu_flexa_validate(&source->isp.flexa);
        if (ret != OPRT_OK) {
            return ret;
        }
        break;

    case TUYA_GPU_SOURCE_MJPEG_FLEXA:
        if (source->mjpeg.jpeg_handle == NULL) {
            return OPRT_INVALID_PARM;
        }
        ret = tkl_gpu_flexa_validate(&source->mjpeg.flexa);
        if (ret != OPRT_OK) {
            return ret;
        }
        break;

    default:
        return OPRT_NOT_SUPPORTED;
    }

    return OPRT_OK;
}

static const TUYA_GPU_FLEXA_BUFFER_T *tkl_gpu_source_flexa_get(const TUYA_GPU_SOURCE_T *source)
{
    if (source == NULL) {
        return NULL;
    }

    switch (source->type) {
    case TUYA_GPU_SOURCE_ISP_FLEXA:
        return &source->isp.flexa;
    case TUYA_GPU_SOURCE_MJPEG_FLEXA:
        return &source->mjpeg.flexa;
    default:
        return NULL;
    }
}

static void *tkl_gpu_frame_malloc(uint32_t size)
{
    tkl_gpu_ctx_t *ctx = s_active_ctx;

    if (ctx == NULL) {
        return NULL;
    }

    if (ctx->sink.alloc_cb != NULL) {
        return ctx->sink.alloc_cb(size, ctx->sink.cb_arg);
    }

    /* The GPU writes this frame and the DSI reads it from another port, so it
     * needs a PSRAM write-through mapping - the one place in the tree that
     * genuinely does. tkl_media_buf owns the channel and the 64-byte rounding
     * the vendor pool demands (see tkl_media_buf.h). */
    return tkl_media_buf_alloc(TKL_BUF_USE_PIXEL_WT, size, TUYA_MEDIA_BUF_OWNER_GPU);
}

static OPERATE_RET tkl_gpu_frame_free(VOID *ptr)
{
    tkl_gpu_ctx_t *ctx = s_active_ctx;

    if (ctx == NULL || ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->sink.free_cb != NULL) {
        return ctx->sink.free_cb(ptr, ctx->sink.cb_arg);
    }

    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_GPU, ptr);
    return OPRT_OK;
}

static void tkl_gpu_frame_done(void *frame, uint32_t frame_size, void *args)
{
    tkl_gpu_ctx_t *ctx = (tkl_gpu_ctx_t *)args;

    if (ctx != NULL && ctx->sink.frame_done_cb != NULL) {
        ctx->sink.frame_done_cb(frame, frame_size, ctx->sink.cb_arg);
    }
}

static void tkl_gpu_release_bond(tkl_gpu_ctx_t *ctx)
{
    if (ctx->bond_handle == NULL) {
        return;
    }

    switch (ctx->source.type) {
    case TUYA_GPU_SOURCE_ISP_FLEXA:
        bk_flexa_isp_gpu_bond_stop(ctx->bond_handle);
        break;

    case TUYA_GPU_SOURCE_MJPEG_FLEXA:
        bk_flexa_mjpegd_gpu_bond_stop(ctx->bond_handle);
        break;

    default:
        break;
    }

    ctx->bond_handle = NULL;
}

static OPERATE_RET tkl_gpu_backend_close(tkl_gpu_ctx_t *ctx)
{
    OPERATE_RET ret = OPRT_OK;
    OPERATE_RET step_ret;

    if (ctx->gpu_handle == NULL) {
        return OPRT_OK;
    }

    if (ctx->backend_opened != TRUE) {
        return OPRT_OK;
    }

    if (ctx->source.type == TUYA_GPU_SOURCE_ISP_FLEXA) {
        (void)bk_gpu_ioctl(ctx->gpu_handle, BK_GPU_IOCTL_FLEXA_ADDR_UNMAPPING, NULL);
    }

    step_ret = bk_gpu_close(ctx->gpu_handle);
    if (step_ret != OPRT_OK && ret == OPRT_OK) {
        tkl_log_output( "bk_gpu_close failed: %d\r\n", step_ret);
        ret = step_ret;
    }

    ctx->backend_opened = FALSE;
    return ret;
}

static OPERATE_RET tkl_gpu_backend_destroy(tkl_gpu_ctx_t *ctx)
{
    OPERATE_RET ret = OPRT_OK;
    OPERATE_RET step_ret;

    if (ctx->gpu_handle == NULL) {
        return OPRT_OK;
    }

    step_ret = bk_gpu_deinit(ctx->gpu_handle);
    if (step_ret != OPRT_OK && ret == OPRT_OK) {
        tkl_log_output( "bk_gpu_deinit failed: %d\r\n", step_ret);
        ret = step_ret;
    }

    step_ret = bk_gpu_delete(ctx->gpu_handle);
    if (step_ret != OPRT_OK && ret == OPRT_OK) {
        tkl_log_output( "bk_gpu_delete failed: %d\r\n", step_ret);
        ret = step_ret;
    }

    ctx->gpu_handle = NULL;
    ctx->backend_opened = FALSE;
    os_memset(&ctx->gpu_cfg, 0, sizeof(ctx->gpu_cfg));

    if (s_active_ctx == ctx) {
        s_active_ctx = NULL;
    }
    if (s_backend_live > 0U) {
        s_backend_live--;
    }

    return ret;
}

/**
 * @brief Derive the vendor controller config from cfg + a candidate source.
 * @param[in]  ctx     pipeline context
 * @param[in]  source  candidate source; may differ from ctx->source
 * @param[out] gpu_cfg receives the derived config
 * @return OPRT_OK on success
 * @note Takes the source explicitly so the rebuild decision can be made before
 *       anything is torn down - the teardown path keys off ctx->source and must
 *       still see the *old* one.
 */
static OPERATE_RET tkl_gpu_backend_cfg_build(tkl_gpu_ctx_t *ctx,
                                             const TUYA_GPU_SOURCE_T *source,
                                             bk_gpu_ctlr_config_t *gpu_cfg)
{
    const TUYA_GPU_FLEXA_BUFFER_T *flexa = tkl_gpu_source_flexa_get(source);
    UINT16_T dst_width;
    UINT16_T dst_height;

    if (flexa == NULL) {
        return OPRT_INVALID_PARM;
    }

    dst_width = ctx->cfg.dst_width;
    dst_height = ctx->cfg.dst_height;
    if (dst_width == 0U || dst_height == 0U) {
        tkl_log_output( "gpu dst size is invalid: %ux%u\r\n", dst_width, dst_height);
        return OPRT_INVALID_PARM;
    }

    os_memset(gpu_cfg, 0, sizeof(*gpu_cfg));

    gpu_cfg->rotate_degree = ctx->cfg.rotate_degree;
    gpu_cfg->src_width = flexa->width;
    gpu_cfg->src_height = flexa->height;
    gpu_cfg->dst_width = dst_width;
    gpu_cfg->dst_height = dst_height;
    gpu_cfg->src_format = (bk_pixel_format_t)flexa->format;
    gpu_cfg->dst_format = (bk_pixel_format_t)ctx->cfg.dst_format;
    gpu_cfg->scale = (ctx->cfg.scale != FALSE ||
                      flexa->width != dst_width ||
                      flexa->height != dst_height);
    gpu_cfg->compress = (ctx->cfg.dst_compress != FALSE);
    gpu_cfg->flexa = true;
    gpu_cfg->flexa_lines = ctx->cfg.flexa_lines;
    gpu_cfg->flexa_buff_cnt = flexa->buf_cnt;
    gpu_cfg->src_buffer = flexa->src_buffer;
    gpu_cfg->frame_malloc = tkl_gpu_frame_malloc;
    gpu_cfg->frame_free = tkl_gpu_frame_free;
    gpu_cfg->frame_done = tkl_gpu_frame_done;
    gpu_cfg->frame_done_args = ctx;

    return OPRT_OK;
}

static OPERATE_RET tkl_gpu_backend_create(tkl_gpu_ctx_t *ctx, const bk_gpu_ctlr_config_t *gpu_cfg)
{
    OPERATE_RET ret;
    bk_gpu_ctlr_config_t local_cfg;

    if (s_backend_live != 0U) {
        /* See the file header: bringing a second controller up is fine, but the
         * first tear-down powers the shared GPU domain off under the survivor.
         * Fail loudly here instead of debugging a dead pipeline later. */
        tkl_log_output("tkl_gpu: another GPU pipeline is already live, "
                       "concurrent bk_gpu controllers are not supported\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    local_cfg = *gpu_cfg;

    tkl_log_output(
            "gpu open controller: src=%ux%u fmt=%u -> dst=%ux%u fmt=%u rotate=%u compress=%u scale=%u flexa_lines=%u\r\n",
            local_cfg.src_width,
            local_cfg.src_height,
            (unsigned int)local_cfg.src_format,
            local_cfg.dst_width,
            local_cfg.dst_height,
            (unsigned int)local_cfg.dst_format,
            local_cfg.rotate_degree,
            (unsigned int)local_cfg.compress,
            (unsigned int)local_cfg.scale,
            local_cfg.flexa_lines);

    ret = bk_gpu_ctlr_new(&ctx->gpu_handle, &local_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_gpu_ctlr_new failed: %d\r\n", ret);
        return ret;
    }

    /* published before bk_gpu_init(): the vendor init path may already reach
     * frame_malloc, which has no context argument to find us with */
    s_active_ctx = ctx;
    s_backend_live++;

    ret = bk_gpu_init(ctx->gpu_handle);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_gpu_init failed: %d\r\n", ret);
        (void)bk_gpu_delete(ctx->gpu_handle);
        ctx->gpu_handle = NULL;
        s_active_ctx = NULL;
        s_backend_live--;
        return ret;
    }

    os_memcpy(&ctx->gpu_cfg, &local_cfg, sizeof(ctx->gpu_cfg));
    ctx->backend_opened = FALSE;
    return OPRT_OK;
}

static OPERATE_RET tkl_gpu_backend_open(tkl_gpu_ctx_t *ctx)
{
    OPERATE_RET ret;

    if (ctx->gpu_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (ctx->backend_opened == TRUE) {
        return OPRT_OK;
    }

    ret = bk_gpu_open(ctx->gpu_handle);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_gpu_open failed: %d\r\n", ret);
        return ret;
    }

    ctx->backend_opened = TRUE;
    return OPRT_OK;
}

static OPERATE_RET tkl_gpu_bind_source(tkl_gpu_ctx_t *ctx)
{
    OPERATE_RET ret = OPRT_OK;

    switch (ctx->source.type) {
    case TUYA_GPU_SOURCE_ISP_FLEXA:
        ret = bk_flexa_isp_gpu_bond_start(&ctx->bond_handle,
                                          ctx->source.isp.isp_handle,
                                          ctx->gpu_handle);
        break;

    case TUYA_GPU_SOURCE_MJPEG_FLEXA:
        ret = bk_flexa_mjpegd_gpu_bond_start(&ctx->bond_handle,
                                             (bk_jpeg_decode_ctlr_handle_t)ctx->source.mjpeg.jpeg_handle,
                                             ctx->gpu_handle);
        break;

    default:
        return OPRT_NOT_SUPPORTED;
    }

    if (ret != OPRT_OK) {
        tkl_log_output( "bind source failed: %d\r\n", ret);
        ctx->bond_handle = NULL;
        return ret;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_gpu_init(CONST TUYA_GPU_CFG_T *config, TKL_GPU_HANDLE *handle)
{
    OPERATE_RET ret;
    TUYA_GPU_CFG_T cfg_local;
    tkl_gpu_ctx_t *ctx;

    if (config == NULL || handle == NULL) {
        return OPRT_INVALID_PARM;
    }

    cfg_local = *config;
    tkl_gpu_cfg_normalize(&cfg_local);
    ret = tkl_gpu_cfg_validate(&cfg_local);
    if (ret != OPRT_OK) {
        return ret;
    }

    ctx = (tkl_gpu_ctx_t *)os_malloc(sizeof(tkl_gpu_ctx_t));
    if (ctx == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    os_memset(ctx, 0, sizeof(*ctx));
    os_memcpy(&ctx->cfg, &cfg_local, sizeof(cfg_local));

    *handle = (TKL_GPU_HANDLE)ctx;
    return OPRT_OK;
}

OPERATE_RET tkl_gpu_deinit(TKL_GPU_HANDLE handle)
{
    tkl_gpu_ctx_t *ctx = tkl_gpu_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_OK;
    }

    ret = tkl_gpu_stop(handle);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_gpu_backend_destroy(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_free(ctx);
    return OPRT_OK;
}

OPERATE_RET tkl_gpu_set_sink(TKL_GPU_HANDLE handle, CONST TUYA_GPU_SINK_T *sink)
{
    tkl_gpu_ctx_t *ctx = tkl_gpu_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_gpu_sink_validate(sink);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memcpy(&ctx->sink, sink, sizeof(*sink));
    ctx->sink_ready = TRUE;
    return OPRT_OK;
}

OPERATE_RET tkl_gpu_set_source(TKL_GPU_HANDLE handle, CONST TUYA_GPU_SOURCE_T *source)
{
    tkl_gpu_ctx_t *ctx = tkl_gpu_ctx_get(handle);
    bk_gpu_ctlr_config_t gpu_cfg;
    BOOL_T reuse;
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_gpu_source_validate(source);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (ctx->started == TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* Decide first, tear down second: ctx->source is what tkl_gpu_release_bond()
     * and tkl_gpu_backend_close() dispatch on, so it must still describe the
     * *old* source while they run. */
    ret = tkl_gpu_backend_cfg_build(ctx, source, &gpu_cfg);
    if (ret != OPRT_OK) {
        return ret;
    }

    /* Q4: keep the instance when nothing the controller was built with changed -
     * bk_gpu_ctlr_new() copies its config and offers no ioctl to change geometry,
     * so this is the only case where a rebuild can be skipped. The source type is
     * part of the test because it decides which bond and which close path apply. */
    reuse = (ctx->gpu_handle != NULL &&
             ctx->source.type == source->type &&
             os_memcmp(&ctx->gpu_cfg, &gpu_cfg, sizeof(gpu_cfg)) == 0) ? TRUE : FALSE;

    tkl_gpu_release_bond(ctx);

    ret = tkl_gpu_backend_close(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (reuse != TRUE) {
        ret = tkl_gpu_backend_destroy(ctx);
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    os_memcpy(&ctx->source, source, sizeof(*source));
    ctx->source_ready = TRUE;

    if (reuse != TRUE) {
        ret = tkl_gpu_backend_create(ctx, &gpu_cfg);
        if (ret != OPRT_OK) {
            ctx->source_ready = FALSE;
            return ret;
        }
    }

    return OPRT_OK;
}

OPERATE_RET tkl_gpu_start(TKL_GPU_HANDLE handle)
{
    tkl_gpu_ctx_t *ctx = tkl_gpu_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->sink_ready != TRUE || ctx->source_ready != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (ctx->started == TRUE) {
        return OPRT_OK;
    }

    if (ctx->gpu_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_gpu_backend_open(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_gpu_bind_source(ctx);
    if (ret != OPRT_OK) {
        tkl_gpu_release_bond(ctx);
        (void)tkl_gpu_backend_close(ctx);
        return ret;
    }

    ctx->started = TRUE;
    return OPRT_OK;
}

OPERATE_RET tkl_gpu_stop(TKL_GPU_HANDLE handle)
{
    tkl_gpu_ctx_t *ctx = tkl_gpu_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == FALSE &&
        ctx->gpu_handle == NULL &&
        ctx->bond_handle == NULL) {
        return OPRT_OK;
    }

    /* Q3 (OSD before stream) is enforced by the owning facade, which calls
     * tkl_osd_engine_close() before it gets here - tkl_gpu no longer owns OSD. */
    tkl_gpu_release_bond(ctx);

    ret = tkl_gpu_backend_close(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }

    ctx->started = FALSE;
    return OPRT_OK;
}

OPERATE_RET tkl_gpu_get_handle(TKL_GPU_HANDLE handle, VOID **raw_handle)
{
    tkl_gpu_ctx_t *ctx = tkl_gpu_ctx_get(handle);

    if (ctx == NULL || raw_handle == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started != TRUE || ctx->gpu_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    *raw_handle = ctx->gpu_handle;
    return OPRT_OK;
}
