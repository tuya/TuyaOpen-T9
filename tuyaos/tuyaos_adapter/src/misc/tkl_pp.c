/**
 * @file tkl_pp.c
 * @brief TKL adapter - frame-mode image processor over bk_pp (T9 / BK7259).
 *
 * @details
 * Thin wrapper over `bk_pp_ctlr`. It exists so that the frame-mode capability
 * limits (NV12 in; no rotate/mirror/compress) and the output-size rounding rules
 * live in exactly one place instead of being rediscovered by every caller.
 *
 * Unlike the GPU, PP instances really are independent: `pp_ctlr_init()` goes
 * through `hw_decoder_register()`, an unbounded refcounted list, and the shared
 * `hw_decoder` worker serialises the hardware for free.
 *
 * @version 0.1
 * @date 2026-08-27
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_pp.h"
#include "tkl_output.h"

#include <os/os.h>
#include <os/mem.h>
#include <common/bk_err.h>
#include <common/avdk_pixel_types.h>
#include <components/avdk_utils/avdk_error.h>
#include <components/bk_decode/bk_pp_ctlr.h>

typedef struct {
    bk_pp_ctlr_handle_t pp_handle;
    beken_mutex_t lock;
    BOOL_T opened;
} tkl_pp_ctx_t;

STATIC tkl_pp_ctx_t *tkl_pp_ctx_get(TKL_PP_HANDLE handle)
{
    return (tkl_pp_ctx_t *)handle;
}

STATIC OPERATE_RET tkl_pp_map_err(avdk_err_t ret)
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

UINT32_T tkl_pp_output_size(UINT32_T format, UINT16_T width, UINT16_T height)
{
    UINT32_T stored_h;

    if (width == 0U || height == 0U) {
        return 0U;
    }

    /* mirrors pp_ctlr_process(): NV12 stores a height that keeps both planes on
     * whole 16-row DMA segments, RGB rounds up to a 16-row block. The NV12 half
     * is subtle: the UV plane is height/2 rows, so height itself must be a
     * multiple of 32 - 320x240 (UV = 120 rows = 7.5 segments) is NOT enough and
     * the PP DMA overruns the tail of the buffer (board-verified: DATA_TAIL slab
     * assert on free). 320x256/288 with UV = 128/144 rows pass. The vendor's own
     * bk_decode_pp_output_size() check accepts the smaller number, so sizing
     * right is the adapter's job. */
    switch (format) {
    case BK_PIXEL_FORMAT_RGB565:
        stored_h = ((UINT32_T)height + 15U) & ~15U;
        return (UINT32_T)width * stored_h * 2U;

    case BK_PIXEL_FORMAT_RGB888:
        stored_h = ((UINT32_T)height + 15U) & ~15U;
        /* the PP writes 4 bytes per pixel for RGB888, not 3 */
        return (UINT32_T)width * stored_h * 4U;

    case BK_PIXEL_FORMAT_NV12:
        stored_h = ((UINT32_T)height + 31U) & ~31U;
        return (UINT32_T)width * stored_h * 3U / 2U;

    default:
        return 0U;
    }
}

OPERATE_RET tkl_pp_open(UINT32_T timeout_ms, TKL_PP_HANDLE *handle)
{
    tkl_pp_ctx_t *ctx;
    bk_pp_config_t cfg;
    avdk_err_t ret;

    if (handle == NULL) {
        return OPRT_INVALID_PARM;
    }

    ctx = (tkl_pp_ctx_t *)os_malloc(sizeof(tkl_pp_ctx_t));
    if (ctx == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    os_memset(ctx, 0, sizeof(*ctx));

    if (rtos_init_mutex(&ctx->lock) != BK_OK) {
        tkl_log_output("tkl_pp: init mutex failed\r\n");
        os_free(ctx);
        return OPRT_COM_ERROR;
    }

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.timeout_ms = timeout_ms;
    cfg.done_cb = NULL;
    cfg.done_args = NULL;

    ret = bk_pp_ctlr_new(&ctx->pp_handle, &cfg);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_pp: bk_pp_ctlr_new failed: %d\r\n", ret);
        goto err_mutex;
    }

    ret = bk_pp_init(ctx->pp_handle);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_pp: bk_pp_init failed: %d\r\n", ret);
        goto err_delete;
    }

    ret = bk_pp_open(ctx->pp_handle);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_pp: bk_pp_open failed: %d\r\n", ret);
        (void)bk_pp_deinit(ctx->pp_handle);
        goto err_delete;
    }

    ctx->opened = TRUE;
    *handle = (TKL_PP_HANDLE)ctx;
    return OPRT_OK;

err_delete:
    (void)bk_pp_delete(ctx->pp_handle);
    ctx->pp_handle = NULL;
err_mutex:
    (void)rtos_deinit_mutex(&ctx->lock);
    os_free(ctx);
    return tkl_pp_map_err(ret);
}

OPERATE_RET tkl_pp_close(TKL_PP_HANDLE handle)
{
    tkl_pp_ctx_t *ctx = tkl_pp_ctx_get(handle);
    OPERATE_RET ret = OPRT_OK;

    if (ctx == NULL) {
        return OPRT_OK;
    }

    if (ctx->pp_handle != NULL) {
        avdk_err_t step;

        if (ctx->opened == TRUE) {
            (void)bk_pp_close(ctx->pp_handle);
            ctx->opened = FALSE;
        }

        step = bk_pp_deinit(ctx->pp_handle);
        if (step != AVDK_ERR_OK) {
            tkl_log_output("tkl_pp: bk_pp_deinit failed: %d\r\n", step);
            ret = tkl_pp_map_err(step);
        }

        step = bk_pp_delete(ctx->pp_handle);
        if (step != AVDK_ERR_OK && ret == OPRT_OK) {
            tkl_log_output("tkl_pp: bk_pp_delete failed: %d\r\n", step);
            ret = tkl_pp_map_err(step);
        }

        ctx->pp_handle = NULL;
    }

    (void)rtos_deinit_mutex(&ctx->lock);
    os_free(ctx);
    return ret;
}

OPERATE_RET tkl_pp_process(TKL_PP_HANDLE handle, CONST TKL_PP_IN_T *in, CONST TKL_PP_OUT_T *out)
{
    tkl_pp_ctx_t *ctx = tkl_pp_ctx_get(handle);
    bk_pp_process_req_t req;
    avdk_err_t ret;

    if (ctx == NULL || ctx->pp_handle == NULL || in == NULL || out == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (in->y == NULL || in->uv == NULL || in->width == 0U || in->height == 0U ||
        out->buffer == NULL || out->size == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (out->format != BK_PIXEL_FORMAT_NV12 &&
        out->format != BK_PIXEL_FORMAT_RGB565 &&
        out->format != BK_PIXEL_FORMAT_RGB888) {
        tkl_log_output("tkl_pp: out format %u unsupported (NV12/RGB565/RGB888)\r\n",
                       (unsigned int)out->format);
        return OPRT_NOT_SUPPORTED;
    }

    os_memset(&req, 0, sizeof(req));
    req.in_y = in->y;
    req.in_c = in->uv;
    req.in_width = in->width;
    req.in_height = in->height;
    req.in_format = BK_PIXEL_FORMAT_NV12;   /* the only input the hardware takes */
    req.out_width = out->width;
    req.out_height = out->height;
    req.out_buffer = out->buffer;
    req.out_size = out->size;
    req.out_format = out->format;

    /* one request at a time per handle: bk_pp_ctlr stashes the request inside the
     * controller before handing it to the shared hw_decoder worker */
    if (rtos_lock_mutex(&ctx->lock) != BK_OK) {
        return OPRT_COM_ERROR;
    }

    ret = bk_pp_process(ctx->pp_handle, &req);

    (void)rtos_unlock_mutex(&ctx->lock);

    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_pp: process failed: %d (in=%ux%u out=%ux%u fmt=%u size=%u)\r\n",
                       ret,
                       in->width,
                       in->height,
                       out->width,
                       out->height,
                       (unsigned int)out->format,
                       (unsigned int)out->size);
    }

    return tkl_pp_map_err(ret);
}
