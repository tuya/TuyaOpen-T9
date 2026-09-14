/**
 * @file tkl_osd_engine.c
 * @brief TKL adapter - OSD slot mapping for T9 (BK7259).
 *
 * The caller supplies complete ARGB8888 bitmaps. This module only validates
 * geometry, maps destination coordinates to hardware slots and submits them.
 */

#include "tuya_cloud_types.h"
#include "tkl_osd_engine.h"
#include "tkl_media_buf.h"
#include "tkl_output.h"

#include <stdint.h>
#include <os/mem.h>
#include <common/avdk_pixel_types.h>
#include <components/avdk_utils/avdk_error.h>
#include <components/bk_gpu.h>
#include <components/bk_encode/bk_h264_encode_ctlr.h>
#include <components/bk_encode/bk_h264_encode_types.h>

#define TKL_OSD_GPU_SLOT_MAX     4U
#define TKL_OSD_VCENC_SLOT_MAX   8U
#define TKL_OSD_BYTES_PER_PIXEL  4U

/** State used to map a business destination coordinate to a hardware slot. */
typedef struct {
    BOOL_T used;
    UINT16_T dst_x;
    UINT16_T dst_y;
    UINT16_T w;
    UINT16_T h;
    UINT16_T rotate;
} tkl_osd_slot_state_t;

typedef struct {
    TKL_OSD_BACKEND_E backend;
    VOID *bk_handle;
    UINT16_T target_w;
    UINT16_T target_h;
    UINT32_T src_format;
    tkl_osd_slot_state_t slots[TKL_OSD_VCENC_SLOT_MAX];
} tkl_osd_engine_ctx_t;

STATIC tkl_osd_engine_ctx_t *tkl_osd_ctx_get(TKL_OSD_ENGINE_HANDLE handle)
{
    return (tkl_osd_engine_ctx_t *)handle;
}

STATIC UINT8_T tkl_osd_slot_max(TKL_OSD_BACKEND_E backend)
{
    return (backend == TKL_OSD_BACKEND_GPU) ? TKL_OSD_GPU_SLOT_MAX : TKL_OSD_VCENC_SLOT_MAX;
}

STATIC OPERATE_RET tkl_osd_map_err(avdk_err_t ret)
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
    return OPRT_COM_ERROR;
}

/** Business owns overlay data; the backend callback must never release it. */
STATIC VOID_T tkl_osd_business_data_done(VOID_T *buffer, VOID_T *args)
{
    (void)buffer;
    (void)args;
}

/** Transparent buffers used only to clear one GPU slot while others remain. */
STATIC VOID_T tkl_osd_clear_buffer_free(VOID_T *buffer, VOID_T *args)
{
    (void)args;
    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_OSD, buffer);
}

STATIC INT_T tkl_osd_find_slot(CONST tkl_osd_engine_ctx_t *ctx, UINT16_T dst_x, UINT16_T dst_y)
{
    UINT8_T i;

    for (i = 0U; i < tkl_osd_slot_max(ctx->backend); i++) {
        if (ctx->slots[i].used == TRUE && ctx->slots[i].dst_x == dst_x &&
            ctx->slots[i].dst_y == dst_y) {
            return (INT_T)i;
        }
    }

    return -1;
}

STATIC INT_T tkl_osd_find_free_slot(const tkl_osd_engine_ctx_t *ctx)
{
    UINT8_T i;

    for (i = 0U; i < tkl_osd_slot_max(ctx->backend); i++) {
        if (ctx->slots[i].used != TRUE) {
            return (INT_T)i;
        }
    }

    return -1;
}

STATIC BOOL_T tkl_osd_any_slot_used(CONST tkl_osd_engine_ctx_t *ctx)
{
    UINT8_T i;

    for (i = 0U; i < tkl_osd_slot_max(ctx->backend); i++) {
        if (ctx->slots[i].used == TRUE) {
            return TRUE;
        }
    }

    return FALSE;
}

/** Resolve the destination footprint of a GPU rotation. */
STATIC OPERATE_RET tkl_osd_gpu_geometry(CONST tkl_osd_engine_ctx_t *ctx,
                                        CONST TUYA_OSD_T *osd,
                                        UINT16_T *dst_x, UINT16_T *dst_y,
                                        UINT16_T *foot_w, UINT16_T *foot_h)
{
    INT_T mapped_x;
    INT_T mapped_y;
    INT_T geometry_w;
    INT_T geometry_h;

    if (osd->rotate == 0U) {
        mapped_x = (INT_T)osd->dst_x;
        mapped_y = (INT_T)osd->dst_y;
        geometry_w = (INT_T)osd->w;
        geometry_h = (INT_T)osd->h;
    } else if (osd->rotate == 90U) {
        mapped_x = (INT_T)ctx->target_w - (INT_T)osd->dst_y - (INT_T)osd->h;
        mapped_y = (INT_T)osd->dst_x;
        geometry_w = (INT_T)osd->h;
        geometry_h = (INT_T)osd->w;
    } else if (osd->rotate == 270U) {
        mapped_x = (INT_T)osd->dst_y;
        mapped_y = (INT_T)ctx->target_h - (INT_T)osd->dst_x - (INT_T)osd->w;
        geometry_w = (INT_T)osd->h;
        geometry_h = (INT_T)osd->w;
    } else {
        return OPRT_NOT_SUPPORTED;
    }

    /* Check the complete signed footprint before narrowing to UINT16_T. */
    if (mapped_x < 0 || mapped_y < 0 || geometry_w <= 0 || geometry_h <= 0 ||
        mapped_x >= (INT_T)ctx->target_w || mapped_y >= (INT_T)ctx->target_h ||
        mapped_x + geometry_w > (INT_T)ctx->target_w ||
        mapped_y + geometry_h > (INT_T)ctx->target_h) {
        return OPRT_INVALID_PARM;
    }

    *dst_x = (UINT16_T)mapped_x;
    *dst_y = (UINT16_T)mapped_y;
    *foot_w = (UINT16_T)geometry_w;
    *foot_h = (UINT16_T)geometry_h;
    return OPRT_OK;
}

STATIC OPERATE_RET tkl_osd_submit_gpu(tkl_osd_engine_ctx_t *ctx,
                                      CONST TUYA_OSD_T *osd, UINT8_T slot)
{
    bk_gpu_blit_config_t blit;
    UINT16_T dst_x;
    UINT16_T dst_y;
    UINT16_T foot_w;
    UINT16_T foot_h;
    avdk_err_t ret;

    ret = tkl_osd_gpu_geometry(ctx, osd, &dst_x, &dst_y, &foot_w, &foot_h);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&blit, 0, sizeof(blit));
    blit.sprite_width = osd->w;
    blit.sprite_height = osd->h;
    blit.src_x = 0U;
    blit.src_y = 0U;
    blit.src_width = osd->w;
    blit.src_height = osd->h;
    blit.src_format = (bk_pixel_format_t)ctx->src_format;
    blit.dst_x = dst_x;
    blit.dst_y = dst_y;
    blit.rotate_degree = osd->rotate;
    blit.alpha_blend = 1U;
    blit.osd_slot = slot;
    blit.args = NULL;
    blit.free = tkl_osd_business_data_done;

    (void)foot_w;
    (void)foot_h;
    ret = bk_gpu_blit_set((bk_gpu_ctlr_handle_t)ctx->bk_handle,
                          (VOID *)osd->data, &blit);
    return tkl_osd_map_err(ret);
}

STATIC OPERATE_RET tkl_osd_submit_vcenc(tkl_osd_engine_ctx_t *ctx,
                                        CONST TUYA_OSD_T *osd, UINT8_T slot)
{
    bk_h264_encode_osd_t bk_osd;
    avdk_err_t ret;

    if (osd->rotate != 0U) {
        return OPRT_NOT_SUPPORTED;
    }
    if (ctx->target_w != 0U && ctx->target_h != 0U &&
        ((UINT32_T)osd->dst_x + osd->w > ctx->target_w ||
         (UINT32_T)osd->dst_y + osd->h > ctx->target_h)) {
        return OPRT_INVALID_PARM;
    }
    if ((osd->w & 1U) != 0U || (osd->h & 1U) != 0U) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&bk_osd, 0, sizeof(bk_osd));
    bk_osd.index = slot;
    bk_osd.buffer = (VOID *)osd->data;
    bk_osd.format = BK_H264_ENCODE_OVERLAY_FORMAT_ARGB8888;
    bk_osd.x = osd->dst_x;
    bk_osd.y = osd->dst_y;
    bk_osd.width = osd->w;
    bk_osd.height = osd->h;
    bk_osd.buffer_free = tkl_osd_business_data_done;
    bk_osd.free_arg = NULL;

    ret = bk_h264_encode_set_osd((bk_h264_encode_ctlr_handle_t)ctx->bk_handle, &bk_osd);
    return tkl_osd_map_err(ret);
}

STATIC OPERATE_RET tkl_osd_gpu_blank_slot(tkl_osd_engine_ctx_t *ctx, UINT8_T slot)
{
    bk_gpu_blit_config_t blit;
    TUYA_OSD_T old_osd;
    uint32_t *buffer;
    UINT32_T bytes;
    avdk_err_t ret;
    UINT16_T dst_x;
    UINT16_T dst_y;
    UINT16_T foot_w;
    UINT16_T foot_h;
    UINT16_T w = ctx->slots[slot].w;
    UINT16_T h = ctx->slots[slot].h;

    if (w == 0U || h == 0U) {
        return OPRT_OK;
    }

    os_memset(&old_osd, 0, sizeof(old_osd));
    old_osd.dst_x = ctx->slots[slot].dst_x;
    old_osd.dst_y = ctx->slots[slot].dst_y;
    old_osd.w = w;
    old_osd.h = h;
    old_osd.rotate = ctx->slots[slot].rotate;
    ret = tkl_osd_gpu_geometry(ctx, &old_osd, &dst_x, &dst_y, &foot_w, &foot_h);
    if (ret != OPRT_OK) {
        return ret;
    }

    bytes = (UINT32_T)w * h * TKL_OSD_BYTES_PER_PIXEL;
    buffer = (uint32_t *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL, bytes,
                                              TUYA_MEDIA_BUF_OWNER_OSD);
    if (buffer == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    os_memset(buffer, 0, bytes);

    os_memset(&blit, 0, sizeof(blit));
    blit.sprite_width = w;
    blit.sprite_height = h;
    blit.src_width = w;
    blit.src_height = h;
    blit.src_format = (bk_pixel_format_t)ctx->src_format;
    blit.dst_x = dst_x;
    blit.dst_y = dst_y;
    blit.rotate_degree = old_osd.rotate;
    blit.alpha_blend = 1U;
    blit.osd_slot = slot;
    blit.free = tkl_osd_clear_buffer_free;

    (void)foot_w;
    (void)foot_h;
    ret = bk_gpu_blit_set((bk_gpu_ctlr_handle_t)ctx->bk_handle, buffer, &blit);
    if (ret != AVDK_ERR_OK) {
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_OSD, buffer);
        return tkl_osd_map_err(ret);
    }

    return OPRT_OK;
}

STATIC OPERATE_RET tkl_osd_clear_slot(tkl_osd_engine_ctx_t *ctx, UINT8_T slot)
{
    avdk_err_t ret;

    if (ctx->backend == TKL_OSD_BACKEND_VCENC) {
        bk_h264_encode_osd_t bk_osd;

        os_memset(&bk_osd, 0, sizeof(bk_osd));
        bk_osd.index = slot;
        bk_osd.buffer = NULL;
        ret = bk_h264_encode_set_osd((bk_h264_encode_ctlr_handle_t)ctx->bk_handle, &bk_osd);
        if (ret == AVDK_ERR_OK) {
            ctx->slots[slot].used = FALSE;
        }
        return tkl_osd_map_err(ret);
    }

    if (ctx->slots[slot].used != TRUE) {
        return OPRT_OK;
    }

    ctx->slots[slot].used = FALSE;
    if (tkl_osd_any_slot_used(ctx) != TRUE) {
        ret = bk_gpu_blit_clear((bk_gpu_ctlr_handle_t)ctx->bk_handle);
        if (ret != AVDK_ERR_OK) {
            ctx->slots[slot].used = TRUE;
        }
        return tkl_osd_map_err(ret);
    }

    ret = tkl_osd_gpu_blank_slot(ctx, slot);
    if (ret != OPRT_OK) {
        ctx->slots[slot].used = TRUE;
    }
    return ret;
}

OPERATE_RET tkl_osd_engine_open(TKL_OSD_BACKEND_E backend,
                                VOID *bk_handle,
                                CONST TKL_OSD_ENGINE_CFG_T *cfg,
                                TKL_OSD_ENGINE_HANDLE *handle)
{
    tkl_osd_engine_ctx_t *ctx;

    if (bk_handle == NULL || cfg == NULL || handle == NULL ||
        (backend != TKL_OSD_BACKEND_GPU && backend != TKL_OSD_BACKEND_VCENC) ||
        cfg->target_w == 0U || cfg->target_h == 0U) {
        return OPRT_INVALID_PARM;
    }

    ctx = (tkl_osd_engine_ctx_t *)os_malloc(sizeof(*ctx));
    if (ctx == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    os_memset(ctx, 0, sizeof(*ctx));
    ctx->backend = backend;
    ctx->bk_handle = bk_handle;
    ctx->target_w = cfg->target_w;
    ctx->target_h = cfg->target_h;
    ctx->src_format = cfg->src_format;
    if (ctx->src_format == 0U ||
        ctx->src_format == (UINT32_T)BK_PIXEL_FORMAT_INVALID ||
        ctx->src_format == (UINT32_T)BK_PIXEL_FORMAT_UNKNOW) {
        ctx->src_format = (UINT32_T)BK_PIXEL_FORMAT_ARGB8888;
    }

    *handle = (TKL_OSD_ENGINE_HANDLE)ctx;
    return OPRT_OK;
}

OPERATE_RET tkl_osd_engine_close(TKL_OSD_ENGINE_HANDLE handle)
{
    tkl_osd_engine_ctx_t *ctx = tkl_osd_ctx_get(handle);

    if (ctx == NULL) {
        return OPRT_OK;
    }

    if (ctx->backend == TKL_OSD_BACKEND_GPU) {
        (void)bk_gpu_blit_clear((bk_gpu_ctlr_handle_t)ctx->bk_handle);
    } else {
        UINT8_T i;
        for (i = 0U; i < TKL_OSD_VCENC_SLOT_MAX; i++) {
            (void)tkl_osd_clear_slot(ctx, i);
        }
    }

    os_memset(ctx->slots, 0, sizeof(ctx->slots));
    os_free(ctx);
    return OPRT_OK;
}

OPERATE_RET tkl_osd_engine_submit(TKL_OSD_ENGINE_HANDLE handle, CONST TUYA_OSD_T *osd)
{
    tkl_osd_engine_ctx_t *ctx = tkl_osd_ctx_get(handle);
    INT_T slot;
    OPERATE_RET ret;

    if (ctx == NULL || osd == NULL) {
        return OPRT_INVALID_PARM;
    }

    slot = tkl_osd_find_slot(ctx, osd->dst_x, osd->dst_y);
    if (osd->data == NULL) {
        if (slot < 0) {
            return OPRT_OK;
        }
        ret = tkl_osd_clear_slot(ctx, (UINT8_T)slot);
        if (ret == OPRT_OK) {
            os_memset(&ctx->slots[slot], 0, sizeof(ctx->slots[slot]));
        }
        return ret;
    }

    if (osd->w == 0U || osd->h == 0U) {
        return OPRT_INVALID_PARM;
    }
    if (ctx->backend == TKL_OSD_BACKEND_GPU) {
        UINT16_T x;
        UINT16_T y;
        UINT16_T w;
        UINT16_T h;
        ret = tkl_osd_gpu_geometry(ctx, osd, &x, &y, &w, &h);
        if (ret != OPRT_OK) {
            return ret;
        }
    } else {
        if (osd->rotate != 0U) {
            return OPRT_NOT_SUPPORTED;
        }
        if ((osd->w & 1U) != 0U || (osd->h & 1U) != 0U ||
            (UINT32_T)osd->dst_x + osd->w > ctx->target_w ||
            (UINT32_T)osd->dst_y + osd->h > ctx->target_h) {
            return OPRT_INVALID_PARM;
        }
    }

    if (slot < 0) {
        slot = tkl_osd_find_free_slot(ctx);
        if (slot < 0) {
            return OPRT_NOT_SUPPORTED;
        }
    }

    if (ctx->backend == TKL_OSD_BACKEND_GPU) {
        ret = tkl_osd_submit_gpu(ctx, osd, (UINT8_T)slot);
    } else {
        ret = tkl_osd_submit_vcenc(ctx, osd, (UINT8_T)slot);
    }
    if (ret != OPRT_OK) {
        return ret;
    }

    ctx->slots[slot].used = TRUE;
    ctx->slots[slot].dst_x = osd->dst_x;
    ctx->slots[slot].dst_y = osd->dst_y;
    ctx->slots[slot].w = osd->w;
    ctx->slots[slot].h = osd->h;
    ctx->slots[slot].rotate = osd->rotate;
    return OPRT_OK;
}
