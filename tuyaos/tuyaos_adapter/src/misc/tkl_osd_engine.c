/**
 * @file tkl_osd_engine.c
 * @brief TKL adapter - shared OSD compositor for T9 (BK7259).
 *
 * @details
 * One composition path, two submit paths. Everything up to "here is an
 * ARGB8888 sprite" is identical for the display and the encode channel; only
 * the final handoff differs (bk_gpu_blit_set vs bk_h264_encode_set_osd).
 *
 * The glyph rasterizers are the vendor's own (bk_osd_emwin_font.c /
 * bk_osd_lv_font.c) - the same ones the vendor's display and H264 overlay paths
 * share, so text renders identically on both channels.
 *
 * Not used: bk_draw_osd_ctlr.c. Its asset table, named-element list and
 * auto-clustering belong to the business layer, not to TKL.
 *
 * @version 0.1
 * @date 2026-08-27
 * @copyright Copyright (c) Tuya Inc.
 */

#include "sdkconfig.h"          /* CONFIG_VG_LITE_GPU gates the LVGL rasterizer */
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
#include "bk_osd_emwin_font.h"
#if CONFIG_VG_LITE_GPU
#include "bk_osd_lv_font.h"
#endif

/** Overlay regions per backend. */
#define TKL_OSD_GPU_SLOT_MAX     4U   /* BK_GPU_BLIT_SLOT_MAX */
#define TKL_OSD_VCENC_SLOT_MAX   8U

/** VCENC wants even overlay geometry; harmless for the GPU path. */
#define TKL_OSD_XY_ALIGN         2U

#define TKL_OSD_BYTES_PER_PIXEL  4U

/** What one slot currently holds; only the GPU backend needs the geometry. */
typedef struct {
    BOOL_T   used;
    UINT16_T w;
    UINT16_T h;
} tkl_osd_slot_state_t;

typedef struct {
    TKL_OSD_BACKEND_E backend;
    VOID *bk_handle;
    UINT16_T target_w;
    UINT16_T target_h;
    UINT16_T rotate;
    UINT32_T src_format;
    tkl_osd_slot_state_t slots[TKL_OSD_VCENC_SLOT_MAX];  /* the larger of the two */
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

/**
 * @brief Release a composed sprite. Signature matches both backends' free hook.
 * @param[in] buffer sprite from tkl_media_buf_alloc()
 * @param[in] args   unused
 */
STATIC VOID_T tkl_osd_sprite_free(VOID_T *buffer, VOID_T *args)
{
    (void)args;
    tkl_media_buf_free(TKL_MEDIA_BUF_OWNER_OSD, buffer);
}

/* ---------------------------------------------------------------------------
 * Composition
 * ------------------------------------------------------------------------- */

/**
 * @brief Pixel extent one element occupies, measured from its own origin.
 * @param[in]  elem  element to measure
 * @param[out] out_w receives width
 * @param[out] out_h receives height
 */
STATIC VOID_T tkl_osd_elem_extent(CONST TKL_OSD_ELEM_T *elem, UINT16_T *out_w, UINT16_T *out_h)
{
    *out_w = 0U;
    *out_h = 0U;

    if (elem->type == TKL_OSD_ELEM_IMAGE) {
        *out_w = elem->w;
        *out_h = elem->h;
        return;
    }

    if (elem->font == NULL || elem->text == NULL) {
        return;
    }

    if (elem->font_kind == TKL_OSD_FONT_LVGL) {
#if CONFIG_VG_LITE_GPU
        osd_lv_font_text_extent((const lv_font_t *)elem->font, elem->text, elem->scale,
                                out_w, out_h);
#endif
        return;
    }

    osd_emwin_font_text_extent((const gui_font_digit_struct *)elem->font, elem->text,
                               out_w, out_h);
}

/**
 * @brief Derive the sprite size that fits every element.
 * @param[in]  elems element array
 * @param[in]  count element count
 * @param[out] out_w receives sprite width
 * @param[out] out_h receives sprite height
 */
STATIC VOID_T tkl_osd_auto_extent(CONST TKL_OSD_ELEM_T *elems, UINT8_T count,
                                  UINT16_T *out_w, UINT16_T *out_h)
{
    UINT32_T max_w = 0U;
    UINT32_T max_h = 0U;
    UINT8_T i;

    for (i = 0U; i < count; i++) {
        UINT16_T ew = 0U;
        UINT16_T eh = 0U;
        UINT32_T right;
        UINT32_T bottom;

        tkl_osd_elem_extent(&elems[i], &ew, &eh);
        if (ew == 0U || eh == 0U) {
            continue;
        }

        right = (UINT32_T)elems[i].x + ew;
        bottom = (UINT32_T)elems[i].y + eh;
        if (right > max_w) {
            max_w = right;
        }
        if (bottom > max_h) {
            max_h = bottom;
        }
    }

    *out_w = (max_w > 0xFFFFU) ? 0xFFFFU : (UINT16_T)max_w;
    *out_h = (max_h > 0xFFFFU) ? 0xFFFFU : (UINT16_T)max_h;
}

/**
 * @brief Copy one ARGB8888 bitmap into the sprite, clipped to its bounds.
 * @param[out] sprite   sprite base, row stride = @p sw
 * @param[in]  sw       sprite width
 * @param[in]  sh       sprite height
 * @param[in]  elem     IMAGE element
 */
STATIC VOID_T tkl_osd_put_image(uint32_t *sprite, UINT16_T sw, UINT16_T sh,
                                CONST TKL_OSD_ELEM_T *elem)
{
    const uint32_t *src = (const uint32_t *)elem->data;
    UINT16_T copy_w;
    UINT16_T copy_h;
    UINT16_T row;

    if (src == NULL || elem->w == 0U || elem->h == 0U) {
        return;
    }

    if (elem->x >= sw || elem->y >= sh) {
        return;
    }

    copy_w = ((UINT32_T)elem->x + elem->w > sw) ? (UINT16_T)(sw - elem->x) : elem->w;
    copy_h = ((UINT32_T)elem->y + elem->h > sh) ? (UINT16_T)(sh - elem->y) : elem->h;

    for (row = 0U; row < copy_h; row++) {
        os_memcpy(&sprite[(UINT32_T)(elem->y + row) * sw + elem->x],
                  &src[(UINT32_T)row * elem->w],
                  (UINT32_T)copy_w * TKL_OSD_BYTES_PER_PIXEL);
    }
}

/**
 * @brief Rasterize one TEXT element into the sprite.
 * @param[out] sprite sprite base, row stride = @p sw
 * @param[in]  sw     sprite width
 * @param[in]  sh     sprite height
 * @param[in]  elem   TEXT element
 */
STATIC VOID_T tkl_osd_put_text(uint32_t *sprite, UINT16_T sw, UINT16_T sh,
                               CONST TKL_OSD_ELEM_T *elem)
{
    if (elem->font == NULL || elem->text == NULL) {
        return;
    }

    if (elem->font_kind == TKL_OSD_FONT_LVGL) {
#if CONFIG_VG_LITE_GPU
        (void)osd_lv_font_blit(sprite, sw, sh,
                               (const lv_font_t *)elem->font, elem->text,
                               elem->x, elem->y, elem->argb, elem->scale, NULL);
#else
        tkl_log_output("tkl_osd: LVGL font needs CONFIG_VG_LITE_GPU\r\n");
#endif
        return;
    }

    (void)osd_emwin_font_blit(sprite, sw, sh,
                              (const gui_font_digit_struct *)elem->font, elem->text,
                              elem->x, elem->y, elem->argb, NULL);
}

/* ---------------------------------------------------------------------------
 * Backend submit
 * ------------------------------------------------------------------------- */

/**
 * @brief Hand a composed sprite to the GPU blit slot.
 *
 * @param[in] ctx    engine context
 * @param[in] osd    slot request (dst_x/dst_y are in the channel's coord space)
 * @param[in] sprite composed sprite; ownership transfers on success
 * @param[in] sw     sprite width
 * @param[in] sh     sprite height
 *
 * @note The rotation mapping is copied from the vendor's osd_engine_commit():
 *       at rotate 90/270 the caller's dst_x/dst_y are pre-rotation viewer
 *       coordinates and have to be mapped into the panel buffer, with the
 *       on-screen footprint's axes swapped. Getting this wrong hands the GPU an
 *       out-of-buffer destination.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_osd_submit_gpu(tkl_osd_engine_ctx_t *ctx, CONST TKL_OSD_T *osd,
                                      uint32_t *sprite, UINT16_T sw, UINT16_T sh)
{
    bk_gpu_blit_config_t blit;
    avdk_err_t ret;
    UINT16_T dst_x;
    UINT16_T dst_y;
    UINT16_T fw;
    UINT16_T fh;

    if (ctx->rotate == 90U) {
        INT_T bx = (INT_T)ctx->target_w - (INT_T)osd->dst_y - (INT_T)sh;
        dst_x = (UINT16_T)((bx < 0) ? 0 : bx);
        dst_y = osd->dst_x;
    } else if (ctx->rotate == 270U) {
        INT_T by = (INT_T)ctx->target_h - (INT_T)osd->dst_x - (INT_T)sw;
        dst_x = osd->dst_y;
        dst_y = (UINT16_T)((by < 0) ? 0 : by);
    } else {
        dst_x = osd->dst_x;
        dst_y = osd->dst_y;
    }

    /* rotated blits swap the on-screen axes */
    fw = (ctx->rotate == 90U || ctx->rotate == 270U) ? sh : sw;
    fh = (ctx->rotate == 90U || ctx->rotate == 270U) ? sw : sh;

    if (dst_x >= ctx->target_w || dst_y >= ctx->target_h) {
        tkl_log_output("tkl_osd: slot %u off-screen (dst=%u,%u panel=%ux%u rot=%u)\r\n",
                       (unsigned int)osd->slot,
                       (unsigned int)dst_x, (unsigned int)dst_y,
                       (unsigned int)ctx->target_w, (unsigned int)ctx->target_h,
                       (unsigned int)ctx->rotate);
        return OPRT_INVALID_PARM;
    }
    if ((UINT32_T)dst_x + fw > ctx->target_w || (UINT32_T)dst_y + fh > ctx->target_h) {
        tkl_log_output("tkl_osd: slot %u exceeds panel (dst=%u,%u fp=%ux%u panel=%ux%u rot=%u)\r\n",
                       (unsigned int)osd->slot,
                       (unsigned int)dst_x, (unsigned int)dst_y,
                       (unsigned int)fw, (unsigned int)fh,
                       (unsigned int)ctx->target_w, (unsigned int)ctx->target_h,
                       (unsigned int)ctx->rotate);
        return OPRT_INVALID_PARM;
    }

    os_memset(&blit, 0, sizeof(blit));
    blit.sprite_width = sw;
    blit.sprite_height = sh;
    blit.src_x = 0U;
    blit.src_y = 0U;
    blit.src_width = sw;
    blit.src_height = sh;
    blit.src_format = (bk_pixel_format_t)ctx->src_format;
    blit.dst_x = dst_x;
    blit.dst_y = dst_y;
    blit.rotate_degree = ctx->rotate;
    blit.alpha_blend = 1U;              /* transparent sprite -> SRC_OVER */
    blit.osd_slot = osd->slot;
    blit.args = NULL;
    blit.free = tkl_osd_sprite_free;

    /* bk_gpu_blit_set() releases whatever sprite the slot held, so periodic
     * re-submits (a clock) do not accumulate */
    ret = bk_gpu_blit_set((bk_gpu_ctlr_handle_t)ctx->bk_handle, sprite, &blit);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_osd: bk_gpu_blit_set failed: %d\r\n", ret);
        return tkl_osd_map_err(ret);
    }

    return OPRT_OK;
}

/**
 * @brief Overwrite one GPU slot with a fully transparent sprite.
 *
 * @param[in] ctx  engine context
 * @param[in] slot slot to blank; must have rendered before
 *
 * @note Used instead of bk_gpu_blit_set(NULL) when other slots must keep
 *       working - see tkl_osd_clear_slot() for why NULL does not clear.
 *       Costs one alpha-zero blit per frame until the slot is reused or the
 *       whole engine is closed.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_osd_gpu_blank_slot(tkl_osd_engine_ctx_t *ctx, UINT8_T slot)
{
    bk_gpu_blit_config_t blit;
    uint32_t *sprite;
    UINT32_T sprite_bytes;
    UINT16_T sw = ctx->slots[slot].w;
    UINT16_T sh = ctx->slots[slot].h;
    avdk_err_t ret;

    if (sw == 0U || sh == 0U) {
        return OPRT_RESOURCE_NOT_READY;
    }

    sprite_bytes = (UINT32_T)sw * sh * TKL_OSD_BYTES_PER_PIXEL;
    sprite = (uint32_t *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL, sprite_bytes,
                                             TKL_MEDIA_BUF_OWNER_OSD);
    if (sprite == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    os_memset(sprite, 0, sprite_bytes);     /* alpha 0 everywhere -> invisible */

    os_memset(&blit, 0, sizeof(blit));
    blit.sprite_width = sw;
    blit.sprite_height = sh;
    blit.src_width = sw;
    blit.src_height = sh;
    blit.src_format = (bk_pixel_format_t)ctx->src_format;
    blit.dst_x = 0U;
    blit.dst_y = 0U;
    blit.rotate_degree = ctx->rotate;
    blit.alpha_blend = 1U;
    blit.osd_slot = slot;
    blit.free = tkl_osd_sprite_free;

    ret = bk_gpu_blit_set((bk_gpu_ctlr_handle_t)ctx->bk_handle, sprite, &blit);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_osd: blank slot %u failed: %d\r\n", (unsigned int)slot, ret);
        tkl_media_buf_free(TKL_MEDIA_BUF_OWNER_OSD, sprite);
        return tkl_osd_map_err(ret);
    }

    return OPRT_OK;
}

/**
 * @brief Hand a composed sprite to a VCENC overlay slot.
 *
 * @param[in] ctx    engine context
 * @param[in] osd    slot request, in un-rotated encoded-frame coordinates
 * @param[in] sprite composed sprite; ownership transfers on success
 * @param[in] sw     sprite width
 * @param[in] sh     sprite height
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_osd_submit_vcenc(tkl_osd_engine_ctx_t *ctx, CONST TKL_OSD_T *osd,
                                        uint32_t *sprite, UINT16_T sw, UINT16_T sh)
{
    bk_h264_encode_osd_t bk_osd;
    avdk_err_t ret;

    if (ctx->target_w != 0U && ctx->target_h != 0U) {
        if (((UINT32_T)osd->dst_x + sw) > ctx->target_w ||
            ((UINT32_T)osd->dst_y + sh) > ctx->target_h) {
            tkl_log_output("tkl_osd: slot %u region %ux%u@(%u,%u) exceeds frame %ux%u\r\n",
                           (unsigned int)osd->slot,
                           (unsigned int)sw, (unsigned int)sh,
                           (unsigned int)osd->dst_x, (unsigned int)osd->dst_y,
                           (unsigned int)ctx->target_w, (unsigned int)ctx->target_h);
            return OPRT_INVALID_PARM;
        }
    }

    os_memset(&bk_osd, 0, sizeof(bk_osd));
    bk_osd.index = osd->slot;
    bk_osd.buffer = sprite;
    bk_osd.format = BK_H264_ENCODE_OVERLAY_FORMAT_ARGB8888;
    bk_osd.x = osd->dst_x;
    bk_osd.y = osd->dst_y;
    bk_osd.width = sw;
    bk_osd.height = sh;
    bk_osd.buffer_free = tkl_osd_sprite_free;
    bk_osd.free_arg = NULL;

    ret = bk_h264_encode_set_osd((bk_h264_encode_ctlr_handle_t)ctx->bk_handle, &bk_osd);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("tkl_osd: bk_h264_encode_set_osd failed: %d\r\n", ret);
        return tkl_osd_map_err(ret);
    }

    return OPRT_OK;
}

/**
 * @brief Whether any slot still holds content.
 * @param[in] ctx engine context
 * @return TRUE when at least one slot is occupied
 */
STATIC BOOL_T tkl_osd_any_slot_used(CONST tkl_osd_engine_ctx_t *ctx)
{
    UINT8_T i;

    for (i = 0U; i < TKL_OSD_VCENC_SLOT_MAX; i++) {
        if (ctx->slots[i].used == TRUE) {
            return TRUE;
        }
    }

    return FALSE;
}

/**
 * @brief Empty one overlay slot.
 *
 * @param[in] ctx  engine context
 * @param[in] slot slot index
 *
 * @note **The two backends clear differently, and the GPU one is a trap.**
 *       VCENC clears on a NULL buffer. The GPU does not: bk_gpu_blit_set(NULL)
 *       only empties update_blit_buffer[slot], and the per-frame compositor
 *       (bk_gpu_ctlr_default.c:1252) promotes update into display only when it
 *       is non-NULL, so the sprite stays frozen on screen.
 *
 *       The GPU path therefore either wipes everything with bk_gpu_blit_clear(),
 *       or - when other slots must survive - hands the slot a fully transparent
 *       sprite so the promote path runs and frees the old one. That sprite
 *       reuses the geometry the slot last rendered at, because vg_lite's source
 *       alignment rules live in a closed library.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_osd_clear_slot(tkl_osd_engine_ctx_t *ctx, UINT8_T slot)
{
    avdk_err_t ret;

    if (ctx->backend != TKL_OSD_BACKEND_GPU) {
        bk_h264_encode_osd_t bk_osd;

        os_memset(&bk_osd, 0, sizeof(bk_osd));
        bk_osd.index = slot;
        bk_osd.buffer = NULL;           /* NULL buffer really clears the slot */

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
        /* nothing left to preserve: this frees both the update and the display
         * buffer of every slot and stops the compositor entirely */
        ret = bk_gpu_blit_clear((bk_gpu_ctlr_handle_t)ctx->bk_handle);
        if (ret != AVDK_ERR_OK) {
            ctx->slots[slot].used = TRUE;
            return tkl_osd_map_err(ret);
        }
        /* bk_gpu_blit_clear() logs nothing of its own; without this the whole
         * wipe is invisible in a trace */
        tkl_log_output("tkl_osd: slot %u cleared, no slot left -> blit_clear\r\n",
                       (unsigned int)slot);
        return OPRT_OK;
    }

    tkl_log_output("tkl_osd: slot %u cleared by transparent sprite (other slots live)\r\n",
                   (unsigned int)slot);
    return tkl_osd_gpu_blank_slot(ctx, slot);
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

OPERATE_RET tkl_osd_engine_open(TKL_OSD_BACKEND_E backend,
                                VOID *bk_handle,
                                CONST TKL_OSD_ENGINE_CFG_T *cfg,
                                TKL_OSD_ENGINE_HANDLE *handle)
{
    tkl_osd_engine_ctx_t *ctx;

    if (bk_handle == NULL || cfg == NULL || handle == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (backend != TKL_OSD_BACKEND_GPU && backend != TKL_OSD_BACKEND_VCENC) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->target_w == 0U || cfg->target_h == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (backend == TKL_OSD_BACKEND_GPU) {
        /* the blit engine implements 0/90/270 only; the pipeline accepts 180 but
         * the overlay has no matching transform */
        if (cfg->rotate != 0U && cfg->rotate != 90U && cfg->rotate != 270U) {
            tkl_log_output("tkl_osd: rotate %u invalid (0/90/270)\r\n",
                           (unsigned int)cfg->rotate);
            return OPRT_INVALID_PARM;
        }
    } else if (cfg->rotate != 0U) {
        tkl_log_output("tkl_osd: encode overlay cannot rotate\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    ctx = (tkl_osd_engine_ctx_t *)os_malloc(sizeof(tkl_osd_engine_ctx_t));
    if (ctx == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    os_memset(ctx, 0, sizeof(*ctx));

    ctx->backend = backend;
    ctx->bk_handle = bk_handle;
    ctx->target_w = cfg->target_w;
    ctx->target_h = cfg->target_h;
    ctx->rotate = cfg->rotate;
    ctx->src_format = cfg->src_format;
    if (ctx->src_format == 0U ||
        ctx->src_format == (UINT32_T)BK_PIXEL_FORMAT_INVALID ||
        ctx->src_format == (UINT32_T)BK_PIXEL_FORMAT_UNKNOW) {
        /* the compositor always writes ARGB8888; a compressed display pipeline
         * wants ABGR8888 instead and must say so explicitly */
        ctx->src_format = (UINT32_T)BK_PIXEL_FORMAT_ARGB8888;
    }

    *handle = (TKL_OSD_ENGINE_HANDLE)ctx;
    return OPRT_OK;
}

OPERATE_RET tkl_osd_engine_close(TKL_OSD_ENGINE_HANDLE handle)
{
    tkl_osd_engine_ctx_t *ctx = tkl_osd_ctx_get(handle);
    OPERATE_RET ret = OPRT_OK;

    if (ctx == NULL) {
        return OPRT_OK;
    }

    if (ctx->backend == TKL_OSD_BACKEND_GPU) {
        /* clears all four slots, update and display, freeing every sprite still
         * held through its free callback - the only GPU call that actually
         * releases display_blit_buffer[] */
        (void)bk_gpu_blit_clear((bk_gpu_ctlr_handle_t)ctx->bk_handle);
    } else {
        UINT8_T slot;

        for (slot = 0U; slot < TKL_OSD_VCENC_SLOT_MAX; slot++) {
            (void)tkl_osd_clear_slot(ctx, slot);
        }
    }

    os_memset(ctx->slots, 0, sizeof(ctx->slots));

    os_free(ctx);
    return ret;
}

OPERATE_RET tkl_osd_engine_submit(TKL_OSD_ENGINE_HANDLE handle, CONST TKL_OSD_T *osd)
{
    tkl_osd_engine_ctx_t *ctx = tkl_osd_ctx_get(handle);
    uint32_t *sprite;
    UINT32_T sprite_bytes;
    UINT16_T sw;
    UINT16_T sh;
    UINT8_T i;
    OPERATE_RET ret;

    if (ctx == NULL || osd == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (osd->slot >= tkl_osd_slot_max(ctx->backend)) {
        tkl_log_output("tkl_osd: slot %u exceeds backend max %u\r\n",
                       (unsigned int)osd->slot,
                       (unsigned int)tkl_osd_slot_max(ctx->backend));
        return OPRT_NOT_SUPPORTED;
    }

    if (osd->rotate != ctx->rotate) {
        tkl_log_output("tkl_osd: slot rotate %u differs from channel rotate %u\r\n",
                       (unsigned int)osd->rotate,
                       (unsigned int)ctx->rotate);
        return OPRT_INVALID_PARM;
    }

    if (osd->elems == NULL || osd->count == 0U) {
        return tkl_osd_clear_slot(ctx, osd->slot);
    }

    sw = osd->w;
    sh = osd->h;
    if (sw == 0U || sh == 0U) {
        UINT16_T auto_w = 0U;
        UINT16_T auto_h = 0U;

        tkl_osd_auto_extent(osd->elems, osd->count, &auto_w, &auto_h);
        if (sw == 0U) {
            sw = auto_w;
        }
        if (sh == 0U) {
            sh = auto_h;
        }
    }

    /* VCENC wants even geometry; the extra column/row is transparent */
    sw = (UINT16_T)((sw + (TKL_OSD_XY_ALIGN - 1U)) & ~(TKL_OSD_XY_ALIGN - 1U));
    sh = (UINT16_T)((sh + (TKL_OSD_XY_ALIGN - 1U)) & ~(TKL_OSD_XY_ALIGN - 1U));

    if (sw == 0U || sh == 0U) {
        tkl_log_output("tkl_osd: slot %u has no drawable content\r\n", (unsigned int)osd->slot);
        return OPRT_INVALID_PARM;
    }

    sprite_bytes = (UINT32_T)sw * sh * TKL_OSD_BYTES_PER_PIXEL;

    /* UNCODED is mandatory: the CPU writes this buffer and the GPU / encoder
     * reads it. A cached allocation shows up as intermittent garbage because the
     * D-Cache has not been written back. */
    sprite = (uint32_t *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL, sprite_bytes,
                                             TKL_MEDIA_BUF_OWNER_OSD);
    if (sprite == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    /* fully transparent ground; glyph coverage and icon alpha paint over it */
    os_memset(sprite, 0, sprite_bytes);

    for (i = 0U; i < osd->count; i++) {
        CONST TKL_OSD_ELEM_T *elem = &osd->elems[i];

        if (elem->type == TKL_OSD_ELEM_IMAGE) {
            tkl_osd_put_image(sprite, sw, sh, elem);
        } else {
            tkl_osd_put_text(sprite, sw, sh, elem);
        }
    }

    if (ctx->backend == TKL_OSD_BACKEND_GPU) {
        ret = tkl_osd_submit_gpu(ctx, osd, sprite, sw, sh);
    } else {
        ret = tkl_osd_submit_vcenc(ctx, osd, sprite, sw, sh);
    }

    if (ret != OPRT_OK) {
        /* ownership only transfers on success */
        tkl_media_buf_free(TKL_MEDIA_BUF_OWNER_OSD, sprite);
        return ret;
    }

    /* remembered so tkl_osd_clear_slot() can blank this slot at a geometry the
     * hardware has already accepted */
    ctx->slots[osd->slot].used = TRUE;
    ctx->slots[osd->slot].w = sw;
    ctx->slots[osd->slot].h = sh;

    return OPRT_OK;
}

OPERATE_RET tkl_osd_engine_caps(TKL_OSD_ENGINE_HANDLE handle, TKL_OSD_CAPS_T *caps)
{
    tkl_osd_engine_ctx_t *ctx = tkl_osd_ctx_get(handle);

    if (ctx == NULL || caps == NULL) {
        return OPRT_INVALID_PARM;
    }

    os_memset(caps, 0, sizeof(*caps));
    caps->max_slots = tkl_osd_slot_max(ctx->backend);
    caps->support_text = TRUE;
    caps->support_rotate = (ctx->backend == TKL_OSD_BACKEND_GPU) ? TRUE : FALSE;
    return OPRT_OK;
}
