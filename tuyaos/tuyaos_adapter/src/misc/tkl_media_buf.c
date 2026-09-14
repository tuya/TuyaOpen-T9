/**
 * @file tkl_media_buf.c
 * @brief TKL adapter - media buffer pool, accounting and free_cb factory (T9 / BK7259).
 *
 * @details
 * The single place in the whole adapter that calls bk_frame_buffer_malloc() /
 * bk_frame_buffer_free(). Everything else - camera, uvc, gpu, venc, osd, tests -
 * goes through here so that in-flight buffers can be counted per owner and a
 * leak can be told apart from a stall.
 *
 * @version 0.1
 * @date 2026-08-27
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_media_buf.h"
#include "tkl_output.h"
#include "tkl_system.h"

#include <os/os.h>
#include <components/bk_frame_buffer.h>

/**
 * The vendor pool refuses a write-through mapping unless both the payload
 * address and the *requested* size are 64-byte aligned (bk_mem_slab.c,
 * bk_mem_slab_is_write_through_aligned()). Payload addresses always are;
 * the requested size is ours to round.
 */
#define TKL_MEDIA_BUF_WT_ALIGN   64U

#define TKL_MEDIA_BUF_DRAIN_STEP_MS 5U

STATIC BOOL_T s_media_buf_inited = FALSE;

/** In-flight count per owner. Touched from the GPU flexa thread, the encoder
 *  worker thread and business threads, so every update runs with interrupts
 *  masked - the critical sections here are a handful of instructions long. */
STATIC volatile UINT32_T s_inflight[TUYA_MEDIA_BUF_OWNER_MAX];

STATIC CONST CHAR_T *CONST s_owner_name[TUYA_MEDIA_BUF_OWNER_MAX] = {
#define TUYA_MEDIA_BUF_OWNER_NAME(sym, name) name,
    TUYA_MEDIA_BUF_OWNER_LIST(TUYA_MEDIA_BUF_OWNER_NAME)
#undef TUYA_MEDIA_BUF_OWNER_NAME
};

STATIC INLINE BOOL_T __owner_valid(TUYA_MEDIA_BUF_OWNER_E owner)
{
    return (owner < TUYA_MEDIA_BUF_OWNER_MAX) ? TRUE : FALSE;
}

STATIC VOID_T __inflight_inc(TUYA_MEDIA_BUF_OWNER_E owner)
{
    UINT32_T flags = rtos_disable_int();

    s_inflight[owner]++;

    rtos_enable_int(flags);
}

STATIC VOID_T __inflight_dec(TUYA_MEDIA_BUF_OWNER_E owner)
{
    BOOL_T underflow = FALSE;
    UINT32_T flags = rtos_disable_int();

    if (s_inflight[owner] == 0U) {
        underflow = TRUE;
    } else {
        s_inflight[owner]--;
    }

    rtos_enable_int(flags);

    if (underflow == TRUE) {
        /* double free, or a buffer that was not allocated through this module */
        tkl_log_output("tkl_media_buf: inflight underflow, owner=%s\r\n", s_owner_name[owner]);
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

VOID_T tkl_media_buf_init(VOID_T)
{
    if (s_media_buf_inited == TRUE) {
        return;
    }

    bk_frame_buffer_init();
    s_media_buf_inited = TRUE;
}

VOID_T *tkl_media_buf_alloc(TKL_BUF_USE_E use, UINT32_T size, TUYA_MEDIA_BUF_OWNER_E owner)
{
    frame_buffer_heap_type_t heap;
    VOID_T *buffer;
    UINT32_T alloc_size = size;

    if (size == 0U || use >= TKL_BUF_USE_MAX || __owner_valid(owner) != TRUE) {
        return NULL;
    }

    /* Deliberately NO lazy init here. bk_frame_buffer_init() is destructive:
     * it resets each heap to a single free block and forgets every live
     * allocation, including the ones media hardware made behind our back (the
     * ISP takes its SP planes from these pools). A "safety net" init on first
     * alloc would hand those addresses straight back out.
     *
     * Nothing here can tell an uninitialised pool from a busy one. An
     * uninitialised pool makes bk_frame_buffer_malloc() return NULL with its own
     * "not init" log - a loud, harmless failure, unlike silent aliasing. */

    heap = (use == TKL_BUF_USE_STREAM) ? MEM_SLAB_HEAP_CODED : MEM_SLAB_HEAP_UNCODED;

    if (use == TKL_BUF_USE_PIXEL_WT) {
        alloc_size = (size + (TKL_MEDIA_BUF_WT_ALIGN - 1U)) & ~(TKL_MEDIA_BUF_WT_ALIGN - 1U);
    }

    buffer = bk_frame_buffer_malloc(heap, alloc_size);
    if (buffer == NULL) {
        /* Throttled: during a pool-exhaustion episode (e.g. an SD stall backing
         * up the H264 writer) this fails once per frame, and a 20 Hz log stream
         * is itself enough load to starve the cross-core UART_LOG lock. */
        STATIC UINT32_T s_fail_cnt = 0U;
        s_fail_cnt++;
        if ((s_fail_cnt & (s_fail_cnt - 1U)) == 0U) { /* 1,2,4,8,... */
            tkl_log_output("tkl_media_buf: alloc failed, use=%u size=%u owner=%s (cnt=%u)\r\n",
                           (unsigned int)use,
                           (unsigned int)alloc_size,
                           s_owner_name[owner],
                           (unsigned int)s_fail_cnt);
        }
        return NULL;
    }

    if (use == TKL_BUF_USE_PIXEL_WT) {
        bk_err_t ret = bk_frame_buffer_set(buffer, BK_FRAME_BUFFER_FLAG_WRITE_THROUGH);

        if (ret == BK_ERR_NOT_SUPPORT) {
            /* CONFIG_PSRAM_WRITE_THROUGH compiled out - the buffer is still a
             * valid uncached pixel frame, so keep going instead of failing the
             * whole pipeline. Logged once per owner is enough noise. */
            STATIC BOOL_T s_wt_unsupported_logged = FALSE;
            if (s_wt_unsupported_logged == FALSE) {
                s_wt_unsupported_logged = TRUE;
                tkl_log_output("tkl_media_buf: psram write-through not supported, degrading to plain pixel\r\n");
            }
        } else if (ret != BK_OK) {
            /* channel exhaustion or a misaligned payload - a real defect, and
             * handing the caller a buffer the hardware will read stale data
             * from is worse than failing the alloc */
            tkl_log_output("tkl_media_buf: write-through setup failed, ret=%d size=%u owner=%s\r\n",
                           (int)ret,
                           (unsigned int)alloc_size,
                           s_owner_name[owner]);
            bk_frame_buffer_free(buffer);
            return NULL;
        }
    }

    __inflight_inc(owner);

    return buffer;
}

VOID_T tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_E owner, VOID_T *buffer)
{
    if (buffer == NULL || __owner_valid(owner) != TRUE) {
        return;
    }

    /* free first, then settle the count: once inflight hits 0 the caller of
     * tkl_media_buf_drain() is entitled to assume every release has completed.
     * The pool detaches the write-through channel on its own here. */
    bk_frame_buffer_free(buffer);
    __inflight_dec(owner);
}

/* One thunk per owner. This is what makes per-owner accounting possible without
 * a buffer-to-owner lookup table: the release callback embedded in a frame has
 * no context argument, so the owner has to be baked into the function itself. */
#define TUYA_MEDIA_BUF_OWNER_THUNK(sym, name)                    \
    STATIC VOID_T __media_buf_free_##sym(UINT8_T *buffer)       \
    {                                                           \
        tkl_media_buf_free(sym, (VOID_T *)buffer);              \
    }
TUYA_MEDIA_BUF_OWNER_LIST(TUYA_MEDIA_BUF_OWNER_THUNK)
#undef TUYA_MEDIA_BUF_OWNER_THUNK

STATIC CONST TUYA_MEDIA_BUF_FREE_CB s_owner_free_cb[TUYA_MEDIA_BUF_OWNER_MAX] = {
#define TUYA_MEDIA_BUF_OWNER_CB(sym, name) __media_buf_free_##sym,
    TUYA_MEDIA_BUF_OWNER_LIST(TUYA_MEDIA_BUF_OWNER_CB)
#undef TUYA_MEDIA_BUF_OWNER_CB
};

TUYA_MEDIA_BUF_FREE_CB tkl_media_buf_free_cb(TUYA_MEDIA_BUF_OWNER_E owner)
{
    if (__owner_valid(owner) != TRUE) {
        return NULL;
    }

    return s_owner_free_cb[owner];
}

UINT32_T tkl_media_buf_inflight(TUYA_MEDIA_BUF_OWNER_E owner)
{
    if (__owner_valid(owner) != TRUE) {
        return 0U;
    }

    return s_inflight[owner];
}

OPERATE_RET tkl_media_buf_drain(TUYA_MEDIA_BUF_OWNER_E owner,
                                UINT32_T expect_residual,
                                UINT32_T timeout_ms)
{
    UINT32_T waited = 0U;

    if (__owner_valid(owner) != TRUE) {
        return OPRT_INVALID_PARM;
    }

    while (s_inflight[owner] > expect_residual) {
        if (waited >= timeout_ms) {
            tkl_log_output("tkl_media_buf: drain timeout, owner=%s inflight=%u expected<=%u after %ums\r\n",
                           s_owner_name[owner],
                           (unsigned int)s_inflight[owner],
                           (unsigned int)expect_residual,
                           (unsigned int)waited);
            /* deliberately no forced free: business may still hold the pointer,
             * and a leak beats a use-after-free */
            return OPRT_TIMEOUT;
        }

        tkl_system_sleep(TKL_MEDIA_BUF_DRAIN_STEP_MS);
        waited += TKL_MEDIA_BUF_DRAIN_STEP_MS;
    }

    return OPRT_OK;
}

VOID_T tkl_media_buf_dump(VOID_T)
{
    UINT32_T i;
    UINT32_T total = 0U;

    for (i = 0U; i < (UINT32_T)TUYA_MEDIA_BUF_OWNER_MAX; i++) {
        UINT32_T cnt = s_inflight[i];

        if (cnt != 0U) {
            tkl_log_output("tkl_media_buf: %-9s inflight=%u\r\n", s_owner_name[i], (unsigned int)cnt);
            total += cnt;
        }
    }

    tkl_log_output("tkl_media_buf: total inflight=%u\r\n", (unsigned int)total);
}
