/**
 * @file tkl_frame_buffer.c
 * @brief TKL adapter - media frame buffer allocator for T9 (BK7259).
 *
 * Thin wrapper over the AVDK mem-slab frame buffer pools. Exists because the
 * GPU / VENC sink callbacks are caller-supplied, while the memory they must
 * return comes from vendor-internal pools that upper layers cannot reach.
 *
 * @version 0.1
 * @date 2026-08-13
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_frame_buffer.h"
#include "tkl_output.h"

#include <components/bk_frame_buffer.h>

STATIC BOOL_T s_frame_buffer_inited = FALSE;

STATIC frame_buffer_heap_type_t __tkl_frame_buffer_heap(TKL_FRAME_BUFFER_TYPE_E type)
{
    return (type == TKL_FRAME_BUFFER_UNCODED) ? MEM_SLAB_HEAP_UNCODED : MEM_SLAB_HEAP_CODED;
}

VOID_T tkl_frame_buffer_init(VOID_T)
{
    if (s_frame_buffer_inited == TRUE) {
        return;
    }

    bk_frame_buffer_init();
    s_frame_buffer_inited = TRUE;
}

VOID_T *tkl_frame_buffer_malloc(TKL_FRAME_BUFFER_TYPE_E type, UINT32_T size)
{
    VOID_T *frame;

    if (size == 0U || type >= TKL_FRAME_BUFFER_TYPE_MAX) {
        return NULL;
    }

    /* safety net only -- media hardware also allocates from these pools behind
     * our back, so callers are expected to have called tkl_frame_buffer_init()
     * before bringing that hardware up */
    tkl_frame_buffer_init();

    frame = bk_frame_buffer_malloc(__tkl_frame_buffer_heap(type), size);
    if (frame == NULL) {
        /* Throttled: during a pool-exhaustion episode (e.g. an SD stall backing
         * up the H264 writer) this fails once per frame, and a 20 Hz log stream
         * is itself enough load to starve the cross-core UART_LOG lock. */
        STATIC UINT32_T s_fail_cnt = 0;
        s_fail_cnt++;
        if ((s_fail_cnt & (s_fail_cnt - 1U)) == 0U) { /* 1,2,4,8,... */
            tkl_log_output("tkl_frame_buffer: malloc failed, type=%u size=%u (cnt=%u)\r\n",
                           (unsigned int)type, (unsigned int)size, (unsigned int)s_fail_cnt);
        }
    }

    return frame;
}

VOID_T tkl_frame_buffer_free(VOID_T *frame)
{
    if (frame == NULL) {
        return;
    }

    bk_frame_buffer_free(frame);
}

OPERATE_RET tkl_frame_buffer_set_write_through(VOID_T *frame)
{
    if (frame == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (bk_frame_buffer_set(frame, BK_FRAME_BUFFER_FLAG_WRITE_THROUGH) != BK_OK) {
        tkl_log_output("tkl_frame_buffer: set write-through failed, frame=%p\r\n", frame);
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}
