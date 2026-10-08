/**
 * @file tkl_hwjpeg.c
 * @brief One hardware JPEG decoder, one session at a time.
 * @version 0.1
 * @date 2026-10-08
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include <os/os.h>
#include "tkl_hwjpeg_internal.h"

/* Binary semaphore: available = nobody is decoding, taken = hardware busy.
 * Created full on first use - the adapter has no module init hook and the gate
 * must exist before the first decoder session, not at boot. */
static beken_semaphore_t s_hwjpeg_gate = NULL;

static VOID_T tkl_hwjpeg_gate_create(VOID_T)
{
    UINT32_T flags;

    if (s_hwjpeg_gate != NULL) {
        return;
    }

    /* The critical section covers the check and the assignment together, so
     * two first-time callers cannot end up with two semaphores - that would
     * let both believe they own the hardware. Creation itself is a small
     * non-blocking allocation. */
    flags = rtos_enter_critical();
    if (s_hwjpeg_gate == NULL) {
        (void)rtos_init_semaphore_ex(&s_hwjpeg_gate, 1, 1);
    }
    rtos_exit_critical(flags);
}

OPERATE_RET tkl_hwjpeg_acquire(UINT32_T timeout_ms)
{
    tkl_hwjpeg_gate_create();

    if (s_hwjpeg_gate == NULL) {
        return OPRT_COM_ERROR;
    }

    if (rtos_get_semaphore(&s_hwjpeg_gate, timeout_ms) != 0) {
        return OPRT_TIMEOUT;
    }

    return OPRT_OK;
}

VOID_T tkl_hwjpeg_release(VOID_T)
{
    if (s_hwjpeg_gate != NULL) {
        (void)rtos_set_semaphore(&s_hwjpeg_gate);
    }
}

BOOL_T tkl_hwjpeg_is_busy(VOID_T)
{
    if (s_hwjpeg_gate == NULL) {
        return FALSE;
    }

    /* Probe without waiting: taking it means nobody held it. */
    if (rtos_get_semaphore(&s_hwjpeg_gate, 0) != 0) {
        return TRUE;
    }

    (void)rtos_set_semaphore(&s_hwjpeg_gate);
    return FALSE;
}
