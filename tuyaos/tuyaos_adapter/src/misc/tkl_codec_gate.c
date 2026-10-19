/**
 * @file tkl_codec_gate.c
 * @brief One hardware codec session at a time, per side.
 * @version 0.2
 * @date 2026-10-14
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include <os/os.h>
#include "tkl_codec_gate_internal.h"

/* Binary semaphores: available = nobody is decoding/encoding, taken = hardware
 * busy. Created full on first use - the adapter has no module init hook and a
 * gate must exist before the first session, not at boot. */
static beken_semaphore_t s_codec_gate[TKL_CODEC_GATE_MAX];

static VOID_T tkl_codec_gate_create(TKL_CODEC_GATE_E which)
{
    UINT32_T flags;

    if (s_codec_gate[which] != NULL) {
        return;
    }

    /* The critical section covers the check and the assignment together, so
     * two first-time callers cannot end up with two semaphores - that would
     * let both believe they own the hardware. Creation itself is a small
     * non-blocking allocation. */
    flags = rtos_enter_critical();
    if (s_codec_gate[which] == NULL) {
        (void)rtos_init_semaphore_ex(&s_codec_gate[which], 1, 1);
    }
    rtos_exit_critical(flags);
}

OPERATE_RET tkl_codec_gate_acquire(TKL_CODEC_GATE_E which, UINT32_T timeout_ms)
{
    if ((UINT32_T)which >= (UINT32_T)TKL_CODEC_GATE_MAX) {
        return OPRT_INVALID_PARM;
    }

    tkl_codec_gate_create(which);

    if (s_codec_gate[which] == NULL) {
        return OPRT_COM_ERROR;
    }

    if (rtos_get_semaphore(&s_codec_gate[which], timeout_ms) != 0) {
        return OPRT_TIMEOUT;
    }

    return OPRT_OK;
}

VOID_T tkl_codec_gate_release(TKL_CODEC_GATE_E which)
{
    if ((UINT32_T)which >= (UINT32_T)TKL_CODEC_GATE_MAX) {
        return;
    }

    if (s_codec_gate[which] != NULL) {
        (void)rtos_set_semaphore(&s_codec_gate[which]);
    }
}

BOOL_T tkl_codec_gate_is_busy(TKL_CODEC_GATE_E which)
{
    if ((UINT32_T)which >= (UINT32_T)TKL_CODEC_GATE_MAX) {
        return FALSE;
    }

    if (s_codec_gate[which] == NULL) {
        return FALSE;
    }

    /* Probe without waiting: taking it means nobody held it. */
    if (rtos_get_semaphore(&s_codec_gate[which], 0) != 0) {
        return TRUE;
    }

    (void)rtos_set_semaphore(&s_codec_gate[which]);
    return FALSE;
}
