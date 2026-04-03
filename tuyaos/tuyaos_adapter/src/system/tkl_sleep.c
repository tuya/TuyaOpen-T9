/**
 * @file tkl_sleep.c
 * @brief Auto-generated TKL adapter stub. Candidate Sleep adapter for T9.
 * @version 0.1
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_sleep.h"

OPERATE_RET tkl_cpu_sleep_callback_register(TUYA_SLEEP_CB_T *sleep_cb)
{
    (void)sleep_cb;

    /* TODO: map `tkl_cpu_sleep_callback_register` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

VOID_T tkl_cpu_allow_sleep(VOID_T)
{
    /* TODO: map `tkl_cpu_allow_sleep` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return;
}

VOID_T tkl_cpu_force_wakeup(VOID_T)
{
    /* TODO: map `tkl_cpu_force_wakeup` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return;
}

OPERATE_RET tkl_cpu_sleep_mode_set(BOOL_T enable, TUYA_CPU_SLEEP_MODE_E mode)
{
    (void)enable;
    (void)mode;

    /* TODO: map `tkl_cpu_sleep_mode_set` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}
