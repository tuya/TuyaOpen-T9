/**
 * @file tkl_thread.c
 * @brief Auto-generated TKL adapter stub. Candidate Thread adapter for T9.
 * @version 0.1
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_thread.h"

OPERATE_RET tkl_thread_create(TKL_THREAD_HANDLE* thread, CONST CHAR_T* name, UINT_T stack_size, UINT_T priority, CONST THREAD_FUNC_T func, VOID_T* CONST arg)
{
    (void)thread;
    (void)name;
    (void)stack_size;
    (void)priority;
    (void)func;
    (void)arg;

    /* TODO: map `tkl_thread_create` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_thread_smp_create(TKL_THREAD_HANDLE* thread, UINT_T affinity, CONST CHAR_T* name, UINT_T stack_size, UINT_T priority, THREAD_FUNC_T func, VOID_T* CONST arg)
{
    (void)thread;
    (void)affinity;
    (void)name;
    (void)stack_size;
    (void)priority;
    (void)func;
    (void)arg;

    /* TODO: map `tkl_thread_smp_create` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_thread_release(CONST TKL_THREAD_HANDLE thread)
{
    (void)thread;

    /* TODO: map `tkl_thread_release` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_thread_get_watermark(CONST TKL_THREAD_HANDLE thread, UINT_T* watermark)
{
    (void)thread;
    (void)watermark;

    /* TODO: map `tkl_thread_get_watermark` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_thread_get_id(TKL_THREAD_HANDLE *thread)
{
    (void)thread;

    /* TODO: map `tkl_thread_get_id` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_thread_set_self_name(CONST CHAR_T* name)
{
    (void)name;

    /* TODO: map `tkl_thread_set_self_name` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_thread_is_self(TKL_THREAD_HANDLE thread, BOOL_T* is_self)
{
    (void)thread;
    (void)is_self;

    /* TODO: map `tkl_thread_is_self` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_thread_get_priority(TKL_THREAD_HANDLE thread, INT_T *priority)
{
    (void)thread;
    (void)priority;

    /* TODO: map `tkl_thread_get_priority` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_thread_set_priority(TKL_THREAD_HANDLE thread, INT_T priority)
{
    (void)thread;
    (void)priority;

    /* TODO: map `tkl_thread_set_priority` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_thread_diagnose(TKL_THREAD_HANDLE thread)
{
    (void)thread;

    /* TODO: map `tkl_thread_diagnose` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_thread_create_in_psram(TKL_THREAD_HANDLE* thread, CONST CHAR_T* name, UINT_T stack_size, UINT_T priority, CONST THREAD_FUNC_T func, VOID_T* CONST arg)
{
    (void)thread;
    (void)name;
    (void)stack_size;
    (void)priority;
    (void)func;
    (void)arg;

    /* TODO: map `tkl_thread_create_in_psram` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}
