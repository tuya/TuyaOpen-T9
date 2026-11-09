/* Audio subsystem OS adapter. Select exactly one backend at build time:
 * AUDIO_OS_HOST: libc / POSIX host (default on non-ARM).
 * AUDIO_OS_BK:   Beken allocation/logging; timing still requires TKL.
 * AUDIO_OS_TKL:  Tuya kernel adaptation layer (default on ARM).
 * AUDIO_OS_TAL:  Tuya service abstraction layer.
 * AUDIO_USE_PSRAM selects the platform PSRAM allocator, never on HOST.
 * Selection must be consistent across the complete library.
 */
#ifndef AUDIO_SUBSYS_OS_H
#define AUDIO_SUBSYS_OS_H

/* Include this header before libc headers in POSIX host translation units. */
#if !defined(_DEFAULT_SOURCE) && !defined(_POSIX_C_SOURCE) && !defined(_GNU_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif

#include <stddef.h>
#include <stdint.h>

#if defined(AUDIO_OS_HOST) + defined(AUDIO_OS_BK) + defined(AUDIO_OS_TKL) + defined(AUDIO_OS_TAL) > 1
#error "Select only one AUDIO_OS_HOST / AUDIO_OS_BK / AUDIO_OS_TKL / AUDIO_OS_TAL"
#endif
#if defined(AUDIO_OS_BKOS) || defined(AUDIO_OS_TUYAOS)
#error "Use AUDIO_OS_BK instead of BKOS, and AUDIO_OS_TAL instead of TUYAOS"
#endif
#if !defined(AUDIO_OS_HOST) && !defined(AUDIO_OS_BK) && !defined(AUDIO_OS_TKL) && !defined(AUDIO_OS_TAL)
#if defined(__arm__) || defined(__ARM_ARCH) || defined(__aarch64__)
#define AUDIO_OS_TKL 1
#else
#define AUDIO_OS_HOST 1
#endif
#endif

#ifdef AUDIO_OS_HOST
#ifdef AUDIO_USE_PSRAM
#error "AUDIO_USE_PSRAM is not supported by AUDIO_OS_HOST"
#endif
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#if defined(_WIN32)
#include <windows.h>
#endif
#define audio_subsys_malloc(size)        malloc(size)
#define audio_subsys_calloc(count, size) calloc((count), (size))
#define audio_subsys_free(ptr)           free(ptr)
#define audio_subsys_printf(fmt, ...)    printf(fmt "\n", ##__VA_ARGS__)
static inline uint32_t audio_subsys_tick_ms(void)
{
#if defined(_WIN32)
    return (uint32_t)GetTickCount64();
#else
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        return 0;
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u);
#endif
}

#elif defined(AUDIO_OS_BK)
#include <stdlib.h>
#include <os/mem.h>
#include <components/system.h>
#include "tkl_system.h"
#ifdef AUDIO_USE_PSRAM
#define audio_subsys_malloc(size)        psram_malloc(size)
#define audio_subsys_calloc(count, size) psram_zalloc((size_t)(count) * (size_t)(size))
#define audio_subsys_free(ptr)           psram_free(ptr)
#else
#define audio_subsys_malloc(size)        os_malloc(size)
#define audio_subsys_calloc(count, size) os_zalloc((size_t)(count) * (size_t)(size))
#define audio_subsys_free(ptr)           os_free(ptr)
#endif
#define audio_subsys_printf(fmt, ...)    bk_printf(fmt "\r\n", ##__VA_ARGS__)
static inline uint32_t audio_subsys_tick_ms(void)
{
    return (uint32_t)tkl_system_get_millisecond();
}

#elif defined(AUDIO_OS_TKL)
#include <stdlib.h>
#include "tkl_system.h"
#include "tkl_memory.h"
#include "tkl_output.h"
#ifdef AUDIO_USE_PSRAM
#define audio_subsys_malloc(size)        tkl_system_psram_malloc((SIZE_T)(size))
#define audio_subsys_calloc(count, size) tkl_system_psram_calloc((size_t)(count), (size_t)(size))
#define audio_subsys_free(ptr)           tkl_system_psram_free(ptr)
#else
#define audio_subsys_malloc(size)        tkl_system_malloc((SIZE_T)(size))
#define audio_subsys_calloc(count, size) tkl_system_calloc((size_t)(count), (size_t)(size))
#define audio_subsys_free(ptr)           tkl_system_free(ptr)
#endif
#define audio_subsys_printf(fmt, ...)    tkl_log_output(fmt "\r\n", ##__VA_ARGS__)
static inline uint32_t audio_subsys_tick_ms(void)
{
    return (uint32_t)tkl_system_get_millisecond();
}

#elif defined(AUDIO_OS_TAL)
#include <stdlib.h>
#include "tal_memory.h"
#include "tal_log.h"
#include "tkl_system.h"
#ifdef AUDIO_USE_PSRAM
#define audio_subsys_malloc(size)        tal_psram_malloc((SIZE_T)(size))
#define audio_subsys_calloc(count, size) tal_psram_calloc((SIZE_T)(count), (SIZE_T)(size))
#define audio_subsys_free(ptr)           tal_psram_free(ptr)
#else
#define audio_subsys_malloc(size)        tal_malloc((SIZE_T)(size))
#define audio_subsys_calloc(count, size) tal_calloc((SIZE_T)(count), (SIZE_T)(size))
#define audio_subsys_free(ptr)           tal_free(ptr)
#endif
#define audio_subsys_printf(fmt, ...)    TAL_PR_NOTICE(fmt, ##__VA_ARGS__)
static inline uint32_t audio_subsys_tick_ms(void)
{
    return (uint32_t)tkl_system_get_millisecond();
}
#endif

/* Unsigned subtraction handles the uint32_t millisecond counter wrapping.
 * This is not a CPU cycle counter. PSRAM routing may affect DSP performance;
 * TKL/TAL allocation can also be affected by platform-global heap policies.
 */
#endif /* AUDIO_SUBSYS_OS_H */
