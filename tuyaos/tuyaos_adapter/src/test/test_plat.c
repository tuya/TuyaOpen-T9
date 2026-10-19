/**
 * @file test_plat.c
 * @brief Platform CPU / SRAM / PSRAM performance matrix (xplat)
 * @version 1.3
 * @date 2026-07-27
 * @copyright Copyright (c) Tuya Inc.
 *
 * @note No runtime CPU frequency switch.
 * @note No enable_dcache(0): that path calls mpu_disable() and is unsafe.
 * @note Extra subcmds (dma/contend/latx/float/sram/psram/psram_nc) also included in xplat full where applicable.
 * @note full also runs xip/irq probes; case freq= uses measured MHz when available.
 * @note Log/SKIP/SUMMARY aligned with docs/platform-perf-test-spec.md.
 * @note XPLAT_KERNEL_O0=1 forces -O0 on kernels (optional anti-DCE debug).
 */
#include "cli.h"
#include "cli_tuya_test.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <components/log.h>
#include <driver/aon_rtc.h>
#include <driver/dma.h>
#if defined(__has_include)
#  if __has_include(<driver/hpdma.h>)
#    define XPLAT_HAS_HPDMA 1
#    include <driver/hpdma.h>
#  endif
#endif
#ifndef XPLAT_HAS_HPDMA
#  define XPLAT_HAS_HPDMA 0
#endif
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include <modules/pm.h>
#include <soc/soc.h>
#include <driver/timer.h>
#if defined(__has_include)
#  if __has_include(<components/bk_frame_buffer.h>)
#    define XPLAT_HAS_FB_UNCODED 1
#    include <components/bk_frame_buffer.h>
#  endif
#endif
#ifndef XPLAT_HAS_FB_UNCODED
#  define XPLAT_HAS_FB_UNCODED 0
#endif

#include "sdkconfig.h"
#include "tkl_media_buf.h"

/* GDMA memcpy helpers live in private bk_general_dma */
extern bk_err_t dma_memcpy(void *out, const void *in, uint32_t len);
extern bk_err_t dma_memcpy_by_chnl(void *out, const void *in, uint32_t len, dma_id_t cpy_chnl);

#ifdef bk_printf
#undef bk_printf
#endif
#define bk_printf(fmt, ...) BK_RAW_LOGE("xplat", fmt, ##__VA_ARGS__)

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define XPLAT_TASK_PRIO              4
#define XPLAT_TASK_STACK             (1024 * 12)
#define XPLAT_CPU_MUL_LOOPS          (1000U * 1000U)
#define XPLAT_CPU_ADD_LOOPS          (2000U * 1000U)
#define XPLAT_CPU_BRANCH_LOOPS       (1000U * 1000U)
#define XPLAT_CPU_DIV_LOOPS          (200U * 1000U)
#define XPLAT_CPU_SHIFT_LOOPS        (2000U * 1000U)
#define XPLAT_CPU_FADD_LOOPS         (500U * 1000U)
#define XPLAT_CPU_FMUL_LOOPS         (500U * 1000U)
#define XPLAT_LATENCY_WORDS          (4096U)
#define XPLAT_LATENCY_ACCESSES       (200000U)
#define XPLAT_BW_MIN_TOTAL_US        2000U
#define XPLAT_ALLOC_LOOPS            2000U
#define XPLAT_SWITCH_LOOPS           5000U
#define XPLAT_CONTEND_SIZE           (64U * 1024U)
#define XPLAT_CONTEND_PEER_STACK     4096U
#define XPLAT_MODE_MIN               0
#define XPLAT_MODE_FULL              1
#define XPLAT_MODE_DMA               2
#define XPLAT_MODE_CONTEND           3
#define XPLAT_MODE_LATX              4
#define XPLAT_MODE_FLOAT             5
#define XPLAT_MODE_SRAM              6
#define XPLAT_MODE_PSRAM             7
#define XPLAT_MODE_PSRAM_NC          8
#define XPLAT_MODE_MPU               9
#define XPLAT_REC_MAX                384U
#define XPLAT_NAME_LEN               24U
#define XPLAT_RAND_IDX_SEED          0xA5A5F00DU

/* Domain selector for xplat_alloc / runners */
#define XPLAT_MEM_SRAM               0
#define XPLAT_MEM_PSRAM              1
#define XPLAT_MEM_PSRAM_NC           2

/*
 * Set to 1 to compile xplat kernels at -O0 (stronger anti-DCE, slower/less
 * representative of product -O). Default 0 keeps product-like opts.
 * When O0=0: Mem BW kernels use XPLAT_ESCAPE_* (empty asm) against DCE.
 * When O0=1: XPLAT_ESCAPE_* become no-ops (rely on -O0); compare A/B this way.
 * CPU/float still use s_xplat_sink either way.
 * Override: -DXPLAT_KERNEL_O0=1
 */
#ifndef XPLAT_KERNEL_O0
#define XPLAT_KERNEL_O0              0
#endif
#if XPLAT_KERNEL_O0
#define XPLAT_KERNEL_ATTR \
    __attribute__((noinline, optimize("O0")))
/* Rely on -O0; ESCAPE disabled so A/B can isolate O0 vs asm escape. */
#define XPLAT_ESCAPE_VAL(v)          ((void)(v))
#define XPLAT_ESCAPE_PTR(p, n)       do { (void)(p); (void)(n); } while (0)
#define XPLAT_ESCAPE_BUF(dst, src, n) \
    do { (void)(dst); (void)(src); (void)(n); } while (0)
#else
#define XPLAT_KERNEL_ATTR \
    __attribute__((noinline, optimize("-fno-tree-loop-distribute-patterns")))
/* Anti-DCE: empty at runtime, keep mem ops / values observable. */
#define XPLAT_ESCAPE_VAL(v) \
    __asm__ volatile("" : : "r"(v))
#define XPLAT_ESCAPE_PTR(p, n) \
    __asm__ volatile("" : : "r"(p), "r"(n) : "memory")
#define XPLAT_ESCAPE_BUF(dst, src, n) \
    __asm__ volatile("" : : "r"(dst), "r"(src), "r"(n) : "memory")
#endif

typedef enum {
    XPLAT_REC_CPU = 0,
    XPLAT_REC_BW,
    XPLAT_REC_LAT,
    XPLAT_REC_OPS,
    XPLAT_REC_SKIP
} xplat_rec_kind_e;

typedef struct {
    uint8_t kind;
    char case_name[XPLAT_NAME_LEN];
    char mem[XPLAT_NAME_LEN];
    uint32_t size;
    float metric;   /* mops / mbps / ns_per / us_per */
    uint64_t us;
} xplat_rec_t;

/* ---------------------------------------------------------------------------
 * File scope variables
 * --------------------------------------------------------------------------- */
static volatile uint32_t s_xplat_sink;
static beken_semaphore_t s_xplat_sem_a;
static beken_semaphore_t s_xplat_sem_b;
static volatile uint32_t s_xplat_switch_done;
static xplat_rec_t *s_xplat_recs;
static uint32_t s_xplat_rec_cnt;
static uint32_t s_xplat_rec_cap;
static int s_xplat_recs_psram;

static volatile uint32_t s_xplat_contend_go;
static volatile uint32_t s_xplat_contend_stop;
static volatile uint32_t s_xplat_contend_done;
static volatile uint64_t s_xplat_contend_bytes;
static volatile uint32_t s_xplat_contend_peer_core;
static uint8_t *s_xplat_contend_src;
static uint8_t *s_xplat_contend_dst;
static uint32_t s_xplat_contend_size;
static int s_xplat_contend_psram;
#if XPLAT_HAS_HPDMA
static hpdma_id_t s_xplat_hp_chn;
#endif
static dma_id_t s_xplat_gd_chn;
static uint32_t s_xplat_meas_mhz;

static const uint32_t s_xplat_mem_sizes_min[] = {
    1024U, 4096U, 16384U
};
static const uint32_t s_xplat_mem_sizes_full[] = {
    64U, 256U, 1024U, 4096U, 16384U, 65536U, 131072U, 262144U
};
/* SRAM heap often cannot dual-buffer 256K; cap sram domain below this */
#define XPLAT_SRAM_MAX_SIZE          (128U * 1024U)
/* Max Flash/.rodata blob for multi-size XIP seq_read (prefix reads) */
#define XPLAT_XIP_BYTES              (256U * 1024U)
#if defined(CONFIG_SOC_BK7259)
#define XPLAT_IRQ_TIMER_ID           TIMER_ID12
#else
#define XPLAT_IRQ_TIMER_ID           TIMER_ID5
#endif
#define XPLAT_IRQ_TIMER_US           1000U
#define XPLAT_IRQ_SAMPLES            32U
#define XPLAT_CRIT_HOLD_US           500U
#define XPLAT_IRQ_WAIT_US            5000U
/* DMA engine memcpy sizes (same full matrix as mem; SRAM-side >128K → sram_cap) */
static const uint32_t s_xplat_dma_sizes[] = {
    64U, 256U, 1024U, 4096U, 16384U, 65536U, 131072U, 262144U
};
/* Random gather/scatter bandwidth working sets (data bytes; idx same size) */
static const uint32_t s_xplat_rand_bw_sizes[] = {
    1024U, 4096U, 16384U, 65536U, 131072U, 262144U
};
/* Pointer-chase latency working-set sizes (bytes of idx table) */
static const uint32_t s_xplat_lat_ws_sizes[] = {
    1024U, 4096U, 16384U, 65536U, 131072U, 262144U
};
static const uint32_t s_xplat_align_offs[] = {
    0U, 1U, 2U, 3U, 4U, 8U, 16U
};

/* XIP/.rodata blob for flash read bandwidth (mostly zeros in flash image).
 * Reads must use volatile loads — const zeros can be constant-propagated away. */
static const uint8_t s_xplat_xip_ro[XPLAT_XIP_BYTES] __attribute__((aligned(32))) = {
    0xA5U, 0x5AU, 0x3CU, 0xC3U
};

static volatile uint64_t s_xplat_irq_t_isr;
static volatile uint32_t s_xplat_irq_hits;
static volatile uint32_t s_xplat_irq_c_isr;

/* ---------------------------------------------------------------------------
 * Helpers
 * --------------------------------------------------------------------------- */

/**
 * @brief Compile-time configured CPU MHz (log tag only)
 * @return MHz from CONFIG_CPU_FREQ_HZ
 */
static uint32_t xplat_cfg_mhz(void)
{
#if defined(CONFIG_CPU_FREQ_HZ)
    return (uint32_t)(CONFIG_CPU_FREQ_HZ / 1000000U);
#else
    return 0U;
#endif
}

/**
 * @brief Map PM CPU freq enum to MHz
 * @param[in] freq PM enum
 * @return MHz, or 0 if unknown/default
 */
static uint32_t xplat_pm_freq_to_mhz(pm_cpu_freq_e freq)
{
    if (freq == PM_CPU_FRQ_480M) {
        return 480U;
    }
    if (freq == PM_CPU_FRQ_320M) {
        return 320U;
    }
    if (freq == PM_CPU_FRQ_240M) {
        return 240U;
    }
    if (freq == PM_CPU_FRQ_120M) {
        return 120U;
    }
    if (freq == PM_CPU_FRQ_80M) {
        return 80U;
    }
    /* XTAL / 26M / 60M / 160M / DEFAULT / unknown: leave 0, caller logs enum.
     * 4.0.1.11 dropped PM_CPU_FRQ_60M from the AP-side pm_cpu_freq_e (CP still has it). */
    return 0U;
}

/**
 * @brief Runtime voted max CPU frequency (PM), not CONFIG_CPU_FREQ_HZ
 * @return MHz, or 0 if PM reports default/unknown
 */
static uint32_t xplat_runtime_mhz(void)
{
    return xplat_pm_freq_to_mhz(bk_pm_current_max_cpu_freq_get());
}

/**
 * @brief Frequency tag for per-case logs (prefer measured, else runtime/cfg)
 * @return MHz value
 */
static uint32_t xplat_current_freq_mhz(void)
{
    if (s_xplat_meas_mhz != 0U) {
        return s_xplat_meas_mhz;
    }
    {
        uint32_t rt = xplat_runtime_mhz();

        return (rt != 0U) ? rt : xplat_cfg_mhz();
    }
}

/* DWT CYCCNT MMIO (Cortex-M); avoid pulling CMSIS headers into adapter. */
#define XPLAT_DWT_CTRL           (*((volatile uint32_t *)(0xE0001000UL)))
#define XPLAT_DWT_CYCCNT         (*((volatile uint32_t *)(0xE0001004UL)))
#define XPLAT_DEMCR              (*((volatile uint32_t *)(0xE000EDFCUL)))
#define XPLAT_DEMCR_TRCENA       (1UL << 24)
#define XPLAT_DWT_CYCCNTENA      (1UL << 0)
#define XPLAT_FREQ_PROBE_US      2000U

/* SCB cache-info MMIO (Cortex-M55 ARMv8.1-M, classic CCSIDR; offsets per armv8m_reg.h). */
#define XPLAT_SCB_CLIDR          (*((volatile uint32_t *)(0xE000ED78UL)))
#define XPLAT_SCB_CTR            (*((volatile uint32_t *)(0xE000ED7CUL)))
#define XPLAT_SCB_CCSIDR         (*((volatile uint32_t *)(0xE000ED80UL)))
#define XPLAT_SCB_CSSELR         (*((volatile uint32_t *)(0xE000ED84UL)))
/* L2 cache controller (ARM PL310), same base as middleware/arch cortex-m l2_cache.c */
#define XPLAT_L2C_PL310_BASE     (0xA0000000UL)
/* PMSAv8 MPU MMIO (ARMv8-M: CM33/CM55 "Star" core; offsets per core_star.h MPU_Type). */
#define XPLAT_MPU_BASE           (0xE000ED90UL)
#define XPLAT_MPU_TYPE           (*((volatile uint32_t *)(XPLAT_MPU_BASE + 0x00UL)))
#define XPLAT_MPU_CTRL           (*((volatile uint32_t *)(XPLAT_MPU_BASE + 0x04UL)))
#define XPLAT_MPU_RNR            (*((volatile uint32_t *)(XPLAT_MPU_BASE + 0x08UL)))
#define XPLAT_MPU_RBAR           (*((volatile uint32_t *)(XPLAT_MPU_BASE + 0x0CUL)))
#define XPLAT_MPU_RLAR           (*((volatile uint32_t *)(XPLAT_MPU_BASE + 0x10UL)))
#define XPLAT_MPU_MAIR0          (*((volatile uint32_t *)(XPLAT_MPU_BASE + 0x30UL)))
#define XPLAT_MPU_MAIR1          (*((volatile uint32_t *)(XPLAT_MPU_BASE + 0x34UL)))

/**
 * @brief Enable DWT cycle counter
 * @return none
 */
static void xplat_dwt_enable(void)
{
    if ((XPLAT_DEMCR & XPLAT_DEMCR_TRCENA) == 0U) {
        XPLAT_DEMCR |= XPLAT_DEMCR_TRCENA;
    }
    XPLAT_DWT_CYCCNT = 0U;
    XPLAT_DWT_CTRL |= XPLAT_DWT_CYCCNTENA;
}

/**
 * @brief Measure CPU MHz via DWT CYCCNT / AON RTC wall time
 * @return Measured MHz, or 0 on failure
 * @note Absolute probe: distinguishes 320 vs 480 regardless of PM enum labels.
 */
static uint32_t xplat_meas_mhz(void)
{
    uint64_t t0;
    uint64_t t1;
    uint32_t c0;
    uint32_t c1;
    uint32_t us;
    uint32_t int_level;

    xplat_dwt_enable();
    /* Warmup: ensure CYCCNT is ticking */
    (void)XPLAT_DWT_CYCCNT;
    int_level = rtos_disable_int();
    t0 = bk_aon_rtc_get_us();
    c0 = XPLAT_DWT_CYCCNT;
    do {
        t1 = bk_aon_rtc_get_us();
    } while ((uint32_t)(t1 - t0) < XPLAT_FREQ_PROBE_US);
    c1 = XPLAT_DWT_CYCCNT;
    t1 = bk_aon_rtc_get_us();
    rtos_enable_int(int_level);
    us = (uint32_t)(t1 - t0);
    if (us == 0U) {
        return 0U;
    }
    /* cycles / us == MHz */
    return (uint32_t)(((uint64_t)(c1 - c0)) / (uint64_t)us);
}

/**
 * @brief Best-effort decode of core clock mux/div to MHz
 * @param[out] cksel_out cksel_core field
 * @param[out] div_out clkdiv/ckdiv_core field
 * @return Decoded MHz, or 0 if unknown / unavailable
 */
static uint32_t xplat_hw_mhz(uint32_t *cksel_out, uint32_t *div_out)
{
    uint32_t reg = 0U;
    uint32_t cksel = 0U;
    uint32_t div = 0U;
    uint32_t mhz = 0U;

#if defined(SYS_CPU_CLK_DIV_MODE1_ADDR)
    reg = *((volatile uint32_t *)SYS_CPU_CLK_DIV_MODE1_ADDR);
#elif defined(SOC_SYS_REG_BASE)
    reg = *((volatile uint32_t *)(SOC_SYS_REG_BASE + (0x8U << 2)));
#else
    if (cksel_out != NULL) {
        *cksel_out = 0U;
    }
    if (div_out != NULL) {
        *div_out = 0U;
    }
    return 0U;
#endif

#if defined(CONFIG_SOC_BK7259)
    /* SoC A: cksel[1:0], ckdiv[5:2]; map known DVFS tuples */
    cksel = reg & 0x3U;
    div = (reg >> 2) & 0xFU;
    if ((cksel == 1U) && (div == 0U)) {
        mhz = 480U;
    } else if ((cksel == 2U) && ((div == 0U) || (div == 1U))) {
        mhz = 320U;
    } else if ((cksel == 1U) && (div == 1U)) {
        mhz = 240U;
    } else if ((cksel == 1U) && (div == 2U)) {
        mhz = 160U;
    } else if ((cksel == 1U) && (div == 3U)) {
        mhz = 120U;
    } else if ((cksel == 3U) && (div == 1U)) {
        mhz = 120U;
    } else if ((cksel == 3U) && (div == 0U)) {
        mhz = 480U;
    } else if ((cksel == 3U) && (div == 7U)) {
        /* Observed AP @~480M when mux encoding != DVFS table */
        mhz = 480U;
    }
#elif defined(CONFIG_SOC_BK7258) || defined(CONFIG_SOC_BK7257)
    /* SoC B: clkdiv[3:0], cksel[5:4]; src 2=320M 3=480M, F/(div+1) */
    div = reg & 0xFU;
    cksel = (reg >> 4) & 0x3U;
    if (cksel == 3U) {
        mhz = 480U / (div + 1U);
    } else if (cksel == 2U) {
        mhz = 320U / (div + 1U);
    } else if ((cksel == 0U) || (cksel == 1U)) {
        mhz = 26U;
    }
#else
    (void)reg;
#endif

    if (cksel_out != NULL) {
        *cksel_out = cksel;
    }
    if (div_out != NULL) {
        *div_out = div;
    }
    return mhz;
}

/**
 * @brief Log L1 (CCSIDR) and L2 (PL310) cache geometry
 * @return none
 * @note L1 decoded from classic ARMv8-M CCSIDR (Cortex-M55 has no FEAT_CCIDX):
 *       line=2^(LineSize+4)B, ways=Assoc+1, sets=NumSets+1, size=line*ways*sets.
 *       L2 is the external PL310; ways/way-size read from AUX (matches l2_cache.c).
 *       Raw register values are printed so the decode can be verified by hand.
 */
static void xplat_log_cache(void)
{
    uint32_t clidr = XPLAT_SCB_CLIDR;
    uint32_t ctr = XPLAT_SCB_CTR;
    uint32_t csselr_save = XPLAT_SCB_CSSELR;
    uint32_t ccsidr;
    uint32_t line;
    uint32_t ways;
    uint32_t sets;
    uint32_t size;

    /* L1 D-cache: CSSELR InD=0, Level=0 */
    XPLAT_SCB_CSSELR = 0U;
    __asm__ volatile("dsb" ::: "memory");
    ccsidr = XPLAT_SCB_CCSIDR;
    line = 1UL << ((ccsidr & 0x7UL) + 4UL);
    ways = ((ccsidr >> 3) & 0x3FFUL) + 1UL;
    sets = ((ccsidr >> 13) & 0x7FFUL) + 1UL;
    size = line * ways * sets;
    bk_printf("[xplat] CACHE L1D CCSIDR=0x%08x line=%u ways=%u sets=%u size=%uKB\r\n",
              (unsigned int)ccsidr, (unsigned int)line, (unsigned int)ways,
              (unsigned int)sets, (unsigned int)(size / 1024U));

    /* L1 I-cache: CSSELR InD=1 */
    XPLAT_SCB_CSSELR = 1U;
    __asm__ volatile("dsb" ::: "memory");
    ccsidr = XPLAT_SCB_CCSIDR;
    line = 1UL << ((ccsidr & 0x7UL) + 4UL);
    ways = ((ccsidr >> 3) & 0x3FFUL) + 1UL;
    sets = ((ccsidr >> 13) & 0x7FFUL) + 1UL;
    size = line * ways * sets;
    bk_printf("[xplat] CACHE L1I CCSIDR=0x%08x line=%u ways=%u sets=%u size=%uKB\r\n",
              (unsigned int)ccsidr, (unsigned int)line, (unsigned int)ways,
              (unsigned int)sets, (unsigned int)(size / 1024U));

    XPLAT_SCB_CSSELR = csselr_save;
    __asm__ volatile("dsb" ::: "memory");
    bk_printf("[xplat] CACHE CLIDR=0x%08x CTR=0x%08x\r\n",
              (unsigned int)clidr, (unsigned int)ctr);

#if CONFIG_L2_CACHE_ENABLE
    {
        uint32_t l2_id = *((volatile uint32_t *)(XPLAT_L2C_PL310_BASE + 0x000UL));
        uint32_t l2_type = *((volatile uint32_t *)(XPLAT_L2C_PL310_BASE + 0x004UL));
        uint32_t l2_aux = *((volatile uint32_t *)(XPLAT_L2C_PL310_BASE + 0x104UL));
        /* PL310 AUX (per L2C-310 TRM / Linux cache-l2x0.h):
         *   way-size field [19:17], encoding field v -> 16KB<<(v-1): 1=16,2=32,3=64,4=128,5=256,6=512 KB/way;
         *   associativity bit[16]: 0=8-way, 1=16-way.
         * BK7259 AP: aux=0x02020000 -> field=1 (16KB/way), 8-way -> 128KB (matches chip architecture). */
        uint32_t aux_waysz = (l2_aux >> 17) & 0x7UL;
        uint32_t aux_assoc = (l2_aux >> 16) & 0x1UL;
        uint32_t l2_way_kb = (aux_waysz >= 1U) ? (16UL << (aux_waysz - 1U)) : 0UL;
        uint32_t l2_ways = (aux_assoc != 0U) ? 16U : 8U;
        uint32_t l2_total_kb = l2_ways * l2_way_kb;

        bk_printf("[xplat] CACHE L2(PL310) id=0x%08x type=0x%08x aux=0x%08x -> "
                  "ways=%u way=%uKB line=32 size=%uKB\r\n",
                  (unsigned int)l2_id, (unsigned int)l2_type, (unsigned int)l2_aux,
                  (unsigned int)l2_ways, (unsigned int)l2_way_kb,
                  (unsigned int)l2_total_kb);
    }
#else
    bk_printf("[xplat] CACHE L2 disabled (CONFIG_L2_CACHE_ENABLE=0)\r\n");
#endif
}

/**
 * @brief Short label for an ARMv8-M MAIR attribute byte
 * @param[in] b MAIR byte (Outer<<4 | Inner)
 * @return label; caller also prints raw hex so decode can be verified
 */
static const char *xplat_mair_label(uint8_t b)
{
    switch (b) {
    case 0x00U:
        return "Device-nGnRnE";
    case 0x44U:
        return "Normal non-cacheable";
    case 0xaaU:
        return "Normal WT RA (cacheable)";
    case 0xbbU:
        return "Normal WT RA+WA (cacheable)";
    case 0xffU:
        return "Normal WB RA+WA (cacheable)";
    case 0xf4U:
        return "Normal WB(L2) non-cache(L1)";
    default:
        break;
    }
    {
        uint8_t o = (uint8_t)((b >> 4) & 0xFU);

        if (o <= 0x1U) {
            return "Device?";
        }
        if (o == 0x4U) {
            return "Normal non-cacheable?";
        }
        if ((o >= 0x8U) && (o <= 0xBU)) {
            return "Normal WT cacheable?";
        }
        if (o >= 0xCU) {
            return "Normal WB cacheable?";
        }
    }
    return "reserved?";
}

/**
 * @brief Dump live MPU regions + MAIR attributes (PMSAv8)
 * @return none
 * @note RBAR = BASE[31:5]/SH[4:3]/AP[2:1]/XN[0];
 *       RLAR = LIMIT[31:5]/AttrIndx[4:1]/EN[0]. Raw values printed for hand check.
 */
static void xplat_log_mpu(void)
{
    uint32_t type = XPLAT_MPU_TYPE;
    uint32_t nreg = (type >> 8) & 0xFFU;
    uint32_t mair0 = XPLAT_MPU_MAIR0;
    uint32_t mair1 = XPLAT_MPU_MAIR1;
    uint32_t i;
    struct {
        uint32_t base;
        uint32_t limit;
        uint32_t attr;
        uint32_t en;
    } r[16];

    bk_printf("[xplat] MPU TYPE=0x%08x regions=%u ctrl=0x%08x (%s)\r\n",
              (unsigned int)type, (unsigned int)nreg,
              (unsigned int)XPLAT_MPU_CTRL,
              (XPLAT_MPU_CTRL & 0x1U) ? "ENABLED" : "disabled");

    for (i = 0U; i < 8U; i++) {
        uint32_t mair = (i < 4U) ? mair0 : mair1;
        uint8_t b = (uint8_t)((mair >> ((i % 4U) * 8U)) & 0xFFU);

        bk_printf("[xplat] MPU attr[%u]=0x%02x  %s\r\n",
                  (unsigned int)i, (unsigned int)b, xplat_mair_label(b));
    }

    for (i = 0U; (i < nreg) && (i < 16U); i++) {
        uint32_t rbar;
        uint32_t rlar;

        XPLAT_MPU_RNR = i;
        __asm__ volatile("dsb" ::: "memory");
        rbar = XPLAT_MPU_RBAR;
        rlar = XPLAT_MPU_RLAR;
        r[i].base = rbar & 0xFFFFFFE0UL;
        r[i].limit = (rlar & 0xFFFFFFE0UL) | 0x1FUL;
        r[i].attr = (rlar >> 1) & 0x7UL;
        r[i].en = rlar & 0x1UL;
        if (r[i].en == 0U) {
            bk_printf("[xplat] MPU r[%02u] -- disabled\r\n", (unsigned int)i);
        } else {
            uint32_t sh = (rbar >> 3) & 0x3UL;
            uint32_t ap = (rbar >> 1) & 0x3UL;
            uint32_t xn = rbar & 0x1UL;

            bk_printf("[xplat] MPU r[%02u] EN 0x%08x-0x%08x attr=%u sh=%u ap=%u xn=%u\r\n",
                      (unsigned int)i,
                      (unsigned int)r[i].base, (unsigned int)r[i].limit,
                      (unsigned int)r[i].attr, (unsigned int)sh,
                      (unsigned int)ap, (unsigned int)xn);
        }
    }

    /* Highlight which enabled region covers SRAM / PSRAM (answers cacheability). */
    {
        const struct {
            const char *name;
            uint32_t addr;
        } keys[] = {
            { "SRAM  ", 0x28000000UL },
            { "PSRAM ", 0x60000000UL },
        };
        uint32_t k;

        for (k = 0U; k < (uint32_t)(sizeof(keys) / sizeof(keys[0])); k++) {
            uint32_t found = 0xFFFFFFFFU;

            for (i = 0U; (i < nreg) && (i < 16U); i++) {
                if ((r[i].en != 0U) && (keys[k].addr >= r[i].base) &&
                    (keys[k].addr <= r[i].limit)) {
                    found = i;
                    break;
                }
            }
            if (found < 16U) {
                uint32_t mair = (r[found].attr < 4U) ? mair0 : mair1;
                uint8_t b = (uint8_t)((mair >> ((r[found].attr % 4U) * 8U)) & 0xFFU);

                bk_printf("[xplat] MPU %s @0x%08x -> r[%u] attr=%u =0x%02x (%s)\r\n",
                          keys[k].name, (unsigned int)keys[k].addr,
                          (unsigned int)found, (unsigned int)r[found].attr,
                          (unsigned int)b, xplat_mair_label(b));
            } else {
                bk_printf("[xplat] MPU %s @0x%08x -> no enabled region (default map)\r\n",
                          keys[k].name, (unsigned int)keys[k].addr);
            }
        }
    }
}

/**
 * @brief Log cfg / PM / hardware mux / measured CPU frequency
 * @return none
 */
static void xplat_log_freq(void)
{
    pm_cpu_freq_e pm = bk_pm_current_max_cpu_freq_get();
    uint32_t rt = xplat_pm_freq_to_mhz(pm);
    uint32_t cksel = 0U;
    uint32_t div = 0U;
    uint32_t hw = xplat_hw_mhz(&cksel, &div);
    uint32_t meas = xplat_meas_mhz();

    s_xplat_meas_mhz = meas;
    if ((hw == 0U) && (meas != 0U)) {
        hw = meas;
    }

    bk_printf("[xplat] INFO freq cfg_mhz=%u runtime_mhz=%u pm_enum=%d "
              "hw_mhz=%u cksel=%u div=%u meas_mhz=%u\r\n",
              (unsigned int)xplat_cfg_mhz(),
              (unsigned int)rt,
              (int)pm,
              (unsigned int)hw,
              (unsigned int)cksel,
              (unsigned int)div,
              (unsigned int)meas);

    xplat_log_cache();
}

/**
 * @brief Record table capacity for mode (keeps PSRAM/SRAM alloc small for light modes)
 * @param[in] mode XPLAT_MODE_*
 * @return entry count
 */
static uint32_t xplat_rec_cap_for_mode(int mode)
{
    switch (mode) {
    case XPLAT_MODE_MPU:
        return 4U;
    case XPLAT_MODE_SRAM:
    case XPLAT_MODE_PSRAM:
    case XPLAT_MODE_PSRAM_NC:
    case XPLAT_MODE_FLOAT:
    case XPLAT_MODE_LATX:
        return 64U;
    case XPLAT_MODE_MIN:
        return 96U;
    case XPLAT_MODE_DMA:
    case XPLAT_MODE_CONTEND:
        return 192U;
    case XPLAT_MODE_FULL:
    default:
        return XPLAT_REC_MAX;
    }
}

/**
 * @brief Allocate summary record table (prefer PSRAM)
 * @param[in] cap entry count
 * @return 0 ok, -1 fail
 */
static int xplat_rec_table_alloc(uint32_t cap)
{
    uint32_t bytes;

    if (cap == 0U) {
        cap = XPLAT_REC_MAX;
    }
    bytes = cap * (uint32_t)sizeof(xplat_rec_t);
    s_xplat_recs = NULL;
    s_xplat_recs_psram = 0;
    s_xplat_rec_cap = 0U;
    s_xplat_rec_cnt = 0U;

#if CONFIG_PSRAM
    s_xplat_recs = (xplat_rec_t *)psram_malloc(bytes);
    if (s_xplat_recs != NULL) {
        s_xplat_recs_psram = 1;
    }
#endif
    if (s_xplat_recs == NULL) {
        s_xplat_recs = (xplat_rec_t *)os_malloc(bytes);
        s_xplat_recs_psram = 0;
    }
    if (s_xplat_recs == NULL) {
        bk_printf("[xplat] ERROR rec table alloc failed bytes=%u\r\n",
                  (unsigned int)bytes);
        return -1;
    }
    s_xplat_rec_cap = cap;
    os_memset(s_xplat_recs, 0, bytes);
    bk_printf("[xplat] INFO rec table cap=%u bytes=%u mem=%s\r\n",
              (unsigned int)cap,
              (unsigned int)bytes,
              (s_xplat_recs_psram != 0) ? "psram" : "sram");
    return 0;
}

/**
 * @brief Free summary record table
 * @return none
 */
static void xplat_rec_table_free(void)
{
    if (s_xplat_recs == NULL) {
        return;
    }
    if (s_xplat_recs_psram != 0) {
#if CONFIG_PSRAM
        psram_free(s_xplat_recs);
#endif
    } else {
        os_free(s_xplat_recs);
    }
    s_xplat_recs = NULL;
    s_xplat_rec_cap = 0U;
    s_xplat_rec_cnt = 0U;
    s_xplat_recs_psram = 0;
}

/**
 * @brief Reset recorded results for a new run
 * @return none
 */
static void xplat_rec_reset(void)
{
    s_xplat_rec_cnt = 0U;
    if ((s_xplat_recs != NULL) && (s_xplat_rec_cap > 0U)) {
        os_memset(s_xplat_recs, 0, s_xplat_rec_cap * sizeof(xplat_rec_t));
    }
}

/**
 * @brief Append one recorded metric
 * @param[in] kind record kind
 * @param[in] case_name case id
 * @param[in] mem memory/detail tag
 * @param[in] size size bytes (0 if N/A)
 * @param[in] metric primary metric
 * @param[in] us elapsed us
 * @return none
 */
static void xplat_rec_add(xplat_rec_kind_e kind,
                          const char *case_name,
                          const char *mem,
                          uint32_t size,
                          float metric,
                          uint64_t us)
{
    xplat_rec_t *r;

    if ((s_xplat_recs == NULL) || (s_xplat_rec_cnt >= s_xplat_rec_cap)) {
        return;
    }
    r = &s_xplat_recs[s_xplat_rec_cnt++];
    r->kind = (uint8_t)kind;
    r->size = size;
    r->metric = metric;
    r->us = us;
    r->case_name[0] = '\0';
    r->mem[0] = '\0';
    if (case_name != NULL) {
        os_strlcpy(r->case_name, case_name, sizeof(r->case_name));
    }
    if (mem != NULL) {
        os_strlcpy(r->mem, mem, sizeof(r->mem));
    }
}

/**
 * @brief Format size for summary columns
 * @param[in] size bytes
 * @param[out] buf output
 * @param[in] buflen buffer length
 * @return none
 */
static void xplat_fmt_size(uint32_t size, char *buf, size_t buflen)
{
    if ((buf == NULL) || (buflen == 0U)) {
        return;
    }
    if (size >= 1024U) {
        (void)snprintf(buf, buflen, "%uK", (unsigned int)(size / 1024U));
    } else {
        (void)snprintf(buf, buflen, "%u", (unsigned int)size);
    }
}

/**
 * @brief Find recorded bandwidth metric
 * @param[in] case_name case id
 * @param[in] mem memory tag
 * @param[in] size bytes
 * @param[out] metric metric out
 * @return 1 found, 0 missing
 */
static int xplat_rec_find_bw(const char *case_name,
                             const char *mem,
                             uint32_t size,
                             float *metric)
{
    uint32_t i;

    for (i = 0U; i < s_xplat_rec_cnt; i++) {
        if ((s_xplat_recs[i].kind == XPLAT_REC_BW) &&
            (os_strcmp(s_xplat_recs[i].case_name, case_name) == 0) &&
            (os_strcmp(s_xplat_recs[i].mem, mem) == 0) &&
            (s_xplat_recs[i].size == size)) {
            if (metric != NULL) {
                *metric = s_xplat_recs[i].metric;
            }
            return 1;
        }
    }
    if (metric != NULL) {
        *metric = 0.0f;
    }
    return 0;
}

/**
 * @brief Find recorded latency metric
 * @param[in] case_name case id
 * @param[in] mem memory tag
 * @param[in] size working-set bytes
 * @param[out] metric metric out
 * @return 1 found, 0 missing
 */
static int xplat_rec_find_lat(const char *case_name,
                              const char *mem,
                              uint32_t size,
                              float *metric)
{
    uint32_t i;

    for (i = 0U; i < s_xplat_rec_cnt; i++) {
        if ((s_xplat_recs[i].kind == XPLAT_REC_LAT) &&
            (os_strcmp(s_xplat_recs[i].case_name, case_name) == 0) &&
            (os_strcmp(s_xplat_recs[i].mem, mem) == 0) &&
            (s_xplat_recs[i].size == size)) {
            if (metric != NULL) {
                *metric = s_xplat_recs[i].metric;
            }
            return 1;
        }
    }
    if (metric != NULL) {
        *metric = 0.0f;
    }
    return 0;
}

/**
 * @brief Find recorded ops metric
 * @param[in] case_name case id
 * @param[in] detail detail tag
 * @param[out] metric metric out
 * @param[out] size_out optional size/count field
 * @return 1 found, 0 missing
 */
static int xplat_rec_find_ops(const char *case_name,
                              const char *detail,
                              float *metric,
                              uint32_t *size_out)
{
    uint32_t i;

    for (i = 0U; i < s_xplat_rec_cnt; i++) {
        if ((s_xplat_recs[i].kind == XPLAT_REC_OPS) &&
            (os_strcmp(s_xplat_recs[i].case_name, case_name) == 0) &&
            (os_strcmp(s_xplat_recs[i].mem, detail) == 0)) {
            if (metric != NULL) {
                *metric = s_xplat_recs[i].metric;
            }
            if (size_out != NULL) {
                *size_out = s_xplat_recs[i].size;
            }
            return 1;
        }
    }
    if (metric != NULL) {
        *metric = 0.0f;
    }
    if (size_out != NULL) {
        *size_out = 0U;
    }
    return 0;
}

/**
 * @brief Find recorded CPU metric
 * @param[in] case_name case id
 * @param[in] mem memory/domain tag ("" allowed)
 * @param[out] metric metric out
 * @return 1 found, 0 missing
 */
static int xplat_rec_find_cpu(const char *case_name,
                              const char *mem,
                              float *metric)
{
    uint32_t i;

    for (i = 0U; i < s_xplat_rec_cnt; i++) {
        if ((s_xplat_recs[i].kind == XPLAT_REC_CPU) &&
            (os_strcmp(s_xplat_recs[i].case_name, case_name) == 0) &&
            (os_strcmp(s_xplat_recs[i].mem, (mem != NULL) ? mem : "") == 0)) {
            if (metric != NULL) {
                *metric = s_xplat_recs[i].metric;
            }
            return 1;
        }
    }
    if (metric != NULL) {
        *metric = 0.0f;
    }
    return 0;
}

/**
 * @brief Format summary cell from optional metric
 * @param[in] found 1 if value present
 * @param[in] v metric
 * @param[in] prec printf precision (1 or 2)
 * @param[out] buf cell text
 * @param[in] buflen buffer length
 * @return none
 */
static void xplat_fmt_cell(int found, float v, int prec, char *buf, size_t buflen)
{
    if ((buf == NULL) || (buflen == 0U)) {
        return;
    }
    if (found == 0) {
        (void)snprintf(buf, buflen, "-");
        return;
    }
    if (prec >= 3) {
        (void)snprintf(buf, buflen, "%.3f", v);
    } else if (prec == 2) {
        (void)snprintf(buf, buflen, "%.2f", v);
    } else {
        (void)snprintf(buf, buflen, "%.1f", v);
    }
}

/**
 * @brief Format one bandwidth summary cell from recorded MB/s
 * @param[in] case_name case id
 * @param[in] mem memory/domain tag
 * @param[in] size bytes
 * @param[out] buf cell text
 * @param[in] buflen buffer length
 * @return none
 */
static void xplat_fmt_bw_cell(const char *case_name,
                              const char *mem,
                              uint32_t size,
                              char *buf,
                              size_t buflen)
{
    float v = 0.0f;
    int found;

    found = xplat_rec_find_bw(case_name, mem, size, &v);
    if ((found == 0) && (buf != NULL) && (buflen != 0U) &&
        (os_strcmp(case_name, "memcpy") == 0)) {
        (void)snprintf(buf, buflen, "SKIP");
        return;
    }
    xplat_fmt_cell(found, v, 1, buf, buflen);
}

/**
 * @brief Collect unique sizes from BW engine/cross or LAT records
 * @param[in] kind record kind
 * @param[in] case_names NULL-terminated list (NULL entry ends; empty list => any case)
 * @param[in] mems NULL-terminated list of mem tags (NULL => any mem)
 * @param[out] sizes size table
 * @param[in] max_n capacity
 * @return count
 */
static uint32_t xplat_summary_collect_sizes(uint8_t kind,
                                           const char *const *case_names,
                                           const char *const *mems,
                                           uint32_t *sizes,
                                           uint32_t max_n)
{
    uint32_t i;
    uint32_t cnt = 0U;

    if ((sizes == NULL) || (max_n == 0U)) {
        return 0U;
    }
    for (i = 0U; i < s_xplat_rec_cnt; i++) {
        uint32_t j;
        uint32_t s;
        int exists = 0;
        int case_ok = 0;
        int mem_ok = 0;

        if (s_xplat_recs[i].kind != kind) {
            continue;
        }
        if (s_xplat_recs[i].size == 0U) {
            continue;
        }
        if (case_names == NULL) {
            case_ok = 1;
        } else {
            for (j = 0U; case_names[j] != NULL; j++) {
                if (os_strcmp(s_xplat_recs[i].case_name, case_names[j]) == 0) {
                    case_ok = 1;
                    break;
                }
            }
        }
        if (case_ok == 0) {
            continue;
        }
        if (mems == NULL) {
            mem_ok = 1;
        } else {
            for (j = 0U; mems[j] != NULL; j++) {
                if (os_strcmp(s_xplat_recs[i].mem, mems[j]) == 0) {
                    mem_ok = 1;
                    break;
                }
            }
        }
        if (mem_ok == 0) {
            continue;
        }
        s = s_xplat_recs[i].size;
        for (j = 0U; j < cnt; j++) {
            if (sizes[j] == s) {
                exists = 1;
                break;
            }
        }
        if ((exists == 0) && (cnt < max_n)) {
            sizes[cnt++] = s;
        }
    }
    for (i = 0U; i + 1U < cnt; i++) {
        uint32_t j;
        for (j = i + 1U; j < cnt; j++) {
            if (sizes[j] < sizes[i]) {
                uint32_t tmp = sizes[i];
                sizes[i] = sizes[j];
                sizes[j] = tmp;
            }
        }
    }
    return cnt;
}

/**
 * @brief Print one bandwidth summary row for a size
 * @param[in] size bytes
 * @return none
 */
static void xplat_summary_bw_row(uint32_t size)
{
    char sz[8];
    float v;
    char a[16];
    char b[16];
    char c[16];
    char d[16];
    char e[16];
    char f[16];

    xplat_fmt_size(size, sz, sizeof(sz));

    if (xplat_rec_find_bw("memcpy", "sram", size, &v)) {
        (void)snprintf(a, sizeof(a), "%.1f", v);
    } else {
        (void)snprintf(a, sizeof(a), "SKIP");
    }
    if (xplat_rec_find_bw("memcpy", "psram", size, &v)) {
        (void)snprintf(b, sizeof(b), "%.1f", v);
    } else {
        (void)snprintf(b, sizeof(b), "SKIP");
    }
    if (xplat_rec_find_bw("fill_u32", "sram", size, &v)) {
        (void)snprintf(c, sizeof(c), "%.1f", v);
    } else {
        (void)snprintf(c, sizeof(c), "-");
    }
    if (xplat_rec_find_bw("fill_u32", "psram", size, &v)) {
        (void)snprintf(d, sizeof(d), "%.1f", v);
    } else {
        (void)snprintf(d, sizeof(d), "-");
    }
    if (xplat_rec_find_bw("seq_read", "sram", size, &v)) {
        (void)snprintf(e, sizeof(e), "%.1f", v);
    } else {
        (void)snprintf(e, sizeof(e), "-");
    }
    if (xplat_rec_find_bw("seq_read", "psram", size, &v)) {
        (void)snprintf(f, sizeof(f), "%.1f", v);
    } else {
        (void)snprintf(f, sizeof(f), "-");
    }

    bk_printf("[xplat] SUM  %-5s | %7s %7s | %7s %7s | %7s %7s\r\n",
              sz, a, b, c, d, e, f);
}

/**
 * @brief Print one single-domain BW summary row
 * @param[in] mem memory tag ("sram" / "psram")
 * @param[in] size bytes
 * @return none
 */
static void xplat_summary_mem_bw_row(const char *mem, uint32_t size)
{
    char sz[8];
    char a[16];
    char b[16];
    char c[16];
    char d[16];
    char e[16];

    xplat_fmt_size(size, sz, sizeof(sz));
    xplat_fmt_bw_cell("memcpy", mem, size, a, sizeof(a));
    xplat_fmt_bw_cell("fill_u32", mem, size, b, sizeof(b));
    xplat_fmt_bw_cell("seq_copy", mem, size, c, sizeof(c));
    xplat_fmt_bw_cell("seq_copy_scalar", mem, size, d, sizeof(d));
    xplat_fmt_bw_cell("seq_read", mem, size, e, sizeof(e));
    bk_printf("[xplat] SUM  %-5s | %8s %7s %7s %7s %7s\r\n",
              sz, a, b, c, d, e);
}

/**
 * @brief Print single-domain end-of-run summary (sram / psram / psram_nc)
 * @param[in] mode_name mode string for header
 * @param[in] mem memory tag ("sram" / "psram" / "psram_nc")
 * @return none
 */
static void xplat_print_summary_mem(const char *mode_name, const char *mem)
{
    uint32_t i;
    uint32_t size_seen[16];
    uint32_t size_cnt = 0U;
    uint32_t mhz;
    int skip_n = 0;

    if ((mode_name == NULL) || (mem == NULL)) {
        return;
    }
    mhz = xplat_current_freq_mhz();
    bk_printf("[xplat] ========== SUMMARY mode=%s cfg_mhz=%u meas_mhz=%u ==========\r\n",
              mode_name,
              (unsigned int)xplat_cfg_mhz(),
              (unsigned int)mhz);
    if (os_strcmp(mem, "psram_nc") == 0) {
        bk_printf("[xplat] SUM  -- PSRAM-NC Bandwidth MB/s --\r\n");
        bk_printf("[xplat] SUM  size  | memcpy_n  fill_n scopy_n scals_n  seqr_n\r\n");
    } else if (os_strcmp(mem, "psram") == 0) {
        bk_printf("[xplat] SUM  -- PSRAM Bandwidth MB/s --\r\n");
        bk_printf("[xplat] SUM  size  | memcpy_p  fill_p scopy_p scals_p  seqr_p\r\n");
    } else {
        bk_printf("[xplat] SUM  -- SRAM Bandwidth MB/s --\r\n");
        bk_printf("[xplat] SUM  size  | memcpy_s  fill_s scopy_s scals_s  seqr_s\r\n");
    }

    for (i = 0U; i < s_xplat_rec_cnt; i++) {
        uint32_t s;
        uint32_t j;
        int exists = 0;

        if (s_xplat_recs[i].kind != XPLAT_REC_BW) {
            continue;
        }
        if (os_strcmp(s_xplat_recs[i].mem, mem) != 0) {
            continue;
        }
        if ((os_strcmp(s_xplat_recs[i].case_name, "memcpy") != 0) &&
            (os_strcmp(s_xplat_recs[i].case_name, "fill_u32") != 0) &&
            (os_strcmp(s_xplat_recs[i].case_name, "seq_copy") != 0) &&
            (os_strcmp(s_xplat_recs[i].case_name, "seq_copy_scalar") != 0) &&
            (os_strcmp(s_xplat_recs[i].case_name, "seq_read") != 0)) {
            continue;
        }
        s = s_xplat_recs[i].size;
        for (j = 0U; j < size_cnt; j++) {
            if (size_seen[j] == s) {
                exists = 1;
                break;
            }
        }
        if ((exists == 0) && (size_cnt < 16U)) {
            size_seen[size_cnt++] = s;
        }
    }
    for (i = 0U; i + 1U < size_cnt; i++) {
        uint32_t j;

        for (j = i + 1U; j < size_cnt; j++) {
            if (size_seen[j] < size_seen[i]) {
                uint32_t tmp = size_seen[i];

                size_seen[i] = size_seen[j];
                size_seen[j] = tmp;
            }
        }
    }
    if (size_cnt == 0U) {
        bk_printf("[xplat] SUM  (none)\r\n");
    } else {
        for (i = 0U; i < size_cnt; i++) {
            xplat_summary_mem_bw_row(mem, size_seen[i]);
        }
    }

    bk_printf("[xplat] SUM  -- SKIP --\r\n");
    for (i = 0U; i < s_xplat_rec_cnt; i++) {
        if (s_xplat_recs[i].kind != XPLAT_REC_SKIP) {
            continue;
        }
        skip_n++;
        bk_printf("[xplat] SUM  %s %s size=%u\r\n",
                  s_xplat_recs[i].case_name,
                  s_xplat_recs[i].mem,
                  (unsigned int)s_xplat_recs[i].size);
    }
    if (skip_n == 0) {
        bk_printf("[xplat] SUM  (none)\r\n");
    }
    if (os_strcmp(mem, "psram_nc") == 0) {
        bk_printf("[xplat] SUM  note: *_n from MEM_SLAB_HEAP_UNCODED (0x60 MPU non-cacheable)\r\n");
        bk_printf("[xplat] SUM  note: memcpy_n=libc; fill_n=fill_u32; scopy_n=x16; scals_n=per-word; seqr_n=seq_read\r\n");
    } else if (os_strcmp(mem, "psram") == 0) {
        bk_printf("[xplat] SUM  note: memcpy_p=libc; fill_p=fill_u32; scopy_p=x16; scals_p=per-word; seqr_p=seq_read\r\n");
    } else {
        bk_printf("[xplat] SUM  note: memcpy_s=libc; fill_s=fill_u32; scopy_s=x16; scals_s=per-word; seqr_s=seq_read\r\n");
    }
    bk_printf("[xplat] ========== SUMMARY END ==========\r\n");
}

/**
 * @brief Print end-of-run summary
 * @param[in] mode_name "min" or "full"
 * @return none
 */
static void xplat_print_summary(const char *mode_name)
{
    uint32_t i;
    uint32_t size_seen[16];
    uint32_t size_cnt = 0U;
    float align_min_s = 0.0f;
    float align_max_s = 0.0f;
    float align_min_p = 0.0f;
    float align_max_p = 0.0f;
    int align_s_n = 0;
    int align_p_n = 0;

    bk_printf("[xplat] ========== SUMMARY mode=%s cfg_mhz=%u runtime_mhz=%u ==========\r\n",
              (mode_name != NULL) ? mode_name : "?",
              (unsigned int)xplat_cfg_mhz(),
              (unsigned int)xplat_runtime_mhz());

    /* CPU */
    bk_printf("[xplat] SUM  -- CPU (Mops/s) --\r\n");
    {
        int cpu_n = 0;

        for (i = 0U; i < s_xplat_rec_cnt; i++) {
            if (s_xplat_recs[i].kind != XPLAT_REC_CPU) {
                continue;
            }
            /* Contended CPU samples live under Contend section */
            if ((os_strcmp(s_xplat_recs[i].case_name, "cpu_mul_base") == 0) ||
                (os_strcmp(s_xplat_recs[i].case_name, "cpu_mul_ct") == 0) ||
                (os_strcmp(s_xplat_recs[i].case_name, "cpu_mul_hpdma_ct") == 0)) {
                continue;
            }
            cpu_n++;
            bk_printf("[xplat] SUM  %-12s %8.2f  (us=%llu)\r\n",
                      s_xplat_recs[i].case_name,
                      s_xplat_recs[i].metric,
                      (unsigned long long)s_xplat_recs[i].us);
        }
        if (cpu_n == 0) {
            bk_printf("[xplat] SUM  (none)\r\n");
        }
    }

    /* Collect unique sizes from bw memcpy records */
    for (i = 0U; i < s_xplat_rec_cnt; i++) {
        uint32_t s;
        uint32_t j;
        int exists = 0;

        if (s_xplat_recs[i].kind != XPLAT_REC_BW) {
            continue;
        }
        if (os_strcmp(s_xplat_recs[i].case_name, "memcpy") != 0) {
            continue;
        }
        if ((os_strcmp(s_xplat_recs[i].mem, "sram") != 0) &&
            (os_strcmp(s_xplat_recs[i].mem, "psram") != 0)) {
            continue;
        }
        s = s_xplat_recs[i].size;
        for (j = 0U; j < size_cnt; j++) {
            if (size_seen[j] == s) {
                exists = 1;
                break;
            }
        }
        if ((exists == 0) && (size_cnt < 16U)) {
            size_seen[size_cnt++] = s;
        }
    }
    /* simple ascending sort */
    for (i = 0U; i + 1U < size_cnt; i++) {
        uint32_t j;
        for (j = i + 1U; j < size_cnt; j++) {
            if (size_seen[j] < size_seen[i]) {
                uint32_t tmp = size_seen[i];
                size_seen[i] = size_seen[j];
                size_seen[j] = tmp;
            }
        }
    }

    bk_printf("[xplat] SUM  -- Bandwidth MB/s (memset omitted: libc+cache inflated) --\r\n");
    if (size_cnt > 0U) {
        bk_printf("[xplat] SUM  size  | memcpy_s memcpy_p | fill_s  fill_p  | seqr_s  seqr_p\r\n");
        for (i = 0U; i < size_cnt; i++) {
            xplat_summary_bw_row(size_seen[i]);
        }
        bk_printf("[xplat] SUM  -- memcpy vs seq_copy MB/s (x16 vs scalar) --\r\n");
        bk_printf("[xplat] SUM  size  | memcpy_s memcpy_p | scopy_s scopy_p | scals_s scals_p\r\n");
        for (i = 0U; i < size_cnt; i++) {
            char sz[8];
            float v;
            char a[16];
            char b[16];
            char c[16];
            char d[16];
            char e[16];
            char f[16];

            xplat_fmt_size(size_seen[i], sz, sizeof(sz));
            if (xplat_rec_find_bw("memcpy", "sram", size_seen[i], &v)) {
                (void)snprintf(a, sizeof(a), "%.1f", v);
            } else {
                (void)snprintf(a, sizeof(a), "SKIP");
            }
            if (xplat_rec_find_bw("memcpy", "psram", size_seen[i], &v)) {
                (void)snprintf(b, sizeof(b), "%.1f", v);
            } else {
                (void)snprintf(b, sizeof(b), "SKIP");
            }
            if (xplat_rec_find_bw("seq_copy", "sram", size_seen[i], &v)) {
                (void)snprintf(c, sizeof(c), "%.1f", v);
            } else {
                (void)snprintf(c, sizeof(c), "-");
            }
            if (xplat_rec_find_bw("seq_copy", "psram", size_seen[i], &v)) {
                (void)snprintf(d, sizeof(d), "%.1f", v);
            } else {
                (void)snprintf(d, sizeof(d), "-");
            }
            if (xplat_rec_find_bw("seq_copy_scalar", "sram", size_seen[i], &v)) {
                (void)snprintf(e, sizeof(e), "%.1f", v);
            } else {
                (void)snprintf(e, sizeof(e), "-");
            }
            if (xplat_rec_find_bw("seq_copy_scalar", "psram", size_seen[i], &v)) {
                (void)snprintf(f, sizeof(f), "%.1f", v);
            } else {
                (void)snprintf(f, sizeof(f), "-");
            }
            bk_printf("[xplat] SUM  %-5s | %7s %7s | %7s %7s | %7s %7s\r\n",
                      sz, a, b, c, d, e, f);
        }
        bk_printf("[xplat] SUM  -- Random gather/scatter MB/s --\r\n");
        bk_printf("[xplat] SUM  size  | rdr_s   rdr_p   | rdw_s   rdw_p\r\n");
        for (i = 0U; i < size_cnt; i++) {
            char sz[8];
            float v;
            char a[16];
            char b[16];
            char c[16];
            char d[16];

            if (!xplat_rec_find_bw("rand_read", "sram", size_seen[i], &v) &&
                !xplat_rec_find_bw("rand_read", "psram", size_seen[i], &v) &&
                !xplat_rec_find_bw("rand_write", "sram", size_seen[i], &v) &&
                !xplat_rec_find_bw("rand_write", "psram", size_seen[i], &v)) {
                continue;
            }
            xplat_fmt_size(size_seen[i], sz, sizeof(sz));
            if (xplat_rec_find_bw("rand_read", "sram", size_seen[i], &v)) {
                (void)snprintf(a, sizeof(a), "%.1f", v);
            } else {
                (void)snprintf(a, sizeof(a), "-");
            }
            if (xplat_rec_find_bw("rand_read", "psram", size_seen[i], &v)) {
                (void)snprintf(b, sizeof(b), "%.1f", v);
            } else {
                (void)snprintf(b, sizeof(b), "-");
            }
            if (xplat_rec_find_bw("rand_write", "sram", size_seen[i], &v)) {
                (void)snprintf(c, sizeof(c), "%.1f", v);
            } else {
                (void)snprintf(c, sizeof(c), "-");
            }
            if (xplat_rec_find_bw("rand_write", "psram", size_seen[i], &v)) {
                (void)snprintf(d, sizeof(d), "%.1f", v);
            } else {
                (void)snprintf(d, sizeof(d), "-");
            }
            bk_printf("[xplat] SUM  %-5s | %7s %7s | %7s %7s\r\n",
                      sz, a, b, c, d);
        }
    } else {
        bk_printf("[xplat] SUM  (none)\r\n");
    }

    /* Flash XIP seq_read vs SRAM seq_read (same sizes / kernel) */
    {
        uint32_t xip_sizes[16];
        uint32_t xip_cnt = 0U;

        for (i = 0U; i < s_xplat_rec_cnt; i++) {
            uint32_t s;
            uint32_t j;
            int exists = 0;

            if ((s_xplat_recs[i].kind != XPLAT_REC_BW) ||
                (os_strcmp(s_xplat_recs[i].case_name, "seq_read") != 0) ||
                (os_strcmp(s_xplat_recs[i].mem, "flash_xip") != 0)) {
                continue;
            }
            s = s_xplat_recs[i].size;
            for (j = 0U; j < xip_cnt; j++) {
                if (xip_sizes[j] == s) {
                    exists = 1;
                    break;
                }
            }
            if ((exists == 0) && (xip_cnt < 16U)) {
                xip_sizes[xip_cnt++] = s;
            }
        }
        for (i = 0U; i + 1U < xip_cnt; i++) {
            uint32_t j;
            for (j = i + 1U; j < xip_cnt; j++) {
                if (xip_sizes[j] < xip_sizes[i]) {
                    uint32_t tmp = xip_sizes[i];
                    xip_sizes[i] = xip_sizes[j];
                    xip_sizes[j] = tmp;
                }
            }
        }
        if (xip_cnt > 0U) {
            bk_printf("[xplat] SUM  -- XIP seq_read vs seqr_s MB/s --\r\n");
            bk_printf("[xplat] SUM  size  | flash_xip  seqr_s\r\n");
            for (i = 0U; i < xip_cnt; i++) {
                char sz[8];
                float v;
                char a[16];
                char b[16];

                xplat_fmt_size(xip_sizes[i], sz, sizeof(sz));
                if (xplat_rec_find_bw("seq_read", "flash_xip", xip_sizes[i], &v)) {
                    (void)snprintf(a, sizeof(a), "%.1f", v);
                } else {
                    (void)snprintf(a, sizeof(a), "-");
                }
                if (xplat_rec_find_bw("seq_read", "sram", xip_sizes[i], &v)) {
                    (void)snprintf(b, sizeof(b), "%.1f", v);
                } else {
                    (void)snprintf(b, sizeof(b), "-");
                }
                bk_printf("[xplat] SUM  %-5s | %9s %7s\r\n", sz, a, b);
            }
        }
    }

    /* DMA / engine memcpy: size rows, engines as columns (per path) */
    {
        static const char *const eng_cases[] = {
            "hpdma_memcpy", "gdma_memcpy", "cpu_memcpy", NULL
        };
        static const char *const eng_paths[] = {
            "sram", "psram", "sram_to_psram", "psram_to_sram", NULL
        };
        uint32_t p;

        bk_printf("[xplat] SUM  -- Engine memcpy MB/s --\r\n");
        for (p = 0U; eng_paths[p] != NULL; p++) {
            const char *path = eng_paths[p];
            const char *path_mems[2];
            uint32_t eng_sizes[16];
            uint32_t eng_cnt;
            uint32_t si;

            path_mems[0] = path;
            path_mems[1] = NULL;
            eng_cnt = xplat_summary_collect_sizes(XPLAT_REC_BW, eng_cases, path_mems,
                                                  eng_sizes, 16U);
            if (eng_cnt == 0U) {
                continue;
            }
            bk_printf("[xplat] SUM  path=%s\r\n", path);
            bk_printf("[xplat] SUM  size  |   hpdma    gdma     cpu\r\n");
            for (si = 0U; si < eng_cnt; si++) {
                char sz[8];
                float v;
                char a[16];
                char b[16];
                char c[16];

                xplat_fmt_size(eng_sizes[si], sz, sizeof(sz));
                xplat_fmt_cell(xplat_rec_find_bw("hpdma_memcpy", path, eng_sizes[si], &v),
                               v, 1, a, sizeof(a));
                xplat_fmt_cell(xplat_rec_find_bw("gdma_memcpy", path, eng_sizes[si], &v),
                               v, 1, b, sizeof(b));
                xplat_fmt_cell(xplat_rec_find_bw("cpu_memcpy", path, eng_sizes[si], &v),
                               v, 1, c, sizeof(c));
                bk_printf("[xplat] SUM  %-5s | %7s %7s %7s\r\n", sz, a, b, c);
            }
        }
    }

    /* Cross memcpy: size | s2p p2s */
    {
        static const char *const cross_cases[] = { "memcpy", NULL };
        static const char *const cross_mems[] = {
            "sram_to_psram", "psram_to_sram", NULL
        };
        uint32_t cross_sizes[16];
        uint32_t cross_cnt;

        cross_cnt = xplat_summary_collect_sizes(XPLAT_REC_BW, cross_cases, cross_mems,
                                                cross_sizes, 16U);
        bk_printf("[xplat] SUM  -- Cross memcpy MB/s --\r\n");
        if (cross_cnt == 0U) {
            bk_printf("[xplat] SUM  (none)\r\n");
        } else {
            bk_printf("[xplat] SUM  size  |     s2p     p2s\r\n");
            for (i = 0U; i < cross_cnt; i++) {
                char sz[8];
                float v;
                char a[16];
                char b[16];

                xplat_fmt_size(cross_sizes[i], sz, sizeof(sz));
                xplat_fmt_cell(xplat_rec_find_bw("memcpy", "sram_to_psram",
                                                 cross_sizes[i], &v),
                               v, 1, a, sizeof(a));
                xplat_fmt_cell(xplat_rec_find_bw("memcpy", "psram_to_sram",
                                                 cross_sizes[i], &v),
                               v, 1, b, sizeof(b));
                bk_printf("[xplat] SUM  %-5s | %7s %7s\r\n", sz, a, b);
            }
        }
    }

    /* Latency: size rows, domains as columns */
    {
        static const char *const lat_dom_cases[] = {
            "latency_rand", "latency_seq", "latency_store", NULL
        };
        static const char *const lat_dom_mems[] = { "sram", "psram", NULL };
        static const char *const lat_x_cases[] = { "latency_cross", NULL };
        static const char *const lat_x_mems[] = {
            "idx_s_data_p", "idx_p_data_s", NULL
        };
        uint32_t lat_sizes[16];
        uint32_t lat_cnt;
        uint32_t lat_x_sizes[16];
        uint32_t lat_x_cnt;
        int lat_any = 0;

        lat_cnt = xplat_summary_collect_sizes(XPLAT_REC_LAT, lat_dom_cases, lat_dom_mems,
                                              lat_sizes, 16U);
        lat_x_cnt = xplat_summary_collect_sizes(XPLAT_REC_LAT, lat_x_cases, lat_x_mems,
                                                lat_x_sizes, 16U);
        bk_printf("[xplat] SUM  -- Latency ns/access --\r\n");
        if (lat_cnt > 0U) {
            lat_any = 1;
            bk_printf("[xplat] SUM  size  | rand_s  rand_p |  seq_s   seq_p | sto_s   sto_p\r\n");
            for (i = 0U; i < lat_cnt; i++) {
                char sz[8];
                float v;
                char a[16];
                char b[16];
                char c[16];
                char d[16];
                char e[16];
                char f[16];

                xplat_fmt_size(lat_sizes[i], sz, sizeof(sz));
                xplat_fmt_cell(xplat_rec_find_lat("latency_rand", "sram", lat_sizes[i], &v),
                               v, 2, a, sizeof(a));
                xplat_fmt_cell(xplat_rec_find_lat("latency_rand", "psram", lat_sizes[i], &v),
                               v, 2, b, sizeof(b));
                xplat_fmt_cell(xplat_rec_find_lat("latency_seq", "sram", lat_sizes[i], &v),
                               v, 2, c, sizeof(c));
                xplat_fmt_cell(xplat_rec_find_lat("latency_seq", "psram", lat_sizes[i], &v),
                               v, 2, d, sizeof(d));
                xplat_fmt_cell(xplat_rec_find_lat("latency_store", "sram", lat_sizes[i], &v),
                               v, 2, e, sizeof(e));
                xplat_fmt_cell(xplat_rec_find_lat("latency_store", "psram", lat_sizes[i], &v),
                               v, 2, f, sizeof(f));
                bk_printf("[xplat] SUM  %-5s | %6s %6s | %6s %6s | %6s %6s\r\n",
                          sz, a, b, c, d, e, f);
            }
        }
        if (lat_x_cnt > 0U) {
            lat_any = 1;
            bk_printf("[xplat] SUM  size  | xs_s_p  xs_p_s\r\n");
            for (i = 0U; i < lat_x_cnt; i++) {
                char sz[8];
                float v;
                char a[16];
                char b[16];

                xplat_fmt_size(lat_x_sizes[i], sz, sizeof(sz));
                xplat_fmt_cell(xplat_rec_find_lat("latency_cross", "idx_s_data_p",
                                                  lat_x_sizes[i], &v),
                               v, 2, a, sizeof(a));
                xplat_fmt_cell(xplat_rec_find_lat("latency_cross", "idx_p_data_s",
                                                  lat_x_sizes[i], &v),
                               v, 2, b, sizeof(b));
                bk_printf("[xplat] SUM  %-5s | %6s %6s\r\n", sz, a, b);
            }
        }
        if (lat_any == 0) {
            bk_printf("[xplat] SUM  (none)\r\n");
        }
    }

    /* Contend: peer memcpy + optional HPDMA (column tables) */
    {
        static const char *const peers[] = { "sram", "psram", NULL };
        uint32_t pi;
        int peer_n = 0;
        int has_hp;
        float v_alone = 0.0f;
        float v_busy = 0.0f;
        float v_drop = 0.0f;
        float v_bw = 0.0f;
        float v_hp = 0.0f;
        char a[16];
        char b[16];
        char c[16];
        char d[16];
        char drop_tag[24];

        for (pi = 0U; peers[pi] != NULL; pi++) {
            if (xplat_rec_find_cpu("cpu_mul_base", peers[pi], NULL) ||
                xplat_rec_find_cpu("cpu_mul_ct", peers[pi], NULL) ||
                xplat_rec_find_bw("memcpy_ct", peers[pi], XPLAT_CONTEND_SIZE, NULL) ||
                xplat_rec_find_bw("memcpy_ct", peers[pi], 16U * 1024U, NULL)) {
                peer_n++;
            }
        }
        has_hp = xplat_rec_find_cpu("cpu_mul_hpdma_ct", "sram", NULL) ||
                 xplat_rec_find_bw("hpdma_ct", "sram", XPLAT_CONTEND_SIZE, NULL);

        if ((peer_n > 0) || (has_hp != 0)) {
            bk_printf("[xplat] SUM  -- Contend: CPU vs peer memcpy --\r\n");
            if (peer_n == 0) {
                bk_printf("[xplat] SUM  (none)\r\n");
            } else {
                bk_printf("[xplat] SUM  peer  | mul_alone mul_busy drop%% | peer_MB/s\r\n");
                for (pi = 0U; peers[pi] != NULL; pi++) {
                    int has_alone;
                    int has_busy;
                    int has_drop;
                    int has_mbps;
                    uint32_t sz = XPLAT_CONTEND_SIZE;

                    has_alone = xplat_rec_find_cpu("cpu_mul_base", peers[pi], &v_alone);
                    has_busy = xplat_rec_find_cpu("cpu_mul_ct", peers[pi], &v_busy);
                    (void)snprintf(drop_tag, sizeof(drop_tag), "mul_vs_%s", peers[pi]);
                    has_drop = xplat_rec_find_ops("drop_cpu_pct", drop_tag, &v_drop, NULL);
                    has_mbps = xplat_rec_find_bw("memcpy_ct", peers[pi], sz, &v_bw);
                    if (has_mbps == 0) {
                        sz = 16U * 1024U;
                        has_mbps = xplat_rec_find_bw("memcpy_ct", peers[pi], sz, &v_bw);
                    }
                    if ((has_alone == 0) && (has_busy == 0) && (has_mbps == 0)) {
                        continue;
                    }
                    xplat_fmt_cell(has_alone, v_alone, 2, a, sizeof(a));
                    xplat_fmt_cell(has_busy, v_busy, 2, b, sizeof(b));
                    xplat_fmt_cell(has_drop, v_drop, 1, c, sizeof(c));
                    xplat_fmt_cell(has_mbps, v_bw, 1, d, sizeof(d));
                    bk_printf("[xplat] SUM  %-5s | %9s %8s %5s | %9s\r\n",
                              peers[pi], a, b, c, d);
                }
            }

            bk_printf("[xplat] SUM  -- Contend: CPU vs HPDMA --\r\n");
            if (has_hp == 0) {
                bk_printf("[xplat] SUM  (none)\r\n");
            } else {
                bk_printf("[xplat] SUM  path  | mul_busy hpdma_MB/s\r\n");
                xplat_fmt_cell(xplat_rec_find_cpu("cpu_mul_hpdma_ct", "sram", &v_busy),
                               v_busy, 2, a, sizeof(a));
                xplat_fmt_cell(xplat_rec_find_bw("hpdma_ct", "sram", XPLAT_CONTEND_SIZE, &v_hp),
                               v_hp, 1, b, sizeof(b));
                bk_printf("[xplat] SUM  %-5s | %8s %10s\r\n", "sram", a, b);
            }
        }
    }

    /* Align: min/max only */
    for (i = 0U; i < s_xplat_rec_cnt; i++) {
        if ((s_xplat_recs[i].kind != XPLAT_REC_BW) ||
            (os_strcmp(s_xplat_recs[i].case_name, "memcpy_align") != 0)) {
            continue;
        }
        if (os_strstr(s_xplat_recs[i].mem, "sram_") == s_xplat_recs[i].mem) {
            if (align_s_n == 0) {
                align_min_s = align_max_s = s_xplat_recs[i].metric;
            } else {
                if (s_xplat_recs[i].metric < align_min_s) {
                    align_min_s = s_xplat_recs[i].metric;
                }
                if (s_xplat_recs[i].metric > align_max_s) {
                    align_max_s = s_xplat_recs[i].metric;
                }
            }
            align_s_n++;
        } else if (os_strstr(s_xplat_recs[i].mem, "psram_") == s_xplat_recs[i].mem) {
            if (align_p_n == 0) {
                align_min_p = align_max_p = s_xplat_recs[i].metric;
            } else {
                if (s_xplat_recs[i].metric < align_min_p) {
                    align_min_p = s_xplat_recs[i].metric;
                }
                if (s_xplat_recs[i].metric > align_max_p) {
                    align_max_p = s_xplat_recs[i].metric;
                }
            }
            align_p_n++;
        }
    }
    if ((align_s_n > 0) || (align_p_n > 0)) {
        bk_printf("[xplat] SUM  -- Align memcpy 4K MB/s (min..max) --\r\n");
        if (align_s_n > 0) {
            bk_printf("[xplat] SUM  sram   %.1f .. %.1f\r\n", align_min_s, align_max_s);
        }
        if (align_p_n > 0) {
            bk_printf("[xplat] SUM  psram  %.1f .. %.1f\r\n", align_min_p, align_max_p);
        }
    }

    /* IRQ: latency avg/min/max + critical-section hold */
    {
        float v_avg = 0.0f;
        float v_min = 0.0f;
        float v_max = 0.0f;
        float v_hold = 0.0f;
        float v_tgt = 0.0f;
        uint32_t n_samp = 0U;
        int has_avg;
        int has_min;
        int has_max;
        int has_hold;
        int has_tgt;
        char a[16];
        char b[16];
        char c[16];
        char d[16];

        has_avg = xplat_rec_find_ops("irq_latency", "timer_entry", &v_avg, NULL);
        has_min = xplat_rec_find_ops("irq_latency", "us_min", &v_min, &n_samp);
        has_max = xplat_rec_find_ops("irq_latency", "us_max", &v_max, NULL);
        has_hold = xplat_rec_find_ops("crit_hold", "disable_int", &v_hold, NULL);
        has_tgt = xplat_rec_find_ops("crit_hold", "target_us", &v_tgt, NULL);

        bk_printf("[xplat] SUM  -- IRQ (us) --\r\n");
        if ((has_avg == 0) && (has_min == 0) && (has_hold == 0)) {
            bk_printf("[xplat] SUM  (none)\r\n");
        } else {
            bk_printf("[xplat] SUM  case         detail        |     avg     min     max    n\r\n");
            xplat_fmt_cell(has_avg, v_avg, 3, a, sizeof(a));
            xplat_fmt_cell(has_min, v_min, 3, b, sizeof(b));
            xplat_fmt_cell(has_max, v_max, 3, c, sizeof(c));
            if (n_samp > 0U) {
                (void)snprintf(d, sizeof(d), "%u", (unsigned int)n_samp);
            } else {
                (void)snprintf(d, sizeof(d), "-");
            }
            bk_printf("[xplat] SUM  %-12s %-12s | %7s %7s %7s %4s\r\n",
                      "irq_latency", "timer_entry", a, b, c, d);
            /* crit_hold: avg=actual hold, min=target */
            xplat_fmt_cell(has_hold, v_hold, 3, a, sizeof(a));
            xplat_fmt_cell(has_tgt, v_tgt, 3, b, sizeof(b));
            bk_printf("[xplat] SUM  %-12s %-12s | %7s %7s %7s %4s\r\n",
                      "crit_hold", "disable_int", a, b, "-",
                      (has_hold != 0) ? "1" : "-");
        }
    }

    /* Context switch: semaphore ping-pong round-trip */
    {
        float v = 0.0f;
        uint32_t n = 0U;
        int has_sw;
        char a[16];
        char b[16];

        has_sw = xplat_rec_find_ops("ctx_switch", "sem_pingpong", &v, &n);
        bk_printf("[xplat] SUM  -- Context switch (us/round-trip) --\r\n");
        if (has_sw == 0) {
            bk_printf("[xplat] SUM  (none)\r\n");
        } else {
            bk_printf("[xplat] SUM  case         detail        |  us_per     n\r\n");
            xplat_fmt_cell(1, v, 3, a, sizeof(a));
            if (n > 0U) {
                (void)snprintf(b, sizeof(b), "%u", (unsigned int)n);
            } else {
                (void)snprintf(b, sizeof(b), "-");
            }
            bk_printf("[xplat] SUM  %-12s %-12s | %7s %5s\r\n",
                      "ctx_switch", "sem_pingpong", a, b);
        }
    }

    /* Alloc (exclude contend / IRQ / ctx_switch already printed above) */
    bk_printf("[xplat] SUM  -- Alloc (us per op) --\r\n");
    {
        int ops_n = 0;

        for (i = 0U; i < s_xplat_rec_cnt; i++) {
            if (s_xplat_recs[i].kind != XPLAT_REC_OPS) {
                continue;
            }
            if ((os_strcmp(s_xplat_recs[i].case_name, "drop_cpu_pct") == 0) ||
                (os_strcmp(s_xplat_recs[i].case_name, "drop_bw_pct") == 0) ||
                (os_strcmp(s_xplat_recs[i].case_name, "irq_latency") == 0) ||
                (os_strcmp(s_xplat_recs[i].case_name, "crit_hold") == 0) ||
                (os_strcmp(s_xplat_recs[i].case_name, "ctx_switch") == 0)) {
                continue;
            }
            ops_n++;
            bk_printf("[xplat] SUM  %-12s %-12s %8.3f\r\n",
                      s_xplat_recs[i].case_name,
                      s_xplat_recs[i].mem,
                      s_xplat_recs[i].metric);
        }
        if (ops_n == 0) {
            bk_printf("[xplat] SUM  (none)\r\n");
        }
    }

    /* Skips */
    bk_printf("[xplat] SUM  -- SKIP --\r\n");
    {
        int any = 0;
        for (i = 0U; i < s_xplat_rec_cnt; i++) {
            if (s_xplat_recs[i].kind == XPLAT_REC_SKIP) {
                any = 1;
                bk_printf("[xplat] SUM  %s %s size=%u\r\n",
                          s_xplat_recs[i].case_name,
                          s_xplat_recs[i].mem,
                          (unsigned int)s_xplat_recs[i].size);
            }
        }
        if (any == 0) {
            bk_printf("[xplat] SUM  (none)\r\n");
        }
    }

    bk_printf("[xplat] SUM  note: prefer fill_u32/seq_read over memset for write/read BW\r\n");
    bk_printf("[xplat] SUM  note: seq_copy=x16-unroll; seq_copy_scalar=per-word; vs memcpy\r\n");
    bk_printf("[xplat] SUM  note: rand_read/write=gather/scatter; latency_rand size=chase WS\r\n");
    bk_printf("[xplat] SUM  note: flash_xip=Flash RO seq_read (volatile); seqr_s=heap SRAM baseline\r\n");
    bk_printf("[xplat] SUM  note: s2p=sram_to_psram; xs_s_p=idx@sram,data@psram\r\n");
    bk_printf("[xplat] SUM  note: IRQ crit_hold avg=hold_us min=target_us\r\n");
    bk_printf("[xplat] SUM  note: ctx_switch us_per=sem ping-pong round-trip (~2 switches)\r\n");
    bk_printf("[xplat] SUM  note: contend mul_alone=CPU alone; mul_busy=CPU while peer copies\r\n");
    bk_printf("[xplat] SUM  note: contend drop%%=(alone-busy)/alone*100; peer_MB/s=peer in busy window\r\n");
    bk_printf("[xplat] ========== SUMMARY END ==========\r\n");
}

/**
 * @brief Convert byte throughput to MB/s
 * @param[in] bytes bytes per round
 * @param[in] us_avg average us per round
 * @return MB/s
 */
static float xplat_mbps(uint32_t bytes, float us_avg)
{
    if (us_avg <= 0.0f) {
        return 0.0f;
    }
    return ((float)bytes * 1000000.0f) / (us_avg * 1024.0f * 1024.0f);
}

/**
 * @brief Choose round count for bandwidth cases
 * @param[in] size buffer size
 * @return round count
 */
static uint32_t xplat_bw_rounds(uint32_t size)
{
    if (size >= 262144U) {
        return 32U;
    }
    if (size >= 65536U) {
        return 64U;
    }
    if (size >= 16384U) {
        return 512U;
    }
    if (size >= 4096U) {
        return 2048U;
    }
    if (size >= 1024U) {
        return 8192U;
    }
    return 32768U;
}

/**
 * @brief Log bandwidth result
 * @param[in] case_name case id
 * @param[in] mem memory tag
 * @param[in] size bytes
 * @param[in] rounds rounds
 * @param[in] total_us total us
 * @return none
 */
static void xplat_log_bw(const char *case_name,
                         const char *mem,
                         uint32_t size,
                         uint32_t rounds,
                         uint64_t total_us)
{
    float us_avg = 0.0f;
    float mbps = 0.0f;

    if (rounds > 0U) {
        us_avg = (float)total_us / (float)rounds;
        mbps = xplat_mbps(size, us_avg);
    }
    xplat_rec_add(XPLAT_REC_BW, case_name, mem, size, mbps, total_us);
    bk_printf("[xplat] case=%s mem=%s size=%u freq=%u cache=asis rounds=%u us_total=%llu us_avg=%.2f mbps=%.2f\r\n",
              case_name,
              mem,
              (unsigned int)size,
              (unsigned int)xplat_current_freq_mhz(),
              (unsigned int)rounds,
              (unsigned long long)total_us,
              us_avg,
              mbps);
}

/**
 * @brief Log CPU kernel result
 * @param[in] case_name case id
 * @param[in] loops loop count
 * @param[in] us elapsed us
 * @return none
 */
static void xplat_log_cpu(const char *case_name, uint32_t loops, uint64_t us)
{
    float mops = 0.0f;

    if (us > 0U) {
        mops = ((float)loops) / ((float)us);
    }
    xplat_rec_add(XPLAT_REC_CPU, case_name, "", 0U, mops, us);
    bk_printf("[xplat] case=%s loops=%u freq=%u cache=asis us=%llu mops=%.2f\r\n",
              case_name,
              (unsigned int)loops,
              (unsigned int)xplat_current_freq_mhz(),
              (unsigned long long)us,
              mops);
}

/**
 * @brief Log latency result
 * @param[in] case_name case id
 * @param[in] mem memory tag
 * @param[in] accesses access count
 * @param[in] us elapsed us
 * @return none
 */
/**
 * @brief Log latency result
 * @param[in] case_name case id
 * @param[in] mem memory tag
 * @param[in] size working-set bytes (0 if N/A)
 * @param[in] accesses access count
 * @param[in] us elapsed us
 * @return none
 */
static void xplat_log_latency(const char *case_name,
                              const char *mem,
                              uint32_t size,
                              uint32_t accesses,
                              uint64_t us)
{
    float ns_per = 0.0f;

    if (accesses > 0U) {
        ns_per = ((float)us * 1000.0f) / (float)accesses;
    }
    xplat_rec_add(XPLAT_REC_LAT, case_name, mem, size, ns_per, us);
    if (size > 0U) {
        bk_printf("[xplat] case=%s mem=%s size=%u accesses=%u freq=%u cache=asis us=%llu ns_per=%.2f\r\n",
                  case_name,
                  mem,
                  (unsigned int)size,
                  (unsigned int)accesses,
                  (unsigned int)xplat_current_freq_mhz(),
                  (unsigned long long)us,
                  ns_per);
    } else {
        bk_printf("[xplat] case=%s mem=%s accesses=%u freq=%u cache=asis us=%llu ns_per=%.2f\r\n",
                  case_name,
                  mem,
                  (unsigned int)accesses,
                  (unsigned int)xplat_current_freq_mhz(),
                  (unsigned long long)us,
                  ns_per);
    }
}

/**
 * @brief Log generic timed case
 * @param[in] case_name case id
 * @param[in] detail extra tag
 * @param[in] count operation count
 * @param[in] us elapsed us
 * @return none
 */
static void xplat_log_ops(const char *case_name,
                          const char *detail,
                          uint32_t count,
                          uint64_t us)
{
    float us_per = 0.0f;

    if (count > 0U) {
        us_per = (float)us / (float)count;
    }
    xplat_rec_add(XPLAT_REC_OPS, case_name, detail, 0U, us_per, us);
    bk_printf("[xplat] case=%s detail=%s count=%u freq=%u cache=asis us=%llu us_per=%.3f\r\n",
              case_name,
              detail,
              (unsigned int)count,
              (unsigned int)xplat_current_freq_mhz(),
              (unsigned long long)us,
              us_per);
}

/**
 * @brief Log and record a skipped case
 * @param[in] case_name case id
 * @param[in] mem memory tag
 * @param[in] size size bytes
 * @param[in] reason skip reason
 * @return none
 */
static void xplat_log_skip(const char *case_name,
                           const char *mem,
                           uint32_t size,
                           const char *reason)
{
    xplat_rec_add(XPLAT_REC_SKIP, case_name, mem, size, 0.0f, 0U);
    if (size > 0U) {
        bk_printf("[xplat] SKIP case=%s mem=%s size=%u reason=%s\r\n",
                  case_name,
                  (mem != NULL) ? mem : "-",
                  (unsigned int)size,
                  (reason != NULL) ? reason : "-");
    } else {
        bk_printf("[xplat] SKIP case=%s mem=%s reason=%s\r\n",
                  case_name,
                  (mem != NULL) ? mem : "-",
                  (reason != NULL) ? reason : "-");
    }
}

typedef uint64_t (*xplat_timed_fn_t)(uint32_t rounds, void *ctx);

static void xplat_ensure_bw_time(uint32_t *rounds, uint64_t *total_us,
                                 xplat_timed_fn_t run_fn, void *ctx)
{
    if ((rounds == NULL) || (total_us == NULL) || (run_fn == NULL)) {
        return;
    }
    if ((*total_us < XPLAT_BW_MIN_TOTAL_US) && (*rounds < 65536U)) {
        uint32_t extra = (*rounds) * 4U;
        *total_us = run_fn(extra, ctx);
        *rounds = extra;
    }
}

/* ---------------------------------------------------------------------------
 * Alloc helpers
 * --------------------------------------------------------------------------- */

/**
 * @brief Allocate buffer in SRAM / PSRAM heap / PSRAM-NC (UNCODED slab)
 * @param[in] size bytes
 * @param[in] mem_dom XPLAT_MEM_SRAM / XPLAT_MEM_PSRAM / XPLAT_MEM_PSRAM_NC
 * @return pointer or NULL
 */
static void *xplat_alloc(uint32_t size, int mem_dom)
{
    if (mem_dom == XPLAT_MEM_PSRAM) {
#if CONFIG_PSRAM
        return psram_malloc(size);
#else
        (void)size;
        return NULL;
#endif
    }
    if (mem_dom == XPLAT_MEM_PSRAM_NC) {
#if CONFIG_PSRAM && XPLAT_HAS_FB_UNCODED
        return bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, size);
#else
        (void)size;
        return NULL;
#endif
    }
    return os_malloc(size);
}

/**
 * @brief Free buffer from xplat_alloc
 * @param[in] ptr buffer
 * @param[in] mem_dom domain used at alloc
 * @return none
 */
static void xplat_free(void *ptr, int mem_dom)
{
    if (ptr == NULL) {
        return;
    }
    if (mem_dom == XPLAT_MEM_PSRAM) {
#if CONFIG_PSRAM
        psram_free(ptr);
#endif
        return;
    }
    if (mem_dom == XPLAT_MEM_PSRAM_NC) {
#if CONFIG_PSRAM && XPLAT_HAS_FB_UNCODED
        bk_frame_buffer_free(ptr);
#endif
        return;
    }
    os_free(ptr);
}

/* ---------------------------------------------------------------------------
 * CPU kernels
 * --------------------------------------------------------------------------- */

/**
 * @brief Integer mul/add kernel
 * @param[in] loops iterations
 * @return sink
 */
XPLAT_KERNEL_ATTR
static uint32_t xplat_cpu_mul_kernel(uint32_t loops)
{
    uint32_t a = 1U;
    uint32_t b = 3U;
    uint32_t c = 0U;
    uint32_t i;

    for (i = 0U; i < loops; i++) {
        a = a * b + 1U;
        b = b * 7U + 3U;
        c ^= (a + b);
    }
    s_xplat_sink = c;
    return c;
}

/**
 * @brief Integer add/xor kernel
 * @param[in] loops iterations
 * @return sink
 */
XPLAT_KERNEL_ATTR
static uint32_t xplat_cpu_add_kernel(uint32_t loops)
{
    uint32_t a = 1U;
    uint32_t b = 0U;
    uint32_t i;

    for (i = 0U; i < loops; i++) {
        a += (i ^ 0x9E3779B9U);
        b ^= a;
        a += b;
    }
    s_xplat_sink = a ^ b;
    return a ^ b;
}

/**
 * @brief Branch-heavy kernel
 * @param[in] loops iterations
 * @return sink
 */
XPLAT_KERNEL_ATTR
static uint32_t xplat_cpu_branch_kernel(uint32_t loops)
{
    uint32_t x = 0U;
    uint32_t i;

    for (i = 0U; i < loops; i++) {
        if ((i ^ x) & 1U) {
            x += 3U;
        } else {
            x += 7U;
        }
        if (x & 2U) {
            x ^= i;
        } else {
            x += 1U;
        }
    }
    s_xplat_sink = x;
    return x;
}

/**
 * @brief Integer divide kernel
 * @param[in] loops iterations
 * @return sink
 */
XPLAT_KERNEL_ATTR
static uint32_t xplat_cpu_div_kernel(uint32_t loops)
{
    uint32_t a = 0x12345678U;
    uint32_t b = 3U;
    uint32_t c = 0U;
    uint32_t i;

    for (i = 0U; i < loops; i++) {
        b = 3U + (i & 0x1FU);
        if (b == 0U) {
            b = 1U;
        }
        a = (a / b) + (i * 17U) + 1U;
        c ^= a;
    }
    s_xplat_sink = c;
    return c;
}

/**
 * @brief Shift/rotate style kernel
 * @param[in] loops iterations
 * @return sink
 */
XPLAT_KERNEL_ATTR
static uint32_t xplat_cpu_shift_kernel(uint32_t loops)
{
    uint32_t a = 0xA5A5A5A5U;
    uint32_t i;

    for (i = 0U; i < loops; i++) {
        a = (a << 1) | (a >> 31);
        a ^= (i << (i & 7U));
        a += 0x9E3779B9U;
    }
    s_xplat_sink = a;
    return a;
}

/**
 * @brief Run one CPU case with warmup
 * @param[in] name case name
 * @param[in] fn kernel
 * @param[in] loops loops
 * @return none
 */
static void xplat_run_cpu_case(const char *name,
                               uint32_t (*fn)(uint32_t),
                               uint32_t loops)
{
    uint64_t t0;
    uint64_t us;

    (void)fn(loops / 16U + 1U);
    t0 = bk_aon_rtc_get_us();
    (void)fn(loops);
    us = bk_aon_rtc_get_us() - t0;
    xplat_log_cpu(name, loops, us);
}

/**
 * @brief Run CPU suite
 * @param[in] full 1 include div/shift/add
 * @return none
 */
static void xplat_run_cpu_suite(int full)
{
    bk_printf("[xplat] INFO phase=cpu\r\n");
    xplat_run_cpu_case("cpu_mul", xplat_cpu_mul_kernel, XPLAT_CPU_MUL_LOOPS);
    xplat_run_cpu_case("cpu_branch", xplat_cpu_branch_kernel, XPLAT_CPU_BRANCH_LOOPS);
    if (full != 0) {
        xplat_run_cpu_case("cpu_add", xplat_cpu_add_kernel, XPLAT_CPU_ADD_LOOPS);
        xplat_run_cpu_case("cpu_div", xplat_cpu_div_kernel, XPLAT_CPU_DIV_LOOPS);
        xplat_run_cpu_case("cpu_shift", xplat_cpu_shift_kernel, XPLAT_CPU_SHIFT_LOOPS);
    }
}

/* ---------------------------------------------------------------------------
 * Memory kernels
 * --------------------------------------------------------------------------- */

/**
 * @brief memcpy wrapper
 * @param[out] dst destination
 * @param[in] src source
 * @param[in] n bytes
 * @return none
 */
XPLAT_KERNEL_ATTR
static void xplat_do_memcpy(void *dst, const void *src, size_t n)
{
    memcpy(dst, src, n);
}

/**
 * @brief memset wrapper
 * @param[out] dst destination
 * @param[in] val value
 * @param[in] n bytes
 * @return none
 */
XPLAT_KERNEL_ATTR
static void xplat_do_memset(void *dst, int val, size_t n)
{
    memset(dst, val, n);
}

/**
 * @brief Manual u32 fill (harder for compiler to turn into huge memset)
 * @param[out] dst word buffer
 * @param[in] words word count
 * @param[in] val fill value
 * @return none
 * @note Anti-DCE via memory barrier; val+i keeps pattern distinct from memset.
 */
XPLAT_KERNEL_ATTR
static void xplat_fill_u32(uint32_t *dst, uint32_t words, uint32_t val)
{
    uint32_t i;

    for (i = 0U; i < words; i++) {
        dst[i] = val + i;
    }
    XPLAT_ESCAPE_PTR(dst, words);
}

/**
 * @brief Sequential word read (bandwidth; no ALU accumulate chain)
 * @param[in] src word buffer
 * @param[in] words word count
 * @return none
 * @note Each load escapes via empty asm (keeps load, avoids sum+= overhead).
 */
XPLAT_KERNEL_ATTR
static void xplat_seq_read_u32(const uint32_t *src, uint32_t words)
{
    uint32_t i;

    for (i = 0U; i < words; i++) {
        uint32_t v = src[i];

        XPLAT_ESCAPE_VAL(v);
    }
}

/**
 * @brief Sequential word read via volatile loads (Flash/XIP RO)
 * @param[in] src word buffer
 * @param[in] words word count
 * @return none
 * @note Volatile prevents const-zero RO from being optimized to no loads.
 */
XPLAT_KERNEL_ATTR
static void xplat_seq_read_u32_vol(const uint32_t *src, uint32_t words)
{
    uint32_t i;
    volatile const uint32_t *vsrc = (volatile const uint32_t *)src;

    for (i = 0U; i < words; i++) {
        uint32_t v = vsrc[i];

        XPLAT_ESCAPE_VAL(v);
    }
}

/**
 * @brief Sequential word write
 * @param[out] dst word buffer
 * @param[in] words word count
 * @param[in] val base value
 * @return none
 * @note Anti-DCE via memory barrier; val^i avoids trivial memset pattern.
 */
XPLAT_KERNEL_ATTR
static void xplat_seq_write_u32(uint32_t *dst, uint32_t words, uint32_t val)
{
    uint32_t i;

    for (i = 0U; i < words; i++) {
        dst[i] = val ^ i;
    }
    XPLAT_ESCAPE_PTR(dst, words);
}

/**
 * @brief Manual word copy, 16-word unroll (hand-tuned upper bound vs libc memcpy)
 * @param[out] dst destination words
 * @param[in] src source words
 * @param[in] words word count
 * @return none
 * @note 16-word unroll (same grain as typical newlib memcpy fast path); tail scalar.
 *       Anti-DCE via XPLAT_ESCAPE_BUF (near-zero runtime cost).
 */
XPLAT_KERNEL_ATTR
static void xplat_seq_copy_u32(uint32_t *dst, const uint32_t *src, uint32_t words)
{
    uint32_t i = 0U;

    while ((i + 16U) <= words) {
        dst[i + 0U] = src[i + 0U];
        dst[i + 1U] = src[i + 1U];
        dst[i + 2U] = src[i + 2U];
        dst[i + 3U] = src[i + 3U];
        dst[i + 4U] = src[i + 4U];
        dst[i + 5U] = src[i + 5U];
        dst[i + 6U] = src[i + 6U];
        dst[i + 7U] = src[i + 7U];
        dst[i + 8U] = src[i + 8U];
        dst[i + 9U] = src[i + 9U];
        dst[i + 10U] = src[i + 10U];
        dst[i + 11U] = src[i + 11U];
        dst[i + 12U] = src[i + 12U];
        dst[i + 13U] = src[i + 13U];
        dst[i + 14U] = src[i + 14U];
        dst[i + 15U] = src[i + 15U];
        i += 16U;
    }
    for (; i < words; i++) {
        dst[i] = src[i];
    }
    XPLAT_ESCAPE_BUF(dst, src, words);
}

/**
 * @brief Manual word copy, one word per iteration (naive baseline)
 * @param[out] dst destination words
 * @param[in] src source words
 * @param[in] words word count
 * @return none
 * @note Contrasts with seq_copy (x16 unroll) and libc memcpy.
 */
XPLAT_KERNEL_ATTR
static void xplat_seq_copy_scalar_u32(uint32_t *dst, const uint32_t *src, uint32_t words)
{
    uint32_t i;

    for (i = 0U; i < words; i++) {
        dst[i] = src[i];
    }
    XPLAT_ESCAPE_BUF(dst, src, words);
}

/**
 * @brief Build single-cycle chase table
 * @param[out] idx index array
 * @param[in] nwords words
 * @return none
 */
static void xplat_build_chase(uint32_t *idx, uint32_t nwords)
{
    uint32_t i;
    uint32_t stride;

    if ((idx == NULL) || (nwords < 2U)) {
        return;
    }
    stride = 7919U % nwords;
    if (stride == 0U) {
        stride = 1U;
    }
    for (i = 0U; i < nwords; i++) {
        idx[i] = (i + stride) % nwords;
    }
}

/**
 * @brief Pointer-chase kernel
 * @param[in] idx chase table
 * @param[in] accesses visits
 * @return none
 * @note Dependent loads are the measured work; escape last index only (no sink store).
 */
XPLAT_KERNEL_ATTR
static void xplat_chase_kernel(const uint32_t *idx, uint32_t accesses)
{
    uint32_t cur = 0U;
    uint32_t i;

    for (i = 0U; i < accesses; i++) {
        cur = idx[cur];
    }
    XPLAT_ESCAPE_VAL(cur);
}

typedef struct {
    uint8_t *src;
    uint8_t *dst;
    uint32_t size;
} xplat_memcpy_ctx_t;

typedef void (*xplat_word_copy_fn_t)(uint32_t *dst, const uint32_t *src, uint32_t words);

typedef struct {
    uint8_t *src;
    uint8_t *dst;
    uint32_t size;
    xplat_word_copy_fn_t copy_fn;
} xplat_seq_copy_ctx_t;

typedef struct {
    uint8_t *buf;
    uint32_t size;
} xplat_memset_ctx_t;

typedef struct {
    uint32_t *buf;
    uint32_t words;
    uint32_t val;
} xplat_u32_ctx_t;

/**
 * @brief Timed memcpy rounds
 * @param[in] rounds round count
 * @param[in] ctx memcpy ctx
 * @return elapsed us
 */
static uint64_t xplat_timed_memcpy(uint32_t rounds, void *ctx)
{
    xplat_memcpy_ctx_t *c = (xplat_memcpy_ctx_t *)ctx;
    uint64_t t0;
    uint32_t r;

    t0 = bk_aon_rtc_get_us();
    for (r = 0U; r < rounds; r++) {
        xplat_do_memcpy(c->dst, c->src, c->size);
        XPLAT_ESCAPE_BUF(c->dst, c->src, c->size);
    }
    return bk_aon_rtc_get_us() - t0;
}

/**
 * @brief Timed memset rounds
 * @param[in] rounds round count
 * @param[in] ctx memset ctx
 * @return elapsed us
 */
static uint64_t xplat_timed_memset(uint32_t rounds, void *ctx)
{
    xplat_memset_ctx_t *c = (xplat_memset_ctx_t *)ctx;
    uint64_t t0;
    uint32_t r;

    t0 = bk_aon_rtc_get_us();
    for (r = 0U; r < rounds; r++) {
        xplat_do_memset(c->buf, (int)(r & 0xFFU), c->size);
        XPLAT_ESCAPE_PTR(c->buf, c->size);
    }
    return bk_aon_rtc_get_us() - t0;
}

/**
 * @brief Timed fill_u32 rounds
 * @param[in] rounds round count
 * @param[in] ctx u32 ctx
 * @return elapsed us
 */
static uint64_t xplat_timed_fill_u32(uint32_t rounds, void *ctx)
{
    xplat_u32_ctx_t *c = (xplat_u32_ctx_t *)ctx;
    uint64_t t0;
    uint32_t r;

    t0 = bk_aon_rtc_get_us();
    for (r = 0U; r < rounds; r++) {
        xplat_fill_u32(c->buf, c->words, c->val + r);
    }
    return bk_aon_rtc_get_us() - t0;
}

/**
 * @brief Timed seq read rounds
 * @param[in] rounds round count
 * @param[in] ctx u32 ctx
 * @return elapsed us
 */
static uint64_t xplat_timed_seq_read(uint32_t rounds, void *ctx)
{
    xplat_u32_ctx_t *c = (xplat_u32_ctx_t *)ctx;
    uint64_t t0;
    uint32_t r;

    t0 = bk_aon_rtc_get_us();
    for (r = 0U; r < rounds; r++) {
        xplat_seq_read_u32(c->buf, c->words);
    }
    return bk_aon_rtc_get_us() - t0;
}

/**
 * @brief Timed volatile seq read rounds (Flash/XIP)
 * @param[in] rounds round count
 * @param[in] ctx u32 ctx
 * @return elapsed us
 */
static uint64_t xplat_timed_seq_read_vol(uint32_t rounds, void *ctx)
{
    xplat_u32_ctx_t *c = (xplat_u32_ctx_t *)ctx;
    uint64_t t0;
    uint32_t r;

    t0 = bk_aon_rtc_get_us();
    for (r = 0U; r < rounds; r++) {
        xplat_seq_read_u32_vol(c->buf, c->words);
    }
    return bk_aon_rtc_get_us() - t0;
}

/**
 * @brief Timed seq write rounds
 * @param[in] rounds round count
 * @param[in] ctx u32 ctx
 * @return elapsed us
 */
static uint64_t xplat_timed_seq_write(uint32_t rounds, void *ctx)
{
    xplat_u32_ctx_t *c = (xplat_u32_ctx_t *)ctx;
    uint64_t t0;
    uint32_t r;

    t0 = bk_aon_rtc_get_us();
    for (r = 0U; r < rounds; r++) {
        xplat_seq_write_u32(c->buf, c->words, c->val + r);
    }
    return bk_aon_rtc_get_us() - t0;
}

/**
 * @brief Timed manual word-copy rounds (seq_copy / seq_copy_scalar)
 * @param[in] rounds round count
 * @param[in] ctx seq_copy ctx (includes copy_fn)
 * @return elapsed us
 */
static uint64_t xplat_timed_seq_copy(uint32_t rounds, void *ctx)
{
    xplat_seq_copy_ctx_t *c = (xplat_seq_copy_ctx_t *)ctx;
    uint64_t t0;
    uint32_t r;
    uint32_t words = c->size / 4U;

    t0 = bk_aon_rtc_get_us();
    for (r = 0U; r < rounds; r++) {
        c->copy_fn((uint32_t *)(void *)c->dst,
                   (const uint32_t *)(void *)c->src,
                   words);
    }
    return bk_aon_rtc_get_us() - t0;
}

/**
 * @brief Run memcpy bandwidth one size
 * @param[in] mem tag
 * @param[in] size bytes
 * @param[in] use_psram domain
 * @return 0 ok, -1 fail
 */
static int xplat_run_memcpy_one(const char *mem, uint32_t size, int use_psram)
{
    xplat_memcpy_ctx_t ctx;
    uint32_t rounds;
    uint64_t total_us;
    uint8_t *raw = NULL;
    int single_buf = 0;

    if ((use_psram == 0) && (size > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("memcpy", mem, size, "sram_cap");
        return -1;
    }

    ctx.src = (uint8_t *)xplat_alloc(size, use_psram);
    ctx.dst = (uint8_t *)xplat_alloc(size, use_psram);
    ctx.size = size;
    if ((ctx.src == NULL) || (ctx.dst == NULL)) {
        xplat_free(ctx.src, use_psram);
        xplat_free(ctx.dst, use_psram);
        ctx.src = NULL;
        ctx.dst = NULL;
        /* Fallback: one contiguous block, halves as src/dst */
        raw = (uint8_t *)xplat_alloc(size * 2U, use_psram);
        if (raw != NULL) {
            ctx.src = raw;
            ctx.dst = raw + size;
            single_buf = 1;
        } else if (size >= 8U) {
            /* Last resort: half size within one buffer */
            raw = (uint8_t *)xplat_alloc(size, use_psram);
            if (raw != NULL) {
                ctx.size = size / 2U;
                ctx.src = raw;
                ctx.dst = raw + ctx.size;
                single_buf = 1;
                bk_printf("[xplat] INFO memcpy mem=%s size=%u fell back to half=%u single_buf\r\n",
                          mem, (unsigned int)size, (unsigned int)ctx.size);
            }
        }
    }
    if ((ctx.src == NULL) || (ctx.dst == NULL)) {
        xplat_free(raw, use_psram);
        xplat_log_skip("memcpy", mem, size, "alloc");
        return -1;
    }

    xplat_do_memset(ctx.src, 0xA5, ctx.size);
    xplat_do_memset(ctx.dst, 0x5A, ctx.size);
    xplat_do_memcpy(ctx.dst, ctx.src, ctx.size);

    rounds = xplat_bw_rounds(ctx.size);
    total_us = xplat_timed_memcpy(rounds, &ctx);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_memcpy, &ctx);
    xplat_log_bw("memcpy", mem, ctx.size, rounds, total_us);

    if (single_buf != 0) {
        xplat_free(raw, use_psram);
    } else {
        xplat_free(ctx.dst, use_psram);
        xplat_free(ctx.src, use_psram);
    }
    return 0;
}

/**
 * @brief Run one manual word-copy bandwidth variant
 * @param[in] case_name case tag (seq_copy / seq_copy_scalar)
 * @param[in] mem tag
 * @param[in] size bytes (must be multiple of 4)
 * @param[in] use_psram domain
 * @param[in] copy_fn kernel
 * @return 0 ok, -1 fail
 */
static int xplat_run_seq_copy_variant(const char *case_name,
                                      const char *mem,
                                      uint32_t size,
                                      int use_psram,
                                      xplat_word_copy_fn_t copy_fn)
{
    xplat_seq_copy_ctx_t ctx;
    uint32_t rounds;
    uint64_t total_us;
    uint8_t *raw = NULL;
    int single_buf = 0;

    if ((case_name == NULL) || (copy_fn == NULL)) {
        return -1;
    }
    if ((size < 4U) || ((size & 3U) != 0U)) {
        xplat_log_skip(case_name, mem, size, "unsupported");
        return -1;
    }
    if ((use_psram == 0) && (size > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip(case_name, mem, size, "sram_cap");
        return -1;
    }

    ctx.src = (uint8_t *)xplat_alloc(size, use_psram);
    ctx.dst = (uint8_t *)xplat_alloc(size, use_psram);
    ctx.size = size;
    ctx.copy_fn = copy_fn;
    if ((ctx.src == NULL) || (ctx.dst == NULL)) {
        xplat_free(ctx.src, use_psram);
        xplat_free(ctx.dst, use_psram);
        ctx.src = NULL;
        ctx.dst = NULL;
        raw = (uint8_t *)xplat_alloc(size * 2U, use_psram);
        if (raw != NULL) {
            ctx.src = raw;
            ctx.dst = raw + size;
            single_buf = 1;
        } else if (size >= 8U) {
            raw = (uint8_t *)xplat_alloc(size, use_psram);
            if (raw != NULL) {
                ctx.size = size / 2U;
                if ((ctx.size & 3U) != 0U) {
                    ctx.size &= ~3U;
                }
                ctx.src = raw;
                ctx.dst = raw + ctx.size;
                single_buf = 1;
                bk_printf("[xplat] INFO %s mem=%s size=%u fell back to half=%u single_buf\r\n",
                          case_name, mem, (unsigned int)size, (unsigned int)ctx.size);
            }
        }
    }
    if ((ctx.src == NULL) || (ctx.dst == NULL) || (ctx.size < 4U)) {
        xplat_free(raw, use_psram);
        xplat_log_skip(case_name, mem, size, "alloc");
        return -1;
    }

    xplat_do_memset(ctx.src, 0xA5, ctx.size);
    xplat_do_memset(ctx.dst, 0x5A, ctx.size);
    copy_fn((uint32_t *)(void *)ctx.dst,
            (const uint32_t *)(void *)ctx.src,
            ctx.size / 4U);

    rounds = xplat_bw_rounds(ctx.size);
    total_us = xplat_timed_seq_copy(rounds, &ctx);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_seq_copy, &ctx);
    xplat_log_bw(case_name, mem, ctx.size, rounds, total_us);

    if (single_buf != 0) {
        xplat_free(raw, use_psram);
    } else {
        xplat_free(ctx.dst, use_psram);
        xplat_free(ctx.src, use_psram);
    }
    return 0;
}

/**
 * @brief Run seq_copy (x16 unroll) bandwidth one size
 * @param[in] mem tag
 * @param[in] size bytes
 * @param[in] use_psram domain
 * @return 0 ok, -1 fail
 */
static int xplat_run_seq_copy_one(const char *mem, uint32_t size, int use_psram)
{
    return xplat_run_seq_copy_variant("seq_copy", mem, size, use_psram, xplat_seq_copy_u32);
}

/**
 * @brief Run seq_copy_scalar (naive per-word) bandwidth one size
 * @param[in] mem tag
 * @param[in] size bytes
 * @param[in] use_psram domain
 * @return 0 ok, -1 fail
 */
static int xplat_run_seq_copy_scalar_one(const char *mem, uint32_t size, int use_psram)
{
    return xplat_run_seq_copy_variant("seq_copy_scalar", mem, size, use_psram,
                                      xplat_seq_copy_scalar_u32);
}

/**
 * @brief Run memset bandwidth one size
 * @param[in] mem tag
 * @param[in] size bytes
 * @param[in] use_psram domain
 * @return 0 ok, -1 fail
 */
static int xplat_run_memset_one(const char *mem, uint32_t size, int use_psram)
{
    xplat_memset_ctx_t ctx;
    uint32_t rounds;
    uint64_t total_us;

    if ((use_psram == 0) && (size > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("memset", mem, size, "sram_cap");
        return -1;
    }

    ctx.buf = (uint8_t *)xplat_alloc(size, use_psram);
    ctx.size = size;
    if (ctx.buf == NULL) {
        xplat_log_skip("memset", mem, size, "alloc");
        return -1;
    }

    xplat_do_memset(ctx.buf, 0x00, size);
    rounds = xplat_bw_rounds(size);
    total_us = xplat_timed_memset(rounds, &ctx);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_memset, &ctx);
    xplat_log_bw("memset", mem, size, rounds, total_us);

    xplat_free(ctx.buf, use_psram);
    return 0;
}

/**
 * @brief Run manual fill_u32 bandwidth
 * @param[in] mem tag
 * @param[in] size bytes (word aligned)
 * @param[in] use_psram domain
 * @return 0 ok, -1 fail
 */
static int xplat_run_fill_u32_one(const char *mem, uint32_t size, int use_psram)
{
    xplat_u32_ctx_t ctx;
    uint32_t rounds;
    uint64_t total_us;

    if ((size < 4U) || ((size & 3U) != 0U)) {
        return -1;
    }
    if ((use_psram == 0) && (size > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("fill_u32", mem, size, "sram_cap");
        return -1;
    }
    ctx.words = size / 4U;
    ctx.val = 0x11223344U;
    ctx.buf = (uint32_t *)xplat_alloc(size, use_psram);
    if (ctx.buf == NULL) {
        xplat_log_skip("fill_u32", mem, size, "alloc");
        return -1;
    }

    rounds = xplat_bw_rounds(size);
    total_us = xplat_timed_fill_u32(rounds, &ctx);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_fill_u32, &ctx);
    xplat_log_bw("fill_u32", mem, size, rounds, total_us);

    xplat_free(ctx.buf, use_psram);
    return 0;
}

/**
 * @brief Run sequential read bandwidth
 * @param[in] mem tag
 * @param[in] size bytes
 * @param[in] use_psram domain
 * @return 0 ok, -1 fail
 */
static int xplat_run_seq_read_one(const char *mem, uint32_t size, int use_psram)
{
    xplat_u32_ctx_t ctx;
    uint32_t rounds;
    uint64_t total_us;

    if ((size < 4U) || ((size & 3U) != 0U)) {
        return -1;
    }
    if ((use_psram == 0) && (size > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("seq_read", mem, size, "sram_cap");
        return -1;
    }
    ctx.words = size / 4U;
    ctx.val = 0U;
    ctx.buf = (uint32_t *)xplat_alloc(size, use_psram);
    if (ctx.buf == NULL) {
        xplat_log_skip("seq_read", mem, size, "alloc");
        return -1;
    }
    xplat_fill_u32(ctx.buf, ctx.words, 0x55AA55AAU);

    rounds = xplat_bw_rounds(size);
    total_us = xplat_timed_seq_read(rounds, &ctx);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_seq_read, &ctx);
    xplat_log_bw("seq_read", mem, size, rounds, total_us);

    xplat_free(ctx.buf, use_psram);
    return 0;
}

/**
 * @brief Run sequential write bandwidth
 * @param[in] mem tag
 * @param[in] size bytes
 * @param[in] use_psram domain
 * @return 0 ok, -1 fail
 */
static int xplat_run_seq_write_one(const char *mem, uint32_t size, int use_psram)
{
    xplat_u32_ctx_t ctx;
    uint32_t rounds;
    uint64_t total_us;

    if ((size < 4U) || ((size & 3U) != 0U)) {
        return -1;
    }
    if ((use_psram == 0) && (size > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("seq_write", mem, size, "sram_cap");
        return -1;
    }
    ctx.words = size / 4U;
    ctx.val = 0xA5A5A5A5U;
    ctx.buf = (uint32_t *)xplat_alloc(size, use_psram);
    if (ctx.buf == NULL) {
        xplat_log_skip("seq_write", mem, size, "alloc");
        return -1;
    }

    rounds = xplat_bw_rounds(size);
    total_us = xplat_timed_seq_write(rounds, &ctx);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_seq_write, &ctx);
    xplat_log_bw("seq_write", mem, size, rounds, total_us);

    xplat_free(ctx.buf, use_psram);
    return 0;
}

/**
 * @brief Build shuffled index table (Fisher-Yates + LCG) for gather/scatter BW
 * @param[out] idx index array (length nwords)
 * @param[in] nwords word count
 * @return none
 */
static void xplat_build_rand_idx(uint32_t *idx, uint32_t nwords)
{
    uint32_t i;
    uint32_t seed = XPLAT_RAND_IDX_SEED;

    if ((idx == NULL) || (nwords < 1U)) {
        return;
    }
    for (i = 0U; i < nwords; i++) {
        idx[i] = i;
    }
    for (i = nwords - 1U; i > 0U; i--) {
        uint32_t j;
        uint32_t tmp;

        seed = (seed * 1664525U) + 1013904223U;
        j = seed % (i + 1U);
        tmp = idx[i];
        idx[i] = idx[j];
        idx[j] = tmp;
    }
}

/**
 * @brief Gather read bandwidth kernel (data[idx[i]])
 * @param[in] data data words
 * @param[in] idx shuffled indices
 * @param[in] words word count
 * @return none
 */
XPLAT_KERNEL_ATTR
static void xplat_rand_read_u32(const uint32_t *data, const uint32_t *idx, uint32_t words)
{
    uint32_t i;

    for (i = 0U; i < words; i++) {
        uint32_t v = data[idx[i]];

        XPLAT_ESCAPE_VAL(v);
    }
}

/**
 * @brief Scatter write bandwidth kernel (data[idx[i]] = ...)
 * @param[out] data data words
 * @param[in] idx shuffled indices
 * @param[in] words word count
 * @param[in] val base value
 * @return none
 */
XPLAT_KERNEL_ATTR
static void xplat_rand_write_u32(uint32_t *data, const uint32_t *idx, uint32_t words, uint32_t val)
{
    uint32_t i;

    for (i = 0U; i < words; i++) {
        data[idx[i]] = val ^ i;
    }
    XPLAT_ESCAPE_PTR(data, words);
}

typedef struct {
    uint32_t *data;
    uint32_t *idx;
    uint32_t words;
    uint32_t val;
} xplat_rand_bw_ctx_t;

/**
 * @brief Timed rand_read rounds
 * @param[in] rounds round count
 * @param[in] ctx ctx
 * @return elapsed us
 */
static uint64_t xplat_timed_rand_read(uint32_t rounds, void *ctx)
{
    xplat_rand_bw_ctx_t *c = (xplat_rand_bw_ctx_t *)ctx;
    uint64_t t0;
    uint32_t r;

    t0 = bk_aon_rtc_get_us();
    for (r = 0U; r < rounds; r++) {
        xplat_rand_read_u32(c->data, c->idx, c->words);
    }
    return bk_aon_rtc_get_us() - t0;
}

/**
 * @brief Timed rand_write rounds
 * @param[in] rounds round count
 * @param[in] ctx ctx
 * @return elapsed us
 */
static uint64_t xplat_timed_rand_write(uint32_t rounds, void *ctx)
{
    xplat_rand_bw_ctx_t *c = (xplat_rand_bw_ctx_t *)ctx;
    uint64_t t0;
    uint32_t r;

    t0 = bk_aon_rtc_get_us();
    for (r = 0U; r < rounds; r++) {
        xplat_rand_write_u32(c->data, c->idx, c->words, c->val + r);
    }
    return bk_aon_rtc_get_us() - t0;
}

/**
 * @brief Run random gather-read bandwidth one size
 * @param[in] mem tag
 * @param[in] size data bytes (word-aligned); idx uses same byte size
 * @param[in] use_psram domain
 * @return 0 ok, -1 fail
 */
static int xplat_run_rand_read_one(const char *mem, uint32_t size, int use_psram)
{
    xplat_rand_bw_ctx_t ctx;
    uint32_t rounds;
    uint64_t total_us;
    uint32_t bytes;

    if ((size < 4U) || ((size & 3U) != 0U)) {
        xplat_log_skip("rand_read", mem, size, "unsupported");
        return -1;
    }
    /* Each buffer (data / idx) capped at SRAM_MAX; two allocs may still fail → alloc SKIP */
    if ((use_psram == 0) && (size > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("rand_read", mem, size, "sram_cap");
        return -1;
    }

    ctx.words = size / 4U;
    ctx.val = 0U;
    ctx.data = (uint32_t *)xplat_alloc(size, use_psram);
    ctx.idx = (uint32_t *)xplat_alloc(size, use_psram);
    if ((ctx.data == NULL) || (ctx.idx == NULL)) {
        xplat_free(ctx.data, use_psram);
        xplat_free(ctx.idx, use_psram);
        xplat_log_skip("rand_read", mem, size, "alloc");
        return -1;
    }

    xplat_fill_u32(ctx.data, ctx.words, 0x11111111U);
    xplat_build_rand_idx(ctx.idx, ctx.words);
    xplat_rand_read_u32(ctx.data, ctx.idx, ctx.words);

    bytes = size;
    rounds = xplat_bw_rounds(bytes);
    total_us = xplat_timed_rand_read(rounds, &ctx);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_rand_read, &ctx);
    xplat_log_bw("rand_read", mem, bytes, rounds, total_us);

    xplat_free(ctx.idx, use_psram);
    xplat_free(ctx.data, use_psram);
    return 0;
}

/**
 * @brief Run random scatter-write bandwidth one size
 * @param[in] mem tag
 * @param[in] size data bytes (word-aligned)
 * @param[in] use_psram domain
 * @return 0 ok, -1 fail
 */
static int xplat_run_rand_write_one(const char *mem, uint32_t size, int use_psram)
{
    xplat_rand_bw_ctx_t ctx;
    uint32_t rounds;
    uint64_t total_us;
    uint32_t bytes;

    if ((size < 4U) || ((size & 3U) != 0U)) {
        xplat_log_skip("rand_write", mem, size, "unsupported");
        return -1;
    }
    if ((use_psram == 0) && (size > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("rand_write", mem, size, "sram_cap");
        return -1;
    }

    ctx.words = size / 4U;
    ctx.val = 0x55AA55AAU;
    ctx.data = (uint32_t *)xplat_alloc(size, use_psram);
    ctx.idx = (uint32_t *)xplat_alloc(size, use_psram);
    if ((ctx.data == NULL) || (ctx.idx == NULL)) {
        xplat_free(ctx.data, use_psram);
        xplat_free(ctx.idx, use_psram);
        xplat_log_skip("rand_write", mem, size, "alloc");
        return -1;
    }

    xplat_fill_u32(ctx.data, ctx.words, 0U);
    xplat_build_rand_idx(ctx.idx, ctx.words);
    xplat_rand_write_u32(ctx.data, ctx.idx, ctx.words, ctx.val);

    bytes = size;
    rounds = xplat_bw_rounds(bytes);
    total_us = xplat_timed_rand_write(rounds, &ctx);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_rand_write, &ctx);
    xplat_log_bw("rand_write", mem, bytes, rounds, total_us);

    xplat_free(ctx.idx, use_psram);
    xplat_free(ctx.data, use_psram);
    return 0;
}

/**
 * @brief Run pointer-chase latency for one working-set size
 * @param[in] mem tag
 * @param[in] use_psram domain
 * @param[in] ws_bytes chase table bytes (word-aligned)
 * @return 0 ok, -1 fail
 */
static int xplat_run_latency_rand_ws(const char *mem, int use_psram, uint32_t ws_bytes)
{
    uint32_t *idx;
    uint64_t t0;
    uint64_t us;
    uint32_t nwords;

    if ((ws_bytes < 8U) || ((ws_bytes & 3U) != 0U)) {
        xplat_log_skip("latency_rand", mem, ws_bytes, "unsupported");
        return -1;
    }
    if ((use_psram == 0) && (ws_bytes > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("latency_rand", mem, ws_bytes, "sram_cap");
        return -1;
    }

    nwords = ws_bytes / 4U;
    idx = (uint32_t *)xplat_alloc(ws_bytes, use_psram);
    if (idx == NULL) {
        xplat_log_skip("latency_rand", mem, ws_bytes, "alloc");
        return -1;
    }

    xplat_build_chase(idx, nwords);
    xplat_chase_kernel(idx, nwords);
    t0 = bk_aon_rtc_get_us();
    xplat_chase_kernel(idx, XPLAT_LATENCY_ACCESSES);
    us = bk_aon_rtc_get_us() - t0;
    xplat_log_latency("latency_rand", mem, ws_bytes, XPLAT_LATENCY_ACCESSES, us);

    xplat_free(idx, use_psram);
    return 0;
}

/**
 * @brief Run random chase latency (multi working-set when full sizes provided)
 * @param[in] mem tag
 * @param[in] use_psram domain
 * @param[in] ws_sizes working-set byte table (NULL => default 16K)
 * @param[in] ws_cnt count
 * @return none
 */
static void xplat_run_latency_rand_matrix(const char *mem,
                                          int use_psram,
                                          const uint32_t *ws_sizes,
                                          uint32_t ws_cnt)
{
    uint32_t i;
    uint32_t one = (uint32_t)(XPLAT_LATENCY_WORDS * sizeof(uint32_t));

    if ((ws_sizes == NULL) || (ws_cnt == 0U)) {
        (void)xplat_run_latency_rand_ws(mem, use_psram, one);
        return;
    }
    for (i = 0U; i < ws_cnt; i++) {
        (void)xplat_run_latency_rand_ws(mem, use_psram, ws_sizes[i]);
    }
}

/**
 * @brief Run sequential word latency for one working-set size
 * @param[in] mem tag
 * @param[in] use_psram domain
 * @param[in] ws_bytes buffer bytes (word-aligned)
 * @return 0 ok, -1 fail
 * @note Per-access escape like seq_read (no acc+= chain).
 */
static int xplat_run_latency_seq_ws(const char *mem, int use_psram, uint32_t ws_bytes)
{
    uint32_t *buf;
    uint64_t t0;
    uint64_t us;
    uint32_t i;
    uint32_t nwords;

    if ((ws_bytes < 4U) || ((ws_bytes & 3U) != 0U)) {
        xplat_log_skip("latency_seq", mem, ws_bytes, "unsupported");
        return -1;
    }
    if ((use_psram == 0) && (ws_bytes > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("latency_seq", mem, ws_bytes, "sram_cap");
        return -1;
    }

    nwords = ws_bytes / 4U;
    buf = (uint32_t *)xplat_alloc(ws_bytes, use_psram);
    if (buf == NULL) {
        xplat_log_skip("latency_seq", mem, ws_bytes, "alloc");
        return -1;
    }
    xplat_fill_u32(buf, nwords, 1U);

    t0 = bk_aon_rtc_get_us();
    for (i = 0U; i < XPLAT_LATENCY_ACCESSES; i++) {
        uint32_t v = buf[i % nwords];

        XPLAT_ESCAPE_VAL(v);
    }
    us = bk_aon_rtc_get_us() - t0;
    xplat_log_latency("latency_seq", mem, ws_bytes, XPLAT_LATENCY_ACCESSES, us);

    xplat_free(buf, use_psram);
    return 0;
}

/**
 * @brief Sequential read latency matrix (multi working-set)
 * @param[in] mem tag
 * @param[in] use_psram domain
 * @param[in] ws_sizes working-set byte table (NULL => default 16K)
 * @param[in] ws_cnt count
 * @return none
 */
static void xplat_run_latency_seq_matrix(const char *mem,
                                         int use_psram,
                                         const uint32_t *ws_sizes,
                                         uint32_t ws_cnt)
{
    uint32_t i;
    uint32_t one = (uint32_t)(XPLAT_LATENCY_WORDS * sizeof(uint32_t));

    if ((ws_sizes == NULL) || (ws_cnt == 0U)) {
        (void)xplat_run_latency_seq_ws(mem, use_psram, one);
        return;
    }
    for (i = 0U; i < ws_cnt; i++) {
        (void)xplat_run_latency_seq_ws(mem, use_psram, ws_sizes[i]);
    }
}

/**
 * @brief Cross-domain SRAM↔PSRAM memcpy
 * @param[in] size bytes
 * @return 0 ok, -1 fail
 */
static int xplat_run_cross_memcpy(uint32_t size)
{
#if !CONFIG_PSRAM
    xplat_log_skip("memcpy", "cross", size, "no_psram");
    return -1;
#else
    xplat_memcpy_ctx_t ctx_sp;
    xplat_memcpy_ctx_t ctx_ps;
    uint32_t rounds;
    uint64_t total_us;
    uint32_t xsize = size;

    if (size > XPLAT_SRAM_MAX_SIZE) {
        xsize = XPLAT_SRAM_MAX_SIZE;
        bk_printf("[xplat] INFO cross size=%u capped to %u (sram_cap)\r\n",
                  (unsigned int)size, (unsigned int)xsize);
    }

    ctx_sp.src = (uint8_t *)os_malloc(xsize);
    ctx_sp.dst = (uint8_t *)psram_malloc(xsize);
    ctx_sp.size = xsize;
    if ((ctx_sp.src == NULL) || (ctx_sp.dst == NULL)) {
        xplat_free(ctx_sp.src, 0);
        xplat_free(ctx_sp.dst, 1);
        xplat_log_skip("memcpy", "cross", size, "alloc");
        return -1;
    }
    ctx_ps.src = ctx_sp.dst;
    ctx_ps.dst = ctx_sp.src;
    ctx_ps.size = xsize;

    xplat_do_memset(ctx_sp.src, 0x11, xsize);
    xplat_do_memset(ctx_sp.dst, 0x22, xsize);

    rounds = xplat_bw_rounds(xsize);
    total_us = xplat_timed_memcpy(rounds, &ctx_sp);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_memcpy, &ctx_sp);
    xplat_log_bw("memcpy", "sram_to_psram", xsize, rounds, total_us);

    rounds = xplat_bw_rounds(xsize);
    total_us = xplat_timed_memcpy(rounds, &ctx_ps);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_memcpy, &ctx_ps);
    xplat_log_bw("memcpy", "psram_to_sram", xsize, rounds, total_us);

    psram_free(ctx_sp.dst);
    os_free(ctx_sp.src);
    return 0;
#endif
}

/**
 * @brief Misaligned memcpy bandwidth (offset into larger buffer)
 * @param[in] mem tag
 * @param[in] use_psram domain
 * @param[in] payload payload bytes
 * @param[in] off source/dest offset
 * @return 0 ok, -1 fail
 */
static int xplat_run_memcpy_align(const char *mem, int use_psram, uint32_t payload, uint32_t off)
{
    xplat_memcpy_ctx_t ctx;
    uint32_t alloc_size;
    uint32_t rounds;
    uint64_t total_us;
    uint8_t *raw_src;
    uint8_t *raw_dst;
    char detail[32];

    alloc_size = payload + off + 32U;
    raw_src = (uint8_t *)xplat_alloc(alloc_size, use_psram);
    raw_dst = (uint8_t *)xplat_alloc(alloc_size, use_psram);
    if ((raw_src == NULL) || (raw_dst == NULL)) {
        xplat_free(raw_src, use_psram);
        xplat_free(raw_dst, use_psram);
        (void)snprintf(detail, sizeof(detail), "%s_off%u", mem, (unsigned int)off);
        xplat_log_skip("memcpy_align", detail, payload, "alloc");
        return -1;
    }

    ctx.src = raw_src + off;
    ctx.dst = raw_dst + off;
    ctx.size = payload;
    xplat_do_memset(raw_src, 0xA5, alloc_size);
    xplat_do_memset(raw_dst, 0x5A, alloc_size);

    rounds = xplat_bw_rounds(payload);
    total_us = xplat_timed_memcpy(rounds, &ctx);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_memcpy, &ctx);

    (void)snprintf(detail, sizeof(detail), "%s_off%u", mem, (unsigned int)off);
    xplat_log_bw("memcpy_align", detail, payload, rounds, total_us);

    xplat_free(raw_dst, use_psram);
    xplat_free(raw_src, use_psram);
    return 0;
}

/**
 * @brief Run memory suite for one domain
 * @param[in] mem tag
 * @param[in] use_psram domain
 * @param[in] sizes size table
 * @param[in] size_cnt count
 * @param[in] full 1 include seq/fill/latency_seq
 * @return none
 */
static void xplat_run_mem_domain(const char *mem,
                                 int use_psram,
                                 const uint32_t *sizes,
                                 uint32_t size_cnt,
                                 int full)
{
    uint32_t i;

#if !CONFIG_PSRAM
    if (use_psram) {
        xplat_log_skip("mem", mem, 0U, "no_psram");
        return;
    }
#endif

    bk_printf("[xplat] INFO phase=mem mem=%s\r\n", mem);
    for (i = 0U; i < size_cnt; i++) {
        (void)xplat_run_memcpy_one(mem, sizes[i], use_psram);
        (void)xplat_run_seq_copy_one(mem, sizes[i], use_psram);
        (void)xplat_run_seq_copy_scalar_one(mem, sizes[i], use_psram);
        (void)xplat_run_memset_one(mem, sizes[i], use_psram);
        if (full != 0) {
            (void)xplat_run_fill_u32_one(mem, sizes[i], use_psram);
            (void)xplat_run_seq_read_one(mem, sizes[i], use_psram);
            (void)xplat_run_seq_write_one(mem, sizes[i], use_psram);
        }
    }
    if (full != 0) {
        uint32_t rcnt = (uint32_t)(sizeof(s_xplat_rand_bw_sizes) / sizeof(s_xplat_rand_bw_sizes[0]));

        for (i = 0U; i < rcnt; i++) {
            (void)xplat_run_rand_read_one(mem, s_xplat_rand_bw_sizes[i], use_psram);
            (void)xplat_run_rand_write_one(mem, s_xplat_rand_bw_sizes[i], use_psram);
        }
        xplat_run_latency_rand_matrix(mem,
                                      use_psram,
                                      s_xplat_lat_ws_sizes,
                                      (uint32_t)(sizeof(s_xplat_lat_ws_sizes) / sizeof(s_xplat_lat_ws_sizes[0])));
        xplat_run_latency_seq_matrix(mem,
                                     use_psram,
                                     s_xplat_lat_ws_sizes,
                                     (uint32_t)(sizeof(s_xplat_lat_ws_sizes) / sizeof(s_xplat_lat_ws_sizes[0])));
    } else {
        xplat_run_latency_rand_matrix(mem, use_psram, NULL, 0U);
    }
}

/**
 * @brief Run allocator timing
 * @param[in] mem tag
 * @param[in] use_psram domain
 * @param[in] size block size
 * @return none
 */
static void xplat_run_alloc_case(const char *mem, int use_psram, uint32_t size)
{
    uint64_t t0;
    uint64_t us;
    uint32_t i;
    void *p;

#if !CONFIG_PSRAM
    if (use_psram) {
        xplat_log_skip("alloc_free", mem, size, "no_psram");
        return;
    }
#endif

    /* warmup */
    p = xplat_alloc(size, use_psram);
    if (p == NULL) {
        xplat_log_skip("alloc_free", mem, size, "alloc");
        return;
    }
    xplat_free(p, use_psram);

    t0 = bk_aon_rtc_get_us();
    for (i = 0U; i < XPLAT_ALLOC_LOOPS; i++) {
        p = xplat_alloc(size, use_psram);
        if (p == NULL) {
            us = bk_aon_rtc_get_us() - t0;
            bk_printf("[xplat] FAIL case=alloc mem=%s size=%u at=%u us=%llu\r\n",
                      mem, (unsigned int)size, (unsigned int)i, (unsigned long long)us);
            return;
        }
        xplat_free(p, use_psram);
    }
    us = bk_aon_rtc_get_us() - t0;
    xplat_log_ops("alloc_free", mem, XPLAT_ALLOC_LOOPS, us);
    (void)size;
}

/**
 * @brief Context-switch ping worker
 * @param[in] arg unused
 * @return none
 */
static void xplat_switch_peer(void *arg)
{
    uint32_t i;

    (void)arg;
    for (i = 0U; i < XPLAT_SWITCH_LOOPS; i++) {
        (void)rtos_get_semaphore(&s_xplat_sem_a, BEKEN_WAIT_FOREVER);
        (void)rtos_set_semaphore(&s_xplat_sem_b);
    }
    s_xplat_switch_done = 1U;
    rtos_delete_thread(NULL);
}

/**
 * @brief Measure semaphore round-trip context switch cost
 * @return none
 */
static void xplat_run_switch_case(void)
{
    uint64_t t0;
    uint64_t us;
    uint32_t i;
    bk_err_t ret;

    s_xplat_switch_done = 0U;
    if (rtos_init_semaphore(&s_xplat_sem_a, 1) != kNoErr) {
        xplat_log_skip("ctx_switch", "sem_pingpong", 0U, "sem");
        return;
    }
    if (rtos_init_semaphore(&s_xplat_sem_b, 1) != kNoErr) {
        (void)rtos_deinit_semaphore(&s_xplat_sem_a);
        xplat_log_skip("ctx_switch", "sem_pingpong", 0U, "sem");
        return;
    }

    ret = rtos_create_thread(NULL,
                             XPLAT_TASK_PRIO,
                             "xplat_sw",
                             xplat_switch_peer,
                             2048,
                             NULL);
    if (ret != kNoErr) {
        (void)rtos_deinit_semaphore(&s_xplat_sem_a);
        (void)rtos_deinit_semaphore(&s_xplat_sem_b);
        xplat_log_skip("ctx_switch", "sem_pingpong", 0U, "task");
        return;
    }

    t0 = bk_aon_rtc_get_us();
    for (i = 0U; i < XPLAT_SWITCH_LOOPS; i++) {
        (void)rtos_set_semaphore(&s_xplat_sem_a);
        (void)rtos_get_semaphore(&s_xplat_sem_b, BEKEN_WAIT_FOREVER);
    }
    us = bk_aon_rtc_get_us() - t0;

    while (s_xplat_switch_done == 0U) {
        rtos_delay_milliseconds(1);
    }

    (void)rtos_deinit_semaphore(&s_xplat_sem_a);
    (void)rtos_deinit_semaphore(&s_xplat_sem_b);
    /* count = round-trips; each trip is two context switches roughly.
     * size field stores loop count for SUMMARY. */
    {
        float us_per = (float)us / (float)XPLAT_SWITCH_LOOPS;

        xplat_rec_add(XPLAT_REC_OPS, "ctx_switch", "sem_pingpong",
                      XPLAT_SWITCH_LOOPS, us_per, us);
        bk_printf("[xplat] case=ctx_switch detail=sem_pingpong count=%u freq=%u "
                  "cache=asis us=%llu us_per=%.3f\r\n",
                  (unsigned int)XPLAT_SWITCH_LOOPS,
                  (unsigned int)xplat_current_freq_mhz(),
                  (unsigned long long)us,
                  us_per);
    }
}

/* ---------------------------------------------------------------------------
 * Extra suites: dma / contend / latx / float (standalone subcmds)
 * --------------------------------------------------------------------------- */
typedef bk_err_t (*xplat_dma_cpy_fn_t)(void *out, const void *in, uint32_t len);

/**
 * @brief HPDMA copy on pre-allocated channel
 * @param[in] out dest
 * @param[in] in src
 * @param[in] len bytes
 * @return BK_OK or error
 */
#if XPLAT_HAS_HPDMA
static bk_err_t xplat_hpdma_chn_cpy(void *out, const void *in, uint32_t len)
{
    return bk_hpdma_memcpy_link(out, in, len, s_xplat_hp_chn);
}
#endif

/**
 * @brief GDMA copy on pre-allocated channel
 * @param[in] out dest
 * @param[in] in src
 * @param[in] len bytes
 * @return BK_OK or error
 */
static bk_err_t xplat_gdma_chn_cpy(void *out, const void *in, uint32_t len)
{
    return dma_memcpy_by_chnl(out, in, len, s_xplat_gd_chn);
}

/**
 * @brief Timed DMA memcpy rounds using one engine callback
 * @param[in] rounds iteration count
 * @param[in] c memcpy context
 * @param[in] eng DMA engine function
 * @return total us, or 0 on DMA failure
 */
static uint64_t xplat_timed_dma_memcpy(uint32_t rounds,
                                       xplat_memcpy_ctx_t *c,
                                       xplat_dma_cpy_fn_t eng)
{
    uint32_t r;
    uint64_t t0;
    bk_err_t ret;

    t0 = bk_aon_rtc_get_us();
    for (r = 0U; r < rounds; r++) {
        ret = eng(c->dst, c->src, (uint32_t)c->size);
        if (ret != BK_OK) {
            return 0U;
        }
    }
    return bk_aon_rtc_get_us() - t0;
}

/**
 * @brief Run one DMA memcpy bandwidth case
 * @param[in] eng_name "hpdma" or "gdma"
 * @param[in] mem domain tag
 * @param[in] size bytes
 * @param[in] src source buffer
 * @param[in] dst dest buffer
 * @param[in] eng copy function
 * @return 0 ok, -1 fail
 */
static int xplat_run_dma_one(const char *eng_name,
                             const char *mem,
                             uint32_t size,
                             uint8_t *src,
                             uint8_t *dst,
                             xplat_dma_cpy_fn_t eng)
{
    xplat_memcpy_ctx_t ctx;
    uint32_t rounds;
    uint64_t total_us;
    char case_name[XPLAT_NAME_LEN];
    bk_err_t ret;

    ctx.src = src;
    ctx.dst = dst;
    ctx.size = size;

    xplat_do_memset(src, 0xA5, size);
    xplat_do_memset(dst, 0x5A, size);
    ret = eng(dst, src, size);
    if (ret != BK_OK) {
        (void)snprintf(case_name, sizeof(case_name), "%s_memcpy", eng_name);
        xplat_log_skip(case_name, mem, size, "unsupported");
        return -1;
    }

    rounds = xplat_bw_rounds(size);
    total_us = xplat_timed_dma_memcpy(rounds, &ctx, eng);
    if (total_us == 0U) {
        (void)snprintf(case_name, sizeof(case_name), "%s_memcpy", eng_name);
        xplat_log_skip(case_name, mem, size, "unsupported");
        return -1;
    }
    while ((total_us < (uint64_t)XPLAT_BW_MIN_TOTAL_US) && (rounds < 100000U)) {
        rounds *= 2U;
        total_us = xplat_timed_dma_memcpy(rounds, &ctx, eng);
        if (total_us == 0U) {
            (void)snprintf(case_name, sizeof(case_name), "%s_memcpy", eng_name);
            xplat_log_skip(case_name, mem, size, "unsupported");
            return -1;
        }
    }

    (void)snprintf(case_name, sizeof(case_name), "%s_memcpy", eng_name);
    xplat_log_bw(case_name, mem, size, rounds, total_us);
    return 0;
}

/**
 * @brief Allocate src/dst for DMA path (same or cross domain)
 * @param[in] src_psram 1 PSRAM src
 * @param[in] dst_psram 1 PSRAM dst
 * @param[in] size bytes
 * @param[out] src out src
 * @param[out] dst out dst
 * @return 0 ok, -1 fail
 */
static int xplat_dma_alloc_pair(int src_psram,
                                int dst_psram,
                                uint32_t size,
                                uint8_t **src,
                                uint8_t **dst)
{
    *src = (uint8_t *)xplat_alloc(size, src_psram);
    *dst = (uint8_t *)xplat_alloc(size, dst_psram);
    if ((*src == NULL) || (*dst == NULL)) {
        xplat_free(*src, src_psram);
        xplat_free(*dst, dst_psram);
        *src = NULL;
        *dst = NULL;
        return -1;
    }
    return 0;
}

/**
 * @brief Free DMA buffer pair
 * @param[in] src src
 * @param[in] dst dst
 * @param[in] src_psram domain
 * @param[in] dst_psram domain
 * @return none
 */
static void xplat_dma_free_pair(uint8_t *src,
                                uint8_t *dst,
                                int src_psram,
                                int dst_psram)
{
    xplat_free(dst, dst_psram);
    xplat_free(src, src_psram);
}

/**
 * @brief Run DMA bandwidth matrix (HPDMA if present + GDMA, persistent channels)
 * @param[in] standalone 1 reset+summary; 0 append into current run
 * @return none
 */
static void xplat_run_dma_matrix(int standalone)
{
    uint32_t i;
    uint8_t *src;
    uint8_t *dst;
#if XPLAT_HAS_HPDMA
    hpdma_id_t hp_chn = HPDMA_ID_MAX;
#endif
    dma_id_t gd_chn = DMA_ID_MAX;
    const struct {
        const char *mem;
        int src_psram;
        int dst_psram;
    } paths[] = {
        { "sram", 0, 0 },
#if CONFIG_PSRAM
        { "psram", 1, 1 },
        { "sram_to_psram", 0, 1 },
        { "psram_to_sram", 1, 0 },
#endif
    };
    uint32_t p;
    uint32_t path_cnt = (uint32_t)(sizeof(paths) / sizeof(paths[0]));
    uint32_t cnt = (uint32_t)(sizeof(s_xplat_dma_sizes) / sizeof(s_xplat_dma_sizes[0]));

    if (standalone != 0) {
        xplat_rec_reset();
    }
    bk_printf("[xplat] INFO dma matrix begin cfg_mhz=%u hpdma=%d (persistent chnl)\r\n",
              (unsigned int)xplat_cfg_mhz(), XPLAT_HAS_HPDMA);
    xplat_log_freq();
#if !XPLAT_HAS_HPDMA
    bk_printf("[xplat] INFO dma note: if gen-dma completion waits with ms-level sleep, "
              "reported MB/s is wait overhead not real BW\r\n");
#endif

#if XPLAT_HAS_HPDMA
    hp_chn = bk_hpdma_alloc(HPDMA_DEV_DTCM);
    if (hp_chn >= HPDMA_ID_MAX) {
        xplat_log_skip("hpdma_memcpy", "all", 0U, "alloc");
        hp_chn = HPDMA_ID_MAX;
    }
#else
    xplat_log_skip("hpdma_memcpy", "all", 0U, "no_hpdma");
#endif
    gd_chn = bk_dma_alloc(DMA_DEV_DTCM);
    if (gd_chn >= DMA_ID_MAX) {
        xplat_log_skip("gdma_memcpy", "all", 0U, "alloc");
        gd_chn = DMA_ID_MAX;
    }

    for (p = 0U; p < path_cnt; p++) {
        for (i = 0U; i < cnt; i++) {
            uint32_t size = s_xplat_dma_sizes[i];

            /* Dual-buffer: each side needs `size`; SRAM heap soft-cap per buffer */
            if (((paths[p].src_psram == 0) || (paths[p].dst_psram == 0)) &&
                (size > XPLAT_SRAM_MAX_SIZE)) {
#if XPLAT_HAS_HPDMA
                xplat_log_skip("hpdma_memcpy", paths[p].mem, size, "sram_cap");
#endif
                xplat_log_skip("gdma_memcpy", paths[p].mem, size, "sram_cap");
                xplat_log_skip("cpu_memcpy", paths[p].mem, size, "sram_cap");
                continue;
            }

            if (xplat_dma_alloc_pair(paths[p].src_psram, paths[p].dst_psram,
                                     size, &src, &dst) != 0) {
#if XPLAT_HAS_HPDMA
                xplat_log_skip("hpdma_memcpy", paths[p].mem, size, "alloc");
#endif
                xplat_log_skip("gdma_memcpy", paths[p].mem, size, "alloc");
                xplat_log_skip("cpu_memcpy", paths[p].mem, size, "alloc");
                continue;
            }

#if XPLAT_HAS_HPDMA
            if (hp_chn < HPDMA_ID_MAX) {
                s_xplat_hp_chn = hp_chn;
                (void)xplat_run_dma_one("hpdma", paths[p].mem, size, src, dst,
                                        xplat_hpdma_chn_cpy);
            }
#endif
            if (gd_chn < DMA_ID_MAX) {
                s_xplat_gd_chn = gd_chn;
                (void)xplat_run_dma_one("gdma", paths[p].mem, size, src, dst,
                                        xplat_gdma_chn_cpy);
            }

            /* CPU memcpy reference for same buffers */
            {
                xplat_memcpy_ctx_t ctx;
                uint32_t rounds;
                uint64_t total_us;

                ctx.src = src;
                ctx.dst = dst;
                ctx.size = size;
                rounds = xplat_bw_rounds(size);
                total_us = xplat_timed_memcpy(rounds, &ctx);
                xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_memcpy, &ctx);
                xplat_log_bw("cpu_memcpy", paths[p].mem, size, rounds, total_us);
            }

            xplat_dma_free_pair(src, dst, paths[p].src_psram, paths[p].dst_psram);
        }
    }

#if XPLAT_HAS_HPDMA
    if (hp_chn < HPDMA_ID_MAX) {
        (void)bk_hpdma_free(HPDMA_DEV_DTCM, hp_chn);
    }
#endif
    if (gd_chn < DMA_ID_MAX) {
        (void)bk_dma_free(DMA_DEV_DTCM, gd_chn);
    }

#if XPLAT_HAS_HPDMA
    bk_printf("[xplat] INFO dma note: gen-dma path may re-init and flush cache each xfer "
              "(expect lower MB/s than hp-dma)\r\n");
#else
    bk_printf("[xplat] INFO dma note: no hp-dma; gen-dma + cpu_memcpy only\r\n");
#endif
    bk_printf("[xplat] INFO dma matrix done\r\n");
    if (standalone != 0) {
        xplat_print_summary("dma");
    }
}

/**
 * @brief Contending memcpy worker (pinned to another core when SMP)
 * @param[in] arg unused
 * @return none
 */
static void xplat_contend_peer(void *arg)
{
    (void)arg;
    s_xplat_contend_peer_core = rtos_get_core_id();
    while (s_xplat_contend_go == 0U) {
        rtos_delay_milliseconds(1);
    }
    while (s_xplat_contend_stop == 0U) {
        xplat_do_memcpy(s_xplat_contend_dst, s_xplat_contend_src, s_xplat_contend_size);
        s_xplat_contend_bytes += (uint64_t)s_xplat_contend_size;
    }
    s_xplat_contend_done = 1U;
    rtos_delete_thread(NULL);
}

/**
 * @brief Flash XIP seq_read for one working-set size (prefix of RO blob)
 * @param[in] size bytes to read (word-aligned, <= XPLAT_XIP_BYTES)
 * @return 0 ok, -1 fail
 * @note Uses volatile loads so const RO cannot be constant-folded away.
 */
static int xplat_run_xip_seq_read_one(uint32_t size)
{
    xplat_u32_ctx_t ctx;
    uint32_t rounds;
    uint64_t total_us;

    if ((size < 4U) || ((size & 3U) != 0U) || (size > XPLAT_XIP_BYTES)) {
        xplat_log_skip("seq_read", "flash_xip", size, "unsupported");
        return -1;
    }

    ctx.buf = (uint32_t *)(void *)s_xplat_xip_ro;
    ctx.words = size / 4U;
    ctx.val = 0U;
    rounds = xplat_bw_rounds(size);
    total_us = xplat_timed_seq_read_vol(rounds, &ctx);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_seq_read_vol, &ctx);
    xplat_log_bw("seq_read", "flash_xip", size, rounds, total_us);
    return 0;
}

/**
 * @brief Flash XIP seq_read size matrix (compare vs seqr_s in SUMMARY)
 * @return none
 * @note Baseline is mem-phase seq_read sram (seqr_s).
 */
static void xplat_run_xip_matrix(void)
{
    uint32_t i;
    uint32_t cnt = (uint32_t)(sizeof(s_xplat_mem_sizes_full) / sizeof(s_xplat_mem_sizes_full[0]));

    bk_printf("[xplat] INFO xip matrix begin blob=%u ptr=%p sizes=%u\r\n",
              (unsigned int)XPLAT_XIP_BYTES,
              (void *)s_xplat_xip_ro,
              (unsigned int)cnt);
    for (i = 0U; i < cnt; i++) {
        (void)xplat_run_xip_seq_read_one(s_xplat_mem_sizes_full[i]);
    }
    bk_printf("[xplat] INFO xip matrix done\r\n");
}

/**
 * @brief Timer ISR for IRQ latency probe
 * @param[in] id timer id
 * @return none
 */
static void xplat_irq_timer_isr(timer_id_t id)
{
    (void)id;
    s_xplat_irq_c_isr = XPLAT_DWT_CYCCNT;
    s_xplat_irq_t_isr = bk_aon_rtc_get_us();
    s_xplat_irq_hits++;
}

/**
 * @brief IRQ latency + critical-section hold probe
 * @return none
 */
static void xplat_run_irq_matrix(void)
{
    bk_err_t ret;
    uint32_t i;
    uint32_t ok = 0U;
    uint64_t sum_lat = 0U;
    uint64_t min_lat = UINT64_MAX;
    uint64_t max_lat = 0U;
    uint64_t t_arm;
    uint32_t c_arm;
    uint32_t int_level;
    uint64_t t0;
    uint64_t t1;
    uint32_t hold_us;

    bk_printf("[xplat] INFO irq matrix begin\r\n");
    (void)bk_timer_driver_init();
    xplat_dwt_enable();

    for (i = 0U; i < XPLAT_IRQ_SAMPLES; i++) {
        uint64_t wait_deadline;
        uint64_t lat;

        s_xplat_irq_hits = 0U;
        s_xplat_irq_t_isr = 0U;
        (void)bk_timer_stop(XPLAT_IRQ_TIMER_ID);
        (void)bk_timer_cancel(XPLAT_IRQ_TIMER_ID);
        /* Prefer delay-with-callback timer API when oneshot start_us is unavailable */
        ret = bk_timer_delay_with_callback(XPLAT_IRQ_TIMER_ID,
                                           (uint64_t)XPLAT_IRQ_TIMER_US,
                                           xplat_irq_timer_isr);
        if (ret != BK_OK) {
            xplat_log_skip("irq_latency", "timer", 0U, "start");
            bk_printf("[xplat] INFO irq matrix abort ret=%d\r\n", (int)ret);
            return;
        }
        t_arm = bk_aon_rtc_get_us();
        c_arm = XPLAT_DWT_CYCCNT;
        wait_deadline = t_arm + (uint64_t)XPLAT_IRQ_WAIT_US;
        while ((s_xplat_irq_hits == 0U) && (bk_aon_rtc_get_us() < wait_deadline)) {
        }
        (void)bk_timer_stop(XPLAT_IRQ_TIMER_ID);
        (void)bk_timer_cancel(XPLAT_IRQ_TIMER_ID);
        if (s_xplat_irq_hits == 0U) {
            continue;
        }
        /* Observed fire time from arm; subtract programmed delay for entry overhead */
        if (s_xplat_irq_t_isr <= t_arm) {
            continue;
        }
        lat = s_xplat_irq_t_isr - t_arm;
        if (lat >= (uint64_t)XPLAT_IRQ_TIMER_US) {
            lat -= (uint64_t)XPLAT_IRQ_TIMER_US;
        }
        (void)c_arm;
        sum_lat += lat;
        if (lat < min_lat) {
            min_lat = lat;
        }
        if (lat > max_lat) {
            max_lat = lat;
        }
        ok++;
    }

    if (ok > 0U) {
        xplat_log_ops("irq_latency", "timer_entry", ok, sum_lat);
        xplat_rec_add(XPLAT_REC_OPS, "irq_latency", "us_min", ok, (float)min_lat, min_lat);
        xplat_rec_add(XPLAT_REC_OPS, "irq_latency", "us_max", ok, (float)max_lat, max_lat);
        bk_printf("[xplat] INFO irq_latency us_min=%llu us_max=%llu samples=%u\r\n",
                  (unsigned long long)min_lat,
                  (unsigned long long)max_lat,
                  (unsigned int)ok);
    } else {
        xplat_log_skip("irq_latency", "timer", 0U, "no_hit");
    }

    /* Critical section: hold IRQs for calibrated wall time */
    xplat_dwt_enable();
    int_level = rtos_disable_int();
    t0 = bk_aon_rtc_get_us();
    do {
        t1 = bk_aon_rtc_get_us();
    } while ((uint32_t)(t1 - t0) < XPLAT_CRIT_HOLD_US);
    t1 = bk_aon_rtc_get_us();
    rtos_enable_int(int_level);
    hold_us = (uint32_t)(t1 - t0);
    xplat_log_ops("crit_hold", "disable_int", 1U, (uint64_t)hold_us);
    xplat_rec_add(XPLAT_REC_OPS, "crit_hold", "target_us", 0U,
                  (float)XPLAT_CRIT_HOLD_US, (uint64_t)XPLAT_CRIT_HOLD_US);
    bk_printf("[xplat] INFO crit_hold target_us=%u\r\n",
              (unsigned int)XPLAT_CRIT_HOLD_US);
    bk_printf("[xplat] INFO irq matrix done\r\n");
}

/**
 * @brief One contend scenario: peer memcpy on domain while main runs cpu_mul
 * @param[in] use_psram 1 for PSRAM buffers
 * @param[in] tag log tag
 * @return none
 */
static void xplat_run_contend_domain(int use_psram, const char *tag)
{
    uint64_t t0;
    uint64_t us_base;
    uint64_t us_c;
    uint64_t bytes;
    float mops_base;
    float mops_c;
    float mbps_base;
    float mbps_c;
    float drop_cpu;
    float drop_bw;
    bk_err_t ret;
    xplat_memcpy_ctx_t ctx;
    uint32_t rounds;
    uint64_t total_us;
    uint32_t main_core;
    int same_core;
    char base_tag[24];
    char drop_tag[24];

    main_core = rtos_get_core_id();
#if defined(CONFIG_FREERTOS_SMP) && (CONFIG_FREERTOS_SMP == 1) && (CONFIG_CPU_CNT > 1)
    same_core = 0;
#else
    same_core = 1;
#endif
    bk_printf("[xplat] INFO contend domain=%s main_core=%u same_core=%d\r\n",
              tag, (unsigned int)main_core, same_core);

    t0 = bk_aon_rtc_get_us();
    (void)xplat_cpu_mul_kernel(XPLAT_CPU_MUL_LOOPS);
    us_base = bk_aon_rtc_get_us() - t0;
    mops_base = (us_base > 0U) ?
                ((float)XPLAT_CPU_MUL_LOOPS / (float)us_base) : 0.0f;
    xplat_rec_add(XPLAT_REC_CPU, "cpu_mul_base", tag, 0U, mops_base, us_base);
    bk_printf("[xplat] case=cpu_mul_base mem=%s loops=%u freq=%u cache=asis us=%llu mops=%.2f\r\n",
              tag,
              (unsigned int)XPLAT_CPU_MUL_LOOPS,
              (unsigned int)xplat_current_freq_mhz(),
              (unsigned long long)us_base,
              mops_base);

    s_xplat_contend_psram = use_psram;
    s_xplat_contend_size = XPLAT_CONTEND_SIZE;
    s_xplat_contend_src = (uint8_t *)xplat_alloc(s_xplat_contend_size, use_psram);
    s_xplat_contend_dst = (uint8_t *)xplat_alloc(s_xplat_contend_size, use_psram);
    if ((s_xplat_contend_src == NULL) || (s_xplat_contend_dst == NULL)) {
        xplat_free(s_xplat_contend_src, use_psram);
        xplat_free(s_xplat_contend_dst, use_psram);
        s_xplat_contend_size = 16U * 1024U;
        s_xplat_contend_src = (uint8_t *)xplat_alloc(s_xplat_contend_size, use_psram);
        s_xplat_contend_dst = (uint8_t *)xplat_alloc(s_xplat_contend_size, use_psram);
    }
    if ((s_xplat_contend_src == NULL) || (s_xplat_contend_dst == NULL)) {
        xplat_free(s_xplat_contend_src, use_psram);
        xplat_free(s_xplat_contend_dst, use_psram);
        xplat_log_skip("contend", tag, s_xplat_contend_size, "alloc");
        return;
    }
    bk_printf("[xplat] INFO contend buf mem=%s size=%u\r\n",
              tag, (unsigned int)s_xplat_contend_size);
    xplat_do_memset(s_xplat_contend_src, 0xA5, s_xplat_contend_size);
    xplat_do_memset(s_xplat_contend_dst, 0x5A, s_xplat_contend_size);

    ctx.src = s_xplat_contend_src;
    ctx.dst = s_xplat_contend_dst;
    ctx.size = s_xplat_contend_size;
    rounds = xplat_bw_rounds(s_xplat_contend_size);
    total_us = xplat_timed_memcpy(rounds, &ctx);
    xplat_ensure_bw_time(&rounds, &total_us, xplat_timed_memcpy, &ctx);
    (void)snprintf(base_tag, sizeof(base_tag), "%s_base", tag);
    xplat_log_bw("memcpy", base_tag, s_xplat_contend_size, rounds, total_us);
    mbps_base = xplat_mbps(s_xplat_contend_size, (float)total_us / (float)rounds);

    s_xplat_contend_go = 0U;
    s_xplat_contend_stop = 0U;
    s_xplat_contend_done = 0U;
    s_xplat_contend_bytes = 0U;
    s_xplat_contend_peer_core = 0xFFFFFFFFU;

#if defined(CONFIG_FREERTOS_SMP) && (CONFIG_FREERTOS_SMP == 1) && (CONFIG_CPU_CNT > 1)
    ret = rtos_core1_create_thread(NULL,
                                   XPLAT_TASK_PRIO,
                                   "xplat_ct",
                                   xplat_contend_peer,
                                   XPLAT_CONTEND_PEER_STACK,
                                   NULL);
#else
    ret = rtos_create_thread(NULL,
                             XPLAT_TASK_PRIO,
                             "xplat_ct",
                             xplat_contend_peer,
                             XPLAT_CONTEND_PEER_STACK,
                             NULL);
#endif
    if (ret != kNoErr) {
        xplat_log_skip("contend", tag, 0U, "task");
        xplat_free(s_xplat_contend_dst, use_psram);
        xplat_free(s_xplat_contend_src, use_psram);
        return;
    }

    s_xplat_contend_go = 1U;
    rtos_delay_milliseconds(5);
    s_xplat_contend_bytes = 0U;
    t0 = bk_aon_rtc_get_us();
    (void)xplat_cpu_mul_kernel(XPLAT_CPU_MUL_LOOPS);
    us_c = bk_aon_rtc_get_us() - t0;
    bytes = s_xplat_contend_bytes;
    s_xplat_contend_stop = 1U;
    {
        uint32_t wait_ms = 0U;

        while ((s_xplat_contend_done == 0U) && (wait_ms < 5000U)) {
            rtos_delay_milliseconds(1);
            wait_ms++;
        }
        if (s_xplat_contend_done == 0U) {
            bk_printf("[xplat] WARN contend peer join timeout\r\n");
        }
    }

    mops_c = (us_c > 0U) ? ((float)XPLAT_CPU_MUL_LOOPS / (float)us_c) : 0.0f;
    mbps_c = (us_c > 0U) ?
             (((float)bytes / (float)us_c) * (1000000.0f / (1024.0f * 1024.0f))) : 0.0f;
    xplat_rec_add(XPLAT_REC_CPU, "cpu_mul_ct", tag, 0U, mops_c, us_c);
    bk_printf("[xplat] case=cpu_mul_ct mem=%s loops=%u freq=%u cache=asis us=%llu mops=%.2f\r\n",
              tag,
              (unsigned int)XPLAT_CPU_MUL_LOOPS,
              (unsigned int)xplat_current_freq_mhz(),
              (unsigned long long)us_c,
              mops_c);
    xplat_rec_add(XPLAT_REC_BW, "memcpy_ct", tag, s_xplat_contend_size, mbps_c, us_c);
    bk_printf("[xplat] case=memcpy_ct mem=%s size=%u freq=%u cache=asis rounds=1 "
              "us_total=%llu us_avg=%.2f mbps=%.2f\r\n",
              tag,
              (unsigned int)s_xplat_contend_size,
              (unsigned int)xplat_current_freq_mhz(),
              (unsigned long long)us_c,
              (float)us_c,
              mbps_c);
    if ((s_xplat_contend_peer_core != 0xFFFFFFFFU) &&
        (s_xplat_contend_peer_core == main_core)) {
        same_core = 1;
    }
    bk_printf("[xplat] INFO contend peer_core=%u same_core=%d\r\n",
              (unsigned int)s_xplat_contend_peer_core, same_core);

    drop_cpu = (mops_base > 0.0f) ? ((mops_base - mops_c) * 100.0f / mops_base) : 0.0f;
    drop_bw = (mbps_base > 0.0f) ? ((mbps_base - mbps_c) * 100.0f / mbps_base) : 0.0f;
    (void)snprintf(drop_tag, sizeof(drop_tag), "mul_vs_%s", tag);
    xplat_rec_add(XPLAT_REC_OPS, "drop_cpu_pct", drop_tag, 0U, drop_cpu, us_c);
    xplat_rec_add(XPLAT_REC_OPS, "drop_bw_pct", drop_tag, 0U, drop_bw, us_c);
    bk_printf("[xplat] INFO contend[%s] drop_cpu=%.1f%% drop_bw=%.1f%% "
              "(mops %.2f->%.2f, mbps %.2f->%.2f)\r\n",
              tag, drop_cpu, drop_bw, mops_base, mops_c, mbps_base, mbps_c);

    xplat_free(s_xplat_contend_dst, use_psram);
    xplat_free(s_xplat_contend_src, use_psram);
    s_xplat_contend_src = NULL;
    s_xplat_contend_dst = NULL;
}

#if XPLAT_HAS_HPDMA
/**
 * @brief CPU mul while HPDMA repeatedly copies (same bank stress)
 * @return none
 */
static void xplat_run_contend_hpdma(void)
{
    uint8_t *src;
    uint8_t *dst;
    uint32_t size = XPLAT_CONTEND_SIZE;
    hpdma_id_t chn;
    uint64_t t0;
    uint64_t us;
    uint64_t bytes = 0U;
    float mops;
    float mbps;
    bk_err_t ret;

    src = (uint8_t *)xplat_alloc(size, 0);
    dst = (uint8_t *)xplat_alloc(size, 0);
    if ((src == NULL) || (dst == NULL)) {
        xplat_free(src, 0);
        xplat_free(dst, 0);
        xplat_log_skip("hpdma_ct", "sram", size, "alloc");
        return;
    }
    xplat_do_memset(src, 0xA5, size);
    xplat_do_memset(dst, 0x5A, size);
    chn = bk_hpdma_alloc(HPDMA_DEV_DTCM);
    if (chn >= HPDMA_ID_MAX) {
        xplat_log_skip("hpdma_ct", "chn", 0U, "alloc");
        xplat_free(dst, 0);
        xplat_free(src, 0);
        return;
    }

    bk_printf("[xplat] INFO contend_hpdma size=%u chn=%u\r\n",
              (unsigned int)size, (unsigned int)chn);
    t0 = bk_aon_rtc_get_us();
    (void)xplat_cpu_mul_kernel(XPLAT_CPU_MUL_LOOPS / 4U);
    {
        uint64_t deadline = bk_aon_rtc_get_us() + 5000U;
        while (bk_aon_rtc_get_us() < deadline) {
            ret = bk_hpdma_memcpy_link(dst, src, size, chn);
            if (ret == BK_OK) {
                bytes += (uint64_t)size;
            } else {
                break;
            }
        }
    }
    (void)xplat_cpu_mul_kernel(XPLAT_CPU_MUL_LOOPS / 4U);
    us = bk_aon_rtc_get_us() - t0;
    mops = (us > 0U) ? ((float)(XPLAT_CPU_MUL_LOOPS / 2U) / (float)us) : 0.0f;
    mbps = (us > 0U) ?
           (((float)bytes / (float)us) * (1000000.0f / (1024.0f * 1024.0f))) : 0.0f;
    xplat_rec_add(XPLAT_REC_BW, "hpdma_ct", "sram", size, mbps, us);
    xplat_rec_add(XPLAT_REC_CPU, "cpu_mul_hpdma_ct", "sram", 0U, mops, us);
    bk_printf("[xplat] case=hpdma_ct mem=sram size=%u freq=%u cache=asis rounds=1 "
              "us_total=%llu us_avg=%.2f mbps=%.2f\r\n",
              (unsigned int)size,
              (unsigned int)xplat_current_freq_mhz(),
              (unsigned long long)us,
              (float)us,
              mbps);
    bk_printf("[xplat] case=cpu_mul_hpdma_ct loops=%u freq=%u cache=asis us=%llu mops=%.2f\r\n",
              (unsigned int)(XPLAT_CPU_MUL_LOOPS / 2U),
              (unsigned int)xplat_current_freq_mhz(),
              (unsigned long long)us,
              mops);

    (void)bk_hpdma_free(HPDMA_DEV_DTCM, chn);
    xplat_free(dst, 0);
    xplat_free(src, 0);
}
#endif

/**
 * @brief CPU vs memcpy contention matrix (SRAM + PSRAM + optional HPDMA)
 * @param[in] standalone 1 reset+summary; 0 append into current run
 * @return none
 */
static void xplat_run_contend_matrix(int standalone)
{
    if (standalone != 0) {
        xplat_rec_reset();
        xplat_log_freq();
    }
    bk_printf("[xplat] INFO contend matrix begin\r\n");
    xplat_run_contend_domain(0, "sram");
#if CONFIG_PSRAM
    xplat_run_contend_domain(1, "psram");
#endif
#if XPLAT_HAS_HPDMA
    xplat_run_contend_hpdma();
#endif
    bk_printf("[xplat] INFO contend matrix done\r\n");
    if (standalone != 0) {
        xplat_print_summary("contend");
    }
}

/**
 * @brief Store-to-load forwarding latency kernel
 * @param[in] buf word buffer
 * @param[in] nwords buffer words
 * @param[in] accesses access count
 * @return none
 * @note Dependent store/load chain is the measured work; escape final val only.
 */
static void xplat_store_kernel(uint32_t *buf, uint32_t nwords, uint32_t accesses)
{
    uint32_t i;
    uint32_t idx = 0U;
    uint32_t val = 1U;

    for (i = 0U; i < accesses; i++) {
        buf[idx] = val;
        val = buf[idx] + 1U;
        idx = (idx + ((val & 0xFFU) | 1U)) % nwords;
    }
    XPLAT_ESCAPE_VAL(val);
}

/**
 * @brief Run store latency for one working-set size
 * @param[in] mem tag
 * @param[in] use_psram domain
 * @param[in] ws_bytes buffer bytes (word-aligned)
 * @return 0 ok, -1 fail
 */
static int xplat_run_latency_store_ws(const char *mem, int use_psram, uint32_t ws_bytes)
{
    uint32_t *buf;
    uint64_t t0;
    uint64_t us;
    uint32_t nwords;

    if ((ws_bytes < 8U) || ((ws_bytes & 3U) != 0U)) {
        xplat_log_skip("latency_store", mem, ws_bytes, "unsupported");
        return -1;
    }
    if ((use_psram == 0) && (ws_bytes > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("latency_store", mem, ws_bytes, "sram_cap");
        return -1;
    }

    nwords = ws_bytes / 4U;
    buf = (uint32_t *)xplat_alloc(ws_bytes, use_psram);
    if (buf == NULL) {
        xplat_log_skip("latency_store", mem, ws_bytes, "alloc");
        return -1;
    }
    xplat_fill_u32(buf, nwords, 0U);
    xplat_store_kernel(buf, nwords, nwords);
    t0 = bk_aon_rtc_get_us();
    xplat_store_kernel(buf, nwords, XPLAT_LATENCY_ACCESSES);
    us = bk_aon_rtc_get_us() - t0;
    xplat_log_latency("latency_store", mem, ws_bytes, XPLAT_LATENCY_ACCESSES, us);
    xplat_free(buf, use_psram);
    return 0;
}

/**
 * @brief Store-load latency matrix (multi working-set)
 * @param[in] mem tag
 * @param[in] use_psram domain
 * @param[in] ws_sizes working-set byte table (NULL => default 16K)
 * @param[in] ws_cnt count
 * @return none
 */
static void xplat_run_latency_store_matrix(const char *mem,
                                           int use_psram,
                                           const uint32_t *ws_sizes,
                                           uint32_t ws_cnt)
{
    uint32_t i;
    uint32_t one = (uint32_t)(XPLAT_LATENCY_WORDS * sizeof(uint32_t));

    if ((ws_sizes == NULL) || (ws_cnt == 0U)) {
        (void)xplat_run_latency_store_ws(mem, use_psram, one);
        return;
    }
    for (i = 0U; i < ws_cnt; i++) {
        (void)xplat_run_latency_store_ws(mem, use_psram, ws_sizes[i]);
    }
}

/**
 * @brief Cross-domain gather latency for one working-set size
 * @param[in] tag mem tag
 * @param[in] idx_psram 1 if index array in PSRAM
 * @param[in] data_psram 1 if data array in PSRAM
 * @param[in] ws_bytes each buffer bytes (word-aligned)
 * @return 0 ok, -1 fail
 * @note Per-access escape like seq_read (no sum+= chain).
 */
static int xplat_run_latency_cross_ws(const char *tag,
                                      int idx_psram,
                                      int data_psram,
                                      uint32_t ws_bytes)
{
    uint32_t *idx;
    uint32_t *data;
    uint64_t t0;
    uint64_t us;
    uint32_t i;
    uint32_t n;

#if !CONFIG_PSRAM
    (void)idx_psram;
    (void)data_psram;
    (void)ws_bytes;
    xplat_log_skip("latency_cross", tag, ws_bytes, "no_psram");
    return -1;
#else
    if ((ws_bytes < 4U) || ((ws_bytes & 3U) != 0U)) {
        xplat_log_skip("latency_cross", tag, ws_bytes, "unsupported");
        return -1;
    }
    if ((idx_psram == 0) && (ws_bytes > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("latency_cross", tag, ws_bytes, "sram_cap");
        return -1;
    }
    if ((data_psram == 0) && (ws_bytes > XPLAT_SRAM_MAX_SIZE)) {
        xplat_log_skip("latency_cross", tag, ws_bytes, "sram_cap");
        return -1;
    }

    n = ws_bytes / 4U;
    idx = (uint32_t *)xplat_alloc(ws_bytes, idx_psram);
    data = (uint32_t *)xplat_alloc(ws_bytes, data_psram);
    if ((idx == NULL) || (data == NULL)) {
        xplat_free(idx, idx_psram);
        xplat_free(data, data_psram);
        xplat_log_skip("latency_cross", tag, ws_bytes, "alloc");
        return -1;
    }

    for (i = 0U; i < n; i++) {
        idx[i] = (i * 2654435761U) % n;
        data[i] = i + 1U;
    }

    t0 = bk_aon_rtc_get_us();
    for (i = 0U; i < XPLAT_LATENCY_ACCESSES; i++) {
        uint32_t v = data[idx[i % n]];

        XPLAT_ESCAPE_VAL(v);
    }
    us = bk_aon_rtc_get_us() - t0;
    xplat_log_latency("latency_cross", tag, ws_bytes, XPLAT_LATENCY_ACCESSES, us);

    xplat_free(data, data_psram);
    xplat_free(idx, idx_psram);
    return 0;
#endif
}

/**
 * @brief Cross-domain gather latency matrix (multi working-set)
 * @param[in] tag mem tag
 * @param[in] idx_psram 1 if index in PSRAM
 * @param[in] data_psram 1 if data in PSRAM
 * @param[in] ws_sizes working-set byte table (NULL => default 16K)
 * @param[in] ws_cnt count
 * @return none
 */
static void xplat_run_latency_cross_matrix(const char *tag,
                                           int idx_psram,
                                           int data_psram,
                                           const uint32_t *ws_sizes,
                                           uint32_t ws_cnt)
{
    uint32_t i;
    uint32_t one = (uint32_t)(XPLAT_LATENCY_WORDS * sizeof(uint32_t));

    if ((ws_sizes == NULL) || (ws_cnt == 0U)) {
        (void)xplat_run_latency_cross_ws(tag, idx_psram, data_psram, one);
        return;
    }
    for (i = 0U; i < ws_cnt; i++) {
        (void)xplat_run_latency_cross_ws(tag, idx_psram, data_psram, ws_sizes[i]);
    }
}

/**
 * @brief Extended latency matrix
 * @param[in] standalone 1 reset+summary; 0 append into current run
 * @return none
 */
static void xplat_run_latx_matrix(int standalone)
{
    uint32_t wscnt = (uint32_t)(sizeof(s_xplat_lat_ws_sizes) / sizeof(s_xplat_lat_ws_sizes[0]));

    if (standalone != 0) {
        xplat_rec_reset();
    }
    bk_printf("[xplat] INFO latx matrix begin\r\n");

    xplat_run_latency_store_matrix("sram", 0, s_xplat_lat_ws_sizes, wscnt);
#if CONFIG_PSRAM
    xplat_run_latency_store_matrix("psram", 1, s_xplat_lat_ws_sizes, wscnt);
    xplat_run_latency_cross_matrix("idx_s_data_p", 0, 1, s_xplat_lat_ws_sizes, wscnt);
    xplat_run_latency_cross_matrix("idx_p_data_s", 1, 0, s_xplat_lat_ws_sizes, wscnt);
#else
    xplat_log_skip("latency_store", "psram", 0U, "no_psram");
    xplat_log_skip("latency_cross", "cross", 0U, "no_psram");
#endif

    bk_printf("[xplat] INFO latx matrix done\r\n");
    if (standalone != 0) {
        xplat_print_summary("latx");
    }
}

/**
 * @brief Float add kernel
 * @param[in] loops iteration count
 * @return sink bits
 */
XPLAT_KERNEL_ATTR
static uint32_t xplat_cpu_fadd_kernel(uint32_t loops)
{
    volatile float a = 1.000001f;
    volatile float b = 1.000002f;
    volatile float c = 0.0f;
    uint32_t i;

    for (i = 0U; i < loops; i++) {
        c = a + b;
        a = c;
    }
    s_xplat_sink = (uint32_t)(int32_t)c;
    return s_xplat_sink;
}

/**
 * @brief Float mul kernel
 * @param[in] loops iteration count
 * @return sink bits
 */
XPLAT_KERNEL_ATTR
static uint32_t xplat_cpu_fmul_kernel(uint32_t loops)
{
    volatile float a = 1.000001f;
    volatile float b = 1.000002f;
    volatile float c = 1.0f;
    uint32_t i;

    for (i = 0U; i < loops; i++) {
        c = a * b;
        a = c + 0.000001f;
    }
    s_xplat_sink = (uint32_t)(int32_t)c;
    return s_xplat_sink;
}

/**
 * @brief Log float case as MFLOPS (reuse CPU record)
 * @param[in] name case name
 * @param[in] loops loops
 * @param[in] us elapsed
 * @return none
 */
static void xplat_log_float(const char *name, uint32_t loops, uint64_t us)
{
    float mflops = 0.0f;

    if (us > 0U) {
        mflops = ((float)loops) / ((float)us);
    }
    xplat_rec_add(XPLAT_REC_CPU, name, "", 0U, mflops, us);
    bk_printf("[xplat] case=%s loops=%u freq=%u fpu=%s us=%llu mflops=%.2f\r\n",
              name,
              (unsigned int)loops,
              (unsigned int)xplat_current_freq_mhz(),
#if defined(__ARM_FP) && (__ARM_FP != 0)
              "hard",
#else
              "soft",
#endif
              (unsigned long long)us,
              mflops);
}

/**
 * @brief Float performance matrix
 * @param[in] standalone 1 reset+summary; 0 append into current run
 * @return none
 */
static void xplat_run_float_matrix(int standalone)
{
    uint64_t t0;
    uint64_t us;

    if (standalone != 0) {
        xplat_rec_reset();
    }
    bk_printf("[xplat] INFO float matrix begin fpu=%s\r\n",
#if defined(__ARM_FP) && (__ARM_FP != 0)
              "hard"
#else
              "soft"
#endif
              );

    (void)xplat_cpu_fadd_kernel(1000U);
    t0 = bk_aon_rtc_get_us();
    (void)xplat_cpu_fadd_kernel(XPLAT_CPU_FADD_LOOPS);
    us = bk_aon_rtc_get_us() - t0;
    xplat_log_float("cpu_fadd", XPLAT_CPU_FADD_LOOPS, us);

    (void)xplat_cpu_fmul_kernel(1000U);
    t0 = bk_aon_rtc_get_us();
    (void)xplat_cpu_fmul_kernel(XPLAT_CPU_FMUL_LOOPS);
    us = bk_aon_rtc_get_us() - t0;
    xplat_log_float("cpu_fmul", XPLAT_CPU_FMUL_LOOPS, us);

    bk_printf("[xplat] INFO float matrix done\r\n");
    if (standalone != 0) {
        xplat_print_summary("float");
    }
}

/* ---------------------------------------------------------------------------
 * Matrices
 * --------------------------------------------------------------------------- */

/**
 * @brief Minimal matrix
 * @return none
 */
static void xplat_run_min_matrix(void)
{
    uint32_t i;
    uint32_t cnt = (uint32_t)(sizeof(s_xplat_mem_sizes_min) / sizeof(s_xplat_mem_sizes_min[0]));

    xplat_rec_reset();
    bk_printf("[xplat] INFO min matrix begin cfg_mhz=%u kernel_o0=%d (freq/cache switch disabled)\r\n",
              (unsigned int)xplat_cfg_mhz(), XPLAT_KERNEL_O0);
    xplat_log_freq();

    xplat_run_cpu_suite(0);
    xplat_run_mem_domain("sram", 0, s_xplat_mem_sizes_min, cnt, 0);
    xplat_run_mem_domain("psram", 1, s_xplat_mem_sizes_min, cnt, 0);
    bk_printf("[xplat] INFO phase=cross\r\n");
    for (i = 0U; i < cnt; i++) {
        (void)xplat_run_cross_memcpy(s_xplat_mem_sizes_min[i]);
    }

    bk_printf("[xplat] INFO min matrix done freq=%u\r\n",
              (unsigned int)xplat_current_freq_mhz());
    xplat_print_summary("min");
}

/**
 * @brief SRAM-only bandwidth matrix (memcpy / fill / seq_copy / scalar / seq_read)
 * @return none
 */
static void xplat_run_sram_matrix(void)
{
    uint32_t i;
    uint32_t cnt = (uint32_t)(sizeof(s_xplat_mem_sizes_full) / sizeof(s_xplat_mem_sizes_full[0]));

    xplat_rec_reset();
    bk_printf("[xplat] INFO sram matrix begin cfg_mhz=%u kernel_o0=%d\r\n",
              (unsigned int)xplat_cfg_mhz(), XPLAT_KERNEL_O0);
    xplat_log_freq();

    bk_printf("[xplat] INFO phase=mem mem=sram\r\n");
    for (i = 0U; i < cnt; i++) {
        (void)xplat_run_memcpy_one("sram", s_xplat_mem_sizes_full[i], 0);
        (void)xplat_run_fill_u32_one("sram", s_xplat_mem_sizes_full[i], 0);
        (void)xplat_run_seq_copy_one("sram", s_xplat_mem_sizes_full[i], 0);
        (void)xplat_run_seq_copy_scalar_one("sram", s_xplat_mem_sizes_full[i], 0);
        (void)xplat_run_seq_read_one("sram", s_xplat_mem_sizes_full[i], 0);
    }

    bk_printf("[xplat] INFO sram matrix done freq=%u\r\n",
              (unsigned int)xplat_current_freq_mhz());
    xplat_print_summary_mem("sram", "sram");
}

/**
 * @brief PSRAM-only bandwidth matrix (same cases as xplat sram)
 * @return none
 */
static void xplat_run_psram_matrix(void)
{
    uint32_t i;
    uint32_t cnt = (uint32_t)(sizeof(s_xplat_mem_sizes_full) / sizeof(s_xplat_mem_sizes_full[0]));

    xplat_rec_reset();
    bk_printf("[xplat] INFO psram matrix begin cfg_mhz=%u kernel_o0=%d\r\n",
              (unsigned int)xplat_cfg_mhz(), XPLAT_KERNEL_O0);
    xplat_log_freq();

#if CONFIG_PSRAM
    /* Print actual heap sample addresses; match to `xplat mpu` regions to settle
     * whether PSRAM/SRAM heaps sit in a cacheable window. */
    {
        void *pp = xplat_alloc(64U, XPLAT_MEM_PSRAM);
        void *ps = xplat_alloc(64U, XPLAT_MEM_SRAM);

        bk_printf("[xplat] INFO psram heap sample addr=%p ; sram heap sample addr=%p\r\n",
                  pp, ps);
        xplat_free(pp, XPLAT_MEM_PSRAM);
        xplat_free(ps, XPLAT_MEM_SRAM);
    }
    bk_printf("[xplat] INFO phase=mem mem=psram\r\n");
    for (i = 0U; i < cnt; i++) {
        (void)xplat_run_memcpy_one("psram", s_xplat_mem_sizes_full[i], 1);
        (void)xplat_run_fill_u32_one("psram", s_xplat_mem_sizes_full[i], 1);
        (void)xplat_run_seq_copy_one("psram", s_xplat_mem_sizes_full[i], 1);
        (void)xplat_run_seq_copy_scalar_one("psram", s_xplat_mem_sizes_full[i], 1);
        (void)xplat_run_seq_read_one("psram", s_xplat_mem_sizes_full[i], 1);
    }
#else
    (void)i;
    (void)cnt;
    xplat_log_skip("mem", "psram", 0U, "no_psram");
#endif

    bk_printf("[xplat] INFO psram matrix done freq=%u\r\n",
              (unsigned int)xplat_current_freq_mhz());
    xplat_print_summary_mem("psram", "psram");
}

/**
 * @brief PSRAM non-cacheable bandwidth matrix (MEM_SLAB_HEAP_UNCODED; 0x60 MPU)
 * @return none
 * @note Same cases as xplat psram, but buffers come from the UNCODED slab whose
 *       MPU attribute is 0x60 (non-cacheable), so reads/writes bypass D-cache.
 *       No D-cache disable is performed (that path is unsafe; see file header).
 */
static void xplat_run_psram_nc_matrix(void)
{
    static int s_uncoded_slab_inited = 0;
    uint32_t i;
    uint32_t cnt = (uint32_t)(sizeof(s_xplat_mem_sizes_full) / sizeof(s_xplat_mem_sizes_full[0]));

    xplat_rec_reset();
    bk_printf("[xplat] INFO psram_nc matrix begin cfg_mhz=%u kernel_o0=%d (non-cacheable)\r\n",
              (unsigned int)xplat_cfg_mhz(), XPLAT_KERNEL_O0);
    xplat_log_freq();

#if CONFIG_PSRAM && XPLAT_HAS_FB_UNCODED
    /* The slab heaps are normally brought up by the video subsystem, which this
     * app never starts, so bk_frame_buffer_malloc() would assert "not init".
     * Routed through tkl_media_buf_init(): it owns the one guard, and
     * bk_frame_buffer_init() is destructive. */
    if (s_uncoded_slab_inited == 0) {
        tkl_media_buf_init();
        s_uncoded_slab_inited = 1;
        bk_printf("[xplat] INFO psram_nc slab init done (MEM_SLAB_HEAP_UNCODED)\r\n");
    }
    bk_printf("[xplat] INFO phase=mem mem=psram_nc\r\n");
    for (i = 0U; i < cnt; i++) {
        (void)xplat_run_memcpy_one("psram_nc", s_xplat_mem_sizes_full[i], XPLAT_MEM_PSRAM_NC);
        (void)xplat_run_fill_u32_one("psram_nc", s_xplat_mem_sizes_full[i], XPLAT_MEM_PSRAM_NC);
        (void)xplat_run_seq_copy_one("psram_nc", s_xplat_mem_sizes_full[i], XPLAT_MEM_PSRAM_NC);
        (void)xplat_run_seq_copy_scalar_one("psram_nc", s_xplat_mem_sizes_full[i], XPLAT_MEM_PSRAM_NC);
        (void)xplat_run_seq_read_one("psram_nc", s_xplat_mem_sizes_full[i], XPLAT_MEM_PSRAM_NC);
    }
#else
    (void)i;
    (void)cnt;
#if !XPLAT_HAS_FB_UNCODED
    xplat_log_skip("mem", "psram_nc", 0U, "no_uncoded_slab");
#else
    xplat_log_skip("mem", "psram_nc", 0U, "no_psram");
#endif
#endif

    bk_printf("[xplat] INFO psram_nc matrix done freq=%u\r\n",
              (unsigned int)xplat_current_freq_mhz());
    xplat_print_summary_mem("psram_nc", "psram_nc");
}

/**
 * @brief Full platform performance matrix
 * @return none
 * @note Excludes frequency switch and D-cache disable (MPU unsafe).
 */
static void xplat_run_full_matrix(void)
{
    uint32_t i;
    uint32_t cnt = (uint32_t)(sizeof(s_xplat_mem_sizes_full) / sizeof(s_xplat_mem_sizes_full[0]));
    uint32_t align_cnt = (uint32_t)(sizeof(s_xplat_align_offs) / sizeof(s_xplat_align_offs[0]));

    xplat_rec_reset();
    bk_printf("[xplat] INFO full matrix begin cfg_mhz=%u kernel_o0=%d (freq/cache switch disabled)\r\n",
              (unsigned int)xplat_cfg_mhz(), XPLAT_KERNEL_O0);
    xplat_log_freq();

    /* 1) CPU + float */
    xplat_run_cpu_suite(1);
    bk_printf("[xplat] INFO phase=float\r\n");
    xplat_run_float_matrix(0);

    /* 2) SRAM / PSRAM bandwidth + latency */
    xplat_run_mem_domain("sram", 0, s_xplat_mem_sizes_full, cnt, 1);
    xplat_run_mem_domain("psram", 1, s_xplat_mem_sizes_full, cnt, 1);

    /* 3) Cross domain */
    bk_printf("[xplat] INFO phase=cross\r\n");
    for (i = 0U; i < cnt; i++) {
        (void)xplat_run_cross_memcpy(s_xplat_mem_sizes_full[i]);
    }

    /* 4) Extended latency */
    bk_printf("[xplat] INFO phase=latx\r\n");
    xplat_run_latx_matrix(0);

    /* 4b) Flash XIP seq_read size matrix (SUMMARY vs seqr_s) */
    bk_printf("[xplat] INFO phase=xip\r\n");
    xplat_run_xip_matrix();

    /* 4c) IRQ latency + critical section */
    bk_printf("[xplat] INFO phase=irq\r\n");
    xplat_run_irq_matrix();

    /* 5) Alignment sensitivity on 4KB payload */
    bk_printf("[xplat] INFO phase=align\r\n");
    for (i = 0U; i < align_cnt; i++) {
        (void)xplat_run_memcpy_align("sram", 0, 4096U, s_xplat_align_offs[i]);
        (void)xplat_run_memcpy_align("psram", 1, 4096U, s_xplat_align_offs[i]);
    }

    /* 6) Allocator */
    bk_printf("[xplat] INFO phase=alloc\r\n");
    xplat_run_alloc_case("sram_64", 0, 64U);
    xplat_run_alloc_case("sram_1024", 0, 1024U);
    xplat_run_alloc_case("sram_4096", 0, 4096U);
    xplat_run_alloc_case("psram_64", 1, 64U);
    xplat_run_alloc_case("psram_1024", 1, 1024U);
    xplat_run_alloc_case("psram_4096", 1, 4096U);

    /* 7) Context switch */
    bk_printf("[xplat] INFO phase=os\r\n");
    xplat_run_switch_case();

    /* 8) Cross-core contention */
    bk_printf("[xplat] INFO phase=contend\r\n");
    xplat_run_contend_matrix(0);

    /* 9) DMA engines */
    bk_printf("[xplat] INFO phase=dma\r\n");
    xplat_run_dma_matrix(0);

    bk_printf("[xplat] INFO full matrix done freq=%u\r\n",
              (unsigned int)xplat_current_freq_mhz());
    xplat_print_summary("full");
}

/* ---------------------------------------------------------------------------
 * CLI / task
 * --------------------------------------------------------------------------- */

/**
 * @brief Background worker
 * @param[in] param mode pointer (heap allocated int)
 * @return none
 */
static void xplat_task(void *param)
{
    int mode = XPLAT_MODE_MIN;

    if (param != NULL) {
        mode = *((int *)param);
        os_free(param);
    }

    if (xplat_rec_table_alloc(xplat_rec_cap_for_mode(mode)) != 0) {
        rtos_delete_thread(NULL);
        return;
    }

    if (mode == XPLAT_MODE_FULL) {
        xplat_run_full_matrix();
    } else if (mode == XPLAT_MODE_DMA) {
        xplat_run_dma_matrix(1);
    } else if (mode == XPLAT_MODE_CONTEND) {
        xplat_run_contend_matrix(1);
    } else if (mode == XPLAT_MODE_LATX) {
        xplat_run_latx_matrix(1);
    } else if (mode == XPLAT_MODE_FLOAT) {
        xplat_run_float_matrix(1);
    } else if (mode == XPLAT_MODE_SRAM) {
        xplat_run_sram_matrix();
    } else if (mode == XPLAT_MODE_PSRAM) {
        xplat_run_psram_matrix();
    } else if (mode == XPLAT_MODE_PSRAM_NC) {
        xplat_run_psram_nc_matrix();
    } else if (mode == XPLAT_MODE_MPU) {
        xplat_log_mpu();
    } else {
        xplat_run_min_matrix();
    }

    xplat_rec_table_free();
    rtos_delete_thread(NULL);
}

/**
 * @brief Mode name for log
 * @param[in] mode mode id
 * @return name string
 */
static const char *xplat_mode_name(int mode)
{
    switch (mode) {
    case XPLAT_MODE_FULL:
        return "full";
    case XPLAT_MODE_DMA:
        return "dma";
    case XPLAT_MODE_CONTEND:
        return "contend";
    case XPLAT_MODE_LATX:
        return "latx";
    case XPLAT_MODE_FLOAT:
        return "float";
    case XPLAT_MODE_SRAM:
        return "sram";
    case XPLAT_MODE_PSRAM:
        return "psram";
    case XPLAT_MODE_PSRAM_NC:
        return "psram_nc";
    case XPLAT_MODE_MPU:
        return "mpu";
    case XPLAT_MODE_MIN:
    default:
        return "min";
    }
}

/**
 * @brief Start xplat worker task
 * @param[in] mode XPLAT_MODE_*
 * @return none
 */
static void xplat_start_task(int mode)
{
    int *arg = (int *)os_malloc(sizeof(int));
    bk_err_t ret;

    if (arg == NULL) {
        bk_printf("[xplat] ERROR no memory for task arg\r\n");
        return;
    }
    *arg = mode;

#if defined(CONFIG_FREERTOS_SMP) && (CONFIG_FREERTOS_SMP == 1) && (CONFIG_CPU_CNT > 1)
    /* Pin full/contend to core0 so contend peer can use core1 */
    if ((mode == XPLAT_MODE_CONTEND) || (mode == XPLAT_MODE_FULL)) {
        ret = rtos_core0_create_thread(NULL,
                                       XPLAT_TASK_PRIO,
                                       "xplat",
                                       xplat_task,
                                       XPLAT_TASK_STACK,
                                       (beken_thread_arg_t)arg);
    } else {
        ret = rtos_create_thread(NULL,
                                 XPLAT_TASK_PRIO,
                                 "xplat",
                                 xplat_task,
                                 XPLAT_TASK_STACK,
                                 (beken_thread_arg_t)arg);
    }
#else
    ret = rtos_create_thread(NULL,
                             XPLAT_TASK_PRIO,
                             "xplat",
                             xplat_task,
                             XPLAT_TASK_STACK,
                             (beken_thread_arg_t)arg);
#endif
    if (ret != kNoErr) {
        os_free(arg);
        bk_printf("[xplat] ERROR create task failed\r\n");
        return;
    }
    bk_printf("[xplat] INFO task started mode=%s\r\n", xplat_mode_name(mode));
}

/**
 * @brief Print usage
 * @return none
 */
static void xplat_usage(void)
{
    bk_printf("Usage:\r\n");
    bk_printf("  xplat help\r\n");
    bk_printf("  xplat min      quick: cpu + memcpy/memset/latency + cross\r\n");
    bk_printf("  xplat full     full: cpu/float + bw/lat + latx + align + alloc + switch + contend + dma\r\n");
    bk_printf("  xplat dma      gen-dma(+hp-dma if present) memcpy BW vs CPU (standalone)\r\n");
    bk_printf("  xplat contend  cpu_mul vs memcpy contention (standalone)\r\n");
    bk_printf("  xplat latx     store + cross-domain latency (standalone)\r\n");
    bk_printf("  xplat float    fadd/fmul MFLOPS (standalone)\r\n");
    bk_printf("  xplat sram     SRAM BW: memcpy/fill/scopy/scals/seqr + SUMMARY\r\n");
    bk_printf("  xplat psram    PSRAM BW: same cases as xplat sram + SUMMARY\r\n");
    bk_printf("  xplat psram_nc PSRAM non-cacheable BW (UNCODED slab, 0x60 MPU) + SUMMARY\r\n");
    bk_printf("  xplat mpu      dump live MPU regions + MAIR attrs (SRAM/PSRAM cacheability)\r\n");
    bk_printf("  (no freq switch; no dcache off)\r\n");
}

/**
 * @brief CLI entry
 * @param[in] pcWriteBuffer unused
 * @param[in] xWriteBufferLen unused
 * @param[in] argc argc
 * @param[in] argv argv
 * @return none
 */
void cli_xplat_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    if ((argc < 2) || (argv == NULL) || (argv[1] == NULL) ||
        (os_strcmp(argv[1], "help") == 0) || (os_strcmp(argv[1], "?") == 0)) {
        xplat_usage();
        return;
    }

    if (os_strcmp(argv[1], "min") == 0) {
        xplat_start_task(XPLAT_MODE_MIN);
        return;
    }
    if (os_strcmp(argv[1], "full") == 0) {
        xplat_start_task(XPLAT_MODE_FULL);
        return;
    }
    if (os_strcmp(argv[1], "dma") == 0) {
        xplat_start_task(XPLAT_MODE_DMA);
        return;
    }
    if (os_strcmp(argv[1], "contend") == 0) {
        xplat_start_task(XPLAT_MODE_CONTEND);
        return;
    }
    if (os_strcmp(argv[1], "latx") == 0) {
        xplat_start_task(XPLAT_MODE_LATX);
        return;
    }
    if (os_strcmp(argv[1], "float") == 0) {
        xplat_start_task(XPLAT_MODE_FLOAT);
        return;
    }
    if (os_strcmp(argv[1], "sram") == 0) {
        xplat_start_task(XPLAT_MODE_SRAM);
        return;
    }
    if (os_strcmp(argv[1], "psram") == 0) {
        xplat_start_task(XPLAT_MODE_PSRAM);
        return;
    }
    if (os_strcmp(argv[1], "psram_nc") == 0) {
        xplat_start_task(XPLAT_MODE_PSRAM_NC);
        return;
    }
    if (os_strcmp(argv[1], "mpu") == 0) {
        xplat_start_task(XPLAT_MODE_MPU);
        return;
    }

    bk_printf("[xplat] ERROR unknown subcmd: %s\r\n", argv[1]);
    xplat_usage();
}
