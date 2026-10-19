/**
 * @file tuya_deepsleep_stress.c
 * @brief T9 深睡唤醒压测
 * @version 0.1
 * @date 2026-10-16
 * @copyright Copyright (c) Tuya Inc.
 *
 * 目的：反复"进深睡 → RTC 唤醒 → 重启"，验证长时间循环的稳定性。
 *
 * ── 循环节奏 ────────────────────────────────────────────────────────────────
 * 每次真深睡需要两次开机，因为 tkl_sleep_param_check_and_set() 在进睡前会把
 * flash 标志擦掉（避免"唤醒→复位→又读到标志→再睡"的死循环）：
 *
 *   开机#N   : flash 无标志 → 报数 → 注册 RTC 唤醒源 → 落盘 → 复位
 *   开机#N+1 : flash 有标志 → 报数 → 进深睡 → RTC 到点唤醒 → 复位
 *   开机#N+2 : 同 #N，循环
 *
 * 所以日志里 boots 每轮 +2、sleeps 每轮 +1。RTC 的间隔是**真的睡了那么久**；
 * 两次开机之间的额外开销是落盘延时（tkl_sleep.c 的 TKL_DS_FLUSH_DELAY_MS）
 * 加重启时间。
 *
 * ── 计数器什么时候归零 ──────────────────────────────────────────────────────
 * flash 里的计数块带三重失效判据，任一命中即从 0 重开：
 *   1. 复位原因是 RESET_SOURCE_POWERON —— 手动拔插电源
 *      （可用 TUYA_DEEPSLEEP_STRESS_RESET_ON_POWERON 关掉，改成跨上下电累加）
 *   2. 编译时间戳变了 —— 重新烧录固件
 *   3. RTC 间隔或轮数上限变了 —— 改了压测参数
 * 而压测循环自身的复位（RESET_SOURCE_REBOOT / RESET_SOURCE_DEEPPS_*）**不会**
 * 归零，否则永远停在 #1。
 *
 * ── 退出方式（按推荐顺序）────────────────────────────────────────────────────
 *   1. 把 TUYA_DEEPSLEEP_STRESS_TEST 改回 0，重新编译烧录 —— 推荐
 *   2. 或把 TUYA_DEEPSLEEP_STRESS_MAX_ROUNDS 设为有限值，跑满自动恢复正常启动
 *   3. 或直接给板子断电再上电（RESET_SOURCE_POWERON）—— 计数器归零，
 *      但压测会立刻重开一轮，要彻底退出仍需方式 1
 *   4. 应急：xsleep ds clear 擦标志 —— 但压测会在下次开机重新落盘，只能争取到
 *      一次正常启动的窗口，仍需配合方式 1
 */

#include <string.h>
#include <stddef.h>

#include "tuya_cloud_types.h"
#include "tkl_flash.h"
#include "tkl_sleep.h"
#include "tkl_wakeup.h"

#include <components/system.h>      /* bk_misc_get_reset_reason / RESET_SOURCE_* */

#include "tuya_deepsleep_stress.h"

#if TUYA_DEEPSLEEP_STRESS_TEST

/*===========================================================================
 * 计数块
 *=========================================================================*/

/* 借用 Tuya 用户区 TUYA_USER0 的首扇区。
 * 该区在 tkl_flash.c 里以 TUYA_USER0(0x7E5000, 32K) 暴露，全树无其他消费者。
 * 擦除范围 0x7E5000~0x7E6000，与深睡参数区 0x7ED000 不重叠。 */
#define DSTRESS_ADDR            0x7E5000u
#define DSTRESS_ERASE_SIZE      4096

#define DSTRESS_MAGIC           0x44535452u     /* "DSTR" */

typedef struct {
    uint32_t magic;
    uint32_t build_stamp;   /* 编译时间戳；重烧固件即失效 */
    uint32_t rtc_ms;        /* 压测参数；改了也失效 */
    uint32_t max_rounds;
    uint32_t boots;         /* 累计开机次数 */
    uint32_t sleeps;        /* 累计真正进入深睡的次数（= 完成的唤醒轮数） */
    uint32_t abnormal;      /* 非预期复位原因的次数 */
    uint32_t last_reason;   /* 上一次开机的复位原因 */
    uint32_t sum;
} dstress_blk_t;

/* 校验和覆盖 sum 之前的全部字节。
 * 用 offsetof 而不是 sizeof-4：结构体可能有尾部填充，sizeof-4 会把 sum 自身
 * 算进校验和（tkl_sleep.c 里踩过这个坑）。 */
#define DSTRESS_SUM_SPAN        offsetof(dstress_blk_t, sum)

static uint32_t __dstress_sum(const dstress_blk_t *b)
{
    const uint8_t *raw = (const uint8_t *)b;
    uint32_t       s   = 0;
    uint32_t       i;

    for (i = 0; i < DSTRESS_SUM_SPAN; i++) {
        s += raw[i];
    }

    return s;
}

/**
 * @brief 本固件的编译时间戳（FNV-1a 哈希）
 *
 * 每次重编本文件都会变 —— 重烧固件后计数自动归零。
 * 注意：只改别的文件时本文件不重编、戳不变；那种情况下靠"手动上下电归零"兜底。
 */
static uint32_t __dstress_build_stamp(void)
{
    static const char stamp[] = __DATE__ " " __TIME__;
    uint32_t          h       = 2166136261u;
    const char       *p;

    for (p = stamp; *p != '\0'; p++) {
        h ^= (uint8_t)*p;
        h *= 16777619u;
    }

    return h;
}

/*===========================================================================
 * 复位原因
 *=========================================================================*/

/**
 * @brief 是否是压测循环自身的复位
 *
 * 循环内预期的复位原因只有两类：
 *   RESET_SOURCE_REBOOT     —— tkl_cpu_sleep_mode_set() 落盘后调的 tkl_system_reset()
 *   RESET_SOURCE_DEEPPS_*   —— 深睡被唤醒源叫醒
 * 其余（看门狗、各类 crash、掉电…）都算异常，计入 abnormal 计数 ——
 * 压测的意义正在于把这些抓出来。
 */
static int __dstress_reason_expected(uint32_t r)
{
    return (r == RESET_SOURCE_REBOOT)      ||
           (r == RESET_SOURCE_DEEPPS_RTC)  ||
           (r == RESET_SOURCE_DEEPPS_GPIO) ||
           (r == RESET_SOURCE_DEEPPS_USB)  ||
           (r == RESET_SOURCE_DEEPPS_TOUCH);
}

static const char *__dstress_reason_name(uint32_t r)
{
    switch (r) {
        case RESET_SOURCE_POWERON:            return "power-on";
        case RESET_SOURCE_REBOOT:             return "reboot";
        case RESET_SOURCE_WATCHDOG:           return "WATCHDOG";
        case RESET_SOURCE_DEEPPS_RTC:         return "deep-sleep/RTC";
        case RESET_SOURCE_DEEPPS_GPIO:        return "deep-sleep/GPIO";
        case RESET_SOURCE_DEEPPS_USB:         return "deep-sleep/USB";
        case RESET_SOURCE_DEEPPS_TOUCH:       return "deep-sleep/TOUCH";
        case RESET_SOURCE_CRASH_ILLEGAL_JUMP: return "CRASH/illegal-jump";
        case RESET_SOURCE_CRASH_ASSERT:       return "CRASH/assert";
        case RESET_SOURCE_CRASH_DATA_ABORT:   return "CRASH/data-abort";
        case RESET_SOURCE_HARD_FAULT:         return "CRASH/hard-fault";
        case RESET_SOURCE_NMI_WDT:            return "NMI-WDT";
        default:                              return "other";
    }
}

/*===========================================================================
 * 对外接口
 *=========================================================================*/

int tuya_deepsleep_stress_boot(void)
{
    TKL_DS_PARAM_T ds_param;
    dstress_blk_t  b;
    uint32_t       reason = bk_misc_get_reset_reason();
    const char    *why    = NULL;
    int            fresh  = 0;
    int            pending;

    /* 深睡参数区里有没有待处理的标志 —— 决定本轮是"落盘复位"还是"真进深睡" */
    memset(&ds_param, 0, sizeof(ds_param));
    tkl_flash_read(DEEPSLEEP_PARAMETER_ADDRESS, (UCHAR_T *)&ds_param, sizeof(ds_param));
    pending = (ds_param.magic == DEEPSLEEP_MAGIC) && (ds_param.entry_flag == DS_ENTRY_FLAG);

    memset(&b, 0, sizeof(b));
    tkl_flash_read(DSTRESS_ADDR, (UCHAR_T *)&b, sizeof(b));

    if ((b.magic != DSTRESS_MAGIC) || (b.sum != __dstress_sum(&b))) {
        fresh = 1;
        why   = "首次使用/计数块损坏";
    } else if (b.build_stamp != __dstress_build_stamp()) {
        fresh = 1;
        why   = "固件已更换";
    } else if ((b.rtc_ms != TUYA_DEEPSLEEP_STRESS_RTC_MS) ||
               (b.max_rounds != TUYA_DEEPSLEEP_STRESS_MAX_ROUNDS)) {
        fresh = 1;
        why   = "压测参数已变更";
    }
#if TUYA_DEEPSLEEP_STRESS_RESET_ON_POWERON
    else if (reason == RESET_SOURCE_POWERON) {
        fresh = 1;
        why   = "手动上下电";
    }
#endif

    if (fresh) {
        bk_printf("[stress] === new run (%s), counter reset ===\r\n", why);
        memset(&b, 0, sizeof(b));
    } else if (!__dstress_reason_expected(reason)) {
        b.abnormal++;
    }

    b.magic       = DSTRESS_MAGIC;
    b.build_stamp = __dstress_build_stamp();
    b.rtc_ms      = TUYA_DEEPSLEEP_STRESS_RTC_MS;
    b.max_rounds  = TUYA_DEEPSLEEP_STRESS_MAX_ROUNDS;
    b.boots++;
    b.last_reason = reason;
    if (pending) {
        b.sleeps++;
    }
    b.sum = __dstress_sum(&b);

    /* 计数块写失败不影响压测本身，只影响报数 —— 打日志即可，不阻断 */
    if (tkl_flash_erase(DSTRESS_ADDR, DSTRESS_ERASE_SIZE) != OPRT_OK) {
        bk_printf("[stress] WARN: counter erase failed\r\n");
    }
    if (tkl_flash_write(DSTRESS_ADDR, (CONST UCHAR_T *)&b, sizeof(b)) != OPRT_OK) {
        bk_printf("[stress] WARN: counter write failed\r\n");
    }

    bk_printf("[stress] boot #%u / deepsleep #%u | reset=%s(0x%x) | abnormal=%u | rtc=%ums\r\n",
              (unsigned)b.boots, (unsigned)b.sleeps,
              __dstress_reason_name(reason), (unsigned)reason,
              (unsigned)b.abnormal, (unsigned)b.rtc_ms);
    bk_printf("[stress] %s\r\n",
              pending ? "flag pending -> enter deep sleep now"
                      : "no flag -> arm wakeup, save param, reboot");

#if (TUYA_DEEPSLEEP_STRESS_MAX_ROUNDS > 0)
    if (b.sleeps >= (uint32_t)TUYA_DEEPSLEEP_STRESS_MAX_ROUNDS) {
        bk_printf("[stress] reached %d deepsleep rounds, stress finished, boot normally\r\n",
                  TUYA_DEEPSLEEP_STRESS_MAX_ROUNDS);
        return 0;
    }
#endif

    return 1;
}

void tuya_deepsleep_stress_arm_next(void)
{
    TUYA_WAKEUP_SOURCE_BASE_CFG_T cfg = {0};
    OPERATE_RET                   ret;

    cfg.source                   = TUYA_WAKEUP_SOURCE_RTC;
    cfg.wakeup_para.rtc_param.ms = TUYA_DEEPSLEEP_STRESS_RTC_MS;

    ret = tkl_wakeup_source_set(&cfg);
    if (ret != OPRT_OK) {
        bk_printf("[stress] arm RTC %u ms FAILED (%d), abort stress\r\n",
                  (unsigned)TUYA_DEEPSLEEP_STRESS_RTC_MS, ret);
        return;
    }

    /* 落盘并复位。走到下一行说明落盘失败（成功的话已经在复位后的开机里了） */
    (void)tkl_cpu_sleep_mode_set(TRUE, TUYA_CPU_DEEP_SLEEP);

    bk_printf("[stress] save param FAILED, abort stress\r\n");
}

#endif /* TUYA_DEEPSLEEP_STRESS_TEST */
