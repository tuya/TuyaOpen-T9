/**
 * @file test_sleep.c
 * @brief 休眠适配的串口验证命令（xsleep）
 *
 * 命令按**休眠模式**分层，两种模式各自一套子命令，互不干扰：
 *
 *   xsleep ds <sub>     深度休眠（deep sleep）
 *   xsleep ka <sub>     保活休眠（keepalive sleep）—— 待实现
 *
 * 深度休眠最小验证集：
 *   xsleep ds status                 读 flash 深睡标志 + 打印 RAM 唤醒源表（只读）
 *   xsleep ds gpio <pin> [level]     注册一个 GPIO 唤醒源，打印厂商返回值
 *                                    level: low|high|rise|fall 或 0|1|2|3
 *   xsleep ds rtc <ms>               注册一个 RTC 唤醒源，打印厂商返回值
 *   xsleep ds clear                  擦掉 flash 里的深睡标志（误入深睡的逃生口）
 *   xsleep ds enter                  落盘并重启，真正进入深睡
 *
 * 典型验证顺序：
 *   1. xsleep ds status              —— 确认无待处理标志
 *   2. xsleep ds gpio 30 rise        —— 确认 GPIO 唤醒源注册成功
 *   3. xsleep ds rtc 30000           —— 确认 RTC 唤醒源注册成功
 *   4. xsleep ds status              —— 确认 RAM 表里有两项
 *   5. xsleep ds enter               —— 落盘并重启
 *   6. 重启后看串口                  —— 应打印 "=== entry deepsleep" 后停住
 *   7. 按键或等 30s                  —— 设备应被唤醒并重新启动
 *
 * 若第 5 步之后设备反复重启、应用起不来，用 xsleep ds clear 擦掉标志恢复。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/str.h>

#include "cli_tuya_test.h"
#include "tuya_cloud_types.h"
#include "tkl_flash.h"
#include "tkl_sleep.h"
#include "tkl_wakeup.h"

extern void bk_printf(const char *fmt, ...);

/* 非契约接口，与 tkl_sleep.c 用同样的 extern 声明方式 */
extern OPERATE_RET tkl_wakeup_source_get(TUYA_WAKEUP_SOURCE_BASE_CFG_T *param,
                                         UINT32_T num, UINT32_T *status);

#define SLEEP_TEST_TABLE_N      6

/*=====================================================================
 * 共用辅助
 *===================================================================*/

/**
 * @brief 判断 flash 里是否有待消费的深睡标志
 *
 * 只看 magic 与 entry_flag —— 与 tkl_sleep.c 里 tkl_sleep_param_check_and_set()
 * 的前两个判据一致。开机路径还会多校验一道 sum，这里不重复实现，
 * sum 的值打印出来供人工判读。
 */
static bool __sleep_test_flag_pending(const TKL_DS_PARAM_T *ds)
{
    return (ds->magic == DEEPSLEEP_MAGIC) && (ds->entry_flag == DS_ENTRY_FLAG);
}

/**
 * @brief 解析唤醒电平
 *
 * 接受名字（low/high/rise/fall）或数字（0=low 1=high 2=rise 3=fall，
 * 与 TUYA_GPIO_WAKE_TYPE_E 的取值一致）。
 *
 * 无法识别时返回 -1，由调用方报错 —— 不要静默退化成默认值：曾经因为
 * "1" 不被识别而悄悄变成 RISE，输出又回显了原始字符串，看着像生效了。
 */
static int __sleep_test_parse_level(const char *s, TUYA_GPIO_WAKE_TYPE_E *out)
{
    if (s == NULL || out == NULL) {
        return -1;
    }
    if (os_strcmp(s, "low") == 0)  { *out = TUYA_GPIO_WAKEUP_LOW;  return 0; }
    if (os_strcmp(s, "high") == 0) { *out = TUYA_GPIO_WAKEUP_HIGH; return 0; }
    if (os_strcmp(s, "rise") == 0) { *out = TUYA_GPIO_WAKEUP_RISE; return 0; }
    if (os_strcmp(s, "fall") == 0) { *out = TUYA_GPIO_WAKEUP_FALL; return 0; }

    if (os_strcmp(s, "0") == 0) { *out = TUYA_GPIO_WAKEUP_LOW;  return 0; }
    if (os_strcmp(s, "1") == 0) { *out = TUYA_GPIO_WAKEUP_HIGH; return 0; }
    if (os_strcmp(s, "2") == 0) { *out = TUYA_GPIO_WAKEUP_RISE; return 0; }
    if (os_strcmp(s, "3") == 0) { *out = TUYA_GPIO_WAKEUP_FALL; return 0; }

    return -1;
}

static const char *__sleep_test_level_name(TUYA_GPIO_WAKE_TYPE_E level)
{
    switch (level) {
        case TUYA_GPIO_WAKEUP_LOW:  return "low";
        case TUYA_GPIO_WAKEUP_HIGH: return "high";
        case TUYA_GPIO_WAKEUP_RISE: return "rise";
        case TUYA_GPIO_WAKEUP_FALL: return "fall";
        default:                    return "?";
    }
}

/**
 * @brief 打印 RAM 里的唤醒源表
 *
 * 表从未分配时会返回 OPRT_NOT_FOUND —— 这正是深睡落盘走"空表"分支的条件，
 * 值得单独报出来。保活休眠将来也复用这张表，故放在共用区。
 */
static void __sleep_test_dump_table(void)
{
    TUYA_WAKEUP_SOURCE_BASE_CFG_T tbl[SLEEP_TEST_TABLE_N];
    UINT32_T                      status = 0;
    OPERATE_RET                   ret;
    INT_T                         i;
    int                           n = 0;

    memset(tbl, 0, sizeof(tbl));
    ret = tkl_wakeup_source_get(tbl, SLEEP_TEST_TABLE_N, &status);
    if (ret != OPRT_OK) {
        bk_printf("[xsleep] ram table: NOT ALLOCATED (ret=%d) -> deepsleep will "
                  "use an empty table (reset-only wake)\r\n", ret);
        return;
    }

    bk_printf("[xsleep] ram table (status=0x%x):\r\n", (unsigned)status);
    for (i = 0; i < SLEEP_TEST_TABLE_N; i++) {
        if (tbl[i].source == TUYA_WAKEUP_SOURCE_GPIO) {
            bk_printf("[xsleep]   slot%d gpio=%d level=%d\r\n", (int)i,
                      (int)tbl[i].wakeup_para.gpio_param.gpio_num,
                      (int)tbl[i].wakeup_para.gpio_param.level);
            n++;
        } else if (tbl[i].source == TUYA_WAKEUP_SOURCE_RTC) {
            bk_printf("[xsleep]   slot%d rtc=%d ms\r\n", (int)i,
                      (int)tbl[i].wakeup_para.rtc_param.ms);
            n++;
        }
    }
    bk_printf("[xsleep] ram table: %d entr%s\r\n", n, (n == 1) ? "y" : "ies");
}

/**
 * @brief 注册一个唤醒源并打印厂商返回值
 *
 * 深睡与保活休眠共用同一张唤醒源表，故提取为共用辅助。
 */
static OPERATE_RET __sleep_test_add_gpio(UINT32_T pin, TUYA_GPIO_WAKE_TYPE_E level)
{
    TUYA_WAKEUP_SOURCE_BASE_CFG_T cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.source = TUYA_WAKEUP_SOURCE_GPIO;
    cfg.wakeup_para.gpio_param.gpio_num = (TUYA_GPIO_NUM_E)pin;
    cfg.wakeup_para.gpio_param.level    = level;

    return tkl_wakeup_source_set(&cfg);
}

static OPERATE_RET __sleep_test_add_rtc(UINT32_T ms)
{
    TUYA_WAKEUP_SOURCE_BASE_CFG_T cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.source = TUYA_WAKEUP_SOURCE_RTC;
    cfg.wakeup_para.rtc_param.ms = ms;

    return tkl_wakeup_source_set(&cfg);
}

/*=====================================================================
 * 深度休眠（xsleep ds ...）
 *===================================================================*/

/**
 * @brief 读 flash 深睡标志并打印
 */
static void __sleep_test_ds_dump_flag(void)
{
    TKL_DS_PARAM_T ds;
    INT_T          i;
    int            valid;

    memset(&ds, 0, sizeof(ds));
    tkl_flash_read(DEEPSLEEP_PARAMETER_ADDRESS, (UCHAR_T *)&ds, sizeof(ds));
    valid = __sleep_test_flag_pending(&ds);

    bk_printf("[xsleep ds] flash @0x%x: magic=0x%08x entry=0x%08x sum=0x%08x -> %s\r\n",
              (unsigned)DEEPSLEEP_PARAMETER_ADDRESS,
              (unsigned)ds.magic, (unsigned)ds.entry_flag,
              (unsigned)ds.sum, valid ? "PENDING (will deepsleep on boot)" : "empty");

    if (!valid) {
        return;
    }

    for (i = 0; i < DS_MAX_CFG_ITEM; i++) {
        if (ds.cfg[i].source == TUYA_WAKEUP_SOURCE_GPIO) {
            bk_printf("[xsleep ds]   slot%d gpio=%d level=%d\r\n", (int)i,
                      (int)ds.cfg[i].wakeup_para.gpio_param.gpio_num,
                      (int)ds.cfg[i].wakeup_para.gpio_param.level);
        } else if (ds.cfg[i].source == TUYA_WAKEUP_SOURCE_RTC) {
            bk_printf("[xsleep ds]   slot%d rtc=%d ms\r\n", (int)i,
                      (int)ds.cfg[i].wakeup_para.rtc_param.ms);
        }
    }
}

static void __sleep_test_ds_usage(char *pcWriteBuffer, int xWriteBufferLen)
{
    snprintf(pcWriteBuffer, xWriteBufferLen,
             "xsleep ds status | gpio <pin> [low|high|rise|fall|0|1|2|3] | "
             "rtc <ms> | clear | enter\r\n");
}

/**
 * @brief 深度休眠子命令分发
 *
 * @param[in] argc/argv: 已偏移到 "ds"，即 argv[0]=="ds"、argv[1]==子命令
 */
static void __sleep_test_ds_cmd(char *pcWriteBuffer, int xWriteBufferLen,
                                int argc, char **argv)
{
    const char  *cmd;
    OPERATE_RET  ret;

    if (argc < 2) {
        __sleep_test_ds_usage(pcWriteBuffer, xWriteBufferLen);
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "status") == 0) {
        __sleep_test_ds_dump_flag();
        __sleep_test_dump_table();
        snprintf(pcWriteBuffer, xWriteBufferLen, "xsleep ds status ok\r\n");
        return;
    }

    if (os_strcmp(cmd, "gpio") == 0) {
        TUYA_GPIO_WAKE_TYPE_E level;
        UINT32_T              pin;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xsleep ds gpio <pin> [low|high|rise|fall | 0|1|2|3]\r\n");
            return;
        }

        if (__sleep_test_parse_level((argc >= 4) ? argv[3] : "rise", &level) != 0) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xsleep ds gpio: bad level '%s', use low|high|rise|fall or 0|1|2|3\r\n",
                     argv[3]);
            return;
        }

        pin = (UINT32_T)os_strtoul(argv[2], NULL, 10);
        ret = __sleep_test_add_gpio(pin, level);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xsleep ds gpio %u level=%s(%d) -> %s (%d)\r\n",
                 (unsigned)pin, __sleep_test_level_name(level), (int)level,
                 (ret == OPRT_OK) ? "ok" : "FAILED", ret);
        return;
    }

    if (os_strcmp(cmd, "rtc") == 0) {
        UINT32_T ms;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "xsleep ds rtc <ms>\r\n");
            return;
        }

        ms  = (UINT32_T)os_strtoul(argv[2], NULL, 10);
        ret = __sleep_test_add_rtc(ms);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xsleep ds rtc %u ms -> %s (%d)\r\n",
                 (unsigned)ms, (ret == OPRT_OK) ? "ok" : "FAILED", ret);
        return;
    }

    if (os_strcmp(cmd, "clear") == 0) {
        TKL_DS_PARAM_T rd;

        ret = tkl_flash_erase(DEEPSLEEP_PARAMETER_ADDRESS, 4096);
        if (ret != OPRT_OK) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xsleep ds clear: erase FAILED (%d)\r\n", ret);
            return;
        }

        /* 回读确认 —— T9 的 tkl_flash_erase() 吞掉厂商返回值，只判返回值不可靠 */
        memset(&rd, 0, sizeof(rd));
        tkl_flash_read(DEEPSLEEP_PARAMETER_ADDRESS, (UCHAR_T *)&rd, sizeof(rd));
        if (__sleep_test_flag_pending(&rd)) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xsleep ds clear: flag STILL VALID after erase\r\n");
            return;
        }

        snprintf(pcWriteBuffer, xWriteBufferLen, "xsleep ds clear: flag erased\r\n");
        return;
    }

    if (os_strcmp(cmd, "enter") == 0) {
        bk_printf("[xsleep ds] entering deepsleep: save param -> reboot -> sleep\r\n");
        bk_printf("[xsleep ds] if the app never comes back, use 'xsleep ds clear' "
                  "after a power cycle\r\n");
        ret = tkl_cpu_sleep_mode_set(TRUE, TUYA_CPU_DEEP_SLEEP);
        /* 走到这里说明落盘失败，没有复位 */
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xsleep ds enter: FAILED to save param (%d), not sleeping\r\n", ret);
        return;
    }

    __sleep_test_ds_usage(pcWriteBuffer, xWriteBufferLen);
}

/*=====================================================================
 * 保活休眠（xsleep ka ...）—— 待实现
 *===================================================================*/

/**
 * @brief 保活休眠子命令分发
 *
 * 尚未实现。这里先把命名空间占住，避免与 ds 的子命令冲突。
 *
 * 实现时在此处分发，可直接复用上面的共用辅助：
 *   __sleep_test_dump_table()      打印唤醒源表
 *   __sleep_test_add_gpio/rtc()    注册唤醒源
 * 保活休眠自己的标志区/参数区应另择 flash 地址，不要复用
 * DEEPSLEEP_PARAMETER_ADDRESS(0x7ED000)。
 */
static void __sleep_test_ka_cmd(char *pcWriteBuffer, int xWriteBufferLen,
                                int argc, char **argv)
{
    (void)argc;
    (void)argv;

    snprintf(pcWriteBuffer, xWriteBufferLen,
             "xsleep ka: keepalive sleep test NOT IMPLEMENTED yet\r\n");
}

/*=====================================================================
 * 顶层分发
 *===================================================================*/

static void __sleep_test_usage(char *pcWriteBuffer, int xWriteBufferLen)
{
    snprintf(pcWriteBuffer, xWriteBufferLen,
             "xsleep ds status|gpio <pin> [level]|rtc <ms>|clear|enter  "
             "(deep sleep)\r\n"
             "xsleep ka ...  (keepalive sleep, not implemented yet)\r\n");
}

/**
 * @brief 休眠适配验证命令
 *
 * 第一层按休眠模式分发（ds / ka），第二层由各模式自己处理。
 */
void cli_tkl_sleep_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        __sleep_test_usage(pcWriteBuffer, xWriteBufferLen);
        return;
    }

    if (os_strcmp(argv[1], "ds") == 0) {
        __sleep_test_ds_cmd(pcWriteBuffer, xWriteBufferLen, argc - 1, argv + 1);
        return;
    }

    if (os_strcmp(argv[1], "ka") == 0) {
        __sleep_test_ka_cmd(pcWriteBuffer, xWriteBufferLen, argc - 1, argv + 1);
        return;
    }

    __sleep_test_usage(pcWriteBuffer, xWriteBufferLen);
}
