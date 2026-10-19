/*
 * tuya_deepsleep_stress.h
 * Copyright (C) 2026 Tuya
 *
 * Distributed under terms of the MIT license.
 *
 * 深睡唤醒压测 —— 编译开关与对外接口。
 * 实现见同目录 tuya_deepsleep_stress.c，调用点见 ap_main.c 的 entry_app_main()。
 */

#ifndef __TUYA_DEEPSLEEP_STRESS_H__
#define __TUYA_DEEPSLEEP_STRESS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * 编译开关
 * =========================================================================*/

/* 1 = 启用深睡压测。可从构建系统覆盖：-DTUYA_DEEPSLEEP_STRESS_TEST=1 */
#ifndef TUYA_DEEPSLEEP_STRESS_TEST
#define TUYA_DEEPSLEEP_STRESS_TEST              0
#endif

/* RTC 唤醒间隔（毫秒） */
#ifndef TUYA_DEEPSLEEP_STRESS_RTC_MS
#define TUYA_DEEPSLEEP_STRESS_RTC_MS            10000
#endif

/* 跑满多少轮真深睡后自动停止、恢复正常启动。0 = 不限轮数 */
#ifndef TUYA_DEEPSLEEP_STRESS_MAX_ROUNDS
#define TUYA_DEEPSLEEP_STRESS_MAX_ROUNDS        0
#endif

/* 手动上下电（RESET_SOURCE_POWERON）时是否重新计数。
 *   1 = 重新计数（默认）：拔插电源即开新一轮，方便拿干净基线。
 *   0 = 跨上下电继续累加，适合"拔电也不影响"的长期老化测试。 */
#ifndef TUYA_DEEPSLEEP_STRESS_RESET_ON_POWERON
#define TUYA_DEEPSLEEP_STRESS_RESET_ON_POWERON  1
#endif

/* ===========================================================================
 * 接口
 * =========================================================================*/

/* 声明始终可见，实现只在 TUYA_DEEPSLEEP_STRESS_TEST=1 时参与编译，
 * 所以调用点必须自己用 #if 包起来（见 ap_main.c）。
 * 故意不提供空操作宏：漏写 #if 时是链接错误，比静默不生效好排查。 */

/**
 * @brief 开机报数：维护 flash 里的计数器，判断本轮是否继续压测
 *
 * @return 1 = 继续压测（调用方应接着调 tuya_deepsleep_stress_arm_next()）
 *         0 = 停止压测，正常启动
 */
int  tuya_deepsleep_stress_boot(void);

/**
 * @brief 注册 RTC 唤醒源、落盘深睡参数并复位，为下一轮真深睡做准备
 *
 * 正常情况下不返回（复位后从 tuya_deepsleep_stress_boot() 重新开始）。
 * 只有落盘失败时才返回，调用方继续正常启动即可。
 */
void tuya_deepsleep_stress_arm_next(void);

#ifdef __cplusplus
}
#endif

#endif /* !__TUYA_DEEPSLEEP_STRESS_H__ */
