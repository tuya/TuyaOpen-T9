/**
 * @file tkl_wakeup.c
 * @brief T9 (BK7259) wakeup source adapter
 * @version 0.1
 * @date 2026-10-15
 * @copyright Copyright (c) Tuya Inc.
 */
#include <string.h>
#include "tuya_cloud_types.h"
#include "tkl_wakeup.h"
#include "tkl_memory.h"

#include <modules/pm.h>
#include <driver/gpio.h>
#include <driver/pwr_clk.h>
#include <driver/pm_ap_core.h>

#define WAKEUP_SOURCE_CNT       6
#define WAKEUP_SOURCE_EMPTY     0xFF

static TUYA_WAKEUP_SOURCE_BASE_CFG_T *s_wakeup_source = NULL;
static UINT32_T                       s_wakeup_source_init_status = 0;

extern void bk_printf(const char *fmt, ...);

/**
 * @brief 懒分配唤醒源表，空槽填 0xFF
 */
static VOID_T __tkl_wakeup_table_alloc(VOID_T)
{
    if (s_wakeup_source != NULL) {
        return;
    }

    s_wakeup_source = (TUYA_WAKEUP_SOURCE_BASE_CFG_T *)tkl_system_malloc(
            WAKEUP_SOURCE_CNT * sizeof(TUYA_WAKEUP_SOURCE_BASE_CFG_T));
    if (s_wakeup_source == NULL) {
        bk_printf("wakeup source malloc failed\r\n");
        return;
    }

    memset(s_wakeup_source, WAKEUP_SOURCE_EMPTY,
           WAKEUP_SOURCE_CNT * sizeof(TUYA_WAKEUP_SOURCE_BASE_CFG_T));
}

/**
 * @brief 存入一项唤醒源
 *
 * RTC 按 source 去重，GPIO 按 gpio_num 去重；无重复则找空槽。
 *
 * @return 槽位下标，失败返回 -1
 */
static INT_T __tkl_wakeup_node_save(CONST TUYA_WAKEUP_SOURCE_BASE_CFG_T *param)
{
    INT_T i;

    for (i = 0; i < WAKEUP_SOURCE_CNT; i++) {
        if ((param->source == TUYA_WAKEUP_SOURCE_RTC) &&
            (s_wakeup_source[i].source == TUYA_WAKEUP_SOURCE_RTC)) {
            memcpy(&s_wakeup_source[i], param, sizeof(TUYA_WAKEUP_SOURCE_BASE_CFG_T));
            return i;
        }
        if ((param->source == TUYA_WAKEUP_SOURCE_GPIO) &&
            (s_wakeup_source[i].source == TUYA_WAKEUP_SOURCE_GPIO) &&
            (param->wakeup_para.gpio_param.gpio_num ==
             s_wakeup_source[i].wakeup_para.gpio_param.gpio_num)) {
            memcpy(&s_wakeup_source[i], param, sizeof(TUYA_WAKEUP_SOURCE_BASE_CFG_T));
            return i;
        }
    }

    for (i = 0; i < WAKEUP_SOURCE_CNT; i++) {
        if (s_wakeup_source[i].source == WAKEUP_SOURCE_EMPTY) {
            memcpy(&s_wakeup_source[i], param, sizeof(TUYA_WAKEUP_SOURCE_BASE_CFG_T));
            return i;
        }
    }

    bk_printf("no free wakeup slot\r\n");
    return -1;
}

/**
 * @brief 把 GPIO 唤醒源 arm 到 CP 侧
 *
 * T9 为 AP+CP 双核：深睡期间由 CP 捕获 GPIO 边沿并把 AP 唤回。
 * 必须显式传 PM_MODE_DEEP_SLEEP —— 不能改用 bk_gpio_set_wakeup()，
 * 那个封装内部把睡眠模式硬编码成了 PM_MODE_LOW_VOLTAGE。
 */
static OPERATE_RET __gpio_wakeup_source_set(TUYA_GPIO_NUM_E gpio_num,
                                            TUYA_GPIO_WAKE_TYPE_E level)
{
    gpio_int_type_t         int_type;
    pm_gpio_wakeup_config_t cfg;
    bk_err_t                err;

    switch (level) {
        case TUYA_GPIO_WAKEUP_LOW:
            int_type = GPIO_INT_TYPE_LOW_LEVEL;
            break;
        case TUYA_GPIO_WAKEUP_HIGH:
            int_type = GPIO_INT_TYPE_HIGH_LEVEL;
            break;
        case TUYA_GPIO_WAKEUP_FALL:
            int_type = GPIO_INT_TYPE_FALLING_EDGE;
            break;
        case TUYA_GPIO_WAKEUP_RISE:
        default:
            int_type = GPIO_INT_TYPE_RISING_EDGE;
            break;
    }

    err = bk_gpio_register_wakeup_source((gpio_id_t)gpio_num, int_type);
    if (err != BK_OK) {
        bk_printf("register gpio wakeup source %d failed: %d\r\n",
                  (int)gpio_num, (int)err);
        return OPRT_COM_ERROR;
    }

    cfg.gpio_id  = (uint16_t)gpio_num;
    cfg.int_type = (uint16_t)int_type;
    err = bk_pm_ap_gpio_wakeup_source_config(PM_MODE_DEEP_SLEEP,
            PM_WAKEUP_SOURCE_INT_GPIO, &cfg);
    if (err != BK_OK) {
        /* CP 的动态唤醒表只有 4 个槽（CONFIG_GPIO_DYNAMIC_WAKEUP_SOURCE_MAX_CNT），
         * 第 5 个不同 GPIO 会在这里被 CP 拒绝。必须上抛，不能静默当成功。 */
        bk_printf("arm gpio wakeup source %d on cp failed: %d\r\n",
                  (int)gpio_num, (int)err);
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

/**
 * @brief 把 RTC 唤醒源 arm 到深睡
 *
 * 只支持 PM_MODE_DEEP_SLEEP。
 */
static OPERATE_RET __rtc_wakeup_source_set(UINT32_T ms)
{
    pm_ap_rtc_info_t info;
    bk_err_t         err;

    memset(&info, 0, sizeof(info));
    info.period_tick = ms;
    info.period_cnt  = 1;
    info.callback    = NULL;
    info.param_p     = NULL;

    err = bk_pm_ap_rtc_regsiter_wakeup(PM_MODE_DEEP_SLEEP, &info);
    if (err != BK_OK) {
        bk_printf("arm rtc wakeup source %d ms failed: %d\r\n", (int)ms, (int)err);
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

/**
 * @brief 把 RAM 表里所有唤醒源 arm 一遍
 *
 * 逐项 arm 并记录首个错误：单项失败不阻断其余项，但错误必须上抛给调用方。
 *
 * @return OPRT_OK 表示全部成功，否则为首个失败的返回码
 */
static OPERATE_RET __tkl_wakeup_arm_all(VOID_T)
{
    INT_T       i;
    OPERATE_RET first_err = OPRT_OK;

    if (s_wakeup_source == NULL) {
        return OPRT_COM_ERROR;
    }

    for (i = 0; i < WAKEUP_SOURCE_CNT; i++) {
        OPERATE_RET ret = OPRT_OK;

        if (s_wakeup_source[i].source == TUYA_WAKEUP_SOURCE_GPIO) {
            bk_printf("arm gpio wakeup source %d\r\n",
                    (int)s_wakeup_source[i].wakeup_para.gpio_param.gpio_num);
            ret = __gpio_wakeup_source_set(
                    s_wakeup_source[i].wakeup_para.gpio_param.gpio_num,
                    s_wakeup_source[i].wakeup_para.gpio_param.level);
        } else if (s_wakeup_source[i].source == TUYA_WAKEUP_SOURCE_RTC) {
            bk_printf("arm rtc wakeup source %d ms\r\n",
                    (int)s_wakeup_source[i].wakeup_para.rtc_param.ms);
            ret = __rtc_wakeup_source_set(
                    s_wakeup_source[i].wakeup_para.rtc_param.ms);
        }

        if ((ret != OPRT_OK) && (first_err == OPRT_OK)) {
            first_err = ret;
        }
    }

    return first_err;
}

/**
 * @brief wake up source set
 *
 * @param[in] param: wake up source set
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_wakeup_source_set(CONST TUYA_WAKEUP_SOURCE_BASE_CFG_T *param)
{
    if (param == NULL) {
        return OPRT_INVALID_PARM;
    }

    if ((param->source != TUYA_WAKEUP_SOURCE_GPIO) &&
        (param->source != TUYA_WAKEUP_SOURCE_RTC)) {
        bk_printf("unsupport wakeup source: %d\r\n", (int)param->source);
        return OPRT_INVALID_PARM;
    }

    __tkl_wakeup_table_alloc();
    if (s_wakeup_source == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    if (__tkl_wakeup_node_save(param) < 0) {
        return OPRT_COM_ERROR;
    }

    return __tkl_wakeup_arm_all();
}

/**
 * @brief 取回唤醒源表（非 TKL 契约接口，仅供 tkl_sleep.c 使用）
 *
 * @param[out] param:  调用方缓冲，容量至少 num 项
 * @param[in]  num:    请求项数，不得超过 WAKEUP_SOURCE_CNT
 * @param[out] status: 初始化状态位图
 *
 * @return OPRT_OK on success. Others on error
 */
OPERATE_RET tkl_wakeup_source_get(TUYA_WAKEUP_SOURCE_BASE_CFG_T *param,
                                  UINT32_T num, UINT32_T *status)
{
    if ((param == NULL) || (status == NULL) || (num > WAKEUP_SOURCE_CNT)) {
        bk_printf("wakeup get: invalid parm\r\n");
        return OPRT_INVALID_PARM;
    }

    if (s_wakeup_source == NULL) {
        bk_printf("wakeup source not set\r\n");
        return OPRT_NOT_FOUND;
    }

    memcpy(param, s_wakeup_source, num * sizeof(TUYA_WAKEUP_SOURCE_BASE_CFG_T));
    *status = s_wakeup_source_init_status;

    return OPRT_OK;
}

/**
 * @brief wake up source clear
 *
 * @param[in] param: wake up source clear
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_wakeup_source_clear(CONST TUYA_WAKEUP_SOURCE_BASE_CFG_T *param)
{
    INT_T i;
    INT_T j;

    if (param == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_wakeup_source == NULL) {
        return OPRT_COM_ERROR;
    }

    for (i = 0; i < WAKEUP_SOURCE_CNT; i++) {
        BOOL_T matched = FALSE;

        if ((s_wakeup_source[i].source == TUYA_WAKEUP_SOURCE_RTC) &&
            (param->source == TUYA_WAKEUP_SOURCE_RTC)) {
            matched = TRUE;
        } else if ((s_wakeup_source[i].source == TUYA_WAKEUP_SOURCE_GPIO) &&
                   (param->source == TUYA_WAKEUP_SOURCE_GPIO) &&
                   (s_wakeup_source[i].wakeup_para.gpio_param.gpio_num ==
                    param->wakeup_para.gpio_param.gpio_num)) {
            matched = TRUE;
        }

        if (!matched) {
            continue;
        }

        /* 删除命中项：后继项整体左移一格，再清空最后一格。
         * T5 只做一次 [i+1]→[i] 拷贝，会复制出副本、丢掉末项、且清不干净。 */
        for (j = i; j + 1 < WAKEUP_SOURCE_CNT; j++) {
            memcpy(&s_wakeup_source[j], &s_wakeup_source[j + 1],
                   sizeof(TUYA_WAKEUP_SOURCE_BASE_CFG_T));
        }
        memset(&s_wakeup_source[WAKEUP_SOURCE_CNT - 1], WAKEUP_SOURCE_EMPTY,
               sizeof(TUYA_WAKEUP_SOURCE_BASE_CFG_T));
    }

    return OPRT_OK;
}
