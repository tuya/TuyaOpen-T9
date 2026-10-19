/**
 * @file tkl_sleep.c
 * @brief T9 (BK7259) sleep adapter
 * @version 0.1
 * @date 2026-10-15
 * @copyright Copyright (c) Tuya Inc.
 */
#include <string.h>
#include <stddef.h>
#include "tuya_cloud_types.h"
#include "tkl_sleep.h"
#include "tkl_flash.h"
#include "tkl_system.h"
#include "tkl_wakeup.h"

#include <modules/pm.h>
#include <driver/pwr_clk.h>

extern OPERATE_RET tkl_wakeup_source_get(TUYA_WAKEUP_SOURCE_BASE_CFG_T *param,
                                         UINT32_T num, UINT32_T *status);

extern void bk_printf(const char *fmt, ...);

#define TKL_DS_FLUSH_DELAY_MS   2000

#define TKL_DS_ENTER_SLEEP_YIELD_MS     100

#define TKL_DS_SUM_SPAN  offsetof(TKL_DS_PARAM_T, sum)

/**
 * @brief 计算校验和（8-bit 累加，与 T5 语义一致）
 *
 * 逐字节累加到 UINT32_T。结构体最大 256 字节、每字节最大 255，
 * 累加上限远小于 UINT32_T 的量程，不会溢出。
 */
static UINT32_T __tkl_ds_checksum(CONST TKL_DS_PARAM_T *ds)
{
    CONST UINT8_T *raw = (CONST UINT8_T *)ds;
    UINT32_T       sum = 0;
    UINT32_T       i;

    for (i = 0; i < TKL_DS_SUM_SPAN; i++) {
        sum += raw[i];
    }

    return sum;
}

/**
 * @brief 把唤醒源表打包成深睡参数
 *
 * 会先清空 ds，拷贝至多 DS_MAX_CFG_ITEM 项，未使用的槽位填 0xFF（与唤醒源
 * RAM 表的空槽哨兵一致），最后填 magic / entry_flag / 校验和。
 * cfg_num 超过 DS_MAX_CFG_ITEM 时截断。
 */
static VOID_T __tkl_ds_pack(TKL_DS_PARAM_T *ds,
                            CONST TUYA_WAKEUP_SOURCE_BASE_CFG_T *cfg,
                            UINT32_T cfg_num)
{
    UINT32_T i;

    if (ds == NULL) {
        return;
    }

    memset(ds, 0, sizeof(TKL_DS_PARAM_T));

    if ((cfg != NULL) && (cfg_num > DS_MAX_CFG_ITEM)) {
        cfg_num = DS_MAX_CFG_ITEM;
    }

    if ((cfg != NULL) && (cfg_num > 0)) {
        for (i = 0; i < cfg_num; i++) {
            ds->cfg[i] = cfg[i];
        }
    } else {
        cfg_num = 0;
    }

    /* 未使用的槽位填 0xFF，与唤醒源 RAM 表的空槽哨兵一致。
     *
     * 不能留 0：TUYA_WAKEUP_SOURCE_GPIO 的值就是 0，留 0 会让消费侧
     * （tkl_sleep_param_check_and_set 的遍历）把空槽误判成 gpio_num=0
     * 的 GPIO 唤醒源。0xFF 既不等于 GPIO(0) 也不等于 RTC(2)，在
     * -fshort-enums 与默认 enum 宽度下都安全。 */
    for (i = cfg_num; i < DS_MAX_CFG_ITEM; i++) {
        memset(&ds->cfg[i], 0xFF, sizeof(ds->cfg[i]));
    }

    ds->magic      = DEEPSLEEP_MAGIC;
    ds->entry_flag = DS_ENTRY_FLAG;
    ds->sum        = 0;
    ds->sum        = __tkl_ds_checksum(ds);
}

/**
 * @brief 校验深睡参数是否有效
 *
 * magic 与 entry_flag 均匹配、且校验和正确时判为有效。全 0xFF（flash
 * 擦除态）与全 0 均判为无效。
 *
 * 注：T5 只比 magic 与 entry_flag，不校验 sum（它算了但没用）。这里多一道
 * 校验和，用于挡住参数被写坏或结构体布局变更后残留的陈旧参数。
 *
 * @return 0 表示有效，-1 表示无效
 */
static INT_T __tkl_ds_verify(CONST TKL_DS_PARAM_T *ds)
{
    if (ds == NULL) {
        return -1;
    }

    if (ds->magic != DEEPSLEEP_MAGIC) {
        return -1;
    }

    if (ds->entry_flag != DS_ENTRY_FLAG) {
        return -1;
    }

    if (ds->sum != __tkl_ds_checksum(ds)) {
        return -1;
    }

    return 0;
}

/**
 * @brief 打印深睡参数（诊断用）
 */
static VOID_T __tkl_sleep_param_dump(CONST CHAR_T *tag, CONST TKL_DS_PARAM_T *ds)
{
    INT_T i;

    if (tag != NULL) {
        bk_printf("=== %s\r\n", tag);
    }

    bk_printf("magic: 0x%x, entry flag: 0x%x, sum: 0x%x\r\n",
              (unsigned)ds->magic, (unsigned)ds->entry_flag, (unsigned)ds->sum);

    for (i = 0; i < DS_MAX_CFG_ITEM; i++) {
        if (ds->cfg[i].source == TUYA_WAKEUP_SOURCE_GPIO) {
            bk_printf("  gpio: %d, level: %d\r\n",
                      (int)ds->cfg[i].wakeup_para.gpio_param.gpio_num,
                      (int)ds->cfg[i].wakeup_para.gpio_param.level);
        } else if (ds->cfg[i].source == TUYA_WAKEUP_SOURCE_RTC) {
            bk_printf("  rtc: %d ms\r\n",
                      (int)ds->cfg[i].wakeup_para.rtc_param.ms);
        }
    }
}

/**
 * @brief 把当前唤醒源表落到 flash，然后复位
 *
 * 复位后由 tkl_sleep_param_check_and_set() 在开机早期接手进睡。
 * 任何一步失败都返回错误且**不复位** —— 否则调用方会以为睡成功了。
 */
static OPERATE_RET __tkl_sleep_param_save(VOID_T)
{
    TKL_DS_PARAM_T                 ds;
    TKL_DS_PARAM_T                 rd;
    TUYA_WAKEUP_SOURCE_BASE_CFG_T  cfg[DS_MAX_CFG_ITEM];
    UINT32_T                       status = 0;
    OPERATE_RET                    ret;

    memset(cfg, 0, sizeof(cfg));

    ret = tkl_wakeup_source_get(cfg, DS_MAX_CFG_ITEM, &status);
    if (ret == OPRT_OK) {
        __tkl_ds_pack(&ds, cfg, DS_MAX_CFG_ITEM);
    } else {
        bk_printf("no wakeup source registered (%d), sleep with reset only\r\n", ret);
        __tkl_ds_pack(&ds, NULL, 0);
    }

    ret = tkl_flash_erase(DEEPSLEEP_PARAMETER_ADDRESS, 4096);
    if (ret != OPRT_OK) {
        bk_printf("erase deepsleep param failed: %d\r\n", ret);
        return ret;
    }

    ret = tkl_flash_write(DEEPSLEEP_PARAMETER_ADDRESS, (CONST UCHAR_T *)&ds, sizeof(ds));
    if (ret != OPRT_OK) {
        bk_printf("write deepsleep param failed: %d\r\n", ret);
        return ret;
    }

    /* 回读确认真的落盘了再复位 */
    memset(&rd, 0, sizeof(rd));
    tkl_flash_read(DEEPSLEEP_PARAMETER_ADDRESS, (UCHAR_T *)&rd, sizeof(rd));
    if (__tkl_ds_verify(&rd) != 0) {
        bk_printf("deepsleep param readback mismatch, abort\r\n");
        return OPRT_COM_ERROR;
    }

    tkl_system_sleep(TKL_DS_FLUSH_DELAY_MS);

    __tkl_sleep_param_dump("save config && reboot", &rd);

    tkl_system_reset();

    return OPRT_OK;
}

/**
 * @brief 开机早期检查深睡标志，命中则配置唤醒源并进睡
 *
 * @return 0 表示已进入深睡流程（正常情况下不会返回），-1 表示不进睡
 */
INT_T tkl_sleep_param_check_and_set(VOID_T)
{
    TKL_DS_PARAM_T ds;
    TKL_DS_PARAM_T rd;
    OPERATE_RET    ret;
    INT_T          i;

    memset(&ds, 0, sizeof(ds));

    tkl_flash_read(DEEPSLEEP_PARAMETER_ADDRESS, (UCHAR_T *)&ds, sizeof(ds));

    if (__tkl_ds_verify(&ds) != 0) {
        bk_printf("no pending deepsleep param: magic 0x%x, entry 0x%x\r\n",
                  (unsigned)ds.magic, (unsigned)ds.entry_flag);
        return -1;
    }

    /* 先把标志擦掉并回读确认，再做任何有副作用的事。
     *
     * 注意 T9 的 tkl_flash_erase() 吞掉了 bk_flash_erase_sector() 的返回值、
     * 永远返回 OPRT_OK（见 src/driver/tkl_flash.c），所以**回读是唯一可靠的
     * 判据**。若擦除没生效却仍然进睡：唤醒→复位→又读到有效标志→再睡……
     * 应用永远起不来，且收不到 OTA，只能重新烧录。此处降级为"这次不睡"。 */
    ret = tkl_flash_erase(DEEPSLEEP_PARAMETER_ADDRESS, 4096);
    if (ret != OPRT_OK) {
        bk_printf("erase deepsleep flag failed: %d, abort deepsleep\r\n", ret);
        return -1;
    }

    memset(&rd, 0, sizeof(rd));
    tkl_flash_read(DEEPSLEEP_PARAMETER_ADDRESS, (UCHAR_T *)&rd, sizeof(rd));
    if (__tkl_ds_verify(&rd) == 0) {
        bk_printf("deepsleep flag still valid after erase, abort deepsleep\r\n");
        return -1;
    }

    /* 重新 arm 唤醒源。__tkl_ds_pack() 会把未使用的槽位填 0xFF，
     * 既不等于 TUYA_WAKEUP_SOURCE_GPIO(0) 也不等于 RTC(2)，故被正确跳过。 */
    for (i = 0; i < DS_MAX_CFG_ITEM; i++) {
        if ((ds.cfg[i].source == TUYA_WAKEUP_SOURCE_GPIO) ||
            (ds.cfg[i].source == TUYA_WAKEUP_SOURCE_RTC)) {
            ret = tkl_wakeup_source_set(&ds.cfg[i]);
            if (ret != OPRT_OK) {
                bk_printf("re-arm wakeup source slot %d failed: %d\r\n", i, ret);
            }
        }
    }

    __tkl_sleep_param_dump("entry deepsleep", &ds);

    bk_pm_ap_sleep_mode_set(PM_MODE_DEEP_SLEEP);
    /* T9 专有：AP 上电是多模块投票，不撤票进不去深睡 */
    bk_pm_module_vote_boot_ap_ctrl(PM_BOOT_AP_MODULE_NAME_APP,
                                   PM_POWER_MODULE_STATE_OFF);

    tkl_system_sleep(TKL_DS_ENTER_SLEEP_YIELD_MS);

    bk_printf("deepsleep did not engage within %d ms, continue booting\r\n",
              TKL_DS_ENTER_SLEEP_YIELD_MS);

    return 0;
}

/**
 * @brief Set the low power mode of CPU
 *
 * @param[in] enable: enable switch
 * @param[in] mode:   cpu sleep mode
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_cpu_sleep_mode_set(BOOL_T enable, TUYA_CPU_SLEEP_MODE_E mode)
{
    bk_printf("cpu sleep set enable: %d, mode: %d\r\n", (int)enable, (int)mode);

    if (mode == TUYA_CPU_DEEP_SLEEP) {
        if (!enable) {
            /* 深睡是"落盘 + 复位"，没有撤销路径 */
            return OPRT_OK;
        }

        bk_printf("prepare to deepsleep\r\n");
        return __tkl_sleep_param_save();
    }

    if (mode == TUYA_CPU_SLEEP) {
        /* 轻睡眠本次不实现 */
        return OPRT_NOT_SUPPORTED;
    }

    return OPRT_OS_ADAPTER_CPU_LPMODE_SET_FAILED;
}

/**
 * @brief sleep callback register
 *
 * @param[in] sleep_cb: sleep callback
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_cpu_sleep_callback_register(TUYA_SLEEP_CB_T *sleep_cb)
{
    (void)sleep_cb;
    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief allow to sleep
 */
VOID_T tkl_cpu_allow_sleep(VOID_T)
{
    return;
}

/**
 * @brief force wakeup
 */
VOID_T tkl_cpu_force_wakeup(VOID_T)
{
    return;
}

/**
 * @brief 设置深睡时长
 */
OPERATE_RET tkl_cpu_sleep_time_set(CONST UINT32_T sleep_ms)
{
    (void)sleep_ms;
    return OPRT_NOT_SUPPORTED;
}
