/**
 * @file mt_adc.c
 * @brief ADC voltage manufacturing test for P24(1V2) and P25(1V8) on TS-SKT9-A1 socket
 * @version 1.0
 * @date 2026-07-06
 * @copyright Copyright (c) Tuya Inc.
 */
#include "mt_adc.h"
#include "tkl_adc.h"
#include "tkl_pinmux.h"
#include "tkl_system.h"
#include "tuya_error_code.h"
#include <common/bk_err.h>
#include <modules/pm.h>
#include <os/os.h>
#include <os/mem.h>
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define MT_ADC_TAG                      "mt_adc"
#define MT_ADC_PORT                     TUYA_ADC_NUM_0
#define MT_ADC_PIN_P24                  24U
#define MT_ADC_PIN_P25                  25U
#define MT_ADC_CONV_CNT                 8U
#define MT_ADC_MAX_CONV_CNT             32U
#define MT_ADC_P24_EXPECT_MV            1200
#define MT_ADC_P25_EXPECT_MV            1800
#define MT_ADC_VOLT_TOLERANCE_PCT       10U
#define MT_ADC_REPLY_BUF_SIZE           160U

#define MT_ADC_LOGI(...)                bk_printf("[I][" MT_ADC_TAG "] " __VA_ARGS__)
#define MT_ADC_LOGE(...)                bk_printf("[E][" MT_ADC_TAG "] " __VA_ARGS__)

extern void bk_printf(const char *fmt, ...);

/* ---------------------------------------------------------------------------
 * Function implementations
 * --------------------------------------------------------------------------- */
/**
 * @brief Clamp conversion count to a safe range
 * @param[in] cnt requested sample count
 * @return sanitized sample count
 */
STATIC UINT32_T __mt_adc_sanitize_cnt(UINT32_T cnt)
{
    if (cnt == 0U) {
        return 1U;
    }
    if (cnt > MT_ADC_MAX_CONV_CNT) {
        return MT_ADC_MAX_CONV_CNT;
    }
    return cnt;
}

/**
 * @brief Map Tuya pin number to SARADC channel id
 * @param[in] pin pin number (24 or 25 on test socket)
 * @return channel id on success, -1 on invalid pin
 */
STATIC INT32_T __mt_adc_pin_to_channel(UINT32_T pin)
{
    INT32_T chan = tkl_io_pin_to_func((TUYA_PIN_NAME_E)pin, TUYA_IO_TYPE_ADC);

    if ((chan < 0) || (chan >= 16)) {
        return -1;
    }
    return chan;
}

/**
 * @brief Check whether measured voltage is within tolerance of expected rail
 * @param[in] measured_mv measured voltage in millivolts
 * @param[in] expect_mv expected rail voltage in millivolts
 * @return TRUE if within tolerance
 */
STATIC BOOL_T __mt_adc_voltage_in_range(INT32_T measured_mv, UINT32_T expect_mv)
{
    UINT32_T margin_mv;

    if (measured_mv < 0) {
        return FALSE;
    }
    margin_mv = (expect_mv * MT_ADC_VOLT_TOLERANCE_PCT) / 100U;
    if ((UINT32_T)measured_mv < (expect_mv - margin_mv)) {
        return FALSE;
    }
    if ((UINT32_T)measured_mv > (expect_mv + margin_mv)) {
        return FALSE;
    }
    return TRUE;
}

/**
 * @brief Enable or disable 1.2V / 1.8V AUX LDO rails for test socket ADC check
 * @param[in] enable TRUE to enable, FALSE to disable
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __mt_adc_ldo_enable(BOOL_T enable)
{
    pm_auxldo_ctrl_cfg_t auxldo_cfg = {0};
    int ldo_en = enable ? PM_AUXLDO_ENABLE : PM_AUXLDO_DISABLE;

    MT_ADC_LOGI("%s, 1p2/1p8 enable: %d\r\n", __func__, ldo_en);

    auxldo_cfg.ldo = AUXLDOS_SEL_1P8V;
    auxldo_cfg.out = PM_AUXLDO_1P8V_OUT_1P8V;
    auxldo_cfg.user = PM_AUXLDO_USER_CAMERA;
    auxldo_cfg.state = ldo_en;
    if (bk_pm_auxldo_ctrl_vote(&auxldo_cfg) != BK_OK) {
        MT_ADC_LOGE("1p8v ldo vote failed\r\n");
        return OPRT_COM_ERROR;
    }

    memset(&auxldo_cfg, 0, sizeof(auxldo_cfg));
    auxldo_cfg.ldo = AUXLDOS_SEL_1P2V;
    auxldo_cfg.out = PM_AUXLDO_1P2V_OUT_1P2V;
    auxldo_cfg.user = PM_AUXLDO_USER_CAMERA;
    auxldo_cfg.state = ldo_en;
    if (bk_pm_auxldo_ctrl_vote(&auxldo_cfg) != BK_OK) {
        MT_ADC_LOGE("1p2v ldo vote failed\r\n");
        return OPRT_COM_ERROR;
    }

    rtos_delay_milliseconds(1);
    return OPRT_OK;
}

/**
 * @brief Read average voltage on one ADC pin
 * @param[in] pin Tuya pin number
 * @param[out] volt_mv average voltage in millivolts
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __mt_adc_read_pin_voltage_mv(UINT32_T pin, INT32_T *volt_mv)
{
    TUYA_ADC_BASE_CFG_T cfg;
    INT32_T samples[MT_ADC_MAX_CONV_CNT];
    INT32_T chan;
    OPERATE_RET ret;
    UINT32_T cnt = __mt_adc_sanitize_cnt(MT_ADC_CONV_CNT);
    INT64_T sum = 0;
    UINT32_T i;

    if (volt_mv == NULL) {
        return OPRT_INVALID_PARM;
    }

    chan = __mt_adc_pin_to_channel(pin);
    if (chan < 0) {
        MT_ADC_LOGE("invalid adc pin: %u\r\n", pin);
        return OPRT_INVALID_PARM;
    }

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.ch_nums = 1U;
    cfg.type = TUYA_ADC_INNER_SAMPLE_VOL;
    cfg.width = 12U;
    cfg.mode = TUYA_ADC_CONTINUOUS;
    cfg.conv_cnt = (UINT16_T)cnt;
    cfg.ref_vol = tkl_adc_ref_voltage_get(MT_ADC_PORT);
    cfg.ch_list.data = (1UL << (UINT32_T)chan);

    ret = tkl_adc_init(MT_ADC_PORT, &cfg);
    if (ret != OPRT_OK) {
        MT_ADC_LOGE("adc init failed, pin=%u ret=%d\r\n", pin, ret);
        return ret;
    }

    os_memset(samples, 0, sizeof(samples));
    ret = tkl_adc_read_voltage(MT_ADC_PORT, samples, (UINT16_T)cnt);
    if (ret != OPRT_OK) {
        MT_ADC_LOGE("adc read voltage failed, pin=%u ret=%d\r\n", pin, ret);
        (VOID_T)tkl_adc_deinit(MT_ADC_PORT);
        return ret;
    }

    for (i = 0U; i < cnt; i++) {
        sum += samples[i];
    }
    *volt_mv = (INT32_T)(sum / (INT64_T)cnt);

    MT_ADC_LOGI("pin=%u ch=%d avg=%d mV\r\n", pin, chan, *volt_mv);
    (VOID_T)tkl_adc_deinit(MT_ADC_PORT);
    return OPRT_OK;
}

/**
 * @brief Read P24/P25 and verify against expected rails on test socket
 * @param[out] p24_mv average P24 voltage in millivolts
 * @param[out] p25_mv average P25 voltage in millivolts
 * @return OPRT_OK on pass, OPRT_COM_ERROR on out-of-range, other errors on ADC failure
 */
STATIC OPERATE_RET __mt_adc_run_test(INT32_T *p24_mv, INT32_T *p25_mv)
{
    OPERATE_RET ret = OPRT_OK;
    BOOL_T p24_ok;
    BOOL_T p25_ok;

    if ((p24_mv == NULL) || (p25_mv == NULL)) {
        return OPRT_INVALID_PARM;
    }

    ret = __mt_adc_ldo_enable(TRUE);
    if (ret != OPRT_OK) {
        return ret;
    }
    tkl_system_sleep(1000);

    ret = __mt_adc_read_pin_voltage_mv(MT_ADC_PIN_P24, p24_mv);
    if (ret != OPRT_OK) {
        goto cleanup;
    }

    ret = __mt_adc_read_pin_voltage_mv(MT_ADC_PIN_P25, p25_mv);
    if (ret != OPRT_OK) {
        goto cleanup;
    }

    p24_ok = __mt_adc_voltage_in_range(*p24_mv, MT_ADC_P24_EXPECT_MV);
    p25_ok = __mt_adc_voltage_in_range(*p25_mv, MT_ADC_P25_EXPECT_MV);

    if ((p24_ok == FALSE) || (p25_ok == FALSE)) {
        MT_ADC_LOGE("voltage check FAIL, P24=%d mV(expect %u), P25=%d mV(expect %u), tol=%u%%\r\n",
                    *p24_mv, MT_ADC_P24_EXPECT_MV, *p25_mv, MT_ADC_P25_EXPECT_MV,
                    MT_ADC_VOLT_TOLERANCE_PCT);
        ret = OPRT_COM_ERROR;
        goto cleanup;
    }

    MT_ADC_LOGI("voltage check PASS, P24=%d mV, P25=%d mV\r\n", *p24_mv, *p25_mv);

cleanup:
    (VOID_T)__mt_adc_ldo_enable(FALSE);
    return ret;
}

/**
 * @brief Run ADC factory test on P24(1V2) and P25(1V8)
 * @return OPRT_OK on pass, OPRT_COM_ERROR on test failure
 */
OPERATE_RET mt_adc_factory_test(VOID_T)
{
    INT32_T p24_mv = 0;
    INT32_T p25_mv = 0;

    return __mt_adc_run_test(&p24_mv, &p25_mv);
}

/**
 * @brief Run ADC factory test and fill JSON reply buffer for mftest framework
 * @param[out] ret_data heap buffer with JSON result (caller frees)
 * @param[out] ret_len length of @p ret_data
 * @return OPRT_OK on dispatch success, OPRT_MALLOC_FAILED on alloc failure
 */
OPERATE_RET mt_adc_factory_test_reply(UCHAR_T **ret_data, USHORT_T *ret_len)
{
    CHAR_T *ret_buf = NULL;
    INT32_T p24_mv = -1;
    INT32_T p25_mv = -1;
    OPERATE_RET test_ret;
    INT32_T written;

    if ((ret_data == NULL) || (ret_len == NULL)) {
        return OPRT_INVALID_PARM;
    }

    *ret_data = NULL;
    *ret_len = 0;

    ret_buf = (CHAR_T *)tkl_system_malloc(MT_ADC_REPLY_BUF_SIZE);
    if (ret_buf == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    os_memset(ret_buf, 0, MT_ADC_REPLY_BUF_SIZE);

    test_ret = __mt_adc_run_test(&p24_mv, &p25_mv);
    if (test_ret == OPRT_OK) {
        written = snprintf(ret_buf, MT_ADC_REPLY_BUF_SIZE,
                           "{\"ret\":true,\"p24_mv\":%d,\"p25_mv\":%d}",
                           p24_mv, p25_mv);
    } else {
        written = snprintf(ret_buf, MT_ADC_REPLY_BUF_SIZE,
                           "{\"ret\":false,\"p24_mv\":%d,\"p25_mv\":%d}",
                           p24_mv, p25_mv);
    }

    if ((written < 0) || ((UINT32_T)written >= MT_ADC_REPLY_BUF_SIZE)) {
        tkl_system_free(ret_buf);
        return OPRT_COM_ERROR;
    }

    *ret_data = (UCHAR_T *)ret_buf;
    *ret_len = (USHORT_T)strlen(ret_buf);
    return OPRT_OK;
}
