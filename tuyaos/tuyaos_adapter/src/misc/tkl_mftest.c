/**
 * @file tkl_mftest.c
 * @brief Platform manufacturing-test hooks for Tuya SDK (T5-aligned stub API)
 * @version 1.0
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */
#include "tkl_mftest.h"
#include "mt_adc.h"
#include "mt_mipi.h"
#include "tkl_system.h"
#include "tuya_error_code.h"
#include "tal_memory.h"
#include <common/bk_err.h>
#include <driver/gpio.h>
#include "gpio_driver.h"
#include <stdlib.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define MFTEST_MIPI_LOOPBACK_SEL_PIN    GPIO_27
#define PT_ADC_JSON_BUF_SIZE            160U

static uint32_t mf_test_idVendor = 0xFFFFFFFF;
static uint32_t mf_test_idProduct = 0xFFFFFFFF;
void tkl_mftest_usb_info(uint32_t vid, uint32_t pid)
{
    mf_test_idVendor = vid;
    mf_test_idProduct = pid;
}

bool tkl_mftest_is_usb_ready(void)
{
    int cnt = 0;
    do {
        if ((mf_test_idVendor != 0xFFFFFFFF) &&
           ((mf_test_idProduct != 0xFFFFFFFF))) {
            // got, next
            return true;
        }
        tkl_system_sleep(50);
    } while (cnt++ < 20);

    return false;
}

STATIC OPERATE_RET __pt_mcu_get_usb_info(USHORT_T cmd, UCHAR_T *data, UINT_T len, UCHAR_T **ret_data, USHORT_T *ret_len)
{
    CHAR_T *ret_buf = tkl_system_malloc(128);
    if (ret_buf == NULL) {
        bk_printf("get usb info, malloc failed\r\n");
        return OPRT_MALLOC_FAILED;
    }
    memset(ret_buf, 0, 128);

    mf_test_idVendor = 0xFFFFFFFF;
    mf_test_idProduct = 0xFFFFFFFF;

    // power on, 测架使能usb电源
    // TUYA_GPIO_BASE_CFG_T cfg;
    // cfg.mode = TUYA_GPIO_PULLUP;
    // cfg.direct = TUYA_GPIO_OUTPUT;
    // cfg.level = TUYA_GPIO_LEVEL_HIGH;
    // tkl_gpio_init(TUYA_GPIO_NUM_28, &cfg);
    // tkl_gpio_write(TUYA_GPIO_NUM_28, TUYA_GPIO_LEVEL_HIGH);

    // open usb
    bk_usb_open(0);

    // get value, wait max 500ms
    tkl_mftest_is_usb_ready();

    bk_printf("usb device: 0x%04x 0x%04x\r\n", mf_test_idVendor, mf_test_idProduct);

    if ((mf_test_idVendor != 0xFFFFFFFF) && (mf_test_idProduct != 0xFFFFFFFF)) {
        bk_printf("get usb info success\r\n");
        char *usb_succ_str = "{\"ret\":true}";
        memcpy(ret_buf, usb_succ_str, strlen(usb_succ_str));
    } else {
        bk_printf("get usb info failed\r\n");
        char *failed_str = "{\"ret\":false}";
        memcpy(ret_buf, failed_str, strlen(failed_str));
    }

    *ret_data = (UCHAR_T *)ret_buf;
    *ret_len = strlen(ret_buf);

    bk_usb_close();
    // tkl_gpio_write(TUYA_GPIO_NUM_28, TUYA_GPIO_LEVEL_LOW);
    tkl_system_sleep(100);
    return OPRT_OK;
}

/**
 * @brief Select MIPI DSI-to-CSI loopback lane pair on test socket via P27
 * @param[in] lane23 FALSE for DSI lane0/1, TRUE for DSI lane2/3
 * @return OPRT_OK on success
 * @note P27 drives TMUX1574 SEL on TS-SKT9-A1 socket board
 */
STATIC OPERATE_RET __pt_mipi_loopback_sel(BOOL_T lane23)
{
    gpio_id_t pin = MFTEST_MIPI_LOOPBACK_SEL_PIN;

    gpio_dev_unmap(pin);
    bk_gpio_disable_input(pin);
    bk_gpio_enable_output(pin);
    if (lane23 == FALSE) {
        bk_gpio_set_output_low(pin);
        bk_printf("mipi loopback sel: P27 low, DSI lane0/1 -> CSI\r\n");
    } else {
        bk_gpio_set_output_high(pin);
        bk_printf("mipi loopback sel: P27 high, DSI lane2/3 -> CSI\r\n");
    }
    tkl_system_sleep(1);
    return OPRT_OK;
}

/**
 * @brief Run MIPI loopback factory test for selected lane pair
 * @param[in] cmd TKL_MIPICN1_TEST_CMD or TKL_MIPICN2_TEST_CMD
 * @param[in] data unused
 * @param[in] len unused
 * @param[out] ret_data JSON reply buffer
 * @param[out] ret_len reply length
 * @return OPRT_OK on dispatch success
 */
STATIC OPERATE_RET __pt_mipi_test(USHORT_T cmd, UCHAR_T *data, UINT_T len, UCHAR_T **ret_data, USHORT_T *ret_len)
{
    OPERATE_RET ret;

    (VOID_T)data;
    (VOID_T)len;

    if (cmd == TKL_MIPICN1_TEST_CMD) {
        ret = __pt_mipi_loopback_sel(FALSE);
        if (ret != OPRT_OK) {
            return ret;
        }
        return mt_mipi_factory_test_reply(ret_data, ret_len);
    }
    if (cmd == TKL_MIPICN2_TEST_CMD) {
        ret = __pt_mipi_loopback_sel(TRUE);
        if (ret != OPRT_OK) {
            return ret;
        }
        return mt_mipi_factory_test_reply(ret_data, ret_len);
    }
    return OPRT_INVALID_PARM;
}

/**
 * @brief Parse ADC tolerance percent from host JSON testItem field
 * @param[in] data JSON payload, e.g. {"testItem":"ADC_10"}
 * @param[in] len payload length in bytes
 * @param[out] tolerance_pct parsed tolerance percent
 * @return OPRT_OK on success, OPRT_INVALID_PARM on parse failure
 * @note "ADC" uses MT_ADC_VOLT_TOLERANCE_PCT_DEFAULT; "ADC_10" means +/-10%
 */
STATIC OPERATE_RET __pt_adc_parse_tolerance(const UCHAR_T *data, UINT_T len, UINT32_T *tolerance_pct)
{
    CHAR_T json_buf[PT_ADC_JSON_BUF_SIZE];
    CHAR_T item[32];
    const CHAR_T *key = NULL;
    const CHAR_T *val_start = NULL;
    const CHAR_T *p = NULL;
    UINT_T copy_len = 0U;
    UINT32_T pct = 0U;
    CHAR_T *endptr = NULL;

    if (tolerance_pct == NULL) {
        return OPRT_INVALID_PARM;
    }
    *tolerance_pct = MT_ADC_VOLT_TOLERANCE_PCT_DEFAULT;

    if ((data == NULL) || (len == 0U)) {
        return OPRT_OK;
    }
    if (len >= PT_ADC_JSON_BUF_SIZE) {
        return OPRT_INVALID_PARM;
    }

    memcpy(json_buf, data, len);
    json_buf[len] = '\0';

    key = strstr(json_buf, "\"testItem\"");
    if (key == NULL) {
        return OPRT_INVALID_PARM;
    }

    p = strchr(key, ':');
    if (p == NULL) {
        return OPRT_INVALID_PARM;
    }
    p++;
    while ((*p == ' ') || (*p == '\t')) {
        p++;
    }
    if (*p != '"') {
        return OPRT_INVALID_PARM;
    }
    p++;
    val_start = p;
    while ((*p != '\0') && (*p != '"')) {
        p++;
    }
    if (*p != '"') {
        return OPRT_INVALID_PARM;
    }

    copy_len = (UINT_T)(p - val_start);
    if ((copy_len == 0U) || (copy_len >= sizeof(item))) {
        return OPRT_INVALID_PARM;
    }
    memcpy(item, val_start, copy_len);
    item[copy_len] = '\0';

    if (strcmp(item, "ADC") == 0) {
        return OPRT_OK;
    }
    if (strncmp(item, "ADC_", 4) != 0) {
        return OPRT_INVALID_PARM;
    }

    pct = (UINT32_T)strtoul(item + 4, &endptr, 10);
    if ((endptr == NULL) || (*endptr != '\0') || (pct == 0U) || (pct > 100U)) {
        return OPRT_INVALID_PARM;
    }
    *tolerance_pct = pct;
    return OPRT_OK;
}

/**
 * @brief Run ADC factory test with tolerance from host JSON
 * @param[in] cmd TKL_ADC_TEST_CMD
 * @param[in] data JSON payload with testItem field
 * @param[in] len payload length in bytes
 * @param[out] ret_data JSON reply buffer
 * @param[out] ret_len reply length
 * @return OPRT_OK on dispatch success
 */
STATIC OPERATE_RET __pt_adc_test(USHORT_T cmd, UCHAR_T *data, UINT_T len, UCHAR_T **ret_data, USHORT_T *ret_len)
{
    OPERATE_RET ret;
    UINT32_T tolerance_pct = MT_ADC_VOLT_TOLERANCE_PCT_DEFAULT;

    (VOID_T)cmd;

    ret = __pt_adc_parse_tolerance(data, len, &tolerance_pct);
    if (ret != OPRT_OK) {
        return ret;
    }
    bk_printf("adc factory test, tolerance=%u%%\r\n", tolerance_pct);
    return mt_adc_factory_test_reply(ret_data, ret_len, tolerance_pct);
}

/**
 * @brief Dispatch manufacturing test command (stub: link symbol for AP; extend for real audio/USB)
 * @param[in] cmd command from TKL_MFTEST_CMD_E
 * @param[in] in optional input payload
 * @param[in] inlen input length in bytes
 * @param[out] out optional heap buffer for JSON/text reply (caller frees with Free())
 * @param[out] outlen output length when @p out is set
 * @return OPRT_OK on success, OPRT_NOT_SUPPORTED if cmd unknown, OPRT_MALLOC_FAILED on alloc failure
 */
OPERATE_RET tkl_mftest_ctrl(USHORT_T cmd, UCHAR_T *in, UINT_T inlen, UCHAR_T **out, USHORT_T *outlen)
{
    (VOID_T)in;
    (VOID_T)inlen;

    if (out != NULL) {
        *out = NULL;
    }
    if (outlen != NULL) {
        *outlen = 0;
    }

    switch (cmd) {
    case TKL_AUDIO_TEST_START_CMD:
    case TKL_AUDIO_TEST_STOP_CMD:
        return OPRT_OK;

    case TKL_USB_INFO_TEST_CMD:
        return __pt_mcu_get_usb_info(cmd, in, inlen, out, outlen);

    case TKL_MIPICN1_TEST_CMD:
    case TKL_MIPICN2_TEST_CMD:
        return __pt_mipi_test(cmd, in, inlen, out, outlen);

    case TKL_ADC_TEST_CMD:
        return __pt_adc_test(cmd, in, inlen, out, outlen);

    default:
        return OPRT_NOT_SUPPORTED;
    }
}
