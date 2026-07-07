/**
 * @file tkl_mftest.c
 * @brief Platform manufacturing-test hooks for Tuya SDK (T5-aligned stub API)
 * @version 1.0
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */
#include "tkl_mftest.h"
#include "mt_mipi.h"
#include "tkl_system.h"
#include "tuya_error_code.h"
#include "tal_memory.h"
#include <common/bk_err.h>
#include <driver/gpio.h>
#include "gpio_driver.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define MFTEST_MIPI_LOOPBACK_SEL_PIN    GPIO_27

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
        return mt_adc_factory_test_reply(out, outlen);

    default:
        return OPRT_NOT_SUPPORTED;
    }
}
