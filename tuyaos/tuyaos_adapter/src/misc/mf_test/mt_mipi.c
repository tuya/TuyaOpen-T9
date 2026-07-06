/**
 * @file mt_mipi.c
 * @brief MIPI DSI display + CSI BIST manufacturing test (adapted from mipi_ft_test.c)
 * @version 1.0
 * @date 2026-07-06
 * @copyright Copyright (c) Tuya Inc.
 */
#include "mt_mipi.h"
#include "tuya_error_code.h"
#include "tkl_system.h"
#include <avdk_check.h>
#include <avdk_error.h>
#include <common/avdk_pixel_types.h>
#include <common/bk_err.h>
#include <components/bk_display.h>
#include <components/bk_display_bus.h>
#include <components/bk_frame_buffer.h>
#include <components/bk_lcd_panel.h>
#include <driver/gpio.h>
#include <lcd/lcd_mipi_hx8399c_1080x1920.h>
#include <modules/pm.h>
#include <os/mem.h>
#include <os/os.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define MT_MIPI_TAG                     "mt_mipi"
#define MT_MIPI_PANEL_RESET_PIN         GPIO_60
#define MT_MIPI_CSI_BIST_PASS           3U
#define MT_MIPI_CSI_BIST_RETRY_MAX      2000
#define MT_MIPI_CSI_CLK_EN_REG          0x48000028U
#define MT_MIPI_CSI_CLK_EN_MASK         0x1200U
#define MT_MIPI_BASEADDR_CSI            0x4C050000U
#define MT_MIPI_BASEADDR_CSI_EXT        0x4C058000U
#define MT_MIPI_REPLY_BUF_SIZE          128U
#define MT_MIPI_BPP_BGR888              3U

#define MT_MIPI_LOGI(...)               BK_LOGI(MT_MIPI_TAG, ##__VA_ARGS__)
#define MT_MIPI_LOGE(...)               BK_LOGE(MT_MIPI_TAG, ##__VA_ARGS__)

/* ---------------------------------------------------------------------------
 * Type definitions
 * --------------------------------------------------------------------------- */
typedef struct {
    bk_display_ctlr_handle_t dpu_ctlr_handle;
    bk_display_bus_handle_t dis_bus_handle;
    bk_avdk_lcd_panel_handle_t panel_handle;
} MT_MIPI_DISP_CTX_T;

/* ---------------------------------------------------------------------------
 * File scope variables
 * --------------------------------------------------------------------------- */
STATIC MT_MIPI_DISP_CTX_T s_mipi_disp_ctx;
STATIC BOOL_T s_mipi_fb_inited = FALSE;

STATIC CONST UINT32_T s_prbs7_data[127] = {
    0x40404040U, 0x30303030U, 0x14141414U, 0x4F4F4F4FU, 0x34343434U, 0x57575757U, 0xBEBEBEBEU, 0x70707070U,
    0x24242424U, 0x5B5B5B5BU, 0x7B7B7B7BU, 0x63636363U, 0xE9E9E9E9U, 0xCECECECEU, 0x54545454U, 0x7F7F7F7FU,
    0x20202020U, 0x18181818U, 0x8A8A8A8AU, 0x27272727U, 0x9A9A9A9AU, 0x2B2B2B2BU, 0x5F5F5F5FU, 0x38383838U,
    0x92929292U, 0xADADADADU, 0xBDBDBDBDU, 0xB1B1B1B1U, 0x74747474U, 0x67676767U, 0xAAAAAAAAU, 0x3F3F3F3FU,
    0x10101010U, 0x0C0C0C0CU, 0xC5C5C5C5U, 0x13131313U, 0xCDCDCDCDU, 0x95959595U, 0x2F2F2F2FU, 0x1C1C1C1CU,
    0xC9C9C9C9U, 0xD6D6D6D6U, 0xDEDEDEDEU, 0x58585858U, 0xBABABABAU, 0x33333333U, 0xD5D5D5D5U, 0x1F1F1F1FU,
    0x08080808U, 0x86868686U, 0xE2E2E2E2U, 0x89898989U, 0xE6E6E6E6U, 0xCACACACAU, 0x17171717U, 0x8E8E8E8EU,
    0x64646464U, 0x6B6B6B6BU, 0x6F6F6F6FU, 0x2C2C2C2CU, 0xDDDDDDDDU, 0x99999999U, 0xEAEAEAEAU, 0x0F0F0F0FU,
    0x04040404U, 0x43434343U, 0xF1F1F1F1U, 0x44444444U, 0x73737373U, 0xE5E5E5E5U, 0x0B0B0B0BU, 0x47474747U,
    0xB2B2B2B2U, 0xB5B5B5B5U, 0x37373737U, 0x96969696U, 0xEEEEEEEEU, 0x4C4C4C4CU, 0xF5F5F5F5U, 0x07070707U,
    0x82828282U, 0xA1A1A1A1U, 0x78787878U, 0xA2A2A2A2U, 0xB9B9B9B9U, 0xF2F2F2F2U, 0x85858585U, 0x23232323U,
    0xD9D9D9D9U, 0xDADADADAU, 0x1B1B1B1BU, 0x4B4B4B4BU, 0x77777777U, 0xA6A6A6A6U, 0xFAFAFAFAU, 0x03030303U,
    0xC1C1C1C1U, 0x50505050U, 0x3C3C3C3CU, 0xD1D1D1D1U, 0x5C5C5C5CU, 0xF9F9F9F9U, 0xC2C2C2C2U, 0x91919191U,
    0x6C6C6C6CU, 0xEDEDEDEDU, 0x8D8D8D8DU, 0xA5A5A5A5U, 0x3B3B3B3BU, 0x53535353U, 0xFDFDFDFDU, 0x81818181U,
    0x60606060U, 0x28282828U, 0x9E9E9E9EU, 0x68686868U, 0xAEAEAEAEU, 0x7C7C7C7CU, 0xE1E1E1E1U, 0x48484848U,
    0xB6B6B6B6U, 0xF6F6F6F6U, 0xC6C6C6C6U, 0xD2D2D2D2U, 0x9D9D9D9DU, 0xA9A9A9A9U, 0xFEFEFEFEU,
};

/* ---------------------------------------------------------------------------
 * Function implementations
 * --------------------------------------------------------------------------- */
/**
 * @brief Initialize frame buffer mem_slab once before UNCODED heap malloc
 * @return none
 * @note Required by bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, ...)
 */
STATIC VOID_T __mt_mipi_frame_buffer_init_once(VOID_T)
{
    if (s_mipi_fb_inited == FALSE) {
        bk_frame_buffer_init();
        s_mipi_fb_inited = TRUE;
    }
}

/**
 * @brief Run CSI PHY built-in self test via MMIO
 * @return BIST result register value; 3 means pass
 */
STATIC UINT32_T __mt_mipi_csi_bist_test(VOID_T)
{
    volatile UINT32_T *csi_clk_en = (volatile UINT32_T *)MT_MIPI_CSI_CLK_EN_REG;
    volatile UINT32_T *csi_ext = (volatile UINT32_T *)MT_MIPI_BASEADDR_CSI_EXT;
    volatile UINT32_T *csi_phy = (volatile UINT32_T *)(MT_MIPI_BASEADDR_CSI + 0x44U);
    volatile UINT32_T *csi_bist = (volatile UINT32_T *)(MT_MIPI_BASEADDR_CSI + 0x4048U);
    volatile UINT32_T *csi_bist_result = (volatile UINT32_T *)(MT_MIPI_BASEADDR_CSI_EXT + 0x40U);
    UINT32_T result = 0U;
    UINT32_T cnt = MT_MIPI_CSI_BIST_RETRY_MAX;

    *csi_clk_en |= MT_MIPI_CSI_CLK_EN_MASK;
    rtos_delay_milliseconds(1);
    *csi_ext = 0x3U;
    rtos_delay_milliseconds(1);
    *csi_phy = 0xFFFFFFFFU;
    rtos_delay_milliseconds(1);
    *csi_bist = 0x8U;
    rtos_delay_milliseconds(1);
    *csi_ext = 0xFU;
    rtos_delay_milliseconds(1);

    result = *csi_bist_result;
    while ((result != MT_MIPI_CSI_BIST_PASS) && (cnt > 0U)) {
        *csi_ext = 0x3U;
        *csi_ext = 0xFU;
        rtos_delay_milliseconds(1);
        result = *csi_bist_result;
        cnt--;
    }

    MT_MIPI_LOGI("%s, read bist result: %x\n", __func__, result);
    return result;
}

/**
 * @brief Enable or disable display 1.8V AUX LDO
 * @param[in] enable TRUE to enable, FALSE to disable
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __mt_mipi_ldo_enable(BOOL_T enable)
{
    pm_auxldo_ctrl_cfg_t auxldo_cfg = {0};
    int ldo_en = enable ? PM_AUXLDO_ENABLE : PM_AUXLDO_DISABLE;

    MT_MIPI_LOGI("%s, vddio enable: %d\n", __func__, ldo_en);
    auxldo_cfg.ldo = AUXLDOS_SEL_1P8V;
    auxldo_cfg.out = PM_AUXLDO_1P8V_OUT_1P8V;
    auxldo_cfg.user = PM_AUXLDO_USER_DISPLAY;
    auxldo_cfg.state = ldo_en;
    if (bk_pm_auxldo_ctrl_vote(&auxldo_cfg) != BK_OK) {
        MT_MIPI_LOGE("display 1p8v ldo vote failed\n");
        return OPRT_COM_ERROR;
    }
    rtos_delay_milliseconds(1);
    return OPRT_OK;
}

/**
 * @brief Release display resources allocated during factory test
 * @param[in,out] context display context
 * @param[in,out] frame frame buffer pointer holder
 * @return none
 */
STATIC VOID_T __mt_mipi_test_cleanup(MT_MIPI_DISP_CTX_T *context, VOID_T **frame)
{
    if (context == NULL) {
        return;
    }
    if (context->dpu_ctlr_handle != NULL) {
        bk_display_close(context->dpu_ctlr_handle);
        bk_display_deinit(context->dpu_ctlr_handle);
        bk_display_delete(context->dpu_ctlr_handle);
        context->dpu_ctlr_handle = NULL;
    }
    if (context->panel_handle != NULL) {
        bk_lcd_panel_delete(context->panel_handle);
        context->panel_handle = NULL;
    }
    if (context->dis_bus_handle != NULL) {
        bk_display_bus_delete(context->dis_bus_handle);
        context->dis_bus_handle = NULL;
    }
    if ((frame != NULL) && (*frame != NULL)) {
        bk_frame_buffer_free(*frame);
        *frame = NULL;
    }
}

/**
 * @brief Run MIPI factory test (DSI PRBS7 flush + CSI BIST)
 * @return OPRT_OK on pass, OPRT_COM_ERROR on test failure, OPRT_NOT_SUPPORTED if panel unavailable
 */
OPERATE_RET mt_mipi_factory_test(VOID_T)
{
#if defined(CONFIG_LCD_HX8399C_MIPI_1080x1920) && (CONFIG_LCD_HX8399C_MIPI_1080x1920 == 1)
    bk_err_t ret = BK_OK;
    avdk_err_t test_ret = AVDK_ERR_OK;
    MT_MIPI_DISP_CTX_T *context = &s_mipi_disp_ctx;
    bk_display_dpu_config_t dpu_config = {0};
    const bk_display_dsi_panel_t *panel = &lcd_device_hx8399c_mipi_1080x1920;
    VOID_T *frame = NULL;
    UINT32_T frame_size;
    UINT32_T i;

    dpu_config.video.enable = TRUE;
    dpu_config.video.decompress = FALSE;
    dpu_config.video.format = BK_PIXEL_FORMAT_BGR888;

    const bk_lcd_panel_config_t panel_config = {
        .reset_pin = MT_MIPI_PANEL_RESET_PIN,
    };

    __mt_mipi_frame_buffer_init_once();

    OPERATE_RET ldo_ret = __mt_mipi_ldo_enable(TRUE);
    if (ldo_ret != OPRT_OK) {
        return ldo_ret;
    }

    AVDK_GOTO_ON_ERROR(bk_display_dsi_bus_new(&context->dis_bus_handle, NULL), cleanup, MT_MIPI_TAG,
                       "display dsi bus new err\n");
    AVDK_GOTO_ON_ERROR(bk_lcd_mipi_panel_new(context->dis_bus_handle, &panel_config, panel,
                                               &context->panel_handle),
                       cleanup, MT_MIPI_TAG, "create panel err\n");
    AVDK_GOTO_ON_ERROR(bk_display_dpu_ctlr_new(&context->dpu_ctlr_handle, context->panel_handle, &dpu_config),
                       cleanup, MT_MIPI_TAG, "display dpu ctlr new err\n");
    AVDK_GOTO_ON_ERROR(bk_display_init(context->dpu_ctlr_handle), cleanup, MT_MIPI_TAG, "display init err\n");
    AVDK_GOTO_ON_ERROR(bk_display_open(context->dpu_ctlr_handle), cleanup, MT_MIPI_TAG, "display open err\n");

    frame_size = panel->timing.h_size * panel->timing.v_size * MT_MIPI_BPP_BGR888;
    frame = bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, frame_size);
    if (frame == NULL) {
        MT_MIPI_LOGE("frame slab malloc failed, size=%u\n", frame_size);
        test_ret = AVDK_ERR_GENERIC;
        goto cleanup;
    }

    for (i = 0U; i < (frame_size / sizeof(s_prbs7_data)); i++) {
        os_memcpy((UINT8_T *)frame + i * sizeof(s_prbs7_data), s_prbs7_data, sizeof(s_prbs7_data));
    }
    bk_display_flush(context->dpu_ctlr_handle, frame, NULL);

    if (__mt_mipi_csi_bist_test() != MT_MIPI_CSI_BIST_PASS) {
        MT_MIPI_LOGE("csi bist TEST FAIL\n");
        test_ret = AVDK_ERR_GENERIC;
    } else {
        MT_MIPI_LOGI("csi bist TEST PASS\n");
    }

cleanup:
    __mt_mipi_test_cleanup(context, &frame);
    __mt_mipi_ldo_enable(FALSE);
    if ((ret != BK_OK) && (test_ret == AVDK_ERR_OK)) {
        test_ret = AVDK_ERR_GENERIC;
    }
    MT_MIPI_LOGI("mipi test done: %s\n", (test_ret == AVDK_ERR_OK) ? "pass" : "fail");
    return (test_ret == AVDK_ERR_OK) ? OPRT_OK : OPRT_COM_ERROR;
#else
    MT_MIPI_LOGE("hx8399c panel not enabled in sdkconfig\n");
    return OPRT_NOT_SUPPORTED;
#endif
}

/**
 * @brief Run MIPI factory test and fill JSON reply buffer for mftest framework
 * @param[out] ret_data heap buffer with JSON result (caller frees)
 * @param[out] ret_len length of @p ret_data
 * @return OPRT_OK on dispatch success, OPRT_MALLOC_FAILED on alloc failure
 */
OPERATE_RET mt_mipi_factory_test_reply(UCHAR_T **ret_data, USHORT_T *ret_len)
{
    CHAR_T *ret_buf = NULL;
    OPERATE_RET test_ret;
    CONST CHAR_T *json_str = NULL;

    if (ret_data == NULL || ret_len == NULL) {
        return OPRT_INVALID_PARM;
    }

    *ret_data = NULL;
    *ret_len = 0;

    ret_buf = (CHAR_T *)tkl_system_malloc(MT_MIPI_REPLY_BUF_SIZE);
    if (ret_buf == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    memset(ret_buf, 0, MT_MIPI_REPLY_BUF_SIZE);

    test_ret = mt_mipi_factory_test();
    if (test_ret == OPRT_OK) {
        json_str = "{\"ret\":true}";
        MT_MIPI_LOGI("mipi factory test success\r\n");
    } else {
        json_str = "{\"ret\":false}";
        MT_MIPI_LOGE("mipi factory test failed, ret=%d\r\n", test_ret);
    }

    strncpy(ret_buf, json_str, MT_MIPI_REPLY_BUF_SIZE - 1U);
    ret_buf[MT_MIPI_REPLY_BUF_SIZE - 1U] = '\0';
    *ret_data = (UCHAR_T *)ret_buf;
    *ret_len = (USHORT_T)strlen(ret_buf);
    return OPRT_OK;
}
