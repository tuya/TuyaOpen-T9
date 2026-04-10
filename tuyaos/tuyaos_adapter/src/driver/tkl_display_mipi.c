#include "tkl_mipi.h"

#include <stdbool.h>
#include <avdk_error.h>
#include <common/avdk_pixel_types.h>
#include <common/bk_err.h>
#include <components/bk_display.h>
#include <components/bk_display_bus.h>
#include <components/bk_display_dpu_ctlr.h>
#include <components/bk_lcd_panel.h>
#include <components/log.h>
#include <driver/gpio.h>
#include <driver/gpio_types.h>
#include <os/mem.h>
#include <os/str.h>
#include "gpio_driver.h"

#define TAG "tkl-mipi"

static frame_buff_free_cb s_flush_free_cb = NULL;

static OPERATE_RET tkl_display_mipi_err_convert(avdk_err_t ret)
{
    if (ret == AVDK_ERR_OK) {
        return OPRT_OK;
    }

    switch (ret) {
    case AVDK_ERR_INVAL:
        return OPRT_INVALID_PARM;
    case AVDK_ERR_NOMEM:
        return OPRT_MALLOC_FAILED;
    case AVDK_ERR_UNSUPPORTED:
        return OPRT_NOT_SUPPORTED;
    default:
        return OPRT_COM_ERROR;
    }
}

static bk_pixel_format_t tkl_display_default_video_format(const ty_display_mipi_device_t *device)
{
    if (device != NULL &&
        device->dpu_video.format != BK_PIXEL_FORMAT_INVALID &&
        device->dpu_video.format != BK_PIXEL_FORMAT_UNKNOW) {
        return (bk_pixel_format_t)device->dpu_video.format;
    }

    return BK_PIXEL_FORMAT_ARGB8888;
}

static avdk_err_t tkl_display_flush_done(void *args)
{
    if (s_flush_free_cb != NULL) {
        s_flush_free_cb((uint8_t *)args);
    }

    return AVDK_ERR_OK;
}

static void tkl_display_runtime_reset(ty_display_mipi_device_t *device)
{
    if (device == NULL) {
        return;
    }

    device->opened = FALSE;
    device->dpu_ctlr_handle = NULL;
    device->dis_bus_handle = NULL;
    device->panel_handle = NULL;
    device->cfg_bus_handle = NULL;
}

static void tkl_display_cleanup(ty_display_mipi_device_t *device)
{
    if (device == NULL) {
        return;
    }

    if (device->mipi.pin_backlight >= 0) {
        bk_gpio_set_output_low((gpio_id_t)device->mipi.pin_backlight);
    }

    if (device->dpu_ctlr_handle != NULL) {
        bk_display_deinit(device->dpu_ctlr_handle);
        bk_display_delete(device->dpu_ctlr_handle);
        device->dpu_ctlr_handle = NULL;
    }

    if (device->panel_handle != NULL) {
        bk_lcd_panel_reset(device->panel_handle);
        bk_lcd_panel_del(device->panel_handle);
        device->panel_handle = NULL;
    }

    if (device->dis_bus_handle != NULL) {
        bk_display_bus_delete(device->dis_bus_handle);
        device->dis_bus_handle = NULL;
    }

    if (device->cfg_bus_handle != NULL) {
        bk_display_bus_delete(device->cfg_bus_handle);
        device->cfg_bus_handle = NULL;
    }

    device->opened = FALSE;
    s_flush_free_cb = NULL;
}

static const bk_display_dsi_panel_t *tkl_display_panel_resolve(ty_display_mipi_device_t *device)
{
    const bk_display_dsi_panel_t *panel = NULL;
    const bk_display_dsi_panel_t *panel_list[1] = {0};

    if (device == NULL) {
        return NULL;
    }

    if (device->mipi.panel != NULL) {
        return device->mipi.panel;
    }

    if (device->mipi.panel_name != NULL && device->mipi.panel_name[0] != '\0') {
        panel = bk_lcd_find_mipi_panel_by_name(device->mipi.panel_name);
        if (panel != NULL) {
            device->mipi.panel = panel;
            return panel;
        }
    }

    if (bk_lcd_get_mipi_panel_list(panel_list, 1) > 0) {
        device->mipi.panel = panel_list[0];
        return panel_list[0];
    }

    return NULL;
}

OPERATE_RET tkl_display_mipi_open(ty_display_mipi_device_t *device)
{
    avdk_err_t ret = AVDK_ERR_OK;
    const bk_display_dsi_panel_t *panel = NULL;
    bk_display_dpu_config_t lcd_cfg;
    bk_lcd_panel_dev_config_t panel_dev_config;

    if (device == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (device->opened == TRUE) {
        return OPRT_OK;
    }

    panel = tkl_display_panel_resolve(device);
    if (panel == NULL) {
        BK_LOGE(TAG, "mipi panel is not configured\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    os_memset(&lcd_cfg, 0, sizeof(lcd_cfg));
    lcd_cfg.video.enable = true;
    lcd_cfg.video.decompress = device->dpu_video.decompress ? true : false;
    lcd_cfg.video.format = tkl_display_default_video_format(device);
    lcd_cfg.graphic.enable = false;

    ret = bk_display_dsi_bus_new(&device->dis_bus_handle, NULL);
    if (ret != AVDK_ERR_OK) {
        goto err;
    }

    ret = bk_display_bus_enable(device->dis_bus_handle);
    if (ret != AVDK_ERR_OK) {
        goto err;
    }

    if (device->mipi.pin_scl >= 0 && device->mipi.pin_sda >= 0) {
        bk_display_i2c_bus_config_t i2c_cfg = {
            .scl_pin = (uint8_t)device->mipi.pin_scl,
            .sda_pin = (uint8_t)device->mipi.pin_sda,
        };

        ret = bk_display_i2c_bus_new(&device->cfg_bus_handle, &i2c_cfg);
        if (ret != AVDK_ERR_OK) {
            goto err;
        }
    }

    os_memset(&panel_dev_config, 0, sizeof(panel_dev_config));
    panel_dev_config.reset_pin = (int8_t)device->mipi.pin_reset;
    panel_dev_config.rgb_ele_order = COLOR_RGB_ELEMENT_ORDER_RGB;
    panel_dev_config.data_endian = LCD_RGB_DATA_ENDIAN_BIG;
    panel_dev_config.bits_per_pixel = 16;
    panel_dev_config.vendor_config = (device->cfg_bus_handle != NULL) ? &device->cfg_bus_handle : NULL;

    ret = bk_lcd_mipi_panel_new(device->dis_bus_handle, &panel_dev_config, panel, &device->panel_handle);
    if (ret != AVDK_ERR_OK) {
        goto err;
    }

    ret = bk_lcd_panel_reset(device->panel_handle);
    if (ret != AVDK_ERR_OK) {
        goto err;
    }

    ret = bk_lcd_panel_init(device->panel_handle);
    if (ret != AVDK_ERR_OK) {
        goto err;
    }

    ret = bk_lcd_panel_get_disp_timing(device->panel_handle, &lcd_cfg.timing);
    if (ret != AVDK_ERR_OK) {
        goto err;
    }

    ret = bk_display_dpu_ctlr_new(&device->dpu_ctlr_handle, &lcd_cfg);
    if (ret != AVDK_ERR_OK) {
        goto err;
    }

    ret = bk_display_init(device->dpu_ctlr_handle);
    if (ret != AVDK_ERR_OK) {
        goto err;
    }

    ret = bk_display_open(device->dpu_ctlr_handle);
    if (ret != AVDK_ERR_OK) {
        goto err;
    }

    if (device->mipi.pin_backlight >= 0) {
        gpio_dev_unmap((gpio_id_t)device->mipi.pin_backlight);
        BK_LOG_ON_ERR(bk_gpio_enable_output((gpio_id_t)device->mipi.pin_backlight));
        BK_LOG_ON_ERR(bk_gpio_pull_up((gpio_id_t)device->mipi.pin_backlight));
        bk_gpio_set_capacity((gpio_id_t)device->mipi.pin_backlight, GPIO_DRIVER_CAPACITY_3);
        bk_gpio_set_output_high((gpio_id_t)device->mipi.pin_backlight);
    }

    device->opened = TRUE;
    return OPRT_OK;

err:
    BK_LOGE(TAG, "open mipi display failed: %d\r\n", ret);
    tkl_display_cleanup(device);
    return tkl_display_mipi_err_convert(ret);
}

OPERATE_RET tkl_display_mipi_close(ty_display_mipi_device_t *device)
{
    if (device == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (device->opened == FALSE &&
        device->dpu_ctlr_handle == NULL &&
        device->panel_handle == NULL &&
        device->dis_bus_handle == NULL &&
        device->cfg_bus_handle == NULL) {
        tkl_display_runtime_reset(device);
        return OPRT_OK;
    }

    tkl_display_cleanup(device);
    return OPRT_OK;
}

OPERATE_RET tkl_display_mipi_flush(ty_display_mipi_device_t *device, ty_frame_buffer_t *frame_buff)
{
    avdk_err_t ret;
    uint8_t *flush_frame = NULL;

    if (device == NULL || frame_buff == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (device->opened == FALSE || device->dpu_ctlr_handle == NULL) {
        return OPRT_COM_ERROR;
    }

    /*
     * The BK display stack calls flush_free_cb_t with the same frame pointer passed
     * to bk_display_flush(). Keep the original frame pointer when available so the
     * upstream release callback receives the allocation address it expects.
     */
    flush_frame = (frame_buff->frame != NULL) ? frame_buff->frame : frame_buff->pdata;

    if (flush_frame == NULL) {
        return OPRT_INVALID_PARM;
    }

    s_flush_free_cb = frame_buff->free_cb;
    ret = bk_display_flush(device->dpu_ctlr_handle, flush_frame, frame_buff->free_cb ? tkl_display_flush_done : NULL);
    if (ret != AVDK_ERR_OK) {
        s_flush_free_cb = NULL;
        return tkl_display_mipi_err_convert(ret);
    }

    return OPRT_OK;
}
