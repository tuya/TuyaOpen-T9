/**
 * @file tkl_mipi.h
 * @brief Common process - adapter the MIPI display api
 * @version 0.1
 * @date 2026-04-09
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_MIPI_H__
#define __TKL_MIPI_H__

#include "tuya_cloud_types.h"
#include <components/bk_display_bus.h>
#include <components/bk_display_types.h>
#include <components/bk_lcd_types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TYPE_SRAM = 0,
    TYPE_PSRAM,
} RAM_TYPE_E;

typedef void (*frame_buff_free_cb)(UINT8_T *frame_buff);

typedef struct {
    RAM_TYPE_E type;
    DISPLAY_PIXEL_FORMAT_E fmt;
    uint16_t x_start;
    uint16_t y_start;
    uint16_t width;
    uint16_t height;
    frame_buff_free_cb free_cb;
    UINT32_T len;
    uint8_t *frame;
    uint8_t *pdata;
} ty_frame_buffer_t;

typedef struct {
    struct {
        BOOL_T enable;
        INT_T pin_reset;
        INT_T pin_backlight;
        INT_T pin_scl;
        INT_T pin_sda;
        const CHAR_T *panel_name;
        const bk_display_dsi_panel_t *panel;
    } mipi;

    struct {
        BOOL_T enable;
        BOOL_T decompress;
        UINT32_T format; /* bk_pixel_format_t */
    } dpu_video;

    BOOL_T opened;
    bk_display_ctlr_handle_t dpu_ctlr_handle;
    bk_display_bus_handle_t dis_bus_handle;
    bk_avdk_lcd_panel_handle_t panel_handle;
    bk_display_bus_handle_t cfg_bus_handle;
} ty_display_mipi_device_t;

OPERATE_RET tkl_display_mipi_open(ty_display_mipi_device_t *device);
OPERATE_RET tkl_display_mipi_close(ty_display_mipi_device_t *device);
OPERATE_RET tkl_display_mipi_flush(ty_display_mipi_device_t *device, ty_frame_buffer_t *frame_buff);

#ifdef __cplusplus
}
#endif

#endif
