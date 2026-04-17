#include <stdbool.h>
#include <common/avdk_pixel_types.h>
#include <common/bk_err.h>
#include <components/bk_display.h>
#include <components/bk_display_bus.h>
#include <components/bk_display_dpu_ctlr.h>
#include <driver/mipi_dsi_types.h>
#include "tkl_mipi_dsi.h"
#include "tkl_output.h"

typedef struct {
    BOOL_T inited;
    tkl_mipi_dsi_io_cfg_t io_cfg;
    bk_display_bus_handle_t dis_bus_handle;
    bk_display_bus_handle_t cfg_bus_handle;
} tkl_mipi_dsi_runtime_t;

typedef struct {
    BOOL_T inited;
    TUYA_MIPI_BASE_CFG_T base_cfg;
    tkl_mipi_dsi_video_cfg_t video_cfg;
    bk_display_ctlr_handle_t dpu_ctlr_handle;
} tkl_mipi_dpu_runtime_t;

static tkl_mipi_dsi_runtime_t s_dsi_runtime;
static tkl_mipi_dpu_runtime_t s_dpu_runtime;
static void (*s_flush_free_cb)(UINT8_T *frame_buff) = NULL;

static BOOL_T tkl_mipi_dsi_lane_count_valid(UINT8_T n_lanes)
{
    return (n_lanes >= 1U && n_lanes <= 4U) ? TRUE : FALSE;
}

static UINT32_T tkl_mipi_dsi_clk_mhz_normalize(UINT32_T clk)
{
    /* BK DSI/DPU clock expects MHz, while TUYA configs may pass raw Hz. */
    if (clk > 1000U) {
        return (clk + 500000U) / 1000000U;
    }

    return clk;
}

static UINT8_T tkl_mipi_dsi_lane_count_convert(UINT8_T n_lanes)
{
    switch (n_lanes) {
        case 1U:
            return DSI_ACTIVE_LANES_1;
        case 2U:
            return DSI_ACTIVE_LANES_2;
        case 3U:
            return DSI_ACTIVE_LANES_3;
        case 4U:
            return DSI_ACTIVE_LANES_4;
        default:
            return 0xFFU;
    }
}

static bk_pixel_format_t tkl_mipi_dpu_pixel_format_convert(DISPLAY_PIXEL_FORMAT_E format)
{
    switch (format) {
        case TY_PIXEL_FMT_RGB565:
            return BK_PIXEL_FORMAT_RGB565;
        case TY_PIXEL_FMT_RGB888:
            return BK_PIXEL_FORMAT_RGB888;
        case TY_PIXEL_FMT_ARGB8888:
            return BK_PIXEL_FORMAT_ARGB8888;
        default:
            return BK_PIXEL_FORMAT_INVALID;
    }
}

static bk_pixel_format_t tkl_mipi_dpu_format_get(const tkl_mipi_dsi_video_cfg_t *video_cfg,
                                                 const TUYA_MIPI_BASE_CFG_T *base_cfg)
{
    bk_pixel_format_t format;

    if (video_cfg != NULL) {
        format = tkl_mipi_dpu_pixel_format_convert(video_cfg->format);
        if (format != BK_PIXEL_FORMAT_INVALID) {
            return format;
        }
    }

    if (base_cfg != NULL) {
        format = tkl_mipi_dpu_pixel_format_convert(base_cfg->pixel_fmt);
        if (format != BK_PIXEL_FORMAT_INVALID) {
            return format;
        }
    }

    return BK_PIXEL_FORMAT_ARGB8888;
}

static OPERATE_RET tkl_mipi_dsi_flush_done(void *args)
{
    if (s_flush_free_cb != NULL) {
        s_flush_free_cb((uint8_t *)args);
    }

    return OPRT_OK;
}

static VOID_T tkl_mipi_dsi_runtime_reset(VOID_T)
{
    os_memset(&s_dsi_runtime, 0, sizeof(s_dsi_runtime));
}

static VOID_T tkl_mipi_dpu_runtime_reset(VOID_T)
{
    os_memset(&s_dpu_runtime, 0, sizeof(s_dpu_runtime));
}

static OPERATE_RET tkl_mipi_dsi_clock_set(const TUYA_MIPI_BASE_CFG_T *base_cfg)
{
    bk_panel_clock_config_t clock_cfg;
    UINT8_T bk_n_lanes;
    UINT32_T dpu_clk_mhz;

    if (base_cfg == NULL || s_dsi_runtime.dis_bus_handle == NULL) {
        return OPRT_INVALID_PARM;
    }

    dpu_clk_mhz = tkl_mipi_dsi_clk_mhz_normalize(base_cfg->clk);

    if (base_cfg->clk == 0U || dpu_clk_mhz == 0U || base_cfg->width == 0U || base_cfg->height == 0U ||
        tkl_mipi_dsi_lane_count_valid(base_cfg->n_lanes) != TRUE) {
        return OPRT_INVALID_PARM;
    }

    bk_n_lanes = tkl_mipi_dsi_lane_count_convert(base_cfg->n_lanes);
    if (bk_n_lanes == 0xFFU) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&clock_cfg, 0, sizeof(clock_cfg));
    clock_cfg.clk = dpu_clk_mhz;
    clock_cfg.n_lanes = bk_n_lanes;
    clock_cfg.timing.clk = dpu_clk_mhz;
    clock_cfg.timing.h_size = base_cfg->width;
    clock_cfg.timing.v_size = base_cfg->height;
    clock_cfg.timing.hsync_back_porch = base_cfg->hsync_back_porch;
    clock_cfg.timing.hsync_front_porch = base_cfg->hsync_front_porch;
    clock_cfg.timing.vsync_back_porch = base_cfg->vsync_back_porch;
    clock_cfg.timing.vsync_front_porch = base_cfg->vsync_front_porch;
    clock_cfg.timing.hsync_pulse_width = base_cfg->hsync_pulse_width;
    clock_cfg.timing.vsync_pulse_width = base_cfg->vsync_pulse_width;

    return bk_display_bus_set_clock(s_dsi_runtime.dis_bus_handle, &clock_cfg);
}

OPERATE_RET tkl_mipi_dsi_init(const tkl_mipi_dsi_io_cfg_t *io_cfg, const TUYA_MIPI_BASE_CFG_T *base_cfg)
{
    OPERATE_RET ret;

    if (io_cfg == NULL || base_cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_dsi_runtime.inited == TRUE) {
        return OPRT_OK;
    }

    ret = bk_display_dsi_bus_new(&s_dsi_runtime.dis_bus_handle, NULL);
    if (ret != OPRT_OK) {
        goto err;
    }

    ret = bk_display_bus_enable(s_dsi_runtime.dis_bus_handle);
    if (ret != OPRT_OK) {
        goto err;
    }

    if (io_cfg->pin_scl >= 0 && io_cfg->pin_scl < TUYA_GPIO_NUM_MAX &&
        io_cfg->pin_sda >= 0 && io_cfg->pin_sda < TUYA_GPIO_NUM_MAX) {
        bk_display_i2c_bus_config_t i2c_cfg = {
            .scl_pin = (uint8_t)io_cfg->pin_scl,
            .sda_pin = (uint8_t)io_cfg->pin_sda,
        };

        ret = bk_display_i2c_bus_new(&s_dsi_runtime.cfg_bus_handle, &i2c_cfg);
        if (ret != OPRT_OK) {
            goto err;
        }
    }

    ret = tkl_mipi_dsi_clock_set(base_cfg);
    if (ret != OPRT_OK) {
        goto err;
    }

    s_dsi_runtime.io_cfg = *io_cfg;
    s_dsi_runtime.inited = TRUE;
    return OPRT_OK;

err:
    tkl_mipi_dsi_deinit();
    return ret;
}

OPERATE_RET tkl_mipi_dsi_deinit(VOID_T)
{
    if (s_dsi_runtime.dis_bus_handle != NULL) {
        bk_display_bus_delete(s_dsi_runtime.dis_bus_handle);
        s_dsi_runtime.dis_bus_handle = NULL;
    }

    if (s_dsi_runtime.cfg_bus_handle != NULL) {
        bk_display_bus_delete(s_dsi_runtime.cfg_bus_handle);
        s_dsi_runtime.cfg_bus_handle = NULL;
    }

    tkl_mipi_dsi_runtime_reset();
    return OPRT_OK;
}

OPERATE_RET tkl_mipi_dsi_tx_cmd(UINT8_T cmd, const VOID_T *param, UINT32_T size)
{
    if (s_dsi_runtime.inited != TRUE || s_dsi_runtime.dis_bus_handle == NULL) {
        return OPRT_INVALID_PARM;
    }

    return bk_display_bus_write(s_dsi_runtime.dis_bus_handle, BK_DISPLAY_BUS_RW_DSI_CMD, cmd, param, size);
}

OPERATE_RET tkl_mipi_dpu_init(const TUYA_MIPI_BASE_CFG_T *base_cfg, const tkl_mipi_dsi_video_cfg_t *video_cfg)
{
    OPERATE_RET ret;
    bk_display_dpu_config_t lcd_cfg;
    UINT32_T dpu_clk_mhz;

    if (base_cfg == NULL || video_cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_dpu_runtime.inited == TRUE) {
        return OPRT_OK;
    }

    dpu_clk_mhz = tkl_mipi_dsi_clk_mhz_normalize(base_cfg->clk);

    if (base_cfg->width == 0U || base_cfg->height == 0U ||
        base_cfg->clk == 0U || dpu_clk_mhz == 0U ||
        tkl_mipi_dsi_lane_count_valid(base_cfg->n_lanes) != TRUE) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&lcd_cfg, 0, sizeof(lcd_cfg));
    lcd_cfg.video.enable = (video_cfg->enable != FALSE) ? true : false;
    lcd_cfg.video.decompress = (video_cfg->decompress != FALSE) ? true : false;
    lcd_cfg.video.format = tkl_mipi_dpu_format_get(video_cfg, base_cfg);
    lcd_cfg.graphic.enable = false;

    lcd_cfg.timing.clk = dpu_clk_mhz;
    lcd_cfg.timing.h_size = base_cfg->width;
    lcd_cfg.timing.v_size = base_cfg->height;
    lcd_cfg.timing.hsync_back_porch = base_cfg->hsync_back_porch;
    lcd_cfg.timing.hsync_front_porch = base_cfg->hsync_front_porch;
    lcd_cfg.timing.vsync_back_porch = base_cfg->vsync_back_porch;
    lcd_cfg.timing.vsync_front_porch = base_cfg->vsync_front_porch;
    lcd_cfg.timing.hsync_pulse_width = base_cfg->hsync_pulse_width;
    lcd_cfg.timing.vsync_pulse_width = base_cfg->vsync_pulse_width;

    ret = bk_display_dpu_ctlr_new(&s_dpu_runtime.dpu_ctlr_handle, &lcd_cfg);
    if (ret != OPRT_OK) {
        goto err;
    }

    ret = bk_display_init(s_dpu_runtime.dpu_ctlr_handle);
    if (ret != OPRT_OK) {
        goto err;
    }

    ret = bk_display_open(s_dpu_runtime.dpu_ctlr_handle);
    if (ret != OPRT_OK) {
        goto err;
    }

    s_dpu_runtime.base_cfg = *base_cfg;
    s_dpu_runtime.video_cfg = *video_cfg;
    s_dpu_runtime.inited = TRUE;
    return OPRT_OK;

err:
    tkl_log_output("init mipi dpu failed: %d\r\n", ret);
    tkl_mipi_dpu_deinit();
    return ret;
}

OPERATE_RET tkl_mipi_dpu_deinit(VOID_T)
{
    if (s_dpu_runtime.dpu_ctlr_handle != NULL) {
        bk_display_deinit(s_dpu_runtime.dpu_ctlr_handle);
        bk_display_delete(s_dpu_runtime.dpu_ctlr_handle);
    }

    tkl_mipi_dpu_runtime_reset();
    s_flush_free_cb = NULL;
    return OPRT_OK;
}

OPERATE_RET tkl_mipi_dsi_flush(tkl_mipi_dsi_flush_frame_t *frame)
{
    OPERATE_RET ret;

    if (frame == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_dpu_runtime.inited != TRUE || s_dpu_runtime.dpu_ctlr_handle == NULL) {
        return OPRT_COM_ERROR;
    }

    if (frame->buffer == NULL) {
        return OPRT_INVALID_PARM;
    }

    s_flush_free_cb = frame->free_cb;
    ret = bk_display_flush(s_dpu_runtime.dpu_ctlr_handle, frame->buffer,
                           (frame->free_cb != NULL) ? tkl_mipi_dsi_flush_done : NULL);
    if (ret != OPRT_OK) {
        s_flush_free_cb = NULL;
        return ret;
    }

    return OPRT_OK;
}
