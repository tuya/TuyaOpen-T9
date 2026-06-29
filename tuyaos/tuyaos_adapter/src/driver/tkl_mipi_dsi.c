#include <stdbool.h>
#include <common/avdk_pixel_types.h>
#include <common/bk_err.h>
#include <components/bk_display.h>
#include <components/bk_display_bus.h>
#include <driver/mipi_dsi.h>
#include <driver/mipi_dsi_types.h>
#include "tkl_mipi_dsi.h"
#include "tkl_output.h"

typedef struct {
    BOOL_T inited;
    bk_display_bus_handle_t dis_bus_handle;
} tkl_mipi_dsi_runtime_t;

typedef struct {
    BOOL_T inited;
    BOOL_T input_decompress;
    TUYA_MIPI_BASE_CFG_T base_cfg;
    bk_display_ctlr_handle_t dpu_ctlr_handle;
    bk_avdk_lcd_panel_handle_t panel_handle;
} tkl_mipi_dpu_runtime_t;

static tkl_mipi_dsi_runtime_t s_dsi_runtime;
static tkl_mipi_dpu_runtime_t s_dpu_runtime;
/* Panel descriptor consumed by bk_lcd_mipi_panel_new(); the panel handle keeps a
 * pointer to it (no copy), so it must outlive the panel -> static storage. */
static bk_display_dsi_panel_t s_dsi_panel_desc;
static void (*s_flush_free_cb)(UINT8_T *frame_buff) = NULL;

static OPERATE_RET tkl_mipi_dpu_init(const TUYA_MIPI_BASE_CFG_T *base_cfg);
static OPERATE_RET tkl_mipi_dpu_deinit(VOID_T);

static BOOL_T tkl_mipi_dsi_lane_count_valid(UINT8_T n_lanes)
{
    return (n_lanes >= 1U && n_lanes <= 4U) ? TRUE : FALSE;
}

static UINT8_T tkl_mipi_dsi_fps_calculate(const TUYA_MIPI_BASE_CFG_T *base_cfg)
{
    UINT64_T pclk_hz;
    UINT64_T frame_pixels;
    UINT64_T fps;
    UINT32_T h_total;
    UINT32_T v_total;

    if (base_cfg == NULL || base_cfg->clk == 0U || base_cfg->width == 0U || base_cfg->height == 0U) {
        return 0U;
    }

    h_total = (UINT32_T)base_cfg->width + base_cfg->hsync_pulse_width +
              base_cfg->hsync_back_porch + base_cfg->hsync_front_porch;
    v_total = (UINT32_T)base_cfg->height + base_cfg->vsync_pulse_width +
              base_cfg->vsync_back_porch + base_cfg->vsync_front_porch;
    frame_pixels = (UINT64_T)h_total * v_total;
    if (frame_pixels == 0U) {
        return 0U;
    }

    pclk_hz = (base_cfg->clk > 1000U) ? base_cfg->clk : ((UINT64_T)base_cfg->clk * 1000000U);
    fps = (pclk_hz + frame_pixels / 2U) / frame_pixels;
    if (fps > 255U) {
        fps = 255U;
    }

    return (UINT8_T)fps;
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
    s_dpu_runtime.input_decompress = TRUE;
}

OPERATE_RET tkl_mipi_dsi_set_input_mode(BOOL_T decompress)
{
    if (s_dsi_runtime.inited == TRUE || s_dpu_runtime.inited == TRUE) {
        return OPRT_COM_ERROR;
    }

    s_dpu_runtime.input_decompress = (decompress != FALSE) ? TRUE : FALSE;
    return OPRT_OK;
}

OPERATE_RET tkl_mipi_dsi_init(const TUYA_MIPI_BASE_CFG_T *base_cfg)
{
    OPERATE_RET ret;

    if (base_cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_dsi_runtime.inited == TRUE) {
        return OPRT_OK;
    }

    ret = bk_display_dsi_bus_new(&s_dsi_runtime.dis_bus_handle, NULL);
    if (ret != OPRT_OK) {
        goto err;
    }

    /* DSI bus already defaults to DPU_CLK_SRC_DPHY_DPLL; set it explicitly to keep
     * behaviour identical to the previous implementation. Must be done between
     * bk_display_dsi_bus_new() and bk_lcd_mipi_panel_new(). */
    ret = bk_display_bus_set_clock_src(s_dsi_runtime.dis_bus_handle, DPU_CLK_SRC_DPHY_DPLL);
    if (ret != OPRT_OK) {
        goto err;
    }

    ret = tkl_mipi_dpu_init(base_cfg);
    if (ret != OPRT_OK) {
        goto err;
    }

    s_dsi_runtime.inited = TRUE;
    return OPRT_OK;

err:
    tkl_mipi_dsi_deinit();
    return ret;
}

OPERATE_RET tkl_mipi_dsi_deinit(VOID_T)
{
    tkl_mipi_dpu_deinit();

    if (s_dsi_runtime.dis_bus_handle != NULL) {
        bk_display_bus_delete(s_dsi_runtime.dis_bus_handle);
        s_dsi_runtime.dis_bus_handle = NULL;
    }

    tkl_mipi_dsi_runtime_reset();
    return OPRT_OK;
}

OPERATE_RET tkl_mipi_dsi_tx_cmd(UINT8_T cmd, const VOID_T *param, UINT32_T size)
{
    if (s_dsi_runtime.inited != TRUE || s_dsi_runtime.dis_bus_handle == NULL) {
        return OPRT_INVALID_PARM;
    }

    return bk_display_bus_tx_param(s_dsi_runtime.dis_bus_handle, cmd, param, size);
}

static OPERATE_RET tkl_mipi_dpu_init(const TUYA_MIPI_BASE_CFG_T *base_cfg)
{
    OPERATE_RET ret;
    bk_display_dpu_config_t lcd_cfg;
    bk_lcd_panel_config_t panel_cfg;
    bk_pixel_format_t pixel_format;
    UINT8_T fps;
    UINT8_T bk_n_lanes;

    if (base_cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_dpu_runtime.inited == TRUE) {
        return OPRT_OK;
    }

    if (base_cfg->width == 0U || base_cfg->height == 0U || base_cfg->clk == 0U ||
        tkl_mipi_dsi_lane_count_valid(base_cfg->n_lanes) != TRUE) {
        return OPRT_INVALID_PARM;
    }

    bk_n_lanes = tkl_mipi_dsi_lane_count_convert(base_cfg->n_lanes);
    if (bk_n_lanes == 0xFFU) {
        return OPRT_INVALID_PARM;
    }

    pixel_format = tkl_mipi_dpu_pixel_format_convert(base_cfg->pixel_fmt);
    if (pixel_format == BK_PIXEL_FORMAT_INVALID) {
        return OPRT_INVALID_PARM;
    }

    /* The new BK display stack derives the DPI pixel clock and DSI timing from the
     * panel descriptor (clock = fps * h_total * v_total, computed in
     * bk_lcd_mipi_panel_new()/bk_display_init()), so the timing/lane/fps that used
     * to be programmed via mipi_dsi_clock_set() are now carried on the panel. */
    fps = tkl_mipi_dsi_fps_calculate(base_cfg);
    if (fps == 0U) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&s_dsi_panel_desc, 0, sizeof(s_dsi_panel_desc));
    s_dsi_panel_desc.name = "tuya_dsi_panel";
    s_dsi_panel_desc.n_lanes = bk_n_lanes;   /* raw active data lanes (1~4) */
    s_dsi_panel_desc.fps = fps;
    s_dsi_panel_desc.timing.h_size = base_cfg->width;
    s_dsi_panel_desc.timing.v_size = base_cfg->height;
    s_dsi_panel_desc.timing.hsync_pulse_width = base_cfg->hsync_pulse_width;
    s_dsi_panel_desc.timing.vsync_pulse_width = base_cfg->vsync_pulse_width;
    s_dsi_panel_desc.timing.hsync_back_porch = base_cfg->hsync_back_porch;
    s_dsi_panel_desc.timing.hsync_front_porch = base_cfg->hsync_front_porch;
    s_dsi_panel_desc.timing.vsync_back_porch = base_cfg->vsync_back_porch;
    s_dsi_panel_desc.timing.vsync_front_porch = base_cfg->vsync_front_porch;
    /* Panel reset and init-command sequences are issued by the upper Tuya display
     * driver via tkl_mipi_dsi_tx_cmd(); keep the BK-side hooks disabled so this
     * adapter behaves as before (no panel reset GPIO, no auto init cmds). */
    s_dsi_panel_desc.init_cmds = NULL;
    s_dsi_panel_desc.reset = NULL;
    s_dsi_panel_desc.init = NULL;

    os_memset(&panel_cfg, 0, sizeof(panel_cfg));
    panel_cfg.reset_pin = -1;   /* reset GPIO managed externally */

    ret = bk_lcd_mipi_panel_new(s_dsi_runtime.dis_bus_handle, &panel_cfg,
                                &s_dsi_panel_desc, &s_dpu_runtime.panel_handle);
    if (ret != OPRT_OK) {
        goto err;
    }

    os_memset(&lcd_cfg, 0, sizeof(lcd_cfg));
    lcd_cfg.video.enable = true;
    lcd_cfg.video.decompress = (s_dpu_runtime.input_decompress != FALSE) ? true : false;
    lcd_cfg.video.format = pixel_format;
    lcd_cfg.video.disp_x = 0;
    lcd_cfg.video.disp_y = 0;
    lcd_cfg.video.disp_w = base_cfg->width;
    lcd_cfg.video.disp_h = base_cfg->height;

    ret = bk_display_dpu_ctlr_new(&s_dpu_runtime.dpu_ctlr_handle,
                                  s_dpu_runtime.panel_handle, &lcd_cfg);
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
    s_dpu_runtime.inited = TRUE;
    return OPRT_OK;

err:
    tkl_log_output("init mipi dpu failed: %d\r\n", ret);
    tkl_mipi_dpu_deinit();
    return ret;
}

static OPERATE_RET tkl_mipi_dpu_deinit(VOID_T)
{
    if (s_dpu_runtime.dpu_ctlr_handle != NULL) {
        bk_display_deinit(s_dpu_runtime.dpu_ctlr_handle);
        bk_display_delete(s_dpu_runtime.dpu_ctlr_handle);
    }

    if (s_dpu_runtime.panel_handle != NULL) {
        bk_lcd_panel_delete(s_dpu_runtime.panel_handle);
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
