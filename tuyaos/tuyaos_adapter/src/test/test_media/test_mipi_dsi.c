#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <components/bk_lcd_panel.h>
#include <components/bk_frame_buffer.h>
#include <common/avdk_pixel_types.h>
#include <driver/gpio.h>
#include <driver/gpio_types.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include "gpio_driver.h"

#include "cli_tuya_test.h"
#include "modules/pm.h"
#include "tkl_image_codec.h"
#include "tkl_mipi_csi.h"
#include "tkl_mipi_dsi.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_thread.h"

/* Declared in <components/bk_platform.h>; declared locally here to match the
 * convention used by tkl_system.c / test_mbedtls.c in this adapter. */
extern int bk_rand(void);

#define MIPI_TEST_TASK_NAME         "mipi_test"
#define MIPI_TEST_TASK_STACK_SIZE   (4 * 1024)
#define MIPI_TEST_RESET_PIN         GPIO_21
#define MIPI_TEST_BACKLIGHT_PIN     GPIO_20
#define MIPI_TEST_INVALID_PIN       (-1)
#define MIPI_TEST_PANEL_NAME_LEN    64
#define MIPI_TEST_BYTES_PER_PIXEL   2U
#define MIPI_TEST_FRAME_COUNT       2U
#define MIPI_TEST_REFRESH_MS        1000U

typedef struct {
    BOOL_T enable;
    TUYA_GPIO_CTRL_T reset;
    TUYA_GPIO_CTRL_T bl;
    TUYA_GPIO_CTRL_T power_ctrl;
} test_mipi_display_cfg_t;

/* TUYA_GPIO_NUM_MAX is the "not wired" sentinel, and it has to be set here:
 * zero-initialising would leave bl.pin == 0, which is a *real* pin, so
 * backlight_set() would pass its range check and unmap whatever owns GPIO 0 -
 * on this board that is P0, and the console died when 'xmipi dsi close' ran
 * before any open had called test_mipi_runtime_reset(). */
static test_mipi_display_cfg_t s_mipi_display_cfg = {
    .bl.pin = TUYA_GPIO_NUM_MAX,
    .reset.pin = TUYA_GPIO_NUM_MAX,
    .power_ctrl.pin = TUYA_GPIO_NUM_MAX,
};
static TUYA_MIPI_BASE_CFG_T s_mipi_base_cfg;
static tkl_mipi_dsi_flush_frame_t s_mipi_flush_frame;
static TKL_THREAD_HANDLE s_mipi_task = NULL;
static volatile bool s_mipi_task_running = false;
static bool s_mipi_frame_buffer_inited = false;
static char s_mipi_panel_name[MIPI_TEST_PANEL_NAME_LEN];
static uint16_t s_mipi_width = CLI_TEST_LCD_WIDTH;
static uint16_t s_mipi_height = CLI_TEST_LCD_HEIGHT;
static uint8_t *s_mipi_frames[MIPI_TEST_FRAME_COUNT] = {0};
static size_t s_mipi_frame_size = 0;
static uint8_t s_mipi_frame_index = 0;
static uint32_t s_mipi_color_state = 0x13579BDFU;
/* MP AE 预热跳帧数，由 "xmipi camera skip <n>" 设置。
 * 每个 open 都会先跑 test_mipi_camera_runtime_reset() 清空 s_mipi_camera_cfg，
 * 所以这个值单独存一份，reset 时再灌回去，否则设完就被抹掉。 */
static bool s_mipi_lcd_opened = false;
static BOOL_T s_mipi_input_decompress = FALSE;

static OPERATE_RET test_mipi_lcd_close(void);
static OPERATE_RET test_mipi_panel_resolve(const char *panel_name, const bk_display_dsi_panel_t **panel);

static void test_mipi_backlight_set(bool enable)
{
    bool active_level = true;

    if (s_mipi_display_cfg.bl.pin >= TUYA_GPIO_NUM_MAX) {
        return;
    }

    active_level = (s_mipi_display_cfg.bl.active_level == TUYA_GPIO_LEVEL_HIGH);
    gpio_dev_unmap((gpio_id_t)s_mipi_display_cfg.bl.pin);
    BK_LOG_ON_ERR(bk_gpio_enable_output((gpio_id_t)s_mipi_display_cfg.bl.pin));
    BK_LOG_ON_ERR(bk_gpio_pull_up((gpio_id_t)s_mipi_display_cfg.bl.pin));
    bk_gpio_set_capacity((gpio_id_t)s_mipi_display_cfg.bl.pin, GPIO_DRIVER_CAPACITY_3);

    if (enable == active_level) {
        bk_gpio_set_output_high((gpio_id_t)s_mipi_display_cfg.bl.pin);
    } else {
        bk_gpio_set_output_low((gpio_id_t)s_mipi_display_cfg.bl.pin);
    }
}

static OPERATE_RET test_mipi_panel_reset(void)
{
    gpio_id_t pin;
    bool active_level;

    if (s_mipi_display_cfg.reset.pin >= TUYA_GPIO_NUM_MAX) {
        return OPRT_OK;
    }

    pin = (gpio_id_t)s_mipi_display_cfg.reset.pin;
    active_level = (s_mipi_display_cfg.reset.active_level == TUYA_GPIO_LEVEL_HIGH);

    gpio_dev_unmap(pin);
    BK_LOG_ON_ERR(bk_gpio_enable_output(pin));
    bk_gpio_set_capacity(pin, GPIO_DRIVER_CAPACITY_3);

    if (active_level) {
        bk_gpio_set_output_low(pin);
        tkl_system_sleep(10);
        bk_gpio_set_output_high(pin);
    } else {
        bk_gpio_set_output_high(pin);
        tkl_system_sleep(10);
        bk_gpio_set_output_low(pin);
        tkl_system_sleep(10);
        bk_gpio_set_output_high(pin);
    }
    tkl_system_sleep(120);

    return OPRT_OK;
}

/**
 * @brief 复位面板并下发其初始化命令序列
 *
 * 模式切换由 tkl_mipi_dsi_tx_cmd() 内部兜住，这里不必关心当前是视频模式还是命令模式。
 */
static OPERATE_RET test_mipi_panel_init_with_desc(const bk_display_dsi_panel_t *panel)
{
    const lcd_mipi_init_cmd_t *init_cmds;
    OPERATE_RET ret;
    uint32_t i;

    if (panel == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = test_mipi_panel_reset();
    if (ret != OPRT_OK) {
        return ret;
    }

    init_cmds = panel->init_cmds;
    if (init_cmds == NULL) {
        return OPRT_OK;
    }

    for (i = 0; init_cmds[i].cmd != 0 || init_cmds[i].data != NULL; i++) {
        /* BK 面板表用伪命令表示延时：cmd==0 && data_len==0xFF，data 指向 1 字节毫秒数。
         * 不识别的话会被当成 255 字节参数的普通命令发出去（超过 gen_write_pkt 的 200
         * 上限，直接被丢弃），面板序列里的延时就全部失效了。 */
        if (init_cmds[i].cmd == 0 && init_cmds[i].data_len == 0xFF &&
            init_cmds[i].data != NULL) {
            tkl_system_sleep(((const uint8_t *)init_cmds[i].data)[0]);
            continue;
        }

        ret = tkl_mipi_dsi_tx_cmd(init_cmds[i].cmd,
                                  init_cmds[i].data,
                                  init_cmds[i].data_len);
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    return OPRT_OK;
}

static void test_mipi_runtime_reset(void)
{
    os_memset(&s_mipi_flush_frame, 0, sizeof(s_mipi_flush_frame));
    os_memset(s_mipi_frames, 0, sizeof(s_mipi_frames));

    s_mipi_width = CLI_TEST_LCD_WIDTH;
    s_mipi_height = CLI_TEST_LCD_HEIGHT;
    s_mipi_frame_size = 0;
    s_mipi_frame_index = 0;
    s_mipi_color_state = 0x13579BDFU;
    s_mipi_task_running = false;
    s_mipi_task = NULL;
    s_mipi_lcd_opened = false;

    os_memset(&s_mipi_display_cfg, 0, sizeof(s_mipi_display_cfg));
    os_memset(&s_mipi_base_cfg, 0, sizeof(s_mipi_base_cfg));

    s_mipi_display_cfg.enable = TRUE;
    s_mipi_display_cfg.reset.pin = MIPI_TEST_RESET_PIN;
    s_mipi_display_cfg.reset.active_level = TUYA_GPIO_LEVEL_HIGH;
    s_mipi_display_cfg.bl.pin = MIPI_TEST_BACKLIGHT_PIN;
    s_mipi_display_cfg.bl.active_level = TUYA_GPIO_LEVEL_HIGH;
    s_mipi_display_cfg.power_ctrl.pin = TUYA_GPIO_NUM_MAX;
    s_mipi_display_cfg.power_ctrl.active_level = TUYA_GPIO_LEVEL_HIGH;

    s_mipi_input_decompress = FALSE;
    s_mipi_base_cfg.pixel_fmt = TY_PIXEL_FMT_RGB565;
}

static void test_mipi_frame_buffer_init_once(void)
{
    if (s_mipi_frame_buffer_inited == false) {
        bk_frame_buffer_init();
        s_mipi_frame_buffer_inited = true;
    }
}

static uint16_t test_mipi_color_to_rgb565(uint32_t color)
{
    uint16_t r = (uint16_t)(((color >> 16) & 0xFFU) >> 3);
    uint16_t g = (uint16_t)(((color >> 8) & 0xFFU) >> 2);
    uint16_t b = (uint16_t)((color & 0xFFU) >> 3);

    return (uint16_t)((r << 11) | (g << 5) | b);
}

static size_t test_mipi_frame_size_get(void)
{
    return (size_t)s_mipi_width * s_mipi_height * MIPI_TEST_BYTES_PER_PIXEL;
}

static void test_mipi_fill_frame_rgb565(uint8_t *frame, size_t frame_size, uint32_t color)
{
    size_t i;
    uint16_t *pixels = (uint16_t *)frame;
    size_t pixel_count = frame_size / sizeof(uint16_t);
    uint16_t pixel = test_mipi_color_to_rgb565(color);

    for (i = 0; i < pixel_count; i++) {
        pixels[i] = pixel;
    }
}

static uint32_t test_mipi_mix32(uint32_t value)
{
    value ^= value >> 16;
    value *= 0x7FEB352DU;
    value ^= value >> 15;
    value *= 0x846CA68BU;
    value ^= value >> 16;

    return value;
}

static uint32_t test_mipi_random_color_get(void)
{
    uint32_t mix;
    uint32_t r5;
    uint32_t g6;
    uint32_t b5;

    s_mipi_color_state += 0x9E3779B9U ^ (uint32_t)bk_rand();
    mix = test_mipi_mix32(s_mipi_color_state);

    r5 = (mix >> 0) & 0x1FU;
    g6 = (mix >> 5) & 0x3FU;
    b5 = (mix >> 11) & 0x1FU;

    if ((r5 + g6 + b5) < 40U) {
        r5 = 12U + (r5 % 20U);
        g6 = 20U + (g6 % 44U);
        b5 = 12U + (b5 % 20U);
    }

    if (r5 < 6U) {
        r5 = 6U;
    }
    if (g6 < 8U) {
        g6 = 8U;
    }
    if (b5 < 6U) {
        b5 = 6U;
    }

    if (r5 >= g6 / 2U && r5 >= b5) {
        r5 = 31U;
    } else if (g6 >= r5 * 2U && g6 >= b5 * 2U) {
        g6 = 63U;
    } else {
        b5 = 31U;
    }

    return 0xFF000000U
           | (((r5 << 3) | (r5 >> 2)) << 16)
           | (((g6 << 2) | (g6 >> 4)) << 8)
           | ((b5 << 3) | (b5 >> 2));
}

static OPERATE_RET test_mipi_frame_buffers_prepare(void)
{
    uint8_t i;

    s_mipi_frame_size = test_mipi_frame_size_get();
    s_mipi_frame_index = 0;

    for (i = 0; i < MIPI_TEST_FRAME_COUNT; i++) {
        s_mipi_frames[i] = (uint8_t *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, (uint32_t)s_mipi_frame_size);
        if (s_mipi_frames[i] == NULL) {
            tkl_log_output( "frame[%u] alloc failed, size=%u\r\n", i, (unsigned int)s_mipi_frame_size);
            while (i > 0U) {
                i--;
                bk_frame_buffer_free(s_mipi_frames[i]);
                s_mipi_frames[i] = NULL;
            }
            s_mipi_frame_size = 0;
            return OPRT_MALLOC_FAILED;
        }
    }

    tkl_log_output( "frame buffers ready, count=%u, frame_size=%u\r\n",
            (unsigned int)MIPI_TEST_FRAME_COUNT,
            (unsigned int)s_mipi_frame_size);
    return OPRT_OK;
}

static void test_mipi_frame_buffers_release(void)
{
    uint8_t i;

    for (i = 0; i < MIPI_TEST_FRAME_COUNT; i++) {
        if (s_mipi_frames[i] != NULL) {
            bk_frame_buffer_free(s_mipi_frames[i]);
            s_mipi_frames[i] = NULL;
        }
    }

    s_mipi_frame_size = 0;
    s_mipi_frame_index = 0;
}

static OPERATE_RET test_mipi_flush_color(uint32_t color)
{
    OPERATE_RET ret;
    uint8_t *frame;

    if (s_mipi_frame_size == 0 || s_mipi_frames[s_mipi_frame_index] == NULL) {
        return OPRT_COM_ERROR;
    }

    frame = s_mipi_frames[s_mipi_frame_index];
    test_mipi_fill_frame_rgb565(frame, s_mipi_frame_size, color);

    os_memset(&s_mipi_flush_frame, 0, sizeof(s_mipi_flush_frame));
    s_mipi_flush_frame.buffer = frame;
    s_mipi_flush_frame.free_cb = NULL;

    ret = tkl_mipi_dsi_flush(&s_mipi_flush_frame);
    if (ret != OPRT_OK) {
        tkl_log_output( "flush failed: %d\r\n", ret);
        return ret;
    }

    s_mipi_frame_index = (uint8_t)((s_mipi_frame_index + 1U) % MIPI_TEST_FRAME_COUNT);
    return ret;
}

static void test_mipi_task_main(void *arg)
{
    uint32_t color;
    TKL_THREAD_HANDLE self = NULL;

    (void)arg;

    while (s_mipi_task_running == true) {
        tkl_system_sleep(MIPI_TEST_REFRESH_MS);
        if (s_mipi_task_running == false) {
            break;
        }

        color = test_mipi_random_color_get();
        if (test_mipi_flush_color(color) != OPRT_OK) {
            tkl_log_output( "periodic flush failed, stop task\r\n");
            s_mipi_task_running = false;
            break;
        }

        tkl_log_output( "random color flush: 0x%08x\r\n", (unsigned int)color);
    }

    s_mipi_task = NULL;
    if (tkl_thread_get_id(&self) == OPRT_OK && self != NULL) {
        tkl_thread_release(self);
    }
}

static OPERATE_RET test_mipi_task_start(void)
{
    OPERATE_RET ret;

    s_mipi_task_running = true;
    ret = tkl_thread_create(&s_mipi_task,
                            MIPI_TEST_TASK_NAME,
                            MIPI_TEST_TASK_STACK_SIZE,
                            TKL_THREAD_PRI_ABOVE_NORMAL,
                            test_mipi_task_main,
                            NULL);
    if (ret != OPRT_OK) {
        s_mipi_task_running = false;
        s_mipi_task = NULL;
        tkl_log_output( "create refresh task failed: %d\r\n", ret);
        return ret;
    }

    return OPRT_OK;
}

static OPERATE_RET test_mipi_task_stop(void)
{
    uint32_t wait_ms = 0;

    s_mipi_task_running = false;
    while (s_mipi_task != NULL) {
        if (wait_ms >= (MIPI_TEST_REFRESH_MS + 500U)) {
            tkl_log_output( "stop refresh task timeout\r\n");
            return OPRT_TIMEOUT;
        }

        tkl_system_sleep(10);
        wait_ms += 10U;
    }

    return OPRT_OK;
}

static OPERATE_RET test_mipi_update_geometry(void)
{
    if (s_mipi_base_cfg.width == 0U || s_mipi_base_cfg.height == 0U) {
        return OPRT_INVALID_PARM;
    }

    s_mipi_width = s_mipi_base_cfg.width;
    s_mipi_height = s_mipi_base_cfg.height;
    return OPRT_OK;
}

static OPERATE_RET test_mipi_panel_cfg_apply(void)
{
    const bk_display_dsi_panel_t *panel = NULL;
    uint32_t clk = 0;

    if (test_mipi_panel_resolve(s_mipi_panel_name, &panel) != OPRT_OK || panel == NULL) {
        return OPRT_NOT_FOUND;
    }

    /* New BK display stack no longer keeps a pre-computed clk on the timing;
     * derive the DPI pixel clock from fps * (h_total * v_total), the same way
     * bk_lcd_mipi_panel_new()/bk_display_init() compute it. */
    if (panel->fps != 0U) {
        uint32_t h_total = (uint32_t)panel->timing.h_size + panel->timing.hsync_pulse_width +
                           panel->timing.hsync_back_porch + panel->timing.hsync_front_porch;
        uint32_t v_total = (uint32_t)panel->timing.v_size + panel->timing.vsync_pulse_width +
                           panel->timing.vsync_back_porch + panel->timing.vsync_front_porch;

        clk = (uint32_t)((uint64_t)h_total * v_total * panel->fps);
    }

    s_mipi_base_cfg.clk = clk;
    s_mipi_base_cfg.n_lanes = (UINT8_T)(panel->n_lanes + 1U);
    s_mipi_base_cfg.width = panel->timing.h_size;
    s_mipi_base_cfg.height = panel->timing.v_size;
    s_mipi_base_cfg.hsync_back_porch = panel->timing.hsync_back_porch;
    s_mipi_base_cfg.hsync_front_porch = panel->timing.hsync_front_porch;
    s_mipi_base_cfg.vsync_back_porch = panel->timing.vsync_back_porch;
    s_mipi_base_cfg.vsync_front_porch = panel->timing.vsync_front_porch;
    s_mipi_base_cfg.hsync_pulse_width = panel->timing.hsync_pulse_width;
    s_mipi_base_cfg.vsync_pulse_width = panel->timing.vsync_pulse_width;

    return OPRT_OK;
}

static OPERATE_RET test_mipi_panel_resolve(const char *panel_name, const bk_display_dsi_panel_t **panel)
{
    const bk_display_dsi_panel_t *panel_list[1] = {0};

    if (panel == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (panel_name != NULL && panel_name[0] != '\0') {
        *panel = bk_lcd_find_mipi_panel_by_name(panel_name);
        if (*panel != NULL) {
            return OPRT_OK;
        }
    }

    if (bk_lcd_get_mipi_panel_list(panel_list, 1) > 0U) {
        *panel = panel_list[0];
        return OPRT_OK;
    }

    *panel = NULL;
    return OPRT_NOT_FOUND;
}

static OPERATE_RET test_mipi_lcd_ensure_open(const char *panel_name)
{
    const bk_display_dsi_panel_t *panel = NULL;
    OPERATE_RET ret;

    if (s_mipi_lcd_opened == true) {
        return OPRT_OK;
    }

    if (panel_name != NULL && panel_name[0] != '\0') {
        snprintf(s_mipi_panel_name, sizeof(s_mipi_panel_name), "%s", panel_name);
    } else {
        snprintf(s_mipi_panel_name, sizeof(s_mipi_panel_name), "%s", CLI_TEST_MIPI_PANEL_NAME);
    }

    ret = test_mipi_panel_resolve(s_mipi_panel_name, &panel);
    if (ret != OPRT_OK || panel == NULL) {
        tkl_log_output("resolve panel failed: %d\r\n", ret);
        test_mipi_runtime_reset();
        return (ret == OPRT_OK) ? OPRT_NOT_FOUND : ret;
    }

    test_mipi_frame_buffer_init_once();
    ret = test_mipi_panel_cfg_apply();
    if (ret != OPRT_OK) {
        tkl_log_output("apply panel cfg failed: %d\r\n", ret);
        test_mipi_runtime_reset();
        return ret;
    }

    ret = tkl_mipi_dsi_set_input_mode(s_mipi_input_decompress);
    if (ret != OPRT_OK) {
        tkl_log_output("set input mode failed: %d\r\n", ret);
        test_mipi_runtime_reset();
        return ret;
    }

    ret = tkl_mipi_dsi_init(&s_mipi_base_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("panel io open failed: %d\r\n", ret);
        test_mipi_runtime_reset();
        return ret;
    }

    ret = test_mipi_panel_init_with_desc(panel);
    if (ret != OPRT_OK) {
        tkl_log_output("panel init failed: %d\r\n", ret);
        tkl_mipi_dsi_deinit();
        test_mipi_runtime_reset();
        return ret;
    }

    ret = test_mipi_update_geometry();
    if (ret != OPRT_OK) {
        tkl_log_output( "update geometry failed: %d\r\n", ret);
        tkl_mipi_dsi_deinit();
        test_mipi_runtime_reset();
        return ret;
    }

    s_mipi_lcd_opened = true;

    test_mipi_backlight_set(true);

    tkl_log_output( "mipi open ok, panel=%s, size=%ux%u\r\n",
            s_mipi_panel_name[0] != '\0' ? s_mipi_panel_name : "auto",
            s_mipi_width, s_mipi_height);

    return OPRT_OK;
}

static OPERATE_RET test_mipi_lcd_ensure_open_with_config(const char *panel_name,
                                                         DISPLAY_PIXEL_FORMAT_E display_format,
                                                         BOOL_T decompress)
{
    test_mipi_runtime_reset();
    s_mipi_base_cfg.pixel_fmt = display_format;
    s_mipi_input_decompress = decompress;
    return test_mipi_lcd_ensure_open(panel_name);
}

static OPERATE_RET test_mipi_lcd_open(const char *panel_name)
{
    OPERATE_RET ret;

    if (s_mipi_lcd_opened == true) {
        tkl_log_output( "mipi lcd already open\r\n");
        return OPRT_OK;
    }

    test_mipi_runtime_reset();
    ret = test_mipi_lcd_ensure_open(panel_name);
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_log_output( "mipi lcd open only, refresh task is idle\r\n");
    return OPRT_OK;
}

static OPERATE_RET test_mipi_lcd_open_gpu(const char *panel_name, BOOL_T decompress)
{
    OPERATE_RET ret;

    if (s_mipi_lcd_opened == true &&
        s_mipi_base_cfg.pixel_fmt == TY_PIXEL_FMT_ARGB8888 &&
        s_mipi_input_decompress == decompress) {
        tkl_log_output( "mipi lcd already open in gpu mode\r\n");
        return OPRT_OK;
    }

    if (s_mipi_lcd_opened == true) {
        ret = test_mipi_lcd_close();
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    ret = test_mipi_lcd_ensure_open_with_config(panel_name, TY_PIXEL_FMT_ARGB8888, decompress);
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_log_output(
            "mipi lcd open for gpu, panel=%s, size=%ux%u, fmt=%u, decompress=%u\r\n",
            (s_mipi_panel_name[0] != '\0') ? s_mipi_panel_name : "auto",
            s_mipi_width,
            s_mipi_height,
            (unsigned int)s_mipi_base_cfg.pixel_fmt,
            (unsigned int)decompress);
    return OPRT_OK;
}

static OPERATE_RET test_mipi_lcd_flush(const char *panel_name)
{
    OPERATE_RET ret;
    uint32_t color;

    (void)panel_name;

    if (s_mipi_lcd_opened != true) {
        tkl_log_output( "mipi lcd is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_mipi_frame_size == 0U || s_mipi_frames[0] == NULL) {
        ret = test_mipi_frame_buffers_prepare();
        if (ret != OPRT_OK) {
            tkl_log_output( "prepare frame buffers failed: %d\r\n", ret);
            return ret;
        }
    }

    color = test_mipi_random_color_get();
    ret = test_mipi_flush_color(color);
    if (ret != OPRT_OK) {
        tkl_log_output( "initial flush failed: %d\r\n", ret);
        return ret;
    }

    tkl_log_output( "flush path=direct, fmt=RGB565, frame_size=%u, buffers=%u, first_color=0x%08x\r\n",
            (unsigned int)test_mipi_frame_size_get(),
            (unsigned int)MIPI_TEST_FRAME_COUNT,
            (unsigned int)color);

    if (s_mipi_task_running == true) {
        tkl_log_output( "random pure-color refresh already running\r\n");
        return OPRT_OK;
    }

    ret = test_mipi_task_start();
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_log_output( "random pure-color refresh started, interval=%u ms\r\n",
            (unsigned int)MIPI_TEST_REFRESH_MS);
    return OPRT_OK;
}

static OPERATE_RET test_mipi_lcd_stop_flush(void)
{
    OPERATE_RET ret;

    if (s_mipi_lcd_opened != true) {
        tkl_log_output( "mipi lcd is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_mipi_task_running != true && s_mipi_task == NULL) {
        tkl_log_output( "random pure-color refresh already stopped\r\n");
        return OPRT_OK;
    }

    ret = test_mipi_task_stop();
    if (ret != OPRT_OK) {
        tkl_log_output( "refresh task stop failed: %d\r\n", ret);
        return ret;
    }

    tkl_log_output( "random pure-color refresh stopped\r\n");
    return OPRT_OK;
}

static OPERATE_RET test_mipi_lcd_close(void)
{
    OPERATE_RET ret;
    OPERATE_RET stop_ret;

    /* Nothing was opened, so there is nothing to close - and running the
     * teardown anyway is actively harmful: backlight_set() would act on the
     * not-yet-configured pin, and tkl_mipi_dsi_deinit() would tear down DPU
     * state it never built. Idempotent by contract, so report success. */
    if (s_mipi_lcd_opened != true) {
        tkl_log_output("mipi close: not open, nothing to do\r\n");
        return OPRT_OK;
    }

    if (s_mipi_task_running == true || s_mipi_task != NULL) {
        stop_ret = test_mipi_task_stop();
        if (stop_ret != OPRT_OK) {
            tkl_log_output( "refresh task stop failed: %d\r\n", stop_ret);
            return stop_ret;
        }
    }

    test_mipi_backlight_set(false);
    ret = tkl_mipi_dsi_deinit();
    test_mipi_frame_buffers_release();
    test_mipi_runtime_reset();
    s_mipi_panel_name[0] = '\0';

    tkl_log_output( "mipi close %s\r\n", (ret == OPRT_OK) ? "ok" : "failed");
    return ret;
}

BOOL_T test_mipi_lcd_is_open(void)
{
    return (s_mipi_lcd_opened == true) ? TRUE : FALSE;
}

void test_mipi_lcd_get_geometry(UINT16_T *width, UINT16_T *height)
{
    if (width != NULL) {
        *width = s_mipi_width;
    }

    if (height != NULL) {
        *height = s_mipi_height;
    }
}

/*
 * Cycle-accurate timing for the paste report. The DWT counter ticks at the CPU
 * clock; the frequency comes from the PM module (the max frequency voted in),
 * so the conversion survives a clock-policy change. An unlisted setting falls
 * back to the BK7259 AP's top speed, which is what a display-active system
 * normally runs at.
 */
#define MIPI_TEST_PROF_CPU_MHZ_FALLBACK  480U

static uint32_t test_mipi_dwt_cycles(void)
{
    extern void dwt_init_cycle_counter(void);
    extern uint32_t dwt_get_cycle_counter_val(void);
    static bool s_armed = false;

    if (s_armed == false) {
        s_armed = true;
        dwt_init_cycle_counter();
    }

    return dwt_get_cycle_counter_val();
}

static uint32_t test_mipi_cpu_mhz(void)
{
    static uint32_t mhz = 0U;

    if (mhz == 0U) {
        switch (bk_pm_current_max_cpu_freq_get()) {
        case PM_CPU_FRQ_80M:  mhz = 80U;  break;
        case PM_CPU_FRQ_120M: mhz = 120U; break;
        case PM_CPU_FRQ_160M: mhz = 160U; break;
        case PM_CPU_FRQ_240M: mhz = 240U; break;
        case PM_CPU_FRQ_320M: mhz = 320U; break;
        case PM_CPU_FRQ_480M: mhz = 480U; break;
        default:              mhz = MIPI_TEST_PROF_CPU_MHZ_FALLBACK; break;
        }
    }

    return mhz;
}

static uint32_t test_mipi_cycles_to_us(uint32_t cycles)
{
    return cycles / test_mipi_cpu_mhz();
}

/*
 * Compose two random colours with tkl_image_codec_memcpy() and push the result to the
 * panel: the destination frame is one solid colour, and the middle of the
 * source frame - a second solid colour - is pasted into its middle. Both
 * windows are off-origin, so the read offset and the paste offset are both
 * exercised, and the pasted rectangle is obvious on the screen.
 *
 * Reuses the two flush-pool frames (RGB565, panel geometry). The periodic
 * refresh is stopped first - it would replace the composed frame with a fresh
 * colour within a second.
 */
static OPERATE_RET test_mipi_paste_mid(uint16_t win_w, uint16_t win_h)
{
    OPERATE_RET ret;
    uint32_t color_src;
    uint32_t color_dst;
    uint32_t t0;
    uint32_t t1;
    uint16_t x;
    uint16_t y;
    TUYA_IMAGE_CODEC_COPY_BUF_T s;
    TUYA_IMAGE_CODEC_COPY_BUF_T d;

    if (s_mipi_lcd_opened != true) {
        tkl_log_output("mipi lcd is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    /* idempotent: succeeds straight away when the refresh is not running */
    ret = test_mipi_task_stop();
    if (ret != OPRT_OK) {
        tkl_log_output("refresh task stop failed: %d\r\n", ret);
        return ret;
    }

    if (s_mipi_frame_size == 0U || s_mipi_frames[0] == NULL || s_mipi_frames[1] == NULL) {
        ret = test_mipi_frame_buffers_prepare();
        if (ret != OPRT_OK) {
            tkl_log_output("prepare frame buffers failed: %d\r\n", ret);
            return ret;
        }
    }

    /* zero or oversized window means "half the panel, centred". Two alignment
     * layers apply here: RGB565 needs an even origin and an even width at the
     * API (4-byte pixel groups), and the GDMA leg additionally wants every row
     * start 16-byte aligned - at 2 Bpp that makes the origin a multiple of 8.
     * Skipping the second layer would not error, it would silently drop the
     * copy onto the CPU leg. */
    if (win_w == 0U || win_h == 0U ||
        win_w > s_mipi_width || win_h > s_mipi_height) {
        win_w = (uint16_t)(s_mipi_width / 2U);
        win_h = (uint16_t)(s_mipi_height / 2U);
    }

    win_w = (uint16_t)(win_w & (uint16_t)~1U);
    if (win_w == 0U) {
        win_w = 2U;
    }
    x = (uint16_t)(((s_mipi_width - win_w) / 2U) & (uint16_t)~7U);
    y = (uint16_t)((s_mipi_height - win_h) / 2U);

    color_src = test_mipi_random_color_get();
    color_dst = test_mipi_random_color_get();
    test_mipi_fill_frame_rgb565(s_mipi_frames[0], s_mipi_frame_size, color_src);
    test_mipi_fill_frame_rgb565(s_mipi_frames[1], s_mipi_frame_size, color_dst);

    /* Both operands describe a whole frame; only the windows differ. That is
     * the picture-in-picture shape tkl_image_codec_memcpy() exists for - the caller
     * owns both buffers and the geometry stays in the caller's hands. */
    os_memset(&s, 0, sizeof(s));
    s.buf.buffer = s_mipi_frames[0];
    s.buf.length = (UINT32_T)s_mipi_frame_size;
    s.buf.format = TUYA_FRAME_FMT_RGB565;
    s.buf.width = s_mipi_width;
    s.buf.height = s_mipi_height;
    s.region.x = x;
    s.region.y = y;
    s.region.width = win_w;
    s.region.height = win_h;

    os_memset(&d, 0, sizeof(d));
    d.buf.buffer = s_mipi_frames[1];
    d.buf.length = (UINT32_T)s_mipi_frame_size;
    d.buf.format = TUYA_FRAME_FMT_RGB565;
    d.buf.width = s_mipi_width;
    d.buf.height = s_mipi_height;
    /* same window size on purpose: no scaling in this interface */
    d.region.x = x;
    d.region.y = y;
    d.region.width = win_w;
    d.region.height = win_h;

    t0 = test_mipi_dwt_cycles();
    ret = tkl_image_codec_memcpy(&s, &d);
    t1 = test_mipi_dwt_cycles();
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_image_codec_memcpy failed: %d\r\n", ret);
        return ret;
    }

    {
        uint32_t cycles = t1 - t0;
        uint32_t moved = (uint32_t)win_w * win_h * MIPI_TEST_BYTES_PER_PIXEL;
        uint32_t us = test_mipi_cycles_to_us(cycles);
        uint32_t mhz = test_mipi_cpu_mhz();

        if (cycles != 0U) {
            uint64_t mbps10 = (uint64_t)moved * mhz * 10U / cycles;

            tkl_log_output("paste copy: %u B in %u us (cpu %u MHz, %u.%u MB/s)\r\n",
                           (unsigned int)moved,
                           (unsigned int)us,
                           (unsigned int)mhz,
                           (unsigned int)(mbps10 / 10U),
                           (unsigned int)(mbps10 % 10U));
        }
    }

    os_memset(&s_mipi_flush_frame, 0, sizeof(s_mipi_flush_frame));
    s_mipi_flush_frame.buffer = s_mipi_frames[1];
    s_mipi_flush_frame.free_cb = NULL;

    ret = tkl_mipi_dsi_flush(&s_mipi_flush_frame);
    if (ret != OPRT_OK) {
        tkl_log_output("flush failed: %d\r\n", ret);
        return ret;
    }

    tkl_log_output("paste ok: canvas=0x%08x patch=0x%08x window=%ux%u at (%u,%u) %u B\r\n",
                   (unsigned int)color_dst,
                   (unsigned int)color_src,
                   (unsigned int)win_w,
                   (unsigned int)win_h,
                   (unsigned int)x,
                   (unsigned int)y,
                   (unsigned int)((UINT32_T)win_w * win_h * MIPI_TEST_BYTES_PER_PIXEL));

    return OPRT_OK;
}

void cli_tuya_mipi_dsi_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret;
    const char *cmd = NULL;
    const char *panel_name = NULL;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xmipi dsi <open [panel]|open_gpu [panel] [compress]|flush|paste [w] [h]|"
                 "stop_flush|close>");
        return;
    }

    cmd = argv[1];

    if (argc >= 3) {
        panel_name = argv[2];
    }

    if (os_strcmp(cmd, "open") == 0) {
        ret = test_mipi_lcd_open(panel_name);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xmipi dsi open %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
    } else if (os_strcmp(cmd, "open_gpu") == 0) {
        BOOL_T decompress = FALSE;
        char *endptr = NULL;

        if (argc >= 3) {
            unsigned long value = os_strtoul(argv[2], &endptr, 10);
            if (endptr != NULL && endptr[0] == '\0') {
                panel_name = NULL;
                decompress = (value != 0U) ? TRUE : FALSE;
            }
        }

        if (argc >= 4) {
            decompress = (os_strtoul(argv[3], NULL, 10) != 0U) ? TRUE : FALSE;
        }

        ret = test_mipi_lcd_open_gpu(panel_name, decompress);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xmipi dsi open_gpu %s (%d), panel=%s, size=%ux%u, decompress=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (s_mipi_panel_name[0] != '\0') ? s_mipi_panel_name : "auto",
                 s_mipi_width,
                 s_mipi_height,
                 (unsigned int)decompress);
    } else if (os_strcmp(cmd, "flush") == 0) {
        ret = test_mipi_lcd_flush(panel_name);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xmipi dsi flush %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
    } else if (os_strcmp(cmd, "paste") == 0) {
        uint16_t win_w = 0U;
        uint16_t win_h = 0U;

        if (argc >= 3) {
            win_w = (uint16_t)os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            win_h = (uint16_t)os_strtoul(argv[3], NULL, 10);
        }

        ret = test_mipi_paste_mid(win_w, win_h);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xmipi dsi paste %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
    } else if (os_strcmp(cmd, "stop_flush") == 0) {
        ret = test_mipi_lcd_stop_flush();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xmipi dsi stop_flush %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
    } else if (os_strcmp(cmd, "close") == 0) {
        ret = test_mipi_lcd_close();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xmipi dsi close %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xmipi dsi <open [panel]|open_gpu [panel] [compress]|flush|paste [w] [h]|"
                 "stop_flush|close>");
    }
}
