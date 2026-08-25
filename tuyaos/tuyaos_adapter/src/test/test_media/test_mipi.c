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
#include "tkl_camera_mipi.h"
#include "tkl_mipi_dsi.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_thread.h"

#define MIPI_TEST_TASK_NAME         "mipi_test"
#define MIPI_TEST_TASK_STACK_SIZE   (4 * 1024)
#define MIPI_TEST_RESET_PIN         GPIO_16
#define MIPI_TEST_BACKLIGHT_PIN     GPIO_20
#define MIPI_TEST_INVALID_PIN       (-1)
#define MIPI_TEST_PANEL_NAME_LEN    64
#define MIPI_TEST_BYTES_PER_PIXEL   2U
#define MIPI_TEST_FRAME_COUNT       2U
#define MIPI_TEST_REFRESH_MS        1000U
#define MIPI_TEST_CAMERA_PIN_SCL    GPIO_65
#define MIPI_TEST_CAMERA_PIN_SDA    GPIO_64
#define MIPI_TEST_CAMERA_I2C_ID     1U
#define MIPI_TEST_CAMERA_PIN_RESET  GPIO_60
#define MIPI_TEST_CAMERA_PIN_PWDN   (-1)
#define MIPI_TEST_CAMERA_PIN_XCLK   GPIO_59
#define MIPI_TEST_CAMERA_SENSOR_W   1920U
#define MIPI_TEST_CAMERA_SENSOR_H   1080U
#define MIPI_TEST_CAMERA_SENSOR_FPS 15U
#define MIPI_TEST_CAMERA_OUT_BUF_CNT 3U
#define MIPI_TEST_CAMERA_HEAD_BYTES 8U
#define MIPI_TEST_FACE_SP_WIDTH     56U
#define MIPI_TEST_FACE_SP_HEIGHT    56U
#define MIPI_TEST_FACE_SP_FORMAT    BK_PIXEL_FORMAT_RGB888
#define MIPI_TEST_MIRROR_SETTLE_FRAMES 2U
#define MIPI_TEST_MIRROR_EDGE_COLS     8U
#define MIPI_TEST_CAMERA_USAGE \
    "Usage: xmipi camera <open [sw sh fps ow oh]|open_gpu [sw sh fps ow oh]|open_gpu_sp [sw sh fps ow oh spw sph]|start|switch <ow> <oh>|switch_sp <spw> <sph>|get_frame [mp|sp]|mirror [hmirror vflip]|mirror_test|luminance|luminance_test [n ms]|cproc|gray [0|1]|night [0|1]|skip [n]|stop|close>"
#define MIPI_TEST_LUMINANCE_MAX         255000U
#define MIPI_TEST_LUMINANCE_DEFAULT_CNT 5U
#define MIPI_TEST_LUMINANCE_DEFAULT_MS  200U

typedef struct {
    BOOL_T enable;
    TUYA_GPIO_CTRL_T reset;
    TUYA_GPIO_CTRL_T bl;
    TUYA_GPIO_CTRL_T power_ctrl;
} test_mipi_display_cfg_t;

static test_mipi_display_cfg_t s_mipi_display_cfg;
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
static TUYA_CAMERA_MIPI_CFG_T s_mipi_camera_cfg;
/* MP AE 预热跳帧数，由 "xmipi camera skip <n>" 设置。
 * 每个 open 都会先跑 test_mipi_camera_runtime_reset() 清空 s_mipi_camera_cfg，
 * 所以这个值单独存一份，reset 时再灌回去，否则设完就被抹掉。 */
static uint8_t s_mipi_camera_skip_frames = 0;
static bool s_mipi_camera_inited = false;
static bool s_mipi_camera_started = false;
static uint8_t *s_mipi_camera_frame = NULL;
static uint32_t s_mipi_camera_frame_size = 0;
static uint32_t s_mipi_camera_last_frame_len = 0;
static uint16_t s_mipi_camera_last_frame_width = 0;
static uint16_t s_mipi_camera_last_frame_height = 0;
static uint32_t s_mipi_camera_last_frame_format = BK_PIXEL_FORMAT_INVALID;
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

static void test_mipi_camera_runtime_reset(void)
{
    os_memset(&s_mipi_camera_cfg, 0, sizeof(s_mipi_camera_cfg));

    s_mipi_camera_cfg.pin_scl = MIPI_TEST_CAMERA_PIN_SCL;
    s_mipi_camera_cfg.pin_sda = MIPI_TEST_CAMERA_PIN_SDA;
    s_mipi_camera_cfg.i2c_id = MIPI_TEST_CAMERA_I2C_ID;
    s_mipi_camera_cfg.pin_reset = MIPI_TEST_CAMERA_PIN_RESET;
    s_mipi_camera_cfg.pin_pwdn = MIPI_TEST_CAMERA_PIN_PWDN;
    s_mipi_camera_cfg.pin_xclk = MIPI_TEST_CAMERA_PIN_XCLK;
    s_mipi_camera_cfg.sensor_width = MIPI_TEST_CAMERA_SENSOR_W;
    s_mipi_camera_cfg.sensor_height = MIPI_TEST_CAMERA_SENSOR_H;
    s_mipi_camera_cfg.sensor_fps = MIPI_TEST_CAMERA_SENSOR_FPS;
    s_mipi_camera_cfg.sensor_input_format = BK_PIXEL_FORMAT_BGGR10;
    s_mipi_camera_cfg.output_width = MIPI_TEST_CAMERA_SENSOR_W;
    s_mipi_camera_cfg.output_height = MIPI_TEST_CAMERA_SENSOR_H;
    s_mipi_camera_cfg.output_format = BK_PIXEL_FORMAT_NV12;
    s_mipi_camera_cfg.output_buf_cnt = MIPI_TEST_CAMERA_OUT_BUF_CNT;
    s_mipi_camera_cfg.enable_flexa = FALSE;
    s_mipi_camera_cfg.work_mode = 0U;
    s_mipi_camera_cfg.read_timeout_ms = 1000U;
    /* 跨 reset 保留，见 s_mipi_camera_skip_frames 的说明 */
    s_mipi_camera_cfg.skip_frames = s_mipi_camera_skip_frames;

    s_mipi_camera_inited = false;
    s_mipi_camera_started = false;
    s_mipi_camera_frame = NULL;
    s_mipi_camera_frame_size = 0;
    s_mipi_camera_last_frame_len = 0;
    s_mipi_camera_last_frame_width = 0;
    s_mipi_camera_last_frame_height = 0;
    s_mipi_camera_last_frame_format = BK_PIXEL_FORMAT_INVALID;
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

static void test_mipi_camera_frame_release(void)
{
    if (s_mipi_camera_frame != NULL) {
        bk_frame_buffer_free(s_mipi_camera_frame);
        s_mipi_camera_frame = NULL;
    }

    s_mipi_camera_frame_size = 0;
    s_mipi_camera_last_frame_len = 0;
    s_mipi_camera_last_frame_width = 0;
    s_mipi_camera_last_frame_height = 0;
    s_mipi_camera_last_frame_format = BK_PIXEL_FORMAT_INVALID;
}

static uint32_t test_mipi_camera_frame_size_get(uint8_t channel_id)
{
    if (channel_id == TUYA_CAMERA_MIPI_CHANNEL_SP) {
        /* ISP emits RGB888/BGR888 as RGBX (4 bytes/pixel); must match
         * tkl_camera_mipi_sp_frame_size_get() in the driver. */
        if (s_mipi_camera_cfg.sp_format == BK_PIXEL_FORMAT_RGB888 ||
            s_mipi_camera_cfg.sp_format == BK_PIXEL_FORMAT_BGR888) {
            return (uint32_t)s_mipi_camera_cfg.sp_width * s_mipi_camera_cfg.sp_height * 4U;
        }
        return bk_image_size_get(s_mipi_camera_cfg.sp_width,
                                 s_mipi_camera_cfg.sp_height,
                                 (bk_pixel_format_t)s_mipi_camera_cfg.sp_format);
    }

    return bk_image_size_get(s_mipi_camera_cfg.output_width,
                             s_mipi_camera_cfg.output_height,
                             (bk_pixel_format_t)s_mipi_camera_cfg.output_format);
}

static OPERATE_RET test_mipi_camera_frame_prepare(uint8_t channel_id)
{
    uint32_t frame_size = test_mipi_camera_frame_size_get(channel_id);

    if (frame_size == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (s_mipi_camera_frame != NULL && s_mipi_camera_frame_size == frame_size) {
        return OPRT_OK;
    }

    test_mipi_camera_frame_release();

    s_mipi_camera_frame = (uint8_t *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, frame_size);
    if (s_mipi_camera_frame == NULL) {
        tkl_log_output( "camera frame alloc failed, size=%u\r\n", (unsigned int)frame_size);
        return OPRT_MALLOC_FAILED;
    }

    s_mipi_camera_frame_size = frame_size;
    return OPRT_OK;
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

static OPERATE_RET test_mipi_camera_mipi_open(uint16_t sensor_width,
                                              uint16_t sensor_height,
                                              uint8_t sensor_fps,
                                              uint16_t output_width,
                                              uint16_t output_height)
{
    OPERATE_RET ret;

    if (s_mipi_camera_inited == true) {
        tkl_log_output( "mipi camera already open\r\n");
        return OPRT_OK;
    }

    test_mipi_camera_runtime_reset();
    s_mipi_camera_cfg.sensor_width = sensor_width;
    s_mipi_camera_cfg.sensor_height = sensor_height;
    s_mipi_camera_cfg.sensor_fps = sensor_fps;
    s_mipi_camera_cfg.output_width = output_width;
    s_mipi_camera_cfg.output_height = output_height;

    test_mipi_frame_buffer_init_once();

    ret = tkl_camera_mipi_init(&s_mipi_camera_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output( "camera open failed: %d\r\n", ret);
        test_mipi_camera_runtime_reset();
        return ret;
    }

    s_mipi_camera_inited = true;
    tkl_log_output( "camera open ok, sensor=%ux%u@%u, output=%ux%u, fmt=%u\r\n",
            s_mipi_camera_cfg.sensor_width,
            s_mipi_camera_cfg.sensor_height,
            s_mipi_camera_cfg.sensor_fps,
            s_mipi_camera_cfg.output_width,
            s_mipi_camera_cfg.output_height,
            (unsigned int)s_mipi_camera_cfg.output_format);
    return OPRT_OK;
}

static OPERATE_RET test_mipi_camera_mipi_open_gpu(uint16_t sensor_width,
                                                  uint16_t sensor_height,
                                                  uint8_t sensor_fps,
                                                  uint16_t output_width,
                                                  uint16_t output_height)
{
    OPERATE_RET ret;

    if (s_mipi_camera_inited == true) {
        tkl_log_output( "mipi camera already open\r\n");
        return OPRT_OK;
    }

    test_mipi_camera_runtime_reset();
    s_mipi_camera_cfg.sensor_width = sensor_width;
    s_mipi_camera_cfg.sensor_height = sensor_height;
    s_mipi_camera_cfg.sensor_fps = sensor_fps;
    s_mipi_camera_cfg.output_width = output_width;
    s_mipi_camera_cfg.output_height = output_height;
    s_mipi_camera_cfg.enable_flexa = TRUE;
    s_mipi_camera_cfg.work_mode = 1U;

    test_mipi_frame_buffer_init_once();

    ret = tkl_camera_mipi_init(&s_mipi_camera_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output( "camera open failed: %d\r\n", ret);
        test_mipi_camera_runtime_reset();
        return ret;
    }

    s_mipi_camera_inited = true;
    tkl_log_output( "camera open ok, sensor=%ux%u@%u, output=%ux%u, fmt=%u\r\n",
            s_mipi_camera_cfg.sensor_width,
            s_mipi_camera_cfg.sensor_height,
            s_mipi_camera_cfg.sensor_fps,
            s_mipi_camera_cfg.output_width,
            s_mipi_camera_cfg.output_height,
            (unsigned int)s_mipi_camera_cfg.output_format);
    return OPRT_OK;
}

static OPERATE_RET test_mipi_camera_mipi_open_gpu_sp(uint16_t sensor_width,
                                                     uint16_t sensor_height,
                                                     uint8_t sensor_fps,
                                                     uint16_t output_width,
                                                     uint16_t output_height,
                                                     uint16_t sp_width,
                                                     uint16_t sp_height,
                                                     uint32_t sp_format)
{
    OPERATE_RET ret;

    if (s_mipi_camera_inited == true) {
        tkl_log_output( "mipi camera already open\r\n");
        return OPRT_OK;
    }

    test_mipi_camera_runtime_reset();
    s_mipi_camera_cfg.sensor_width = sensor_width;
    s_mipi_camera_cfg.sensor_height = sensor_height;
    s_mipi_camera_cfg.sensor_fps = sensor_fps;
    s_mipi_camera_cfg.output_width = output_width;
    s_mipi_camera_cfg.output_height = output_height;
    s_mipi_camera_cfg.enable_flexa = TRUE;
    s_mipi_camera_cfg.work_mode = 1U;
    s_mipi_camera_cfg.sp_enable = TRUE;
    s_mipi_camera_cfg.sp_width = sp_width;
    s_mipi_camera_cfg.sp_height = sp_height;
    s_mipi_camera_cfg.sp_format = sp_format;
    s_mipi_camera_cfg.sp_buf_cnt = 2U;
    s_mipi_camera_cfg.sp_enable_flexa = FALSE;
    s_mipi_camera_cfg.sp_work_mode = 0U;

    test_mipi_frame_buffer_init_once();

    ret = tkl_camera_mipi_init(&s_mipi_camera_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output( "camera open failed: %d\r\n", ret);
        test_mipi_camera_runtime_reset();
        return ret;
    }

    s_mipi_camera_inited = true;
    tkl_log_output(
            "camera open_gpu_sp ok, sensor=%ux%u@%u, mp=%ux%u fmt=%u, sp=%ux%u fmt=%u\r\n",
            s_mipi_camera_cfg.sensor_width,
            s_mipi_camera_cfg.sensor_height,
            s_mipi_camera_cfg.sensor_fps,
            s_mipi_camera_cfg.output_width,
            s_mipi_camera_cfg.output_height,
            (unsigned int)s_mipi_camera_cfg.output_format,
            s_mipi_camera_cfg.sp_width,
            s_mipi_camera_cfg.sp_height,
            (unsigned int)s_mipi_camera_cfg.sp_format);
    return OPRT_OK;
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

BOOL_T test_mipi_camera_is_inited(void)
{
    return (s_mipi_camera_inited == true) ? TRUE : FALSE;
}

BOOL_T test_mipi_camera_is_started(void)
{
    return (s_mipi_camera_started == true) ? TRUE : FALSE;
}

BOOL_T test_mipi_camera_flexa_enabled(void)
{
    return (s_mipi_camera_cfg.enable_flexa != FALSE) ? TRUE : FALSE;
}

OPERATE_RET test_mipi_camera_get_sp_info(UINT16_T *width,
                                        UINT16_T *height,
                                        UINT32_T *format,
                                        UINT32_T *timeout_ms)
{
    if (s_mipi_camera_cfg.sp_enable != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_mipi_camera_cfg.sp_enable_flexa != FALSE || s_mipi_camera_cfg.sp_work_mode != 0U) {
        return OPRT_NOT_SUPPORTED;
    }

    if (width != NULL) {
        *width = s_mipi_camera_cfg.sp_width;
    }

    if (height != NULL) {
        *height = s_mipi_camera_cfg.sp_height;
    }

    if (format != NULL) {
        *format = s_mipi_camera_cfg.sp_format;
    }

    if (timeout_ms != NULL) {
        *timeout_ms = s_mipi_camera_cfg.read_timeout_ms;
    }

    return OPRT_OK;
}

OPERATE_RET test_mipi_camera_get_flexa_source(TUYA_CAMERA_MIPI_FLEXA_SOURCE_T *source)
{
    return tkl_camera_mipi_get_flexa_source(source);
}
static OPERATE_RET test_mipi_camera_mipi_close(void)
{
    OPERATE_RET ret = OPRT_OK;

    if (s_mipi_camera_started == true) {
        ret = tkl_camera_mipi_stop();
        if (ret != OPRT_OK) {
            tkl_log_output( "camera stop before close failed: %d\r\n", ret);
            return ret;
        }
        s_mipi_camera_started = false;
    }

    if (s_mipi_camera_inited == true) {
        ret = tkl_camera_mipi_deinit();
        if (ret != OPRT_OK) {
            tkl_log_output( "camera close failed: %d\r\n", ret);
            return ret;
        }
    }

    test_mipi_camera_frame_release();
    test_mipi_camera_runtime_reset();
    tkl_log_output( "camera close ok\r\n");
    return OPRT_OK;
}
static OPERATE_RET test_mipi_camera_mipi_start(void)
{
    OPERATE_RET ret;

    if (s_mipi_camera_inited != true) {
        tkl_log_output( "mipi camera is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_mipi_camera_started == true) {
        tkl_log_output( "mipi camera already started\r\n");
        return OPRT_OK;
    }

    ret = tkl_camera_mipi_start();
    if (ret != OPRT_OK) {
        tkl_log_output( "camera start failed: %d\r\n", ret);
        return ret;
    }

    s_mipi_camera_started = true;
    tkl_log_output( "camera start ok\r\n");
    return OPRT_OK;
}
static OPERATE_RET test_mipi_camera_mipi_stop(void)
{
    OPERATE_RET ret;

    if (s_mipi_camera_inited != true) {
        return OPRT_OK;
    }

    if (s_mipi_camera_started != true) {
        tkl_log_output( "mipi camera already stopped\r\n");
        return OPRT_OK;
    }

    ret = tkl_camera_mipi_stop();
    if (ret != OPRT_OK) {
        tkl_log_output( "camera stop failed: %d\r\n", ret);
        return ret;
    }

    s_mipi_camera_started = false;
    tkl_log_output( "camera stop ok\r\n");
    return OPRT_OK;
}

static OPERATE_RET test_mipi_camera_mipi_switch_output(uint16_t output_width, uint16_t output_height)
{
    OPERATE_RET ret;
    uint64_t t_total;

    if (s_mipi_camera_inited != true) {
        tkl_log_output("camera switch failed: camera is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    t_total = (uint64_t)tkl_system_get_millisecond();

    /* live switch: closes+reopens MP only; SP (if armed) keeps streaming */
    ret = tkl_camera_mipi_set_output(TUYA_CAMERA_MIPI_CHANNEL_MP, output_width, output_height);
    if (ret != OPRT_OK) {
        tkl_log_output("camera switch set_output failed: %d\r\n", ret);
        return ret;
    }

    s_mipi_camera_cfg.output_width = output_width;
    s_mipi_camera_cfg.output_height = output_height;

    tkl_log_output("camera switch ok (live), output=%ux%u, switch=%llums\r\n",
            output_width,
            output_height,
            (unsigned long long)((uint64_t)tkl_system_get_millisecond() - t_total));
    return OPRT_OK;
}

static OPERATE_RET test_mipi_camera_mipi_switch_sp(uint16_t sp_width, uint16_t sp_height)
{
    OPERATE_RET ret;
    uint64_t t_total;

    if (s_mipi_camera_started != true) {
        tkl_log_output("camera switch_sp failed: camera is not started\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    t_total = (uint64_t)tkl_system_get_millisecond();

    /* live switch: closes+reopens SP only; MP keeps streaming (driver force-drops
     * a couple of MP frames around the MI_CFG_UPD latch pulse) */
    ret = tkl_camera_mipi_set_output(TUYA_CAMERA_MIPI_CHANNEL_SP, sp_width, sp_height);
    if (ret != OPRT_OK) {
        tkl_log_output("camera switch_sp failed: %d\r\n", ret);
        return ret;
    }

    s_mipi_camera_cfg.sp_width = sp_width;
    s_mipi_camera_cfg.sp_height = sp_height;

    tkl_log_output("camera switch_sp ok (live), sp=%ux%u, switch=%llums\r\n",
            sp_width,
            sp_height,
            (unsigned long long)((uint64_t)tkl_system_get_millisecond() - t_total));
    return OPRT_OK;
}

static OPERATE_RET test_mipi_camera_capture_frame_ch(TUYA_CAMERA_MIPI_FRAME_T *frame,
                                                    uint8_t channel_id)
{
    OPERATE_RET ret;

    if (frame == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_mipi_camera_started != true) {
        tkl_log_output( "mipi camera is not started\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (channel_id == TUYA_CAMERA_MIPI_CHANNEL_SP) {
        if (s_mipi_camera_cfg.sp_enable == FALSE) {
            tkl_log_output( "sp get_frame requires sp_enable\r\n");
            return OPRT_NOT_SUPPORTED;
        }

        if (s_mipi_camera_cfg.sp_enable_flexa != FALSE || s_mipi_camera_cfg.sp_work_mode != 0U) {
            tkl_log_output( "sp get_frame requires frame mode, flexa=%u work_mode=%u\r\n",
                            (unsigned int)s_mipi_camera_cfg.sp_enable_flexa,
                            (unsigned int)s_mipi_camera_cfg.sp_work_mode);
            return OPRT_NOT_SUPPORTED;
        }

        goto do_capture;
    }

    if (s_mipi_camera_cfg.enable_flexa != FALSE || s_mipi_camera_cfg.work_mode != 0U) {
        tkl_log_output( "mp get_frame requires frame mode, flexa=%u work_mode=%u\r\n",
                        (unsigned int)s_mipi_camera_cfg.enable_flexa,
                        (unsigned int)s_mipi_camera_cfg.work_mode);
        return OPRT_NOT_SUPPORTED;
    }

do_capture:
    ret = test_mipi_camera_frame_prepare(channel_id);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(frame, 0, sizeof(*frame));
    frame->buffer = s_mipi_camera_frame;
    frame->size = s_mipi_camera_frame_size;
    frame->timeout_ms = s_mipi_camera_cfg.read_timeout_ms;
    frame->channel_id = channel_id;

    ret = tkl_camera_mipi_get_frame(frame);
    if (ret != OPRT_OK) {
        tkl_log_output( "camera get_frame failed: %d\r\n", ret);
        return ret;
    }

    s_mipi_camera_last_frame_len = frame->out_len;
    s_mipi_camera_last_frame_width = frame->width;
    s_mipi_camera_last_frame_height = frame->height;
    s_mipi_camera_last_frame_format = frame->format;

    return OPRT_OK;
}

OPERATE_RET test_mipi_camera_capture_frame(TUYA_CAMERA_MIPI_FRAME_T *frame)
{
    return test_mipi_camera_capture_frame_ch(frame, TUYA_CAMERA_MIPI_CHANNEL_MP);
}

static OPERATE_RET test_mipi_camera_mipi_get_frame_ch(uint8_t channel_id)
{
    OPERATE_RET ret;
    TUYA_CAMERA_MIPI_FRAME_T frame;
    uint8_t head[MIPI_TEST_CAMERA_HEAD_BYTES] = {0};
    uint32_t head_len;

    ret = test_mipi_camera_capture_frame_ch(&frame, channel_id);
    if (ret != OPRT_OK) {
        return ret;
    }

    head_len = (frame.out_len < MIPI_TEST_CAMERA_HEAD_BYTES) ? frame.out_len : MIPI_TEST_CAMERA_HEAD_BYTES;
    if (head_len > 0U) {
        os_memcpy(head, frame.buffer, head_len);
    }

    tkl_log_output(
            "camera %s frame ok, %ux%u fmt=%u len=%u head=%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
            (channel_id == TUYA_CAMERA_MIPI_CHANNEL_SP) ? "sp" : "mp",
            frame.width,
            frame.height,
            (unsigned int)frame.format,
            (unsigned int)frame.out_len,
            head[0],
            head[1],
            head[2],
            head[3],
            head[4],
            head[5],
            head[6],
            head[7]);
    return OPRT_OK;
}

/**
 * @brief Set camera mirror/flip and sync local cfg
 * @param[in] hmirror 0=off, non-zero=on
 * @param[in] vflip 0=off, non-zero=on
 * @return OPRT_OK on success
 */
static OPERATE_RET test_mipi_camera_set_mirror_flip(UINT8_T hmirror, UINT8_T vflip)
{
    OPERATE_RET ret;

    if (s_mipi_camera_inited != true) {
        tkl_log_output("mipi camera is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_camera_mipi_set_mirror_flip(hmirror, vflip);
    if (ret != OPRT_OK) {
        tkl_log_output("camera set_mirror_flip failed: %d\r\n", ret);
        return ret;
    }

    s_mipi_camera_cfg.hmirror = (hmirror != 0U) ? 1U : 0U;
    s_mipi_camera_cfg.vflip = (vflip != 0U) ? 1U : 0U;
    tkl_log_output("camera mirror set ok, hmirror=%u vflip=%u (started=%u)\r\n",
                   s_mipi_camera_cfg.hmirror,
                   s_mipi_camera_cfg.vflip,
                   (unsigned int)s_mipi_camera_started);
    return OPRT_OK;
}

/**
 * @brief Set MP AE warmup skip frames, applied on the next camera open
 *
 * 该值在 tkl_camera_mipi_start() 的 port_init 与 MP channel_open 之间下发，
 * 因此必须在 open 之前设置；已 open 的实例要重开才会生效。
 *
 * 只影响 MP 上“靠 ISP_FRAME_END_DONE 中断交付帧”的路径（flexa bond 到编码器）。
 * SP 抓图走 DQBUF 拉取，其预热丢帧由 TKL 内部的
 * TKL_CAM_MIPI_SP_WARMUP_DROP_CNT 处理，不受本命令影响。
 *
 * @param[in] count frames to drop, 0 = disabled
 * @return OPRT_OK on success
 */
static OPERATE_RET test_mipi_camera_set_skip_frames(UINT8_T count)
{
    s_mipi_camera_skip_frames = count;
    s_mipi_camera_cfg.skip_frames = count;

    tkl_log_output("camera skip_frames set to %u (applies on next open, inited=%u)\r\n",
                   (unsigned int)count,
                   (unsigned int)s_mipi_camera_inited);
    return OPRT_OK;
}

/**
 * @brief Get camera mirror/flip from TKL
 * @param[out] hmirror optional
 * @param[out] vflip optional
 * @return OPRT_OK on success
 */
static OPERATE_RET test_mipi_camera_get_mirror_flip(UINT8_T *hmirror, UINT8_T *vflip)
{
    OPERATE_RET ret;
    UINT8_T h = 0U;
    UINT8_T v = 0U;

    if (s_mipi_camera_inited != true) {
        tkl_log_output("mipi camera is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_camera_mipi_get_mirror_flip(&h, &v);
    if (ret != OPRT_OK) {
        tkl_log_output("camera get_mirror_flip failed: %d\r\n", ret);
        return ret;
    }

    if (hmirror != NULL) {
        *hmirror = h;
    }
    if (vflip != NULL) {
        *vflip = v;
    }

    tkl_log_output("camera mirror get ok, hmirror=%u vflip=%u\r\n", h, v);
    return OPRT_OK;
}

/**
 * @brief Rolling checksum for frame payload
 * @param[in] buf data buffer
 * @param[in] len byte length
 * @return checksum
 */
static uint32_t test_mipi_frame_checksum(const uint8_t *buf, uint32_t len)
{
    uint32_t sum = 0U;
    uint32_t i;

    if (buf == NULL || len == 0U) {
        return 0U;
    }

    for (i = 0U; i < len; i++) {
        sum = (sum * 131U) + buf[i];
    }
    return sum;
}

/**
 * @brief Checksum of a vertical Y strip [x, x+cols)
 * @param[in] y Y plane base
 * @param[in] width image width
 * @param[in] height image height
 * @param[in] stride Y stride in bytes
 * @param[in] x start column
 * @param[in] cols column count
 * @return checksum
 */
static uint32_t test_mipi_y_strip_checksum(const uint8_t *y,
                                          uint16_t width,
                                          uint16_t height,
                                          uint32_t stride,
                                          uint16_t x,
                                          uint16_t cols)
{
    uint32_t sum = 0U;
    uint16_t row;
    uint16_t col;

    if (y == NULL || width == 0U || height == 0U || stride == 0U || cols == 0U) {
        return 0U;
    }
    if ((uint32_t)x + (uint32_t)cols > (uint32_t)width) {
        return 0U;
    }

    for (row = 0U; row < height; row++) {
        const uint8_t *line = y + ((uint32_t)row * stride) + x;
        for (col = 0U; col < cols; col++) {
            sum = (sum * 131U) + line[col];
        }
    }
    return sum;
}

/**
 * @brief Drop frames then capture one for mirror compare
 * @param[out] frame captured frame
 * @param[out] checksum full-buffer checksum
 * @param[out] left_edge left Y strip checksum
 * @param[out] right_edge right Y strip checksum
 * @return OPRT_OK on success
 */
static OPERATE_RET test_mipi_camera_capture_mirror_sample(TUYA_CAMERA_MIPI_FRAME_T *frame,
                                                          uint32_t *checksum,
                                                          uint32_t *left_edge,
                                                          uint32_t *right_edge)
{
    OPERATE_RET ret;
    UINT32_T i;
    UINT16_T edge_cols;
    UINT32_T y_stride;
    UINT32_T y_size;

    if (frame == NULL || checksum == NULL || left_edge == NULL || right_edge == NULL) {
        return OPRT_INVALID_PARM;
    }

    for (i = 0U; i < MIPI_TEST_MIRROR_SETTLE_FRAMES; i++) {
        ret = test_mipi_camera_capture_frame(frame);
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    ret = test_mipi_camera_capture_frame(frame);
    if (ret != OPRT_OK) {
        return ret;
    }

    *checksum = test_mipi_frame_checksum(frame->buffer, frame->out_len);

    y_stride = (frame->y_stride != 0U) ? frame->y_stride : (UINT32_T)frame->width;
    y_size = y_stride * (UINT32_T)frame->height;
    if (frame->format != BK_PIXEL_FORMAT_NV12 || frame->out_len < y_size || frame->width < 16U) {
        *left_edge = 0U;
        *right_edge = 0U;
        return OPRT_OK;
    }

    edge_cols = MIPI_TEST_MIRROR_EDGE_COLS;
    if (edge_cols > (frame->width / 4U)) {
        edge_cols = (UINT16_T)(frame->width / 4U);
    }
    if (edge_cols == 0U) {
        edge_cols = 1U;
    }

    *left_edge = test_mipi_y_strip_checksum(frame->buffer,
                                            frame->width,
                                            frame->height,
                                            y_stride,
                                            0U,
                                            edge_cols);
    *right_edge = test_mipi_y_strip_checksum(frame->buffer,
                                             frame->width,
                                             frame->height,
                                             y_stride,
                                             (UINT16_T)(frame->width - edge_cols),
                                             edge_cols);
    return OPRT_OK;
}

/**
 * @brief Verify mirror/flip API and that image content changes
 * @return OPRT_OK if set/get and content change checks pass
 * @note Requires frame-mode camera already started. Point lens at a non-uniform scene.
 */
static OPERATE_RET test_mipi_camera_mirror_test(void)
{
    OPERATE_RET ret;
    TUYA_CAMERA_MIPI_FRAME_T frame;
    UINT8_T saved_h = 0U;
    UINT8_T saved_v = 0U;
    UINT8_T got_h = 0U;
    UINT8_T got_v = 0U;
    uint32_t base_sum = 0U;
    uint32_t base_left = 0U;
    uint32_t base_right = 0U;
    uint32_t h_sum = 0U;
    uint32_t h_left = 0U;
    uint32_t h_right = 0U;
    uint32_t v_sum = 0U;
    uint32_t v_left = 0U;
    uint32_t v_right = 0U;
    uint32_t hv_sum = 0U;
    uint32_t hv_left = 0U;
    uint32_t hv_right = 0U;
    BOOL_T pass = TRUE;

    if (s_mipi_camera_started != true) {
        tkl_log_output("mirror_test needs started camera: xmipi camera open && xmipi camera start\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_mipi_camera_cfg.enable_flexa != FALSE || s_mipi_camera_cfg.work_mode != 0U) {
        tkl_log_output("mirror_test needs frame mode (not flexa), use: xmipi camera open ...\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    ret = test_mipi_camera_get_mirror_flip(&saved_h, &saved_v);
    if (ret != OPRT_OK) {
        return ret;
    }

    /* Baseline: no mirror/flip */
    ret = test_mipi_camera_set_mirror_flip(0U, 0U);
    if (ret != OPRT_OK) {
        return ret;
    }
    ret = test_mipi_camera_get_mirror_flip(&got_h, &got_v);
    if (ret != OPRT_OK || got_h != 0U || got_v != 0U) {
        tkl_log_output("mirror_test FAIL: get after set(0,0) h=%u v=%u ret=%d\r\n",
                       got_h, got_v, ret);
        pass = FALSE;
        goto restore;
    }
    ret = test_mipi_camera_capture_mirror_sample(&frame, &base_sum, &base_left, &base_right);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    tkl_log_output("mirror_test base: sum=0x%08x left=0x%08x right=0x%08x %ux%u\r\n",
                   (unsigned int)base_sum, (unsigned int)base_left, (unsigned int)base_right,
                   frame.width, frame.height);

    /* Horizontal mirror */
    ret = test_mipi_camera_set_mirror_flip(1U, 0U);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    ret = test_mipi_camera_get_mirror_flip(&got_h, &got_v);
    if (ret != OPRT_OK || got_h != 1U || got_v != 0U) {
        tkl_log_output("mirror_test FAIL: get after set(1,0) h=%u v=%u ret=%d\r\n",
                       got_h, got_v, ret);
        pass = FALSE;
        goto restore;
    }
    ret = test_mipi_camera_capture_mirror_sample(&frame, &h_sum, &h_left, &h_right);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    tkl_log_output("mirror_test hmirror: sum=0x%08x left=0x%08x right=0x%08x\r\n",
                   (unsigned int)h_sum, (unsigned int)h_left, (unsigned int)h_right);
    if (h_sum == base_sum) {
        tkl_log_output("mirror_test FAIL: hmirror checksum unchanged (flat scene or not applied)\r\n");
        pass = FALSE;
    } else {
        tkl_log_output("mirror_test PASS: hmirror changed frame checksum\r\n");
    }
    if (base_left != 0U && base_right != 0U && h_left != 0U && h_right != 0U) {
        /* Soft check: mirrored left edge should be closer to original right edge. */
        uint32_t d_lr = (base_left > h_right) ? (base_left - h_right) : (h_right - base_left);
        uint32_t d_ll = (base_left > h_left) ? (base_left - h_left) : (h_left - base_left);
        if (d_lr < d_ll) {
            tkl_log_output("mirror_test PASS: hmirror edge swap looks correct (d_lr=%u < d_ll=%u)\r\n",
                           (unsigned int)d_lr, (unsigned int)d_ll);
        } else {
            tkl_log_output("mirror_test WARN: hmirror edge soft-check weak (d_lr=%u d_ll=%u), check scene\r\n",
                           (unsigned int)d_lr, (unsigned int)d_ll);
        }
    }

    /* Vertical flip */
    ret = test_mipi_camera_set_mirror_flip(0U, 1U);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    ret = test_mipi_camera_get_mirror_flip(&got_h, &got_v);
    if (ret != OPRT_OK || got_h != 0U || got_v != 1U) {
        tkl_log_output("mirror_test FAIL: get after set(0,1) h=%u v=%u ret=%d\r\n",
                       got_h, got_v, ret);
        pass = FALSE;
        goto restore;
    }
    ret = test_mipi_camera_capture_mirror_sample(&frame, &v_sum, &v_left, &v_right);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    tkl_log_output("mirror_test vflip: sum=0x%08x left=0x%08x right=0x%08x\r\n",
                   (unsigned int)v_sum, (unsigned int)v_left, (unsigned int)v_right);
    if (v_sum == base_sum || v_sum == h_sum) {
        tkl_log_output("mirror_test FAIL: vflip checksum not distinct (flat scene or not applied)\r\n");
        pass = FALSE;
    } else {
        tkl_log_output("mirror_test PASS: vflip changed frame checksum\r\n");
    }

    /* Both */
    ret = test_mipi_camera_set_mirror_flip(1U, 1U);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    ret = test_mipi_camera_get_mirror_flip(&got_h, &got_v);
    if (ret != OPRT_OK || got_h != 1U || got_v != 1U) {
        tkl_log_output("mirror_test FAIL: get after set(1,1) h=%u v=%u ret=%d\r\n",
                       got_h, got_v, ret);
        pass = FALSE;
        goto restore;
    }
    ret = test_mipi_camera_capture_mirror_sample(&frame, &hv_sum, &hv_left, &hv_right);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    tkl_log_output("mirror_test both: sum=0x%08x left=0x%08x right=0x%08x\r\n",
                   (unsigned int)hv_sum, (unsigned int)hv_left, (unsigned int)hv_right);
    if (hv_sum == base_sum) {
        tkl_log_output("mirror_test FAIL: both mirror+flip checksum unchanged\r\n");
        pass = FALSE;
    } else {
        tkl_log_output("mirror_test PASS: mirror+flip changed frame checksum\r\n");
    }

restore:
    (void)test_mipi_camera_set_mirror_flip(saved_h, saved_v);
    if (pass != TRUE) {
        tkl_log_output("mirror_test FAILED\r\n");
        return OPRT_COM_ERROR;
    }
    tkl_log_output("mirror_test ALL PASSED\r\n");
    return OPRT_OK;
}

/**
 * @brief Query ISP exposure luminance once
 * @param[out] luminance optional output
 * @return OPRT_OK on success
 */
static OPERATE_RET test_mipi_camera_get_luminance(UINT32_T *luminance)
{
    OPERATE_RET ret;
    UINT32_T lum = 0U;

    if (s_mipi_camera_started != true) {
        tkl_log_output("luminance needs started camera\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_camera_mipi_get_exposure_luminance(&lum);
    if (ret != OPRT_OK) {
        tkl_log_output("camera get_exposure_luminance failed: %d\r\n", ret);
        return ret;
    }

    tkl_log_output("camera luminance=%u (meanLum*1000, range 0~%u)\r\n",
                   (unsigned int)lum, (unsigned int)MIPI_TEST_LUMINANCE_MAX);
    if (luminance != NULL) {
        *luminance = lum;
    }
    return OPRT_OK;
}

/**
 * @brief Sample exposure luminance repeatedly and validate range
 * @param[in] count sample count
 * @param[in] interval_ms delay between samples
 * @return OPRT_OK if all samples succeed and stay in range
 */
static OPERATE_RET test_mipi_camera_luminance_test(UINT32_T count, UINT32_T interval_ms)
{
    OPERATE_RET ret;
    UINT32_T i;
    UINT32_T lum = 0U;
    UINT32_T min_lum = 0xFFFFFFFFU;
    UINT32_T max_lum = 0U;
    UINT64_T sum = 0U;
    BOOL_T pass = TRUE;

    if (count == 0U) {
        count = MIPI_TEST_LUMINANCE_DEFAULT_CNT;
    }
    if (interval_ms == 0U) {
        interval_ms = MIPI_TEST_LUMINANCE_DEFAULT_MS;
    }

    tkl_log_output("luminance_test start, count=%u interval=%ums\r\n",
                   (unsigned int)count, (unsigned int)interval_ms);

    for (i = 0U; i < count; i++) {
        ret = test_mipi_camera_get_luminance(&lum);
        if (ret != OPRT_OK) {
            pass = FALSE;
            break;
        }
        if (lum > MIPI_TEST_LUMINANCE_MAX) {
            tkl_log_output("luminance_test FAIL: sample[%u]=%u out of range\r\n",
                           (unsigned int)i, (unsigned int)lum);
            pass = FALSE;
            break;
        }
        if (lum < min_lum) {
            min_lum = lum;
        }
        if (lum > max_lum) {
            max_lum = lum;
        }
        sum += (UINT64_T)lum;
        if ((i + 1U) < count) {
            tkl_system_sleep(interval_ms);
        }
    }

    if (pass != TRUE) {
        tkl_log_output("luminance_test FAILED\r\n");
        return OPRT_COM_ERROR;
    }

    tkl_log_output("luminance_test PASS: samples=%u min=%u max=%u avg=%u\r\n",
                   (unsigned int)count,
                   (unsigned int)min_lum,
                   (unsigned int)max_lum,
                   (unsigned int)(sum / (UINT64_T)count));
    return OPRT_OK;
}

/**
 * @brief Read current CPROC and print it
 * @param[out] cproc optional snapshot
 * @return OPRT_OK on success
 */
static OPERATE_RET test_mipi_camera_get_cproc(TKL_CAMERA_MIPI_CPROC_T *cproc)
{
    OPERATE_RET ret;
    TKL_CAMERA_MIPI_CPROC_T attr = {0};

    if (s_mipi_camera_started != true) {
        tkl_log_output("cproc needs started camera\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_camera_mipi_get_cproc(&attr);
    if (ret != OPRT_OK) {
        tkl_log_output("camera get_cproc failed: %d\r\n", ret);
        return ret;
    }

    tkl_log_output("camera cproc enable=%u op=%u bri=%d con=%u sat=%u hue=%d\r\n",
                   (unsigned int)attr.enable,
                   (unsigned int)attr.op_type,
                   (int)attr.brightness,
                   (unsigned int)attr.contrast,
                   (unsigned int)attr.saturation,
                   (int)attr.hue);
    if (cproc != NULL) {
        *cproc = attr;
    }
    return OPRT_OK;
}

/**
 * @brief Set gray/color via CPROC and verify saturation
 * @param[in] gray TRUE=gray, FALSE=restore default CPROC
 * @return OPRT_OK on success
 */
static OPERATE_RET test_mipi_camera_set_color_to_gray(BOOL_T gray)
{
    OPERATE_RET ret;
    TKL_CAMERA_MIPI_CPROC_T attr = {0};

    if (s_mipi_camera_started != true) {
        tkl_log_output("gray needs started camera\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_camera_mipi_set_color_to_gray(gray);
    if (ret != OPRT_OK) {
        tkl_log_output("camera set_color_to_gray(%u) failed: %d\r\n",
                       (unsigned int)gray, ret);
        return ret;
    }

    ret = test_mipi_camera_get_cproc(&attr);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (gray != FALSE && attr.saturation != 0U) {
        tkl_log_output("gray FAIL: saturation=%u, expect 0\r\n",
                       (unsigned int)attr.saturation);
        return OPRT_COM_ERROR;
    }
    return OPRT_OK;
}

/**
 * @brief Switch gamma-out between night curve and backed-up day curve
 * @param[in] night TRUE=apply night gamma-out, FALSE=restore backup
 * @return OPRT_OK on success
 */
static OPERATE_RET test_mipi_camera_set_night_gamma(BOOL_T night)
{
    OPERATE_RET ret;

    if (s_mipi_camera_started != true) {
        tkl_log_output("night needs started camera\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_camera_mipi_set_night_gamma(night);
    if (ret != OPRT_OK) {
        tkl_log_output("camera set_night_gamma(%u) failed: %d\r\n",
                       (unsigned int)night, ret);
        return ret;
    }

    tkl_log_output("camera night_gamma=%u ok\r\n", (unsigned int)night);
    return OPRT_OK;
}

void cli_tuya_mipi_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret;
    const char *type = NULL;
    const char *cmd = NULL;
    const char *panel_name = NULL;
    uint16_t sensor_width = MIPI_TEST_CAMERA_SENSOR_W;
    uint16_t sensor_height = MIPI_TEST_CAMERA_SENSOR_H;
    uint8_t sensor_fps = MIPI_TEST_CAMERA_SENSOR_FPS;
    uint16_t output_width = MIPI_TEST_CAMERA_SENSOR_W;
    uint16_t output_height = MIPI_TEST_CAMERA_SENSOR_H;
    uint16_t sp_width = MIPI_TEST_FACE_SP_WIDTH;
    uint16_t sp_height = MIPI_TEST_FACE_SP_HEIGHT;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 3) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xmipi lcd <open [panel]|open_gpu [panel] [compress]|flush|stop_flush|close>; " MIPI_TEST_CAMERA_USAGE);
        return;
    }

    type = argv[1];
    cmd = argv[2];

    if (os_strcmp(type, "lcd") == 0) {
        if (argc >= 4) {
            panel_name = argv[3];
        }

        if (os_strcmp(cmd, "open") == 0) {
            ret = test_mipi_lcd_open(panel_name);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi lcd open %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else if (os_strcmp(cmd, "open_gpu") == 0) {
            BOOL_T decompress = FALSE;
            char *endptr = NULL;

            if (argc >= 4) {
                unsigned long value = os_strtoul(argv[3], &endptr, 10);
                if (endptr != NULL && endptr[0] == '\0') {
                    panel_name = NULL;
                    decompress = (value != 0U) ? TRUE : FALSE;
                }
            }

            if (argc >= 5) {
                decompress = (os_strtoul(argv[4], NULL, 10) != 0U) ? TRUE : FALSE;
            }

            ret = test_mipi_lcd_open_gpu(panel_name, decompress);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi lcd open_gpu %s (%d), panel=%s, size=%ux%u, decompress=%u\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     (s_mipi_panel_name[0] != '\0') ? s_mipi_panel_name : "auto",
                     s_mipi_width,
                     s_mipi_height,
                     (unsigned int)decompress);
        } else if (os_strcmp(cmd, "flush") == 0) {
            ret = test_mipi_lcd_flush(panel_name);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi lcd flush %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else if (os_strcmp(cmd, "stop_flush") == 0) {
            ret = test_mipi_lcd_stop_flush();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi lcd stop_flush %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else if (os_strcmp(cmd, "close") == 0) {
            ret = test_mipi_lcd_close();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi lcd close %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xmipi lcd <open [panel]|open_gpu [panel] [compress]|flush|stop_flush|close>");
        }
    } else if (os_strcmp(type, "camera") == 0) {
        if (argc >= 4) {
            sensor_width = (uint16_t)os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            sensor_height = (uint16_t)os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            sensor_fps = (uint8_t)os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            output_width = (uint16_t)os_strtoul(argv[6], NULL, 10);
        }
        if (argc >= 8) {
            output_height = (uint16_t)os_strtoul(argv[7], NULL, 10);
        }
        if (argc >= 9) {
            sp_width = (uint16_t)os_strtoul(argv[8], NULL, 10);
        }
        if (argc >= 10) {
            sp_height = (uint16_t)os_strtoul(argv[9], NULL, 10);
        }

        if (os_strcmp(cmd, "open") == 0) {
            ret = test_mipi_camera_mipi_open(sensor_width, sensor_height, sensor_fps, output_width, output_height);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera open %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else if (os_strcmp(cmd, "open_gpu") == 0) {
            ret = test_mipi_camera_mipi_open_gpu(sensor_width, sensor_height, sensor_fps, output_width, output_height);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera open_gpu %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else if (os_strcmp(cmd, "open_gpu_sp") == 0) {
            uint32_t sp_format = MIPI_TEST_FACE_SP_FORMAT;

            if (argc >= 11 && os_strcmp(argv[10], "nv12") == 0) {
                sp_format = BK_PIXEL_FORMAT_NV12;
            }

            ret = test_mipi_camera_mipi_open_gpu_sp(sensor_width,
                                                   sensor_height,
                                                   sensor_fps,
                                                   output_width,
                                                   output_height,
                                                   sp_width,
                                                   sp_height,
                                                   sp_format);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera open_gpu_sp %s (%d), sp=%ux%u fmt=%u\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     sp_width,
                     sp_height,
                     (unsigned int)sp_format);
        } else if (os_strcmp(cmd, "start") == 0) {
            ret = test_mipi_camera_mipi_start();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera start %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else if (os_strcmp(cmd, "get_frame") == 0 || os_strcmp(cmd, "frame") == 0) {
            uint8_t gf_chn = TUYA_CAMERA_MIPI_CHANNEL_MP;

            if (argc >= 4 && os_strcmp(argv[3], "sp") == 0) {
                gf_chn = TUYA_CAMERA_MIPI_CHANNEL_SP;
            }
            ret = test_mipi_camera_mipi_get_frame_ch(gf_chn);
            if (ret == OPRT_OK) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xmipi camera get_frame ok (%d), %ux%u fmt=%u len=%u\r\n",
                         ret,
                         s_mipi_camera_last_frame_width,
                         s_mipi_camera_last_frame_height,
                         (unsigned int)s_mipi_camera_last_frame_format,
                         (unsigned int)s_mipi_camera_last_frame_len);
            } else {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xmipi camera get_frame failed (%d)\r\n", ret);
            }
        } else if (os_strcmp(cmd, "mirror") == 0) {
            UINT8_T hmirror = 0U;
            UINT8_T vflip = 0U;

            if (argc < 4) {
                ret = test_mipi_camera_get_mirror_flip(&hmirror, &vflip);
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xmipi camera mirror get %s (%d), hmirror=%u vflip=%u\r\n",
                         (ret == OPRT_OK) ? "ok" : "failed",
                         ret,
                         hmirror,
                         vflip);
            } else if (argc < 5) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "Usage: xmipi camera mirror [hmirror vflip]\r\n");
            } else {
                hmirror = (UINT8_T)((os_strtoul(argv[3], NULL, 10) != 0U) ? 1U : 0U);
                vflip = (UINT8_T)((os_strtoul(argv[4], NULL, 10) != 0U) ? 1U : 0U);
                ret = test_mipi_camera_set_mirror_flip(hmirror, vflip);
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xmipi camera mirror set %s (%d), hmirror=%u vflip=%u\r\n",
                         (ret == OPRT_OK) ? "ok" : "failed",
                         ret,
                         hmirror,
                         vflip);
            }
        } else if (os_strcmp(cmd, "mirror_test") == 0) {
            ret = test_mipi_camera_mirror_test();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera mirror_test %s (%d)\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret);
        } else if (os_strcmp(cmd, "luminance") == 0 || os_strcmp(cmd, "lum") == 0) {
            UINT32_T lum = 0U;

            ret = test_mipi_camera_get_luminance(&lum);
            if (ret == OPRT_OK) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xmipi camera luminance ok (%d), luminance=%u\r\n",
                         ret, (unsigned int)lum);
            } else {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xmipi camera luminance failed (%d)\r\n", ret);
            }
        } else if (os_strcmp(cmd, "luminance_test") == 0 || os_strcmp(cmd, "lum_test") == 0) {
            UINT32_T count = MIPI_TEST_LUMINANCE_DEFAULT_CNT;
            UINT32_T interval_ms = MIPI_TEST_LUMINANCE_DEFAULT_MS;

            if (argc >= 4) {
                count = (UINT32_T)os_strtoul(argv[3], NULL, 10);
            }
            if (argc >= 5) {
                interval_ms = (UINT32_T)os_strtoul(argv[4], NULL, 10);
            }
            ret = test_mipi_camera_luminance_test(count, interval_ms);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera luminance_test %s (%d)\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret);
        } else if (os_strcmp(cmd, "cproc") == 0) {
            TKL_CAMERA_MIPI_CPROC_T attr = {0};

            ret = test_mipi_camera_get_cproc(&attr);
            if (ret == OPRT_OK) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xmipi camera cproc ok (%d), enable=%u op=%u bri=%d con=%u sat=%u hue=%d\r\n",
                         ret,
                         (unsigned int)attr.enable,
                         (unsigned int)attr.op_type,
                         (int)attr.brightness,
                         (unsigned int)attr.contrast,
                         (unsigned int)attr.saturation,
                         (int)attr.hue);
            } else {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xmipi camera cproc failed (%d)\r\n", ret);
            }
        } else if (os_strcmp(cmd, "gray") == 0) {
            BOOL_T gray = TRUE;

            if (argc >= 4) {
                gray = (os_strtoul(argv[3], NULL, 10) != 0U) ? TRUE : FALSE;
            }
            ret = test_mipi_camera_set_color_to_gray(gray);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera gray %s (%d), enable=%u\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     (unsigned int)gray);
        } else if (os_strcmp(cmd, "night") == 0) {
            BOOL_T night = TRUE;

            if (argc >= 4) {
                night = (os_strtoul(argv[3], NULL, 10) != 0U) ? TRUE : FALSE;
            }
            ret = test_mipi_camera_set_night_gamma(night);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera night %s (%d), enable=%u\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     (unsigned int)night);
        } else if (os_strcmp(cmd, "stop") == 0) {
            ret = test_mipi_camera_mipi_stop();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera stop %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else if (os_strcmp(cmd, "switch") == 0) {
            if (argc < 5) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "Usage: xmipi camera switch <ow> <oh>\r\n");
                return;
            }

            output_width = (uint16_t)os_strtoul(argv[3], NULL, 10);
            output_height = (uint16_t)os_strtoul(argv[4], NULL, 10);
            ret = test_mipi_camera_mipi_switch_output(output_width, output_height);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera switch %s (%d), output=%ux%u\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     output_width,
                     output_height);
        } else if (os_strcmp(cmd, "skip") == 0) {
            if (argc < 4) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xmipi camera skip_frames=%u (set before open); Usage: xmipi camera skip <n>\r\n",
                         (unsigned int)s_mipi_camera_cfg.skip_frames);
                return;
            }

            ret = test_mipi_camera_set_skip_frames((UINT8_T)os_strtoul(argv[3], NULL, 10));
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera skip %s (%d), skip_frames=%u, applies on next open\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     (unsigned int)s_mipi_camera_cfg.skip_frames);
        } else if (os_strcmp(cmd, "switch_sp") == 0) {
            if (argc < 5) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "Usage: xmipi camera switch_sp <spw> <sph>\r\n");
                return;
            }

            sp_width = (uint16_t)os_strtoul(argv[3], NULL, 10);
            sp_height = (uint16_t)os_strtoul(argv[4], NULL, 10);
            ret = test_mipi_camera_mipi_switch_sp(sp_width, sp_height);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera switch_sp %s (%d), sp=%ux%u\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     sp_width,
                     sp_height);
        } else if (os_strcmp(cmd, "close") == 0) {
            ret = test_mipi_camera_mipi_close();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera close %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen, "%s", MIPI_TEST_CAMERA_USAGE);
        }
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xmipi lcd <open [panel]|open_gpu [panel] [compress]|flush|stop_flush|close>; " MIPI_TEST_CAMERA_USAGE);
    }
}
