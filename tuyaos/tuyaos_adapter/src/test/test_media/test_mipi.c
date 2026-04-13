#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <components/bk_lcd_panel.h>
#include <components/bk_frame_buffer.h>
#include <common/avdk_pixel_types.h>
#include <driver/gpio_types.h>
#include <driver/trng.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include "sys_driver.h"

#include "cli_tuya_test.h"
#include "tkl_camera_mipi.h"
#include "tkl_display_mipi.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_thread.h"

#define MIPI_TEST_TASK_NAME         "mipi_test"
#define MIPI_TEST_TASK_STACK_SIZE   (4 * 1024)
#define MIPI_TEST_RESET_PIN         GPIO_60
#define MIPI_TEST_BACKLIGHT_PIN     GPIO_7
#define MIPI_TEST_INVALID_PIN       (-1)
#define MIPI_TEST_PANEL_NAME_LEN    64
#define MIPI_TEST_BYTES_PER_PIXEL   2U
#define MIPI_TEST_FRAME_COUNT       2U
#define MIPI_TEST_REFRESH_MS        1000U
#define SYS_ANA_REG_BASE            (0x44010000)
#define LDO_ANA_REG                 (0x69)
#define MIPI_TEST_CAMERA_PIN_SCL    GPIO_69
#define MIPI_TEST_CAMERA_PIN_SDA    GPIO_70
#define MIPI_TEST_CAMERA_I2C_ID     1U
#define MIPI_TEST_CAMERA_PIN_RESET  GPIO_71
#define MIPI_TEST_CAMERA_PIN_PWDN   (-1)
#define MIPI_TEST_CAMERA_PIN_XCLK   GPIO_59
#define MIPI_TEST_CAMERA_SENSOR_W   1920U
#define MIPI_TEST_CAMERA_SENSOR_H   1080U
#define MIPI_TEST_CAMERA_SENSOR_FPS 15U
#define MIPI_TEST_CAMERA_OUT_BUF_CNT 3U
#define MIPI_TEST_CAMERA_HEAD_BYTES 8U

static ty_display_mipi_device_t s_mipi_device;
static ty_frame_buffer_t s_mipi_frame_buff;
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
static bool s_mipi_camera_inited = false;
static bool s_mipi_camera_started = false;
static uint8_t *s_mipi_camera_frame = NULL;
static uint32_t s_mipi_camera_frame_size = 0;
static uint32_t s_mipi_camera_last_frame_len = 0;
static uint16_t s_mipi_camera_last_frame_width = 0;
static uint16_t s_mipi_camera_last_frame_height = 0;
static uint32_t s_mipi_camera_last_frame_format = BK_PIXEL_FORMAT_INVALID;

static OPERATE_RET test_mipi_lcd_close(void);

static void test_mipi_auxldo_enable(void)
{
    uint32_t reg = REG_READ(SYS_ANA_REG_BASE + LDO_ANA_REG * 4);

    reg |= (0xF << 28) | (0x2 << 23) | (0x7 << 19) | (0x7 << 15);
    reg &= ~(0xF << 11);
    reg |= (0x8 << 11);
    REG_WRITE(SYS_ANA_REG_BASE + LDO_ANA_REG * 4, reg);
}

static void test_mipi_runtime_reset(void)
{
    os_memset(&s_mipi_device, 0, sizeof(s_mipi_device));
    os_memset(&s_mipi_frame_buff, 0, sizeof(s_mipi_frame_buff));
    os_memset(s_mipi_frames, 0, sizeof(s_mipi_frames));

    s_mipi_width = CLI_TEST_LCD_WIDTH;
    s_mipi_height = CLI_TEST_LCD_HEIGHT;
    s_mipi_frame_size = 0;
    s_mipi_frame_index = 0;
    s_mipi_color_state = 0x13579BDFU;
    s_mipi_task_running = false;
    s_mipi_task = NULL;

    s_mipi_device.mipi.enable = TRUE;
    s_mipi_device.mipi.pin_reset = MIPI_TEST_RESET_PIN;
    s_mipi_device.mipi.pin_backlight = MIPI_TEST_BACKLIGHT_PIN;
    s_mipi_device.mipi.pin_scl = MIPI_TEST_INVALID_PIN;
    s_mipi_device.mipi.pin_sda = MIPI_TEST_INVALID_PIN;
    s_mipi_device.mipi.panel = NULL;
    s_mipi_device.mipi.panel_name = NULL;

    s_mipi_device.dpu_video.enable = TRUE;
    s_mipi_device.dpu_video.decompress = FALSE;
    s_mipi_device.dpu_video.format = BK_PIXEL_FORMAT_RGB565;
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
    s_mipi_camera_cfg.sensor_input_format = BK_PIXEL_FORMAT_RGGB10;
    s_mipi_camera_cfg.output_width = MIPI_TEST_CAMERA_SENSOR_W;
    s_mipi_camera_cfg.output_height = MIPI_TEST_CAMERA_SENSOR_H;
    s_mipi_camera_cfg.output_format = BK_PIXEL_FORMAT_NV12;
    s_mipi_camera_cfg.output_buf_cnt = MIPI_TEST_CAMERA_OUT_BUF_CNT;
    s_mipi_camera_cfg.enable_flexa = FALSE;
    s_mipi_camera_cfg.work_mode = 0U;
    s_mipi_camera_cfg.read_timeout_ms = 1000U;

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

static uint32_t test_mipi_camera_frame_size_get(void)
{
    return bk_image_size_get(s_mipi_camera_cfg.output_width,
                             s_mipi_camera_cfg.output_height,
                             (bk_pixel_format_t)s_mipi_camera_cfg.output_format);
}

static OPERATE_RET test_mipi_camera_frame_prepare(void)
{
    uint32_t frame_size = test_mipi_camera_frame_size_get();

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

    os_memset(&s_mipi_frame_buff, 0, sizeof(s_mipi_frame_buff));
    s_mipi_frame_buff.type = TYPE_PSRAM;
    s_mipi_frame_buff.fmt = TY_PIXEL_FMT_RGB565;
    s_mipi_frame_buff.x_start = 0;
    s_mipi_frame_buff.y_start = 0;
    s_mipi_frame_buff.width = s_mipi_width;
    s_mipi_frame_buff.height = s_mipi_height;
    s_mipi_frame_buff.len = (uint32_t)s_mipi_frame_size;
    s_mipi_frame_buff.frame = frame;
    s_mipi_frame_buff.pdata = frame;
    s_mipi_frame_buff.free_cb = NULL;

    ret = tkl_display_mipi_flush(&s_mipi_device, &s_mipi_frame_buff);
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
    bk_err_t ret;
    bk_display_timing_t timing;

    if (s_mipi_device.panel_handle == NULL) {
        return OPRT_COM_ERROR;
    }

    os_memset(&timing, 0, sizeof(timing));
    ret = bk_lcd_panel_get_disp_timing(s_mipi_device.panel_handle, &timing);
    if (ret != BK_OK) {
        if (s_mipi_device.mipi.panel == NULL) {
            tkl_log_output( "get panel timing failed: %d, panel desc is null\r\n", ret);
            return OPRT_COM_ERROR;
        }

        timing = s_mipi_device.mipi.panel->timing;
        tkl_log_output( "get panel timing failed: %d, fallback to panel desc %ux%u\r\n",
                ret, timing.h_size, timing.v_size);
    }

    if (timing.h_size == 0 || timing.v_size == 0) {
        return OPRT_INVALID_PARM;
    }

    s_mipi_width = timing.h_size;
    s_mipi_height = timing.v_size;
    return OPRT_OK;
}

static OPERATE_RET test_mipi_lcd_ensure_open(const char *panel_name)
{
    if (s_mipi_device.opened == TRUE) {
        return OPRT_OK;
    }

    if (panel_name != NULL && panel_name[0] != '\0') {
        snprintf(s_mipi_panel_name, sizeof(s_mipi_panel_name), "%s", panel_name);
        s_mipi_device.mipi.panel_name = s_mipi_panel_name;
    } else {
        snprintf(s_mipi_panel_name, sizeof(s_mipi_panel_name), "%s", CLI_TEST_MIPI_PANEL_NAME);
        s_mipi_device.mipi.panel_name = s_mipi_panel_name;
    }

    test_mipi_auxldo_enable();
    test_mipi_frame_buffer_init_once();
    OPERATE_RET ret = tkl_display_mipi_open(&s_mipi_device);
    if (ret != OPRT_OK) {
        tkl_log_output( "open failed: %d\r\n", ret);
        test_mipi_runtime_reset();
        return ret;
    }

    ret = test_mipi_update_geometry();
    if (ret != OPRT_OK) {
        tkl_log_output( "update geometry failed: %d\r\n", ret);
        tkl_display_mipi_close(&s_mipi_device);
        test_mipi_runtime_reset();
        return ret;
    }

    tkl_log_output( "mipi open ok, panel=%s, size=%ux%u\r\n",
            s_mipi_device.mipi.panel_name ? s_mipi_device.mipi.panel_name : "auto",
            s_mipi_width, s_mipi_height);

    return OPRT_OK;
}

static OPERATE_RET test_mipi_lcd_ensure_open_with_config(const char *panel_name,
                                                         UINT32_T display_format,
                                                         BOOL_T decompress)
{
    test_mipi_runtime_reset();
    s_mipi_device.dpu_video.format = display_format;
    s_mipi_device.dpu_video.decompress = decompress;
    return test_mipi_lcd_ensure_open(panel_name);
}

static OPERATE_RET test_mipi_lcd_open(const char *panel_name)
{
    OPERATE_RET ret;

    if (s_mipi_device.opened == TRUE) {
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

    if (s_mipi_device.opened == TRUE &&
        s_mipi_device.dpu_video.format == BK_PIXEL_FORMAT_ARGB8888 &&
        s_mipi_device.dpu_video.decompress == decompress) {
        tkl_log_output( "mipi lcd already open in gpu mode\r\n");
        return OPRT_OK;
    }

    if (s_mipi_device.opened == TRUE) {
        ret = test_mipi_lcd_close();
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    ret = test_mipi_lcd_ensure_open_with_config(panel_name, BK_PIXEL_FORMAT_ARGB8888, decompress);
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_log_output(
            "mipi lcd open for gpu, panel=%s, size=%ux%u, fmt=%u, decompress=%u\r\n",
            (s_mipi_device.mipi.panel_name != NULL) ? s_mipi_device.mipi.panel_name : "auto",
            s_mipi_width,
            s_mipi_height,
            (unsigned int)s_mipi_device.dpu_video.format,
            (unsigned int)decompress);
    return OPRT_OK;
}

static OPERATE_RET test_mipi_lcd_flush(const char *panel_name)
{
    OPERATE_RET ret;
    uint32_t color;

    (void)panel_name;

    if (s_mipi_device.opened != TRUE) {
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

    if (s_mipi_device.opened != TRUE) {
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

    ret = tkl_display_mipi_close(&s_mipi_device);
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

    test_mipi_auxldo_enable();
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

    test_mipi_auxldo_enable();
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

BOOL_T test_mipi_lcd_is_open(void)
{
    return (s_mipi_device.opened == TRUE) ? TRUE : FALSE;
}

ty_display_mipi_device_t *test_mipi_lcd_device_get(void)
{
    return (s_mipi_device.opened == TRUE) ? &s_mipi_device : NULL;
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

static OPERATE_RET test_mipi_camera_mipi_get_frame(void)
{
    OPERATE_RET ret;
    TUYA_CAMERA_MIPI_FRAME_T frame;
    uint8_t head[MIPI_TEST_CAMERA_HEAD_BYTES] = {0};
    uint32_t head_len;

    if (s_mipi_camera_started != true) {
        tkl_log_output( "mipi camera is not started\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = test_mipi_camera_frame_prepare();
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&frame, 0, sizeof(frame));
    frame.buffer = s_mipi_camera_frame;
    frame.size = s_mipi_camera_frame_size;
    frame.timeout_ms = s_mipi_camera_cfg.read_timeout_ms;

    ret = tkl_camera_mipi_get_frame(&frame);
    if (ret != OPRT_OK) {
        tkl_log_output( "camera get_frame failed: %d\r\n", ret);
        return ret;
    }

    s_mipi_camera_last_frame_len = frame.out_len;
    s_mipi_camera_last_frame_width = frame.width;
    s_mipi_camera_last_frame_height = frame.height;
    s_mipi_camera_last_frame_format = frame.format;

    head_len = (frame.out_len < MIPI_TEST_CAMERA_HEAD_BYTES) ? frame.out_len : MIPI_TEST_CAMERA_HEAD_BYTES;
    if (head_len > 0U) {
        os_memcpy(head, frame.buffer, head_len);
    }

    tkl_log_output(
            "camera frame ok, %ux%u fmt=%u len=%u head=%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
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

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 3) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xmipi lcd <open [panel]|open_gpu [panel] [compress]|flush|stop_flush|close>; xmipi camera <open [sw sh fps ow oh]|open_gpu [sw sh fps ow oh]|start|get_frame|stop|close>");
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
                     (s_mipi_device.mipi.panel_name != NULL) ? s_mipi_device.mipi.panel_name : "auto",
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

        if (os_strcmp(cmd, "open") == 0) {
            ret = test_mipi_camera_mipi_open(sensor_width, sensor_height, sensor_fps, output_width, output_height);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera open %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else if (os_strcmp(cmd, "open_gpu") == 0) {
            ret = test_mipi_camera_mipi_open_gpu(sensor_width, sensor_height, sensor_fps, output_width, output_height);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera open_gpu %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else if (os_strcmp(cmd, "start") == 0) {
            ret = test_mipi_camera_mipi_start();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera start %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else if (os_strcmp(cmd, "get_frame") == 0 || os_strcmp(cmd, "frame") == 0) {
            ret = test_mipi_camera_mipi_get_frame();
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
        } else if (os_strcmp(cmd, "stop") == 0) {
            ret = test_mipi_camera_mipi_stop();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera stop %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else if (os_strcmp(cmd, "close") == 0) {
            ret = test_mipi_camera_mipi_close();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi camera close %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xmipi camera <open [sw sh fps ow oh]|open_gpu [sw sh fps ow oh]|start|get_frame|stop|close>");
        }
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xmipi lcd <open [panel]|open_gpu [panel] [compress]|flush|stop_flush|close>; xmipi camera <open [sw sh fps ow oh]|open_gpu [sw sh fps ow oh]|start|get_frame|stop|close>");
    }
}
