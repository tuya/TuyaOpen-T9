#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <components/bk_lcd_panel.h>
#include <components/bk_frame_buffer.h>
#include <components/log.h>
#include <driver/gpio_types.h>
#include <driver/trng.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include "sys_driver.h"

#include "cli_tuya_test.h"
#include "tkl_mipi.h"
#include "tkl_system.h"
#include "tkl_thread.h"

#define TAG "test-mipi"

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
            BK_LOGE(TAG, "frame[%u] alloc failed, size=%u\r\n", i, (unsigned int)s_mipi_frame_size);
            while (i > 0U) {
                i--;
                bk_frame_buffer_free(s_mipi_frames[i]);
                s_mipi_frames[i] = NULL;
            }
            s_mipi_frame_size = 0;
            return OPRT_MALLOC_FAILED;
        }
    }

    BK_LOGI(TAG, "frame buffers ready, count=%u, frame_size=%u\r\n",
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
        BK_LOGE(TAG, "flush failed: %d\r\n", ret);
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
            BK_LOGE(TAG, "periodic flush failed, stop task\r\n");
            s_mipi_task_running = false;
            break;
        }

        BK_LOGI(TAG, "random color flush: 0x%08x\r\n", (unsigned int)color);
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
        BK_LOGE(TAG, "create refresh task failed: %d\r\n", ret);
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
            BK_LOGE(TAG, "stop refresh task timeout\r\n");
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
            BK_LOGE(TAG, "get panel timing failed: %d, panel desc is null\r\n", ret);
            return OPRT_COM_ERROR;
        }

        timing = s_mipi_device.mipi.panel->timing;
        BK_LOGW(TAG, "get panel timing failed: %d, fallback to panel desc %ux%u\r\n",
                ret, timing.h_size, timing.v_size);
    }

    if (timing.h_size == 0 || timing.v_size == 0) {
        return OPRT_INVALID_PARM;
    }

    s_mipi_width = timing.h_size;
    s_mipi_height = timing.v_size;
    return OPRT_OK;
}

static OPERATE_RET test_mipi_lcd_open(const char *panel_name)
{
    OPERATE_RET ret;
    uint32_t color;

    if (s_mipi_task_running == true || s_mipi_device.opened == TRUE) {
        BK_LOGW(TAG, "mipi test already running\r\n");
        return OPRT_OK;
    }

    test_mipi_runtime_reset();

    if (panel_name != NULL && panel_name[0] != '\0') {
        snprintf(s_mipi_panel_name, sizeof(s_mipi_panel_name), "%s", panel_name);
        s_mipi_device.mipi.panel_name = s_mipi_panel_name;
    } else {
        snprintf(s_mipi_panel_name, sizeof(s_mipi_panel_name), "%s", CLI_TEST_MIPI_PANEL_NAME);
        s_mipi_device.mipi.panel_name = s_mipi_panel_name;
    }

    test_mipi_auxldo_enable();
    test_mipi_frame_buffer_init_once();
    ret = tkl_display_mipi_open(&s_mipi_device);
    if (ret != OPRT_OK) {
        BK_LOGE(TAG, "open failed: %d\r\n", ret);
        test_mipi_runtime_reset();
        return ret;
    }

    ret = test_mipi_update_geometry();
    if (ret != OPRT_OK) {
        BK_LOGE(TAG, "update geometry failed: %d\r\n", ret);
        tkl_display_mipi_close(&s_mipi_device);
        test_mipi_runtime_reset();
        return ret;
    }

    ret = test_mipi_frame_buffers_prepare();
    if (ret != OPRT_OK) {
        BK_LOGE(TAG, "prepare frame buffers failed: %d\r\n", ret);
        tkl_display_mipi_close(&s_mipi_device);
        test_mipi_runtime_reset();
        return ret;
    }

    color = test_mipi_random_color_get();
    ret = test_mipi_flush_color(color);
    if (ret != OPRT_OK) {
        BK_LOGE(TAG, "initial flush failed: %d\r\n", ret);
        test_mipi_frame_buffers_release();
        tkl_display_mipi_close(&s_mipi_device);
        test_mipi_runtime_reset();
        return ret;
    }

    BK_LOGI(TAG, "mipi open ok, panel=%s, size=%ux%u\r\n",
            s_mipi_device.mipi.panel_name ? s_mipi_device.mipi.panel_name : "auto",
            s_mipi_width, s_mipi_height);
    BK_LOGI(TAG, "flush path=direct, fmt=RGB565, frame_size=%u, buffers=%u, first_color=0x%08x\r\n",
            (unsigned int)test_mipi_frame_size_get(),
            (unsigned int)MIPI_TEST_FRAME_COUNT,
            (unsigned int)color);

    ret = test_mipi_task_start();
    if (ret != OPRT_OK) {
        test_mipi_frame_buffers_release();
        tkl_display_mipi_close(&s_mipi_device);
        test_mipi_runtime_reset();
        return ret;
    }

    BK_LOGI(TAG, "random pure-color refresh started, interval=%u ms\r\n",
            (unsigned int)MIPI_TEST_REFRESH_MS);
    return OPRT_OK;
}

static OPERATE_RET test_mipi_lcd_close(void)
{
    OPERATE_RET ret;
    OPERATE_RET stop_ret;

    stop_ret = test_mipi_task_stop();
    if (stop_ret != OPRT_OK) {
        BK_LOGW(TAG, "refresh task stop failed: %d\r\n", stop_ret);
        return stop_ret;
    }

    ret = tkl_display_mipi_close(&s_mipi_device);
    test_mipi_frame_buffers_release();
    test_mipi_runtime_reset();
    s_mipi_panel_name[0] = '\0';

    BK_LOGI(TAG, "mipi close %s\r\n", (ret == OPRT_OK) ? "ok" : "failed");
    return ret;
}

void cli_tuya_mipi_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret;
    const char *cmd = NULL;
    const char *panel_name = NULL;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: mipi open [panel_name] | close");
        return;
    }

    cmd = argv[1];
    if (argc >= 3) {
        panel_name = argv[2];
    }

    if (os_strcmp(cmd, "open") == 0) {
        ret = test_mipi_lcd_open(panel_name);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "mipi open %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
    } else if (os_strcmp(cmd, "close") == 0) {
        ret = test_mipi_lcd_close();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "mipi close %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: mipi open [panel_name] | close");
    }
}
