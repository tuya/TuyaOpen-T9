#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <components/bk_frame_buffer.h>
#include <components/bk_hardware_ram.h>
#include <modules/vg_lite_gpu/vg_lite.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>

#include "sys_driver.h"
#include "cli_tuya_test.h"
#include "lv_demo_widgets.h"
#include "lvgl_stress_demo.h"
#include "lv_vendor.h"
#include "tkl_mipi_dsi.h"
#include "tkl_output.h"
#include "tuya_cloud_types.h"

#if (CONFIG_LVGL && CONFIG_LVGL_V9)

#define TUYA_LVGL_DRAW_LINES     64U
#define TUYA_LVGL_MIN_WIDTH      1U
#define TUYA_LVGL_MIN_HEIGHT     1U
#define TUYA_LVGL_GPU_BUFFER_GUARD_SIZE 0U
#define TUYA_LVGL_DRAW_BUFFER_ALIGN_SIZE 64U

extern BOOL_T test_mipi_lcd_is_open(void);
extern void test_mipi_lcd_get_geometry(UINT16_T *width, UINT16_T *height);

typedef enum {
    TUYA_LVGL_DEMO_NONE = 0,
    TUYA_LVGL_DEMO_WIDGETS,
    TUYA_LVGL_DEMO_STRESS,
} tuya_lvgl_demo_type_t;

typedef struct {
    bool inited;
    bool started;
    bool output_compress;
    tuya_lvgl_demo_type_t demo;
    uint16_t width;
    uint16_t height;
    uint16_t disp_width;
    uint16_t disp_height;
    void *gpu_contiguous_buffer;
    void *draw_buffer;
    bool draw_buffer_owned_by_vendor;
    void *frame_buffer[CONFIG_LVGL_FRAME_BUFFER_NUM];
    int (*flush_done_cb)(void *args);
} tuya_lvgl_widgets_ctx_t;

static tuya_lvgl_widgets_ctx_t s_lvgl_widgets_ctx;

static OPERATE_RET tuya_lvgl_gpu_buffer_prepare(void)
{
#if CONFIG_LV_USE_DRAW_VG_LITE
    uint32_t buffer_size = CONFIG_VG_LITE_GPU_CONTIGUOUS_MEM_SZ + TUYA_LVGL_GPU_BUFFER_GUARD_SIZE;

    if (s_lvgl_widgets_ctx.gpu_contiguous_buffer != NULL) {
        vg_lite_set_buffer((uint8_t *)s_lvgl_widgets_ctx.gpu_contiguous_buffer);
        return OPRT_OK;
    }

    s_lvgl_widgets_ctx.gpu_contiguous_buffer = bk_get_gpu_flexa_buffer(buffer_size);
    if (s_lvgl_widgets_ctx.gpu_contiguous_buffer == NULL) {
        tkl_log_output("lvgl gpu buffer alloc failed, size=%u\r\n",
                       (unsigned int)buffer_size);
        return OPRT_MALLOC_FAILED;
    }

    vg_lite_set_buffer((uint8_t *)s_lvgl_widgets_ctx.gpu_contiguous_buffer);
    tkl_log_output("lvgl gpu buffer=%p, size=%u\r\n",
                   s_lvgl_widgets_ctx.gpu_contiguous_buffer,
                   (unsigned int)buffer_size);
#endif

    return OPRT_OK;
}

static void tuya_lvgl_gpu_buffer_release(void)
{
#if CONFIG_LV_USE_DRAW_VG_LITE
    if (s_lvgl_widgets_ctx.gpu_contiguous_buffer != NULL) {
        tkl_log_output("lvgl gpu buffer free=%p\r\n", s_lvgl_widgets_ctx.gpu_contiguous_buffer);
        os_free(s_lvgl_widgets_ctx.gpu_contiguous_buffer);
        s_lvgl_widgets_ctx.gpu_contiguous_buffer = NULL;
    }

    vg_lite_set_buffer(NULL);
#endif
}

static bool tuya_lvgl_is_aligned(const void *ptr, uint32_t align)
{
    return (((uintptr_t)ptr & ((uintptr_t)align - 1U)) == 0U);
}

static void tuya_lvgl_draw_buffer_free_if_needed(void)
{
#if CONFIG_LV_USE_DRAW_VG_LITE
    if (s_lvgl_widgets_ctx.draw_buffer != NULL &&
        !s_lvgl_widgets_ctx.draw_buffer_owned_by_vendor) {
        os_free(s_lvgl_widgets_ctx.draw_buffer);
    }

    s_lvgl_widgets_ctx.draw_buffer = NULL;
    s_lvgl_widgets_ctx.draw_buffer_owned_by_vendor = false;
#endif
}

static void tuya_lvgl_draw_buffer_forget(void)
{
#if CONFIG_LV_USE_DRAW_VG_LITE
    s_lvgl_widgets_ctx.draw_buffer = NULL;
    s_lvgl_widgets_ctx.draw_buffer_owned_by_vendor = false;
#endif
}

static OPERATE_RET tuya_lvgl_draw_buffer_prepare(lv_vnd_config_t *config)
{
#if CONFIG_LV_USE_DRAW_VG_LITE
    void *pad = NULL;

    if (config == NULL || config->draw_pixel_size == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (s_lvgl_widgets_ctx.draw_buffer != NULL) {
        config->draw_buf_2_1 = s_lvgl_widgets_ctx.draw_buffer;
        config->draw_buf_2_2 = NULL;
        return OPRT_OK;
    }

    for (uint32_t pad_size = 0U; pad_size <= TUYA_LVGL_DRAW_BUFFER_ALIGN_SIZE; pad_size++) {
        void *draw_buffer = NULL;

        if (pad_size != 0U) {
            pad = lv_vendor_malloc(pad_size);
            if (pad == NULL) {
                tkl_log_output("lvgl draw buffer pad alloc failed, size=%u\r\n",
                               (unsigned int)pad_size);
                return OPRT_MALLOC_FAILED;
            }
        }

        draw_buffer = lv_vendor_malloc(config->draw_pixel_size);
        if (draw_buffer == NULL) {
            if (pad != NULL) {
                os_free(pad);
            }
            tkl_log_output("lvgl draw buffer alloc failed, size=%u\r\n",
                           (unsigned int)config->draw_pixel_size);
            return OPRT_MALLOC_FAILED;
        }

        if (tuya_lvgl_is_aligned(draw_buffer, TUYA_LVGL_DRAW_BUFFER_ALIGN_SIZE)) {
            s_lvgl_widgets_ctx.draw_buffer = draw_buffer;
            s_lvgl_widgets_ctx.draw_buffer_owned_by_vendor = false;
            config->draw_buf_2_1 = draw_buffer;
            config->draw_buf_2_2 = NULL;

            if (pad != NULL) {
                os_free(pad);
            }

            tkl_log_output("lvgl draw buffer=%p, size=%u, align=%u, pad=%u\r\n",
                           draw_buffer,
                           (unsigned int)config->draw_pixel_size,
                           (unsigned int)TUYA_LVGL_DRAW_BUFFER_ALIGN_SIZE,
                           (unsigned int)pad_size);
            return OPRT_OK;
        }

        os_free(draw_buffer);
        if (pad != NULL) {
            os_free(pad);
            pad = NULL;
        }
    }

    tkl_log_output("lvgl draw buffer 64-byte alignment failed\r\n");
    return OPRT_MALLOC_FAILED;
#else
    (void)config;
    return OPRT_OK;
#endif
}

static const char *tuya_lvgl_demo_name_get(tuya_lvgl_demo_type_t demo)
{
    switch (demo) {
        case TUYA_LVGL_DEMO_WIDGETS:
            return "widgets";
        case TUYA_LVGL_DEMO_STRESS:
            return "stress";
        default:
            return "none";
    }
}

static void tuya_lvgl_widgets_frame_done(UINT8_T *frame)
{
    if (s_lvgl_widgets_ctx.flush_done_cb != NULL) {
        (void)s_lvgl_widgets_ctx.flush_done_cb(frame);
    }
}

static void tuya_lvgl_widgets_flush(void *args, void *frame_buffer, int (*cb)(void *args))
{
    OPERATE_RET ret;
    tkl_mipi_dsi_flush_frame_t flush_frame;

    (void)args;

    if (frame_buffer == NULL) {
        if (cb != NULL) {
            (void)cb(frame_buffer);
        }
        return;
    }

    os_memset(&flush_frame, 0, sizeof(flush_frame));
    flush_frame.buffer = (UINT8_T *)frame_buffer;
    flush_frame.free_cb = tuya_lvgl_widgets_frame_done;
    s_lvgl_widgets_ctx.flush_done_cb = cb;

    ret = tkl_mipi_dsi_flush(&flush_frame);
    if (ret != OPRT_OK) {
        tkl_log_output("lvgl mipi flush failed: %d\r\n", ret);
        tuya_lvgl_widgets_frame_done((UINT8_T *)frame_buffer);
    }
}

static uint32_t tuya_lvgl_widgets_frame_size_get(uint16_t width, uint16_t height, bool output_compress)
{
    uint16_t disp_width = width;
    uint16_t disp_height = height;

    if (output_compress && (disp_width % 16U || disp_height % 4U)) {
        disp_width = (uint16_t)((disp_width + 15U) & ~15U);
        disp_height = (uint16_t)((disp_height + 3U) & ~3U);
    }

    return (uint32_t)disp_width * disp_height * sizeof(bk_color_t);
}

static OPERATE_RET tuya_lvgl_widgets_frame_buffers_alloc(uint16_t width, uint16_t height, bool output_compress)
{
    uint32_t frame_size = tuya_lvgl_widgets_frame_size_get(width, height, output_compress);

    if (frame_size == 0U) {
        return OPRT_INVALID_PARM;
    }

    for (int i = 0; i < CONFIG_LVGL_FRAME_BUFFER_NUM; i++) {
        frame_buffer_heap_type_t heap_type = (i % 2) ? MEM_SLAB_HEAP_UNCODED : MEM_SLAB_HEAP_CODED;

        s_lvgl_widgets_ctx.frame_buffer[i] = bk_frame_buffer_malloc(heap_type, frame_size);
        if (s_lvgl_widgets_ctx.frame_buffer[i] == NULL) {
            tkl_log_output("lvgl frame[%d] alloc failed, size=%u\r\n", i, frame_size);
            return OPRT_MALLOC_FAILED;
        }
    }

    return OPRT_OK;
}

static void tuya_lvgl_widgets_frame_buffers_free(void)
{
    for (int i = 0; i < CONFIG_LVGL_FRAME_BUFFER_NUM; i++) {
        if (s_lvgl_widgets_ctx.frame_buffer[i] != NULL) {
            bk_frame_buffer_free(s_lvgl_widgets_ctx.frame_buffer[i]);
            s_lvgl_widgets_ctx.frame_buffer[i] = NULL;
        }
    }
}

static const char *tuya_lvgl_widgets_state_get(void)
{
    if (s_lvgl_widgets_ctx.started) {
        return "started";
    }

    if (s_lvgl_widgets_ctx.inited) {
        return "stopped";
    }

    return "released";
}

static OPERATE_RET tuya_lvgl_widgets_resume(uint16_t width, uint16_t height, bool output_compress)
{
    if (s_lvgl_widgets_ctx.width != width ||
        s_lvgl_widgets_ctx.height != height ||
        s_lvgl_widgets_ctx.output_compress != output_compress) {
        tkl_log_output("lvgl %s config changed, release after lcd close first\r\n",
                       tuya_lvgl_demo_name_get(s_lvgl_widgets_ctx.demo));
        return OPRT_COM_ERROR;
    }

    lv_vendor_disp_lock();
    if (s_lvgl_widgets_ctx.demo == TUYA_LVGL_DEMO_STRESS) {
        tuya_lvgl_stress_resume();
    }
    lv_obj_invalidate(lv_screen_active());
    lv_vendor_disp_unlock();

    lv_vendor_start();
    s_lvgl_widgets_ctx.started = true;

    tkl_log_output("lvgl %s resume ok, size=%ux%u, compress=%u\r\n",
                   tuya_lvgl_demo_name_get(s_lvgl_widgets_ctx.demo),
                   width,
                   height,
                   output_compress ? 1U : 0U);
    return OPRT_OK;
}

static OPERATE_RET tuya_lvgl_demo_create(lv_vnd_config_t *lv_vnd_config,
                                         tuya_lvgl_demo_type_t demo)
{
    OPERATE_RET ret;

    if (lv_vnd_config == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = tuya_lvgl_gpu_buffer_prepare();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tuya_lvgl_draw_buffer_prepare(lv_vnd_config);
    if (ret != OPRT_OK) {
        tuya_lvgl_draw_buffer_free_if_needed();
        return ret;
    }

    if (lv_vendor_init(lv_vnd_config) != BK_OK) {
        tkl_log_output("lv_vendor_init failed\r\n");
        tuya_lvgl_draw_buffer_free_if_needed();
        return OPRT_COM_ERROR;
    }
    s_lvgl_widgets_ctx.draw_buffer_owned_by_vendor = true;

    lv_vendor_disp_lock();
    if (demo == TUYA_LVGL_DEMO_WIDGETS) {
        lv_demo_widgets();
    } else {
        tuya_lvgl_stress_create();
    }
    lv_vendor_disp_unlock();

    lv_vendor_start();
    return OPRT_OK;
}

static OPERATE_RET tuya_lvgl_demo_start(tuya_lvgl_demo_type_t demo, bool output_compress)
{
    OPERATE_RET ret;
    UINT16_T width = 0;
    UINT16_T height = 0;
    lv_vnd_config_t lv_vnd_config;

    if (s_lvgl_widgets_ctx.started) {
        tkl_log_output("lvgl %s already started\r\n",
                       tuya_lvgl_demo_name_get(s_lvgl_widgets_ctx.demo));
        return OPRT_OK;
    }

    if (test_mipi_lcd_is_open() != TRUE) {
        tkl_log_output("mipi lcd is not open, use xmipi lcd open first\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    test_mipi_lcd_get_geometry(&width, &height);
    if (width < TUYA_LVGL_MIN_WIDTH || height < TUYA_LVGL_MIN_HEIGHT) {
        tkl_log_output("invalid mipi geometry: %ux%u\r\n", width, height);
        return OPRT_INVALID_PARM;
    }

    if (s_lvgl_widgets_ctx.inited) {
        if (s_lvgl_widgets_ctx.demo != demo) {
            tkl_log_output("lvgl %s is inited, release after lcd close first\r\n",
                           tuya_lvgl_demo_name_get(s_lvgl_widgets_ctx.demo));
            return OPRT_COM_ERROR;
        }
        return tuya_lvgl_widgets_resume(width, height, output_compress);
    }

    os_memset(&s_lvgl_widgets_ctx, 0, sizeof(s_lvgl_widgets_ctx));
    s_lvgl_widgets_ctx.width = width;
    s_lvgl_widgets_ctx.height = height;
    s_lvgl_widgets_ctx.disp_width = width;
    s_lvgl_widgets_ctx.disp_height = height;
    s_lvgl_widgets_ctx.output_compress = output_compress;
    s_lvgl_widgets_ctx.demo = demo;

    if (output_compress && (s_lvgl_widgets_ctx.disp_width % 16U || s_lvgl_widgets_ctx.disp_height % 4U)) {
        s_lvgl_widgets_ctx.disp_width = (uint16_t)((s_lvgl_widgets_ctx.disp_width + 15U) & ~15U);
        s_lvgl_widgets_ctx.disp_height = (uint16_t)((s_lvgl_widgets_ctx.disp_height + 3U) & ~3U);
    }

    ret = tuya_lvgl_widgets_frame_buffers_alloc(width, height, output_compress);
    if (ret != OPRT_OK) {
        tuya_lvgl_widgets_frame_buffers_free();
        return ret;
    }

    os_memset(&lv_vnd_config, 0, sizeof(lv_vnd_config));
    lv_vnd_config.width = width;
    lv_vnd_config.height = height;
    lv_vnd_config.render_mode = RENDER_PARTIAL_MODE;
    lv_vnd_config.rotation = ROTATE_NONE;
    lv_vnd_config.draw_pixel_size = (uint32_t)width * TUYA_LVGL_DRAW_LINES * sizeof(bk_color_t);
    lv_vnd_config.flush_cb = tuya_lvgl_widgets_flush;
    lv_vnd_config.output_compress = output_compress;
    lv_vnd_config.disp_width = s_lvgl_widgets_ctx.disp_width;
    lv_vnd_config.disp_height = s_lvgl_widgets_ctx.disp_height;

    for (int i = 0; i < CONFIG_LVGL_FRAME_BUFFER_NUM; i++) {
        lv_vnd_config.frame_buffer[i] = s_lvgl_widgets_ctx.frame_buffer[i];
    }

    ret = tuya_lvgl_demo_create(&lv_vnd_config, demo);
    if (ret != OPRT_OK) {
        tuya_lvgl_widgets_frame_buffers_free();
        os_memset(&s_lvgl_widgets_ctx, 0, sizeof(s_lvgl_widgets_ctx));
        return ret;
    }

    s_lvgl_widgets_ctx.inited = true;
    s_lvgl_widgets_ctx.started = true;

    tkl_log_output("lvgl %s start ok, size=%ux%u, compress=%u\r\n",
                   tuya_lvgl_demo_name_get(demo),
                   width,
                   height,
                   output_compress ? 1U : 0U);
    return OPRT_OK;
}

static OPERATE_RET tuya_lvgl_demo_stop(tuya_lvgl_demo_type_t demo)
{
    if (s_lvgl_widgets_ctx.inited && s_lvgl_widgets_ctx.demo != demo) {
        tkl_log_output("lvgl %s is active, stop it first\r\n",
                       tuya_lvgl_demo_name_get(s_lvgl_widgets_ctx.demo));
        return OPRT_COM_ERROR;
    }

    if (!s_lvgl_widgets_ctx.started) {
        tkl_log_output("lvgl %s already stopped\r\n", tuya_lvgl_demo_name_get(demo));
        return OPRT_OK;
    }

    if (s_lvgl_widgets_ctx.demo == TUYA_LVGL_DEMO_STRESS) {
        lv_vendor_disp_lock();
        tuya_lvgl_stress_pause();
        lv_vendor_disp_unlock();
    }

    lv_vendor_stop();
    s_lvgl_widgets_ctx.started = false;

    tkl_log_output("lvgl %s stop ok\r\n", tuya_lvgl_demo_name_get(demo));
    return OPRT_OK;
}

static OPERATE_RET tuya_lvgl_demo_release(tuya_lvgl_demo_type_t demo)
{
    if (s_lvgl_widgets_ctx.inited && s_lvgl_widgets_ctx.demo != demo) {
        tkl_log_output("lvgl %s is active, release it first\r\n",
                       tuya_lvgl_demo_name_get(s_lvgl_widgets_ctx.demo));
        return OPRT_COM_ERROR;
    }

    if (s_lvgl_widgets_ctx.started) {
        tkl_log_output("lvgl %s is started, stop first\r\n", tuya_lvgl_demo_name_get(demo));
        return OPRT_COM_ERROR;
    }

    if (!s_lvgl_widgets_ctx.inited) {
        tkl_log_output("lvgl %s already released\r\n", tuya_lvgl_demo_name_get(demo));
        return OPRT_OK;
    }

    if (test_mipi_lcd_is_open() == TRUE) {
        tkl_log_output("mipi lcd is open, close lcd before release\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_lvgl_widgets_ctx.demo == TUYA_LVGL_DEMO_STRESS) {
        lv_vendor_disp_lock();
        tuya_lvgl_stress_destroy();
        lv_vendor_disp_unlock();
    }

    lv_vendor_deinit();
    tuya_lvgl_gpu_buffer_release();
    tuya_lvgl_draw_buffer_forget();
    tuya_lvgl_widgets_frame_buffers_free();
    os_memset(&s_lvgl_widgets_ctx, 0, sizeof(s_lvgl_widgets_ctx));

    tkl_log_output("lvgl %s release ok\r\n", tuya_lvgl_demo_name_get(demo));
    return OPRT_OK;
}

static bool tuya_lvgl_parse_bool_arg(const char *arg)
{
    if (arg == NULL) {
        return false;
    }

    if (os_strcmp(arg, "1") == 0 ||
        os_strcmp(arg, "true") == 0 ||
        os_strcmp(arg, "compress") == 0 ||
        os_strcmp(arg, "compressed") == 0) {
        return true;
    }

    return false;
}

static tuya_lvgl_demo_type_t tuya_lvgl_demo_arg_parse(const char *arg)
{
    if (arg == NULL) {
        return TUYA_LVGL_DEMO_NONE;
    }

    if (os_strcmp(arg, "widgets") == 0) {
        return TUYA_LVGL_DEMO_WIDGETS;
    }

    if (os_strcmp(arg, "stress") == 0) {
        return TUYA_LVGL_DEMO_STRESS;
    }

    return TUYA_LVGL_DEMO_NONE;
}

void cli_tuya_lvgl_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret;
    const char *cmd = NULL;
    bool output_compress = false;
    tuya_lvgl_demo_type_t demo;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 3) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xlvgl <widgets|stress> <start [compress]|stop|release|status>");
        return;
    }

    demo = tuya_lvgl_demo_arg_parse(argv[1]);
    cmd = argv[2];

    if (demo == TUYA_LVGL_DEMO_NONE) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xlvgl <widgets|stress> <start [compress]|stop|release|status>");
        return;
    }

    if (os_strcmp(cmd, "start") == 0) {
        if (argc >= 4) {
            output_compress = tuya_lvgl_parse_bool_arg(argv[3]);
        }

        ret = tuya_lvgl_demo_start(demo, output_compress);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xlvgl %s start %s (%d), compress=%u\r\n",
                 tuya_lvgl_demo_name_get(demo),
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 output_compress ? 1U : 0U);
    } else if (os_strcmp(cmd, "stop") == 0) {
        ret = tuya_lvgl_demo_stop(demo);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xlvgl %s stop %s (%d)\r\n",
                 tuya_lvgl_demo_name_get(demo),
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "release") == 0) {
        ret = tuya_lvgl_demo_release(demo);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xlvgl %s release %s (%d)\r\n",
                 tuya_lvgl_demo_name_get(demo),
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "status") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xlvgl %s, active=%s, size=%ux%u, compress=%u\r\n",
                 tuya_lvgl_widgets_state_get(),
                 tuya_lvgl_demo_name_get(s_lvgl_widgets_ctx.demo),
                 s_lvgl_widgets_ctx.width,
                 s_lvgl_widgets_ctx.height,
                 s_lvgl_widgets_ctx.output_compress ? 1U : 0U);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xlvgl <widgets|stress> <start [compress]|stop|release|status>");
    }
}

#endif
