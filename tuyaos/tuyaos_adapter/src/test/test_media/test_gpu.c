#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <components/bk_frame_buffer.h>
#include <common/avdk_pixel_types.h>
#include <os/mem.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_camera_mipi.h"
#include "tkl_display_mipi.h"
#include "tkl_gpu.h"
#include "tkl_output.h"

#define GPU_TEST_DEFAULT_ROTATE    90U
#define GPU_TEST_DEFAULT_COMPRESS  0U
#define GPU_TEST_FLEXA_LINES       16U

typedef struct {
    bool initialized;
    bool started;
    uint16_t rotate_degree;
    bool compress;
    uint16_t lcd_width;
    uint16_t lcd_height;
    uint16_t src_width;
    uint16_t src_height;
    uint32_t src_format;
} test_gpu_ctx_t;

static test_gpu_ctx_t s_gpu_ctx = {
    .initialized = false,
    .started = false,
    .rotate_degree = GPU_TEST_DEFAULT_ROTATE,
    .compress = (GPU_TEST_DEFAULT_COMPRESS != 0U),
    .lcd_width = 0U,
    .lcd_height = 0U,
    .src_width = 0U,
    .src_height = 0U,
    .src_format = BK_PIXEL_FORMAT_INVALID,
};

static void test_gpu_runtime_reset(void)
{
    s_gpu_ctx.initialized = false;
    s_gpu_ctx.started = false;
    s_gpu_ctx.rotate_degree = GPU_TEST_DEFAULT_ROTATE;
    s_gpu_ctx.compress = (GPU_TEST_DEFAULT_COMPRESS != 0U);
    s_gpu_ctx.lcd_width = 0U;
    s_gpu_ctx.lcd_height = 0U;
    s_gpu_ctx.src_width = 0U;
    s_gpu_ctx.src_height = 0U;
    s_gpu_ctx.src_format = BK_PIXEL_FORMAT_INVALID;
}

static void test_gpu_dst_geometry_get(uint16_t rotate_degree, uint16_t *dst_width, uint16_t *dst_height)
{
    if (dst_width == NULL || dst_height == NULL) {
        return;
    }

    if (rotate_degree == 90U || rotate_degree == 270U) {
        *dst_width = s_gpu_ctx.lcd_height;
        *dst_height = s_gpu_ctx.lcd_width;
    } else {
        *dst_width = s_gpu_ctx.lcd_width;
        *dst_height = s_gpu_ctx.lcd_height;
    }
}

static OPERATE_RET test_gpu_dst_validate(uint16_t rotate_degree, bool compress)
{
    uint16_t dst_width = 0U;
    uint16_t dst_height = 0U;

    test_gpu_dst_geometry_get(rotate_degree, &dst_width, &dst_height);
    if (dst_width == 0U || dst_height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (compress && ((dst_width % 4U) != 0U)) {
        tkl_log_output(
                "gpu compress invalid dst=%ux%u from lcd=%ux%u rotate=%u, dst_width must align to 4, check xmipi lcd open_gpu panel\r\n",
                dst_width,
                dst_height,
                s_gpu_ctx.lcd_width,
                s_gpu_ctx.lcd_height,
                rotate_degree);
        return OPRT_INVALID_PARM;
    }

    tkl_log_output(
            "gpu prepare: lcd=%ux%u rotate=%u -> dst=%ux%u compress=%u\r\n",
            s_gpu_ctx.lcd_width,
            s_gpu_ctx.lcd_height,
            rotate_degree,
            dst_width,
            dst_height,
            (unsigned int)compress);
    return OPRT_OK;
}

static void test_gpu_display_frame_free(UINT8_T *frame_buff)
{
    if (frame_buff != NULL) {
        bk_frame_buffer_free(frame_buff);
    }
}

static VOID *test_gpu_frame_alloc(UINT32_T size, VOID *arg)
{
    (void)arg;
    return bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, size);
}

static OPERATE_RET test_gpu_frame_free(VOID *ptr, VOID *arg)
{
    (void)arg;

    if (ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    bk_frame_buffer_free(ptr);
    return OPRT_OK;
}

static uint32_t test_gpu_frame_len_get(uint16_t width, uint16_t height, bool compress)
{
    uint32_t pixel_size = bk_pixel_size_get(BK_PIXEL_FORMAT_ARGB8888);

    if (pixel_size == 0U || width == 0U || height == 0U) {
        return 0U;
    }

    return pixel_size * (compress ? (width / 4U) : width) * height;
}

static void test_gpu_frame_done(void *frame, uint32_t frame_size, void *arg)
{
    OPERATE_RET ret;
    ty_frame_buffer_t frame_buff;
    ty_display_mipi_device_t *lcd_dev = (ty_display_mipi_device_t *)arg;

    if (frame == NULL) {
        return;
    }

    if (lcd_dev == NULL || lcd_dev->opened != TRUE) {
        test_gpu_display_frame_free((UINT8_T *)frame);
        return;
    }

    os_memset(&frame_buff, 0, sizeof(frame_buff));
    frame_buff.type = TYPE_PSRAM;
    frame_buff.fmt = TY_PIXEL_FMT_ARGB8888;
    frame_buff.x_start = 0U;
    frame_buff.y_start = 0U;
    frame_buff.width = s_gpu_ctx.lcd_width;
    frame_buff.height = s_gpu_ctx.lcd_height;
    frame_buff.len = (frame_size != 0U) ? frame_size :
                     test_gpu_frame_len_get(s_gpu_ctx.lcd_width, s_gpu_ctx.lcd_height, s_gpu_ctx.compress);
    frame_buff.frame = (uint8_t *)frame;
    frame_buff.pdata = (uint8_t *)frame;
    frame_buff.free_cb = test_gpu_display_frame_free;

    ret = tkl_display_mipi_flush(lcd_dev, &frame_buff);
    if (ret != OPRT_OK) {
        tkl_log_output( "gpu display flush failed: %d\r\n", ret);
        test_gpu_display_frame_free((UINT8_T *)frame);
    }
}

static OPERATE_RET test_gpu_prepare_display(uint16_t rotate_degree, bool compress)
{
    OPERATE_RET ret;
    ty_display_mipi_device_t *lcd_dev;

    if (test_mipi_lcd_is_open() != TRUE) {
        tkl_log_output( "mipi lcd is not open, use: xmipi lcd open_gpu [panel] [%u]\r\n", (unsigned int)compress);
        return OPRT_RESOURCE_NOT_READY;
    }

    lcd_dev = test_mipi_lcd_device_get();
    if (lcd_dev == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (lcd_dev->dpu_video_cfg == NULL || lcd_dev->dpu_video_cfg->format != BK_PIXEL_FORMAT_ARGB8888) {
        tkl_log_output( "mipi lcd format mismatch, need ARGB8888, use: xmipi lcd open_gpu ...\r\n");
        return OPRT_INVALID_PARM;
    }

    if (lcd_dev->dpu_video_cfg->decompress != (compress ? TRUE : FALSE)) {
        tkl_log_output( "mipi lcd decompress mismatch, current=%u, need=%u, reopen with xmipi lcd open_gpu ... %u\r\n",
                (unsigned int)lcd_dev->dpu_video_cfg->decompress,
                (unsigned int)(compress ? TRUE : FALSE),
                (unsigned int)compress);
        return OPRT_INVALID_PARM;
    }

    test_mipi_lcd_get_geometry(&s_gpu_ctx.lcd_width, &s_gpu_ctx.lcd_height);
    if (s_gpu_ctx.lcd_width == 0U || s_gpu_ctx.lcd_height == 0U) {
        return OPRT_INVALID_PARM;
    }

    s_gpu_ctx.rotate_degree = rotate_degree;
    s_gpu_ctx.compress = compress;
    ret = test_gpu_dst_validate(rotate_degree, compress);
    if (ret != OPRT_OK) {
        return ret;
    }

    return OPRT_OK;
}

static OPERATE_RET test_gpu_get_camera_source(TUYA_CAMERA_MIPI_FLEXA_SOURCE_T *camera_source)
{
    if (camera_source == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (test_mipi_camera_is_inited() != TRUE || test_mipi_camera_is_started() != TRUE) {
        tkl_log_output( "mipi camera is not started, use: xmipi camera open_gpu ... && xmipi camera start\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (test_mipi_camera_flexa_enabled() != TRUE) {
        tkl_log_output( "mipi camera flexa is not enabled, use: xmipi camera open_gpu ...\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    os_memset(camera_source, 0, sizeof(*camera_source));
    return test_mipi_camera_get_flexa_source(camera_source);
}

static OPERATE_RET test_gpu_init(uint16_t rotate_degree, bool compress)
{
    OPERATE_RET ret;
    TUYA_CAMERA_MIPI_FLEXA_SOURCE_T camera_source;
    TUYA_GPU_CFG_T gpu_cfg;
    TUYA_GPU_SOURCE_T gpu_source;
    TUYA_GPU_SINK_T gpu_sink;
    ty_display_mipi_device_t *lcd_dev;
    uint16_t dst_width = 0U;
    uint16_t dst_height = 0U;

    if (s_gpu_ctx.started == true) {
        tkl_log_output( "gpu pipeline is running, stop before reinit\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = test_gpu_prepare_display(rotate_degree, compress);
    if (ret != OPRT_OK) {
        return ret;
    }

    lcd_dev = test_mipi_lcd_device_get();
    if (lcd_dev == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    test_gpu_dst_geometry_get(rotate_degree, &dst_width, &dst_height);

    ret = test_gpu_get_camera_source(&camera_source);
    if (ret != OPRT_OK) {
        tkl_log_output( "camera get flexa source failed: %d\r\n", ret);
        return ret;
    }

    if (s_gpu_ctx.initialized == true) {
        ret = tkl_gpu_deinit();
        if (ret != OPRT_OK) {
            tkl_log_output( "gpu deinit before reinit failed: %d\r\n", ret);
            return ret;
        }
    }

    os_memset(&gpu_cfg, 0, sizeof(gpu_cfg));
    gpu_cfg.rotate_degree = rotate_degree;
    gpu_cfg.dst_format = BK_PIXEL_FORMAT_ARGB8888;
    gpu_cfg.scale = TRUE;
    gpu_cfg.dst_compress = compress ? TRUE : FALSE;
    gpu_cfg.flexa_lines = GPU_TEST_FLEXA_LINES;
    gpu_cfg.dst_width = dst_width;
    gpu_cfg.dst_height = dst_height;

    ret = tkl_gpu_init(&gpu_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output( "gpu init failed: %d\r\n", ret);
        return ret;
    }

    os_memset(&gpu_sink, 0, sizeof(gpu_sink));
    gpu_sink.alloc_cb = test_gpu_frame_alloc;
    gpu_sink.free_cb = test_gpu_frame_free;
    gpu_sink.frame_done_cb = test_gpu_frame_done;
    gpu_sink.cb_arg = lcd_dev;

    ret = tkl_gpu_set_sink(&gpu_sink);
    if (ret != OPRT_OK) {
        tkl_log_output( "gpu set sink failed: %d\r\n", ret);
        (void)tkl_gpu_deinit();
        return ret;
    }

    os_memset(&gpu_source, 0, sizeof(gpu_source));
    gpu_source.type = TUYA_GPU_SOURCE_ISP_FLEXA;
    gpu_source.isp.isp_handle = camera_source.isp_handle;
    gpu_source.isp.flexa.src_buffer = camera_source.src_buffer;
    gpu_source.isp.flexa.buf_cnt = camera_source.buf_cnt;
    gpu_source.isp.flexa.width = camera_source.width;
    gpu_source.isp.flexa.height = camera_source.height;
    gpu_source.isp.flexa.format = camera_source.format;

    ret = tkl_gpu_set_source(&gpu_source);
    if (ret != OPRT_OK) {
        tkl_log_output( "gpu set source failed: %d\r\n", ret);
        (void)tkl_gpu_deinit();
        return ret;
    }

    s_gpu_ctx.initialized = true;
    s_gpu_ctx.started = false;
    s_gpu_ctx.src_width = camera_source.width;
    s_gpu_ctx.src_height = camera_source.height;
    s_gpu_ctx.src_format = camera_source.format;
    tkl_log_output( "gpu pipeline initialized, src=%ux%u fmt=%u -> dst=%ux%u (lcd=%ux%u) rotate=%u compress=%u\r\n",
            s_gpu_ctx.src_width,
            s_gpu_ctx.src_height,
            (unsigned int)s_gpu_ctx.src_format,
            dst_width,
            dst_height,
            s_gpu_ctx.lcd_width,
            s_gpu_ctx.lcd_height,
            rotate_degree,
            (unsigned int)compress);
    return OPRT_OK;
}

static OPERATE_RET test_gpu_start(void)
{
    OPERATE_RET ret;
    uint16_t dst_width = 0U;
    uint16_t dst_height = 0U;

    if (s_gpu_ctx.initialized != true) {
        tkl_log_output( "gpu is not initialized, use: xgpu init [rotate] [compress]\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_gpu_ctx.started == true) {
        tkl_log_output( "gpu pipeline already started\r\n");
        return OPRT_OK;
    }

    test_gpu_dst_geometry_get(s_gpu_ctx.rotate_degree, &dst_width, &dst_height);

    tkl_log_output(
            "gpu start request: src=%ux%u fmt=%u -> dst=%ux%u (lcd=%ux%u) rotate=%u compress=%u\r\n",
            s_gpu_ctx.src_width,
            s_gpu_ctx.src_height,
            (unsigned int)s_gpu_ctx.src_format,
            dst_width,
            dst_height,
            s_gpu_ctx.lcd_width,
            s_gpu_ctx.lcd_height,
            s_gpu_ctx.rotate_degree,
            (unsigned int)s_gpu_ctx.compress);

    ret = tkl_gpu_start();
    if (ret != OPRT_OK) {
        tkl_log_output( "gpu start failed: %d\r\n", ret);
        return ret;
    }

    s_gpu_ctx.started = true;
    tkl_log_output( "gpu pipeline started, src=%ux%u fmt=%u -> dst=%ux%u (lcd=%ux%u) rotate=%u compress=%u\r\n",
            s_gpu_ctx.src_width,
            s_gpu_ctx.src_height,
            (unsigned int)s_gpu_ctx.src_format,
            dst_width,
            dst_height,
            s_gpu_ctx.lcd_width,
            s_gpu_ctx.lcd_height,
            s_gpu_ctx.rotate_degree,
            (unsigned int)s_gpu_ctx.compress);
    return OPRT_OK;
}

static OPERATE_RET test_gpu_stop(void)
{
    OPERATE_RET ret;

    if (s_gpu_ctx.initialized != true) {
        return OPRT_OK;
    }

    ret = tkl_gpu_stop();
    if (ret != OPRT_OK) {
        return ret;
    }

    s_gpu_ctx.started = false;
    return OPRT_OK;
}

static OPERATE_RET test_gpu_deinit(void)
{
    OPERATE_RET ret;

    if (s_gpu_ctx.initialized != true) {
        test_gpu_runtime_reset();
        return OPRT_OK;
    }

    ret = tkl_gpu_deinit();
    if (ret != OPRT_OK) {
        return ret;
    }

    test_gpu_runtime_reset();
    return OPRT_OK;
}

void cli_tuya_gpu_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    uint16_t rotate_degree = GPU_TEST_DEFAULT_ROTATE;
    bool compress = (GPU_TEST_DEFAULT_COMPRESS != 0U);

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xgpu <init [rotate] [compress]|start|stop|deinit|close|status>");
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "init") == 0) {
        if (argc >= 3) {
            rotate_degree = (uint16_t)os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            compress = (os_strtoul(argv[3], NULL, 10) != 0U);
        }

        ret = test_gpu_init(rotate_degree, compress);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xgpu init %s (%d), rotate=%u, compress=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 rotate_degree,
                 (unsigned int)compress);
    } else if (os_strcmp(cmd, "start") == 0) {
        ret = test_gpu_start();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xgpu start %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "stop") == 0) {
        ret = test_gpu_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xgpu stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "deinit") == 0 || os_strcmp(cmd, "close") == 0) {
        ret = test_gpu_deinit();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xgpu %s %s (%d)\r\n",
                 cmd,
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "status") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xgpu status gpu_init=%u gpu_start=%u lcd_open=%u camera_init=%u camera_start=%u camera_flexa=%u lcd=%ux%u rotate=%u compress=%u\r\n",
                 (unsigned int)s_gpu_ctx.initialized,
                 (unsigned int)s_gpu_ctx.started,
                 (unsigned int)test_mipi_lcd_is_open(),
                 (unsigned int)test_mipi_camera_is_inited(),
                 (unsigned int)test_mipi_camera_is_started(),
                 (unsigned int)test_mipi_camera_flexa_enabled(),
                 s_gpu_ctx.lcd_width,
                 s_gpu_ctx.lcd_height,
                 s_gpu_ctx.rotate_degree,
                 (unsigned int)s_gpu_ctx.compress);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xgpu <init [rotate] [compress]|start|stop|deinit|close|status>");
    }
}
