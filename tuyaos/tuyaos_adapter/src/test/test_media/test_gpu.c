#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <components/bk_frame_buffer.h>
#include <common/avdk_pixel_types.h>
#if (CONFIG_PSRAM_WRITE_THROUGH)
#include <common/bk_err.h>
#include <driver/psram.h>
#include <driver/psram_types.h>
#endif
#include <os/mem.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_camera_mipi.h"
#include "tkl_mipi_dsi.h"
#include "tkl_gpu.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_thread.h"
#include "tkl_vcodec_types.h"
#include "blend.h"

#define GPU_TEST_DEFAULT_ROTATE      90U
#define GPU_TEST_DEFAULT_COMPRESS    0U
#define GPU_TEST_FLEXA_LINES         16U
#define GPU_TEST_OSD_TIME_REFRESH_MS 1000U
#define GPU_TEST_OSD_TIME_STACK      (3U * 1024U)
#define GPU_TEST_OSD_TIME_TEXT_LEN   16U
#define GPU_TEST_OSD_TEXT_NAME       "clock"

typedef enum {
    GPU_TEST_SOURCE_CAMERA = 0,
    GPU_TEST_SOURCE_VDEC,
} test_gpu_source_t;

typedef struct {
    bool initialized;
    bool started;
    test_gpu_source_t source;
    uint16_t rotate_degree;
    bool compress;
    uint16_t target_width;
    uint16_t target_height;
    uint16_t src_width;
    uint16_t src_height;
    uint32_t src_format;
} test_gpu_ctx_t;

static test_gpu_ctx_t s_gpu_ctx = {
    .initialized = false,
    .started = false,
    .source = GPU_TEST_SOURCE_CAMERA,
    .rotate_degree = GPU_TEST_DEFAULT_ROTATE,
    .compress = (GPU_TEST_DEFAULT_COMPRESS != 0U),
    .target_width = 0U,
    .target_height = 0U,
    .src_width = 0U,
    .src_height = 0U,
    .src_format = BK_PIXEL_FORMAT_INVALID,
};

typedef struct {
    volatile bool running;
    volatile bool stop_req;
    TKL_THREAD_HANDLE task;
    char last_text[GPU_TEST_OSD_TIME_TEXT_LEN];
} test_gpu_osd_time_ctx_t;

static test_gpu_osd_time_ctx_t s_gpu_osd_time = {
    .running = false,
    .stop_req = false,
    .task = NULL,
    .last_text = {0},
};

#if (CONFIG_PSRAM_WRITE_THROUGH)
static uint32_t s_gpu_psram_cover_area;
static bool s_gpu_psram_wt_ready = false;
#endif

extern BOOL_T test_mipi_lcd_is_open(void);
extern BOOL_T test_mipi_camera_is_inited(void);
extern BOOL_T test_mipi_camera_is_started(void);
extern BOOL_T test_mipi_camera_flexa_enabled(void);
extern OPERATE_RET test_mipi_camera_get_flexa_source(TUYA_CAMERA_MIPI_FLEXA_SOURCE_T *camera_source);
extern OPERATE_RET test_vdec_get_flexa_source(TUYA_VCODEC_FLEXA_BUFFER_T *flexa);

static const char *test_gpu_source_name(test_gpu_source_t source)
{
    return (source == GPU_TEST_SOURCE_VDEC) ? "vdec" : "camera";
}

static void test_gpu_runtime_reset(void)
{
    s_gpu_ctx.initialized = false;
    s_gpu_ctx.started = false;
    s_gpu_ctx.source = GPU_TEST_SOURCE_CAMERA;
    s_gpu_ctx.rotate_degree = GPU_TEST_DEFAULT_ROTATE;
    s_gpu_ctx.compress = (GPU_TEST_DEFAULT_COMPRESS != 0U);
    s_gpu_ctx.target_width = 0U;
    s_gpu_ctx.target_height = 0U;
    s_gpu_ctx.src_width = 0U;
    s_gpu_ctx.src_height = 0U;
    s_gpu_ctx.src_format = BK_PIXEL_FORMAT_INVALID;
}

static void test_gpu_osd_time_stop(void);

static void test_gpu_dst_geometry_get(uint16_t rotate_degree, uint16_t *dst_width, uint16_t *dst_height)
{
    (void)rotate_degree;

    if (dst_width == NULL || dst_height == NULL) {
        return;
    }

    *dst_width = s_gpu_ctx.target_width;
    *dst_height = s_gpu_ctx.target_height;
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
                "gpu compress invalid dst=%ux%u from target=%ux%u rotate=%u, dst_width must align to 4\r\n",
                dst_width,
                dst_height,
                s_gpu_ctx.target_width,
                s_gpu_ctx.target_height,
                rotate_degree);
        return OPRT_INVALID_PARM;
    }

    tkl_log_output(
            "gpu prepare: target=%ux%u rotate=%u -> dst=%ux%u compress=%u\r\n",
            s_gpu_ctx.target_width,
            s_gpu_ctx.target_height,
            rotate_degree,
            dst_width,
            dst_height,
            (unsigned int)compress);
    return OPRT_OK;
}

static OPERATE_RET test_gpu_psram_wt_open(void)
{
#if (CONFIG_PSRAM_WRITE_THROUGH)
    if (s_gpu_psram_wt_ready == true) {
        return OPRT_OK;
    }

    s_gpu_psram_cover_area = bk_psram_alloc_write_through_channel_with_psram_id(0);
    if (s_gpu_psram_cover_area >= PSRAM_WRITE_THROUGH_AREA_COUNT) {
        tkl_log_output("gpu alloc psram write-through channel failed, area=%u\r\n",
                       (unsigned int)s_gpu_psram_cover_area);
        return OPRT_COM_ERROR;
    }

    s_gpu_psram_wt_ready = true;
#endif
    return OPRT_OK;
}

static void test_gpu_psram_wt_close(void)
{
#if (CONFIG_PSRAM_WRITE_THROUGH)
    if (s_gpu_psram_wt_ready != true) {
        return;
    }

    (void)bk_psram_disable_write_through(s_gpu_psram_cover_area);
    (void)bk_psram_free_write_through_channel((psram_write_through_area_t)s_gpu_psram_cover_area);
    s_gpu_psram_wt_ready = false;
#endif
}

static OPERATE_RET test_gpu_psram_wt_enable(void *ptr, uint32_t size)
{
#if (CONFIG_PSRAM_WRITE_THROUGH)
    uint32_t start;
    uint32_t end;

    if (s_gpu_psram_wt_ready != true || ptr == NULL || size == 0U) {
        return OPRT_OK;
    }

    start = (uint32_t)(uintptr_t)ptr;
    end = start + size;
    if (bk_psram_enable_write_through(s_gpu_psram_cover_area, start, end) != BK_OK) {
        tkl_log_output("gpu enable psram write-through failed: buf=%p size=%u\r\n",
                       ptr,
                       (unsigned int)size);
        return OPRT_COM_ERROR;
    }
#else
    (void)ptr;
    (void)size;
#endif
    return OPRT_OK;
}

static void test_gpu_psram_wt_disable(void)
{
#if (CONFIG_PSRAM_WRITE_THROUGH)
    if (s_gpu_psram_wt_ready == true) {
        (void)bk_psram_disable_write_through(s_gpu_psram_cover_area);
    }
#endif
}

static void test_gpu_display_frame_free(UINT8_T *frame_buff)
{
    if (frame_buff != NULL) {
        test_gpu_psram_wt_disable();
        bk_frame_buffer_free(frame_buff);
    }
}

static VOID *test_gpu_frame_alloc(UINT32_T size, VOID *arg)
{
    void *frame;

    (void)arg;

    frame = bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, size);
    if (frame == NULL) {
        tkl_log_output("gpu frame malloc failed, size=%u\r\n", (unsigned int)size);
        return NULL;
    }

    if (test_gpu_psram_wt_enable(frame, size) != OPRT_OK) {
        bk_frame_buffer_free(frame);
        return NULL;
    }

    return frame;
}

static OPERATE_RET test_gpu_frame_free(VOID *ptr, VOID *arg)
{
    (void)arg;

    if (ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    test_gpu_psram_wt_disable();
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

static void test_gpu_frame_done(void *frame, UINT32_T frame_size, void *arg)
{
    OPERATE_RET ret;
    tkl_mipi_dsi_flush_frame_t flush_frame;

    if (frame == NULL) {
        return;
    }

    if (test_mipi_lcd_is_open() != TRUE) {
        test_gpu_display_frame_free((UINT8_T *)frame);
        return;
    }

    (void)arg;
    (void)frame_size;

    os_memset(&flush_frame, 0, sizeof(flush_frame));
    flush_frame.buffer = (UINT8_T *)frame;
    flush_frame.free_cb = test_gpu_display_frame_free;

    ret = tkl_mipi_dsi_flush(&flush_frame);
    if (ret != OPRT_OK) {
        tkl_log_output( "gpu display flush failed: %d\r\n", ret);
        test_gpu_display_frame_free((UINT8_T *)frame);
    }
}

static OPERATE_RET test_gpu_prepare_target(uint16_t src_width,
                                           uint16_t src_height,
                                           uint16_t target_width,
                                           uint16_t target_height,
                                           uint16_t rotate_degree,
                                           bool compress)
{
    if (src_width == 0U || src_height == 0U) {
        return OPRT_INVALID_PARM;
    }

    s_gpu_ctx.target_width = (target_width != 0U) ? target_width : src_width;
    s_gpu_ctx.target_height = (target_height != 0U) ? target_height : src_height;

    s_gpu_ctx.rotate_degree = rotate_degree;
    s_gpu_ctx.compress = compress;
    return test_gpu_dst_validate(rotate_degree, compress);
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

static OPERATE_RET test_gpu_init(test_gpu_source_t source_type,
                                 uint16_t rotate_degree,
                                 bool compress,
                                 uint16_t target_width,
                                 uint16_t target_height)
{
    OPERATE_RET ret;
    TUYA_CAMERA_MIPI_FLEXA_SOURCE_T camera_source;
    TUYA_VCODEC_FLEXA_BUFFER_T vdec_source;
    TUYA_GPU_CFG_T gpu_cfg;
    TUYA_GPU_SOURCE_T gpu_source;
    TUYA_GPU_SINK_T gpu_sink;
    uint16_t dst_width = 0U;
    uint16_t dst_height = 0U;
    uint16_t src_width = 0U;
    uint16_t src_height = 0U;
    uint32_t src_format = BK_PIXEL_FORMAT_INVALID;

    if (s_gpu_ctx.started == true) {
        tkl_log_output( "gpu pipeline is running, stop before reinit\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    os_memset(&camera_source, 0, sizeof(camera_source));
    os_memset(&vdec_source, 0, sizeof(vdec_source));
    os_memset(&gpu_source, 0, sizeof(gpu_source));

    if (source_type == GPU_TEST_SOURCE_VDEC) {
        ret = test_vdec_get_flexa_source(&vdec_source);
        if (ret != OPRT_OK) {
            tkl_log_output("vdec get flexa source failed: %d, use: xvdec init ... flexa && xvdec start\r\n", ret);
            return ret;
        }
        if (vdec_source.owner_handle == NULL) {
            return OPRT_RESOURCE_NOT_READY;
        }
        src_width = vdec_source.width;
        src_height = vdec_source.height;
        src_format = vdec_source.format;
        gpu_source.type = TUYA_GPU_SOURCE_MJPEG_FLEXA;
        gpu_source.mjpeg.jpeg_handle = vdec_source.owner_handle;
        gpu_source.mjpeg.flexa.src_buffer = vdec_source.src_buffer;
        gpu_source.mjpeg.flexa.buf_cnt = vdec_source.buf_cnt;
        gpu_source.mjpeg.flexa.width = vdec_source.width;
        gpu_source.mjpeg.flexa.height = vdec_source.height;
        gpu_source.mjpeg.flexa.format = vdec_source.format;
    } else {
        ret = test_gpu_get_camera_source(&camera_source);
        if (ret != OPRT_OK) {
            tkl_log_output( "camera get flexa source failed: %d\r\n", ret);
            return ret;
        }
        src_width = camera_source.width;
        src_height = camera_source.height;
        src_format = camera_source.format;
        gpu_source.type = TUYA_GPU_SOURCE_ISP_FLEXA;
        gpu_source.isp.isp_handle = camera_source.isp_handle;
        gpu_source.isp.flexa.src_buffer = camera_source.src_buffer;
        gpu_source.isp.flexa.buf_cnt = camera_source.buf_cnt;
        gpu_source.isp.flexa.width = camera_source.width;
        gpu_source.isp.flexa.height = camera_source.height;
        gpu_source.isp.flexa.format = camera_source.format;
    }

    ret = test_gpu_prepare_target(src_width, src_height, target_width, target_height, rotate_degree, compress);
    if (ret != OPRT_OK) {
        return ret;
    }

    test_gpu_dst_geometry_get(rotate_degree, &dst_width, &dst_height);

    if (s_gpu_ctx.initialized == true) {
        ret = tkl_gpu_deinit();
        if (ret != OPRT_OK) {
            tkl_log_output( "gpu deinit before reinit failed: %d\r\n", ret);
            return ret;
        }
        test_gpu_psram_wt_close();
    }

    ret = test_gpu_psram_wt_open();
    if (ret != OPRT_OK) {
        return ret;
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
        test_gpu_psram_wt_close();
        return ret;
    }

    os_memset(&gpu_sink, 0, sizeof(gpu_sink));
    gpu_sink.alloc_cb = test_gpu_frame_alloc;
    gpu_sink.free_cb = test_gpu_frame_free;
    gpu_sink.frame_done_cb = test_gpu_frame_done;
    gpu_sink.cb_arg = NULL;

    ret = tkl_gpu_set_sink(&gpu_sink);
    if (ret != OPRT_OK) {
        tkl_log_output( "gpu set sink failed: %d\r\n", ret);
        (void)tkl_gpu_deinit();
        test_gpu_psram_wt_close();
        return ret;
    }

    ret = tkl_gpu_set_source(&gpu_source);
    if (ret != OPRT_OK) {
        tkl_log_output( "gpu set source failed: %d\r\n", ret);
        (void)tkl_gpu_deinit();
        test_gpu_psram_wt_close();
        return ret;
    }

    s_gpu_ctx.initialized = true;
    s_gpu_ctx.started = false;
    s_gpu_ctx.source = source_type;
    s_gpu_ctx.src_width = src_width;
    s_gpu_ctx.src_height = src_height;
    s_gpu_ctx.src_format = src_format;
    tkl_log_output( "gpu pipeline initialized, source=%s src=%ux%u fmt=%u -> dst=%ux%u (target=%ux%u, lcd_open=%u) rotate=%u compress=%u\r\n",
            test_gpu_source_name(source_type),
            s_gpu_ctx.src_width,
            s_gpu_ctx.src_height,
            (unsigned int)s_gpu_ctx.src_format,
            dst_width,
            dst_height,
            s_gpu_ctx.target_width,
            s_gpu_ctx.target_height,
            (unsigned int)test_mipi_lcd_is_open(),
            rotate_degree,
            (unsigned int)compress);
    return OPRT_OK;
}

static void test_gpu_osd_format_time(char *text, size_t text_len)
{
    SYS_TIME_T now_ms;
    uint32_t total_sec;
    uint32_t hour;
    uint32_t min;
    uint32_t sec;

    if (text == NULL || text_len < 9U) {
        return;
    }

    now_ms = tkl_system_get_millisecond();
    total_sec = (uint32_t)(now_ms / 1000U);
    hour = (total_sec / 3600U) % 100U;
    min = (total_sec / 60U) % 60U;
    sec = total_sec % 60U;

    text[0] = (char)('0' + (hour / 10U));
    text[1] = (char)('0' + (hour % 10U));
    text[2] = ':';
    text[3] = (char)('0' + (min / 10U));
    text[4] = (char)('0' + (min % 10U));
    text[5] = ':';
    text[6] = (char)('0' + (sec / 10U));
    text[7] = (char)('0' + (sec % 10U));
    text[8] = '\0';
}

static OPERATE_RET test_gpu_osd_open(BOOL_T by_flexa)
{
    TUYA_GPU_OSD_OPEN_T cfg;

    if (s_gpu_ctx.initialized != true || s_gpu_ctx.started != true) {
        tkl_log_output("gpu is not started, use: xgpu init && xgpu start\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.blend_assets = blend_assets;
    cfg.blend_info = NULL; /* open does not draw; first paint via UPDATE */
    cfg.src_format = 0U;
    cfg.by_flexa = by_flexa;
    return tkl_gpu_osd_open(&cfg);
}

static void test_gpu_osd_time_task(void *arg)
{
    TKL_THREAD_HANDLE self = NULL;
    char text[GPU_TEST_OSD_TIME_TEXT_LEN];

    (void)arg;
    (void)tkl_thread_get_id(&self);

    while (s_gpu_osd_time.stop_req != true) {
        test_gpu_osd_format_time(text, sizeof(text));
        if (os_strcmp(text, s_gpu_osd_time.last_text) != 0) {
            if (tkl_gpu_osd_ctrl(TUYA_GPU_OSD_OP_UPDATE, GPU_TEST_OSD_TEXT_NAME, text) == OPRT_OK) {
                os_strncpy(s_gpu_osd_time.last_text, text, sizeof(s_gpu_osd_time.last_text) - 1U);
                s_gpu_osd_time.last_text[sizeof(s_gpu_osd_time.last_text) - 1U] = '\0';
            }
        }
        tkl_system_sleep(GPU_TEST_OSD_TIME_REFRESH_MS);
    }

    s_gpu_osd_time.running = false;
    s_gpu_osd_time.task = NULL;
    tkl_thread_release(self);
}

static void test_gpu_osd_time_stop(void)
{
    uint32_t waited_ms = 0U;

    if (s_gpu_osd_time.running != true && s_gpu_osd_time.task == NULL) {
        return;
    }

    s_gpu_osd_time.stop_req = true;
    while (s_gpu_osd_time.running == true && waited_ms < 3000U) {
        tkl_system_sleep(20U);
        waited_ms += 20U;
    }

    s_gpu_osd_time.running = false;
    s_gpu_osd_time.task = NULL;
    s_gpu_osd_time.stop_req = false;
    s_gpu_osd_time.last_text[0] = '\0';
}

static OPERATE_RET test_gpu_osd_time_start(void)
{
    OPERATE_RET ret;
    char text[GPU_TEST_OSD_TIME_TEXT_LEN];

    if (s_gpu_ctx.initialized != true || s_gpu_ctx.started != true) {
        tkl_log_output("gpu is not started, use: xgpu init && xgpu start\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    test_gpu_osd_time_stop();
    test_gpu_osd_format_time(text, sizeof(text));
    ret = tkl_gpu_osd_ctrl(TUYA_GPU_OSD_OP_UPDATE, GPU_TEST_OSD_TEXT_NAME, text);
    if (ret != OPRT_OK) {
        tkl_log_output("gpu osd time update failed (%d), try: xgpu osd open first\r\n", ret);
        return ret;
    }

    os_strncpy(s_gpu_osd_time.last_text, text, sizeof(s_gpu_osd_time.last_text) - 1U);
    s_gpu_osd_time.last_text[sizeof(s_gpu_osd_time.last_text) - 1U] = '\0';
    s_gpu_osd_time.stop_req = false;
    s_gpu_osd_time.running = true;

    ret = tkl_thread_create(&s_gpu_osd_time.task,
                            "gpu_osd_time",
                            GPU_TEST_OSD_TIME_STACK,
                            TKL_THREAD_PRI_NORMAL,
                            test_gpu_osd_time_task,
                            NULL);
    if (ret != OPRT_OK) {
        s_gpu_osd_time.running = false;
        s_gpu_osd_time.task = NULL;
        return ret;
    }

    tkl_log_output("gpu osd time start: %s\r\n", text);
    return OPRT_OK;
}

static OPERATE_RET test_gpu_start(void)
{
    OPERATE_RET ret;
    uint16_t dst_width = 0U;
    uint16_t dst_height = 0U;

    if (s_gpu_ctx.initialized != true) {
        tkl_log_output( "gpu is not initialized, use: xgpu init [camera|vdec] [rotate] [compress] [dst_w dst_h]\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_gpu_ctx.started == true) {
        tkl_log_output( "gpu pipeline already started\r\n");
        return OPRT_OK;
    }

    test_gpu_dst_geometry_get(s_gpu_ctx.rotate_degree, &dst_width, &dst_height);

    tkl_log_output(
            "gpu start request: source=%s src=%ux%u fmt=%u -> dst=%ux%u (target=%ux%u, lcd_open=%u) rotate=%u compress=%u\r\n",
            test_gpu_source_name(s_gpu_ctx.source),
            s_gpu_ctx.src_width,
            s_gpu_ctx.src_height,
            (unsigned int)s_gpu_ctx.src_format,
            dst_width,
            dst_height,
            s_gpu_ctx.target_width,
            s_gpu_ctx.target_height,
            (unsigned int)test_mipi_lcd_is_open(),
            s_gpu_ctx.rotate_degree,
            (unsigned int)s_gpu_ctx.compress);

    ret = tkl_gpu_start();
    if (ret != OPRT_OK) {
        tkl_log_output( "gpu start failed: %d\r\n", ret);
        return ret;
    }

    s_gpu_ctx.started = true;
    tkl_log_output( "gpu pipeline started, source=%s src=%ux%u fmt=%u -> dst=%ux%u (target=%ux%u, lcd_open=%u) rotate=%u compress=%u\r\n",
            test_gpu_source_name(s_gpu_ctx.source),
            s_gpu_ctx.src_width,
            s_gpu_ctx.src_height,
            (unsigned int)s_gpu_ctx.src_format,
            dst_width,
            dst_height,
            s_gpu_ctx.target_width,
            s_gpu_ctx.target_height,
            (unsigned int)test_mipi_lcd_is_open(),
            s_gpu_ctx.rotate_degree,
            (unsigned int)s_gpu_ctx.compress);
    return OPRT_OK;
}

static OPERATE_RET test_gpu_stop(void)
{
    OPERATE_RET ret;

    test_gpu_osd_time_stop();

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

    test_gpu_osd_time_stop();

    if (s_gpu_ctx.initialized != true) {
        test_gpu_runtime_reset();
        return OPRT_OK;
    }

    ret = tkl_gpu_deinit();
    if (ret != OPRT_OK) {
        return ret;
    }

    test_gpu_psram_wt_close();
    test_gpu_runtime_reset();
    return OPRT_OK;
}

void cli_tuya_gpu_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    uint16_t rotate_degree = GPU_TEST_DEFAULT_ROTATE;
    uint16_t target_width = 0U;
    uint16_t target_height = 0U;
    bool compress = (GPU_TEST_DEFAULT_COMPRESS != 0U);
    test_gpu_source_t source_type = GPU_TEST_SOURCE_CAMERA;
    int arg_base = 2;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xgpu <init [camera|vdec] [rotate] [compress] [dst_w dst_h]|start|stop|deinit|close|osd|status>");
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "init") == 0) {
        if (argc >= 3) {
            if (os_strcmp(argv[2], "vdec") == 0) {
                source_type = GPU_TEST_SOURCE_VDEC;
                arg_base = 3;
            } else if (os_strcmp(argv[2], "camera") == 0) {
                source_type = GPU_TEST_SOURCE_CAMERA;
                arg_base = 3;
            }
        }
        if (argc > arg_base) {
            rotate_degree = (uint16_t)os_strtoul(argv[arg_base], NULL, 10);
        }
        if (argc > (arg_base + 1)) {
            compress = (os_strtoul(argv[arg_base + 1], NULL, 10) != 0U);
        }
        if (argc > (arg_base + 3)) {
            target_width = (uint16_t)os_strtoul(argv[arg_base + 2], NULL, 10);
            target_height = (uint16_t)os_strtoul(argv[arg_base + 3], NULL, 10);
        }

        ret = test_gpu_init(source_type, rotate_degree, compress, target_width, target_height);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xgpu init %s (%d), source=%s rotate=%u, compress=%u target=%ux%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 test_gpu_source_name(source_type),
                 rotate_degree,
                 (unsigned int)compress,
                 s_gpu_ctx.target_width,
                 s_gpu_ctx.target_height);
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
                 "xgpu status gpu_init=%u gpu_start=%u source=%s lcd_open=%u camera_init=%u camera_start=%u camera_flexa=%u target=%ux%u rotate=%u compress=%u\r\n",
                 (unsigned int)s_gpu_ctx.initialized,
                 (unsigned int)s_gpu_ctx.started,
                 test_gpu_source_name(s_gpu_ctx.source),
                 (unsigned int)test_mipi_lcd_is_open(),
                 (unsigned int)test_mipi_camera_is_inited(),
                 (unsigned int)test_mipi_camera_is_started(),
                 (unsigned int)test_mipi_camera_flexa_enabled(),
                 s_gpu_ctx.target_width,
                 s_gpu_ctx.target_height,
                 s_gpu_ctx.rotate_degree,
                 (unsigned int)s_gpu_ctx.compress);
    } else if (os_strcmp(cmd, "osd") == 0) {
        const char *osd_sub = NULL;
        const char *name = NULL;
        const char *content = NULL;
        BOOL_T by_flexa = FALSE;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xgpu osd <open [flexa]|update <name> <content>|remove <name>|time|clear|close|frame|flexa>\r\n"
                     "  update examples: update clock 12:00:00 | update wifi wifi_rssi_none\r\n");
            return;
        }

        osd_sub = argv[2];
        if (os_strcmp(osd_sub, "open") == 0) {
            if (argc >= 4 && os_strcmp(argv[3], "flexa") == 0) {
                by_flexa = TRUE;
            }
            ret = test_gpu_osd_open(by_flexa);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xgpu osd open %s (%d), by_flexa=%u (no draw until update)\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     (unsigned int)by_flexa);
        } else if (os_strcmp(osd_sub, "update") == 0) {
            if (argc < 5) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "Usage: xgpu osd update <name> <content>\r\n"
                         "  name=clock|date|ver|wifi; wifi content=wifi_rssi_none|wifi_rssi_1|wifi_rssi_2|wifi_rssi_3|wifi_rssi_full\r\n");
                return;
            }
            name = argv[3];
            content = argv[4];
            ret = tkl_gpu_osd_ctrl(TUYA_GPU_OSD_OP_UPDATE, name, content);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xgpu osd update %s (%d), name=%s content=%s\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     name,
                     content);
        } else if (os_strcmp(osd_sub, "remove") == 0) {
            if (argc < 4) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "Usage: xgpu osd remove <name>\r\n");
                return;
            }
            name = argv[3];
            ret = tkl_gpu_osd_ctrl(TUYA_GPU_OSD_OP_REMOVE, name, NULL);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xgpu osd remove %s (%d), name=%s\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     name);
        } else if (os_strcmp(osd_sub, "time") == 0) {
            ret = test_gpu_osd_time_start();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xgpu osd time %s (%d), text=%s\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     s_gpu_osd_time.last_text);
        } else if (os_strcmp(osd_sub, "clear") == 0) {
            test_gpu_osd_time_stop();
            ret = tkl_gpu_osd_ctrl(TUYA_GPU_OSD_OP_CLEAR, NULL, NULL);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xgpu osd clear %s (%d)\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret);
        } else if (os_strcmp(osd_sub, "close") == 0) {
            test_gpu_osd_time_stop();
            ret = tkl_gpu_osd_close();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xgpu osd close %s (%d)\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret);
        } else if (os_strcmp(osd_sub, "frame") == 0) {
            ret = tkl_gpu_osd_set_mode(FALSE);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xgpu osd frame %s (%d)\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret);
        } else if (os_strcmp(osd_sub, "flexa") == 0) {
            ret = tkl_gpu_osd_set_mode(TRUE);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xgpu osd flexa %s (%d)\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret);
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xgpu osd <open [flexa]|update <name> <content>|remove <name>|time|clear|close|frame|flexa>\r\n"
                     "  update examples: update clock 12:00:00 | update wifi wifi_rssi_none\r\n");
        }
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xgpu <init [camera|vdec] [rotate] [compress] [dst_w dst_h]|start|stop|deinit|close|osd|status>");
    }
}
