#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#include <components/bk_frame_buffer.h>
#include <common/avdk_pixel_types.h>
#include <os/mem.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_output.h"
#include "tkl_venc.h"
#include "tkl_camera_mipi.h"

#define VENC_TEST_DEFAULT_FPS      15U
#define VENC_TEST_DEFAULT_GOP      30U
#define VENC_TEST_DEFAULT_BITRATE  (2U * 1024U * 1024U)
#define VENC_TEST_LOG_INTERVAL     30U
#define VENC_TEST_LOG_BUF_SIZE     192U

typedef struct {
    bool initialized;
    bool started;
    TUYA_VENC_HANDLE handle;
    uint8_t fps;
    uint32_t gop;
    uint32_t bitrate;
    uint16_t width;
    uint16_t height;
    uint32_t format;
    uint32_t total_frames;
    uint32_t i_frames;
    uint32_t p_frames;
    uint32_t header_frames;
    uint32_t unknown_frames;
    uint32_t last_frame_size;
    uint32_t last_frame_type;
    uint64_t total_bytes;
} test_venc_ctx_t;

static test_venc_ctx_t s_venc_ctx = {
    .initialized = false,
    .started = false,
    .handle = NULL,
    .fps = VENC_TEST_DEFAULT_FPS,
    .gop = VENC_TEST_DEFAULT_GOP,
    .bitrate = VENC_TEST_DEFAULT_BITRATE,
    .width = 0U,
    .height = 0U,
    .format = BK_PIXEL_FORMAT_INVALID,
    .total_frames = 0U,
    .i_frames = 0U,
    .p_frames = 0U,
    .header_frames = 0U,
    .unknown_frames = 0U,
    .last_frame_size = 0U,
    .last_frame_type = TUYA_VCODEC_FRAME_UNKNOWN,
    .total_bytes = 0U,
};

static void test_venc_stream_stats_reset(void)
{
    s_venc_ctx.total_frames = 0U;
    s_venc_ctx.i_frames = 0U;
    s_venc_ctx.p_frames = 0U;
    s_venc_ctx.header_frames = 0U;
    s_venc_ctx.unknown_frames = 0U;
    s_venc_ctx.last_frame_size = 0U;
    s_venc_ctx.last_frame_type = TUYA_VCODEC_FRAME_UNKNOWN;
    s_venc_ctx.total_bytes = 0U;
}

static void test_venc_stats_reset(void)
{
    s_venc_ctx.width = 0U;
    s_venc_ctx.height = 0U;
    s_venc_ctx.format = BK_PIXEL_FORMAT_INVALID;
    test_venc_stream_stats_reset();
}

static void test_venc_runtime_reset(void)
{
    s_venc_ctx.initialized = false;
    s_venc_ctx.started = false;
    s_venc_ctx.handle = NULL;
    s_venc_ctx.fps = VENC_TEST_DEFAULT_FPS;
    s_venc_ctx.gop = VENC_TEST_DEFAULT_GOP;
    s_venc_ctx.bitrate = VENC_TEST_DEFAULT_BITRATE;
    test_venc_stats_reset();
}

static const char *test_venc_frame_type_name(uint32_t frame_type)
{
    switch (frame_type) {
    case TUYA_VCODEC_FRAME_I:
        return "I";
    case TUYA_VCODEC_FRAME_P:
        return "P";
    case TUYA_VCODEC_FRAME_B:
        return "B";
    case TUYA_VCODEC_FRAME_HEADER:
        return "HDR";
    case TUYA_VCODEC_FRAME_END:
        return "END";
    default:
        return "UNK";
    }
}

static void test_venc_stream_cb(const TUYA_VCODEC_STREAM_T *stream, VOID *arg)
{
    (void)arg;

    if (stream == NULL) {
        return;
    }

    s_venc_ctx.total_frames++;
    s_venc_ctx.total_bytes += stream->size;
    s_venc_ctx.last_frame_size = stream->size;
    s_venc_ctx.last_frame_type = stream->frame_type;

    switch (stream->frame_type) {
    case TUYA_VCODEC_FRAME_I:
        s_venc_ctx.i_frames++;
        break;
    case TUYA_VCODEC_FRAME_P:
        s_venc_ctx.p_frames++;
        break;
    case TUYA_VCODEC_FRAME_HEADER:
        s_venc_ctx.header_frames++;
        break;
    default:
        s_venc_ctx.unknown_frames++;
        break;
    }

    if (s_venc_ctx.total_frames <= 3U ||
        (s_venc_ctx.total_frames % VENC_TEST_LOG_INTERVAL) == 0U) {
        tkl_log_output(
                "venc frame[%u]: type=%s size=%u pts=%llu total_bytes=%llu\r\n",
                (unsigned int)s_venc_ctx.total_frames,
                test_venc_frame_type_name(stream->frame_type),
                (unsigned int)stream->size,
                (unsigned long long)stream->pts,
                (unsigned long long)s_venc_ctx.total_bytes);
    }
}

static VOID *test_venc_stream_alloc(UINT32_T size, VOID *arg)
{
    (void)arg;
    return bk_frame_buffer_malloc(MEM_SLAB_HEAP_CODED, size);
}

static OPERATE_RET test_venc_stream_free(VOID *ptr, VOID *arg)
{
    (void)arg;

    if (ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    bk_frame_buffer_free(ptr);
    return OPRT_OK;
}

static OPERATE_RET test_venc_get_camera_source(TUYA_CAMERA_MIPI_FLEXA_SOURCE_T *camera_source)
{
    if (camera_source == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (test_mipi_camera_is_inited() != TRUE || test_mipi_camera_is_started() != TRUE) {
        tkl_log_output("mipi camera is not started, use: xmipi camera open_gpu ... && xmipi camera start\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (test_mipi_camera_flexa_enabled() != TRUE) {
        tkl_log_output("mipi camera flexa is not enabled, use: xmipi camera open_gpu ...\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    os_memset(camera_source, 0, sizeof(*camera_source));
    return test_mipi_camera_get_flexa_source(camera_source);
}

static OPERATE_RET test_venc_set_camera_source(void)
{
    OPERATE_RET ret;
    TUYA_CAMERA_MIPI_FLEXA_SOURCE_T camera_source;
    TUYA_VENC_SOURCE_T source;

    ret = test_venc_get_camera_source(&camera_source);
    if (ret != OPRT_OK) {
        tkl_log_output("camera get flexa source failed: %d\r\n", ret);
        return ret;
    }

    os_memset(&source, 0, sizeof(source));
    source.type = TUYA_VENC_SOURCE_ISP_FLEXA;
    source.isp.isp_handle = camera_source.isp_handle;
    source.isp.flexa.src_buffer = camera_source.src_buffer;
    source.isp.flexa.buf_cnt = camera_source.buf_cnt;
    source.isp.flexa.width = camera_source.width;
    source.isp.flexa.height = camera_source.height;
    source.isp.flexa.format = camera_source.format;
    source.isp.flexa.y_addr = camera_source.y_addr;
    source.isp.flexa.u_addr = camera_source.u_addr;
    source.isp.flexa.v_addr = camera_source.v_addr;

    ret = tkl_venc_set_source(s_venc_ctx.handle, &source);
    if (ret != OPRT_OK) {
        tkl_log_output("venc set source failed: %d\r\n", ret);
        return ret;
    }

    s_venc_ctx.width = camera_source.width;
    s_venc_ctx.height = camera_source.height;
    s_venc_ctx.format = camera_source.format;
    test_venc_stream_stats_reset();

    tkl_log_output("venc source ready, src=%ux%u fmt=%u\r\n",
            camera_source.width,
            camera_source.height,
            (unsigned int)camera_source.format);
    return OPRT_OK;
}

static OPERATE_RET test_venc_init(uint8_t fps, uint32_t gop, uint32_t bitrate)
{
    OPERATE_RET ret;
    TUYA_VENC_CFG_T cfg;
    TUYA_VENC_SINK_T sink;

    if (s_venc_ctx.started == true) {
        tkl_log_output("venc is running, stop before reinit\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_venc_ctx.initialized == true) {
        ret = tkl_venc_deinit(s_venc_ctx.handle);
        if (ret != OPRT_OK) {
            tkl_log_output("venc deinit before reinit failed: %d\r\n", ret);
            return ret;
        }
        test_venc_runtime_reset();
    }

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.codec = TUYA_VCODEC_TYPE_H264;
    cfg.fps = fps;
    cfg.gop = gop;
    cfg.bitrate = bitrate;
    cfg.input_format = BK_PIXEL_FORMAT_NV12;

    ret = tkl_venc_init(&s_venc_ctx.handle, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("venc init failed: %d\r\n", ret);
        return ret;
    }

    os_memset(&sink, 0, sizeof(sink));
    sink.stream_cb = test_venc_stream_cb;
    sink.alloc_cb = test_venc_stream_alloc;
    sink.free_cb = test_venc_stream_free;
    ret = tkl_venc_set_sink(s_venc_ctx.handle, &sink);
    if (ret != OPRT_OK) {
        tkl_log_output("venc set sink failed: %d\r\n", ret);
        (void)tkl_venc_deinit(s_venc_ctx.handle);
        test_venc_runtime_reset();
        return ret;
    }

    ret = test_venc_set_camera_source();
    if (ret != OPRT_OK) {
        (void)tkl_venc_deinit(s_venc_ctx.handle);
        test_venc_runtime_reset();
        return ret;
    }

    s_venc_ctx.initialized = true;
    s_venc_ctx.started = false;
    s_venc_ctx.fps = fps;
    s_venc_ctx.gop = gop;
    s_venc_ctx.bitrate = bitrate;
    test_venc_stream_stats_reset();

    tkl_log_output("venc init ok, codec=H264 src=%ux%u fmt=%u fps=%u gop=%u bitrate=%u\r\n",
            s_venc_ctx.width,
            s_venc_ctx.height,
            (unsigned int)s_venc_ctx.format,
            (unsigned int)fps,
            (unsigned int)gop,
            (unsigned int)bitrate);
    return OPRT_OK;
}

static OPERATE_RET test_venc_start(void)
{
    OPERATE_RET ret;

    if (s_venc_ctx.initialized != true || s_venc_ctx.handle == NULL) {
        tkl_log_output("venc is not initialized, use: xvenc init [fps] [gop] [bitrate]\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_venc_ctx.started == true) {
        tkl_log_output("venc already started\r\n");
        return OPRT_OK;
    }
    test_venc_stream_stats_reset();

    tkl_log_output("venc start request: src=%ux%u fmt=%u fps=%u gop=%u bitrate=%u\r\n",
            s_venc_ctx.width,
            s_venc_ctx.height,
            (unsigned int)s_venc_ctx.format,
            (unsigned int)s_venc_ctx.fps,
            (unsigned int)s_venc_ctx.gop,
            (unsigned int)s_venc_ctx.bitrate);

    ret = tkl_venc_start(s_venc_ctx.handle);
    if (ret != OPRT_OK) {
        tkl_log_output("venc start failed: %d\r\n", ret);
        return ret;
    }

    s_venc_ctx.started = true;
    tkl_log_output("venc started, src=%ux%u fmt=%u -> H264\r\n",
            s_venc_ctx.width,
            s_venc_ctx.height,
            (unsigned int)s_venc_ctx.format);
    return OPRT_OK;
}

static OPERATE_RET test_venc_stop(void)
{
    OPERATE_RET ret;

    if (s_venc_ctx.initialized != true || s_venc_ctx.handle == NULL) {
        return OPRT_OK;
    }

    ret = tkl_venc_stop(s_venc_ctx.handle);
    if (ret != OPRT_OK) {
        return ret;
    }

    s_venc_ctx.started = false;
    return OPRT_OK;
}

static OPERATE_RET test_venc_deinit(void)
{
    OPERATE_RET ret;

    if (s_venc_ctx.initialized != true || s_venc_ctx.handle == NULL) {
        test_venc_runtime_reset();
        return OPRT_OK;
    }

    ret = tkl_venc_deinit(s_venc_ctx.handle);
    if (ret != OPRT_OK) {
        return ret;
    }

    test_venc_runtime_reset();
    return OPRT_OK;
}

static OPERATE_RET test_venc_request_idr(void)
{
    if (s_venc_ctx.initialized != true || s_venc_ctx.handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    return tkl_venc_request_idr(s_venc_ctx.handle);
}

static void test_venc_status_dump(void)
{
    tkl_log_output("xvenc status:\r\n");
    tkl_log_output("  venc_init=%u venc_start=%u camera_init=%u camera_start=%u camera_flexa=%u\r\n",
            (unsigned int)s_venc_ctx.initialized,
            (unsigned int)s_venc_ctx.started,
            (unsigned int)test_mipi_camera_is_inited(),
            (unsigned int)test_mipi_camera_is_started(),
            (unsigned int)test_mipi_camera_flexa_enabled());
    tkl_log_output("  src=%ux%u fmt=%u fps=%u gop=%u bitrate=%u\r\n",
            s_venc_ctx.width,
            s_venc_ctx.height,
            (unsigned int)s_venc_ctx.format,
            (unsigned int)s_venc_ctx.fps,
            (unsigned int)s_venc_ctx.gop,
            (unsigned int)s_venc_ctx.bitrate);
    tkl_log_output("  frames=%u i=%u p=%u hdr=%u unk=%u last_type=%s last_size=%u total_bytes=%llu\r\n",
            (unsigned int)s_venc_ctx.total_frames,
            (unsigned int)s_venc_ctx.i_frames,
            (unsigned int)s_venc_ctx.p_frames,
            (unsigned int)s_venc_ctx.header_frames,
            (unsigned int)s_venc_ctx.unknown_frames,
            test_venc_frame_type_name(s_venc_ctx.last_frame_type),
            (unsigned int)s_venc_ctx.last_frame_size,
            (unsigned long long)s_venc_ctx.total_bytes);
}

void cli_tuya_venc_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    uint8_t fps = VENC_TEST_DEFAULT_FPS;
    uint32_t gop = VENC_TEST_DEFAULT_GOP;
    uint32_t bitrate = VENC_TEST_DEFAULT_BITRATE;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xvenc <init [fps] [gop] [bitrate]|start|stop|deinit|close|idr|status>");
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "init") == 0) {
        if (argc >= 3) {
            fps = (uint8_t)os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            gop = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            bitrate = os_strtoul(argv[4], NULL, 10);
        }

        ret = test_venc_init(fps, gop, bitrate);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc init %s (%d), fps=%u gop=%u bitrate=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)fps,
                 (unsigned int)gop,
                 (unsigned int)bitrate);
    } else if (os_strcmp(cmd, "start") == 0) {
        ret = test_venc_start();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc start %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "stop") == 0) {
        ret = test_venc_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "deinit") == 0 || os_strcmp(cmd, "close") == 0) {
        ret = test_venc_deinit();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc %s %s (%d)\r\n",
                 cmd,
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "idr") == 0) {
        ret = test_venc_request_idr();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc idr %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "status") == 0) {
        test_venc_status_dump();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc status ok, see logs\r\n");
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xvenc <init [fps] [gop] [bitrate]|start|stop|deinit|close|idr|status>");
    }
}
