#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <common/avdk_pixel_types.h>
#include <components/bk_frame_buffer.h>
#include <os/mem.h>
#include <os/str.h>

#include "tkl_output.h"
#include "tkl_vdec.h"

#define VDEC_TEST_DEFAULT_WIDTH        640U
#define VDEC_TEST_DEFAULT_HEIGHT       480U
#define VDEC_TEST_DEFAULT_FLEXA_LINES  16U
#define VDEC_TEST_DEFAULT_BUF_CNT      3U
#define VDEC_TEST_LOG_INTERVAL         900U

typedef enum {
    VDEC_TEST_MODE_FRAME = 0,
    VDEC_TEST_MODE_FLEXA,
} test_vdec_mode_t;

typedef struct {
    bool initialized;
    bool started;
    TUYA_VDEC_HANDLE handle;
    test_vdec_mode_t mode;
    uint16_t width;
    uint16_t height;
    uint16_t aligned_height;
    uint8_t flexa_lines;
    uint8_t buf_cnt;
    uint8_t *flexa_buffer;
    uint32_t flexa_size;
    uint32_t frame_count;
    uint32_t flexa_count;
    uint32_t submit_count;
    uint32_t submit_fail;
    uint32_t last_frame_size;
    uint32_t last_wr_cnt;
    uint64_t total_frame_bytes;
    uint64_t last_pts;
} test_vdec_ctx_t;

static test_vdec_ctx_t s_vdec_ctx = {
    .initialized = false,
    .started = false,
    .handle = NULL,
    .mode = VDEC_TEST_MODE_FLEXA,
    .width = VDEC_TEST_DEFAULT_WIDTH,
    .height = VDEC_TEST_DEFAULT_HEIGHT,
    .aligned_height = VDEC_TEST_DEFAULT_HEIGHT,
    .flexa_lines = VDEC_TEST_DEFAULT_FLEXA_LINES,
    .buf_cnt = VDEC_TEST_DEFAULT_BUF_CNT,
};

static uint16_t test_vdec_align16(uint16_t value)
{
    return (uint16_t)((value + 15U) & ~15U);
}

static uint8_t test_vdec_flexa_lines_normalize(uint8_t flexa_lines)
{
    if (flexa_lines < 16U) {
        flexa_lines = 16U;
    }
    if (flexa_lines > 240U) {
        flexa_lines = 240U;
    }

    return (uint8_t)test_vdec_align16(flexa_lines);
}

static uint32_t test_vdec_flexa_size(uint16_t width, uint8_t flexa_lines, uint8_t buf_cnt)
{
    return (uint32_t)width * (uint32_t)flexa_lines * 3U / 2U * (uint32_t)buf_cnt;
}

static const char *test_vdec_mode_name(test_vdec_mode_t mode)
{
    return (mode == VDEC_TEST_MODE_FRAME) ? "frame" : "flexa";
}

static test_vdec_mode_t test_vdec_mode_parse(const char *mode)
{
    if (mode != NULL && os_strcmp(mode, "frame") == 0) {
        return VDEC_TEST_MODE_FRAME;
    }

    return VDEC_TEST_MODE_FLEXA;
}

static void test_vdec_stats_reset(void)
{
    s_vdec_ctx.frame_count = 0U;
    s_vdec_ctx.flexa_count = 0U;
    s_vdec_ctx.submit_count = 0U;
    s_vdec_ctx.submit_fail = 0U;
    s_vdec_ctx.last_frame_size = 0U;
    s_vdec_ctx.last_wr_cnt = 0U;
    s_vdec_ctx.total_frame_bytes = 0U;
    s_vdec_ctx.last_pts = 0U;
}

static void test_vdec_runtime_reset(void)
{
    s_vdec_ctx.initialized = false;
    s_vdec_ctx.started = false;
    s_vdec_ctx.handle = NULL;
    s_vdec_ctx.mode = VDEC_TEST_MODE_FLEXA;
    s_vdec_ctx.width = VDEC_TEST_DEFAULT_WIDTH;
    s_vdec_ctx.height = VDEC_TEST_DEFAULT_HEIGHT;
    s_vdec_ctx.aligned_height = VDEC_TEST_DEFAULT_HEIGHT;
    s_vdec_ctx.flexa_lines = VDEC_TEST_DEFAULT_FLEXA_LINES;
    s_vdec_ctx.buf_cnt = VDEC_TEST_DEFAULT_BUF_CNT;
    s_vdec_ctx.flexa_buffer = NULL;
    s_vdec_ctx.flexa_size = 0U;
    test_vdec_stats_reset();
}

static VOID *test_vdec_frame_alloc(UINT32_T size, VOID *arg)
{
    (void)arg;
    return bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, size);
}

static OPERATE_RET test_vdec_frame_free(VOID *ptr, VOID *arg)
{
    (void)arg;

    if (ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    bk_frame_buffer_free(ptr);
    return OPRT_OK;
}

static void test_vdec_frame_cb(const TUYA_VCODEC_FRAME_T *frame, VOID *arg)
{
    (void)arg;

    if (frame == NULL) {
        return;
    }

    s_vdec_ctx.frame_count++;
    s_vdec_ctx.total_frame_bytes += frame->size;
    s_vdec_ctx.last_frame_size = frame->size;
    s_vdec_ctx.last_pts = frame->pts;

    if (s_vdec_ctx.frame_count <= 3U ||
        (s_vdec_ctx.frame_count % VDEC_TEST_LOG_INTERVAL) == 0U) {
        tkl_log_output(
                "vdec frame[%u]: %ux%u stride=%u slice=%u fmt=%u size=%u pts=%llu\r\n",
                (unsigned int)s_vdec_ctx.frame_count,
                frame->width,
                frame->height,
                frame->stride,
                frame->slice_height,
                (unsigned int)frame->format,
                (unsigned int)frame->size,
                (unsigned long long)frame->pts);
    }
}

static void test_vdec_flexa_cb(UINT32_T wr_cnt, VOID *arg)
{
    (void)arg;

    s_vdec_ctx.flexa_count++;
    s_vdec_ctx.last_wr_cnt = wr_cnt;

    if (s_vdec_ctx.flexa_count <= 3U ||
        (s_vdec_ctx.flexa_count % VDEC_TEST_LOG_INTERVAL) == 0U) {
        tkl_log_output("vdec flexa[%u]: wr_cnt=%u\r\n",
                (unsigned int)s_vdec_ctx.flexa_count,
                (unsigned int)wr_cnt);
    }
}

static OPERATE_RET test_vdec_alloc_flexa_buffer(uint16_t width, uint8_t flexa_lines, uint8_t buf_cnt)
{
    uint32_t size = test_vdec_flexa_size(width, flexa_lines, buf_cnt);

    if (size == 0U) {
        return OPRT_INVALID_PARM;
    }

    s_vdec_ctx.flexa_buffer = (uint8_t *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, size);
    if (s_vdec_ctx.flexa_buffer == NULL) {
        tkl_log_output("vdec flexa malloc failed, size=%u\r\n", (unsigned int)size);
        return OPRT_MALLOC_FAILED;
    }

    s_vdec_ctx.flexa_size = size;
    return OPRT_OK;
}

static void test_vdec_free_flexa_buffer(void)
{
    if (s_vdec_ctx.flexa_buffer != NULL) {
        bk_frame_buffer_free(s_vdec_ctx.flexa_buffer);
        s_vdec_ctx.flexa_buffer = NULL;
    }
    s_vdec_ctx.flexa_size = 0U;
}

static OPERATE_RET test_vdec_init(uint16_t width,
                                  uint16_t height,
                                  test_vdec_mode_t mode,
                                  uint8_t buf_cnt,
                                  uint8_t flexa_lines)
{
    OPERATE_RET ret;
    TUYA_VDEC_CFG_T cfg;
    TUYA_VDEC_SINK_T sink;

    if (width == 0U || height == 0U || buf_cnt == 0U || flexa_lines == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (s_vdec_ctx.started == true) {
        tkl_log_output("vdec is running, stop before reinit\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_vdec_ctx.initialized == true) {
        ret = tkl_vdec_deinit(s_vdec_ctx.handle);
        if (ret != OPRT_OK) {
            tkl_log_output("vdec deinit before reinit failed: %d\r\n", ret);
            return ret;
        }
        test_vdec_free_flexa_buffer();
        test_vdec_runtime_reset();
    }

    flexa_lines = test_vdec_flexa_lines_normalize(flexa_lines);

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.codec = TUYA_VCODEC_TYPE_MJPEG;
    cfg.width = width;
    cfg.height = height;
    cfg.output_format = BK_PIXEL_FORMAT_NV12;
    cfg.flexa_lines = flexa_lines;

    ret = tkl_vdec_init(&s_vdec_ctx.handle, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("vdec init failed: %d\r\n", ret);
        return ret;
    }

    os_memset(&sink, 0, sizeof(sink));
    sink.type = (mode == VDEC_TEST_MODE_FRAME) ? TUYA_VDEC_SINK_FRAME : TUYA_VDEC_SINK_FLEXA;
    if (mode == VDEC_TEST_MODE_FRAME) {
        sink.alloc_cb = test_vdec_frame_alloc;
        sink.free_cb = test_vdec_frame_free;
        sink.frame.frame_cb = test_vdec_frame_cb;
    } else {
        ret = test_vdec_alloc_flexa_buffer(width, flexa_lines, buf_cnt);
        if (ret != OPRT_OK) {
            (void)tkl_vdec_deinit(s_vdec_ctx.handle);
            test_vdec_runtime_reset();
            return ret;
        }
        sink.flexa.flexa.src_buffer = s_vdec_ctx.flexa_buffer;
        sink.flexa.flexa.buf_cnt = buf_cnt;
        sink.flexa.flexa_cb = test_vdec_flexa_cb;
    }

    ret = tkl_vdec_set_sink(s_vdec_ctx.handle, &sink);
    if (ret != OPRT_OK) {
        tkl_log_output("vdec set sink failed: %d\r\n", ret);
        (void)tkl_vdec_deinit(s_vdec_ctx.handle);
        test_vdec_free_flexa_buffer();
        test_vdec_runtime_reset();
        return ret;
    }

    s_vdec_ctx.initialized = true;
    s_vdec_ctx.started = false;
    s_vdec_ctx.mode = mode;
    s_vdec_ctx.width = width;
    s_vdec_ctx.height = height;
    s_vdec_ctx.aligned_height = test_vdec_align16(height);
    s_vdec_ctx.flexa_lines = flexa_lines;
    s_vdec_ctx.buf_cnt = buf_cnt;
    test_vdec_stats_reset();

    tkl_log_output("vdec init ok, mode=%s mjpeg=%ux%u -> NV12 flexa_lines=%u buf_cnt=%u flexa_size=%u\r\n",
            test_vdec_mode_name(mode),
            width,
            height,
            (unsigned int)flexa_lines,
            (unsigned int)buf_cnt,
            (unsigned int)s_vdec_ctx.flexa_size);
    return OPRT_OK;
}

static OPERATE_RET test_vdec_start(void)
{
    OPERATE_RET ret;

    if (s_vdec_ctx.initialized != true || s_vdec_ctx.handle == NULL) {
        tkl_log_output("vdec is not initialized, use: xvdec init [w] [h] [frame|flexa] [buf_cnt] [flexa_lines]\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_vdec_ctx.started == true) {
        return OPRT_OK;
    }

    ret = tkl_vdec_start(s_vdec_ctx.handle);
    if (ret != OPRT_OK) {
        tkl_log_output("vdec start failed: %d\r\n", ret);
        return ret;
    }

    s_vdec_ctx.started = true;
    test_vdec_stats_reset();
    tkl_log_output("vdec started, mode=%s %ux%u\r\n",
            test_vdec_mode_name(s_vdec_ctx.mode),
            s_vdec_ctx.width,
            s_vdec_ctx.height);
    return OPRT_OK;
}

static OPERATE_RET test_vdec_stop(void)
{
    OPERATE_RET ret;

    if (s_vdec_ctx.initialized != true || s_vdec_ctx.handle == NULL) {
        return OPRT_OK;
    }

    ret = tkl_vdec_stop(s_vdec_ctx.handle);
    if (ret != OPRT_OK) {
        tkl_log_output("vdec stop failed: %d\r\n", ret);
        return ret;
    }

    s_vdec_ctx.started = false;
    return OPRT_OK;
}

static OPERATE_RET test_vdec_deinit(void)
{
    OPERATE_RET ret;

    if (s_vdec_ctx.initialized != true || s_vdec_ctx.handle == NULL) {
        test_vdec_free_flexa_buffer();
        test_vdec_runtime_reset();
        return OPRT_OK;
    }

    ret = tkl_vdec_deinit(s_vdec_ctx.handle);
    if (ret != OPRT_OK) {
        tkl_log_output("vdec deinit failed: %d\r\n", ret);
        return ret;
    }

    test_vdec_free_flexa_buffer();
    test_vdec_runtime_reset();
    return OPRT_OK;
}

static void test_vdec_status_dump(void)
{
    VOID *backend = NULL;
    TUYA_VCODEC_FLEXA_BUFFER_T flexa;
    OPERATE_RET flexa_ret = OPRT_NOT_SUPPORTED;

    if (s_vdec_ctx.handle != NULL) {
        (void)tkl_vdec_get_handle(s_vdec_ctx.handle, &backend);
        os_memset(&flexa, 0, sizeof(flexa));
        flexa_ret = tkl_vdec_get_flexa_source(s_vdec_ctx.handle, &flexa);
    } else {
        os_memset(&flexa, 0, sizeof(flexa));
    }

    tkl_log_output("xvdec status:\r\n");
    tkl_log_output("  init=%u start=%u mode=%s backend=%p\r\n",
            (unsigned int)s_vdec_ctx.initialized,
            (unsigned int)s_vdec_ctx.started,
            test_vdec_mode_name(s_vdec_ctx.mode),
            backend);
    tkl_log_output("  cfg=%ux%u aligned_h=%u flexa_lines=%u buf_cnt=%u flexa_buf=%p flexa_size=%u\r\n",
            s_vdec_ctx.width,
            s_vdec_ctx.height,
            s_vdec_ctx.aligned_height,
            (unsigned int)s_vdec_ctx.flexa_lines,
            (unsigned int)s_vdec_ctx.buf_cnt,
            s_vdec_ctx.flexa_buffer,
            (unsigned int)s_vdec_ctx.flexa_size);
    tkl_log_output("  submits=%u submit_fail=%u frames=%u flexa=%u last_size=%u last_wr=%u bytes=%llu last_pts=%llu\r\n",
            (unsigned int)s_vdec_ctx.submit_count,
            (unsigned int)s_vdec_ctx.submit_fail,
            (unsigned int)s_vdec_ctx.frame_count,
            (unsigned int)s_vdec_ctx.flexa_count,
            (unsigned int)s_vdec_ctx.last_frame_size,
            (unsigned int)s_vdec_ctx.last_wr_cnt,
            (unsigned long long)s_vdec_ctx.total_frame_bytes,
            (unsigned long long)s_vdec_ctx.last_pts);
    if (flexa_ret == OPRT_OK) {
        tkl_log_output("  flexa_source: owner=%p buf=%p y=0x%08x u=0x%08x cnt=%u %ux%u fmt=%u\r\n",
                flexa.owner_handle,
                flexa.src_buffer,
                (unsigned int)flexa.y_addr,
                (unsigned int)flexa.u_addr,
                (unsigned int)flexa.buf_cnt,
                flexa.width,
                flexa.height,
                (unsigned int)flexa.format);
    }
}

BOOL_T test_vdec_is_started(void)
{
    return (s_vdec_ctx.started == true && s_vdec_ctx.handle != NULL) ? TRUE : FALSE;
}

OPERATE_RET test_vdec_get_flexa_source(TUYA_VCODEC_FLEXA_BUFFER_T *flexa)
{
    if (flexa == NULL) {
        return OPRT_INVALID_PARM;
    }

    os_memset(flexa, 0, sizeof(*flexa));

    if (s_vdec_ctx.started != true || s_vdec_ctx.handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_vdec_ctx.mode != VDEC_TEST_MODE_FLEXA) {
        return OPRT_NOT_SUPPORTED;
    }

    return tkl_vdec_get_flexa_source(s_vdec_ctx.handle, flexa);
}

OPERATE_RET test_vdec_submit_stream_from_uvc(const TUYA_VCODEC_STREAM_T *stream)
{
    OPERATE_RET ret;

    if (stream == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_vdec_ctx.started != true || s_vdec_ctx.handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    s_vdec_ctx.submit_count++;
    ret = tkl_vdec_submit_stream(s_vdec_ctx.handle, stream);
    if (ret != OPRT_OK) {
        s_vdec_ctx.submit_fail++;
        if (s_vdec_ctx.submit_fail <= 3U ||
            (s_vdec_ctx.submit_fail % VDEC_TEST_LOG_INTERVAL) == 0U) {
            tkl_log_output("vdec submit failed: ret=%d codec=%u size=%u submits=%u fail=%u\r\n",
                    ret,
                    (unsigned int)stream->codec,
                    (unsigned int)stream->size,
                    (unsigned int)s_vdec_ctx.submit_count,
                    (unsigned int)s_vdec_ctx.submit_fail);
        }
    }

    return ret;
}

void cli_tuya_vdec_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    uint16_t width = VDEC_TEST_DEFAULT_WIDTH;
    uint16_t height = VDEC_TEST_DEFAULT_HEIGHT;
    test_vdec_mode_t mode = VDEC_TEST_MODE_FLEXA;
    uint8_t buf_cnt = VDEC_TEST_DEFAULT_BUF_CNT;
    uint8_t flexa_lines = VDEC_TEST_DEFAULT_FLEXA_LINES;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xvdec <init [w] [h] [frame|flexa] [buf_cnt] [flexa_lines]|start|stop|deinit|close|status>");
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "init") == 0) {
        if (argc >= 3) {
            width = (uint16_t)os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            height = (uint16_t)os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            mode = test_vdec_mode_parse(argv[4]);
        }
        if (argc >= 6) {
            buf_cnt = (uint8_t)os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            flexa_lines = (uint8_t)os_strtoul(argv[6], NULL, 10);
        }

        ret = test_vdec_init(width, height, mode, buf_cnt, flexa_lines);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvdec init %s (%d), mode=%s %ux%u buf_cnt=%u flexa_lines=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 test_vdec_mode_name(mode),
                 width,
                 height,
                 (unsigned int)buf_cnt,
                 (unsigned int)flexa_lines);
    } else if (os_strcmp(cmd, "start") == 0) {
        ret = test_vdec_start();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvdec start %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "stop") == 0) {
        ret = test_vdec_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvdec stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "deinit") == 0 || os_strcmp(cmd, "close") == 0) {
        ret = test_vdec_deinit();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvdec %s %s (%d)\r\n",
                 cmd,
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "status") == 0) {
        test_vdec_status_dump();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvdec status ok, see logs\r\n");
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xvdec <init [w] [h] [frame|flexa] [buf_cnt] [flexa_lines]|start|stop|deinit|close|status>");
    }
}
