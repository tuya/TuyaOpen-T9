#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <components/bk_frame_buffer.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>

#include "tkl_output.h"
#include "tkl_uvc.h"

#define UVC_TEST_DEFAULT_WIDTH       640U
#define UVC_TEST_DEFAULT_HEIGHT      480U
#define UVC_TEST_DEFAULT_FPS         15U
#define UVC_TEST_DEFAULT_PORT        1U
#define UVC_TEST_LOG_INTERVAL        30U
#define UVC_TEST_VDEC_QUEUE_DEPTH    4U
#define UVC_TEST_VDEC_THREAD_STACK   (1024U * 4U)
#define UVC_TEST_VDEC_STOP_TIMEOUT   2000U
#define UVC_TEST_VDEC_INPUT_GUARD    1024U

extern BOOL_T test_vdec_is_started(void);
extern OPERATE_RET test_vdec_submit_stream_from_uvc(const TUYA_VCODEC_STREAM_T *stream);

typedef struct {
    UINT8_T *data;
    UINT32_T size;
    UINT64_T pts;
    UINT32_T codec;
    UINT32_T frame_type;
} test_uvc_vdec_msg_t;

typedef struct {
    bool initialized;
    bool started;
    bool connected;
    bool pipe_vdec;
    bool vdec_queue_ready;
    bool vdec_thread_running;
    bool vdec_thread_stopping;
    TUYA_UVC_HANDLE handle;
    beken_queue_t vdec_queue;
    beken_thread_t vdec_thread;
    beken_semaphore_t vdec_thread_sem;
    uint16_t width;
    uint16_t height;
    uint8_t fps;
    uint8_t port;
    uint8_t drop_num;
    uint32_t codec;
    uint32_t frame_count;
    uint32_t submit_vdec_count;
    uint32_t submit_vdec_fail;
    uint32_t submit_vdec_drop;
    uint32_t last_frame_size;
    uint32_t last_frame_type;
    uint64_t total_bytes;
    uint64_t last_pts;
} test_uvc_ctx_t;

static test_uvc_ctx_t s_uvc_ctx = {
    .initialized = false,
    .started = false,
    .connected = false,
    .pipe_vdec = false,
    .vdec_queue_ready = false,
    .vdec_thread_running = false,
    .vdec_thread_stopping = false,
    .handle = NULL,
    .vdec_thread = NULL,
    .vdec_thread_sem = NULL,
    .width = UVC_TEST_DEFAULT_WIDTH,
    .height = UVC_TEST_DEFAULT_HEIGHT,
    .fps = UVC_TEST_DEFAULT_FPS,
    .port = UVC_TEST_DEFAULT_PORT,
    .drop_num = 0U,
    .codec = TUYA_VCODEC_TYPE_MJPEG,
};

static const char *test_uvc_codec_name(uint32_t codec)
{
    switch (codec) {
    case TUYA_VCODEC_TYPE_H264:
        return "h264";
    case TUYA_VCODEC_TYPE_JPEG:
    case TUYA_VCODEC_TYPE_MJPEG:
        return "mjpeg";
    default:
        return "none";
    }
}

static uint32_t test_uvc_codec_parse(const char *codec)
{
    if (codec != NULL && os_strcmp(codec, "h264") == 0) {
        return TUYA_VCODEC_TYPE_H264;
    }

    return TUYA_VCODEC_TYPE_MJPEG;
}

static const char *test_uvc_frame_type_name(uint32_t frame_type)
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

static void test_uvc_stats_reset(void)
{
    s_uvc_ctx.frame_count = 0U;
    s_uvc_ctx.submit_vdec_count = 0U;
    s_uvc_ctx.submit_vdec_fail = 0U;
    s_uvc_ctx.submit_vdec_drop = 0U;
    s_uvc_ctx.last_frame_size = 0U;
    s_uvc_ctx.last_frame_type = TUYA_VCODEC_FRAME_UNKNOWN;
    s_uvc_ctx.total_bytes = 0U;
    s_uvc_ctx.last_pts = 0U;
}

static void test_uvc_runtime_reset(void)
{
    s_uvc_ctx.initialized = false;
    s_uvc_ctx.started = false;
    s_uvc_ctx.connected = false;
    s_uvc_ctx.pipe_vdec = false;
    s_uvc_ctx.vdec_queue_ready = false;
    s_uvc_ctx.vdec_thread_running = false;
    s_uvc_ctx.vdec_thread_stopping = false;
    s_uvc_ctx.handle = NULL;
    s_uvc_ctx.vdec_thread = NULL;
    s_uvc_ctx.vdec_thread_sem = NULL;
    s_uvc_ctx.width = UVC_TEST_DEFAULT_WIDTH;
    s_uvc_ctx.height = UVC_TEST_DEFAULT_HEIGHT;
    s_uvc_ctx.fps = UVC_TEST_DEFAULT_FPS;
    s_uvc_ctx.port = UVC_TEST_DEFAULT_PORT;
    s_uvc_ctx.drop_num = 0U;
    s_uvc_ctx.codec = TUYA_VCODEC_TYPE_MJPEG;
    test_uvc_stats_reset();
}

static VOID *test_uvc_stream_alloc(UINT32_T size, VOID *arg)
{
    (void)arg;
    return bk_frame_buffer_malloc(MEM_SLAB_HEAP_CODED, size);
}

static OPERATE_RET test_uvc_stream_free(VOID *ptr, VOID *arg)
{
    (void)arg;

    if (ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    bk_frame_buffer_free(ptr);
    return OPRT_OK;
}

static UINT32_T test_uvc_align64(UINT32_T value)
{
    return (value + 63U) & ~63U;
}

static void test_uvc_vdec_msg_free(test_uvc_vdec_msg_t *msg)
{
    if (msg != NULL && msg->data != NULL) {
        bk_frame_buffer_free(msg->data);
        msg->data = NULL;
    }
}

static void test_uvc_vdec_worker(beken_thread_arg_t arg)
{
    (void)arg;

    while (1) {
        OPERATE_RET ret;
        TUYA_VCODEC_STREAM_T stream;
        test_uvc_vdec_msg_t msg;

        os_memset(&msg, 0, sizeof(msg));
        ret = rtos_pop_from_queue(&s_uvc_ctx.vdec_queue, &msg, BEKEN_WAIT_FOREVER);
        if (ret != OPRT_OK) {
            continue;
        }

        if (msg.data == NULL) {
            break;
        }

        if (s_uvc_ctx.vdec_thread_stopping == true) {
            test_uvc_vdec_msg_free(&msg);
            continue;
        }

        os_memset(&stream, 0, sizeof(stream));
        stream.data = msg.data;
        stream.size = msg.size;
        stream.pts = msg.pts;
        stream.codec = msg.codec;
        stream.frame_type = msg.frame_type;

        s_uvc_ctx.submit_vdec_count++;
        ret = test_vdec_submit_stream_from_uvc(&stream);
        if (ret != OPRT_OK) {
            s_uvc_ctx.submit_vdec_fail++;
            if (s_uvc_ctx.submit_vdec_fail <= 3U ||
                (s_uvc_ctx.submit_vdec_fail % UVC_TEST_LOG_INTERVAL) == 0U) {
                tkl_log_output("uvc -> vdec submit failed: ret=%d codec=%u size=%u fail=%u\r\n",
                        ret,
                        (unsigned int)stream.codec,
                        (unsigned int)stream.size,
                        (unsigned int)s_uvc_ctx.submit_vdec_fail);
            }
        }

        test_uvc_vdec_msg_free(&msg);
    }

    s_uvc_ctx.vdec_thread_running = false;
    (void)rtos_set_semaphore(&s_uvc_ctx.vdec_thread_sem);
    rtos_delete_thread(NULL);
}

static void test_uvc_vdec_queue_drain(void)
{
    test_uvc_vdec_msg_t msg;

    if (s_uvc_ctx.vdec_queue_ready != true) {
        return;
    }

    while (rtos_pop_from_queue(&s_uvc_ctx.vdec_queue, &msg, BEKEN_NO_WAIT) == OPRT_OK) {
        test_uvc_vdec_msg_free(&msg);
    }
}

static OPERATE_RET test_uvc_vdec_async_start(void)
{
    OPERATE_RET ret;

    if (s_uvc_ctx.vdec_thread_running == true) {
        return OPRT_OK;
    }

    ret = rtos_init_queue(&s_uvc_ctx.vdec_queue,
                          "uvc_vdec_q",
                          sizeof(test_uvc_vdec_msg_t),
                          UVC_TEST_VDEC_QUEUE_DEPTH);
    if (ret != OPRT_OK) {
        tkl_log_output("uvc vdec queue init failed: %d\r\n", ret);
        return ret;
    }
    s_uvc_ctx.vdec_queue_ready = true;

    ret = rtos_init_semaphore_ex(&s_uvc_ctx.vdec_thread_sem, 1, 0);
    if (ret != OPRT_OK) {
        tkl_log_output("uvc vdec sem init failed: %d\r\n", ret);
        (void)rtos_deinit_queue(&s_uvc_ctx.vdec_queue);
        s_uvc_ctx.vdec_queue_ready = false;
        return ret;
    }

    s_uvc_ctx.vdec_thread_stopping = false;
    ret = rtos_create_thread(&s_uvc_ctx.vdec_thread,
                             BEKEN_DEFAULT_WORKER_PRIORITY,
                             "uvc_vdec",
                             (beken_thread_function_t)test_uvc_vdec_worker,
                             UVC_TEST_VDEC_THREAD_STACK,
                             NULL);
    if (ret != OPRT_OK) {
        tkl_log_output("uvc vdec thread create failed: %d\r\n", ret);
        (void)rtos_deinit_semaphore(&s_uvc_ctx.vdec_thread_sem);
        (void)rtos_deinit_queue(&s_uvc_ctx.vdec_queue);
        s_uvc_ctx.vdec_thread_sem = NULL;
        s_uvc_ctx.vdec_queue_ready = false;
        return ret;
    }

    s_uvc_ctx.vdec_thread_running = true;
    return OPRT_OK;
}

static void test_uvc_vdec_async_stop(void)
{
    test_uvc_vdec_msg_t stop_msg;

    if (s_uvc_ctx.vdec_queue_ready != true) {
        return;
    }

    s_uvc_ctx.vdec_thread_stopping = true;
    test_uvc_vdec_queue_drain();

    if (s_uvc_ctx.vdec_thread_running == true) {
        os_memset(&stop_msg, 0, sizeof(stop_msg));
        if (rtos_push_to_queue(&s_uvc_ctx.vdec_queue, &stop_msg, BEKEN_NO_WAIT) != OPRT_OK) {
            tkl_log_output("uvc vdec stop message push failed\r\n");
        }
        (void)rtos_get_semaphore(&s_uvc_ctx.vdec_thread_sem, UVC_TEST_VDEC_STOP_TIMEOUT);
    }

    test_uvc_vdec_queue_drain();

    if (s_uvc_ctx.vdec_thread_sem != NULL) {
        (void)rtos_deinit_semaphore(&s_uvc_ctx.vdec_thread_sem);
        s_uvc_ctx.vdec_thread_sem = NULL;
    }

    (void)rtos_deinit_queue(&s_uvc_ctx.vdec_queue);
    s_uvc_ctx.vdec_queue_ready = false;
    s_uvc_ctx.vdec_thread = NULL;
    s_uvc_ctx.vdec_thread_running = false;
    s_uvc_ctx.vdec_thread_stopping = false;
}

static OPERATE_RET test_uvc_submit_vdec_async(const TUYA_VCODEC_STREAM_T *stream)
{
    OPERATE_RET ret;
    test_uvc_vdec_msg_t msg;
    UINT8_T *payload;
    UINT32_T alloc_size;

    if (stream == NULL || stream->data == NULL || stream->size == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (s_uvc_ctx.vdec_queue_ready != true ||
        s_uvc_ctx.vdec_thread_running != true ||
        s_uvc_ctx.vdec_thread_stopping == true) {
        return OPRT_RESOURCE_NOT_READY;
    }

    alloc_size = test_uvc_align64(stream->size + UVC_TEST_VDEC_INPUT_GUARD);
    payload = (UINT8_T *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_CODED, alloc_size);
    if (payload == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    os_memcpy(payload, stream->data, stream->size);
    os_memset(&msg, 0, sizeof(msg));
    msg.data = payload;
    msg.size = stream->size;
    msg.pts = stream->pts;
    msg.codec = stream->codec;
    msg.frame_type = stream->frame_type;

    ret = rtos_push_to_queue(&s_uvc_ctx.vdec_queue, &msg, BEKEN_NO_WAIT);
    if (ret != OPRT_OK) {
        test_uvc_vdec_msg_free(&msg);
        return ret;
    }

    return OPRT_OK;
}

static void test_uvc_stream_cb(const TUYA_VCODEC_STREAM_T *stream, VOID *arg)
{
    (void)arg;

    if (stream == NULL) {
        return;
    }

    s_uvc_ctx.frame_count++;
    s_uvc_ctx.total_bytes += stream->size;
    s_uvc_ctx.last_frame_size = stream->size;
    s_uvc_ctx.last_frame_type = stream->frame_type;
    s_uvc_ctx.last_pts = stream->pts;

    if (s_uvc_ctx.pipe_vdec == true) {
        OPERATE_RET ret = test_uvc_submit_vdec_async(stream);

        if (ret != OPRT_OK) {
            s_uvc_ctx.submit_vdec_drop++;
            if (s_uvc_ctx.submit_vdec_drop <= 3U ||
                (s_uvc_ctx.submit_vdec_drop % UVC_TEST_LOG_INTERVAL) == 0U) {
                tkl_log_output("uvc -> vdec queue drop: ret=%d codec=%u size=%u drop=%u\r\n",
                        ret,
                        (unsigned int)stream->codec,
                        (unsigned int)stream->size,
                        (unsigned int)s_uvc_ctx.submit_vdec_drop);
            }
        }
    }

    if (s_uvc_ctx.frame_count <= 3U ||
        (s_uvc_ctx.frame_count % UVC_TEST_LOG_INTERVAL) == 0U) {
        tkl_log_output(
                "uvc frame[%u]: codec=%s type=%s size=%u pts=%llu total_bytes=%llu pipe_vdec=%u\r\n",
                (unsigned int)s_uvc_ctx.frame_count,
                test_uvc_codec_name(stream->codec),
                test_uvc_frame_type_name(stream->frame_type),
                (unsigned int)stream->size,
                (unsigned long long)stream->pts,
                (unsigned long long)s_uvc_ctx.total_bytes,
                (unsigned int)s_uvc_ctx.pipe_vdec);
    }
}

static void test_uvc_state_cb(BOOL_T connected, VOID *arg)
{
    (void)arg;

    s_uvc_ctx.connected = (connected != FALSE);
    tkl_log_output("uvc state: %s\r\n", s_uvc_ctx.connected ? "connected" : "disconnected");
}

static OPERATE_RET test_uvc_init(uint16_t width,
                                 uint16_t height,
                                 uint8_t fps,
                                 uint32_t codec,
                                 uint8_t port,
                                 uint8_t drop_num)
{
    OPERATE_RET ret;
    TUYA_UVC_CFG_T cfg;
    TUYA_UVC_SINK_T sink;

    if (width == 0U || height == 0U || fps == 0U || port == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (s_uvc_ctx.started == true) {
        tkl_log_output("uvc is running, stop before reinit\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_uvc_ctx.initialized == true) {
        s_uvc_ctx.pipe_vdec = false;
        test_uvc_vdec_async_stop();
        ret = tkl_uvc_deinit(s_uvc_ctx.handle);
        if (ret != OPRT_OK) {
            tkl_log_output("uvc deinit before reinit failed: %d\r\n", ret);
            return ret;
        }
        test_uvc_runtime_reset();
    }

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.port = port;
    cfg.width = width;
    cfg.height = height;
    cfg.fps = fps;
    cfg.drop_num = drop_num;
    cfg.codec = codec;

    ret = tkl_uvc_init(&s_uvc_ctx.handle, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("uvc init failed: %d\r\n", ret);
        return ret;
    }

    os_memset(&sink, 0, sizeof(sink));
    sink.stream_cb = test_uvc_stream_cb;
    sink.state_cb = test_uvc_state_cb;
    sink.alloc_cb = test_uvc_stream_alloc;
    sink.free_cb = test_uvc_stream_free;

    ret = tkl_uvc_set_sink(s_uvc_ctx.handle, &sink);
    if (ret != OPRT_OK) {
        tkl_log_output("uvc set sink failed: %d\r\n", ret);
        (void)tkl_uvc_deinit(s_uvc_ctx.handle);
        test_uvc_runtime_reset();
        return ret;
    }

    s_uvc_ctx.initialized = true;
    s_uvc_ctx.started = false;
    s_uvc_ctx.connected = false;
    s_uvc_ctx.width = width;
    s_uvc_ctx.height = height;
    s_uvc_ctx.fps = fps;
    s_uvc_ctx.port = port;
    s_uvc_ctx.drop_num = drop_num;
    s_uvc_ctx.codec = codec;
    test_uvc_stats_reset();

    tkl_log_output("uvc init ok, port=%u codec=%s %ux%u fps=%u drop=%u\r\n",
            (unsigned int)port,
            test_uvc_codec_name(codec),
            width,
            height,
            (unsigned int)fps,
            (unsigned int)drop_num);
    return OPRT_OK;
}

static OPERATE_RET test_uvc_start(void)
{
    OPERATE_RET ret;

    if (s_uvc_ctx.initialized != true || s_uvc_ctx.handle == NULL) {
        tkl_log_output("uvc is not initialized, use: xuvc init [w] [h] [fps] [mjpeg|h264] [port] [drop]\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_uvc_ctx.started == true) {
        return OPRT_OK;
    }

    test_uvc_stats_reset();
    ret = tkl_uvc_start(s_uvc_ctx.handle);
    if (ret != OPRT_OK) {
        tkl_log_output("uvc start failed: %d\r\n", ret);
        return ret;
    }

    s_uvc_ctx.started = true;
    tkl_log_output("uvc started, port=%u codec=%s %ux%u fps=%u pipe_vdec=%u\r\n",
            (unsigned int)s_uvc_ctx.port,
            test_uvc_codec_name(s_uvc_ctx.codec),
            s_uvc_ctx.width,
            s_uvc_ctx.height,
            (unsigned int)s_uvc_ctx.fps,
            (unsigned int)s_uvc_ctx.pipe_vdec);
    return OPRT_OK;
}

static OPERATE_RET test_uvc_stop(void)
{
    OPERATE_RET ret;

    if (s_uvc_ctx.initialized != true || s_uvc_ctx.handle == NULL) {
        return OPRT_OK;
    }

    s_uvc_ctx.pipe_vdec = false;
    ret = tkl_uvc_stop(s_uvc_ctx.handle);
    if (ret != OPRT_OK) {
        tkl_log_output("uvc stop failed: %d\r\n", ret);
        return ret;
    }
    test_uvc_vdec_async_stop();

    s_uvc_ctx.started = false;
    s_uvc_ctx.connected = false;
    return OPRT_OK;
}

static OPERATE_RET test_uvc_deinit(void)
{
    OPERATE_RET ret;

    if (s_uvc_ctx.initialized != true || s_uvc_ctx.handle == NULL) {
        test_uvc_vdec_async_stop();
        test_uvc_runtime_reset();
        return OPRT_OK;
    }

    s_uvc_ctx.pipe_vdec = false;
    test_uvc_vdec_async_stop();
    ret = tkl_uvc_deinit(s_uvc_ctx.handle);
    if (ret != OPRT_OK) {
        tkl_log_output("uvc deinit failed: %d\r\n", ret);
        return ret;
    }

    test_uvc_runtime_reset();
    return OPRT_OK;
}

static OPERATE_RET test_uvc_pipe_vdec(bool enable)
{
    OPERATE_RET ret;

    if (enable == true) {
        if (s_uvc_ctx.codec != TUYA_VCODEC_TYPE_MJPEG &&
            s_uvc_ctx.codec != TUYA_VCODEC_TYPE_JPEG) {
            tkl_log_output("uvc pipe vdec requires mjpeg codec\r\n");
            return OPRT_NOT_SUPPORTED;
        }

        if (test_vdec_is_started() != TRUE) {
            tkl_log_output("vdec is not started yet, use: xvdec init ... && xvdec start\r\n");
            return OPRT_RESOURCE_NOT_READY;
        }

        ret = test_uvc_vdec_async_start();
        if (ret != OPRT_OK) {
            return ret;
        }
    } else {
        s_uvc_ctx.pipe_vdec = false;
        test_uvc_vdec_async_stop();
        tkl_log_output("uvc pipe vdec off\r\n");
        return OPRT_OK;
    }

    s_uvc_ctx.pipe_vdec = enable;
    tkl_log_output("uvc pipe vdec on, async queue=%u\r\n", (unsigned int)UVC_TEST_VDEC_QUEUE_DEPTH);
    return OPRT_OK;
}

static void test_uvc_status_dump(void)
{
    VOID *backend = NULL;

    if (s_uvc_ctx.handle != NULL) {
        (void)tkl_uvc_get_handle(s_uvc_ctx.handle, &backend);
    }

    tkl_log_output("xuvc status:\r\n");
    tkl_log_output("  init=%u start=%u connected=%u pipe_vdec=%u vdec_q=%u vdec_thread=%u backend=%p\r\n",
            (unsigned int)s_uvc_ctx.initialized,
            (unsigned int)s_uvc_ctx.started,
            (unsigned int)s_uvc_ctx.connected,
            (unsigned int)s_uvc_ctx.pipe_vdec,
            (unsigned int)s_uvc_ctx.vdec_queue_ready,
            (unsigned int)s_uvc_ctx.vdec_thread_running,
            backend);
    tkl_log_output("  cfg: port=%u codec=%s %ux%u fps=%u drop=%u\r\n",
            (unsigned int)s_uvc_ctx.port,
            test_uvc_codec_name(s_uvc_ctx.codec),
            s_uvc_ctx.width,
            s_uvc_ctx.height,
            (unsigned int)s_uvc_ctx.fps,
            (unsigned int)s_uvc_ctx.drop_num);
    tkl_log_output("  frames=%u last_type=%s last_size=%u total_bytes=%llu last_pts=%llu vdec_submit=%u vdec_fail=%u vdec_drop=%u\r\n",
            (unsigned int)s_uvc_ctx.frame_count,
            test_uvc_frame_type_name(s_uvc_ctx.last_frame_type),
            (unsigned int)s_uvc_ctx.last_frame_size,
            (unsigned long long)s_uvc_ctx.total_bytes,
            (unsigned long long)s_uvc_ctx.last_pts,
            (unsigned int)s_uvc_ctx.submit_vdec_count,
            (unsigned int)s_uvc_ctx.submit_vdec_fail,
            (unsigned int)s_uvc_ctx.submit_vdec_drop);
}

void cli_tuya_uvc_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    uint16_t width = UVC_TEST_DEFAULT_WIDTH;
    uint16_t height = UVC_TEST_DEFAULT_HEIGHT;
    uint8_t fps = UVC_TEST_DEFAULT_FPS;
    uint32_t codec = TUYA_VCODEC_TYPE_MJPEG;
    uint8_t port = UVC_TEST_DEFAULT_PORT;
    uint8_t drop_num = 0U;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xuvc <init [w] [h] [fps] [mjpeg|h264] [port] [drop]|start|stop|deinit|close|pipe vdec on|pipe vdec off|status>");
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
            fps = (uint8_t)os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            codec = test_uvc_codec_parse(argv[5]);
        }
        if (argc >= 7) {
            port = (uint8_t)os_strtoul(argv[6], NULL, 10);
        }
        if (argc >= 8) {
            drop_num = (uint8_t)os_strtoul(argv[7], NULL, 10);
        }

        ret = test_uvc_init(width, height, fps, codec, port, drop_num);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc init %s (%d), port=%u codec=%s %ux%u fps=%u drop=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port,
                 test_uvc_codec_name(codec),
                 width,
                 height,
                 (unsigned int)fps,
                 (unsigned int)drop_num);
    } else if (os_strcmp(cmd, "start") == 0) {
        ret = test_uvc_start();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc start %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "stop") == 0) {
        ret = test_uvc_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "deinit") == 0 || os_strcmp(cmd, "close") == 0) {
        ret = test_uvc_deinit();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc %s %s (%d)\r\n",
                 cmd,
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "pipe") == 0) {
        if (argc >= 4 && os_strcmp(argv[2], "vdec") == 0) {
            if (os_strcmp(argv[3], "on") == 0 || os_strcmp(argv[3], "off") == 0) {
                ret = test_uvc_pipe_vdec(os_strcmp(argv[3], "on") == 0);
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xuvc pipe vdec %s (%d)\r\n",
                         (ret == OPRT_OK) ? "ok" : "failed",
                         ret);
            } else {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "Usage: xuvc pipe vdec <on|off>");
            }
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xuvc pipe vdec <on|off>");
        }
    } else if (os_strcmp(cmd, "status") == 0) {
        test_uvc_status_dump();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc status ok, see logs\r\n");
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xuvc <init [w] [h] [fps] [mjpeg|h264] [port] [drop]|start|stop|deinit|close|pipe vdec on|pipe vdec off|status>");
    }
}
