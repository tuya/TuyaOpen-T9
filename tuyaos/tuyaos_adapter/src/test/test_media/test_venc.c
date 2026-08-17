#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <components/bk_frame_buffer.h>
#include <common/avdk_pixel_types.h>
#include <os/mem.h>
#include <os/str.h>

/* bk_draw_osd 的 emWin 字形栅格化，4.0.1.8 起被移出 CONFIG_VG_LITE_GPU 守卫，
 * 不依赖 GPU / frame buffer，专供 H264 叠加这类纯 CPU 路径复用。 */
#include "bk_osd_emwin_font.h"

#include "cli_tuya_test.h"
#include "tkl_camera_mipi.h"
#include "tkl_fs.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_thread.h"
#include "tkl_venc.h"

#define VENC_TEST_DEFAULT_FPS         15U
#define VENC_TEST_DEFAULT_GOP         40U
#define VENC_TEST_DEFAULT_QUALITY     2U
#define VENC_TEST_DEFAULT_BITRATE     1500000U
#define VENC_TEST_STATUS_MIN_WINDOW_MS 100U
#define VENC_TEST_LOG_INTERVAL        30U
#define VENC_TEST_RECORD_DEFAULT_SEC  3U
#define VENC_TEST_RECORD_POLL_MS      20U
#define VENC_TEST_RECORD_PATH_MAX     128U
#define VENC_TEST_RECORD_NAME_MAX     64U
#define VENC_TEST_RECORD_DEFAULT_FILE "h264_cap.h264"
#define VENC_TEST_RECORD_DEFAULT_JPEG_FILE "jpeg_cap.mjpeg"
#define VENC_TEST_SNAPSHOT_DEFAULT_FILE    "jpeg_snap.jpg"
#define VENC_TEST_SNAPSHOT_WAIT_MS         5000U
/* Depth must absorb FatFS write stalls; 8 slots drops frames at 2304x1296@15. */
#define VENC_TEST_RECORD_QUEUE_DEPTH  64U
#define VENC_TEST_RECORD_WRITER_STACK (4U * 1024U)
#define VENC_TEST_RECORD_WRITER_WAIT_MS 30000U
#define VENC_TEST_RECORD_FILE_READY_MS 3000U
#define VENC_TEST_RECORD_WRITER_IDLE_MS 5U
#define VENC_TEST_JPEGSW_OUTBUF_MIN       (256U * 1024U)
#define VENC_TEST_JPEGSW_OUTBUF_MAX       (1024U * 1024U)
#define VENC_TEST_JPEGSW_DEFAULT_FILE     "jpeg_cap.jpg"
#define VENC_TEST_JPEGSW_DEFAULT_QUALITY  80U
#define VENC_TEST_DUMP_SP_DEFAULT_PREFIX  "sp_dump"
#define VENC_TEST_OSD_MAX_INDEX           7U
#define VENC_TEST_OSD_TIME_INDEX          0U
#define VENC_TEST_OSD_TIME_X              2U
#define VENC_TEST_OSD_TIME_Y              2U
/* 尺寸由 osd_emwin_font_text_extent() 按字库实测得出，不再写死 */
#define VENC_TEST_OSD_TIME_ARGB           0xFFFFFF00U /* opaque yellow */
/* H264 硬件 OSD 要求 x/y 2 像素对齐；宽高一并取偶，便于后续改用 NV12 格式 */
#define VENC_TEST_OSD_XY_ALIGN            2U
#define VENC_TEST_OSD_TIME_REFRESH_MS     1000U
#define VENC_TEST_OSD_TIME_STACK          (3U * 1024U)
#define VENC_TEST_OSD_TIME_TEXT_LEN       16U

#if defined(CONFIG_BK_ENCODER_H264_MAX_OUTPUT_BUFFER) && (CONFIG_BK_ENCODER_H264_MAX_OUTPUT_BUFFER > 0)
#define VENC_TEST_RECORD_MAX_FRAME_SIZE ((uint32_t)CONFIG_BK_ENCODER_H264_MAX_OUTPUT_BUFFER)
#else
#define VENC_TEST_RECORD_MAX_FRAME_SIZE (512U * 1024U)
#endif

extern INT_T test_fs_mount(CONST CHAR_T *path, FS_DEV_TYPE_T dev_type);

typedef enum {
    VENC_TEST_SOURCE_CAMERA = 0,
    VENC_TEST_SOURCE_VDEC,
} test_venc_source_t;

typedef enum {
    VENC_TEST_CODEC_H264 = 0,
    VENC_TEST_CODEC_JPEG,
} test_venc_codec_t;

typedef struct {
    bool initialized;
    bool started;
    TUYA_VENC_HANDLE handle;
    test_venc_source_t source;
    test_venc_codec_t codec;
    uint8_t fps;
    uint32_t gop;
    uint8_t quality;
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
    uint64_t session_start_ms;
    uint64_t status_mark_ms;
    uint64_t status_mark_bytes;
    uint32_t status_mark_frames;
    bool session_timing_valid;
} test_venc_ctx_t;

static test_venc_ctx_t s_venc_ctx = {
    .initialized = false,
    .started = false,
    .handle = NULL,
    .source = VENC_TEST_SOURCE_CAMERA,
    .codec = VENC_TEST_CODEC_H264,
    .fps = VENC_TEST_DEFAULT_FPS,
    .gop = VENC_TEST_DEFAULT_GOP,
    .quality = VENC_TEST_DEFAULT_QUALITY,
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
    .session_start_ms = 0U,
    .status_mark_ms = 0U,
    .status_mark_bytes = 0U,
    .status_mark_frames = 0U,
    .session_timing_valid = false,
};

typedef struct {
    uint8_t *data;
    uint32_t size;
} test_venc_record_slot_t;

typedef struct {
    volatile bool capturing;
    volatile bool stop_req;
    volatile bool writer_running;
    volatile bool writer_done;
    volatile bool file_ready;
    volatile bool open_failed;
    TUYA_FILE file;
    TKL_THREAD_HANDLE writer_task;
    uint32_t duration_ms;
    uint32_t target_frames; /* 0: duration mode; >0: stop after N queued frames */
    uint64_t written_bytes;
    uint32_t queued_frames;
    uint32_t dropped_frames;
    uint32_t write_errors;
    char path[VENC_TEST_RECORD_PATH_MAX];
    test_venc_record_slot_t slots[VENC_TEST_RECORD_QUEUE_DEPTH];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
} test_venc_record_ctx_t;

static test_venc_record_ctx_t s_venc_record = {
    .capturing = false,
    .stop_req = false,
    .writer_running = false,
    .writer_done = true,
    .file_ready = false,
    .open_failed = false,
    .file = NULL,
    .writer_task = NULL,
    .duration_ms = 0U,
    .target_frames = 0U,
    .written_bytes = 0U,
    .queued_frames = 0U,
    .dropped_frames = 0U,
    .write_errors = 0U,
    .path = {0},
    .head = 0U,
    .tail = 0U,
    .count = 0U,
};

typedef struct {
    volatile bool running;
    volatile bool stop_req;
    TKL_THREAD_HANDLE task;
    char last_text[VENC_TEST_OSD_TIME_TEXT_LEN];
} test_venc_osd_time_ctx_t;

static test_venc_osd_time_ctx_t s_venc_osd_time = {
    .running = false,
    .stop_req = false,
    .task = NULL,
    .last_text = {0},
};

/* osd_assets/bk_font.c 导出的 emWin 字库，53px，含 0-9 / ':' / 空格，
 * 正是时间 OSD 需要的字符集。bk_font.c 没有配套头文件，这里直接 extern。 */
extern const gui_font_digit_struct *const font_digit_Roboto53;

extern OPERATE_RET test_vdec_get_flexa_source(TUYA_VCODEC_FLEXA_BUFFER_T *flexa);
extern BOOL_T test_mipi_camera_is_inited(void);
extern BOOL_T test_mipi_camera_is_started(void);
extern BOOL_T test_mipi_camera_flexa_enabled(void);
extern OPERATE_RET test_mipi_camera_get_sp_info(UINT16_T *width,
                                                UINT16_T *height,
                                                UINT32_T *format,
                                                UINT32_T *timeout_ms);
extern OPERATE_RET test_mipi_camera_get_flexa_source(TUYA_CAMERA_MIPI_FLEXA_SOURCE_T *camera_source);
extern OPERATE_RET test_mipi_camera_capture_frame(TUYA_CAMERA_MIPI_FRAME_T *frame);

static OPERATE_RET test_venc_stream_free(VOID *ptr, VOID *arg);
static bool test_venc_record_queue_push(const TUYA_VCODEC_STREAM_T *stream);
static bool test_venc_record_queue_contains(const VOID *ptr);
static void test_venc_osd_time_stop(void);

static const char *test_venc_source_name(test_venc_source_t source)
{
    return (source == VENC_TEST_SOURCE_VDEC) ? "vdec" : "camera";
}

static const char *test_venc_codec_name(test_venc_codec_t codec)
{
    return (codec == VENC_TEST_CODEC_JPEG) ? "jpeg" : "h264";
}

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

static uint32_t test_venc_elapsed_ms(uint64_t start_ms, uint64_t now_ms)
{
    if (now_ms >= start_ms) {
        return (uint32_t)(now_ms - start_ms);
    }

    return (uint32_t)(((UINT64_MAX - start_ms) + now_ms + 1U));
}

static uint32_t test_venc_calc_kbps(uint64_t bytes, uint32_t elapsed_ms)
{
    uint64_t bit_rate_kbps;

    if (elapsed_ms < VENC_TEST_STATUS_MIN_WINDOW_MS || bytes == 0U) {
        return 0U;
    }

    bit_rate_kbps = (bytes * 8U * 1000U) / (uint64_t)elapsed_ms;
    return (uint32_t)(bit_rate_kbps / 1024U);
}

static uint32_t test_venc_calc_fps(uint32_t frames, uint32_t elapsed_ms)
{
    if (elapsed_ms < VENC_TEST_STATUS_MIN_WINDOW_MS || frames == 0U) {
        return 0U;
    }

    return (frames * 1000U + elapsed_ms / 2U) / elapsed_ms;
}

static void test_venc_timing_mark_start(void)
{
    uint64_t now_ms = (uint64_t)tkl_system_get_millisecond();

    s_venc_ctx.session_start_ms = now_ms;
    s_venc_ctx.status_mark_ms = now_ms;
    s_venc_ctx.status_mark_bytes = 0U;
    s_venc_ctx.status_mark_frames = 0U;
    s_venc_ctx.session_timing_valid = true;
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
    s_venc_ctx.source = VENC_TEST_SOURCE_CAMERA;
    s_venc_ctx.codec = VENC_TEST_CODEC_H264;
    s_venc_ctx.fps = VENC_TEST_DEFAULT_FPS;
    s_venc_ctx.gop = VENC_TEST_DEFAULT_GOP;
    s_venc_ctx.quality = VENC_TEST_DEFAULT_QUALITY;
    s_venc_ctx.bitrate = VENC_TEST_DEFAULT_BITRATE;
    s_venc_ctx.session_timing_valid = false;
    test_venc_stats_reset();
}

static void test_venc_record_ctx_reset(void)
{
    s_venc_record.capturing = false;
    s_venc_record.stop_req = false;
    s_venc_record.writer_running = false;
    s_venc_record.writer_done = true;
    s_venc_record.file_ready = false;
    s_venc_record.open_failed = false;
    s_venc_record.file = NULL;
    s_venc_record.writer_task = NULL;
    s_venc_record.duration_ms = 0U;
    s_venc_record.target_frames = 0U;
    s_venc_record.written_bytes = 0U;
    s_venc_record.queued_frames = 0U;
    s_venc_record.dropped_frames = 0U;
    s_venc_record.write_errors = 0U;
    s_venc_record.path[0] = '\0';
    s_venc_record.head = 0U;
    s_venc_record.tail = 0U;
    s_venc_record.count = 0U;
}

static bool test_venc_record_queue_contains_locked(const VOID *ptr)
{
    uint32_t idx;
    uint32_t pos;

    if (ptr == NULL || s_venc_record.count == 0U) {
        return false;
    }

    pos = s_venc_record.head;
    for (idx = 0U; idx < s_venc_record.count; idx++) {
        if (s_venc_record.slots[pos].data == ptr) {
            return true;
        }
        pos = (pos + 1U) % VENC_TEST_RECORD_QUEUE_DEPTH;
    }

    return false;
}

static void test_venc_record_queue_purge(void)
{
    uint32_t idx;
    UINT_T irq_mask;
    test_venc_record_slot_t pending[VENC_TEST_RECORD_QUEUE_DEPTH];
    uint32_t pending_count = 0U;

    os_memset(pending, 0, sizeof(pending));

    irq_mask = tkl_system_enter_critical();
    while (s_venc_record.count > 0U) {
        pending[pending_count].data = s_venc_record.slots[s_venc_record.head].data;
        pending[pending_count].size = s_venc_record.slots[s_venc_record.head].size;
        pending_count++;
        s_venc_record.slots[s_venc_record.head].data = NULL;
        s_venc_record.slots[s_venc_record.head].size = 0U;
        s_venc_record.head = (s_venc_record.head + 1U) % VENC_TEST_RECORD_QUEUE_DEPTH;
        s_venc_record.count--;
    }
    s_venc_record.head = 0U;
    s_venc_record.tail = 0U;
    tkl_system_exit_critical(irq_mask);

    for (idx = 0U; idx < pending_count; idx++) {
        if (pending[idx].data != NULL) {
            (void)test_venc_stream_free(pending[idx].data, NULL);
        }
    }
}

static bool test_venc_record_name_valid(const char *name)
{
    size_t len;

    if (name == NULL || name[0] == '\0') {
        return false;
    }

    if (os_strstr(name, "..") != NULL) {
        return false;
    }

    len = os_strlen(name);
    if (len >= VENC_TEST_RECORD_NAME_MAX) {
        return false;
    }

    return true;
}

static OPERATE_RET test_venc_record_build_path(const char *filename, char *path, size_t path_size)
{
    int ret;

    if (filename == NULL || path == NULL || path_size == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (!test_venc_record_name_valid(filename)) {
        return OPRT_INVALID_PARM;
    }

    if (filename[0] == '/') {
        ret = os_snprintf(path, path_size, "%s", filename);
    } else {
        ret = os_snprintf(path, path_size, "%s/%s", TEST_SDCARD_MOUNT_POINT, filename);
    }

    if (ret <= 0 || (size_t)ret >= path_size) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static bool test_venc_record_queue_contains(const VOID *ptr)
{
    bool found;
    UINT_T irq_mask;

    irq_mask = tkl_system_enter_critical();
    found = test_venc_record_queue_contains_locked(ptr);
    tkl_system_exit_critical(irq_mask);
    return found;
}

static bool test_venc_record_queue_push(const TUYA_VCODEC_STREAM_T *stream)
{
    uint32_t tail;
    UINT_T irq_mask;
    bool pushed = false;

    if (stream == NULL || stream->data == NULL || stream->size == 0U) {
        return false;
    }

    if (stream->frame_type == TUYA_VCODEC_FRAME_END) {
        return false;
    }

    if (stream->size > VENC_TEST_RECORD_MAX_FRAME_SIZE) {
        irq_mask = tkl_system_enter_critical();
        s_venc_record.dropped_frames++;
        tkl_system_exit_critical(irq_mask);
        return false;
    }

    irq_mask = tkl_system_enter_critical();
    if (s_venc_record.capturing != true) {
        tkl_system_exit_critical(irq_mask);
        return false;
    }

    if (s_venc_record.count >= VENC_TEST_RECORD_QUEUE_DEPTH) {
        s_venc_record.dropped_frames++;
        tkl_system_exit_critical(irq_mask);
        return false;
    }

    tail = s_venc_record.tail;
    s_venc_record.slots[tail].data = stream->data;
    s_venc_record.slots[tail].size = stream->size;
    s_venc_record.tail = (tail + 1U) % VENC_TEST_RECORD_QUEUE_DEPTH;
    s_venc_record.count++;
    s_venc_record.queued_frames++;
    if (s_venc_record.target_frames > 0U &&
        s_venc_record.queued_frames >= s_venc_record.target_frames) {
        s_venc_record.capturing = false;
        s_venc_record.stop_req = true;
    }
    pushed = true;
    tkl_system_exit_critical(irq_mask);

    return pushed;
}

static void test_venc_record_hold_stream(const TUYA_VCODEC_STREAM_T *stream)
{
    (void)test_venc_record_queue_push(stream);
}

static OPERATE_RET test_venc_record_stream_free(VOID *ptr, VOID *arg)
{
    if (ptr != NULL && test_venc_record_queue_contains(ptr) == true) {
        return OPRT_OK;
    }

    return test_venc_stream_free(ptr, arg);
}

static void test_venc_record_writer_main(void *arg)
{
    TUYA_FILE file = NULL;
    TKL_THREAD_HANDLE self = NULL;
    char path[VENC_TEST_RECORD_PATH_MAX];
    uint64_t written_bytes = 0U;
    uint32_t dropped_frames = 0U;
    uint32_t write_errors = 0U;
    uint32_t queued_frames = 0U;

    (void)arg;

    os_snprintf(path, sizeof(path), "%s", s_venc_record.path);
    file = tkl_fopen(path, "wb");
    if (file == NULL) {
        tkl_log_output("venc record open failed: %s\r\n", path);
        s_venc_record.open_failed = true;
        s_venc_record.writer_done = true;
        s_venc_record.writer_running = false;
        goto writer_exit;
    }

    s_venc_record.file = file;
    s_venc_record.file_ready = true;

    while (true) {
        uint8_t *data = NULL;
        uint32_t size = 0U;
        bool should_exit = false;
        INT_T write_len;
        UINT_T irq_mask;

        irq_mask = tkl_system_enter_critical();
        if (s_venc_record.count > 0U) {
            data = s_venc_record.slots[s_venc_record.head].data;
            size = s_venc_record.slots[s_venc_record.head].size;
            s_venc_record.slots[s_venc_record.head].data = NULL;
            s_venc_record.slots[s_venc_record.head].size = 0U;
            s_venc_record.head = (s_venc_record.head + 1U) % VENC_TEST_RECORD_QUEUE_DEPTH;
            s_venc_record.count--;
        }
        if (s_venc_record.stop_req == true && s_venc_record.count == 0U) {
            should_exit = true;
        }
        dropped_frames = s_venc_record.dropped_frames;
        queued_frames = s_venc_record.queued_frames;
        tkl_system_exit_critical(irq_mask);

        if (data != NULL && size > 0U) {
            write_len = tkl_fwrite(data, (INT_T)size, file);
            if (write_len != (INT_T)size) {
                write_errors++;
                tkl_log_output("venc record write failed: expect=%u actual=%d\r\n",
                               (unsigned int)size,
                               write_len);
            } else {
                written_bytes += size;
            }
            (void)test_venc_stream_free(data, NULL);
        } else if (should_exit == true) {
            break;
        } else {
            tkl_system_sleep(VENC_TEST_RECORD_WRITER_IDLE_MS);
        }
    }

    (void)tkl_fsync((INT_T)file);
    (void)tkl_fclose(file);
    s_venc_record.file = NULL;
    s_venc_record.written_bytes = written_bytes;
    s_venc_record.write_errors = write_errors;

    tkl_log_output("venc record done: path=%s bytes=%llu queued=%u dropped=%u write_err=%u\r\n",
                   path,
                   (unsigned long long)written_bytes,
                   (unsigned int)queued_frames,
                   (unsigned int)dropped_frames,
                   (unsigned int)write_errors);

    s_venc_record.writer_running = false;
    s_venc_record.writer_done = true;

writer_exit:
    if (tkl_thread_get_id(&self) == OPRT_OK && self != NULL) {
        tkl_thread_release(self);
    }
}

static OPERATE_RET test_venc_record_writer_start(void)
{
    OPERATE_RET ret;

    if (s_venc_record.writer_running == true) {
        return OPRT_RESOURCE_NOT_READY;
    }

    s_venc_record.writer_running = true;
    s_venc_record.writer_done = false;
    s_venc_record.file_ready = false;
    s_venc_record.open_failed = false;
    s_venc_record.stop_req = false;
    s_venc_record.writer_task = NULL;

    ret = tkl_thread_create(&s_venc_record.writer_task,
                            "venc_rec",
                            VENC_TEST_RECORD_WRITER_STACK,
                            TKL_THREAD_PRI_NORMAL,
                            test_venc_record_writer_main,
                            NULL);
    if (ret != OPRT_OK) {
        s_venc_record.writer_running = false;
        s_venc_record.writer_done = true;
        tkl_log_output("venc record create writer task failed: %d\r\n", ret);
        return ret;
    }

    return OPRT_OK;
}

static OPERATE_RET test_venc_record_writer_wait_ready(uint32_t timeout_ms)
{
    uint32_t waited_ms = 0U;

    while (s_venc_record.file_ready != true &&
           s_venc_record.open_failed != true &&
           waited_ms < timeout_ms) {
        tkl_system_sleep(VENC_TEST_RECORD_POLL_MS);
        waited_ms += VENC_TEST_RECORD_POLL_MS;
    }

    if (s_venc_record.open_failed == true) {
        return OPRT_COM_ERROR;
    }

    if (s_venc_record.file_ready != true) {
        tkl_log_output("venc record wait writer ready timeout\r\n");
        return OPRT_TIMEOUT;
    }

    return OPRT_OK;
}

static OPERATE_RET test_venc_record_writer_wait_done(uint32_t timeout_ms)
{
    uint32_t waited_ms = 0U;

    while (s_venc_record.writer_done != true && waited_ms < timeout_ms) {
        tkl_system_sleep(VENC_TEST_RECORD_POLL_MS);
        waited_ms += VENC_TEST_RECORD_POLL_MS;
    }

    if (s_venc_record.writer_done != true) {
        tkl_log_output("venc record wait writer done timeout\r\n");
        return OPRT_TIMEOUT;
    }

    s_venc_record.writer_task = NULL;
    return OPRT_OK;
}

static OPERATE_RET test_venc_record_stop(void)
{
    OPERATE_RET ret;

    if (s_venc_record.writer_running != true && s_venc_record.writer_done == true) {
        test_venc_record_queue_purge();
        test_venc_record_ctx_reset();
        return OPRT_OK;
    }

    s_venc_record.capturing = false;
    s_venc_record.stop_req = true;
    ret = test_venc_record_writer_wait_done(VENC_TEST_RECORD_WRITER_WAIT_MS);
    test_venc_record_queue_purge();
    test_venc_record_ctx_reset();

    return ret;
}

static bool test_venc_record_is_busy(void)
{
    return (s_venc_record.writer_running == true ||
            s_venc_record.capturing == true ||
            s_venc_record.writer_done == false);
}

static OPERATE_RET test_venc_record(uint32_t duration_sec, const char *filename)
{
    OPERATE_RET ret;
    uint32_t waited_ms = 0U;
    const char *default_file;

    if (s_venc_ctx.initialized != true || s_venc_ctx.started != true) {
        tkl_log_output("venc is not started, use: xmipi camera open_gpu ... && xmipi camera start && xvenc init && xvenc start\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_venc_ctx.codec != VENC_TEST_CODEC_H264 &&
        s_venc_ctx.codec != VENC_TEST_CODEC_JPEG) {
        tkl_log_output("venc record only supports h264/jpeg\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    if (test_venc_record_is_busy() == true) {
        tkl_log_output("venc record is already running\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (duration_sec == 0U) {
        duration_sec = VENC_TEST_RECORD_DEFAULT_SEC;
    }

    default_file = (s_venc_ctx.codec == VENC_TEST_CODEC_JPEG) ?
            VENC_TEST_RECORD_DEFAULT_JPEG_FILE : VENC_TEST_RECORD_DEFAULT_FILE;
    if (filename == NULL || filename[0] == '\0') {
        filename = default_file;
    }

    os_memset(&s_venc_record, 0, sizeof(s_venc_record));
    s_venc_record.writer_done = true;
    ret = test_venc_record_build_path(filename, s_venc_record.path, sizeof(s_venc_record.path));
    if (ret != OPRT_OK) {
        tkl_log_output("venc record invalid filename: %s\r\n",
                       (filename != NULL) ? filename : "null");
        return ret;
    }

    if (test_fs_mount(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD) != 0) {
        tkl_log_output("venc record mount %s failed\r\n", TEST_SDCARD_MOUNT_POINT);
        test_venc_record_ctx_reset();
        return OPRT_COM_ERROR;
    }

    s_venc_record.duration_ms = duration_sec * 1000U;
    s_venc_record.target_frames = 0U;
    s_venc_record.capturing = true;
    s_venc_record.stop_req = false;

    ret = test_venc_record_writer_start();
    if (ret != OPRT_OK) {
        s_venc_record.capturing = false;
        test_venc_record_ctx_reset();
        return ret;
    }

    ret = test_venc_record_writer_wait_ready(VENC_TEST_RECORD_FILE_READY_MS);
    if (ret != OPRT_OK) {
        s_venc_record.capturing = false;
        s_venc_record.stop_req = true;
        (void)test_venc_record_writer_wait_done(VENC_TEST_RECORD_WRITER_WAIT_MS);
        test_venc_record_queue_purge();
        test_venc_record_ctx_reset();
        return ret;
    }

    tkl_log_output(
            "venc record start: codec=%s path=%s duration=%us src=%ux%u fps=%u async=1 queue=%u zero_copy=1\r\n",
            test_venc_codec_name(s_venc_ctx.codec),
            s_venc_record.path,
            (unsigned int)duration_sec,
            s_venc_ctx.width,
            s_venc_ctx.height,
            (unsigned int)s_venc_ctx.fps,
            (unsigned int)VENC_TEST_RECORD_QUEUE_DEPTH);
    if (s_venc_ctx.codec == VENC_TEST_CODEC_JPEG) {
        tkl_log_output("venc record note: jpeg frames are concatenated (MJPEG-like)\r\n");
    }

    while (waited_ms < s_venc_record.duration_ms) {
        tkl_system_sleep(VENC_TEST_RECORD_POLL_MS);
        waited_ms += VENC_TEST_RECORD_POLL_MS;
    }

    s_venc_record.capturing = false;
    s_venc_record.stop_req = true;
    ret = test_venc_record_writer_wait_done(VENC_TEST_RECORD_WRITER_WAIT_MS);
    test_venc_record_queue_purge();
    test_venc_record_ctx_reset();
    return ret;
}

/**
 * @brief Capture exactly one HW JPEG frame to SD card
 * @param[in] filename output file name under /sdcard; NULL uses default
 * @return OPRT_OK on success
 */
static OPERATE_RET test_venc_snapshot(const char *filename)
{
    OPERATE_RET ret;

    if (s_venc_ctx.initialized != true || s_venc_ctx.started != true) {
        tkl_log_output("venc snapshot needs HW jpeg running: xvenc init camera jpeg ... && xvenc start\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_venc_ctx.codec != VENC_TEST_CODEC_JPEG) {
        tkl_log_output("venc snapshot only supports jpeg codec (use jpegsw for soft encode)\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    if (test_venc_record_is_busy() == true) {
        tkl_log_output("venc record/snapshot is already running\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (filename == NULL || filename[0] == '\0') {
        filename = VENC_TEST_SNAPSHOT_DEFAULT_FILE;
    }

    os_memset(&s_venc_record, 0, sizeof(s_venc_record));
    s_venc_record.writer_done = true;
    ret = test_venc_record_build_path(filename, s_venc_record.path, sizeof(s_venc_record.path));
    if (ret != OPRT_OK) {
        tkl_log_output("venc snapshot invalid filename: %s\r\n", filename);
        return ret;
    }

    if (test_fs_mount(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD) != 0) {
        tkl_log_output("venc snapshot mount %s failed\r\n", TEST_SDCARD_MOUNT_POINT);
        test_venc_record_ctx_reset();
        return OPRT_COM_ERROR;
    }

    s_venc_record.duration_ms = 0U;
    s_venc_record.target_frames = 1U;
    s_venc_record.capturing = true;
    s_venc_record.stop_req = false;

    ret = test_venc_record_writer_start();
    if (ret != OPRT_OK) {
        s_venc_record.capturing = false;
        test_venc_record_ctx_reset();
        return ret;
    }

    ret = test_venc_record_writer_wait_ready(VENC_TEST_RECORD_FILE_READY_MS);
    if (ret != OPRT_OK) {
        s_venc_record.capturing = false;
        s_venc_record.stop_req = true;
        (void)test_venc_record_writer_wait_done(VENC_TEST_RECORD_WRITER_WAIT_MS);
        test_venc_record_queue_purge();
        test_venc_record_ctx_reset();
        return ret;
    }

    tkl_log_output("venc snapshot start: path=%s src=%ux%u frames=1\r\n",
                   s_venc_record.path,
                   s_venc_ctx.width,
                   s_venc_ctx.height);

    ret = test_venc_record_writer_wait_done(VENC_TEST_SNAPSHOT_WAIT_MS);
    if (ret != OPRT_OK) {
        s_venc_record.capturing = false;
        s_venc_record.stop_req = true;
        (void)test_venc_record_writer_wait_done(VENC_TEST_RECORD_WRITER_WAIT_MS);
    }

    test_venc_record_queue_purge();
    test_venc_record_ctx_reset();
    return ret;
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

    test_venc_record_hold_stream(stream);

    if (s_venc_ctx.total_frames <= 3U ||
        (s_venc_ctx.total_frames % VENC_TEST_LOG_INTERVAL) == 0U) {
        tkl_log_output(
                "venc frame[%u]: codec=%s type=%s size=%u total_bytes=%llu\r\n",
                (unsigned int)s_venc_ctx.total_frames,
                test_venc_codec_name(s_venc_ctx.codec),
                test_venc_frame_type_name(stream->frame_type),
                (unsigned int)stream->size,
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

static OPERATE_RET test_venc_set_vdec_source(void)
{
    OPERATE_RET ret;
    TUYA_VCODEC_FLEXA_BUFFER_T flexa;
    TUYA_VENC_SOURCE_T source;

    if (s_venc_ctx.codec == VENC_TEST_CODEC_JPEG) {
        tkl_log_output("jpeg encode only supports camera source, use: xvenc init camera jpeg ...\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    os_memset(&flexa, 0, sizeof(flexa));
    ret = test_vdec_get_flexa_source(&flexa);
    if (ret != OPRT_OK) {
        tkl_log_output("vdec get flexa source failed: %d, use: xvdec init ... flexa && xvdec start\r\n", ret);
        return ret;
    }

    if (flexa.owner_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    os_memset(&source, 0, sizeof(source));
    source.type = TUYA_VENC_SOURCE_VDEC_FLEXA;
    source.vdec.vdec_handle = flexa.owner_handle;
    source.vdec.flexa = flexa;

    ret = tkl_venc_set_source(s_venc_ctx.handle, &source);
    if (ret != OPRT_OK) {
        tkl_log_output("venc set vdec source failed: %d\r\n", ret);
        return ret;
    }

    s_venc_ctx.width = flexa.width;
    s_venc_ctx.height = flexa.height;
    s_venc_ctx.format = flexa.format;
    test_venc_stream_stats_reset();

    tkl_log_output("venc vdec source ready, owner=%p src=%ux%u fmt=%u buf=%p cnt=%u\r\n",
            flexa.owner_handle,
            flexa.width,
            flexa.height,
            (unsigned int)flexa.format,
            flexa.src_buffer,
            (unsigned int)flexa.buf_cnt);
    return OPRT_OK;
}

static OPERATE_RET test_venc_init(test_venc_source_t source_type,
                                  test_venc_codec_t codec_type,
                                  uint8_t fps,
                                  uint32_t gop,
                                  uint8_t quality,
                                  uint32_t bitrate)
{
    OPERATE_RET ret;
    TUYA_VENC_CFG_T cfg;
    TUYA_VENC_SINK_T sink;

    if (s_venc_ctx.started == true) {
        tkl_log_output("venc is running, stop before reinit\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (source_type == VENC_TEST_SOURCE_VDEC && codec_type == VENC_TEST_CODEC_JPEG) {
        tkl_log_output("jpeg encode only supports mipi camera source\r\n");
        return OPRT_NOT_SUPPORTED;
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
    cfg.codec = (codec_type == VENC_TEST_CODEC_JPEG) ?
            TUYA_VCODEC_TYPE_JPEG : TUYA_VCODEC_TYPE_H264;
    cfg.fps = fps;
    cfg.gop = gop;
    cfg.quality = quality;
    cfg.input_format = BK_PIXEL_FORMAT_NV12;

    ret = tkl_venc_init(&s_venc_ctx.handle, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("venc init failed: %d\r\n", ret);
        return ret;
    }

    os_memset(&sink, 0, sizeof(sink));
    sink.stream_cb = test_venc_stream_cb;
    sink.alloc_cb = test_venc_stream_alloc;
    sink.free_cb = test_venc_record_stream_free;
    ret = tkl_venc_set_sink(s_venc_ctx.handle, &sink);
    if (ret != OPRT_OK) {
        tkl_log_output("venc set sink failed: %d\r\n", ret);
        (void)tkl_venc_deinit(s_venc_ctx.handle);
        test_venc_runtime_reset();
        return ret;
    }

    if (source_type == VENC_TEST_SOURCE_VDEC) {
        ret = test_venc_set_vdec_source();
    } else {
        ret = test_venc_set_camera_source();
    }
    if (ret != OPRT_OK) {
        (void)tkl_venc_deinit(s_venc_ctx.handle);
        test_venc_runtime_reset();
        return ret;
    }

    if (codec_type == VENC_TEST_CODEC_H264 && bitrate != VENC_TEST_DEFAULT_BITRATE) {
        TUYA_VENC_RATE_CTRL_T rc;

        os_memset(&rc, 0, sizeof(rc));
        rc.bitrate = bitrate;
        ret = tkl_venc_set_rate_ctrl(s_venc_ctx.handle, &rc);
        if (ret != OPRT_OK) {
            tkl_log_output("venc set rate ctrl failed: %d\r\n", ret);
            (void)tkl_venc_deinit(s_venc_ctx.handle);
            test_venc_runtime_reset();
            return ret;
        }
    }

    s_venc_ctx.initialized = true;
    s_venc_ctx.started = false;
    s_venc_ctx.source = source_type;
    s_venc_ctx.codec = codec_type;
    s_venc_ctx.fps = fps;
    s_venc_ctx.gop = gop;
    s_venc_ctx.quality = quality;
    s_venc_ctx.bitrate = bitrate;
    test_venc_stream_stats_reset();

    if (codec_type == VENC_TEST_CODEC_JPEG) {
        tkl_log_output("venc init ok, codec=JPEG source=%s src=%ux%u fmt=%u fps=%u quality=%u\r\n",
                test_venc_source_name(source_type),
                s_venc_ctx.width,
                s_venc_ctx.height,
                (unsigned int)s_venc_ctx.format,
                (unsigned int)fps,
                (unsigned int)quality);
    } else {
        tkl_log_output("venc init ok, codec=H264 source=%s src=%ux%u fmt=%u fps=%u gop=%u bitrate=%u\r\n",
                test_venc_source_name(source_type),
                s_venc_ctx.width,
                s_venc_ctx.height,
                (unsigned int)s_venc_ctx.format,
                (unsigned int)fps,
                (unsigned int)gop,
                (unsigned int)bitrate);
    }
    return OPRT_OK;
}

static OPERATE_RET test_venc_start(void)
{
    OPERATE_RET ret;

    if (s_venc_ctx.initialized != true || s_venc_ctx.handle == NULL) {
        tkl_log_output("venc is not initialized, use: xvenc init [camera|vdec] [h264|jpeg] [fps] [gop|quality] [bitrate]\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_venc_ctx.started == true) {
        tkl_log_output("venc already started\r\n");
        return OPRT_OK;
    }
    test_venc_stream_stats_reset();

    if (s_venc_ctx.codec == VENC_TEST_CODEC_JPEG) {
        tkl_log_output("venc start request: codec=JPEG source=%s src=%ux%u fmt=%u fps=%u quality=%u\r\n",
                test_venc_source_name(s_venc_ctx.source),
                s_venc_ctx.width,
                s_venc_ctx.height,
                (unsigned int)s_venc_ctx.format,
                (unsigned int)s_venc_ctx.fps,
                (unsigned int)s_venc_ctx.quality);
    } else {
        tkl_log_output("venc start request: codec=H264 source=%s src=%ux%u fmt=%u fps=%u gop=%u bitrate=%u\r\n",
                test_venc_source_name(s_venc_ctx.source),
                s_venc_ctx.width,
                s_venc_ctx.height,
                (unsigned int)s_venc_ctx.format,
                (unsigned int)s_venc_ctx.fps,
                (unsigned int)s_venc_ctx.gop,
                (unsigned int)s_venc_ctx.bitrate);
    }

    ret = tkl_venc_start(s_venc_ctx.handle);
    if (ret != OPRT_OK) {
        tkl_log_output("venc start failed: %d\r\n", ret);
        return ret;
    }

    test_venc_timing_mark_start();
    s_venc_ctx.started = true;
    tkl_log_output("venc started, source=%s src=%ux%u fmt=%u -> %s\r\n",
            test_venc_source_name(s_venc_ctx.source),
            s_venc_ctx.width,
            s_venc_ctx.height,
            (unsigned int)s_venc_ctx.format,
            test_venc_codec_name(s_venc_ctx.codec));
    return OPRT_OK;
}

static OPERATE_RET test_venc_stop(void)
{
    OPERATE_RET ret;

    test_venc_osd_time_stop();

    if (test_venc_record_is_busy() == true) {
        (void)test_venc_record_stop();
    }

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

    test_venc_osd_time_stop();

    if (test_venc_record_is_busy() == true) {
        (void)test_venc_record_stop();
    }

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

    if (s_venc_ctx.codec == VENC_TEST_CODEC_JPEG) {
        tkl_log_output("idr is only supported for h264\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    return tkl_venc_request_idr(s_venc_ctx.handle);
}

static void test_venc_osd_buffer_free(void *buffer, void *free_arg)
{
    (void)free_arg;
    if (buffer != NULL) {
        bk_frame_buffer_free(buffer);
    }
}

/**
 * @brief 用 emWin 字库量出 text 的 OSD 区域尺寸（已按硬件对齐要求取偶）
 * @param[in] text 待渲染字符串
 * @param[out] out_w OSD 宽
 * @param[out] out_h OSD 高
 * @return OPRT_OK 表示尺寸有效
 */
static OPERATE_RET test_venc_osd_time_extent(const char *text,
                                             uint16_t *out_w,
                                             uint16_t *out_h)
{
    uint16_t w = 0U;
    uint16_t h = 0U;

    if (text == NULL || out_w == NULL || out_h == NULL) {
        return OPRT_INVALID_PARM;
    }

    osd_emwin_font_text_extent(font_digit_Roboto53, text, &w, &h);
    if (w == 0U || h == 0U) {
        tkl_log_output("osd time: font has no glyph for \"%s\"\r\n", text);
        return OPRT_COM_ERROR;
    }

    *out_w = (uint16_t)((w + VENC_TEST_OSD_XY_ALIGN - 1U) & ~(VENC_TEST_OSD_XY_ALIGN - 1U));
    *out_h = (uint16_t)((h + VENC_TEST_OSD_XY_ALIGN - 1U) & ~(VENC_TEST_OSD_XY_ALIGN - 1U));
    return OPRT_OK;
}

/**
 * @brief Format uptime as HH:MM:SS
 * @param[out] text output string buffer
 * @param[in] text_len buffer length
 * @return none
 */
static void test_venc_osd_format_time(char *text, size_t text_len)
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

/**
 * @brief Push one time OSD frame to slot 0 at top-left
 * @param[in] text HH:MM:SS string
 * @return OPRT_OK on success
 */
static OPERATE_RET test_venc_osd_submit_time(const char *text)
{
    TUYA_VENC_OSD_T osd;
    uint32_t *pixels = NULL;
    uint32_t buf_bytes;
    uint16_t osd_w = 0U;
    uint16_t osd_h = 0U;
    OPERATE_RET ret;

    if (s_venc_ctx.initialized != true || s_venc_ctx.handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_venc_ctx.codec != VENC_TEST_CODEC_H264) {
        return OPRT_NOT_SUPPORTED;
    }

    ret = test_venc_osd_time_extent(text, &osd_w, &osd_h);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (s_venc_ctx.width > 0U && s_venc_ctx.height > 0U) {
        if (((uint32_t)VENC_TEST_OSD_TIME_X + osd_w) > s_venc_ctx.width ||
            ((uint32_t)VENC_TEST_OSD_TIME_Y + osd_h) > s_venc_ctx.height) {
            tkl_log_output("osd time region %ux%u exceeds frame %ux%u\r\n",
                           (unsigned int)osd_w,
                           (unsigned int)osd_h,
                           (unsigned int)s_venc_ctx.width,
                           (unsigned int)s_venc_ctx.height);
            return OPRT_INVALID_PARM;
        }
    }

    buf_bytes = (uint32_t)osd_w * osd_h * sizeof(uint32_t);
    /* 必须用 UNCODED slab：这块非缓存内存让 CPU 写入对 VPU 立即可见。
     * 换成普通 malloc 会因 D-Cache 未回写而出现间歇性花屏。 */
    pixels = (uint32_t *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, buf_bytes);
    if (pixels == NULL) {
        tkl_log_output("osd time malloc %u bytes failed\r\n", (unsigned int)buf_bytes);
        return OPRT_MALLOC_FAILED;
    }

    /* 底色清成全 0（alpha=0 全透明），字形覆盖度由 blit 写成 per-pixel alpha */
    os_memset(pixels, 0, buf_bytes);
    (void)osd_emwin_font_blit(pixels, osd_w, osd_h,
                              font_digit_Roboto53, text,
                              0U, 0U, VENC_TEST_OSD_TIME_ARGB, NULL);

    os_memset(&osd, 0, sizeof(osd));
    osd.index = VENC_TEST_OSD_TIME_INDEX;
    osd.buffer = pixels;
    osd.format = TUYA_VENC_OSD_FMT_ARGB8888;
    osd.x = VENC_TEST_OSD_TIME_X;
    osd.y = VENC_TEST_OSD_TIME_Y;
    osd.width = osd_w;
    osd.height = osd_h;
    osd.buffer_free = test_venc_osd_buffer_free;
    osd.free_arg = NULL;

    ret = tkl_venc_set_osd(s_venc_ctx.handle, &osd);
    if (ret != OPRT_OK) {
        bk_frame_buffer_free(pixels);
    }

    return ret;
}

static void test_venc_osd_time_task(void *arg)
{
    TKL_THREAD_HANDLE self = NULL;
    char text[VENC_TEST_OSD_TIME_TEXT_LEN];

    (void)arg;
    (void)tkl_thread_get_id(&self);

    while (s_venc_osd_time.stop_req != true) {
        test_venc_osd_format_time(text, sizeof(text));
        if (os_strcmp(text, s_venc_osd_time.last_text) != 0) {
            if (test_venc_osd_submit_time(text) == OPRT_OK) {
                os_strncpy(s_venc_osd_time.last_text, text, sizeof(s_venc_osd_time.last_text) - 1U);
                s_venc_osd_time.last_text[sizeof(s_venc_osd_time.last_text) - 1U] = '\0';
            }
        }
        tkl_system_sleep(VENC_TEST_OSD_TIME_REFRESH_MS);
    }

    s_venc_osd_time.running = false;
    s_venc_osd_time.task = NULL;
    tkl_thread_release(self);
}

/**
 * @brief Stop time OSD refresh task
 * @return none
 */
static void test_venc_osd_time_stop(void)
{
    uint32_t waited_ms = 0U;

    if (s_venc_osd_time.running != true && s_venc_osd_time.task == NULL) {
        return;
    }

    s_venc_osd_time.stop_req = true;
    while (s_venc_osd_time.running == true && waited_ms < 3000U) {
        tkl_system_sleep(20U);
        waited_ms += 20U;
    }

    s_venc_osd_time.running = false;
    s_venc_osd_time.task = NULL;
    s_venc_osd_time.stop_req = false;
    s_venc_osd_time.last_text[0] = '\0';
}

static OPERATE_RET test_venc_osd_clear(uint32_t index)
{
    TUYA_VENC_OSD_T osd;

    if (s_venc_ctx.initialized != true || s_venc_ctx.handle == NULL) {
        tkl_log_output("venc is not initialized\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_venc_ctx.codec != VENC_TEST_CODEC_H264) {
        tkl_log_output("osd is only supported for h264\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    if (index > VENC_TEST_OSD_MAX_INDEX) {
        tkl_log_output("osd index must be 0..%u\r\n", (unsigned int)VENC_TEST_OSD_MAX_INDEX);
        return OPRT_INVALID_PARM;
    }

    if (index == VENC_TEST_OSD_TIME_INDEX) {
        test_venc_osd_time_stop();
    }

    os_memset(&osd, 0, sizeof(osd));
    osd.index = index;
    osd.buffer = NULL;

    return tkl_venc_set_osd(s_venc_ctx.handle, &osd);
}

/**
 * @brief Start top-left HH:MM:SS OSD and refresh every second
 * @return OPRT_OK on success
 */
static OPERATE_RET test_venc_osd_time_start(void)
{
    OPERATE_RET ret;
    char text[VENC_TEST_OSD_TIME_TEXT_LEN];
    uint16_t osd_w = 0U;
    uint16_t osd_h = 0U;

    if (s_venc_ctx.initialized != true || s_venc_ctx.handle == NULL) {
        tkl_log_output("venc is not initialized\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_venc_ctx.codec != VENC_TEST_CODEC_H264) {
        tkl_log_output("osd is only supported for h264\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    test_venc_osd_time_stop();

    test_venc_osd_format_time(text, sizeof(text));
    ret = test_venc_osd_submit_time(text);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_strncpy(s_venc_osd_time.last_text, text, sizeof(s_venc_osd_time.last_text) - 1U);
    s_venc_osd_time.last_text[sizeof(s_venc_osd_time.last_text) - 1U] = '\0';
    s_venc_osd_time.stop_req = false;
    s_venc_osd_time.running = true;

    ret = tkl_thread_create(&s_venc_osd_time.task,
                            "venc_osd_time",
                            VENC_TEST_OSD_TIME_STACK,
                            TKL_THREAD_PRI_NORMAL,
                            test_venc_osd_time_task,
                            NULL);
    if (ret != OPRT_OK) {
        s_venc_osd_time.running = false;
        s_venc_osd_time.task = NULL;
        tkl_log_output("osd time thread create failed (%d)\r\n", ret);
        return ret;
    }

    (void)test_venc_osd_time_extent(text, &osd_w, &osd_h);
    tkl_log_output("osd time start: %s @(%u,%u) %ux%u slot=%u font=Roboto53\r\n",
                   text,
                   (unsigned int)VENC_TEST_OSD_TIME_X,
                   (unsigned int)VENC_TEST_OSD_TIME_Y,
                   (unsigned int)osd_w,
                   (unsigned int)osd_h,
                   (unsigned int)VENC_TEST_OSD_TIME_INDEX);
    return OPRT_OK;
}

static OPERATE_RET test_venc_set_bitrate(uint32_t bitrate, bool force_idr)
{
    TUYA_VENC_RATE_CTRL_T rc;
    OPERATE_RET ret;

    if (s_venc_ctx.initialized != true || s_venc_ctx.handle == NULL) {
        tkl_log_output("venc is not initialized\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_venc_ctx.codec != VENC_TEST_CODEC_H264) {
        tkl_log_output("bitrate is only supported for h264\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    if (bitrate == 0U) {
        tkl_log_output("bitrate must be > 0\r\n");
        return OPRT_INVALID_PARM;
    }

    os_memset(&rc, 0, sizeof(rc));
    rc.bitrate = bitrate;
    rc.force_idr = force_idr ? TRUE : FALSE;
    ret = tkl_venc_set_rate_ctrl(s_venc_ctx.handle, &rc);
    if (ret != OPRT_OK) {
        return ret;
    }

    s_venc_ctx.bitrate = bitrate;
    tkl_log_output("venc bitrate set to %u bps%s\r\n",
            (unsigned int)bitrate,
            force_idr ? ", idr requested" : "");
    return OPRT_OK;
}

static void test_venc_status_dump(void)
{
    uint64_t now_ms = (uint64_t)tkl_system_get_millisecond();
    uint32_t session_elapsed_ms = 0U;
    uint32_t window_elapsed_ms = 0U;
    uint64_t window_bytes = 0U;
    uint32_t window_frames = 0U;
    uint32_t avg_kbps = 0U;
    uint32_t window_kbps = 0U;
    uint32_t avg_fps = 0U;
    uint32_t window_fps = 0U;

    tkl_log_output("xvenc status:\r\n");
    tkl_log_output("  venc_init=%u venc_start=%u codec=%s source=%s camera_init=%u camera_start=%u camera_flexa=%u\r\n",
            (unsigned int)s_venc_ctx.initialized,
            (unsigned int)s_venc_ctx.started,
            test_venc_codec_name(s_venc_ctx.codec),
            test_venc_source_name(s_venc_ctx.source),
            (unsigned int)test_mipi_camera_is_inited(),
            (unsigned int)test_mipi_camera_is_started(),
            (unsigned int)test_mipi_camera_flexa_enabled());
    if (s_venc_ctx.codec == VENC_TEST_CODEC_JPEG) {
        tkl_log_output("  src=%ux%u fmt=%u fps=%u quality=%u\r\n",
                s_venc_ctx.width,
                s_venc_ctx.height,
                (unsigned int)s_venc_ctx.format,
                (unsigned int)s_venc_ctx.fps,
                (unsigned int)s_venc_ctx.quality);
    } else {
        tkl_log_output("  src=%ux%u fmt=%u fps=%u gop=%u target_bitrate=%u\r\n",
                s_venc_ctx.width,
                s_venc_ctx.height,
                (unsigned int)s_venc_ctx.format,
                (unsigned int)s_venc_ctx.fps,
                (unsigned int)s_venc_ctx.gop,
                (unsigned int)s_venc_ctx.bitrate);
    }
    tkl_log_output("  frames=%u i=%u p=%u hdr=%u unk=%u last_type=%s last_size=%u total_bytes=%llu\r\n",
            (unsigned int)s_venc_ctx.total_frames,
            (unsigned int)s_venc_ctx.i_frames,
            (unsigned int)s_venc_ctx.p_frames,
            (unsigned int)s_venc_ctx.header_frames,
            (unsigned int)s_venc_ctx.unknown_frames,
            test_venc_frame_type_name(s_venc_ctx.last_frame_type),
            (unsigned int)s_venc_ctx.last_frame_size,
            (unsigned long long)s_venc_ctx.total_bytes);

    if (s_venc_ctx.codec != VENC_TEST_CODEC_H264 ||
        s_venc_ctx.session_timing_valid != true ||
        s_venc_ctx.started != true) {
        return;
    }

    session_elapsed_ms = test_venc_elapsed_ms(s_venc_ctx.session_start_ms, now_ms);
    window_elapsed_ms = test_venc_elapsed_ms(s_venc_ctx.status_mark_ms, now_ms);
    if (s_venc_ctx.total_bytes >= s_venc_ctx.status_mark_bytes) {
        window_bytes = s_venc_ctx.total_bytes - s_venc_ctx.status_mark_bytes;
    }
    if (s_venc_ctx.total_frames >= s_venc_ctx.status_mark_frames) {
        window_frames = s_venc_ctx.total_frames - s_venc_ctx.status_mark_frames;
    }

    avg_kbps = test_venc_calc_kbps(s_venc_ctx.total_bytes, session_elapsed_ms);
    window_kbps = test_venc_calc_kbps(window_bytes, window_elapsed_ms);
    avg_fps = test_venc_calc_fps(s_venc_ctx.total_frames, session_elapsed_ms);
    window_fps = test_venc_calc_fps(window_frames, window_elapsed_ms);

    tkl_log_output("  avg_bitrate=%u kbps avg_fps=%u session_ms=%u\r\n",
            avg_kbps,
            avg_fps,
            session_elapsed_ms);
    tkl_log_output("  window_bitrate=%u kbps window_fps=%u window_ms=%u\r\n",
            window_kbps,
            window_fps,
            window_elapsed_ms);

    s_venc_ctx.status_mark_ms = now_ms;
    s_venc_ctx.status_mark_bytes = s_venc_ctx.total_bytes;
    s_venc_ctx.status_mark_frames = s_venc_ctx.total_frames;
}

static OPERATE_RET test_venc_jpegsw_write_file(const char *path, const UINT8_T *data, UINT32_T size)
{
    TUYA_FILE file = NULL;
    INT_T write_len;

    if (path == NULL || data == NULL || size == 0U) {
        return OPRT_INVALID_PARM;
    }

    file = tkl_fopen(path, "wb");
    if (file == NULL) {
        tkl_log_output("venc jpegsw open failed: %s\r\n", path);
        return OPRT_COM_ERROR;
    }

    write_len = tkl_fwrite((VOID_T *)data, (INT_T)size, file);
    (void)tkl_fsync((INT_T)file);
    (void)tkl_fclose(file);
    if (write_len != (INT_T)size) {
        tkl_log_output("venc jpegsw write failed: expect=%u actual=%d\r\n",
                       (unsigned int)size,
                       write_len);
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

static OPERATE_RET test_venc_jpegsw_capture_sp_nv12(UINT8_T **nv12_buf,
                                                    UINT16_T *width,
                                                    UINT16_T *height)
{
    OPERATE_RET ret;
    UINT16_T sp_w = 0U;
    UINT16_T sp_h = 0U;
    UINT32_T format = 0U;
    UINT32_T timeout_ms = 0U;
    UINT32_T nv12_size = 0U;
    UINT8_T *buffer = NULL;
    TUYA_CAMERA_MIPI_FRAME_T frame;

    if (nv12_buf == NULL || width == NULL || height == NULL) {
        return OPRT_INVALID_PARM;
    }

    *nv12_buf = NULL;
    *width = 0U;
    *height = 0U;

    ret = test_mipi_camera_get_sp_info(&sp_w, &sp_h, &format, &timeout_ms);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (format != (UINT32_T)BK_PIXEL_FORMAT_NV12) {
        return OPRT_NOT_SUPPORTED;
    }

    nv12_size = (UINT32_T)sp_w * (UINT32_T)sp_h * 3U / 2U;
    buffer = (UINT8_T *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, nv12_size);
    if (buffer == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    os_memset(&frame, 0, sizeof(frame));
    frame.buffer = buffer;
    frame.size = nv12_size;
    frame.timeout_ms = timeout_ms;
    frame.channel_id = TUYA_CAMERA_MIPI_CHANNEL_SP;

    ret = tkl_camera_mipi_get_frame(&frame);
    if (ret != OPRT_OK) {
        bk_frame_buffer_free(buffer);
        tkl_log_output("venc jpegsw sp get_frame failed: %d\r\n", ret);
        return ret;
    }

    *nv12_buf = buffer;
    *width = frame.width;
    *height = frame.height;
    return OPRT_OK;
}

/**
 * @brief Dump SP NV12 and soft-encode YUYV raw buffers to SD card
 * @param[in] prefix file prefix under /sdcard (no extension)
 * @return OPRT_OK on success
 */
static OPERATE_RET test_venc_dump_sp(const char *prefix)
{
    OPERATE_RET ret;
    UINT8_T *nv12_buf = NULL;
    UINT8_T *yuyv_buf = NULL;
    UINT16_T width = 0U;
    UINT16_T height = 0U;
    UINT32_T nv12_size = 0U;
    UINT32_T yuyv_size = 0U;
    char nv12_path[VENC_TEST_RECORD_PATH_MAX];
    char yuyv_path[VENC_TEST_RECORD_PATH_MAX];
    char info_path[VENC_TEST_RECORD_PATH_MAX];
    char info[384];
    int info_len;

    if (prefix == NULL || prefix[0] == '\0') {
        prefix = VENC_TEST_DUMP_SP_DEFAULT_PREFIX;
    }

    if (!test_venc_record_name_valid(prefix)) {
        tkl_log_output("venc dump_sp invalid prefix: %s\r\n", prefix);
        return OPRT_INVALID_PARM;
    }

    if (test_mipi_camera_is_inited() != TRUE || test_mipi_camera_is_started() != TRUE) {
        tkl_log_output("venc dump_sp requires camera, use: xmipi camera open_gpu_sp ... nv12 && xmipi camera start\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = test_venc_jpegsw_capture_sp_nv12(&nv12_buf, &width, &height);
    if (ret != OPRT_OK) {
        tkl_log_output("venc dump_sp capture NV12 failed: %d\r\n", ret);
        return ret;
    }

    nv12_size = (UINT32_T)width * (UINT32_T)height * 3U / 2U;
    yuyv_size = (UINT32_T)width * (UINT32_T)height * 2U;
    yuyv_buf = (UINT8_T *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, yuyv_size);
    if (yuyv_buf == NULL) {
        bk_frame_buffer_free(nv12_buf);
        return OPRT_MALLOC_FAILED;
    }

    ret = tkl_venc_nv12_to_yuyv(width, height, nv12_buf, nv12_size, yuyv_buf, yuyv_size);
    if (ret != OPRT_OK) {
        tkl_log_output("venc dump_sp nv12_to_yuyv failed: %d\r\n", ret);
        bk_frame_buffer_free(yuyv_buf);
        bk_frame_buffer_free(nv12_buf);
        return ret;
    }

    if (test_fs_mount(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD) != 0) {
        tkl_log_output("venc dump_sp mount %s failed\r\n", TEST_SDCARD_MOUNT_POINT);
        bk_frame_buffer_free(yuyv_buf);
        bk_frame_buffer_free(nv12_buf);
        return OPRT_COM_ERROR;
    }

    ret = test_venc_record_build_path(prefix, nv12_path, sizeof(nv12_path));
    if (ret != OPRT_OK) {
        bk_frame_buffer_free(yuyv_buf);
        bk_frame_buffer_free(nv12_buf);
        return ret;
    }

    /* rebuild with extensions */
    if (os_snprintf(nv12_path, sizeof(nv12_path), "%s/%s.nv12",
                    TEST_SDCARD_MOUNT_POINT, prefix) <= 0) {
        bk_frame_buffer_free(yuyv_buf);
        bk_frame_buffer_free(nv12_buf);
        return OPRT_COM_ERROR;
    }
    if (os_snprintf(yuyv_path, sizeof(yuyv_path), "%s/%s.yuyv",
                    TEST_SDCARD_MOUNT_POINT, prefix) <= 0) {
        bk_frame_buffer_free(yuyv_buf);
        bk_frame_buffer_free(nv12_buf);
        return OPRT_COM_ERROR;
    }
    if (os_snprintf(info_path, sizeof(info_path), "%s/%s.txt",
                    TEST_SDCARD_MOUNT_POINT, prefix) <= 0) {
        bk_frame_buffer_free(yuyv_buf);
        bk_frame_buffer_free(nv12_buf);
        return OPRT_COM_ERROR;
    }

    ret = test_venc_jpegsw_write_file(nv12_path, nv12_buf, nv12_size);
    if (ret != OPRT_OK) {
        bk_frame_buffer_free(yuyv_buf);
        bk_frame_buffer_free(nv12_buf);
        return ret;
    }

    ret = test_venc_jpegsw_write_file(yuyv_path, yuyv_buf, yuyv_size);
    if (ret != OPRT_OK) {
        bk_frame_buffer_free(yuyv_buf);
        bk_frame_buffer_free(nv12_buf);
        return ret;
    }

    info_len = os_snprintf(info, sizeof(info),
                           "width=%u\nheight=%u\nnv12_size=%u\nyuyv_size=%u\n"
                           "nv12=tight packed stride=width\n"
                           "yuyv=soft-jpeg packing (Y0 V Y1 U / YVYU-like)\n"
                           "ffplay -f rawvideo -pixel_format nv12 -video_size %ux%u %s.nv12\n"
                           "ffplay -f rawvideo -pixel_format yvyu422 -video_size %ux%u %s.yuyv\n",
                           (unsigned int)width,
                           (unsigned int)height,
                           (unsigned int)nv12_size,
                           (unsigned int)yuyv_size,
                           (unsigned int)width,
                           (unsigned int)height,
                           prefix,
                           (unsigned int)width,
                           (unsigned int)height,
                           prefix);
    if (info_len > 0) {
        (void)test_venc_jpegsw_write_file(info_path, (const UINT8_T *)info, (UINT32_T)info_len);
    }

    tkl_log_output("venc dump_sp ok: %ux%u nv12=%s (%u) yuyv=%s (%u) info=%s\r\n",
                   width,
                   height,
                   nv12_path,
                   (unsigned int)nv12_size,
                   yuyv_path,
                   (unsigned int)yuyv_size,
                   info_path);

    bk_frame_buffer_free(yuyv_buf);
    bk_frame_buffer_free(nv12_buf);
    return OPRT_OK;
}

/**
 * @brief Resolve JPEG SW output buffer size for capture resolution
 * @param[in] width frame width
 * @param[in] height frame height
 * @return buffer size in bytes, clamped to [MIN, MAX]
 * @note ~1 byte/pixel gives headroom for high quality (e.g. 1024x600 Q100).
 */
static uint32_t test_venc_jpegsw_outbuf_size(uint16_t width, uint16_t height)
{
    uint32_t need = (uint32_t)width * (uint32_t)height;

    if (need < VENC_TEST_JPEGSW_OUTBUF_MIN) {
        need = VENC_TEST_JPEGSW_OUTBUF_MIN;
    }
    if (need > VENC_TEST_JPEGSW_OUTBUF_MAX) {
        need = VENC_TEST_JPEGSW_OUTBUF_MAX;
    }
    return need;
}

static OPERATE_RET test_venc_jpegsw(uint8_t quality, const char *filename)
{
    OPERATE_RET ret;
    TUYA_VENC_JPEG_SW_IN_T enc_in;
    TUYA_VENC_JPEG_SW_OUT_T enc_out;
    TUYA_CAMERA_MIPI_FRAME_T cam_frame;
    UINT8_T *jpeg_buf = NULL;
    UINT8_T *pixel_buf = NULL;
    UINT16_T capt_w = 0U;
    UINT16_T capt_h = 0U;
    uint32_t jpeg_outbuf_size = 0U;
    char path[VENC_TEST_RECORD_PATH_MAX];
    bool from_sp = false;

    if (test_mipi_camera_is_inited() != TRUE || test_mipi_camera_is_started() != TRUE) {
        tkl_log_output("venc jpegsw requires camera, use: xmipi camera open_gpu_sp ... nv12 && xmipi camera start\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    os_memset(&cam_frame, 0, sizeof(cam_frame));
    ret = test_venc_jpegsw_capture_sp_nv12(&pixel_buf, &capt_w, &capt_h);
    if (ret == OPRT_OK) {
        from_sp = true;
        cam_frame.buffer = pixel_buf;
        cam_frame.out_len = (UINT32_T)capt_w * (UINT32_T)capt_h * 3U / 2U;
        cam_frame.width = capt_w;
        cam_frame.height = capt_h;
        cam_frame.format = (UINT32_T)BK_PIXEL_FORMAT_NV12;
    } else if (ret == OPRT_NOT_SUPPORTED || ret == OPRT_RESOURCE_NOT_READY) {
        if (test_mipi_camera_flexa_enabled() == TRUE) {
            tkl_log_output("venc jpegsw needs SP NV12 frame mode, try: xmipi camera open_gpu_sp ... nv12\r\n");
            return OPRT_NOT_SUPPORTED;
        }

        ret = test_mipi_camera_capture_frame(&cam_frame);
        if (ret != OPRT_OK) {
            tkl_log_output("venc jpegsw mp get_frame failed: %d\r\n", ret);
            return ret;
        }
    } else {
        tkl_log_output("venc jpegsw sp capture failed: %d\r\n", ret);
        return ret;
    }

    ret = test_venc_record_build_path(
            (filename != NULL) ? filename : VENC_TEST_JPEGSW_DEFAULT_FILE,
            path,
            sizeof(path));
    if (ret != OPRT_OK) {
        tkl_log_output("venc jpegsw invalid filename: %s\r\n",
                         (filename != NULL) ? filename : VENC_TEST_JPEGSW_DEFAULT_FILE);
        if (pixel_buf != NULL) {
            bk_frame_buffer_free(pixel_buf);
        }
        return ret;
    }

    if (test_fs_mount(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD) != 0) {
        tkl_log_output("venc jpegsw mount %s failed\r\n", TEST_SDCARD_MOUNT_POINT);
        if (pixel_buf != NULL) {
            bk_frame_buffer_free(pixel_buf);
        }
        return OPRT_COM_ERROR;
    }

    if (cam_frame.format != (UINT32_T)BK_PIXEL_FORMAT_NV12 &&
        cam_frame.format != (UINT32_T)BK_PIXEL_FORMAT_YUYV) {
        tkl_log_output("venc jpegsw unsupported format=%u, need NV12 or YUYV\r\n",
                       (unsigned int)cam_frame.format);
        if (pixel_buf != NULL) {
            bk_frame_buffer_free(pixel_buf);
        }
        return OPRT_NOT_SUPPORTED;
    }

    jpeg_outbuf_size = test_venc_jpegsw_outbuf_size(cam_frame.width, cam_frame.height);
    jpeg_buf = (UINT8_T *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_CODED, jpeg_outbuf_size);
    if (jpeg_buf == NULL) {
        if (pixel_buf != NULL) {
            bk_frame_buffer_free(pixel_buf);
        }
        return OPRT_MALLOC_FAILED;
    }

    os_memset(&enc_in, 0, sizeof(enc_in));
    enc_in.buffer = cam_frame.buffer;
    enc_in.size = cam_frame.out_len;
    enc_in.width = cam_frame.width;
    enc_in.height = cam_frame.height;
    enc_in.format = cam_frame.format;
    enc_in.quality = quality;

    os_memset(&enc_out, 0, sizeof(enc_out));
    enc_out.buffer = jpeg_buf;
    enc_out.buf_size = jpeg_outbuf_size;

    ret = tkl_venc_jpeg_sw_encode(&enc_in, &enc_out);
    if (pixel_buf != NULL) {
        bk_frame_buffer_free(pixel_buf);
        pixel_buf = NULL;
    }
    if (ret != OPRT_OK) {
        tkl_log_output("venc jpegsw encode failed: %d\r\n", ret);
        bk_frame_buffer_free(jpeg_buf);
        return ret;
    }

    ret = test_venc_jpegsw_write_file(path, enc_out.buffer, enc_out.size);
    bk_frame_buffer_free(jpeg_buf);
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_log_output("venc jpegsw ok: ch=%s %ux%u fmt=%u jpeg=%u bytes -> %s\r\n",
                   from_sp ? "SP" : "MP",
                   cam_frame.width,
                   cam_frame.height,
                   (unsigned int)cam_frame.format,
                   (unsigned int)enc_out.size,
                   path);
    return OPRT_OK;
}

static test_venc_codec_t test_venc_parse_codec(const char *token)
{
    if (token == NULL) {
        return VENC_TEST_CODEC_H264;
    }

    if (os_strcmp(token, "jpeg") == 0 || os_strcmp(token, "mjpeg") == 0) {
        return VENC_TEST_CODEC_JPEG;
    }

    return VENC_TEST_CODEC_H264;
}

static test_venc_source_t test_venc_parse_source(const char *token)
{
    if (token != NULL && os_strcmp(token, "vdec") == 0) {
        return VENC_TEST_SOURCE_VDEC;
    }

    return VENC_TEST_SOURCE_CAMERA;
}

void cli_tuya_venc_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    const char *record_file = VENC_TEST_RECORD_DEFAULT_FILE;
    uint8_t fps = VENC_TEST_DEFAULT_FPS;
    uint32_t gop = VENC_TEST_DEFAULT_GOP;
    uint32_t record_sec = VENC_TEST_RECORD_DEFAULT_SEC;
    uint8_t quality = VENC_TEST_DEFAULT_QUALITY;
    uint32_t bitrate = VENC_TEST_DEFAULT_BITRATE;
    test_venc_source_t source_type = VENC_TEST_SOURCE_CAMERA;
    test_venc_codec_t codec_type = VENC_TEST_CODEC_H264;
    int arg_idx = 2;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xvenc <init [camera|vdec] [h264|jpeg] [fps] [gop|quality] [bitrate]|start|stop|bitrate <bps> [idr]|record [sec] [file]|snapshot [file]|dump_sp [prefix]|jpegsw [quality] [file]|deinit|close|idr|osd|status>");
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "init") == 0) {
        if (argc > arg_idx) {
            if (os_strcmp(argv[arg_idx], "camera") == 0 ||
                os_strcmp(argv[arg_idx], "vdec") == 0) {
                source_type = test_venc_parse_source(argv[arg_idx]);
                arg_idx++;
            }
        }
        if (argc > arg_idx) {
            if (os_strcmp(argv[arg_idx], "h264") == 0 ||
                os_strcmp(argv[arg_idx], "jpeg") == 0 ||
                os_strcmp(argv[arg_idx], "mjpeg") == 0) {
                codec_type = test_venc_parse_codec(argv[arg_idx]);
                arg_idx++;
            }
        }
        if (argc > arg_idx) {
            fps = (uint8_t)os_strtoul(argv[arg_idx], NULL, 10);
            arg_idx++;
        }
        if (argc > arg_idx) {
            if (codec_type == VENC_TEST_CODEC_JPEG) {
                quality = (uint8_t)os_strtoul(argv[arg_idx], NULL, 10);
            } else {
                gop = os_strtoul(argv[arg_idx], NULL, 10);
            }
            arg_idx++;
        }
        if (argc > arg_idx && codec_type == VENC_TEST_CODEC_H264) {
            bitrate = os_strtoul(argv[arg_idx], NULL, 10);
        }

        ret = test_venc_init(source_type, codec_type, fps, gop, quality, bitrate);
        if (codec_type == VENC_TEST_CODEC_JPEG) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xvenc init %s (%d), source=%s codec=jpeg fps=%u quality=%u\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     test_venc_source_name(source_type),
                     (unsigned int)fps,
                     (unsigned int)quality);
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xvenc init %s (%d), source=%s codec=h264 fps=%u gop=%u bitrate=%u\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     test_venc_source_name(source_type),
                     (unsigned int)fps,
                     (unsigned int)gop,
                     (unsigned int)bitrate);
        }
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
    } else if (os_strcmp(cmd, "bitrate") == 0) {
        bool force_idr = false;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xvenc bitrate <bps> [idr]\r\n");
            return;
        }

        bitrate = os_strtoul(argv[2], NULL, 10);
        if (argc >= 4 && os_strcmp(argv[3], "idr") == 0) {
            force_idr = true;
        }

        ret = test_venc_set_bitrate(bitrate, force_idr);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc bitrate %s (%d), bps=%u%s\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)bitrate,
                 force_idr ? " idr=1" : "");
    } else if (os_strcmp(cmd, "record") == 0) {
        if (argc >= 3) {
            record_sec = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            record_file = argv[3];
        } else if (s_venc_ctx.codec == VENC_TEST_CODEC_JPEG) {
            record_file = VENC_TEST_RECORD_DEFAULT_JPEG_FILE;
        }

        ret = test_venc_record(record_sec, record_file);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc record %s (%d), sec=%u file=%s/%s codec=%s\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)((record_sec == 0U) ? VENC_TEST_RECORD_DEFAULT_SEC : record_sec),
                 TEST_SDCARD_MOUNT_POINT,
                 record_file,
                 test_venc_codec_name(s_venc_ctx.codec));
    } else if (os_strcmp(cmd, "snapshot") == 0 || os_strcmp(cmd, "snap") == 0) {
        const char *snap_file = VENC_TEST_SNAPSHOT_DEFAULT_FILE;

        if (argc >= 3) {
            snap_file = argv[2];
        }

        ret = test_venc_snapshot(snap_file);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc snapshot %s (%d), file=%s/%s\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 TEST_SDCARD_MOUNT_POINT,
                 snap_file);
    } else if (os_strcmp(cmd, "dump_sp") == 0 || os_strcmp(cmd, "dump_yuv") == 0) {
        const char *dump_prefix = VENC_TEST_DUMP_SP_DEFAULT_PREFIX;

        if (argc >= 3) {
            dump_prefix = argv[2];
        }

        ret = test_venc_dump_sp(dump_prefix);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc dump_sp %s (%d), prefix=%s files=%s/{%s.nv12,%s.yuyv,%s.txt}\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 dump_prefix,
                 TEST_SDCARD_MOUNT_POINT,
                 dump_prefix,
                 dump_prefix,
                 dump_prefix);
    } else if (os_strcmp(cmd, "jpegsw") == 0) {
        const char *jpeg_file = VENC_TEST_JPEGSW_DEFAULT_FILE;
        uint8_t jpeg_quality = VENC_TEST_JPEGSW_DEFAULT_QUALITY;

        if (argc >= 3) {
            jpeg_quality = (uint8_t)os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            jpeg_file = argv[3];
        }

        ret = test_venc_jpegsw(jpeg_quality, jpeg_file);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc jpegsw %s (%d), quality=%u file=%s/%s\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)jpeg_quality,
                 TEST_SDCARD_MOUNT_POINT,
                 jpeg_file);
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
    } else if (os_strcmp(cmd, "osd") == 0) {
        const char *osd_sub = NULL;
        uint32_t osd_index = VENC_TEST_OSD_TIME_INDEX;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xvenc osd <time|clear [idx]|stop>\r\n");
            return;
        }

        osd_sub = argv[2];
        if (os_strcmp(osd_sub, "time") == 0) {
            ret = test_venc_osd_time_start();
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xvenc osd time %s (%d), text=%s @(%u,%u)\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     s_venc_osd_time.last_text,
                     (unsigned int)VENC_TEST_OSD_TIME_X,
                     (unsigned int)VENC_TEST_OSD_TIME_Y);
        } else if (os_strcmp(osd_sub, "stop") == 0) {
            ret = test_venc_osd_clear(VENC_TEST_OSD_TIME_INDEX);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xvenc osd stop %s (%d)\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret);
        } else if (os_strcmp(osd_sub, "clear") == 0) {
            if (argc >= 4) {
                osd_index = os_strtoul(argv[3], NULL, 10);
            }
            ret = test_venc_osd_clear(osd_index);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xvenc osd clear %s (%d), idx=%u\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed",
                     ret,
                     (unsigned int)osd_index);
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xvenc osd <time|clear [idx]|stop>\r\n");
        }
    } else if (os_strcmp(cmd, "status") == 0) {
        test_venc_status_dump();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xvenc status ok, see logs\r\n");
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xvenc <init [camera|vdec] [h264|jpeg] [fps] [gop|quality] [bitrate]|start|stop|bitrate <bps> [idr]|record [sec] [file]|snapshot [file]|dump_sp [prefix]|jpegsw [quality] [file]|deinit|close|idr|osd|status>");
    }
}
