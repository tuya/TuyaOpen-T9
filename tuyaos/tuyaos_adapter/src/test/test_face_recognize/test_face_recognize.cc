extern "C" {
#include "cli.h"
#include "os/os.h"

#include <common/bk_err.h>
#include <components/bk_frame_buffer.h>
#include <common/avdk_pixel_types.h>

#include "tkl_camera_mipi.h"
#include "tkl_media_fmt.h"
#include "tuya_cloud_types.h"

extern BOOL_T test_mipi_lcd_is_open(void);
extern void test_mipi_lcd_get_geometry(UINT16_T *width, UINT16_T *height);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "YolofaceDetectionModel.h"
#include "box.h"

#define FACE_TEST_TASK_PRIORITY     (BEKEN_DEFAULT_WORKER_PRIORITY - 1)
#define FACE_TEST_TASK_STACK_SIZE   (20 * 1024)
#define FACE_TEST_WIDTH             56U
#define FACE_TEST_HEIGHT            56U
#define FACE_TEST_BYTES_PER_PIXEL   4U
#define FACE_TEST_FRAME_SIZE        (FACE_TEST_WIDTH * FACE_TEST_HEIGHT * FACE_TEST_BYTES_PER_PIXEL)
#define FACE_TEST_LOOP_FOREVER      (0xFFFFFFFFU)
#define FACE_TEST_DEFAULT_DELAY_MS  (200U)

static beken_thread_t s_face_thread = NULL;
static volatile int s_face_stop = 0;
static uint32_t s_face_rounds = 1;
static uint32_t s_face_delay_ms = FACE_TEST_DEFAULT_DELAY_MS;
static YolofaceDetectionModel *s_face_model = NULL;
static bool s_face_model_ready = false;

static void face_test_box_cb(Box *boxes, int count)
{
    UINT16_T lcd_width = 0U;
    UINT16_T lcd_height = 0U;

    if (boxes == NULL || count <= 0) {
        box_detection_path_clear();
        return;
    }

    bk_printf("[xface] box count=%d best=score=%.3f x=%.1f y=%.1f w=%.1f h=%.1f\r\n",
              count,
              boxes[0].score,
              boxes[0].x,
              boxes[0].y,
              boxes[0].w,
              boxes[0].h);

    if (s_face_model == NULL) {
        return;
    }

    if (test_mipi_lcd_is_open() != TRUE) {
        bk_printf("[xface] lcd not open, skip box draw\r\n");
        return;
    }

    test_mipi_lcd_get_geometry(&lcd_width, &lcd_height);
    if (lcd_width == 0U || lcd_height == 0U) {
        bk_printf("[xface] invalid lcd geometry %ux%u, skip box draw\r\n",
                  (unsigned int)lcd_width, (unsigned int)lcd_height);
        return;
    }

    box_detection_path_build(boxes,
                             count,
                             count,
                             0,
                             s_face_model->getWidth(),
                             s_face_model->getHeight(),
                             (int)lcd_width,
                             (int)lcd_height);
}

static void face_test_usage(void)
{
    bk_printf("xface help\r\n");
    bk_printf("xface once\r\n");
    bk_printf("xface start [delay_ms]\r\n");
    bk_printf("xface stop\r\n");
    bk_printf("xface release\r\n");
    bk_printf("xface status\r\n");
}

static uint32_t face_test_parse_u32(const char *str, uint32_t fallback)
{
    char *end = NULL;
    unsigned long value;

    if ((str == NULL) || (str[0] == '\0')) {
        return fallback;
    }

    value = strtoul(str, &end, 10);
    if ((end == str) || ((end != NULL) && (*end != '\0'))) {
        bk_printf("[xface] invalid number: %s, use %u\r\n", str, (unsigned int)fallback);
        return fallback;
    }

    return (uint32_t)value;
}

/**
 * @brief Borrow one SP frame. Release with frame->free_cb(frame->data).
 * @param[out] frame receives the frame on success; untouched on failure
 * @return BK_OK on success
 */
static bk_err_t face_test_read_sp_frame(TKL_MEDIA_BUF_T *frame)
{
    OPERATE_RET ret;

    if (frame == NULL) {
        return BK_ERR_PARAM;
    }

    /* The camera owns the buffer and lends it; nothing to size or allocate here. */
    ret = tkl_camera_mipi_get_frame(frame, TUYA_CAMERA_MIPI_CHANNEL_SP, 1000U);
    if (ret != OPRT_OK) {
        bk_printf("[xface] get SP frame failed, ret=%d\r\n", ret);
        return BK_FAIL;
    }

    if ((frame->width != FACE_TEST_WIDTH) ||
        (frame->height != FACE_TEST_HEIGHT) ||
        (frame->size < FACE_TEST_FRAME_SIZE)) {
        bk_printf("[xface] unexpected SP frame: %ux%u fmt=%u len=%u\r\n",
                  frame->width,
                  frame->height,
                  (unsigned int)frame->format,
                  (unsigned int)frame->size);
        frame->free_cb(frame->data);
        return BK_FAIL;
    }

    return BK_OK;
}

static bk_pixel_format_t face_test_get_model_input_format(const TKL_MEDIA_BUF_T *frame)
{
    bk_pixel_format_t bk_fmt;

    if (frame == NULL) {
        return BK_PIXEL_FORMAT_UNKNOW;
    }

    bk_fmt = (bk_pixel_format_t)tkl_media_fmt_to_bk(frame->format);

    if ((bk_fmt == BK_PIXEL_FORMAT_RGB888 || bk_fmt == BK_PIXEL_FORMAT_BGR888) &&
        frame->size == FACE_TEST_FRAME_SIZE) {
        return BK_PIXEL_FORMAT_BGRA8888;
    }

    return bk_fmt;
}

static void face_test_model_release(void)
{
    if (s_face_model != NULL) {
        if (s_face_model_ready) {
            s_face_model->deinit();
        }
        s_face_model->LogEnable(false);
        delete s_face_model;
        s_face_model = NULL;
        s_face_model_ready = false;
    }

}

static bk_err_t face_test_model_prepare(void)
{
    int ret;

    if (s_face_model == NULL) {
        s_face_model = new YolofaceDetectionModel();
        if (s_face_model == NULL) {
            bk_printf("[xface] model alloc failed\r\n");
            face_test_model_release();
            return BK_ERR_NO_MEM;
        }
        s_face_model->setBoxDetectionCallback(face_test_box_cb);
    }

    if (s_face_model_ready) {
        return BK_OK;
    }

    s_face_model->LogEnable(true);
    ret = s_face_model->init();
    if (ret != BK_OK) {
        bk_printf("[xface] model init failed, ret=%d\r\n", ret);
        face_test_model_release();
        return BK_FAIL;
    }

    s_face_model_ready = true;
    bk_printf("[xface] model initialized and kept resident\r\n");
    return BK_OK;
}

static void face_test_task(void *arg)
{
    TKL_MEDIA_BUF_T frame;
    uint32_t rounds = s_face_rounds;
    uint32_t delay_ms = s_face_delay_ms;
    uint32_t index = 0;
    int ret;

    (void)arg;

    bk_printf("[xface] task start, rounds=%u delay=%ums\r\n",
              (unsigned int)rounds, (unsigned int)delay_ms);

    if (face_test_model_prepare() != BK_OK) {
        goto exit;
    }

    while (!s_face_stop && ((rounds == FACE_TEST_LOOP_FOREVER) || (index < rounds))) {
        bk_printf("[xface] round %u start\r\n", (unsigned int)(index + 1));

        memset(&frame, 0, sizeof(frame));

        if (face_test_read_sp_frame(&frame) == BK_OK) {
            ret = s_face_model->run(frame.data,
                                    frame.size,
                                    face_test_get_model_input_format(&frame));
            bk_printf("[xface] round %u done, face=%d\r\n",
                      (unsigned int)(index + 1), ret);
            frame.free_cb(frame.data);
        }

        index++;
        if (!s_face_stop && (rounds == FACE_TEST_LOOP_FOREVER) && delay_ms > 0U) {
            rtos_delay_milliseconds(delay_ms);
        }
    }

exit:
    bk_printf("[xface] task exit, stop=%d rounds_done=%u model_ready=%d\r\n",
              s_face_stop, (unsigned int)index, s_face_model_ready);

    s_face_thread = NULL;
    s_face_stop = 0;
    rtos_delete_thread(NULL);
}

static void face_test_start(uint32_t rounds, uint32_t delay_ms)
{
    bk_err_t ret;

    if (s_face_thread != NULL) {
        bk_printf("[xface] task already running\r\n");
        return;
    }

    s_face_stop = 0;
    s_face_rounds = rounds;
    s_face_delay_ms = delay_ms;

    ret = rtos_create_hsram_thread(&s_face_thread,
                                   FACE_TEST_TASK_PRIORITY,
                                   "xface_demo",
                                   (beken_thread_function_t)face_test_task,
                                   FACE_TEST_TASK_STACK_SIZE,
                                   NULL);
    if (ret != BK_OK) {
        s_face_thread = NULL;
        bk_printf("[xface] create task failed, ret=%d\r\n", ret);
        return;
    }

    bk_printf("[xface] task created\r\n");
}

extern "C" void cli_tuya_face_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    if ((argc < 2) || (argv[1] == NULL) ||
        (strcmp(argv[1], "help") == 0) ||
        (strcmp(argv[1], "?") == 0)) {
        face_test_usage();
        return;
    }

    if (strcmp(argv[1], "once") == 0) {
        face_test_start(1, 0);
        return;
    }

    if (strcmp(argv[1], "start") == 0) {
        uint32_t delay_ms = face_test_parse_u32((argc >= 3) ? argv[2] : NULL,
                                                FACE_TEST_DEFAULT_DELAY_MS);
        face_test_start(FACE_TEST_LOOP_FOREVER, delay_ms);
        return;
    }

    if (strcmp(argv[1], "stop") == 0) {
        if (s_face_thread == NULL) {
            bk_printf("[xface] task not running\r\n");
            return;
        }
        s_face_stop = 1;
        bk_printf("[xface] stop requested\r\n");
        return;
    }

    if (strcmp(argv[1], "release") == 0) {
        if (s_face_thread != NULL) {
            bk_printf("[xface] stop task before release\r\n");
            return;
        }
        face_test_model_release();
        bk_printf("[xface] model released\r\n");
        return;
    }

    if (strcmp(argv[1], "status") == 0) {
        bk_printf("[xface] running=%d stop=%d rounds=%u delay=%ums model_ready=%d\r\n",
                  (s_face_thread != NULL),
                  s_face_stop,
                  (unsigned int)s_face_rounds,
                  (unsigned int)s_face_delay_ms,
                  s_face_model_ready);
        return;
    }

    bk_printf("[xface] unknown command: %s\r\n", argv[1]);
    face_test_usage();
}
