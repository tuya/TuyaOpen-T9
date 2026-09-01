/**
 * @file test_face_recog.cc
 * @brief CLI demo for face recognition + SD gallery (xrecog)
 * @version 1.0
 * @date 2026-08-25
 * @copyright Copyright (c) Tuya Inc.
 *
 * Prerequisite (other tests):
 *   xsd mount
 *   xmipi cam open frame 640 1024 rgb888 320 320   (SP must be exactly 320x320)
 *   xmipi dsi open_gpu <panel> 1                   (only if a preview is wanted)
 *
 * Models on SD:
 *   /sdcard/tflite/face_detection_int8_vela.tflite
 *   /sdcard/tflite/face_verify_int8_vela.tflite
 */
extern "C" {
#include "cli.h"
#include "os/os.h"

#include <common/bk_err.h>
#include <components/bk_frame_buffer.h>
#include <common/avdk_pixel_types.h>

#include "cli_tuya_test.h"
#include "tkl_camera_mipi.h"
#include "tkl_media_fmt.h"
#include "tkl_fs.h"
#include "tuya_cloud_types.h"
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FaceRecognitionModel.h"
#include "face_gallery.h"

#define RECOG_TEST_TASK_PRIORITY    (BEKEN_DEFAULT_WORKER_PRIORITY - 1)
#define RECOG_TEST_TASK_STACK_SIZE  (24 * 1024)
#define RECOG_TEST_WIDTH            320U
#define RECOG_TEST_HEIGHT           320U
#define RECOG_TEST_BPP              4U
#define RECOG_TEST_FRAME_SIZE       (RECOG_TEST_WIDTH * RECOG_TEST_HEIGHT * RECOG_TEST_BPP)
#define RECOG_TEST_LOOP_FOREVER     (0xFFFFFFFFU)
#define RECOG_TEST_DEFAULT_DELAY_MS (200U)

typedef enum {
    RECOG_MODE_DETECT = 0,
    RECOG_MODE_ENROLL,
    RECOG_MODE_VERIFY,
} recog_mode_t;

static beken_thread_t s_recog_thread = NULL;
static volatile int s_recog_stop = 0;
static uint32_t s_recog_rounds = 1;
static uint32_t s_recog_delay_ms = RECOG_TEST_DEFAULT_DELAY_MS;
static recog_mode_t s_recog_mode = RECOG_MODE_DETECT;
static FaceRecognitionModel *s_model = NULL;
static bool s_model_ready = false;
static volatile bool s_enroll_cb_done = false;
static volatile bool s_enroll_cb_ok = false;

static void recog_box_cb(FaceRecogBox *boxes, int count)
{
    if (boxes == NULL || count <= 0) {
        bk_printf("[xrecog] no face\r\n");
        return;
    }
    bk_printf("[xrecog] box count=%d best=score=%.3f x=%.1f y=%.1f w=%.1f h=%.1f\r\n",
              count,
              boxes[0].score,
              boxes[0].x,
              boxes[0].y,
              boxes[0].w,
              boxes[0].h);
}

static void recog_enroll_cb(const FaceVerifyResult *result,
                            const uint8_t *aligned_rgb,
                            uint32_t aligned_rgb_size)
{
    uint32_t saved = 0;
    uint32_t profile_id = 0;

    s_enroll_cb_done = true;
    s_enroll_cb_ok = false;

    if (result == NULL || !result->valid || aligned_rgb == NULL) {
        bk_printf("[xrecog] enroll callback failed (no face/feature)\r\n");
        return;
    }

    if (s_recog_mode == RECOG_MODE_ENROLL) {
        if (face_gallery_enroll_save(result, aligned_rgb, aligned_rgb_size,
                                     &saved, &profile_id) == 0) {
            bk_printf("[xrecog] enroll ok profile=%u sample=%u/%u\r\n",
                      (unsigned)profile_id,
                      (unsigned)saved,
                      (unsigned)FACE_GALLERY_SAMPLES_PER_ID);
            s_enroll_cb_ok = true;
        } else {
            bk_printf("[xrecog] enroll save failed\r\n");
        }
        return;
    }

    if (s_recog_mode == RECOG_MODE_VERIFY) {
        float best_score = 0.0f;
        float best_avg = 0.0f;
        uint32_t best_profile = 0;
        uint32_t best_sample = 0;

        if (face_gallery_verify(result, &best_score, &best_avg,
                                &best_profile, &best_sample) != 0) {
            bk_printf("[xrecog] verify failed: empty gallery or IO error\r\n");
            return;
        }

        bool pass = (best_score >= kFaceVerifySameThreshold) &&
                    (best_avg >= FACE_GALLERY_AVG_THRESHOLD);
        bk_printf("[xrecog] verify profile=%u sample=%u score=%.3f avg=%.3f thr=%.2f/%.2f => %s\r\n",
                  (unsigned)best_profile,
                  (unsigned)best_sample,
                  best_score,
                  best_avg,
                  kFaceVerifySameThreshold,
                  FACE_GALLERY_AVG_THRESHOLD,
                  pass ? "PASS" : "FAIL");
        s_enroll_cb_ok = pass;
    }
}

static void recog_usage(void)
{
    bk_printf("xrecog help\r\n");
    bk_printf("xrecog init\r\n");
    bk_printf("xrecog detect once|start [delay_ms]|stop\r\n");
    bk_printf("xrecog enroll once|start [delay_ms]|stop\r\n");
    bk_printf("xrecog verify once|start [delay_ms]|stop\r\n");
    bk_printf("xrecog thr <0.0-1.0>     (detect score threshold, default 0.5)\r\n");
    bk_printf("xrecog layout auto|bgrx|rgbx  (keep bgrx; rgbx is worse on this board)\r\n");
    bk_printf("xrecog dump [path]       (save SP 320x320 RGB ppm to SD)\r\n");
    bk_printf("xrecog list\r\n");
    bk_printf("xrecog clear [id|all]\r\n");
    bk_printf("xrecog release\r\n");
    bk_printf("xrecog status\r\n");
    bk_printf("need: xsd mount; SP=320x320 rgb888 started; models under /sdcard/tflite/\r\n");
    bk_printf("  no preview: xmipi cam open frame 640 1024 rgb888 320 320\r\n");
    bk_printf("  + preview:  xmipi dsi open_gpu <panel> 1, then open display ... rgb888 320 320\r\n");
}

static uint32_t recog_parse_u32(const char *str, uint32_t fallback)
{
    char *end = NULL;
    unsigned long value;

    if ((str == NULL) || (str[0] == '\0')) {
        return fallback;
    }
    value = strtoul(str, &end, 10);
    if ((end == str) || ((end != NULL) && (*end != '\0'))) {
        return fallback;
    }
    return (uint32_t)value;
}

/**
 * @brief Borrow one SP frame. Release with frame->free_cb(frame->data).
 * @param[out] frame receives the frame on success; untouched on failure
 * @return BK_OK on success
 */
static bk_err_t recog_read_sp_frame(TKL_MEDIA_BUF_T *frame)
{
    OPERATE_RET ret;

    if (frame == NULL) {
        return BK_ERR_PARAM;
    }

    /* The camera owns the buffer and lends it; nothing to size or allocate here. */
    ret = tkl_camera_mipi_get_frame(frame, TUYA_CAMERA_MIPI_CHANNEL_SP, 1000U);
    if (ret != OPRT_OK) {
        bk_printf("[xrecog] get SP frame failed, ret=%d\r\n", ret);
        return BK_FAIL;
    }

    if ((frame->width != RECOG_TEST_WIDTH) ||
        (frame->height != RECOG_TEST_HEIGHT) ||
        (frame->size < RECOG_TEST_FRAME_SIZE)) {
        bk_printf("[xrecog] unexpected SP frame: %ux%u fmt=%u len=%u (need 320x320 BGRX)\r\n",
                  frame->width,
                  frame->height,
                  (unsigned int)frame->format,
                  (unsigned int)frame->size);
        frame->free_cb(frame->data);
        return BK_FAIL;
    }
    return BK_OK;
}

/**
 * @brief Dump one SP frame as RGB P6 ppm (assumes BGRX source like vendor)
 * @param[in] path output path under /sdcard
 * @return BK_OK on success
 */
static bk_err_t recog_dump_sp_ppm(const char *path)
{
    TKL_MEDIA_BUF_T frame;
    TUYA_FILE f = NULL;
    char header[48];
    int n;
    INT_T bw;
    uint32_t i;
    uint8_t rgb[3];
    uint32_t sum = 0;
    const char *out_path = (path != NULL && path[0] != '\0') ? path : "/sdcard/sp_dump.ppm";

    memset(&frame, 0, sizeof(frame));
    if (recog_read_sp_frame(&frame) != BK_OK) {
        return BK_FAIL;
    }

    for (i = 0; i < 64U; i++) {
        const uint32_t off = (i * 503U) % (RECOG_TEST_WIDTH * RECOG_TEST_HEIGHT);
        sum += frame.data[off * 4U] + frame.data[off * 4U + 1U] + frame.data[off * 4U + 2U];
    }

    f = tkl_fopen(out_path, "wb");
    if (f == NULL) {
        bk_printf("[xrecog] dump open failed: %s\r\n", out_path);
        frame.free_cb(frame.data);
        return BK_FAIL;
    }

    n = snprintf(header, sizeof(header), "P6\n%u %u\n255\n",
                 (unsigned)RECOG_TEST_WIDTH, (unsigned)RECOG_TEST_HEIGHT);
    bw = tkl_fwrite(header, n, f);
    if (bw != n) {
        tkl_fclose(f);
        frame.free_cb(frame.data);
        bk_printf("[xrecog] dump header write failed\r\n");
        return BK_FAIL;
    }

    /* Write as RGB: vendor/ISP 4bpp treated as BGRX -> R=s[2],G=s[1],B=s[0] */
    for (i = 0; i < RECOG_TEST_WIDTH * RECOG_TEST_HEIGHT; i++) {
        const uint8_t *s = &frame.data[i * 4U];
        rgb[0] = s[2];
        rgb[1] = s[1];
        rgb[2] = s[0];
        bw = tkl_fwrite(rgb, 3, f);
        if (bw != 3) {
            tkl_fclose(f);
            frame.free_cb(frame.data);
            bk_printf("[xrecog] dump pixel write failed at %u\r\n", (unsigned)i);
            return BK_FAIL;
        }
    }

    tkl_fclose(f);
    bk_printf("[xrecog] dump ok path=%s fmt=%u len=%u mean~%u (BGRX->RGB)\r\n",
              out_path,
              (unsigned)frame.format,
              (unsigned)frame.size,
              (unsigned)(sum / (64U * 3U)));
    frame.free_cb(frame.data);
    return BK_OK;
}

static void recog_model_release(void)
{
    if (s_model != NULL) {
        if (s_model_ready) {
            s_model->deinit();
        }
        FaceRecognitionModel::LogEnable(false);
        delete s_model;
        s_model = NULL;
        s_model_ready = false;
    }
}

static bk_err_t recog_model_prepare(void)
{
    int ret;

    if (face_gallery_prepare() != 0) {
        bk_printf("[xrecog] sd/gallery prepare failed (try: xsd mount)\r\n");
        return BK_FAIL;
    }

    if (s_model == NULL) {
        s_model = new FaceRecognitionModel();
        if (s_model == NULL) {
            recog_model_release();
            return BK_ERR_NO_MEM;
        }
        s_model->setBoxCallback(recog_box_cb);
        s_model->setEnrollResultCallback(recog_enroll_cb);
    }

    if (s_model_ready) {
        return BK_OK;
    }

    FaceRecognitionModel::LogEnable(true);
    ret = s_model->init();
    if (ret != 0) {
        bk_printf("[xrecog] model init failed, ret=%d (check SD tflite files)\r\n", ret);
        recog_model_release();
        return BK_FAIL;
    }
    s_model_ready = true;
    bk_printf("[xrecog] model ready\r\n");
    return BK_OK;
}

static void recog_task(void *arg)
{
    TKL_MEDIA_BUF_T frame;
    uint32_t rounds = s_recog_rounds;
    uint32_t delay_ms = s_recog_delay_ms;
    recog_mode_t mode = s_recog_mode;
    uint32_t index = 0;
    int ret;

    (void)arg;
    bk_printf("[xrecog] task start mode=%d rounds=%u delay=%ums\r\n",
              (int)mode, (unsigned)rounds, (unsigned)delay_ms);

    if (recog_model_prepare() != BK_OK) {
        goto exit;
    }

    while (!s_recog_stop && ((rounds == RECOG_TEST_LOOP_FOREVER) || (index < rounds))) {
        bk_printf("[xrecog] round %u start\r\n", (unsigned)(index + 1));

        memset(&frame, 0, sizeof(frame));

        if (recog_read_sp_frame(&frame) == BK_OK) {
            float max_score = -1.0f;
            int pass_count = 0;

            s_enroll_cb_done = false;
            s_enroll_cb_ok = false;
            s_model->setVerifyEnabled(mode != RECOG_MODE_DETECT);
            ret = s_model->run(frame.data,
                               frame.size,
                               (bk_pixel_format_t)tkl_media_fmt_to_bk(frame.format));
            s_model->getLastDetectDebug(&max_score, &pass_count);
            bk_printf("[xrecog] round %u done faces=%d mode=%d ok=%d fmt=%u max_score=%.3f pass=%d\r\n",
                      (unsigned)(index + 1),
                      ret,
                      (int)mode,
                      (mode == RECOG_MODE_DETECT) ? 1 : (int)s_enroll_cb_ok,
                      (unsigned)frame.format,
                      max_score,
                      pass_count);

            /* Borrowed for this round only - skipping this leaks a 400 KB
             * buffer per round and wedges the pool. */
            frame.free_cb(frame.data);
        }

        index++;
        if (!s_recog_stop && (rounds == RECOG_TEST_LOOP_FOREVER) && delay_ms > 0U) {
            rtos_delay_milliseconds(delay_ms);
        }
    }

exit:
    if (s_model != NULL) {
        s_model->setVerifyEnabled(false);
    }
    bk_printf("[xrecog] task exit stop=%d rounds_done=%u\r\n",
              s_recog_stop, (unsigned)index);
    s_recog_thread = NULL;
    s_recog_stop = 0;
    rtos_delete_thread(NULL);
}

static void recog_start(recog_mode_t mode, uint32_t rounds, uint32_t delay_ms)
{
    bk_err_t ret;

    if (s_recog_thread != NULL) {
        bk_printf("[xrecog] task already running\r\n");
        return;
    }

    s_recog_stop = 0;
    s_recog_mode = mode;
    s_recog_rounds = rounds;
    s_recog_delay_ms = delay_ms;

    ret = rtos_create_hsram_thread(&s_recog_thread,
                                   RECOG_TEST_TASK_PRIORITY,
                                   "xrecog_demo",
                                   (beken_thread_function_t)recog_task,
                                   RECOG_TEST_TASK_STACK_SIZE,
                                   NULL);
    if (ret != BK_OK) {
        s_recog_thread = NULL;
        bk_printf("[xrecog] create task failed, ret=%d\r\n", ret);
        return;
    }
    bk_printf("[xrecog] task created\r\n");
}

extern "C" void cli_tuya_face_recog_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    if ((argc < 2) || (argv[1] == NULL) ||
        (strcmp(argv[1], "help") == 0) ||
        (strcmp(argv[1], "?") == 0)) {
        recog_usage();
        return;
    }

    if (strcmp(argv[1], "init") == 0) {
        if (recog_model_prepare() == BK_OK) {
            bk_printf("[xrecog] init ok\r\n");
        }
        return;
    }

    if (strcmp(argv[1], "list") == 0) {
        (void)face_gallery_list();
        return;
    }

    if (strcmp(argv[1], "clear") == 0) {
        uint32_t id = 0;
        if (argc >= 3 && argv[2] != NULL && strcmp(argv[2], "all") != 0) {
            id = recog_parse_u32(argv[2], 0);
        }
        (void)face_gallery_clear(id);
        return;
    }

    if (strcmp(argv[1], "release") == 0) {
        if (s_recog_thread != NULL) {
            bk_printf("[xrecog] stop task before release\r\n");
            return;
        }
        recog_model_release();
        bk_printf("[xrecog] released\r\n");
        return;
    }

    if (strcmp(argv[1], "status") == 0) {
        bk_printf("[xrecog] running=%d stop=%d mode=%d rounds=%u delay=%ums model_ready=%d\r\n",
                  (s_recog_thread != NULL),
                  s_recog_stop,
                  (int)s_recog_mode,
                  (unsigned)s_recog_rounds,
                  (unsigned)s_recog_delay_ms,
                  s_model_ready);
        return;
    }

    if (strcmp(argv[1], "stop") == 0) {
        if (s_recog_thread == NULL) {
            bk_printf("[xrecog] task not running\r\n");
            return;
        }
        s_recog_stop = 1;
        bk_printf("[xrecog] stop requested\r\n");
        return;
    }

    if (strcmp(argv[1], "layout") == 0) {
        int mode = 0;
        if (argc < 3 || argv[2] == NULL) {
            recog_usage();
            return;
        }
        if (strcmp(argv[2], "auto") == 0 || strcmp(argv[2], "bgrx") == 0) {
            mode = (strcmp(argv[2], "bgrx") == 0) ? 1 : 0;
        } else if (strcmp(argv[2], "rgbx") == 0) {
            mode = 2;
        } else {
            bk_printf("[xrecog] layout need auto|bgrx|rgbx\r\n");
            return;
        }
        if (recog_model_prepare() != BK_OK) {
            return;
        }
        s_model->setInputLayoutMode(mode);
        bk_printf("[xrecog] layout mode=%d (%s)\r\n", mode, argv[2]);
        return;
    }

    if (strcmp(argv[1], "thr") == 0) {
        float thr = 0.5f;
        if (argc < 3 || argv[2] == NULL) {
            recog_usage();
            return;
        }
        thr = (float)strtod(argv[2], NULL);
        if (thr < 0.0f) {
            thr = 0.0f;
        }
        if (thr > 1.0f) {
            thr = 1.0f;
        }
        if (recog_model_prepare() != BK_OK) {
            return;
        }
        s_model->setDetectThreshold(thr);
        bk_printf("[xrecog] detect thr=%.3f\r\n", thr);
        return;
    }

    if (strcmp(argv[1], "dump") == 0) {
        const char *path = (argc >= 3) ? argv[2] : NULL;
        (void)recog_dump_sp_ppm(path);
        return;
    }

    /* detect|enroll|verify  once|start */
    if ((strcmp(argv[1], "detect") == 0) ||
        (strcmp(argv[1], "enroll") == 0) ||
        (strcmp(argv[1], "verify") == 0)) {
        recog_mode_t mode = RECOG_MODE_DETECT;
        if (strcmp(argv[1], "enroll") == 0) {
            mode = RECOG_MODE_ENROLL;
        } else if (strcmp(argv[1], "verify") == 0) {
            mode = RECOG_MODE_VERIFY;
        }

        if (argc < 3 || argv[2] == NULL) {
            recog_usage();
            return;
        }
        if (strcmp(argv[2], "once") == 0) {
            recog_start(mode, 1, 0);
            return;
        }
        if (strcmp(argv[2], "start") == 0) {
            uint32_t delay_ms = recog_parse_u32((argc >= 4) ? argv[3] : NULL,
                                                RECOG_TEST_DEFAULT_DELAY_MS);
            recog_start(mode, RECOG_TEST_LOOP_FOREVER, delay_ms);
            return;
        }
        if (strcmp(argv[2], "stop") == 0) {
            s_recog_stop = 1;
            bk_printf("[xrecog] stop requested\r\n");
            return;
        }
    }

    bk_printf("[xrecog] unknown command\r\n");
    recog_usage();
}
