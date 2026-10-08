

#include <stdbool.h>

#include <os/str.h>

#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_thread.h"
#include "cli_tuya_test.h"
#include "tkl_mipi_csi.h"
#include "tkl_uvc.h"

/* The caller keys the overlay by channel - ch2 feeds the display path, ch3
 * the encode path. Both map onto two fixed slots through test_overlay_ctx_get(),
 * so a renumbered enum cannot silently index the wrong slot: an unknown
 * channel is rejected, not misrouted. */
#define TEST_OVERLAY_SLOT_MAX 2U

#define TEST_OVERLAY_TIME_TEXT_LEN 16U

#define TEST_OVERLAY_REFRESH_MS 1000U
#define TEST_OVERLAY_STACK      (3U * 1024U)
#define TEST_OVERLAY_STOP_WAIT_MS 3000U
/* The refresh wait is served in slices this long. Sleeping the whole period in
 * one go charged up to TEST_OVERLAY_REFRESH_MS to every close that had a clock
 * running - measured at 640 ms of a 690 ms close. */
#define TEST_OVERLAY_POLL_MS      20U

/**
 * @brief Push one clock string to an output's overlay.
 * @param[in] text HH:MM:SS
 * @param[in] arg  caller context handed to test_overlay_clock_start()
 * @return OPRT_OK when the overlay was accepted
 */
typedef OPERATE_RET (*TEST_OVERLAY_SUBMIT_CB)(CONST CHAR_T *text, VOID_T *arg);

typedef struct {
    volatile BOOL_T running;
    volatile BOOL_T stop_req;
    TKL_THREAD_HANDLE task;
    TEST_OVERLAY_SUBMIT_CB cb;
    VOID_T *arg;
    CHAR_T last_text[TEST_OVERLAY_TIME_TEXT_LEN];
} test_overlay_ctx_t;

static test_overlay_ctx_t s_overlay[TEST_OVERLAY_SLOT_MAX];

static test_overlay_ctx_t *test_overlay_ctx_get(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    switch (channel) {
    case TUYA_CAMERA_FRAME_CHANNEL_2:
        return &s_overlay[0];
    case TUYA_CAMERA_FRAME_CHANNEL_3:
        return &s_overlay[1];
    default:
        return NULL;
    }
}

static const char *test_overlay_type_name(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    return (channel == TUYA_CAMERA_FRAME_CHANNEL_3) ? "enc" : "disp";
}

STATIC VOID_T test_overlay_format_time(CHAR_T *text, UINT32_T text_len)
{
    SYS_TIME_T now_ms;
    UINT32_T total_sec;
    UINT32_T hour;
    UINT32_T min;
    UINT32_T sec;

    if (text == NULL || text_len < 9U) {
        return;
    }

    now_ms = tkl_system_get_millisecond();
    total_sec = (UINT32_T)(now_ms / 1000U);
    hour = (total_sec / 3600U) % 100U;
    min = (total_sec / 60U) % 60U;
    sec = total_sec % 60U;

    text[0] = (CHAR_T)('0' + (hour / 10U));
    text[1] = (CHAR_T)('0' + (hour % 10U));
    text[2] = ':';
    text[3] = (CHAR_T)('0' + (min / 10U));
    text[4] = (CHAR_T)('0' + (min % 10U));
    text[5] = ':';
    text[6] = (CHAR_T)('0' + (sec / 10U));
    text[7] = (CHAR_T)('0' + (sec % 10U));
    text[8] = '\0';
}

static void test_overlay_task(void *arg)
{
    test_overlay_ctx_t *ctx = (test_overlay_ctx_t *)arg;
    TKL_THREAD_HANDLE self = NULL;
    CHAR_T text[TEST_OVERLAY_TIME_TEXT_LEN];
    UINT32_T waited;

    (void)tkl_thread_get_id(&self);

    while (ctx->stop_req != TRUE) {
        test_overlay_format_time(text, sizeof(text));
        if (os_strcmp(text, ctx->last_text) != 0) {
            if (ctx->cb(text, ctx->arg) == OPRT_OK) {
                os_strncpy(ctx->last_text, text, sizeof(ctx->last_text) - 1U);
                ctx->last_text[sizeof(ctx->last_text) - 1U] = '\0';
            }
        }
        for (waited = 0U;
             waited < TEST_OVERLAY_REFRESH_MS && ctx->stop_req != TRUE;
             waited += TEST_OVERLAY_POLL_MS) {
            tkl_system_sleep(TEST_OVERLAY_POLL_MS);
        }
    }

    ctx->running = FALSE;
    ctx->task = NULL;
    tkl_thread_release(self);
}

OPERATE_RET test_overlay_clock_start(TUYA_CAMERA_FRAME_CHANNEL_E channel,
                                     TEST_OVERLAY_SUBMIT_CB cb, VOID_T *arg)
{
    test_overlay_ctx_t *ctx;
    CHAR_T text[TEST_OVERLAY_TIME_TEXT_LEN];
    OPERATE_RET ret;

    if (cb == NULL) {
        return OPRT_INVALID_PARM;
    }

    ctx = test_overlay_ctx_get(channel);
    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    test_overlay_clock_stop(channel);

    ctx->cb = cb;
    ctx->arg = arg;

    /* submit once here so a misconfiguration is reported to the CLI, not just
     * logged from the refresh task */
    test_overlay_format_time(text, sizeof(text));
    ret = cb(text, arg);
    if (ret != OPRT_OK) {
        tkl_log_output("overlay %s: first submit failed: %d\r\n",
                       test_overlay_type_name(channel), ret);
        return ret;
    }

    os_strncpy(ctx->last_text, text, sizeof(ctx->last_text) - 1U);
    ctx->last_text[sizeof(ctx->last_text) - 1U] = '\0';
    ctx->stop_req = FALSE;
    ctx->running = TRUE;

    ret = tkl_thread_create(&ctx->task,
                            (channel == TUYA_CAMERA_FRAME_CHANNEL_3) ? "osd_enc" : "osd_disp",
                            TEST_OVERLAY_STACK,
                            TKL_THREAD_PRI_NORMAL,
                            test_overlay_task,
                            ctx);
    if (ret != OPRT_OK) {
        ctx->running = FALSE;
        ctx->task = NULL;
        return ret;
    }

    tkl_log_output("overlay %s: clock started at %s\r\n",
                   test_overlay_type_name(channel), text);
    return OPRT_OK;
}

VOID_T test_overlay_clock_stop(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    test_overlay_ctx_t *ctx;
    UINT32_T waited_ms = 0U;

    ctx = test_overlay_ctx_get(channel);
    if (ctx == NULL) {
        return;
    }

    if (ctx->running != TRUE && ctx->task == NULL) {
        return;
    }

    ctx->stop_req = TRUE;
    while (ctx->running == TRUE && waited_ms < TEST_OVERLAY_STOP_WAIT_MS) {
        tkl_system_sleep(20U);
        waited_ms += 20U;
    }

    ctx->running = FALSE;
    ctx->task = NULL;
    ctx->stop_req = FALSE;
    ctx->last_text[0] = '\0';
}

STATIC BOOL_T test_overlay_clock_running(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    test_overlay_ctx_t *ctx = test_overlay_ctx_get(channel);

    if (ctx == NULL) {
        return FALSE;
    }

    return ctx->running;
}
