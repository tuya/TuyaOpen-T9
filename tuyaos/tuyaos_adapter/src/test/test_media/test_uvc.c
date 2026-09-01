#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <components/bk_frame_buffer.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>

#include "blend_dsc.h"
#include "cli_tuya_test.h"
#include "tkl_media_buf.h"
#include "tkl_mipi_dsi.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_uvc.h"

#define UVC_TEST_DEFAULT_WIDTH       640U
#define UVC_TEST_DEFAULT_HEIGHT      480U
#define UVC_TEST_DEFAULT_FPS         15U
#define UVC_TEST_DEFAULT_PORT        1U
#define UVC_TEST_LOG_INTERVAL        30U

extern BOOL_T test_mipi_lcd_is_open(void);

/* ===========================================================================
 * Peripheral-oriented facade (design spec P4a): xuvc open / close / stats
 *
 * The pull model needs a thread and TKL does not own one, so this is it. The
 * loop is the whole contract: get a frame, use it, hand it back through the
 * free_cb that came with it - with no idea whether that returns a slot to a
 * pool or frees memory.
 * =========================================================================== */

#define UVC_OUT_THREAD_STACK   (1024U * 4U)
#define UVC_OUT_GET_TIMEOUT_MS 1000U
#define UVC_OUT_JOIN_TIMEOUT   3000U
#define UVC_OUT_LOG_INTERVAL   30U
#define UVC_OUT_LUMA_STEP      16U
/* Panel geometry for xuvc open display, matching what xmipi dsi open_gpu sets
 * up; the facade scales and rotates the decoded picture into it. */
#define UVC_OUT_DISPLAY_ROTATE 90U
#define UVC_OUT_DISPLAY_W      640U
#define UVC_OUT_DISPLAY_H      1024U
#define UVC_OUT_ENCODE_GOP     40
/* "xuvc restart" waits for this many decoded frames before pausing, so that
 * each round measures pausing a stream that is actually running. Without it the
 * loop stops the pipeline before the device has delivered anything and the
 * worker sits out its whole decode_once() timeout - which measures UVC
 * stream-start latency, not the cost of stopping. Board-measured: 300 ms of
 * first-frame latency turned a 33 ms stop into a 494 ms one. */
/* one grammar, shared by the two usage sites and both reject paths - and the
 * same shape as "xmipi cam osd", so the two facades read alike on the wire */
#define TEST_UVC_OSD_USAGE \
    "xuvc osd <display|encode> <time [off]|wifi <0..4|off>|clear|caps>"

/* How long close() waits for one overlay-free frame to be composited. Four
 * frames at 19 fps; it returns as soon as the frame lands, so this is only the
 * ceiling for a pipeline that has already stopped delivering. */
#define UVC_OSD_CLEAR_SETTLE_MS   200U

#define UVC_RESTART_SETTLE_FRAMES 5U
#define UVC_RESTART_SETTLE_MS     1500U

/* Which stream "xuvc record" is writing. H264 comes off the encode callback,
 * MJPEG off whichever path already carries the device bitstream: mjpeg_cb in
 * push modes, get_frame() in pull MJPEG mode. */
typedef enum {
    UVC_REC_NONE = 0,
    UVC_REC_H264,
    UVC_REC_MJPEG,
} uvc_rec_kind_t;

typedef struct {
    bool opened;
    /* TRUE between "xuvc stop" and "xuvc start": the device is still enumerated
     * and the pools still exist, but the adapter is not streaming and our
     * consumer thread is gone. Kept separately from run/thread because close()
     * must know not to wait on a semaphore nobody is left to signal. */
    bool paused;
    bool push;                  /* DISPLAY / H264: decode_once, not get_frame */
    bool tee_mjpeg;             /* also keep the raw bitstream mjpeg_cb carries */
    uint8_t rec_kind;           /* uvc_rec_kind_t */
    uint32_t tee_frames;
    uint64_t tee_bytes;
    volatile bool run;
    beken_thread_t thread;
    beken_semaphore_t done;
    uint32_t kind;
    uint32_t display_frames;
    uint32_t stream_frames;
    uint64_t stream_bytes;
    uint32_t decodes;
    uint32_t frames;
    uint32_t timeouts;
    uint32_t errors;
    uint64_t bytes;
    uint32_t last_size;
    uint16_t last_w;
    uint16_t last_h;
    uint32_t last_luma;
    uint32_t start_ms;

    /* fps is measured from here, not from start_ms, and both are re-based on
     * every tkl_uvc_start(). start_ms is wall time since open and therefore
     * includes the paused stretches of "xuvc restart"; dividing the cumulative
     * frame count by it reported 7.9 fps on a pipeline that was running at
     * 19 - a reporting artifact that looked exactly like a real throughput
     * collapse. The counters themselves stay cumulative: display == h264 ==
     * decodes across restarts is the check that matters. */
    uint32_t fps_mark_ms;
    uint32_t fps_mark_frames;
} test_uvc_out_ctx_t;

static test_uvc_out_ctx_t s_uvc_out_t;

/* What the last successful "xuvc open" asked for. Deliberately outside
 * test_uvc_out_ctx_t: that struct is zeroed by both open and close, and
 * "xuvc reopen" has to replay these parameters after the close. */
static struct {
    bool     valid;
    uint32_t kind;
    uint16_t width;
    uint16_t height;
    uint8_t  fps;
    uint8_t  port;
    uint8_t  drop_num;
} s_uvc_last_open;

/* "xuvc idr" arms this; see the identical note in test_camera.c. */
static volatile uint8_t s_uvc_idr_trace;

/**
 * @brief Coarse mean of the Y plane.
 *
 * Sampling one pixel in 16 both ways is enough to tell a decoded picture from a
 * buffer nobody wrote: a real frame lands somewhere in the tens to low
 * hundreds, an untouched one reads 0. The buffer is non-cacheable PSRAM, so
 * reading all of it would be slow for no extra information.
 */
static uint32_t test_uvc_out_mean_luma(const TKL_MEDIA_BUF_T *buf)
{
    uint32_t stride = (buf->y_stride != 0U) ? buf->y_stride : (uint32_t)buf->width;
    uint32_t sum = 0U;
    uint32_t n = 0U;
    uint16_t y;

    if (buf->data == NULL || buf->width == 0U || buf->height == 0U) {
        return 0U;
    }

    for (y = 0U; y < buf->height; y = (uint16_t)(y + UVC_OUT_LUMA_STEP)) {
        const uint8_t *row = buf->data + ((uint32_t)y * stride);
        uint16_t x;

        for (x = 0U; x < buf->width; x = (uint16_t)(x + UVC_OUT_LUMA_STEP)) {
            sum += row[x];
            n++;
        }
    }

    return (n != 0U) ? (sum / n) : 0U;
}

/**
 * @brief Composed picture from the GPU. Runs on the GPU worker thread.
 *
 * @note Hands ownership straight on to the panel: tkl_mipi_dsi_flush() takes the
 *       buffer and its free_cb together, so the frame is released once the DPU
 *       is done scanning it. Identical to test_camera.c's - which is the point,
 *       the two peripherals deliver the same contract.
 */
static VOID_T test_uvc_out_display_cb(CONST TKL_MEDIA_BUF_T *buf, VOID_T *arg)
{
    tkl_mipi_dsi_flush_frame_t flush;
    OPERATE_RET ret;

    (void)arg;

    if (buf == NULL || buf->data == NULL) {
        return;
    }

    s_uvc_out_t.display_frames++;

    if (test_mipi_lcd_is_open() != TRUE) {
        if (buf->free_cb != NULL) {
            buf->free_cb(buf->data);
        }
        return;
    }

    os_memset(&flush, 0, sizeof(flush));
    flush.buffer = buf->data;
    flush.free_cb = buf->free_cb;

    ret = tkl_mipi_dsi_flush(&flush);
    if (ret != OPRT_OK) {
        /* a flush that got past validation released the frame itself */
        if (ret == OPRT_INVALID_PARM || ret == OPRT_RESOURCE_NOT_READY) {
            if (buf->free_cb != NULL) {
                buf->free_cb(buf->data);
            }
        }
    }
}

/**
 * @brief Transcoded H264. Runs on the encoder worker thread, borrowed buffer.
 */
static VOID_T test_uvc_out_stream_cb(CONST TKL_MEDIA_BUF_T *buf, VOID_T *arg)
{
    (void)arg;

    if (buf == NULL || buf->data == NULL) {
        return;
    }

    s_uvc_out_t.stream_frames++;
    s_uvc_out_t.stream_bytes += buf->size;

    if (s_uvc_out_t.rec_kind == (uint8_t)UVC_REC_H264) {
        /* copies and returns; the card is the writer thread's problem */
        test_record_feed(buf->data, buf->size);
    }

    if (s_uvc_idr_trace > 0U) {
        s_uvc_idr_trace--;
        tkl_log_output("xuvc idr trace[%u]: %s size=%u frame_type=%u\r\n",
                (unsigned int)s_uvc_out_t.stream_frames,
                (buf->frame_type == (UINT32_T)TKL_MEDIA_FRAME_I) ? "I" : "P",
                (unsigned int)buf->size,
                (unsigned int)buf->frame_type);
    }

    if ((s_uvc_out_t.stream_frames % UVC_OUT_LOG_INTERVAL) == 0U) {
        tkl_log_output("xuvc h264[%u]: %s size=%u total=%llu display=%u decodes=%u\r\n",
                (unsigned int)s_uvc_out_t.stream_frames,
                (buf->frame_type == (UINT32_T)TKL_MEDIA_FRAME_I) ? "I" : "P",
                (unsigned int)buf->size,
                (unsigned long long)s_uvc_out_t.stream_bytes,
                (unsigned int)s_uvc_out_t.display_frames,
                (unsigned int)s_uvc_out_t.decodes);
    }
}

/**
 * @brief The device's own MJPEG for a picture that just reached display / h264.
 *
 * @note Runs inside tkl_uvc_decode_once(), on the decode thread below, and the
 *       buffer is borrowed for this call only - so the copy into the recorder's
 *       ring has to happen here rather than being deferred. Reading tee_mjpeg
 *       here is safe however the CLI thread flips it: there is no second
 *       decision to keep in step with it any more.
 */
static VOID_T test_uvc_out_mjpeg_cb(CONST TKL_MEDIA_BUF_T *buf, VOID_T *arg)
{
    (void)arg;

    if (buf == NULL || buf->data == NULL || s_uvc_out_t.tee_mjpeg != true) {
        return;
    }

    s_uvc_out_t.tee_frames++;
    s_uvc_out_t.tee_bytes += buf->size;

    if (s_uvc_out_t.rec_kind == (uint8_t)UVC_REC_MJPEG) {
        test_record_feed(buf->data, buf->size);
    }
}

/**
 * @brief The thread tkl_uvc_decode_once() needs. TKL owns none, so business
 *        brings one.
 */
static void test_uvc_out_decode_worker(beken_thread_arg_t arg)
{
    (void)arg;

    while (s_uvc_out_t.run == true) {
        OPERATE_RET ret = tkl_uvc_decode_once(UVC_OUT_GET_TIMEOUT_MS);

        if (ret == OPRT_TIMEOUT) {
            s_uvc_out_t.timeouts++;
            continue;
        }
        if (ret != OPRT_OK) {
            s_uvc_out_t.errors++;
            rtos_delay_milliseconds(10);
            continue;
        }

        s_uvc_out_t.decodes++;

        if ((s_uvc_out_t.decodes % UVC_OUT_LOG_INTERVAL) == 0U) {
            uint32_t ms = (uint32_t)tkl_system_get_millisecond() - s_uvc_out_t.fps_mark_ms;
            uint32_t frames = s_uvc_out_t.decodes - s_uvc_out_t.fps_mark_frames;

            tkl_log_output("xuvc decode: %u done, display=%u h264=%u mjpeg=%u(%llu B), "
                    "%u.%u fps to=%u err=%u\r\n",
                    (unsigned int)s_uvc_out_t.decodes,
                    (unsigned int)s_uvc_out_t.display_frames,
                    (unsigned int)s_uvc_out_t.stream_frames,
                    (unsigned int)s_uvc_out_t.tee_frames,
                    (unsigned long long)s_uvc_out_t.tee_bytes,
                    (unsigned int)((ms != 0U) ? (frames * 1000U / ms) : 0U),
                    (unsigned int)((ms != 0U) ? ((frames * 10000U / ms) % 10U) : 0U),
                    (unsigned int)s_uvc_out_t.timeouts,
                    (unsigned int)s_uvc_out_t.errors);
        }
    }

    s_uvc_out_t.run = false;
    (void)rtos_set_semaphore(&s_uvc_out_t.done);
    rtos_delete_thread(NULL);
}

static void test_uvc_out_worker(beken_thread_arg_t arg)
{
    (void)arg;

    while (s_uvc_out_t.run == true) {
        TKL_MEDIA_BUF_T buf;
        OPERATE_RET ret;

        ret = tkl_uvc_get_frame(&buf, UVC_OUT_GET_TIMEOUT_MS);
        if (ret == OPRT_TIMEOUT) {
            s_uvc_out_t.timeouts++;
            continue;
        }
        if (ret != OPRT_OK) {
            s_uvc_out_t.errors++;
            rtos_delay_milliseconds(10);
            continue;
        }

        s_uvc_out_t.frames++;
        s_uvc_out_t.bytes += buf.size;
        s_uvc_out_t.last_size = buf.size;
        s_uvc_out_t.last_w = buf.width;
        s_uvc_out_t.last_h = buf.height;
        if (buf.kind == (uint8_t)TKL_MEDIA_BUF_PIXEL) {
            s_uvc_out_t.last_luma = test_uvc_out_mean_luma(&buf);
        } else if (s_uvc_out_t.rec_kind == (uint8_t)UVC_REC_MJPEG) {
            /* pull MJPEG mode: this thread already holds the device bitstream */
            test_record_feed(buf.data, buf.size);
        }

        if ((s_uvc_out_t.frames % UVC_OUT_LOG_INTERVAL) == 0U) {
            uint32_t ms = (uint32_t)tkl_system_get_millisecond() - s_uvc_out_t.start_ms;

            tkl_log_output("xuvc out: %u frames %ux%u %u bytes luma=%u %u.%u fps to=%u err=%u\r\n",
                    (unsigned int)s_uvc_out_t.frames,
                    (unsigned int)buf.width,
                    (unsigned int)buf.height,
                    (unsigned int)buf.size,
                    (unsigned int)s_uvc_out_t.last_luma,
                    (unsigned int)((ms != 0U) ? (s_uvc_out_t.frames * 1000U / ms) : 0U),
                    (unsigned int)((ms != 0U) ? ((s_uvc_out_t.frames * 10000U / ms) % 10U) : 0U),
                    (unsigned int)s_uvc_out_t.timeouts,
                    (unsigned int)s_uvc_out_t.errors);
        }

        /* The whole point: business does not know which pool this came from. */
        if (buf.free_cb != NULL) {
            buf.free_cb(buf.data);
        }
    }

    s_uvc_out_t.run = false;
    (void)rtos_set_semaphore(&s_uvc_out_t.done);
    rtos_delete_thread(NULL);
}

static void test_uvc_out_state_cb(BOOL_T connected, VOID *arg)
{
    (void)arg;
    tkl_log_output("xuvc out: device %s\r\n", (connected == TRUE) ? "connected" : "gone");
}

/**
 * @brief Push the shared clock string to one uvc overlay channel.
 * @note Same content and same helper as xmipi cam - only the submit call differs,
 *       which is the whole point of the two facades having one OSD contract.
 */
static OPERATE_RET test_uvc_osd_submit(CONST CHAR_T *text, VOID_T *arg)
{
    TKL_UVC_CHN_E chn = (TKL_UVC_CHN_E)(uintptr_t)arg;
    TKL_OSD_T osd;

    osd_assets_set_clock(text);

    os_memset(&osd, 0, sizeof(osd));
    osd.slot = OSD_SLOT_CLOCK;
    osd.elems = osd_elems_clock;
    osd.count = osd_elems_clock_count;

    if (chn == TKL_UVC_CHN_DISPLAY) {
        osd.dst_x = OSD_CLOCK_DST_X;
        osd.dst_y = OSD_CLOCK_DST_Y;
        /* left at 0: tkl_uvc_set_osd() overwrites it with the angle the display
         * compositor was actually opened at */
    } else {
        /* the encoder sees the un-rotated decoder output, so its origin is its own */
        osd.dst_x = 2U;
        osd.dst_y = 2U;
        osd.rotate = 0U;
    }

    return tkl_uvc_set_osd(chn, &osd);
}

/**
 * @brief Take the clock sprite away, leaving every other slot alone.
 *
 * @param[in] chn which channel's compositor
 *
 * @note Stopping the refresh task only stops the redraw - the last string stays
 *       composited until its own slot is cleared, so "time off" has to do both.
 *       A slot with no element is the compositor's "clear this slot".
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
static OPERATE_RET test_uvc_osd_clock_clear(TKL_UVC_CHN_E chn)
{
    TKL_OSD_T osd;

    os_memset(&osd, 0, sizeof(osd));
    osd.slot = OSD_SLOT_CLOCK;

    return tkl_uvc_set_osd(chn, &osd);
}

/**
 * @brief Put the wifi icon in its own slot, or take it away.
 *
 * @param[in] chn   which channel's compositor
 * @param[in] level 0..4 signal bars; anything above 3 shows full
 * @param[in] on    FALSE clears the slot instead of drawing
 *
 * @note Separate slot from the clock (OSD_SLOT_WIFI vs OSD_SLOT_CLOCK) so the
 *       two can be raised and cleared independently - a slot is one sprite, and
 *       sharing one would mean redrawing the clock every time the bars change.
 * @note This is the only exercise TKL_OSD_ELEM_IMAGE gets. The clock path is
 *       TKL_OSD_ELEM_TEXT, and every other CLI OSD command goes through it - so
 *       without this the bitmap element type has no coverage at all.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
static OPERATE_RET test_uvc_osd_wifi(TKL_UVC_CHN_E chn, UINT8_T level, BOOL_T on)
{
    TKL_OSD_T osd;

    os_memset(&osd, 0, sizeof(osd));
    osd.slot = OSD_SLOT_WIFI;

    if (on == TRUE) {
        osd_assets_set_wifi(level);
        osd.elems = osd_elems_wifi;
        osd.count = osd_elems_wifi_count;

        if (chn == TKL_UVC_CHN_DISPLAY) {
            osd.dst_x = OSD_WIFI_DST_X;
            osd.dst_y = OSD_WIFI_DST_Y;
            /* rotate left at 0 - the facade fills in the compositor's angle */
        } else {
            /* the encoder sees the un-rotated frame; keep clear of the clock */
            osd.dst_x = 2U;
            osd.dst_y = 72U;
            osd.rotate = 0U;
        }
    }
    /* elems == NULL with a valid slot is the clear operation */

    return tkl_uvc_set_osd(chn, &osd);
}

static OPERATE_RET test_uvc_out_open(uint32_t kind,
                                     uint16_t width,
                                     uint16_t height,
                                     uint8_t fps,
                                     uint8_t port,
                                     uint8_t drop_num)
{
    TKL_UVC_CFG_T cfg;
    OPERATE_RET ret;

    if (s_uvc_out_t.opened == true) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* Before anything can allocate. The slab heaps are only up if some earlier
     * command happened to bring them up - "xmipi dsi open_gpu" does it as a side
     * effect - and xuvc needs neither a panel nor a camera, so without this the
     * first pool allocation asserts with "type:0 not init". Same call, same
     * reason, as test_camera.c's xmipi cam open. */
    tkl_media_buf_init();

    os_memset(&s_uvc_out_t, 0, sizeof(s_uvc_out_t));
    s_uvc_out_t.kind = kind;

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.port = port;
    cfg.width = width;
    cfg.height = height;
    cfg.fps = fps;
    cfg.drop_num = drop_num;
    cfg.src_codec = (uint32_t)TKL_MEDIA_CODEC_JPEG;
    cfg.out_kinds = kind;
    cfg.state_cb = test_uvc_out_state_cb;

    if ((kind & (TKL_UVC_OUT_DISPLAY | TKL_UVC_OUT_H264)) != 0U) {
        s_uvc_out_t.push = true;
        cfg.display_cb = test_uvc_out_display_cb;
        cfg.stream_cb = test_uvc_out_stream_cb;
        /* Always registered, gated by tee_mjpeg inside: "xuvc mjpeg on" has to
         * take effect on a pipeline that is already running. */
        cfg.mjpeg_cb = test_uvc_out_mjpeg_cb;
        /* The panel is the target, so the display geometry is the panel's, not
         * the camera's - rotate 90 to match how xmipi opens it. */
        cfg.display.rotate = UVC_OUT_DISPLAY_ROTATE;
        cfg.display.dst_w = UVC_OUT_DISPLAY_W;
        cfg.display.dst_h = UVC_OUT_DISPLAY_H;
        cfg.display.format = (uint32_t)TKL_PIXEL_FMT_ARGB8888;
        cfg.encode.gop = UVC_OUT_ENCODE_GOP;
    }

    ret = tkl_uvc_init(&cfg);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_uvc_start();
    if (ret != OPRT_OK) {
        (void)tkl_uvc_deinit();
        return ret;
    }

    ret = rtos_init_semaphore_ex(&s_uvc_out_t.done, 1, 0);
    if (ret != OPRT_OK) {
        (void)tkl_uvc_deinit();
        return ret;
    }

    s_uvc_out_t.run = true;
    s_uvc_out_t.start_ms = (uint32_t)tkl_system_get_millisecond();
    s_uvc_out_t.fps_mark_ms = s_uvc_out_t.start_ms;
    s_uvc_out_t.fps_mark_frames = s_uvc_out_t.decodes;

    ret = rtos_create_thread(&s_uvc_out_t.thread,
                             BEKEN_DEFAULT_WORKER_PRIORITY,
                             "uvc_out",
                             (beken_thread_function_t)(s_uvc_out_t.push
                                                           ? test_uvc_out_decode_worker
                                                           : test_uvc_out_worker),
                             UVC_OUT_THREAD_STACK,
                             NULL);
    if (ret != OPRT_OK) {
        s_uvc_out_t.run = false;
        (void)rtos_deinit_semaphore(&s_uvc_out_t.done);
        s_uvc_out_t.done = NULL;
        (void)tkl_uvc_deinit();
        return ret;
    }

    s_uvc_out_t.opened = true;

    s_uvc_last_open.valid = true;
    s_uvc_last_open.kind = kind;
    s_uvc_last_open.width = width;
    s_uvc_last_open.height = height;
    s_uvc_last_open.fps = fps;
    s_uvc_last_open.port = port;
    s_uvc_last_open.drop_num = drop_num;
    return OPRT_OK;
}

/**
 * @brief Drop every overlay sprite and let one clean frame reach the panel.
 *
 * @note Clearing the slots is not enough on its own. The panel keeps whatever
 *       the compositor last flushed and nothing repaints it once the stream is
 *       down, so a frame has to be composited *after* the clear - this must run
 *       while the pipeline is still streaming, and then wait for one to arrive.
 *       This facade stops its producer first, so the wait is not optional.
 * @note Test-layer policy, not adapter behaviour: business decides for itself
 *       whether an overlay should survive a camera switch. The CLI always
 *       clears, so a stale clock never carries into the next case.
 */
static void test_uvc_out_osd_clear_settle(void)
{
    uint32_t mark;
    uint32_t t;

    /* stopped: set_osd refuses with -23, and no frame would repaint anyway */
    if (s_uvc_out_t.push != true || s_uvc_out_t.paused == true) {
        return;
    }

    (void)test_uvc_osd_clock_clear(TKL_UVC_CHN_DISPLAY);
    (void)test_uvc_osd_wifi(TKL_UVC_CHN_DISPLAY, 0U, FALSE);
    (void)test_uvc_osd_clock_clear(TKL_UVC_CHN_ENCODE);
    (void)test_uvc_osd_wifi(TKL_UVC_CHN_ENCODE, 0U, FALSE);

    mark = s_uvc_out_t.display_frames;
    t = (uint32_t)tkl_system_get_millisecond();
    while (s_uvc_out_t.display_frames == mark &&
           ((uint32_t)tkl_system_get_millisecond() - t) < UVC_OSD_CLEAR_SETTLE_MS) {
        rtos_delay_milliseconds(5);
    }
}

static OPERATE_RET test_uvc_out_close(void)
{
    uint32_t t0 = (uint32_t)tkl_system_get_millisecond();
    uint32_t overlay_ms;
    OPERATE_RET ret;

    if (s_uvc_out_t.opened != true) {
        return OPRT_OK;
    }

    /* Consumer first. It may be parked in get_frame() for up to one timeout, and
     * it must not be inside the pipeline when tkl_uvc_deinit() takes it apart. */
    test_overlay_clock_stop((UINT8_T)TKL_UVC_CHN_DISPLAY);
    test_overlay_clock_stop((UINT8_T)TKL_UVC_CHN_ENCODE);
    /* the clock task only tests stop_req once per refresh period, so parking it
     * costs up to TEST_OVERLAY_REFRESH_MS - report it rather than let it hide
     * inside "close felt slow" */
    overlay_ms = (uint32_t)tkl_system_get_millisecond() - t0;

    /* before the consumer goes: the clear only becomes visible through a frame
     * that the consumer is still pumping */
    test_uvc_out_osd_clear_settle();

    s_uvc_out_t.run = false;
    if (s_uvc_out_t.done != NULL) {
        /* a paused pipeline has no worker left to signal this - waiting would
         * just spend the whole join timeout before deinit */
        if (s_uvc_out_t.paused != true) {
            (void)rtos_get_semaphore(&s_uvc_out_t.done, UVC_OUT_JOIN_TIMEOUT);
        }
        (void)rtos_deinit_semaphore(&s_uvc_out_t.done);
        s_uvc_out_t.done = NULL;
    }

    ret = tkl_uvc_deinit();

    /* Only here are all three feeders certainly quiet: decode_once()/get_frame()
     * joined above - which also parks mjpeg_cb, since it only ever runs inside
     * decode_once() - and the encode callback stops with tkl_uvc_deinit(). It
     * frees the ring, so it must not race a test_record_feed() still in flight. */
    s_uvc_out_t.rec_kind = (uint8_t)UVC_REC_NONE;
    test_record_stop();

    if (s_uvc_out_t.push == true) {
        tkl_log_output("xuvc out: closed, %u decoded, display %u, h264 %u frames %llu bytes, "
                "timeouts %u errors %u\r\n",
                (unsigned int)s_uvc_out_t.decodes,
                (unsigned int)s_uvc_out_t.display_frames,
                (unsigned int)s_uvc_out_t.stream_frames,
                (unsigned long long)s_uvc_out_t.stream_bytes,
                (unsigned int)s_uvc_out_t.timeouts,
                (unsigned int)s_uvc_out_t.errors);
    } else {
        tkl_log_output("xuvc out: closed, %u frames %llu bytes, timeouts %u errors %u\r\n",
                (unsigned int)s_uvc_out_t.frames,
                (unsigned long long)s_uvc_out_t.bytes,
                (unsigned int)s_uvc_out_t.timeouts,
                (unsigned int)s_uvc_out_t.errors);
    }

    tkl_log_output("xuvc out: close took %u ms (overlay %u)\r\n",
                   (unsigned int)((uint32_t)tkl_system_get_millisecond() - t0),
                   (unsigned int)overlay_ms);

    os_memset(&s_uvc_out_t, 0, sizeof(s_uvc_out_t));
    return ret;
}

/**
 * @brief xuvc record - dump one of the live streams to the card.
 *
 * @param[in] kind UVC_REC_H264 or UVC_REC_MJPEG
 * @param[in] sec  how long to capture
 * @param[in] path destination file
 *
 * @note H264 is a bare Annex-B elementary stream (`ffplay f.h264`); MJPEG is
 *       JPEGs back to back, which ffmpeg reads as `-f mjpeg`.
 *
 * @note MJPEG in a push mode turns the mjpeg_cb tee on - that callback is the
 *       only place the device bitstream still exists once the decoder has
 *       consumed it.
 *
 * @return OPRT_OK once capture has started
 */
static OPERATE_RET test_uvc_record(uvc_rec_kind_t kind, uint32_t sec, const char *path)
{
    OPERATE_RET ret;

    if (s_uvc_out_t.opened != true) {
        tkl_log_output("xuvc is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }
    if (s_uvc_out_t.rec_kind != (uint8_t)UVC_REC_NONE || test_record_busy() == TRUE) {
        tkl_log_output("xuvc record already running\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (kind == UVC_REC_H264) {
        if ((s_uvc_out_t.kind & TKL_UVC_OUT_H264) == 0U) {
            tkl_log_output("xuvc record: no h264 output; open with 'h264' or 'display+h264'\r\n");
            return OPRT_NOT_SUPPORTED;
        }
    } else {
        /* Pull FRAME mode decodes inside get_frame() and hands out NV12 only -
         * by then the bitstream is gone, so there is nothing to record. */
        if (s_uvc_out_t.push != true && (s_uvc_out_t.kind & TKL_UVC_OUT_MJPEG) == 0U) {
            tkl_log_output("xuvc record: mjpeg needs 'open mjpeg' or a push mode\r\n");
            return OPRT_NOT_SUPPORTED;
        }
    }

    /* Same ring as H264. MJPEG runs ~951 KB/s and still peaks at ~95 KB of it:
     * the ring absorbs card stalls, not steady-state backlog. */
    ret = test_record_start("xuvc", path, sec, 0U);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (kind == UVC_REC_MJPEG && s_uvc_out_t.push == true) {
        s_uvc_out_t.tee_mjpeg = true;
    }
    /* last, so no feeder sees a kind the recorder is not ready for */
    s_uvc_out_t.rec_kind = (uint8_t)kind;
    return OPRT_OK;
}

static void test_uvc_out_status_dump(void)
{
    uint32_t ms;

    if (s_uvc_out_t.opened != true) {
        tkl_log_output("xuvc out: not open\r\n");
        return;
    }

    ms = (uint32_t)tkl_system_get_millisecond() - s_uvc_out_t.start_ms;

    if (s_uvc_out_t.push == true) {
        tkl_log_output("xuvc out: kind=%s decodes=%u display=%u h264=%u %llu bytes in %u ms\r\n",
                (s_uvc_out_t.kind == (TKL_UVC_OUT_DISPLAY | TKL_UVC_OUT_H264)) ? "display+h264"
                    : ((s_uvc_out_t.kind == TKL_UVC_OUT_DISPLAY) ? "display" : "h264"),
                (unsigned int)s_uvc_out_t.decodes,
                (unsigned int)s_uvc_out_t.display_frames,
                (unsigned int)s_uvc_out_t.stream_frames,
                (unsigned long long)s_uvc_out_t.stream_bytes,
                (unsigned int)ms);
        tkl_log_output("  timeouts=%u errors=%u\r\n",
                (unsigned int)s_uvc_out_t.timeouts,
                (unsigned int)s_uvc_out_t.errors);
        test_record_status_dump();
        return;
    }

    tkl_log_output("xuvc out: kind=%s frames=%u %ux%u last=%u bytes luma=%u\r\n",
            (s_uvc_out_t.kind == TKL_UVC_OUT_FRAME) ? "frame" : "mjpeg",
            (unsigned int)s_uvc_out_t.frames,
            (unsigned int)s_uvc_out_t.last_w,
            (unsigned int)s_uvc_out_t.last_h,
            (unsigned int)s_uvc_out_t.last_size,
            (unsigned int)s_uvc_out_t.last_luma);
    tkl_log_output("  total=%llu bytes in %u ms, timeouts=%u errors=%u\r\n",
            (unsigned long long)s_uvc_out_t.bytes,
            (unsigned int)ms,
            (unsigned int)s_uvc_out_t.timeouts,
            (unsigned int)s_uvc_out_t.errors);
    test_record_status_dump();
}

/**
 * @brief "xuvc reopen [n]" - time n close+open cycles at the current settings.
 *
 * @details
 * The baseline: a full teardown and rebuild, device enumeration included.
 * Compare against "xuvc restart", which pauses and resumes instead - the gap is
 * the USB enumeration init() takes off the restart path, most of the cycle here.
 *
 * No settle delay between the close and the next open, on purpose: a restart
 * that needs one is itself a finding.
 *
 * @param[in] rounds how many cycles; 0 is treated as 1
 * @return OPRT_OK when every round completed
 */
static OPERATE_RET test_uvc_reopen(uint32_t rounds)
{
    uint32_t worst_close = 0U;
    uint32_t worst_open = 0U;
    uint32_t best = 0xFFFFFFFFU;
    uint64_t total = 0U;
    uint32_t i;

    if (s_uvc_last_open.valid != true) {
        tkl_log_output("xuvc reopen: nothing opened yet, run 'xuvc open ...' first\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }
    if (s_uvc_out_t.opened != true) {
        tkl_log_output("xuvc reopen: not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }
    if (rounds == 0U) {
        rounds = 1U;
    }

    for (i = 0U; i < rounds; i++) {
        uint32_t t;
        uint32_t close_ms;
        uint32_t open_ms;
        OPERATE_RET ret;

        t = (uint32_t)tkl_system_get_millisecond();
        (void)test_uvc_out_close();
        close_ms = (uint32_t)tkl_system_get_millisecond() - t;

        t = (uint32_t)tkl_system_get_millisecond();
        ret = test_uvc_out_open(s_uvc_last_open.kind,
                                s_uvc_last_open.width,
                                s_uvc_last_open.height,
                                s_uvc_last_open.fps,
                                s_uvc_last_open.port,
                                s_uvc_last_open.drop_num);
        open_ms = (uint32_t)tkl_system_get_millisecond() - t;

        if (ret != OPRT_OK) {
            tkl_log_output("xuvc reopen: round %u/%u failed to reopen: %d "
                           "(close %u ms)\r\n",
                           (unsigned int)(i + 1U), (unsigned int)rounds, ret,
                           (unsigned int)close_ms);
            return ret;
        }

        tkl_log_output("xuvc reopen %u/%u: close %u ms + open %u ms = %u ms\r\n",
                       (unsigned int)(i + 1U), (unsigned int)rounds,
                       (unsigned int)close_ms, (unsigned int)open_ms,
                       (unsigned int)(close_ms + open_ms));

        if (close_ms > worst_close) {
            worst_close = close_ms;
        }
        if (open_ms > worst_open) {
            worst_open = open_ms;
        }
        if ((close_ms + open_ms) < best) {
            best = close_ms + open_ms;
        }
        total += (uint64_t)(close_ms + open_ms);
    }

    tkl_log_output("xuvc reopen: %u rounds, cycle min %u avg %u ms, "
                   "worst close %u worst open %u\r\n",
                   (unsigned int)rounds,
                   (unsigned int)best,
                   (unsigned int)(total / rounds),
                   (unsigned int)worst_close,
                   (unsigned int)worst_open);
    return OPRT_OK;
}

/**
 * @brief "xuvc restart [n]" - time n stop+start cycles, the init/start payoff.
 *
 * @details
 * The counterpart to "xuvc reopen": same loop, but it pauses and resumes instead
 * of tearing the device down and enumerating it again.
 *
 * The consumer thread is stopped and restarted around each cycle, because
 * tkl_uvc_stop() cannot rescue a caller already parked inside decode_once().
 * That join is part of the reported "stop", which is why the number here is
 * larger than the one tkl_uvc_stop() logs.
 *
 * @param[in] rounds how many cycles; 0 is treated as 1
 * @return OPRT_OK when every round completed
 */
/**
 * @brief Park the pipeline: join our consumer, then tkl_uvc_stop().
 *
 * @param[out] ms milliseconds the pause cost, join included; may be NULL
 *
 * @note The join has to come first and its cost belongs to the caller:
 *       tkl_uvc_stop() cannot rescue a thread already parked inside
 *       decode_once()/get_frame() - only that thread can leave. This is why the
 *       number reported here is larger than the one tkl_uvc_stop() logs.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
static OPERATE_RET test_uvc_pause(uint32_t *ms)
{
    uint32_t t = (uint32_t)tkl_system_get_millisecond();
    OPERATE_RET ret;

    s_uvc_out_t.run = false;
    if (s_uvc_out_t.done != NULL) {
        (void)rtos_get_semaphore(&s_uvc_out_t.done, UVC_OUT_JOIN_TIMEOUT);
    }
    ret = tkl_uvc_stop();
    s_uvc_out_t.paused = true;

    if (ms != NULL) {
        *ms = (uint32_t)tkl_system_get_millisecond() - t;
    }
    return ret;
}

/**
 * @brief Resume: tkl_uvc_start(), re-base the fps window, respawn the consumer.
 *
 * @param[out] ms milliseconds tkl_uvc_start() itself cost; may be NULL
 *
 * @note Only tkl_uvc_start() is timed. The thread respawn afterwards is our own
 *       cost and would flatter nothing to include.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
static OPERATE_RET test_uvc_resume(uint32_t *ms)
{
    uint32_t t = (uint32_t)tkl_system_get_millisecond();
    OPERATE_RET ret;

    ret = tkl_uvc_start();
    if (ms != NULL) {
        *ms = (uint32_t)tkl_system_get_millisecond() - t;
    }
    if (ret != OPRT_OK) {
        return ret;
    }

    /* Re-base the fps window: the paused stretch is not streaming time. */
    s_uvc_out_t.fps_mark_ms = (uint32_t)tkl_system_get_millisecond();
    s_uvc_out_t.fps_mark_frames = s_uvc_out_t.decodes;

    s_uvc_out_t.run = true;
    ret = rtos_create_thread(&s_uvc_out_t.thread,
                             BEKEN_DEFAULT_WORKER_PRIORITY,
                             "uvc_out",
                             (beken_thread_function_t)(s_uvc_out_t.push
                                                           ? test_uvc_out_decode_worker
                                                           : test_uvc_out_worker),
                             UVC_OUT_THREAD_STACK,
                             NULL);
    if (ret != OPRT_OK) {
        s_uvc_out_t.run = false;
        return ret;
    }

    s_uvc_out_t.paused = false;
    return OPRT_OK;
}

static OPERATE_RET test_uvc_restart(uint32_t rounds)
{
    uint32_t worst_stop = 0U;
    uint32_t worst_start = 0U;
    uint32_t best = 0xFFFFFFFFU;
    uint64_t total = 0U;
    uint32_t i;

    if (s_uvc_out_t.opened != true) {
        tkl_log_output("xuvc restart: not open, run 'xuvc open ...' first\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }
    if (rounds == 0U) {
        rounds = 1U;
    }

    for (i = 0U; i < rounds; i++) {
        uint32_t t;
        uint32_t stop_ms;
        uint32_t start_ms;
        uint32_t mark;
        OPERATE_RET ret;

        /* Let the stream actually deliver before pausing it - see
         * UVC_RESTART_SETTLE_FRAMES. Bounded, so a dead pipeline still makes
         * progress through the rounds instead of hanging the CLI. */
        mark = s_uvc_out_t.decodes;
        t = (uint32_t)tkl_system_get_millisecond();
        while (s_uvc_out_t.decodes < (mark + UVC_RESTART_SETTLE_FRAMES) &&
               ((uint32_t)tkl_system_get_millisecond() - t) < UVC_RESTART_SETTLE_MS) {
            rtos_delay_milliseconds(10);
        }

        /* exactly what "xuvc stop" / "xuvc start" do, so the two commands and
         * this loop can never drift apart */
        (void)test_uvc_pause(&stop_ms);

        ret = test_uvc_resume(&start_ms);
        if (ret != OPRT_OK) {
            tkl_log_output("xuvc restart: round %u/%u failed to resume: %d "
                           "(stop %u ms)\r\n",
                           (unsigned int)(i + 1U), (unsigned int)rounds, ret,
                           (unsigned int)stop_ms);
            return ret;
        }

        tkl_log_output("xuvc restart %u/%u: stop %u ms + start %u ms = %u ms\r\n",
                       (unsigned int)(i + 1U), (unsigned int)rounds,
                       (unsigned int)stop_ms, (unsigned int)start_ms,
                       (unsigned int)(stop_ms + start_ms));

        if (stop_ms > worst_stop) {
            worst_stop = stop_ms;
        }
        if (start_ms > worst_start) {
            worst_start = start_ms;
        }
        if ((stop_ms + start_ms) < best) {
            best = stop_ms + start_ms;
        }
        total += (uint64_t)(stop_ms + start_ms);
    }

    tkl_log_output("xuvc restart: %u rounds, cycle min %u avg %u ms, "
                   "worst stop %u worst start %u\r\n",
                   (unsigned int)rounds,
                   (unsigned int)best,
                   (unsigned int)(total / rounds),
                   (unsigned int)worst_stop,
                   (unsigned int)worst_start);
    return OPRT_OK;
}

void cli_tuya_uvc_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    uint16_t width = UVC_TEST_DEFAULT_WIDTH;
    uint16_t height = UVC_TEST_DEFAULT_HEIGHT;
    uint8_t fps = UVC_TEST_DEFAULT_FPS;
    uint8_t port = UVC_TEST_DEFAULT_PORT;
    uint8_t drop_num = 0U;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xuvc <open mjpeg|frame|display|h264|display+h264 [w] [h] [fps] [port] [drop]|"
                 "close|stop|start|reopen [n]|restart [n]|stats|mjpeg on|off|bitrate <bps>|idr|osd <display|encode> <time [off]|wifi <0..4|off>|clear|caps>|"
                 "record [h264|mjpeg] [sec] [file]|record stop>");
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "open") == 0) {
        uint32_t kind = TKL_UVC_OUT_MJPEG;

        if (argc >= 3) {
            if (os_strcmp(argv[2], "frame") == 0) {
                kind = TKL_UVC_OUT_FRAME;
            } else if (os_strcmp(argv[2], "display") == 0) {
                kind = TKL_UVC_OUT_DISPLAY;
            } else if (os_strcmp(argv[2], "h264") == 0) {
                kind = TKL_UVC_OUT_H264;
            } else if (os_strcmp(argv[2], "display+h264") == 0) {
                kind = TKL_UVC_OUT_DISPLAY | TKL_UVC_OUT_H264;
            } else if (os_strcmp(argv[2], "mjpeg") != 0) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "Usage: xuvc open <mjpeg|frame|display|h264|display+h264> "
                         "[w] [h] [fps] [port] [drop]");
                return;
            }
        }
        if (argc >= 4) {
            width = (uint16_t)os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            height = (uint16_t)os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            fps = (uint8_t)os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            port = (uint8_t)os_strtoul(argv[6], NULL, 10);
        }
        if (argc >= 8) {
            drop_num = (uint8_t)os_strtoul(argv[7], NULL, 10);
        }

        /* Same as xmipi cam: the clock element's font pointer is an object, not a
         * compile-time constant, so it is NULL until this runs. Without it
         * "xuvc osd" fails with "slot 0 has no drawable content". Idempotent. */
        osd_assets_init();
        ret = test_uvc_out_open(kind, width, height, fps, port, drop_num);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc open %s %s (%d), %ux%u fps=%u port=%u drop=%u\r\n",
                 (argc >= 3) ? argv[2] : "mjpeg",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 width,
                 height,
                 (unsigned int)fps,
                 (unsigned int)port,
                 (unsigned int)drop_num);
    } else if (os_strcmp(cmd, "close") == 0) {
        ret = test_uvc_out_close();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc close %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);

    } else if (os_strcmp(cmd, "reopen") == 0) {
        uint32_t rounds = (argc >= 3) ? (uint32_t)os_strtoul(argv[2], NULL, 10) : 3U;

        ret = test_uvc_reopen(rounds);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc reopen %s (%d)\r\n",
                 (ret == OPRT_OK) ? "done" : "failed", ret);

    } else if (os_strcmp(cmd, "restart") == 0) {
        uint32_t rounds = (argc >= 3) ? (uint32_t)os_strtoul(argv[2], NULL, 10) : 3U;

        ret = test_uvc_restart(rounds);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc restart %s (%d)\r\n",
                 (ret == OPRT_OK) ? "done" : "failed", ret);
    } else if (os_strcmp(cmd, "stop") == 0) {
        uint32_t ms = 0U;

        if (s_uvc_out_t.opened != true) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xuvc stop: not open, run 'xuvc open ...' first\r\n");
            return;
        }
        if (s_uvc_out_t.paused == true) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc stop: already stopped\r\n");
            return;
        }

        /* Clear here, not in close(): stop() is the last moment the pipeline is
         * still composing, and a cleared slot only reaches the panel through a
         * frame drawn after it. Once paused, set_osd refuses with -23 and
         * nothing would repaint anyway - so "osd time -> stop -> close" would
         * otherwise leave the overlay burnt into the last frame. Deliberately
         * outside the timed section: the wait is test-layer policy and must not
         * pollute the stop number that "restart" is compared against. */
        test_uvc_out_osd_clear_settle();

        ret = test_uvc_pause(&ms);
        /* the reply buffer is ~136 bytes; anything longer is silently cut, so
         * the guidance goes out through the log and only the verdict here */
        tkl_log_output("xuvc stop: device stays enumerated; set_encode/set_osd/"
                       "get_osd_caps/get_frame must now return -23. Try "
                       "'xuvc osd display time', 'xuvc bitrate 800000', 'xuvc idr'\r\n");
        snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc stop %s (%d), %u ms\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret, (unsigned int)ms);

    } else if (os_strcmp(cmd, "start") == 0) {
        uint32_t ms = 0U;

        if (s_uvc_out_t.opened != true) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xuvc start: not open, run 'xuvc open ...' first\r\n");
            return;
        }
        if (s_uvc_out_t.paused != true) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc start: already streaming\r\n");
            return;
        }

        ret = test_uvc_resume(&ms);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc start %s (%d), %u ms\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret, (unsigned int)ms);

    } else if (os_strcmp(cmd, "record") == 0) {
        uvc_rec_kind_t rk;
        uint32_t sec = 5U;
        int ai = 2;
        const char *path;

        if (argc > 2 && os_strcmp(argv[2], "stop") == 0) {
            s_uvc_out_t.rec_kind = (uint8_t)UVC_REC_NONE;
            test_record_stop();
            snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc record stopped\r\n");
            return;
        }

        /* h264 when the encoder is up, else the device bitstream */
        rk = ((s_uvc_out_t.kind & TKL_UVC_OUT_H264) != 0U) ? UVC_REC_H264 : UVC_REC_MJPEG;
        if (argc > 2 && os_strcmp(argv[2], "h264") == 0) {
            rk = UVC_REC_H264;
            ai = 3;
        } else if (argc > 2 && os_strcmp(argv[2], "mjpeg") == 0) {
            rk = UVC_REC_MJPEG;
            ai = 3;
        }

        path = (rk == UVC_REC_H264) ? TEST_SDCARD_MOUNT_POINT "/xuvc_rec.h264"
                                    : TEST_SDCARD_MOUNT_POINT "/xuvc_rec.mjpeg";
        if (argc > ai) {
            sec = os_strtoul(argv[ai], NULL, 10);
        }
        if (argc > (ai + 1)) {
            path = argv[ai + 1];
        }

        ret = test_uvc_record(rk, sec, path);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc record %s %s (%d)\r\n",
                 (rk == UVC_REC_H264) ? "h264" : "mjpeg",
                 (ret == OPRT_OK) ? "started" : "failed", ret);

    } else if (os_strcmp(cmd, "mjpeg") == 0) {
        bool on = !(argc >= 3 && os_strcmp(argv[2], "off") == 0);

        /* an mjpeg capture in a push mode is fed by this tee and nothing else */
        if (on == false && s_uvc_out_t.push == true &&
            s_uvc_out_t.rec_kind == (uint8_t)UVC_REC_MJPEG) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xuvc mjpeg: an mjpeg record is running off this tee; "
                     "stop it first ('xuvc record stop')\r\n");
            return;
        }

        /* let mjpeg_cb keep the device bitstream, alongside display / h264 */
        s_uvc_out_t.tee_mjpeg = on;
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc mjpeg tee %s (takes effect immediately, push modes only)\r\n",
                 s_uvc_out_t.tee_mjpeg ? "on" : "off");

    } else if (os_strcmp(cmd, "bitrate") == 0) {
        TKL_MEDIA_ENC_CTRL_T ctrl;

        os_memset(&ctrl, 0, sizeof(ctrl));
        ctrl.bitrate = (argc >= 3) ? (uint32_t)os_strtoul(argv[2], NULL, 10) : 1000000U;
        ret = tkl_uvc_set_encode(&ctrl);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc bitrate %u %s (%d)\r\n",
                 (unsigned int)ctrl.bitrate,
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "idr") == 0) {
        TKL_MEDIA_ENC_CTRL_T ctrl;

        os_memset(&ctrl, 0, sizeof(ctrl));
        ctrl.force_idr = TRUE;
        s_uvc_idr_trace = 5U;   /* arm before the call, so a fast IDR is not missed */
        ret = tkl_uvc_set_encode(&ctrl);
        if (ret != OPRT_OK) {
            /* nothing was requested, so disarm: left armed it would fire on the
             * next "xuvc start" and print an I frame that is just the encoder's
             * first frame, reading exactly like an IDR that finally landed */
            s_uvc_idr_trace = 0U;
        }
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc idr %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "osd") == 0) {
        TKL_UVC_CHN_E chn;
        const char *act = (argc > 3) ? argv[3] : NULL;
        const char *sub = (argc > 4) ? argv[4] : NULL;

        if (argc < 4) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: %s\r\n", TEST_UVC_OSD_USAGE);
            return;
        }

        chn = (os_strcmp(argv[2], "encode") == 0) ? TKL_UVC_CHN_ENCODE
                                                  : TKL_UVC_CHN_DISPLAY;

        if (os_strcmp(act, "caps") == 0) {
            TKL_OSD_CAPS_T caps;

            ret = tkl_uvc_get_osd_caps(chn, &caps);
            if (ret == OPRT_OK) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xuvc osd %s caps: slots=%u text=%u rotate=%u\r\n",
                         argv[2],
                         (unsigned int)caps.max_slots,
                         (unsigned int)caps.support_text,
                         (unsigned int)caps.support_rotate);
            } else {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xuvc osd %s caps failed (%d)\r\n", argv[2], ret);
            }
            return;
        }

        if (os_strcmp(act, "clear") == 0) {
            test_overlay_clock_stop((UINT8_T)chn);
            ret = test_uvc_osd_clock_clear(chn);
            /* the icon lives in its own slot and would survive otherwise */
            (void)test_uvc_osd_wifi(chn, 0U, FALSE);
        } else if (os_strcmp(act, "time") == 0) {
            if (sub != NULL && os_strcmp(sub, "off") == 0) {
                test_overlay_clock_stop((UINT8_T)chn);
                ret = test_uvc_osd_clock_clear(chn);
            } else {
                ret = test_overlay_clock_start((UINT8_T)chn, test_uvc_osd_submit,
                                               (VOID_T *)(uintptr_t)chn);
            }
        } else if (os_strcmp(act, "wifi") == 0) {
            if (sub != NULL && os_strcmp(sub, "off") == 0) {
                ret = test_uvc_osd_wifi(chn, 0U, FALSE);
            } else {
                UINT8_T level = (sub != NULL) ? (UINT8_T)os_strtoul(sub, NULL, 10) : 4U;

                ret = test_uvc_osd_wifi(chn, level, TRUE);
            }
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: %s\r\n", TEST_UVC_OSD_USAGE);
            return;
        }

        snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc osd %s %s%s%s %s (%d)\r\n",
                 argv[2], act, (sub != NULL) ? " " : "", (sub != NULL) ? sub : "",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "stats") == 0) {
        test_uvc_out_status_dump();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc stats ok, see logs\r\n");
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xuvc <open mjpeg|frame|display|h264|display+h264 [w] [h] [fps] [port] [drop]|"
                 "close|stop|start|reopen [n]|restart [n]|stats|mjpeg on|off|bitrate <bps>|idr|osd <display|encode> <time [off]|wifi <0..4|off>|clear|caps>|"
                 "record [h264|mjpeg] [sec] [file]|record stop>");
    }
}
