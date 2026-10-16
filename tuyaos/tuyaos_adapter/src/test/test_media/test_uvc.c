#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <components/bk_frame_buffer.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>

#include "test_osd.h"

typedef OPERATE_RET (*TEST_UVC_OVERLAY_SUBMIT_CB)(CONST CHAR_T *text, VOID_T *arg);

extern VOID_T test_record_feed(CONST UINT8_T *data, UINT32_T size);
extern OPERATE_RET test_record_start(CONST CHAR_T *tag, CONST CHAR_T *path,
                                     UINT32_T sec, UINT32_T ring_bytes);
extern VOID_T test_record_stop(VOID_T);
extern BOOL_T test_record_busy(VOID_T);
extern VOID_T test_record_status_dump(VOID_T);
#include "cli_tuya_test.h"
#include "tkl_media_buf.h"
#include "tkl_mipi_dsi.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_thread.h"
#include "tkl_uvc.h"

extern OPERATE_RET test_overlay_clock_start(TUYA_CAMERA_FRAME_CHANNEL_E channel,
                                            TEST_UVC_OVERLAY_SUBMIT_CB cb,
                                            VOID_T *arg);
extern VOID_T test_overlay_clock_stop(TUYA_CAMERA_FRAME_CHANNEL_E channel);

/* Channel-bit view of "xuvc open <mode>": bit = TUYA_CAMERA_FRAME_CHANNEL_x.
 * ch0 parks the device JPEG (was FRAME), ch1 is the on-demand decoded NV12,
 * ch2 is the display push (was PIXEL), ch3 the h264 push (was STREAM). */
#define UVC_KIND_CH0  (1U << TUYA_CAMERA_FRAME_CHANNEL_0)
#define UVC_KIND_CH1  (1U << TUYA_CAMERA_FRAME_CHANNEL_1)
#define UVC_KIND_CH2  (1U << TUYA_CAMERA_FRAME_CHANNEL_2)
#define UVC_KIND_CH3  (1U << TUYA_CAMERA_FRAME_CHANNEL_3)

#define UVC_TEST_DEFAULT_WIDTH       640U
#define UVC_TEST_DEFAULT_HEIGHT      480U
#define UVC_TEST_DEFAULT_FPS         15U
#define UVC_TEST_DEFAULT_PORT        1U
#define UVC_TEST_LOG_INTERVAL        30U

/* get_frame(ch1) decodes one parked MJPEG on demand, so it waits for a slot
 * between the ch2/ch3 jobs rather than for the next pipeline frame. */
#define UVC_PULL_TIMEOUT_MS          2000U

extern BOOL_T test_mipi_lcd_is_open(void);

/* ===========================================================================
 * Peripheral-oriented facade (design spec P4a): xuvc open / close / stats
 *
 * PIXEL arrives through pixel_cb and STREAM through stream_cb, both borrowed
 * - the adapter owns the decode task that feeds them. FRAME is pull-only now:
 * the adapter parks delivered frames in a ready queue, and a small pump thread
 * in this file drains tkl_uvc_get_frame() - it is the whole consumer in the
 * FRAME-only mode and the "xuvc mjpeg" tee in a PIXEL/STREAM one. This test
 * owns nothing but the pump, the handlers, the recorder and the CLI.
 * =========================================================================== */

#define UVC_OUT_LOG_INTERVAL   30U
/* Panel geometry for xuvc open display, matching what xmipi dsi open_gpu sets
 * up; the facade scales and rotates the decoded picture into it. */
#define UVC_OUT_DISPLAY_ROTATE 90U
#define UVC_OUT_DISPLAY_W      640U
#define UVC_OUT_DISPLAY_H      1024U
#define UVC_OUT_ENCODE_GOP     40
/* "xuvc restart" waits for this many delivered frames before pausing, so that
 * each round measures pausing a stream that is actually running. Without it the
 * loop stops the pipeline before the device has delivered anything and measures
 * UVC stream-start latency, not the cost of stopping. Board-measured: 300 ms of
 * first-frame latency turned a 33 ms stop into a 494 ms one. */
/* one grammar, shared by the two usage sites and both reject paths - and the
 * same shape as "xmipi cam osd", so the two facades read alike on the wire */
#define TEST_UVC_OSD_USAGE \
    "xuvc osd <display|encode> <time [off]|wifi <0..4|off>|clear>"

/* How long close() waits for one overlay-free frame to be composited. Four
 * frames at 19 fps; it returns as soon as the frame lands, so this is only the
 * ceiling for a pipeline that has already stopped delivering. */
#define UVC_OSD_CLEAR_SETTLE_MS   200U

#define UVC_RESTART_SETTLE_FRAMES 5U
#define UVC_RESTART_SETTLE_MS     1500U

/* Which stream "xuvc record" is writing. H264 comes off the encode callback;
 * MJPEG off the frame pump, which drains the device bitstream in every mode. */
typedef enum {
    UVC_REC_NONE = 0,
    UVC_REC_H264,
    UVC_REC_MJPEG,
} uvc_rec_kind_t;

typedef struct {
    bool opened;
    /* TRUE between "xuvc stop" and "xuvc start": the device is still enumerated
     * and the pools still exist, but the adapter is not streaming. */
    bool paused;
    bool push;                  /* PIXEL / STREAM use the processing pipeline */
    bool tee_mjpeg;             /* push mode: also keep the raw bitstream the pump drains */
    uint8_t rec_kind;           /* uvc_rec_kind_t */
    uint32_t tee_frames;
    uint64_t tee_bytes;
    uint32_t kind;
    uint32_t display_frames;    /* pixel_cb arrivals (GPU-composited frames) */
    uint32_t stream_frames;     /* stream_cb arrivals (H.264 frames) */
    uint64_t stream_bytes;
    uint32_t frames;            /* pump arrivals in the FRAME-only mode */
    uint64_t bytes;
    uint32_t last_size;
    uint16_t last_w;
    uint16_t last_h;
    uint32_t start_ms;

    /* fps is measured from here, not from start_ms, and re-based on every
     * tkl_uvc_start(). start_ms is wall time since open and therefore includes
     * the paused stretches of "xuvc restart"; dividing the cumulative frame
     * count by it reported 7.9 fps on a pipeline that was running at 19 - a
     * reporting artifact that looked exactly like a real throughput collapse. */
    uint32_t fps_mark_ms;
    uint32_t fps_mark_frames;
} test_uvc_out_ctx_t;

static test_uvc_out_ctx_t s_uvc_out_t;

/* One delivered frame, for pacing and settle decisions: a composited pixel or
 * an encoded frame in a PIXEL/STREAM configuration (the two arrive in
 * parallel), a raw pump frame in the FRAME-only one. */
static uint32_t test_uvc_out_delivered(void)
{
    return (s_uvc_out_t.push == true)
               ? (s_uvc_out_t.display_frames + s_uvc_out_t.stream_frames)
               : s_uvc_out_t.frames;
}

/* Which counter the pacing window sits on: the pixel side when a display
 * consumer exists (pixel_cb logs the rate), the raw frame counter otherwise. */
static uint32_t test_uvc_out_fps_base(void)
{
    if (s_uvc_out_t.push == true &&
        (s_uvc_out_t.kind & UVC_KIND_CH2) != 0U) {
        return s_uvc_out_t.display_frames;
    }
    return s_uvc_out_t.frames;
}

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
} s_uvc_last_open;

/* "xuvc idr" arms this; see the identical note in test_camera.c. */
static volatile uint8_t s_uvc_idr_trace;

/**
 * @brief Composed picture from the GPU. Runs on the GPU worker thread.
 *
 * @note Hands ownership straight on to the panel: tkl_mipi_dsi_flush() takes the
 *       buffer and its free_cb together, so the frame is released once the DPU
 *       is done scanning it. Identical to test_camera.c's - which is the point,
 *       the two peripherals deliver the same contract.
 */
/* tkl_mipi_dsi_flush_frame_t releases through a UINT8_T payload callback; a
 * ch2 frame is one block (see TUYA_MEDIA_BUF_HDR), so recover the struct and
 * let its own free_cb do the work. */
static VOID_T test_uvc_display_release(UINT8_T *payload)
{
    TUYA_MEDIA_BUF_T *frame = (TUYA_MEDIA_BUF_T *)(VOID_T *)(payload - TUYA_MEDIA_BUF_HDR);

    if (frame != NULL && frame->free_cb != NULL) {
        frame->free_cb(frame);
    }
}

static VOID_T test_uvc_out_pixel_cb(CONST TUYA_MEDIA_BUF_T *buf)
{
    tkl_mipi_dsi_flush_frame_t flush;
    OPERATE_RET ret;

    if (buf == NULL || buf->buffer == NULL) {
        return;
    }

    s_uvc_out_t.display_frames++;

    if ((s_uvc_out_t.display_frames % UVC_OUT_LOG_INTERVAL) == 0U) {
        uint32_t ms = (uint32_t)tkl_system_get_millisecond() - s_uvc_out_t.fps_mark_ms;
        uint32_t f = s_uvc_out_t.display_frames - s_uvc_out_t.fps_mark_frames;

        tkl_log_output("xuvc display: %u frames, h264=%u mjpeg=%u(%llu B), %u.%u fps\r\n",
                (unsigned int)s_uvc_out_t.display_frames,
                (unsigned int)s_uvc_out_t.stream_frames,
                (unsigned int)s_uvc_out_t.tee_frames,
                (unsigned long long)s_uvc_out_t.tee_bytes,
                (unsigned int)((ms != 0U) ? (f * 1000U / ms) : 0U),
                (unsigned int)((ms != 0U) ? ((f * 10000U / ms) % 10U) : 0U));
    }

    if (test_mipi_lcd_is_open() != TRUE) {
        if (buf->free_cb != NULL) {
            buf->free_cb((TUYA_MEDIA_BUF_T *)(uintptr_t)buf);
        }
        return;
    }

    os_memset(&flush, 0, sizeof(flush));
    flush.buffer = buf->buffer;
    flush.free_cb = test_uvc_display_release;

    ret = tkl_mipi_dsi_flush(&flush);
    if (ret != OPRT_OK) {
        /* a flush that got past validation released the frame itself */
        if (ret == OPRT_INVALID_PARM || ret == OPRT_RESOURCE_NOT_READY) {
            if (buf->free_cb != NULL) {
                buf->free_cb((TUYA_MEDIA_BUF_T *)(uintptr_t)buf);
            }
        }
    }
}

/**
 * @brief Transcoded H264. Runs on the encoder worker thread, borrowed buffer.
 */
static VOID_T test_uvc_out_stream_cb(CONST TUYA_MEDIA_BUF_T *buf)
{
    if (buf == NULL || buf->buffer == NULL) {
        return;
    }

    s_uvc_out_t.stream_frames++;
    s_uvc_out_t.stream_bytes += buf->length;

    if (s_uvc_out_t.rec_kind == (uint8_t)UVC_REC_H264) {
        /* copies and returns; the card is the writer thread's problem */
        test_record_feed(buf->buffer, buf->length);
    }

    if (s_uvc_idr_trace > 0U) {
        s_uvc_idr_trace--;
        tkl_log_output("xuvc idr trace[%u]: %s size=%u format=%u\r\n",
                (unsigned int)s_uvc_out_t.stream_frames,
                (buf->format == TUYA_FRAME_FMT_H264_I) ? "I" : "P",
                (unsigned int)buf->length,
                (unsigned int)buf->format);
    }

    if ((s_uvc_out_t.stream_frames % UVC_OUT_LOG_INTERVAL) == 0U) {
        tkl_log_output("xuvc h264[%u]: %s size=%u total=%llu display=%u\r\n",
                (unsigned int)s_uvc_out_t.stream_frames,
                (buf->format == TUYA_FRAME_FMT_H264_I) ? "I" : "P",
                (unsigned int)buf->length,
                (unsigned long long)s_uvc_out_t.stream_bytes,
                (unsigned int)s_uvc_out_t.display_frames);
    }
}

/* ch0 is pull-only: the adapter parks each device JPEG in an internal pool and
 * tkl_uvc_get_frame(CH0) pops one. A small pump thread drains it. In the
 * ch0-only mode it IS the consumer (counters and pacing live here); in a
 * ch2/ch3 configuration it only serves the "xuvc mjpeg" tee and an mjpeg
 * recording, both gated by tee_mjpeg. The returned buffer carries its own
 * release function - hand the frame back through buf->free_cb(buf). */

#define UVC_PUMP_TIMEOUT_MS 200U
#define UVC_PUMP_STACK      (3U * 1024U)
#define UVC_PUMP_STOP_WAIT_MS 1000U

static TKL_THREAD_HANDLE s_uvc_pump_task;
static volatile bool s_uvc_pump_stop;

static void test_uvc_frame_pump(void *arg)
{
    TKL_THREAD_HANDLE self = NULL;

    (void)arg;
    (void)tkl_thread_get_id(&self);

    while (s_uvc_pump_stop != true) {
        TUYA_MEDIA_BUF_T *buf = NULL;
        OPERATE_RET ret = tkl_uvc_get_frame(TUYA_CAMERA_FRAME_CHANNEL_0, &buf,
                                            UVC_PUMP_TIMEOUT_MS);

        if (ret != OPRT_OK || buf == NULL) {
            /* idle stretch: stopped, or the device has not delivered yet */
            continue;
        }

        if (s_uvc_out_t.push == true) {
            if (s_uvc_out_t.tee_mjpeg == true) {
                s_uvc_out_t.tee_frames++;
                s_uvc_out_t.tee_bytes += buf->length;

                if (s_uvc_out_t.rec_kind == (uint8_t)UVC_REC_MJPEG) {
                    /* copies and returns; the card is the writer thread's problem */
                    test_record_feed(buf->buffer, buf->length);
                }
            }
        } else {
            /* ch0-only mode: the pacing that used to live in the get_frame() loop. */
            s_uvc_out_t.frames++;
            s_uvc_out_t.bytes += buf->length;
            s_uvc_out_t.last_size = buf->length;
            s_uvc_out_t.last_w = buf->width;
            s_uvc_out_t.last_h = buf->height;

            /* ch0-only has no tee to gate on - the pump IS the consumer, so an
             * mjpeg recording is fed straight from here. Without this the
             * frames are counted and dropped: 'xuvc record mjpeg' wrote 0
             * frames while the rate line showed a healthy 20fps stream. */
            if (s_uvc_out_t.rec_kind == (uint8_t)UVC_REC_MJPEG) {
                test_record_feed(buf->buffer, buf->length);
            }

            if ((s_uvc_out_t.frames % UVC_OUT_LOG_INTERVAL) == 0U) {
                uint32_t ms = (uint32_t)tkl_system_get_millisecond() - s_uvc_out_t.fps_mark_ms;
                uint32_t f = s_uvc_out_t.frames - s_uvc_out_t.fps_mark_frames;

                tkl_log_output("xuvc out: %u frames %ux%u %u bytes %u.%u fps\r\n",
                        (unsigned int)s_uvc_out_t.frames,
                        (unsigned int)buf->width,
                        (unsigned int)buf->height,
                        (unsigned int)buf->length,
                        (unsigned int)((ms != 0U) ? (f * 1000U / ms) : 0U),
                        (unsigned int)((ms != 0U) ? ((f * 10000U / ms) % 10U) : 0U));
            }
        }

        buf->free_cb(buf);
    }

    s_uvc_pump_task = NULL;
    tkl_thread_release(self);
}

static OPERATE_RET test_uvc_pump_start(void)
{
    s_uvc_pump_stop = false;
    return tkl_thread_create(&s_uvc_pump_task, "uvc_pump", UVC_PUMP_STACK,
                             TKL_THREAD_PRI_NORMAL, test_uvc_frame_pump, NULL);
}

static void test_uvc_pump_stop(void)
{
    uint32_t waited = 0U;

    if (s_uvc_pump_task == NULL) {
        return;
    }

    s_uvc_pump_stop = true;
    while (s_uvc_pump_task != NULL && waited < UVC_PUMP_STOP_WAIT_MS) {
        tkl_system_sleep(20U);
        waited += 20U;
    }
}

static VOID_T test_uvc_out_event_cb(TUYA_CAMERA_EVENT_E event, VOID_T *arg)
{
    (void)arg;
    if (event == TUYA_CAMERA_EVENT_CONNECT) {
        tkl_log_output("xuvc out: device connected\r\n");
    } else if (event == TUYA_CAMERA_EVENT_DISCONNECT) {
        tkl_log_output("xuvc out: device gone\r\n");
    }
}

/**
 * @brief Push the shared clock string to one uvc overlay channel.
 * @note Same content and same helper as xmipi cam - only the submit call differs,
 *       which is the whole point of the two facades having one OSD contract.
 */
static OPERATE_RET test_uvc_osd_submit(CONST CHAR_T *text, VOID_T *arg)
{
    TUYA_CAMERA_FRAME_CHANNEL_E channel = (TUYA_CAMERA_FRAME_CHANNEL_E)(uintptr_t)arg;
    TUYA_OSD_T osd;

    test_osd_set_clock(text);

    os_memset(&osd, 0, sizeof(osd));

    if (channel == TUYA_CAMERA_FRAME_CHANNEL_2) {
        osd.dst_x = TEST_OSD_CLOCK_DST_X;
        osd.dst_y = TEST_OSD_CLOCK_DST_Y;
    } else {
        /* The encoder sees the un-rotated decoder output. */
        osd.dst_x = 2U;
        osd.dst_y = 2U;
    }
    osd.rotate = 0U;
    osd.data = test_osd_clock_bitmap(&osd.w, &osd.h);
    if (osd.data == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    return tkl_uvc_set_osd(channel, &osd);
}

/**
 * @brief Take the clock sprite away, leaving every other slot alone.
 *
 * @param[in] chn which channel's compositor
 *
 * @note Stopping the refresh task only stops the redraw - the last string stays
 *       composited until its coordinate is cleared, so "time off" has to do both.
 *       A NULL data pointer clears that business coordinate.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
static OPERATE_RET test_uvc_osd_clock_clear(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    TUYA_OSD_T osd;

    os_memset(&osd, 0, sizeof(osd));
    if (channel == TUYA_CAMERA_FRAME_CHANNEL_2) {
        osd.dst_x = TEST_OSD_CLOCK_DST_X;
        osd.dst_y = TEST_OSD_CLOCK_DST_Y;
    } else {
        osd.dst_x = 2U;
        osd.dst_y = 2U;
    }
    osd.data = NULL;

    return tkl_uvc_set_osd(channel, &osd);
}

/**
 * @brief Put the wifi icon in its own slot, or take it away.
 *
 * @param[in] chn   which channel's compositor
 * @param[in] level 0..4 signal bars; anything above 3 shows full
 * @param[in] on    FALSE clears the slot instead of drawing
 *
 * @note Separate coordinate from the clock so the two can be raised and cleared
 *       independently; sharing one coordinate would mean redrawing the clock
 *       every time the bars change.
 * @note This submits business-composed ARGB8888 bitmaps; TKL does not rasterize
 *       text or manage overlay content.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
static OPERATE_RET test_uvc_osd_wifi(TUYA_CAMERA_FRAME_CHANNEL_E channel, UINT8_T level, BOOL_T on)
{
    TUYA_OSD_T osd;

    /* vcenc DMA cannot reach flash XIP: the const rssi bitmaps live in rodata,
     * so stage the selected one in RAM before handing it to the encoder. The
     * stage lives in SRAM's dynamic heap: this is a CPU-writes -> HW-reads
     * buffer, and PSRAM would need a write-through channel for that (only 4
     * per PSRAM chip, the display frame rotation already holds two). SRAM
     * stores are non-cacheable and land directly, and both consumers reach
     * SRAM (GPU 0x28 alias, vcenc). */
    static UINT8_T *s_wifi_osd_ram;

    os_memset(&osd, 0, sizeof(osd));

    if (on == TRUE) {
        if (s_wifi_osd_ram == NULL) {
            s_wifi_osd_ram = (UINT8_T *)hsram_malloc(
                    TEST_OSD_WIFI_BITMAP_W * TEST_OSD_WIFI_BITMAP_H * 4U);
            if (s_wifi_osd_ram == NULL) {
                tkl_log_output("xuvc osd wifi: no SRAM for RAM stage\r\n");
                return OPRT_MALLOC_FAILED;
            }
        }
        os_memcpy(s_wifi_osd_ram, test_osd_wifi_bitmap(level),
                  TEST_OSD_WIFI_BITMAP_W * TEST_OSD_WIFI_BITMAP_H * 4U);
        osd.data = s_wifi_osd_ram;
        osd.w = TEST_OSD_WIFI_BITMAP_W;
        osd.h = TEST_OSD_WIFI_BITMAP_H;

        if (channel == TUYA_CAMERA_FRAME_CHANNEL_2) {
            osd.dst_x = TEST_OSD_WIFI_DST_X;
            osd.dst_y = TEST_OSD_WIFI_DST_Y;
        } else {
            osd.dst_x = 2U;
            osd.dst_y = 72U;
        }
    } else if (channel == TUYA_CAMERA_FRAME_CHANNEL_2) {
        osd.dst_x = TEST_OSD_WIFI_DST_X;
        osd.dst_y = TEST_OSD_WIFI_DST_Y;
    } else {
        osd.dst_x = 2U;
        osd.dst_y = 72U;
    }

    osd.rotate = 0U;
    return tkl_uvc_set_osd(channel, &osd);
}

/**
 * @brief Parse a "+"-joined channel word list into a kind bitmask.
 *
 * "mjpeg"/"frame" -> ch0 (parked JPEG), "nv12" -> ch1 (on-demand decoded NV12),
 * "display" -> ch2, "h264" -> ch3. Split by hand rather than with strtok(): the
 * SDK has no os_strtok, and a '+' split over a small stack buffer needs neither
 * heap nor global state.
 *
 * @return bitmask; 0 when a word is unknown or the list is malformed
 */
static uint32_t test_uvc_kind_parse(const char *name)
{
    uint32_t kind = 0U;
    uint32_t i = 0U;

    if (name == NULL) {
        return 0U;
    }

    while (name[i] != '\0') {
        char word[16];
        uint32_t n = 0U;

        while (name[i] != '\0' && name[i] != '+' && n < (sizeof(word) - 1U)) {
            word[n++] = name[i++];
        }
        word[n] = '\0';
        if (name[i] == '+') {
            i++;
        }
        if (n == 0U) {
            return 0U;                  /* empty word: "a++b", leading or trailing '+' */
        }

        if (os_strcmp(word, "mjpeg") == 0 || os_strcmp(word, "frame") == 0) {
            kind |= UVC_KIND_CH0;
        } else if (os_strcmp(word, "nv12") == 0) {
            kind |= UVC_KIND_CH1;
        } else if (os_strcmp(word, "display") == 0) {
            kind |= UVC_KIND_CH2;
        } else if (os_strcmp(word, "h264") == 0) {
            kind |= UVC_KIND_CH3;
        } else {
            return 0U;                  /* unknown word */
        }
    }

    return kind;
}

static OPERATE_RET test_uvc_out_open(uint32_t kind,
                                     uint16_t width,
                                     uint16_t height,
                                     uint8_t fps,
                                     uint8_t port)
{
    TUYA_MEDIA_OUTPUT_CFG_T outputs[4];
    TUYA_UVC_CFG_T cfg;
    UINT8_T output_num = 0U;
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

    os_memset(outputs, 0, sizeof(outputs));
    os_memset(&cfg, 0, sizeof(cfg));
    cfg.port = port;
    cfg.uvc_cfg.width = width;
    cfg.uvc_cfg.height = height;
    cfg.uvc_cfg.fps = fps;
    cfg.uvc_cfg.format = TUYA_FRAME_FMT_JPEG;

    /* ch0 parks the device JPEG in every configuration: the pump consumes it
     * directly in the ch0-only mode and serves the "xuvc mjpeg" tee in a
     * ch2/ch3 one. */
    outputs[output_num].channel = TUYA_CAMERA_FRAME_CHANNEL_0;
    outputs[output_num].width = width;
    outputs[output_num].height = height;
    outputs[output_num].format = TUYA_FRAME_FMT_JPEG;
    output_num++;

    /* ch1: decoded NV12, pull-only. No cb on purpose - with one it would be a
     * FRAME-sink push, which init() only allows when ch2/ch3 are absent (the
     * flexa ring owns the decoder then). Pull mode coexists: get_frame(ch1)
     * decodes one parked JPEG on demand and the decoder serves ch2/ch3 first. */
    if ((kind & UVC_KIND_CH1) != 0U) {
        outputs[output_num].channel = TUYA_CAMERA_FRAME_CHANNEL_1;
        outputs[output_num].width = width;
        outputs[output_num].height = height;
        outputs[output_num].format = TUYA_FRAME_FMT_NV12;
        output_num++;
    }

    if ((kind & (UVC_KIND_CH2 | UVC_KIND_CH3)) != 0U) {
        s_uvc_out_t.push = true;

        /* ch2: GPU -> panel. Only when display was actually asked for: h264-only
         * is a valid configuration - the camera facade's "xmipi cam open h264"
         * is exactly that - and the panel chain must not be dragged in by a kind
         * bit that never mentioned it. The panel is the target when it is on, so
         * the geometry is the panel's, rotate 90 to match how xmipi opens it. */
        if ((kind & UVC_KIND_CH2) != 0U) {
            outputs[output_num].channel = TUYA_CAMERA_FRAME_CHANNEL_2;
            outputs[output_num].cb = test_uvc_out_pixel_cb;
            outputs[output_num].rotate = UVC_OUT_DISPLAY_ROTATE;
            outputs[output_num].width = UVC_OUT_DISPLAY_W;
            outputs[output_num].height = UVC_OUT_DISPLAY_H;
            outputs[output_num].format = TUYA_FRAME_FMT_ARGB8888;
            output_num++;
        }

        if ((kind & UVC_KIND_CH3) != 0U) {
            /* ch3: the encoder has no scaler, so it encodes the source size */
            outputs[output_num].channel = TUYA_CAMERA_FRAME_CHANNEL_3;
            outputs[output_num].cb = test_uvc_out_stream_cb;
            outputs[output_num].width = width;
            outputs[output_num].height = height;
            outputs[output_num].format = TUYA_FRAME_FMT_H264;
            outputs[output_num].gop = UVC_OUT_ENCODE_GOP;
            outputs[output_num].level = TUYA_ENCODE_BITRATE_LEVEL_MEDIUM;
            output_num++;
        }
    }

    cfg.outputs = outputs;
    cfg.output_num = output_num;
    cfg.event_cb = test_uvc_out_event_cb;

    ret = tkl_uvc_init(&cfg);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_uvc_start();
    if (ret != OPRT_OK) {
        (void)tkl_uvc_deinit();
        return ret;
    }

    ret = test_uvc_pump_start();
    if (ret != OPRT_OK) {
        (void)tkl_uvc_deinit();
        return ret;
    }

    s_uvc_out_t.start_ms = (uint32_t)tkl_system_get_millisecond();
    s_uvc_out_t.fps_mark_ms = s_uvc_out_t.start_ms;
    s_uvc_out_t.fps_mark_frames = test_uvc_out_fps_base();

    s_uvc_out_t.opened = true;

    s_uvc_last_open.valid = true;
    s_uvc_last_open.kind = kind;
    s_uvc_last_open.width = width;
    s_uvc_last_open.height = height;
    s_uvc_last_open.fps = fps;
    s_uvc_last_open.port = port;
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

    (void)test_uvc_osd_clock_clear(TUYA_CAMERA_FRAME_CHANNEL_2);
    (void)test_uvc_osd_wifi(TUYA_CAMERA_FRAME_CHANNEL_2, 0U, FALSE);
    (void)test_uvc_osd_clock_clear(TUYA_CAMERA_FRAME_CHANNEL_3);
    (void)test_uvc_osd_wifi(TUYA_CAMERA_FRAME_CHANNEL_3, 0U, FALSE);

    mark = s_uvc_out_t.display_frames;
    t = (uint32_t)tkl_system_get_millisecond();
    while (s_uvc_out_t.display_frames == mark &&
           ((uint32_t)tkl_system_get_millisecond() - t) < UVC_OSD_CLEAR_SETTLE_MS) {
        rtos_delay_milliseconds(5);
    }
}

/**
 * @brief Pull ch1 (decoded NV12) frames and report the rate.
 *
 * @param[in] count frames to pull back to back; 0 or 1 means one frame
 *
 * @note ch1 is not a continuously produced stream: each get_frame() decodes one
 *       parked MJPEG on demand, in a slot between the ch2/ch3 jobs. The rate
 *       reported here is the on-demand decode rate, not a pipeline frame rate.
 *
 * @return OPRT_OK on success
 */
static OPERATE_RET test_uvc_out_pull(uint32_t count)
{
    TUYA_MEDIA_BUF_T *frame = NULL;
    OPERATE_RET ret;
    uint32_t i;
    uint32_t t0;
    uint32_t elapsed;

    if (s_uvc_out_t.opened != true) {
        tkl_log_output("xuvc is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if ((s_uvc_out_t.kind & UVC_KIND_CH1) == 0U) {
        tkl_log_output("xuvc frame: ch1 (nv12) is not configured; re-open with an "
                       "'nv12' kind word, e.g. 'xuvc open display+h264+nv12 640 480 15 1'\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (count == 0U) {
        count = 1U;
    }

    t0 = (uint32_t)tkl_system_get_millisecond();
    for (i = 0U; i < count; i++) {
        ret = tkl_uvc_get_frame(TUYA_CAMERA_FRAME_CHANNEL_1, &frame,
                                UVC_PULL_TIMEOUT_MS);
        if (ret != OPRT_OK) {
            tkl_log_output("xuvc frame %u/%u failed: %d after %u ms\r\n",
                           (unsigned int)(i + 1U), (unsigned int)count, ret,
                           (unsigned int)((uint32_t)tkl_system_get_millisecond() - t0));
            return ret;
        }

        /* one geometry line is enough; the loop is about the rate */
        if (i == 0U) {
            tkl_log_output("xuvc frame: %ux%u fmt=%u %u bytes\r\n",
                           (unsigned int)frame->width, (unsigned int)frame->height,
                           (unsigned int)frame->format, (unsigned int)frame->length);
        }

        frame->free_cb(frame);
    }
    elapsed = (uint32_t)tkl_system_get_millisecond() - t0;

    if (count > 1U) {
        tkl_log_output("xuvc frame: %u frames in %u ms, avg %u ms\r\n",
                       (unsigned int)count, (unsigned int)elapsed,
                       (unsigned int)(elapsed / count));
    }
    return OPRT_OK;
}

static OPERATE_RET test_uvc_out_close(void)
{
    uint32_t t0 = (uint32_t)tkl_system_get_millisecond();
    uint32_t overlay_ms;
    OPERATE_RET ret;

    if (s_uvc_out_t.opened != true) {
        return OPRT_OK;
    }

    /* The pump first: it blocks inside tkl_uvc_get_frame(), which has to be
     * out of the adapter before deinit() retires the frame queue it reads. */
    test_uvc_pump_stop();

    /* Overlay refreshers next: their threads must stop before the pipeline
     * they submit to goes away. */
    test_overlay_clock_stop(TUYA_CAMERA_FRAME_CHANNEL_2);
    test_overlay_clock_stop(TUYA_CAMERA_FRAME_CHANNEL_3);
    /* the clock task only tests stop_req once per refresh period, so parking it
     * costs up to TEST_OVERLAY_REFRESH_MS - report it rather than let it hide
     * inside "close felt slow" */
    overlay_ms = (uint32_t)tkl_system_get_millisecond() - t0;

    /* before the pipeline goes: the clear only becomes visible through a frame
     * the pipeline is still delivering */
    test_uvc_out_osd_clear_settle();

    /* deinit() stops the producer, joins the adapter's decode task and retires
     * the pools - every callback is quiet when it returns. */
    ret = tkl_uvc_deinit();

    /* Only here are all the feeders certainly quiet: the pump was joined
     * above, and pixel_cb/stream_cb run on vendor worker threads inside the
     * pipeline deinit() just took down. It frees the ring, so the recorder
     * must not race a test_record_feed() still in flight. */
    s_uvc_out_t.rec_kind = (uint8_t)UVC_REC_NONE;
    test_record_stop();

    if (s_uvc_out_t.push == true) {
        tkl_log_output("xuvc out: closed, display %u, h264 %u frames %llu bytes, "
                "mjpeg tee %u(%llu B)\r\n",
                (unsigned int)s_uvc_out_t.display_frames,
                (unsigned int)s_uvc_out_t.stream_frames,
                (unsigned long long)s_uvc_out_t.stream_bytes,
                (unsigned int)s_uvc_out_t.tee_frames,
                (unsigned long long)s_uvc_out_t.tee_bytes);
    } else {
        tkl_log_output("xuvc out: closed, %u frames %llu bytes\r\n",
                (unsigned int)s_uvc_out_t.frames,
                (unsigned long long)s_uvc_out_t.bytes);
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
 * @note MJPEG in a processing-pipeline configuration keeps the raw input via
 *       the frame pump's tee; that pump is the only place the device bitstream
 *       remains available after the decoder consumes it.
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
        if ((s_uvc_out_t.kind & UVC_KIND_CH3) == 0U) {
            tkl_log_output("xuvc record: no h264 output; open with 'h264' or 'display+h264'\r\n");
            return OPRT_NOT_SUPPORTED;
        }
    }
    /* MJPEG needs no guard: the pump drains the device bitstream in every mode
     * - ch0 is configured alongside ch2/ch3 at open(), the ch0-only one lives
     * entirely on the pump. In a push mode record() turns the tee flag on so
     * the pump routes frames here. */

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
        tkl_log_output("xuvc out: kind=%s display=%u h264=%u %llu bytes in %u ms, "
                "mjpeg tee %u(%llu B)\r\n",
                (s_uvc_out_t.kind == (UVC_KIND_CH2 | UVC_KIND_CH3)) ? "display+h264"
                    : ((s_uvc_out_t.kind == UVC_KIND_CH2) ? "display" : "h264"),
                (unsigned int)s_uvc_out_t.display_frames,
                (unsigned int)s_uvc_out_t.stream_frames,
                (unsigned long long)s_uvc_out_t.stream_bytes,
                (unsigned int)ms,
                (unsigned int)s_uvc_out_t.tee_frames,
                (unsigned long long)s_uvc_out_t.tee_bytes);
        test_record_status_dump();
        return;
    }

    tkl_log_output("xuvc out: kind=frame frames=%u %ux%u last=%u bytes\r\n",
            (unsigned int)s_uvc_out_t.frames,
            (unsigned int)s_uvc_out_t.last_w,
            (unsigned int)s_uvc_out_t.last_h,
            (unsigned int)s_uvc_out_t.last_size);
    tkl_log_output("  total=%llu bytes in %u ms\r\n",
            (unsigned long long)s_uvc_out_t.bytes,
            (unsigned int)ms);
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
                                s_uvc_last_open.port);
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
 * of tearing the device down and enumerating it again. The adapter owns the
 * decode task, so tkl_uvc_stop() reports the whole pause - task join included.
 *
 * @param[in] rounds how many cycles; 0 is treated as 1
 * @return OPRT_OK when every round completed
 */
/**
 * @brief Park the pipeline: tkl_uvc_stop(), which also joins the adapter's
 *        internal decode task.
 *
 * @param[out] ms milliseconds the pause cost; may be NULL
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
static OPERATE_RET test_uvc_pause(uint32_t *ms)
{
    uint32_t t = (uint32_t)tkl_system_get_millisecond();
    OPERATE_RET ret;

    ret = tkl_uvc_stop();
    s_uvc_out_t.paused = true;

    if (ms != NULL) {
        *ms = (uint32_t)tkl_system_get_millisecond() - t;
    }
    return ret;
}

/**
 * @brief Resume: tkl_uvc_start() and re-base the fps window.
 *
 * @param[out] ms milliseconds tkl_uvc_start() itself cost; may be NULL
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

    /* Re-base the pacing window: the paused stretch is not streaming time. */
    s_uvc_out_t.fps_mark_ms = (uint32_t)tkl_system_get_millisecond();
    s_uvc_out_t.fps_mark_frames = test_uvc_out_fps_base();

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
        mark = test_uvc_out_delivered();
        t = (uint32_t)tkl_system_get_millisecond();
        while (test_uvc_out_delivered() < (mark + UVC_RESTART_SETTLE_FRAMES) &&
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

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xuvc <open mjpeg|frame|nv12|display|h264 ('+' joins them) [w] [h] [fps] [port]|"
                 "close|stop|start|reopen [n]|restart [n]|stats|info|mjpeg on|off|bitrate <bps>|idr|gop <n>|frame [n]|switch <ch> <w> <h>|cancel <ch>|resume <ch>|osd <display|encode> <time [off]|wifi <0..4|off>|clear>|"
                 "record [h264|mjpeg] [sec] [file]|record stop>");
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "open") == 0) {
        uint32_t kind = UVC_KIND_CH0;

        if (argc >= 3) {
            kind = test_uvc_kind_parse(argv[2]);
            if (kind == 0U) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "Usage: xuvc open <mjpeg|frame|nv12|display|h264> "
                         "(join with '+' to combine) [w] [h] [fps] [port]");
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

        /* Same as xmipi cam: the clock element's font pointer is an object, not a
         * compile-time constant, so it is NULL until this runs. Without it
         * "xuvc osd" fails with "slot 0 has no drawable content". Idempotent. */
        test_osd_init();
        ret = test_uvc_out_open(kind, width, height, fps, port);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc open %s %s (%d), %ux%u fps=%u port=%u\r\n",
                 (argc >= 3) ? argv[2] : "mjpeg",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 width,
                 height,
                 (unsigned int)fps,
                 (unsigned int)port);
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
        tkl_log_output("xuvc stop: device stays enumerated; set_bitrate/set_osd/"
                       "must now return -23. Try "
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
        rk = ((s_uvc_out_t.kind & UVC_KIND_CH3) != 0U) ? UVC_REC_H264 : UVC_REC_MJPEG;
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

        /* let the pump keep the device bitstream, alongside display / h264 */
        s_uvc_out_t.tee_mjpeg = on;
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc mjpeg tee %s (takes effect immediately, push modes only)\r\n",
                 s_uvc_out_t.tee_mjpeg ? "on" : "off");

    } else if (os_strcmp(cmd, "bitrate") == 0) {
        UINT32_T bitrate = (argc >= 3) ? (uint32_t)os_strtoul(argv[2], NULL, 10) : 1000000U;

        ret = tkl_uvc_set_bitrate(bitrate);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc bitrate %u %s (%d)\r\n",
                 (unsigned int)bitrate,
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "idr") == 0) {
        s_uvc_idr_trace = 5U;   /* arm before the call, so a fast IDR is not missed */
        ret = tkl_uvc_force_idr();
        if (ret != OPRT_OK) {
            /* nothing was requested, so disarm: left armed it would fire on the
             * next "xuvc start" and print an I frame that is just the encoder's
             * first frame, reading exactly like an IDR that finally landed */
            s_uvc_idr_trace = 0U;
        }
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc idr %s (%d)\r\n", (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "switch") == 0) {
        TUYA_CAMERA_FRAME_CHANNEL_E chn;
        uint16_t w;
        uint16_t h;

        if (argc < 5) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xuvc switch <ch> <w> <h> (1=decoded NV12, 2=display; "
                     "ch0/ch3 rejected - the USB format is negotiated at open and "
                     "the encoder has no scaler)\r\n");
            return;
        }

        chn = (TUYA_CAMERA_FRAME_CHANNEL_E)os_strtoul(argv[2], NULL, 10);
        if ((uint32_t)chn < (uint32_t)TUYA_CAMERA_FRAME_CHANNEL_1 ||
            (uint32_t)chn > (uint32_t)TUYA_CAMERA_FRAME_CHANNEL_3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xuvc switch: ch%u invalid (1..3; ch0 is the parking pool)\r\n",
                     (unsigned int)os_strtoul(argv[2], NULL, 10));
            return;
        }

        w = (uint16_t)os_strtoul(argv[3], NULL, 10);
        h = (uint16_t)os_strtoul(argv[4], NULL, 10);
        ret = tkl_uvc_set_pp(chn, w, h);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc switch %s (%d), ch%u %ux%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret, (unsigned int)chn, w, h);

    } else if (os_strcmp(cmd, "gop") == 0) {
        UINT32_T gop;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xuvc gop <n> (0 = default; takes effect at the next start - "
                     "a running encoder refuses it)\r\n");
            return;
        }

        gop = os_strtoul(argv[2], NULL, 10);
        ret = tkl_uvc_set_gop(gop);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc gop %s (%d), %u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret, (unsigned int)gop);

    } else if (os_strcmp(cmd, "cancel") == 0 || os_strcmp(cmd, "resume") == 0) {
        /* One grammar for both: the two are inverses, and reading them side by
         * side beats two near-identical branches. */
        BOOL_T is_cancel = (os_strcmp(cmd, "cancel") == 0) ? TRUE : FALSE;
        TUYA_CAMERA_FRAME_CHANNEL_E chn;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xuvc %s <ch> (1=decoded NV12, 2=display, 3=h264; "
                     "ch0 is the parking pool)\r\n", cmd);
            return;
        }

        chn = (TUYA_CAMERA_FRAME_CHANNEL_E)os_strtoul(argv[2], NULL, 10);
        if ((uint32_t)chn < (uint32_t)TUYA_CAMERA_FRAME_CHANNEL_1 ||
            (uint32_t)chn > (uint32_t)TUYA_CAMERA_FRAME_CHANNEL_3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xuvc %s: ch%u invalid (1..3; ch0 is the parking pool)\r\n",
                     cmd, (unsigned int)os_strtoul(argv[2], NULL, 10));
            return;
        }

        ret = (is_cancel == TRUE) ? tkl_uvc_cancel_channel(chn)
                                  : tkl_uvc_resume_channel(chn);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc %s %s (%d), ch%u\r\n",
                 cmd, (ret == OPRT_OK) ? "ok" : "failed", ret, (unsigned int)chn);

    } else if (os_strcmp(cmd, "frame") == 0) {
        uint32_t count = 1U;

        if (argc > 2) {
            count = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_uvc_out_pull(count);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc frame %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "osd") == 0) {
        TUYA_CAMERA_FRAME_CHANNEL_E channel;
        const char *act = (argc > 3) ? argv[3] : NULL;
        const char *sub = (argc > 4) ? argv[4] : NULL;

        if (argc < 4) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: %s\r\n", TEST_UVC_OSD_USAGE);
            return;
        }

        channel = (os_strcmp(argv[2], "encode") == 0) ? TUYA_CAMERA_FRAME_CHANNEL_3
                                                      : TUYA_CAMERA_FRAME_CHANNEL_2;

        if (os_strcmp(act, "clear") == 0) {
            test_overlay_clock_stop(channel);
            ret = test_uvc_osd_clock_clear(channel);
            /* the icon lives in its own slot and would survive otherwise */
            (void)test_uvc_osd_wifi(channel, 0U, FALSE);
        } else if (os_strcmp(act, "time") == 0) {
            if (sub != NULL && os_strcmp(sub, "off") == 0) {
                test_overlay_clock_stop(channel);
                ret = test_uvc_osd_clock_clear(channel);
            } else {
                ret = test_overlay_clock_start(channel, test_uvc_osd_submit,
                                               (VOID_T *)(uintptr_t)channel);
            }
        } else if (os_strcmp(act, "wifi") == 0) {
            if (sub != NULL && os_strcmp(sub, "off") == 0) {
                ret = test_uvc_osd_wifi(channel, 0U, FALSE);
            } else {
                UINT8_T level = (sub != NULL) ? (UINT8_T)os_strtoul(sub, NULL, 10) : 4U;

                ret = test_uvc_osd_wifi(channel, level, TRUE);
            }
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: %s\r\n", TEST_UVC_OSD_USAGE);
            return;
        }

        snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc osd %s %s%s%s %s (%d)\r\n",
                 argv[2], act, (sub != NULL) ? " " : "", (sub != NULL) ? sub : "",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "info") == 0) {
        /* Enumeration snapshot, init() not required: one power-on probe of what
         * is plugged into the port right now. */
        TUYA_UVC_INFO_T info;

        os_memset(&info, 0, sizeof(info));
        ret = tkl_uvc_get_info(&info);
        if (ret != OPRT_OK) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xuvc info: no UVC device ready (%d)\r\n", ret);
        } else {
            UINT8_T fi;

            tkl_log_output("xuvc info: vid=0x%04x pid=0x%04x usb=0x%04x dev=0x%04x "
                           "speed=%u bulk=%u mps=%u bcd_uvc=0x%04x\r\n",
                    (unsigned int)info.vid, (unsigned int)info.pid,
                    (unsigned int)info.bcd_usb, (unsigned int)info.bcd_device,
                    (unsigned int)info.speed, (unsigned int)info.is_bulk,
                    (unsigned int)info.ep_mps, (unsigned int)info.bcd_uvc);
            tkl_log_output("xuvc info: %s %s s/n %s, intf c=%u d=%u\r\n",
                    info.manufacturer, info.product, info.serial,
                    (unsigned int)info.ctrl_intf, (unsigned int)info.data_intf);
            for (fi = 0U; fi < info.format_num; fi++) {
                UINT8_T fr;

                tkl_log_output("xuvc info: format %u: %s, %u frame size(s)\r\n",
                        (unsigned int)fi,
                        (info.formats[fi].format == TUYA_FRAME_FMT_H264) ? "h264" : "mjpeg",
                        (unsigned int)info.formats[fi].frame_num);
                for (fr = 0U; fr < info.formats[fi].frame_num; fr++) {
                    tkl_log_output("xuvc info:   %ux%u\r\n",
                            (unsigned int)info.formats[fi].frames[fr].width,
                            (unsigned int)info.formats[fi].frames[fr].height);
                }
            }
            snprintf(pcWriteBuffer, xWriteBufferLen, "xuvc info ok, see logs\r\n");
        }

    } else if (os_strcmp(cmd, "stats") == 0) {
        test_uvc_out_status_dump();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xuvc stats ok, see logs\r\n");
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xuvc <open mjpeg|frame|nv12|display|h264 ('+' joins them) [w] [h] [fps] [port]|"
                 "close|stop|start|reopen [n]|restart [n]|stats|info|mjpeg on|off|bitrate <bps>|idr|gop <n>|frame [n]|switch <ch> <w> <h>|cancel <ch>|resume <ch>|osd <display|encode> <time [off]|wifi <0..4|off>|clear>|"
                 "record [h264|mjpeg] [sec] [file]|record stop>");
    }
}
