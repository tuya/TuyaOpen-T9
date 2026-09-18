/**
 * @file test_camera_mipi.c
 * @brief CLI for the MIPI camera output facade ("xmipi cam")
 *
 * @details
 * Exercises tkl_mipi_csi_init()/start() - two calls that bring up the sensor,
 * the display pipeline and the encoder together. Its own CLI rather than a
 * branch of `xmipi camera`, so it is obvious which layer a test exercises.
 *
 * @version 0.1
 * @date 2026-08-27
 * @copyright Copyright (c) Tuya Inc.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <driver/gpio_types.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>

#include "test_osd.h"
#include "cli_tuya_test.h"
#include "tkl_mipi_csi.h"
#include "tkl_fs.h"
#include "tkl_media_buf.h"
#include "tkl_memory.h"
#include "tkl_mipi_dsi.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_thread.h"

typedef OPERATE_RET (*TEST_CAM_OVERLAY_SUBMIT_CB)(CONST CHAR_T *text, VOID_T *arg);

/* shared helpers; declarations live here because the test files have no
 * common header (same pattern as test_uvc.c) */
extern VOID_T test_record_feed(CONST UINT8_T *data, UINT32_T size);
extern OPERATE_RET test_record_start(CONST CHAR_T *tag, CONST CHAR_T *path,
                                     UINT32_T sec, UINT32_T ring_bytes);
extern VOID_T test_record_stop(VOID_T);
extern VOID_T test_record_status_dump(VOID_T);
extern OPERATE_RET test_overlay_clock_start(TUYA_CAMERA_OUTPUT_TYPE_E type,
                                            TEST_CAM_OVERLAY_SUBMIT_CB cb,
                                            VOID_T *arg);
extern VOID_T test_overlay_clock_stop(TUYA_CAMERA_OUTPUT_TYPE_E type);

#define CAM_TEST_SENSOR_W      2304U
#define CAM_TEST_SENSOR_H      1296U
#define CAM_TEST_SENSOR_FPS    20U
/* matches MIPI_TEST_CAMERA_* in test_mipi.c - the driver default is RGGB10, and
 * against this BGGR sensor that comes out with red and blue exchanged */
#define CAM_TEST_SENSOR_IN_FMT ((UINT32_T)TUYA_FRAME_FMT_BGGR10)
/* MP output size, which xmipi cam also uses as the GPU destination size.
 *
 * With rotate 90/270 the vendor GPU treats dst_width/dst_height as the size
 * BEFORE rotation and hands the panel the swap of the two
 * (bk_gpu_ctlr_default.c, frame_done_blit: display_buffer.width =
 * output_height), so this pair is portrait for the landscape 1024x600 panel.
 *
 * 640 rather than the panel's 600: the GPU needs the width aligned, so the
 * composed buffer is wider than the panel and the panel shows the left 600 of
 * it. Do not "correct" this to 600.
 *
 * Height must stay a multiple of TUYA_MIPI_CSI_FLEXA_LINES (16) - the flexa DMA
 * walks it in flexa_lines steps. 1024 / 16 = 64. */
#define CAM_TEST_MP_W          640U
#define CAM_TEST_MP_H          1024U
/* the sensor is mounted rotated on this board - 0 puts the picture on its side */
#define CAM_TEST_DISPLAY_ROT   90U
#define CAM_TEST_GOP           40U
#define CAM_TEST_BITRATE       1500000U
#define CAM_TEST_STATUS_MS     5000U
#define CAM_TEST_FRAME_TIMEOUT_MS 1000U

/* One definition, used both for "no arguments" and for "unknown subcommand". */
#define XCAM_USAGE \
    "Usage: xmipi cam <open <mp_kinds> [w] [h] [sp_fmt] [spw] [sph]|close|\r\n" \
    "             stop|start|reopen [n]|restart [n]|frame|\r\n" \
    "             switch <w> <h> [mp|sp]|stress [loops]|bitrate <bps>|idr|\r\n" \
    "             osd <display|encode> <time [off|0|90|270]|wifi <0..4|off>|clear>|\r\n" \
    "             rotate [0|90|180|270]|\r\n" \
    "             record [sec] [file]|record stop|mirror [h v]|luminance|gray [0|1]|\r\n" \
    "             night [0|1]|mirror_test|luminance_test [n] [ms]|status>\r\n" \
    "  mp_kinds = frame|display|h264|display+h264   MP is the only flexa channel\r\n" \
    "  sp_fmt   = nv12|rgb888|yuyv|rgb565           giving one enables SP (frame mode)\r\n" \
    "  display needs the panel up first: xmipi dsi open_gpu <panel> 1\r\n" \
    "  e.g. xmipi cam open display+h264 640 1024                screen + encode, no SP\r\n" \
    "       xmipi cam open display+h264 640 1024 nv12 640 480   ... plus SP for snapshots\r\n" \
    "       xmipi cam open display+h264 640 1024 rgb888 640 480 ... plus SP for the face model\r\n"
/* one grammar, shared by the two reject paths inside the osd branch - the same
 * shape as TEST_UVC_OSD_USAGE, so the two facades read alike on the wire */
#define TEST_CAM_OSD_USAGE \
    "xmipi cam osd <display|encode> <time [off|0|90|270]|wifi <0..4|off>|clear>"

/* SP defaults when 'xmipi cam open ... sp' omits a size */
#define CAM_TEST_SP_W          640U
#define CAM_TEST_SP_H          480U

/* Same pins the xmipi camera command uses. */
/* Board wiring for the sc3336p on this module. Keep these identical to
 * MIPI_TEST_CAMERA_PIN_* in test_mipi.c - that set is board-proven, and an
 * invented value fails in two different ways: an out-of-range clk pin is rejected outright
 * by tkl_mipi_csi_cfg_validate(), and a wrong scl/sda/port gets past
 * validation only to fail later at sensor detect. */
#define CAM_TEST_PIN_SCL       GPIO_65
#define CAM_TEST_PIN_SDA       GPIO_64
#define CAM_TEST_I2C_ID        1U
#define CAM_TEST_PIN_RESET     GPIO_60
/* PWDN is not wired on this module; the vendor sensor drives reset/pwdn
 * active-high and the adapter validates active_level == HIGH, so both
 * control pins are declared with TUYA_GPIO_LEVEL_HIGH below. */
#define CAM_TEST_PIN_XCLK      GPIO_59

#define CAM_QUALITY_FRAME_TIMEOUT_MS      1000U
#define CAM_QUALITY_MIRROR_SETTLE_FRAMES  2U
#define CAM_QUALITY_MIRROR_EDGE_COLS      8U
#define CAM_QUALITY_LUMINANCE_DEFAULT_CNT 5U
#define CAM_QUALITY_LUMINANCE_DEFAULT_MS  200U

/* How long close() waits for one overlay-free frame to be composited. Four
 * frames at 20 fps; it returns as soon as the frame lands. */
#define CAM_OSD_CLEAR_SETTLE_MS           200U

typedef struct {
    bool opened;
    /** TRUE between "xmipi cam stop" and "xmipi cam start": sensor still
     *  powered and configured, adapter not streaming. Unlike xuvc there is no
     *  consumer thread to park - MIPI delivers through callbacks. */
    bool paused;
    uint32_t out_kinds;      /**< kinds placed on MP */
    bool     sp_on;          /**< SP enabled; it is always frame mode */
    uint32_t sp_format;      /**< TUYA_FRAME_FMT_E the SP channel was opened with */
    uint16_t sp_w;
    uint16_t sp_h;
    uint32_t display_frames;
    uint32_t stream_frames;
    uint64_t stream_bytes;
    uint32_t mark_ms;
} test_cam_ctx_t;

static test_cam_ctx_t s_cam;

/* What the last successful "xmipi cam open" asked for. Deliberately outside
 * test_cam_ctx_t: that struct is zeroed by close, and "xmipi cam reopen" has to
 * replay these parameters afterwards. */
static struct {
    bool     valid;
    uint32_t mp_kinds;
    uint16_t mp_w;
    uint16_t mp_h;
    uint32_t sp_fmt;
    uint16_t sp_w;
    uint16_t sp_h;
} s_cam_last_open;

/* ---------------------------------------------------------------------------
 * State shared with the other media CLIs: the geometry the camera was opened
 * with. "xmipi cam" is the only way to bring a MIPI camera up, so it owns it.
 * ------------------------------------------------------------------------- */

BOOL_T test_camera_is_open(void)
{
    return (s_cam.opened == true) ? TRUE : FALSE;
}

/**
 * @brief Geometry of the SP channel, for a test that wants to pull from it.
 *
 * @param[out] width, height, format  SP output as opened; any may be NULL
 * @param[out] timeout_ms             suggested read timeout; may be NULL
 *
 * @return OPRT_OK when the camera is open with SP on, OPRT_RESOURCE_NOT_READY
 *         otherwise
 */
OPERATE_RET test_camera_get_sp_info(UINT16_T *width, UINT16_T *height,
                                    UINT32_T *format, UINT32_T *timeout_ms)
{
    if (s_cam.opened != true || s_cam.sp_on != true) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (width != NULL) {
        *width = s_cam.sp_w;
    }
    if (height != NULL) {
        *height = s_cam.sp_h;
    }
    if (format != NULL) {
        *format = s_cam.sp_format;
    }
    if (timeout_ms != NULL) {
        *timeout_ms = CAM_TEST_FRAME_TIMEOUT_MS;
    }
    return OPRT_OK;
}

/* Display rotation, settable with "xmipi cam rotate". Kept outside test_cam_ctx_t on
 * purpose: test_cam_open() memsets that struct, and a rotation chosen before
 * open must survive into it. Which of 90 and 270 is upright depends on how the
 * sensor is mounted, and that is only answerable by looking at the panel. */
static uint16_t s_cam_rotate = CAM_TEST_DISPLAY_ROT;

static uint16_t s_cam_osd_rotate;

/* "xmipi cam idr" arms this; the encode callback then prints the next few frames
 * unconditionally. The 1-in-30 sampling below is far too coarse to catch a
 * single forced keyframe - it reported "ok" while proving nothing. */
static volatile uint8_t s_cam_idr_trace;

/* H264 capture to card. The ring, the writer thread and the write
 * granularity all live in test_record.c now - xuvc needs the same thing,
 * and the numbers in there were paid for once already. */

extern BOOL_T test_mipi_lcd_is_open(void);

/**
 * @brief Display sink: push the composed picture to the panel.
 *
 * @param[in] buf composed frame; ownership arrives with it
 *
 * @note Runs on the GPU worker thread. Handing buf->free_cb straight to the DSI
 *       layer is the whole point of carrying the release function inside the
 *       frame - this callback never has to know which pool the frame came from.
 */
static VOID_T test_cam_display_cb(CONST TUYA_MEDIA_BUF_T *buf)
{
    tkl_mipi_dsi_flush_frame_t flush;
    OPERATE_RET ret;

    if (buf == NULL || buf->data == NULL) {
        return;
    }

    s_cam.display_frames++;

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
 * @brief xmipi cam record - dump the raw H264 elementary stream to the card.
 *
 * @param[in] sec  how long to capture
 * @param[in] path destination file
 *
 * @note The result is a bare Annex-B elementary stream, not a container. It
 *       plays with `ffplay file.h264`; most players want
 *       `ffmpeg -f h264 -i file.h264 -c copy out.mp4` first.
 *
 * @note Everything below the validation is test_record.c's - the encode
 *       callback feeds it and never waits on the card.
 *
 * @return OPRT_OK once capture has started
 */
static OPERATE_RET test_cam_record(uint32_t sec, const char *path)
{
    if (s_cam.opened != true) {
        tkl_log_output("xmipi cam is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }
    if ((s_cam.out_kinds & TUYA_CAMERA_OUTPUT_STREAM) == 0U) {
        tkl_log_output("xmipi cam record: no h264 output; open with 'h264' or 'display+h264'\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    return test_record_start("xmipi cam", path, sec, 0U);
}

/**
 * @brief Encode sink: count the bitstream.
 * @param[in] buf encoded frame, borrowed for the call only
 */
static VOID_T test_cam_stream_cb(CONST TUYA_MEDIA_BUF_T *buf)
{
    if (buf == NULL || buf->data == NULL) {
        return;
    }

    s_cam.stream_frames++;
    s_cam.stream_bytes += buf->size;

    /* no-op unless a capture is running; the deadline lives in there too */
    test_record_feed(buf->data, buf->size);

    if (s_cam_idr_trace > 0U) {
        s_cam_idr_trace--;
        tkl_log_output("xmipi cam idr trace[%u]: %s size=%u format=%u\r\n",
                       (unsigned int)s_cam.stream_frames,
                       (buf->format == TUYA_FRAME_FMT_H264_I) ? "I" : "P",
                       (unsigned int)buf->size,
                       (unsigned int)buf->format);
    }

    if ((s_cam.stream_frames % 30U) == 0U) {
        tkl_log_output("xmipi cam stream[%u]: %s size=%u total=%llu\r\n",
                       (unsigned int)s_cam.stream_frames,
                       (buf->format == TUYA_FRAME_FMT_H264_I) ? "I" : "P",
                       (unsigned int)buf->size,
                       (unsigned long long)s_cam.stream_bytes);
    }
}

/**
 * @brief Parse the output selection word.
 * @param[in] name frame / display / h264 / display+h264
 * @return TUYA_CAMERA_OUTPUT_* bitmask, TUYA_CAMERA_OUTPUT_NONE when
 *         unrecognised
 */
static uint32_t test_cam_kinds_parse(const char *name)
{
    if (name == NULL) {
        return TUYA_CAMERA_OUTPUT_NONE;
    }
    if (os_strcmp(name, "frame") == 0) {
        return TUYA_CAMERA_OUTPUT_FRAME;
    }
    if (os_strcmp(name, "display") == 0) {
        return TUYA_CAMERA_OUTPUT_PIXEL;
    }
    if (os_strcmp(name, "h264") == 0) {
        return TUYA_CAMERA_OUTPUT_STREAM;
    }
    if (os_strcmp(name, "display+h264") == 0 || os_strcmp(name, "both") == 0) {
        return TUYA_CAMERA_OUTPUT_PIXEL | TUYA_CAMERA_OUTPUT_STREAM;
    }
    return TUYA_CAMERA_OUTPUT_NONE;
}

/**
 * @brief xmipi cam open - bring the whole pipeline up.
 *
 * @details
 * The two channels do different jobs and the arguments follow that. MP takes a
 * kinds bitmask because it is the only channel a flexa bond attaches to, so it
 * can drive the display, the encoder, or both. SP is always frame mode, so what
 * there is to choose about it is the pixel format: NV12 to JPEG-encode or
 * transform what you pull, RGB888 to hand it straight to the face model.
 *
 * @param[in] mp_kinds TUYA_CAMERA_OUTPUT_* bitmask for the main channel
 * @param[in] mp_w     MP output width
 * @param[in] mp_h     MP output height
 * @param[in] sp_fmt   TUYA_FRAME_FMT_E for the sub channel; 0 leaves SP disabled
 * @param[in] sp_w     SP output width, ignored when @p sp_fmt is 0
 * @param[in] sp_h     SP output height, ignored when @p sp_fmt is 0
 * @return OPRT_OK on success
 */
/**
 * @brief Parse an SP pixel-format keyword.
 * @param[in] name "nv12" | "rgb888" | "yuyv" | "rgb565"
 * @return TUYA_FRAME_FMT_E, 0 when unrecognised
 */
static uint32_t test_cam_fmt_parse(const char *name)
{
    if (name == NULL) {
        return 0U;
    }
    if (os_strcmp(name, "nv12") == 0) {
        return (uint32_t)TUYA_FRAME_FMT_NV12;
    }
    if (os_strcmp(name, "rgb888") == 0) {
        return (uint32_t)TUYA_FRAME_FMT_RGB888;
    }
    if (os_strcmp(name, "yuyv") == 0) {
        return (uint32_t)TUYA_FRAME_FMT_YUYV;
    }
    if (os_strcmp(name, "rgb565") == 0) {
        return (uint32_t)TUYA_FRAME_FMT_RGB565;
    }
    return 0U;
}

/**
 * @brief Name of a TUYA_FRAME_FMT_E, for logging.
 * @param[in] fmt TUYA_FRAME_FMT_E
 * @return static string, "?" when unrecognised
 */
static const char *test_cam_fmt_name(uint32_t fmt)
{
    switch (fmt) {
    case (uint32_t)TUYA_FRAME_FMT_NV12:   return "nv12";
    case (uint32_t)TUYA_FRAME_FMT_RGB888: return "rgb888";
    case (uint32_t)TUYA_FRAME_FMT_YUYV:   return "yuyv";
    case (uint32_t)TUYA_FRAME_FMT_RGB565: return "rgb565";
    default:                             return "?";
    }
}

/**
 * @brief Bytes one frame of @p fmt occupies at @p w x @p h.
 * @param[in] fmt TUYA_FRAME_FMT_E
 * @param[in] w   width
 * @param[in] h   height
 * @return byte count, 0 when @p fmt has no known size
 */
static uint32_t test_cam_fmt_size(uint32_t fmt, uint16_t w, uint16_t h)
{
    uint32_t px = (uint32_t)w * h;

    switch (fmt) {
    case (uint32_t)TUYA_FRAME_FMT_NV12:   return px * 3U / 2U;
    /* Four, not three: the ISP writes RGB888 as RGBX, one padding byte per
     * pixel, and tkl_mipi_csi_get_frame() rejects a buffer sized for three
     * (tkl_mipi_csi_sp_frame_size_get() applies the same rule). A 640x480
     * SP plane really is 1228800 bytes. */
    case (uint32_t)TUYA_FRAME_FMT_RGB888: return px * 4U;
    case (uint32_t)TUYA_FRAME_FMT_YUYV:   return px * 2U;
    case (uint32_t)TUYA_FRAME_FMT_RGB565: return px * 2U;
    default:                             return 0U;
    }
}

static OPERATE_RET test_cam_open(uint32_t mp_kinds, uint16_t mp_w, uint16_t mp_h,
                                 uint32_t sp_fmt, uint16_t sp_w, uint16_t sp_h)
{
    TUYA_MIPI_CSI_CFG_T cfg;
    OPERATE_RET ret;

    if (s_cam.opened == true) {
        tkl_log_output("xmipi cam already open, close first\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    /* Before anything can allocate a pixel buffer. Every sequence that worked
     * so far happened to start with "xmipi dsi open_gpu", which brings the slab
     * heaps up as a side effect; "xmipi cam open frame" needs no panel and asserted
     * with "type:1 not init" without this. */
    tkl_media_buf_init();

    os_memset(&s_cam, 0, sizeof(s_cam));
    os_memset(&cfg, 0, sizeof(cfg));

    cfg.board_cfg.scl = (TUYA_GPIO_NUM_E)CAM_TEST_PIN_SCL;
    cfg.board_cfg.sda = (TUYA_GPIO_NUM_E)CAM_TEST_PIN_SDA;
    cfg.board_cfg.port = (TUYA_I2C_NUM_E)CAM_TEST_I2C_ID;
    cfg.board_cfg.clk = (TUYA_GPIO_NUM_E)CAM_TEST_PIN_XCLK;
    /* the vendor sensor drives its control pins active-high */
    cfg.board_cfg.reset.pin = (TUYA_GPIO_NUM_E)CAM_TEST_PIN_RESET;
    cfg.board_cfg.reset.active_level = TUYA_GPIO_LEVEL_HIGH;
    /* PWDN is not wired; TUYA_GPIO_NUM_MAX means "not connected" */
    cfg.board_cfg.power_ctrl.pin = TUYA_GPIO_NUM_MAX;
    cfg.board_cfg.power_ctrl.active_level = TUYA_GPIO_LEVEL_HIGH;

    cfg.sensor_cfg.width = CAM_TEST_SENSOR_W;
    cfg.sensor_cfg.height = CAM_TEST_SENSOR_H;
    cfg.sensor_cfg.fps = CAM_TEST_SENSOR_FPS;
    cfg.sensor_cfg.format = (TUYA_FRAME_FMT_E)CAM_TEST_SENSOR_IN_FMT;

    /* One shared output selection: PIXEL/STREAM place the primary channel in
     * flexa mode; FRAME is the pull-only secondary, turned on by an sp_fmt or
     * the "frame" kind word. */
    cfg.frame.enable = (((mp_kinds & TUYA_CAMERA_OUTPUT_FRAME) != 0U) ||
                        (sp_fmt != 0U)) ? TRUE : FALSE;
    cfg.pixel.enable = ((mp_kinds & TUYA_CAMERA_OUTPUT_PIXEL) != 0U) ? TRUE : FALSE;
    cfg.stream.enable = ((mp_kinds & TUYA_CAMERA_OUTPUT_STREAM) != 0U) ? TRUE : FALSE;

    if (cfg.frame.enable == TRUE) {
        /* "frame" without an sp_fmt: default SP geometry, NV12 so the pulled
         * frame feeds JPEG encoding or a transform directly */
        if (sp_fmt == 0U) {
            sp_fmt = (uint32_t)TUYA_FRAME_FMT_NV12;
            sp_w = CAM_TEST_SP_W;
            sp_h = CAM_TEST_SP_H;
        }
        cfg.frame.width = sp_w;
        cfg.frame.height = sp_h;
        cfg.frame.format = (TUYA_FRAME_FMT_E)sp_fmt;
    }

    if (cfg.pixel.enable == TRUE) {
        cfg.pixel.rotate = s_cam_rotate;
        cfg.pixel.width = mp_w;
        cfg.pixel.height = mp_h;
        cfg.pixel.format = TUYA_FRAME_FMT_ARGB8888;
        cfg.pixel.pixel_cb = test_cam_display_cb;
    }

    if (cfg.stream.enable == TRUE) {
        cfg.stream.width = mp_w;
        cfg.stream.height = mp_h;
        cfg.stream.fps = CAM_TEST_SENSOR_FPS;
        cfg.stream.gop = CAM_TEST_GOP;
        cfg.stream.bitrate = CAM_TEST_BITRATE;
        cfg.stream.stream_cb = test_cam_stream_cb;
    }

    ret = tkl_mipi_csi_init(&cfg);
    if (ret == OPRT_OK) {
        ret = tkl_mipi_csi_start();
        if (ret != OPRT_OK) {
            (void)tkl_mipi_csi_deinit();
        }
    }
    if (ret != OPRT_OK) {
        tkl_log_output("xmipi cam open failed: %d\r\n", ret);
        return ret;
    }

    s_cam.opened = true;
    s_cam.out_kinds = mp_kinds;
    s_cam.sp_on = (sp_fmt != 0U);
    s_cam.sp_format = sp_fmt;
    s_cam.sp_w = sp_w;
    s_cam.sp_h = sp_h;
    s_cam.mark_ms = (uint32_t)tkl_system_get_millisecond();

    s_cam_last_open.valid = true;
    s_cam_last_open.mp_kinds = mp_kinds;
    s_cam_last_open.mp_w = mp_w;
    s_cam_last_open.mp_h = mp_h;
    s_cam_last_open.sp_fmt = sp_fmt;
    s_cam_last_open.sp_w = sp_w;
    s_cam_last_open.sp_h = sp_h;

    if (s_cam.sp_on) {
        tkl_log_output("xmipi cam open ok: mp=%ux%u kinds=0x%x, sp=%ux%u %s, display=%ux%u rot=%u\r\n",
                       mp_w, mp_h, (unsigned int)mp_kinds,
                       sp_w, sp_h, test_cam_fmt_name(sp_fmt),
                       cfg.pixel.width, cfg.pixel.height, s_cam_rotate);
    } else {
        tkl_log_output("xmipi cam open ok: mp=%ux%u kinds=0x%x, sp=off, display=%ux%u rot=%u\r\n",
                       mp_w, mp_h, (unsigned int)mp_kinds,
                       cfg.pixel.width, cfg.pixel.height, s_cam_rotate);
    }
    return OPRT_OK;
}

/**
 * @brief xmipi cam frame - pull one frame, the frame.enable path.
 *
 * @details
 * The only exercise for tkl_mipi_csi_get_frame(). The frame output maps to the
 * camera's secondary channel; display/h264 run the primary channel in flexa
 * mode, where the ISP streams straight into the consumer and there is nothing
 * to pull.
 *
 * @return OPRT_OK on success
 */
static OPERATE_RET test_cam_frame(VOID_T)
{
    TUYA_MEDIA_BUF_T *frame = NULL;
    OPERATE_RET ret;

    if (s_cam.opened != true) {
        tkl_log_output("xmipi cam is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_cam.sp_on != true) {
        tkl_log_output("xmipi cam frame: FRAME output is off; re-open with an sp_fmt, "
                       "e.g. xmipi cam open display+h264 640 1024 nv12 640 480\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    /* No sizing, no pool choice, no allocation: the camera returns its own
     * one-block buffer and the release function travels inside it. */
    ret = tkl_mipi_csi_get_frame(&frame, CAM_TEST_FRAME_TIMEOUT_MS);
    if (ret != OPRT_OK) {
        tkl_log_output("xmipi cam frame failed: %d\r\n", ret);
        return ret;
    }

    tkl_log_output("xmipi cam frame: %ux%u asked=%s tkl_fmt=%u %u bytes\r\n",
                   (unsigned int)frame->width, (unsigned int)frame->height,
                   test_cam_fmt_name(s_cam.sp_format),
                   (unsigned int)frame->format,
                   (unsigned int)frame->size);

    frame->free_cb(frame->data);
    return OPRT_OK;
}

STATIC VOID_T test_cam_osd_clear_settle(VOID_T);

static OPERATE_RET test_cam_close(void)
{
    OPERATE_RET ret;

    if (s_cam.opened != true) {
        return OPRT_OK;
    }

    test_overlay_clock_stop(TUYA_CAMERA_OUTPUT_PIXEL);
    test_overlay_clock_stop(TUYA_CAMERA_OUTPUT_STREAM);
    test_cam_osd_clear_settle();

    ret = tkl_mipi_csi_deinit();

    /* After close, not before: stop() frees the ring, and the encode callback
     * that feeds it only stops firing once the pipeline is down. The writer
     * itself needs nothing from the pipeline, so the capture still flushes. */
    test_record_stop();
    tkl_log_output("xmipi cam close: display_frames=%u stream_frames=%u bytes=%llu\r\n",
                   (unsigned int)s_cam.display_frames,
                   (unsigned int)s_cam.stream_frames,
                   (unsigned long long)s_cam.stream_bytes);
    os_memset(&s_cam, 0, sizeof(s_cam));
    return ret;
}

/**
 * @brief "xmipi cam reopen [n]" - time n close+open cycles at the current settings.
 *
 * @details
 * Each round is a full teardown and rebuild. Compare against
 * "xmipi cam restart", which pauses and resumes instead: the gap is what init()
 * takes off the restart path, and it is small here (~30 ms).
 *
 * No settle delay between the close and the next open, on purpose: a restart
 * that needs one is itself a finding.
 *
 * @param[in] rounds how many cycles; 0 is treated as 1
 * @return OPRT_OK when every round completed
 */
static OPERATE_RET test_cam_reopen(uint32_t rounds)
{
    uint32_t worst_close = 0U;
    uint32_t worst_open = 0U;
    uint32_t best = 0xFFFFFFFFU;
    uint64_t total = 0U;
    uint32_t i;

    if (s_cam_last_open.valid != true) {
        tkl_log_output("xmipi cam reopen: nothing opened yet, run 'xmipi cam open ...' first\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }
    if (s_cam.opened != true) {
        tkl_log_output("xmipi cam reopen: not open\r\n");
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
        (void)test_cam_close();
        close_ms = (uint32_t)tkl_system_get_millisecond() - t;

        t = (uint32_t)tkl_system_get_millisecond();
        ret = test_cam_open(s_cam_last_open.mp_kinds,
                            s_cam_last_open.mp_w,
                            s_cam_last_open.mp_h,
                            s_cam_last_open.sp_fmt,
                            s_cam_last_open.sp_w,
                            s_cam_last_open.sp_h);
        open_ms = (uint32_t)tkl_system_get_millisecond() - t;

        if (ret != OPRT_OK) {
            tkl_log_output("xmipi cam reopen: round %u/%u failed to reopen: %d "
                           "(close %u ms)\r\n",
                           (unsigned int)(i + 1U), (unsigned int)rounds, ret,
                           (unsigned int)close_ms);
            return ret;
        }

        tkl_log_output("xmipi cam reopen %u/%u: close %u ms + open %u ms = %u ms\r\n",
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

    tkl_log_output("xmipi cam reopen: %u rounds, cycle min %u avg %u ms, "
                   "worst close %u worst open %u\r\n",
                   (unsigned int)rounds,
                   (unsigned int)best,
                   (unsigned int)(total / rounds),
                   (unsigned int)worst_close,
                   (unsigned int)worst_open);
    return OPRT_OK;
}

/**
 * @brief "xmipi cam restart [n]" - time n stop+start cycles.
 *
 * @details
 * The counterpart to "xmipi cam reopen": same loop, but it pauses and resumes
 * instead of powering the sensor down and probing it again.
 *
 * Worth running once with SP enabled and once without: SP adds ~266 ms to every
 * start(), which dwarfs anything the API split can save.
 *
 * @param[in] rounds how many cycles; 0 is treated as 1
 * @return OPRT_OK when every round completed
 */
/**
 * @brief Park the pipeline. Consumers are callbacks, so there is nothing to join.
 *
 * @param[out] ms milliseconds tkl_mipi_csi_stop() cost; may be NULL
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
static OPERATE_RET test_cam_pause(uint32_t *ms)
{
    uint32_t t = (uint32_t)tkl_system_get_millisecond();
    OPERATE_RET ret;

    ret = tkl_mipi_csi_stop();
    s_cam.paused = true;

    if (ms != NULL) {
        *ms = (uint32_t)tkl_system_get_millisecond() - t;
    }
    return ret;
}

/**
 * @brief Resume streaming after test_cam_pause().
 *
 * @param[out] ms milliseconds tkl_mipi_csi_start() cost; may be NULL
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
static OPERATE_RET test_cam_resume(uint32_t *ms)
{
    uint32_t t = (uint32_t)tkl_system_get_millisecond();
    OPERATE_RET ret;

    ret = tkl_mipi_csi_start();
    if (ms != NULL) {
        *ms = (uint32_t)tkl_system_get_millisecond() - t;
    }
    if (ret == OPRT_OK) {
        s_cam.paused = false;
    }
    return ret;
}

static OPERATE_RET test_cam_restart(uint32_t rounds)
{
    uint32_t worst_stop = 0U;
    uint32_t worst_start = 0U;
    uint32_t best = 0xFFFFFFFFU;
    uint64_t total = 0U;
    uint32_t i;

    if (s_cam.opened != true) {
        tkl_log_output("xmipi cam restart: not open, run 'xmipi cam open ...' first\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }
    if (rounds == 0U) {
        rounds = 1U;
    }

    for (i = 0U; i < rounds; i++) {
        uint32_t stop_ms;
        uint32_t start_ms;
        OPERATE_RET ret;

        /* exactly what "xmipi cam stop" / "start" do, so the two commands and
         * this loop can never drift apart */
        (void)test_cam_pause(&stop_ms);

        ret = test_cam_resume(&start_ms);
        if (ret != OPRT_OK) {
            tkl_log_output("xmipi cam restart: round %u/%u failed to resume: %d "
                           "(stop %u ms)\r\n",
                           (unsigned int)(i + 1U), (unsigned int)rounds, ret,
                           (unsigned int)stop_ms);
            return ret;
        }

        tkl_log_output("xmipi cam restart %u/%u: stop %u ms + start %u ms = %u ms\r\n",
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

    tkl_log_output("xmipi cam restart: %u rounds, cycle min %u avg %u ms, "
                   "worst stop %u worst start %u\r\n",
                   (unsigned int)rounds,
                   (unsigned int)best,
                   (unsigned int)(total / rounds),
                   (unsigned int)worst_stop,
                   (unsigned int)worst_start);
    return OPRT_OK;
}

/**
 * @brief Push one clock string to a facade channel.
 * @param[in] text HH:MM:SS
 * @param[in] arg  the output type, cast from TUYA_CAMERA_OUTPUT_TYPE_E
 * @return OPRT_OK on success
 */
static OPERATE_RET test_cam_osd_submit(CONST CHAR_T *text, VOID_T *arg)
{
    TUYA_CAMERA_OUTPUT_TYPE_E type = (TUYA_CAMERA_OUTPUT_TYPE_E)(uintptr_t)arg;
    TUYA_OSD_T osd;

    test_osd_set_clock(text);

    os_memset(&osd, 0, sizeof(osd));

    if (type == TUYA_CAMERA_OUTPUT_PIXEL) {
        osd.dst_x = TEST_OSD_CLOCK_DST_X;
        osd.dst_y = TEST_OSD_CLOCK_DST_Y;
    } else {
        /* The encoder sees the un-rotated frame, so its origin is its own. */
        osd.dst_x = 2U;
        osd.dst_y = 2U;
    }
    osd.w = TEST_OSD_CLOCK_BITMAP_W;
    osd.h = TEST_OSD_CLOCK_BITMAP_H;
    osd.rotate = s_cam_osd_rotate;
    osd.data = test_osd_clock_bitmap(&osd.w, &osd.h);
    if (osd.data == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    return tkl_mipi_csi_set_osd(type, &osd);
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
static OPERATE_RET test_cam_osd_clock_clear(TUYA_CAMERA_OUTPUT_TYPE_E type)
{
    TUYA_OSD_T osd;

    os_memset(&osd, 0, sizeof(osd));
    if (type == TUYA_CAMERA_OUTPUT_PIXEL) {
        osd.dst_x = TEST_OSD_CLOCK_DST_X;
        osd.dst_y = TEST_OSD_CLOCK_DST_Y;
    } else {
        osd.dst_x = 2U;
        osd.dst_y = 2U;
    }
    osd.data = NULL;

    return tkl_mipi_csi_set_osd(type, &osd);
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
static OPERATE_RET test_cam_osd_wifi(TUYA_CAMERA_OUTPUT_TYPE_E type, UINT8_T level, BOOL_T on)
{
    TUYA_OSD_T osd;

    /* vcenc DMA cannot reach flash XIP: the const rssi bitmaps live in rodata,
     * so stage the selected one in RAM before handing it to the encoder. The
     * stage lives in SRAM's dynamic heap: this is a CPU-writes -> HW-reads
     * buffer, and PSRAM would need a write-through channel for that (only 4
     * per PSRAM chip, the display frame rotation already holds two - the
     * canvases exhausted it on the board). SRAM stores are non-cacheable and
     * land directly, and both consumers reach SRAM (GPU 0x28 alias, vcenc). */
    static UINT8_T *s_wifi_osd_ram;

    os_memset(&osd, 0, sizeof(osd));

    if (on == TRUE) {
        if (s_wifi_osd_ram == NULL) {
            s_wifi_osd_ram = (UINT8_T *)hsram_malloc(
                    TEST_OSD_WIFI_BITMAP_W * TEST_OSD_WIFI_BITMAP_H * 4U);
            if (s_wifi_osd_ram == NULL) {
                tkl_log_output("xmipi cam osd wifi: no SRAM for RAM stage\r\n");
                return OPRT_MALLOC_FAILED;
            }
        }
        os_memcpy(s_wifi_osd_ram, test_osd_wifi_bitmap(level),
                  TEST_OSD_WIFI_BITMAP_W * TEST_OSD_WIFI_BITMAP_H * 4U);
        osd.data = s_wifi_osd_ram;
        osd.w = TEST_OSD_WIFI_BITMAP_W;
        osd.h = TEST_OSD_WIFI_BITMAP_H;

        if (type == TUYA_CAMERA_OUTPUT_PIXEL) {
            osd.dst_x = TEST_OSD_WIFI_DST_X;
            osd.dst_y = TEST_OSD_WIFI_DST_Y;
        } else {
            /* The encoder sees the un-rotated frame; keep clear of the clock. */
            osd.dst_x = 2U;
            osd.dst_y = 72U;
        }
    } else if (type == TUYA_CAMERA_OUTPUT_PIXEL) {
        osd.dst_x = TEST_OSD_WIFI_DST_X;
        osd.dst_y = TEST_OSD_WIFI_DST_Y;
    } else {
        osd.dst_x = 2U;
        osd.dst_y = 72U;
    }

    osd.rotate = 0U;
    return tkl_mipi_csi_set_osd(type, &osd);
}

/**
 * @brief Drop every overlay sprite and let one clean frame reach the panel.
 *
 * @note Clearing the slots is not enough on its own. The panel keeps whatever
 *       the compositor last flushed and nothing repaints it once the stream is
 *       down, so a frame has to be composited *after* the clear - this must run
 *       while the pipeline is still streaming, and then wait for one to arrive.
 * @note Test-layer policy, not adapter behaviour: business decides for itself
 *       whether an overlay should survive a camera switch. The CLI always
 *       clears, so a stale clock never carries into the next case.
 */
STATIC VOID_T test_cam_osd_clear_settle(VOID_T)
{
    uint32_t mark;
    uint32_t t;

    /* stopped: set_osd refuses with -23, and no frame would repaint anyway */
    if (s_cam.paused == true || (s_cam.out_kinds & TUYA_CAMERA_OUTPUT_PIXEL) == 0U) {
        return;
    }

    (void)test_cam_osd_clock_clear(TUYA_CAMERA_OUTPUT_PIXEL);
    (void)test_cam_osd_wifi(TUYA_CAMERA_OUTPUT_PIXEL, 0U, FALSE);
    (void)test_cam_osd_clock_clear(TUYA_CAMERA_OUTPUT_STREAM);
    (void)test_cam_osd_wifi(TUYA_CAMERA_OUTPUT_STREAM, 0U, FALSE);

    mark = s_cam.display_frames;
    t = (uint32_t)tkl_system_get_millisecond();
    while (s_cam.display_frames == mark &&
           ((uint32_t)tkl_system_get_millisecond() - t) < CAM_OSD_CLEAR_SETTLE_MS) {
        rtos_delay_milliseconds(5);
    }
}

/* ===========================================================================
 * Sensor / ISP image-quality self-tests: "xmipi cam mirror_test" / "luminance_test".
 * =========================================================================== */


/* What was last asked for. The sensor is the authority once it is running -
 * these only answer "what did you ask for" before the camera is up. */
static BOOL_T s_cam_quality_hmirror;
static BOOL_T s_cam_quality_vflip;

/**
 * @brief Set camera mirror/flip and sync local cfg
 * @param[in] hmirror FALSE=off, TRUE=on
 * @param[in] vflip FALSE=off, TRUE=on
 * @return OPRT_OK on success
 */
static OPERATE_RET cam_quality_set_mirror_flip(BOOL_T hmirror, BOOL_T vflip)
{
    OPERATE_RET ret;

    if (test_camera_is_open() != TRUE) {
        tkl_log_output("mipi camera is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_mipi_csi_set_mirror_flip(hmirror, vflip);
    if (ret != OPRT_OK) {
        tkl_log_output("camera set_mirror_flip failed: %d\r\n", ret);
        return ret;
    }

    s_cam_quality_hmirror = hmirror;
    s_cam_quality_vflip = vflip;
    tkl_log_output("camera mirror set ok, hmirror=%u vflip=%u (started=%u)\r\n",
                   s_cam_quality_hmirror,
                   s_cam_quality_vflip,
                   (unsigned int)test_camera_is_open());
    return OPRT_OK;
}

/**
 * @brief Get camera mirror/flip from TKL
 * @param[out] hmirror optional
 * @param[out] vflip optional
 * @return OPRT_OK on success
 */
static OPERATE_RET cam_quality_get_mirror_flip(BOOL_T *hmirror, BOOL_T *vflip)
{
    OPERATE_RET ret;
    BOOL_T h = FALSE;
    BOOL_T v = FALSE;

    if (test_camera_is_open() != TRUE) {
        tkl_log_output("mipi camera is not open\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_mipi_csi_get_mirror_flip(&h, &v);
    if (ret != OPRT_OK) {
        tkl_log_output("camera get_mirror_flip failed: %d\r\n", ret);
        return ret;
    }

    if (hmirror != NULL) {
        *hmirror = h;
    }
    if (vflip != NULL) {
        *vflip = v;
    }

    tkl_log_output("camera mirror get ok, hmirror=%u vflip=%u\r\n", h, v);
    return OPRT_OK;
}

/**
 * @brief Rolling checksum for frame payload
 * @param[in] buf data buffer
 * @param[in] len byte length
 * @return checksum
 */
static uint32_t cam_quality_frame_checksum(const uint8_t *buf, uint32_t len)
{
    uint32_t sum = 0U;
    uint32_t i;

    if (buf == NULL || len == 0U) {
        return 0U;
    }

    for (i = 0U; i < len; i++) {
        sum = (sum * 131U) + buf[i];
    }
    return sum;
}

/**
 * @brief Checksum of a vertical Y strip [x, x+cols)
 * @param[in] y Y plane base
 * @param[in] width image width
 * @param[in] height image height
 * @param[in] stride Y stride in bytes
 * @param[in] x start column
 * @param[in] cols column count
 * @return checksum
 */
static uint32_t cam_quality_y_strip_checksum(const uint8_t *y,
                                          uint16_t width,
                                          uint16_t height,
                                          uint32_t stride,
                                          uint16_t x,
                                          uint16_t cols)
{
    uint32_t sum = 0U;
    uint16_t row;
    uint16_t col;

    if (y == NULL || width == 0U || height == 0U || stride == 0U || cols == 0U) {
        return 0U;
    }
    if ((uint32_t)x + (uint32_t)cols > (uint32_t)width) {
        return 0U;
    }

    for (row = 0U; row < height; row++) {
        const uint8_t *line = y + ((uint32_t)row * stride) + x;
        for (col = 0U; col < cols; col++) {
            sum = (sum * 131U) + line[col];
        }
    }
    return sum;
}

/**
 * @brief Settle, then grab one SP frame and reduce it to three checksums.
 *
 * @param[out] checksum   whole-frame checksum
 * @param[out] left_edge  checksum of the leftmost columns (0 when not NV12)
 * @param[out] right_edge checksum of the rightmost columns
 *
 * @note The camera lends the frame and the pool is only two deep, so each
 *       capture is released before the next one is asked for. Settling matters:
 *       a mirror change takes a couple of frames to appear in the output.
 */
static OPERATE_RET cam_quality_capture_mirror_sample(uint32_t *checksum,
                                                     uint32_t *left_edge,
                                                     uint32_t *right_edge)
{
    TUYA_MEDIA_BUF_T *frame = NULL;
    OPERATE_RET ret;
    UINT32_T i;
    UINT16_T edge_cols;
    UINT32_T y_stride;
    UINT32_T y_size;
    UINT16_T width;
    UINT16_T height;

    if (checksum == NULL || left_edge == NULL || right_edge == NULL) {
        return OPRT_INVALID_PARM;
    }

    for (i = 0U; i <= CAM_QUALITY_MIRROR_SETTLE_FRAMES; i++) {
        ret = tkl_mipi_csi_get_frame(&frame, CAM_QUALITY_FRAME_TIMEOUT_MS);
        if (ret != OPRT_OK) {
            return ret;
        }
        if (i < CAM_QUALITY_MIRROR_SETTLE_FRAMES) {
            frame->free_cb(frame->data);
        }
    }

    width = frame->width;
    height = frame->height;
    *checksum = cam_quality_frame_checksum(frame->data, frame->size);

    y_stride = (UINT32_T)width;    /* camera frames are tightly packed */
    y_size = y_stride * (UINT32_T)height;
    if (frame->format != (UINT32_T)TUYA_FRAME_FMT_NV12 || frame->size < y_size || width < 16U) {
        *left_edge = 0U;
        *right_edge = 0U;
        frame->free_cb(frame->data);
        return OPRT_OK;
    }

    edge_cols = CAM_QUALITY_MIRROR_EDGE_COLS;
    if (edge_cols > (width / 4U)) {
        edge_cols = (UINT16_T)(width / 4U);
    }
    if (edge_cols == 0U) {
        edge_cols = 1U;
    }

    *left_edge = cam_quality_y_strip_checksum(frame->data, width, height, y_stride,
                                              0U, edge_cols);
    *right_edge = cam_quality_y_strip_checksum(frame->data, width, height, y_stride,
                                               (UINT16_T)(width - edge_cols), edge_cols);
    frame->free_cb(frame->data);
    return OPRT_OK;
}

/**
 * @brief Verify mirror/flip API and that image content changes
 * @return OPRT_OK if set/get and content change checks pass
 * @note Requires frame-mode camera already started. Point lens at a non-uniform scene.
 */
static OPERATE_RET cam_quality_mirror_test(void)
{
    OPERATE_RET ret;
    BOOL_T saved_h = FALSE;
    BOOL_T saved_v = FALSE;
    BOOL_T got_h = FALSE;
    BOOL_T got_v = FALSE;
    uint32_t base_sum = 0U;
    uint32_t base_left = 0U;
    uint32_t base_right = 0U;
    uint32_t h_sum = 0U;
    uint32_t h_left = 0U;
    uint32_t h_right = 0U;
    uint32_t v_sum = 0U;
    uint32_t v_left = 0U;
    uint32_t v_right = 0U;
    uint32_t hv_sum = 0U;
    uint32_t hv_left = 0U;
    uint32_t hv_right = 0U;
    BOOL_T pass = TRUE;

    if (test_camera_is_open() != TRUE) {
        tkl_log_output("mirror_test needs a running camera: xmipi cam open display 640 1024 nv12 640 480\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    if (test_camera_get_sp_info(NULL, NULL, NULL, NULL) != OPRT_OK) {
        tkl_log_output("mirror_test needs the SP channel, use: "
                       "xmipi cam open display 640 1024 nv12 640 480\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    ret = cam_quality_get_mirror_flip(&saved_h, &saved_v);
    if (ret != OPRT_OK) {
        return ret;
    }

    /* Baseline: no mirror/flip */
    ret = cam_quality_set_mirror_flip(FALSE, FALSE);
    if (ret != OPRT_OK) {
        return ret;
    }
    ret = cam_quality_get_mirror_flip(&got_h, &got_v);
    if (ret != OPRT_OK || got_h != FALSE || got_v != FALSE) {
        tkl_log_output("mirror_test FAIL: get after set(0,0) h=%u v=%u ret=%d\r\n",
                       got_h, got_v, ret);
        pass = FALSE;
        goto restore;
    }
    ret = cam_quality_capture_mirror_sample(&base_sum, &base_left, &base_right);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    tkl_log_output("mirror_test base: sum=0x%08x left=0x%08x right=0x%08x\r\n",
                   (unsigned int)base_sum, (unsigned int)base_left, (unsigned int)base_right);

    /* Horizontal mirror */
    ret = cam_quality_set_mirror_flip(TRUE, FALSE);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    ret = cam_quality_get_mirror_flip(&got_h, &got_v);
    if (ret != OPRT_OK || got_h != TRUE || got_v != FALSE) {
        tkl_log_output("mirror_test FAIL: get after set(1,0) h=%u v=%u ret=%d\r\n",
                       got_h, got_v, ret);
        pass = FALSE;
        goto restore;
    }
    ret = cam_quality_capture_mirror_sample(&h_sum, &h_left, &h_right);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    tkl_log_output("mirror_test hmirror: sum=0x%08x left=0x%08x right=0x%08x\r\n",
                   (unsigned int)h_sum, (unsigned int)h_left, (unsigned int)h_right);
    if (h_sum == base_sum) {
        tkl_log_output("mirror_test FAIL: hmirror checksum unchanged (flat scene or not applied)\r\n");
        pass = FALSE;
    } else {
        tkl_log_output("mirror_test PASS: hmirror changed frame checksum\r\n");
    }
    if (base_left != 0U && base_right != 0U && h_left != 0U && h_right != 0U) {
        /* Soft check: mirrored left edge should be closer to original right edge. */
        uint32_t d_lr = (base_left > h_right) ? (base_left - h_right) : (h_right - base_left);
        uint32_t d_ll = (base_left > h_left) ? (base_left - h_left) : (h_left - base_left);
        if (d_lr < d_ll) {
            tkl_log_output("mirror_test PASS: hmirror edge swap looks correct (d_lr=%u < d_ll=%u)\r\n",
                           (unsigned int)d_lr, (unsigned int)d_ll);
        } else {
            tkl_log_output("mirror_test WARN: hmirror edge soft-check weak (d_lr=%u d_ll=%u), check scene\r\n",
                           (unsigned int)d_lr, (unsigned int)d_ll);
        }
    }

    /* Vertical flip */
    ret = cam_quality_set_mirror_flip(FALSE, TRUE);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    ret = cam_quality_get_mirror_flip(&got_h, &got_v);
    if (ret != OPRT_OK || got_h != FALSE || got_v != TRUE) {
        tkl_log_output("mirror_test FAIL: get after set(0,1) h=%u v=%u ret=%d\r\n",
                       got_h, got_v, ret);
        pass = FALSE;
        goto restore;
    }
    ret = cam_quality_capture_mirror_sample(&v_sum, &v_left, &v_right);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    tkl_log_output("mirror_test vflip: sum=0x%08x left=0x%08x right=0x%08x\r\n",
                   (unsigned int)v_sum, (unsigned int)v_left, (unsigned int)v_right);
    if (v_sum == base_sum || v_sum == h_sum) {
        tkl_log_output("mirror_test FAIL: vflip checksum not distinct (flat scene or not applied)\r\n");
        pass = FALSE;
    } else {
        tkl_log_output("mirror_test PASS: vflip changed frame checksum\r\n");
    }

    /* Both */
    ret = cam_quality_set_mirror_flip(TRUE, TRUE);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    ret = cam_quality_get_mirror_flip(&got_h, &got_v);
    if (ret != OPRT_OK || got_h != TRUE || got_v != TRUE) {
        tkl_log_output("mirror_test FAIL: get after set(1,1) h=%u v=%u ret=%d\r\n",
                       got_h, got_v, ret);
        pass = FALSE;
        goto restore;
    }
    ret = cam_quality_capture_mirror_sample(&hv_sum, &hv_left, &hv_right);
    if (ret != OPRT_OK) {
        pass = FALSE;
        goto restore;
    }
    tkl_log_output("mirror_test both: sum=0x%08x left=0x%08x right=0x%08x\r\n",
                   (unsigned int)hv_sum, (unsigned int)hv_left, (unsigned int)hv_right);
    if (hv_sum == base_sum) {
        tkl_log_output("mirror_test FAIL: both mirror+flip checksum unchanged\r\n");
        pass = FALSE;
    } else {
        tkl_log_output("mirror_test PASS: mirror+flip changed frame checksum\r\n");
    }

restore:
    (void)cam_quality_set_mirror_flip(saved_h, saved_v);
    if (pass != TRUE) {
        tkl_log_output("mirror_test FAILED\r\n");
        return OPRT_COM_ERROR;
    }
    tkl_log_output("mirror_test ALL PASSED\r\n");
    return OPRT_OK;
}

/**
 * @brief Query ISP exposure luminance once
 * @param[out] luminance optional output
 * @return OPRT_OK on success
 * @note Value is exposure-normalized: (meanLum * 1e6) / (exposure[0] * 39).
 */
static OPERATE_RET cam_quality_get_luminance(UINT32_T *luminance)
{
    OPERATE_RET ret;
    UINT32_T lum = 0U;

    if (test_camera_is_open() != TRUE) {
        tkl_log_output("luminance needs started camera\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_mipi_csi_get_exposure_luminance(&lum);
    if (ret != OPRT_OK) {
        tkl_log_output("camera get_exposure_luminance failed: %d\r\n", ret);
        return ret;
    }

    tkl_log_output("camera luminance=%u (norm: meanLum*1e6/(exposure*39))\r\n",
                   (unsigned int)lum);
    if (luminance != NULL) {
        *luminance = lum;
    }
    return OPRT_OK;
}

/**
 * @brief Sample exposure luminance repeatedly and report stats
 * @param[in] count sample count
 * @param[in] interval_ms delay between samples
 * @return OPRT_OK if all samples succeed
 */
static OPERATE_RET cam_quality_luminance_test(UINT32_T count, UINT32_T interval_ms)
{
    OPERATE_RET ret;
    UINT32_T i;
    UINT32_T lum = 0U;
    UINT32_T min_lum = 0xFFFFFFFFU;
    UINT32_T max_lum = 0U;
    UINT64_T sum = 0U;
    BOOL_T pass = TRUE;

    if (count == 0U) {
        count = CAM_QUALITY_LUMINANCE_DEFAULT_CNT;
    }
    if (interval_ms == 0U) {
        interval_ms = CAM_QUALITY_LUMINANCE_DEFAULT_MS;
    }

    tkl_log_output("luminance_test start, count=%u interval=%ums\r\n",
                   (unsigned int)count, (unsigned int)interval_ms);

    for (i = 0U; i < count; i++) {
        ret = cam_quality_get_luminance(&lum);
        if (ret != OPRT_OK) {
            pass = FALSE;
            break;
        }
        if (lum < min_lum) {
            min_lum = lum;
        }
        if (lum > max_lum) {
            max_lum = lum;
        }
        sum += (UINT64_T)lum;
        if ((i + 1U) < count) {
            tkl_system_sleep(interval_ms);
        }
    }

    if (pass != TRUE) {
        tkl_log_output("luminance_test FAILED\r\n");
        return OPRT_COM_ERROR;
    }

    tkl_log_output("luminance_test PASS: samples=%u min=%u max=%u avg=%u\r\n",
                   (unsigned int)count,
                   (unsigned int)min_lum,
                   (unsigned int)max_lum,
                   (unsigned int)(sum / (UINT64_T)count));
    return OPRT_OK;
}




/**
 * @brief Dispatch one "xmipi cam <sub> ..." image-quality subcommand.
 *
 * @param[out] out     reply buffer
 * @param[in]  out_len its size
 * @param[in]  argc/argv as handed to cli_tuya_cam_cmd, argv[1] being the subcommand
 *
 * @return TRUE when argv[1] was one of ours and @p out was filled, FALSE when
 *         the caller should keep looking
 */
static BOOL_T test_camera_quality_cmd(char *out, int out_len, int argc, char **argv)
{
    const char *cmd = argv[1];
    OPERATE_RET ret;

    if (os_strcmp(cmd, "mirror_test") == 0) {
        ret = cam_quality_mirror_test();
        snprintf(out, out_len, "xmipi cam mirror_test %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);
    } else if (os_strcmp(cmd, "luminance_test") == 0 || os_strcmp(cmd, "lum_test") == 0) {
        UINT32_T count = CAM_QUALITY_LUMINANCE_DEFAULT_CNT;
        UINT32_T interval_ms = CAM_QUALITY_LUMINANCE_DEFAULT_MS;

        if (argc >= 3) {
            count = (UINT32_T)os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            interval_ms = (UINT32_T)os_strtoul(argv[3], NULL, 10);
        }
        ret = cam_quality_luminance_test(count, interval_ms);
        snprintf(out, out_len, "xmipi cam luminance_test %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);
    } else {
        return FALSE;
    }

    return TRUE;
}

void cli_tuya_mipi_cam_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen, XCAM_USAGE);
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "open") == 0) {
        /* xmipi cam open <mp_kinds> [w] [h] [sp_fmt] [spw] [sph]
         *   argv     2          3   4   5        6     7    */
        uint32_t mp_kinds;
        uint32_t sp_fmt = 0U;
        uint16_t w = CAM_TEST_MP_W;
        uint16_t h = CAM_TEST_MP_H;
        uint16_t spw = CAM_TEST_SP_W;
        uint16_t sph = CAM_TEST_SP_H;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen, XCAM_USAGE);
            return;
        }

        mp_kinds = test_cam_kinds_parse(argv[2]);
        if (mp_kinds == TUYA_CAMERA_OUTPUT_NONE) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi cam: unknown output '%s'\r\n", argv[2]);
            return;
        }
        if (argc > 4) {
            w = (uint16_t)os_strtoul(argv[3], NULL, 10);
            h = (uint16_t)os_strtoul(argv[4], NULL, 10);
        }
        if (argc > 5) {
            sp_fmt = test_cam_fmt_parse(argv[5]);
            if (sp_fmt == 0U) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xmipi cam: unknown sp format '%s' (nv12|rgb888|yuyv|rgb565)\r\n",
                         argv[5]);
                return;
            }
        }
        if (argc > 7) {
            spw = (uint16_t)os_strtoul(argv[6], NULL, 10);
            sph = (uint16_t)os_strtoul(argv[7], NULL, 10);
        }

        test_osd_init();
        ret = test_cam_open(mp_kinds, w, h, sp_fmt, spw, sph);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam open %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "frame") == 0) {
        ret = test_cam_frame();
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam frame %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "record") == 0) {
        uint32_t sec = 5U;
        const char *path = TEST_SDCARD_MOUNT_POINT "/xcam_rec.h264";

        if (argc > 2 && os_strcmp(argv[2], "stop") == 0) {
            test_record_stop();
            snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam record stopped\r\n");
            return;
        }
        if (argc > 2) {
            sec = os_strtoul(argv[2], NULL, 10);
        }
        if (argc > 3) {
            path = argv[3];
        }

        ret = test_cam_record(sec, path);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam record %s (%d)\r\n",
                 (ret == OPRT_OK) ? "started" : "failed", ret);

    } else if (os_strcmp(cmd, "mirror") == 0) {
        BOOL_T h = FALSE;
        BOOL_T v = FALSE;

        if (argc > 3) {
            h = (os_strtoul(argv[2], NULL, 10) != 0U) ? TRUE : FALSE;
            v = (os_strtoul(argv[3], NULL, 10) != 0U) ? TRUE : FALSE;
            ret = tkl_mipi_csi_set_mirror_flip(h, v);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi cam mirror %s (%d), hmirror=%u vflip=%u\r\n",
                     (ret == OPRT_OK) ? "ok" : "failed", ret,
                     (unsigned int)h, (unsigned int)v);
        } else {
            ret = tkl_mipi_csi_get_mirror_flip(&h, &v);
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi cam mirror: hmirror=%u vflip=%u (%d)\r\n",
                     (unsigned int)h, (unsigned int)v, ret);
        }

    } else if (os_strcmp(cmd, "luminance") == 0) {
        UINT32_T lum = 0U;

        ret = tkl_mipi_csi_get_exposure_luminance(&lum);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xmipi cam luminance: %u (%d)\r\n", (unsigned int)lum, ret);

    } else if (os_strcmp(cmd, "gray") == 0) {
        BOOL_T on = (argc > 2 && os_strtoul(argv[2], NULL, 10) != 0U) ? TRUE : FALSE;

        ret = tkl_mipi_csi_set_color_to_gray(on);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam gray %s (%d), enable=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret, (unsigned int)on);

    } else if (os_strcmp(cmd, "night") == 0) {
        BOOL_T on = (argc > 2 && os_strtoul(argv[2], NULL, 10) != 0U) ? TRUE : FALSE;

        ret = tkl_mipi_csi_set_night_gamma(on);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam night %s (%d), enable=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret, (unsigned int)on);

    } else if (os_strcmp(cmd, "rotate") == 0) {
        uint16_t deg;

        if (argc <= 2) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi cam rotate: picture %u\r\n",
                     (unsigned int)s_cam_rotate);
            return;
        }

        deg = (uint16_t)os_strtoul(argv[2], NULL, 10);
        if (deg != 0U && deg != 90U && deg != 180U && deg != 270U) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi cam rotate: %u invalid, use 0|90|180|270\r\n",
                     (unsigned int)deg);
            return;
        }

        /* The picture angle is configured before opening the pipeline. */
        s_cam_rotate = deg;
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xmipi cam rotate picture set to %u, takes effect on the next 'xmipi cam open'%s\r\n",
                 (unsigned int)deg,
                 (s_cam.opened == true) ? " (close and reopen to apply)" : "");

    } else if (os_strcmp(cmd, "close") == 0) {
        ret = test_cam_close();
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam close %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "reopen") == 0) {
        uint32_t rounds = (argc >= 3) ? (uint32_t)os_strtoul(argv[2], NULL, 10) : 3U;

        ret = test_cam_reopen(rounds);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam reopen %s (%d)\r\n",
                 (ret == OPRT_OK) ? "done" : "failed", ret);

    } else if (os_strcmp(cmd, "stop") == 0) {
        uint32_t ms = 0U;

        if (s_cam.opened != true) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi cam stop: not open, run 'xmipi cam open ...' first\r\n");
            return;
        }
        if (s_cam.paused == true) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam stop: already stopped\r\n");
            return;
        }

        /* Clear here, not in close(): stop() is the last moment the pipeline is
         * still composing, and a cleared slot only reaches the panel through a
         * frame drawn after it. Once paused, set_osd refuses with -23 and
         * nothing would repaint anyway - so "osd time -> stop -> close" would
         * otherwise leave the overlay burnt into the last frame. Deliberately
         * outside the timed section: the wait is test-layer policy and must not
         * pollute the stop number that "restart" is compared against. */
        test_cam_osd_clear_settle();

        ret = test_cam_pause(&ms);
        /* the reply buffer is ~136 bytes; anything longer is silently cut, so
         * the guidance goes out through the log and only the verdict here */
        tkl_log_output("xmipi cam stop: sensor stays powered; set_pp/set_bitrate/"
                       "set_osd must now return -23. Try 'xmipi cam "
                       "switch 800 480', 'xmipi cam osd display time'. get_frame "
                       "needs FRAME - on a flexa-only MP the test layer rejects it "
                       "first with -4\r\n");
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam stop %s (%d), %u ms\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret, (unsigned int)ms);

    } else if (os_strcmp(cmd, "start") == 0) {
        uint32_t ms = 0U;

        if (s_cam.opened != true) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi cam start: not open, run 'xmipi cam open ...' first\r\n");
            return;
        }
        if (s_cam.paused != true) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xmipi cam start: already streaming\r\n");
            return;
        }

        ret = test_cam_resume(&ms);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam start %s (%d), %u ms\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret, (unsigned int)ms);

    } else if (os_strcmp(cmd, "restart") == 0) {
        uint32_t rounds = (argc >= 3) ? (uint32_t)os_strtoul(argv[2], NULL, 10) : 3U;

        ret = test_cam_restart(rounds);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam restart %s (%d)\r\n",
                 (ret == OPRT_OK) ? "done" : "failed", ret);

    } else if (os_strcmp(cmd, "switch") == 0) {
        TUYA_CAMERA_OUTPUT_TYPE_E type = TUYA_CAMERA_OUTPUT_STREAM;
        uint16_t w;
        uint16_t h;

        if (argc < 4) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xmipi cam switch <w> <h> [mp|sp]\r\n");
            return;
        }

        w = (uint16_t)os_strtoul(argv[2], NULL, 10);
        h = (uint16_t)os_strtoul(argv[3], NULL, 10);
        if (argc > 4 && os_strcmp(argv[4], "sp") == 0) {
            type = TUYA_CAMERA_OUTPUT_FRAME;
        }

        ret = tkl_mipi_csi_set_pp(type, w, h);
        if (ret == OPRT_OK && type == TUYA_CAMERA_OUTPUT_FRAME) {
            /* keep the cached geometry in step, or the next 'xmipi cam frame'
             * reports the pre-switch dimensions */
            s_cam.sp_w = w;
            s_cam.sp_h = h;
        }
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xmipi cam switch %s (%d), type=%u %ux%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret,
                 (unsigned int)type, w, h);

    } else if (os_strcmp(cmd, "stress") == 0) {
        uint32_t loops = 100U;
        uint32_t i;
        uint32_t fails = 0U;

        if (argc > 2) {
            loops = os_strtoul(argv[2], NULL, 10);
        }

        /* alternate between two geometries; the point is the teardown/rebuild
         * of every consumer around a switch, not the geometries themselves */
        for (i = 0U; i < loops; i++) {
            uint16_t w = ((i & 1U) != 0U) ? 640U : 480U;
            uint16_t h = ((i & 1U) != 0U) ? 1024U : 800U;

            if (tkl_mipi_csi_set_pp(TUYA_CAMERA_OUTPUT_STREAM, w, h) != OPRT_OK) {
                fails++;
            }
            tkl_system_sleep(50U);
        }

        tkl_log_output("xmipi cam stress: %u loops, %u failures\r\n",
                       (unsigned int)loops, (unsigned int)fails);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xmipi cam stress done: %u loops, %u failures\r\n",
                 (unsigned int)loops, (unsigned int)fails);

    } else if (os_strcmp(cmd, "bitrate") == 0) {
        UINT32_T bitrate;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: xmipi cam bitrate <bps>\r\n");
            return;
        }

        bitrate = os_strtoul(argv[2], NULL, 10);
        ret = tkl_mipi_csi_set_bitrate(bitrate);
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam bitrate %s (%d), %u bps\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret, (unsigned int)bitrate);

    } else if (os_strcmp(cmd, "idr") == 0) {
        s_cam_idr_trace = 5U;   /* arm before the call, so a fast IDR is not missed */
        ret = tkl_mipi_csi_force_idr();
        if (ret != OPRT_OK) {
            /* nothing was requested, so disarm: left armed it would fire on the
             * next "xmipi cam start" and print an I frame that is just the
             * encoder's first frame, reading exactly like an IDR that landed */
            s_cam_idr_trace = 0U;
        }
        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam idr %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "osd") == 0) {
        TUYA_CAMERA_OUTPUT_TYPE_E type;
        const char *act = (argc > 3) ? argv[3] : NULL;
        const char *sub = (argc > 4) ? argv[4] : NULL;

        if (argc < 4) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: %s\r\n", TEST_CAM_OSD_USAGE);
            return;
        }

        type = (os_strcmp(argv[2], "encode") == 0) ? TUYA_CAMERA_OUTPUT_STREAM
                                                   : TUYA_CAMERA_OUTPUT_PIXEL;

        if (os_strcmp(act, "clear") == 0) {
            test_overlay_clock_stop(type);
            ret = test_cam_osd_clock_clear(type);
            /* the icon lives in its own slot and would survive otherwise */
            (void)test_cam_osd_wifi(type, 0U, FALSE);
        } else if (os_strcmp(act, "time") == 0) {
            UINT16_T rotate = 0U;

            if (sub != NULL && os_strcmp(sub, "off") == 0) {
                test_overlay_clock_stop(type);
                ret = test_cam_osd_clock_clear(type);
            } else {
                if (sub != NULL) {
                    rotate = (UINT16_T)os_strtoul(sub, NULL, 10);
                }
                if (rotate != 0U && rotate != 90U && rotate != 270U) {
                    snprintf(pcWriteBuffer, xWriteBufferLen,
                             "Usage: %s\r\n", TEST_CAM_OSD_USAGE);
                    return;
                }
                if (type != TUYA_CAMERA_OUTPUT_PIXEL && rotate != 0U) {
                    snprintf(pcWriteBuffer, xWriteBufferLen,
                             "xmipi cam osd encode time: rotate %u unsupported\r\n",
                             (unsigned int)rotate);
                    return;
                }
                s_cam_osd_rotate = rotate;
                ret = test_overlay_clock_start(type, test_cam_osd_submit,
                                               (VOID_T *)(uintptr_t)type);
            }
        } else if (os_strcmp(act, "wifi") == 0) {
            if (sub != NULL && os_strcmp(sub, "off") == 0) {
                ret = test_cam_osd_wifi(type, 0U, FALSE);
            } else {
                UINT8_T level = (sub != NULL) ? (UINT8_T)os_strtoul(sub, NULL, 10) : 4U;

                ret = test_cam_osd_wifi(type, level, TRUE);
            }
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: %s\r\n", TEST_CAM_OSD_USAGE);
            return;
        }

        snprintf(pcWriteBuffer, xWriteBufferLen, "xmipi cam osd %s %s%s%s %s (%d)\r\n",
                 argv[2], act, (sub != NULL) ? " " : "", (sub != NULL) ? sub : "",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "status") == 0) {
        uint32_t now = (uint32_t)tkl_system_get_millisecond();
        uint32_t elapsed = (now > s_cam.mark_ms) ? (now - s_cam.mark_ms) : 1U;
        char sp_desc[32];

        /* One field, formatted up front: the size belongs to the SP channel, so
         * printing it next to "off" claimed a geometry that is not configured -
         * those were just the struct's defaults. */
        if (s_cam.sp_on) {
            snprintf(sp_desc, sizeof(sp_desc), "%s %ux%u",
                     test_cam_fmt_name(s_cam.sp_format),
                     (unsigned int)s_cam.sp_w,
                     (unsigned int)s_cam.sp_h);
        } else {
            snprintf(sp_desc, sizeof(sp_desc), "off");
        }

        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xmipi cam status: open=%u mp_kinds=0x%x sp=%s "
                 "display=%u (%u fps) stream=%u (%u fps) bytes=%llu\r\n",
                 (unsigned int)s_cam.opened,
                 (unsigned int)s_cam.out_kinds,
                 sp_desc,
                 (unsigned int)s_cam.display_frames,
                 (unsigned int)(s_cam.display_frames * 1000U / elapsed),
                 (unsigned int)s_cam.stream_frames,
                 (unsigned int)(s_cam.stream_frames * 1000U / elapsed),
                 (unsigned long long)s_cam.stream_bytes);
        test_record_status_dump();

    } else if (test_camera_quality_cmd(pcWriteBuffer, xWriteBufferLen, argc, argv) == TRUE) {
        /* mirror_test / luminance_test */
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, XCAM_USAGE);
    }
}
