/**
 * @file tkl_mipi_csi.c
 * @brief TKL adapter - the MIPI camera peripheral for T9 (BK7259).
 *
 * @details
 * Two layers, split internally only; the public face is
 * include/mipi/tkl_mipi_csi.h.
 *
 *   hardware layer (static)  sensor power, ISP channel open/close, frame pop,
 *                            live resolution switch
 *   facade (public)          open / close / switch / set_bitrate / force_idr /
 *                            set_osd, plus get_frame and the image-quality calls
 *
 * Start / stop order follows the vendor's own dual-bond product code:
 *
 *   start : producer up -> consumers open -> bonds start -> overlays open
 *   stop  : overlays close -> **all** bonds stop -> consumers close -> producer down
 *
 * The iron rule is the third step of stop: every bond must be stopped before
 * any backend is touched. The order *between* two bonds does not matter.
 *
 * @version 0.2
 * @date 2026-08-29
 * @copyright Copyright (c) Tuya Inc.
 */

#include "sys_driver.h"
#include "tkl_mipi_csi.h"
#include "tkl_gpu_internal.h"
#include "tkl_media_buf.h"
#include "tkl_media_fmt.h"
#include "tkl_osd_engine.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_vcodec_types.h"
#include "tkl_venc_internal.h"
#include <avdk_error.h>
#include <avdk_types.h>
#include <common/avdk_pixel_types.h>
#include <components/bk_camera_bus.h>
#include <components/bk_camera_configs.h>
#include <components/bk_camera_isp_ctlr.h>
#include <components/bk_camera_sensor.h>
#include <components/bk_isp_camera.h>
#include <driver/isp_base.h>
#include <driver/isp_types.h>
#include <modules/pm.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Hardware-layer types - internal only.
 *
 * They lived in the public header while the lower layer was an API. Nothing
 * outside this file constructs them any more, so they live here rather than in
 * a header: there is no second implementation to share them with.
 * ------------------------------------------------------------------------- */


typedef struct {
    INT_T pin_scl;
    INT_T pin_sda;
    UINT8_T i2c_id;
    INT_T pin_reset;
    INT_T pin_pwdn;
    INT_T pin_xclk;

    UINT16_T sensor_width;
    UINT16_T sensor_height;
    UINT8_T sensor_fps;
    UINT32_T sensor_input_format;   /* bk_pixel_format_t, default RAW10 */
    UINT8_T hmirror;                /* Runtime mirror state, default off */
    UINT8_T vflip;                  /* Runtime flip state, default off */

    UINT16_T output_width;
    UINT16_T output_height;
    UINT32_T output_format;         /* bk_pixel_format_t, default NV12 */
    UINT8_T output_buf_cnt;
    BOOL_T enable_flexa;
    UINT8_T work_mode;              /* 0: frame mode, 1: flexa mode */
    UINT32_T read_timeout_ms;

    BOOL_T sp_enable;
    UINT16_T sp_width;
    UINT16_T sp_height;
    UINT32_T sp_format;             /* bk_pixel_format_t, default output_format */
    UINT8_T sp_buf_cnt;
    BOOL_T sp_enable_flexa;
    UINT8_T sp_work_mode;           /* 0: frame mode, 1: flexa mode */
} TUYA_MIPI_CSI_HW_CFG_T;

typedef struct {
    VOID *isp_handle;
    UINT8_T *src_buffer;
    UINT8_T buf_cnt;
    UINT16_T width;
    UINT16_T height;
    UINT32_T format;                /* bk_pixel_format_t */
    UINT32_T y_addr;
    UINT32_T u_addr;
    UINT32_T v_addr;
} TUYA_MIPI_CSI_FLEXA_SOURCE_T;

/* The public interface no longer names ISP channels - callers choose FRAME vs
 * PIXEL/STREAM and the facade derives the two legs from the enable bits. The
 * driver still has two legs underneath (frame leg = SP, flexa leg = MP), so
 * the internal routing helpers keep private selectors. */
typedef enum {
    CAM_CHN_MP = 0,    /* primary: flexa leg, feeds the pixel/stream consumers */
    CAM_CHN_SP = 1,    /* secondary: frame leg, serves get_frame() */
} CAM_CHN_T;

/* ---------------------------------------------------------------------------
 * Hardware layer - internal only. The facade below is the only caller.
 * ------------------------------------------------------------------------- */
STATIC OPERATE_RET cam_mipi_init(TUYA_MIPI_CSI_HW_CFG_T *config);
STATIC OPERATE_RET cam_mipi_deinit(void);
STATIC OPERATE_RET cam_mipi_start(void);
STATIC OPERATE_RET cam_mipi_stop(void);
/* Renamed off the tkl_mipi_csi_* prefix on purpose: the facade's public
 * entry point took that name. This is the hardware-layer half of it - it
 * reprograms one ISP channel and nothing else. */
STATIC OPERATE_RET cam_mipi_set_output(UINT8_T channel_id, UINT16_T width, UINT16_T height);
STATIC OPERATE_RET cam_mipi_get_flexa_source(TUYA_MIPI_CSI_FLEXA_SOURCE_T *source);


#define TUYA_MIPI_CSI_POWER_SETTLE_MS    20U
#define TUYA_MIPI_CSI_SYS_ANA_REG_BASE   (0x44010000U)
#define TUYA_MIPI_CSI_LDO_ANA_REG        (0x69U)

typedef struct {
    BOOL_T initialized;
    BOOL_T started;
    BOOL_T sensor_power_on;
    TUYA_MIPI_CSI_HW_CFG_T cfg;
    UINT16_T active_output_width;
    UINT16_T active_output_height;
    VOID *isp_handle;
    bk_camera_bus_t *bus_handle;
    bk_camera_sensor_handle_t sensor_handle;
    bk_isp_camera_ctlr_handle_t camera_ctlr_handle;
    /* Flexa rings parked in the ISP controller through BK_CAM_IOCTL_SET_FLEXA_BUF,
     * indexed by ISP channel id. The controller has no ioctl to unpark a slot and
     * refuses a second SET on a live one, so a ring is allocated on the first
     * channel open and held until the controller goes away - which
     * cam_mipi_cleanup_handles() does on every stop(). See
     * cam_mipi_flexa_buf_park() for why they are sized off the sensor width. */
    VOID *flexa_buf[ISP_CHANNEL_INSTANCE_MAX];
    UINT32_T flexa_buf_size[ISP_CHANNEL_INSTANCE_MAX];
} tkl_mipi_csi_ctx_t;

static tkl_mipi_csi_ctx_t s_cam_ctx;
static ISP_GAMMA_OUT_ATTR_S s_gamma_out_backup;
static BOOL_T s_gamma_out_backup_valid = FALSE;

/**
 * Facade lifecycle. INIT means the sensor is powered and probed but nothing is
 * streaming; it is what makes start() after stop() cheaper than a fresh open.
 * The saving here is smaller than UVC's - 30 ms of the 99 ms cycle without SP -
 * because the ISP bring-up cannot move out of start(). See the design spec
 * §12.6 for the numbers and why.
 */
typedef enum {
    TKL_CAM_ST_IDLE = 0,    /**< nothing built */
    TKL_CAM_ST_INIT,        /**< sensor powered and probed, not streaming */
    TKL_CAM_ST_STARTED,     /**< streaming */
} tkl_cam_state_e;

typedef struct {
    tkl_cam_state_e state;
    TUYA_MIPI_CSI_CFG_T cfg;    /* outputs[] deep-copied at init; freed at deinit */

    /* Output channels that are currently running. Seeded from the configured
     * set at start(); cancel_channel() clears an entry and resume_channel()
     * sets it again. want_* below stay fixed - they are what init() built. */
    BOOL_T  ch_active[TUYA_CAMERA_FRAME_CHANNEL_MAX];

    /* fixed shape built at init: ch1 -> SP, ch2 -> GPU, ch3 -> encoder */
    BOOL_T  want_sp;
    BOOL_T  want_display;
    BOOL_T  want_encode;

    /* a set_pp() while stopped records here per channel; start() applies it */
    BOOL_T  pp_pending[TUYA_CAMERA_FRAME_CHANNEL_MAX];
    UINT16_T pp_width[TUYA_CAMERA_FRAME_CHANNEL_MAX];
    UINT16_T pp_height[TUYA_CAMERA_FRAME_CHANNEL_MAX];

    TKL_GPU_HANDLE gpu;
    TUYA_VENC_HANDLE venc;
    TKL_OSD_ENGINE_HANDLE osd_display;
    TKL_OSD_ENGINE_HANDLE osd_encode;

    UINT16_T src_width;         /* ISP output feeding the consumers */
    UINT16_T src_height;

    /* ch1 push delivery: one task pops SP frames and calls the business cb,
     * so the cb never runs on an ISP or USB context. Lives from start() to
     * stop(); cancel/resume of ch1 only toggles ch_active[1]. */
    beken_thread_t    deliver_task;
    beken_semaphore_t deliver_done;  /**< posted once, when the task exits */
    volatile BOOL_T   deliver_exit;
    BOOL_T            deliver_live;  /**< task created and not yet joined */

    UINT32_T ch1_no_frame_ms;   /* consecutive no-frame time, for TIMEOUT_CH1 */
} tkl_cam_out_ctx_t;

STATIC tkl_cam_out_ctx_t s_cam_out;

/* ---------------------------------------------------------------------------
 * Facade constants
 * ------------------------------------------------------------------------- */

#define TUYA_MIPI_CSI_FLEXA_LINES     16U
#define TUYA_MIPI_CSI_BUF_CNT         3U
#define TUYA_MIPI_CSI_SP_BUF_CNT      2U
#define TUYA_MIPI_CSI_READ_TIMEOUT_MS 1000U
#define TUYA_MIPI_CSI_DRAIN_MS        200U
/* the frame the panel is still scanning out when the camera stops - see the
 * drain call in tkl_mipi_csi_deinit() */
#define TUYA_MIPI_CSI_OUTPUT_PIXEL_RESIDUAL 1U

#define TUYA_MIPI_CSI_MS()            ((UINT32_T)tkl_system_get_millisecond())

/* ch1 push delivery task */
#define TUYA_MIPI_CSI_DELIVER_POLL_MS   100U
#define TUYA_MIPI_CSI_DELIVER_POP_MS    500U
#define TUYA_MIPI_CSI_DELIVER_STACK     (1024U * 4U)
#define TUYA_MIPI_CSI_NO_FRAME_EVENT_MS 3000U

/* Delivered frames carry the public struct in the same pool block: the alloc
 * cb returns block base + header, and the header is padded to 64 bytes so the
 * write-through payload address stays 64-byte aligned (bk_mem_slab refuses a
 * misaligned WT mapping). */
/* one-block frame layout: see TUYA_MEDIA_BUF_HDR in tkl_media_types.h */
#define TUYA_MIPI_CSI_BUF_HDR  TUYA_MEDIA_BUF_HDR

/* ---------------------------------------------------------------------------
 * Channel view over the deep-copied outputs
 * ------------------------------------------------------------------------- */

/** The config entry for one channel, or NULL when that channel was not
 *  requested. Linear scan: output_num is capped at 4, so this is cheap. */
STATIC TUYA_MEDIA_OUTPUT_CFG_T *tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    UINT32_T i;

    if (s_cam_out.cfg.outputs == NULL) {
        return NULL;
    }

    for (i = 0U; i < s_cam_out.cfg.output_num; i++) {
        if (s_cam_out.cfg.outputs[i].channel == channel) {
            return &s_cam_out.cfg.outputs[i];
        }
    }

    return NULL;
}

/** Lifecycle and error events. event_cb may be NULL - the adapter works
 *  without it, business just learns nothing. */
STATIC VOID_T tkl_cam_out_event(TUYA_CAMERA_EVENT_E event)
{
    if (s_cam_out.cfg.event_cb != NULL) {
        s_cam_out.cfg.event_cb(event, s_cam_out.cfg.event_arg);
    }
}

/** Same, for repeatable faults: at most once per second per event value, so a
 *  persistent failure cannot flood the log or the business handler. */
STATIC VOID_T tkl_cam_out_event_throttled(TUYA_CAMERA_EVENT_E event)
{
    STATIC UINT32_T s_last_ms[32];
    UINT32_T now;

    if (event >= 32U) {
        return;
    }

    now = TUYA_MIPI_CSI_MS();
    if (s_last_ms[event] != 0U && (now - s_last_ms[event]) < 1000U) {
        return;
    }

    s_last_ms[event] = now;
    tkl_cam_out_event(event);
}


static VOID_T cam_mipi_gpio_bank_ldo_enable(VOID_T)
{
    UINT32_T reg = REG_READ(TUYA_MIPI_CSI_SYS_ANA_REG_BASE + TUYA_MIPI_CSI_LDO_ANA_REG * 4U);

    reg |= (0xFU << 28) | (0x2U << 23) | (0x7U << 19) | (0x7U << 15);
    reg &= ~(0xFU << 11);
    reg |= (0x8U << 11);
    REG_WRITE(TUYA_MIPI_CSI_SYS_ANA_REG_BASE + TUYA_MIPI_CSI_LDO_ANA_REG * 4U, reg);
}

static OPERATE_RET cam_mipi_sensor_power_enable(BOOL_T enable)
{
    pm_auxldo_ctrl_cfg_t auxldo_cfg = {0};
    int ldo_en = (enable != FALSE) ? PM_AUXLDO_ENABLE : PM_AUXLDO_DISABLE;

    tkl_log_output("camera sensor power %s (gpio_bank+1p8v+1p2v)\r\n",
                   (enable != FALSE) ? "on" : "off");

    if (enable != FALSE) {
        /* Must run before any GPIO59+ / I2C access. */
        cam_mipi_gpio_bank_ldo_enable();
    }

    auxldo_cfg.ldo = AUXLDOS_SEL_1P8V;
    auxldo_cfg.out = PM_AUXLDO_1P8V_OUT_1P8V;
    auxldo_cfg.user = PM_AUXLDO_USER_CAMERA;
    auxldo_cfg.state = ldo_en;
    if (bk_pm_auxldo_ctrl_vote(&auxldo_cfg) != BK_OK) {
        tkl_log_output("camera 1p8v ldo vote failed\r\n");
        return OPRT_COM_ERROR;
    }

    os_memset(&auxldo_cfg, 0, sizeof(auxldo_cfg));
    auxldo_cfg.ldo = AUXLDOS_SEL_1P2V;
    auxldo_cfg.out = PM_AUXLDO_1P2V_OUT_1P2V;
    auxldo_cfg.user = PM_AUXLDO_USER_CAMERA;
    auxldo_cfg.state = ldo_en;
    if (bk_pm_auxldo_ctrl_vote(&auxldo_cfg) != BK_OK) {
        tkl_log_output("camera 1p2v ldo vote failed\r\n");
        return OPRT_COM_ERROR;
    }

    if (enable != FALSE) {
        tkl_system_sleep(TUYA_MIPI_CSI_POWER_SETTLE_MS);
    }
    s_cam_ctx.sensor_power_on = (enable != FALSE) ? TRUE : FALSE;
    return OPRT_OK;
}

static void cam_mipi_runtime_reset(BOOL_T keep_config)
{
    TUYA_MIPI_CSI_HW_CFG_T cfg_backup;

    if (keep_config == TRUE) {
        os_memcpy(&cfg_backup, &s_cam_ctx.cfg, sizeof(cfg_backup));
    }

    os_memset(&s_cam_ctx, 0, sizeof(s_cam_ctx));
    s_gamma_out_backup_valid = FALSE;

    if (keep_config == TRUE) {
        os_memcpy(&s_cam_ctx.cfg, &cfg_backup, sizeof(cfg_backup));
        s_cam_ctx.initialized = TRUE;
    }
}

static void cam_mipi_cfg_normalize(TUYA_MIPI_CSI_HW_CFG_T *cfg)
{
    if (cfg->i2c_id == 0U) {
        cfg->i2c_id = 1U;
    }

    if (cfg->sensor_width == 0U) {
        cfg->sensor_width = 1920U;
    }

    if (cfg->sensor_height == 0U) {
        cfg->sensor_height = 1080U;
    }

    if (cfg->sensor_fps == 0U) {
        cfg->sensor_fps = 15U;
    }

    if (cfg->sensor_input_format == BK_PIXEL_FORMAT_INVALID ||
        cfg->sensor_input_format == BK_PIXEL_FORMAT_UNKNOW) {
        cfg->sensor_input_format = BK_PIXEL_FORMAT_RGGB10;
    }

    if (cfg->output_width == 0U) {
        cfg->output_width = cfg->sensor_width;
    }

    if (cfg->output_height == 0U) {
        cfg->output_height = cfg->sensor_height;
    }

    if (cfg->output_format == BK_PIXEL_FORMAT_INVALID ||
        cfg->output_format == BK_PIXEL_FORMAT_UNKNOW) {
        cfg->output_format = BK_PIXEL_FORMAT_NV12;
    }

    if (cfg->output_buf_cnt == 0U) {
        cfg->output_buf_cnt = 3U;
    }

    if (cfg->read_timeout_ms == 0U) {
        cfg->read_timeout_ms = 1000U;
    }

    if (cfg->enable_flexa != FALSE) {
        cfg->enable_flexa = TRUE;
        cfg->work_mode = 1U;
    }

    if (cfg->sp_enable != FALSE) {
        cfg->sp_enable = TRUE;

        if (cfg->sp_width == 0U) {
            cfg->sp_width = cfg->output_width;
        }

        if (cfg->sp_height == 0U) {
            cfg->sp_height = cfg->output_height;
        }

        if (cfg->sp_format == BK_PIXEL_FORMAT_INVALID ||
            cfg->sp_format == BK_PIXEL_FORMAT_UNKNOW) {
            cfg->sp_format = cfg->output_format;
        }

        if (cfg->sp_buf_cnt == 0U) {
            cfg->sp_buf_cnt = 2U;
        }

        if (cfg->sp_enable_flexa != FALSE) {
            cfg->sp_enable_flexa = TRUE;
            cfg->sp_work_mode = 1U;
        }
    }

    cfg->hmirror = (cfg->hmirror != 0U) ? 1U : 0U;
    cfg->vflip = (cfg->vflip != 0U) ? 1U : 0U;
}

/**
 * @brief Apply hmirror/vflip to the detected sensor
 * @return OPRT_OK on success; UNSUPPORTED sensor hooks are ignored
 */
static OPERATE_RET cam_mipi_apply_mirror(VOID_T)
{
    avdk_err_t ret;

    if (s_cam_ctx.sensor_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = bk_camera_sensor_set_hmirror(s_cam_ctx.sensor_handle, s_cam_ctx.cfg.hmirror != 0U);
    if (ret != AVDK_ERR_OK && ret != AVDK_ERR_UNSUPPORTED) {
        tkl_log_output("set_hmirror failed: %d\r\n", ret);
        return OPRT_COM_ERROR;
    }

    ret = bk_camera_sensor_set_vflip(s_cam_ctx.sensor_handle, s_cam_ctx.cfg.vflip != 0U);
    if (ret != AVDK_ERR_OK && ret != AVDK_ERR_UNSUPPORTED) {
        tkl_log_output("set_vflip failed: %d\r\n", ret);
        return OPRT_COM_ERROR;
    }

    tkl_log_output("camera mirror applied: hmirror=%u vflip=%u\r\n",
                   s_cam_ctx.cfg.hmirror, s_cam_ctx.cfg.vflip);
    return OPRT_OK;
}

static OPERATE_RET cam_mipi_cfg_validate(const TUYA_MIPI_CSI_HW_CFG_T *cfg)
{
    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->pin_scl < 0 || cfg->pin_sda < 0 || cfg->pin_xclk < 0) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->sensor_width == 0U || cfg->sensor_height == 0U || cfg->sensor_fps == 0U) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static uint32_t cam_mipi_sp_frame_size_get(void)
{
    uint32_t frame_size;

    /* ISP hardware outputs RGB888/BGR888 as RGBX with 4 bytes per pixel */
    if (s_cam_ctx.cfg.sp_format == BK_PIXEL_FORMAT_RGB888 ||
        s_cam_ctx.cfg.sp_format == BK_PIXEL_FORMAT_BGR888) {
        return (uint32_t)s_cam_ctx.cfg.sp_width * s_cam_ctx.cfg.sp_height * 4U;
    }

    frame_size = bk_image_size_get(s_cam_ctx.cfg.sp_width,
                                   s_cam_ctx.cfg.sp_height,
                                   (bk_pixel_format_t)s_cam_ctx.cfg.sp_format);
    return frame_size;
}

/**
 * @brief Park an on-chip flexa ring in the ISP controller for one channel.
 *
 * @details
 * The buffer_ioctl patch moved the flexa ring out of the ISP core: bk_isp_open()
 * now installs whatever BK_CAM_IOCTL_SET_FLEXA_BUF parked for the channel, and
 * rejects it with "ext flexa buffer too small" if it cannot cover
 * buf_cnt * width * FLEXA_LINES * 3 / 2. Without a parked buffer the vendor
 * still allocates one itself, so failing here only costs the placement choice.
 *
 * Sized for the width the channel is about to open with, which is the same
 * value bk_isp_open() recomputes from - anything larger is dead weight, and the
 * rings are big enough that the slack decides whether SRAM can hold them at all.
 * A 2304-wide sensor feeding a 640-wide MP channel would waste 120 KB sizing off
 * the sensor, which is more than the whole ring needs.
 *
 * That makes a parked slot a one-shot fit: there is no unpark ioctl and a second
 * SET on a live slot is refused, so a later re-open at a *larger* geometry cannot
 * be served. Both re-open paths avoid it - the MP restart tears the controller
 * down and parks afresh, and set_output() escalates a growing flexa-mode SP to
 * that same restart. SP is frame mode in every facade-built config
 * (cam_mipi_build_cfg_from_channels forces sp_work_mode 0), so its ring only
 * exists for a caller driving the raw config.
 *
 * The ring goes to SRAM when it fits. The ISP's own output rate does not care,
 * but the GPU and the H264 encoder downstream read it faster there.
 *
 * @param[in] ctlr    camera controller, must already exist
 * @param[in] channel ISP_MP_CHN_ID or ISP_SP_CHN_ID
 * @param[in] width   channel width the open will use, in pixels
 * @param[in] buf_cnt flexa slice count the channel will open with
 *
 * @return OPRT_OK when a ring is parked or was already parked for this channel;
 *         OPRT_COM_ERROR / OPRT_MALLOC_FAILED when not - the caller carries on
 *         and lets the vendor allocate internally
 */
static OPERATE_RET cam_mipi_flexa_buf_park(bk_isp_camera_ctlr_handle_t ctlr,
                                           UINT8_T channel,
                                           UINT16_T width,
                                           UINT8_T buf_cnt)
{
    bk_isp_camera_flexa_buf_size_t query;
    bk_isp_camera_flexa_buf_t park;
    avdk_err_t ret;

    if (ctlr == NULL || channel >= (UINT8_T)ISP_CHANNEL_INSTANCE_MAX) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_ctx.flexa_buf[channel] != NULL) {
        /* Second open of this channel under the same controller - set_output()
         * live-switching the SP, for instance. The parked ring is still the
         * controller's, and bk_isp_open() re-validates its size on its own. */
        return OPRT_OK;
    }

    os_memset(&query, 0, sizeof(query));
    query.buf_cnt = buf_cnt;
    query.width = width;
    ret = bk_isp_camera_ctlr_ioctl(ctlr, BK_CAM_IOCTL_GET_FLEXA_BUF_SIZE, &query);
    if (ret != AVDK_ERR_OK || query.size == 0U) {
        tkl_log_output("camera: flexa size query failed (%d), ch%u %ux%u x%u\r\n",
                       (int)ret,
                       (unsigned int)channel,
                       (unsigned int)width,
                       (unsigned int)buf_cnt);
        return OPRT_COM_ERROR;
    }

    s_cam_ctx.flexa_buf[channel] = tkl_media_sram_alloc(query.size, "isp_flexa");
    if (s_cam_ctx.flexa_buf[channel] == NULL) {
        tkl_log_output("camera: flexa ring %u bytes alloc failed, ch%u\r\n",
                       (unsigned int)query.size, (unsigned int)channel);
        return OPRT_MALLOC_FAILED;
    }
    s_cam_ctx.flexa_buf_size[channel] = query.size;

    os_memset(&park, 0, sizeof(park));
    park.channel = channel;
    park.buffer = s_cam_ctx.flexa_buf[channel];
    park.size = query.size;
    /* release_cb stays NULL, so the controller treats the ring as caller-owned
     * and leaves it parked instead of freeing it from its own teardown.
     * cam_mipi_cleanup_handles() frees it once the controller is gone. */
    ret = bk_isp_camera_ctlr_ioctl(ctlr, BK_CAM_IOCTL_SET_FLEXA_BUF, &park);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("camera: flexa ring park failed (%d), ch%u\r\n",
                       (int)ret, (unsigned int)channel);
        tkl_media_sram_free(s_cam_ctx.flexa_buf[channel]);
        s_cam_ctx.flexa_buf[channel] = NULL;
        s_cam_ctx.flexa_buf_size[channel] = 0U;
        return OPRT_COM_ERROR;
    }

    tkl_log_output("camera: flexa ring ch%u parked, %u bytes\r\n",
                   (unsigned int)channel, (unsigned int)query.size);
    return OPRT_OK;
}

/**
 * @brief Give back every parked flexa ring.
 *
 * @note Call only after the controller is deleted: it still holds the parked
 *       addresses until then, and it never frees them itself (release_cb NULL).
 */
static void cam_mipi_flexa_buf_release_all(void)
{
    UINT8_T channel;

    for (channel = 0U; channel < (UINT8_T)ISP_CHANNEL_INSTANCE_MAX; channel++) {
        if (s_cam_ctx.flexa_buf[channel] != NULL) {
            tkl_media_sram_free(s_cam_ctx.flexa_buf[channel]);
            s_cam_ctx.flexa_buf[channel] = NULL;
            s_cam_ctx.flexa_buf_size[channel] = 0U;
        }
    }
}

/**
 * @brief Would the ring parked for @p channel still cover an open at @p width?
 *
 * @details
 * A parked ring is one fixed size for the controller's whole life, so a re-open
 * at a larger geometry cannot be served by it. cam_mipi_set_output() asks this
 * before taking the cheap live-switch path, and rebuilds the controller instead
 * when the answer is no.
 *
 * @note The comparison is exact rather than conservative: the size we parked is
 *       the same value bk_isp_open() recomputes for the same width and buf_cnt,
 *       so anything at or above it passes the vendor's own check.
 *
 * @return TRUE when nothing is parked, the size cannot be established, or the
 *         parked ring is big enough; FALSE only when it is provably too small
 */
static BOOL_T cam_mipi_flexa_buf_covers(UINT8_T channel, UINT16_T width, UINT8_T buf_cnt)
{
    bk_isp_camera_flexa_buf_size_t query;

    if (channel >= (UINT8_T)ISP_CHANNEL_INSTANCE_MAX ||
        s_cam_ctx.flexa_buf[channel] == NULL ||
        s_cam_ctx.camera_ctlr_handle == NULL) {
        return TRUE;
    }

    os_memset(&query, 0, sizeof(query));
    query.buf_cnt = buf_cnt;
    query.width = width;
    if (bk_isp_camera_ctlr_ioctl(s_cam_ctx.camera_ctlr_handle,
                                 BK_CAM_IOCTL_GET_FLEXA_BUF_SIZE, &query) != AVDK_ERR_OK) {
        /* Cannot tell - let the open decide, bk_isp_open() validates anyway. */
        return TRUE;
    }

    return (query.size <= s_cam_ctx.flexa_buf_size[channel]) ? TRUE : FALSE;
}

static OPERATE_RET cam_mipi_open_sp_channel(void)
{
    OPERATE_RET ret;
    bk_isp_camera_channel_config_t sp_cfg;

    if (s_cam_ctx.cfg.sp_enable != TRUE) {
        return OPRT_OK;
    }

    /* Same reason as in set_pp(): the ISP takes an odd NV12 geometry but the
     * pull path cannot size it, so it must never reach the channel open. This
     * covers the init() path too, which has no set_pp() in front of it. */
    if (s_cam_ctx.cfg.sp_format == (UINT32_T)BK_PIXEL_FORMAT_NV12 &&
        (((s_cam_ctx.cfg.sp_width & 1U) != 0U) || ((s_cam_ctx.cfg.sp_height & 1U) != 0U))) {
        tkl_log_output("camera SP NV12 needs even dimensions, %ux%u rejected\r\n",
                       s_cam_ctx.cfg.sp_width, s_cam_ctx.cfg.sp_height);
        return OPRT_INVALID_PARM;
    }

    sp_cfg = (bk_isp_camera_channel_config_t)CAM_MP_NV12_RB_INSTANCE_CONFIG(
        s_cam_ctx.cfg.sp_width,
        s_cam_ctx.cfg.sp_height
    );
    sp_cfg.port_id = ISP_MP_CHN_ID;
    sp_cfg.buf_cnt = s_cam_ctx.cfg.sp_buf_cnt;
    sp_cfg.enable_flexa = s_cam_ctx.cfg.sp_enable_flexa ? 1U : 0U;
    sp_cfg.work_mode = s_cam_ctx.cfg.sp_work_mode;
    sp_cfg.format = (uint16_t)s_cam_ctx.cfg.sp_format;
    sp_cfg.width = s_cam_ctx.cfg.sp_width;
    sp_cfg.height = s_cam_ctx.cfg.sp_height;

    /* Flexa mode takes its ring from us now; frame mode never allocates one
     * (bk_isp_open() sends work_mode 0 down the complete-buffer path). */
    if (s_cam_ctx.cfg.sp_work_mode != 0U) {
        (void)cam_mipi_flexa_buf_park(s_cam_ctx.camera_ctlr_handle,
                                      ISP_SP_CHN_ID,
                                      s_cam_ctx.cfg.sp_width,
                                      s_cam_ctx.cfg.sp_buf_cnt);
    }

    ret = bk_isp_camera_channel_open(s_cam_ctx.camera_ctlr_handle, ISP_SP_CHN_ID, &sp_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_isp_camera_channel_open SP failed: %d\r\n", ret);
        return ret;
    }

    tkl_log_output( "camera SP open ok, output=%ux%u fmt=%u buf_cnt=%u flexa=%u work_mode=%u\r\n",
            s_cam_ctx.cfg.sp_width,
            s_cam_ctx.cfg.sp_height,
            (unsigned int)s_cam_ctx.cfg.sp_format,
            (unsigned int)s_cam_ctx.cfg.sp_buf_cnt,
            (unsigned int)s_cam_ctx.cfg.sp_enable_flexa,
            (unsigned int)s_cam_ctx.cfg.sp_work_mode);
    return OPRT_OK;
}

static OPERATE_RET cam_mipi_open_mp_channel(void)
{
    OPERATE_RET ret;
    bk_isp_camera_channel_config_t mp_cfg;

    mp_cfg = (bk_isp_camera_channel_config_t)CAM_MP_NV12_RB_INSTANCE_CONFIG(
        s_cam_ctx.cfg.output_width,
        s_cam_ctx.cfg.output_height
    );
    mp_cfg.port_id = ISP_MP_CHN_ID;
    mp_cfg.buf_cnt = s_cam_ctx.cfg.output_buf_cnt;
    mp_cfg.enable_flexa = s_cam_ctx.cfg.enable_flexa ? 1U : 0U;
    mp_cfg.work_mode = s_cam_ctx.cfg.work_mode;
    mp_cfg.format = (uint16_t)s_cam_ctx.cfg.output_format;
    mp_cfg.width = s_cam_ctx.cfg.output_width;
    mp_cfg.height = s_cam_ctx.cfg.output_height;

    /* Same as the SP leg above: flexa mode needs a ring parked before the open,
     * frame mode does not. */
    if (s_cam_ctx.cfg.work_mode != 0U) {
        (void)cam_mipi_flexa_buf_park(s_cam_ctx.camera_ctlr_handle,
                                      ISP_MP_CHN_ID,
                                      s_cam_ctx.cfg.output_width,
                                      s_cam_ctx.cfg.output_buf_cnt);
    }

    ret = bk_isp_camera_channel_open(s_cam_ctx.camera_ctlr_handle, ISP_MP_CHN_ID, &mp_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("bk_isp_camera_channel_open MP failed: %d\r\n", ret);
        return ret;
    }

    tkl_log_output("camera MP open ok, output=%ux%u fmt=%u buf_cnt=%u flexa=%u work_mode=%u\r\n",
            s_cam_ctx.cfg.output_width,
            s_cam_ctx.cfg.output_height,
            (unsigned int)s_cam_ctx.cfg.output_format,
            (unsigned int)s_cam_ctx.cfg.output_buf_cnt,
            (unsigned int)s_cam_ctx.cfg.enable_flexa,
            (unsigned int)s_cam_ctx.cfg.work_mode);
    return OPRT_OK;
}

static OPERATE_RET cam_mipi_sensor_check_format(bk_camera_sensor_handle_t sensor_handle,
                                                       const TUYA_MIPI_CSI_HW_CFG_T *cfg)
{
    int detect_index = 0;
    bk_camera_sensor_format_array_t format_array;

    os_memset(&format_array, 0, sizeof(format_array));

    if (sensor_handle == NULL || cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (bk_camera_sensor_query_support_formats(sensor_handle, &format_array) != OPRT_OK ||
        format_array.size == 0U) {
        tkl_log_output( "query sensor formats failed\r\n");
        return OPRT_COM_ERROR;
    }

    for (detect_index = 0; detect_index < (int)format_array.size; detect_index++) {
        if (format_array.format_array[detect_index].width == cfg->sensor_width &&
            format_array.format_array[detect_index].height == cfg->sensor_height &&
            format_array.format_array[detect_index].fps == cfg->sensor_fps) {
            break;
        }
    }

    if (detect_index >= (int)format_array.size) {
        tkl_log_output( "sensor format %ux%u@%u not supported\r\n",
                cfg->sensor_width, cfg->sensor_height, cfg->sensor_fps);
        return OPRT_NOT_SUPPORTED;
    }

    return OPRT_OK;
}

static OPERATE_RET cam_mipi_sensor_detect_and_check(TUYA_MIPI_CSI_HW_CFG_T *cfg)
{
    bk_camera_sensor_config_t sensor_cfg;
    OPERATE_RET ret;

    os_memset(&sensor_cfg, 0, sizeof(sensor_cfg));

    sensor_cfg.pin_reset = (uint8_t)cfg->pin_reset;
    sensor_cfg.pin_pwdn = (uint8_t)cfg->pin_pwdn;
    sensor_cfg.bus = s_cam_ctx.bus_handle;

    s_cam_ctx.sensor_handle = bk_camera_sensor_auto_detect(&sensor_cfg, CSI_CAMERA_PORT);
    if (s_cam_ctx.sensor_handle == NULL) {
        tkl_log_output( "sensor detect failed\r\n");
        return OPRT_COM_ERROR;
    }

    ret = cam_mipi_sensor_check_format(s_cam_ctx.sensor_handle, cfg);
    if (ret != OPRT_OK) {
        bk_camera_sensor_destroy(s_cam_ctx.sensor_handle);
        s_cam_ctx.sensor_handle = NULL;
        return ret;
    }

    return OPRT_OK;
}

static OPERATE_RET cam_mipi_build_isp_ctlr_cfg(bk_isp_camera_ctlr_config_t *isp_ctlr_cfg)
{
    if (isp_ctlr_cfg == NULL || s_cam_ctx.sensor_handle == NULL) {
        return OPRT_INVALID_PARM;
    }

    *isp_ctlr_cfg = (bk_isp_camera_ctlr_config_t)CAM_CSI_DEFAULT_RAW10_CONFIG(
        s_cam_ctx.cfg.sensor_width,
        s_cam_ctx.cfg.sensor_height,
        s_cam_ctx.cfg.sensor_fps
    );
    isp_ctlr_cfg->input_pixel_fmt = (bk_pixel_format_t)s_cam_ctx.cfg.sensor_input_format;
    isp_ctlr_cfg->sensor_object = bk_camera_sensor_get_sensor_object(s_cam_ctx.sensor_handle);
    if (isp_ctlr_cfg->sensor_object == NULL) {
        tkl_log_output( "sensor object is null\r\n");
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

static void cam_mipi_close_channels(void)
{
    if (s_cam_ctx.camera_ctlr_handle != NULL) {
        if (bk_isp_camera_channel_state_get(s_cam_ctx.camera_ctlr_handle, ISP_MP_CHN_ID) == ISP_CHANNEL_STATE_TURN_ON) {
            bk_isp_camera_channel_close(s_cam_ctx.camera_ctlr_handle, ISP_MP_CHN_ID);
        }

        if (bk_isp_camera_channel_state_get(s_cam_ctx.camera_ctlr_handle, ISP_SP_CHN_ID) == ISP_CHANNEL_STATE_TURN_ON) {
            bk_isp_camera_channel_close(s_cam_ctx.camera_ctlr_handle, ISP_SP_CHN_ID);
        }
    }

    s_cam_ctx.started = FALSE;
}

/**
 * @brief Hand back the frame-end ISR slot the vendor controller took.
 *
 * @details
 * isp_camera_ctlr_read() registers a frame-end callback in the global
 * isp_isr_handler[ISP_FRAME_END_DONE][] table, and isp_camera_ctlr_delete()
 * frees the controller without taking the entry back out. Nothing dereferences
 * the stale entry, but it holds a slot, and the table has only
 * ISP_ISR_MODULE_MAX (8) per type. Once the controller address varies across
 * open/close cycles the type fills up and the real users of frame-end - the
 * h264 and jpeg ISP bonds - start failing to register.
 *
 * Done here rather than in the vendor tree because an SDK drop reverts that
 * tree; this only protects our own call path, so report it to Beken too.
 *
 * @note Must run before bk_isp_camera_deinit(): that nulls controller->isp_handle
 *       and bk_isp_deregister_isr_callback() refuses a NULL one. Channel close
 *       does not null it, so here is the only window.
 */
static void cam_mipi_release_frame_end_isr(bk_isp_camera_ctlr_handle_t handle)
{
    bk_camera_isp_ctlr_t *ctl;
    avdk_err_t ret;

    if (handle == NULL) {
        return;
    }

    ctl = __containerof(handle, bk_camera_isp_ctlr_t, ops);

    /* Only the read path registers, and only frame-mode channels reach it - an
     * SP-less pipeline never took a slot and has nothing to give back. */
    if (ctl->read_register == 0U) {
        return;
    }

    ret = bk_isp_camera_deregister_isr_callback(handle, ISR_TYPE_FRAME_COMPLETE, ctl);
    if (ret != AVDK_ERR_OK) {
        /* The table matches by param pointer, so this failing means the vendor
         * changed what isp_camera_ctlr_read() registers as its arg and this
         * guard has quietly stopped working. Say so rather than reverting to
         * the old silence. */
        tkl_log_output("camera: frame-end isr deregister no-op (%d) - vendor "
                       "register arg may have changed\r\n", ret);
    }
}

static void cam_mipi_cleanup_handles(void)
{
    cam_mipi_close_channels();

    s_cam_ctx.isp_handle = NULL;

    if (s_cam_ctx.camera_ctlr_handle != NULL) {
        cam_mipi_release_frame_end_isr(s_cam_ctx.camera_ctlr_handle);
        bk_isp_camera_deinit(s_cam_ctx.camera_ctlr_handle);
        bk_isp_camera_delete(s_cam_ctx.camera_ctlr_handle);
        s_cam_ctx.camera_ctlr_handle = NULL;
    }

    /* Only now: the controller held the parked addresses until the delete, and
     * it never frees them itself (parked with release_cb NULL). */
    cam_mipi_flexa_buf_release_all();
}

static void cam_mipi_cleanup_resources(void)
{
    cam_mipi_cleanup_handles();

    if (s_cam_ctx.bus_handle != NULL) {
        bk_camera_bus_disable(s_cam_ctx.bus_handle);
        bk_camera_bus_delete(s_cam_ctx.bus_handle);
        s_cam_ctx.bus_handle = NULL;
    }

    if (s_cam_ctx.sensor_handle != NULL) {
        bk_camera_sensor_destroy(s_cam_ctx.sensor_handle);
        s_cam_ctx.sensor_handle = NULL;
    }

    if (s_cam_ctx.sensor_power_on != FALSE) {
        (void)cam_mipi_sensor_power_enable(FALSE);
    }
}

STATIC OPERATE_RET cam_mipi_init(TUYA_MIPI_CSI_HW_CFG_T *config)
{
    OPERATE_RET ret;
    TUYA_MIPI_CSI_HW_CFG_T cfg_local;
    bk_camera_bus_config_t bus_cfg;

    if (config == NULL) {
        return OPRT_INVALID_PARM;
    }

    cfg_local = *config;
    cam_mipi_cfg_normalize(&cfg_local);
    ret = cam_mipi_cfg_validate(&cfg_local);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (s_cam_ctx.started == TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_cam_ctx.initialized == TRUE) {
        cam_mipi_cleanup_resources();
    }

    cam_mipi_runtime_reset(FALSE);
    os_memcpy(&s_cam_ctx.cfg, &cfg_local, sizeof(cfg_local));

    ret = cam_mipi_sensor_power_enable(TRUE);
    if (ret != OPRT_OK) {
        cam_mipi_runtime_reset(FALSE);
        return ret;
    }

    os_memset(&bus_cfg, 0, sizeof(bus_cfg));
    bus_cfg.pin_scl = (uint8_t)s_cam_ctx.cfg.pin_scl;
    bus_cfg.pin_sda = (uint8_t)s_cam_ctx.cfg.pin_sda;
    bus_cfg.i2c_id = s_cam_ctx.cfg.i2c_id;
    bus_cfg.data_size = 1U;
    bus_cfg.timeout_ms = 2000U;
    bus_cfg.mipi_port_en = 1U;
    bus_cfg.dvp_port_en = 0U;
    /* 4.0.1.11 moved pin_xclk from bk_camera_sensor_config_t to bk_camera_bus_config_t. */
    bus_cfg.pin_xclk = (uint8_t)s_cam_ctx.cfg.pin_xclk;

    s_cam_ctx.bus_handle = bk_camera_bus_new(&bus_cfg);
    if (s_cam_ctx.bus_handle == NULL) {
        tkl_log_output( "camera bus new failed\r\n");
        (void)cam_mipi_sensor_power_enable(FALSE);
        cam_mipi_runtime_reset(FALSE);
        return OPRT_COM_ERROR;
    }

    ret = bk_camera_bus_enable(s_cam_ctx.bus_handle);
    if (ret != OPRT_OK) {
        tkl_log_output( "camera bus enable failed: %d\r\n", ret);
        goto err_init;
    }

    ret = cam_mipi_sensor_detect_and_check(&s_cam_ctx.cfg);
    if (ret != OPRT_OK) {
        goto err_init;
    }

    s_cam_ctx.initialized = TRUE;
    return OPRT_OK;

err_init:
    cam_mipi_cleanup_resources();
    cam_mipi_runtime_reset(FALSE);
    return ret;
}

static OPERATE_RET cam_mipi_output_validate(UINT16_T width, UINT16_T height)
{
    if (width == 0U || height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_ctx.initialized != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (width > s_cam_ctx.cfg.sensor_width || height > s_cam_ctx.cfg.sensor_height) {
        tkl_log_output("ISP output %ux%u exceeds sensor %ux%u\r\n",
                width, height,
                s_cam_ctx.cfg.sensor_width,
                s_cam_ctx.cfg.sensor_height);
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

/**
 * @brief Set ISP output resolution of the given channel
 * @param[in] channel_id CAM_CHN_MP or CAM_CHN_SP
 * @param[in] width new output width in pixels
 * @param[in] height new output height in pixels
 * @return OPRT_OK on success; on live reopen failure the channel is left closed while
 *         the other channel keeps streaming
 * @note MP always rebuilds via stop()+start(); only SP live-switches. Two
 *       independent reasons, either one fatal on its own:
 *       - MP alone: closing the last channel drops the camera FSM to INIT, and the
 *         channel_open that follows needs CAM_FSM_ENABLE, so it fails.
 *       - MP with SP up: bk_isp_close() disables port and dev only once *every*
 *         channel is closed, so the MP **flexa** channel gets reprogrammed under a
 *         live port. That reloads the flexa shadow registers mid-frame and takes
 *         the ISP down.
 *       Closing SP first only degenerates into the first case. SP carries no flexa
 *       and switches live without disturbing MP.
 */
STATIC OPERATE_RET cam_mipi_set_output(UINT8_T channel_id, UINT16_T width, UINT16_T height)
{
    OPERATE_RET ret;
    uint8_t isp_chn_id;

    if (channel_id != CAM_CHN_MP &&
        channel_id != CAM_CHN_SP) {
        return OPRT_INVALID_PARM;
    }

    if (channel_id == CAM_CHN_SP && s_cam_ctx.cfg.sp_enable != TRUE) {
        tkl_log_output("set_output SP rejected: SP not enabled\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    ret = cam_mipi_output_validate(width, height);
    if (ret != OPRT_OK) {
        return ret;
    }

    /* record target geometry */
    if (channel_id == CAM_CHN_MP) {
        s_cam_ctx.cfg.output_width = width;
        s_cam_ctx.cfg.output_height = height;
    } else {
        s_cam_ctx.cfg.sp_width = width;
        s_cam_ctx.cfg.sp_height = height;
    }

    /* not streaming yet: defer to next start() */
    if (s_cam_ctx.started != TRUE || s_cam_ctx.camera_ctlr_handle == NULL) {
        return OPRT_OK;
    }

    /*
     * MP is the flexa channel and does not survive a live channel swap in either
     * configuration - see the @note above. SP is frame-mode and does, so it keeps the
     * cheap path below. This branch rebuilds SP too when it is open; that is the point,
     * not a side effect.
     *
     * A flexa-mode SP also belongs here when the new geometry outgrows the ring
     * parked for it: that ring is one fixed size for the controller's life (there
     * is no unpark ioctl), so only a rebuild can park a bigger one. Every
     * facade-built config forces SP to frame mode and never reaches this; it
     * guards a caller driving the raw config.
     */
    if (channel_id == CAM_CHN_MP ||
        (channel_id == CAM_CHN_SP &&
         s_cam_ctx.cfg.sp_work_mode != 0U &&
         cam_mipi_flexa_buf_covers(ISP_SP_CHN_ID, width, s_cam_ctx.cfg.sp_buf_cnt) != TRUE)) {
        tkl_log_output("set_output ch%u restart, %ux%u\r\n",
                       (unsigned int)channel_id, width, height);
        ret = cam_mipi_stop();
        if (ret != OPRT_OK) {
            tkl_log_output("set_output ch%u restart stop failed: %d\r\n",
                           (unsigned int)channel_id, ret);
            return ret;
        }
        ret = cam_mipi_start();
        if (ret != OPRT_OK) {
            tkl_log_output("set_output ch%u restart start failed: %d\r\n",
                           (unsigned int)channel_id, ret);
            return ret;
        }
        tkl_log_output("set_output ch%u restart ok, %ux%u\r\n",
                       (unsigned int)channel_id, width, height);
        return OPRT_OK;
    }

    /* SP only from here down - MP returned through the restart branch above. */
    isp_chn_id = ISP_SP_CHN_ID;

    if (bk_isp_camera_channel_state_get(s_cam_ctx.camera_ctlr_handle, isp_chn_id)
        == ISP_CHANNEL_STATE_TURN_ON) {
        ret = bk_isp_camera_channel_close(s_cam_ctx.camera_ctlr_handle, isp_chn_id);
        if (ret != OPRT_OK) {
            tkl_log_output("set_output close ch%u failed: %d\r\n",
                           (unsigned int)channel_id, ret);
            return ret;
        }
    }

    ret = cam_mipi_open_sp_channel();
    if (ret != OPRT_OK) {
        tkl_log_output("set_output reopen ch%u failed: %d (channel left closed)\r\n",
                       (unsigned int)channel_id, ret);
        return ret;
    }

    /* active_output_* tracks MP only and is refreshed by cam_mipi_start(). */

    tkl_log_output("set_output ch%u live switch ok, %ux%u\r\n",
                   (unsigned int)channel_id, width, height);
    return OPRT_OK;
}

STATIC OPERATE_RET cam_mipi_deinit(void)
{
    if (s_cam_ctx.started == TRUE) {
        OPERATE_RET ret = cam_mipi_stop();
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    cam_mipi_cleanup_resources();
    cam_mipi_runtime_reset(FALSE);
    return OPRT_OK;
}

STATIC OPERATE_RET cam_mipi_start(void)
{
    OPERATE_RET ret = OPRT_OK;
    bk_isp_camera_ctlr_config_t isp_ctlr_cfg;
    bk_camera_sensor_format_t sensor_format;
    bk_camera_isp_ctlr_t *control = NULL;

    if (s_cam_ctx.initialized != TRUE ||
        s_cam_ctx.bus_handle == NULL ||
        s_cam_ctx.sensor_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_cam_ctx.started == TRUE) {
        return OPRT_OK;
    }

    if (s_cam_ctx.active_output_width != 0U &&
        (s_cam_ctx.active_output_width != s_cam_ctx.cfg.output_width ||
         s_cam_ctx.active_output_height != s_cam_ctx.cfg.output_height)) {
        tkl_log_output("camera ISP output changed: %ux%u -> %ux%u\r\n",
                s_cam_ctx.active_output_width,
                s_cam_ctx.active_output_height,
                s_cam_ctx.cfg.output_width,
                s_cam_ctx.cfg.output_height);
    }

    cam_mipi_cleanup_handles();

    ret = cam_mipi_build_isp_ctlr_cfg(&isp_ctlr_cfg);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&sensor_format, 0, sizeof(sensor_format));
    sensor_format.width = s_cam_ctx.cfg.sensor_width;
    sensor_format.height = s_cam_ctx.cfg.sensor_height;
    sensor_format.fps = s_cam_ctx.cfg.sensor_fps;

    ret = bk_camera_sensor_init(s_cam_ctx.sensor_handle);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_camera_sensor_init failed: %d\r\n", ret);
        goto err_start;
    }

    ret = bk_camera_sensor_set_format(s_cam_ctx.sensor_handle, &sensor_format);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_camera_sensor_set_format failed: %d\r\n", ret);
        goto err_start;
    }

    /* Apply after set_format: init table may reset mirror regs. */
    ret = cam_mipi_apply_mirror();
    if (ret != OPRT_OK) {
        goto err_start;
    }

    ret = bk_camera_isp_ctlr_new(&s_cam_ctx.camera_ctlr_handle);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_camera_isp_ctlr_new failed: %d\r\n", ret);
        goto err_start;
    }

    ret = bk_isp_camera_dev_init(s_cam_ctx.camera_ctlr_handle);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_isp_camera_dev_init failed: %d\r\n", ret);
        goto err_start;
    }

    control = __containerof(s_cam_ctx.camera_ctlr_handle, bk_camera_isp_ctlr_t, ops);
    s_cam_ctx.isp_handle = control->isp_handle;

    ret = bk_isp_camera_port_init(s_cam_ctx.camera_ctlr_handle, &isp_ctlr_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_isp_camera_port_init failed: %d\r\n", ret);
        goto err_start;
    }

    ret = cam_mipi_open_mp_channel();
    if (ret != OPRT_OK) {
        goto err_start;
    }

    ret = cam_mipi_open_sp_channel();
    if (ret != OPRT_OK) {
        goto err_start;
    }

    s_cam_ctx.active_output_width = s_cam_ctx.cfg.output_width;
    s_cam_ctx.active_output_height = s_cam_ctx.cfg.output_height;
    s_cam_ctx.started = TRUE;
    tkl_log_output("camera start ok, sensor=%ux%u@%u, isp_output=%ux%u fmt=%u hmirror=%u vflip=%u\r\n",
            s_cam_ctx.cfg.sensor_width,
            s_cam_ctx.cfg.sensor_height,
            s_cam_ctx.cfg.sensor_fps,
            s_cam_ctx.cfg.output_width,
            s_cam_ctx.cfg.output_height,
            (unsigned int)s_cam_ctx.cfg.output_format,
            s_cam_ctx.cfg.hmirror,
            s_cam_ctx.cfg.vflip);
    return OPRT_OK;

err_start:
    cam_mipi_cleanup_handles();
    return ret;
}

STATIC OPERATE_RET cam_mipi_stop(void)
{
    if (s_cam_ctx.initialized != TRUE) {
        return OPRT_OK;
    }

    if (s_cam_ctx.started != TRUE) {
        if (s_cam_ctx.camera_ctlr_handle == NULL) {
            return OPRT_OK;
        }
    }

    cam_mipi_cleanup_handles();

    /* Say so if a puller kept frames. The facade close() drains this owner, but
     * legacy consumers stop here instead, and without this a leak stays silent
     * until the slab runs dry - which is exactly how one went unnoticed for 26
     * rounds of xrecog before surfacing as "alloc failed". Report, never force:
     * business may still hold the pointer. */
    {
        UINT32_T held = tkl_media_buf_inflight(TUYA_MEDIA_BUF_OWNER_CAMERA_FRAME);

        if (held != 0U) {
            tkl_log_output("camera stop: %u pulled frame(s) never released - "
                           "call buf.free_cb(buf.data) after every get_frame\r\n",
                           (unsigned int)held);
        }
    }

    return OPRT_OK;
}

/**
 * @brief Read one ISP frame via camera_ctlr->read into caller buffer
 * @param[in] chn_id ISP channel id (MP/SP frame mode)
 * @param[in] format pixel format
 * @param[in] width frame width
 * @param[in] height frame height
 * @param[out] dst caller buffer
 * @param[in] dst_size buffer capacity
 * @param[in] timeout_ms read timeout
 * @param[out] out_len packed bytes written
 * @return OPRT_OK on success
 * @note Dequeue is owned by cam_thread (same path as arm_probe). Flexa is rejected by caller.
 */
STATIC OPERATE_RET cam_mipi_pop_copy_frame(UINT8_T chn_id,
                                                    UINT32_T format,
                                                    UINT16_T width,
                                                    UINT16_T height,
                                                    UINT8_T *dst,
                                                    UINT32_T dst_size,
                                                    UINT32_T timeout_ms,
                                                    UINT32_T *out_len)
{
    avdk_err_t read_ret;
    UINT32_T need_size;

    if (dst == NULL || dst_size == 0U || width == 0U || height == 0U ||
        out_len == NULL ||
        (chn_id != ISP_MP_CHN_ID && chn_id != ISP_SP_CHN_ID)) {
        return OPRT_INVALID_PARM;
    }

    *out_len = 0U;

    if (format == (UINT32_T)BK_PIXEL_FORMAT_NV12) {
        if (((width & 1U) != 0U) || ((height & 1U) != 0U)) {
            return OPRT_INVALID_PARM;
        }
        need_size = (UINT32_T)width * (UINT32_T)height * 3U / 2U;
    } else if (format == (UINT32_T)BK_PIXEL_FORMAT_RGB888 ||
               format == (UINT32_T)BK_PIXEL_FORMAT_BGR888) {
        /* ISP outputs RGB888/BGR888 as RGBX (4 bytes/pixel). */
        need_size = (UINT32_T)width * (UINT32_T)height * 4U;
    } else {
        need_size = bk_image_size_get(width, height, (bk_pixel_format_t)format);
        if (need_size == 0U || (need_size % (UINT32_T)height) != 0U) {
            return OPRT_NOT_SUPPORTED;
        }
    }

    if (dst_size < need_size) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_ctx.camera_ctlr_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* MP/SP 帧模式：走 camera_ctlr->read，由 cam_thread 统一 dequeue，与 arm_probe 一致 */
    read_ret = bk_isp_camera_read(s_cam_ctx.camera_ctlr_handle, chn_id,
                                  dst, need_size, timeout_ms);
    if (read_ret != AVDK_ERR_OK) {
        tkl_log_output("camera chn%u read failed: %d timeout_ms:%d\r\n",
                       (unsigned int)chn_id, (int)read_ret, timeout_ms);
        return OPRT_COM_ERROR;
    }

    *out_len = need_size;
    return OPRT_OK;
}


/**
 * @brief Work out the pull geometry for a channel, shared by both get_frame forms.
 *
 * @param[in]  channel_id  CAM_CHN_MP / CAM_CHN_SP
 * @param[out] isp_chn     ISP_MP_CHN_ID / ISP_SP_CHN_ID
 * @param[out] width       channel output width
 * @param[out] height      channel output height
 * @param[out] format      bk_pixel_format_t the channel was opened with
 * @param[out] size        bytes one frame occupies
 *
 * @note Rejects flexa: that channel streams into a bond and has no frame queue.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET cam_mipi_pull_geometry(UINT8_T channel_id,
                                                   UINT8_T *isp_chn,
                                                   UINT16_T *width,
                                                   UINT16_T *height,
                                                   UINT32_T *format,
                                                   UINT32_T *size)
{
    if (channel_id == CAM_CHN_SP) {
        if (s_cam_ctx.cfg.sp_enable != TRUE) {
            return OPRT_RESOURCE_NOT_READY;
        }
        if (s_cam_ctx.cfg.sp_enable_flexa != FALSE || s_cam_ctx.cfg.sp_work_mode != 0U) {
            tkl_log_output(
                    "get_frame is not supported in SP flexa mode, current enable_flexa=%u work_mode=%u\r\n",
                    (unsigned int)s_cam_ctx.cfg.sp_enable_flexa,
                    (unsigned int)s_cam_ctx.cfg.sp_work_mode);
            return OPRT_NOT_SUPPORTED;
        }

        *isp_chn = ISP_SP_CHN_ID;
        *width = s_cam_ctx.cfg.sp_width;
        *height = s_cam_ctx.cfg.sp_height;
        *format = s_cam_ctx.cfg.sp_format;
        *size = cam_mipi_sp_frame_size_get();
    } else if (channel_id == CAM_CHN_MP) {
        if (s_cam_ctx.cfg.enable_flexa != FALSE || s_cam_ctx.cfg.work_mode != 0U) {
            tkl_log_output(
                    "get_frame is not supported in MP flexa mode, current enable_flexa=%u work_mode=%u, use gpu/flexa path or reopen camera in frame mode\r\n",
                    (unsigned int)s_cam_ctx.cfg.enable_flexa,
                    (unsigned int)s_cam_ctx.cfg.work_mode);
            return OPRT_NOT_SUPPORTED;
        }

        *isp_chn = ISP_MP_CHN_ID;
        *width = s_cam_ctx.cfg.output_width;
        *height = s_cam_ctx.cfg.output_height;
        *format = s_cam_ctx.cfg.output_format;
        *size = bk_image_size_get(*width, *height, (bk_pixel_format_t)*format);
    } else {
        return OPRT_INVALID_PARM;
    }

    return (*size != 0U) ? OPRT_OK : OPRT_NOT_SUPPORTED;
}

/*
 * The struct and the frame come out of one pool block with the struct at the
 * base, so the release callback frees both through the struct pointer. The
 * caller only ever calls free_cb - it never learns the pool.
 */
STATIC VOID_T tkl_mipi_csi_frame_free(TUYA_MEDIA_BUF_T *frame)
{
    if (frame == NULL) {
        return;
    }
    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_CAMERA_FRAME, (UINT8_T *)frame);
}

OPERATE_RET tkl_mipi_csi_get_frame(TUYA_CAMERA_FRAME_CHANNEL_E channel,
                                   TUYA_MEDIA_BUF_T **out, UINT32_T timeout_ms)
{
    TUYA_MEDIA_BUF_T *buf;
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    OPERATE_RET op_ret;
    UINT8_T isp_channel_id = 0U;
    UINT16_T width = 0U;
    UINT16_T height = 0U;
    UINT32_T format = 0U;
    UINT32_T size = 0U;
    UINT32_T out_len = 0U;
    UINT32_T wait_ms;

    if (out == NULL) {
        return OPRT_INVALID_PARM;
    }
    *out = NULL;

    if (s_cam_out.state == TKL_CAM_ST_IDLE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ch = tkl_cam_out_ch(channel);
    if (ch == NULL) {
        return OPRT_INVALID_PARM;      /* channel not configured */
    }

    /* This version pulls ch1 (SP) only. The topology pins ch0 to the vendor MP
     * tap that has not shipped, and ch2/ch3 never park frames (push-only). */
    if (channel != TUYA_CAMERA_FRAME_CHANNEL_1) {
        return OPRT_NOT_SUPPORTED;
    }

    if (ch->cb != NULL) {
        return OPRT_NOT_SUPPORTED;     /* push-only channel */
    }

    if (s_cam_out.state != TKL_CAM_ST_STARTED || s_cam_out.ch_active[1] != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    op_ret = cam_mipi_pull_geometry(CAM_CHN_SP, &isp_channel_id,
                                             &width, &height, &format, &size);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    /* UNCODED, like every vendor frame-mode reader (isp_func_test, dvp_func_test,
     * qr_demo, motion_detect) and like the ISP's own frame_buffer[] - the source
     * of the copy is in this same heap. It is not a cacheability choice: both
     * slab heaps carry the same non-cacheable MPU attribute. It keeps a 1.4 MB
     * pixel frame out of the bitstream budget. One block carries the struct
     * and the frame, so a single free returns both. */
    buf = (TUYA_MEDIA_BUF_T *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL,
                    (UINT32_T)sizeof(TUYA_MEDIA_BUF_T) + size,
                    TUYA_MEDIA_BUF_OWNER_CAMERA_FRAME);
    if (buf == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    wait_ms = (timeout_ms != 0U) ? timeout_ms : TUYA_MIPI_CSI_READ_TIMEOUT_MS;

    op_ret = cam_mipi_pop_copy_frame(isp_channel_id, format, width, height,
                                              (UINT8_T *)(buf + 1), size, wait_ms, &out_len);
    if (op_ret != OPRT_OK) {
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_CAMERA_FRAME, (UINT8_T *)buf);
        return op_ret;
    }

    os_memset(buf, 0, sizeof(*buf));
    buf->buffer = (UINT8_T *)(buf + 1);
    buf->length = out_len;
    /* the whole point: business releases through this and never learns the pool */
    buf->free_cb = tkl_mipi_csi_frame_free;
    buf->channel = channel;
    buf->timestamp = (UINT64_T)tkl_system_get_millisecond();
    buf->width = width;
    buf->height = height;
    buf->format = tkl_media_fmt_from_bk(format);

    *out = buf;
    return OPRT_OK;
}

OPERATE_RET tkl_mipi_csi_set_mirror_flip(BOOL_T hmirror, BOOL_T vflip)
{
    if (s_cam_ctx.initialized != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    s_cam_ctx.cfg.hmirror = (hmirror == TRUE) ? 1U : 0U;
    s_cam_ctx.cfg.vflip = (vflip == TRUE) ? 1U : 0U;

    if (s_cam_ctx.started != TRUE || s_cam_ctx.sensor_handle == NULL) {
        return OPRT_OK;
    }

    return cam_mipi_apply_mirror();
}

OPERATE_RET tkl_mipi_csi_get_mirror_flip(BOOL_T *hmirror, BOOL_T *vflip)
{
    if (s_cam_ctx.initialized != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (hmirror != NULL) {
        *hmirror = (s_cam_ctx.cfg.hmirror != 0U) ? TRUE : FALSE;
    }
    if (vflip != NULL) {
        *vflip = (s_cam_ctx.cfg.vflip != 0U) ? TRUE : FALSE;
    }

    return OPRT_OK;
}

/**
 * @brief Get ISP exposure-normalized luminance via camera IOCTL
 * @param[out] luminance (meanLum * 1e6) / (exposure[0] * 39)
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_mipi_csi_get_exposure_luminance(UINT32_T *luminance)
{
    avdk_err_t ret;
    uint32_t lum = 0U;

    if (luminance == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_ctx.started != TRUE || s_cam_ctx.camera_ctlr_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = bk_isp_camera_ctlr_ioctl(s_cam_ctx.camera_ctlr_handle,
                                   BK_CAM_IOCTL_GET_EXPOSURE_LUMINANCE,
                                   &lum);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("get exposure luminance failed: %d\r\n", ret);
        return OPRT_COM_ERROR;
    }

    *luminance = (UINT32_T)lum;
    return OPRT_OK;
}

/**
 * @brief Switch ISP output between gray and color via CPROC saturation
 * @param[in] enable TRUE=gray (saturation 0, manual), FALSE=restore sensor default CPROC
 * @return OPRT_OK on success
 * @note Requires camera started. Gray keeps brightness/contrast/hue of the current CPROC.
 */
OPERATE_RET tkl_mipi_csi_set_color_to_gray(BOOL_T enable)
{
    avdk_err_t ret;
    bk_isp_cproc_attr_t cproc = {0};

    if (s_cam_ctx.started != TRUE || s_cam_ctx.camera_ctlr_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (enable != FALSE) {
        ret = bk_isp_camera_ctlr_ioctl(s_cam_ctx.camera_ctlr_handle,
                                       BK_CAM_IOCTL_GET_CPROC,
                                       &cproc);
        if (ret != AVDK_ERR_OK) {
            tkl_log_output("get cproc failed: %d\r\n", ret);
            return OPRT_COM_ERROR;
        }
        cproc.enable = 1U;
        cproc.op_type = 1U; /* OP_TYPE_MANUAL */
        cproc.manual.saturation = 0U;
    } else {
        if (s_cam_ctx.sensor_handle == NULL) {
            return OPRT_RESOURCE_NOT_READY;
        }
        ret = bk_camera_sensor_ioctl(s_cam_ctx.sensor_handle,
                                     BK_CAMERA_SENSOR_IOCTL_GET_DEFAULT_CPROC,
                                     &cproc);
        if (ret != AVDK_ERR_OK) {
            tkl_log_output("get default cproc failed: %d\r\n", ret);
            return OPRT_COM_ERROR;
        }
    }

    ret = bk_isp_camera_ctlr_ioctl(s_cam_ctx.camera_ctlr_handle,
                                   BK_CAM_IOCTL_SET_CPROC,
                                   &cproc);
    if (ret != AVDK_ERR_OK) {
        tkl_log_output("set cproc failed: %d\r\n", ret);
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

/**
 * @brief Switch ISP gamma-out between night curve and backed-up day curve
 * @param[in] enable TRUE=apply night gamma-out (backup current first), FALSE=restore backup
 * @return OPRT_OK on success
 * @note Requires camera started. Night curve fields are left for product fill-in.
 */
OPERATE_RET tkl_mipi_csi_set_night_gamma(BOOL_T enable)
{
    int ret;
    isp_control_t *isp = NULL;
    ISP_GAMMA_OUT_ATTR_S gamma_out = {0};

    if (s_cam_ctx.started != TRUE || s_cam_ctx.isp_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    isp = (isp_control_t *)s_cam_ctx.isp_handle;

    if (enable != FALSE) {
        if (s_gamma_out_backup_valid == FALSE) {
            ret = VSI_MPI_ISP_GetGammaOutAttr(isp->port, &gamma_out);
            if (ret != 0) {
                tkl_log_output("get gamma out failed: %d\r\n", ret);
                return OPRT_COM_ERROR;
            }
            os_memcpy(&s_gamma_out_backup, &gamma_out, sizeof(s_gamma_out_backup));
            s_gamma_out_backup_valid = TRUE;
        }
        else{
            os_memcpy(&gamma_out, &s_gamma_out_backup, sizeof(gamma_out));   
        }
        vsi_u16_t  night_defY[ISP_GAMMA_OUT_NODE_NUM] = {
            31,   57,  79, 101, 122, 142, 161, 181, 199, 218,
            236, 253, 271, 288, 305, 322, 339, 355, 372, 388,
            404, 420, 436, 452, 467, 483, 498, 514, 529, 544,
            559, 574, 589, 604, 619, 633, 648, 663, 677, 692,
            706, 720, 735, 749, 763, 777, 791, 805, 819, 833,
            847, 861, 875, 888, 902, 916, 929, 943, 956, 970,
            983, 997, 1010, 1023
        };
        os_memcpy(gamma_out.curve.redY, night_defY, sizeof(night_defY));
        os_memcpy(gamma_out.curve.greenY, night_defY, sizeof(night_defY));
        os_memcpy(gamma_out.curve.blueY, night_defY, sizeof(night_defY));

        ret = VSI_MPI_ISP_SetGammaOutAttr(isp->port, &gamma_out);
        if (ret != 0) {
            tkl_log_output("set night gamma out failed: %d\r\n", ret);
            return OPRT_COM_ERROR;
        }
    } else {
        if (s_gamma_out_backup_valid == FALSE) {
            return OPRT_RESOURCE_NOT_READY;
        }

        ret = VSI_MPI_ISP_SetGammaOutAttr(isp->port, &s_gamma_out_backup);
        if (ret != 0) {
            tkl_log_output("restore gamma out failed: %d\r\n", ret);
            return OPRT_COM_ERROR;
        }
    }

    return OPRT_OK;
}

STATIC OPERATE_RET cam_mipi_get_flexa_source(TUYA_MIPI_CSI_FLEXA_SOURCE_T *source)
{
    isp_control_t *isp = NULL;

    if (source == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_ctx.started != TRUE || s_cam_ctx.isp_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    isp = (isp_control_t *)s_cam_ctx.isp_handle;
    if (isp->chn[ISP_MP_CHN_ID].y_addr == 0U || isp->chn[ISP_MP_CHN_ID].buf_cnt == 0U) {
        return OPRT_RESOURCE_NOT_READY;
    }

    os_memset(source, 0, sizeof(*source));
    source->isp_handle = s_cam_ctx.isp_handle;
    source->src_buffer = (UINT8_T *)(uintptr_t)isp->chn[ISP_MP_CHN_ID].y_addr;
    source->buf_cnt = isp->chn[ISP_MP_CHN_ID].buf_cnt;
    source->width = s_cam_ctx.cfg.output_width;
    source->height = s_cam_ctx.cfg.output_height;
    source->format = s_cam_ctx.cfg.output_format;
    source->y_addr = isp->chn[ISP_MP_CHN_ID].y_addr;
    source->u_addr = isp->chn[ISP_MP_CHN_ID].u_addr;
    source->v_addr = isp->chn[ISP_MP_CHN_ID].v_addr;

    return OPRT_OK;
}





/* Shared by close() and switch(): both need the exact same teardown, and the
 * order inside it is the part that must not drift. */
STATIC VOID_T tkl_cam_out_consumers_close(VOID_T);
STATIC OPERATE_RET tkl_cam_out_consumers_open(VOID_T);

/* ---------------------------------------------------------------------------
 * Delivery
 * ------------------------------------------------------------------------- */

/**
 * @brief GPU frame allocator; display frames are CPU-visible and hardware-read.
 *
 * The returned pointer carries the public struct in front of the pixels (see
 * TUYA_MIPI_CSI_BUF_HDR), so the frame done hook can hand business a buffer it
 * may keep and release through free_cb.
 *
 * @param[in] size bytes wanted
 * @param[in] arg  unused
 * @return buffer or NULL
 */
STATIC VOID *tkl_cam_out_display_alloc(UINT32_T size, VOID *arg)
{
    VOID *block;

    (void)arg;

    block = tkl_media_buf_alloc(TKL_BUF_USE_PIXEL_WT, TUYA_MIPI_CSI_BUF_HDR + size,
                                TUYA_MEDIA_BUF_OWNER_CAMERA_DISPLAY);
    if (block == NULL) {
        tkl_cam_out_event_throttled(TUYA_CAMERA_EVENT_ERR_ALLOC);
        return NULL;
    }

    return (UINT8_T *)block + TUYA_MIPI_CSI_BUF_HDR;
}

STATIC OPERATE_RET tkl_cam_out_display_free(VOID *ptr, VOID *arg)
{
    (void)arg;

    if (ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_CAMERA_DISPLAY,
                       (UINT8_T *)ptr - TUYA_MIPI_CSI_BUF_HDR);
    return OPRT_OK;
}

/** Public free_cb for a delivered display frame: releases the whole pool block
 *  (struct + frame) through the struct pointer. */
STATIC VOID_T tkl_cam_out_display_buf_free(TUYA_MEDIA_BUF_T *frame)
{
    if (frame == NULL) {
        return;
    }
    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_CAMERA_DISPLAY, (UINT8_T *)frame);
}

/**
 * @brief GPU frame-done hook: wrap and hand the picture to business.
 *
 * @param[in] frame      composed picture
 * @param[in] frame_size bytes
 * @param[in] arg        unused
 *
 * @note Runs on the GPU's flexa worker thread, concurrently with the encoder
 *       thread that drives tkl_cam_out_stream_cb(). Ownership passes to
 *       business through TUYA_MEDIA_BUF_T::free_cb; if business does not release
 *       it, the pool drains and the pipeline stalls rather than corrupting.
 */
STATIC VOID tkl_cam_out_display_done(VOID *frame, UINT32_T frame_size, VOID *arg)
{
    TUYA_MEDIA_BUF_T *buf;
    TUYA_MEDIA_OUTPUT_CFG_T *ch;

    (void)arg;

    if (frame == NULL) {
        return;
    }

    buf = (TUYA_MEDIA_BUF_T *)((UINT8_T *)frame - TUYA_MIPI_CSI_BUF_HDR);
    ch = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_2);
    if (ch == NULL || ch->cb == NULL) {
        /* nobody listening (ch2 cancelled mid-flight): recycle the block */
        tkl_cam_out_display_buf_free(buf);
        return;
    }

    os_memset(buf, 0, sizeof(*buf));
    buf->buffer = (UINT8_T *)frame;
    buf->length = frame_size;
    buf->free_cb = tkl_cam_out_display_buf_free;
    buf->channel = TUYA_CAMERA_FRAME_CHANNEL_2;
    buf->timestamp = (UINT64_T)tkl_system_get_millisecond();
    buf->width = ch->width;
    buf->height = ch->height;
    buf->format = ch->format;
    buf->user_args = ch->arg;

    ch->cb(buf);
}

STATIC VOID *tkl_cam_out_stream_alloc(UINT32_T size, VOID *arg)
{
    VOID *block;

    (void)arg;

    block = tkl_media_buf_alloc(TKL_BUF_USE_STREAM, size, TUYA_MEDIA_BUF_OWNER_CAMERA_STREAM);
    if (block == NULL) {
        tkl_cam_out_event_throttled(TUYA_CAMERA_EVENT_ERR_ALLOC);
    }
    return block;
}

STATIC OPERATE_RET tkl_cam_out_stream_free(VOID *ptr, VOID *arg)
{
    (void)arg;

    if (ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_CAMERA_STREAM, ptr);
    return OPRT_OK;
}

/**
 * @brief Map the vendor stream descriptor onto the public format.
 * @param[in]  stream vendor stream descriptor
 * @param[out] buf    receives the format; for H264 the frame type rides in it
 *                    (H264_I / H264_P, bare H264 when the vendor gives no
 *                    usable type)
 */
STATIC VOID tkl_cam_out_stream_meta(CONST TUYA_VCODEC_STREAM_T *stream, TUYA_MEDIA_BUF_T *buf)
{
    if (stream->codec != (UINT32_T)TUYA_VCODEC_TYPE_H264) {
        buf->format = TUYA_FRAME_FMT_JPEG;
        return;
    }

    switch (stream->frame_type) {
    case TUYA_VCODEC_FRAME_I:
        buf->format = TUYA_FRAME_FMT_H264_I;
        break;
    case TUYA_VCODEC_FRAME_P:
        buf->format = TUYA_FRAME_FMT_H264_P;
        break;
    default:
        buf->format = TUYA_FRAME_FMT_H264;
        break;
    }
}

/**
 * @brief Encoder output hook: wrap and hand the bitstream to business.
 *
 * @param[in] stream encoded frame
 * @param[in] arg    unused
 *
 * @note Runs on the shared hardware-encoder thread. tkl_venc releases the
 *       payload as soon as this returns, so the buffer handed on carries a NULL
 *       free_cb - business must copy anything it needs to keep.
 */
STATIC VOID tkl_cam_out_stream_cb(CONST TUYA_VCODEC_STREAM_T *stream, VOID *arg)
{
    TUYA_MEDIA_BUF_T buf;
    TUYA_MEDIA_OUTPUT_CFG_T *ch;

    (void)arg;

    if (stream == NULL || stream->data == NULL) {
        return;
    }

    ch = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_3);
    if (ch == NULL || ch->cb == NULL) {
        return;
    }

    os_memset(&buf, 0, sizeof(buf));
    buf.buffer = stream->data;
    buf.length = stream->size;
    buf.free_cb = NULL;             /* borrowed for the duration of this call */
    buf.channel = TUYA_CAMERA_FRAME_CHANNEL_3;
    buf.timestamp = stream->pts;
    buf.width = s_cam_out.src_width;
    buf.height = s_cam_out.src_height;
    tkl_cam_out_stream_meta(stream, &buf);
    buf.user_args = ch->arg;


    ch->cb(&buf);
}

/* ---------------------------------------------------------------------------
 * ch1 push delivery
 * ------------------------------------------------------------------------- */

/**
 * @brief Pop one SP frame and hand it to the ch1 business callback.
 *
 * @return the bytes delivered, 0 when nothing was delivered (dropped frame,
 *         timeout, or a failed allocation)
 *
 * @note One pool block carries the struct and the frame (struct at the block
 *       base) - the same layout the pull path hands out, so the free_cb
 *       releases both through the struct pointer. A slow consumer simply keeps
 *       more blocks out; when the owner pool runs dry the overflow event fires.
 */
STATIC UINT32_T tkl_cam_out_deliver_one(VOID_T)
{
    TUYA_MEDIA_BUF_T *buf;
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    UINT8_T isp_channel_id = 0U;
    UINT16_T width = 0U;
    UINT16_T height = 0U;
    UINT32_T format = 0U;
    UINT32_T size = 0U;
    UINT32_T out_len = 0U;
    OPERATE_RET ret;

    ch = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_1);
    if (ch == NULL || ch->cb == NULL) {
        return 0U;
    }

    ret = cam_mipi_pull_geometry(CAM_CHN_SP, &isp_channel_id,
                                 &width, &height, &format, &size);
    if (ret != OPRT_OK) {
        return 0U;
    }

    buf = (TUYA_MEDIA_BUF_T *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL,
                    (UINT32_T)sizeof(TUYA_MEDIA_BUF_T) + size,
                    TUYA_MEDIA_BUF_OWNER_CAMERA_FRAME);
    if (buf == NULL) {
        /* the business consumer is holding more blocks than the pool has */
        tkl_cam_out_event_throttled(TUYA_CAMERA_EVENT_ERR_OVERFLOW_CH1);
        return 0U;
    }

    ret = cam_mipi_pop_copy_frame(isp_channel_id, format, width, height,
                                  (UINT8_T *)(buf + 1), size, TUYA_MIPI_CSI_DELIVER_POP_MS,
                                  &out_len);
    if (ret != OPRT_OK) {
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_CAMERA_FRAME, (UINT8_T *)buf);
        return 0U;
    }

    os_memset(buf, 0, sizeof(*buf));
    buf->buffer = (UINT8_T *)(buf + 1);
    buf->length = out_len;
    buf->free_cb = tkl_mipi_csi_frame_free;
    buf->channel = TUYA_CAMERA_FRAME_CHANNEL_1;
    buf->timestamp = (UINT64_T)tkl_system_get_millisecond();
    buf->width = width;
    buf->height = height;
    buf->format = tkl_media_fmt_from_bk(format);
    buf->user_args = ch->arg;

    ch->cb(buf);
    return out_len;
}

/**
 * @brief The ch1 delivery task.
 *
 * @note Runs from start() to stop(). While ch1 is cancelled (ch_active[1]
 *       cleared) or the camera is stopped it only polls; the SP frame queue
 *       drops what nobody takes, and hardware overflow is the business
 *       handler's business. A silent SP reports ERR_TIMEOUT_CH1 after 3 s.
 */
STATIC VOID tkl_cam_out_deliver_task(beken_thread_arg_t arg)
{
    (void)arg;

    while (s_cam_out.deliver_exit != TRUE) {
        UINT32_T delivered;

        if (s_cam_out.state != TKL_CAM_ST_STARTED || s_cam_out.ch_active[1] != TRUE) {
            s_cam_out.ch1_no_frame_ms = 0U;
            tkl_system_sleep(TUYA_MIPI_CSI_DELIVER_POLL_MS);
            continue;
        }

        delivered = tkl_cam_out_deliver_one();
        if (delivered != 0U) {
            s_cam_out.ch1_no_frame_ms = 0U;
            continue;
        }

        /* nothing this round: track the silent window */
        s_cam_out.ch1_no_frame_ms += TUYA_MIPI_CSI_DELIVER_POP_MS;
        if (s_cam_out.ch1_no_frame_ms >= TUYA_MIPI_CSI_NO_FRAME_EVENT_MS) {
            tkl_cam_out_event_throttled(TUYA_CAMERA_EVENT_ERR_TIMEOUT_CH1);
        }
    }

    (void)rtos_set_semaphore(&s_cam_out.deliver_done);
    rtos_delete_thread(NULL);
}

/**
 * @brief Spawn the delivery task. ch1-with-cb configurations only - a pull
 *        ch1 has no task and get_frame() serves it.
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_cam_out_deliver_start(VOID_T)
{
    OPERATE_RET ret;
    TUYA_MEDIA_OUTPUT_CFG_T *ch = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_1);

    if (ch == NULL || ch->cb == NULL) {
        return OPRT_OK;                 /* pull channel: no task */
    }

    ret = rtos_init_semaphore_ex(&s_cam_out.deliver_done, 1, 0);
    if (ret != OPRT_OK) {
        return ret;
    }

    s_cam_out.deliver_exit = FALSE;
    ret = rtos_create_thread(&s_cam_out.deliver_task,
                             BEKEN_DEFAULT_WORKER_PRIORITY,
                             "cam_deliver",
                             (beken_thread_function_t)tkl_cam_out_deliver_task,
                             TUYA_MIPI_CSI_DELIVER_STACK,
                             NULL);
    if (ret != OPRT_OK) {
        (void)rtos_deinit_semaphore(&s_cam_out.deliver_done);
        return ret;
    }

    s_cam_out.deliver_live = TRUE;
    s_cam_out.ch_active[1] = TRUE;
    return OPRT_OK;
}

/**
 * @brief End the delivery task. Joins on deliver_done; the bound is one pop
 *        timeout plus one idle poll.
 */
STATIC VOID_T tkl_cam_out_deliver_stop(VOID_T)
{
    if (s_cam_out.deliver_live != TRUE) {
        return;
    }

    s_cam_out.deliver_exit = TRUE;
    (void)rtos_get_semaphore(&s_cam_out.deliver_done,
                             TUYA_MIPI_CSI_DELIVER_POP_MS + 2U * TUYA_MIPI_CSI_DELIVER_POLL_MS);
    (void)rtos_deinit_semaphore(&s_cam_out.deliver_done);
    s_cam_out.deliver_live = FALSE;
    s_cam_out.ch_active[1] = FALSE;
}

/* ---------------------------------------------------------------------------
 * Bring-up
 * ------------------------------------------------------------------------- */

/**
 * @brief Translate the requested channels into the legacy camera config.
 *
 * MP geometry follows ch3 when present (the VPU has no scaler, so the encoder
 * input cannot move); with ch2 alone it follows ch2. SP geometry comes from
 * ch1, where zero width/height falls through to the driver default (MP size).
 *
 * @param[in]  cfg facade config
 * @param[out] old receives the legacy config
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_cam_out_build_camera_cfg(CONST TUYA_MIPI_CSI_CFG_T *cfg,
                                                TUYA_MIPI_CSI_HW_CFG_T *old)
{
    CONST TUYA_MEDIA_OUTPUT_CFG_T *ch1 = NULL;
    CONST TUYA_MEDIA_OUTPUT_CFG_T *ch2 = NULL;
    CONST TUYA_MEDIA_OUTPUT_CFG_T *ch3 = NULL;
    BOOL_T mp_wanted;
    UINT32_T i;

    os_memset(old, 0, sizeof(*old));
    /* board: the facade takes typed GPIO config, the hardware layer keeps its
     * INT_T pins with -1 = not wired. TUYA_GPIO_NUM_MAX maps to that sentinel.
     * Vendor sensor drivers drive reset/pwdn active-high; a low-active request
     * is rejected in tkl_mipi_csi_validate_formats(). */
    old->i2c_id   = (UINT8_T)cfg->board_cfg.port;
    old->pin_scl  = (cfg->board_cfg.scl  == TUYA_GPIO_NUM_MAX) ? -1 : (INT_T)cfg->board_cfg.scl;
    old->pin_sda  = (cfg->board_cfg.sda  == TUYA_GPIO_NUM_MAX) ? -1 : (INT_T)cfg->board_cfg.sda;
    old->pin_xclk = (cfg->board_cfg.clk  == TUYA_GPIO_NUM_MAX) ? -1 : (INT_T)cfg->board_cfg.clk;
    old->pin_reset = (cfg->board_cfg.reset.pin      == TUYA_GPIO_NUM_MAX) ? -1 : (INT_T)cfg->board_cfg.reset.pin;
    old->pin_pwdn  = (cfg->board_cfg.power_ctrl.pin == TUYA_GPIO_NUM_MAX) ? -1 : (INT_T)cfg->board_cfg.power_ctrl.pin;

    old->sensor_width  = cfg->sensor_cfg.width;
    old->sensor_height = cfg->sensor_cfg.height;
    old->sensor_fps    = cfg->sensor_cfg.fps;
    /* UNKNOWN stays 0 so the driver applies its own default */
    old->sensor_input_format =
        (cfg->sensor_cfg.format != TUYA_FRAME_FMT_UNKNOWN)
            ? tkl_media_fmt_to_bk(cfg->sensor_cfg.format) : 0U;
    old->hmirror = 0U;
    old->vflip = 0U;
    old->read_timeout_ms = TUYA_MIPI_CSI_READ_TIMEOUT_MS;

    for (i = 0U; i < cfg->output_num; i++) {
        CONST TUYA_MEDIA_OUTPUT_CFG_T *e = &cfg->outputs[i];

        if (e->channel == TUYA_CAMERA_FRAME_CHANNEL_1) {
            ch1 = e;
        } else if (e->channel == TUYA_CAMERA_FRAME_CHANNEL_2) {
            ch2 = e;
        } else if (e->channel == TUYA_CAMERA_FRAME_CHANNEL_3) {
            ch3 = e;
        }
    }

    mp_wanted = (ch2 != NULL || ch3 != NULL);

    /* MP is a fixed FLEXA feed in NV12 - the GPU and the encoder take it from
     * there. Its resolution follows ch3 when present: the VPU has no scaler
     * while the GPU does, so the encode side is the one that cannot move.
     * ch2-only runs at ch2's geometry. */
    if (ch3 != NULL) {
        old->output_width = ch3->width;
        old->output_height = ch3->height;
    } else if (ch2 != NULL) {
        old->output_width = ch2->width;
        old->output_height = ch2->height;
    }
    old->output_format = (UINT32_T)BK_PIXEL_FORMAT_NV12;
    old->output_buf_cnt = TUYA_MIPI_CSI_BUF_CNT;
    old->enable_flexa = mp_wanted;
    old->work_mode = (mp_wanted == TRUE) ? 1U : 0U;

    if (ch1 != NULL) {
        old->sp_enable = TRUE;
        old->sp_width = ch1->width;
        old->sp_height = ch1->height;
        /* the caller's choice, not a constant: NV12 to encode or transform the
         * frame, RGB888 to feed the face model directly */
        old->sp_format = (ch1->format != TUYA_FRAME_FMT_UNKNOWN)
                             ? tkl_media_fmt_to_bk(ch1->format)
                             : (UINT32_T)BK_PIXEL_FORMAT_NV12;
        old->sp_buf_cnt = TUYA_MIPI_CSI_SP_BUF_CNT;
        /* always frame mode - SP has no bond to be in flexa for */
        old->sp_enable_flexa = FALSE;
        old->sp_work_mode = 0U;
    }

    return OPRT_OK;
}

/**
 * @brief Create and bind the display pipeline.
 * @param[in] flexa ISP flexa source of the channel that carries DISPLAY
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_cam_out_display_open(CONST TUYA_MIPI_CSI_FLEXA_SOURCE_T *flexa)
{
    TUYA_GPU_CFG_T gpu_cfg;
    TUYA_GPU_SINK_T sink;
    TUYA_GPU_SOURCE_T source;
    TUYA_MEDIA_OUTPUT_CFG_T *ch = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_2);
    OPERATE_RET ret;

    if (ch == NULL) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&gpu_cfg, 0, sizeof(gpu_cfg));
    gpu_cfg.dst_width = ch->width;
    gpu_cfg.dst_height = ch->height;
    gpu_cfg.rotate_degree = ch->rotate;
    gpu_cfg.dst_format = tkl_media_fmt_to_bk(ch->format);
    gpu_cfg.scale = TRUE;
    /* always compressed: a display frame goes straight to the DPU, which
     * decompresses. Exposing this as a choice only invites getting it wrong. */
    gpu_cfg.dst_compress = TRUE;
    gpu_cfg.flexa_lines = TUYA_MIPI_CSI_FLEXA_LINES;

    ret = tkl_gpu_init(&gpu_cfg, &s_cam_out.gpu);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: gpu init failed: %d\r\n", ret);
        return ret;
    }

    os_memset(&sink, 0, sizeof(sink));
    sink.alloc_cb = tkl_cam_out_display_alloc;
    sink.free_cb = tkl_cam_out_display_free;
    sink.frame_done_cb = tkl_cam_out_display_done;

    ret = tkl_gpu_set_sink(s_cam_out.gpu, &sink);
    if (ret != OPRT_OK) {
        goto err;
    }

    os_memset(&source, 0, sizeof(source));
    source.type = TUYA_GPU_SOURCE_ISP_FLEXA;
    source.isp.isp_handle = flexa->isp_handle;
    source.isp.flexa.src_buffer = flexa->src_buffer;
    source.isp.flexa.buf_cnt = flexa->buf_cnt;
    source.isp.flexa.width = flexa->width;
    source.isp.flexa.height = flexa->height;
    source.isp.flexa.format = flexa->format;

    ret = tkl_gpu_set_source(s_cam_out.gpu, &source);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: gpu set_source failed: %d\r\n", ret);
        goto err;
    }

    return OPRT_OK;

err:
    (void)tkl_gpu_deinit(s_cam_out.gpu);
    s_cam_out.gpu = NULL;
    return ret;
}

/**
 * @brief Create and bind the H264 pipeline.
 * @param[in] flexa ISP flexa source of the channel that carries H264
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_cam_out_encode_open(CONST TUYA_MIPI_CSI_FLEXA_SOURCE_T *flexa)
{
    TUYA_VENC_CFG_T venc_cfg;
    TUYA_VENC_SINK_T sink;
    TUYA_VENC_SOURCE_T source;
    TUYA_VENC_RATE_CTRL_T rc;
    TUYA_MEDIA_OUTPUT_CFG_T *ch = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_3);
    OPERATE_RET ret;

    if (ch == NULL) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&venc_cfg, 0, sizeof(venc_cfg));
    venc_cfg.width = flexa->width;
    venc_cfg.height = flexa->height;
    venc_cfg.fps = s_cam_out.cfg.sensor_cfg.fps;
    venc_cfg.gop = ch->gop;
    /* ch3.format is the encoded output format. This platform has no H265
     * encoder backend - init rejects that request - so the codec is always
     * H264 here. */
    venc_cfg.codec = (UINT32_T)TUYA_VCODEC_TYPE_H264;
    venc_cfg.input_format = flexa->format;

    ret = tkl_venc_init(&s_cam_out.venc, &venc_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: venc init failed: %d\r\n", ret);
        return ret;
    }

    /* the level presets the bitrate (LOW 0.75 / MEDIUM 1.5 / HIGH 3 Mbps);
     * tkl_mipi_csi_set_bitrate() fine-tunes from there */
    os_memset(&rc, 0, sizeof(rc));
    rc.bitrate = tkl_media_bitrate_from_level(ch->level);
    (void)tkl_venc_set_rate_ctrl(s_cam_out.venc, &rc);

    os_memset(&sink, 0, sizeof(sink));
    sink.stream_cb = tkl_cam_out_stream_cb;
    sink.alloc_cb = tkl_cam_out_stream_alloc;
    sink.free_cb = tkl_cam_out_stream_free;

    ret = tkl_venc_set_sink(s_cam_out.venc, &sink);
    if (ret != OPRT_OK) {
        goto err;
    }

    os_memset(&source, 0, sizeof(source));
    source.type = TUYA_VENC_SOURCE_ISP_FLEXA;
    source.isp.isp_handle = flexa->isp_handle;
    source.isp.flexa.owner_handle = flexa->isp_handle;
    source.isp.flexa.src_buffer = flexa->src_buffer;
    source.isp.flexa.buf_cnt = flexa->buf_cnt;
    source.isp.flexa.width = flexa->width;
    source.isp.flexa.height = flexa->height;
    source.isp.flexa.format = flexa->format;
    source.isp.flexa.y_addr = flexa->y_addr;
    source.isp.flexa.u_addr = flexa->u_addr;
    source.isp.flexa.v_addr = flexa->v_addr;

    ret = tkl_venc_set_source(s_cam_out.venc, &source);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: venc set_source failed: %d\r\n", ret);
        goto err;
    }

    return OPRT_OK;

err:
    (void)tkl_venc_deinit(s_cam_out.venc);
    s_cam_out.venc = NULL;
    return ret;
}

/* ---------------------------------------------------------------------------
 * Display OSD backend
 * ------------------------------------------------------------------------- */
STATIC OPERATE_RET tkl_cam_out_osd_display_open(VOID_T)
{
    TKL_OSD_ENGINE_CFG_T cfg;
    VOID *gpu_raw = NULL;
    OPERATE_RET ret;

    ret = tkl_gpu_get_handle(s_cam_out.gpu, &gpu_raw);
    if (ret != OPRT_OK) {
        return ret;
    }

    TUYA_MEDIA_OUTPUT_CFG_T *ch = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_2);

    if (ch == NULL) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.target_w = ch->width;
    cfg.target_h = ch->height;
    /* the caller supplies the rotation per overlay request */
    cfg.src_format = (UINT32_T)BK_PIXEL_FORMAT_ARGB8888;

    return tkl_osd_engine_open(TKL_OSD_BACKEND_GPU, gpu_raw, &cfg, &s_cam_out.osd_display);
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

static BOOL_T tkl_mipi_csi_is_bayer_input(TUYA_FRAME_FMT_E format)
{
    switch (format) {
    case TUYA_FRAME_FMT_BGGR10:
    case TUYA_FRAME_FMT_GBRG10:
    case TUYA_FRAME_FMT_GRBG10:
    case TUYA_FRAME_FMT_RGGB10:
        return TRUE;
    default:
        return FALSE;
    }
}

static OPERATE_RET tkl_mipi_csi_validate_formats(CONST TUYA_MIPI_CSI_CFG_T *cfg)
{
    UINT32_T i;
    UINT32_T j;

    if (cfg->sensor_cfg.format != TUYA_FRAME_FMT_UNKNOWN &&
        (!tkl_mipi_csi_is_bayer_input(cfg->sensor_cfg.format) ||
         tkl_media_fmt_to_bk(cfg->sensor_cfg.format) == (UINT32_T)BK_PIXEL_FORMAT_INVALID)) {
        return OPRT_INVALID_PARM;
    }

    /* The vendor sensor drivers toggle reset/pwdn active-high (and use 0xFF for
     * "not wired"), so a low-active request cannot be honored end to end. */
    if ((cfg->board_cfg.reset.pin != TUYA_GPIO_NUM_MAX &&
         cfg->board_cfg.reset.active_level != TUYA_GPIO_LEVEL_HIGH) ||
        (cfg->board_cfg.power_ctrl.pin != TUYA_GPIO_NUM_MAX &&
         cfg->board_cfg.power_ctrl.active_level != TUYA_GPIO_LEVEL_HIGH)) {
        tkl_log_output("tkl_mipi_csi: reset/power active_level must be HIGH "
                       "(vendor sensor drivers drive active-high)\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    for (i = 0U; i < cfg->output_num; i++) {
        CONST TUYA_MEDIA_OUTPUT_CFG_T *e = &cfg->outputs[i];

        /* one entry per channel; 0 is reserved until the vendor ships the MP
         * tap and 4..9 are reserved by the type */
        if (e->channel == TUYA_CAMERA_FRAME_CHANNEL_0) {
            tkl_log_output("tkl_mipi_csi: ch0 is reserved (no MP full-frame tap)\r\n");
            return OPRT_NOT_SUPPORTED;
        }
        if (e->channel >= TUYA_CAMERA_FRAME_CHANNEL_4) {
            return OPRT_INVALID_PARM;
        }
        for (j = i + 1U; j < cfg->output_num; j++) {
            if (cfg->outputs[j].channel == e->channel) {
                tkl_log_output("tkl_mipi_csi: duplicate channel %u\r\n",
                               (unsigned)e->channel);
                return OPRT_INVALID_PARM;
            }
        }

        switch (e->channel) {
        case TUYA_CAMERA_FRAME_CHANNEL_1:
            /* SP carries the caller's pixel format; zero size = source size */
            if (e->format != TUYA_FRAME_FMT_UNKNOWN &&
                tkl_media_fmt_to_bk(e->format) == (UINT32_T)BK_PIXEL_FORMAT_INVALID) {
                return OPRT_NOT_SUPPORTED;
            }
            if (e->rotate != 0U || e->gop != 0U || e->level != 0U) {
                return OPRT_INVALID_PARM;
            }
            break;

        case TUYA_CAMERA_FRAME_CHANNEL_2:
            /* GPU display: a push output, so the cb is required; the GPU
             * scales, so its own geometry must be given; rotate 0/90/270 */
            if (e->cb == NULL) {
                tkl_log_output("tkl_mipi_csi: ch2 requires cb (push output)\r\n");
                return OPRT_INVALID_PARM;
            }
            if (e->width == 0U || e->height == 0U) {
                return OPRT_INVALID_PARM;
            }
            if (e->format == TUYA_FRAME_FMT_UNKNOWN ||
                tkl_media_fmt_to_bk(e->format) == (UINT32_T)BK_PIXEL_FORMAT_INVALID) {
                return OPRT_NOT_SUPPORTED;
            }
            if (e->rotate != 0U && e->rotate != 90U && e->rotate != 270U) {
                tkl_log_output("tkl_mipi_csi: ch2 rotate %u not supported (0/90/270)\r\n",
                               (unsigned)e->rotate);
                return OPRT_NOT_SUPPORTED;
            }
            if (e->gop != 0U || e->level != 0U) {
                return OPRT_INVALID_PARM;
            }
            break;

        case TUYA_CAMERA_FRAME_CHANNEL_3:
            /* H264 encode: a push output; the only encoder backend on this
             * platform is H264 (an H265 request lands here too). */
            if (e->cb == NULL) {
                tkl_log_output("tkl_mipi_csi: ch3 requires cb (push output)\r\n");
                return OPRT_INVALID_PARM;
            }
            if (e->width == 0U || e->height == 0U) {
                return OPRT_INVALID_PARM;
            }
            if (e->format != TUYA_FRAME_FMT_UNKNOWN &&
                e->format != TUYA_FRAME_FMT_H264) {
                tkl_log_output("tkl_mipi_csi: ch3 format 0x%x not supported "
                               "(H264-only encoder)\r\n", (unsigned)e->format);
                return OPRT_NOT_SUPPORTED;
            }
            if (e->rotate != 0U) {
                return OPRT_INVALID_PARM;
            }
            break;

        default:
            return OPRT_INVALID_PARM;
        }
    }

    return OPRT_OK;
}

OPERATE_RET tkl_mipi_csi_init(CONST TUYA_MIPI_CSI_CFG_T *cfg)
{
    TUYA_MIPI_CSI_HW_CFG_T dev_cfg;
    UINT32_T t;
    UINT32_T ms_power;
    UINT32_T i;
    OPERATE_RET ret;

    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_out.state != TKL_CAM_ST_IDLE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (cfg->output_num == 0U || cfg->outputs == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_mipi_csi_validate_formats(cfg);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&s_cam_out, 0, sizeof(s_cam_out));
    s_cam_out.cfg = *cfg;               /* shallow: pointers first, then swap */
    s_cam_out.cfg.outputs = NULL;

    /* deep-copy the outputs, so the caller may release its cfg after init */
    s_cam_out.cfg.outputs = (TUYA_MEDIA_OUTPUT_CFG_T *)tkl_system_malloc(
        (UINT32_T)cfg->output_num * sizeof(TUYA_MEDIA_OUTPUT_CFG_T));
    if (s_cam_out.cfg.outputs == NULL) {
        os_memset(&s_cam_out, 0, sizeof(s_cam_out));
        return OPRT_MALLOC_FAILED;
    }
    os_memcpy(s_cam_out.cfg.outputs, cfg->outputs,
              (UINT32_T)cfg->output_num * sizeof(TUYA_MEDIA_OUTPUT_CFG_T));

    /* fixed shape; the running set starts as everything configured */
    for (i = 0U; i < TUYA_CAMERA_FRAME_CHANNEL_MAX; i++) {
        s_cam_out.ch_active[i] = (tkl_cam_out_ch((TUYA_CAMERA_FRAME_CHANNEL_E)i) != NULL) ? TRUE : FALSE;
    }
    s_cam_out.want_sp      = (tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_1) != NULL) ? TRUE : FALSE;
    s_cam_out.want_display = (tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_2) != NULL) ? TRUE : FALSE;
    s_cam_out.want_encode  = (tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_3) != NULL) ? TRUE : FALSE;

    (void)tkl_cam_out_build_camera_cfg(&s_cam_out.cfg, &dev_cfg);

    t = TUYA_MIPI_CSI_MS();
    ret = cam_mipi_init(&dev_cfg);
    ms_power = TUYA_MIPI_CSI_MS() - t;
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: camera init failed: %d\r\n", ret);
        tkl_system_free(s_cam_out.cfg.outputs);
        os_memset(&s_cam_out, 0, sizeof(s_cam_out));
        return ret;
    }

    s_cam_out.state = TKL_CAM_ST_INIT;
    tkl_log_output("tkl_cam_out: init took %u ms (power %u)\r\n",
                   (unsigned)ms_power, (unsigned)ms_power);
    tkl_cam_out_event(TUYA_CAMERA_EVENT_INITIALIZED);
    return OPRT_OK;
}

OPERATE_RET tkl_mipi_csi_start(VOID_T)
{
    UINT32_T t;
    UINT32_T ms_sensor;
    UINT32_T ms_consumer;
    OPERATE_RET ret;

    if (s_cam_out.state == TKL_CAM_ST_STARTED) {
        return OPRT_OK;
    }
    if (s_cam_out.state != TKL_CAM_ST_INIT) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* A set_pp() while stopped only recorded its value. Fold a *valid* one into
     * the config before the producer starts: the ISP then comes up at the
     * target geometry in one pass, instead of starting at the old size and
     * rebuilding right after (~300ms of every resume that carried a switch).
     * An out-of-range value stays pending for the post-start apply below,
     * which reports it without failing the start itself. */
    if (s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_1] == TRUE &&
        cam_mipi_output_validate(s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_1],
                                 s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_1]) == OPRT_OK) {
        s_cam_ctx.cfg.sp_width = s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_1];
        s_cam_ctx.cfg.sp_height = s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_1];
        s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_1] = FALSE;
    }

    /* ch3 owns the MP geometry; ch2 owns it only when no encoder pins it */
    if (s_cam_out.want_encode == TRUE &&
        s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_3] == TRUE &&
        cam_mipi_output_validate(s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_3],
                                 s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_3]) == OPRT_OK) {
        s_cam_ctx.cfg.output_width = s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_3];
        s_cam_ctx.cfg.output_height = s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_3];
        s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_3] = FALSE;
    } else if (s_cam_out.want_encode != TRUE &&
               s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_2] == TRUE &&
               cam_mipi_output_validate(s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_2],
                                        s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_2]) == OPRT_OK) {
        s_cam_ctx.cfg.output_width = s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_2];
        s_cam_ctx.cfg.output_height = s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_2];
        s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_2] = FALSE;
    }

    /* --- producer first: the flexa source is only valid once it streams --- */
    t = TUYA_MIPI_CSI_MS();
    ret = cam_mipi_start();
    ms_sensor = TUYA_MIPI_CSI_MS() - t;
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: camera start failed: %d\r\n", ret);
        return ret;
    }

    /* A set_pp() while stopped only recorded its value - apply it now that
     * the channels exist. ch1 reprograms the SP; ch3 (or ch2 without ch3)
     * reprograms the MP. ch2 above an encoder needs no camera work: the GPU
     * opens below from the recorded config. On failure the started geometry
     * stays and says so. */
    if (s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_1] == TRUE) {
        ret = cam_mipi_set_output(CAM_CHN_SP,
                                  s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_1],
                                  s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_1]);
        if (ret != OPRT_OK) {
            tkl_log_output("tkl_cam_out: recorded set_pp ch1 %ux%u failed: %d\r\n",
                           s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_1],
                           s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_1], ret);
        }
        s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_1] = FALSE;
    }

    if (s_cam_out.want_encode == TRUE &&
        s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_3] == TRUE) {
        ret = cam_mipi_set_output(CAM_CHN_MP,
                                  s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_3],
                                  s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_3]);
        if (ret != OPRT_OK) {
            tkl_log_output("tkl_cam_out: recorded set_pp ch3 %ux%u failed: %d\r\n",
                           s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_3],
                           s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_3], ret);
        }
        s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_3] = FALSE;
    } else if (s_cam_out.want_encode != TRUE &&
               s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_2] == TRUE) {
        ret = cam_mipi_set_output(CAM_CHN_MP,
                                  s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_2],
                                  s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_2]);
        if (ret != OPRT_OK) {
            tkl_log_output("tkl_cam_out: recorded set_pp ch2 %ux%u failed: %d\r\n",
                           s_cam_out.pp_width[TUYA_CAMERA_FRAME_CHANNEL_2],
                           s_cam_out.pp_height[TUYA_CAMERA_FRAME_CHANNEL_2], ret);
        }
    }
    s_cam_out.pp_pending[TUYA_CAMERA_FRAME_CHANNEL_2] = FALSE;

    /* cam_mipi_start() applies zero-dimension path defaults. Keep the channel
     * geometry in sync so stream metadata and later set_pp() use the actual
     * ISP dimensions rather than the caller's shorthand zeros. */
    s_cam_out.src_width = s_cam_ctx.cfg.output_width;
    s_cam_out.src_height = s_cam_ctx.cfg.output_height;
    if (s_cam_out.want_encode == TRUE) {
        TUYA_MEDIA_OUTPUT_CFG_T *ch3 = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_3);

        if (ch3 != NULL) {
            ch3->width = s_cam_ctx.cfg.output_width;
            ch3->height = s_cam_ctx.cfg.output_height;
        }
    } else if (s_cam_out.want_display == TRUE) {
        TUYA_MEDIA_OUTPUT_CFG_T *ch2 = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_2);

        if (ch2 != NULL) {
            ch2->width = s_cam_ctx.cfg.output_width;
            ch2->height = s_cam_ctx.cfg.output_height;
        }
    }
    if (s_cam_out.want_sp == TRUE) {
        TUYA_MEDIA_OUTPUT_CFG_T *ch1 = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_1);

        if (ch1 != NULL && ch1->width == 0U && ch1->height == 0U) {
            ch1->width = s_cam_ctx.cfg.sp_width;
            ch1->height = s_cam_ctx.cfg.sp_height;
        }
    }

    /* stop() clears the per-channel active flags; re-seed them from the init
     * configuration so a stop()/start() cycle reopens every channel. The
     * delivery task start below re-asserts ch1 for the push case. */
    s_cam_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_1] = s_cam_out.want_sp;
    s_cam_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_2] = s_cam_out.want_display;
    s_cam_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_3] = s_cam_out.want_encode;

    /* --- consumers: backends, then bonds, then overlays --- */
    t = TUYA_MIPI_CSI_MS();
    ret = tkl_cam_out_consumers_open();
    ms_consumer = TUYA_MIPI_CSI_MS() - t;
    if (ret != OPRT_OK) {
        (void)cam_mipi_stop();
        return ret;
    }

    ret = tkl_cam_out_deliver_start();
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: deliver task start failed: %d\r\n", ret);
        tkl_cam_out_consumers_close();
        (void)cam_mipi_stop();
        return ret;
    }

    s_cam_out.state = TKL_CAM_ST_STARTED;
    /* "sensor" is the register table plus the ISP bring-up; with SP enabled it
     * also carries the ~266 ms arm probe, which is why an SP-less pipeline
     * restarts in a fraction of the time. */
    tkl_log_output("tkl_cam_out: start took %u ms (sensor %u, consumer %u)\r\n",
                   (unsigned)(ms_sensor + ms_consumer),
                   (unsigned)ms_sensor, (unsigned)ms_consumer);
    tkl_cam_out_event(TUYA_CAMERA_EVENT_STARTED);
    return OPRT_OK;
}

OPERATE_RET tkl_mipi_csi_stop(VOID_T)
{
    UINT32_T t;
    UINT32_T ms_consumer;
    UINT32_T ms_camera;

    if (s_cam_out.state != TKL_CAM_ST_STARTED) {
        return OPRT_OK;
    }

    /* Turn away set_output / set_osd / set_encode before anything is
     * dismantled. It cannot help a caller already parked inside get_frame():
     * stop your own puller first, as the xmipi cam CLI does. */
    s_cam_out.state = TKL_CAM_ST_INIT;

    /* the delivery task runs against the SP read path; end it before the
     * consumers it can race with go away */
    tkl_cam_out_deliver_stop();

    t = TUYA_MIPI_CSI_MS();

    /* 1-3. overlays, then all bonds, then the backends. tkl_venc_stop() and
     *      tkl_gpu_stop() each stop their own bond and close only their own
     *      backend, so calling them back to back keeps the rule that no backend
     *      is touched while any bond is still live. */
    tkl_cam_out_consumers_close();
    s_cam_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_2] = FALSE;
    s_cam_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_3] = FALSE;
    ms_consumer = TUYA_MIPI_CSI_MS() - t;
    t = TUYA_MIPI_CSI_MS();

    /* 4. producer last */
    (void)cam_mipi_stop();
    ms_camera = TUYA_MIPI_CSI_MS() - t;

    /* Deliberately NOT draining tkl_media_buf here. Business is allowed to hold
     * a frame across a stop/start; the pool survives and is simply one buffer
     * short until it comes back. Waiting would add up to TUYA_MIPI_CSI_DRAIN_MS
     * per owner to every pause - three of them - which is most of what this API
     * split exists to remove. tkl_mipi_csi_deinit() does the waiting. */
    tkl_log_output("tkl_cam_out: stop took %u ms (consumer %u, camera %u)\r\n",
                   (unsigned)(ms_consumer + ms_camera),
                   (unsigned)ms_consumer, (unsigned)ms_camera);
    tkl_cam_out_event(TUYA_CAMERA_EVENT_STOPPED);
    return OPRT_OK;
}

OPERATE_RET tkl_mipi_csi_deinit(VOID_T)
{
    UINT32_T t;
    UINT32_T ms_camera;
    UINT32_T ms_drain;

    if (s_cam_out.state == TKL_CAM_ST_IDLE) {
        return OPRT_OK;
    }

    (void)tkl_mipi_csi_stop();

    t = TUYA_MIPI_CSI_MS();
    (void)cam_mipi_deinit();
    ms_camera = TUYA_MIPI_CSI_MS() - t;
    t = TUYA_MIPI_CSI_MS();

    /* Frames already handed to business are still out there; wait briefly so a
     * leak shows up here rather than as pool exhaustion later. Never force a
     * release - business may still hold the pointer. Display tolerates one: the
     * panel is still scanning the last frame and nothing will replace it. A
     * bitstream has no such holder, so stream must reach zero. */
    (void)tkl_media_buf_drain(TUYA_MEDIA_BUF_OWNER_CAMERA_DISPLAY,
                              TUYA_MIPI_CSI_OUTPUT_PIXEL_RESIDUAL, TUYA_MIPI_CSI_DRAIN_MS);
    (void)tkl_media_buf_drain(TUYA_MEDIA_BUF_OWNER_CAMERA_STREAM, 0U, TUYA_MIPI_CSI_DRAIN_MS);
    /* Pulled frames have no such holder either: business asked for each one and
     * is expected to have released it before deinit. */
    (void)tkl_media_buf_drain(TUYA_MEDIA_BUF_OWNER_CAMERA_FRAME, 0U, TUYA_MIPI_CSI_DRAIN_MS);
    ms_drain = TUYA_MIPI_CSI_MS() - t;

    tkl_log_output("tkl_cam_out: deinit took %u ms (camera %u, drain %u)\r\n",
                   (unsigned)(ms_camera + ms_drain),
                   (unsigned)ms_camera, (unsigned)ms_drain);

    /* release the deep-copied outputs, then tell business - the handler must
     * not look at adapter state after this point anyway */
    tkl_cam_out_event(TUYA_CAMERA_EVENT_DEINITIALIZED);
    if (s_cam_out.cfg.outputs != NULL) {
        tkl_system_free(s_cam_out.cfg.outputs);
        s_cam_out.cfg.outputs = NULL;
    }
    os_memset(&s_cam_out, 0, sizeof(s_cam_out));
    return OPRT_OK;
}

/* One-channel bring-up/teardown primitives. resume_channel() and
 * cancel_channel() use them alone; start()/stop() use the combined forms
 * below, which keep the overlays -> bonds -> backends order. */

/** Bring the ch2 display pipeline up against the current flexa source.
 *  Claims the GPU first; releases it again on any failure. */
STATIC OPERATE_RET tkl_cam_out_display_start(VOID_T)
{
    TUYA_MIPI_CSI_FLEXA_SOURCE_T flexa;
    OPERATE_RET ret;

    ret = tkl_media_hw_claim(TUYA_MEDIA_HW_GPU, TUYA_MEDIA_HW_CLAIM_CSI);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&flexa, 0, sizeof(flexa));
    ret = cam_mipi_get_flexa_source(&flexa);
    if (ret == OPRT_OK) {
        ret = tkl_cam_out_display_open(&flexa);
    }
    if (ret == OPRT_OK) {
        ret = tkl_gpu_start(s_cam_out.gpu);
    }
    if (ret == OPRT_OK) {
        ret = tkl_cam_out_osd_display_open();
    }
    if (ret != OPRT_OK) {
        if (s_cam_out.gpu != NULL) {
            (void)tkl_gpu_deinit(s_cam_out.gpu);
            s_cam_out.gpu = NULL;
        }
        tkl_media_hw_release(TUYA_MEDIA_HW_GPU, TUYA_MEDIA_HW_CLAIM_CSI);
        return ret;
    }

    return OPRT_OK;
}

/** Bring the ch3 encode pipeline up against the current flexa source. Claims
 *  the VPU encoder first; releases it again on any failure. */
STATIC OPERATE_RET tkl_cam_out_encode_start(VOID_T)
{
    TUYA_MIPI_CSI_FLEXA_SOURCE_T flexa;
    OPERATE_RET ret;

    ret = tkl_media_hw_claim(TUYA_MEDIA_HW_VPU_ENC, TUYA_MEDIA_HW_CLAIM_CSI);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&flexa, 0, sizeof(flexa));
    ret = cam_mipi_get_flexa_source(&flexa);
    if (ret == OPRT_OK) {
        ret = tkl_cam_out_encode_open(&flexa);
    }
    if (ret == OPRT_OK) {
        ret = tkl_venc_start(s_cam_out.venc);
    }
    if (ret != OPRT_OK) {
        if (s_cam_out.venc != NULL) {
            (void)tkl_venc_deinit(s_cam_out.venc);
            s_cam_out.venc = NULL;
        }
        tkl_media_hw_release(TUYA_MEDIA_HW_VPU_ENC, TUYA_MEDIA_HW_CLAIM_CSI);
        return ret;
    }

    return OPRT_OK;
}

/** Tear the ch2 display side down (overlays, bond, backend) and release the
 *  GPU claim when we held one. */
STATIC VOID_T tkl_cam_out_display_stop_one(VOID_T)
{
    BOOL_T had_gpu = (s_cam_out.gpu != NULL);

    if (s_cam_out.osd_display != NULL) {
        (void)tkl_osd_engine_close(s_cam_out.osd_display);
        s_cam_out.osd_display = NULL;
    }
    if (s_cam_out.gpu != NULL) {
        (void)tkl_gpu_stop(s_cam_out.gpu);
        (void)tkl_gpu_deinit(s_cam_out.gpu);
        s_cam_out.gpu = NULL;
    }

    if (had_gpu == TRUE) {
        tkl_media_hw_release(TUYA_MEDIA_HW_GPU, TUYA_MEDIA_HW_CLAIM_CSI);
    }
}

/** Tear the ch3 encode side down (bond, backend) and release the VPU claim
 *  when we held one. */
STATIC VOID_T tkl_cam_out_encode_stop_one(VOID_T)
{
    BOOL_T had_venc = (s_cam_out.venc != NULL);

    if (s_cam_out.venc != NULL) {
        (void)tkl_venc_stop(s_cam_out.venc);
        (void)tkl_venc_deinit(s_cam_out.venc);
        s_cam_out.venc = NULL;
    }

    if (had_venc == TRUE) {
        tkl_media_hw_release(TUYA_MEDIA_HW_VPU_ENC, TUYA_MEDIA_HW_CLAIM_CSI);
    }
}

/**
 * @brief Tear down every consumer, leaving the camera running.
 *
 * @note Same order as tkl_mipi_csi_stop() minus the producer: overlays,
 *       then all bonds, then the backends.
 */
STATIC VOID_T tkl_cam_out_consumers_close(VOID_T)
{
    if (s_cam_out.osd_display != NULL) {
        (void)tkl_osd_engine_close(s_cam_out.osd_display);
        s_cam_out.osd_display = NULL;
    }
    if (s_cam_out.osd_encode != NULL) {
        (void)tkl_osd_engine_close(s_cam_out.osd_encode);
        s_cam_out.osd_encode = NULL;
    }

    if (s_cam_out.venc != NULL) {
        (void)tkl_venc_stop(s_cam_out.venc);
    }
    if (s_cam_out.gpu != NULL) {
        (void)tkl_gpu_stop(s_cam_out.gpu);
    }

    tkl_cam_out_encode_stop_one();
    tkl_cam_out_display_stop_one();
}

/**
 * @brief Rebuild every active consumer against the camera's current flexa
 *        source.
 *
 * @note Re-reads the flexa source rather than reusing a cached one. With a
 *       single channel open, cam_mipi_set_output() restarts the ISP and
 *       hands back a **different** isp_handle; binding the old one would bond
 *       to freed state.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_cam_out_consumers_open(VOID_T)
{
    OPERATE_RET ret = OPRT_OK;
    BOOL_T display_up = FALSE;

    if (s_cam_out.want_display == TRUE && s_cam_out.ch_active[2] == TRUE) {
        ret = tkl_cam_out_display_start();
        if (ret != OPRT_OK) {
            return ret;
        }
        display_up = TRUE;
    }

    if (s_cam_out.want_encode == TRUE && s_cam_out.ch_active[3] == TRUE) {
        ret = tkl_cam_out_encode_start();
        if (ret != OPRT_OK) {
            if (display_up == TRUE) {
                tkl_cam_out_display_stop_one();
            }
            return ret;
        }
    }

    return OPRT_OK;
}

/**
 * @brief Rebuild the ch2 display side at a new destination size.
 *
 * Only valid while ch2 runs above an encoder: the MP keeps its geometry (the
 * GPU scales), so the camera and the encode side are untouched. On failure
 * the previous size is restored; if even that fails the display stays down
 * and the error is reported.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_cam_out_display_rebuild(UINT16_T width, UINT16_T height)
{
    TUYA_MEDIA_OUTPUT_CFG_T *ch = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_2);
    UINT16_T old_w;
    UINT16_T old_h;
    OPERATE_RET ret;

    if (ch == NULL) {
        return OPRT_INVALID_PARM;
    }

    old_w = ch->width;
    old_h = ch->height;
    if (old_w == width && old_h == height) {
        return OPRT_OK;
    }

    /* tear the display side down (overlays, then the GPU bond/backend); the
     * camera and the encode side keep running. The GPU claim stays with us -
     * the restart below re-claims it idempotently. */
    if (s_cam_out.osd_display != NULL) {
        (void)tkl_osd_engine_close(s_cam_out.osd_display);
        s_cam_out.osd_display = NULL;
    }
    if (s_cam_out.gpu != NULL) {
        (void)tkl_gpu_stop(s_cam_out.gpu);
        (void)tkl_gpu_deinit(s_cam_out.gpu);
        s_cam_out.gpu = NULL;
    }

    ch->width = width;
    ch->height = height;

    ret = tkl_cam_out_display_start();
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: display rebuild %ux%u failed: %d, reverting\r\n",
                       width, height, ret);

        ch->width = old_w;
        ch->height = old_h;
        if (tkl_cam_out_display_start() != OPRT_OK) {
            tkl_log_output("tkl_cam_out: display rebuild revert failed too\r\n");
        }
        return ret;
    }

    tkl_log_output("tkl_cam_out: ch2 destination switched %ux%u -> %ux%u\r\n",
                   old_w, old_h, width, height);
    return OPRT_OK;
}

OPERATE_RET tkl_mipi_csi_set_pp(TUYA_CAMERA_FRAME_CHANNEL_E channel, UINT16_T width, UINT16_T height)
{
    TUYA_MIPI_CSI_FLEXA_SOURCE_T flexa;
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    UINT16_T old_w;
    UINT16_T old_h;
    OPERATE_RET ret;
    VOID *isp_before = NULL;
    UINT8_T *src_before = NULL;

    if (width == 0U || height == 0U) {
        return OPRT_INVALID_PARM;
    }

    ch = tkl_cam_out_ch(channel);
    if (ch == NULL) {
        return OPRT_INVALID_PARM;      /* channel not configured */
    }

    if (s_cam_out.state == TKL_CAM_ST_INIT) {
        /* not streaming yet - record the value, start() applies it */
        s_cam_out.pp_pending[channel] = TRUE;
        s_cam_out.pp_width[channel] = width;
        s_cam_out.pp_height[channel] = height;
        return OPRT_OK;
    }

    /* Switching geometry needs a running pipeline to switch. */
    if (s_cam_out.state != TKL_CAM_ST_STARTED) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (channel == TUYA_CAMERA_FRAME_CHANNEL_1) {
        /* --- SP live switch: feeds nobody downstream, rebuilds nothing --- */
        if (s_cam_out.ch_active[1] != TRUE) {
            return OPRT_RESOURCE_NOT_READY;
        }

        /* NV12 packs 1.5 bytes per pixel, so an odd dimension has no whole-block
         * size. The ISP accepts the geometry, but cam_mipi_pop_copy_frame() then
         * refuses the pull with -2 - leaving a channel that reports a successful
         * switch and delivers nothing. Refuse it where the operator can see it. */
        if (s_cam_ctx.cfg.sp_format == (UINT32_T)BK_PIXEL_FORMAT_NV12 &&
            (((width & 1U) != 0U) || ((height & 1U) != 0U))) {
            tkl_log_output("tkl_cam_out: SP NV12 needs even dimensions, %ux%u rejected\r\n",
                           width, height);
            return OPRT_INVALID_PARM;
        }

        old_w = ch->width;
        old_h = ch->height;
        if (old_w == width && old_h == height) {
            return OPRT_OK;
        }

        /* Verified rather than trusted: isp_handle and the MP flexa buffer
         * are snapshotted and re-checked below - if the SP switch moved them
         * after all, every live bond points at dead state and must be
         * rebuilt. */
        os_memset(&flexa, 0, sizeof(flexa));
        if (cam_mipi_get_flexa_source(&flexa) == OPRT_OK) {
            isp_before = flexa.isp_handle;
            src_before = flexa.src_buffer;
        }

        ret = cam_mipi_set_output(CAM_CHN_SP, width, height);
        if (ret != OPRT_OK) {
            tkl_log_output("tkl_cam_out: set_output %ux%u failed: %d\r\n",
                           width, height, ret);
            return ret;
        }

        ch->width = width;
        ch->height = height;

        os_memset(&flexa, 0, sizeof(flexa));
        if (cam_mipi_get_flexa_source(&flexa) != OPRT_OK ||
            flexa.isp_handle != isp_before ||
            flexa.src_buffer != src_before) {
            tkl_log_output("tkl_cam_out: SP switch moved the MP source "
                           "(isp %p->%p buf %p->%p), rebuilding consumers\r\n",
                           isp_before, flexa.isp_handle,
                           (VOID *)src_before, (VOID *)flexa.src_buffer);
            tkl_cam_out_consumers_close();
            ret = tkl_cam_out_consumers_open();
            if (ret != OPRT_OK) {
                return ret;
            }
        }

        tkl_log_output("tkl_cam_out: ch1 switched %ux%u -> %ux%u (MP consumers kept)\r\n",
                       old_w, old_h, width, height);
        return OPRT_OK;
    }

    if (channel == TUYA_CAMERA_FRAME_CHANNEL_2 && s_cam_out.want_encode == TRUE) {
        /* --- GPU destination change: the encoder pins the MP geometry and
         * the GPU scales, so only the display side rebuilds --- */
        if (s_cam_out.ch_active[2] != TRUE) {
            return OPRT_RESOURCE_NOT_READY;
        }
        return tkl_cam_out_display_rebuild(width, height);
    }

    if (channel == TUYA_CAMERA_FRAME_CHANNEL_3 ||
        (channel == TUYA_CAMERA_FRAME_CHANNEL_2 && s_cam_out.want_encode != TRUE)) {
        /* --- MP geometry change: the ISP restarts and every active consumer
         * rebuilds. ch3 is the authoritative MP geometry; ch2 without ch3
         * owns it alone. --- */
        if ((channel == TUYA_CAMERA_FRAME_CHANNEL_3 && s_cam_out.ch_active[3] != TRUE) ||
            (channel == TUYA_CAMERA_FRAME_CHANNEL_2 && s_cam_out.ch_active[2] != TRUE)) {
            return OPRT_RESOURCE_NOT_READY;
        }

        old_w = ch->width;
        old_h = ch->height;
        if (old_w == width && old_h == height) {
            return OPRT_OK;
        }

        tkl_cam_out_consumers_close();

        ret = cam_mipi_set_output(CAM_CHN_MP, width, height);
        if (ret != OPRT_OK) {
            tkl_log_output("tkl_cam_out: set_output %ux%u failed: %d, restoring\r\n",
                           width, height, ret);
            /* the camera kept its old geometry, so rebuilding is enough */
            (void)tkl_cam_out_consumers_open();
            return ret;
        }

        ch->width = width;
        ch->height = height;
        s_cam_out.src_width = width;
        s_cam_out.src_height = height;

        ret = tkl_cam_out_consumers_open();
        if (ret != OPRT_OK) {
            tkl_log_output("tkl_cam_out: rebuild after switch failed: %d, reverting to %ux%u\r\n",
                           ret, old_w, old_h);

            if (cam_mipi_set_output(CAM_CHN_MP, old_w, old_h) == OPRT_OK) {
                ch->width = old_w;
                ch->height = old_h;
                s_cam_out.src_width = old_w;
                s_cam_out.src_height = old_h;
                (void)tkl_cam_out_consumers_open();
            }
            return ret;
        }

        tkl_log_output("tkl_cam_out: ch%u switched %ux%u -> %ux%u\r\n",
                       (unsigned int)channel, old_w, old_h, width, height);
        return OPRT_OK;
    }

    return OPRT_NOT_SUPPORTED;         /* ch0 is reserved */
}

OPERATE_RET tkl_mipi_csi_set_bitrate(UINT32_T bitrate)
{
    TUYA_VENC_RATE_CTRL_T rc;

    if (bitrate == 0U) {
        return OPRT_INVALID_PARM;
    }

    /* The encoder is built by start() and torn down by stop(), so there is
     * nothing to configure in between. Refused rather than remembered: a
     * setting that silently waits for the next start() is a bug factory. A
     * later pipeline rebuild (resume/revert) returns the preset from
     * ch3.level - tkl_mipi_csi_set_bitrate() is live tuning. */
    if (s_cam_out.state != TKL_CAM_ST_STARTED || s_cam_out.venc == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    os_memset(&rc, 0, sizeof(rc));
    rc.bitrate = bitrate;
    return tkl_venc_set_rate_ctrl(s_cam_out.venc, &rc);
}

OPERATE_RET tkl_mipi_csi_set_gop(UINT32_T gop)
{
    TUYA_MEDIA_OUTPUT_CFG_T *ch = tkl_cam_out_ch(TUYA_CAMERA_FRAME_CHANNEL_3);

    if (ch == NULL) {
        return OPRT_INVALID_PARM;      /* no ch3 configured */
    }

    /* The encoder takes gop only at build time (tkl_venc_gop_resolve()).
     * A running stream refuses; a stopped one records the value for the next
     * start(). gop 0 selects the encoder default (40). */
    if (s_cam_out.state == TKL_CAM_ST_STARTED && s_cam_out.ch_active[3] == TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ch->gop = gop;
    return OPRT_OK;
}

OPERATE_RET tkl_mipi_csi_force_idr(VOID_T)
{
    if (s_cam_out.state != TKL_CAM_ST_STARTED || s_cam_out.venc == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    return tkl_venc_request_idr(s_cam_out.venc);
}

OPERATE_RET tkl_mipi_csi_set_osd(TUYA_CAMERA_FRAME_CHANNEL_E channel, CONST TUYA_OSD_T *osd)
{
    if (osd == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_out.state != TKL_CAM_ST_STARTED) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (channel == TUYA_CAMERA_FRAME_CHANNEL_2) {
        if (s_cam_out.osd_display == NULL) {
            return OPRT_NOT_SUPPORTED;
        }
        return tkl_osd_engine_submit(s_cam_out.osd_display, osd);
    }

    if (channel == TUYA_CAMERA_FRAME_CHANNEL_3) {
        if (s_cam_out.venc == NULL) {
            return OPRT_NOT_SUPPORTED;
        }
        return tkl_venc_set_osd(s_cam_out.venc, osd);
    }

    return OPRT_NOT_SUPPORTED;         /* ch0/ch1 carry no overlay */
}

OPERATE_RET tkl_mipi_csi_resume_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    OPERATE_RET ret;

    if (s_cam_out.state != TKL_CAM_ST_STARTED) {
        return OPRT_RESOURCE_NOT_READY;
    }

    switch (channel) {
    case TUYA_CAMERA_FRAME_CHANNEL_1:
        if (s_cam_out.want_sp != TRUE) {
            return OPRT_INVALID_PARM;
        }
        if (s_cam_out.ch_active[1] == TRUE) {
            return OPRT_OK;            /* idempotent */
        }

        if (s_cam_ctx.cfg.sp_enable != TRUE ||
            bk_isp_camera_channel_state_get(s_cam_ctx.camera_ctlr_handle, ISP_SP_CHN_ID)
                != ISP_CHANNEL_STATE_TURN_ON) {
            ret = cam_mipi_open_sp_channel();
            if (ret != OPRT_OK) {
                tkl_log_output("tkl_cam_out: ch1 resume open failed: %d\r\n", ret);
                return ret;
            }
        }

        s_cam_out.ch_active[1] = TRUE;
        tkl_log_output("tkl_cam_out: ch1 resumed\r\n");
        return OPRT_OK;

    case TUYA_CAMERA_FRAME_CHANNEL_2:
        if (s_cam_out.want_display != TRUE) {
            return OPRT_INVALID_PARM;
        }
        if (s_cam_out.ch_active[2] == TRUE) {
            return OPRT_OK;            /* idempotent */
        }

        ret = tkl_cam_out_display_start();
        if (ret != OPRT_OK) {
            return ret;                /* OPRT_RESOURCE_NOT_READY when the other
                                        * peripheral holds the GPU */
        }

        s_cam_out.ch_active[2] = TRUE;
        tkl_log_output("tkl_cam_out: ch2 resumed\r\n");
        return OPRT_OK;

    case TUYA_CAMERA_FRAME_CHANNEL_3:
        if (s_cam_out.want_encode != TRUE) {
            return OPRT_INVALID_PARM;
        }
        if (s_cam_out.ch_active[3] == TRUE) {
            return OPRT_OK;            /* idempotent */
        }

        ret = tkl_cam_out_encode_start();
        if (ret != OPRT_OK) {
            return ret;                /* OPRT_RESOURCE_NOT_READY when the other
                                        * peripheral holds the encoder */
        }

        s_cam_out.ch_active[3] = TRUE;
        tkl_log_output("tkl_cam_out: ch3 resumed\r\n");
        return OPRT_OK;

    default:
        return OPRT_NOT_SUPPORTED;     /* ch0 is reserved */
    }
}

OPERATE_RET tkl_mipi_csi_cancel_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    if (s_cam_out.state != TKL_CAM_ST_STARTED) {
        return OPRT_RESOURCE_NOT_READY;
    }

    switch (channel) {
    case TUYA_CAMERA_FRAME_CHANNEL_1:
        if (s_cam_out.want_sp != TRUE) {
            return OPRT_INVALID_PARM;
        }
        if (s_cam_out.ch_active[1] != TRUE) {
            return OPRT_OK;            /* idempotent */
        }

        /* stop the flow first (the delivery task idles / get_frame refuses),
         * then close the SP channel; MP keeps streaming */
        s_cam_out.ch_active[1] = FALSE;
        tkl_system_sleep(TUYA_MIPI_CSI_DELIVER_POP_MS);

        if (s_cam_ctx.camera_ctlr_handle != NULL &&
            bk_isp_camera_channel_state_get(s_cam_ctx.camera_ctlr_handle, ISP_SP_CHN_ID)
                == ISP_CHANNEL_STATE_TURN_ON) {
            (void)bk_isp_camera_channel_close(s_cam_ctx.camera_ctlr_handle, ISP_SP_CHN_ID);
        }

        tkl_log_output("tkl_cam_out: ch1 cancelled\r\n");
        return OPRT_OK;

    case TUYA_CAMERA_FRAME_CHANNEL_2:
        if (s_cam_out.want_display != TRUE) {
            return OPRT_INVALID_PARM;
        }
        if (s_cam_out.ch_active[2] != TRUE) {
            return OPRT_OK;            /* idempotent */
        }

        /* the encoder keeps streaming off the shared MP - BOARD-TEST ITEM:
         * a partial MP consumer teardown has no vendor precedent; if the
         * encoder glitches, the fallback is stop()+start() of the camera */
        s_cam_out.ch_active[2] = FALSE;
        tkl_cam_out_display_stop_one();

        tkl_log_output("tkl_cam_out: ch2 cancelled, ch3 keeps streaming\r\n");
        return OPRT_OK;

    case TUYA_CAMERA_FRAME_CHANNEL_3:
        if (s_cam_out.want_encode != TRUE) {
            return OPRT_INVALID_PARM;
        }
        if (s_cam_out.ch_active[3] != TRUE) {
            return OPRT_OK;            /* idempotent */
        }

        /* the GPU keeps displaying off the shared MP */
        s_cam_out.ch_active[3] = FALSE;
        tkl_cam_out_encode_stop_one();

        tkl_log_output("tkl_cam_out: ch3 cancelled, ch2 keeps displaying\r\n");
        return OPRT_OK;

    default:
        return OPRT_NOT_SUPPORTED;     /* ch0 is reserved */
    }
}
