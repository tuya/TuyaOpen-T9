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
    TUYA_MIPI_CSI_CFG_T cfg;

    /* which channel carries which sink */
    /* No display_chn/encode_chn here on purpose: both consumers can only ever
     * be on MP, so a field able to hold SP would be state that never takes its
     * other value and would read as if the choice existed. */
    BOOL_T  want_display;
    BOOL_T  want_encode;

    /* a set_pp() while stopped records here; start() applies it */
    BOOL_T pp_pending;
    TUYA_CAMERA_OUTPUT_TYPE_E pp_type;
    UINT16_T pp_width;
    UINT16_T pp_height;

    TKL_GPU_HANDLE gpu;
    TUYA_VENC_HANDLE venc;
    TKL_OSD_ENGINE_HANDLE osd_display;
    TKL_OSD_ENGINE_HANDLE osd_encode;

    UINT16_T src_width;         /* ISP output feeding the consumers */
    UINT16_T src_height;

} tkl_cam_out_ctx_t;

STATIC tkl_cam_out_ctx_t s_cam_out;

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

static OPERATE_RET cam_mipi_open_sp_channel(void)
{
    OPERATE_RET ret;
    bk_isp_camera_channel_config_t sp_cfg;

    if (s_cam_ctx.cfg.sp_enable != TRUE) {
        return OPRT_OK;
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
    sensor_cfg.pin_xclk = (uint8_t)cfg->pin_xclk;
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
     */
    if (channel_id == CAM_CHN_MP) {
        tkl_log_output("set_output MP restart, %ux%u\r\n", width, height);
        ret = cam_mipi_stop();
        if (ret != OPRT_OK) {
            tkl_log_output("set_output MP restart stop failed: %d\r\n", ret);
            return ret;
        }
        ret = cam_mipi_start();
        if (ret != OPRT_OK) {
            tkl_log_output("set_output MP restart start failed: %d\r\n", ret);
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
 * The returned buf struct and its frame come out of one slab block: the frame
 * data sits right behind the struct, so the release walks back to the block
 * base and frees both with it. The caller only ever calls free_cb.
 */
STATIC VOID_T tkl_mipi_csi_frame_free(VOID_T *data)
{
    TUYA_MEDIA_BUF_T *buf;

    if (data == NULL) {
        return;
    }
    buf = (TUYA_MEDIA_BUF_T *)((UINT8_T *)data - sizeof(TUYA_MEDIA_BUF_T));
    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_CAMERA_FRAME, (UINT8_T *)buf);
}

OPERATE_RET tkl_mipi_csi_get_frame(TUYA_MEDIA_BUF_T **out, UINT32_T timeout_ms)
{
    TUYA_MEDIA_BUF_T *buf;
    OPERATE_RET op_ret;
    UINT8_T isp_channel_id = 0U;
    UINT16_T width = 0U;
    UINT16_T height = 0U;
    UINT32_T format = 0U;
    UINT32_T size = 0U;
    UINT32_T out_len = 0U;
    UINT32_T wait_ms;
    UINT8_T *data;

    if (out == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_out.cfg.frame.enable != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_cam_ctx.started != TRUE || s_cam_ctx.camera_ctlr_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* SP only this version; cam_mipi_pull_geometry() still knows how to serve
     * MP frame mode, so a channel argument can come back if MP pull ships. */
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
    data = (UINT8_T *)(buf + 1);

    wait_ms = (timeout_ms != 0U) ? timeout_ms : s_cam_ctx.cfg.read_timeout_ms;
    if (wait_ms == 0U) {
        wait_ms = 3000U;
    }

    op_ret = cam_mipi_pop_copy_frame(isp_channel_id, format, width, height,
                                              data, size, wait_ms, &out_len);
    if (op_ret != OPRT_OK) {
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_CAMERA_FRAME, (UINT8_T *)buf);
        return op_ret;
    }

    os_memset(buf, 0, sizeof(*buf));
    buf->data = data;
    buf->size = out_len;
    /* the whole point: business releases through this and never learns the pool */
    buf->free_cb = tkl_mipi_csi_frame_free;
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



#define TUYA_MIPI_CSI_FLEXA_LINES     16U
#define TUYA_MIPI_CSI_BUF_CNT         3U
#define TUYA_MIPI_CSI_SP_BUF_CNT      2U
#define TUYA_MIPI_CSI_READ_TIMEOUT_MS 1000U
#define TUYA_MIPI_CSI_DRAIN_MS        200U
/* the frame the panel is still scanning out when the camera stops - see the
 * drain call in tkl_mipi_csi_deinit() */
#define TUYA_MIPI_CSI_OUTPUT_PIXEL_RESIDUAL 1U

#define TUYA_MIPI_CSI_MS()            ((UINT32_T)tkl_system_get_millisecond())


/* Shared by close() and switch(): both need the exact same teardown, and the
 * order inside it is the part that must not drift. */
STATIC VOID_T tkl_cam_out_consumers_close(VOID_T);
STATIC OPERATE_RET tkl_cam_out_consumers_open(VOID_T);

/* ---------------------------------------------------------------------------
 * Delivery
 * ------------------------------------------------------------------------- */

/**
 * @brief GPU frame allocator; display frames are CPU-visible and hardware-read.
 * @param[in] size bytes wanted
 * @param[in] arg  unused
 * @return buffer or NULL
 */
STATIC VOID *tkl_cam_out_display_alloc(UINT32_T size, VOID *arg)
{
    (void)arg;
    return tkl_media_buf_alloc(TKL_BUF_USE_PIXEL_WT, size, TUYA_MEDIA_BUF_OWNER_CAMERA_DISPLAY);
}

STATIC OPERATE_RET tkl_cam_out_display_free(VOID *ptr, VOID *arg)
{
    (void)arg;

    if (ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_CAMERA_DISPLAY, ptr);
    return OPRT_OK;
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
    TUYA_MEDIA_BUF_T buf;

    (void)arg;

    if (frame == NULL) {
        return;
    }

    if (s_cam_out.cfg.pixel.pixel_cb == NULL) {
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_CAMERA_DISPLAY, frame);
        return;
    }

    os_memset(&buf, 0, sizeof(buf));
    buf.data = (UINT8_T *)frame;
    buf.size = frame_size;
    buf.free_cb = tkl_media_buf_free_cb(TUYA_MEDIA_BUF_OWNER_CAMERA_DISPLAY);
    buf.timestamp = (UINT64_T)tkl_system_get_millisecond();
    buf.width = s_cam_out.cfg.pixel.width;
    buf.height = s_cam_out.cfg.pixel.height;
    buf.format = s_cam_out.cfg.pixel.format;
    buf.user_args = s_cam_out.cfg.pixel.pixel_arg;

    s_cam_out.cfg.pixel.pixel_cb(&buf);
}

STATIC VOID *tkl_cam_out_stream_alloc(UINT32_T size, VOID *arg)
{
    (void)arg;
    return tkl_media_buf_alloc(TKL_BUF_USE_STREAM, size, TUYA_MEDIA_BUF_OWNER_CAMERA_STREAM);
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

    (void)arg;

    if (stream == NULL || stream->data == NULL || s_cam_out.cfg.stream.stream_cb == NULL) {
        return;
    }

    os_memset(&buf, 0, sizeof(buf));
    buf.data = stream->data;
    buf.size = stream->size;
    buf.free_cb = NULL;             /* borrowed for the duration of this call */
    buf.timestamp = stream->pts;
    buf.width = s_cam_out.src_width;
    buf.height = s_cam_out.src_height;
    tkl_cam_out_stream_meta(stream, &buf);
    buf.user_args = s_cam_out.cfg.stream.stream_arg;

    s_cam_out.cfg.stream.stream_cb(&buf);
}

/* ---------------------------------------------------------------------------
 * Bring-up
 * ------------------------------------------------------------------------- */

/**
 * @brief Translate the wanted outputs into the legacy camera config.
 * @param[in]  cfg facade config
 * @param[out] old receives the legacy config
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_cam_out_build_camera_cfg(CONST TUYA_MIPI_CSI_CFG_T *cfg,
                                                TUYA_MIPI_CSI_HW_CFG_T *old)
{
    BOOL_T mp_wanted;
    BOOL_T sp_wanted;

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

    /* The routing derives from the enables alone: frame pulls its raw frames
     * from SP, pixel/stream bond the shared MP line-by-line. The two groups
     * sit on different channels, so no combination needs rejecting.
     *
     * MP is a fixed FLEXA feed in NV12 - the GPU and the encoder take it from
     * there. Its resolution follows stream when both are enabled: the VPU has
     * no scaler while the GPU does, so the stream side is the one that cannot
     * move. pixel-only runs at pixel's geometry. */
    sp_wanted = (cfg->frame.enable == TRUE);
    mp_wanted = (cfg->pixel.enable == TRUE || cfg->stream.enable == TRUE);

    if (cfg->stream.enable == TRUE) {
        old->output_width = cfg->stream.width;
        old->output_height = cfg->stream.height;
    } else {
        old->output_width = cfg->pixel.width;
        old->output_height = cfg->pixel.height;
    }
    old->output_format = (UINT32_T)BK_PIXEL_FORMAT_NV12;
    old->output_buf_cnt = TUYA_MIPI_CSI_BUF_CNT;
    old->enable_flexa = mp_wanted;
    old->work_mode = (mp_wanted == TRUE) ? 1U : 0U;

    if (sp_wanted) {
        old->sp_enable = TRUE;
        old->sp_width = cfg->frame.width;
        old->sp_height = cfg->frame.height;
        /* the caller's choice, not a constant: NV12 to encode or transform the
         * frame, RGB888 to feed the face model directly */
        old->sp_format = (cfg->frame.format != TUYA_FRAME_FMT_UNKNOWN)
                             ? tkl_media_fmt_to_bk(cfg->frame.format)
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
    OPERATE_RET ret;

    os_memset(&gpu_cfg, 0, sizeof(gpu_cfg));
    gpu_cfg.dst_width = s_cam_out.cfg.pixel.width;
    gpu_cfg.dst_height = s_cam_out.cfg.pixel.height;
    gpu_cfg.rotate_degree = s_cam_out.cfg.pixel.rotate;
    gpu_cfg.dst_format = tkl_media_fmt_to_bk(s_cam_out.cfg.pixel.format);
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
    OPERATE_RET ret;

    os_memset(&venc_cfg, 0, sizeof(venc_cfg));
    venc_cfg.width = flexa->width;
    venc_cfg.height = flexa->height;
    venc_cfg.fps = (s_cam_out.cfg.stream.fps != 0U) ? s_cam_out.cfg.stream.fps
                                                    : s_cam_out.cfg.sensor_cfg.fps;
    venc_cfg.gop = s_cam_out.cfg.stream.gop;
    /* stream.format is the encoded output format (reviewed semantics). This
     * platform has no H265 encoder backend - init rejects that request - so
     * the codec is always H264 here. */
    venc_cfg.codec = (UINT32_T)TUYA_VCODEC_TYPE_H264;
    venc_cfg.input_format = flexa->format;

    ret = tkl_venc_init(&s_cam_out.venc, &venc_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: venc init failed: %d\r\n", ret);
        return ret;
    }

    if (s_cam_out.cfg.stream.bitrate != 0U) {
        os_memset(&rc, 0, sizeof(rc));
        rc.bitrate = s_cam_out.cfg.stream.bitrate;
        (void)tkl_venc_set_rate_ctrl(s_cam_out.venc, &rc);
    }

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

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.target_w = s_cam_out.cfg.pixel.width;
    cfg.target_h = s_cam_out.cfg.pixel.height;
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
    if (cfg->sensor_cfg.format != TUYA_FRAME_FMT_UNKNOWN &&
        (!tkl_mipi_csi_is_bayer_input(cfg->sensor_cfg.format) ||
         tkl_media_fmt_to_bk(cfg->sensor_cfg.format) == (UINT32_T)BK_PIXEL_FORMAT_INVALID)) {
        return OPRT_INVALID_PARM;
    }

    /* geometry: every enabled output needs its pixels sized */
    if ((cfg->frame.enable == TRUE &&
         (cfg->frame.width == 0U || cfg->frame.height == 0U)) ||
        (cfg->pixel.enable == TRUE &&
         (cfg->pixel.width == 0U || cfg->pixel.height == 0U)) ||
        (cfg->stream.enable == TRUE &&
         (cfg->stream.width == 0U || cfg->stream.height == 0U))) {
        return OPRT_INVALID_PARM;
    }

    /* SP carries the caller's pixel format; MP is NV12 by construction. */
    if (cfg->frame.enable == TRUE &&
        cfg->frame.format != TUYA_FRAME_FMT_UNKNOWN &&
        tkl_media_fmt_to_bk(cfg->frame.format) == (UINT32_T)BK_PIXEL_FORMAT_INVALID) {
        return OPRT_NOT_SUPPORTED;
    }

    if (cfg->pixel.enable == TRUE &&
        (cfg->pixel.format == TUYA_FRAME_FMT_UNKNOWN ||
         tkl_media_fmt_to_bk(cfg->pixel.format) == (UINT32_T)BK_PIXEL_FORMAT_INVALID)) {
        return OPRT_NOT_SUPPORTED;
    }

    /* stream.format is the encoded output format; the only encoder backend on
     * this platform is H264 (an H265 request lands here too). */
    if (cfg->stream.enable == TRUE &&
        cfg->stream.format != TUYA_FRAME_FMT_UNKNOWN &&
        cfg->stream.format != TUYA_FRAME_FMT_H264) {
        tkl_log_output("tkl_mipi_csi: stream.format 0x%x not supported "
                       "(H264-only encoder)\r\n", (unsigned)cfg->stream.format);
        return OPRT_NOT_SUPPORTED;
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

    return OPRT_OK;
}

OPERATE_RET tkl_mipi_csi_init(CONST TUYA_MIPI_CSI_CFG_T *cfg)
{
    TUYA_MIPI_CSI_HW_CFG_T dev_cfg;
    UINT32_T t;
    UINT32_T ms_power;
    OPERATE_RET ret;

    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_out.state != TKL_CAM_ST_IDLE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (cfg->frame.enable != TRUE && cfg->pixel.enable != TRUE &&
        cfg->stream.enable != TRUE) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_mipi_csi_validate_formats(cfg);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&s_cam_out, 0, sizeof(s_cam_out));
    os_memcpy(&s_cam_out.cfg, cfg, sizeof(s_cam_out.cfg));

    if (cfg->pixel.enable == TRUE) {
        s_cam_out.want_display = TRUE;
    }
    if (cfg->stream.enable == TRUE) {
        s_cam_out.want_encode = TRUE;
    }

    (void)tkl_cam_out_build_camera_cfg(cfg, &dev_cfg);

    t = TUYA_MIPI_CSI_MS();
    ret = cam_mipi_init(&dev_cfg);
    ms_power = TUYA_MIPI_CSI_MS() - t;
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: camera init failed: %d\r\n", ret);
        os_memset(&s_cam_out, 0, sizeof(s_cam_out));
        return ret;
    }

    s_cam_out.state = TKL_CAM_ST_INIT;
    tkl_log_output("tkl_cam_out: init took %u ms (power %u)\r\n",
                   (unsigned)ms_power, (unsigned)ms_power);
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

    /* --- producer first: the flexa source is only valid once it streams --- */
    t = TUYA_MIPI_CSI_MS();
    ret = cam_mipi_start();
    ms_sensor = TUYA_MIPI_CSI_MS() - t;
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: camera start failed: %d\r\n", ret);
        return ret;
    }

    /* A set_pp() while stopped only recorded its value - apply it now that
     * the channels exist. On failure the started geometry stays and says so. */
    if (s_cam_out.pp_pending == TRUE) {
        ret = cam_mipi_set_output(
            (s_cam_out.pp_type == TUYA_CAMERA_OUTPUT_FRAME)
                ? CAM_CHN_SP : CAM_CHN_MP,
            s_cam_out.pp_width, s_cam_out.pp_height);
        if (ret != OPRT_OK) {
            tkl_log_output("tkl_cam_out: recorded set_pp %ux%u failed: %d\r\n",
                           s_cam_out.pp_width, s_cam_out.pp_height, ret);
        }
        s_cam_out.pp_pending = FALSE;
    }

    /* cam_mipi_start() applies zero-dimension path defaults. Keep the facade
     * geometry in sync so stream metadata and later set_pp() use the
     * actual ISP dimensions rather than the caller's shorthand zeros. */
    s_cam_out.cfg.stream.width = s_cam_ctx.cfg.output_width;
    s_cam_out.cfg.stream.height = s_cam_ctx.cfg.output_height;
    if (s_cam_ctx.cfg.sp_enable == TRUE) {
        s_cam_out.cfg.frame.width = s_cam_ctx.cfg.sp_width;
        s_cam_out.cfg.frame.height = s_cam_ctx.cfg.sp_height;
    }
    s_cam_out.src_width = s_cam_ctx.cfg.output_width;
    s_cam_out.src_height = s_cam_ctx.cfg.output_height;

    /* --- consumers: backends, then bonds, then overlays --- */
    t = TUYA_MIPI_CSI_MS();
    ret = tkl_cam_out_consumers_open();
    ms_consumer = TUYA_MIPI_CSI_MS() - t;
    if (ret != OPRT_OK) {
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

    t = TUYA_MIPI_CSI_MS();

    /* 1-3. overlays, then all bonds, then the backends. tkl_venc_stop() and
     *      tkl_gpu_stop() each stop their own bond and close only their own
     *      backend, so calling them back to back keeps the rule that no backend
     *      is touched while any bond is still live. */
    tkl_cam_out_consumers_close();
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

    os_memset(&s_cam_out, 0, sizeof(s_cam_out));
    return OPRT_OK;
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

    if (s_cam_out.venc != NULL) {
        (void)tkl_venc_deinit(s_cam_out.venc);
        s_cam_out.venc = NULL;
    }
    if (s_cam_out.gpu != NULL) {
        (void)tkl_gpu_deinit(s_cam_out.gpu);
        s_cam_out.gpu = NULL;
    }
}

/**
 * @brief Rebuild every consumer against the camera's current flexa source.
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
    TUYA_MIPI_CSI_FLEXA_SOURCE_T flexa;
    OPERATE_RET ret;

    if (s_cam_out.want_display != TRUE && s_cam_out.want_encode != TRUE) {
        return OPRT_OK;
    }

    os_memset(&flexa, 0, sizeof(flexa));
    ret = cam_mipi_get_flexa_source(&flexa);
    if (ret != OPRT_OK || flexa.isp_handle == NULL) {
        tkl_log_output("tkl_cam_out: flexa source unavailable after switch: %d\r\n", ret);
        return (ret != OPRT_OK) ? ret : OPRT_RESOURCE_NOT_READY;
    }

    if (s_cam_out.want_display == TRUE) {
        ret = tkl_cam_out_display_open(&flexa);
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    if (s_cam_out.want_encode == TRUE) {
        ret = tkl_cam_out_encode_open(&flexa);
        if (ret != OPRT_OK) {
            goto err_display;
        }
    }

    if (s_cam_out.want_display == TRUE) {
        ret = tkl_gpu_start(s_cam_out.gpu);
        if (ret != OPRT_OK) {
            goto err_encode;
        }
    }

    if (s_cam_out.want_encode == TRUE) {
        ret = tkl_venc_start(s_cam_out.venc);
        if (ret != OPRT_OK) {
            goto err_gpu_started;
        }
    }

    if (s_cam_out.want_display == TRUE) {
        ret = tkl_cam_out_osd_display_open();
        if (ret != OPRT_OK) {
            goto err_gpu_started;
        }
    }

    return OPRT_OK;

err_gpu_started:
    if (s_cam_out.want_display == TRUE) {
        (void)tkl_gpu_stop(s_cam_out.gpu);
    }
err_encode:
    if (s_cam_out.venc != NULL) {
        (void)tkl_venc_deinit(s_cam_out.venc);
        s_cam_out.venc = NULL;
    }
err_display:
    if (s_cam_out.gpu != NULL) {
        (void)tkl_gpu_deinit(s_cam_out.gpu);
        s_cam_out.gpu = NULL;
    }
    return ret;
}

OPERATE_RET tkl_mipi_csi_set_pp(TUYA_CAMERA_OUTPUT_TYPE_E type, UINT16_T width, UINT16_T height)
{
    TUYA_MIPI_CSI_FLEXA_SOURCE_T flexa;
    UINT16_T old_w;
    UINT16_T old_h;
    OPERATE_RET ret;
    BOOL_T rebuild;
    BOOL_T want_frame;
    UINT8_T chn_id;
    VOID *isp_before = NULL;
    UINT8_T *src_before = NULL;

    if (width == 0U || height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (type != TUYA_CAMERA_OUTPUT_FRAME &&
        type != TUYA_CAMERA_OUTPUT_PIXEL &&
        type != TUYA_CAMERA_OUTPUT_STREAM) {
        return OPRT_INVALID_PARM;
    }

    /* PIXEL and STREAM are the same request: both address the shared MP, whose
     * authoritative resolution lives in cfg.stream. FRAME addresses the SP. */
    want_frame = (type == TUYA_CAMERA_OUTPUT_FRAME) ? TRUE : FALSE;
    chn_id = (want_frame == TRUE) ? CAM_CHN_SP
                                  : CAM_CHN_MP;

    if (s_cam_out.state == TKL_CAM_ST_INIT) {
        /* not streaming yet - record the value, start() applies it */
        s_cam_out.pp_pending = TRUE;
        s_cam_out.pp_type = type;
        s_cam_out.pp_width = width;
        s_cam_out.pp_height = height;
        return OPRT_OK;
    }

    /* Switching output geometry restarts the ISP and rebuilds every consumer,
     * so there has to be a running pipeline to switch. */
    if (s_cam_out.state != TKL_CAM_ST_STARTED) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (want_frame == TRUE) {
        if (s_cam_ctx.cfg.sp_enable != TRUE) {
            return OPRT_RESOURCE_NOT_READY;     /* no SP was built */
        }
        old_w = s_cam_out.cfg.frame.width;
        old_h = s_cam_out.cfg.frame.height;
    } else {
        if (s_cam_ctx.cfg.enable_flexa != TRUE) {
            return OPRT_RESOURCE_NOT_READY;     /* no MP was built */
        }
        old_w = s_cam_out.cfg.stream.width;
        old_h = s_cam_out.cfg.stream.height;
    }

    if (old_w == width && old_h == height) {
        return OPRT_OK;
    }

    /* Only an MP switch needs the consumers torn down. Both consumers bond to MP
     * alone, and cam_mipi_set_output() reaches its stop+start rebuild - the thing
     * that invalidates isp_handle - only for MP with SP off. An SP switch is
     * rejected outright when SP is off, so it always takes the channel
     * close/open path and leaves controller, ISP, sensor and MP untouched.
     *
     * Leaving the MP bonds live across it is safe only because both bond ISRs
     * filter the channel. **Do not reinstate this shortcut if either filter is
     * removed** - without it an SP close/open fires spurious SET_FRAME_READY at
     * the encoder mid-transition.
     *
     * Verified rather than trusted: isp_handle and the MP flexa buffer are
     * snapshotted and re-checked below. */
    rebuild = (chn_id == CAM_CHN_MP) ? TRUE : FALSE;

    if (rebuild == TRUE) {
        tkl_cam_out_consumers_close();
    } else {
        os_memset(&flexa, 0, sizeof(flexa));
        if (cam_mipi_get_flexa_source(&flexa) == OPRT_OK) {
            isp_before = flexa.isp_handle;
            src_before = flexa.src_buffer;
        }
    }

    ret = cam_mipi_set_output(chn_id, width, height);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: set_output %ux%u failed: %d, restoring\r\n",
                       width, height, ret);
        /* the camera kept its old geometry, so rebuilding is enough. On the SP
         * path nothing was torn down and MP never stopped, so there is nothing
         * to restore - set_output leaves SP closed and MP streaming. */
        if (rebuild == TRUE) {
            (void)tkl_cam_out_consumers_open();
        }
        return ret;
    }

    if (chn_id == CAM_CHN_MP) {
        s_cam_out.cfg.stream.width = width;
        s_cam_out.cfg.stream.height = height;
        s_cam_out.src_width = width;
        s_cam_out.src_height = height;
    } else {
        /* SP feeds nobody downstream - resizing it rebuilds nothing */
        s_cam_out.cfg.frame.width = width;
        s_cam_out.cfg.frame.height = height;
    }

    if (rebuild != TRUE) {
        /* Assumption check, not a routine branch: if the SP switch moved the ISP
         * handle or the MP flexa buffer after all, every live bond now points at
         * dead state and must be rebuilt. */
        os_memset(&flexa, 0, sizeof(flexa));
        if (cam_mipi_get_flexa_source(&flexa) != OPRT_OK ||
            flexa.isp_handle != isp_before ||
            flexa.src_buffer != src_before) {
            tkl_log_output("tkl_cam_out: SP switch moved the MP source "
                           "(isp %p->%p buf %p->%p), rebuilding consumers\r\n",
                           isp_before, flexa.isp_handle,
                           (VOID *)src_before, (VOID *)flexa.src_buffer);
            tkl_cam_out_consumers_close();
            rebuild = TRUE;
        }
    }

    if (rebuild != TRUE) {
        tkl_log_output("tkl_cam_out: chn%u switched %ux%u -> %ux%u (MP consumers kept)\r\n",
                       (unsigned int)chn_id, old_w, old_h, width, height);
        return OPRT_OK;
    }

    ret = tkl_cam_out_consumers_open();
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_cam_out: rebuild after switch failed: %d, reverting to %ux%u\r\n",
                       ret, old_w, old_h);

        if (cam_mipi_set_output(chn_id, old_w, old_h) == OPRT_OK) {
            if (chn_id == CAM_CHN_MP) {
                s_cam_out.cfg.stream.width = old_w;
                s_cam_out.cfg.stream.height = old_h;
                s_cam_out.src_width = old_w;
                s_cam_out.src_height = old_h;
            } else {
                s_cam_out.cfg.frame.width = old_w;
                s_cam_out.cfg.frame.height = old_h;
            }
            (void)tkl_cam_out_consumers_open();
        }
        return ret;
    }

    tkl_log_output("tkl_cam_out: chn%u switched %ux%u -> %ux%u\r\n",
                   (unsigned int)chn_id, old_w, old_h, width, height);
    return OPRT_OK;
}

OPERATE_RET tkl_mipi_csi_set_bitrate(UINT32_T bitrate)
{
    TUYA_VENC_RATE_CTRL_T rc;

    if (bitrate == 0U) {
        return OPRT_INVALID_PARM;
    }

    /* The encoder is built by start() and torn down by stop(), so there is
     * nothing to configure in between. Refused rather than remembered: a
     * setting that silently waits for the next start() is a bug factory. */
    if (s_cam_out.state != TKL_CAM_ST_STARTED || s_cam_out.venc == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    os_memset(&rc, 0, sizeof(rc));
    rc.bitrate = bitrate;
    s_cam_out.cfg.stream.bitrate = bitrate;
    return tkl_venc_set_rate_ctrl(s_cam_out.venc, &rc);
}

OPERATE_RET tkl_mipi_csi_force_idr(VOID_T)
{
    if (s_cam_out.state != TKL_CAM_ST_STARTED || s_cam_out.venc == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    return tkl_venc_request_idr(s_cam_out.venc);
}

OPERATE_RET tkl_mipi_csi_set_osd(TUYA_CAMERA_OUTPUT_TYPE_E type, CONST TUYA_OSD_T *osd)
{
    if (osd == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_out.state != TKL_CAM_ST_STARTED) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (type == TUYA_CAMERA_OUTPUT_PIXEL) {
        if (s_cam_out.osd_display == NULL) {
            return OPRT_NOT_SUPPORTED;
        }
        return tkl_osd_engine_submit(s_cam_out.osd_display, osd);
    }

    if (type != TUYA_CAMERA_OUTPUT_STREAM) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_out.venc == NULL) {
        return OPRT_NOT_SUPPORTED;
    }

    return tkl_venc_set_osd(s_cam_out.venc, osd);
}
