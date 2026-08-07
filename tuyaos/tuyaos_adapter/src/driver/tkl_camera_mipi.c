#include "tkl_camera_mipi.h"

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
#if CONFIG_L2_CACHE_ENABLE || CONFIG_DCACHE
#include "cache.h"
#endif
#include "sys_driver.h"
#include "tkl_output.h"
#include "tkl_system.h"

#define TKL_CAM_MIPI_POWER_SETTLE_MS    20U
#define TKL_CAM_MIPI_SYS_ANA_REG_BASE   (0x44010000U)
#define TKL_CAM_MIPI_LDO_ANA_REG        (0x69U)
#define TKL_CAM_MIPI_SP_WARMUP_DROP_CNT 1U
#define TKL_CAM_MIPI_MP_FRAME_WAIT_MS   200U

typedef struct {
    BOOL_T initialized;
    BOOL_T started;
    BOOL_T sensor_power_on;
    BOOL_T sp_warmup_pending;
    TUYA_CAMERA_MIPI_CFG_T cfg;
    UINT16_T active_output_width;
    UINT16_T active_output_height;
    VOID *isp_handle;
    bk_camera_bus_t *bus_handle;
    bk_camera_sensor_handle_t sensor_handle;
    bk_isp_camera_ctlr_handle_t camera_ctlr_handle;
} tkl_camera_mipi_ctx_t;

typedef struct {
    beken_semaphore_t sem;
    BOOL_T sem_inited;
    BOOL_T waiting;
    BOOL_T frame_complete_hit;
} tkl_camera_mipi_mp_sync_t;

static tkl_camera_mipi_ctx_t s_cam_ctx;
static tkl_camera_mipi_mp_sync_t s_mp_frame_sync;

static VOID_T tkl_camera_mipi_gpio_bank_ldo_enable(VOID_T)
{
    UINT32_T reg = REG_READ(TKL_CAM_MIPI_SYS_ANA_REG_BASE + TKL_CAM_MIPI_LDO_ANA_REG * 4U);

    reg |= (0xFU << 28) | (0x2U << 23) | (0x7U << 19) | (0x7U << 15);
    reg &= ~(0xFU << 11);
    reg |= (0x8U << 11);
    REG_WRITE(TKL_CAM_MIPI_SYS_ANA_REG_BASE + TKL_CAM_MIPI_LDO_ANA_REG * 4U, reg);
}

static OPERATE_RET tkl_camera_mipi_sensor_power_enable(BOOL_T enable)
{
    pm_auxldo_ctrl_cfg_t auxldo_cfg = {0};
    int ldo_en = (enable != FALSE) ? PM_AUXLDO_ENABLE : PM_AUXLDO_DISABLE;

    tkl_log_output("camera sensor power %s (gpio_bank+1p8v+1p2v)\r\n",
                   (enable != FALSE) ? "on" : "off");

    if (enable != FALSE) {
        /* Must run before any GPIO59+ / I2C access. */
        tkl_camera_mipi_gpio_bank_ldo_enable();
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
        tkl_system_sleep(TKL_CAM_MIPI_POWER_SETTLE_MS);
    }
    s_cam_ctx.sensor_power_on = (enable != FALSE) ? TRUE : FALSE;
    return OPRT_OK;
}

static void tkl_camera_mipi_runtime_reset(BOOL_T keep_config)
{
    TUYA_CAMERA_MIPI_CFG_T cfg_backup;

    if (keep_config == TRUE) {
        os_memcpy(&cfg_backup, &s_cam_ctx.cfg, sizeof(cfg_backup));
    }

    os_memset(&s_cam_ctx, 0, sizeof(s_cam_ctx));

    if (keep_config == TRUE) {
        os_memcpy(&s_cam_ctx.cfg, &cfg_backup, sizeof(cfg_backup));
        s_cam_ctx.initialized = TRUE;
    }
}

static void tkl_camera_mipi_cfg_normalize(TUYA_CAMERA_MIPI_CFG_T *cfg)
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
static OPERATE_RET tkl_camera_mipi_apply_mirror(VOID_T)
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

static OPERATE_RET tkl_camera_mipi_cfg_validate(const TUYA_CAMERA_MIPI_CFG_T *cfg)
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

static uint32_t tkl_camera_mipi_sp_frame_size_get(void)
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

static OPERATE_RET tkl_camera_mipi_open_sp_channel(void)
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

static OPERATE_RET tkl_camera_mipi_sensor_check_format(bk_camera_sensor_handle_t sensor_handle,
                                                       const TUYA_CAMERA_MIPI_CFG_T *cfg)
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

static OPERATE_RET tkl_camera_mipi_sensor_detect_and_check(TUYA_CAMERA_MIPI_CFG_T *cfg)
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

    ret = tkl_camera_mipi_sensor_check_format(s_cam_ctx.sensor_handle, cfg);
    if (ret != OPRT_OK) {
        bk_camera_sensor_destroy(s_cam_ctx.sensor_handle);
        s_cam_ctx.sensor_handle = NULL;
        return ret;
    }

    return OPRT_OK;
}

static OPERATE_RET tkl_camera_mipi_build_isp_ctlr_cfg(bk_isp_camera_ctlr_config_t *isp_ctlr_cfg)
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

static void tkl_camera_mipi_close_channels(void)
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

static void tkl_camera_mipi_cleanup_handles(void)
{
    tkl_camera_mipi_close_channels();

    s_cam_ctx.isp_handle = NULL;

    if (s_cam_ctx.camera_ctlr_handle != NULL) {
        bk_isp_camera_deinit(s_cam_ctx.camera_ctlr_handle);
        bk_isp_camera_delete(s_cam_ctx.camera_ctlr_handle);
        s_cam_ctx.camera_ctlr_handle = NULL;
    }
}

static void tkl_camera_mipi_cleanup_resources(void)
{
    tkl_camera_mipi_cleanup_handles();

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
        (void)tkl_camera_mipi_sensor_power_enable(FALSE);
    }
}

OPERATE_RET tkl_camera_mipi_init(TUYA_CAMERA_MIPI_CFG_T *config)
{
    OPERATE_RET ret;
    TUYA_CAMERA_MIPI_CFG_T cfg_local;
    bk_camera_bus_config_t bus_cfg;

    if (config == NULL) {
        return OPRT_INVALID_PARM;
    }

    cfg_local = *config;
    tkl_camera_mipi_cfg_normalize(&cfg_local);
    ret = tkl_camera_mipi_cfg_validate(&cfg_local);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (s_cam_ctx.started == TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_cam_ctx.initialized == TRUE) {
        tkl_camera_mipi_cleanup_resources();
    }

    tkl_camera_mipi_runtime_reset(FALSE);
    os_memcpy(&s_cam_ctx.cfg, &cfg_local, sizeof(cfg_local));

    ret = tkl_camera_mipi_sensor_power_enable(TRUE);
    if (ret != OPRT_OK) {
        tkl_camera_mipi_runtime_reset(FALSE);
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
        (void)tkl_camera_mipi_sensor_power_enable(FALSE);
        tkl_camera_mipi_runtime_reset(FALSE);
        return OPRT_COM_ERROR;
    }

    ret = bk_camera_bus_enable(s_cam_ctx.bus_handle);
    if (ret != OPRT_OK) {
        tkl_log_output( "camera bus enable failed: %d\r\n", ret);
        goto err_init;
    }

    ret = tkl_camera_mipi_sensor_detect_and_check(&s_cam_ctx.cfg);
    if (ret != OPRT_OK) {
        goto err_init;
    }

    s_cam_ctx.initialized = TRUE;
    return OPRT_OK;

err_init:
    tkl_camera_mipi_cleanup_resources();
    tkl_camera_mipi_runtime_reset(FALSE);
    return ret;
}

static OPERATE_RET tkl_camera_mipi_output_validate(UINT16_T width, UINT16_T height)
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
 * @brief Set ISP output resolution (main path)
 * @param[in] width ISP output width in pixels
 * @param[in] height ISP output height in pixels
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_camera_mipi_set_output(UINT16_T width, UINT16_T height)
{
    OPERATE_RET ret;

    if (s_cam_ctx.started == TRUE) {
        tkl_log_output("set_output rejected: camera is started, stop first\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_camera_mipi_output_validate(width, height);
    if (ret != OPRT_OK) {
        return ret;
    }

    s_cam_ctx.cfg.output_width = width;
    s_cam_ctx.cfg.output_height = height;
    return OPRT_OK;
}

OPERATE_RET tkl_camera_mipi_deinit(void)
{
    if (s_cam_ctx.started == TRUE) {
        OPERATE_RET ret = tkl_camera_mipi_stop();
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    tkl_camera_mipi_cleanup_resources();
    tkl_camera_mipi_runtime_reset(FALSE);
    return OPRT_OK;
}

OPERATE_RET tkl_camera_mipi_start(void)
{
    OPERATE_RET ret = OPRT_OK;
    bk_isp_camera_channel_config_t mp_cfg;
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

    tkl_camera_mipi_cleanup_handles();

    ret = tkl_camera_mipi_build_isp_ctlr_cfg(&isp_ctlr_cfg);
    if (ret != OPRT_OK) {
        return ret;
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
        tkl_log_output( "bk_isp_camera_channel_open failed: %d\r\n", ret);
        goto err_start;
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
    ret = tkl_camera_mipi_apply_mirror();
    if (ret != OPRT_OK) {
        goto err_start;
    }

    ret = tkl_camera_mipi_open_sp_channel();
    if (ret != OPRT_OK) {
        goto err_start;
    }

    s_cam_ctx.active_output_width = s_cam_ctx.cfg.output_width;
    s_cam_ctx.active_output_height = s_cam_ctx.cfg.output_height;
    s_cam_ctx.started = TRUE;
    s_cam_ctx.sp_warmup_pending =
        (s_cam_ctx.cfg.sp_enable != FALSE) ? TRUE : FALSE;
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
    tkl_camera_mipi_cleanup_handles();
    return ret;
}

OPERATE_RET tkl_camera_mipi_stop(void)
{
    if (s_cam_ctx.initialized != TRUE) {
        return OPRT_OK;
    }

    if (s_cam_ctx.started != TRUE) {
        if (s_cam_ctx.camera_ctlr_handle == NULL) {
            return OPRT_OK;
        }
    }

    tkl_camera_mipi_cleanup_handles();
    return OPRT_OK;
}

/**
 * @brief Flush CPU dcache for ISP DMA plane before CPU read
 * @param[in] addr plane base address
 * @param[in] size plane size in bytes
 * @return none
 */
STATIC VOID_T __tkl_camera_mipi_plane_cache_sync(VOID_T *addr, UINT32_T size)
{
    if (addr == NULL || size == 0U) {
        return;
    }

#if CONFIG_L2_CACHE_ENABLE || CONFIG_DCACHE
    arch_dcache_flush_and_invd_range(addr, size);
#else
    (void)addr;
    (void)size;
#endif
}

/**
 * @brief MP frame-complete ISR used to sync SP pop with flexa MP
 * @param[in] seqence frame sequence
 * @param[in] line line index
 * @param[in] chnl channel id
 * @param[in] ok non-zero means frame complete success (vendor callback polarity)
 * @param[in] param unused
 * @return none
 */
STATIC VOID_T __tkl_camera_mipi_mp_frame_complete_cb(UINT32_T seqence,
                                                     UINT32_T line,
                                                     UINT8_T chnl,
                                                     UINT8_T ok,
                                                     VOID_T *param)
{
    (void)seqence;
    (void)line;
    (void)param;

    if (chnl != ISP_MP_CHN_ID || ok == 0U) {
        return;
    }

    if (s_mp_frame_sync.waiting != FALSE && s_mp_frame_sync.frame_complete_hit == FALSE) {
        s_mp_frame_sync.frame_complete_hit = TRUE;
        (void)rtos_set_semaphore(&s_mp_frame_sync.sem);
    }
}

/**
 * @brief Wait one MP frame-complete when MP is running flexa
 * @return OPRT_OK on success; non-fatal timeout returns OPRT_OK with log
 */
STATIC OPERATE_RET __tkl_camera_mipi_wait_mp_frame_complete(VOID_T)
{
    OPERATE_RET ret = OPRT_OK;

    if (s_cam_ctx.camera_ctlr_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_cam_ctx.cfg.enable_flexa == FALSE || s_cam_ctx.cfg.work_mode == 0U) {
        return OPRT_OK;
    }

    if (s_mp_frame_sync.sem_inited == FALSE) {
        if (rtos_init_semaphore(&s_mp_frame_sync.sem, 1) != BK_OK) {
            return OPRT_COM_ERROR;
        }
        s_mp_frame_sync.sem_inited = TRUE;
    }

    while (rtos_get_semaphore(&s_mp_frame_sync.sem, 0) == BK_OK) {
    }

    s_mp_frame_sync.waiting = TRUE;
    s_mp_frame_sync.frame_complete_hit = FALSE;

    ret = bk_isp_camera_register_isr_callback(s_cam_ctx.camera_ctlr_handle,
                                              ISR_TYPE_FRAME_COMPLETE,
                                              __tkl_camera_mipi_mp_frame_complete_cb,
                                              &s_mp_frame_sync);
    if (ret != OPRT_OK) {
        s_mp_frame_sync.waiting = FALSE;
        tkl_log_output("camera wait MP frame-complete register failed: %d\r\n", ret);
        return ret;
    }

    if (rtos_get_semaphore(&s_mp_frame_sync.sem, TKL_CAM_MIPI_MP_FRAME_WAIT_MS) != BK_OK) {
        tkl_log_output("camera wait MP frame-complete timeout %ums\r\n",
                       TKL_CAM_MIPI_MP_FRAME_WAIT_MS);
    }

    (void)bk_isp_camera_deregister_isr_callback(s_cam_ctx.camera_ctlr_handle,
                                                ISR_TYPE_FRAME_COMPLETE,
                                                &s_mp_frame_sync);
    s_mp_frame_sync.waiting = FALSE;
    return OPRT_OK;
}

/**
 * @brief Resolve ISP control and channel config
 * @param[in] chn_id ISP_MP_CHN_ID or ISP_SP_CHN_ID
 * @param[out] isp_control ISP control pointer
 * @param[out] chn_cfg channel config pointer
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_camera_mipi_chn_get_control(UINT8_T chn_id,
                                                     isp_control_t **isp_control,
                                                     isp_channel_config_t **chn_cfg)
{
    isp_control_t *isp;

    if (isp_control == NULL || chn_cfg == NULL ||
        (chn_id != ISP_MP_CHN_ID && chn_id != ISP_SP_CHN_ID)) {
        return OPRT_INVALID_PARM;
    }

    isp = (isp_control_t *)s_cam_ctx.isp_handle;
    if (isp == NULL || isp->pop_buf == NULL || isp->free_buf == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    *isp_control = isp;
    *chn_cfg = &isp->chn[chn_id];
    if ((*chn_cfg)->enable == 0U) {
        return OPRT_RESOURCE_NOT_READY;
    }

    return OPRT_OK;
}

/**
 * @brief Drop one frame for warmup
 * @param[in] chn_id ISP channel id
 * @param[in] timeout_ms pop timeout
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_camera_mipi_chn_drop_frame(UINT8_T chn_id, UINT32_T timeout_ms)
{
    isp_control_t *isp_control = NULL;
    isp_channel_config_t *chn_cfg = NULL;
    VIDEO_BUF_S buf;
    OPERATE_RET op_ret;
    INT_T pop_ret;

    op_ret = __tkl_camera_mipi_chn_get_control(chn_id, &isp_control, &chn_cfg);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    os_memset(&buf, 0, sizeof(buf));
    pop_ret = isp_control->pop_buf(chn_cfg->channel, &buf, timeout_ms);
    if (pop_ret != BK_OK) {
        tkl_log_output("camera chn%u warmup pop_buf failed: %d\r\n",
                       (unsigned int)chn_id, pop_ret);
        return OPRT_COM_ERROR;
    }

    (void)isp_control->free_buf(chn_cfg->channel, &buf);
    return OPRT_OK;
}

/**
 * @brief Copy NV12 planes into tightly packed buffer (stride == width)
 * @param[in] width frame width, must be even
 * @param[in] height frame height, must be even
 * @param[in] y_plane Y plane base
 * @param[in] y_stride Y plane stride in bytes
 * @param[in] uv_plane UV plane base
 * @param[in] uv_stride UV plane stride in bytes
 * @param[out] nv12 tightly packed NV12 output
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_camera_mipi_nv12_copy_tight(UINT16_T width,
                                                     UINT16_T height,
                                                     const UINT8_T *y_plane,
                                                     UINT32_T y_stride,
                                                     const UINT8_T *uv_plane,
                                                     UINT32_T uv_stride,
                                                     UINT8_T *nv12)
{
    UINT32_T row;
    UINT32_T y_plane_size;

    if (y_plane == NULL || uv_plane == NULL || nv12 == NULL ||
        (width & 1U) != 0U || (height & 1U) != 0U ||
        y_stride < (UINT32_T)width || uv_stride < (UINT32_T)width) {
        return OPRT_INVALID_PARM;
    }

    y_plane_size = (UINT32_T)width * (UINT32_T)height;

    if (y_stride == (UINT32_T)width && uv_stride == (UINT32_T)width) {
        if (uv_plane == (y_plane + y_plane_size)) {
            os_memcpy(nv12, y_plane, y_plane_size + (y_plane_size / 2U));
        } else {
            os_memcpy(nv12, y_plane, y_plane_size);
            os_memcpy(nv12 + y_plane_size, uv_plane, y_plane_size / 2U);
        }
        return OPRT_OK;
    }

    for (row = 0U; row < (UINT32_T)height; row++) {
        os_memcpy(nv12 + row * (UINT32_T)width,
                  y_plane + row * y_stride,
                  (UINT32_T)width);
    }

    for (row = 0U; row < ((UINT32_T)height / 2U); row++) {
        os_memcpy(nv12 + y_plane_size + row * (UINT32_T)width,
                  uv_plane + row * uv_stride,
                  (UINT32_T)width);
    }

    return OPRT_OK;
}

/**
 * @brief Copy RGBX/BGRx plane into tightly packed buffer
 * @param[in] width frame width
 * @param[in] height frame height
 * @param[in] src plane base
 * @param[in] src_stride source row stride in bytes
 * @param[in] row_bytes valid bytes per row (width * 4 for ISP RGB888)
 * @param[out] dst tightly packed output
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_camera_mipi_plane_copy_tight(UINT16_T width,
                                                      UINT16_T height,
                                                      const UINT8_T *src,
                                                      UINT32_T src_stride,
                                                      UINT32_T row_bytes,
                                                      UINT8_T *dst)
{
    UINT32_T row;

    (void)width;
    if (src == NULL || dst == NULL || height == 0U || row_bytes == 0U ||
        src_stride < row_bytes) {
        return OPRT_INVALID_PARM;
    }

    if (src_stride == row_bytes) {
        os_memcpy(dst, src, row_bytes * (UINT32_T)height);
        return OPRT_OK;
    }

    for (row = 0U; row < (UINT32_T)height; row++) {
        os_memcpy(dst + row * row_bytes, src + row * src_stride, row_bytes);
    }

    return OPRT_OK;
}

/**
 * @brief Resolve plane CPU pointer and size from VIDEO_BUF_S
 * @param[in] plane plane descriptor
 * @param[out] addr plane address
 * @param[out] size plane size
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_camera_mipi_plane_addr_get(const VB_PLANE_S *plane,
                                                    uintptr_t *addr,
                                                    UINT32_T *size)
{
    if (plane == NULL || addr == NULL || size == NULL) {
        return OPRT_INVALID_PARM;
    }

    *addr = (uintptr_t)plane->dmaPhyAddr;
    if (plane->pUserAddr != NULL) {
        *addr = (uintptr_t)plane->pUserAddr;
    }
    *size = plane->size;
    if (*addr == 0U || *size == 0U) {
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

/**
 * @brief Pop one ISP frame and copy to tightly packed caller buffer
 * @param[in] chn_id ISP channel id
 * @param[in] format pixel format
 * @param[in] width frame width
 * @param[in] height frame height
 * @param[out] dst caller buffer
 * @param[in] dst_size buffer capacity
 * @param[in] timeout_ms pop timeout
 * @param[out] out_len packed bytes written
 * @param[out] y_stride packed Y/RGB row stride
 * @param[out] uv_stride packed UV row stride (0 if N/A)
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_camera_mipi_pop_copy_frame(UINT8_T chn_id,
                                                    UINT32_T format,
                                                    UINT16_T width,
                                                    UINT16_T height,
                                                    UINT8_T *dst,
                                                    UINT32_T dst_size,
                                                    UINT32_T timeout_ms,
                                                    UINT32_T *out_len,
                                                    UINT32_T *y_stride,
                                                    UINT32_T *uv_stride)
{
    isp_control_t *isp_control = NULL;
    isp_channel_config_t *chn_cfg = NULL;
    VIDEO_BUF_S vbuf;
    OPERATE_RET op_ret;
    INT_T pop_ret;
    BOOL_T held = FALSE;
    UINT32_T need_size;
    UINT32_T packed_y_stride = 0U;
    UINT32_T packed_uv_stride = 0U;

    if (dst == NULL || dst_size == 0U || width == 0U || height == 0U ||
        out_len == NULL || y_stride == NULL || uv_stride == NULL) {
        return OPRT_INVALID_PARM;
    }

    *out_len = 0U;
    *y_stride = 0U;
    *uv_stride = 0U;

    if (format == (UINT32_T)BK_PIXEL_FORMAT_NV12) {
        if (((width & 1U) != 0U) || ((height & 1U) != 0U)) {
            return OPRT_INVALID_PARM;
        }
        need_size = (UINT32_T)width * (UINT32_T)height * 3U / 2U;
        packed_y_stride = (UINT32_T)width;
        packed_uv_stride = (UINT32_T)width;
    } else if (format == (UINT32_T)BK_PIXEL_FORMAT_RGB888 ||
               format == (UINT32_T)BK_PIXEL_FORMAT_BGR888) {
        /* ISP outputs RGB888/BGR888 as RGBX (4 bytes/pixel). */
        need_size = (UINT32_T)width * (UINT32_T)height * 4U;
        packed_y_stride = (UINT32_T)width * 4U;
        packed_uv_stride = 0U;
    } else {
        need_size = bk_image_size_get(width, height, (bk_pixel_format_t)format);
        if (need_size == 0U || (need_size % (UINT32_T)height) != 0U) {
            return OPRT_NOT_SUPPORTED;
        }
        packed_y_stride = need_size / (UINT32_T)height;
        packed_uv_stride = 0U;
    }

    if (dst_size < need_size) {
        return OPRT_INVALID_PARM;
    }

    op_ret = __tkl_camera_mipi_chn_get_control(chn_id, &isp_control, &chn_cfg);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    os_memset(&vbuf, 0, sizeof(vbuf));
    pop_ret = isp_control->pop_buf(chn_cfg->channel, &vbuf, timeout_ms);
    if (pop_ret != BK_OK) {
        tkl_log_output("camera chn%u pop_buf failed: %d\r\n",
                       (unsigned int)chn_id, pop_ret);
        return OPRT_COM_ERROR;
    }
    held = TRUE;

    if (format == (UINT32_T)BK_PIXEL_FORMAT_NV12) {
        UINT32_T y_plane_size = (UINT32_T)width * (UINT32_T)height;
        const UINT8_T *nv12_y = NULL;
        const UINT8_T *nv12_uv = NULL;
        UINT32_T src_y_stride = (UINT32_T)width;
        UINT32_T src_uv_stride = (UINT32_T)width;

        if (vbuf.numPlanes >= 2U) {
            uintptr_t y_addr = 0U;
            uintptr_t uv_addr = 0U;
            UINT32_T y_size = 0U;
            UINT32_T uv_size = 0U;

            op_ret = __tkl_camera_mipi_plane_addr_get(&vbuf.planes[0], &y_addr, &y_size);
            if (op_ret == OPRT_OK) {
                op_ret = __tkl_camera_mipi_plane_addr_get(&vbuf.planes[1], &uv_addr, &uv_size);
            }
            if (op_ret != OPRT_OK || y_size < y_plane_size || uv_size < (y_plane_size / 2U)) {
                op_ret = OPRT_COM_ERROR;
                goto cleanup;
            }

            src_y_stride = y_size / (UINT32_T)height;
            src_uv_stride = uv_size / ((UINT32_T)height / 2U);
            if (src_y_stride < (UINT32_T)width) {
                src_y_stride = (UINT32_T)width;
            }
            if (src_uv_stride < (UINT32_T)width) {
                src_uv_stride = (UINT32_T)width;
            }

            nv12_y = (const UINT8_T *)y_addr;
            nv12_uv = (const UINT8_T *)uv_addr;
            __tkl_camera_mipi_plane_cache_sync((VOID_T *)y_addr, y_size);
            __tkl_camera_mipi_plane_cache_sync((VOID_T *)uv_addr, uv_size);
        } else if (vbuf.numPlanes >= 1U) {
            uintptr_t y_addr = 0U;
            UINT32_T frame_size = 0U;

            op_ret = __tkl_camera_mipi_plane_addr_get(&vbuf.planes[0], &y_addr, &frame_size);
            if (vbuf.imageSize != 0U) {
                frame_size = vbuf.imageSize;
            }
            if (op_ret != OPRT_OK || frame_size < need_size) {
                op_ret = OPRT_COM_ERROR;
                goto cleanup;
            }

            nv12_y = (const UINT8_T *)y_addr;
            nv12_uv = nv12_y + y_plane_size;
            __tkl_camera_mipi_plane_cache_sync((VOID_T *)y_addr, frame_size);
        } else {
            op_ret = OPRT_COM_ERROR;
            goto cleanup;
        }

        op_ret = __tkl_camera_mipi_nv12_copy_tight(width, height, nv12_y, src_y_stride,
                                                  nv12_uv, src_uv_stride, dst);
    } else {
        uintptr_t src_addr = 0U;
        UINT32_T plane_size = 0U;
        UINT32_T src_stride;

        if (vbuf.numPlanes < 1U) {
            op_ret = OPRT_COM_ERROR;
            goto cleanup;
        }

        op_ret = __tkl_camera_mipi_plane_addr_get(&vbuf.planes[0], &src_addr, &plane_size);
        if (op_ret != OPRT_OK) {
            goto cleanup;
        }

        src_stride = plane_size / (UINT32_T)height;
        if (src_stride < packed_y_stride) {
            op_ret = OPRT_COM_ERROR;
            goto cleanup;
        }

        __tkl_camera_mipi_plane_cache_sync((VOID_T *)src_addr, plane_size);
        op_ret = __tkl_camera_mipi_plane_copy_tight(width, height,
                                                   (const UINT8_T *)src_addr,
                                                   src_stride,
                                                   packed_y_stride,
                                                   dst);
    }

    if (op_ret != OPRT_OK) {
        goto cleanup;
    }

    *out_len = need_size;
    *y_stride = packed_y_stride;
    *uv_stride = packed_uv_stride;

cleanup:
    if (held != FALSE && isp_control != NULL && chn_cfg != NULL) {
        (void)isp_control->free_buf(chn_cfg->channel, &vbuf);
    }

    return op_ret;
}

/**
 * @brief Read one frame from MP/SP (frame mode only)
 * @param[in,out] frame caller buffer and channel; filled on success
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_camera_mipi_get_frame(TUYA_CAMERA_MIPI_FRAME_T *frame)
{
    OPERATE_RET op_ret;
    UINT8_T channel_id;
    UINT8_T isp_channel_id;
    UINT16_T width;
    UINT16_T height;
    UINT32_T format;
    UINT32_T expect_size;
    UINT32_T wait_ms;
    UINT32_T out_len = 0U;
    UINT32_T y_stride = 0U;
    UINT32_T uv_stride = 0U;
    UINT32_T i;

    if (frame == NULL || frame->buffer == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_ctx.started != TRUE || s_cam_ctx.camera_ctlr_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    channel_id = frame->channel_id;
    if (channel_id == TUYA_CAMERA_MIPI_CHANNEL_SP) {
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

        isp_channel_id = ISP_SP_CHN_ID;
        width = s_cam_ctx.cfg.sp_width;
        height = s_cam_ctx.cfg.sp_height;
        format = s_cam_ctx.cfg.sp_format;
        expect_size = tkl_camera_mipi_sp_frame_size_get();
    } else if (channel_id == TUYA_CAMERA_MIPI_CHANNEL_MP) {
        if (s_cam_ctx.cfg.enable_flexa != FALSE || s_cam_ctx.cfg.work_mode != 0U) {
            tkl_log_output(
                    "get_frame is not supported in MP flexa mode, current enable_flexa=%u work_mode=%u, use gpu/flexa path or reopen camera in frame mode\r\n",
                    (unsigned int)s_cam_ctx.cfg.enable_flexa,
                    (unsigned int)s_cam_ctx.cfg.work_mode);
            return OPRT_NOT_SUPPORTED;
        }

        isp_channel_id = ISP_MP_CHN_ID;
        width = s_cam_ctx.cfg.output_width;
        height = s_cam_ctx.cfg.output_height;
        format = s_cam_ctx.cfg.output_format;
        expect_size = bk_image_size_get(width, height, (bk_pixel_format_t)format);
    } else {
        return OPRT_INVALID_PARM;
    }

    if (expect_size == 0U || frame->size < expect_size) {
        return OPRT_INVALID_PARM;
    }

    wait_ms = (frame->timeout_ms != 0U) ? frame->timeout_ms : s_cam_ctx.cfg.read_timeout_ms;
    if (wait_ms == 0U) {
        wait_ms = 3000U;
    }

    if (channel_id == TUYA_CAMERA_MIPI_CHANNEL_SP) {
        op_ret = __tkl_camera_mipi_wait_mp_frame_complete();
        if (op_ret != OPRT_OK) {
            return op_ret;
        }

        if (s_cam_ctx.sp_warmup_pending != FALSE) {
            for (i = 0U; i < TKL_CAM_MIPI_SP_WARMUP_DROP_CNT; i++) {
                op_ret = __tkl_camera_mipi_chn_drop_frame(ISP_SP_CHN_ID, wait_ms);
                if (op_ret != OPRT_OK) {
                    return op_ret;
                }
            }
            s_cam_ctx.sp_warmup_pending = FALSE;

            op_ret = __tkl_camera_mipi_wait_mp_frame_complete();
            if (op_ret != OPRT_OK) {
                return op_ret;
            }
        }
    }

    op_ret = __tkl_camera_mipi_pop_copy_frame(isp_channel_id, format, width, height,
                                             frame->buffer, frame->size, wait_ms,
                                             &out_len, &y_stride, &uv_stride);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    frame->out_len = out_len;
    frame->width = width;
    frame->height = height;
    frame->format = format;
    frame->y_stride = y_stride;
    frame->uv_stride = uv_stride;

    return OPRT_OK;
}

OPERATE_RET tkl_camera_mipi_set_mirror_flip(UINT8_T hmirror, UINT8_T vflip)
{
    if (s_cam_ctx.initialized != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    s_cam_ctx.cfg.hmirror = (hmirror != 0U) ? 1U : 0U;
    s_cam_ctx.cfg.vflip = (vflip != 0U) ? 1U : 0U;

    if (s_cam_ctx.started != TRUE || s_cam_ctx.sensor_handle == NULL) {
        return OPRT_OK;
    }

    return tkl_camera_mipi_apply_mirror();
}

OPERATE_RET tkl_camera_mipi_get_mirror_flip(UINT8_T *hmirror, UINT8_T *vflip)
{
    if (s_cam_ctx.initialized != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (hmirror != NULL) {
        *hmirror = s_cam_ctx.cfg.hmirror;
    }
    if (vflip != NULL) {
        *vflip = s_cam_ctx.cfg.vflip;
    }

    return OPRT_OK;
}

/**
 * @brief Get ISP weighted mean exposure luminance via camera IOCTL
 * @param[out] luminance mean luminance * 1000
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_camera_mipi_get_exposure_luminance(UINT32_T *luminance)
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

OPERATE_RET tkl_camera_mipi_get_isp_handle(VOID **isp_handle)
{
    if (isp_handle == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_ctx.started != TRUE || s_cam_ctx.isp_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    *isp_handle = s_cam_ctx.isp_handle;
    return OPRT_OK;
}

OPERATE_RET tkl_camera_mipi_get_flexa_source(TUYA_CAMERA_MIPI_FLEXA_SOURCE_T *source)
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
