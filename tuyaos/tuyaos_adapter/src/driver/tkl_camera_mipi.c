#include "tkl_camera_mipi.h"

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
#include <os/str.h>
#include "sys_driver.h"
#include "tkl_output.h"
#include "tkl_system.h"

#define TKL_CAM_MIPI_POWER_SETTLE_MS    20U
#define TKL_CAM_MIPI_SYS_ANA_REG_BASE   (0x44010000U)
#define TKL_CAM_MIPI_LDO_ANA_REG        (0x69U)

typedef struct {
    BOOL_T initialized;
    BOOL_T started;
    BOOL_T sensor_power_on;
    TUYA_CAMERA_MIPI_CFG_T cfg;
    UINT16_T active_output_width;
    UINT16_T active_output_height;
    VOID *isp_handle;
    bk_camera_bus_t *bus_handle;
    bk_camera_sensor_handle_t sensor_handle;
    bk_isp_camera_ctlr_handle_t camera_ctlr_handle;
} tkl_camera_mipi_ctx_t;

static tkl_camera_mipi_ctx_t s_cam_ctx;

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

    ret = tkl_camera_mipi_open_sp_channel();
    if (ret != OPRT_OK) {
        goto err_start;
    }

    s_cam_ctx.active_output_width = s_cam_ctx.cfg.output_width;
    s_cam_ctx.active_output_height = s_cam_ctx.cfg.output_height;
    s_cam_ctx.started = TRUE;
    tkl_log_output("camera start ok, sensor=%ux%u@%u, isp_output=%ux%u fmt=%u\r\n",
            s_cam_ctx.cfg.sensor_width,
            s_cam_ctx.cfg.sensor_height,
            s_cam_ctx.cfg.sensor_fps,
            s_cam_ctx.cfg.output_width,
            s_cam_ctx.cfg.output_height,
            (unsigned int)s_cam_ctx.cfg.output_format);
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

OPERATE_RET tkl_camera_mipi_get_frame(TUYA_CAMERA_MIPI_FRAME_T *frame)
{
    OPERATE_RET ret;
    uint32_t expect_size;
    uint8_t channel_id;
    uint8_t isp_channel_id;
    uint16_t width;
    uint16_t height;
    uint32_t format;

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

    ret = bk_isp_camera_read(s_cam_ctx.camera_ctlr_handle,
                             isp_channel_id,
                             frame->buffer,
                             expect_size,
                             (frame->timeout_ms != 0U) ? frame->timeout_ms : s_cam_ctx.cfg.read_timeout_ms);
    if (ret != OPRT_OK) {
        return ret;
    }

    frame->out_len = expect_size;
    frame->width = width;
    frame->height = height;
    frame->format = format;

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
