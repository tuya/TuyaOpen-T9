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
#include <os/mem.h>
#include <os/str.h>
#include "tkl_output.h"

typedef struct {
    BOOL_T initialized;
    BOOL_T started;
    TUYA_CAMERA_MIPI_CFG_T cfg;
    VOID *isp_handle;
    bk_camera_bus_t *bus_handle;
    bk_camera_sensor_handle_t sensor_handle;
    bk_isp_camera_ctlr_handle_t camera_ctlr_handle;
} tkl_camera_mipi_ctx_t;

static tkl_camera_mipi_ctx_t s_cam_ctx;

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
        tkl_camera_mipi_runtime_reset(FALSE);
    }

    tkl_camera_mipi_runtime_reset(FALSE);
    os_memcpy(&s_cam_ctx.cfg, &cfg_local, sizeof(cfg_local));

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
    if (control == NULL) {
        tkl_log_output( "camera control is null\r\n");
        ret = OPRT_COM_ERROR;
        goto err_start;
    }
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

    s_cam_ctx.started = TRUE;
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

OPERATE_RET tkl_camera_mipi_get_frame(void *frame)
{
    OPERATE_RET ret;
    uint32_t expect_size;
    TUYA_CAMERA_MIPI_FRAME_T *req = (TUYA_CAMERA_MIPI_FRAME_T *)frame;

    if (req == NULL || req->buffer == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_cam_ctx.started != TRUE || s_cam_ctx.camera_ctlr_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_cam_ctx.cfg.enable_flexa != FALSE || s_cam_ctx.cfg.work_mode != 0U) {
        tkl_log_output(
                "get_frame is not supported in flexa mode, current enable_flexa=%u work_mode=%u, use gpu/flexa path or reopen camera in frame mode\r\n",
                (unsigned int)s_cam_ctx.cfg.enable_flexa,
                (unsigned int)s_cam_ctx.cfg.work_mode);
        return OPRT_NOT_SUPPORTED;
    }

    expect_size = bk_image_size_get(s_cam_ctx.cfg.output_width,
                                    s_cam_ctx.cfg.output_height,
                                    (bk_pixel_format_t)s_cam_ctx.cfg.output_format);
    if (expect_size == 0U || req->size < expect_size) {
        return OPRT_INVALID_PARM;
    }

    ret = bk_isp_camera_read(s_cam_ctx.camera_ctlr_handle,
                             ISP_MP_CHN_ID,
                             req->buffer,
                             expect_size,
                             (req->timeout_ms != 0U) ? req->timeout_ms : s_cam_ctx.cfg.read_timeout_ms);
    if (ret != OPRT_OK) {
        return ret;
    }

    req->out_len = expect_size;
    req->width = s_cam_ctx.cfg.output_width;
    req->height = s_cam_ctx.cfg.output_height;
    req->format = s_cam_ctx.cfg.output_format;

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
