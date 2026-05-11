/**
 * @file tkl_uvc.c
 * @brief USB video camera TKL adapter
 * @version 0.1
 * @date 2026-05-09
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_memory.h"
#include "tkl_uvc.h"

#if CONFIG_USB_CAMERA
#include <common/avdk_pixel_types.h>
#include <components/avdk_utils/avdk_error.h>
#include <components/bk_uvc_camera.h>
#include <components/usb_types.h>
#include <components/usbh_hub_multiple_classes_api.h>
#include <os/os.h>

#define TKL_UVC_DEFAULT_PORT               1U
#define TKL_UVC_DEFAULT_FPS                30U
#define TKL_UVC_DEFAULT_CONNECT_TIMEOUT_MS 4000U

typedef struct {
    frame_buffer_t frame;
    VOID *payload;
    VOID *owner;
} TUYA_UVC_FRAME_T;

typedef struct {
    BOOL_T started;
    BOOL_T backend_inited;
    TUYA_UVC_CFG_T cfg;
    TUYA_UVC_SINK_T sink;
    bk_uvc_callback_t callbacks;
    bk_uvc_ctlr_handle_t backend_handle;
    beken_semaphore_t connect_sem;
} TUYA_UVC_CTX_T;

static TUYA_UVC_CTX_T *s_active_frame_ctx = NULL;

static TUYA_UVC_CTX_T *tkl_uvc_ctx_get(TUYA_UVC_HANDLE handle)
{
    return (TUYA_UVC_CTX_T *)handle;
}

static OPERATE_RET tkl_uvc_ret_convert(avdk_err_t ret)
{
    switch (ret) {
    case AVDK_ERR_OK:
        return OPRT_OK;
    case AVDK_ERR_INVAL:
        return OPRT_INVALID_PARM;
    case AVDK_ERR_NOMEM:
        return OPRT_MALLOC_FAILED;
    case AVDK_ERR_UNSUPPORTED:
        return OPRT_NOT_SUPPORTED;
    case AVDK_ERR_TIMEOUT:
        return OPRT_TIMEOUT;
    default:
        return OPRT_COM_ERROR;
    }
}

static OPERATE_RET tkl_uvc_format_convert(UINT32_T codec, bk_image_format_t *format)
{
    if (format == NULL) {
        return OPRT_INVALID_PARM;
    }

    switch (codec) {
    case TUYA_VCODEC_TYPE_NONE:
    case TUYA_VCODEC_TYPE_MJPEG:
    case TUYA_VCODEC_TYPE_JPEG:
        *format = BK_IMAGE_FORMAT_MJPEG;
        return OPRT_OK;

    case TUYA_VCODEC_TYPE_H264:
        *format = BK_IMAGE_FORMAT_H264;
        return OPRT_OK;

    default:
        return OPRT_NOT_SUPPORTED;
    }
}

static UINT32_T tkl_uvc_codec_from_format(bk_image_format_t format)
{
    switch (format) {
    case BK_IMAGE_FORMAT_H264:
        return TUYA_VCODEC_TYPE_H264;
    case BK_IMAGE_FORMAT_MJPEG:
    default:
        return TUYA_VCODEC_TYPE_MJPEG;
    }
}

static TUYA_VCODEC_FRAME_TYPE_E tkl_uvc_frame_type_convert(UINT32_T h264_type)
{
    switch (h264_type) {
    case 0:
        return TUYA_VCODEC_FRAME_I;
    case 1:
        return TUYA_VCODEC_FRAME_P;
    case 2:
        return TUYA_VCODEC_FRAME_B;
    case 3:
        return TUYA_VCODEC_FRAME_HEADER;
    case 4:
        return TUYA_VCODEC_FRAME_END;
    default:
        return TUYA_VCODEC_FRAME_UNKNOWN;
    }
}

static OPERATE_RET tkl_uvc_cfg_validate(const TUYA_UVC_CFG_T *cfg)
{
    bk_image_format_t format;

    if (cfg == NULL || cfg->width == 0U || cfg->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    return tkl_uvc_format_convert(cfg->codec, &format);
}

static OPERATE_RET tkl_uvc_sink_validate(const TUYA_UVC_SINK_T *sink)
{
    if (sink == NULL || sink->stream_cb == NULL) {
        return OPRT_INVALID_PARM;
    }

    if ((sink->alloc_cb == NULL) != (sink->free_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static VOID tkl_uvc_cfg_resolve(TUYA_UVC_CFG_T *cfg)
{
    if (cfg->port == 0U) {
        cfg->port = TKL_UVC_DEFAULT_PORT;
    }

    if (cfg->fps == 0U) {
        cfg->fps = TKL_UVC_DEFAULT_FPS;
    }

    if (cfg->codec == TUYA_VCODEC_TYPE_NONE) {
        cfg->codec = TUYA_VCODEC_TYPE_MJPEG;
    }

    if (cfg->connect_timeout_ms == 0U) {
        cfg->connect_timeout_ms = TKL_UVC_DEFAULT_CONNECT_TIMEOUT_MS;
    }
}

static frame_buffer_t *tkl_uvc_frame_malloc(bk_image_format_t format, uint32_t size)
{
    TUYA_UVC_CTX_T *ctx = s_active_frame_ctx;
    TUYA_UVC_FRAME_T *node = NULL;
    VOID *payload = NULL;

    if (ctx == NULL || size == 0U) {
        return NULL;
    }

    node = (TUYA_UVC_FRAME_T *)tkl_system_calloc(1, sizeof(TUYA_UVC_FRAME_T));
    if (node == NULL) {
        return NULL;
    }

    if (ctx->sink.alloc_cb != NULL) {
        payload = ctx->sink.alloc_cb(size, ctx->sink.cb_arg);
    } else {
        payload = tkl_system_malloc(size);
    }

    if (payload == NULL) {
        tkl_system_free(node);
        return NULL;
    }

    node->payload = payload;
    node->owner = ctx;
    node->frame.fmt = format;
    node->frame.frame = (UINT8_T *)payload;
    node->frame.size = size;
    node->frame.width = ctx->cfg.width;
    node->frame.height = ctx->cfg.height;

    return &node->frame;
}

static VOID tkl_uvc_frame_free(TUYA_UVC_CTX_T *ctx, TUYA_UVC_FRAME_T *node)
{
    if (node == NULL) {
        return;
    }

    if (node->payload != NULL) {
        if (ctx != NULL && ctx->sink.free_cb != NULL) {
            (void)ctx->sink.free_cb(node->payload, ctx->sink.cb_arg);
        } else {
            tkl_system_free(node->payload);
        }
    }

    tkl_system_free(node);
}

static void tkl_uvc_frame_complete(uint8_t port,
                                   bk_image_format_t format,
                                   frame_buffer_t *frame,
                                   int result)
{
    TUYA_UVC_FRAME_T *node = (TUYA_UVC_FRAME_T *)frame;
    TUYA_UVC_CTX_T *ctx;

    (void)port;

    if (node == NULL) {
        return;
    }

    ctx = (TUYA_UVC_CTX_T *)node->owner;
    if (ctx != NULL && result == AVDK_ERR_OK && ctx->sink.stream_cb != NULL) {
        TUYA_VCODEC_STREAM_T stream;

        tkl_system_memset(&stream, 0, sizeof(stream));
        stream.data = frame->frame;
        stream.size = frame->length;
        stream.pts = frame->timestamp;
        stream.codec = tkl_uvc_codec_from_format(format);
        stream.frame_type = (format == BK_IMAGE_FORMAT_H264) ?
                            tkl_uvc_frame_type_convert(frame->h264_type) :
                            TUYA_VCODEC_FRAME_UNKNOWN;
        stream.priv = frame;

        ctx->sink.stream_cb(&stream, ctx->sink.cb_arg);
    }

    tkl_uvc_frame_free(ctx, node);
}

static void tkl_uvc_state_change(uvc_state_t state, void *user_data)
{
    TUYA_UVC_CTX_T *ctx = (TUYA_UVC_CTX_T *)user_data;

    if (ctx != NULL && ctx->sink.state_cb != NULL) {
        ctx->sink.state_cb((state == UVC_CONNECTED) ? TRUE : FALSE, ctx->sink.cb_arg);
    }
}

static const bk_uvc_callback_t s_tkl_uvc_callbacks = {
    .frame_malloc = tkl_uvc_frame_malloc,
    .frame_complete = tkl_uvc_frame_complete,
    .state_change_cb = tkl_uvc_state_change,
    .user_data = NULL,
};

static OPERATE_RET tkl_uvc_check_mjpeg_config(bk_uvc_device_brief_info_t *device,
                                              bk_cam_uvc_config_t *config)
{
    if (device == NULL || config == NULL) {
        return OPRT_INVALID_PARM;
    }

    for (uint8_t index = 0; index < device->all_frame.mjpeg_frame_num; index++) {
        if (device->all_frame.mjpeg_frame[index].width == config->width &&
            device->all_frame.mjpeg_frame[index].height == config->height) {
            for (uint8_t fps_index = 0; fps_index < device->all_frame.mjpeg_frame[index].fps_num; fps_index++) {
                if (device->all_frame.mjpeg_frame[index].fps[fps_index] == config->fps) {
                    return OPRT_OK;
                }
            }

            if (device->all_frame.mjpeg_frame[index].fps_num > 0U) {
                config->fps = device->all_frame.mjpeg_frame[index].fps[0];
                return OPRT_OK;
            }

            return OPRT_NOT_SUPPORTED;
        }
    }

    return OPRT_NOT_SUPPORTED;
}

static OPERATE_RET tkl_uvc_check_h264_config(bk_uvc_device_brief_info_t *device,
                                             bk_cam_uvc_config_t *config)
{
    if (device == NULL || config == NULL) {
        return OPRT_INVALID_PARM;
    }

    for (uint8_t index = 0; index < device->all_frame.h264_frame_num; index++) {
        if (device->all_frame.h264_frame[index].width == config->width &&
            device->all_frame.h264_frame[index].height == config->height) {
            for (uint8_t fps_index = 0; fps_index < device->all_frame.h264_frame[index].fps_num; fps_index++) {
                if (device->all_frame.h264_frame[index].fps[fps_index] == config->fps) {
                    return OPRT_OK;
                }
            }

            if (device->all_frame.h264_frame[index].fps_num > 0U) {
                config->fps = device->all_frame.h264_frame[index].fps[0];
                return OPRT_OK;
            }

            return OPRT_NOT_SUPPORTED;
        }
    }

    return OPRT_NOT_SUPPORTED;
}

static OPERATE_RET tkl_uvc_check_port_info(bk_cam_uvc_config_t *config)
{
    bk_usb_hub_port_info *port_info = NULL;
    bk_uvc_device_brief_info_t *device = NULL;
    avdk_err_t ret;

    if (config == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (config->format == BK_IMAGE_FORMAT_H264 || config->format == BK_IMAGE_FORMAT_H265) {
        ret = bk_usbh_hub_port_check_device(config->port, USB_UVC_H26X_DEVICE, &port_info);
    } else {
        ret = bk_usbh_hub_port_check_device(config->port, USB_UVC_DEVICE, &port_info);
    }

    if (ret != AVDK_ERR_OK) {
        rtos_delay_milliseconds(3000);
        ret = bk_usbh_hub_port_check_device(config->port, USB_UVC_DEVICE, &port_info);
    }

    if (ret != AVDK_ERR_OK || port_info == NULL) {
        return tkl_uvc_ret_convert(ret);
    }

    device = (bk_uvc_device_brief_info_t *)port_info->usb_device_param;
    if (device == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    switch (config->format) {
    case BK_IMAGE_FORMAT_MJPEG:
        return tkl_uvc_check_mjpeg_config(device, config);
    case BK_IMAGE_FORMAT_H264:
        return tkl_uvc_check_h264_config(device, config);
    default:
        return OPRT_NOT_SUPPORTED;
    }
}

static void tkl_uvc_connect_callback(bk_usb_hub_port_info *port_info, void *arg)
{
    TUYA_UVC_CTX_T *ctx = (TUYA_UVC_CTX_T *)arg;

    if (port_info == NULL || ctx == NULL) {
        return;
    }

    if (ctx->connect_sem != NULL) {
        rtos_set_semaphore(&ctx->connect_sem);
    }
}

static OPERATE_RET tkl_uvc_power_on(TUYA_UVC_CTX_T *ctx)
{
    bk_usb_hub_port_info *port_info = NULL;
    bk_usb_hub_port_info *h26x_port_info = NULL;
    avdk_err_t ret1;
    avdk_err_t ret2;
    avdk_err_t ret;
    uint8_t port;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    port = ctx->cfg.port;
    if (port == 0U || port > UVC_PORT_MAX) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->connect_sem == NULL) {
        ret = rtos_init_semaphore(&ctx->connect_sem, 1);
        if (ret != AVDK_ERR_OK) {
            return tkl_uvc_ret_convert(ret);
        }
    }

    rtos_get_semaphore(&ctx->connect_sem, BEKEN_NO_WAIT);

    bk_usbh_hub_port_register_connect_callback(port, USB_UVC_DEVICE, tkl_uvc_connect_callback, ctx);
    bk_usbh_hub_multiple_devices_power_on(USB_HOST_MODE, port, USB_UVC_DEVICE);
    bk_usbh_hub_port_register_connect_callback(port, USB_UVC_H26X_DEVICE, tkl_uvc_connect_callback, ctx);
    bk_usbh_hub_multiple_devices_power_on(USB_HOST_MODE, port, USB_UVC_H26X_DEVICE);

    ret1 = bk_usbh_hub_port_check_device(port, USB_UVC_DEVICE, &port_info);
    ret2 = bk_usbh_hub_port_check_device(port, USB_UVC_H26X_DEVICE, &h26x_port_info);
    if (ret1 == AVDK_ERR_OK || ret2 == AVDK_ERR_OK) {
        return OPRT_OK;
    }

    ret = rtos_get_semaphore(&ctx->connect_sem, ctx->cfg.connect_timeout_ms);
    return tkl_uvc_ret_convert(ret);
}

static VOID tkl_uvc_power_off(TUYA_UVC_CTX_T *ctx)
{
    uint8_t port;

    if (ctx == NULL) {
        return;
    }

    port = ctx->cfg.port;
    if (port != 0U && port <= UVC_PORT_MAX) {
        bk_usbh_hub_port_register_connect_callback(port, USB_UVC_DEVICE, NULL, NULL);
        bk_usbh_hub_multiple_devices_power_down(USB_HOST_MODE, port, USB_UVC_DEVICE);
        bk_usbh_hub_port_register_connect_callback(port, USB_UVC_H26X_DEVICE, NULL, NULL);
        bk_usbh_hub_multiple_devices_power_down(USB_HOST_MODE, port, USB_UVC_H26X_DEVICE);
    }

    if (ctx->connect_sem != NULL) {
        rtos_deinit_semaphore(&ctx->connect_sem);
        ctx->connect_sem = NULL;
    }
}

static OPERATE_RET tkl_uvc_config_fill(TUYA_UVC_CTX_T *ctx, bk_cam_uvc_config_t *config)
{
    bk_image_format_t format;
    OPERATE_RET op_ret;

    if (ctx == NULL || config == NULL) {
        return OPRT_INVALID_PARM;
    }

    op_ret = tkl_uvc_format_convert(ctx->cfg.codec, &format);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    tkl_system_memset(config, 0, sizeof(*config));
    config->format = format;
    config->width = ctx->cfg.width;
    config->height = ctx->cfg.height;
    config->fps = ctx->cfg.fps;
    config->port = ctx->cfg.port;
    config->drop_num = ctx->cfg.drop_num;

    return OPRT_OK;
}

static OPERATE_RET tkl_uvc_backend_prepare(TUYA_UVC_CTX_T *ctx)
{
    bk_cam_uvc_config_t config;
    avdk_err_t ret;
    OPERATE_RET op_ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    op_ret = tkl_uvc_config_fill(ctx, &config);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    op_ret = tkl_uvc_power_on(ctx);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    op_ret = tkl_uvc_check_port_info(&config);
    if (op_ret != OPRT_OK) {
        tkl_uvc_power_off(ctx);
        return op_ret;
    }

    ctx->callbacks = s_tkl_uvc_callbacks;
    ctx->callbacks.user_data = ctx;

    ret = bk_uvc_ctrl_new(&ctx->backend_handle, &ctx->callbacks);
    if (ret != AVDK_ERR_OK) {
        tkl_uvc_power_off(ctx);
        return tkl_uvc_ret_convert(ret);
    }

    ret = bk_uvc_init(ctx->backend_handle);
    if (ret != AVDK_ERR_OK) {
        (void)bk_uvc_delete(ctx->backend_handle);
        ctx->backend_handle = NULL;
        tkl_uvc_power_off(ctx);
        return tkl_uvc_ret_convert(ret);
    }
    ctx->backend_inited = TRUE;
    ctx->cfg.fps = (UINT8_T)config.fps;
    return OPRT_OK;
}

static OPERATE_RET tkl_uvc_backend_open(TUYA_UVC_CTX_T *ctx)
{
    bk_cam_uvc_config_t config;
    avdk_err_t ret;
    OPERATE_RET op_ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->backend_handle == NULL || ctx->backend_inited != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    op_ret = tkl_uvc_config_fill(ctx, &config);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    ret = bk_uvc_open(ctx->backend_handle, &config);
    return tkl_uvc_ret_convert(ret);
}

static OPERATE_RET tkl_uvc_backend_close(TUYA_UVC_CTX_T *ctx)
{
    avdk_err_t ret;

    if (ctx == NULL || ctx->backend_handle == NULL) {
        return OPRT_OK;
    }

    ret = bk_uvc_close(ctx->backend_handle);
    return tkl_uvc_ret_convert(ret);
}

static OPERATE_RET tkl_uvc_backend_release(TUYA_UVC_CTX_T *ctx)
{
    OPERATE_RET op_ret = OPRT_OK;
    avdk_err_t ret;

    if (ctx == NULL) {
        return OPRT_OK;
    }

    if (ctx->backend_handle == NULL) {
        tkl_uvc_power_off(ctx);
        return OPRT_OK;
    }

    if (ctx->backend_inited == TRUE) {
        ret = bk_uvc_deinit(ctx->backend_handle);
        if (ret != AVDK_ERR_OK && op_ret == OPRT_OK) {
            op_ret = tkl_uvc_ret_convert(ret);
        }
        ctx->backend_inited = FALSE;
    }

    ret = bk_uvc_delete(ctx->backend_handle);
    if (ret != AVDK_ERR_OK && op_ret == OPRT_OK) {
        op_ret = tkl_uvc_ret_convert(ret);
    }
    ctx->backend_handle = NULL;

    tkl_uvc_power_off(ctx);
    return op_ret;
}

#endif

OPERATE_RET tkl_uvc_init(TUYA_UVC_HANDLE *handle, const TUYA_UVC_CFG_T *config)
{
#if CONFIG_USB_CAMERA
    TUYA_UVC_CTX_T *ctx = NULL;
    OPERATE_RET ret;

    if (handle == NULL || config == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (*handle != NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_uvc_cfg_validate(config);
    if (ret != OPRT_OK) {
        return ret;
    }

    ctx = (TUYA_UVC_CTX_T *)tkl_system_malloc(sizeof(TUYA_UVC_CTX_T));
    if (ctx == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    tkl_system_memset(ctx, 0, sizeof(TUYA_UVC_CTX_T));
    tkl_system_memcpy(&ctx->cfg, config, sizeof(TUYA_UVC_CFG_T));
    tkl_uvc_cfg_resolve(&ctx->cfg);

    ret = tkl_uvc_backend_prepare(ctx);
    if (ret != OPRT_OK) {
        (void)tkl_uvc_backend_release(ctx);
        tkl_system_free(ctx);
        return ret;
    }

    *handle = (TUYA_UVC_HANDLE)ctx;
    return OPRT_OK;
#else
    (void)handle;
    (void)config;
    return OPRT_NOT_SUPPORTED;
#endif
}

OPERATE_RET tkl_uvc_deinit(TUYA_UVC_HANDLE handle)
{
#if CONFIG_USB_CAMERA
    TUYA_UVC_CTX_T *ctx = tkl_uvc_ctx_get(handle);
    OPERATE_RET ret;
    OPERATE_RET release_ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_uvc_stop(handle);
    release_ret = tkl_uvc_backend_release(ctx);

    tkl_system_free(ctx);
    return (ret != OPRT_OK) ? ret : release_ret;
#else
    (void)handle;
    return OPRT_NOT_SUPPORTED;
#endif
}

OPERATE_RET tkl_uvc_set_sink(TUYA_UVC_HANDLE handle, const TUYA_UVC_SINK_T *sink)
{
#if CONFIG_USB_CAMERA
    TUYA_UVC_CTX_T *ctx = tkl_uvc_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == TRUE) {
        return OPRT_COM_ERROR;
    }

    ret = tkl_uvc_sink_validate(sink);
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_system_memcpy(&ctx->sink, sink, sizeof(TUYA_UVC_SINK_T));
    return OPRT_OK;
#else
    (void)handle;
    (void)sink;
    return OPRT_NOT_SUPPORTED;
#endif
}

OPERATE_RET tkl_uvc_start(TUYA_UVC_HANDLE handle)
{
#if CONFIG_USB_CAMERA
    TUYA_UVC_CTX_T *ctx = tkl_uvc_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == TRUE) {
        return OPRT_OK;
    }

    ret = tkl_uvc_sink_validate(&ctx->sink);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (s_active_frame_ctx != NULL && s_active_frame_ctx != ctx) {
        return OPRT_COM_ERROR;
    }
    s_active_frame_ctx = ctx;

    ret = tkl_uvc_backend_open(ctx);
    if (ret != OPRT_OK) {
        if (s_active_frame_ctx == ctx) {
            s_active_frame_ctx = NULL;
        }
        return ret;
    }

    ctx->started = TRUE;
    return OPRT_OK;
#else
    (void)handle;
    return OPRT_NOT_SUPPORTED;
#endif
}

OPERATE_RET tkl_uvc_stop(TUYA_UVC_HANDLE handle)
{
#if CONFIG_USB_CAMERA
    TUYA_UVC_CTX_T *ctx = tkl_uvc_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == FALSE) {
        return OPRT_OK;
    }

    ret = tkl_uvc_backend_close(ctx);
    ctx->started = FALSE;

    if (s_active_frame_ctx == ctx) {
        s_active_frame_ctx = NULL;
    }

    return ret;
#else
    (void)handle;
    return OPRT_NOT_SUPPORTED;
#endif
}

OPERATE_RET tkl_uvc_get_handle(TUYA_UVC_HANDLE handle, VOID **backend_handle)
{
#if CONFIG_USB_CAMERA
    TUYA_UVC_CTX_T *ctx = tkl_uvc_ctx_get(handle);

    if (ctx == NULL || backend_handle == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->backend_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    *backend_handle = (VOID *)ctx->backend_handle;
    return OPRT_OK;
#else
    (void)handle;
    (void)backend_handle;
    return OPRT_NOT_SUPPORTED;
#endif
}
