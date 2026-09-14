/**
 * @file tkl_uvc.c
 * @brief TKL adapter - the USB camera peripheral for T9 (BK7259).
 *
 * @details
 * Two layers, split internally only; the public face is
 * include/uvc/tkl_uvc.h.
 *
 *   device layer (static)  USB enumeration, the vendor uvc backend, per-frame
 *                          callbacks
 *   facade (public)        init / start / stop / deinit / set_osd / set_encode
 *
 * The facade exists for buffer ownership and for the decode task: an adapter
 * thread that pops the capture queue into the decoder, which is what feeds the
 * PIXEL/STREAM outputs. The device layer and tkl_vdec both **lend** their
 * buffer for one callback and reclaim it on return, which would force every
 * caller to memcpy the frame out. Here the buffers come from pools this file
 * owns and the reclaim hook publishes to the ready queue instead, so the
 * borrow becomes a loan business ends by calling buf.free_cb.
 *
 * @version 0.2
 * @date 2026-08-29
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tkl_gpu_internal.h"
#include "tkl_media_buf.h"
#include "tkl_media_fmt.h"
#include "tkl_media_types.h"
#include "tkl_memory.h"
#include "tkl_osd_engine.h"
#include "tkl_output.h"
#include "tkl_system.h"
#include "tkl_uvc.h"
#include "tkl_vcodec_types.h"
#include "tkl_vdec_internal.h"
#include "tkl_venc_internal.h"
#include "tuya_cloud_types.h"

#if CONFIG_USB_CAMERA

#include <common/avdk_pixel_types.h>
#include <components/avdk_utils/avdk_error.h>
#include <components/bk_uvc_camera.h>
#include <components/usb_types.h>
#include <components/usbh_hub_multiple_classes_api.h>
#include <os/mem.h>
#include <os/os.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Device-layer types - internal only.
 * ------------------------------------------------------------------------- */

typedef VOID *TUYA_UVC_HANDLE;

typedef VOID (*TUYA_UVC_STREAM_CB)(const TUYA_VCODEC_STREAM_T *stream, VOID *arg);

typedef struct {
    UINT8_T port;                   /* USB hub port, default 1 */
    UINT16_T width;
    UINT16_T height;
    UINT8_T fps;                    /* default 30 */
    UINT32_T input_format;          /* TUYA_VCODEC_TYPE_MJPEG/H264, default MJPEG */
} TKL_UVC_CFG_T;

/**
 * @brief Where the device's frames go, and optionally who owns their memory.
 *
 * @note alloc_cb/free_cb are **optional and all-or-nothing**. Both NULL is the
 *       normal case: TKL picks the pool, and the buffer must not be freed or
 *       kept past the callback. Install both when the buffer has to outlive the
 *       callback or come from a pool of yours. Setting just one is rejected.
 */
typedef struct {
    TUYA_UVC_STREAM_CB stream_cb;
    TUYA_UVC_STATE_CB  state_cb;
    TUYA_VCODEC_ALLOC_CB alloc_cb;
    TUYA_VCODEC_FREE_CB free_cb;
    VOID *cb_arg;
} TUYA_UVC_SINK_T;

/* ---------------------------------------------------------------------------
 * Device layer - internal only. The facade below is the only caller.
 * ------------------------------------------------------------------------- */
STATIC OPERATE_RET uvc_dev_init(TUYA_UVC_HANDLE *handle, const TKL_UVC_CFG_T *config);
STATIC OPERATE_RET uvc_dev_deinit(TUYA_UVC_HANDLE handle);
STATIC OPERATE_RET uvc_dev_set_sink(TUYA_UVC_HANDLE handle, const TUYA_UVC_SINK_T *sink);
STATIC OPERATE_RET uvc_dev_start(TUYA_UVC_HANDLE handle);
STATIC OPERATE_RET uvc_dev_stop(TUYA_UVC_HANDLE handle);

#include <common/avdk_pixel_types.h>
#include <components/avdk_utils/avdk_error.h>
#include <components/bk_uvc_camera.h>
#include <components/usb_types.h>
#include <components/usbh_hub_multiple_classes_api.h>
#include <os/os.h>

#define TUYA_UVC_DEFAULT_PORT               1U
#define TUYA_UVC_DEFAULT_FPS                30U
#define TUYA_UVC_DEFAULT_CONNECT_TIMEOUT_MS 4000U

typedef struct {
    frame_buffer_t frame;
    VOID *payload;
    VOID *owner;
} TUYA_UVC_FRAME_T;

typedef struct {
    BOOL_T started;
    BOOL_T backend_inited;
    TKL_UVC_CFG_T cfg;
    TUYA_UVC_SINK_T sink;
    bk_uvc_callback_t callbacks;
    bk_uvc_ctlr_handle_t backend_handle;
    beken_semaphore_t connect_sem;
} TUYA_UVC_CTX_T;

static TUYA_UVC_CTX_T *s_active_frame_ctx = NULL;

static TUYA_UVC_CTX_T *uvc_dev_ctx_get(TUYA_UVC_HANDLE handle)
{
    return (TUYA_UVC_CTX_T *)handle;
}

static OPERATE_RET uvc_dev_ret_convert(avdk_err_t ret)
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

static OPERATE_RET uvc_dev_format_convert(UINT32_T codec, bk_image_format_t *format)
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

static UINT32_T uvc_dev_codec_from_format(bk_image_format_t format)
{
    switch (format) {
    case BK_IMAGE_FORMAT_H264:
        return TUYA_VCODEC_TYPE_H264;
    case BK_IMAGE_FORMAT_MJPEG:
    default:
        return TUYA_VCODEC_TYPE_MJPEG;
    }
}

static TUYA_VCODEC_FRAME_TYPE_E uvc_dev_frame_type_convert(UINT32_T h264_type)
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

static OPERATE_RET uvc_dev_cfg_validate(const TKL_UVC_CFG_T *cfg)
{
    bk_image_format_t format;

    if (cfg == NULL || cfg->width == 0U || cfg->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    return uvc_dev_format_convert(cfg->input_format, &format);
}

static OPERATE_RET uvc_dev_sink_validate(const TUYA_UVC_SINK_T *sink)
{
    if (sink == NULL || sink->stream_cb == NULL) {
        return OPRT_INVALID_PARM;
    }

    if ((sink->alloc_cb == NULL) != (sink->free_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static VOID uvc_dev_cfg_resolve(TKL_UVC_CFG_T *cfg)
{
    if (cfg->port == 0U) {
        cfg->port = TUYA_UVC_DEFAULT_PORT;
    }

    if (cfg->fps == 0U) {
        cfg->fps = TUYA_UVC_DEFAULT_FPS;
    }

    if (cfg->input_format == TUYA_VCODEC_TYPE_NONE) {
        cfg->input_format = TUYA_VCODEC_TYPE_MJPEG;
    }
}

static frame_buffer_t *uvc_dev_frame_malloc(bk_image_format_t format, uint32_t size)
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
        /* No sink allocator: TKL picks the pool. USB DMAs an encoded frame
         * straight into it, so it belongs in the stream slab, not the heap. */
        payload = tkl_media_buf_alloc(TKL_BUF_USE_STREAM, size,
                                      TUYA_MEDIA_BUF_OWNER_UVC_STREAM);
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

static VOID uvc_dev_frame_free(TUYA_UVC_CTX_T *ctx, TUYA_UVC_FRAME_T *node)
{
    if (node == NULL) {
        return;
    }

    if (node->payload != NULL) {
        if (ctx != NULL && ctx->sink.free_cb != NULL) {
            (void)ctx->sink.free_cb(node->payload, ctx->sink.cb_arg);
        } else {
            tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_UVC_STREAM, node->payload);
        }
    }

    tkl_system_free(node);
}

static void uvc_dev_frame_complete(uint8_t port,
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
        stream.codec = uvc_dev_codec_from_format(format);
        stream.frame_type = (format == BK_IMAGE_FORMAT_H264) ?
                            uvc_dev_frame_type_convert(frame->h264_type) :
                            TUYA_VCODEC_FRAME_UNKNOWN;
        stream.priv = frame;

        ctx->sink.stream_cb(&stream, ctx->sink.cb_arg);
    }

    uvc_dev_frame_free(ctx, node);
}

static void uvc_dev_state_change(uvc_state_t state, void *user_data)
{
    TUYA_UVC_CTX_T *ctx = (TUYA_UVC_CTX_T *)user_data;

    if (ctx != NULL && ctx->sink.state_cb != NULL) {
        ctx->sink.state_cb((state == UVC_CONNECTED) ? TRUE : FALSE, ctx->sink.cb_arg);
    }
}

static const bk_uvc_callback_t s_tkl_uvc_callbacks = {
    .frame_malloc = uvc_dev_frame_malloc,
    .frame_complete = uvc_dev_frame_complete,
    .state_change_cb = uvc_dev_state_change,
    .user_data = NULL,
};

static OPERATE_RET uvc_dev_check_mjpeg_config(bk_uvc_device_brief_info_t *device,
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

static OPERATE_RET uvc_dev_check_h264_config(bk_uvc_device_brief_info_t *device,
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

static OPERATE_RET uvc_dev_check_port_info(bk_cam_uvc_config_t *config)
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
        return uvc_dev_ret_convert(ret);
    }

    device = (bk_uvc_device_brief_info_t *)port_info->usb_device_param;
    if (device == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    switch (config->format) {
    case BK_IMAGE_FORMAT_MJPEG:
        return uvc_dev_check_mjpeg_config(device, config);
    case BK_IMAGE_FORMAT_H264:
        return uvc_dev_check_h264_config(device, config);
    default:
        return OPRT_NOT_SUPPORTED;
    }
}

static void uvc_dev_connect_callback(bk_usb_hub_port_info *port_info, void *arg)
{
    TUYA_UVC_CTX_T *ctx = (TUYA_UVC_CTX_T *)arg;

    if (port_info == NULL || ctx == NULL) {
        return;
    }

    if (ctx->connect_sem != NULL) {
        rtos_set_semaphore(&ctx->connect_sem);
    }
}

static OPERATE_RET uvc_dev_power_on(TUYA_UVC_CTX_T *ctx)
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
            return uvc_dev_ret_convert(ret);
        }
    }

    rtos_get_semaphore(&ctx->connect_sem, BEKEN_NO_WAIT);

    bk_usbh_hub_port_register_connect_callback(port, USB_UVC_DEVICE, uvc_dev_connect_callback, ctx);
    bk_usbh_hub_multiple_devices_power_on(USB_HOST_MODE, port, USB_UVC_DEVICE);
    bk_usbh_hub_port_register_connect_callback(port, USB_UVC_H26X_DEVICE, uvc_dev_connect_callback, ctx);
    bk_usbh_hub_multiple_devices_power_on(USB_HOST_MODE, port, USB_UVC_H26X_DEVICE);

    ret1 = bk_usbh_hub_port_check_device(port, USB_UVC_DEVICE, &port_info);
    ret2 = bk_usbh_hub_port_check_device(port, USB_UVC_H26X_DEVICE, &h26x_port_info);
    if (ret1 == AVDK_ERR_OK || ret2 == AVDK_ERR_OK) {
        return OPRT_OK;
    }

    ret = rtos_get_semaphore(&ctx->connect_sem, TUYA_UVC_DEFAULT_CONNECT_TIMEOUT_MS);
    return uvc_dev_ret_convert(ret);
}

static VOID uvc_dev_power_off(TUYA_UVC_CTX_T *ctx)
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

static OPERATE_RET uvc_dev_config_fill(TUYA_UVC_CTX_T *ctx, bk_cam_uvc_config_t *config)
{
    bk_image_format_t format;
    OPERATE_RET op_ret;

    if (ctx == NULL || config == NULL) {
        return OPRT_INVALID_PARM;
    }

    op_ret = uvc_dev_format_convert(ctx->cfg.input_format, &format);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    tkl_system_memset(config, 0, sizeof(*config));
    config->format = format;
    config->width = ctx->cfg.width;
    config->height = ctx->cfg.height;
    config->fps = ctx->cfg.fps;
    config->port = ctx->cfg.port;

    return OPRT_OK;
}

static OPERATE_RET uvc_dev_backend_set_skip_frames(TUYA_UVC_CTX_T *ctx)
{
    (void)ctx;
    /* AE warm-up is a backend policy, not part of the public UVC contract. */
    return OPRT_OK;
}

static OPERATE_RET uvc_dev_backend_prepare(TUYA_UVC_CTX_T *ctx)
{
    bk_cam_uvc_config_t config;
    avdk_err_t ret;
    OPERATE_RET op_ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    op_ret = uvc_dev_config_fill(ctx, &config);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    op_ret = uvc_dev_power_on(ctx);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    op_ret = uvc_dev_check_port_info(&config);
    if (op_ret != OPRT_OK) {
        uvc_dev_power_off(ctx);
        return op_ret;
    }

    ctx->callbacks = s_tkl_uvc_callbacks;
    ctx->callbacks.user_data = ctx;

    ret = bk_uvc_ctrl_new(&ctx->backend_handle, &ctx->callbacks);
    if (ret != AVDK_ERR_OK) {
        uvc_dev_power_off(ctx);
        return uvc_dev_ret_convert(ret);
    }

    ret = bk_uvc_init(ctx->backend_handle);
    if (ret != AVDK_ERR_OK) {
        (void)bk_uvc_delete(ctx->backend_handle);
        ctx->backend_handle = NULL;
        uvc_dev_power_off(ctx);
        return uvc_dev_ret_convert(ret);
    }
    ctx->backend_inited = TRUE;

    op_ret = uvc_dev_backend_set_skip_frames(ctx);
    if (op_ret != OPRT_OK) {
        (void)bk_uvc_deinit(ctx->backend_handle);
        ctx->backend_inited = FALSE;
        (void)bk_uvc_delete(ctx->backend_handle);
        ctx->backend_handle = NULL;
        uvc_dev_power_off(ctx);
        return op_ret;
    }

    ctx->cfg.fps = (UINT8_T)config.fps;
    return OPRT_OK;
}

static OPERATE_RET uvc_dev_backend_open(TUYA_UVC_CTX_T *ctx)
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

    op_ret = uvc_dev_config_fill(ctx, &config);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    ret = bk_uvc_open(ctx->backend_handle, &config);
    return uvc_dev_ret_convert(ret);
}

static OPERATE_RET uvc_dev_backend_close(TUYA_UVC_CTX_T *ctx)
{
    avdk_err_t ret;

    if (ctx == NULL || ctx->backend_handle == NULL) {
        return OPRT_OK;
    }

    ret = bk_uvc_close(ctx->backend_handle);
    return uvc_dev_ret_convert(ret);
}

static OPERATE_RET uvc_dev_backend_release(TUYA_UVC_CTX_T *ctx)
{
    OPERATE_RET op_ret = OPRT_OK;
    avdk_err_t ret;

    if (ctx == NULL) {
        return OPRT_OK;
    }

    if (ctx->backend_handle == NULL) {
        uvc_dev_power_off(ctx);
        return OPRT_OK;
    }

    if (ctx->backend_inited == TRUE) {
        ret = bk_uvc_deinit(ctx->backend_handle);
        if (ret != AVDK_ERR_OK && op_ret == OPRT_OK) {
            op_ret = uvc_dev_ret_convert(ret);
        }
        ctx->backend_inited = FALSE;
    }

    ret = bk_uvc_delete(ctx->backend_handle);
    if (ret != AVDK_ERR_OK && op_ret == OPRT_OK) {
        op_ret = uvc_dev_ret_convert(ret);
    }
    ctx->backend_handle = NULL;

    uvc_dev_power_off(ctx);
    return op_ret;
}

STATIC OPERATE_RET uvc_dev_init(TUYA_UVC_HANDLE *handle, const TKL_UVC_CFG_T *config)
{
    TUYA_UVC_CTX_T *ctx = NULL;
    OPERATE_RET ret;

    if (handle == NULL || config == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (*handle != NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = uvc_dev_cfg_validate(config);
    if (ret != OPRT_OK) {
        return ret;
    }

    ctx = (TUYA_UVC_CTX_T *)tkl_system_malloc(sizeof(TUYA_UVC_CTX_T));
    if (ctx == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    tkl_system_memset(ctx, 0, sizeof(TUYA_UVC_CTX_T));
    tkl_system_memcpy(&ctx->cfg, config, sizeof(TKL_UVC_CFG_T));
    uvc_dev_cfg_resolve(&ctx->cfg);

    ret = uvc_dev_backend_prepare(ctx);
    if (ret != OPRT_OK) {
        (void)uvc_dev_backend_release(ctx);
        tkl_system_free(ctx);
        return ret;
    }

    *handle = (TUYA_UVC_HANDLE)ctx;
    return OPRT_OK;
}

STATIC OPERATE_RET uvc_dev_deinit(TUYA_UVC_HANDLE handle)
{
    TUYA_UVC_CTX_T *ctx = uvc_dev_ctx_get(handle);
    OPERATE_RET ret;
    OPERATE_RET release_ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = uvc_dev_stop(handle);
    release_ret = uvc_dev_backend_release(ctx);

    tkl_system_free(ctx);
    return (ret != OPRT_OK) ? ret : release_ret;
}

STATIC OPERATE_RET uvc_dev_set_sink(TUYA_UVC_HANDLE handle, const TUYA_UVC_SINK_T *sink)
{
    TUYA_UVC_CTX_T *ctx = uvc_dev_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == TRUE) {
        return OPRT_COM_ERROR;
    }

    ret = uvc_dev_sink_validate(sink);
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_system_memcpy(&ctx->sink, sink, sizeof(TUYA_UVC_SINK_T));
    return OPRT_OK;
}

STATIC OPERATE_RET uvc_dev_start(TUYA_UVC_HANDLE handle)
{
    TUYA_UVC_CTX_T *ctx = uvc_dev_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == TRUE) {
        return OPRT_OK;
    }

    ret = uvc_dev_sink_validate(&ctx->sink);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (s_active_frame_ctx != NULL && s_active_frame_ctx != ctx) {
        return OPRT_COM_ERROR;
    }
    s_active_frame_ctx = ctx;

    ret = uvc_dev_backend_open(ctx);
    if (ret != OPRT_OK) {
        if (s_active_frame_ctx == ctx) {
            s_active_frame_ctx = NULL;
        }
        return ret;
    }

    ctx->started = TRUE;
    return OPRT_OK;
}

STATIC OPERATE_RET uvc_dev_stop(TUYA_UVC_HANDLE handle)
{
    TUYA_UVC_CTX_T *ctx = uvc_dev_ctx_get(handle);
    OPERATE_RET ret;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->started == FALSE) {
        return OPRT_OK;
    }

    ret = uvc_dev_backend_close(ctx);
    ctx->started = FALSE;

    if (s_active_frame_ctx == ctx) {
        s_active_frame_ctx = NULL;
    }

    return ret;
}




/* The vendor driver always asks frame_malloc() for the full frame size whatever
 * resolution was negotiated (bk_uvc_stream.c:988), so a slot has to be that big
 * or every allocation is refused. Same expression as the vendor's own pipeline
 * demo, uvc_pipeline.c:37. */
#ifdef CONFIG_UVC_FRAME_SIZE
#define TUYA_UVC_INTERNAL_SLOT_SIZE (CONFIG_UVC_FRAME_SIZE)
#else
#define TUYA_UVC_INTERNAL_SLOT_SIZE (1024U * 200U)
#endif

#define TUYA_UVC_INTERNAL_POOL_DEPTH_DEFAULT 5U

/* Slack in the ring the decoder writes slices into. */
#define TUYA_UVC_INTERNAL_FLEXA_DEPTH      3U

/* Sizes the scratch array in tkl_uvc_pool_free_count(), which must hold the
 * deepest pool this file creates. */
#define TUYA_UVC_INTERNAL_POOL_DEPTH_MAX   TUYA_UVC_INTERNAL_POOL_DEPTH_DEFAULT
/* The decoder emits in slices of this many lines; tkl_vdec rounds anything
 * smaller up to it, and every vendor pipeline uses exactly 16. */
#define TUYA_UVC_INTERNAL_FLEXA_LINES      16U

#define TUYA_UVC_INTERNAL_DRAIN_MS         500U
#define TUYA_UVC_INTERNAL_DRAIN_STEP_MS    10U
/* the picture the panel is still scanning out when the pipeline stops - same
 * reasoning as tkl_mipi_csi_deinit() */
#define TUYA_UVC_OUTPUT_PIXEL_RESIDUAL 1U

/* How often the internal decode task re-checks its stop flag while the ready
 * queue is idle. That idle poll is the join bound of stop(). */
#define TUYA_UVC_INTERNAL_TASK_POLL_MS     100U
#define TUYA_UVC_INTERNAL_TASK_STACK       (1024U * 4U)

#define TUYA_UVC_OUTPUT_KIND_MASK     (TUYA_UVC_OUTPUT_FRAME | TUYA_UVC_OUTPUT_PIXEL | \
                                      TUYA_UVC_OUTPUT_STREAM)
#define TUYA_UVC_OUTPUT_PIPELINE_MASK (TUYA_UVC_OUTPUT_PIXEL | TUYA_UVC_OUTPUT_STREAM)

#define TUYA_UVC_INTERNAL_ALIGN16(v)       (((v) + 15U) & ~15U)

#define TUYA_UVC_INTERNAL_MS()             ((UINT32_T)tkl_system_get_millisecond())

/** One pool slot. Allocated once at open, recycled for the life of the stream. */
typedef struct {
    UINT8_T *data;
    UINT32_T capacity;

    /* filled in by the producer, read by whoever pops it */
    UINT32_T len;
    UINT64_T pts;
    TUYA_FRAME_FMT_E fmt;   /**< TUYA_FRAME_FMT_JPEG or TUYA_FRAME_FMT_H264 */
    TUYA_VIDEO_FRAME_TYPE_E frame_type;

    /** Set between the two halves of one vendor callback pair and read only by
     *  the second half, on the same thread. See tkl_uvc_out_free(). */
    BOOL_T ready;
} tkl_uvc_slot_t;

typedef struct {
    tkl_uvc_slot_t *slots;
    UINT8_T   depth;
    UINT32_T  slot_size;
    beken_queue_t free_q;
    beken_queue_t ready_q;      /**< frames the decode task has not taken yet */
} tkl_uvc_pool_t;

/**
 * Facade lifecycle. INIT is the point of the split: the device is enumerated
 * and the capture pool built, but nothing is streaming - so start() after
 * stop() costs about 180 ms against 720 ms for a fresh open.
 */
typedef enum {
    TUYA_UVC_ST_IDLE = 0,    /**< nothing built */
    TUYA_UVC_ST_INIT,        /**< enumerated and pooled, not streaming */
    TUYA_UVC_ST_STARTED,     /**< streaming */
} tkl_uvc_state_e;

typedef struct {
    tkl_uvc_state_e state;
    TUYA_UVC_CFG_T cfg;

    TUYA_UVC_HANDLE uvc;
    TUYA_VDEC_HANDLE vdec;      /**< every mode but TUYA_UVC_OUTPUT_FRAME */

    /* PIXEL and STREAM outputs use the internal decoder pipeline.
     * The public API does not expose how they are delivered internally. */
    BOOL_T want_display;
    BOOL_T want_encode;
    TKL_GPU_HANDLE gpu;
    TUYA_VENC_HANDLE venc;
    TKL_OSD_ENGINE_HANDLE osd_display;
    UINT8_T *flexa_ring;        /**< decoder ring the bonds read from */
    UINT32_T flexa_size;
    UINT16_T src_width;         /**< decoder output feeding the bonds */
    UINT16_T src_height;

    tkl_uvc_pool_t stream_pool;

    /* The internal decode task: pops the ready queue and feeds the decoder,
     * which is what PIXEL/STREAM outputs run on. Owned by the adapter, so
     * business never has to pump - see tkl_uvc_out_decode_task(). */
    beken_thread_t    task;
    beken_semaphore_t task_done;    /**< posted once, when the task exits */
    BOOL_T            task_live;
    volatile BOOL_T   task_stop;

    /* diagnostics, reported by tkl_uvc_deinit() */
    UINT32_T frames;
    UINT32_T drop_stream_empty;
    UINT32_T drop_too_big;
    UINT32_T decode_fail;
    UINT32_T decoded;

} tkl_uvc_out_ctx_t;

STATIC tkl_uvc_out_ctx_t s_uvc_out;

STATIC OPERATE_RET tkl_uvc_out_task_start(VOID_T);
STATIC VOID tkl_uvc_out_decode_task(beken_thread_arg_t arg);

/* ---------------------------------------------------------------------------
 * Pool
 *
 * rtos_*_queue rather than tkl_queue_*: the latter rounds a zero timeout up to
 * one tick, and tkl_uvc_out_alloc() runs on the vendor's URB thread where
 * blocking for a tick stalls capture.
 * ------------------------------------------------------------------------- */

/**
 * @brief Allocate a pool's slots and the queues that track them.
 *
 * @param[out] pool        pool to build
 * @param[in]  depth       slot count
 * @param[in]  slot_size   bytes per slot
 * @param[in]  use         which slab the slots come from
 * @param[in]  owner       accounting bucket for the slabs
 * @param[in]  with_ready  also create the ready queue
 * @param[in]  name        queue name prefix, for the RTOS view
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_uvc_pool_create(tkl_uvc_pool_t *pool,
                                       UINT8_T depth,
                                       UINT32_T slot_size,
                                       TKL_BUF_USE_E use,
                                       TUYA_MEDIA_BUF_OWNER_E owner,
                                       BOOL_T with_ready,
                                       CONST CHAR_T *name)
{
    UINT8_T i;

    os_memset(pool, 0, sizeof(*pool));

    pool->slots = (tkl_uvc_slot_t *)tkl_system_malloc(sizeof(tkl_uvc_slot_t) * depth);
    if (pool->slots == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    os_memset(pool->slots, 0, sizeof(tkl_uvc_slot_t) * depth);
    pool->depth = depth;
    pool->slot_size = slot_size;

    if (rtos_init_queue(&pool->free_q, name, sizeof(VOID *), depth) != 0) {
        tkl_log_output("tkl_uvc_out: %s free queue failed\r\n", name);
        return OPRT_COM_ERROR;
    }

    if (with_ready == TRUE) {
        if (rtos_init_queue(&pool->ready_q, name, sizeof(VOID *), depth) != 0) {
            tkl_log_output("tkl_uvc_out: %s ready queue failed\r\n", name);
            return OPRT_COM_ERROR;
        }
    }

    for (i = 0U; i < depth; i++) {
        tkl_uvc_slot_t *slot = &pool->slots[i];

        slot->data = (UINT8_T *)tkl_media_buf_alloc(use, slot_size, owner);
        if (slot->data == NULL) {
            tkl_log_output("tkl_uvc_out: %s slot %u (%u bytes) failed\r\n",
                           name, (unsigned)i, (unsigned)slot_size);
            return OPRT_MALLOC_FAILED;
        }
        slot->capacity = slot_size;

        if (rtos_push_to_queue(&pool->free_q, &slot, BEKEN_NO_WAIT) != 0) {
            return OPRT_COM_ERROR;
        }
    }

    return OPRT_OK;
}

/**
 * @brief Release a pool.
 *
 * @param[in] pool  pool to release; safe on a partially built one
 * @param[in] owner accounting bucket the slabs were charged to
 *
 * @warning Frees the slot memory. The caller must have established that nobody
 *          still holds a slot - see the drain in tkl_uvc_deinit().
 */
STATIC VOID_T tkl_uvc_pool_destroy(tkl_uvc_pool_t *pool, TUYA_MEDIA_BUF_OWNER_E owner)
{
    UINT8_T i;

    if (pool->slots != NULL) {
        for (i = 0U; i < pool->depth; i++) {
            if (pool->slots[i].data != NULL) {
                tkl_media_buf_free(owner, pool->slots[i].data);
            }
        }
        tkl_system_free(pool->slots);
    }

    if (pool->free_q != NULL) {
        rtos_deinit_queue(&pool->free_q);
    }
    if (pool->ready_q != NULL) {
        rtos_deinit_queue(&pool->ready_q);
    }

    os_memset(pool, 0, sizeof(*pool));
}

/**
 * @brief Take a free slot, without blocking.
 * @param[in] pool pool to take from
 * @return slot, or NULL when the pool is empty
 */
STATIC tkl_uvc_slot_t *tkl_uvc_pool_take(tkl_uvc_pool_t *pool)
{
    tkl_uvc_slot_t *slot = NULL;

    if (pool->free_q == NULL) {
        return NULL;
    }

    if (rtos_pop_from_queue(&pool->free_q, &slot, BEKEN_NO_WAIT) != 0) {
        return NULL;
    }

    slot->ready = FALSE;
    slot->len = 0U;
    return slot;
}

/**
 * @brief Return a slot to the pool.
 * @param[in] pool pool it came from
 * @param[in] slot slot to return; NULL is ignored
 */
STATIC VOID_T tkl_uvc_pool_give(tkl_uvc_pool_t *pool, tkl_uvc_slot_t *slot)
{
    if (slot == NULL || pool->free_q == NULL) {
        return;
    }

    slot->ready = FALSE;
    (void)rtos_push_to_queue(&pool->free_q, &slot, BEKEN_NO_WAIT);
}

/**
 * @brief Find the slot backing a payload pointer.
 *
 * @param[in] pool pool to search
 * @param[in] data payload pointer handed out earlier
 *
 * @note Returns NULL for a pool that has been destroyed, which is what makes a
 *       late release from business a no-op rather than a crash.
 *
 * @return slot, or NULL if @p data did not come from @p pool
 */
STATIC tkl_uvc_slot_t *tkl_uvc_pool_find(tkl_uvc_pool_t *pool, CONST UINT8_T *data)
{
    UINT8_T i;

    if (pool->slots == NULL || data == NULL) {
        return NULL;
    }

    for (i = 0U; i < pool->depth; i++) {
        if (pool->slots[i].data == data) {
            return &pool->slots[i];
        }
    }

    return NULL;
}

/**
 * @brief Count the slots currently sitting in a pool's free queue.
 *
 * @param[in] pool pool to inspect
 *
 * @note Empties the free queue and puts everything straight back, so it is only
 *       safe once production has stopped - a live producer would see an empty
 *       pool for the duration and drop a frame.
 *
 * @return number of free slots
 */
STATIC UINT8_T tkl_uvc_pool_free_count(tkl_uvc_pool_t *pool)
{
    /* sized for the larger of the two pools; both maxima are checked at open */
    tkl_uvc_slot_t *held[TUYA_UVC_INTERNAL_POOL_DEPTH_MAX];
    UINT8_T n = 0U;
    UINT8_T i;

    if (pool->free_q == NULL) {
        return 0U;
    }

    while (n < pool->depth &&
           rtos_pop_from_queue(&pool->free_q, &held[n], BEKEN_NO_WAIT) == 0) {
        n++;
    }

    for (i = 0U; i < n; i++) {
        (void)rtos_push_to_queue(&pool->free_q, &held[i], BEKEN_NO_WAIT);
    }

    return n;
}

/**
 * @brief Wait for a pool's loaned slots to come back.
 * @param[in] pool       pool to drain
 * @param[in] timeout_ms how long to wait
 * @return OPRT_OK when every slot is back, OPRT_TIMEOUT otherwise
 */
STATIC OPERATE_RET tkl_uvc_pool_drain(tkl_uvc_pool_t *pool, UINT32_T timeout_ms)
{
    UINT32_T waited = 0U;

    while (tkl_uvc_pool_free_count(pool) < pool->depth) {
        if (waited >= timeout_ms) {
            return OPRT_TIMEOUT;
        }
        rtos_delay_milliseconds(TUYA_UVC_INTERNAL_DRAIN_STEP_MS);
        waited += TUYA_UVC_INTERNAL_DRAIN_STEP_MS;
    }

    return OPRT_OK;
}

/* ---------------------------------------------------------------------------
 * Release callbacks handed to business
 *
 * TUYA_MEDIA_BUF_T::free_cb takes no context argument, so these reach the pools
 * through the singleton context.
 * ------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 * tkl_uvc sink - capture side
 * ------------------------------------------------------------------------- */

/**
 * @brief Hand the vendor driver a slot to fill.
 *
 * @param[in] size bytes the driver wants
 * @param[in] arg  unused
 *
 * @note Runs on the vendor's URB thread. Must not block: returning NULL is a
 *       supported answer and puts the driver on its own drop path
 *       (bk_uvc_stream.c:1003), which is the back pressure this pool relies on.
 *
 * @return slot payload, or NULL
 */
STATIC VOID *tkl_uvc_out_alloc(UINT32_T size, VOID *arg)
{
    tkl_uvc_slot_t *slot;

    (void)arg;

    if (size > TUYA_UVC_INTERNAL_SLOT_SIZE) {
        s_uvc_out.drop_too_big++;
        return NULL;
    }

    slot = tkl_uvc_pool_take(&s_uvc_out.stream_pool);
    if (slot == NULL) {
        s_uvc_out.drop_stream_empty++;
        return NULL;
    }

    return slot->data;
}

/**
 * @brief Record a completed frame's metadata on its slot.
 *
 * @param[in] stream what the driver captured
 * @param[in] arg    unused
 *
 * @note Only called for a frame that arrived intact; a failed one goes straight
 *       to tkl_uvc_out_free(). Deliberately does **not** publish the slot - see
 *       the note there.
 */
STATIC VOID tkl_uvc_out_stream_cb(CONST TUYA_VCODEC_STREAM_T *stream, VOID *arg)
{
    tkl_uvc_slot_t *slot;

    (void)arg;

    if (stream == NULL || stream->data == NULL) {
        return;
    }

    slot = tkl_uvc_pool_find(&s_uvc_out.stream_pool, stream->data);
    if (slot == NULL) {
        return;
    }

    slot->len = stream->size;
    slot->pts = stream->pts;
    slot->fmt = (stream->codec == (UINT32_T)TUYA_VCODEC_TYPE_H264)
                    ? TUYA_FRAME_FMT_H264
                    : TUYA_FRAME_FMT_JPEG;

    switch (stream->frame_type) {
    case TUYA_VCODEC_FRAME_I:
        slot->frame_type = (UINT32_T)TUYA_VIDEO_FRAME_I;
        break;
    case TUYA_VCODEC_FRAME_P:
        slot->frame_type = (UINT32_T)TUYA_VIDEO_FRAME_P;
        break;
    default:
        slot->frame_type = (UINT32_T)TUYA_VIDEO_FRAME_UNKNOWN;
        break;
    }

    /* FRAME is an independent raw-input output. The vendor buffer is borrowed
     * only for this callback - free_cb stays NULL, so business must copy; the
     * slot is published to the decode task or recycled by free_cb below. */
    if ((s_uvc_out.cfg.output_kinds & TUYA_UVC_OUTPUT_FRAME) != 0U &&
        s_uvc_out.cfg.frame_cb != NULL) {
        TUYA_MEDIA_BUF_T buf;

        os_memset(&buf, 0, sizeof(buf));
        buf.data = slot->data;
        buf.size = slot->len;
        buf.pts = slot->pts;
        buf.width = s_uvc_out.cfg.width;
        buf.height = s_uvc_out.cfg.height;
        buf.format = slot->fmt;
        buf.frame_type = slot->frame_type;
        s_uvc_out.cfg.frame_cb(&buf, s_uvc_out.cfg.frame_arg);
    }

    slot->ready = TRUE;
}

/**
 * @brief Reclaim hook: either publish the slot or recycle it.
 *
 * @param[in] ptr payload the driver is done with
 * @param[in] arg unused
 *
 * @note This, not tkl_uvc_out_stream_cb(), is where a frame becomes visible to
 *       the decode task. Publishing from stream_cb would let a fast consumer
 *       pop, release and have the slot reissued before this call runs, which
 *       would then free the reissued slot a second time. Publish only once the
 *       driver is done with the slot.
 *
 * @return OPRT_OK
 */
STATIC OPERATE_RET tkl_uvc_out_free(VOID *ptr, VOID *arg)
{
    tkl_uvc_slot_t *slot;

    (void)arg;

    slot = tkl_uvc_pool_find(&s_uvc_out.stream_pool, (CONST UINT8_T *)ptr);
    if (slot == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (slot->ready != TRUE) {
        tkl_uvc_pool_give(&s_uvc_out.stream_pool, slot);
        return OPRT_OK;
    }

    slot->ready = FALSE;
    s_uvc_out.frames++;

    /* With a PIXEL/STREAM consumer the frame goes to the decode task; without
     * one it already reached frame_cb and must recycle straight back, or the
     * pool fills with frames nobody will pop. Cannot fill either way: the
     * ready queue is as deep as the pool, and a slot sitting in it is not in
     * the free queue, so it could not have been handed out again. Recycle
     * rather than lose the slot if that ever changes. */
    if (s_uvc_out.task_live == TRUE) {
        if (rtos_push_to_queue(&s_uvc_out.stream_pool.ready_q, &slot, BEKEN_NO_WAIT) != 0) {
            tkl_uvc_pool_give(&s_uvc_out.stream_pool, slot);
        }
    } else {
        tkl_uvc_pool_give(&s_uvc_out.stream_pool, slot);
    }

    return OPRT_OK;
}

STATIC VOID tkl_uvc_out_state_cb(BOOL_T connected, VOID *arg)
{
    (void)arg;

    if (s_uvc_out.cfg.state_cb != NULL) {
        s_uvc_out.cfg.state_cb(connected, s_uvc_out.cfg.state_arg);
    }
}

/* ---------------------------------------------------------------------------
 * PIXEL and STREAM output implementation - GPU and encoder consumers.
 *
 * Same shape as the MIPI camera's: the hardware connections stay internal, the
 * buffers on their far side are ours, and the release callback travels with the
 * buffer. The USB decoder is fed by the adapter's internal decode task.
 * ------------------------------------------------------------------------- */

STATIC VOID *tkl_uvc_out_display_alloc(UINT32_T size, VOID *arg)
{
    (void)arg;
    return tkl_media_buf_alloc(TKL_BUF_USE_PIXEL_WT, size, TUYA_MEDIA_BUF_OWNER_UVC_DISPLAY);
}

STATIC OPERATE_RET tkl_uvc_out_display_free(VOID *ptr, VOID *arg)
{
    (void)arg;

    if (ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_UVC_DISPLAY, ptr);
    return OPRT_OK;
}

/**
 * @brief GPU frame-done hook: wrap the composed picture and hand it to business.
 *
 * @param[in] frame      composed picture
 * @param[in] frame_size bytes
 * @param[in] arg        unused
 *
 * @note Runs on the GPU's flexa worker thread, concurrently with the encoder
 *       thread behind tkl_uvc_out_venc_stream_cb(). Ownership passes to
 *       business through free_cb; if business never releases, the pool drains
 *       and the pipeline stalls rather than corrupting.
 */
STATIC VOID tkl_uvc_out_display_done(VOID *frame, UINT32_T frame_size, VOID *arg)
{
    TUYA_MEDIA_BUF_T buf;

    (void)arg;

    if (frame == NULL) {
        return;
    }

    if (s_uvc_out.cfg.pixel_cb == NULL) {
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_UVC_DISPLAY, frame);
        return;
    }

    os_memset(&buf, 0, sizeof(buf));
    buf.data = (UINT8_T *)frame;
    buf.size = frame_size;
    buf.free_cb = tkl_media_buf_free_cb(TUYA_MEDIA_BUF_OWNER_UVC_DISPLAY);
    buf.pts = (UINT64_T)tkl_system_get_millisecond();
    buf.width = s_uvc_out.cfg.pixel.target_width;
    buf.height = s_uvc_out.cfg.pixel.target_height;
    buf.format = s_uvc_out.cfg.pixel.format;

    s_uvc_out.cfg.pixel_cb(&buf, s_uvc_out.cfg.pixel_arg);
}

STATIC VOID *tkl_uvc_out_venc_alloc(UINT32_T size, VOID *arg)
{
    (void)arg;
    return tkl_media_buf_alloc(TKL_BUF_USE_STREAM, size, TUYA_MEDIA_BUF_OWNER_UVC_STREAM);
}

STATIC OPERATE_RET tkl_uvc_out_venc_free(VOID *ptr, VOID *arg)
{
    (void)arg;

    if (ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_UVC_STREAM, ptr);
    return OPRT_OK;
}

/**
 * @brief Encoder output hook: wrap the bitstream and hand it to business.
 *
 * @param[in] stream encoded frame
 * @param[in] arg    unused
 *
 * @note Runs on the shared hardware-encoder thread. tkl_venc releases the
 *       payload as soon as this returns, so the buffer carries a NULL free_cb -
 *       business must copy anything it needs to keep. That is the one place
 *       where an asynchronously delivered buffer differs from a synchronous
 *       compatibility pull.
 */
STATIC VOID tkl_uvc_out_venc_stream_cb(CONST TUYA_VCODEC_STREAM_T *stream, VOID *arg)
{
    TUYA_MEDIA_BUF_T buf;

    (void)arg;

    if (stream == NULL || stream->data == NULL || s_uvc_out.cfg.stream_cb == NULL) {
        return;
    }

    os_memset(&buf, 0, sizeof(buf));
    buf.data = stream->data;
    buf.size = stream->size;
    buf.free_cb = NULL;             /* borrowed for the duration of this call */
    buf.pts = stream->pts;
    buf.width = s_uvc_out.src_width;
    buf.height = s_uvc_out.src_height;
    buf.format = TUYA_FRAME_FMT_H264;

    switch (stream->frame_type) {
    case TUYA_VCODEC_FRAME_I:
        buf.frame_type = (UINT32_T)TUYA_VIDEO_FRAME_I;
        break;
    case TUYA_VCODEC_FRAME_P:
        buf.frame_type = (UINT32_T)TUYA_VIDEO_FRAME_P;
        break;
    default:
        buf.frame_type = (UINT32_T)TUYA_VIDEO_FRAME_UNKNOWN;
        break;
    }

    s_uvc_out.cfg.stream_cb(&buf, s_uvc_out.cfg.stream_arg);
}

/**
 * @brief Allocate the ring the decoder writes slices into.
 *
 * @note Sixteen lines per segment - 92 KB at 1280 wide, against 1.3 MB for a
 *       decoded pixel picture. Uncoded pool rather than the vendor demo's HSRAM,
 *       so every media buffer in this file goes through one allocator.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_uvc_out_flexa_ring_alloc(UINT8_T depth)
{
    s_uvc_out.flexa_size = (UINT32_T)s_uvc_out.cfg.width *
                           TUYA_UVC_INTERNAL_FLEXA_LINES * 3U / 2U * (UINT32_T)depth;

    s_uvc_out.flexa_ring = (UINT8_T *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL,
                                                          s_uvc_out.flexa_size,
                                                          TUYA_MEDIA_BUF_OWNER_UVC_FRAME);
    if (s_uvc_out.flexa_ring == NULL) {
        tkl_log_output("tkl_uvc_out: flexa ring %u bytes failed\r\n",
                       (unsigned)s_uvc_out.flexa_size);
        s_uvc_out.flexa_size = 0U;
        return OPRT_MALLOC_FAILED;
    }

    return OPRT_OK;
}

STATIC VOID_T tkl_uvc_out_flexa_ring_free(VOID_T)
{
    if (s_uvc_out.flexa_ring != NULL) {
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_UVC_FRAME, s_uvc_out.flexa_ring);
        s_uvc_out.flexa_ring = NULL;
    }
    s_uvc_out.flexa_size = 0U;
}

/**
 * @brief Decoder flexa-done hook.
 *
 * @param[in] wr_cnt segments written so far
 * @param[in] arg    unused
 *
 * @note Deliberately empty. Read-pointer bookkeeping belongs to the bonds -
 *       each issues BK_JPEG_DECODE_IOCTL_PORT_SET_RD_PTR for its own port as it
 *       consumes, so the write pointer is already held back by the slower of
 *       the two. Anything here would be a second writer to those registers.
 */
STATIC VOID tkl_uvc_out_flexa_done_cb(UINT32_T wr_cnt, VOID *arg)
{
    (void)wr_cnt;
    (void)arg;
}

/**
 * @brief Open the decoder in flexa mode and publish its ring.
 * @param[out] flexa receives the ring descriptor the bonds attach to
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_uvc_out_decoder_open_flexa(TUYA_VCODEC_FLEXA_BUFFER_T *flexa)
{
    TUYA_VDEC_CFG_T cfg;
    TUYA_VDEC_SINK_T sink;
    OPERATE_RET ret;

    ret = tkl_uvc_out_flexa_ring_alloc(TUYA_UVC_INTERNAL_FLEXA_DEPTH);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.codec = (UINT32_T)TUYA_VCODEC_TYPE_MJPEG;
    cfg.width = s_uvc_out.cfg.width;
    cfg.height = s_uvc_out.cfg.height;
    cfg.flexa_lines = (UINT8_T)TUYA_UVC_INTERNAL_FLEXA_LINES;

    ret = tkl_vdec_init(&s_uvc_out.vdec, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: vdec init failed: %d\r\n", ret);
        goto err_ring;
    }

    os_memset(&sink, 0, sizeof(sink));
    sink.type = TUYA_VDEC_SINK_FLEXA;
    sink.flexa.flexa.src_buffer = s_uvc_out.flexa_ring;
    /* Must be the same count the ring was allocated with: the decoder walks the
     * ring by this number, so a smaller one wastes segments and a larger one
     * runs off the end. One constant, read twice, rather than a variable. */
    sink.flexa.flexa.buf_cnt = TUYA_UVC_INTERNAL_FLEXA_DEPTH;
    sink.flexa.flexa_cb = tkl_uvc_out_flexa_done_cb;

    ret = tkl_vdec_set_sink(s_uvc_out.vdec, &sink);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: vdec flexa sink failed: %d\r\n", ret);
        goto err_vdec;
    }

    /* start before asking for the source: the ring descriptor is only filled in
     * once the backend exists, and the bonds need the handle it carries. */
    ret = tkl_vdec_start(s_uvc_out.vdec);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: vdec start failed: %d\r\n", ret);
        goto err_vdec;
    }

    os_memset(flexa, 0, sizeof(*flexa));
    ret = tkl_vdec_get_flexa_source(s_uvc_out.vdec, flexa);
    if (ret != OPRT_OK || flexa->owner_handle == NULL) {
        tkl_log_output("tkl_uvc_out: vdec flexa source unavailable: %d\r\n", ret);
        ret = (ret != OPRT_OK) ? ret : OPRT_RESOURCE_NOT_READY;
        goto err_vdec;
    }

    s_uvc_out.src_width = flexa->width;
    s_uvc_out.src_height = flexa->height;

    tkl_log_output("tkl_uvc_out: flexa ring %u x %u lines = %u bytes, src %ux%u\r\n",
                   (unsigned)TUYA_UVC_INTERNAL_FLEXA_DEPTH, (unsigned)TUYA_UVC_INTERNAL_FLEXA_LINES,
                   (unsigned)s_uvc_out.flexa_size,
                   (unsigned)flexa->width, (unsigned)flexa->height);
    return OPRT_OK;

err_vdec:
    (void)tkl_vdec_deinit(s_uvc_out.vdec);
    s_uvc_out.vdec = NULL;
err_ring:
    tkl_uvc_out_flexa_ring_free();
    return ret;
}

/**
 * @brief Create and bind the display pipeline onto the decoder's ring.
 * @param[in] flexa decoder ring descriptor
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_uvc_out_display_open(CONST TUYA_VCODEC_FLEXA_BUFFER_T *flexa)
{
    TUYA_GPU_CFG_T gpu_cfg;
    TUYA_GPU_SINK_T sink;
    TUYA_GPU_SOURCE_T source;
    OPERATE_RET ret;

    os_memset(&gpu_cfg, 0, sizeof(gpu_cfg));
    gpu_cfg.dst_width = s_uvc_out.cfg.pixel.target_width;
    gpu_cfg.dst_height = s_uvc_out.cfg.pixel.target_height;
    gpu_cfg.rotate_degree = s_uvc_out.cfg.pixel.rotate;
    gpu_cfg.dst_format = tkl_media_fmt_to_bk(s_uvc_out.cfg.pixel.format);
    gpu_cfg.scale = TRUE;
    /* always compressed: a display frame goes straight to the DPU, which
     * decompresses. Same call as the camera facade makes. */
    gpu_cfg.dst_compress = TRUE;
    gpu_cfg.flexa_lines = TUYA_UVC_INTERNAL_FLEXA_LINES;

    ret = tkl_gpu_init(&gpu_cfg, &s_uvc_out.gpu);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: gpu init failed: %d "
                       "(only one display pipeline may be live - close the camera's)\r\n", ret);
        return ret;
    }

    os_memset(&sink, 0, sizeof(sink));
    sink.alloc_cb = tkl_uvc_out_display_alloc;
    sink.free_cb = tkl_uvc_out_display_free;
    sink.frame_done_cb = tkl_uvc_out_display_done;

    ret = tkl_gpu_set_sink(s_uvc_out.gpu, &sink);
    if (ret != OPRT_OK) {
        goto err;
    }

    os_memset(&source, 0, sizeof(source));
    source.type = TUYA_GPU_SOURCE_MJPEG_FLEXA;
    source.mjpeg.jpeg_handle = flexa->owner_handle;
    source.mjpeg.flexa.src_buffer = flexa->src_buffer;
    source.mjpeg.flexa.buf_cnt = flexa->buf_cnt;
    source.mjpeg.flexa.width = flexa->width;
    source.mjpeg.flexa.height = flexa->height;
    source.mjpeg.flexa.format = flexa->format;

    ret = tkl_gpu_set_source(s_uvc_out.gpu, &source);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: gpu set_source failed: %d\r\n", ret);
        goto err;
    }

    return OPRT_OK;

err:
    (void)tkl_gpu_deinit(s_uvc_out.gpu);
    s_uvc_out.gpu = NULL;
    return ret;
}

/**
 * @brief Create and bind the H264 transcoder onto the decoder's ring.
 * @param[in] flexa decoder ring descriptor
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_uvc_out_encode_open(CONST TUYA_VCODEC_FLEXA_BUFFER_T *flexa)
{
    TUYA_VENC_CFG_T venc_cfg;
    TUYA_VENC_SINK_T sink;
    TUYA_VENC_SOURCE_T source;
    TUYA_VENC_RATE_CTRL_T rc;
    OPERATE_RET ret;

    os_memset(&venc_cfg, 0, sizeof(venc_cfg));
    venc_cfg.width = flexa->width;
    venc_cfg.height = flexa->height;
    venc_cfg.fps = (s_uvc_out.cfg.encoder.fps != 0U) ? s_uvc_out.cfg.encoder.fps
                                                    : s_uvc_out.cfg.fps;
    venc_cfg.gop = s_uvc_out.cfg.encoder.gop;
    venc_cfg.codec = (UINT32_T)TUYA_VCODEC_TYPE_H264;
    venc_cfg.input_format = flexa->format;

    ret = tkl_venc_init(&s_uvc_out.venc, &venc_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: venc init failed: %d\r\n", ret);
        return ret;
    }

    if (s_uvc_out.cfg.encoder.bitrate != 0U) {
        os_memset(&rc, 0, sizeof(rc));
        rc.bitrate = s_uvc_out.cfg.encoder.bitrate;
        (void)tkl_venc_set_rate_ctrl(s_uvc_out.venc, &rc);
    }

    os_memset(&sink, 0, sizeof(sink));
    sink.stream_cb = tkl_uvc_out_venc_stream_cb;
    sink.alloc_cb = tkl_uvc_out_venc_alloc;
    sink.free_cb = tkl_uvc_out_venc_free;

    ret = tkl_venc_set_sink(s_uvc_out.venc, &sink);
    if (ret != OPRT_OK) {
        goto err;
    }

    os_memset(&source, 0, sizeof(source));
    source.type = TUYA_VENC_SOURCE_VDEC_FLEXA;
    source.vdec.vdec_handle = flexa->owner_handle;
    source.vdec.flexa = *flexa;

    ret = tkl_venc_set_source(s_uvc_out.venc, &source);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: venc set_source failed: %d\r\n", ret);
        goto err;
    }

    return OPRT_OK;

err:
    (void)tkl_venc_deinit(s_uvc_out.venc);
    s_uvc_out.venc = NULL;
    return ret;
}

/**
 * @brief Attach the display overlay to the running GPU pipeline.
 */
STATIC OPERATE_RET tkl_uvc_out_osd_display_open(VOID_T)
{
    TKL_OSD_ENGINE_CFG_T cfg;
    VOID *gpu_raw = NULL;
    OPERATE_RET ret;

    ret = tkl_gpu_get_handle(s_uvc_out.gpu, &gpu_raw);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.target_w = s_uvc_out.cfg.pixel.target_width;
    cfg.target_h = s_uvc_out.cfg.pixel.target_height;
    /* the caller supplies the rotation per overlay request */
    cfg.src_format = (UINT32_T)BK_PIXEL_FORMAT_ABGR8888;

    return tkl_osd_engine_open(TKL_OSD_BACKEND_GPU, gpu_raw, &cfg, &s_uvc_out.osd_display);
}

/**
 * @brief Build both push consumers and start their bonds.
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_uvc_out_push_open(VOID_T)
{
    TUYA_VCODEC_FLEXA_BUFFER_T flexa;
    OPERATE_RET ret;

    ret = tkl_uvc_out_decoder_open_flexa(&flexa);
    if (ret != OPRT_OK) {
        return ret;
    }

    /* consumers first, bonds after - a bond must never start against a backend
     * that is not fully built. Order between the two bonds does not matter; the
     * vendor's own product code is inconsistent about it, which is how we know. */
    if (s_uvc_out.want_display == TRUE) {
        ret = tkl_uvc_out_display_open(&flexa);
        if (ret != OPRT_OK) {
            goto err_vdec;
        }
    }

    if (s_uvc_out.want_encode == TRUE) {
        ret = tkl_uvc_out_encode_open(&flexa);
        if (ret != OPRT_OK) {
            goto err_display;
        }
    }

    if (s_uvc_out.want_display == TRUE) {
        ret = tkl_gpu_start(s_uvc_out.gpu);
        if (ret != OPRT_OK) {
            tkl_log_output("tkl_uvc_out: gpu start failed: %d\r\n", ret);
            goto err_encode;
        }
    }

    if (s_uvc_out.want_encode == TRUE) {
        ret = tkl_venc_start(s_uvc_out.venc);
        if (ret != OPRT_OK) {
            tkl_log_output("tkl_uvc_out: venc start failed: %d\r\n", ret);
            goto err_gpu_started;
        }
    }

    /* overlays last: they bind to a backend that is already running */
    if (s_uvc_out.want_display == TRUE) {
        (void)tkl_uvc_out_osd_display_open();   /* optional, never fatal */
    }

    /* The decode task consumes the capture queue into the decoder. It must
     * exist before the camera starts producing - tkl_uvc_start() starts the
     * camera last. Failure is fatal for PIXEL/STREAM: the caller's err path
     * runs push_close(), which also retires a half-started task. */
    ret = tkl_uvc_out_task_start();
    return ret;

err_gpu_started:
    if (s_uvc_out.want_display == TRUE) {
        (void)tkl_gpu_stop(s_uvc_out.gpu);
    }
err_encode:
    if (s_uvc_out.venc != NULL) {
        (void)tkl_venc_deinit(s_uvc_out.venc);
        s_uvc_out.venc = NULL;
    }
err_display:
    if (s_uvc_out.gpu != NULL) {
        (void)tkl_gpu_deinit(s_uvc_out.gpu);
        s_uvc_out.gpu = NULL;
    }
err_vdec:
    (void)tkl_vdec_deinit(s_uvc_out.vdec);
    s_uvc_out.vdec = NULL;
    tkl_uvc_out_flexa_ring_free();
    return ret;
}

/**
 * @brief Tear down both push consumers.
 *
 * @note Iron rule: every bond stops before any backend is touched.
 *       tkl_venc_stop() and tkl_gpu_stop() each stop their own bond and close
 *       only their own backend, so calling them back to back keeps it.
 */
STATIC VOID_T tkl_uvc_out_push_close(VOID_T)
{
    if (s_uvc_out.task_live == TRUE) {
        /* End the decode task before any backend it feeds goes away. It polls
         * task_stop while the ready queue idles, so this join costs at most one
         * poll plus whatever decode is already in flight. */
        s_uvc_out.task_stop = TRUE;
        (void)rtos_get_semaphore(&s_uvc_out.task_done, TUYA_UVC_INTERNAL_DRAIN_MS);
        (void)rtos_deinit_semaphore(&s_uvc_out.task_done);
        s_uvc_out.task_live = FALSE;
        s_uvc_out.task_stop = FALSE;
    }

    if (s_uvc_out.osd_display != NULL) {
        (void)tkl_osd_engine_close(s_uvc_out.osd_display);
        s_uvc_out.osd_display = NULL;
    }

    if (s_uvc_out.venc != NULL) {
        (void)tkl_venc_stop(s_uvc_out.venc);
    }
    if (s_uvc_out.gpu != NULL) {
        (void)tkl_gpu_stop(s_uvc_out.gpu);
    }

    if (s_uvc_out.venc != NULL) {
        (void)tkl_venc_deinit(s_uvc_out.venc);
        s_uvc_out.venc = NULL;
    }
    if (s_uvc_out.gpu != NULL) {
        (void)tkl_gpu_deinit(s_uvc_out.gpu);
        s_uvc_out.gpu = NULL;
    }
}

/* ---------------------------------------------------------------------------
 * Bring-up
 * ------------------------------------------------------------------------- */

/**
 * @brief Enumerate the USB camera and attach the capture sink. No streaming.
 *
 * @note The expensive half of the facade: uvc_dev_init() powers the hub port
 *       and waits for enumeration, roughly 540 ms. Splitting it out of start()
 *       is the point of the init/start API.
 * @note Deliberately does not start the stream. Nothing may produce until the
 *       consumers built by tkl_uvc_start() are standing.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_uvc_out_camera_init(VOID_T)
{
    TUYA_UVC_CFG_T cfg;
    TUYA_UVC_SINK_T sink;
    OPERATE_RET ret;

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.port = s_uvc_out.cfg.port;
    cfg.width = s_uvc_out.cfg.width;
    cfg.height = s_uvc_out.cfg.height;
    cfg.fps = s_uvc_out.cfg.fps;
    cfg.input_format = (s_uvc_out.cfg.input_format == (UINT32_T)TUYA_FRAME_FMT_H264)
                    ? (UINT32_T)TUYA_VCODEC_TYPE_H264
                    : (UINT32_T)TUYA_VCODEC_TYPE_MJPEG;
    
    ret = uvc_dev_init(&s_uvc_out.uvc, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: uvc init failed: %d\r\n", ret);
        return ret;
    }

    os_memset(&sink, 0, sizeof(sink));
    sink.stream_cb = tkl_uvc_out_stream_cb;
    sink.state_cb = tkl_uvc_out_state_cb;
    sink.alloc_cb = tkl_uvc_out_alloc;
    sink.free_cb = tkl_uvc_out_free;

    ret = uvc_dev_set_sink(s_uvc_out.uvc, &sink);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: uvc sink failed: %d\r\n", ret);
        (void)uvc_dev_deinit(s_uvc_out.uvc);
        s_uvc_out.uvc = NULL;
    }

    return ret;
}

/* Defined below, next to the pool code they belong to; declared here because
 * tkl_uvc_stop() and tkl_uvc_deinit() sit above them in lifecycle order, which
 * is the order a reader wants. */
STATIC UINT32_T tkl_uvc_out_flush_ready(tkl_uvc_pool_t *pool);
STATIC VOID_T tkl_uvc_out_retire_pool(tkl_uvc_pool_t *pool,
                                      TUYA_MEDIA_BUF_OWNER_E owner,
                                      CONST CHAR_T *name);

OPERATE_RET tkl_uvc_init(CONST TUYA_UVC_CFG_T *cfg)
{
    UINT8_T depth;
    UINT32_T ms_pool;
    UINT32_T ms_device;
    UINT32_T t;
    OPERATE_RET ret;

    if (cfg == NULL || cfg->width == 0U || cfg->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (s_uvc_out.state != TUYA_UVC_ST_IDLE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* All requested outputs share one capture queue. The adapter chooses its
     * internal depth; output combinations are independent of delivery style. */
    if ((cfg->output_kinds & ~TUYA_UVC_OUTPUT_KIND_MASK) != 0U ||
        cfg->output_kinds == TUYA_UVC_OUTPUT_NONE) {
        tkl_log_output("tkl_uvc_out: output_kinds 0x%x has no meaning\r\n",
                       (unsigned)cfg->output_kinds);
        return OPRT_INVALID_PARM;
    }

    /* This backend decodes MJPEG for PIXEL/STREAM. H264 remains a raw FRAME
     * source until a H264 decode path is added. */
    if ((cfg->output_kinds & ~TUYA_UVC_OUTPUT_FRAME) != 0U &&
        cfg->input_format == (UINT32_T)TUYA_FRAME_FMT_H264) {
        tkl_log_output("tkl_uvc_out: only JPEG can feed PIXEL/STREAM\r\n");
        return OPRT_NOT_SUPPORTED;
    }

    if ((cfg->output_kinds & TUYA_UVC_OUTPUT_PIXEL) != 0U &&
        (cfg->pixel.target_width == 0U || cfg->pixel.target_height == 0U)) {
        tkl_log_output("tkl_uvc_out: PIXEL needs pixel.target_width/target_height\r\n");
        return OPRT_INVALID_PARM;
    }

    os_memset(&s_uvc_out, 0, sizeof(s_uvc_out));
    os_memcpy(&s_uvc_out.cfg, cfg, sizeof(s_uvc_out.cfg));

    if ((cfg->output_kinds & TUYA_UVC_OUTPUT_PIXEL) != 0U) {
        s_uvc_out.want_display = TRUE;
    }
    if ((cfg->output_kinds & TUYA_UVC_OUTPUT_STREAM) != 0U) {
        s_uvc_out.want_encode = TRUE;
    }

    depth = TUYA_UVC_INTERNAL_POOL_DEPTH_DEFAULT;

    t = TUYA_UVC_INTERNAL_MS();
    ret = tkl_uvc_pool_create(&s_uvc_out.stream_pool, depth, TUYA_UVC_INTERNAL_SLOT_SIZE,
                              TKL_BUF_USE_STREAM, TUYA_MEDIA_BUF_OWNER_UVC_STREAM,
                              TRUE, "uvc_stream");
    ms_pool = TUYA_UVC_INTERNAL_MS() - t;
    if (ret != OPRT_OK) {
        os_memset(&s_uvc_out, 0, sizeof(s_uvc_out));
        return ret;
    }

    t = TUYA_UVC_INTERNAL_MS();
    ret = tkl_uvc_out_camera_init();
    ms_device = TUYA_UVC_INTERNAL_MS() - t;
    if (ret != OPRT_OK) {
        tkl_uvc_pool_destroy(&s_uvc_out.stream_pool, TUYA_MEDIA_BUF_OWNER_UVC_STREAM);
        os_memset(&s_uvc_out, 0, sizeof(s_uvc_out));
        return ret;
    }

    s_uvc_out.state = TUYA_UVC_ST_INIT;
    tkl_log_output("tkl_uvc_out: init %ux%u kinds 0x%x, pool %u x %u bytes, "
                   "%u ms (pool %u, device %u)\r\n",
                   (unsigned)cfg->width, (unsigned)cfg->height,
                   (unsigned)cfg->output_kinds,
                   (unsigned)s_uvc_out.stream_pool.depth,
                   (unsigned)TUYA_UVC_INTERNAL_SLOT_SIZE,
                   (unsigned)(ms_pool + ms_device),
                   (unsigned)ms_pool, (unsigned)ms_device);
    return OPRT_OK;
}

OPERATE_RET tkl_uvc_start(VOID_T)
{
    UINT32_T ms_consumer;
    UINT32_T ms_stream;
    UINT32_T t;
    OPERATE_RET ret;

    if (s_uvc_out.state == TUYA_UVC_ST_STARTED) {
        return OPRT_OK;
    }
    if (s_uvc_out.state != TUYA_UVC_ST_INIT) {
        return OPRT_RESOURCE_NOT_READY;
    }

    t = TUYA_UVC_INTERNAL_MS();
    if ((s_uvc_out.cfg.output_kinds & (TUYA_UVC_OUTPUT_PIXEL | TUYA_UVC_OUTPUT_STREAM)) != 0U) {
        ret = tkl_uvc_out_push_open();
    } else {
        ret = OPRT_OK;
    }
    ms_consumer = TUYA_UVC_INTERNAL_MS() - t;
    if (ret != OPRT_OK) {
        goto err_consumer;
    }

    /* Camera last. It is the only thing here that produces: from the moment it
     * starts, alloc/stream/free run against the pool, so everything downstream
     * has to be standing already or the first frames are dropped for nothing. */
    t = TUYA_UVC_INTERNAL_MS();
    ret = uvc_dev_start(s_uvc_out.uvc);
    ms_stream = TUYA_UVC_INTERNAL_MS() - t;
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: uvc start failed: %d\r\n", ret);
        goto err_consumer;
    }

    s_uvc_out.state = TUYA_UVC_ST_STARTED;
    tkl_log_output("tkl_uvc_out: start took %u ms (consumer %u, stream %u)\r\n",
                   (unsigned)(ms_consumer + ms_stream),
                   (unsigned)ms_consumer, (unsigned)ms_stream);
    return OPRT_OK;

err_consumer:
    /* Back to INIT, not IDLE: the device stays enumerated and the pool stays
     * built, so the caller can fix whatever failed and try start() again
     * without paying the 539 ms enumeration a second time. */
    tkl_uvc_out_push_close();
    if (s_uvc_out.vdec != NULL) {
        (void)tkl_vdec_deinit(s_uvc_out.vdec);
        s_uvc_out.vdec = NULL;
    }
    tkl_uvc_out_flexa_ring_free();
    return ret;
}

OPERATE_RET tkl_uvc_stop(VOID_T)
{
    UINT32_T unconsumed;
    UINT32_T ms_device;
    UINT32_T ms_consumer;
    UINT32_T t;

    if (s_uvc_out.state != TUYA_UVC_ST_STARTED) {
        return OPRT_OK;
    }

    /* Nothing new can enter from here on: the state gate turns fresh calls away
     * while the pipeline is dismantled. */
    s_uvc_out.state = TUYA_UVC_ST_INIT;

    t = TUYA_UVC_INTERNAL_MS();

    /* 1. producer first - once uvc_dev_stop() returns, nothing else will call
     *    alloc / stream / free, so the pool stops moving and the decode task
     *    drains whatever the ready queue still holds within a frame. */
    (void)uvc_dev_stop(s_uvc_out.uvc);
    ms_device = TUYA_UVC_INTERNAL_MS() - t;
    t = TUYA_UVC_INTERNAL_MS();

    /* 2. every bond, then every backend - the iron rule. Both calls are no-ops
     *    when the PIXEL/STREAM consumers were never built. */
    tkl_uvc_out_push_close();

    /* 3. then the decoder; push_close() above has already joined the decode
     *    task that feeds it. FRAME-only never built one. */
    if (s_uvc_out.vdec != NULL) {
        (void)tkl_vdec_deinit(s_uvc_out.vdec);
        s_uvc_out.vdec = NULL;
    }
    tkl_uvc_out_flexa_ring_free();
    ms_consumer = TUYA_UVC_INTERNAL_MS() - t;

    /* 4. frames sitting in the ready queue belong to nobody now that production
     *    has stopped - take them back so the pool is whole again for the next
     *    start(). Cheap: no waiting, just a queue drain. */
    unconsumed = tkl_uvc_out_flush_ready(&s_uvc_out.stream_pool);

    /* Deliberately NOT draining tkl_media_buf here. Business is allowed to hold
     * a frame across a stop/start - the pool survives, it is just one slot
     * short until the buffer comes back. Waiting for it would add up to
     * TUYA_UVC_INTERNAL_DRAIN_MS to every pause, which is exactly the cost this API
     * split exists to remove. tkl_uvc_deinit() does the waiting instead. */
    tkl_log_output("tkl_uvc_out: stop took %u ms (device %u, consumer %u), "
                   "%u frame(s) reclaimed unconsumed\r\n",
                   (unsigned)(ms_device + ms_consumer),
                   (unsigned)ms_device, (unsigned)ms_consumer,
                   (unsigned)unconsumed);
    return OPRT_OK;
}

OPERATE_RET tkl_uvc_deinit(VOID_T)
{
    UINT32_T ms_device;
    UINT32_T ms_pool;
    UINT32_T t;

    if (s_uvc_out.state == TUYA_UVC_ST_IDLE) {
        return OPRT_OK;
    }

    (void)tkl_uvc_stop();

    tkl_log_output("tkl_uvc_out: deinit, %u frames published, %u decoded, "
                   "drops stream %u size %u, decode fail %u\r\n",
                   (unsigned)s_uvc_out.frames,
                   (unsigned)s_uvc_out.decoded,
                   (unsigned)s_uvc_out.drop_stream_empty,
                   (unsigned)s_uvc_out.drop_too_big,
                   (unsigned)s_uvc_out.decode_fail);

    t = TUYA_UVC_INTERNAL_MS();
    if (s_uvc_out.uvc != NULL) {
        (void)uvc_dev_deinit(s_uvc_out.uvc);
        s_uvc_out.uvc = NULL;
    }
    ms_device = TUYA_UVC_INTERNAL_MS() - t;
    t = TUYA_UVC_INTERNAL_MS();

    tkl_uvc_out_retire_pool(&s_uvc_out.stream_pool,
                            TUYA_MEDIA_BUF_OWNER_UVC_STREAM, "stream pool");

    /* Pushed buffers are per-frame rather than pooled, so they settle by
     * counter. Display tolerates one still out - the panel is scanning it and
     * nothing will replace it now - while a bitstream must reach zero. Waited
     * here rather than in stop(), where the memory does not yet go away. */
    if (s_uvc_out.want_display == TRUE) {
        (void)tkl_media_buf_drain(TUYA_MEDIA_BUF_OWNER_UVC_DISPLAY,
                                  TUYA_UVC_OUTPUT_PIXEL_RESIDUAL, TUYA_UVC_INTERNAL_DRAIN_MS);
    }
    if (s_uvc_out.want_encode == TRUE) {
        (void)tkl_media_buf_drain(TUYA_MEDIA_BUF_OWNER_UVC_STREAM, 0U, TUYA_UVC_INTERNAL_DRAIN_MS);
    }
    ms_pool = TUYA_UVC_INTERNAL_MS() - t;

    tkl_log_output("tkl_uvc_out: deinit took %u ms (device %u, pool+drain %u)\r\n",
                   (unsigned)(ms_device + ms_pool),
                   (unsigned)ms_device, (unsigned)ms_pool);

    os_memset(&s_uvc_out, 0, sizeof(s_uvc_out));
    return OPRT_OK;
}

/**
 * @brief Empty a pool's ready queue back into its free queue.
 *
 * @param[in] pool pool to flush; no-op when it has no ready queue
 *
 * @note Frames captured and published but never fetched, because the consumer
 *       stopped before the producer did. Normal at teardown, and the reason the
 *       published and consumed counts can differ - close() reports it.
 *
 * @return how many frames were reclaimed unconsumed
 */
STATIC UINT32_T tkl_uvc_out_flush_ready(tkl_uvc_pool_t *pool)
{
    tkl_uvc_slot_t *slot = NULL;
    UINT32_T n = 0U;

    if (pool->ready_q == NULL) {
        return 0U;
    }

    while (rtos_pop_from_queue(&pool->ready_q, &slot, BEKEN_NO_WAIT) == 0) {
        tkl_uvc_pool_give(pool, slot);
        n++;
    }

    return n;
}

/**
 * @brief Give up a pool's memory, or leak it if business still holds a slot.
 *
 * @param[in] pool  pool to retire
 * @param[in] owner accounting bucket the slabs were charged to
 * @param[in] name  pool name, for the leak message
 *
 * @note Freeing memory a caller still points at turns a leak into a
 *       use-after-free, so on timeout the slabs are deliberately left
 *       allocated. Zeroing the descriptor makes the release that eventually
 *       arrives find no slot and do nothing, which is the safe outcome.
 */
STATIC VOID_T tkl_uvc_out_retire_pool(tkl_uvc_pool_t *pool,
                                      TUYA_MEDIA_BUF_OWNER_E owner,
                                      CONST CHAR_T *name)
{
    if (pool->slots == NULL) {
        return;
    }

    if (tkl_uvc_pool_drain(pool, TUYA_UVC_INTERNAL_DRAIN_MS) != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: %s still has %u buffers out, leaking %u bytes\r\n",
                       name,
                       (unsigned)(pool->depth - tkl_uvc_pool_free_count(pool)),
                       (unsigned)(pool->slot_size * pool->depth));
        if (pool->free_q != NULL) {
            rtos_deinit_queue(&pool->free_q);
        }
        if (pool->ready_q != NULL) {
            rtos_deinit_queue(&pool->ready_q);
        }
        os_memset(pool, 0, sizeof(*pool));
        return;
    }

    tkl_uvc_pool_destroy(pool, owner);
}

/* ---------------------------------------------------------------------------
 * Decode task - the adapter-owned consumer that turns captured frames into
 * PIXEL/STREAM outputs. Business no longer pumps: start() spawns this task
 * inside the consumer lifecycle and stop() joins it.
 * ------------------------------------------------------------------------- */

/**
 * @brief Feed one captured frame to the decoder.
 *
 * @param[in] slot frame to decode; returned to the pool either way
 *
 * @return OPRT_OK when the decoder accepted the frame
 */
STATIC OPERATE_RET tkl_uvc_out_decode_frame(tkl_uvc_slot_t *slot)
{
    TUYA_VCODEC_STREAM_T stream;
    OPERATE_RET ret;

    os_memset(&stream, 0, sizeof(stream));
    stream.data = slot->data;
    stream.size = slot->len;
    stream.pts = slot->pts;
    stream.codec = (UINT32_T)TUYA_VCODEC_TYPE_MJPEG;

    /* Synchronous: blocks until the decoder has finished this picture. Nothing
     * comes back here - the output leaves in slices through the ring the GPU
     * and the encoder are bonded to, and their results reach business on
     * pixel_cb / stream_cb, from the vendor worker threads. */
    ret = tkl_vdec_submit_stream(s_uvc_out.vdec, &stream);

    /* The frame has served its purpose the moment the decoder is done with it,
     * whether or not it decoded - hand the slot straight back so capture never
     * waits on the downstream consumers. */
    tkl_uvc_pool_give(&s_uvc_out.stream_pool, slot);

    if (ret != OPRT_OK) {
        s_uvc_out.decode_fail++;
        return ret;
    }

    s_uvc_out.decoded++;
    return OPRT_OK;
}

/**
 * @brief The adapter's decode task.
 *
 * @param[in] arg unused
 *
 * @note Pops frames off the capture queue and feeds the decoder. The ready
 *       queue is as deep as the pool, so a slow decode backs up into capture
 *       dropping frames (drop_stream_empty) - the same back pressure the old
 *       business pump had, owned by the adapter now.
 * @note stop() ends the task by setting task_stop and joining on task_done.
 *       While the queue is idle the task wakes every poll period to re-check
 *       the flag, which bounds the join to one poll plus one decode in flight.
 */
STATIC VOID tkl_uvc_out_decode_task(beken_thread_arg_t arg)
{
    tkl_uvc_slot_t *slot = NULL;

    (void)arg;

    for (;;) {
        if (s_uvc_out.task_stop == TRUE) {
            break;
        }

        if (rtos_pop_from_queue(&s_uvc_out.stream_pool.ready_q, &slot,
                                TUYA_UVC_INTERNAL_TASK_POLL_MS) != 0 ||
            slot == NULL) {
            continue;   /* idle poll; the loop head re-checks task_stop */
        }

        /* stop() may already be waiting for us: a frame nobody can consume
         * goes straight back to the pool instead of through the decoder. */
        if (s_uvc_out.task_stop == TRUE) {
            tkl_uvc_pool_give(&s_uvc_out.stream_pool, slot);
            break;
        }

        (void)tkl_uvc_out_decode_frame(slot);
    }

    (void)rtos_set_semaphore(&s_uvc_out.task_done);
    rtos_delete_thread(NULL);
}

/**
 * @brief Spawn the decode task. PIXEL/STREAM consumers only - the pure FRAME
 *        configuration has nothing to decode and never starts one.
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_uvc_out_task_start(VOID_T)
{
    OPERATE_RET ret;

    ret = rtos_init_semaphore_ex(&s_uvc_out.task_done, 1, 0);
    if (ret != OPRT_OK) {
        return ret;
    }

    s_uvc_out.task_stop = FALSE;
    s_uvc_out.task_live = TRUE;

    ret = rtos_create_thread(&s_uvc_out.task,
                             BEKEN_DEFAULT_WORKER_PRIORITY,
                             "uvc_decode",
                             (beken_thread_function_t)tkl_uvc_out_decode_task,
                             TUYA_UVC_INTERNAL_TASK_STACK,
                             NULL);
    if (ret != OPRT_OK) {
        (void)rtos_deinit_semaphore(&s_uvc_out.task_done);
        s_uvc_out.task_live = FALSE;
        return ret;
    }

    return OPRT_OK;
}

/* ---------------------------------------------------------------------------
 * Runtime controls
 * ------------------------------------------------------------------------- */

OPERATE_RET tkl_uvc_set_encode(CONST TUYA_MEDIA_ENC_CTRL_T *ctrl)
{
    TUYA_VENC_RATE_CTRL_T rc;
    OPERATE_RET ret = OPRT_OK;

    if (ctrl == NULL) {
        return OPRT_INVALID_PARM;
    }

    /* The encoder is built by start() and torn down by stop(), so there is
     * nothing to configure in between. Refused rather than remembered: a
     * setting that silently waits for the next start() is a bug factory. */
    if (s_uvc_out.state != TUYA_UVC_ST_STARTED || s_uvc_out.venc == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (ctrl->bitrate != 0U) {
        os_memset(&rc, 0, sizeof(rc));
        rc.bitrate = ctrl->bitrate;
        rc.force_idr = ctrl->force_idr;
        s_uvc_out.cfg.encoder.bitrate = ctrl->bitrate;
        return tkl_venc_set_rate_ctrl(s_uvc_out.venc, &rc);
    }

    if (ctrl->force_idr == TRUE) {
        ret = tkl_venc_request_idr(s_uvc_out.venc);
    }

    return ret;
}

OPERATE_RET tkl_uvc_set_osd(TUYA_UVC_OSD_CHANNEL_E chn, CONST TUYA_OSD_T *osd)
{
    if (osd == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_uvc_out.state != TUYA_UVC_ST_STARTED) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (chn == TUYA_UVC_OSD_PIXEL) {
        if (s_uvc_out.osd_display == NULL) {
            return OPRT_NOT_SUPPORTED;
        }
        return tkl_osd_engine_submit(s_uvc_out.osd_display, osd);
    }

    if (s_uvc_out.venc == NULL) {
        return OPRT_NOT_SUPPORTED;
    }

    return tkl_venc_set_osd(s_uvc_out.venc, osd);
}

#else /* CONFIG_USB_CAMERA */

OPERATE_RET tkl_uvc_init(CONST TUYA_UVC_CFG_T *cfg)
{
    (void)cfg;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_start(VOID_T)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_stop(VOID_T)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_deinit(VOID_T)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_set_encode(CONST TUYA_MEDIA_ENC_CTRL_T *ctrl)
{
    (void)ctrl;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_set_osd(TUYA_UVC_OSD_CHANNEL_E chn, CONST TUYA_OSD_T *osd)
{
    (void)chn;
    (void)osd;
    return OPRT_NOT_SUPPORTED;
}


#endif /* CONFIG_USB_CAMERA */
