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
 *   facade (public)        init / start / stop / deinit / set_osd / set_bitrate
 *                          / force_idr / get_frame, plus get_info
 *
 * The facade exists for buffer ownership and for the decode task: an adapter
 * thread that pops the capture queue into the decoder, which is what feeds the
 * PIXEL/STREAM outputs. The device layer and tkl_vdec both **lend** their
 * buffer for one callback and reclaim it on return, which would force every
 * caller to memcpy the frame out. Here the buffers come from pools this file
 * owns and the reclaim hook publishes to the ready queue instead, so the
 * borrow becomes a loan business ends by calling buf.free_cb.
 *
 * @version 0.3
 * @date 2026-09-17
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tkl_gpu_internal.h"
#include "tkl_image_codec.h"
#include "tkl_media_buf.h"
#include "tkl_media_fmt.h"
#include "tkl_media_types.h"
#include "tkl_memory.h"
#include "tkl_osd_engine.h"
#include "tkl_output.h"
#include "tkl_pp.h"
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
#include <components/cherryusb/usbh_core.h>
#include <components/cherryusb/usbh_video.h>
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
/** Vendor plug/unplug notification, mapped to CONNECT/DISCONNECT events by
 *  tkl_uvc_out_state_cb(). Local to this adapter: the public API reports
 *  TUYA_CAMERA_EVENT_E instead. */
typedef VOID (*TUYA_UVC_STATE_CB)(BOOL_T connected, VOID *arg);

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
#define TUYA_UVC_INFO_POLL_MS               100U
#define TUYA_UVC_INFO_DEV_MAX               4U

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

/* Slack in the ring the decoder writes slices into.
 *
 * Deliberately small: the vendor decoder only accepts a handful of segments
 * (its own default is 2, the video player uses 4), and asking for more leaves
 * it unable to start at all - "decode timeout", in:0 out:0, every frame. ch1
 * pull therefore copies each finished segment out from the segment hook
 * instead of waiting for a whole frame to accumulate here. */
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

/* Slots parked for get_frame(): one being copied, one waiting to be pulled.
 * A pull API that is never pulled drops fresh frames (drop_frame_empty) rather
 * than growing stale ones, so two is enough. */
#define TUYA_UVC_FRAME_POOL_DEPTH          2U

#define TUYA_UVC_INTERNAL_ALIGN16(v)       (((v) + 15U) & ~15U)

/* Header in front of a ch2 display payload. sizeof(TUYA_MEDIA_BUF_T) is not
 * 64-aligned, and the write-through mapping the vendor pool installs refuses a
 * non-aligned payload address - so the struct is padded up to 64 and the
 * payload stays aligned no matter what the struct grows to. */
/* one-block frame layout: see TUYA_MEDIA_BUF_HDR in tkl_media_types.h */
#define TUYA_UVC_BUF_HDR  TUYA_MEDIA_BUF_HDR

#define TUYA_UVC_INTERNAL_MS()             ((UINT32_T)tkl_system_get_millisecond())

/** One pool slot. Allocated once at open, recycled for the life of the stream. */
typedef struct {
    UINT8_T *data;
    UINT32_T capacity;

    /* filled in by the producer, read by whoever pops it */
    UINT32_T len;
    UINT64_T pts;
    TUYA_FRAME_FMT_E fmt;   /**< content format; an H264 frame carries its
                                 frame type inside the format (H264_I/H264_P) */

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
    TUYA_UVC_CFG_T cfg;         /**< outputs deep-copied at init() */

    TUYA_UVC_HANDLE uvc;
    TUYA_VDEC_HANDLE vdec;      /**< decoder: flexa ring for ch2/ch3, FRAME sink for ch1 push */
    BOOL_T vdec_frame_sink;     /**< TRUE when vdec runs the FRAME sink (ch1 push) */

    /* The channel view of cfg.outputs, resolved once at init():
     *  ch0 = parked JPEG (get_frame / cb), ch1 = decoded NV12,
     *  ch2 = GPU display, ch3 = H264 (transcode, or passthrough from an
     *  H264 source). See the topology table in tkl_uvc.h. */
    BOOL_T want_sp;             /**< ch1 configured */
    BOOL_T want_display;        /**< ch2 configured */
    BOOL_T want_encode;         /**< ch3 configured on an MJPEG source (transcode) */
    BOOL_T ch_active[TUYA_CAMERA_FRAME_CHANNEL_MAX];
    BOOL_T pp_pending[TUYA_CAMERA_FRAME_CHANNEL_MAX];
    UINT16_T pp_width[TUYA_CAMERA_FRAME_CHANNEL_MAX];
    UINT16_T pp_height[TUYA_CAMERA_FRAME_CHANNEL_MAX];

    TKL_GPU_HANDLE gpu;
    TUYA_VENC_HANDLE venc;
    TKL_OSD_ENGINE_HANDLE osd_display;
    UINT8_T *flexa_ring;        /**< decoder ring the bonds read from */
    UINT32_T flexa_size;

    /* ch1 pull while the flexa pipeline runs: the ring is only FLEXA_DEPTH
     * segments deep and cannot hold a frame, so the decoder's segment callback
     * copies each finished segment into this full-frame buffer. get_frame(ch1)
     * then returns it instead of running the on-demand decoder - a second
     * decoder session on the same hw_decoder_ctlr shares one IRQ with the
     * streaming one and they corrupt each other ("codec mismatch", decode
     * timeout, then a lost heartbeat). Board-verified 2026-10-08: the
     * streaming session never recovers, not even after the second is closed.
     * tkl_codec_gate_internal.h holds the gate that keeps that from happening.
     *
     * That copy is a full frame per frame, executed inside the decoder's PP
     * ISR, and only a pull ever needs it - so it runs on demand: a pull arms
     * flexa_grab_want and waits for one whole frame, and an assembled frame
     * nobody takes disarms it again. An idle pipeline therefore pays nothing
     * (measured: 1280x720 display+h264+nv12 went 14.5 -> 19.9 fps). */
    UINT8_T *flexa_grab_buf;        /**< assembled NV12 frame, NULL when no ch1 */
    UINT32_T flexa_grab_bytes;      /**< width * height * 3 / 2 */
    UINT16_T flexa_grab_segs;       /**< segments per frame: height / flexa_lines */
    UINT16_T flexa_grab_seg_bytes;  /**< bytes per segment: lines * width * 3 / 2 */
    volatile BOOL_T flexa_grab_ready;   /**< a complete frame is in the buffer */
    volatile BOOL_T flexa_grab_want;    /**< a pull is waiting: assemble the next frame */
    volatile BOOL_T flexa_grab_live;    /**< assembling now; latched at the frame's first segment */
    beken_semaphore_t flexa_grab_sem;   /**< given when an assembled frame completes */
    UINT16_T src_width;         /**< decoder output feeding the bonds */
    UINT16_T src_height;

    tkl_uvc_pool_t stream_pool;
    tkl_uvc_pool_t frame_pool;  /**< parked JPEG: ch0 pull + ch1 on-demand decode */

    /* The internal decode task: pops the ready queue and feeds the flexa ring.
     * Only ch2/ch3 need it; with neither active it parks slots straight back -
     * see tkl_uvc_out_decode_task(). */
    beken_thread_t    task;
    beken_semaphore_t task_done;    /**< posted once, when the task exits */
    BOOL_T            task_live;
    volatile BOOL_T   task_stop;

    /* diagnostics, reported by tkl_uvc_deinit() */
    UINT32_T frames;
    UINT32_T drop_stream_empty;
    UINT32_T drop_too_big;
    UINT32_T drop_frame_empty;  /**< FRAME parking pool had no slot */
    UINT32_T decode_fail;
    UINT32_T decoded;

} tkl_uvc_out_ctx_t;

STATIC tkl_uvc_out_ctx_t s_uvc_out;

STATIC VOID_T tkl_uvc_out_frame_free(TUYA_MEDIA_BUF_T *frame);
STATIC OPERATE_RET tkl_uvc_out_task_start(VOID_T);
STATIC VOID tkl_uvc_out_decode_task(beken_thread_arg_t arg);
STATIC OPERATE_RET tkl_uvc_out_display_start(VOID_T);
STATIC OPERATE_RET tkl_uvc_out_encode_start(VOID_T);
STATIC VOID_T tkl_uvc_out_display_stop_one(VOID_T);
STATIC VOID_T tkl_uvc_out_encode_stop_one(VOID_T);
STATIC VOID_T tkl_uvc_out_consumers_close(VOID_T);

/**
 * @brief Find one configured output channel, NULL when not configured.
 *
 * @note The scan is over the deep copy init() made, so it stays valid for the
 *       adapter's lifetime; callers may then write back resolved geometry.
 */
STATIC TUYA_MEDIA_OUTPUT_CFG_T *tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    UINT8_T i;

    for (i = 0U; i < s_uvc_out.cfg.output_num; i++) {
        if (s_uvc_out.cfg.outputs[i].channel == channel) {
            return &s_uvc_out.cfg.outputs[i];
        }
    }

    return NULL;
}

STATIC VOID_T tkl_uvc_out_event(TUYA_CAMERA_EVENT_E event)
{
    if (s_uvc_out.cfg.event_cb != NULL) {
        s_uvc_out.cfg.event_cb(event, s_uvc_out.cfg.event_arg);
    }
}

/**
 * @brief Report a repeating fault at most once per second per event value.
 *
 * @note Overflow/timeout faults fire per frame while the condition lasts; a
 *       20 Hz log stream is itself enough load to starve the fault it reports.
 */
STATIC VOID_T tkl_uvc_out_event_throttled(TUYA_CAMERA_EVENT_E event)
{
    STATIC UINT32_T s_last_ms[32] = {0};
    UINT32_T now;

    if (event >= 32U) {
        return;
    }

    now = TUYA_UVC_INTERNAL_MS();
    if (s_last_ms[event] != 0U && now - s_last_ms[event] < 1000U) {
        return;
    }
    s_last_ms[event] = now;

    tkl_uvc_out_event(event);
}

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
    /* sized for the deepest pool this file builds; every depth is a constant */
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
/** Map one captured frame to its public content format; an H264 frame carries
 *  its frame type inside the format (TUYA_FRAME_FMT_H264_I / H264_P). */
STATIC TUYA_FRAME_FMT_E tkl_uvc_out_slot_fmt(CONST TUYA_VCODEC_STREAM_T *stream)
{
    if (stream->codec != (UINT32_T)TUYA_VCODEC_TYPE_H264) {
        return TUYA_FRAME_FMT_JPEG;
    }

    switch (stream->frame_type) {
    case TUYA_VCODEC_FRAME_I:
        return TUYA_FRAME_FMT_H264_I;
    case TUYA_VCODEC_FRAME_P:
        return TUYA_FRAME_FMT_H264_P;
    default:
        return TUYA_FRAME_FMT_H264;
    }
}

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
    slot->fmt = tkl_uvc_out_slot_fmt(stream);

    /* The parking pool serves ch0 (pull and push) and the on-demand ch1 decode:
     * one copy per captured frame, dropped - never queued behind - when the
     * pool is empty. That is the back pressure a pull API asks for. */
    if (s_uvc_out.frame_pool.slots != NULL) {
        TUYA_MEDIA_OUTPUT_CFG_T *ch0 = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_0);
        tkl_uvc_slot_t *park = tkl_uvc_pool_take(&s_uvc_out.frame_pool);

        if (park == NULL) {
            s_uvc_out.drop_frame_empty++;
            tkl_uvc_out_event_throttled(TUYA_CAMERA_EVENT_ERR_OVERFLOW_CH1);
        } else if (slot->len > park->capacity) {
            s_uvc_out.drop_too_big++;
            tkl_uvc_pool_give(&s_uvc_out.frame_pool, park);
        } else {
            os_memcpy(park->data, slot->data, slot->len);
            park->len = slot->len;
            park->pts = slot->pts;
            park->fmt = slot->fmt;

            /* ch0 push delivers here, on the vendor URB thread: the buffer is
             * a fresh one-block copy (struct + payload), so the slot goes back
             * to circulation right away and a slow consumer stalls nothing. */
            if (ch0 != NULL && ch0->cb != NULL) {
                TUYA_MEDIA_BUF_T *buf = (TUYA_MEDIA_BUF_T *)tkl_media_buf_alloc(
                    TKL_BUF_USE_STREAM,
                    (UINT32_T)sizeof(TUYA_MEDIA_BUF_T) + park->len,
                    TUYA_MEDIA_BUF_OWNER_UVC_FRAME);

                if (buf == NULL) {
                    tkl_uvc_out_event_throttled(TUYA_CAMERA_EVENT_ERR_ALLOC);
                } else {
                    os_memset(buf, 0, sizeof(*buf));
                    os_memcpy(buf + 1, park->data, park->len);
                    buf->buffer = (UINT8_T *)(buf + 1);
                    buf->length = park->len;
                    buf->free_cb = tkl_uvc_out_frame_free;
                    buf->channel = TUYA_CAMERA_FRAME_CHANNEL_0;
                    buf->format = park->fmt;
                    buf->timestamp = park->pts;
                    buf->width = s_uvc_out.cfg.uvc_cfg.width;
                    buf->height = s_uvc_out.cfg.uvc_cfg.height;
                    buf->user_args = ch0->arg;
                    ch0->cb(buf);
                }
            }

            if (rtos_push_to_queue(&s_uvc_out.frame_pool.ready_q, &park,
                                   BEKEN_NO_WAIT) != 0) {
                tkl_uvc_pool_give(&s_uvc_out.frame_pool, park);
            }
        }
    }

    /* ch3 from an H264 source is a pass-through: the bitstream goes to the
     * ch3 callback exactly as captured - no decoder and no encoder exist in
     * this configuration (want_encode stays FALSE). The buffer is borrowed for
     * the duration of the call - free_cb stays NULL. The slot is never marked
     * ready, so free() below recycles it instead of queueing it for the decode
     * task. */
    if (s_uvc_out.want_encode != TRUE) {
        TUYA_MEDIA_OUTPUT_CFG_T *ch3 = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_3);

        if (ch3 != NULL && ch3->cb != NULL) {
            TUYA_MEDIA_BUF_T buf;

            os_memset(&buf, 0, sizeof(buf));
            buf.buffer = slot->data;
            buf.length = slot->len;
            buf.free_cb = NULL;         /* borrowed for the duration of this call */
            buf.channel = TUYA_CAMERA_FRAME_CHANNEL_3;
            buf.timestamp = slot->pts;
            buf.width = s_uvc_out.cfg.uvc_cfg.width;
            buf.height = s_uvc_out.cfg.uvc_cfg.height;
            buf.format = slot->fmt;
            buf.user_args = ch3->arg;
            ch3->cb(&buf);
            return;
        }
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
     * one FRAME has already copied it out and it must recycle straight back, or the
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

    tkl_uvc_out_event(connected ? TUYA_CAMERA_EVENT_CONNECT
                                : TUYA_CAMERA_EVENT_DISCONNECT);
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
    VOID *base;

    (void)arg;

    /* One block = padded header + payload. The header carries the public buf
     * struct; the padding keeps the write-through payload 64-aligned (see
     * TUYA_UVC_BUF_HDR). */
    base = tkl_media_buf_alloc(TKL_BUF_USE_PIXEL_WT, TUYA_UVC_BUF_HDR + size,
                               TUYA_MEDIA_BUF_OWNER_UVC_DISPLAY);
    if (base == NULL) {
        tkl_uvc_out_event_throttled(TUYA_CAMERA_EVENT_ERR_ALLOC);
        return NULL;
    }

    return (UINT8_T *)base + TUYA_UVC_BUF_HDR;
}

STATIC OPERATE_RET tkl_uvc_out_display_free(VOID *ptr, VOID *arg)
{
    (void)arg;

    if (ptr == NULL) {
        return OPRT_INVALID_PARM;
    }

    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_UVC_DISPLAY,
                       (UINT8_T *)ptr - TUYA_UVC_BUF_HDR);
    return OPRT_OK;
}

/**
 * @brief Release a ch2 display buffer business kept past the callback.
 *
 * @param[in] frame the buf struct as delivered; display_done() already walked
 *                  back from the payload, so it sits at the block base and the
 *                  struct pointer IS the base.
 *
 * @note Subtracting the header again here freed `base - HDR`: bk_mem_slab_free
 *       then read a descriptor belonging to somebody else and hit HEAD_BACK
 *       (-4), asserting on the DPU completion path. Same shape as
 *       tkl_uvc_out_frame_free() and the camera facade's
 *       tkl_cam_out_display_buf_free().
 */
STATIC VOID_T tkl_uvc_out_display_buf_free(TUYA_MEDIA_BUF_T *frame)
{
    if (frame == NULL) {
        return;
    }

    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_UVC_DISPLAY, (UINT8_T *)frame);
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
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    TUYA_MEDIA_BUF_T *buf;

    (void)arg;

    if (frame == NULL) {
        return;
    }

    /* The struct lives in the header in front of the payload (see
     * tkl_uvc_out_display_alloc) - the business release walks back to the
     * block base from it. */
    buf = (TUYA_MEDIA_BUF_T *)((UINT8_T *)frame - TUYA_UVC_BUF_HDR);
    ch = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_2);

    if (ch == NULL || ch->cb == NULL) {
        tkl_uvc_out_display_free(frame, NULL);
        return;
    }

    os_memset(buf, 0, sizeof(*buf));
    buf->buffer = (UINT8_T *)frame;
    buf->length = frame_size;
    buf->free_cb = tkl_uvc_out_display_buf_free;
    buf->channel = TUYA_CAMERA_FRAME_CHANNEL_2;
    buf->timestamp = (UINT64_T)tkl_system_get_millisecond();
    buf->width = ch->width;
    buf->height = ch->height;
    buf->format = ch->format;
    buf->user_args = ch->arg;

    ch->cb(buf);
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
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    TUYA_MEDIA_BUF_T buf;

    (void)arg;

    if (stream == NULL || stream->data == NULL) {
        return;
    }

    ch = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_3);
    if (ch == NULL || ch->cb == NULL) {
        return;
    }

    os_memset(&buf, 0, sizeof(buf));
    buf.buffer = stream->data;
    buf.length = stream->size;
    buf.free_cb = NULL;             /* borrowed for the duration of this call */
    buf.channel = TUYA_CAMERA_FRAME_CHANNEL_3;
    buf.timestamp = stream->pts;
    buf.width = s_uvc_out.src_width;
    buf.height = s_uvc_out.src_height;
    buf.format = (stream->frame_type == TUYA_VCODEC_FRAME_I)
                     ? TUYA_FRAME_FMT_H264_I
                     : ((stream->frame_type == TUYA_VCODEC_FRAME_P)
                            ? TUYA_FRAME_FMT_H264_P : TUYA_FRAME_FMT_H264);
    buf.user_args = ch->arg;

    ch->cb(&buf);
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
    s_uvc_out.flexa_size = (UINT32_T)s_uvc_out.cfg.uvc_cfg.width *
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

    /* ch1 pull needs a whole frame assembled out of ring segments (see the
     * struct comment). Only when ch1 is configured; a failure is not fatal -
     * get_frame(ch1) then falls back to the on-demand decode. */
    if (s_uvc_out.want_sp == TRUE) {
        UINT32_T w = s_uvc_out.cfg.uvc_cfg.width;
        UINT32_T h = s_uvc_out.cfg.uvc_cfg.height;

        s_uvc_out.flexa_grab_segs =
            (UINT16_T)(h / TUYA_UVC_INTERNAL_FLEXA_LINES);
        s_uvc_out.flexa_grab_seg_bytes =
            (UINT16_T)(TUYA_UVC_INTERNAL_FLEXA_LINES * w * 3U / 2U);
        s_uvc_out.flexa_grab_bytes = w * h * 3U / 2U;
        s_uvc_out.flexa_grab_ready = FALSE;

        s_uvc_out.flexa_grab_buf =
            (UINT8_T *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL,
                                           s_uvc_out.flexa_grab_bytes,
                                           TUYA_MEDIA_BUF_OWNER_UVC_FRAME);
        if (s_uvc_out.flexa_grab_buf == NULL) {
            tkl_log_output("tkl_uvc_out: ch1 grab buffer %u bytes failed, "
                           "pull falls back to on-demand decode\r\n",
                           (unsigned)s_uvc_out.flexa_grab_bytes);
            s_uvc_out.flexa_grab_bytes = 0U;
            s_uvc_out.flexa_grab_segs = 0U;
        } else if (rtos_init_semaphore_ex(&s_uvc_out.flexa_grab_sem, 1, 0) != 0) {
            /* The pull waits on this for a whole frame; without it there is no
             * way to wait, so treat it like a failed allocation. */
            tkl_log_output("tkl_uvc_out: ch1 grab semaphore failed, "
                           "pull falls back to on-demand decode\r\n");
            tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_UVC_FRAME, s_uvc_out.flexa_grab_buf);
            s_uvc_out.flexa_grab_buf = NULL;
            s_uvc_out.flexa_grab_bytes = 0U;
            s_uvc_out.flexa_grab_segs = 0U;
        } else {
            s_uvc_out.flexa_grab_ready = FALSE;
            s_uvc_out.flexa_grab_want = FALSE;
            s_uvc_out.flexa_grab_live = FALSE;
            tkl_log_output("tkl_uvc_out: ch1 grab %u bytes, %u segments of %u\r\n",
                           (unsigned)s_uvc_out.flexa_grab_bytes,
                           (unsigned)s_uvc_out.flexa_grab_segs,
                           (unsigned)s_uvc_out.flexa_grab_seg_bytes);
        }
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

    if (s_uvc_out.flexa_grab_buf != NULL) {
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_UVC_FRAME, s_uvc_out.flexa_grab_buf);
        s_uvc_out.flexa_grab_buf = NULL;
    }
    s_uvc_out.flexa_grab_bytes = 0U;
    s_uvc_out.flexa_grab_segs = 0U;
    s_uvc_out.flexa_grab_ready = FALSE;
    s_uvc_out.flexa_grab_want = FALSE;
    s_uvc_out.flexa_grab_live = FALSE;
    if (s_uvc_out.flexa_grab_sem != NULL) {
        (void)rtos_deinit_semaphore(&s_uvc_out.flexa_grab_sem);
        s_uvc_out.flexa_grab_sem = NULL;
    }
}

/**
 * @brief Decoder flexa-done hook: record how far the current frame got.
 *
 * @param[in] wr_cnt segment just written; 1-based and reset every frame (the
 *                   h264e bond reads "1" as frame start and arms the encoder)
 * @param[in] arg    unused
 *
 * @note Read-pointer bookkeeping still belongs to the bonds - each issues
 *       BK_JPEG_DECODE_IOCTL_PORT_SET_RD_PTR for its own port as it consumes,
 *       so the write pointer is already held back by the slower of the two.
 *       This hook only *reads* the ring, never those registers.
 *
 *       The copy happens here, per segment, and not on the decode task: the
 *       ring is only FLEXA_DEPTH segments deep, so by the time a frame is
 *       complete its first segments have long been overwritten. Waiting for
 *       the whole frame needs a ring that holds one, and the vendor decoder
 *       refuses a segment count that large.
 */
STATIC VOID tkl_uvc_out_flexa_done_cb(UINT32_T wr_cnt, VOID *arg)
{
    UINT32_T idx;

    (void)arg;

    if (s_uvc_out.flexa_grab_buf == NULL || wr_cnt == 0U) {
        return;
    }

    if (wr_cnt == 1U) {
        /* Decide for the whole frame at its first segment - arming mid-frame
         * would splice two pictures into one buffer. An assembled frame that
         * nobody took means the puller has gone away, so disarm: that is what
         * keeps this copy off a pipeline no one is pulling from. */
        if (s_uvc_out.flexa_grab_ready == TRUE) {
            s_uvc_out.flexa_grab_want = FALSE;
        }
        s_uvc_out.flexa_grab_live = s_uvc_out.flexa_grab_want;
        s_uvc_out.flexa_grab_ready = FALSE;
    }

    if (s_uvc_out.flexa_grab_live != TRUE) {
        return;                                 /* nobody is pulling */
    }

    idx = wr_cnt - 1U;                          /* wr_cnt is 1-based */
    if (idx >= (UINT32_T)s_uvc_out.flexa_grab_segs) {
        return;                                 /* past this frame's segments */
    }

    os_memcpy(s_uvc_out.flexa_grab_buf + idx * (UINT32_T)s_uvc_out.flexa_grab_seg_bytes,
              s_uvc_out.flexa_ring +
                  (idx % (UINT32_T)TUYA_UVC_INTERNAL_FLEXA_DEPTH) *
                      (UINT32_T)s_uvc_out.flexa_grab_seg_bytes,
              (UINT32_T)s_uvc_out.flexa_grab_seg_bytes);

    /* the last segment closes the frame */
    if ((idx + 1U) == (UINT32_T)s_uvc_out.flexa_grab_segs) {
        s_uvc_out.flexa_grab_live = FALSE;
        s_uvc_out.flexa_grab_ready = TRUE;
        if (s_uvc_out.flexa_grab_sem != NULL) {
            (void)rtos_set_semaphore(&s_uvc_out.flexa_grab_sem);
        }
    }
}

/**
 * @brief Decoder FRAME-sink hook: copy the decoded NV12 and hand it to ch1.
 *
 * @param[in] frame decoded picture
 * @param[in] arg   unused
 *
 * @note Runs on the decode task (submit_stream is synchronous). vdec recycles
 *       the payload the moment this returns, so the frame is copied into a
 *       one-block buffer (struct + payload) before it leaves; a failed
 *       allocation drops the frame and throttles the overflow event.
 */
STATIC VOID tkl_uvc_out_frame_sink_cb(CONST TUYA_VCODEC_FRAME_T *frame, VOID *arg)
{
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    TUYA_MEDIA_BUF_T *buf;
    UINT32_T need;

    (void)arg;

    ch = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_1);
    if (frame == NULL || ch == NULL || ch->cb == NULL ||
        s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_1] != TRUE) {
        return;                     /* vdec recycles the payload after this */
    }

    /* Size by what is actually copied: frame->size is the decoder's own span
     * and rounds the height up to a whole 16-row unit (tkl_vdec's
     * aligned_height), while frame->height does not - so allocating
     * width x height x 3/2 and then copying frame->size bytes runs past the
     * block by width x (aligned_height - height) x 3/2, which is 15 KB for
     * 1280x1080. The source already carries the number, so no sizing rule is
     * re-derived here. */
    need = frame->size;
    buf = (TUYA_MEDIA_BUF_T *)tkl_media_buf_alloc(
        TKL_BUF_USE_PIXEL,
        (UINT32_T)sizeof(TUYA_MEDIA_BUF_T) + need,
        TUYA_MEDIA_BUF_OWNER_UVC_FRAME);
    if (buf == NULL) {
        tkl_uvc_out_event_throttled(TUYA_CAMERA_EVENT_ERR_OVERFLOW_CH1);
        return;
    }

    os_memset(buf, 0, sizeof(*buf));
    os_memcpy(buf + 1, frame->buffer, frame->size);
    buf->buffer = (UINT8_T *)(buf + 1);
    buf->length = frame->size;
    buf->free_cb = tkl_uvc_out_frame_free;
    buf->channel = TUYA_CAMERA_FRAME_CHANNEL_1;
    buf->format = TUYA_FRAME_FMT_NV12;
    buf->timestamp = frame->pts;
    buf->width = frame->width;
    buf->height = frame->height;
    buf->user_args = ch->arg;

    ch->cb(buf);
}

/**
 * @brief Open the decoder in FRAME-sink mode for the ch1 push configuration.
 *
 * @note No ring and no bonds: complete NV12 pictures leave through
 *       tkl_uvc_out_frame_sink_cb(). ch1 push requires ch2/ch3 absent (the
 *       flexa ring owns the decoder then) - init() enforces that.
 *
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_uvc_out_decoder_open_frame(VOID_T)
{
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    TUYA_VDEC_CFG_T cfg;
    TUYA_VDEC_SINK_T sink;
    OPERATE_RET ret;

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.codec = (UINT32_T)TUYA_VCODEC_TYPE_MJPEG;
    cfg.width = s_uvc_out.cfg.uvc_cfg.width;
    cfg.height = s_uvc_out.cfg.uvc_cfg.height;

    ret = tkl_vdec_init(&s_uvc_out.vdec, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: vdec init failed: %d\r\n", ret);
        return ret;
    }

    /* ch1's resolved geometry drives the post-processing size; init() already
     * rewrote a 0 to the source size, so a mismatch here is a real request. */
    ch = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_1);
    if (ch != NULL && (ch->width != cfg.width || ch->height != cfg.height)) {
        TUYA_VDEC_PP_T pp;

        os_memset(&pp, 0, sizeof(pp));
        pp.out_width = ch->width;
        pp.out_height = ch->height;
        pp.output_format = tkl_media_fmt_to_bk(TUYA_FRAME_FMT_NV12);
        ret = tkl_vdec_set_pp(s_uvc_out.vdec, &pp);
        if (ret != OPRT_OK) {
            tkl_log_output("tkl_uvc_out: vdec pp %ux%u failed: %d\r\n",
                           (unsigned)ch->width, (unsigned)ch->height, ret);
            goto err;
        }
    }

    os_memset(&sink, 0, sizeof(sink));
    sink.type = TUYA_VDEC_SINK_FRAME;
    sink.frame.frame_cb = tkl_uvc_out_frame_sink_cb;

    ret = tkl_vdec_set_sink(s_uvc_out.vdec, &sink);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: vdec frame sink failed: %d\r\n", ret);
        goto err;
    }

    ret = tkl_vdec_start(s_uvc_out.vdec);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: vdec start failed: %d\r\n", ret);
        goto err;
    }

    s_uvc_out.vdec_frame_sink = TRUE;
    return OPRT_OK;

err:
    (void)tkl_vdec_deinit(s_uvc_out.vdec);
    s_uvc_out.vdec = NULL;
    return ret;
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
    cfg.width = s_uvc_out.cfg.uvc_cfg.width;
    cfg.height = s_uvc_out.cfg.uvc_cfg.height;
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
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    OPERATE_RET ret;

    ch = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_2);
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
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    OPERATE_RET ret;

    ch = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_3);
    if (ch == NULL) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&venc_cfg, 0, sizeof(venc_cfg));
    venc_cfg.width = flexa->width;
    venc_cfg.height = flexa->height;
    venc_cfg.fps = s_uvc_out.cfg.uvc_cfg.fps;
    venc_cfg.gop = ch->gop;
    /* ch3.format is the encoded output format. This platform has no H265
     * encoder backend - init rejects that request - so the codec is always
     * H264 here. */
    venc_cfg.codec = (UINT32_T)TUYA_VCODEC_TYPE_H264;
    venc_cfg.input_format = flexa->format;

    ret = tkl_venc_init(&s_uvc_out.venc, &venc_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: venc init failed: %d\r\n", ret);
        return ret;
    }

    /* the ch3 preset decides the bitrate; tkl_uvc_set_bitrate() fine-tunes
     * from there while the stream is up */
    os_memset(&rc, 0, sizeof(rc));
    rc.bitrate = tkl_media_bitrate_from_level(ch->level);
    (void)tkl_venc_set_rate_ctrl(s_uvc_out.venc, &rc);

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
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    VOID *gpu_raw = NULL;
    OPERATE_RET ret;

    ret = tkl_gpu_get_handle(s_uvc_out.gpu, &gpu_raw);
    if (ret != OPRT_OK) {
        return ret;
    }

    ch = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_2);
    if (ch == NULL) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.target_w = ch->width;
    cfg.target_h = ch->height;
    /* the caller supplies the rotation per overlay request */
    cfg.src_format = (UINT32_T)BK_PIXEL_FORMAT_ABGR8888;

    return tkl_osd_engine_open(TKL_OSD_BACKEND_GPU, gpu_raw, &cfg, &s_uvc_out.osd_display);
}

/* ---------------------------------------------------------------------------
 * Single-consumer primitives - what start() builds and cancel/resume move.
 * Each start takes its hardware claim first and releases it on the way out,
 * so the CSI camera and the UVC camera can fight for the GPU / encoder
 * without corrupting either.
 * ------------------------------------------------------------------------- */

STATIC OPERATE_RET tkl_uvc_out_display_start(VOID_T)
{
    TUYA_VCODEC_FLEXA_BUFFER_T flexa;
    OPERATE_RET ret;

    ret = tkl_media_hw_claim(TUYA_MEDIA_HW_GPU, TUYA_MEDIA_HW_CLAIM_UVC);
    if (ret != OPRT_OK) {
        return ret;     /* the CSI camera side holds the GPU */
    }

    ret = tkl_vdec_get_flexa_source(s_uvc_out.vdec, &flexa);
    if (ret != OPRT_OK) {
        goto err;
    }

    ret = tkl_uvc_out_display_open(&flexa);
    if (ret != OPRT_OK) {
        goto err;
    }

    ret = tkl_gpu_start(s_uvc_out.gpu);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: gpu start failed: %d\r\n", ret);
        (void)tkl_gpu_deinit(s_uvc_out.gpu);
        s_uvc_out.gpu = NULL;
        goto err;
    }

    /* overlays last: they bind to a backend that is already running */
    (void)tkl_uvc_out_osd_display_open();   /* optional, never fatal */
    return OPRT_OK;

err:
    tkl_media_hw_release(TUYA_MEDIA_HW_GPU, TUYA_MEDIA_HW_CLAIM_UVC);
    return ret;
}

STATIC OPERATE_RET tkl_uvc_out_encode_start(VOID_T)
{
    TUYA_VCODEC_FLEXA_BUFFER_T flexa;
    OPERATE_RET ret;

    ret = tkl_media_hw_claim(TUYA_MEDIA_HW_VPU_ENC, TUYA_MEDIA_HW_CLAIM_UVC);
    if (ret != OPRT_OK) {
        return ret;     /* the CSI camera side holds the encoder */
    }

    ret = tkl_vdec_get_flexa_source(s_uvc_out.vdec, &flexa);
    if (ret != OPRT_OK) {
        goto err;
    }

    ret = tkl_uvc_out_encode_open(&flexa);
    if (ret != OPRT_OK) {
        goto err;
    }

    ret = tkl_venc_start(s_uvc_out.venc);
    if (ret != OPRT_OK) {
        tkl_log_output("tkl_uvc_out: venc start failed: %d\r\n", ret);
        (void)tkl_venc_deinit(s_uvc_out.venc);
        s_uvc_out.venc = NULL;
        goto err;
    }

    return OPRT_OK;

err:
    tkl_media_hw_release(TUYA_MEDIA_HW_VPU_ENC, TUYA_MEDIA_HW_CLAIM_UVC);
    return ret;
}

STATIC VOID_T tkl_uvc_out_display_stop_one(VOID_T)
{
    BOOL_T had_gpu = (s_uvc_out.gpu != NULL);

    if (s_uvc_out.osd_display != NULL) {
        (void)tkl_osd_engine_close(s_uvc_out.osd_display);
        s_uvc_out.osd_display = NULL;
    }
    if (s_uvc_out.gpu != NULL) {
        (void)tkl_gpu_stop(s_uvc_out.gpu);
        (void)tkl_gpu_deinit(s_uvc_out.gpu);
        s_uvc_out.gpu = NULL;
    }
    if (had_gpu == TRUE) {
        tkl_media_hw_release(TUYA_MEDIA_HW_GPU, TUYA_MEDIA_HW_CLAIM_UVC);
    }
}

STATIC VOID_T tkl_uvc_out_encode_stop_one(VOID_T)
{
    BOOL_T had_venc = (s_uvc_out.venc != NULL);

    if (s_uvc_out.venc != NULL) {
        (void)tkl_venc_stop(s_uvc_out.venc);
        (void)tkl_venc_deinit(s_uvc_out.venc);
        s_uvc_out.venc = NULL;
    }
    if (had_venc == TRUE) {
        tkl_media_hw_release(TUYA_MEDIA_HW_VPU_ENC, TUYA_MEDIA_HW_CLAIM_UVC);
    }
}

STATIC VOID_T tkl_uvc_out_consumers_close(VOID_T)
{
    /* every bond stops before any backend is touched - the iron rule. */
    tkl_uvc_out_encode_stop_one();
    tkl_uvc_out_display_stop_one();
}

/**
 * @brief Build the decoder ring and every configured push consumer.
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET tkl_uvc_out_push_open(VOID_T)
{
    TUYA_VCODEC_FLEXA_BUFFER_T flexa;
    BOOL_T display_up = FALSE;
    OPERATE_RET ret;

    ret = tkl_uvc_out_decoder_open_flexa(&flexa);
    if (ret != OPRT_OK) {
        return ret;
    }

    /* consumers first, bonds after - a bond must never start against a backend
     * that is not fully built. Each *_start claims its hardware and releases
     * it again on its own failure. */
    if (s_uvc_out.want_display == TRUE) {
        ret = tkl_uvc_out_display_start();
        if (ret != OPRT_OK) {
            goto err_vdec;
        }
        display_up = TRUE;
    }

    if (s_uvc_out.want_encode == TRUE) {
        ret = tkl_uvc_out_encode_start();
        if (ret != OPRT_OK) {
            if (display_up == TRUE) {
                tkl_uvc_out_display_stop_one();
            }
            goto err_vdec;
        }
    }

    /* The decode task consumes the capture queue into the decoder. It must
     * exist before the camera starts producing - tkl_uvc_start() starts the
     * camera last. Failure is fatal: the caller's err path runs push_close(),
     * which also retires a half-started task. */
    ret = tkl_uvc_out_task_start();
    if (ret != OPRT_OK) {
        tkl_uvc_out_consumers_close();
        goto err_vdec;
    }

    return OPRT_OK;

err_vdec:
    (void)tkl_vdec_deinit(s_uvc_out.vdec);
    s_uvc_out.vdec = NULL;
    tkl_uvc_out_flexa_ring_free();
    return ret;
}

/**
 * @brief Tear down the decode task, the push consumers and the decoder.
 *
 * @note Iron rule: every bond stops before any backend is touched - the
 *       consumers_close order keeps it, and the decoder goes last of all.
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

    tkl_uvc_out_consumers_close();

    if (s_uvc_out.vdec != NULL) {
        (void)tkl_vdec_deinit(s_uvc_out.vdec);
        s_uvc_out.vdec = NULL;
    }
    tkl_uvc_out_flexa_ring_free();
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
    TKL_UVC_CFG_T cfg;
    TUYA_UVC_SINK_T sink;
    OPERATE_RET ret;

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.port = s_uvc_out.cfg.port;
    cfg.width = s_uvc_out.cfg.uvc_cfg.width;
    cfg.height = s_uvc_out.cfg.uvc_cfg.height;
    cfg.fps = s_uvc_out.cfg.uvc_cfg.fps;
    /* format 0 (UNKNOWN) keeps the adapter default: MJPEG. */
    cfg.input_format = (s_uvc_out.cfg.uvc_cfg.format == TUYA_FRAME_FMT_H264)
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

/* ---------------------------------------------------------------------------
 * FRAME output - the pull half
 * ------------------------------------------------------------------------- */

/**
 * @brief Copy one USB descriptor string, or "" when it is not there.
 */
STATIC VOID tkl_uvc_info_copy_str(CHAR_T *dst, UINT32_T cap, CONST CHAR_T *src)
{
    UINT32_T i = 0U;

    if (dst == NULL || cap == 0U) {
        return;
    }
    dst[0] = '\0';
    if (src == NULL) {
        return;
    }
    while (i + 1U < cap && src[i] != '\0') {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

/*
 * Same one-block shape as the MIPI camera's get_frame(): the returned buf
 * struct and its frame come out of one slab allocation, the frame right behind
 * the struct, so the release walks back to the block base and frees both with
 * it. The caller only ever calls free_cb.
 */
STATIC VOID_T tkl_uvc_out_frame_free(TUYA_MEDIA_BUF_T *frame)
{
    if (frame == NULL) {
        return;
    }
    /* ch0/ch1 blocks are one allocation: struct at the base, payload right
     * behind it - so the struct pointer IS the block base. */
    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_UVC_FRAME, (UINT8_T *)frame);
}

/**
 * @brief Decode one parked MJPEG into a fresh NV12 buffer for get_frame(ch1).
 *
 * @param[in]  slot parked JPEG frame; returned to its pool by the caller
 * @param[out] out  receives the one-block buffer (struct + payload)
 *
 * @note Reached only when the flexa grab buffer is unavailable; the flexa path
 *       itself never comes here (see the flexa_grab_buf note in the state
 *       struct). A second decoder session next to a running one is NOT safe -
 *       board-verified 2026-10-08: it logs "dec isr codec mismatch", then both
 *       sessions time out forever and the streaming one never recovers, even
 *       after the second is closed. The gate in tkl_codec_gate_internal.h is what
 *       enforces that now: this call fails cleanly (or runs on the software
 *       decoder, once one is wired in) rather than reaching the hardware.
 *       Output geometry is the ch1 entry's, resolved at init().
 *
 * @return OPRT_OK on success; the buffer is released again on any error
 */
STATIC OPERATE_RET tkl_uvc_out_decode_parked(tkl_uvc_slot_t *slot, TUYA_MEDIA_BUF_T **out)
{
    TUYA_IMAGE_CODEC_BUF_T in;
    TUYA_IMAGE_CODEC_BUF_T dec;
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    TUYA_MEDIA_BUF_T *buf;
    UINT32_T need;
    OPERATE_RET ret;

    ch = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_1);
    if (ch == NULL || ch->width == 0U || ch->height == 0U) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* Ask the storage rule instead of deriving it: NV12 rounds the height up to
     * a whole number of 32-row units (the UV plane is height/2 rows of 16-row
     * DMA segments), so width x height x 3/2 undersizes every geometry whose
     * height is not already a multiple of 32 - 1280x720 needs 1280x736 worth of
     * bytes and came back OPRT_INVALID_PARM from the decoder. Same rule
     * tkl_image_codec sizes its own outputs by. */
    need = tkl_pp_output_size(tkl_media_fmt_to_bk(TUYA_FRAME_FMT_NV12),
                              ch->width, ch->height);
    if (need == 0U) {
        return OPRT_RESOURCE_NOT_READY;
    }
    buf = (TUYA_MEDIA_BUF_T *)tkl_media_buf_alloc(
        TKL_BUF_USE_PIXEL,
        (UINT32_T)sizeof(TUYA_MEDIA_BUF_T) + need,
        TUYA_MEDIA_BUF_OWNER_UVC_FRAME);
    if (buf == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    os_memset(buf, 0, sizeof(*buf));

    os_memset(&in, 0, sizeof(in));
    in.buffer = slot->data;
    in.length = slot->len;
    in.format = TUYA_FRAME_FMT_JPEG;
    in.width = s_uvc_out.cfg.uvc_cfg.width;
    in.height = s_uvc_out.cfg.uvc_cfg.height;

    os_memset(&dec, 0, sizeof(dec));
    dec.buffer = (UINT8_T *)(buf + 1);
    dec.length = need;
    dec.format = TUYA_FRAME_FMT_NV12;
    dec.width = ch->width;
    dec.height = ch->height;

    ret = tkl_image_codec_convert(&in, &dec);
    if (ret != OPRT_OK) {
        tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_UVC_FRAME, (UINT8_T *)buf);
        return ret;
    }

    buf->buffer = (UINT8_T *)(buf + 1);
    buf->length = dec.length;
    buf->free_cb = tkl_uvc_out_frame_free;
    buf->channel = TUYA_CAMERA_FRAME_CHANNEL_1;
    buf->format = TUYA_FRAME_FMT_NV12;
    buf->timestamp = slot->pts;
    buf->width = dec.width;
    buf->height = dec.height;
    buf->user_args = ch->arg;

    *out = buf;
    return OPRT_OK;
}

/**
 * @brief Pull one frame from a pull channel: ch0 (parked JPEG as captured) or
 *        ch1 (parked MJPEG decoded to NV12 on demand).
 *
 * @param[in]  channel    ch0 or ch1
 * @param[out] out        the returned buf; release via buf->free_cb(buf)
 * @param[in]  timeout_ms how long to wait for a frame; 0 = return immediately
 *
 * @note Frame data is copied out of the parking slot, so business may hold the
 *       buffer arbitrarily long - the slot returns to the pool the moment the
 *       copy is done and capture never stalls on this API.
 *
 * @return OPRT_OK on success; OPRT_TIMEOUT when no frame arrived in time;
 *         OPRT_NOT_SUPPORTED for the push channels or a channel configured
 *         with cb; OPRT_RESOURCE_NOT_READY when the channel was not configured
 */
OPERATE_RET tkl_uvc_get_frame(TUYA_CAMERA_FRAME_CHANNEL_E channel,
                              TUYA_MEDIA_BUF_T **out, UINT32_T timeout_ms)
{
    TUYA_MEDIA_BUF_T *buf;
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    tkl_uvc_slot_t *slot = NULL;
    OPERATE_RET ret;

    if (out == NULL) {
        return OPRT_INVALID_PARM;
    }
    *out = NULL;

    if (channel != TUYA_CAMERA_FRAME_CHANNEL_0 &&
        channel != TUYA_CAMERA_FRAME_CHANNEL_1) {
        return OPRT_NOT_SUPPORTED;      /* ch2/ch3 are push-only */
    }

    ch = tkl_uvc_out_ch(channel);
    if (ch == NULL) {
        return OPRT_RESOURCE_NOT_READY; /* not configured at init() */
    }
    if (ch->cb != NULL) {
        return OPRT_NOT_SUPPORTED;      /* push-only channel */
    }
    if (s_uvc_out.state != TUYA_UVC_ST_STARTED ||
        s_uvc_out.ch_active[channel] != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* ch1 while the flexa pipeline runs: hand back a copy of the assembled
     * frame - no parked JPEG and, more to the point, no second decoder session
     * (see tkl_uvc_out_flexa_done_cb). Assembly is on demand, so arm it and
     * wait for one whole frame when nothing is ready yet. */
    if (channel == TUYA_CAMERA_FRAME_CHANNEL_1 &&
        s_uvc_out.flexa_grab_buf != NULL) {
        UINT32_T block_size;

        if (s_uvc_out.flexa_grab_ready != TRUE) {
            /* Drop a token left by a frame nobody waited for, then arm and
             * wait for the frame the assembler is about to produce. */
            if (s_uvc_out.flexa_grab_sem != NULL) {
                (void)rtos_get_semaphore(&s_uvc_out.flexa_grab_sem, BEKEN_NO_WAIT);
            }
            s_uvc_out.flexa_grab_want = TRUE;
            if (s_uvc_out.flexa_grab_sem != NULL) {
                (void)rtos_get_semaphore(&s_uvc_out.flexa_grab_sem, timeout_ms);
            }
        }
        if (s_uvc_out.flexa_grab_ready != TRUE) {
            return OPRT_TIMEOUT;                /* no whole frame within timeout_ms */
        }
        s_uvc_out.flexa_grab_ready = FALSE;     /* take it */
        s_uvc_out.flexa_grab_want = TRUE;       /* and keep one coming */

        block_size = (UINT32_T)sizeof(TUYA_MEDIA_BUF_T) + s_uvc_out.flexa_grab_bytes;
        buf = (TUYA_MEDIA_BUF_T *)tkl_media_buf_alloc(TKL_BUF_USE_STREAM, block_size,
                                                      TUYA_MEDIA_BUF_OWNER_UVC_FRAME);
        if (buf == NULL) {
            tkl_uvc_out_event_throttled(TUYA_CAMERA_EVENT_ERR_ALLOC);
            return OPRT_MALLOC_FAILED;
        }

        os_memset(buf, 0, sizeof(*buf));
        os_memcpy(buf + 1, s_uvc_out.flexa_grab_buf, s_uvc_out.flexa_grab_bytes);
        buf->buffer = (UINT8_T *)(buf + 1);
        buf->length = s_uvc_out.flexa_grab_bytes;
        buf->free_cb = tkl_uvc_out_frame_free;
        buf->channel = TUYA_CAMERA_FRAME_CHANNEL_1;
        buf->timestamp = (UINT64_T)tkl_system_get_millisecond();
        buf->width = s_uvc_out.cfg.uvc_cfg.width;
        buf->height = s_uvc_out.cfg.uvc_cfg.height;
        buf->format = TUYA_FRAME_FMT_NV12;

        *out = buf;
        return OPRT_OK;
    }

    if (rtos_pop_from_queue(&s_uvc_out.frame_pool.ready_q, &slot,
                            timeout_ms) != 0 || slot == NULL) {
        return OPRT_TIMEOUT;
    }

    if (channel == TUYA_CAMERA_FRAME_CHANNEL_1) {
        ret = tkl_uvc_out_decode_parked(slot, &buf);
        tkl_uvc_pool_give(&s_uvc_out.frame_pool, slot);
        if (ret != OPRT_OK) {
            return ret;
        }
    } else {
        UINT32_T block_size = (UINT32_T)sizeof(TUYA_MEDIA_BUF_T) + slot->len;

        buf = (TUYA_MEDIA_BUF_T *)tkl_media_buf_alloc(TKL_BUF_USE_STREAM, block_size,
                                                      TUYA_MEDIA_BUF_OWNER_UVC_FRAME);
        if (buf == NULL) {
            tkl_uvc_pool_give(&s_uvc_out.frame_pool, slot);
            tkl_uvc_out_event_throttled(TUYA_CAMERA_EVENT_ERR_ALLOC);
            return OPRT_MALLOC_FAILED;
        }

        os_memset(buf, 0, sizeof(*buf));
        os_memcpy(buf + 1, slot->data, slot->len);
        buf->buffer = (UINT8_T *)(buf + 1);
        buf->length = slot->len;
        buf->free_cb = tkl_uvc_out_frame_free;
        buf->channel = TUYA_CAMERA_FRAME_CHANNEL_0;
        buf->timestamp = slot->pts;
        buf->width = s_uvc_out.cfg.uvc_cfg.width;
        buf->height = s_uvc_out.cfg.uvc_cfg.height;
        buf->format = slot->fmt;

        tkl_uvc_pool_give(&s_uvc_out.frame_pool, slot);
    }

    *out = buf;
    return OPRT_OK;
}

/* ---------------------------------------------------------------------------
 * Enumeration snapshot - get_info()
 *
 * Reads what the USB host already knows about the plugged camera: the hub
 * port's device descriptor and strings, plus the video class instance's
 * format/frame lists. Pure query - the port is powered for the lookup and
 * powered down again; init() pays its own enumeration.
 * ------------------------------------------------------------------------- */

/**
 * @brief Fill @p info from the enumerated camera on the default hub port.
 *
 * @note /dev/videoN is the devname the CherryUSB video class registers
 *       (DEV_FORMAT in usbh_video.c); the instance whose hub port matches the
 *       UVC port wins, with the first registered one as fallback.
 */
OPERATE_RET tkl_uvc_get_info(TUYA_UVC_INFO_T *info)
{
    bk_usb_hub_port_info *port_info = NULL;
    struct usbh_video *video = NULL;
    struct usbh_hubport *hport;
    UINT32_T waited = 0U;
    UINT8_T port = TUYA_UVC_DEFAULT_PORT;
    UINT8_T i;
    UINT8_T f;
    UINT8_T j;

    if (info == NULL) {
        return OPRT_INVALID_PARM;
    }

    os_memset(info, 0, sizeof(*info));

    bk_usbh_hub_multiple_devices_power_on(USB_HOST_MODE, port, USB_UVC_DEVICE);
    bk_usbh_hub_multiple_devices_power_on(USB_HOST_MODE, port, USB_UVC_H26X_DEVICE);

    /* Wait for enumeration, UVC first then the H26X flavor. */
    for (;;) {
        if (bk_usbh_hub_port_check_device(port, USB_UVC_DEVICE, &port_info) == AVDK_ERR_OK &&
            port_info != NULL) {
            break;
        }
        if (bk_usbh_hub_port_check_device(port, USB_UVC_H26X_DEVICE, &port_info) == AVDK_ERR_OK &&
            port_info != NULL) {
            break;
        }
        if (waited >= TUYA_UVC_DEFAULT_CONNECT_TIMEOUT_MS) {
            break;
        }
        rtos_delay_milliseconds(TUYA_UVC_INFO_POLL_MS);
        waited += TUYA_UVC_INFO_POLL_MS;
    }

    if (port_info == NULL) {
        bk_usbh_hub_port_register_connect_callback(port, USB_UVC_DEVICE, NULL, NULL);
        bk_usbh_hub_multiple_devices_power_down(USB_HOST_MODE, port, USB_UVC_DEVICE);
        bk_usbh_hub_port_register_connect_callback(port, USB_UVC_H26X_DEVICE, NULL, NULL);
        bk_usbh_hub_multiple_devices_power_down(USB_HOST_MODE, port, USB_UVC_H26X_DEVICE);
        return OPRT_RESOURCE_NOT_READY;
    }

    hport = port_info->hport;
    if (hport != NULL) {
        info->vid = hport->device_desc.idVendor;
        info->pid = hport->device_desc.idProduct;
        info->bcd_usb = hport->device_desc.bcdUSB;
        info->bcd_device = hport->device_desc.bcdDevice;
        info->speed = hport->speed;
        tkl_uvc_info_copy_str(info->manufacturer, sizeof(info->manufacturer),
                              hport->iManufacturer);
        tkl_uvc_info_copy_str(info->product, sizeof(info->product), hport->iProduct);
        tkl_uvc_info_copy_str(info->serial, sizeof(info->serial), hport->iSerialNumber);
    }

    for (i = 0U; i < TUYA_UVC_INFO_DEV_MAX && video == NULL; i++) {
        CHAR_T devname[16] = "/dev/video0";
        struct usbh_video *candidate;

        devname[10] = (CHAR_T)('0' + i);
        candidate = (struct usbh_video *)usbh_find_class_instance(devname);
        if (candidate == NULL) {
            continue;
        }
        if (candidate->hport == hport) {
            video = candidate;
        } else if (video == NULL) {
            video = candidate;      /* fallback: the only camera around */
        }
    }

    if (video != NULL) {
        info->ctrl_intf = video->ctrl_intf;
        info->data_intf = video->data_intf;
        info->bcd_uvc = video->bcdVDC;
#if CONFIG_BK_USB_CHERRYUSB_V1_6
        info->is_bulk = (video->is_bulk == true) ? TRUE : FALSE;
        if (info->is_bulk == TRUE && video->bulkin != NULL) {
            info->ep_mps = video->bulkin->wMaxPacketSize;
        } else if (video->isoin != NULL) {
            info->ep_mps = video->isoin->wMaxPacketSize;
        }
#endif

        for (i = 0U; i < video->num_of_formats; i++) {
            TUYA_UVC_FORMAT_INFO_T *dst;
            UINT8_T n;

            if (info->format_num >= (UINT8_T)(sizeof(info->formats) / sizeof(info->formats[0]))) {
                break;
            }
            if (video->format[i].format_type != USBH_VIDEO_FORMAT_MJPEG &&
                video->format[i].format_type != USBH_VIDEO_FORMAT_H264) {
                continue;       /* uncompressed etc. - not this API's vocabulary */
            }

            dst = &info->formats[info->format_num];
            dst->format = (video->format[i].format_type == USBH_VIDEO_FORMAT_MJPEG)
                              ? TUYA_FRAME_FMT_JPEG : TUYA_FRAME_FMT_H264;

            n = video->format[i].num_of_frames;
            if (n > (UINT8_T)(sizeof(dst->frames) / sizeof(dst->frames[0]))) {
                n = (UINT8_T)(sizeof(dst->frames) / sizeof(dst->frames[0]));
            }
            for (f = 0U; f < n; f++) {
                TUYA_UVC_FRAME_INFO_T *fr = &dst->frames[f];
                const struct usbh_video_resolution *res = &video->format[i].frame[f];
                UINT8_T k;

                fr->width = res->wWidth;
                fr->height = res->wHeight;
                k = res->fps_num;
                if (k > (UINT8_T)(sizeof(fr->fps) / sizeof(fr->fps[0]))) {
                    k = (UINT8_T)(sizeof(fr->fps) / sizeof(fr->fps[0]));
                }
                fr->fps_num = k;
                for (j = 0U; j < k && res->fps != NULL; j++) {
                    fr->fps[j] = (UINT16_T)res->fps[j];
                }
            }
            dst->frame_num = n;
            info->format_num++;
        }
    }

    bk_usbh_hub_port_register_connect_callback(port, USB_UVC_DEVICE, NULL, NULL);
    bk_usbh_hub_multiple_devices_power_down(USB_HOST_MODE, port, USB_UVC_DEVICE);
    bk_usbh_hub_port_register_connect_callback(port, USB_UVC_H26X_DEVICE, NULL, NULL);
    bk_usbh_hub_multiple_devices_power_down(USB_HOST_MODE, port, USB_UVC_H26X_DEVICE);

    return OPRT_OK;
}

/* Defined below, next to the pool code they belong to; declared here because
 * tkl_uvc_stop() and tkl_uvc_deinit() sit above them in lifecycle order, which
 * is the order a reader wants. */
STATIC UINT32_T tkl_uvc_out_flush_ready(tkl_uvc_pool_t *pool);
STATIC VOID_T tkl_uvc_out_retire_pool(tkl_uvc_pool_t *pool,
                                      TUYA_MEDIA_BUF_OWNER_E owner,
                                      CONST CHAR_T *name);

/**
 * @brief Check every output request against the ch0..ch3 topology rules.
 *
 * @note MJPEG feeds every channel: ch0 parks the JPEG, ch1 is decoded NV12,
 *       ch2/ch3 run the internal decoder. An H264 source cannot be decoded,
 *       so it has exactly one output - ch3, a pass-through to its cb.
 *
 * @return OPRT_OK, or the rejection reason with a log line
 */
STATIC OPERATE_RET tkl_uvc_out_validate(CONST TUYA_UVC_CFG_T *cfg)
{
    BOOL_T seen[TUYA_CAMERA_FRAME_CHANNEL_4] = {FALSE, FALSE, FALSE, FALSE};
    BOOL_T ch1_push = FALSE;
    BOOL_T is_h264 = (cfg->uvc_cfg.format == TUYA_FRAME_FMT_H264);
    UINT8_T i;

    if (cfg->outputs == NULL || cfg->output_num == 0U) {
        tkl_log_output("tkl_uvc_out: no output configured\r\n");
        return OPRT_INVALID_PARM;
    }

    for (i = 0U; i < cfg->output_num; i++) {
        CONST TUYA_MEDIA_OUTPUT_CFG_T *o = &cfg->outputs[i];

        if (o->channel >= TUYA_CAMERA_FRAME_CHANNEL_4) {
            tkl_log_output("tkl_uvc_out: channel %u out of range\r\n",
                           (unsigned)o->channel);
            return OPRT_INVALID_PARM;
        }
        if (seen[o->channel] == TRUE) {
            tkl_log_output("tkl_uvc_out: channel %u configured twice\r\n",
                           (unsigned)o->channel);
            return OPRT_INVALID_PARM;
        }
        seen[o->channel] = TRUE;

        switch (o->channel) {
        case TUYA_CAMERA_FRAME_CHANNEL_0:
            /* parked JPEG as captured - geometry comes from the USB negotiation */
            if (o->rotate != 0U || o->gop != 0U || o->level != 0U) {
                tkl_log_output("tkl_uvc_out: ch0 takes no rotate/gop/level\r\n");
                return OPRT_INVALID_PARM;
            }
            break;

        case TUYA_CAMERA_FRAME_CHANNEL_1:
            /* decoded NV12; geometry 0 = source size. cb is checked against
             * ch2/ch3 after the scan. */
            if (o->rotate != 0U || o->gop != 0U || o->level != 0U) {
                tkl_log_output("tkl_uvc_out: ch1 takes no rotate/gop/level\r\n");
                return OPRT_INVALID_PARM;
            }
            if (o->cb != NULL) {
                ch1_push = TRUE;
            }
            break;

        case TUYA_CAMERA_FRAME_CHANNEL_2:
            if (o->cb == NULL) {
                tkl_log_output("tkl_uvc_out: ch2 is push-only, cb required\r\n");
                return OPRT_INVALID_PARM;
            }
            if (o->width == 0U || o->height == 0U ||
                o->format == TUYA_FRAME_FMT_UNKNOWN) {
                tkl_log_output("tkl_uvc_out: ch2 needs size and format\r\n");
                return OPRT_INVALID_PARM;
            }
            if (o->rotate != 0U && o->rotate != 90U && o->rotate != 270U) {
                tkl_log_output("tkl_uvc_out: ch2 rotate %u unsupported (0/90/270)\r\n",
                               (unsigned)o->rotate);
                return OPRT_INVALID_PARM;
            }
            if (o->gop != 0U || o->level != 0U) {
                tkl_log_output("tkl_uvc_out: ch2 takes no gop/level\r\n");
                return OPRT_INVALID_PARM;
            }
            break;

        case TUYA_CAMERA_FRAME_CHANNEL_3:
            if (o->cb == NULL) {
                tkl_log_output("tkl_uvc_out: ch3 is push-only, cb required\r\n");
                return OPRT_INVALID_PARM;
            }
            if (o->width == 0U || o->height == 0U) {
                tkl_log_output("tkl_uvc_out: ch3 needs width/height\r\n");
                return OPRT_INVALID_PARM;
            }
            if (o->format != TUYA_FRAME_FMT_UNKNOWN &&
                o->format != TUYA_FRAME_FMT_H264) {
                tkl_log_output("tkl_uvc_out: ch3 format 0x%x not supported "
                               "(H264-only encoder)\r\n", (unsigned)o->format);
                return OPRT_NOT_SUPPORTED;
            }
            if (o->rotate != 0U) {
                tkl_log_output("tkl_uvc_out: ch3 takes no rotate\r\n");
                return OPRT_INVALID_PARM;
            }
            break;

        default:
            return OPRT_INVALID_PARM;
        }
    }

    /* H264 source: ch3 passthrough is the only output it can serve */
    if (is_h264 == TRUE) {
        if (seen[TUYA_CAMERA_FRAME_CHANNEL_0] == TRUE ||
            seen[TUYA_CAMERA_FRAME_CHANNEL_1] == TRUE ||
            seen[TUYA_CAMERA_FRAME_CHANNEL_2] == TRUE) {
            tkl_log_output("tkl_uvc_out: H264 source delivers via ch3 only\r\n");
            return OPRT_NOT_SUPPORTED;
        }
        if (seen[TUYA_CAMERA_FRAME_CHANNEL_3] != TRUE) {
            tkl_log_output("tkl_uvc_out: H264 pass-through needs ch3 + cb\r\n");
            return OPRT_INVALID_PARM;
        }
    }

    /* ch1 push runs the vdec FRAME sink; the flexa ring owns the decoder
     * whenever ch2/ch3 exist, and a decoder has one sink. */
    if (ch1_push == TRUE &&
        (seen[TUYA_CAMERA_FRAME_CHANNEL_2] == TRUE ||
         seen[TUYA_CAMERA_FRAME_CHANNEL_3] == TRUE)) {
        tkl_log_output("tkl_uvc_out: ch1 cb conflicts with ch2/ch3 (one vdec sink)\r\n");
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_uvc_init(CONST TUYA_UVC_CFG_T *cfg)
{
    TUYA_MEDIA_OUTPUT_CFG_T *ch1;
    TUYA_MEDIA_OUTPUT_CFG_T *outputs_copy;
    UINT8_T depth;
    UINT8_T i;
    UINT32_T ms_pool;
    UINT32_T ms_device;
    UINT32_T t;
    OPERATE_RET ret;

    if (cfg == NULL || cfg->uvc_cfg.width == 0U || cfg->uvc_cfg.height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (s_uvc_out.state != TUYA_UVC_ST_IDLE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_uvc_out_validate(cfg);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&s_uvc_out, 0, sizeof(s_uvc_out));
    os_memcpy(&s_uvc_out.cfg, cfg, sizeof(s_uvc_out.cfg));

    /* Deep-copy the outputs so the caller may release its cfg right away. */
    outputs_copy = (TUYA_MEDIA_OUTPUT_CFG_T *)tkl_system_malloc(
        (UINT32_T)sizeof(TUYA_MEDIA_OUTPUT_CFG_T) * cfg->output_num);
    if (outputs_copy == NULL) {
        os_memset(&s_uvc_out, 0, sizeof(s_uvc_out));
        return OPRT_MALLOC_FAILED;
    }
    os_memcpy(outputs_copy, cfg->outputs,
              (UINT32_T)sizeof(TUYA_MEDIA_OUTPUT_CFG_T) * cfg->output_num);
    s_uvc_out.cfg.outputs = outputs_copy;

    /* resolve the channel view once */
    for (i = 0U; i < cfg->output_num; i++) {
        TUYA_MEDIA_OUTPUT_CFG_T *o = &outputs_copy[i];

        s_uvc_out.ch_active[o->channel] = TRUE;
        if (o->channel == TUYA_CAMERA_FRAME_CHANNEL_1) {
            s_uvc_out.want_sp = TRUE;
        } else if (o->channel == TUYA_CAMERA_FRAME_CHANNEL_2) {
            s_uvc_out.want_display = TRUE;
        } else if (o->channel == TUYA_CAMERA_FRAME_CHANNEL_3 &&
                   cfg->uvc_cfg.format != TUYA_FRAME_FMT_H264) {
            /* ch3 on an MJPEG source runs the transcoder; on an H264 source it
             * is the pass-through and builds no encoder. */
            s_uvc_out.want_encode = TRUE;
        }
    }

    /* ch1 geometry 0 = source size */
    ch1 = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_1);
    if (ch1 != NULL && ch1->width == 0U && ch1->height == 0U) {
        ch1->width = cfg->uvc_cfg.width;
        ch1->height = cfg->uvc_cfg.height;
    }

    depth = TUYA_UVC_INTERNAL_POOL_DEPTH_DEFAULT;

    t = TUYA_UVC_INTERNAL_MS();
    ret = tkl_uvc_pool_create(&s_uvc_out.stream_pool, depth, TUYA_UVC_INTERNAL_SLOT_SIZE,
                              TKL_BUF_USE_STREAM, TUYA_MEDIA_BUF_OWNER_UVC_STREAM,
                              TRUE, "uvc_stream");
    ms_pool = TUYA_UVC_INTERNAL_MS() - t;
    if (ret != OPRT_OK) {
        tkl_system_free(outputs_copy);
        os_memset(&s_uvc_out, 0, sizeof(s_uvc_out));
        return ret;
    }

    /* The parking pool serves ch0 (pull and push) and the on-demand ch1
     * decode. Same slot size as the capture pool: the driver asks for the full
     * frame size whatever was negotiated. */
    if (s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_0] == TRUE ||
        s_uvc_out.want_sp == TRUE) {
        ret = tkl_uvc_pool_create(&s_uvc_out.frame_pool, TUYA_UVC_FRAME_POOL_DEPTH,
                                  TUYA_UVC_INTERNAL_SLOT_SIZE, TKL_BUF_USE_STREAM,
                                  TUYA_MEDIA_BUF_OWNER_UVC_FRAME, TRUE, "uvc_frame");
        if (ret != OPRT_OK) {
            tkl_uvc_pool_destroy(&s_uvc_out.stream_pool, TUYA_MEDIA_BUF_OWNER_UVC_STREAM);
            tkl_system_free(outputs_copy);
            os_memset(&s_uvc_out, 0, sizeof(s_uvc_out));
            return ret;
        }
    }

    t = TUYA_UVC_INTERNAL_MS();
    ret = tkl_uvc_out_camera_init();
    ms_device = TUYA_UVC_INTERNAL_MS() - t;
    if (ret != OPRT_OK) {
        tkl_uvc_pool_destroy(&s_uvc_out.stream_pool, TUYA_MEDIA_BUF_OWNER_UVC_STREAM);
        if (s_uvc_out.frame_pool.slots != NULL) {
            tkl_uvc_pool_destroy(&s_uvc_out.frame_pool, TUYA_MEDIA_BUF_OWNER_UVC_FRAME);
        }
        tkl_system_free(outputs_copy);
        os_memset(&s_uvc_out, 0, sizeof(s_uvc_out));
        return ret;
    }

    s_uvc_out.state = TUYA_UVC_ST_INIT;
    tkl_log_output("tkl_uvc_out: init %ux%u fmt %u, %u channel(s) (sp %u disp %u enc %u), "
                   "pool %u x %u bytes, %u ms (pool %u, device %u)\r\n",
                   (unsigned)cfg->uvc_cfg.width, (unsigned)cfg->uvc_cfg.height,
                   (unsigned)cfg->uvc_cfg.format, (unsigned)cfg->output_num,
                   (unsigned)s_uvc_out.want_sp, (unsigned)s_uvc_out.want_display,
                   (unsigned)s_uvc_out.want_encode,
                   (unsigned)s_uvc_out.stream_pool.depth,
                   (unsigned)TUYA_UVC_INTERNAL_SLOT_SIZE,
                   (unsigned)(ms_pool + ms_device),
                   (unsigned)ms_pool, (unsigned)ms_device);
    tkl_uvc_out_event(TUYA_CAMERA_EVENT_INITIALIZED);
    return OPRT_OK;
}

OPERATE_RET tkl_uvc_start(VOID_T)
{
    TUYA_MEDIA_OUTPUT_CFG_T *ch1;
    UINT32_T ms_consumer;
    UINT32_T ms_stream;
    UINT32_T t;
    UINT8_T i;
    BOOL_T sp_push;
    OPERATE_RET ret;

    if (s_uvc_out.state == TUYA_UVC_ST_STARTED) {
        return OPRT_OK;
    }
    if (s_uvc_out.state != TUYA_UVC_ST_INIT) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* set_pp() requests recorded while stopped land before anything is built,
     * so every pipeline opens at the requested size. */
    for (i = (UINT8_T)TUYA_CAMERA_FRAME_CHANNEL_1;
         i <= (UINT8_T)TUYA_CAMERA_FRAME_CHANNEL_3; i++) {
        if (s_uvc_out.pp_pending[i] == TRUE) {
            TUYA_MEDIA_OUTPUT_CFG_T *ch = tkl_uvc_out_ch((TUYA_CAMERA_FRAME_CHANNEL_E)i);

            if (ch != NULL) {
                ch->width = s_uvc_out.pp_width[i];
                ch->height = s_uvc_out.pp_height[i];
            }
            s_uvc_out.pp_pending[i] = FALSE;
        }
    }

    /* stop() cleared the per-channel active flags; re-seed them from the init
     * configuration so a stop()/start() cycle reopens every channel. */
    s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_1] = s_uvc_out.want_sp;
    s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_2] = s_uvc_out.want_display;
    s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_3] = s_uvc_out.want_encode;

    /* ch1 push: the decoder runs the FRAME sink (no ch2/ch3 - validate
     * rejected that combination). Otherwise ch2/ch3 share the flexa ring. */
    ch1 = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_1);
    sp_push = (s_uvc_out.want_sp == TRUE && ch1 != NULL && ch1->cb != NULL);

    t = TUYA_UVC_INTERNAL_MS();
    if (sp_push == TRUE) {
        ret = tkl_uvc_out_decoder_open_frame();
        if (ret == OPRT_OK) {
            ret = tkl_uvc_out_task_start();
            if (ret != OPRT_OK) {
                (void)tkl_vdec_deinit(s_uvc_out.vdec);
                s_uvc_out.vdec = NULL;
                s_uvc_out.vdec_frame_sink = FALSE;
            }
        }
    } else if (s_uvc_out.want_display == TRUE || s_uvc_out.want_encode == TRUE) {
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
    tkl_uvc_out_event(TUYA_CAMERA_EVENT_STARTED);
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
    s_uvc_out.vdec_frame_sink = FALSE;
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

    /* 2. every bond, then every backend - the iron rule. Covers both decoder
     *    modes: push_close() joins the decode task, tears the consumers down
     *    and then retires the decoder, FRAME sink or flexa ring alike. */
    tkl_uvc_out_push_close();
    s_uvc_out.vdec_frame_sink = FALSE;

    /* every channel stops producing; start() re-seeds them from the init
     * configuration */
    s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_1] = FALSE;
    s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_2] = FALSE;
    s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_3] = FALSE;
    ms_consumer = TUYA_UVC_INTERNAL_MS() - t;

    /* 3. frames sitting in the ready queues belong to nobody now that
     *    production has stopped - take them back so the pools are whole again
     *    for the next start(). Cheap: no waiting, just a queue drain. */
    unconsumed = tkl_uvc_out_flush_ready(&s_uvc_out.stream_pool);
    /* Frames parked for get_frame() and never pulled - same nobody-home case. */
    if (s_uvc_out.frame_pool.slots != NULL) {
        unconsumed += tkl_uvc_out_flush_ready(&s_uvc_out.frame_pool);
    }

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
    tkl_uvc_out_event(TUYA_CAMERA_EVENT_STOPPED);
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
                   "drops stream %u size %u frame-pull %u, decode fail %u\r\n",
                   (unsigned)s_uvc_out.frames,
                   (unsigned)s_uvc_out.decoded,
                   (unsigned)s_uvc_out.drop_stream_empty,
                   (unsigned)s_uvc_out.drop_too_big,
                   (unsigned)s_uvc_out.drop_frame_empty,
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
    if (s_uvc_out.frame_pool.slots != NULL) {
        tkl_uvc_out_retire_pool(&s_uvc_out.frame_pool,
                                TUYA_MEDIA_BUF_OWNER_UVC_FRAME, "frame pool");
    }

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

    tkl_uvc_out_event(TUYA_CAMERA_EVENT_DEINITIALIZED);
    if (s_uvc_out.cfg.outputs != NULL) {
        tkl_system_free(s_uvc_out.cfg.outputs);
    }
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
 * @brief Release the memory of every slot that is back in a queue.
 *
 * @note retire_pool() must not free a slot business still points at, but the
 *       ones sitting in free_q/ready_q are held by nobody. Without this a
 *       single stuck slot leaked the whole pool: the log showed "1 buffers
 *       out, leaking 2048000 bytes" for a 5-slot pool, i.e. one lost frame
 *       cost every buffer in it.
 *
 * @return number of slots reclaimed
 */
STATIC UINT32_T tkl_uvc_out_reclaim_idle(tkl_uvc_pool_t *pool, TUYA_MEDIA_BUF_OWNER_E owner)
{
    tkl_uvc_slot_t *slot;
    UINT32_T n = 0U;

    if (pool->free_q != NULL) {
        while (rtos_pop_from_queue(&pool->free_q, &slot, BEKEN_NO_WAIT) == 0) {
            if (slot->data != NULL) {
                tkl_media_buf_free(owner, slot->data);
                slot->data = NULL;
                n++;
            }
        }
    }

    if (pool->ready_q != NULL) {
        while (rtos_pop_from_queue(&pool->ready_q, &slot, BEKEN_NO_WAIT) == 0) {
            if (slot->data != NULL) {
                tkl_media_buf_free(owner, slot->data);
                slot->data = NULL;
                n++;
            }
        }
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
 *       use-after-free, so on timeout the slabs that are still out are
 *       deliberately left allocated. Zeroing the descriptor makes the release
 *       that eventually arrives find no slot and do nothing, which is the safe
 *       outcome. The idle slots are released first - only the stuck ones leak.
 */
STATIC VOID_T tkl_uvc_out_retire_pool(tkl_uvc_pool_t *pool,
                                      TUYA_MEDIA_BUF_OWNER_E owner,
                                      CONST CHAR_T *name)
{
    if (pool->slots == NULL) {
        return;
    }

    if (tkl_uvc_pool_drain(pool, TUYA_UVC_INTERNAL_DRAIN_MS) != OPRT_OK) {
        UINT32_T out = (UINT32_T)pool->depth - tkl_uvc_pool_free_count(pool);
        UINT32_T reclaimed;

        tkl_log_output("tkl_uvc_out: %s still has %u buffers out, leaking %u bytes\r\n",
                       name,
                       (unsigned)out,
                       (unsigned)(pool->slot_size * out));

        /* Only the slots still out are unsafe to free - hand back the rest so
         * one stuck frame does not cost the whole pool. */
        reclaimed = tkl_uvc_out_reclaim_idle(pool, owner);
        tkl_log_output("tkl_uvc_out: %s reclaimed %u idle slots\r\n",
                       name, (unsigned)reclaimed);

        if (pool->free_q != NULL) {
            rtos_deinit_queue(&pool->free_q);
        }
        if (pool->ready_q != NULL) {
            rtos_deinit_queue(&pool->ready_q);
        }
        /* the descriptor array is ours, not business's */
        tkl_system_free(pool->slots);
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
    BOOL_T want;

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

        /* With every consumer cancelled the decoder would only fill a ring
         * nobody reads (ch2/ch3) or decode pictures nobody wants (ch1 push) -
         * park the slot back and idle until resume_channel() wakes one up. */
        if (s_uvc_out.vdec_frame_sink == TRUE) {
            want = (s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_1] == TRUE);
        } else {
            want = (s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_2] == TRUE ||
                    s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_3] == TRUE);
        }
        if (want != TRUE) {
            tkl_uvc_pool_give(&s_uvc_out.stream_pool, slot);
            continue;
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

OPERATE_RET tkl_uvc_set_bitrate(UINT32_T bitrate)
{
    TUYA_VENC_RATE_CTRL_T rc;

    if (bitrate == 0U) {
        return OPRT_INVALID_PARM;
    }

    /* The transcoder is built by start() and torn down by stop(), so there is
     * nothing to configure in between. */
    if (s_uvc_out.state != TUYA_UVC_ST_STARTED || s_uvc_out.venc == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* Live tuning only: the preset that built the encoder comes from
     * ch3.level, and nothing records a manual bitrate across stop/start. */
    os_memset(&rc, 0, sizeof(rc));
    rc.bitrate = bitrate;
    return tkl_venc_set_rate_ctrl(s_uvc_out.venc, &rc);
}

OPERATE_RET tkl_uvc_set_gop(UINT32_T gop)
{
    TUYA_MEDIA_OUTPUT_CFG_T *ch;

    ch = tkl_uvc_out_ch(TUYA_CAMERA_FRAME_CHANNEL_3);
    if (ch == NULL) {
        return OPRT_NOT_SUPPORTED;
    }

    /* The encoder takes gop only at build time - a running ch3 refuses. */
    if (s_uvc_out.state == TUYA_UVC_ST_STARTED &&
        s_uvc_out.ch_active[TUYA_CAMERA_FRAME_CHANNEL_3] == TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ch->gop = gop;                      /* 0 restores the default 40 */
    return OPRT_OK;
}

OPERATE_RET tkl_uvc_force_idr(VOID_T)
{
    if (s_uvc_out.state != TUYA_UVC_ST_STARTED || s_uvc_out.venc == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    return tkl_venc_request_idr(s_uvc_out.venc);
}

OPERATE_RET tkl_uvc_set_osd(TUYA_CAMERA_FRAME_CHANNEL_E channel, CONST TUYA_OSD_T *osd)
{
    if (osd == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_uvc_out.state != TUYA_UVC_ST_STARTED) {
        return OPRT_RESOURCE_NOT_READY;
    }

    switch (channel) {
    case TUYA_CAMERA_FRAME_CHANNEL_2:
        if (s_uvc_out.ch_active[channel] != TRUE || s_uvc_out.osd_display == NULL) {
            return OPRT_NOT_SUPPORTED;
        }
        return tkl_osd_engine_submit(s_uvc_out.osd_display, osd);

    case TUYA_CAMERA_FRAME_CHANNEL_3:
        if (s_uvc_out.ch_active[channel] != TRUE || s_uvc_out.venc == NULL) {
            return OPRT_NOT_SUPPORTED;
        }
        return tkl_venc_set_osd(s_uvc_out.venc, osd);

    default:
        return OPRT_NOT_SUPPORTED;      /* ch0/ch1 carry no overlay */
    }
}

OPERATE_RET tkl_uvc_set_pp(TUYA_CAMERA_FRAME_CHANNEL_E channel, UINT16_T width, UINT16_T height)
{
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    OPERATE_RET ret;

    if (width == 0U || height == 0U) {
        return OPRT_INVALID_PARM;
    }

    ch = tkl_uvc_out_ch(channel);
    if (ch == NULL) {
        return OPRT_NOT_SUPPORTED;
    }

    /* stopped: record it, start() applies it before building anything */
    if (s_uvc_out.state == TUYA_UVC_ST_INIT) {
        s_uvc_out.pp_pending[channel] = TRUE;
        s_uvc_out.pp_width[channel] = width;
        s_uvc_out.pp_height[channel] = height;
        return OPRT_OK;
    }
    if (s_uvc_out.state != TUYA_UVC_ST_STARTED) {
        return OPRT_RESOURCE_NOT_READY;
    }

    switch (channel) {
    case TUYA_CAMERA_FRAME_CHANNEL_1:
        if (s_uvc_out.vdec_frame_sink == TRUE) {
            /* push: the FRAME-sink output size lives in the decoder. Park the
             * decode task on the flag first so nothing submits while the
             * backend is down, then stop - resize - restart. */
            TUYA_VDEC_PP_T pp;

            s_uvc_out.ch_active[channel] = FALSE;
            ret = tkl_vdec_stop(s_uvc_out.vdec);
            if (ret == OPRT_OK) {
                os_memset(&pp, 0, sizeof(pp));
                pp.out_width = width;
                pp.out_height = height;
                pp.output_format = tkl_media_fmt_to_bk(TUYA_FRAME_FMT_NV12);
                ret = tkl_vdec_set_pp(s_uvc_out.vdec, &pp);
                if (ret == OPRT_OK) {
                    ret = tkl_vdec_start(s_uvc_out.vdec);
                } else {
                    (void)tkl_vdec_start(s_uvc_out.vdec);
                }
            }
            s_uvc_out.ch_active[channel] = s_uvc_out.want_sp;
            if (ret != OPRT_OK) {
                tkl_log_output("tkl_uvc_out: ch1 pp %ux%u failed: %d\r\n",
                               (unsigned)width, (unsigned)height, ret);
                return ret;
            }
        }
        /* pull (flexa ring) or after the restart above: the on-demand decode
         * reads the channel entry per frame, so this lands immediately */
        ch->width = width;
        ch->height = height;
        return OPRT_OK;

    case TUYA_CAMERA_FRAME_CHANNEL_2:
        /* the GPU pipeline owns the scaling - rebuild it around the new size;
         * the vdec ring is untouched, ch3 keeps streaming */
        tkl_uvc_out_display_stop_one();
        ch->width = width;
        ch->height = height;
        ret = tkl_uvc_out_display_start();
        if (ret != OPRT_OK) {
            s_uvc_out.ch_active[channel] = FALSE;
            tkl_log_output("tkl_uvc_out: ch2 pp %ux%u rebuild failed: %d\r\n",
                           (unsigned)width, (unsigned)height, ret);
            return ret;
        }
        return OPRT_OK;

    case TUYA_CAMERA_FRAME_CHANNEL_3:
        /* the encoder has no scaler and encodes the flexa source size, and
         * that size comes from the USB negotiation - not adjustable */
        return OPRT_NOT_SUPPORTED;

    default:
        return OPRT_NOT_SUPPORTED;
    }
}

OPERATE_RET tkl_uvc_resume_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    TUYA_MEDIA_OUTPUT_CFG_T *ch;
    OPERATE_RET ret;

    if (channel == TUYA_CAMERA_FRAME_CHANNEL_0) {
        return OPRT_NOT_SUPPORTED;
    }
    if (s_uvc_out.state != TUYA_UVC_ST_STARTED) {
        return OPRT_RESOURCE_NOT_READY;
    }
    ch = tkl_uvc_out_ch(channel);
    if (ch == NULL) {
        return OPRT_NOT_SUPPORTED;
    }

    switch (channel) {
    case TUYA_CAMERA_FRAME_CHANNEL_1:
        /* the decoder never went away: the FRAME sink keeps its session and
         * the flexa ring keeps being fed - only the flag gates delivery */
        s_uvc_out.ch_active[channel] = TRUE;
        return OPRT_OK;

    case TUYA_CAMERA_FRAME_CHANNEL_2:
        if (s_uvc_out.ch_active[channel] == TRUE) {
            return OPRT_OK;             /* idempotent */
        }
        ret = tkl_uvc_out_display_start();
        if (ret != OPRT_OK) {
            return ret;                 /* GPU claim conflict lands here */
        }
        s_uvc_out.ch_active[channel] = TRUE;
        return OPRT_OK;

    case TUYA_CAMERA_FRAME_CHANNEL_3:
        if (s_uvc_out.ch_active[channel] == TRUE) {
            return OPRT_OK;
        }
        ret = tkl_uvc_out_encode_start();
        if (ret != OPRT_OK) {
            return ret;
        }
        s_uvc_out.ch_active[channel] = TRUE;
        return OPRT_OK;

    default:
        return OPRT_NOT_SUPPORTED;
    }
}

OPERATE_RET tkl_uvc_cancel_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    if (channel == TUYA_CAMERA_FRAME_CHANNEL_0) {
        return OPRT_NOT_SUPPORTED;
    }
    if (s_uvc_out.state != TUYA_UVC_ST_STARTED) {
        return OPRT_RESOURCE_NOT_READY;
    }
    if (tkl_uvc_out_ch(channel) == NULL) {
        return OPRT_NOT_SUPPORTED;
    }

    switch (channel) {
    case TUYA_CAMERA_FRAME_CHANNEL_1:
        /* FRAME sink: the decode task parks itself on the flag; flexa ring:
         * only get_frame() gates on it */
        s_uvc_out.ch_active[channel] = FALSE;
        return OPRT_OK;

    case TUYA_CAMERA_FRAME_CHANNEL_2:
        if (s_uvc_out.ch_active[channel] != TRUE) {
            return OPRT_OK;             /* idempotent */
        }
        s_uvc_out.ch_active[channel] = FALSE;
        tkl_uvc_out_display_stop_one(); /* ch3 keeps streaming */
        return OPRT_OK;

    case TUYA_CAMERA_FRAME_CHANNEL_3:
        if (s_uvc_out.ch_active[channel] != TRUE) {
            return OPRT_OK;
        }
        s_uvc_out.ch_active[channel] = FALSE;
        tkl_uvc_out_encode_stop_one();  /* ch2 keeps displaying */
        return OPRT_OK;

    default:
        return OPRT_NOT_SUPPORTED;
    }
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

OPERATE_RET tkl_uvc_set_pp(TUYA_CAMERA_FRAME_CHANNEL_E channel, UINT16_T width, UINT16_T height)
{
    (void)channel;
    (void)width;
    (void)height;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_set_gop(UINT32_T gop)
{
    (void)gop;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_set_osd(TUYA_CAMERA_FRAME_CHANNEL_E channel, CONST TUYA_OSD_T *osd)
{
    (void)channel;
    (void)osd;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_get_info(TUYA_UVC_INFO_T *info)
{
    (void)info;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_get_frame(TUYA_CAMERA_FRAME_CHANNEL_E channel, TUYA_MEDIA_BUF_T **out, UINT32_T timeout_ms)
{
    (void)channel;
    (void)out;
    (void)timeout_ms;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_resume_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    (void)channel;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_cancel_channel(TUYA_CAMERA_FRAME_CHANNEL_E channel)
{
    (void)channel;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_set_bitrate(UINT32_T bitrate)
{
    (void)bitrate;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uvc_force_idr(VOID_T)
{
    return OPRT_NOT_SUPPORTED;
}

#endif /* CONFIG_USB_CAMERA */
