#include "tkl_gpu.h"

#include <stdbool.h>
#include <common/avdk_pixel_types.h>
#include <components/bk_flexa_bond.h>
#include <components/bk_gpu.h>
#include <components/bk_gpu_ctlr.h>
#include <os/mem.h>
#include <os/str.h>
#include "tkl_output.h"

typedef struct {
    BOOL_T initialized;
    BOOL_T started;
    BOOL_T source_ready;
    TUYA_GPU_CFG_T cfg;
    TUYA_GPU_SOURCE_T source;
    bk_gpu_ctlr_handle_t gpu_handle;
    VOID *bond_handle;
} tkl_gpu_ctx_t;

static tkl_gpu_ctx_t s_gpu_ctx;

static void tkl_gpu_runtime_reset(BOOL_T keep_cfg)
{
    TUYA_GPU_CFG_T cfg_backup;

    if (keep_cfg == TRUE) {
        os_memcpy(&cfg_backup, &s_gpu_ctx.cfg, sizeof(cfg_backup));
    }

    os_memset(&s_gpu_ctx, 0, sizeof(s_gpu_ctx));

    if (keep_cfg == TRUE) {
        os_memcpy(&s_gpu_ctx.cfg, &cfg_backup, sizeof(cfg_backup));
        s_gpu_ctx.initialized = TRUE;
    }
}

static void tkl_gpu_cfg_normalize(TUYA_GPU_CFG_T *cfg)
{
    if (cfg->dst_format == BK_PIXEL_FORMAT_INVALID ||
        cfg->dst_format == BK_PIXEL_FORMAT_UNKNOW) {
        cfg->dst_format = BK_PIXEL_FORMAT_ARGB8888;
    }

    if (cfg->flexa_lines == 0U) {
        cfg->flexa_lines = 16U;
    }
}

static OPERATE_RET tkl_gpu_cfg_validate(const TUYA_GPU_CFG_T *cfg)
{
    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->rotate_degree != 0U &&
        cfg->rotate_degree != 90U &&
        cfg->rotate_degree != 180U &&
        cfg->rotate_degree != 270U) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->alloc_cb == NULL || cfg->free_cb == NULL || cfg->frame_done_cb == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->dst_width == 0U || cfg->dst_height == 0U) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static OPERATE_RET tkl_gpu_flexa_validate(const TUYA_GPU_FLEXA_BUFFER_T *flexa)
{
    if (flexa == NULL ||
        flexa->src_buffer == NULL ||
        flexa->buf_cnt == 0U ||
        flexa->width == 0U ||
        flexa->height == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (flexa->format == BK_PIXEL_FORMAT_INVALID ||
        flexa->format == BK_PIXEL_FORMAT_UNKNOW) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static OPERATE_RET tkl_gpu_source_validate(const TUYA_GPU_SOURCE_T *source)
{
    OPERATE_RET ret;

    if (source == NULL) {
        return OPRT_INVALID_PARM;
    }

    switch (source->type) {
    case TUYA_GPU_SOURCE_ISP_FLEXA:
        if (source->isp.isp_handle == NULL) {
            return OPRT_INVALID_PARM;
        }
        ret = tkl_gpu_flexa_validate(&source->isp.flexa);
        if (ret != OPRT_OK) {
            return ret;
        }
        break;

    case TUYA_GPU_SOURCE_MJPEG_FLEXA:
        if (source->mjpeg.jpeg_handle == NULL) {
            return OPRT_INVALID_PARM;
        }
        ret = tkl_gpu_flexa_validate(&source->mjpeg.flexa);
        if (ret != OPRT_OK) {
            return ret;
        }
        break;

    default:
        return OPRT_NOT_SUPPORTED;
    }

    return OPRT_OK;
}

static const TUYA_GPU_FLEXA_BUFFER_T *tkl_gpu_source_flexa_get(const TUYA_GPU_SOURCE_T *source)
{
    if (source == NULL) {
        return NULL;
    }

    switch (source->type) {
    case TUYA_GPU_SOURCE_ISP_FLEXA:
        return &source->isp.flexa;
    case TUYA_GPU_SOURCE_MJPEG_FLEXA:
        return &source->mjpeg.flexa;
    default:
        return NULL;
    }
}

static VOID *tkl_gpu_frame_malloc(UINT32_T size)
{
    if (s_gpu_ctx.cfg.alloc_cb == NULL) {
        return NULL;
    }

    return s_gpu_ctx.cfg.alloc_cb(size, s_gpu_ctx.cfg.cb_arg);
}

static OPERATE_RET tkl_gpu_frame_free(VOID *ptr)
{
    OPERATE_RET ret;

    if (s_gpu_ctx.cfg.free_cb == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = s_gpu_ctx.cfg.free_cb(ptr, s_gpu_ctx.cfg.cb_arg);
    return ret;
}

static void tkl_gpu_frame_done(void *frame, uint32_t frame_size, void *args)
{
    (void)args;

    if (s_gpu_ctx.cfg.frame_done_cb != NULL) {
        s_gpu_ctx.cfg.frame_done_cb(frame, frame_size, s_gpu_ctx.cfg.cb_arg);
    }
}

static void tkl_gpu_release_bond(void)
{
    if (s_gpu_ctx.bond_handle == NULL) {
        return;
    }

    switch (s_gpu_ctx.source.type) {
    case TUYA_GPU_SOURCE_ISP_FLEXA:
        bk_flexa_isp_gpu_bond_stop(s_gpu_ctx.bond_handle);
        break;

    case TUYA_GPU_SOURCE_MJPEG_FLEXA:
        bk_flexa_mjpegd_gpu_bond_stop(s_gpu_ctx.bond_handle);
        break;

    default:
        break;
    }

    s_gpu_ctx.bond_handle = NULL;
}

static OPERATE_RET tkl_gpu_open_controller(void)
{
    OPERATE_RET ret;
    bk_gpu_ctlr_config_t gpu_cfg;
    const TUYA_GPU_FLEXA_BUFFER_T *flexa = tkl_gpu_source_flexa_get(&s_gpu_ctx.source);
    UINT16_T dst_width;
    UINT16_T dst_height;

    if (flexa == NULL) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&gpu_cfg, 0, sizeof(gpu_cfg));

    dst_width = s_gpu_ctx.cfg.dst_width;
    dst_height = s_gpu_ctx.cfg.dst_height;
    if (dst_width == 0U || dst_height == 0U) {
        tkl_log_output( "gpu dst size is invalid: %ux%u\r\n", dst_width, dst_height);
        return OPRT_INVALID_PARM;
    }

    gpu_cfg.rotate_degree = s_gpu_ctx.cfg.rotate_degree;
    gpu_cfg.src_width = flexa->width;
    gpu_cfg.src_height = flexa->height;
    gpu_cfg.dst_width = dst_width;
    gpu_cfg.dst_height = dst_height;
    gpu_cfg.src_format = (bk_pixel_format_t)flexa->format;
    gpu_cfg.dst_format = (bk_pixel_format_t)s_gpu_ctx.cfg.dst_format;
    gpu_cfg.scale = (s_gpu_ctx.cfg.scale != FALSE ||
                     flexa->width != dst_width ||
                     flexa->height != dst_height);
    gpu_cfg.compress = (s_gpu_ctx.cfg.dst_compress != FALSE);
    gpu_cfg.flexa = true;
    gpu_cfg.flexa_lines = s_gpu_ctx.cfg.flexa_lines;
    gpu_cfg.flexa_buff_cnt = flexa->buf_cnt;
    gpu_cfg.src_buffer = flexa->src_buffer;
    gpu_cfg.malloc = tkl_gpu_frame_malloc;
    gpu_cfg.free = tkl_gpu_frame_free;
    gpu_cfg.frame_display = tkl_gpu_frame_done;
    gpu_cfg.frame_display_args = NULL;

    tkl_log_output(
            "gpu open controller: src=%ux%u fmt=%u -> dst=%ux%u fmt=%u rotate=%u compress=%u scale=%u flexa_lines=%u\r\n",
            gpu_cfg.src_width,
            gpu_cfg.src_height,
            (unsigned int)gpu_cfg.src_format,
            gpu_cfg.dst_width,
            gpu_cfg.dst_height,
            (unsigned int)gpu_cfg.dst_format,
            gpu_cfg.rotate_degree,
            (unsigned int)gpu_cfg.compress,
            (unsigned int)gpu_cfg.scale,
            gpu_cfg.flexa_lines);

    ret = bk_gpu_ctlr_new(&s_gpu_ctx.gpu_handle, &gpu_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_gpu_ctlr_new failed: %d\r\n", ret);
        return ret;
    }

    ret = bk_gpu_init(s_gpu_ctx.gpu_handle);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_gpu_init failed: %d\r\n", ret);
        return ret;
    }

    ret = bk_gpu_open(s_gpu_ctx.gpu_handle);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_gpu_open failed: %d\r\n", ret);
        return ret;
    }

    return OPRT_OK;
}

static OPERATE_RET tkl_gpu_bind_source(void)
{
    OPERATE_RET ret = OPRT_OK;

    switch (s_gpu_ctx.source.type) {
    case TUYA_GPU_SOURCE_ISP_FLEXA:
        ret = bk_flexa_isp_gpu_bond_start(&s_gpu_ctx.bond_handle,
                                          s_gpu_ctx.source.isp.isp_handle,
                                          s_gpu_ctx.gpu_handle);
        break;

    case TUYA_GPU_SOURCE_MJPEG_FLEXA:
        ret = bk_flexa_mjpegd_gpu_bond_start(&s_gpu_ctx.bond_handle,
                                             (bk_jpeg_decode_ctlr_handle_t)s_gpu_ctx.source.mjpeg.jpeg_handle,
                                             s_gpu_ctx.gpu_handle);
        break;

    default:
        return OPRT_NOT_SUPPORTED;
    }

    if (ret != OPRT_OK) {
        tkl_log_output( "bind source failed: %d\r\n", ret);
        s_gpu_ctx.bond_handle = NULL;
        return ret;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_gpu_init(TUYA_GPU_CFG_T *config)
{
    OPERATE_RET ret;
    TUYA_GPU_CFG_T cfg_local;

    if (config == NULL) {
        return OPRT_INVALID_PARM;
    }

    cfg_local = *config;
    tkl_gpu_cfg_normalize(&cfg_local);
    ret = tkl_gpu_cfg_validate(&cfg_local);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (s_gpu_ctx.started == TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    tkl_gpu_runtime_reset(FALSE);
    os_memcpy(&s_gpu_ctx.cfg, &cfg_local, sizeof(cfg_local));
    s_gpu_ctx.initialized = TRUE;
    return OPRT_OK;
}

OPERATE_RET tkl_gpu_deinit(VOID)
{
    OPERATE_RET ret;

    ret = tkl_gpu_stop();
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_gpu_runtime_reset(FALSE);
    return OPRT_OK;
}

OPERATE_RET tkl_gpu_set_source(TUYA_GPU_SOURCE_T *source)
{
    OPERATE_RET ret;

    ret = tkl_gpu_source_validate(source);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (s_gpu_ctx.initialized != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_gpu_ctx.started == TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    os_memcpy(&s_gpu_ctx.source, source, sizeof(*source));
    s_gpu_ctx.source_ready = TRUE;
    return OPRT_OK;
}

OPERATE_RET tkl_gpu_start(VOID)
{
    OPERATE_RET ret;

    if (s_gpu_ctx.initialized != TRUE || s_gpu_ctx.source_ready != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_gpu_ctx.started == TRUE) {
        return OPRT_OK;
    }

    ret = tkl_gpu_open_controller();
    if (ret != OPRT_OK) {
        (void)tkl_gpu_stop();
        return ret;
    }

    ret = tkl_gpu_bind_source();
    if (ret != OPRT_OK) {
        (void)tkl_gpu_stop();
        return ret;
    }

    s_gpu_ctx.started = TRUE;
    return OPRT_OK;
}

OPERATE_RET tkl_gpu_stop(VOID)
{
    OPERATE_RET ret = OPRT_OK;
    OPERATE_RET step_ret;

    tkl_gpu_release_bond();

    if (s_gpu_ctx.gpu_handle != NULL) {
        if (s_gpu_ctx.source.type == TUYA_GPU_SOURCE_ISP_FLEXA) {
            (void)bk_gpu_ioctl(s_gpu_ctx.gpu_handle, BK_GPU_IOCTL_FLEXA_ADDR_UNMAPPING, NULL);
        }

        step_ret = bk_gpu_close(s_gpu_ctx.gpu_handle);
        if (step_ret != OPRT_OK && ret == OPRT_OK) {
            tkl_log_output( "bk_gpu_close failed: %d\r\n", step_ret);
            ret = step_ret;
        }

        step_ret = bk_gpu_deinit(s_gpu_ctx.gpu_handle);
        if (step_ret != OPRT_OK && ret == OPRT_OK) {
            tkl_log_output( "bk_gpu_deinit failed: %d\r\n", step_ret);
            ret = step_ret;
        }

        step_ret = bk_gpu_delete(s_gpu_ctx.gpu_handle);
        if (step_ret != OPRT_OK && ret == OPRT_OK) {
            tkl_log_output( "bk_gpu_delete failed: %d\r\n", step_ret);
            ret = step_ret;
        }

        s_gpu_ctx.gpu_handle = NULL;
    }

    s_gpu_ctx.started = FALSE;
    return ret;
}

OPERATE_RET tkl_gpu_get_handle(VOID **handle)
{
    if (handle == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_gpu_ctx.started != TRUE || s_gpu_ctx.gpu_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    *handle = s_gpu_ctx.gpu_handle;
    return OPRT_OK;
}
