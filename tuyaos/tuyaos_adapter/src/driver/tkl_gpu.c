#include "tkl_gpu.h"

#include <stdbool.h>
#include <stdint.h>
#include <common/avdk_pixel_types.h>
#include <components/avdk_utils/avdk_error.h>
#include <components/bk_draw_osd.h>
#include <components/bk_flexa_bond.h>
#include <components/bk_gpu.h>
#include <components/bk_gpu_ctlr.h>
#include <os/mem.h>
#include <os/str.h>
#include "tkl_output.h"

typedef struct {
    BOOL_T initialized;
    BOOL_T started;
    BOOL_T sink_ready;
    BOOL_T source_ready;
    BOOL_T backend_opened;
    TUYA_GPU_CFG_T cfg;
    TUYA_GPU_SINK_T sink;
    TUYA_GPU_SOURCE_T source;
    bk_gpu_ctlr_handle_t gpu_handle;
    VOID *bond_handle;
    bk_draw_osd_ctlr_handle_t osd_handle;
} tkl_gpu_ctx_t;

static tkl_gpu_ctx_t s_gpu_ctx;

static void tkl_gpu_runtime_reset(BOOL_T keep_cfg)
{
    TUYA_GPU_CFG_T cfg_backup;
    TUYA_GPU_SINK_T sink_backup;
    BOOL_T sink_ready = FALSE;

    if (keep_cfg == TRUE) {
        os_memcpy(&cfg_backup, &s_gpu_ctx.cfg, sizeof(cfg_backup));
        os_memcpy(&sink_backup, &s_gpu_ctx.sink, sizeof(sink_backup));
        sink_ready = s_gpu_ctx.sink_ready;
    }

    os_memset(&s_gpu_ctx, 0, sizeof(s_gpu_ctx));

    if (keep_cfg == TRUE) {
        os_memcpy(&s_gpu_ctx.cfg, &cfg_backup, sizeof(cfg_backup));
        os_memcpy(&s_gpu_ctx.sink, &sink_backup, sizeof(sink_backup));
        s_gpu_ctx.initialized = TRUE;
        s_gpu_ctx.sink_ready = sink_ready;
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
    BOOL_T has_legacy_sink;

    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->rotate_degree != 0U &&
        cfg->rotate_degree != 90U &&
        cfg->rotate_degree != 180U &&
        cfg->rotate_degree != 270U) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->dst_width == 0U || cfg->dst_height == 0U) {
        return OPRT_INVALID_PARM;
    }

    has_legacy_sink = (cfg->alloc_cb != NULL ||
                       cfg->free_cb != NULL ||
                       cfg->frame_done_cb != NULL);
    if (has_legacy_sink == TRUE &&
        (cfg->alloc_cb == NULL || cfg->free_cb == NULL || cfg->frame_done_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

static OPERATE_RET tkl_gpu_sink_validate(const TUYA_GPU_SINK_T *sink)
{
    if (sink == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (sink->alloc_cb == NULL || sink->free_cb == NULL || sink->frame_done_cb == NULL) {
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

static void *tkl_gpu_frame_malloc(uint32_t size)
{
    if (s_gpu_ctx.sink.alloc_cb == NULL) {
        return NULL;
    }

    return s_gpu_ctx.sink.alloc_cb(size, s_gpu_ctx.sink.cb_arg);
}

static OPERATE_RET tkl_gpu_frame_free(VOID *ptr)
{
    OPERATE_RET ret;

    if (s_gpu_ctx.sink.free_cb == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = s_gpu_ctx.sink.free_cb(ptr, s_gpu_ctx.sink.cb_arg);
    return ret;
}

static void tkl_gpu_frame_done(void *frame, uint32_t frame_size, void *args)
{
    (void)args;

    if (s_gpu_ctx.sink.frame_done_cb != NULL) {
        s_gpu_ctx.sink.frame_done_cb(frame, frame_size, s_gpu_ctx.sink.cb_arg);
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

static OPERATE_RET tkl_gpu_backend_close(void)
{
    OPERATE_RET ret = OPRT_OK;
    OPERATE_RET step_ret;

    if (s_gpu_ctx.gpu_handle == NULL) {
        return OPRT_OK;
    }

    if (s_gpu_ctx.backend_opened != TRUE) {
        return OPRT_OK;
    }

    if (s_gpu_ctx.source.type == TUYA_GPU_SOURCE_ISP_FLEXA) {
        (void)bk_gpu_ioctl(s_gpu_ctx.gpu_handle, BK_GPU_IOCTL_FLEXA_ADDR_UNMAPPING, NULL);
    }

    step_ret = bk_gpu_close(s_gpu_ctx.gpu_handle);
    if (step_ret != OPRT_OK && ret == OPRT_OK) {
        tkl_log_output( "bk_gpu_close failed: %d\r\n", step_ret);
        ret = step_ret;
    }

    s_gpu_ctx.backend_opened = FALSE;
    return ret;
}

static OPERATE_RET tkl_gpu_backend_destroy(void)
{
    OPERATE_RET ret = OPRT_OK;
    OPERATE_RET step_ret;

    if (s_gpu_ctx.gpu_handle == NULL) {
        return OPRT_OK;
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
    s_gpu_ctx.backend_opened = FALSE;
    return ret;
}

static OPERATE_RET tkl_gpu_backend_create(void)
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
    gpu_cfg.frame_malloc = tkl_gpu_frame_malloc;
    gpu_cfg.frame_free = tkl_gpu_frame_free;
    gpu_cfg.frame_done = tkl_gpu_frame_done;
    gpu_cfg.frame_done_args = NULL;

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
        (void)bk_gpu_delete(s_gpu_ctx.gpu_handle);
        s_gpu_ctx.gpu_handle = NULL;
        return ret;
    }

    s_gpu_ctx.backend_opened = FALSE;
    return OPRT_OK;
}

static OPERATE_RET tkl_gpu_backend_open(void)
{
    OPERATE_RET ret;

    if (s_gpu_ctx.gpu_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = bk_gpu_open(s_gpu_ctx.gpu_handle);
    if (ret != OPRT_OK) {
        tkl_log_output( "bk_gpu_open failed: %d\r\n", ret);
        return ret;
    }

    s_gpu_ctx.backend_opened = TRUE;
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
    TUYA_GPU_SINK_T sink_local;
    BOOL_T has_legacy_sink;

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

    tkl_gpu_release_bond();
    ret = tkl_gpu_backend_close();
    if (ret != OPRT_OK) {
        return ret;
    }
    ret = tkl_gpu_backend_destroy();
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_gpu_runtime_reset(FALSE);
    os_memcpy(&s_gpu_ctx.cfg, &cfg_local, sizeof(cfg_local));
    s_gpu_ctx.initialized = TRUE;

    has_legacy_sink = (cfg_local.alloc_cb != NULL ||
                       cfg_local.free_cb != NULL ||
                       cfg_local.frame_done_cb != NULL);
    if (has_legacy_sink == TRUE) {
        os_memset(&sink_local, 0, sizeof(sink_local));
        sink_local.alloc_cb = cfg_local.alloc_cb;
        sink_local.free_cb = cfg_local.free_cb;
        sink_local.frame_done_cb = cfg_local.frame_done_cb;
        sink_local.cb_arg = cfg_local.cb_arg;
        os_memcpy(&s_gpu_ctx.sink, &sink_local, sizeof(sink_local));
        s_gpu_ctx.sink_ready = TRUE;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_gpu_deinit(VOID)
{
    OPERATE_RET ret;
    OPERATE_RET close_ret;

    ret = tkl_gpu_stop();
    if (ret != OPRT_OK) {
        return ret;
    }

    close_ret = tkl_gpu_backend_destroy();
    if (close_ret != OPRT_OK) {
        return close_ret;
    }

    tkl_gpu_runtime_reset(FALSE);
    return OPRT_OK;
}

OPERATE_RET tkl_gpu_set_sink(const TUYA_GPU_SINK_T *sink)
{
    OPERATE_RET ret;

    if (s_gpu_ctx.initialized != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_gpu_ctx.started == TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_gpu_sink_validate(sink);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memcpy(&s_gpu_ctx.sink, sink, sizeof(*sink));
    s_gpu_ctx.sink_ready = TRUE;
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

    tkl_gpu_release_bond();
    ret = tkl_gpu_backend_close();
    if (ret != OPRT_OK) {
        return ret;
    }
    ret = tkl_gpu_backend_destroy();
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memcpy(&s_gpu_ctx.source, source, sizeof(*source));
    s_gpu_ctx.source_ready = TRUE;
    ret = tkl_gpu_backend_create();
    if (ret != OPRT_OK) {
        s_gpu_ctx.source_ready = FALSE;
        return ret;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_gpu_start(VOID)
{
    OPERATE_RET ret;

    if (s_gpu_ctx.initialized != TRUE ||
        s_gpu_ctx.sink_ready != TRUE ||
        s_gpu_ctx.source_ready != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_gpu_ctx.started == TRUE) {
        return OPRT_OK;
    }

    if (s_gpu_ctx.gpu_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_gpu_backend_open();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_gpu_bind_source();
    if (ret != OPRT_OK) {
        (void)tkl_gpu_release_bond();
        (void)tkl_gpu_backend_close();
        return ret;
    }

    s_gpu_ctx.started = TRUE;
    return OPRT_OK;
}

OPERATE_RET tkl_gpu_stop(VOID)
{
    OPERATE_RET ret;

    if (s_gpu_ctx.started == FALSE &&
        s_gpu_ctx.gpu_handle == NULL &&
        s_gpu_ctx.bond_handle == NULL) {
        return OPRT_OK;
    }

    (void)tkl_gpu_osd_close();

    tkl_gpu_release_bond();

    ret = tkl_gpu_backend_close();
    if (ret != OPRT_OK) {
        return ret;
    }

    s_gpu_ctx.started = FALSE;
    return OPRT_OK;
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

/**
 * @brief Resolve OSD sprite format from cfg / open request
 * @param[in] req_format requested format, 0=auto
 * @return bk_pixel_format_t
 */
static bk_pixel_format_t tkl_gpu_osd_src_format_resolve(UINT32_T req_format)
{
    if (req_format != 0U &&
        req_format != BK_PIXEL_FORMAT_INVALID &&
        req_format != BK_PIXEL_FORMAT_UNKNOW) {
        return (bk_pixel_format_t)req_format;
    }

    if (s_gpu_ctx.cfg.dst_compress == TRUE) {
        return BK_PIXEL_FORMAT_ABGR8888;
    }
    return BK_PIXEL_FORMAT_ARGB8888;
}

/**
 * @brief Map avdk_err_t to OPERATE_RET
 * @param[in] ret vendor error
 * @return OPERATE_RET
 */
static OPERATE_RET tkl_gpu_osd_map_err(avdk_err_t ret)
{
    if (ret == AVDK_ERR_OK) {
        return OPRT_OK;
    }
    if (ret == AVDK_ERR_INVAL) {
        return OPRT_INVALID_PARM;
    }
    if (ret == AVDK_ERR_NOMEM) {
        return OPRT_MALLOC_FAILED;
    }
    if (ret == AVDK_ERR_UNSUPPORTED) {
        return OPRT_NOT_SUPPORTED;
    }
    return OPRT_COM_ERROR;
}

/**
 * @brief Create bk_draw_osd on started GPU (no draw; first paint via UPDATE)
 * @param[in] cfg open config
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_gpu_osd_open(CONST TUYA_GPU_OSD_OPEN_T *cfg)
{
    osd_ctlr_config_t osd_cfg;
    avdk_err_t ret;
    bool by_flexa;

    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (s_gpu_ctx.started != TRUE || s_gpu_ctx.gpu_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    (void)tkl_gpu_osd_close();

    by_flexa = (cfg->by_flexa == TRUE);
    (void)bk_gpu_ioctl(s_gpu_ctx.gpu_handle, BK_GPU_IOCTL_SET_OSD_BY_FLEXA, &by_flexa);

    os_memset(&osd_cfg, 0, sizeof(osd_cfg));
    osd_cfg.gpu = s_gpu_ctx.gpu_handle;
    osd_cfg.panel_w = s_gpu_ctx.cfg.dst_width;
    osd_cfg.panel_h = s_gpu_ctx.cfg.dst_height;
    osd_cfg.src_format = tkl_gpu_osd_src_format_resolve(cfg->src_format);
    osd_cfg.blend_assets = cfg->blend_assets;
    osd_cfg.blend_info = cfg->blend_info;

    ret = bk_draw_osd_new(&s_gpu_ctx.osd_handle, &osd_cfg);
    if (ret != AVDK_ERR_OK) {
        s_gpu_ctx.osd_handle = NULL;
        tkl_log_output("tkl_gpu: bk_draw_osd_new failed (%d)\r\n", ret);
        return tkl_gpu_osd_map_err(ret);
    }

    /* Intentionally no bk_draw_osd_array here: first paint is tkl_gpu_osd_ctrl(UPDATE) */
    return OPRT_OK;
}

/**
 * @brief Re-render dynamic list after list mutation (clear leftover blit slots first)
 * @return OPRT_OK on success
 */
static OPERATE_RET tkl_gpu_osd_redraw(VOID)
{
    avdk_err_t ret;

    (void)bk_draw_osd_clear(s_gpu_ctx.osd_handle);
    ret = bk_draw_osd_array(s_gpu_ctx.osd_handle, NULL);
    return tkl_gpu_osd_map_err(ret);
}

/**
 * @brief Runtime OSD control: UPDATE / REMOVE / CLEAR
 * @param[in] op operation
 * @param[in] name element name
 * @param[in] content UPDATE content
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_gpu_osd_ctrl(TUYA_GPU_OSD_OP_E op, CONST CHAR_T *name, CONST CHAR_T *content)
{
    avdk_err_t ret;

    if (s_gpu_ctx.osd_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    switch (op) {
    case TUYA_GPU_OSD_OP_UPDATE:
        if (name == NULL || name[0] == '\0' || content == NULL || content[0] == '\0') {
            return OPRT_INVALID_PARM;
        }
        ret = bk_draw_osd_add_or_update(s_gpu_ctx.osd_handle, name, content);
        if (ret != AVDK_ERR_OK) {
            return tkl_gpu_osd_map_err(ret);
        }
        return tkl_gpu_osd_redraw();

    case TUYA_GPU_OSD_OP_REMOVE:
        if (name == NULL || name[0] == '\0') {
            return OPRT_INVALID_PARM;
        }
        ret = bk_draw_osd_remove(s_gpu_ctx.osd_handle, name);
        if (ret != AVDK_ERR_OK) {
            return tkl_gpu_osd_map_err(ret);
        }
        return tkl_gpu_osd_redraw();

    case TUYA_GPU_OSD_OP_CLEAR:
        ret = bk_draw_osd_clear(s_gpu_ctx.osd_handle);
        return tkl_gpu_osd_map_err(ret);

    default:
        return OPRT_INVALID_PARM;
    }
}

/**
 * @brief Destroy OSD instance
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_gpu_osd_close(VOID)
{
    if (s_gpu_ctx.osd_handle == NULL) {
        return OPRT_OK;
    }

    (void)bk_draw_osd_delete(s_gpu_ctx.osd_handle);
    s_gpu_ctx.osd_handle = NULL;
    return OPRT_OK;
}

/**
 * @brief Set frame-end / per-flexa OSD blend mode
 * @param[in] by_flexa TRUE for per-flexa
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_gpu_osd_set_mode(BOOL_T by_flexa)
{
    bool en = (by_flexa == TRUE);
    avdk_err_t ret;

    if (s_gpu_ctx.gpu_handle == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = bk_gpu_ioctl(s_gpu_ctx.gpu_handle, BK_GPU_IOCTL_SET_OSD_BY_FLEXA, &en);
    return tkl_gpu_osd_map_err(ret);
}
