/**
 * @file tkl_aud_dmic.c
 * @brief TKL audio digital (DMIC) driver implementation
 * @version 1.1
 * @date 2026-10-21
 * @copyright Copyright (c) Tuya Inc.
 *
 * On BK7259 (CONFIG_AUD_DRIVER_V2) the digital mic is not a separate capture
 * path: its samples are injected into ADC lanes 0..2 through sys_cfg.digmic_en
 * and drained by the same GDMA channel the analog mic uses. All of that lives in
 * tkl_aud_capture.c, which is shared with tkl_aud_adc.c. This file only keeps
 * the public TKL surface and the pad configuration.
 */

#include <common/bk_include.h>
#include <os/os.h>
#include <os/mem.h>

#if CONFIG_AUD_DRIVER_V2
#include <driver/aud_adc_types.h>
#include <driver/aud_adc.h>
#else
#include <driver/aud_dmic_types.h>
#endif
#include <driver/aud_dmic.h>
#include <driver/aud_common.h>
#include <driver/dma.h>
#include <driver/audio_ring_buff.h>
#include <driver/gpio.h>
#include <components/bk_audio/audio_pipeline/audio_mem.h>

#include "tuya_cloud_types.h"
#include "tkl_aud_dmic.h"
#include "gpio_driver.h"

#if CONFIG_AUD_DRIVER_V2
#include "tkl_aud_capture.h"
#endif

extern void bk_printf(const char *fmt, ...);

#define MODULE_LOG_HELPER(fmt, ...) \
    bk_printf(fmt, ##__VA_ARGS__)
#define TKL_AUD_DMIC_LOG(...) MODULE_LOG_HELPER(__VA_ARGS__)

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define TKL_AUD_DMIC_GAIN_NUM         (2U)        /* dig then ana */
#if CONFIG_AUD_DRIVER_V2
#define TKL_DMIC_CLK_GPIO             GPIO_6
#define TKL_DMIC_DAT_GPIO             GPIO_5
#else
#define AUD_DMIC_FRAME_SIZE(sr, sb, t, ch)  (((sr) * ((sb) / 8) / 1000) * (ch) * (t))

#define DMIC_DMA_BUF_SAFE_INTERVAL    (8)
#define TKL_DMIC_CLK_GPIO             GPIO_8
#define TKL_DMIC_DAT_GPIO             GPIO_9
#endif

/* ---------------------------------------------------------------------------
 * Type definitions
 * --------------------------------------------------------------------------- */
typedef struct {
    gpio_id_t clk_gpio;
    gpio_id_t data_gpio;
} tkl_aud_dmic_pinmux_cfg_t;

/* ---------------------------------------------------------------------------
 * File scope variables
 * --------------------------------------------------------------------------- */
/* One pad pair per port: MODE_0 and MODE_1 are different pad functions
 * (DMIC0_* vs DMIC1_*), so the two ports cannot share a pin. */
STATIC tkl_aud_dmic_pinmux_cfg_t gl_dmic_pinmux[2] = {
    { .clk_gpio = TKL_DMIC_CLK_GPIO, .data_gpio = TKL_DMIC_DAT_GPIO },
    { .clk_gpio = TKL_DMIC_CLK_GPIO, .data_gpio = TKL_DMIC_DAT_GPIO },
};

STATIC OPERATE_RET __tkl_aud_dmic_pinmux_validate(UINT32_T clk_gpio, UINT32_T data_gpio)
{
    if ((clk_gpio >= GPIO_NUM) || (data_gpio >= GPIO_NUM)) {
        TKL_AUD_DMIC_LOG("dmic: invalid pinmux clk=%u data=%u\n", clk_gpio, data_gpio);
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

/* ===========================================================================
 * CONFIG_AUD_DRIVER_V2: thin wrapper over the shared capture core
 * =========================================================================== */
#if CONFIG_AUD_DRIVER_V2

/**
 * @brief Map a public DMIC port onto its capture-core user
 * @param[in] port TUYA_AUDIO_DMIC_PORT_0 / _1
 * @return the core user, or TKL_AUD_CAP_USER_MAX if @p port is invalid
 */
STATIC TKL_AUD_CAP_USER_E __tkl_aud_dmic_user(TUYA_AUDIO_DMIC_PORT_E port)
{
    switch (port) {
        case TUYA_AUDIO_DMIC_PORT_0:
            return TKL_AUD_CAP_USER_DMIC0;
        case TUYA_AUDIO_DMIC_PORT_1:
            return TKL_AUD_CAP_USER_DMIC1;
        default:
            return TKL_AUD_CAP_USER_MAX;
    }
}

/**
 * @brief Whether @p port is open on the shared core
 * @param[in] port digital port
 * @return TRUE if that port (and only that port) is attached
 */
STATIC BOOL_T __tkl_aud_dmic_attached(TUYA_AUDIO_DMIC_PORT_E port)
{
    TKL_AUD_CAP_USER_E user = __tkl_aud_dmic_user(port);

    if (user == TKL_AUD_CAP_USER_MAX) {
        return FALSE;
    }

    return __t9_aud_cap_user_attached(user);
}

/**
 * @brief The DMIC channel bits a port owns
 * @param[in] port digital port
 * @return bitmap: BIT(8) for port 0, BIT(9)|BIT(10) for port 1
 */
STATIC UINT32_T __tkl_aud_dmic_own_mask(TUYA_AUDIO_DMIC_PORT_E port)
{
    if (port == TUYA_AUDIO_DMIC_PORT_0) {
        return (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_0;
    }

    return (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_1 |
           (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_2;
}

/**
 * @brief Turn a single DMIC channel bit into an ADC lane index
 * @param[in]  port    digital port that must own the channel
 * @param[in]  channel channel bitmap with exactly one bit of @p port
 * @param[out] lane    lane index 0..2
 * @return OPRT_OK on success; OPRT_INVALID_PARM otherwise
 */
STATIC OPERATE_RET __tkl_aud_dmic_channel_lane(TUYA_AUDIO_DMIC_PORT_E port,
                                                TUYA_AUDIO_MIC_CHANNEL_E channel,
                                                UINT32_T *lane)
{
    UINT32_T bit = (UINT32_T)channel;
    UINT32_T own = __tkl_aud_dmic_own_mask(port);
    UINT32_T idx;

    if ((bit == 0U) || ((bit & ~own) != 0U)) {
        return OPRT_INVALID_PARM;
    }

    for (idx = 0U; idx < 3U; idx++) {
        if (bit == ((UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_0 << idx)) {
            *lane = idx;
            return OPRT_OK;
        }
    }

    return OPRT_INVALID_PARM;
}

/**
 * @brief Configure DMIC pinmux before init
 * @param[in] port Digital audio port number whose pads are set
 * @param[in] clk_gpio DMIC clock gpio id
 * @param[in] data_gpio DMIC data gpio id
 * @return OPRT_OK on success, error code on failure
 * @note Only the pads are recorded here; the core muxes them when the DMIC
 *       interface attaches and releases them when it detaches.
 */
OPERATE_RET tkl_aud_dmic_pinmux_config(TUYA_AUDIO_DMIC_PORT_E port,
                                       UINT32_T clk_gpio, UINT32_T data_gpio)
{
    OPERATE_RET ret;

    if (__tkl_aud_dmic_user(port) == TKL_AUD_CAP_USER_MAX) {
        TKL_AUD_DMIC_LOG("dmic: pinmux port %d not support\n", port);
        return OPRT_INVALID_PARM;
    }

    ret = __tkl_aud_dmic_pinmux_validate(clk_gpio, data_gpio);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (__tkl_aud_dmic_attached(port) == TRUE) {
        TKL_AUD_DMIC_LOG("dmic: pinmux config should be set before init\n");
        return OPRT_COM_ERROR;
    }

    gl_dmic_pinmux[port].clk_gpio = (gpio_id_t)clk_gpio;
    gl_dmic_pinmux[port].data_gpio = (gpio_id_t)data_gpio;

    TKL_AUD_DMIC_LOG("dmic: port %d pinmux configured clk=%u data=%u\n",
                     port,
                     (uint32_t)gl_dmic_pinmux[port].clk_gpio,
                     (uint32_t)gl_dmic_pinmux[port].data_gpio);

    return OPRT_OK;
}

/**
 * @brief Initialize audio digital (DMIC) input
 * @param[in] port Digital audio port number
 * @param[in] config DMIC configuration
 * @return OPRT_OK on success, error code on failure
 * @note Gain is set via tkl_aud_dmic_set_gain(); init keeps OEM default dig/ana.
 * @note @c config->chan is a bitmap of this port's own DMIC bits; the callback
 *       receives only those lanes, interleaved in ascending lane order.
 *       Re-initializing reconfigures the port and returns OPRT_OK.
 */
OPERATE_RET tkl_aud_dmic_init(TUYA_AUDIO_DMIC_PORT_E port, CONST TUYA_AUDIO_DMIC_BASE_CFG_T *config)
{
    TKL_AUD_CAP_REQ_T req;
    TKL_AUD_CAP_USER_E user = __tkl_aud_dmic_user(port);
    UINT32_T mask;
    UINT32_T own;
    OPERATE_RET ret;
    BOOL_T reopening;

    if ((config == NULL) || (config->upper_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (user == TKL_AUD_CAP_USER_MAX) {
        TKL_AUD_DMIC_LOG("dmic: port %d not support\n", port);
        return OPRT_INVALID_PARM;
    }

    mask = (UINT32_T)config->chan;
    own = __tkl_aud_dmic_own_mask(port);
    if ((mask == 0U) || ((mask & ~own) != 0U)) {
        TKL_AUD_DMIC_LOG("dmic: chan 0x%x not support on port %d\n",
                         (unsigned)mask, port);
        return OPRT_INVALID_PARM;
    }

    if (config->sample_bits != TUYA_AUDIO_SAMPLE_BITS_16) {
        TKL_AUD_DMIC_LOG("dmic: bits %d not support, only 16bits\n", config->sample_bits);
        return OPRT_INVALID_PARM;
    }

    if ((config->sample_rate == 0U) || (config->frame_time_ms == 0U)) {
        return OPRT_INVALID_PARM;
    }

    TKL_AUD_DMIC_LOG("dmic: init request port=%d chan=0x%x bits=%u rate=%u frame_ms=%u args=%p\n",
                     port, (unsigned)mask, config->sample_bits, config->sample_rate,
                     config->frame_time_ms, config->args);

    os_memset(&req, 0, sizeof(req));
    req.sample_rate = config->sample_rate;
    req.sample_bits = (UINT8_T)config->sample_bits;
    req.frame_time_ms = config->frame_time_ms;
    req.upper_cb = config->upper_cb;
    req.args = config->args;
    req.chan = config->chan;
    req.dmic_port = port;
    req.dmic_clk_gpio = (UINT32_T)gl_dmic_pinmux[port].clk_gpio;
    req.dmic_data_gpio = (UINT32_T)gl_dmic_pinmux[port].data_gpio;

    /* Contract: a second init reconfigures the instance and returns OK.
     * Propagate the core's error code verbatim - callers must be able to tell
     * a lane/pad conflict (OPRT_INVALID_PARM) from a core failure. */
    reopening = __t9_aud_cap_user_attached(user);
    ret = (reopening == TRUE) ? __t9_aud_cap_reopen(user, &req)
                              : __t9_aud_cap_open(user, &req);
    if (ret != OPRT_OK) {
        TKL_AUD_DMIC_LOG("dmic: capture core %s failed: %d\n",
                         (reopening == TRUE) ? "reopen" : "open", (int)ret);
        return ret;
    }

    TKL_AUD_DMIC_LOG("dmic: init success, port=%d clk=%u data=%u\n",
                     port, req.dmic_clk_gpio, req.dmic_data_gpio);
    return OPRT_OK;
}

/**
 * @brief Start audio digital (DMIC) capture
 * @param[in] port Digital audio port number
 * @return OPRT_OK on success, error code on failure
 * @note Start is reference counted by the core: the first start brings the
 *       stream up, a start while the other port is already running only adds
 *       this port's callback.
 */
OPERATE_RET tkl_aud_dmic_start(TUYA_AUDIO_DMIC_PORT_E port)
{
    TKL_AUD_CAP_USER_E user = __tkl_aud_dmic_user(port);

    if ((user == TKL_AUD_CAP_USER_MAX) || (__t9_aud_cap_user_attached(user) != TRUE)) {
        return OPRT_COM_ERROR;
    }

    return __t9_aud_cap_start(user);
}

/**
 * @brief Set DMIC dig/ana gain for one physical channel
 * @param[in] port Digital audio port number
 * @param[in] channel one DMIC bit of TUYA_AUDIO_MIC_CHANNEL_E owned by @p port
 * @param[in] gain gain[0]=dig, gain[1]=ana; AUDIO_DMIC_INVALID_GAIN_VALUE to skip
 * @param[in] num 1 (dig only) or 2 (dig then ana)
 * @return OPRT_OK on success
 * @note dig is the effective gain for DMIC; ana maps to the analog MIC PGA.
 * @note Only the lanes the DMIC port actually owns may be moved: port 0 owns
 *       CH0, port 1 owns CH1/CH2. The analog lane belongs to tkl_aud_adc.
 */
OPERATE_RET tkl_aud_dmic_set_gain(TUYA_AUDIO_DMIC_PORT_E port, TUYA_AUDIO_MIC_CHANNEL_E channel,
                                  INT32_T gain[], UINT32_T num)
{
    TKL_AUD_CAP_USER_E user = __tkl_aud_dmic_user(port);
    BOOL_T set_dig;
    BOOL_T set_ana;
    FLOAT_T dig_db = 0.0f;
    INT32_T ana_db = 0;
    UINT32_T lane = 0U;

    if ((user == TKL_AUD_CAP_USER_MAX) || (__t9_aud_cap_user_attached(user) != TRUE)) {
        return OPRT_COM_ERROR;
    }

    if ((gain == NULL) || (num == 0U) || (num > TKL_AUD_DMIC_GAIN_NUM)) {
        return OPRT_INVALID_PARM;
    }

    if (__tkl_aud_dmic_channel_lane(port, channel, &lane) != OPRT_OK) {
        return OPRT_INVALID_PARM;
    }

    set_dig = ((UINT32_T)gain[0] == (UINT32_T)AUDIO_DMIC_INVALID_GAIN_VALUE) ? FALSE : TRUE;
    if (num >= 2U) {
        set_ana = ((UINT32_T)gain[1] == (UINT32_T)AUDIO_DMIC_INVALID_GAIN_VALUE) ? FALSE : TRUE;
    } else {
        set_ana = FALSE;
    }

    if ((set_dig == FALSE) && (set_ana == FALSE)) {
        return OPRT_INVALID_PARM;
    }

    if (set_dig == TRUE) {
        dig_db = (FLOAT_T)gain[0];
    }
    if (set_ana == TRUE) {
        ana_db = gain[1];
    }

    return __t9_aud_cap_set_gain(user, lane,
                                set_dig, dig_db, set_ana, ana_db);
}

/**
 * @brief Stop audio digital (DMIC) capture
 * @param[in] port Digital audio port number
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_aud_dmic_stop(TUYA_AUDIO_DMIC_PORT_E port)
{
    TKL_AUD_CAP_USER_E user = __tkl_aud_dmic_user(port);

    if ((user == TKL_AUD_CAP_USER_MAX) || (__t9_aud_cap_user_attached(user) != TRUE)) {
        return OPRT_COM_ERROR;
    }

    return __t9_aud_cap_stop(user);
}

/**
 * @brief Deinitialize audio digital (DMIC) input
 * @param[in] port Digital audio port number
 * @return OPRT_OK on success, error code on failure
 * @note Detaches only this port from the shared capture core. The hardware is
 *       only released when no other interface is using it either.
 */
OPERATE_RET tkl_aud_dmic_deinit(TUYA_AUDIO_DMIC_PORT_E port)
{
    TKL_AUD_CAP_USER_E user = __tkl_aud_dmic_user(port);

    if ((user == TKL_AUD_CAP_USER_MAX) || (__t9_aud_cap_user_attached(user) != TRUE)) {
        return OPRT_OK;
    }

    return __t9_aud_cap_close(user);
}

/* ===========================================================================
 * CONFIG_AUD_DRIVER_V1 (LEGACY): DMIC has its own fifo and DMA
 * =========================================================================== */
#else

typedef struct {
    TUYA_AUDIO_DMIC_BASE_CFG_T cfg;
    bool                    is_open;
    bool                    dma_started;    /* GDMA was started; REPEAT cannot dma_stop */
    uint32_t                frame_size;
    uint32_t                dma_frame_size;
    dma_id_t                dmic_dma_id;
    RingBufferContext       dmic_rb;
    int8_t                 *dmic_dma_buf;
    uint8_t                *read_buf;
} tkl_aud_dmic_ctx_t;

STATIC tkl_aud_dmic_ctx_t *gl_dmic_ctx = NULL;

STATIC VOID_T __tkl_aud_dmic_pinmux_apply(gpio_dev_t clk_dev, gpio_dev_t data_dev)
{
    gpio_dev_unmap(gl_dmic_pinmux[0].clk_gpio);
    gpio_dev_map(gl_dmic_pinmux[0].clk_gpio, clk_dev);

    gpio_dev_unmap(gl_dmic_pinmux[0].data_gpio);
    gpio_dev_map(gl_dmic_pinmux[0].data_gpio, data_dev);
}

OPERATE_RET tkl_aud_dmic_pinmux_config(TUYA_AUDIO_DMIC_PORT_E port,
                                       UINT32_T clk_gpio, UINT32_T data_gpio)
{
    OPERATE_RET ret = __tkl_aud_dmic_pinmux_validate(clk_gpio, data_gpio);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (gl_dmic_ctx != NULL) {
        TKL_AUD_DMIC_LOG("dmic: pinmux config should be set before init\n");
        return OPRT_COM_ERROR;
    }

    /* V1 keeps a single FIFO/pad pair for both ports. */
    (void)port;
    gl_dmic_pinmux[0].clk_gpio = (gpio_id_t)clk_gpio;
    gl_dmic_pinmux[0].data_gpio = (gpio_id_t)data_gpio;

    TKL_AUD_DMIC_LOG("dmic: pinmux configured clk=%u data=%u\n",
                     (uint32_t)gl_dmic_pinmux[0].clk_gpio,
                     (uint32_t)gl_dmic_pinmux[0].data_gpio);

    return OPRT_OK;
}

/**
 * @brief DMA finish ISR for DMIC
 * @return none
 */
STATIC VOID_T __tkl_aud_dmic_dma_finish_isr(void)
{
    TUYA_AUDIO_INPUT_BUFFER_T frame;

    if (gl_dmic_ctx == NULL || !gl_dmic_ctx->is_open) {
        return;
    }

    if (gl_dmic_ctx->cfg.upper_cb == NULL) {
        return;
    }

    uint32_t dma_frame_size = gl_dmic_ctx->dma_frame_size;

    uint32_t fill_size = ring_buffer_get_fill_size(&gl_dmic_ctx->dmic_rb);
    if (fill_size < dma_frame_size) {
        return;
    }

    uint32_t read_size = ring_buffer_read(&gl_dmic_ctx->dmic_rb,
                                          gl_dmic_ctx->read_buf,
                                          dma_frame_size);
    if (read_size != dma_frame_size) {
        return;
    }

    os_memset(&frame, 0, sizeof(frame));
    frame.event = TUYA_AUDIO_FRAME_EVENT_DMIC_RX;
    frame.data = gl_dmic_ctx->read_buf;
    frame.data_len = gl_dmic_ctx->frame_size;
    frame.args = gl_dmic_ctx->cfg.args;

    /* V1 always captures LR interleaved; report exactly the selected lanes. */
    frame.chan_num = 0U;
    if (((UINT32_T)gl_dmic_ctx->cfg.chan & (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_0) != 0U) {
        frame.chan[frame.chan_num++] = TUYA_AUDIO_DMIC_CHANNEL_0;
    }
    if (((UINT32_T)gl_dmic_ctx->cfg.chan & (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_1) != 0U) {
        frame.chan[frame.chan_num++] = TUYA_AUDIO_DMIC_CHANNEL_1;
    }

    gl_dmic_ctx->cfg.upper_cb(&frame);
}

STATIC bk_err_t __tkl_aud_dmic_dma_deconfig(void)
{
    if (gl_dmic_ctx == NULL) {
        return BK_OK;
    }

    bk_dma_deinit(gl_dmic_ctx->dmic_dma_id);
    bk_dma_free(DMA_DEV_AUDIO, gl_dmic_ctx->dmic_dma_id);

    if (gl_dmic_ctx->dmic_dma_buf) {
        ring_buffer_clear(&gl_dmic_ctx->dmic_rb);
        audio_dma_mem_free(gl_dmic_ctx->dmic_dma_buf);
        gl_dmic_ctx->dmic_dma_buf = NULL;
    }

    return BK_OK;
}

STATIC bk_err_t __tkl_aud_dmic_dma_config(void)
{
    bk_err_t ret = BK_OK;
    dma_config_t dma_config = {0};
    uint32_t dmic_port_addr;
    uint32_t frame_size = gl_dmic_ctx->dma_frame_size;

    os_memset(&dma_config, 0, sizeof(dma_config_t));

    gl_dmic_ctx->dmic_dma_id = bk_dma_alloc(DMA_DEV_AUDIO);
    if ((gl_dmic_ctx->dmic_dma_id < DMA_ID_0) || (gl_dmic_ctx->dmic_dma_id >= DMA_ID_MAX)) {
        TKL_AUD_DMIC_LOG("dmic: malloc dma fail\n");
        goto exit;
    }

    gl_dmic_ctx->dmic_dma_buf = (int8_t *)audio_dma_mem_calloc(2, frame_size + DMIC_DMA_BUF_SAFE_INTERVAL / 2);
    if (gl_dmic_ctx->dmic_dma_buf == NULL) {
        TKL_AUD_DMIC_LOG("dmic: malloc dma buffer fail\n");
        goto exit;
    }

    dma_config.mode = DMA_WORK_MODE_REPEAT;
    dma_config.chan_prio = 1;
    dma_config.trans_type = DMA_TRANS_DEFAULT;
    dma_config.src.dev = DMA_DEV_AUD_DMIC;
    dma_config.dst.dev = DMA_DEV_DTCM;
    dma_config.src.width = DMA_DATA_WIDTH_32BITS;
    dma_config.dst.width = DMA_DATA_WIDTH_32BITS;

    if (bk_aud_dmic_get_fifo_addr(&dmic_port_addr) != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: get fifo address failed\n");
        goto exit;
    }

    dma_config.src.addr_inc_en = DMA_ADDR_INC_ENABLE;
    dma_config.src.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
    dma_config.src.start_addr = dmic_port_addr;
    dma_config.src.end_addr = dmic_port_addr + 4;

    dma_config.dst.addr_inc_en = DMA_ADDR_INC_ENABLE;
    dma_config.dst.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
    dma_config.dst.start_addr = (uint32_t)(uintptr_t)gl_dmic_ctx->dmic_dma_buf;
    dma_config.dst.end_addr = (uint32_t)(uintptr_t)gl_dmic_ctx->dmic_dma_buf
                              + frame_size * 2 + DMIC_DMA_BUF_SAFE_INTERVAL;

    ret = bk_dma_init(gl_dmic_ctx->dmic_dma_id, &dma_config);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: dma_init fail\n");
        goto exit;
    }

    bk_dma_set_transfer_len(gl_dmic_ctx->dmic_dma_id, frame_size);

    bk_dma_register_isr(gl_dmic_ctx->dmic_dma_id, NULL, (void *)__tkl_aud_dmic_dma_finish_isr);
    bk_dma_enable_finish_interrupt(gl_dmic_ctx->dmic_dma_id);

#if (CONFIG_SPE)
    bk_dma_set_dest_sec_attr(gl_dmic_ctx->dmic_dma_id, DMA_ATTR_SEC);
    bk_dma_set_src_sec_attr(gl_dmic_ctx->dmic_dma_id, DMA_ATTR_SEC);
#endif

    ring_buffer_init(&gl_dmic_ctx->dmic_rb,
                     (uint8_t *)gl_dmic_ctx->dmic_dma_buf,
                     frame_size * 2 + DMIC_DMA_BUF_SAFE_INTERVAL,
                     gl_dmic_ctx->dmic_dma_id,
                     RB_DMA_TYPE_WRITE);

    TKL_AUD_DMIC_LOG("dmic: dma_config dma_id=%d transfer_len=%u fifo_addr=0x%08x dma_buf=%p rb_size=%u\n",
                     gl_dmic_ctx->dmic_dma_id, frame_size, dmic_port_addr,
                     (void *)gl_dmic_ctx->dmic_dma_buf, frame_size * 2 + DMIC_DMA_BUF_SAFE_INTERVAL);

    return BK_OK;

exit:
    __tkl_aud_dmic_dma_deconfig();
    return BK_FAIL;
}

/**
 * @brief Rebuild the DMIC GDMA channel without bk_dma_stop()
 */
STATIC bk_err_t __tkl_aud_dmic_dma_reset(void)
{
    (void)__tkl_aud_dmic_dma_deconfig();
    return __tkl_aud_dmic_dma_config();
}

/**
 * @brief Pause GDMA by parking dst_pause_addr at the ring end
 */
STATIC VOID_T __tkl_aud_dmic_pause_dma(VOID_T)
{
    if ((gl_dmic_ctx == NULL) || (gl_dmic_ctx->dmic_dma_buf == NULL)) {
        return;
    }
    ring_buffer_clear(&gl_dmic_ctx->dmic_rb);
}

OPERATE_RET tkl_aud_dmic_init(TUYA_AUDIO_DMIC_PORT_E port, CONST TUYA_AUDIO_DMIC_BASE_CFG_T *config)
{
    UINT32_T mask;

    if ((config == NULL) || (config->upper_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    /* V1 has a single LR FIFO: L -> BIT(8), R -> BIT(9), both -> BIT(8)|BIT(9). */
    mask = (UINT32_T)config->chan;
    if ((mask != (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_0) &&
        (mask != (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_1) &&
        (mask != ((UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_0 |
                  (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_1))) {
        TKL_AUD_DMIC_LOG("dmic: chan 0x%x not support\n", (unsigned)mask);
        return OPRT_INVALID_PARM;
    }

    if (config->sample_bits != TUYA_AUDIO_SAMPLE_BITS_16) {
        TKL_AUD_DMIC_LOG("dmic: bits %d not support, only 16bits\n", config->sample_bits);
        return OPRT_INVALID_PARM;
    }

    if ((config->sample_rate == 0U) || (config->frame_time_ms == 0U)) {
        return OPRT_INVALID_PARM;
    }

    TKL_AUD_DMIC_LOG("dmic: init request port=%d chan=0x%x bits=%u rate=%u frame_ms=%u args=%p\n",
                     port, (unsigned)mask, config->sample_bits, config->sample_rate,
                     config->frame_time_ms, config->args);

    if (gl_dmic_ctx != NULL) {
        /* Contract: a second init reconfigures the instance and returns OK. */
        TKL_AUD_DMIC_LOG("dmic: already initialized, reconfigure\n");
        (void)tkl_aud_dmic_deinit(port);
    }

    bk_err_t ret = BK_OK;
    gpio_dev_t dmic_clk_func = GPIO_DEV_DMIC0_CLK;
    gpio_dev_t dmic_data_func = GPIO_DEV_DMIC0_DAT;
    (void)port;
    aud_dmic_chl_t dmic_chl = AUD_DMIC_CHL_LR;

    gl_dmic_ctx = (tkl_aud_dmic_ctx_t *)tkl_system_psram_malloc(sizeof(tkl_aud_dmic_ctx_t));
    if (gl_dmic_ctx == NULL) {
        TKL_AUD_DMIC_LOG("dmic: malloc context fail\n");
        return OPRT_MALLOC_FAILED;
    }
    os_memset(gl_dmic_ctx, 0, sizeof(tkl_aud_dmic_ctx_t));
    os_memcpy(&gl_dmic_ctx->cfg, config, sizeof(TUYA_AUDIO_DMIC_BASE_CFG_T));

    aud_dmic_config_t tkl_aud_dmic_cfg = DEFAULT_AUD_DMIC_CONFIG();
    tkl_aud_dmic_cfg.dmic_chl = dmic_chl;
    tkl_aud_dmic_cfg.samp_rate = config->sample_rate;
    tkl_aud_dmic_cfg.clk_src = AUD_CLK_APLL;

    TKL_AUD_DMIC_LOG("dmic config, chl=%d clk_gpio=%u data_gpio=%u\n",
                     dmic_chl,
                     (uint32_t)gl_dmic_pinmux[0].clk_gpio,
                     (uint32_t)gl_dmic_pinmux[0].data_gpio);

    ret = bk_aud_dmic_init(&tkl_aud_dmic_cfg);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: bk_aud_dmic_init fail\n");
        goto __dmic_init_exit;
    }

    __tkl_aud_dmic_pinmux_apply(dmic_clk_func, dmic_data_func);

    gl_dmic_ctx->frame_size = AUD_DMIC_FRAME_SIZE(config->sample_rate, config->sample_bits, config->frame_time_ms, 2);
    gl_dmic_ctx->dma_frame_size = AUD_DMIC_FRAME_SIZE(config->sample_rate, config->sample_bits, config->frame_time_ms, 2);
    TKL_AUD_DMIC_LOG("dmic: computed frame_size=%u dma_frame_size=%u\n",
                     gl_dmic_ctx->frame_size, gl_dmic_ctx->dma_frame_size);

    ret = __tkl_aud_dmic_dma_config();
    TKL_AUD_DMIC_LOG("dmic: __tkl_aud_dmic_dma_config ret=%d\n", ret);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: dma config fail\n");
        goto __dmic_init_exit;
    }

    gl_dmic_ctx->read_buf = (uint8_t *)tkl_system_psram_malloc(gl_dmic_ctx->dma_frame_size);
    if (gl_dmic_ctx->read_buf == NULL) {
        TKL_AUD_DMIC_LOG("dmic: malloc read buffer fail\n");
        goto __dmic_init_exit;
    }

    TKL_AUD_DMIC_LOG("dmic: init success, frame_size=%u, dma_frame_size=%u\n",
                      gl_dmic_ctx->frame_size, gl_dmic_ctx->dma_frame_size);
    return OPRT_OK;

__dmic_init_exit:
    __tkl_aud_dmic_dma_deconfig();
    bk_aud_dmic_deinit();

    if (gl_dmic_ctx) {
        if (gl_dmic_ctx->read_buf) {
            tkl_system_psram_free(gl_dmic_ctx->read_buf);
        }

        tkl_system_psram_free(gl_dmic_ctx);
        gl_dmic_ctx = NULL;
    }

    TKL_AUD_DMIC_LOG("dmic: init failed\n");
    return OPRT_COM_ERROR;
}

OPERATE_RET tkl_aud_dmic_start(TUYA_AUDIO_DMIC_PORT_E port)
{
    bk_err_t ret = BK_OK;

    (void) port;

    if (gl_dmic_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (gl_dmic_ctx->is_open) {
        TKL_AUD_DMIC_LOG("dmic: already started\n");
        return OPRT_OK;
    }

    if (gl_dmic_ctx->dma_started) {
        ret = __tkl_aud_dmic_dma_reset();
        if (ret != BK_OK) {
            TKL_AUD_DMIC_LOG("dmic: dma reset fail on restart\n");
            return OPRT_COM_ERROR;
        }
        gl_dmic_ctx->dma_started = false;
    }

    __tkl_aud_dmic_pause_dma();

    ret = bk_dma_start(gl_dmic_ctx->dmic_dma_id);
    TKL_AUD_DMIC_LOG("dmic: bk_dma_start ret=%d\n", ret);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: dma start fail\n");
        return OPRT_COM_ERROR;
    }
    gl_dmic_ctx->dma_started = true;

    ret = bk_aud_dmic_start();
    TKL_AUD_DMIC_LOG("dmic: bk_aud_dmic_start ret=%d\n", ret);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: start fail\n");
        __tkl_aud_dmic_pause_dma();
        return OPRT_COM_ERROR;
    }

    gl_dmic_ctx->is_open = true;
    TKL_AUD_DMIC_LOG("dmic: start ok\n");

    return OPRT_OK;
}

OPERATE_RET tkl_aud_dmic_set_gain(TUYA_AUDIO_DMIC_PORT_E port, TUYA_AUDIO_MIC_CHANNEL_E channel,
                                  INT32_T gain[], UINT32_T num)
{
    (void)port;
    (void)channel;
    (void)gain;
    (void)num;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_aud_dmic_stop(TUYA_AUDIO_DMIC_PORT_E port)
{
    bk_err_t ret = BK_OK;

    (void) port;

    if (gl_dmic_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (!gl_dmic_ctx->is_open) {
        return OPRT_OK;
    }

    ret = bk_aud_dmic_stop();
    TKL_AUD_DMIC_LOG("dmic: bk_aud_dmic_stop ret=%d\n", ret);

    /* Pause GDMA without bk_dma_stop() (unsupported in REPEAT mode). */
    __tkl_aud_dmic_pause_dma();

    gl_dmic_ctx->is_open = false;

    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: stop fail\n");
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_aud_dmic_deinit(TUYA_AUDIO_DMIC_PORT_E port)
{
    if (gl_dmic_ctx == NULL) {
        return OPRT_OK;
    }

    if (gl_dmic_ctx->is_open) {
        tkl_aud_dmic_stop(port);
    }

    __tkl_aud_dmic_dma_deconfig();
    bk_aud_dmic_deinit();

    if (gl_dmic_ctx->read_buf) {
        tkl_system_psram_free(gl_dmic_ctx->read_buf);
    }

    tkl_system_psram_free(gl_dmic_ctx);
    gl_dmic_ctx = NULL;

    return OPRT_OK;
}

#endif /* CONFIG_AUD_DRIVER_V2 */
