/**
 * @file tkl_aud_adc.c
 * @brief TKL audio analog (ADC) driver implementation
 * @version 1.1
 * @date 2026-10-21
 * @copyright Copyright (c) Tuya Inc.
 *
 * On BK7259 (CONFIG_AUD_DRIVER_V2) the analog and digital mics share one FIFO
 * and one GDMA channel. All of that - DMA, ring buffer, ISR, digmic_en and the
 * per-lane gain store - lives in tkl_aud_capture.c, shared with
 * tkl_aud_dmic.c. This file only keeps the public TKL surface.
 */

#include <common/bk_include.h>
#include <os/os.h>
#include <os/mem.h>
#include <components/audio_param_ctrl.h>

#if CONFIG_AUD_DRIVER_V2
#include <driver/aud_adc_types.h>
#include <driver/aud_adc.h>
#include <driver/aud_common.h>
#include <driver/dma.h>
#include <driver/audio_ring_buff.h>
#include <driver/flash.h>
#include <driver/flash_types.h>
#include <components/bk_audio/audio_pipeline/audio_mem.h>

#include "tuya_cloud_types.h"
#include "tkl_aud_adc.h"
#include "tkl_aud_capture.h"
#else
#include <driver/aud_adc_types.h>
#include <driver/aud_adc.h>
#include <driver/aud_common.h>
#include <driver/dma.h>
#include <driver/audio_ring_buff.h>
#include <driver/flash.h>
#include <driver/flash_types.h>
#include <components/bk_audio/audio_pipeline/audio_mem.h>

#include "tuya_cloud_types.h"
#include "tkl_aud_adc.h"
#endif

extern void bk_printf(const char *fmt, ...);

#define MODULE_LOG_HELPER(fmt, ...) \
    bk_printf(fmt, ##__VA_ARGS__)
#define TKL_AUD_ADC_LOG(...) MODULE_LOG_HELPER(__VA_ARGS__)

#define TKL_AUD_ADC_GAIN_NUM               (2U)         /* dig then ana */

/* ===========================================================================
 * CONFIG_AUD_DRIVER_V2: thin wrapper over the shared capture core
 * =========================================================================== */
#if CONFIG_AUD_DRIVER_V2

/**
 * @brief Check whether gain value means "do not change"
 * @param[in] v gain value from caller
 * @return TRUE if invalid/skip
 */
STATIC BOOL_T __tkl_aud_adc_gain_is_invalid(INT32_T v)
{
    return ((UINT32_T)v == (UINT32_T)AUDIO_ADC_INVALID_GAIN_VALUE) ? TRUE : FALSE;
}

/**
 * @brief Whether @p chan names at least one ADC lane and no foreign bit
 * @param[in] chan channel bitmap from the caller
 * @return TRUE if the bitmap is usable
 * @note Only lanes 0..2 exist on this platform, so BIT(3) and every DMIC bit
 *       are rejected here rather than silently ignored.
 */
STATIC BOOL_T __tkl_aud_adc_chan_supported(TUYA_AUDIO_MIC_CHANNEL_E chan)
{
    UINT32_T mask = (UINT32_T)chan;
    UINT32_T supported = (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_0 |
                         (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_1 |
                         (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_2;

    return ((mask != 0U) && ((mask & ~supported) == 0U)) ? TRUE : FALSE;
}

/**
 * @brief Turn a single ADC channel bit into a lane index
 * @param[in]  channel channel bitmap with exactly one ADC bit
 * @param[out] lane    lane index 0..2
 * @return OPRT_OK on success; OPRT_INVALID_PARM otherwise
 */
STATIC OPERATE_RET __tkl_aud_adc_channel_lane(TUYA_AUDIO_MIC_CHANNEL_E channel, UINT32_T *lane)
{
    UINT32_T bit = (UINT32_T)channel;
    UINT32_T idx;

    for (idx = 0U; idx < 3U; idx++) {
        if (bit == ((UINT32_T)TUYA_AUDIO_ADC_CHANNEL_0 << idx)) {
            *lane = idx;
            return OPRT_OK;
        }
    }

    return OPRT_INVALID_PARM;
}

/**
* @brief Initialize audio ADC
*
* @param[in] port   ADC port (unused on this platform)
* @param[in] config ADC configuration including chan, sample_rate, sample_bits,
*                   frame_time_ms, upper_cb. Gain is set via tkl_aud_adc_set_gain().
*
* @note On BK7259 V2 the onboard 3ch frame layout is [mic1, mic2, mic3] and
*       @c config->chan names the lanes to deliver (BIT(0)..BIT(2)). The
*       callback receives only those lanes, interleaved in ascending order.
*       Re-initializing reconfigures the instance and returns OPRT_OK.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_init(TUYA_AUDIO_ADC_PORT_E port, CONST TUYA_AUDIO_ADC_BASE_CFG_T *config)
{
    TKL_AUD_CAP_REQ_T req;

    if ((config == NULL) || (config->upper_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (port >= TUYA_AUDIO_ADC_PORT_MAX) {
        return OPRT_INVALID_PARM;
    }

    if (__tkl_aud_adc_chan_supported(config->chan) != TRUE) {
        TKL_AUD_ADC_LOG("chan 0x%x not support\n", (unsigned)config->chan);
        return OPRT_INVALID_PARM;
    }

    if (config->sample_bits != TUYA_AUDIO_SAMPLE_BITS_16) {
        TKL_AUD_ADC_LOG("bits: %d not support, only 16bits\n", config->sample_bits);
        return OPRT_INVALID_PARM;
    }

    if ((config->sample_rate == 0U) || (config->frame_time_ms == 0U)) {
        return OPRT_INVALID_PARM;
    }

    os_memset(&req, 0, sizeof(req));
    req.sample_rate = config->sample_rate;
    req.sample_bits = (UINT8_T)config->sample_bits;
    req.frame_time_ms = config->frame_time_ms;
    req.upper_cb = config->upper_cb;
    req.args = config->args;
    req.chan = config->chan;

    {
        OPERATE_RET ret;

        if (__t9_aud_cap_user_attached(TKL_AUD_CAP_USER_ADC) == TRUE) {
            /* Contract: a second init reconfigures the instance and returns OK. */
            ret = __t9_aud_cap_reopen(TKL_AUD_CAP_USER_ADC, &req);
        } else {
            ret = __t9_aud_cap_open(TKL_AUD_CAP_USER_ADC, &req);
        }

        /* Propagate the core's code verbatim: a lane conflict or a parameter
         * mismatch must surface as OPRT_INVALID_PARM, not as a generic
         * OPRT_COM_ERROR that callers cannot act on. */
        if (ret != OPRT_OK) {
            TKL_AUD_ADC_LOG("audio analog mic init failed: %d\n", (int)ret);
            return ret;
        }
    }

    TKL_AUD_ADC_LOG("audio analog mic init success, chan=0x%x rate=%u frame_ms=%u\n",
                    (unsigned)config->chan, config->sample_rate, config->frame_time_ms);
    return OPRT_OK;
}

/**
* @brief Start audio ADC capture
*
* @note The shared capture hardware is started by whichever interface starts
*       first; a second start only raises the reference count.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_start(TUYA_AUDIO_ADC_PORT_E port)
{
    (void)port;

    if (__t9_aud_cap_user_attached(TKL_AUD_CAP_USER_ADC) != TRUE) {
        return OPRT_COM_ERROR;
    }

    return __t9_aud_cap_start(TKL_AUD_CAP_USER_ADC);
}

/**
* @brief Set audio ADC digital/analog gain for one MIC channel
*
* @param[in] port: ADC port (unused on this platform)
* @param[in] channel: one ADC bit of TUYA_AUDIO_MIC_CHANNEL_E,
*                     BIT(0)/BIT(1)/BIT(2) -> MIC1/MIC2/MIC3
* @param[in] gain: gain[0]=dig dB, gain[1]=ana dB (when num>=2). Pass
*                  AUDIO_ADC_INVALID_GAIN_VALUE to leave dig or ana unchanged.
* @param[in] num: 1 (dig only) or 2 (dig then ana)
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_set_gain(TUYA_AUDIO_ADC_PORT_E port, TUYA_AUDIO_MIC_CHANNEL_E channel,
                                 INT32_T gain[], UINT32_T num)
{
    BOOL_T set_dig;
    BOOL_T set_ana;
    FLOAT_T dig_db = 0.0f;
    INT32_T ana_db = 0;
    UINT32_T lane = 0U;
    OPERATE_RET rt;

    (void)port;

    if (__t9_aud_cap_user_attached(TKL_AUD_CAP_USER_ADC) != TRUE) {
        return OPRT_COM_ERROR;
    }

    if ((gain == NULL) || (num == 0U) || (num > TKL_AUD_ADC_GAIN_NUM)) {
        return OPRT_INVALID_PARM;
    }

    rt = __tkl_aud_adc_channel_lane(channel, &lane);
    if (rt != OPRT_OK) {
        return rt;
    }

    set_dig = (__tkl_aud_adc_gain_is_invalid(gain[0]) == TRUE) ? FALSE : TRUE;
    if (num >= 2U) {
        set_ana = (__tkl_aud_adc_gain_is_invalid(gain[1]) == TRUE) ? FALSE : TRUE;
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

    return __t9_aud_cap_set_gain(TKL_AUD_CAP_USER_ADC, lane,
                                set_dig, dig_db, set_ana, ana_db);
}

/**
* @brief Stop audio ADC capture
*
* @note The last interface to stop parks the shared stream.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_stop(TUYA_AUDIO_ADC_PORT_E port)
{
    (void)port;

    if (__t9_aud_cap_user_attached(TKL_AUD_CAP_USER_ADC) != TRUE) {
        return OPRT_COM_ERROR;
    }

    return __t9_aud_cap_stop(TKL_AUD_CAP_USER_ADC);
}

/**
* @brief Deinitialize audio ADC
*
* @note Detaches from the shared capture core. The hardware is only released
*       when the digital interface is not using it either.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_deinit(TUYA_AUDIO_ADC_PORT_E port)
{
    (void)port;

    if (__t9_aud_cap_user_attached(TKL_AUD_CAP_USER_ADC) != TRUE) {
        return OPRT_OK;
    }

    return __t9_aud_cap_close(TKL_AUD_CAP_USER_ADC);
}

/* ===========================================================================
 * CONFIG_AUD_DRIVER_V1 (LEGACY): analog ADC has its own fifo and DMA
 * =========================================================================== */
#else

// sr: sample rate, sb: sample bits, ch: channel num, t: frame time in ms
#define AUD_ADC_FRAME_SIZE(sr, sb, t, ch)  (((sr) * ((sb) / 8) / 1000) * (ch) * (t))

/*
 * DMA_BUF_SAFE_INTERVAL works with ring_buffer_init / ring_buffer_read /
 * ring_buffer_clear to manage dma_set_dst_pause_addr for DMA flow control:
 *   - ring_buffer_init:  pause_addr = buf + capacity - 8
 *   - ring_buffer_clear: pause_addr = buf + capacity - 4
 *   - ring_buffer_read:  pause_addr = rp (or rp-4 boundary case)
 * This prevents DMA from overwriting unread data.
 */
#define DMA_BUF_SAFE_INTERVAL    (8)
#define DMA_BUF_FRAME_CNT        (8)

typedef struct {
    TUYA_AUDIO_ADC_BASE_CFG_T cfg;
    bool                is_open;
    bool                dma_started;       /* GDMA was started; REPEAT cannot dma_stop */
    uint32_t            dma_frame_size;
    uint8_t             chl_num;
    uint8_t             raw_ch_num;
    uint32_t            dig_reg;           /**< Last dig register gain from set_gain */
    BOOL_T              gain_set;          /**< TRUE after first successful set_gain */
    dma_id_t            mic_dma_id;
    RingBufferContext   mic_rb;
    int8_t             *mic_dma_buf;
    uint8_t            *read_buf;          /* dma_frame_size: ring_buffer_read target */
    uint8_t            *output_buf;        /* frame_size: extracted analog mic payload */
    uint32_t            output_buf_size;
} tkl_aud_adc_ctx_t;

static tkl_aud_adc_ctx_t *gl_adc_ctx = NULL;

/**
 * @brief Check whether gain value means "do not change"
 * @param[in] v gain value from caller
 * @return TRUE if invalid/skip
 */
static BOOL_T __tkl_aud_adc_gain_is_invalid(INT32_T v)
{
    return ((UINT32_T)v == (UINT32_T)AUDIO_ADC_INVALID_GAIN_VALUE) ? TRUE : FALSE;
}

/* ---------------------- flash operation notify ------------------------------ */

static void __tkl_aud_adc_flash_op_notify_handler(uint32_t param, void *args)
{
    (void)param;
    (void)args;
    /*
     * Keep ADC capture running in Tuya AP build.
     * Reference onboard_mic_stream_v2 disables this callback path to avoid
     * frequent flash ops starving audio capture.
     */
    return;
}

/* ---------------------- DMA ISR -------------------------------------------- */

/**
 * Turn one chunk already sitting in read_buf into the PCM the caller asked for
 * and hand it to upper_cb.
 */
static void __tkl_aud_adc_push_frame(uint32_t read_size)
{
    TUYA_AUDIO_INPUT_BUFFER_T frame;

    os_memset(&frame, 0, sizeof(frame));
    frame.event = TUYA_AUDIO_FRAME_EVENT_ADC_RX;
    frame.args = gl_adc_ctx->cfg.args;

    if (gl_adc_ctx->chl_num == 1)
    {
        // PCM DATA: LRLRLRLR.....
        int16_t *src = (int16_t *)gl_adc_ctx->read_buf;
        int16_t *dst = (int16_t *)gl_adc_ctx->output_buf;
        uint32_t offset;

        // T5仅支持16位深采样
        uint32_t samples = gl_adc_ctx->output_buf_size / sizeof(int16_t);

        if ((UINT32_T)gl_adc_ctx->cfg.chan == (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_1)
        {
            // 提取右声道数据
            offset = 1U;
        }
        else
        {
            // 默认左声道数据
            offset = 0U;
        }

        for (uint32_t i = 0; i < samples; i++)
        {
            dst[i] = src[2 * i + offset];
        }

        frame.data = gl_adc_ctx->output_buf;
        frame.data_len = gl_adc_ctx->output_buf_size;
        frame.chan[0] = gl_adc_ctx->cfg.chan;
        frame.chan_num = 1U;
    }
    else
    {
        frame.data = gl_adc_ctx->read_buf;
        frame.data_len = read_size;
        frame.chan[0] = TUYA_AUDIO_ADC_CHANNEL_0;
        frame.chan[1] = TUYA_AUDIO_ADC_CHANNEL_1;
        frame.chan_num = 2U;
    }

    gl_adc_ctx->cfg.upper_cb(&frame);
}

/*
 * DMA finish ISR: at least one dma_frame_size chunk has landed in the ring buffer.
 *
 * ring_buffer_read does three things atomically:
 *   1. Gets the DMA write position from the hardware register (wp)
 *   2. Copies data from the ring buffer into read_buf
 *   3. Updates dma_set_dst_pause_addr so the DMA can keep writing
 */
static void __tkl_aud_adc_dma_finish_isr(void)
{
    uint32_t dma_frame_size;

    if (gl_adc_ctx == NULL || !gl_adc_ctx->is_open)
    {
        return;
    }

    if (gl_adc_ctx->cfg.upper_cb == NULL)
    {
        return;
    }

    dma_frame_size = gl_adc_ctx->dma_frame_size;
    if (dma_frame_size == 0U)
    {
        return;
    }

    while (ring_buffer_get_fill_size(&gl_adc_ctx->mic_rb) >= dma_frame_size)
    {
        uint32_t read_size = ring_buffer_read(&gl_adc_ctx->mic_rb,
                                              gl_adc_ctx->read_buf,
                                              dma_frame_size);
        if (read_size != dma_frame_size)
        {
            break;
        }

        __tkl_aud_adc_push_frame(read_size);
    }
}

/* ---------------------- DMA config / deconfig ------------------------------ */

static bk_err_t __tkl_aud_adc_dma_deconfig(void)
{
    if (gl_adc_ctx == NULL)
    {
        return BK_OK;
    }

    bk_dma_deinit(gl_adc_ctx->mic_dma_id);
    bk_dma_free(DMA_DEV_AUDIO, gl_adc_ctx->mic_dma_id);

    if (gl_adc_ctx->mic_dma_buf)
    {
        ring_buffer_clear(&gl_adc_ctx->mic_rb);
        audio_dma_mem_free(gl_adc_ctx->mic_dma_buf);
        gl_adc_ctx->mic_dma_buf = NULL;
    }

    return BK_OK;
}

static bk_err_t __tkl_aud_adc_dma_config(void)
{
    bk_err_t ret = BK_OK;
    dma_config_t dma_config = {0};
    uint32_t adc_port_addr;
    uint32_t frame_size = gl_adc_ctx->dma_frame_size;

    os_memset(&dma_config, 0, sizeof(dma_config_t));

    gl_adc_ctx->mic_dma_id = bk_dma_alloc(DMA_DEV_AUDIO);
    if ((gl_adc_ctx->mic_dma_id < DMA_ID_0) || (gl_adc_ctx->mic_dma_id >= DMA_ID_MAX))
    {
        TKL_AUD_ADC_LOG("malloc dma fail\n");
        goto exit;
    }

    /* DMA_BUF_FRAME_CNT frames + safe interval for dma_set_dst_pause_addr flow control */
    gl_adc_ctx->mic_dma_buf = (int8_t *)audio_dma_mem_calloc(DMA_BUF_FRAME_CNT,
                                          frame_size + DMA_BUF_SAFE_INTERVAL / DMA_BUF_FRAME_CNT);
    if (gl_adc_ctx->mic_dma_buf == NULL)
    {
        TKL_AUD_ADC_LOG("malloc dma buffer fail\n");
        goto exit;
    }

    dma_config.mode = DMA_WORK_MODE_REPEAT;
    dma_config.chan_prio = 1;
    dma_config.trans_type = DMA_TRANS_DEFAULT;
    dma_config.src.dev = DMA_DEV_AUDIO_RX;
    dma_config.dst.dev = DMA_DEV_DTCM;
    dma_config.src.width = DMA_DATA_WIDTH_32BITS;
    dma_config.dst.width = DMA_DATA_WIDTH_32BITS;

    if (bk_aud_adc_get_fifo_addr(&adc_port_addr) != BK_OK)
    {
        TKL_AUD_ADC_LOG("get adc fifo address failed\n");
        goto exit;
    }

    dma_config.src.addr_inc_en = DMA_ADDR_INC_ENABLE;
    dma_config.src.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
    dma_config.src.start_addr = adc_port_addr;
    dma_config.src.end_addr = adc_port_addr + 4;

    dma_config.dst.addr_inc_en = DMA_ADDR_INC_ENABLE;
    dma_config.dst.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
    dma_config.dst.start_addr = (uint32_t)(uintptr_t)gl_adc_ctx->mic_dma_buf;
    dma_config.dst.end_addr = (uint32_t)(uintptr_t)gl_adc_ctx->mic_dma_buf
                              + frame_size * DMA_BUF_FRAME_CNT + DMA_BUF_SAFE_INTERVAL;

    ret = bk_dma_init(gl_adc_ctx->mic_dma_id, &dma_config);
    if (ret != BK_OK)
    {
        TKL_AUD_ADC_LOG("dma_init fail\n");
        goto exit;
    }

    bk_dma_set_transfer_len(gl_adc_ctx->mic_dma_id, frame_size);

    bk_dma_register_isr(gl_adc_ctx->mic_dma_id, NULL, (void *)__tkl_aud_adc_dma_finish_isr);
    bk_dma_enable_finish_interrupt(gl_adc_ctx->mic_dma_id);

#if (CONFIG_SPE)
    bk_dma_set_dest_sec_attr(gl_adc_ctx->mic_dma_id, DMA_ATTR_SEC);
    bk_dma_set_src_sec_attr(gl_adc_ctx->mic_dma_id, DMA_ATTR_SEC);
#endif

    /* ring_buffer_init sets dma_set_dst_pause_addr(dma_id, buf + capacity - 8) */
    ring_buffer_init(&gl_adc_ctx->mic_rb,
                     (uint8_t *)gl_adc_ctx->mic_dma_buf,
                     frame_size * DMA_BUF_FRAME_CNT + DMA_BUF_SAFE_INTERVAL,
                     gl_adc_ctx->mic_dma_id,
                     RB_DMA_TYPE_WRITE);

    TKL_AUD_ADC_LOG("dma_config dma_id: %d, transfer_len: %d\n",
                     gl_adc_ctx->mic_dma_id, frame_size);

    return BK_OK;

exit:
    __tkl_aud_adc_dma_deconfig();
    return BK_FAIL;
}

/**
 * @brief Rebuild the ADC GDMA channel without bk_dma_stop()
 * @return BK_OK on success
 */
static bk_err_t __tkl_aud_adc_dma_reset(void)
{
    (void)__tkl_aud_adc_dma_deconfig();
    return __tkl_aud_adc_dma_config();
}

/**
 * @brief Pause GDMA by parking dst_pause_addr at the ring end
 */
static VOID_T __tkl_aud_adc_pause_dma(VOID_T)
{
    if ((gl_adc_ctx == NULL) || (gl_adc_ctx->mic_dma_buf == NULL)) {
        return;
    }
    ring_buffer_clear(&gl_adc_ctx->mic_rb);
}

/* ---------------------- Public API ----------------------------------------- */

OPERATE_RET tkl_aud_adc_init(TUYA_AUDIO_ADC_PORT_E port, CONST TUYA_AUDIO_ADC_BASE_CFG_T *config)
{
    if ((config == NULL) || (config->upper_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (gl_adc_ctx != NULL) {
        /* Contract: a second init reconfigures the instance and returns OK. */
        TKL_AUD_ADC_LOG("adc already initialized, reconfigure\n");
        (void)tkl_aud_adc_deinit(port);
    }

    (void)port;

    bk_err_t ret = BK_OK;
    uint8_t chl_num = 2;
    uint8_t raw_ch_num = 2;
    aud_adc_config_t aud_adc_cfg = DEFAULT_AUD_ADC_CONFIG();
    aud_mic_id_t mic_id = AUD_MIC_BOTH;
    aud_adc_mode_t adc_mode = AUD_ADC_MODE_DIFFEN;

    gl_adc_ctx = (tkl_aud_adc_ctx_t *)tkl_system_psram_malloc(sizeof(tkl_aud_adc_ctx_t));
    if (gl_adc_ctx == NULL) {
        TKL_AUD_ADC_LOG("malloc adc context fail\n");
        return OPRT_MALLOC_FAILED;
    }
    os_memset(gl_adc_ctx, 0, sizeof(tkl_aud_adc_ctx_t));
    os_memcpy(&gl_adc_ctx->cfg, config, sizeof(TUYA_AUDIO_ADC_BASE_CFG_T));
    gl_adc_ctx->gain_set = FALSE;
    gl_adc_ctx->dig_reg = 0x2DU; /* OEM DEFAULT_AUD_ADC_CONFIG adc_gain */

    {
        UINT32_T mask = (UINT32_T)config->chan;
        UINT32_T supported = (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_0 |
                             (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_1;

        if ((mask == 0U) || ((mask & ~supported) != 0U)) {
            TKL_AUD_ADC_LOG("chan 0x%x not support\n", (unsigned)mask);
            goto __adc_init_exit;
        }

        if ((mask == (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_0) ||
            (mask == (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_1)) {
            /* V1 ADC always outputs LR interleaved data on one fifo bus. */
            chl_num = 1;
        } else {
            mic_id = AUD_MIC_BOTH;
            adc_mode = AUD_ADC_MODE_DIFFEN;
            chl_num = 2;
        }
    }

    if (config->sample_bits != TUYA_AUDIO_SAMPLE_BITS_16) {
        TKL_AUD_ADC_LOG("bits: %d not support, only 16bits\n", config->sample_bits);
        goto __adc_init_exit;
    }

    gl_adc_ctx->chl_num = chl_num;
    gl_adc_ctx->raw_ch_num = raw_ch_num;
    /* V1 keeps LR interleaved capture and extracts one channel in ISR. */
    gl_adc_ctx->dma_frame_size = AUD_ADC_FRAME_SIZE(config->sample_rate, config->sample_bits, config->frame_time_ms, 2);

    /* Gain applied later via tkl_aud_adc_set_gain(); keep OEM default for HW init. */
    aud_adc_cfg.adc_chl = AUD_ADC_CHL_LR;
    aud_adc_cfg.samp_rate = config->sample_rate;
    aud_adc_cfg.adc_gain = gl_adc_ctx->dig_reg;
    aud_adc_cfg.clk_src = AUD_CLK_APLL;
    aud_adc_cfg.adc_mode = adc_mode;
    TKL_AUD_ADC_LOG("adc config, chan %d, dig_reg 0x%02x, samp_rate: %d, clk_src: %s, adc_mode: %s\n",
                     aud_adc_cfg.adc_chl, aud_adc_cfg.adc_gain, aud_adc_cfg.samp_rate,
                     aud_adc_cfg.clk_src == 1 ? "APLL" : "XTAL",
                     aud_adc_cfg.adc_mode == 1 ? "AUD_ADC_MODE_SIGNAL_END" : "AUD_ADC_MODE_DIFFEN");

    /*
     * Bring audio common driver (APB + ana defaults) up before digital IP
     * reset. After a full driver_deinit that zeros ana_reg20~30, resetting
     * first then init leaves ADC producing DMA frames of silence.
     */
    (void)bk_aud_driver_init();
    bk_aud_hardware_reset();
    ret = bk_aud_adc_init(&aud_adc_cfg);
    if (ret != BK_OK) {
        TKL_AUD_ADC_LOG("bk_aud_adc_init fail\n");
        goto __adc_init_exit;
    }

    ret = bk_aud_adc_set_mic_mode(mic_id, adc_mode);
    if (ret != BK_OK) {
        TKL_AUD_ADC_LOG("bk_aud_adc_set_mic_mode fail, ret %d mic_id %d mode %d\n",
                         ret, mic_id, adc_mode);
        goto __adc_init_exit;
    }

    ret = __tkl_aud_adc_dma_config();
    if (ret != BK_OK) {
        TKL_AUD_ADC_LOG("dma config fail\n");
        goto __adc_init_exit;
    }

    /* read_buf: ring_buffer_read destination, always dma_frame_size */
    gl_adc_ctx->read_buf = (uint8_t *)tkl_system_psram_malloc(gl_adc_ctx->dma_frame_size);
    if (gl_adc_ctx->read_buf == NULL) {
        TKL_AUD_ADC_LOG("malloc read buffer fail\n");
        goto __adc_init_exit;
    }

    /* output_buf: extraction result for the single channel case. */
    if (raw_ch_num != chl_num) {
        gl_adc_ctx->output_buf_size = AUD_ADC_FRAME_SIZE(gl_adc_ctx->cfg.sample_rate, gl_adc_ctx->cfg.sample_bits, gl_adc_ctx->cfg.frame_time_ms, chl_num);
        gl_adc_ctx->output_buf = (uint8_t *)tkl_system_psram_malloc(gl_adc_ctx->output_buf_size);
        if (gl_adc_ctx->output_buf == NULL) {
            TKL_AUD_ADC_LOG("malloc output buffer fail\n");
            goto __adc_init_exit;
        }
    }

    bk_aud_set_ana_mic0_gain(0x08);
    bk_aud_set_ana_mic1_gain(0x08);

    /* 4.0.1.11 dropped the per-peripheral onboard-mic flash-notify API; the generic
     * registry takes the same void(uint32_t busy, void *args) callback. */
    mb_flash_register_op_notify_cb(__tkl_aud_adc_flash_op_notify_handler, NULL);

    TKL_AUD_ADC_LOG("audio analog mic init success\n");
    return OPRT_OK;

__adc_init_exit:
    __tkl_aud_adc_dma_deconfig();
    bk_aud_adc_deinit();

    if (gl_adc_ctx) {
        if (gl_adc_ctx->read_buf) {
            tkl_system_psram_free(gl_adc_ctx->read_buf);
        }
        if (gl_adc_ctx->output_buf) {
            tkl_system_psram_free(gl_adc_ctx->output_buf);
        }
        tkl_system_psram_free(gl_adc_ctx);
        gl_adc_ctx = NULL;
    }

    TKL_AUD_ADC_LOG("audio analog mic init failed\n");
    return OPRT_COM_ERROR;
}

OPERATE_RET tkl_aud_adc_start(TUYA_AUDIO_ADC_PORT_E port)
{
    bk_err_t ret = BK_OK;

    (void) port;
    if (gl_adc_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (gl_adc_ctx->is_open)
    {
        TKL_AUD_ADC_LOG("aud adc had been open\n");
        return OPRT_OK;
    }

    if (gl_adc_ctx->dma_started) {
        ret = __tkl_aud_adc_dma_reset();
        if (ret != BK_OK) {
            TKL_AUD_ADC_LOG("dma reset fail on restart\n");
            return OPRT_COM_ERROR;
        }
        gl_adc_ctx->dma_started = false;
    }

    __tkl_aud_adc_pause_dma();

    ret = bk_dma_start(gl_adc_ctx->mic_dma_id);
    if (ret != BK_OK)
    {
        TKL_AUD_ADC_LOG("dma start fail\n");
        return OPRT_COM_ERROR;
    }
    gl_adc_ctx->dma_started = true;

    ret = bk_aud_adc_start();
    if (ret != BK_OK)
    {
        TKL_AUD_ADC_LOG("adc start fail\n");
        __tkl_aud_adc_pause_dma();
        return OPRT_COM_ERROR;
    }

    gl_adc_ctx->is_open = true;
    TKL_AUD_ADC_LOG("adc start ok\n");

    return OPRT_OK;
}

OPERATE_RET tkl_aud_adc_set_gain(TUYA_AUDIO_ADC_PORT_E port, TUYA_AUDIO_MIC_CHANNEL_E channel,
                                 INT32_T gain[], UINT32_T num)
{
    (void)port;

    if (gl_adc_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if ((gain == NULL) || (num == 0U) || (num > TKL_AUD_ADC_GAIN_NUM)) {
        return OPRT_INVALID_PARM;
    }

    {
        UINT32_T mask = (UINT32_T)channel;
        uint32_t hw_gain;

        /* V1 has a single digital gain register shared by both lanes. */
        if ((mask == 0U) ||
            ((mask & ~((UINT32_T)TUYA_AUDIO_ADC_CHANNEL_0 |
                       (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_1)) != 0U)) {
            return OPRT_INVALID_PARM;
        }

        if (__tkl_aud_adc_gain_is_invalid(gain[0]) == TRUE) {
            return OPRT_INVALID_PARM;
        }

        hw_gain = (gain[0] < 0) ? 0U : (uint32_t)gain[0];
        if (hw_gain > 0x3FU) {
            TKL_AUD_ADC_LOG("gain 0x%02x out of range: 0x00 ~ 0x3f\n", hw_gain);
            return OPRT_INVALID_PARM;
        }

        if (bk_aud_adc_set_gain(hw_gain) != BK_OK) {
            TKL_AUD_ADC_LOG("set digital gain fail\n");
            return OPRT_COM_ERROR;
        }

        gl_adc_ctx->dig_reg = hw_gain;
        gl_adc_ctx->gain_set = TRUE;
        return OPRT_OK;
    }
}

OPERATE_RET tkl_aud_adc_stop(TUYA_AUDIO_ADC_PORT_E port)
{
    bk_err_t ret = BK_OK;

    (void) port;
    if (gl_adc_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (!gl_adc_ctx->is_open)
    {
        return OPRT_OK;
    }

    ret = bk_aud_adc_stop();

    /* Pause GDMA without bk_dma_stop() (unsupported in REPEAT mode). */
    __tkl_aud_adc_pause_dma();

    gl_adc_ctx->is_open = false;

    if (ret != BK_OK)
    {
        TKL_AUD_ADC_LOG("adc stop fail\n");
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_aud_adc_deinit(TUYA_AUDIO_ADC_PORT_E port)
{
    (void) port;
    if (gl_adc_ctx == NULL)
    {
        return OPRT_OK;
    }

    if (gl_adc_ctx->is_open)
    {
        tkl_aud_adc_stop(port);
    }

    __tkl_aud_adc_dma_deconfig();

    bk_aud_adc_deinit();

    mb_flash_unregister_op_notify_cb(__tkl_aud_adc_flash_op_notify_handler);

    if (gl_adc_ctx->read_buf)
    {
        tkl_system_psram_free(gl_adc_ctx->read_buf);
    }
    if (gl_adc_ctx->output_buf)
    {
        tkl_system_psram_free(gl_adc_ctx->output_buf);
    }
    tkl_system_psram_free(gl_adc_ctx);
    gl_adc_ctx = NULL;

    return OPRT_OK;
}

#endif /* CONFIG_AUD_DRIVER_V2 */
