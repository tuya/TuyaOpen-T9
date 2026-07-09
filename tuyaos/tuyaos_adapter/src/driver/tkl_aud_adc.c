
#include <common/bk_include.h>
#include <math.h>
#include <os/os.h>
#include <components/audio_param_ctrl.h>

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

#ifndef TUYA_AUDIO_ADC_CHANNEL_MIC3
#define TUYA_AUDIO_ADC_CHANNEL_MIC3   ((TUYA_AUDIO_ADC_CHAN_E)3)
#endif

extern void bk_printf(const char *fmt, ...);

#define MODULE_LOG_HELPER(fmt, ...) \
    bk_printf(fmt, ##__VA_ARGS__)
#define TKL_AUD_ADC_LOG(...) MODULE_LOG_HELPER(__VA_ARGS__)

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

#if CONFIG_AUD_DRIVER_V2
#define TKL_AUD_ADC_LOGICAL_GAIN_DB0       (0x2D)      /* 0 dB point */
#define TKL_AUD_ADC_V2_DIG_GAIN_DB0_REG    (0x4000U)   /* unity gain */
#define TKL_AUD_ADC_V2_DIG_GAIN_MAX        (0x1FFFFU)  /* 17-bit gain field */
#define TKL_AUD_ADC_V2_DIG_GAIN_SAFE_MAX   (0x2AU)     /* -3 dB safety ceiling for speech headroom */
#define TKL_AUD_ADC_V2_ANA_GAIN_DEFAULT    (0x07U)     /* SDK default analog preamp */
#endif

typedef struct {
    TKL_AUD_ADC_CFG_T   cfg;
    bool                is_open;
    uint32_t            dma_frame_size;
    uint8_t             chl_num;
    uint8_t             raw_ch_num;
#if CONFIG_AUD_DRIVER_V2
    uint32_t            adc_ch_bitmap;
#endif
    dma_id_t            mic_dma_id;
    RingBufferContext   mic_rb;
    int8_t             *mic_dma_buf;
    uint8_t            *read_buf;          /* dma_frame_size: ring_buffer_read target */
    uint8_t            *output_buf;        /* frame_size: extracted analog mic payload */
    uint32_t            output_buf_size;
} tkl_aud_adc_ctx_t;

static tkl_aud_adc_ctx_t *gl_adc_ctx = NULL;

#if CONFIG_AUD_DRIVER_V2
/*
 * Convert logical gain [0, 0x3F] used by upper layer to V2 digital gain reg.
 * Audio stack convention: 0x2D => 0 dB, step is 1 dB.
 */
static uint32_t __tkl_aud_adc_map_gain_to_dig_reg(uint32_t gain)
{
    float db = (float)((int32_t)gain - (int32_t)TKL_AUD_ADC_LOGICAL_GAIN_DB0);
    float linear = powf(10.0f, db / 20.0f);
    uint32_t reg;

    if (linear <= 0.0f) {
        return 0;
    }

    reg = (uint32_t)(linear * (float)TKL_AUD_ADC_V2_DIG_GAIN_DB0_REG + 0.5f);
    if (reg > TKL_AUD_ADC_V2_DIG_GAIN_MAX) {
        reg = TKL_AUD_ADC_V2_DIG_GAIN_MAX;
    }
    return reg;
}
#endif

#if CONFIG_AUD_DRIVER_V2
static bk_err_t __tkl_aud_adc_start_channels(uint32_t ch_bitmap, uint32_t *started_bitmap)
{
    bk_err_t ret = BK_OK;
    uint32_t local_started = 0;

    for (uint32_t ch = 0; ch < AUD_ADC_CHL_MAX; ch++) {
        if (!(ch_bitmap & (1U << ch))) {
            continue;
        }

        ret = bk_aud_adc_start((aud_adc_chl_t)ch);
        if (ret != BK_OK) {
            for (uint32_t stop_ch = 0; stop_ch < AUD_ADC_CHL_MAX; stop_ch++) {
                if (local_started & (1U << stop_ch)) {
                    (void)bk_aud_adc_stop((aud_adc_chl_t)stop_ch);
                }
            }

            if (started_bitmap != NULL) {
                *started_bitmap = 0;
            }
            return ret;
        }

        local_started |= (1U << ch);
    }

    if (started_bitmap != NULL) {
        *started_bitmap = local_started;
    }

    return BK_OK;
}

static void __tkl_aud_adc_stop_channels(uint32_t ch_bitmap)
{
    for (uint32_t ch = 0; ch < AUD_ADC_CHL_MAX; ch++) {
        if (ch_bitmap & (1U << ch)) {
            (void)bk_aud_adc_stop((aud_adc_chl_t)ch);
        }
    }
}
#endif

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

#if 0
    if (gl_adc_ctx == NULL || !gl_adc_ctx->is_open)
    {
        return;
    }

    if (param)
    {
        bk_dma_stop(gl_adc_ctx->mic_dma_id);
#if CONFIG_AUD_DRIVER_V2
        __tkl_aud_adc_stop_channels(gl_adc_ctx->adc_ch_bitmap);
#else
        bk_aud_adc_stop();
#endif
        ring_buffer_clear(&gl_adc_ctx->mic_rb);
    }
    else
    {
        bk_dma_start(gl_adc_ctx->mic_dma_id);
#if CONFIG_AUD_DRIVER_V2
        if (BK_OK == __tkl_aud_adc_start_channels(gl_adc_ctx->adc_ch_bitmap, NULL)) {
            bk_aud_adc_enable_used_channel(gl_adc_ctx->adc_ch_bitmap);
        }
#else
        bk_aud_adc_start();
#endif
    }
#endif
}

/* ---------------------- DMA ISR -------------------------------------------- */

/**
 * DMA finish ISR: one dma_frame_size chunk has been written into the ring buffer.
 *
 * ring_buffer_read does three things atomically:
 *   1. Gets DMA write position from hardware register (wp)
 *   2. Copies data from ring buffer to read_buf
 *   3. Updates dma_set_dst_pause_addr so DMA can continue writing
 *
 * For single channel: extract L from L+R interleaved read_buf into output_buf
 * For dual   channel: read_buf already contains the final PCM data
 *
 * Then call upper_aud_adc_cb(pcm_data_ptr) to push data to TAL layer.
 */
static void __tkl_aud_adc_dma_finish_isr(void)
{
    if (gl_adc_ctx == NULL || !gl_adc_ctx->is_open)
    {
        return;
    }

    if (gl_adc_ctx->cfg.upper_cb == NULL)
    {
        return;
    }

    uint32_t dma_frame_size = gl_adc_ctx->dma_frame_size;

    uint32_t fill_size = ring_buffer_get_fill_size(&gl_adc_ctx->mic_rb);
    if (fill_size < dma_frame_size)
    {
        return;
    }

    uint32_t read_size = ring_buffer_read(&gl_adc_ctx->mic_rb,
                                          gl_adc_ctx->read_buf,
                                          dma_frame_size);
    if (read_size != dma_frame_size)
    {
        return;
    }

    if (gl_adc_ctx->chl_num == 1)
    {
#if CONFIG_AUD_DRIVER_V2
        int16_t *src = (int16_t *)gl_adc_ctx->read_buf;
        int16_t *dst = (int16_t *)gl_adc_ctx->output_buf;
        uint32_t frame_pts;
        uint32_t raw_ch_num = gl_adc_ctx->raw_ch_num;

        if ((raw_ch_num == 0U) || ((read_size & 0x1U) != 0U)) {
            return;
        }

        if (raw_ch_num == 1U) {
            gl_adc_ctx->cfg.upper_cb(TUYA_AUDIO_FRAME_EVENT_ADC_RX,
                                     gl_adc_ctx->read_buf,
                                     read_size,
                                     gl_adc_ctx->cfg.args);
            return;
        }

        if (dst == NULL) {
            return;
        }

        frame_pts = read_size / (sizeof(int16_t) * raw_ch_num);
        if ((frame_pts == 0U) || ((frame_pts * sizeof(int16_t)) > gl_adc_ctx->output_buf_size)) {
            return;
        }

        /*
         * Mixed 3ch layout from reference pipeline:
         * [ref, mic1, mic2, ref, mic1, mic2, ...]
         * For single-channel request:
         *   MIC3 -> ref (AEC reference, index 0)
         *   L    -> mic1
         *   R    -> mic2
         */
        if (raw_ch_num == 3U) {
            uint32_t mic_idx;

            if (gl_adc_ctx->cfg.chan == TUYA_AUDIO_ADC_CHANNEL_MIC3) {
                mic_idx = 0U;
            } else if (gl_adc_ctx->cfg.chan == TUYA_AUDIO_ADC_CHANNEL_R) {
                mic_idx = 2U;
            } else {
                mic_idx = 1U;
            }
            for (uint32_t i = 0; i < frame_pts; i++) {
                dst[i] = src[i * 3U + mic_idx];
            }
        } else {
            uint32_t lane = (gl_adc_ctx->cfg.chan == TUYA_AUDIO_ADC_CHANNEL_R) ? 1U : 0U;

            for (uint32_t i = 0; i < frame_pts; i++) {
                dst[i] = src[2U * i + lane];
            }
        }

        gl_adc_ctx->cfg.upper_cb(TUYA_AUDIO_FRAME_EVENT_ADC_RX,
                                 gl_adc_ctx->output_buf,
                                 frame_pts * sizeof(int16_t),
                                 gl_adc_ctx->cfg.args);
#else
        // PCM DATA: LRLRLRLR.....
        int16_t *src = (int16_t *)gl_adc_ctx->read_buf;
        int16_t *dst = (int16_t *)gl_adc_ctx->output_buf;
        
        // T5仅支持16位深采样
        uint32_t samples = gl_adc_ctx->output_buf_size / sizeof(int16_t);
        if (gl_adc_ctx->cfg.chan == TUYA_AUDIO_ADC_CHANNEL_L)
        {
            // 提取左声道数据
            for (uint32_t i = 0; i < samples; i++)
            {
                dst[i] = src[2 * i];
            }
        }
        else if (gl_adc_ctx->cfg.chan == TUYA_AUDIO_ADC_CHANNEL_R)
        {
            // 提取右声道数据
            for (uint32_t i = 0; i < samples; i++)
            {
                dst[i] = src[2 * i + 1];
            }
        }
        else if (gl_adc_ctx->cfg.chan == TUYA_AUDIO_ADC_CHANNEL_MIC3)
        {
            TKL_AUD_ADC_LOG("MIC3(AEC ref) not supported on V1 ADC, fallback to L\r\n");
            for (uint32_t i = 0; i < samples; i++)
            {
                dst[i] = src[2 * i];
            }
        }
        else
        {
            // 默认左声道数据
            for (uint32_t i = 0; i < samples; i++)
            {
                dst[i] = src[2 * i];
            }
        }
        gl_adc_ctx->cfg.upper_cb(TUYA_AUDIO_FRAME_EVENT_ADC_RX, gl_adc_ctx->output_buf, gl_adc_ctx->output_buf_size, gl_adc_ctx->cfg.args);
#endif
    }
    else
    {
#if CONFIG_AUD_DRIVER_V2
        if ((gl_adc_ctx->raw_ch_num == 3U) && (gl_adc_ctx->chl_num == 3U)) {
            gl_adc_ctx->cfg.upper_cb(TUYA_AUDIO_FRAME_EVENT_ADC_RX,
                                     gl_adc_ctx->read_buf,
                                     read_size,
                                     gl_adc_ctx->cfg.args);
        } else if (gl_adc_ctx->raw_ch_num == 3U) {
            int16_t *src = (int16_t *)gl_adc_ctx->read_buf;
            int16_t *dst = (int16_t *)gl_adc_ctx->output_buf;
            uint32_t frame_pts;

            if ((dst == NULL) || ((read_size & 0x1U) != 0U)) {
                return;
            }

            frame_pts = read_size / (sizeof(int16_t) * 3U);
            if ((frame_pts == 0U) || ((frame_pts * sizeof(int16_t) * 2U) > gl_adc_ctx->output_buf_size)) {
                return;
            }

            /* Remove ref lane and keep analog mic pair [mic1, mic2]. */
            for (uint32_t i = 0; i < frame_pts; i++) {
                dst[2U * i] = src[3U * i + 1U];
                dst[2U * i + 1U] = src[3U * i + 2U];
            }

            gl_adc_ctx->cfg.upper_cb(TUYA_AUDIO_FRAME_EVENT_ADC_RX,
                                     gl_adc_ctx->output_buf,
                                     frame_pts * sizeof(int16_t) * 2U,
                                     gl_adc_ctx->cfg.args);
        } else {
            gl_adc_ctx->cfg.upper_cb(TUYA_AUDIO_FRAME_EVENT_ADC_RX, gl_adc_ctx->read_buf, read_size, gl_adc_ctx->cfg.args);
        }
#else
        gl_adc_ctx->cfg.upper_cb(TUYA_AUDIO_FRAME_EVENT_ADC_RX, gl_adc_ctx->read_buf, read_size, gl_adc_ctx->cfg.args);
#endif
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
#if CONFIG_AUD_DRIVER_V2
    dma_config.src.dev = DMA_DEV_AUD_MIC0;
#else
    dma_config.src.dev = DMA_DEV_AUDIO_RX;
#endif
    dma_config.dst.dev = DMA_DEV_DTCM;
#if CONFIG_AUD_DRIVER_V2
    /*
     * Keep V2 ADC DMA width aligned with onboard_mic_stream_v2:
     * source fifo read in 16-bit units, destination ring buffer in 32-bit units.
     * This avoids malformed sample packing on single analog mic capture.
     */
    dma_config.src.width = DMA_DATA_WIDTH_16BITS;
    dma_config.dst.width = DMA_DATA_WIDTH_32BITS;
#else
    dma_config.src.width = DMA_DATA_WIDTH_32BITS;
    dma_config.dst.width = DMA_DATA_WIDTH_32BITS;
#endif

#if CONFIG_AUD_DRIVER_V2
    if (bk_aud_adc_get_fifo_addr(AUD_ADC_MIC_DATA_BUS_0, &adc_port_addr) != BK_OK)
#else
    if (bk_aud_adc_get_fifo_addr(&adc_port_addr) != BK_OK)
#endif
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

/* ---------------------- Public API ----------------------------------------- */

/**
* @brief Initialize audio ADC
*
* @param[in] config: ADC configuration including chan, sample_rate, sample_bits,
*                    frame_size, vol, upper_aud_adc_cb
*
* @note This API allocates context, configures ADC hardware, DMA ring buffer,
*       and registers flash operation notify handler. On BK7259 V2 the onboard
*       frame layout is [ref, mic1, mic2]: CHANNEL_L/R/LR select mic1/mic2/both,
*       CHANNEL_MIC3 selects the AEC reference lane (index 0). Must be called
*       before tkl_aud_adc_start.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_init(TUYA_AUDIO_ADC_PORT_E port, TKL_AUD_ADC_CFG_T *config)
{
    if (config == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (gl_adc_ctx != NULL) {
        TKL_AUD_ADC_LOG("adc already initialized\n");
        return OPRT_COM_ERROR;
    }

    (void)port;

    bk_err_t ret = BK_OK;
    uint8_t chl_num = 2;
    uint8_t raw_ch_num = 2;
    aud_adc_config_t aud_adc_cfg = DEFAULT_AUD_ADC_CONFIG();
#if CONFIG_AUD_DRIVER_V2
    uint32_t hw_dig_gain = 0;
    uint32_t adc_ch_bitmap = (1U << AUD_ADC_CHL_0);
    aud_adc_mode_t adc_mode = AUD_ADC_MODE_SIGNAL_END;
#else
    aud_mic_id_t mic_id = AUD_MIC_BOTH;
    aud_adc_mode_t adc_mode = AUD_ADC_MODE_DIFFEN;
#endif

    gl_adc_ctx = (tkl_aud_adc_ctx_t *)tkl_system_psram_malloc(sizeof(tkl_aud_adc_ctx_t));
    if (gl_adc_ctx == NULL) {
        TKL_AUD_ADC_LOG("malloc adc context fail\n");
        return OPRT_MALLOC_FAILED;
    }
    os_memset(gl_adc_ctx, 0, sizeof(tkl_aud_adc_ctx_t));
    os_memcpy(&gl_adc_ctx->cfg, config, sizeof(TKL_AUD_ADC_CFG_T));

#if CONFIG_AUD_DRIVER_V2
    switch (config->chan) {
        case TUYA_AUDIO_ADC_CHANNEL_L:
            adc_ch_bitmap = (1U << AUD_ADC_CHL_0);
            chl_num = 1;
            raw_ch_num = 1;
            adc_mode = AUD_ADC_MODE_DIFFEN;
            break;
        case TUYA_AUDIO_ADC_CHANNEL_R:
            adc_ch_bitmap = (1U << AUD_ADC_CHL_1);
            chl_num = 1;
            raw_ch_num = 1;
            adc_mode = AUD_ADC_MODE_DIFFEN;
            break;
        case TUYA_AUDIO_ADC_CHANNEL_LR:
            /* Mixed frame: [ref, mic1, mic2], callback only mic lanes. */
            adc_ch_bitmap = (1U << AUD_ADC_CHL_0) | (1U << AUD_ADC_CHL_1) | (1U << AUD_ADC_CHL_2);
            adc_mode = AUD_ADC_MODE_DIFFEN;
            chl_num = 2;
            raw_ch_num = 3;
            break;
        case TUYA_AUDIO_ADC_CHANNEL_MIC3:
            /* AEC ref lane: same 3ch DMA as LR, strip index 0 in ISR. */
            adc_ch_bitmap = (1U << AUD_ADC_CHL_0) | (1U << AUD_ADC_CHL_1) | (1U << AUD_ADC_CHL_2);
            adc_mode = AUD_ADC_MODE_DIFFEN;
            chl_num = 1;
            raw_ch_num = 3;
            TKL_AUD_ADC_LOG("chan MIC3(AEC ref): 3ch DMA [ref,mic1,mic2], output ref lane\r\n");
            break;
        case TUYA_AUDIO_ADC_CHANNEL_RAW3:
            /* Pass through full [ref,mic1,mic2] frame for dump/analysis. */
            adc_ch_bitmap = (1U << AUD_ADC_CHL_0) | (1U << AUD_ADC_CHL_1) | (1U << AUD_ADC_CHL_2);
            adc_mode = AUD_ADC_MODE_DIFFEN;
            chl_num = 3;
            raw_ch_num = 3;
            break;
        default:
            TKL_AUD_ADC_LOG("chan %d not support\n", config->chan);
            goto __adc_init_exit;
    }
#else
    switch (config->chan) {
        case TUYA_AUDIO_ADC_CHANNEL_L:
        case TUYA_AUDIO_ADC_CHANNEL_R:
        case TUYA_AUDIO_ADC_CHANNEL_MIC3:
            /* V1 ADC always outputs LR interleaved data on one fifo bus. */
            chl_num = 1;
            break;
        case TUYA_AUDIO_ADC_CHANNEL_LR:
            mic_id = AUD_MIC_BOTH;
            adc_mode = AUD_ADC_MODE_DIFFEN;
            chl_num = 2;
            break;
        default:
            TKL_AUD_ADC_LOG("chan %d not support\n", config->chan);
            goto __adc_init_exit;
    }
#endif

    if (config->sample_bits != 16) {
        TKL_AUD_ADC_LOG("bits: %d not support, only 16bits\n", config->sample_bits);
        goto __adc_init_exit;
    }

    if (config->vol > 0x3FU) {
        TKL_AUD_ADC_LOG("gain 0x%02x out of range: 0x00 ~ 0x3f\n", config->vol);
        goto __adc_init_exit;
    }

    gl_adc_ctx->chl_num = chl_num;
    gl_adc_ctx->raw_ch_num = raw_ch_num;
#if CONFIG_AUD_DRIVER_V2
    gl_adc_ctx->adc_ch_bitmap = adc_ch_bitmap;
    gl_adc_ctx->dma_frame_size = AUD_ADC_FRAME_SIZE(config->sample_rate, config->sample_bits, config->frame_time_ms, raw_ch_num);
#else
    /* V1 keeps LR interleaved capture and extracts one channel in ISR. */
    gl_adc_ctx->dma_frame_size = AUD_ADC_FRAME_SIZE(config->sample_rate, config->sample_bits, config->frame_time_ms, 2);
#endif

#if CONFIG_AUD_DRIVER_V2
    uint32_t apply_gain = config->vol;
    if (apply_gain > TKL_AUD_ADC_V2_DIG_GAIN_SAFE_MAX) {
        TKL_AUD_ADC_LOG("adc vol=%u too high, clamp to safe max=%u\n",
                        config->vol, (unsigned int)TKL_AUD_ADC_V2_DIG_GAIN_SAFE_MAX);
        apply_gain = TKL_AUD_ADC_V2_DIG_GAIN_SAFE_MAX;
    }
    hw_dig_gain = __tkl_aud_adc_map_gain_to_dig_reg(apply_gain);
    gl_adc_ctx->cfg.vol = (uint8_t)apply_gain;
    aud_adc_cfg.chl_num = raw_ch_num;
    aud_adc_cfg.adc_chl = AUD_ADC_CHL_0;
    aud_adc_cfg.sample_rate = config->sample_rate;
    aud_adc_cfg.clk_src = AUD_CLK_APLL;
    for (uint32_t ch = 0; ch < AUD_ADC_CHL_MAX; ch++) {
        if (adc_ch_bitmap & (1U << ch)) {
            aud_adc_cfg.chl_cfg[ch].bits = config->sample_bits;
            aud_adc_cfg.chl_cfg[ch].adc_mode = adc_mode;
            aud_adc_cfg.chl_cfg[ch].dig_gain = hw_dig_gain;
            aud_adc_cfg.chl_cfg[ch].ana_gain = TKL_AUD_ADC_V2_ANA_GAIN_DEFAULT;
        }
    }
    TKL_AUD_ADC_LOG("adc config(v2): bitmap=0x%x chl_num=%u raw_ch=%u samp_rate=%u mode=%u vol_req=%u vol_apply=%u dig=0x%x ana=0x%x\n",
                    adc_ch_bitmap, chl_num, raw_ch_num, aud_adc_cfg.sample_rate, adc_mode, config->vol, apply_gain,
                    hw_dig_gain, (unsigned int)TKL_AUD_ADC_V2_ANA_GAIN_DEFAULT);
#else
    aud_adc_cfg.adc_chl = AUD_ADC_CHL_LR;
    aud_adc_cfg.samp_rate = config->sample_rate;
    aud_adc_cfg.adc_gain = config->vol;
    aud_adc_cfg.clk_src = AUD_CLK_APLL;
    aud_adc_cfg.adc_mode = adc_mode;
    TKL_AUD_ADC_LOG("adc config, chan %d, vol 0x%02x, samp_rate: %d, clk_src: %s, adc_mode: %s\n",
                     aud_adc_cfg.adc_chl, aud_adc_cfg.adc_gain, aud_adc_cfg.samp_rate,
                     aud_adc_cfg.clk_src == 1 ? "APLL" : "XTAL",
                     aud_adc_cfg.adc_mode == 1 ? "AUD_ADC_MODE_SIGNAL_END" : "AUD_ADC_MODE_DIFFEN");
#endif

    /*
     * Reset shared audio hardware before ADC-only init. DAC init already does
     * this; without it, cold-start ADC can report started but never produce DMA frames.
     */
    bk_aud_hardware_reset();
    ret = bk_aud_adc_init(&aud_adc_cfg);
    if (ret != BK_OK) {
        TKL_AUD_ADC_LOG("bk_aud_adc_init fail\n");
        goto __adc_init_exit;
    }

#if CONFIG_AUD_DRIVER_V2
    for (uint32_t ch = 0; ch < AUD_ADC_CHL_MAX; ch++) {
        if (!(adc_ch_bitmap & (1U << ch))) {
            continue;
        }
        ret = bk_aud_adc_set_mic_mode((aud_adc_chl_t)ch, adc_mode);
        if (ret != BK_OK) {
            TKL_AUD_ADC_LOG("bk_aud_adc_set_mic_mode fail, ret=%d ch=%u mode=%d\n", ret, ch, adc_mode);
            goto __adc_init_exit;
        }
        (void)bk_aud_adc_set_ana_gain((aud_adc_chl_t)ch, TKL_AUD_ADC_V2_ANA_GAIN_DEFAULT);
        (void)bk_aud_adc_set_dig_gain((aud_adc_chl_t)ch, hw_dig_gain);
    }
#else
    ret = bk_aud_adc_set_mic_mode(mic_id, adc_mode);
    if (ret != BK_OK) {
        TKL_AUD_ADC_LOG("bk_aud_adc_set_mic_mode fail, ret %d mic_id %d mode %d\n",
                         ret, mic_id, adc_mode);
        goto __adc_init_exit;
    }
#endif

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

    /* output_buf: extraction result (single channel or ref-stripped dual channel). */
    if (raw_ch_num != chl_num) {
        gl_adc_ctx->output_buf_size = AUD_ADC_FRAME_SIZE(gl_adc_ctx->cfg.sample_rate, gl_adc_ctx->cfg.sample_bits, gl_adc_ctx->cfg.frame_time_ms, chl_num);
        gl_adc_ctx->output_buf = (uint8_t *)tkl_system_psram_malloc(gl_adc_ctx->output_buf_size);
        if (gl_adc_ctx->output_buf == NULL) {
            TKL_AUD_ADC_LOG("malloc output buffer fail\n");
            goto __adc_init_exit;
        }
    }

#if !CONFIG_AUD_DRIVER_V2
    bk_aud_set_ana_mic0_gain(0x08);
    bk_aud_set_ana_mic1_gain(0x08);
#endif

    mb_flash_register_op_onboard_mic_stream_notify(__tkl_aud_adc_flash_op_notify_handler, NULL);

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

/**
* @brief Start audio ADC capture
*
* @note Starts DMA transfer and ADC hardware. After starting, DMA finish ISR
*       will be triggered periodically, calling upper_aud_adc_cb with PCM data.
*       Safe to call multiple times (idempotent).
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_start(TUYA_AUDIO_ADC_PORT_E port)
{
    (void) port;
    if (gl_adc_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (gl_adc_ctx->is_open)
    {
        TKL_AUD_ADC_LOG("aud adc had been open\n");
        return OPRT_OK;
    }

    ring_buffer_clear(&gl_adc_ctx->mic_rb);

    bk_err_t ret = bk_dma_start(gl_adc_ctx->mic_dma_id);
    if (ret != BK_OK)
    {
        TKL_AUD_ADC_LOG("dma start fail\n");
        return OPRT_COM_ERROR;
    }

#if CONFIG_AUD_DRIVER_V2
    ret = __tkl_aud_adc_start_channels(gl_adc_ctx->adc_ch_bitmap, NULL);
    if (ret == BK_OK) {
        bk_aud_adc_enable_used_channel(gl_adc_ctx->adc_ch_bitmap);
    }
#else
    ret = bk_aud_adc_start();
#endif
    if (ret != BK_OK)
    {
        TKL_AUD_ADC_LOG("adc start fail\n");
        bk_dma_stop(gl_adc_ctx->mic_dma_id);
        return OPRT_COM_ERROR;
    }

    gl_adc_ctx->is_open = true;
    TKL_AUD_ADC_LOG("adc start ok\n");

    return OPRT_OK;
}

/**
* @brief Set audio ADC digital gain
*
* @param[in] gain: digital gain value, range 0x00~0x3F
*
* @note This API sets the ADC digital gain register.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_set_vol(TUYA_AUDIO_ADC_PORT_E port, UINT32_T gain)
{
    (void) port;
    if (gl_adc_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (gain > 0x3f)
    {
        TKL_AUD_ADC_LOG("gain 0x%02x out of range: 0x00 ~ 0x3f\n", gain);
        return OPRT_INVALID_PARM;
    }

#if CONFIG_AUD_DRIVER_V2
    uint32_t apply_gain = gain;
    if (apply_gain > TKL_AUD_ADC_V2_DIG_GAIN_SAFE_MAX) {
        TKL_AUD_ADC_LOG("set vol=%u too high, clamp to safe max=%u\n",
                        gain, (unsigned int)TKL_AUD_ADC_V2_DIG_GAIN_SAFE_MAX);
        apply_gain = TKL_AUD_ADC_V2_DIG_GAIN_SAFE_MAX;
    }
    uint32_t hw_dig_gain = __tkl_aud_adc_map_gain_to_dig_reg(apply_gain);
    TKL_AUD_ADC_LOG("set vol req=%u apply=%u dig=0x%x\n", gain, apply_gain, hw_dig_gain);
    for (uint32_t ch = 0; ch < AUD_ADC_CHL_MAX; ch++) {
        if (!(gl_adc_ctx->adc_ch_bitmap & (1U << ch))) {
            continue;
        }
        if (bk_aud_adc_set_dig_gain((aud_adc_chl_t)ch, hw_dig_gain) != BK_OK) {
            TKL_AUD_ADC_LOG("set digital gain fail, ch=%u\n", ch);
            return OPRT_COM_ERROR;
        }
    }
#else
    if (bk_aud_adc_set_gain(gain) != BK_OK) {
        TKL_AUD_ADC_LOG("set digital gain fail\n");
        return OPRT_COM_ERROR;
    }
#endif

#if CONFIG_AUD_DRIVER_V2
    gl_adc_ctx->cfg.vol = (uint8_t)apply_gain;
#else
    gl_adc_ctx->cfg.vol = (uint8_t)gain;
#endif

    return OPRT_OK;
}

/**
* @brief Stop audio ADC capture
*
* @note Stops DMA transfer and ADC hardware. After stopping, DMA finish ISR
*       will no longer fire. Safe to call multiple times (idempotent).
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_stop(TUYA_AUDIO_ADC_PORT_E port)
{
    (void) port;
    if (gl_adc_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (!gl_adc_ctx->is_open)
    {
        return OPRT_OK;
    }

    bk_err_t ret = bk_dma_stop(gl_adc_ctx->mic_dma_id);
    if (ret != BK_OK)
    {
        TKL_AUD_ADC_LOG("dma stop fail\n");
        return OPRT_COM_ERROR;
    }

#if CONFIG_AUD_DRIVER_V2
    __tkl_aud_adc_stop_channels(gl_adc_ctx->adc_ch_bitmap);
    ret = BK_OK;
#else
    ret = bk_aud_adc_stop();
#endif
    if (ret != BK_OK)
    {
        TKL_AUD_ADC_LOG("adc stop fail\n");
        return OPRT_COM_ERROR;
    }

    gl_adc_ctx->is_open = false;

    return OPRT_OK;
}

/**
* @brief Deinitialize audio ADC
*
* @note Stops capture if running, releases DMA channel and ring buffer,
*       unregisters flash operation notify, frees context memory,
*       and restores CPU frequency.
*/
VOID_T tkl_aud_adc_deinit(TUYA_AUDIO_ADC_PORT_E port)
{
    (void) port;
    if (gl_adc_ctx == NULL)
    {
        return;
    }

    if (gl_adc_ctx->is_open)
    {
        tkl_aud_adc_stop(port);
    }

    __tkl_aud_adc_dma_deconfig();
    bk_aud_adc_deinit();

    mb_flash_unregister_op_onboard_mic_stream_notify();

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
}
