
#include <common/bk_include.h>
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

#define TKL_AUD_ADC_GAIN_NUM               (2U)         /* dig then ana */

#if CONFIG_AUD_DRIVER_V2
#define TKL_AUD_ADC_DEFAULT_DIG_DB         (16.0f)      /* OEM DEFAULT_AUD_ADC_CONFIG dig_gain */
#define TKL_AUD_ADC_DEFAULT_ANA_DB         (20)         /* OEM DEFAULT_AUD_ADC_CONFIG ana_gain */
/*
 * Index of AEC/loopback-reference mic in the 3ch DMA frame [mic1,mic2,mic3].
 *   0 = MIC1 is ref, talk path uses MIC2+MIC3 (this board)
 *   2 = MIC3 is ref, talk path uses MIC1+MIC2 (legacy T9 / mftest)
 * Override via -DTKL_AUD_ADC_AEC_REF_IDX=<n> if needed.
 */
#ifndef TKL_AUD_ADC_AEC_REF_IDX
#define TKL_AUD_ADC_AEC_REF_IDX            (2U)
#endif
#if (TKL_AUD_ADC_AEC_REF_IDX > 2U)
#error "TKL_AUD_ADC_AEC_REF_IDX must be 0..2"
#endif
#endif

typedef struct {
    TKL_AUD_ADC_CFG_T   cfg;
    bool                is_open;
    bool                dma_started;       /* GDMA was started; REPEAT cannot dma_stop */
    uint32_t            dma_frame_size;
    uint8_t             chl_num;
    uint8_t             raw_ch_num;
#if CONFIG_AUD_DRIVER_V2
    uint32_t            adc_ch_bitmap;
    aud_adc_mode_t      adc_mode;
    float               dig_db;            /**< Last dig gain from set_gain (dB) */
    int32_t             ana_db;            /**< Last ana gain from set_gain (dB) */
#else
    uint32_t            dig_reg;           /**< Last dig register gain from set_gain */
#endif
    BOOL_T              gain_set;          /**< TRUE after first successful set_gain */
    dma_id_t            mic_dma_id;
    RingBufferContext   mic_rb;
    int8_t             *mic_dma_buf;
    uint8_t            *read_buf;          /* dma_frame_size: ring_buffer_read target */
    uint8_t            *output_buf;        /* frame_size: extracted analog mic payload */
    uint32_t            output_buf_size;
} tkl_aud_adc_ctx_t;

static tkl_aud_adc_ctx_t *gl_adc_ctx = NULL;

#if CONFIG_AUD_DRIVER_V2
/**
 * @brief Apply V2 digital/ana gain to active ADC channels via dB API
 * @param[in] dig_db digital gain in dB
 * @param[in] ana_db analog gain in dB
 * @return OPRT_OK on success
 */
static OPERATE_RET __tkl_aud_adc_apply_v2_gain(float dig_db, int32_t ana_db)
{
    if (dig_db > BK_AUD_ADC_DIG_GAIN_DB_MAX) {
        dig_db = BK_AUD_ADC_DIG_GAIN_DB_MAX;
    }
    if (ana_db > BK_AUD_ADC_ANA_GAIN_DB_MAX) {
        ana_db = BK_AUD_ADC_ANA_GAIN_DB_MAX;
    }
    if (ana_db < 0) {
        ana_db = 0;
    }

    for (uint32_t ch = 0; ch < AUD_ADC_CHL_MAX; ch++) {
        if (!(gl_adc_ctx->adc_ch_bitmap & (1U << ch))) {
            continue;
        }
        if (bk_aud_adc_set_ana_gain_db((aud_adc_chl_t)ch, ana_db) != BK_OK) {
            TKL_AUD_ADC_LOG("set ana gain fail, ch=%u\n", ch);
            return OPRT_COM_ERROR;
        }
        if (bk_aud_adc_set_dig_gain_db((aud_adc_chl_t)ch, dig_db) != BK_OK) {
            TKL_AUD_ADC_LOG("set dig gain fail, ch=%u\n", ch);
            return OPRT_COM_ERROR;
        }
    }

    gl_adc_ctx->dig_db = dig_db;
    gl_adc_ctx->ana_db = ana_db;
    TKL_AUD_ADC_LOG("adc gain apply dig_db=%.1f ana_db=%d\n", dig_db, ana_db);
    return OPRT_OK;
}

/**
 * @brief Re-apply ADC sample rate / mode / bits / gain after a shared HW reset
 * @return OPRT_OK on success
 * @note DAC init also calls bk_aud_hardware_reset(); re-apply before start so
 *       ADC digital path is valid even if another client reset the IP.
 */
static OPERATE_RET __tkl_aud_adc_reapply_hw_config(VOID_T)
{
    bk_err_t ret;

    if (gl_adc_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    ret = bk_aud_adc_set_samp_rate(gl_adc_ctx->cfg.sample_rate);
    if (ret != BK_OK) {
        TKL_AUD_ADC_LOG("reapply samp_rate fail, ret=%d rate=%u\n",
                        ret, gl_adc_ctx->cfg.sample_rate);
        return OPRT_COM_ERROR;
    }

    for (uint32_t ch = 0; ch < AUD_ADC_CHL_MAX; ch++) {
        if (!(gl_adc_ctx->adc_ch_bitmap & (1U << ch))) {
            continue;
        }
        ret = bk_aud_adc_set_mic_mode((aud_adc_chl_t)ch, gl_adc_ctx->adc_mode);
        if (ret != BK_OK) {
            TKL_AUD_ADC_LOG("reapply mic_mode fail, ret=%d ch=%u\n", ret, ch);
            return OPRT_COM_ERROR;
        }
        ret = bk_aud_adc_set_bits_width((aud_adc_chl_t)ch, gl_adc_ctx->cfg.sample_bits);
        if (ret != BK_OK) {
            TKL_AUD_ADC_LOG("reapply bits fail, ret=%d ch=%u\n", ret, ch);
            return OPRT_COM_ERROR;
        }
    }

    return __tkl_aud_adc_apply_v2_gain(gl_adc_ctx->dig_db, gl_adc_ctx->ana_db);
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
        /* REPEAT mode cannot bk_dma_stop(); park dst_pause_addr instead. */
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
 * Turn one chunk already sitting in read_buf into the PCM the caller asked for
 * and hand it to upper_cb.
 *
 * For single channel: extract one lane out of the raw interleaved frame into
 * output_buf. For dual/triple channel: read_buf is already the final PCM.
 *
 * read_size is by value: the compact step below rewrites it.
 */
static void __tkl_aud_adc_push_frame(uint32_t read_size)
{
#if CONFIG_AUD_DRIVER_V2
    /*
     * Every 32-bit slot carries one 16-bit sample in its low half. Compact in
     * place to dense 16-bit so the channel extraction below sees the layout it
     * expects. Ascending i reads index 2*i >= i, so this never clobbers a slot it
     * still needs. Mirrors ex_audio_mic_unpack_adc16_from_dma32() in
     * test_media/test_aud_utils.c.
     */
    {
        int16_t *raw = (int16_t *)gl_adc_ctx->read_buf;
        uint32_t slots = read_size / 4U;
        uint32_t i;

        for (i = 0U; i < slots; i++) {
            raw[i] = raw[2U * i];
        }
        read_size = slots * (uint32_t)sizeof(int16_t);
    }
#endif

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
         * Mixed 3ch layout when CHL_0|1|2 are enabled (aec_en=0):
         * [mic1, mic2, mic3, ...] matching AUD_ADC_CHL_0/1/2 order.
         * TKL_AUD_ADC_AEC_REF_IDX selects the board AEC/loopback-ref lane.
         * CHANNEL_MIC3 always means "ref mic", not hard-wired physical MIC3.
         *   L    -> first talk mic (lowest non-ref index)
         *   R    -> second talk mic (highest non-ref index)
         *   MIC3 -> AEC ref (TKL_AUD_ADC_AEC_REF_IDX)
         */
        if (raw_ch_num == 3U) {
            uint32_t mic_idx;
            uint32_t talk0 = (TKL_AUD_ADC_AEC_REF_IDX == 0U) ? 1U : 0U;
            uint32_t talk1 = (TKL_AUD_ADC_AEC_REF_IDX == 2U) ? 1U : 2U;

            if (gl_adc_ctx->cfg.chan == TUYA_AUDIO_ADC_CHANNEL_MIC3) {
                mic_idx = TKL_AUD_ADC_AEC_REF_IDX;
            } else if (gl_adc_ctx->cfg.chan == TUYA_AUDIO_ADC_CHANNEL_R) {
                mic_idx = talk1;
            } else {
                mic_idx = talk0;
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
            uint32_t talk0 = (TKL_AUD_ADC_AEC_REF_IDX == 0U) ? 1U : 0U;
            uint32_t talk1 = (TKL_AUD_ADC_AEC_REF_IDX == 2U) ? 1U : 2U;

            if ((dst == NULL) || ((read_size & 0x1U) != 0U)) {
                return;
            }

            frame_pts = read_size / (sizeof(int16_t) * 3U);
            if ((frame_pts == 0U) || ((frame_pts * sizeof(int16_t) * 2U) > gl_adc_ctx->output_buf_size)) {
                return;
            }

            /* Keep talk mics, drop AEC/loopback-ref lane (TKL_AUD_ADC_AEC_REF_IDX). */
            for (uint32_t i = 0; i < frame_pts; i++) {
                dst[2U * i] = src[3U * i + talk0];
                dst[2U * i + 1U] = src[3U * i + talk1];
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
#if CONFIG_AUD_DRIVER_V2
    dma_config.src.dev = DMA_DEV_AUD_MIC0;
#else
    dma_config.src.dev = DMA_DEV_AUDIO_RX;
#endif
    dma_config.dst.dev = DMA_DEV_DTCM;
    dma_config.src.width = DMA_DATA_WIDTH_32BITS;
    dma_config.dst.width = DMA_DATA_WIDTH_32BITS;

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

/**
 * @brief Rebuild the ADC GDMA channel without bk_dma_stop()
 * @return BK_OK on success
 * @note REPEAT-mode GDMA src-stalls after analog ADC is stopped while the
 *       channel stays enabled. bk_dma_deinit() (inside deconfig) resets the
 *       channel; that is not the unsupported repeat-mode stop.
 */
static bk_err_t __tkl_aud_adc_dma_reset(void)
{
    (void)__tkl_aud_adc_dma_deconfig();
    return __tkl_aud_adc_dma_config();
}

/**
 * @brief Pause GDMA by parking dst_pause_addr at the ring end
 * @return none
 * @note BK GDMA in REPEAT mode does not support bk_dma_stop(). Clearing the
 *       WRITE ring sets dma_set_dst_pause_addr so the channel stalls without
 *       being disabled.
 */
static VOID_T __tkl_aud_adc_pause_dma(VOID_T)
{
    if ((gl_adc_ctx == NULL) || (gl_adc_ctx->mic_dma_buf == NULL)) {
        return;
    }
    ring_buffer_clear(&gl_adc_ctx->mic_rb);
}

/* ---------------------- Public API ----------------------------------------- */

/**
* @brief Initialize audio ADC
*
* @param[in] config: ADC configuration including chan, sample_rate, sample_bits,
*                    frame_time_ms, upper_cb. Gain is set via tkl_aud_adc_set_gain().
*
* @note This API allocates context, configures ADC hardware, DMA ring buffer,
*       and registers flash operation notify handler. On BK7259 V2 the onboard
*       3ch frame layout is [mic1, mic2, mic3]: CHANNEL_L/R/LR select mic1/mic2/both,
*       CHANNEL_MIC3 selects the AEC-reference mic (index 2). Must be called
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
    gl_adc_ctx->gain_set = FALSE;
#if CONFIG_AUD_DRIVER_V2
    gl_adc_ctx->dig_db = TKL_AUD_ADC_DEFAULT_DIG_DB;
    gl_adc_ctx->ana_db = TKL_AUD_ADC_DEFAULT_ANA_DB;
#else
    gl_adc_ctx->dig_reg = 0x2DU; /* OEM DEFAULT_AUD_ADC_CONFIG adc_gain */
#endif

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
            /* Mixed frame: [mic1,mic2,mic3]; callback keeps two talk mics
             * (drops TKL_AUD_ADC_AEC_REF_IDX). With REF=0 -> [mic2,mic3]. */
            adc_ch_bitmap = (1U << AUD_ADC_CHL_0) | (1U << AUD_ADC_CHL_1) | (1U << AUD_ADC_CHL_2);
            adc_mode = AUD_ADC_MODE_DIFFEN;
            chl_num = 2;
            raw_ch_num = 3;
            break;
        case TUYA_AUDIO_ADC_CHANNEL_MIC3:
            /* AEC/loopback ref lane: extract TKL_AUD_ADC_AEC_REF_IDX from 3ch DMA. */
            adc_ch_bitmap = (1U << AUD_ADC_CHL_0) | (1U << AUD_ADC_CHL_1) | (1U << AUD_ADC_CHL_2);
            adc_mode = AUD_ADC_MODE_DIFFEN;
            chl_num = 1;
            raw_ch_num = 3;
            TKL_AUD_ADC_LOG("chan MIC3(AEC ref): 3ch DMA, output ref idx=%u\r\n",
                            (unsigned)TKL_AUD_ADC_AEC_REF_IDX);
            break;
        case TUYA_AUDIO_ADC_CHANNEL_RAW3:
            /* Pass through full [mic1, mic2, mic3] frame for dump/analysis. */
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

    gl_adc_ctx->chl_num = chl_num;
    gl_adc_ctx->raw_ch_num = raw_ch_num;
#if CONFIG_AUD_DRIVER_V2
    gl_adc_ctx->adc_ch_bitmap = adc_ch_bitmap;
    gl_adc_ctx->adc_mode = adc_mode;
    /* Raw ring bytes per frame: one sample per 32-bit slot, so twice the dense
     * payload. Compacted back down in __tkl_aud_adc_push_frame(). */
    gl_adc_ctx->dma_frame_size = AUD_ADC_FRAME_SIZE(config->sample_rate, config->sample_bits, config->frame_time_ms, raw_ch_num) * 2U;
#else
    /* V1 keeps LR interleaved capture and extracts one channel in ISR. */
    gl_adc_ctx->dma_frame_size = AUD_ADC_FRAME_SIZE(config->sample_rate, config->sample_bits, config->frame_time_ms, 2);
#endif

#if CONFIG_AUD_DRIVER_V2
    /* Gain applied later via tkl_aud_adc_set_gain(); keep OEM default dig/ana for HW init. */
    aud_adc_cfg.chl_num = raw_ch_num;
    aud_adc_cfg.adc_chl = AUD_ADC_CHL_0;
    aud_adc_cfg.sample_rate = config->sample_rate;
    aud_adc_cfg.clk_src = AUD_CLK_APLL;
    for (uint32_t ch = 0; ch < AUD_ADC_CHL_MAX; ch++) {
        if (adc_ch_bitmap & (1U << ch)) {
            aud_adc_cfg.chl_cfg[ch].bits = config->sample_bits;
            aud_adc_cfg.chl_cfg[ch].adc_mode = adc_mode;
            aud_adc_cfg.chl_cfg[ch].dig_gain = gl_adc_ctx->dig_db;
            aud_adc_cfg.chl_cfg[ch].ana_gain = gl_adc_ctx->ana_db;
        }
    }
    TKL_AUD_ADC_LOG("adc config(v2): bitmap=0x%x chl_num=%u raw_ch=%u samp_rate=%u mode=%u dig_db=%.1f ana_db=%d (OEM default, set_gain later)\n",
                    adc_ch_bitmap, chl_num, raw_ch_num, aud_adc_cfg.sample_rate, adc_mode,
                    gl_adc_ctx->dig_db, gl_adc_ctx->ana_db);
#else
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
#endif

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
* @note Starts DMA transfer and ADC hardware. After stop, the GDMA channel is
*       rebuilt (deinit+init, not dma_stop) so src-stall is cleared.
*       Safe to call multiple times (idempotent).
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
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

#if CONFIG_AUD_DRIVER_V2
    /* DAC (or other clients) may have called bk_aud_hardware_reset() after
     * ADC init; restore rate/mode/gain before enabling the mic path. */
    if (__tkl_aud_adc_reapply_hw_config() != OPRT_OK) {
        return OPRT_COM_ERROR;
    }
#endif

    ret = bk_dma_start(gl_adc_ctx->mic_dma_id);
    if (ret != BK_OK)
    {
        TKL_AUD_ADC_LOG("dma start fail\n");
        return OPRT_COM_ERROR;
    }
    gl_adc_ctx->dma_started = true;

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
        __tkl_aud_adc_pause_dma();
        return OPRT_COM_ERROR;
    }

    gl_adc_ctx->is_open = true;
    TKL_AUD_ADC_LOG("adc start ok\n");

    return OPRT_OK;
}

/**
* @brief Set audio ADC digital/analog gain
*
* @param[in] gain: gain array in dB (V2) or dig register units (V1)
* @param[in] num:  gain count; V2: 2=dig then ana, 1=dig only; V1: 1
*
* @note No volume remapping is performed; caller passes hardware gain directly.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aud_adc_set_gain(TUYA_AUDIO_ADC_PORT_E port, INT32_T gain[], UINT32_T num)
{
    (void)port;

    if (gl_adc_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if ((gain == NULL) || (num == 0U)) {
        return OPRT_INVALID_PARM;
    }

#if CONFIG_AUD_DRIVER_V2
    {
        float dig_db;
        int32_t ana_db;

        if (num > TKL_AUD_ADC_GAIN_NUM) {
            return OPRT_INVALID_PARM;
        }

        dig_db = (float)gain[0];
        ana_db = (num >= 2U) ? gain[1] : gl_adc_ctx->ana_db;
        if (__tkl_aud_adc_apply_v2_gain(dig_db, ana_db) != OPRT_OK) {
            return OPRT_COM_ERROR;
        }
        gl_adc_ctx->gain_set = TRUE;
        return OPRT_OK;
    }
#else
    {
        uint32_t hw_gain;

        if (num != 1U) {
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
#endif
}

/**
* @brief Stop audio ADC capture
*
* @note Stops ADC hardware. REPEAT mode cannot bk_dma_stop(); the WRITE ring
*       is cleared to park dst_pause_addr. The channel is rebuilt on the next
*       start via dma_deinit+init. Safe to call multiple times (idempotent).
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
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

#if CONFIG_AUD_DRIVER_V2
    __tkl_aud_adc_stop_channels(gl_adc_ctx->adc_ch_bitmap);
    ret = BK_OK;
#else
    ret = bk_aud_adc_stop();
#endif

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

#if CONFIG_AUD_DRIVER_V2
    /*
     * Soft teardown: stop channels and clear ADC module flag, but do NOT call
     * bk_aud_adc_deinit() → bk_aud_driver_deinit(). The latter zeros
     * ana_reg20~30 / kills micbias; a following init+start then keeps producing
     * DMA frames that are all zeros. stop→start without deinit stays healthy.
     */
    if (bk_aud_get_module_init_sta(AUD_MODULE_ADC)) {
        (void)bk_aud_adc_stop(AUD_ADC_CHL_0);
        (void)bk_aud_adc_stop(AUD_ADC_CHL_1);
        (void)bk_aud_adc_stop(AUD_ADC_CHL_2);
        bk_aud_set_module_init_sta(AUD_MODULE_ADC, false);
    }
#else
    bk_aud_adc_deinit();
#endif

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
