
#include <common/bk_include.h>
#include <os/os.h>
#include <driver/aud_dac.h>
#include <driver/aud_common.h>
#include <driver/dma.h>
#include <driver/audio_ring_buff.h>
#include <driver/gpio.h>
#include <driver/flash.h>
#include <driver/flash_types.h>
#include <components/bk_audio/audio_pipeline/audio_mem.h>
#include <modules/pm.h>
#include "gpio_driver.h"

#include "tuya_cloud_types.h"
#include "tkl_aud_dac.h"

extern void bk_printf(const char *fmt, ...);

#define TKL_DAC_LOG(fmt, ...) \
    bk_printf("%s %d, " fmt, __func__, __LINE__, ##__VA_ARGS__)

#define DMA_BUF_SAFE_INTERVAL    (8)

// x: sample rate, y: sample bits, z: frame time in ms, channel only 1
#define TKL_DAC_FRAME_SIZE_MS(x, y, z)  ((x * (y / 8) / 1000) * z)

typedef enum {
    TKL_AUD_DAC_STAT_IDLE = 0,
    TKL_AUD_DAC_STAT_INIT,
    TKL_AUD_DAC_STAT_START,
    TKL_AUD_DAC_STAT_STOP,
    TKL_AUD_DAC_STAT_DEINIT,
} TKL_AUD_DAC_STAT_E;

typedef struct {
    TKL_AUD_DAC_CFG_T       cfg;
    TUYA_AUDIO_DAC_PORT_E   port;
    TKL_AUD_DAC_STAT_E      stat;
    dma_id_t                spk_dma_id;
    RingBufferContext       spk_rb;
    int8_t                 *spk_dma_buf;
    uint32_t                frame_size;
    TKL_AUD_DAC_FRAME_CB    frame_cb;
    VOID_T                 *frame_cb_arg;
#if CONFIG_AUD_DRIVER_V2
    aud_dac_source_t        dac_source; /**< Resolved OEM digital source */
#endif
} tkl_aud_dac_ctx_t;

static tkl_aud_dac_ctx_t *gl_dac_ctx = NULL;
static aud_dac_config_t aud_dac_cfg = DEFAULT_AUD_DAC_CONFIG();

dma_id_t test_dma_id = DMA_ID_MAX;

#if CONFIG_AUD_DRIVER_V2
#define TKL_AUD_DAC_DMA_CH          (0U)
#define TKL_AUD_DAC_DIG_GAIN_DB_MIN (-7.0f)   /* OEM DEFAULT_AUD_DAC_CONFIG dig_gain */

typedef struct {
    float dig_db;
    int32_t ana_db;
} tkl_aud_dac_gain_db_t;

/**
 * @brief Clamp user volume to 0..100
 * @param[in] volume user volume
 * @return clamped volume
 */
static UINT32_T __tkl_aud_dac_clamp_volume(UINT32_T volume)
{
    if (volume > 100U) {
        return 100U;
    }
    return volume;
}

/**
 * @brief Map user volume 0..100 to OEM DAC dig/ana gain in dB
 * @param[in] volume user volume, 0=mute
 * @return dig_db in [-7,+12], ana_db in [0,7]
 */
static tkl_aud_dac_gain_db_t __tkl_aud_dac_volume_to_gain_db(UINT32_T volume)
{
    tkl_aud_dac_gain_db_t out = {0};
    UINT32_T vol;

    if (volume == 0U) {
        out.dig_db = BK_AUD_DAC_DIG_GAIN_DB_SILENCE;
        out.ana_db = 0;
        return out;
    }

    vol = __tkl_aud_dac_clamp_volume(volume);
    out.dig_db = TKL_AUD_DAC_DIG_GAIN_DB_MIN +
                 ((float)(vol - 1U) * (BK_AUD_DAC_DIG_GAIN_DB_MAX - TKL_AUD_DAC_DIG_GAIN_DB_MIN)) / 99.0f;
    if (out.dig_db > BK_AUD_DAC_DIG_GAIN_DB_MAX) {
        out.dig_db = BK_AUD_DAC_DIG_GAIN_DB_MAX;
    }
    out.ana_db = (int32_t)((vol * (UINT32_T)BK_AUD_DAC_ANA_GAIN_DB_MAX) / 100U);
    if (out.ana_db > BK_AUD_DAC_ANA_GAIN_DB_MAX) {
        out.ana_db = BK_AUD_DAC_ANA_GAIN_DB_MAX;
    }
    return out;
}

/**
 * @brief Apply user volume via OEM dB gain APIs (V2)
 * @param[in] volume user volume 0..100
 * @return OPRT_OK on success
 */
static OPERATE_RET __tkl_aud_dac_apply_v2_volume(UINT32_T volume)
{
    tkl_aud_dac_gain_db_t gain = __tkl_aud_dac_volume_to_gain_db(volume);

    if (volume == 0U) {
        bk_aud_dac_mute();
    } else {
        bk_aud_dac_unmute();
        if (bk_aud_dac_set_dig_gain_db(gain.dig_db) != BK_OK) {
            TKL_DAC_LOG("set dac dig gain fail\n");
            return OPRT_COM_ERROR;
        }
        if (bk_aud_dac_set_ana_gain_db(gain.ana_db) != BK_OK) {
            TKL_DAC_LOG("set dac ana gain fail\n");
            return OPRT_COM_ERROR;
        }
    }

    gl_dac_ctx->cfg.volume = (uint8_t)__tkl_aud_dac_clamp_volume(volume);
    bk_printf("dac volume user=%u dig=%.1fdB ana=%ddB\r\n", volume, gain.dig_db, gain.ana_db);
    return OPRT_OK;
}

/**
 * @brief Map TKL source enum / sample_rate to OEM aud_dac_source_t
 * @param[in] cfg DAC config
 * @param[out] out resolved OEM source
 * @return OPRT_OK on success
 */
static OPERATE_RET __tkl_aud_dac_resolve_source(CONST TKL_AUD_DAC_CFG_T *cfg, aud_dac_source_t *out)
{
    TKL_AUD_DAC_SRC_E src;

    if ((cfg == NULL) || (out == NULL)) {
        return OPRT_INVALID_PARM;
    }

    src = cfg->src;
    if (src == TKL_AUD_DAC_SRC_AUTO) {
        switch (cfg->sample_rate) {
            case 8000:
            case 16000:
                src = TKL_AUD_DAC_SRC_VOICE;
                break;
            case 44100:
            case 48000:
            case 88200:
            case 96000:
            case 176400:
            case 192000:
            case 352800:
            case 384000:
                src = TKL_AUD_DAC_SRC_MEDIA;
                break;
            default:
                TKL_DAC_LOG("AUTO cannot map sample_rate %u\n", cfg->sample_rate);
                return OPRT_INVALID_PARM;
        }
    }

    switch (src) {
        case TKL_AUD_DAC_SRC_MEDIA:
            *out = AUD_DAC_SOURCE_A2DP;
            break;
        case TKL_AUD_DAC_SRC_VOICE:
            *out = AUD_DAC_SOURCE_CALL;
            break;
        case TKL_AUD_DAC_SRC_HINT:
            *out = AUD_DAC_SOURCE_HINT;
            break;
        default:
            TKL_DAC_LOG("unsupported src %d\n", src);
            return OPRT_INVALID_PARM;
    }
    return OPRT_OK;
}

/**
 * @brief Validate sample_rate against OEM source capability
 * @param[in] source OEM DAC source
 * @param[in] sample_rate Hz
 * @return OPRT_OK if supported
 */
static OPERATE_RET __tkl_aud_dac_check_rate(aud_dac_source_t source, UINT32_T sample_rate)
{
    if (source == AUD_DAC_SOURCE_A2DP) {
        switch (sample_rate) {
            case 44100:
            case 48000:
            case 88200:
            case 96000:
            case 176400:
            case 192000:
            case 352800:
            case 384000:
                return OPRT_OK;
            default:
                TKL_DAC_LOG("A2DP/media not support rate %u (use CALL for 8k/16k)\n", sample_rate);
                return OPRT_INVALID_PARM;
        }
    }

    /* CALL / HINT */
    switch (sample_rate) {
        case 8000:
        case 16000:
        case 48000:
            return OPRT_OK;
        default:
            TKL_DAC_LOG("CALL/HINT not support rate %u\n", sample_rate);
            return OPRT_INVALID_PARM;
    }
}
#endif

#if !CONFIG_AUD_DRIVER_V2
static uint8_t __tkl_aud_dac_map_volume_to_gain(uint32_t volume)
{
    if (volume > 100) {
        volume = 100;
    }

    if (volume <= 30) {
        volume = volume * 50 / 30;
    } else {
        volume = 50 + (volume - 30) * 20 / 70;
    }

    return (uint8_t)(volume * 0x3F / 100);
}
#endif
/* ---------------------- DMA ISR -------------------------------------------- */

static void __tkl_aud_dac_dma_finish_isr(void)
{
    if ((gl_dac_ctx != NULL) && (gl_dac_ctx->frame_cb != NULL)) {
        gl_dac_ctx->frame_cb(TUYA_AUDIO_DAC_FRAME_EVENT_TX_COMPLETE,
                gl_dac_ctx->frame_cb_arg);
    }
}

/* ---------------------- DMA config / deconfig ------------------------------ */

static bk_err_t __tkl_aud_dac_dma_deconfig(TUYA_AUDIO_DAC_PORT_E port)
{
    if (gl_dac_ctx == NULL) {
        return BK_OK;
    }

    bk_dma_deinit(gl_dac_ctx->spk_dma_id);
    bk_dma_free(DMA_DEV_AUDIO, gl_dac_ctx->spk_dma_id);

    if (gl_dac_ctx->spk_dma_buf) {
        ring_buffer_clear(&gl_dac_ctx->spk_rb);
        audio_dma_mem_free(gl_dac_ctx->spk_dma_buf);
        gl_dac_ctx->spk_dma_buf = NULL;
    }

    return BK_OK;
}

static bk_err_t __tkl_aud_dac_dma_config(TUYA_AUDIO_DAC_PORT_E port)
{
    bk_err_t ret = BK_OK;
    dma_config_t dma_config = {0};
    uint32_t dac_port_addr;
    uint32_t frame_size = gl_dac_ctx->frame_size;

    gl_dac_ctx->spk_dma_id = bk_dma_alloc(DMA_DEV_AUDIO);
    if ((gl_dac_ctx->spk_dma_id < DMA_ID_0) || (gl_dac_ctx->spk_dma_id >= DMA_ID_MAX)) {
        TKL_DAC_LOG("malloc dma fail\n");
        goto exit;
    }

    test_dma_id = gl_dac_ctx->spk_dma_id;

    /* Two frames + safe interval for dma_set_src_pause_addr flow control */
    gl_dac_ctx->spk_dma_buf = (int8_t *)audio_dma_mem_calloc(2, frame_size + DMA_BUF_SAFE_INTERVAL / 2);
    if (gl_dac_ctx->spk_dma_buf == NULL) {
        TKL_DAC_LOG("malloc dma buffer fail\n");
        goto exit;
    }

    /*
     * RB_DMA_TYPE_READ: DMA reads FROM ring buffer TO DAC FIFO.
     * ring_buffer_init sets dma_set_src_pause_addr(dma_id, addr) — DMA paused at start.
     * ring_buffer_write updates dma_set_src_pause_addr to allow DMA to read new data.
     */
    ring_buffer_init(&gl_dac_ctx->spk_rb,
                     (uint8_t *)gl_dac_ctx->spk_dma_buf,
                     frame_size * 2 + DMA_BUF_SAFE_INTERVAL,
                     gl_dac_ctx->spk_dma_id,
                     RB_DMA_TYPE_READ);

    os_memset(&dma_config, 0, sizeof(dma_config_t));
    dma_config.mode = DMA_WORK_MODE_REPEAT;
    dma_config.chan_prio = 1;
    dma_config.trans_type = DMA_TRANS_DEFAULT;

    /* src: ring buffer (DTCM) */
    dma_config.src.dev = DMA_DEV_DTCM;
    dma_config.src.width = DMA_DATA_WIDTH_32BITS;
    dma_config.src.addr_inc_en = DMA_ADDR_INC_ENABLE;
    dma_config.src.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
    dma_config.src.start_addr = (uint32_t)(uintptr_t)gl_dac_ctx->spk_dma_buf;
    dma_config.src.end_addr = (uint32_t)(uintptr_t)gl_dac_ctx->spk_dma_buf
                              + frame_size * 2 + DMA_BUF_SAFE_INTERVAL;

    /* dst: DAC FIFO */
#if CONFIG_AUD_DRIVER_V1
    dma_config.dst.dev = DMA_DEV_AUDIO;
#endif
    switch (gl_dac_ctx->cfg.chan_num) {
        case 1:
            dma_config.dst.width = DMA_DATA_WIDTH_16BITS;
            break;
        case 2:
            dma_config.dst.width = DMA_DATA_WIDTH_32BITS;
            break;
        default:
            break;
    }
    bk_printf("port: %d, chan_num: %d, sample_bits: %d, frame_size: %d\n",
              port, gl_dac_ctx->cfg.chan_num, gl_dac_ctx->cfg.sample_bits, frame_size);
    #if CONFIG_AUD_DRIVER_V1
    ret = bk_aud_dac_get_fifo_addr(&dac_port_addr);
    #elif CONFIG_AUD_DRIVER_V2
    ret = bk_aud_dac_get_fifo_addr(gl_dac_ctx->dac_source,
                                   port,
                                   &dma_config.dst.dev,
                                   &dac_port_addr);
    #endif
    if (ret != BK_OK) {
        TKL_DAC_LOG("get dac fifo address fail\n");
        goto exit;
    }
    dma_config.dst.addr_inc_en = DMA_ADDR_INC_ENABLE;
    dma_config.dst.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
    dma_config.dst.start_addr = dac_port_addr;
    dma_config.dst.end_addr = dac_port_addr + 4;
    bk_printf("port: %d, dac fifo addr: 0x%08x\n", port,dac_port_addr);
    ret = bk_dma_init(gl_dac_ctx->spk_dma_id, &dma_config);
    if (ret != BK_OK) {
        TKL_DAC_LOG("dma_init fail\n");
        goto exit;
    }

    bk_dma_set_transfer_len(gl_dac_ctx->spk_dma_id, frame_size);

#if (CONFIG_SPE)
    bk_dma_set_dest_sec_attr(gl_dac_ctx->spk_dma_id, DMA_ATTR_SEC);
    bk_dma_set_src_sec_attr(gl_dac_ctx->spk_dma_id, DMA_ATTR_SEC);
#endif

    bk_dma_register_isr(gl_dac_ctx->spk_dma_id, NULL, (void *)__tkl_aud_dac_dma_finish_isr);
    bk_dma_enable_finish_interrupt(gl_dac_ctx->spk_dma_id);

    TKL_DAC_LOG("dma_config dma_id: %d, transfer_len: %d\n",
                 gl_dac_ctx->spk_dma_id, frame_size);

    return BK_OK;

exit:
    __tkl_aud_dac_dma_deconfig(port);
    return BK_FAIL;
}

/**
 * @brief Rebuild the DAC GDMA channel without bk_dma_stop()
 * @param[in] port DAC output port
 * @return BK_OK on success
 * @note REPEAT-mode GDMA dest-stalls after analog DAC is stopped while the
 *       channel stays enabled. bk_dma_deinit() (inside deconfig) resets the
 *       channel; that is not the unsupported repeat-mode stop.
 */
static bk_err_t __tkl_aud_dac_dma_reset(TUYA_AUDIO_DAC_PORT_E port)
{
    (void)__tkl_aud_dac_dma_deconfig(port);
    return __tkl_aud_dac_dma_config(port);
}

/**
 * @brief Pause GDMA by parking src_pause_addr at the ring start
 * @return none
 * @note BK GDMA in REPEAT mode does not support bk_dma_stop(). Clearing the
 *       ring sets dma_set_src_pause_addr to the buffer base so the channel
 *       stalls without being disabled.
 */
static VOID_T __tkl_aud_dac_pause_dma(VOID_T)
{
    if ((gl_dac_ctx == NULL) || (gl_dac_ctx->spk_dma_buf == NULL)) {
        return;
    }
    ring_buffer_clear(&gl_dac_ctx->spk_rb);
}

/**
 * @brief Prefill the DMA ring with silence and unpause GDMA
 * @return none
 */
static VOID_T __tkl_aud_dac_fill_silence(VOID_T)
{
    uint32_t free_size;
    uint32_t fill_cap;
    uint32_t fill_len;
    uint8_t *silence;

    if (gl_dac_ctx == NULL) {
        return;
    }

    free_size = ring_buffer_get_free_size(&gl_dac_ctx->spk_rb);
    if (free_size <= DMA_BUF_SAFE_INTERVAL) {
        return;
    }

    fill_cap = free_size - DMA_BUF_SAFE_INTERVAL;
    fill_len = gl_dac_ctx->frame_size;
    if ((fill_len == 0U) || (fill_len > fill_cap)) {
        fill_len = fill_cap;
    }
    silence = (uint8_t *)tkl_system_malloc(fill_len);
    if (silence == NULL) {
        return;
    }
    os_memset(silence, 0, fill_len);
    ring_buffer_write(&gl_dac_ctx->spk_rb, silence, fill_len);
    tkl_system_free(silence);
}

/* ---------------------- Public API ----------------------------------------- */

/**
 * @brief Initialize the DAC playback context and hardware resources.
 * @param[in] config DAC configuration used to set sample rate, frame size,
 *                   initial volume, and frame callback.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @return OPRT_OK on success, other error codes on failure.
 * @note The current platform only supports 16-bit PCM output and forces the
 *       playback channel count to one onboard speaker.
 * @note This function must be called before `tkl_aud_dac_start()`.
 */
OPERATE_RET tkl_aud_dac_init(TUYA_AUDIO_DAC_PORT_E port, TKL_AUD_DAC_CFG_T *config)
{
    if (config == NULL || port > TUYA_AUDIO_DAC_PORT_MAX) {
        return OPRT_INVALID_PARM;
    }

    if (config->sample_bits != TUYA_AUDIO_SAMPLE_BITS_16) {
        TKL_DAC_LOG("dac only support 16bits sample");
        return OPRT_INVALID_PARM;
    }

    if (config->frame_time_ms == 0) {
        TKL_DAC_LOG("dac frame time is invalid\n");
        return OPRT_INVALID_PARM;
    }

    if (gl_dac_ctx != NULL) {
        TKL_DAC_LOG("dac already initialized\n");
        return OPRT_COM_ERROR;
    }

    bk_err_t ret = BK_OK;

    bk_printf("dac init, sample_rate %d, sample_bits %d",
              config->sample_rate, config->sample_bits);
    gl_dac_ctx = (tkl_aud_dac_ctx_t *)tkl_system_malloc(sizeof(tkl_aud_dac_ctx_t));
    if (gl_dac_ctx == NULL) {
        TKL_DAC_LOG("malloc dac context fail\n");
        return OPRT_MALLOC_FAILED;
    }
    os_memset(gl_dac_ctx, 0, sizeof(tkl_aud_dac_ctx_t));
    os_memcpy(&gl_dac_ctx->cfg, config, sizeof(TKL_AUD_DAC_CFG_T));
    gl_dac_ctx->frame_cb = config->frame_cb;
    gl_dac_ctx->frame_cb_arg = config->args;

#if CONFIG_AUD_DRIVER_V2
    if (__tkl_aud_dac_resolve_source(config, &gl_dac_ctx->dac_source) != OPRT_OK) {
        tkl_system_free(gl_dac_ctx);
        gl_dac_ctx = NULL;
        return OPRT_INVALID_PARM;
    }
    if (__tkl_aud_dac_check_rate(gl_dac_ctx->dac_source, config->sample_rate) != OPRT_OK) {
        tkl_system_free(gl_dac_ctx);
        gl_dac_ctx = NULL;
        return OPRT_INVALID_PARM;
    }
    bk_printf("dac source=%d (0=a2dp,1=call,2=hint), src_cfg=%d\r\n",
              gl_dac_ctx->dac_source, config->src);
#endif

    /* Onboard speaker path supports mono (1) or stereo/LR (2). Clamp any other
     * value to mono. (Was previously forced to 1 for T5's single speaker.) */
    if (gl_dac_ctx->cfg.chan_num != 1 && gl_dac_ctx->cfg.chan_num != 2) {
        gl_dac_ctx->cfg.chan_num = 1;
    }

    bk_printf("dac init, sample_rate %d, sample_bits %d",
              config->sample_rate, config->sample_bits);
    /* DAC init */
#if CONFIG_AUD_DRIVER_V2
    if (gl_dac_ctx->cfg.chan_num == 1) {
        if (port == TUYA_AUDIO_DAC_PORT_0) {
            aud_dac_cfg.dac_chl = AUD_DAC_CHL_L;
        } else {
            aud_dac_cfg.dac_chl = AUD_DAC_CHL_R;
        }
    } else if (gl_dac_ctx->cfg.chan_num == 2) {
        aud_dac_cfg.dac_chl = AUD_DAC_CHL_LR;
        port = 0;  // For stereo, use port 0 for DAC FIFO access
    } else {
        TKL_DAC_LOG("unsupported channel num: %d\n", gl_dac_ctx->cfg.chan_num);
        ret = BK_FAIL;
        goto __dac_init_exit;
    }
#else
    if (gl_dac_ctx->cfg.chan_num == 1) {
        if (port == TUYA_AUDIO_DAC_PORT_0) {
            aud_dac_cfg.dac_chl = AUD_DAC_CHL_L;
        } else {
            aud_dac_cfg.dac_chl = AUD_DAC_CHL_R;
        }
    } else {
        aud_dac_cfg.dac_chl = AUD_DAC_CHL_LR;
        port = 0;  // For stereo, use port 0 for DAC FIFO access
    }
#endif

    bk_printf("dac init, sample_rate %d, sample_bits %d, frame_time_ms %d, volume %d, chan_num %d\n",
              config->sample_rate, config->sample_bits, config->frame_time_ms,
              config->volume, config->chan_num);
#if CONFIG_AUD_DRIVER_V1
    aud_dac_cfg.samp_rate = config->sample_rate;
    aud_dac_cfg.dac_gain = __tkl_aud_dac_map_volume_to_gain(config->volume);
#elif CONFIG_AUD_DRIVER_V2
    {
        tkl_aud_dac_gain_db_t gain = __tkl_aud_dac_volume_to_gain_db(config->volume);

        aud_dac_cfg.sample_rate = config->sample_rate;
        aud_dac_cfg.bits = config->sample_bits;
        aud_dac_cfg.dig_gain = gain.dig_db;
        aud_dac_cfg.ana_gain = gain.ana_db;
    }
#endif
    aud_dac_cfg.work_mode = AUD_DAC_WORK_MODE_DIFFEN;
    aud_dac_cfg.clk_src = AUD_CLK_APLL;

    bk_printf("dac init, sample_rate %d, sample_bits %d, dac_cfg.sample_rate %d, dac_cfg.bits %d\r\n",
              config->sample_rate, config->sample_bits, aud_dac_cfg.sample_rate, aud_dac_cfg.bits);
#if CONFIG_AUD_DRIVER_V1
    TKL_DAC_LOG("dac config, chl %d, vol 0x%02x, samp_rate: %d\n",
                 aud_dac_cfg.dac_chl, aud_dac_cfg.dac_gain, aud_dac_cfg.samp_rate);
#elif CONFIG_AUD_DRIVER_V2
    bk_printf("dac config, sample_rate %d, bits %d, dig=%.1fdB ana=%ddB\r\n",
              aud_dac_cfg.sample_rate, aud_dac_cfg.bits,
              aud_dac_cfg.dig_gain, aud_dac_cfg.ana_gain);
#endif

    /*
     * Reset audio HW state before DAC init to avoid stale FIFO/source status
     * when previous audio users exited unexpectedly.
     */
    bk_aud_hardware_reset();
    ret = bk_aud_dac_init(&aud_dac_cfg);
    if (ret != BK_OK) {
        TKL_DAC_LOG("bk_aud_dac_init fail\n");
        goto __dac_init_exit;
    }

#if CONFIG_AUD_DRIVER_V2
    ret = bk_aud_dac_set_sample_rate(gl_dac_ctx->dac_source, config->sample_rate);
    if (ret != BK_OK) {
        TKL_DAC_LOG("set sample rate fail, source=%d, rate=%d\n",
                    gl_dac_ctx->dac_source, config->sample_rate);
        goto __dac_init_exit;
    }
#endif

    if (config->volume == 0) {
        bk_aud_dac_mute();
    } else {
        bk_aud_dac_unmute();
    }

    /* frame_size is the per-frame byte count fed to the DMA/FIFO; it must scale
     * with the channel count (stereo packs an L+R pair per sample instant). The
     * macro itself only covers one channel, so multiply by chan_num. */
    gl_dac_ctx->frame_size = TKL_DAC_FRAME_SIZE_MS(config->sample_rate, config->sample_bits,
                                                      config->frame_time_ms) * gl_dac_ctx->cfg.chan_num;
    if (gl_dac_ctx->frame_size == 0) {
        TKL_DAC_LOG("dac frame size calc fail\n");
        ret = BK_FAIL;
        goto __dac_init_exit;
    }

    /* DMA config */
    ret = __tkl_aud_dac_dma_config(port);
    if (ret != BK_OK) {
        TKL_DAC_LOG("dma config fail\n");
        goto __dac_init_exit;
    }

    gl_dac_ctx->port = port;
    gl_dac_ctx->stat = TKL_AUD_DAC_STAT_INIT;

    return OPRT_OK;

__dac_init_exit:

    gl_dac_ctx->stat = TKL_AUD_DAC_STAT_IDLE;

    __tkl_aud_dac_dma_deconfig(port);
    bk_aud_dac_deinit();

    if (gl_dac_ctx) {
        tkl_system_free(gl_dac_ctx);
        gl_dac_ctx = NULL;
    }
    return OPRT_COM_ERROR;
}

/*
 * On-board speaker analog power + power-amplifier enable.
 *
 * The DAC *digital* path (DMA, FIFO, tx callbacks) runs fine without these, but
 * there is NO audible output until:
 *   the speaker PA enable pin (cfg.pa_pin, active-high) is driven when valid.
 * PA GPIO comes from tkl_aud_dac_init() config; use TKL_AUD_DAC_PA_PIN_NONE to skip.
 */
static void __tkl_aud_dac_spk_power_on(void)
{
    UINT32_T pa_pin;

    if (gl_dac_ctx == NULL) {
        return;
    }

    pa_pin = gl_dac_ctx->cfg.pa_pin;
    if (pa_pin >= TUYA_GPIO_NUM_MAX) {
        return;
    }

    gpio_dev_unmap((gpio_id_t)pa_pin);
    (void)bk_gpio_enable_output((gpio_id_t)pa_pin);
    (void)bk_gpio_set_output_high((gpio_id_t)pa_pin);
}

static void __tkl_aud_dac_spk_power_off(void)
{
    UINT32_T pa_pin;

    if (gl_dac_ctx == NULL) {
        return;
    }

    pa_pin = gl_dac_ctx->cfg.pa_pin;
    if (pa_pin >= TUYA_GPIO_NUM_MAX) {
        return;
    }

    (void)bk_gpio_set_output_low((gpio_id_t)pa_pin);
}

/**
 * @brief Start DAC playback.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @return OPRT_OK on success, other error codes on failure.
 * @note Prefills the DMA ring with silence before enabling DMA and DAC.
 *       After stop, the GDMA channel is rebuilt (deinit+init, not dma_stop)
 *       so dest-stall is cleared, then the same first-start sequence is used.
 * @note Calling this function after playback has already started returns
 *       success without reinitializing the hardware.
 */
OPERATE_RET tkl_aud_dac_start(TUYA_AUDIO_DAC_PORT_E port)
{
    bk_err_t ret = BK_OK;
    (void)port;

    if (gl_dac_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (gl_dac_ctx->stat == TKL_AUD_DAC_STAT_START) {
        return OPRT_OK;
    } else if (gl_dac_ctx->stat == TKL_AUD_DAC_STAT_IDLE ||
               gl_dac_ctx->stat == TKL_AUD_DAC_STAT_DEINIT) {
        return OPRT_COM_ERROR;
    }

    bk_printf("port: %d \n", gl_dac_ctx->port);

    if (gl_dac_ctx->stat == TKL_AUD_DAC_STAT_STOP) {
        ret = __tkl_aud_dac_dma_reset(gl_dac_ctx->port);
        if (ret != BK_OK) {
            TKL_DAC_LOG("dma reset fail on restart\n");
            return OPRT_COM_ERROR;
        }
    }

    /*
     * DMA is not running yet (first start, or just reset). Queue silence
     * before bk_dma_start() so the first FIFO fill does not underrun.
     */
    __tkl_aud_dac_pause_dma();
    __tkl_aud_dac_fill_silence();

#if CONFIG_AUD_DRIVER_V2
    ret = bk_aud_dac_source_enable(gl_dac_ctx->port, gl_dac_ctx->dac_source, 1);
    if (ret != BK_OK) {
        TKL_DAC_LOG("source enable fail, ch=%d, source=%d\n",
                    gl_dac_ctx->port, gl_dac_ctx->dac_source);
        return OPRT_COM_ERROR;
    }
#endif

    ret = bk_dma_start(gl_dac_ctx->spk_dma_id);
    if (ret != BK_OK) {
        TKL_DAC_LOG("dma start fail\n");
#if CONFIG_AUD_DRIVER_V2
        (void)bk_aud_dac_source_enable(gl_dac_ctx->port, gl_dac_ctx->dac_source, 0);
#endif
        return OPRT_COM_ERROR;
    }

#if CONFIG_AUD_DRIVER_V1
    ret = bk_aud_dac_start();
#elif CONFIG_AUD_DRIVER_V2
    ret = bk_aud_dac_start(AUD_DAC_CHL_LR);
#endif
    if (ret != BK_OK) {
        TKL_DAC_LOG("dac start fail\n");
        __tkl_aud_dac_pause_dma();
#if CONFIG_AUD_DRIVER_V2
        (void)bk_aud_dac_source_enable(gl_dac_ctx->port, gl_dac_ctx->dac_source, 0);
#endif
        return OPRT_COM_ERROR;
    }

    if (gl_dac_ctx->cfg.volume > 0) {
        bk_aud_dac_unmute();
    }

    /* Power the analog output stage + enable the speaker PA, otherwise the DAC
     * runs but stays silent (see helper comment above). */
    __tkl_aud_dac_spk_power_on();

    gl_dac_ctx->stat = TKL_AUD_DAC_STAT_START;
    TKL_DAC_LOG("dac start ok\n");

    return OPRT_OK;
}

/**
 * @brief Stop DAC playback.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @return OPRT_OK on success, other error codes on failure.
 * @note Mutes output, disables the digital source and analog DAC channels.
 *       Does not deinit DAC, so volume/gain registers remain writable.
 * @note GDMA stays allocated: REPEAT mode cannot bk_dma_stop(); the ring is
 *       cleared to park src_pause_addr instead. The channel is rebuilt on the
 *       next start via dma_deinit+init.
 * @note Calling this function after playback has already stopped returns
 *       success without additional hardware operations.
 */
OPERATE_RET tkl_aud_dac_stop(TUYA_AUDIO_DAC_PORT_E port)
{
    (void)port;

    if (gl_dac_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (gl_dac_ctx->stat == TKL_AUD_DAC_STAT_STOP) {
        return OPRT_OK;
    } else if (gl_dac_ctx->stat == TKL_AUD_DAC_STAT_IDLE ||
               gl_dac_ctx->stat == TKL_AUD_DAC_STAT_DEINIT) {
        return OPRT_COM_ERROR;
    }

    /* Disable the speaker PA before tearing down the data path. */
    __tkl_aud_dac_spk_power_off();

    bk_aud_dac_mute();

#if CONFIG_AUD_DRIVER_V2
    (void)bk_aud_dac_source_enable(gl_dac_ctx->port, gl_dac_ctx->dac_source, 0);
    (void)bk_aud_dac_stop(AUD_DAC_CHL_LR);
#elif CONFIG_AUD_DRIVER_V1
    (void)bk_aud_dac_stop();
#endif

    /* Pause GDMA without bk_dma_stop() (unsupported in REPEAT mode). */
    __tkl_aud_dac_pause_dma();

    gl_dac_ctx->stat = TKL_AUD_DAC_STAT_STOP;

    return OPRT_OK;
}

/**
 * @brief Update the DAC output volume.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @param[in] volume User volume 0..100 (0=mute)
 * @return OPRT_OK on success, other error codes on failure.
 */
OPERATE_RET tkl_aud_dac_set_volume(TUYA_AUDIO_DAC_PORT_E port, UINT32_T volume)
{
    (void)port;

    if (gl_dac_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (gl_dac_ctx->stat == TKL_AUD_DAC_STAT_IDLE ||
        gl_dac_ctx->stat == TKL_AUD_DAC_STAT_DEINIT) {
        return OPRT_COM_ERROR;
    }

#if CONFIG_AUD_DRIVER_V2
    return __tkl_aud_dac_apply_v2_volume(volume);
#else
    uint32_t input_volume = volume;
    uint32_t hw_gain = __tkl_aud_dac_map_volume_to_gain(volume);

    TKL_DAC_LOG("volume set input=%d hw_gain=0x%02x\n", input_volume, hw_gain);

    if (hw_gain == 0) {
        bk_aud_dac_mute();
    } else {
        bk_aud_dac_unmute();
    }

    if (bk_aud_dac_set_dig_gain(hw_gain) != BK_OK) {
        TKL_DAC_LOG("set dac gain fail\n");
        return OPRT_COM_ERROR;
    }

    gl_dac_ctx->cfg.volume = (uint8_t)(input_volume > 100 ? 100 : input_volume);

    return OPRT_OK;
#endif
}

/**
 * @brief Write PCM data into the DAC DMA ring buffer.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @param[in] buffer Pointer to the PCM data buffer.
 * @param[in] len PCM data length in bytes.
 * @return OPRT_OK on success, `OPRT_OS_ADAPTER_DAC_BUSY` when the ring buffer
 *         does not have enough free space, retry after delay, or other error
 *         codes on failure.
 * @note The DAC must already be in the started state before this function is
 *       called.
 * @note Data is copied into the internal DMA ring buffer, so the caller must
 *       ensure that enough free space is available for the requested length.
 */
OPERATE_RET tkl_aud_dac_write(TUYA_AUDIO_DAC_PORT_E port, UINT8_T *buffer, UINT32_T len)
{
    UINT32_T write_len;
    UINT32_T free_len;

    (void)port;

    if (gl_dac_ctx == NULL || buffer == NULL || len == 0) {
        return OPRT_INVALID_PARM;
    }

    if (gl_dac_ctx->stat != TKL_AUD_DAC_STAT_START) {
        return OPRT_COM_ERROR;
    }

    free_len = ring_buffer_get_free_size(&gl_dac_ctx->spk_rb);
    if (free_len < len) {
        return OPRT_OS_ADAPTER_DAC_BUSY;
    }

    write_len = ring_buffer_write(&gl_dac_ctx->spk_rb, buffer, len);
    if (write_len != len) {
        TKL_DAC_LOG("ring_buffer_write error %d, expect %d\n", write_len, len);
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

/**
 * @brief Deinitialize the DAC driver instance.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @return OPRT_OK on success, other error codes on failure.
 * @note If playback is still running, this function stops it first and then
 *       releases DMA, DAC, callback, and power-management resources.
 */
OPERATE_RET tkl_aud_dac_deinit(TUYA_AUDIO_DAC_PORT_E port)
{
    (void)port;

    if (gl_dac_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (gl_dac_ctx->stat == TKL_AUD_DAC_STAT_START) {
        tkl_aud_dac_stop(gl_dac_ctx->port);
    }

    gl_dac_ctx->stat = TKL_AUD_DAC_STAT_DEINIT;
    gl_dac_ctx->frame_cb = NULL;
    gl_dac_ctx->frame_cb_arg = NULL;

    __tkl_aud_dac_dma_deconfig(gl_dac_ctx->port);
#if CONFIG_AUD_DRIVER_V2
    (void)bk_aud_dac_source_enable(gl_dac_ctx->port, gl_dac_ctx->dac_source, 0);
    (void)bk_aud_dac_stop(aud_dac_cfg.dac_chl);
#elif CONFIG_AUD_DRIVER_V1
    (void)bk_aud_dac_stop();
#endif
    bk_aud_dac_deinit();

    tkl_system_free(gl_dac_ctx);
    gl_dac_ctx = NULL;

    return OPRT_OK;
}

/**
 * @brief Get the configured DAC frame size in bytes.
 * @param[in] port DAC output port. This parameter is ignored on T5.
 * @return Frame size in bytes, or `0` if the DAC driver is not initialized.
 */
UINT32_T tkl_aud_dac_get_frame_size(TUYA_AUDIO_DAC_PORT_E port)
{
    (void)port;

    if (gl_dac_ctx == NULL) {
        return 0;
    }

    return gl_dac_ctx->frame_size;
}
