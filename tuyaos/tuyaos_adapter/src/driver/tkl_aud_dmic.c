/**
 * @file tkl_aud_dmic.c
 * @brief TKL audio digital (DMIC) driver implementation
 * @version 1.0
 * @date 2025-03-25
 * @copyright Copyright (c) Tuya Inc.
 */

#include <common/bk_include.h>
#include <os/os.h>

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
#include "tkl_aud_pm.h"
#include "gpio_driver.h"

extern void bk_printf(const char *fmt, ...);

#define MODULE_LOG_HELPER(fmt, ...) \
    bk_printf(fmt, ##__VA_ARGS__)
#define TKL_AUD_DMIC_LOG(...) MODULE_LOG_HELPER(__VA_ARGS__)

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define AUD_DMIC_FRAME_SIZE(sr, sb, t, ch)  (((sr) * ((sb) / 8) / 1000) * (ch) * (t))

#define DMIC_DMA_BUF_SAFE_INTERVAL    (8)
#if CONFIG_AUD_DRIVER_V2
#define TKL_DMIC_CLK_GPIO             GPIO_6
#define TKL_DMIC_DAT_GPIO             GPIO_5
#else
#define TKL_DMIC_CLK_GPIO             GPIO_8
#define TKL_DMIC_DAT_GPIO             GPIO_9
#endif

/* ---------------------------------------------------------------------------
 * Type definitions
 * --------------------------------------------------------------------------- */
typedef struct {
    TKL_AUD_DMIC_CFG_T   cfg;
    bool                    is_open;
    bool                    pm_voted;
#if CONFIG_AUD_DRIVER_V2
    aud_dmic_mode_t          dmic_mode;
    uint32_t                 adc_ch_bitmap;
#endif
    uint32_t                frame_size;
    uint32_t                dma_frame_size;
    dma_id_t                dmic_dma_id;
    RingBufferContext       dmic_rb;
    int8_t                 *dmic_dma_buf;
    uint8_t                *read_buf;
} tkl_aud_dmic_ctx_t;

typedef struct {
    gpio_id_t clk_gpio;
    gpio_id_t data_gpio;
} tkl_aud_dmic_pinmux_cfg_t;

/* ---------------------------------------------------------------------------
 * File scope variables
 * --------------------------------------------------------------------------- */
STATIC tkl_aud_dmic_ctx_t *gl_dmic_ctx = NULL;
STATIC tkl_aud_dmic_pinmux_cfg_t gl_dmic_pinmux = {
    .clk_gpio = TKL_DMIC_CLK_GPIO,
    .data_gpio = TKL_DMIC_DAT_GPIO,
};

/* ---------------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------------- */
STATIC OPERATE_RET __tkl_aud_dmic_pinmux_validate(UINT32_T clk_gpio, UINT32_T data_gpio)
{
    if ((clk_gpio >= GPIO_NUM) || (data_gpio >= GPIO_NUM)) {
        TKL_AUD_DMIC_LOG("dmic: invalid pinmux clk=%u data=%u\n", clk_gpio, data_gpio);
        return OPRT_INVALID_PARM;
    }

    return OPRT_OK;
}

STATIC VOID_T __tkl_aud_dmic_pinmux_apply(gpio_dev_t clk_dev, gpio_dev_t data_dev)
{
    gpio_dev_unmap(gl_dmic_pinmux.clk_gpio);
    gpio_dev_map(gl_dmic_pinmux.clk_gpio, clk_dev);

    gpio_dev_unmap(gl_dmic_pinmux.data_gpio);
    gpio_dev_map(gl_dmic_pinmux.data_gpio, data_dev);
}

/* ---------------------------------------------------------------------------
 * Function implementations
 * --------------------------------------------------------------------------- */

OPERATE_RET tkl_aud_dmic_pinmux_config(UINT32_T clk_gpio, UINT32_T data_gpio)
{
    OPERATE_RET ret = __tkl_aud_dmic_pinmux_validate(clk_gpio, data_gpio);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (gl_dmic_ctx != NULL) {
        TKL_AUD_DMIC_LOG("dmic: pinmux config should be set before init\n");
        return OPRT_COM_ERROR;
    }

    gl_dmic_pinmux.clk_gpio = (gpio_id_t)clk_gpio;
    gl_dmic_pinmux.data_gpio = (gpio_id_t)data_gpio;

    TKL_AUD_DMIC_LOG("dmic: pinmux configured clk=%u data=%u\n",
                     (uint32_t)gl_dmic_pinmux.clk_gpio,
                     (uint32_t)gl_dmic_pinmux.data_gpio);

    return OPRT_OK;
}

/**
 * @brief DMA finish ISR for DMIC
 * @return none
 * @note Reads one DMA frame from ring buffer, extracts single channel
 *       if needed, then calls upper_cb.
 */
STATIC VOID_T __tkl_aud_dmic_dma_finish_isr(void)
{
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

#if 1
    gl_dmic_ctx->cfg.upper_cb(TUYA_AUDIO_FRAME_EVENT_DMIC_RX,
            gl_dmic_ctx->read_buf,
            gl_dmic_ctx->frame_size,
            gl_dmic_ctx->cfg.args);
#endif
}

/**
 * @brief Release DMA channel and ring buffer
 * @return BK_OK on success
 */
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

/**
 * @brief Configure DMA channel for DMIC ring buffer transfer
 * @return BK_OK on success, BK_FAIL on error
 */
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
    #if CONFIG_AUD_DRIVER_V2
    dma_config.src.dev = DMA_DEV_AUD_MIC0;
    #else
    dma_config.src.dev = DMA_DEV_AUD_DMIC;
    #endif
    dma_config.dst.dev = DMA_DEV_DTCM;
    dma_config.src.width = DMA_DATA_WIDTH_32BITS;
    dma_config.dst.width = DMA_DATA_WIDTH_32BITS;

    #if CONFIG_AUD_DRIVER_V2
    if (bk_aud_adc_get_fifo_addr(AUD_ADC_MIC_DATA_BUS_0, &dmic_port_addr) != BK_OK) {
    #else
    if (bk_aud_dmic_get_fifo_addr(&dmic_port_addr) != BK_OK) {
    #endif
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
 * @brief Initialize audio digital (DMIC) input
 * @param[in] port Digital audio port number
 * @param[in] config DMIC configuration
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_aud_dmic_init(TUYA_AUDIO_DMIC_PORT_E port, TKL_AUD_DMIC_CFG_T *config)
{
    if (config == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (config->chan != TUYA_AUDIO_DMIC_CHANNEL_LR) {
        TKL_AUD_DMIC_LOG("dmic: chan %d not support\n", config->chan);
        return OPRT_INVALID_PARM;
    }

    if (config->sample_bits != 16) {
        TKL_AUD_DMIC_LOG("dmic: bits %d not support, only 16bits\n", config->sample_bits);
        return OPRT_INVALID_PARM;
    }

    TKL_AUD_DMIC_LOG("dmic: init request port=%d chan=%d bits=%u rate=%u frame_ms=%u cb_set=%d args=%p\n",
                     port, config->chan, config->sample_bits, config->sample_rate,
                     config->frame_time_ms, config->upper_cb != NULL, config->args);

    if (gl_dmic_ctx != NULL) {
        TKL_AUD_DMIC_LOG("dmic: already initialized\n");
        return OPRT_COM_ERROR;
    }

    bk_err_t ret = BK_OK;
    gpio_dev_t dmic_clk_func = GPIO_DEV_DMIC1_CLK;
    gpio_dev_t dmic_data_func = GPIO_DEV_DMIC1_DAT;
#if CONFIG_AUD_DRIVER_V2
    aud_dmic_channel_t dmic_chl = AUD_DMIC_CHANNEL_L;
    aud_dmic_mode_t dmic_mode = AUD_DMIC_MODE_0;
    aud_adc_config_t tkl_aud_adc_cfg = DEFAULT_AUD_ADC_CONFIG();
    uint32_t adc_ch_bitmap = (1 << AUD_ADC_CHL_0);
#else
    (void)port;
    aud_dmic_chl_t dmic_chl = AUD_DMIC_CHL_LR;
#endif

    // 初始化上下文控制块
    gl_dmic_ctx = (tkl_aud_dmic_ctx_t *)tkl_system_psram_malloc(sizeof(tkl_aud_dmic_ctx_t));
    if (gl_dmic_ctx == NULL) {
        TKL_AUD_DMIC_LOG("dmic: malloc context fail\n");
        return OPRT_MALLOC_FAILED;
    }
    os_memset(gl_dmic_ctx, 0, sizeof(tkl_aud_dmic_ctx_t));
    os_memcpy(&gl_dmic_ctx->cfg, config, sizeof(TKL_AUD_DMIC_CFG_T));

    // 设置主频
    tkl_aud_pm_acquire();
    gl_dmic_ctx->pm_voted = true;

#if CONFIG_AUD_DRIVER_V2
    if (port == TUYA_AUDIO_DMIC_PORT_1) {
        dmic_mode = AUD_DMIC_MODE_1;
        dmic_clk_func = GPIO_DEV_DMIC2_CLK;
        dmic_data_func = GPIO_DEV_DMIC2_DAT;
        adc_ch_bitmap |= (1 << AUD_ADC_CHL_1) | (1 << AUD_ADC_CHL_2);
    }
#endif

    // 配置DMIC
    aud_dmic_config_t tkl_aud_dmic_cfg = DEFAULT_AUD_DMIC_CONFIG();
#if CONFIG_AUD_DRIVER_V2
    tkl_aud_dmic_cfg.channel = dmic_chl;
    tkl_aud_dmic_cfg.dmic_mode = dmic_mode;
    tkl_aud_dmic_cfg.dmic_clk_gpio = gl_dmic_pinmux.clk_gpio;
    tkl_aud_dmic_cfg.dmic_data_gpio = gl_dmic_pinmux.data_gpio;
    tkl_aud_adc_cfg.sample_rate = config->sample_rate;
    ret = bk_aud_adc_init(&tkl_aud_adc_cfg);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: bk_aud_adc_init fail\n");
        goto __dmic_init_exit;
    }

    gl_dmic_ctx->dmic_mode = dmic_mode;
    gl_dmic_ctx->adc_ch_bitmap = adc_ch_bitmap;
#else
    tkl_aud_dmic_cfg.dmic_chl = dmic_chl;
    tkl_aud_dmic_cfg.samp_rate = config->sample_rate;
    tkl_aud_dmic_cfg.clk_src = AUD_CLK_APLL;
#endif

    TKL_AUD_DMIC_LOG("dmic config, chl=%d clk_gpio=%u data_gpio=%u\n",
                     dmic_chl,
                     (uint32_t)gl_dmic_pinmux.clk_gpio,
                     (uint32_t)gl_dmic_pinmux.data_gpio);


    ret = bk_aud_dmic_init(&tkl_aud_dmic_cfg);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: bk_aud_dmic_init fail\n");
        goto __dmic_init_exit;
    }

    __tkl_aud_dmic_pinmux_apply(dmic_clk_func, dmic_data_func);

    // 计算帧大小
    gl_dmic_ctx->frame_size = AUD_DMIC_FRAME_SIZE(config->sample_rate, config->sample_bits, config->frame_time_ms, 2);
    gl_dmic_ctx->dma_frame_size = AUD_DMIC_FRAME_SIZE(config->sample_rate, config->sample_bits, config->frame_time_ms, 2);
    TKL_AUD_DMIC_LOG("dmic: computed frame_size=%u dma_frame_size=%u\n",
                     gl_dmic_ctx->frame_size, gl_dmic_ctx->dma_frame_size);

    // 配置DMA
    ret = __tkl_aud_dmic_dma_config();
    TKL_AUD_DMIC_LOG("dmic: __tkl_aud_dmic_dma_config ret=%d\n", ret);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: dma config fail\n");
        goto __dmic_init_exit;
    }

    // 分配读缓冲区，临时存储一帧数据，用于上层回调处理
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
#if CONFIG_AUD_DRIVER_V2
    bk_aud_adc_deinit();
#else
    bk_aud_dmic_deinit();
#endif
    if (gl_dmic_ctx->pm_voted) {
        tkl_aud_pm_release();
        gl_dmic_ctx->pm_voted = false;
    }

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

/**
 * @brief Start audio digital (DMIC) capture
 * @param[in] port Digital audio port number
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_aud_dmic_start(TUYA_AUDIO_DMIC_PORT_E port)
{
    (void) port;

    if (gl_dmic_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (gl_dmic_ctx->is_open) {
        TKL_AUD_DMIC_LOG("dmic: already started\n");
        return OPRT_OK;
    }

    bk_err_t ret = bk_dma_start(gl_dmic_ctx->dmic_dma_id);
    TKL_AUD_DMIC_LOG("dmic: bk_dma_start ret=%d\n", ret);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: dma start fail\n");
        return OPRT_COM_ERROR;
    }

#if CONFIG_AUD_DRIVER_V2
    ret = bk_aud_adc_start(AUD_ADC_CHL_0);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: bk_aud_adc_start(ch0) fail\n");
        bk_dma_stop(gl_dmic_ctx->dmic_dma_id);
        return OPRT_COM_ERROR;
    }

    if (gl_dmic_ctx->dmic_mode == AUD_DMIC_MODE_1) {
        ret = bk_aud_adc_start(AUD_ADC_CHL_1);
        if (ret != BK_OK) {
            TKL_AUD_DMIC_LOG("dmic: bk_aud_adc_start(ch1) fail\n");
            bk_aud_adc_stop(AUD_ADC_CHL_0);
            bk_dma_stop(gl_dmic_ctx->dmic_dma_id);
            return OPRT_COM_ERROR;
        }

        ret = bk_aud_adc_start(AUD_ADC_CHL_2);
        if (ret != BK_OK) {
            TKL_AUD_DMIC_LOG("dmic: bk_aud_adc_start(ch2) fail\n");
            bk_aud_adc_stop(AUD_ADC_CHL_1);
            bk_aud_adc_stop(AUD_ADC_CHL_0);
            bk_dma_stop(gl_dmic_ctx->dmic_dma_id);
            return OPRT_COM_ERROR;
        }
    }

    bk_aud_adc_enable_used_channel(gl_dmic_ctx->adc_ch_bitmap);
#else
    ret = bk_aud_dmic_start();
    TKL_AUD_DMIC_LOG("dmic: bk_aud_dmic_start ret=%d\n", ret);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: start fail\n");
        bk_dma_stop(gl_dmic_ctx->dmic_dma_id);
        return OPRT_COM_ERROR;
    }
#endif

    gl_dmic_ctx->is_open = true;
    TKL_AUD_DMIC_LOG("dmic: start ok\n");

    return OPRT_OK;
}

/**
 * @brief Set audio digital (DMIC) gain
 * @param[in] port Digital audio port number
 * @param[in] gain Digital gain value
 * @return OPRT_OK on success, error code on failure
 * @note DMIC gain control is hardware dependent. Currently a no-op
 *       placeholder; implement when hardware gain register is available.
 */
OPERATE_RET tkl_aud_dmic_set_vol(TUYA_AUDIO_DMIC_PORT_E port, UINT32_T gain)
{
    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief Stop audio digital (DMIC) capture
 * @param[in] port Digital audio port number
 * @return OPRT_OK on success, error code on failure
 */
OPERATE_RET tkl_aud_dmic_stop(TUYA_AUDIO_DMIC_PORT_E port)
{
    (void) port;

    if (gl_dmic_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (!gl_dmic_ctx->is_open) {
        return OPRT_OK;
    }

    bk_err_t ret = bk_dma_stop(gl_dmic_ctx->dmic_dma_id);
    TKL_AUD_DMIC_LOG("dmic: bk_dma_stop ret=%d\n", ret);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: dma stop fail\n");
        return OPRT_COM_ERROR;
    }

#if CONFIG_AUD_DRIVER_V2
    if (gl_dmic_ctx->dmic_mode == AUD_DMIC_MODE_1) {
        (void)bk_aud_adc_stop(AUD_ADC_CHL_2);
        (void)bk_aud_adc_stop(AUD_ADC_CHL_1);
    }
    ret = bk_aud_adc_stop(AUD_ADC_CHL_0);
    TKL_AUD_DMIC_LOG("dmic: bk_aud_adc_stop(ch0) ret=%d\n", ret);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: adc stop fail\n");
        return OPRT_COM_ERROR;
    }
#else
    ret = bk_aud_dmic_stop();
    TKL_AUD_DMIC_LOG("dmic: bk_aud_dmic_stop ret=%d\n", ret);
    if (ret != BK_OK) {
        TKL_AUD_DMIC_LOG("dmic: stop fail\n");
        return OPRT_COM_ERROR;
    }
#endif

    gl_dmic_ctx->is_open = false;

    return OPRT_OK;
}

/**
 * @brief Deinitialize audio digital (DMIC) input
 * @param[in] port Digital audio port number
 * @return none
 */
VOID_T tkl_aud_dmic_deinit(TUYA_AUDIO_DMIC_PORT_E port)
{
    if (gl_dmic_ctx == NULL) {
        return;
    }

    if (gl_dmic_ctx->is_open) {
        tkl_aud_dmic_stop(port);
    }

    __tkl_aud_dmic_dma_deconfig();
#if CONFIG_AUD_DRIVER_V2
    bk_aud_adc_deinit();
#else
    bk_aud_dmic_deinit();
#endif

    if (gl_dmic_ctx->pm_voted) {
        tkl_aud_pm_release();
        gl_dmic_ctx->pm_voted = false;
    }

    if (gl_dmic_ctx->read_buf) {
        tkl_system_psram_free(gl_dmic_ctx->read_buf);
    }

    tkl_system_psram_free(gl_dmic_ctx);
    gl_dmic_ctx = NULL;
}
