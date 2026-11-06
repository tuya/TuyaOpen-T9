/**
 * @file tkl_aud_capture.c
 * @brief Shared audio-capture core for tkl_aud_adc.c / tkl_aud_dmic.c
 * @version 1.0
 * @date 2026-10-21
 * @copyright Copyright (c) Tuya Inc. All Rights Reserved.
 *
 * On BK7259 (CONFIG_AUD_DRIVER_V2) the analog and digital mics are NOT two
 * peripherals: ADC lanes 0..2 are interleaved onto one FIFO (mic0_data_bus)
 * and drained by one GDMA channel. The digital mic is injected into those same
 * lanes by rewriting sys_cfg.digmic_en:
 *
 *   digmic_en = 0x0 -> lanes 0,1,2 analog
 *   digmic_en = 0x1 -> lane 0 digital   (DMIC port 0, MODE_0)
 *   digmic_en = 0x6 -> lanes 1,2 digital (DMIC port 1, MODE_1)
 *   digmic_en = 0x7 -> all three digital (both ports, one pad pair each)
 *
 * digmic_en is the OR of the attached ports, so opening / closing either port
 * only rewrites that one field. The DMA geometry is therefore fixed to all
 * three lanes and never changes. This file owns the single DMA channel, ring
 * buffer, ISR, mode register and the per-lane gain store. Every started
 * interface receives its own frame, containing only the lanes its channel
 * bitmap selected.
 *
 * See tkl_aud_capture.h for the ownership / callback / conflict rules.
 */

#include <common/bk_include.h>
#include <os/os.h>
#include <os/mem.h>

#include <driver/aud_adc_types.h>
#include <driver/aud_adc.h>
#include <driver/aud_common.h>
#include <driver/dma.h>
#include <driver/audio_ring_buff.h>
#include <driver/gpio.h>
#include <driver/flash.h>
#include <driver/flash_types.h>
#include <components/bk_audio/audio_pipeline/audio_mem.h>

#include "tuya_cloud_types.h"
#include "tkl_memory.h"
#include "tkl_mutex.h"
#include "tkl_system.h"
#include "tkl_aud_capture.h"
#include "gpio_driver.h"

extern void bk_printf(const char *fmt, ...);

#define MODULE_LOG_HELPER(fmt, ...) \
    bk_printf(fmt, ##__VA_ARGS__)
#define TKL_AUD_CAP_LOG(...) MODULE_LOG_HELPER(__VA_ARGS__)

#if CONFIG_AUD_DRIVER_V2

/*
 * These three helpers live in middleware/driver/audio/aud/V2/aud_adc_driver.c
 * but have no declaration in any public header. The DMIC mode is just three
 * sys_cfg fields, so this module drives them directly instead of going through
 * bk_aud_dmic_init(), which would also re-mux pads behind our back.
 */
extern bk_err_t bk_aud_dmic_en(uint32_t v);
extern bk_err_t bk_aud_dmic_select(uint32_t chl);
extern bk_err_t bk_aud_dmic_clk_sel(uint32_t v);

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */

/* Always capture all three ADC lanes. Lane order is ascending channel index and
 * is independent of whether the lane is analog or digital. */
#define CAP_LANES                   (3U)
#define CAP_LANE_BITMAP             ((1U << 0) | (1U << 1) | (1U << 2))

/* 32-bit DMA slots per frame: one 16-bit sample sits in the low half of each
 * slot, so the ring holds twice the dense payload. */
#define CAP_SLOT_BYTES              (4U)
#define CAP_SAMPLE_BYTES            (2U)

#define CAP_DMA_BUF_SAFE_INTERVAL   (8U)
#define CAP_DMA_BUF_FRAME_CNT       (8U)

#define CAP_DEFAULT_DIG_DB          (16.0f)  /* OEM DEFAULT_AUD_ADC_CONFIG */
#define CAP_DEFAULT_ANA_DB          (20)     /* OEM DEFAULT_AUD_ADC_CONFIG */

#define CAP_USER_BIT(u)             (1U << (UINT32_T)(u))

/* sr: sample rate, sb: sample bits, t: frame time in ms, ch: lane count */
#define CAP_FRAME_SIZE(sr, sb, t, ch)  (((sr) * ((sb) / 8) / 1000) * (ch) * (t))

/* ---------------------------------------------------------------------------
 * Types
 * --------------------------------------------------------------------------- */

typedef struct {
    BOOL_T                 attached;
    BOOL_T                 started;
    TUYA_AUDIO_INPUT_CB    cb;
    VOID_T                *args;
    UINT32_T               lane_mask;       /**< lanes this user wants delivered */
    UINT32_T               gain_lane_mask;  /**< lanes this user may move */
    TUYA_AUDIO_DMIC_PORT_E dmic_port;       /**< DMIC users only */
    UINT32_T               dmic_clk_gpio;   /**< DMIC users only, recorded per port */
    UINT32_T               dmic_data_gpio;  /**< DMIC users only, recorded per port */
    BOOL_T                 dmic_muxed;      /**< this port's pads are muxed */
} cap_user_t;

typedef struct {
    BOOL_T          inited;
    UINT32_T        sample_rate;
    UINT8_T         sample_bits;
    UINT32_T        frame_time_ms;

    UINT32_T        attached_mask;
    UINT32_T        started_mask;
    cap_user_t      users[TKL_AUD_CAP_USER_MAX];

    UINT32_T        digmic_en;      /**< current sys_cfg.digmic_en */

    dma_id_t        dma_id;
    BOOL_T          dma_owned;
    BOOL_T          rb_inited;      /**< ring_buffer_init() ran on @c rb */
    RingBufferContext rb;
    int8_t         *dma_buf;
    uint8_t        *read_buf;       /**< dma_frame_size: ring_buffer_read target */
    uint8_t        *out_buf;        /**< dense_frame_size: lane extraction target */
    uint32_t        dma_frame_size; /**< raw, 32-bit slots */
    uint32_t        dense_frame_size; /**< 16-bit, 3 lanes */
    BOOL_T          dma_started;    /**< GDMA was started; REPEAT cannot dma_stop */
    volatile BOOL_T reconfiguring;  /**< ISR must not touch the ring meanwhile */

    float           dig_db[CAP_LANES];
    int32_t         ana_db[CAP_LANES];

    BOOL_T          flash_registered;
} cap_ctx_t;

STATIC cap_ctx_t *gl_cap = NULL;

/*
 * One global mutex serialises every public core call. The context itself can
 * be destroyed by the last closer, so a per-context lock cannot protect the
 * gl_cap hand-off; a global lock also makes the lock order trivial (there is
 * only one). The DMA ISR never takes it.
 */
STATIC TKL_MUTEX_HANDLE gl_cap_lock = NULL;

/**
 * @brief Create the global lock on first use
 * @return OPRT_OK on success
 * @note The creation itself is guarded by a critical section so two tasks that
 *       race on the first audio call cannot both allocate a mutex.
 */
STATIC OPERATE_RET __cap_lock_init(VOID_T)
{
    UINT_T mask;

    if (gl_cap_lock != NULL) {
        return OPRT_OK;
    }

    mask = tkl_system_enter_critical();
    if (gl_cap_lock == NULL) {
        (void)tkl_mutex_create_init(&gl_cap_lock);
    }
    tkl_system_exit_critical(mask);

    return (gl_cap_lock != NULL) ? OPRT_OK : OPRT_COM_ERROR;
}

/* ---------------------------------------------------------------------------
 * Small helpers
 * --------------------------------------------------------------------------- */

STATIC UINT32_T __cap_lane_bit(UINT32_T lane)
{
    return (lane < CAP_LANES) ? (1U << lane) : 0U;
}

STATIC UINT32_T __cap_popcount(UINT32_T v)
{
    UINT32_T n = 0;

    while (v) {
        n += (v & 1U);
        v >>= 1;
    }
    return n;
}

/* ---------------------------------------------------------------------------
 * Lane / mode masks
 * --------------------------------------------------------------------------- */

/**
 * @brief Translate an ADC channel bitmap into lane masks
 * @param[in]  chan      requested ADC channel bitmap
 * @param[out] lane_mask lanes delivered to the ADC callback
 * @param[out] gain_mask lanes whose gain the ADC interface owns
 * @return OPRT_OK on success; OPRT_INVALID_PARM if the bitmap is empty or names
 *         a channel this platform does not have
 */
STATIC OPERATE_RET __cap_adc_masks(TUYA_AUDIO_MIC_CHANNEL_E chan,
                                   UINT32_T *lane_mask, UINT32_T *gain_mask)
{
    UINT32_T mask = (UINT32_T)chan;

    /* Only the three physical ADC lanes exist; reject empty or foreign bits. */
    if (((mask & CAP_LANE_BITMAP) == 0U) || ((mask & ~CAP_LANE_BITMAP) != 0U)) {
        return OPRT_INVALID_PARM;
    }

    *lane_mask = mask & CAP_LANE_BITMAP;
    *gain_mask = *lane_mask;
    return OPRT_OK;
}

/**
 * @brief Translate a DMIC port and channel bitmap into lane / gain masks
 * @param[in]  port      requested DMIC port
 * @param[in]  chan      requested DMIC channel bitmap
 * @param[out] lane_mask lanes delivered to the DMIC callback
 * @param[out] gain_mask lanes whose gain the DMIC interface owns
 * @param[out] digmic_en digmic_en the port implies
 * @return OPRT_OK on success
 * @note Port 0 owns DMIC channel 0 (BIT(8) -> lane 0); port 1 owns DMIC
 *       channels 1 and 2 (BIT(9)/BIT(10) -> lanes 1/2). At least one of the
 *       port's own bits must be selected and no bit of the other port may be.
 */
STATIC OPERATE_RET __cap_dmic_masks(TUYA_AUDIO_DMIC_PORT_E port,
                                    TUYA_AUDIO_MIC_CHANNEL_E chan,
                                    UINT32_T *lane_mask, UINT32_T *gain_mask,
                                    UINT32_T *digmic_en)
{
    UINT32_T bits = (UINT32_T)chan;
    UINT32_T own;
    UINT32_T mask = 0;
    UINT32_T lane;

    switch (port) {
        case TUYA_AUDIO_DMIC_PORT_0:
            /* MODE_0: lane 0 digital, lanes 1/2 stay analog. */
            own = (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_0;
            *digmic_en = 0x1U;
            break;
        case TUYA_AUDIO_DMIC_PORT_1:
            /* MODE_1: lanes 1/2 digital, lane 0 stays analog. */
            own = (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_1 | (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_2;
            *digmic_en = 0x6U;
            break;
        default:
            return OPRT_INVALID_PARM;
    }

    if (((bits & own) == 0U) || ((bits & ~own) != 0U)) {
        return OPRT_INVALID_PARM;
    }

    for (lane = 0U; lane < CAP_LANES; lane++) {
        if (bits & ((UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_0 << lane)) {
            mask |= __cap_lane_bit(lane);
        }
    }

    *lane_mask = mask;
    *gain_mask = mask;
    return OPRT_OK;
}

/**
 * @brief Compute the masks a user's request implies
 * @param[in]  user      requesting interface
 * @param[in]  req       its request
 * @param[out] lane_mask lanes delivered to its callback
 * @param[out] gain_mask lanes whose gain it owns
 * @param[out] digmic_en digmic_en the request implies (0 for the ADC)
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __cap_masks_for(TKL_AUD_CAP_USER_E user, const TKL_AUD_CAP_REQ_T *req,
                                   UINT32_T *lane_mask, UINT32_T *gain_mask,
                                   UINT32_T *digmic_en)
{
    *digmic_en = 0U;

    if (user == TKL_AUD_CAP_USER_ADC) {
        return __cap_adc_masks(req->chan, lane_mask, gain_mask);
    }

    return __cap_dmic_masks(req->dmic_port, req->chan, lane_mask, gain_mask, digmic_en);
}

/* ---------------------------------------------------------------------------
 * Mode register
 * --------------------------------------------------------------------------- */

/**
 * @brief digmic_en implied by the attached set
 * @note Each port owns one MODE bit group; en is their OR, so opening or
 *       closing either port is a pure re-evaluation of this formula.
 */
STATIC UINT32_T __cap_mode_for(cap_ctx_t *c)
{
    UINT32_T en = 0U;

    if (c->users[TKL_AUD_CAP_USER_DMIC0].attached) {
        en |= 0x1U;   /* MODE_0: lane 0 digital */
    }

    if (c->users[TKL_AUD_CAP_USER_DMIC1].attached) {
        en |= 0x6U;   /* MODE_1: lanes 1+2 digital */
    }

    return en;
}

/**
 * @brief Write digmic_en (and the two fields bk_aud_dmic_init() sets with it)
 * @return OPRT_OK on success
 * @note bk_aud_hardware_reset() - also reached through DAC init - clears the
 *       whole SYS_CFG block, so this must be re-run at every start.
 */
STATIC OPERATE_RET __cap_apply_mode(cap_ctx_t *c)
{
    bk_err_t ret;

    ret = bk_aud_dmic_clk_sel(0);
    if (ret != BK_OK) {
        TKL_AUD_CAP_LOG("cap: dmic clk_sel fail, ret=%d\n", ret);
        return OPRT_COM_ERROR;
    }

    ret = bk_aud_dmic_select(AUD_DMIC_CHANNEL_L);
    if (ret != BK_OK) {
        TKL_AUD_CAP_LOG("cap: dmic select fail, ret=%d\n", ret);
        return OPRT_COM_ERROR;
    }

    ret = bk_aud_dmic_en(c->digmic_en);
    if (ret != BK_OK) {
        TKL_AUD_CAP_LOG("cap: digmic_en(0x%x) fail, ret=%d\n",
                        c->digmic_en, ret);
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

/**
 * @brief Recompute digmic_en from the attached set and push it to hardware
 */
STATIC OPERATE_RET __cap_sync_mode(cap_ctx_t *c)
{
    UINT32_T mode = __cap_mode_for(c);

    if (mode == c->digmic_en) {
        return OPRT_OK;
    }

    c->digmic_en = mode;
    TKL_AUD_CAP_LOG("cap: digmic_en -> 0x%x\n", c->digmic_en);
    return __cap_apply_mode(c);
}

/* ---------------------------------------------------------------------------
 * DMIC pad mux
 * --------------------------------------------------------------------------- */

/**
 * @brief Mux one digital port's pads
 * @param[in] c    capture context
 * @param[in] user TKL_AUD_CAP_USER_DMIC0 or TKL_AUD_CAP_USER_DMIC1
 * @note Each port keeps its own pad pair and its own muxed flag: the two ports
 *       use different pad functions (DMIC0_* vs DMIC1_*) and may be configured
 *       on different pins.
 */
STATIC VOID_T __cap_dmic_mux(cap_ctx_t *c, TKL_AUD_CAP_USER_E user)
{
    cap_user_t *u = &c->users[user];
    gpio_dev_t clk_dev;
    gpio_dev_t dat_dev;

    if (user == TKL_AUD_CAP_USER_DMIC1) {
        clk_dev = GPIO_DEV_DMIC1_CLK;
        dat_dev = GPIO_DEV_DMIC1_DAT;
    } else {
        clk_dev = GPIO_DEV_DMIC0_CLK;
        dat_dev = GPIO_DEV_DMIC0_DAT;
    }

    (void)gpio_dev_unmap((gpio_id_t)u->dmic_clk_gpio);
    (void)gpio_dev_map((gpio_id_t)u->dmic_clk_gpio, clk_dev);

    (void)gpio_dev_unmap((gpio_id_t)u->dmic_data_gpio);
    (void)gpio_dev_map((gpio_id_t)u->dmic_data_gpio, dat_dev);

    u->dmic_muxed = TRUE;
    TKL_AUD_CAP_LOG("cap: dmic%d pads muxed clk=%u dat=%u\n",
                    (user == TKL_AUD_CAP_USER_DMIC1) ? 1 : 0,
                    u->dmic_clk_gpio, u->dmic_data_gpio);
}

STATIC VOID_T __cap_dmic_unmux(cap_ctx_t *c, TKL_AUD_CAP_USER_E user)
{
    cap_user_t *u = &c->users[user];

    if (!u->dmic_muxed) {
        return;
    }

    (void)gpio_dev_unmap((gpio_id_t)u->dmic_clk_gpio);
    (void)gpio_dev_unmap((gpio_id_t)u->dmic_data_gpio);
    u->dmic_muxed = FALSE;
    TKL_AUD_CAP_LOG("cap: dmic%d pads released\n",
                    (user == TKL_AUD_CAP_USER_DMIC1) ? 1 : 0);
}

/** @brief Release every port's pads (destroy path) */
STATIC VOID_T __cap_dmic_unmux_all(cap_ctx_t *c)
{
    UINT32_T u;

    for (u = 0U; u < (UINT32_T)TKL_AUD_CAP_USER_MAX; u++) {
        if (TKL_AUD_CAP_USER_IS_DMIC((TKL_AUD_CAP_USER_E)u)) {
            __cap_dmic_unmux(c, (TKL_AUD_CAP_USER_E)u);
        }
    }
}

/* ---------------------------------------------------------------------------
 * Flash operation notify
 * --------------------------------------------------------------------------- */

STATIC VOID_T __cap_flash_op_notify_handler(uint32_t param, void *args)
{
    (void)param;
    (void)args;
    /*
     * Keep capture running: reference onboard_mic_stream_v2 disables this path
     * to avoid frequent flash ops starving the audio stream.
     */
}

/* ---------------------------------------------------------------------------
 * DMA
 * --------------------------------------------------------------------------- */

STATIC VOID_T __cap_dma_finish_isr(VOID_T);

STATIC bk_err_t __cap_dma_deconfig(cap_ctx_t *c)
{
    if (c == NULL) {
        return BK_OK;
    }

    if (c->dma_owned) {
        bk_dma_deinit(c->dma_id);
        bk_dma_free(DMA_DEV_AUDIO, c->dma_id);
        c->dma_owned = FALSE;
    }

    if (c->dma_buf) {
        /* Failure paths can reach here before ring_buffer_init() ran; clearing
         * an uninitialised context would poke a foreign DMA id. */
        if (c->rb_inited) {
            ring_buffer_clear(&c->rb);
            c->rb_inited = FALSE;
        }
        audio_dma_mem_free(c->dma_buf);
        c->dma_buf = NULL;
    }

    return BK_OK;
}

STATIC bk_err_t __cap_dma_config(cap_ctx_t *c)
{
    bk_err_t ret = BK_OK;
    dma_config_t dma_config = {0};
    uint32_t adc_port_addr;
    uint32_t frame_size = c->dma_frame_size;

    os_memset(&dma_config, 0, sizeof(dma_config_t));

    c->dma_id = bk_dma_alloc(DMA_DEV_AUDIO);
    if ((c->dma_id < DMA_ID_0) || (c->dma_id >= DMA_ID_MAX)) {
        TKL_AUD_CAP_LOG("cap: malloc dma fail\n");
        goto exit;
    }
    c->dma_owned = TRUE;

    /* 8 frames + a safe interval for dma_set_dst_pause_addr flow control. */
    c->dma_buf = (int8_t *)audio_dma_mem_calloc(CAP_DMA_BUF_FRAME_CNT,
                                   frame_size + CAP_DMA_BUF_SAFE_INTERVAL / CAP_DMA_BUF_FRAME_CNT);
    if (c->dma_buf == NULL) {
        TKL_AUD_CAP_LOG("cap: malloc dma buffer fail\n");
        goto exit;
    }

    dma_config.mode = DMA_WORK_MODE_REPEAT;
    dma_config.chan_prio = 1;
    dma_config.trans_type = DMA_TRANS_DEFAULT;
    dma_config.src.dev = DMA_DEV_AUD_MIC0;
    dma_config.dst.dev = DMA_DEV_DTCM;
    dma_config.src.width = DMA_DATA_WIDTH_32BITS;
    dma_config.dst.width = DMA_DATA_WIDTH_32BITS;

    if (bk_aud_adc_get_fifo_addr(AUD_ADC_MIC_DATA_BUS_0, &adc_port_addr) != BK_OK) {
        TKL_AUD_CAP_LOG("cap: get adc fifo address failed\n");
        goto exit;
    }

    dma_config.src.addr_inc_en = DMA_ADDR_INC_ENABLE;
    dma_config.src.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
    dma_config.src.start_addr = adc_port_addr;
    dma_config.src.end_addr = adc_port_addr + 4;

    dma_config.dst.addr_inc_en = DMA_ADDR_INC_ENABLE;
    dma_config.dst.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
    dma_config.dst.start_addr = (uint32_t)(uintptr_t)c->dma_buf;
    dma_config.dst.end_addr = (uint32_t)(uintptr_t)c->dma_buf
                              + frame_size * CAP_DMA_BUF_FRAME_CNT + CAP_DMA_BUF_SAFE_INTERVAL;

    ret = bk_dma_init(c->dma_id, &dma_config);
    if (ret != BK_OK) {
        TKL_AUD_CAP_LOG("cap: dma_init fail, ret=%d\n", ret);
        goto exit;
    }

    bk_dma_set_transfer_len(c->dma_id, frame_size);

    bk_dma_register_isr(c->dma_id, NULL, (void *)__cap_dma_finish_isr);
    bk_dma_enable_finish_interrupt(c->dma_id);

#if (CONFIG_SPE)
    bk_dma_set_dest_sec_attr(c->dma_id, DMA_ATTR_SEC);
    bk_dma_set_src_sec_attr(c->dma_id, DMA_ATTR_SEC);
#endif

    /* ring_buffer_init parks dst_pause_addr at buf + capacity - 8. */
    ring_buffer_init(&c->rb,
                     (uint8_t *)c->dma_buf,
                     frame_size * CAP_DMA_BUF_FRAME_CNT + CAP_DMA_BUF_SAFE_INTERVAL,
                     c->dma_id,
                     RB_DMA_TYPE_WRITE);
    c->rb_inited = TRUE;

    TKL_AUD_CAP_LOG("cap: dma_config id=%d transfer_len=%u fifo=0x%08x rb=%u\n",
                    c->dma_id, frame_size, adc_port_addr,
                    frame_size * CAP_DMA_BUF_FRAME_CNT + CAP_DMA_BUF_SAFE_INTERVAL);

    return BK_OK;

exit:
    __cap_dma_deconfig(c);
    return BK_FAIL;
}

/**
 * @brief Rebuild the GDMA channel without bk_dma_stop()
 * @note REPEAT-mode GDMA does not support bk_dma_stop(); the channel src-stalls
 *       after the analog ADC is stopped while the channel stays enabled.
 *       bk_dma_deinit() resets it - that is not the unsupported stop.
 */
STATIC bk_err_t __cap_dma_reset(cap_ctx_t *c)
{
    (void)__cap_dma_deconfig(c);
    return __cap_dma_config(c);
}

/**
 * @brief Park the GDMA by clearing the WRITE ring
 * @note ring_buffer_clear() sets dma_set_dst_pause_addr to the ring end, which
 *       stalls the channel without disabling it.
 */
STATIC VOID_T __cap_pause_dma(cap_ctx_t *c)
{
    if ((c == NULL) || (c->dma_buf == NULL) || (c->rb_inited == FALSE)) {
        return;
    }
    ring_buffer_clear(&c->rb);
}

/* ---------------------------------------------------------------------------
 * Frame path
 * --------------------------------------------------------------------------- */

/**
 * @brief Compact one DMA chunk from 32-bit slots to dense 16-bit samples
 * @param[in,out] buf       ring_buffer_read destination
 * @param[in]     read_size raw size in bytes
 * @return dense payload size in bytes
 * @note Every slot carries one 16-bit sample in its low half. Ascending i reads
 *       index 2*i >= i, so the in-place copy never clobbers a slot it needs.
 */
STATIC uint32_t __cap_compact(uint8_t *buf, uint32_t read_size)
{
    int16_t *raw = (int16_t *)buf;
    uint32_t slots = read_size / CAP_SLOT_BYTES;
    uint32_t i;

    for (i = 0U; i < slots; i++) {
        raw[i] = raw[2U * i];
    }
    return slots * CAP_SAMPLE_BYTES;
}

/**
 * @brief Extract the lanes named by @p mask into the shared output buffer
 * @param[in] dense      dense 3-lane frame
 * @param[in] dense_size size of @p dense in bytes
 * @param[in] mask       lanes to keep, delivered in ascending order
 * @return output size in bytes, or 0 if nothing could be extracted
 */
STATIC uint32_t __cap_extract(cap_ctx_t *c, const uint8_t *dense,
                              uint32_t dense_size, UINT32_T mask)
{
    const int16_t *src = (const int16_t *)dense;
    int16_t *dst = (int16_t *)c->out_buf;
    uint32_t pts = dense_size / (CAP_SAMPLE_BYTES * CAP_LANES);
    uint32_t out_ch = __cap_popcount(mask);
    uint32_t out_bytes = pts * out_ch * CAP_SAMPLE_BYTES;
    uint32_t i;
    UINT32_T lane;
    uint32_t o = 0U;

    if ((dst == NULL) || (out_ch == 0U) || (pts == 0U)) {
        return 0U;
    }

    if (out_bytes > c->dense_frame_size) {
        return 0U;
    }

    for (i = 0U; i < pts; i++) {
        for (lane = 0U; lane < CAP_LANES; lane++) {
            if (mask & (1U << lane)) {
                dst[o++] = src[i * CAP_LANES + lane];
            }
        }
    }

    return out_bytes;
}

/**
 * @brief Deliver one dense frame to one started user
 * @param[in] c         capture context
 * @param[in] u         user to deliver to
 * @param[in] event     frame event of that user's interface
 * @param[in] chan_base channel bit of lane 0 (ADC: BIT(0), DMIC: BIT(8))
 * @param[in] dense     dense 3-lane frame
 * @param[in] dense_size size of @p dense in bytes
 * @note The buffer handed to the callback is stack allocated and valid only for
 *       the duration of the call, matching the public type's contract. Lanes are
 *       delivered in ascending order and named in buffer->chan[].
 */
STATIC VOID_T __cap_emit(cap_ctx_t *c, cap_user_t *u,
                         TUYA_AUDIO_FRAME_EVT_E event, UINT32_T chan_base,
                         const uint8_t *dense, uint32_t dense_size)
{
    TUYA_AUDIO_INPUT_BUFFER_T buffer;
    uint8_t *buf;
    uint32_t len;
    UINT32_T lane;

    if ((u->attached == FALSE) || (u->started == FALSE) || (u->cb == NULL)) {
        return;
    }

    if (u->lane_mask == CAP_LANE_BITMAP) {
        buf = c->read_buf;
        len = dense_size;
    } else {
        len = __cap_extract(c, dense, dense_size, u->lane_mask);
        if (len == 0U) {
            return;
        }
        buf = c->out_buf;
    }

    os_memset(&buffer, 0, sizeof(buffer));
    buffer.event = event;
    buffer.data = buf;
    buffer.data_len = len;
    buffer.args = u->args;

    for (lane = 0U; lane < CAP_LANES; lane++) {
        if (u->lane_mask & __cap_lane_bit(lane)) {
            buffer.chan[buffer.chan_num] =
                (TUYA_AUDIO_MIC_CHANNEL_E)(chan_base << lane);
            buffer.chan_num++;
        }
    }

    u->cb(&buffer);
}

/**
 * @brief Hand one dense frame to every started interface
 * @note Each user gets its own frame with exactly the lanes its channel bitmap
 *       selected. The lane sets of attached users never overlap, so no user
 *       sees samples that belong to another interface.
 */
STATIC VOID_T __cap_dispatch(cap_ctx_t *c, uint32_t dense_size)
{
    __cap_emit(c, &c->users[TKL_AUD_CAP_USER_ADC],
               TUYA_AUDIO_FRAME_EVENT_ADC_RX, (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_0,
               c->read_buf, dense_size);
    __cap_emit(c, &c->users[TKL_AUD_CAP_USER_DMIC0],
               TUYA_AUDIO_FRAME_EVENT_DMIC_RX, (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_0,
               c->read_buf, dense_size);
    __cap_emit(c, &c->users[TKL_AUD_CAP_USER_DMIC1],
               TUYA_AUDIO_FRAME_EVENT_DMIC_RX, (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_0,
               c->read_buf, dense_size);
}

/**
 * @brief DMA finish ISR
 * @note ring_buffer_read() atomically re-reads the DMA write pointer, copies the
 *       chunk and re-parks dst_pause_addr so the DMA can keep writing.
 */
STATIC VOID_T __cap_dma_finish_isr(VOID_T)
{
    cap_ctx_t *c = gl_cap;

    if ((c == NULL) || !c->inited) {
        return;
    }

    if (c->reconfiguring || (c->dma_frame_size == 0U)) {
        return;
    }

    while (ring_buffer_get_fill_size(&c->rb) >= c->dma_frame_size) {
        uint32_t read_size = ring_buffer_read(&c->rb, c->read_buf, c->dma_frame_size);

        if (read_size != c->dma_frame_size) {
            break;
        }

        __cap_dispatch(c, __cap_compact(c->read_buf, read_size));
    }
}

/* ---------------------------------------------------------------------------
 * Hardware bring-up
 * --------------------------------------------------------------------------- */

STATIC VOID_T __cap_stop_channels(VOID_T)
{
    uint32_t ch;

    for (ch = 0U; ch < CAP_LANES; ch++) {
        (void)bk_aud_adc_stop((aud_adc_chl_t)ch);
    }
}

STATIC bk_err_t __cap_start_channels(uint32_t ch_bitmap)
{
    uint32_t ch;

    for (ch = 0U; ch < CAP_LANES; ch++) {
        if (!(ch_bitmap & (1U << ch))) {
            continue;
        }

        if (bk_aud_adc_start((aud_adc_chl_t)ch) != BK_OK) {
            uint32_t stop_ch;

            for (stop_ch = 0U; stop_ch < ch; stop_ch++) {
                if (ch_bitmap & (1U << stop_ch)) {
                    (void)bk_aud_adc_stop((aud_adc_chl_t)stop_ch);
                }
            }
            return BK_FAIL;
        }
    }

    return BK_OK;
}

/**
 * @brief Re-apply rate / lane mode / bits / gains / digmic_en
 * @note bk_aud_hardware_reset() (also reached through DAC init) clears SYS_CFG
 *       and the lane config, so everything is pushed again before every start.
 */
STATIC OPERATE_RET __cap_hw_reapply(cap_ctx_t *c)
{
    bk_err_t ret;
    uint32_t ch;

    ret = bk_aud_adc_set_samp_rate(c->sample_rate);
    if (ret != BK_OK) {
        TKL_AUD_CAP_LOG("cap: reapply samp_rate fail, ret=%d rate=%u\n",
                        ret, c->sample_rate);
        return OPRT_COM_ERROR;
    }

    for (ch = 0U; ch < CAP_LANES; ch++) {
        ret = bk_aud_adc_set_mic_mode((aud_adc_chl_t)ch, AUD_ADC_MODE_DIFFEN);
        if (ret != BK_OK) {
            TKL_AUD_CAP_LOG("cap: reapply mic_mode fail, ret=%d ch=%u\n", ret, ch);
            return OPRT_COM_ERROR;
        }

        ret = bk_aud_adc_set_bits_width((aud_adc_chl_t)ch, c->sample_bits);
        if (ret != BK_OK) {
            TKL_AUD_CAP_LOG("cap: reapply bits fail, ret=%d ch=%u\n", ret, ch);
            return OPRT_COM_ERROR;
        }

        ret = bk_aud_adc_set_dig_gain_db((aud_adc_chl_t)ch, c->dig_db[ch]);
        if (ret != BK_OK) {
            TKL_AUD_CAP_LOG("cap: reapply dig gain fail, ret=%d ch=%u\n", ret, ch);
            return OPRT_COM_ERROR;
        }

        ret = bk_aud_adc_set_ana_gain_db((aud_adc_chl_t)ch, c->ana_db[ch]);
        if (ret != BK_OK) {
            TKL_AUD_CAP_LOG("cap: reapply ana gain fail, ret=%d ch=%u\n", ret, ch);
            return OPRT_COM_ERROR;
        }
    }

    return __cap_apply_mode(c);
}

STATIC OPERATE_RET __cap_hw_create(cap_ctx_t *c)
{
    bk_err_t ret;
    aud_adc_config_t adc_cfg = DEFAULT_AUD_ADC_CONFIG();
    uint32_t ch;

    adc_cfg.chl_num = CAP_LANES;
    adc_cfg.adc_chl = AUD_ADC_CHL_0;
    adc_cfg.sample_rate = c->sample_rate;
    adc_cfg.clk_src = AUD_CLK_APLL;
    adc_cfg.aec_en = 0;

    for (ch = 0U; ch < CAP_LANES; ch++) {
        adc_cfg.chl_cfg[ch].bits = c->sample_bits;
        adc_cfg.chl_cfg[ch].adc_mode = AUD_ADC_MODE_DIFFEN;
        adc_cfg.chl_cfg[ch].dig_gain = c->dig_db[ch];
        adc_cfg.chl_cfg[ch].ana_gain = c->ana_db[ch];
    }

    /*
     * Bring the audio common driver up (APB + ana defaults) before the digital
     * IP reset: after a full driver_deinit that zeroes ana_reg20~30, resetting
     * first and initialising afterwards leaves the ADC producing silence.
     */
    (void)bk_aud_driver_init();
    bk_aud_hardware_reset();

    ret = bk_aud_adc_init(&adc_cfg);
    if (ret != BK_OK) {
        TKL_AUD_CAP_LOG("cap: bk_aud_adc_init fail, ret=%d\n", ret);
        return OPRT_COM_ERROR;
    }

    if (__cap_apply_mode(c) != OPRT_OK) {
        return OPRT_COM_ERROR;
    }

    if (__cap_dma_config(c) != BK_OK) {
        return OPRT_COM_ERROR;
    }

    c->read_buf = (uint8_t *)tkl_system_psram_malloc(c->dma_frame_size);
    if (c->read_buf == NULL) {
        TKL_AUD_CAP_LOG("cap: malloc read buffer fail\n");
        return OPRT_MALLOC_FAILED;
    }

    c->out_buf = (uint8_t *)tkl_system_psram_malloc(c->dense_frame_size);
    if (c->out_buf == NULL) {
        TKL_AUD_CAP_LOG("cap: malloc output buffer fail\n");
        return OPRT_MALLOC_FAILED;
    }

    mb_flash_register_op_notify_cb(__cap_flash_op_notify_handler, NULL);
    c->flash_registered = TRUE;

    TKL_AUD_CAP_LOG("cap: hw created rate=%u bits=%u frame=%ums dma_frame=%u\n",
                    c->sample_rate, c->sample_bits, c->frame_time_ms,
                    c->dma_frame_size);
    return OPRT_OK;
}

/**
 * @brief Soft teardown: free every hardware-side resource, keep the context
 * @note Deliberately does NOT call bk_aud_adc_deinit(): that reaches
 *       bk_aud_driver_deinit(), which zeros ana_reg20~30 and kills micbias, so
 *       the next init+start would only produce all-zero DMA frames. Clearing
 *       the module flag lets the next open run bk_aud_adc_init() again.
 * @note Used by both the destroy path and the re-init path, which rebuilds the
 *       hardware inside the same context.
 */
STATIC VOID_T __cap_release(cap_ctx_t *c)
{
    if (c == NULL) {
        return;
    }

    __cap_dma_deconfig(c);

    if (bk_aud_get_module_init_sta(AUD_MODULE_ADC)) {
        __cap_stop_channels();
        bk_aud_set_module_init_sta(AUD_MODULE_ADC, false);
    }

    if (c->flash_registered) {
        mb_flash_unregister_op_notify_cb(__cap_flash_op_notify_handler);
        c->flash_registered = FALSE;
    }

    __cap_dmic_unmux_all(c);

    if (c->read_buf) {
        tkl_system_psram_free(c->read_buf);
        c->read_buf = NULL;
    }
    if (c->out_buf) {
        tkl_system_psram_free(c->out_buf);
        c->out_buf = NULL;
    }

    c->dma_started = FALSE;
    c->digmic_en = 0U;
    c->inited = FALSE;
}

/**
 * @brief Release the hardware, the context and its lock
 */
STATIC VOID_T __cap_destroy(cap_ctx_t *c)
{
    if (c == NULL) {
        return;
    }

    __cap_release(c);

    tkl_system_psram_free(c);
}

STATIC OPERATE_RET __cap_hw_start(cap_ctx_t *c)
{
    bk_err_t ret;

    c->reconfiguring = TRUE;

    if (c->dma_started) {
        if (__cap_dma_reset(c) != BK_OK) {
            TKL_AUD_CAP_LOG("cap: dma reset fail on restart\n");
            c->reconfiguring = FALSE;
            return OPRT_COM_ERROR;
        }
        c->dma_started = FALSE;
    }

    __cap_pause_dma(c);

    if (__cap_hw_reapply(c) != OPRT_OK) {
        c->reconfiguring = FALSE;
        return OPRT_COM_ERROR;
    }

    ret = bk_dma_start(c->dma_id);
    if (ret != BK_OK) {
        TKL_AUD_CAP_LOG("cap: dma start fail, ret=%d\n", ret);
        c->reconfiguring = FALSE;
        return OPRT_COM_ERROR;
    }
    c->dma_started = TRUE;

    ret = __cap_start_channels(CAP_LANE_BITMAP);
    if (ret != BK_OK) {
        TKL_AUD_CAP_LOG("cap: adc start channels fail\n");
        __cap_pause_dma(c);
        c->reconfiguring = FALSE;
        return OPRT_COM_ERROR;
    }

    bk_aud_adc_enable_used_channel(CAP_LANE_BITMAP);

    c->reconfiguring = FALSE;
    TKL_AUD_CAP_LOG("cap: started mode=0x%x\n", c->digmic_en);
    return OPRT_OK;
}

STATIC VOID_T __cap_hw_stop(cap_ctx_t *c)
{
    c->reconfiguring = TRUE;
    __cap_stop_channels();
    __cap_pause_dma(c);
    c->reconfiguring = FALSE;
    TKL_AUD_CAP_LOG("cap: stopped\n");
}

/* ---------------------------------------------------------------------------
 * Validation
 * --------------------------------------------------------------------------- */

/**
 * @brief Check that @p user may attach with the given masks
 * @return OPRT_OK if compatible; OPRT_INVALID_PARM otherwise
 * @note Must not modify any state: a rejected attach leaves the core, and every
 *       interface already attached to it, completely untouched.
 */
STATIC OPERATE_RET __cap_check_attach(cap_ctx_t *c, TKL_AUD_CAP_USER_E user,
                                      const TKL_AUD_CAP_REQ_T *req,
                                      UINT32_T lane_mask)
{
    cap_ctx_t *ctx = c;
    UINT32_T   u;

    if (req->sample_bits != 16U) {
        TKL_AUD_CAP_LOG("cap: bits %u not supported, only 16\n", req->sample_bits);
        return OPRT_INVALID_PARM;
    }

    if ((req->sample_rate == 0U) || (req->frame_time_ms == 0U)) {
        TKL_AUD_CAP_LOG("cap: invalid rate/frame_time\n");
        return OPRT_INVALID_PARM;
    }

    if (ctx->inited) {
        if ((ctx->sample_rate != req->sample_rate) ||
            (ctx->sample_bits != req->sample_bits) ||
            (ctx->frame_time_ms != req->frame_time_ms)) {
            TKL_AUD_CAP_LOG("cap: param conflict rate %u/%u frame %u/%u\n",
                            ctx->sample_rate, req->sample_rate,
                            ctx->frame_time_ms, req->frame_time_ms);
            return OPRT_INVALID_PARM;
        }
    }

    for (u = 0U; u < (UINT32_T)TKL_AUD_CAP_USER_MAX; u++) {
        cap_user_t *other = &ctx->users[u];

        if ((u == (UINT32_T)user) || (other->attached == FALSE)) {
            continue;
        }

        /*
         * Lane ownership is exclusive. An ADC request that includes a lane a
         * digital port owns (or vice versa) is rejected here; the old
         * "LR/MIC3/RAW3 cannot share with DMIC" rule is just this test, since
         * those modes are exactly the bitmaps that overlap a DMIC port.
         */
        if (other->lane_mask & lane_mask) {
            TKL_AUD_CAP_LOG("cap: lane conflict user %u mask 0x%x vs user %d mask 0x%x\n",
                            (unsigned)u, other->lane_mask, (int)user, lane_mask);
            return OPRT_INVALID_PARM;
        }

        /* The two digital ports must land on different physical pads. */
        if (TKL_AUD_CAP_USER_IS_DMIC(user) &&
            TKL_AUD_CAP_USER_IS_DMIC((TKL_AUD_CAP_USER_E)u)) {
            if ((other->dmic_clk_gpio == req->dmic_clk_gpio) ||
                (other->dmic_data_gpio == req->dmic_data_gpio)) {
                TKL_AUD_CAP_LOG("cap: dmic pad conflict user %u clk=%u dat=%u vs clk=%u dat=%u\n",
                                (unsigned)u, other->dmic_clk_gpio, other->dmic_data_gpio,
                                req->dmic_clk_gpio, req->dmic_data_gpio);
                return OPRT_INVALID_PARM;
            }
        }
    }

    return OPRT_OK;
}

/* ---------------------------------------------------------------------------
 * Public core API
 * --------------------------------------------------------------------------- */

OPERATE_RET __t9_aud_cap_open(TKL_AUD_CAP_USER_E user, const TKL_AUD_CAP_REQ_T *req)
{
    cap_ctx_t *c;
    UINT32_T lane_mask = 0;
    UINT32_T gain_mask = 0;
    UINT32_T digmic_en = 0;
    OPERATE_RET ret;

    if ((req == NULL) || (user >= TKL_AUD_CAP_USER_MAX) || (req->upper_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (__cap_lock_init() != OPRT_OK) {
        return OPRT_COM_ERROR;
    }
    (void)tkl_mutex_lock(gl_cap_lock);

    c = gl_cap;

    if ((c != NULL) && (c->users[user].attached == TRUE)) {
        TKL_AUD_CAP_LOG("cap: user %d already open\n", (int)user);
        ret = OPRT_COM_ERROR;
        goto out;
    }

    ret = __cap_masks_for(user, req, &lane_mask, &gain_mask, &digmic_en);
    if (ret != OPRT_OK) {
        TKL_AUD_CAP_LOG("cap: user %d bad channel/port\n", (int)user);
        goto out;
    }

    if (c != NULL) {
        ret = __cap_check_attach(c, user, req, lane_mask);
        if (ret != OPRT_OK) {
            goto out;
        }
    } else {
        /* First opener: validate what this module itself needs. */
        if ((req->sample_bits != 16U) || (req->sample_rate == 0U) ||
            (req->frame_time_ms == 0U)) {
            TKL_AUD_CAP_LOG("cap: invalid parameters\n");
            ret = OPRT_INVALID_PARM;
            goto out;
        }
    }

    if (c == NULL) {
        uint32_t ch;

        c = (cap_ctx_t *)tkl_system_psram_malloc(sizeof(cap_ctx_t));
        if (c == NULL) {
            TKL_AUD_CAP_LOG("cap: malloc context fail\n");
            ret = OPRT_MALLOC_FAILED;
            goto out;
        }
        os_memset(c, 0, sizeof(cap_ctx_t));

        c->sample_rate = req->sample_rate;
        c->sample_bits = req->sample_bits;
        c->frame_time_ms = req->frame_time_ms;
        for (ch = 0U; ch < CAP_LANES; ch++) {
            c->dig_db[ch] = CAP_DEFAULT_DIG_DB;
            c->ana_db[ch] = CAP_DEFAULT_ANA_DB;
        }

        c->dense_frame_size = CAP_FRAME_SIZE(c->sample_rate, c->sample_bits,
                                             c->frame_time_ms, CAP_LANES);
        c->dma_frame_size = c->dense_frame_size * (CAP_SLOT_BYTES / CAP_SAMPLE_BYTES);

        c->users[user].attached = TRUE;
        c->users[user].cb = req->upper_cb;
        c->users[user].args = req->args;
        c->users[user].lane_mask = lane_mask;
        c->users[user].gain_lane_mask = gain_mask;
        c->users[user].dmic_port = req->dmic_port;
        c->users[user].dmic_clk_gpio = req->dmic_clk_gpio;
        c->users[user].dmic_data_gpio = req->dmic_data_gpio;
        c->users[user].dmic_muxed = FALSE;
        c->attached_mask = CAP_USER_BIT(user);

        c->digmic_en = digmic_en;

        if (__cap_hw_create(c) != OPRT_OK) {
            TKL_AUD_CAP_LOG("cap: hardware create fail\n");
            c->inited = FALSE;
            __cap_release(c);
            tkl_system_psram_free(c);
            ret = OPRT_COM_ERROR;
            goto out;
        }
        c->inited = TRUE;
        gl_cap = c;

        if (TKL_AUD_CAP_USER_IS_DMIC(user)) {
            __cap_dmic_mux(c, user);
        }

        TKL_AUD_CAP_LOG("cap: created by user %d lanes=0x%x\n",
                        (int)user, lane_mask);
        ret = OPRT_OK;
        goto out;
    }

    /* Attach to the running core. */
    c->users[user].attached = TRUE;
    c->users[user].cb = req->upper_cb;
    c->users[user].args = req->args;
    c->users[user].lane_mask = lane_mask;
    c->users[user].gain_lane_mask = gain_mask;
    c->users[user].dmic_port = req->dmic_port;
    c->users[user].dmic_clk_gpio = req->dmic_clk_gpio;
    c->users[user].dmic_data_gpio = req->dmic_data_gpio;
    c->users[user].dmic_muxed = FALSE;
    c->attached_mask |= CAP_USER_BIT(user);

    if (TKL_AUD_CAP_USER_IS_DMIC(user)) {
        __cap_dmic_mux(c, user);
    }

    ret = __cap_sync_mode(c);
    if (ret != OPRT_OK) {
        /* Undo the attach: a rejected open must leave the core untouched. */
        c->users[user].attached = FALSE;
        c->users[user].cb = NULL;
        c->users[user].args = NULL;
        c->users[user].lane_mask = 0U;
        c->users[user].gain_lane_mask = 0U;
        c->attached_mask &= ~CAP_USER_BIT(user);
        if (TKL_AUD_CAP_USER_IS_DMIC(user)) {
            __cap_dmic_unmux(c, user);
        }
        (void)__cap_sync_mode(c);
        goto out;
    }

    TKL_AUD_CAP_LOG("cap: user %d attached lanes=0x%x\n",
                    (int)user, lane_mask);

out:
    (void)tkl_mutex_unlock(gl_cap_lock);
    return ret;
}

OPERATE_RET __t9_aud_cap_close(TKL_AUD_CAP_USER_E user)
{
    cap_ctx_t *c;
    cap_ctx_t *dying = NULL;

    if (user >= TKL_AUD_CAP_USER_MAX) {
        return OPRT_INVALID_PARM;
    }

    if (__cap_lock_init() != OPRT_OK) {
        return OPRT_COM_ERROR;
    }
    (void)tkl_mutex_lock(gl_cap_lock);

    c = gl_cap;
    if ((c == NULL) || (c->users[user].attached == FALSE)) {
        (void)tkl_mutex_unlock(gl_cap_lock);
        return OPRT_OK;
    }

    /* Reference-count the hardware: a close while started stops just this user. */
    c->users[user].started = FALSE;
    c->started_mask &= ~CAP_USER_BIT(user);

    c->users[user].attached = FALSE;
    c->users[user].cb = NULL;
    c->users[user].args = NULL;
    c->attached_mask &= ~CAP_USER_BIT(user);

    if (TKL_AUD_CAP_USER_IS_DMIC(user)) {
        __cap_dmic_unmux(c, user);
    }

    if (c->attached_mask == 0U) {
        if (c->started_mask == 0U && c->dma_started) {
            __cap_stop_channels();
            __cap_pause_dma(c);
        }
        /* Publish the destruction before dropping the lock; the global lock
         * makes the context unreachable as soon as gl_cap is cleared. */
        gl_cap = NULL;
        dying = c;
    } else {
        if (c->started_mask == 0U) {
            __cap_stop_channels();
            __cap_pause_dma(c);
        }
        (void)__cap_sync_mode(c);
    }

    (void)tkl_mutex_unlock(gl_cap_lock);

    if (dying != NULL) {
        TKL_AUD_CAP_LOG("cap: last user %d closed, destroying\n", (int)user);
        __cap_destroy(dying);
    } else {
        TKL_AUD_CAP_LOG("cap: user %d closed\n", (int)user);
    }

    return OPRT_OK;
}

/**
 * @brief Re-open an attached user with a new request (public re-init)
 * @note See tkl_aud_capture.h for the exact semantics.
 */
OPERATE_RET __t9_aud_cap_reopen(TKL_AUD_CAP_USER_E user, const TKL_AUD_CAP_REQ_T *req)
{
    cap_ctx_t *c;
    UINT32_T lane_mask = 0;
    UINT32_T gain_mask = 0;
    UINT32_T digmic_en = 0;
    BOOL_T was_started;
    uint32_t ch;
    OPERATE_RET ret;

    if ((req == NULL) || (user >= TKL_AUD_CAP_USER_MAX) || (req->upper_cb == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (__cap_lock_init() != OPRT_OK) {
        return OPRT_COM_ERROR;
    }
    (void)tkl_mutex_lock(gl_cap_lock);

    c = gl_cap;

    /* Not open yet: init behaves like open. */
    if ((c == NULL) || (c->users[user].attached == FALSE)) {
        (void)tkl_mutex_unlock(gl_cap_lock);
        return __t9_aud_cap_open(user, req);
    }

    ret = __cap_masks_for(user, req, &lane_mask, &gain_mask, &digmic_en);
    if (ret != OPRT_OK) {
        TKL_AUD_CAP_LOG("cap: reopen user %d bad channel/port\n", (int)user);
        goto out;
    }

    if ((req->sample_bits != 16U) || (req->sample_rate == 0U) ||
        (req->frame_time_ms == 0U)) {
        ret = OPRT_INVALID_PARM;
        goto out;
    }

    if ((c->sample_rate == req->sample_rate) &&
        (c->sample_bits == req->sample_bits) &&
        (c->frame_time_ms == req->frame_time_ms) &&
        (c->users[user].lane_mask == lane_mask) &&
        (c->users[user].dmic_port == req->dmic_port) &&
        (c->users[user].dmic_clk_gpio == req->dmic_clk_gpio) &&
        (c->users[user].dmic_data_gpio == req->dmic_data_gpio)) {
        /* Same geometry: the hardware does not need to know about a new
         * callback, args or gain ownership. */
        c->users[user].cb = req->upper_cb;
        c->users[user].args = req->args;
        c->users[user].gain_lane_mask = gain_mask;
        ret = OPRT_OK;
        goto out;
    }

    /*
     * A real reconfiguration rebuilds the one shared DMA geometry. That is only
     * possible while no other interface depends on it.
     */
    if (c->attached_mask != CAP_USER_BIT(user)) {
        TKL_AUD_CAP_LOG("cap: reopen user %d rejected, shared with mask 0x%x\n",
                        (int)user, c->attached_mask);
        ret = OPRT_COM_ERROR;
        goto out;
    }

    was_started = c->users[user].started;

    if (was_started == TRUE) {
        __cap_hw_stop(c);
    }
    __cap_release(c);

    c->sample_rate = req->sample_rate;
    c->sample_bits = req->sample_bits;
    c->frame_time_ms = req->frame_time_ms;
    for (ch = 0U; ch < CAP_LANES; ch++) {
        c->dig_db[ch] = CAP_DEFAULT_DIG_DB;
        c->ana_db[ch] = CAP_DEFAULT_ANA_DB;
    }

    c->dense_frame_size = CAP_FRAME_SIZE(c->sample_rate, c->sample_bits,
                                         c->frame_time_ms, CAP_LANES);
    c->dma_frame_size = c->dense_frame_size * (CAP_SLOT_BYTES / CAP_SAMPLE_BYTES);

    c->users[user].cb = req->upper_cb;
    c->users[user].args = req->args;
    c->users[user].lane_mask = lane_mask;
    c->users[user].gain_lane_mask = gain_mask;
    c->users[user].dmic_port = req->dmic_port;
    c->users[user].dmic_clk_gpio = req->dmic_clk_gpio;
    c->users[user].dmic_data_gpio = req->dmic_data_gpio;
    c->users[user].dmic_muxed = FALSE;
    c->users[user].started = FALSE;
    c->attached_mask = CAP_USER_BIT(user);
    c->started_mask = 0U;
    c->digmic_en = digmic_en;

    if (__cap_hw_create(c) != OPRT_OK) {
        TKL_AUD_CAP_LOG("cap: reopen user %d hardware create fail\n", (int)user);
        c->users[user].attached = FALSE;
        c->attached_mask = 0U;
        gl_cap = NULL;
        (void)tkl_mutex_unlock(gl_cap_lock);
        __cap_destroy(c);
        return OPRT_COM_ERROR;
    }
    c->inited = TRUE;

    if (TKL_AUD_CAP_USER_IS_DMIC(user)) {
        __cap_dmic_mux(c, user);
    }

    ret = OPRT_OK;
    if (was_started == TRUE) {
        ret = __cap_hw_start(c);
        if (ret == OPRT_OK) {
            c->users[user].started = TRUE;
            c->started_mask = CAP_USER_BIT(user);
        }
    }

    TKL_AUD_CAP_LOG("cap: user %d reopened rate=%u frame=%ums lanes=0x%x started=%d\n",
                    (int)user, c->sample_rate, c->frame_time_ms,
                    lane_mask, (int)was_started);

out:
    (void)tkl_mutex_unlock(gl_cap_lock);
    return ret;
}

OPERATE_RET __t9_aud_cap_start(TKL_AUD_CAP_USER_E user)
{
    cap_ctx_t *c;
    OPERATE_RET ret = OPRT_OK;

    if (user >= TKL_AUD_CAP_USER_MAX) {
        return OPRT_INVALID_PARM;
    }

    if (__cap_lock_init() != OPRT_OK) {
        return OPRT_COM_ERROR;
    }
    (void)tkl_mutex_lock(gl_cap_lock);

    c = gl_cap;
    if ((c == NULL) || (c->users[user].attached == FALSE)) {
        (void)tkl_mutex_unlock(gl_cap_lock);
        return OPRT_COM_ERROR;
    }

    if (c->users[user].started) {
        (void)tkl_mutex_unlock(gl_cap_lock);
        return OPRT_OK;
    }

    if (c->started_mask == 0U) {
        ret = __cap_hw_start(c);
        if (ret != OPRT_OK) {
            (void)tkl_mutex_unlock(gl_cap_lock);
            return ret;
        }
    } else {
        /*
         * Another interface already owns the stream. Only the mode can change:
         * the DMA geometry is fixed, so a lane becoming digital is a register
         * write, not a reconfiguration.
         */
        ret = __cap_sync_mode(c);
        if (ret != OPRT_OK) {
            (void)tkl_mutex_unlock(gl_cap_lock);
            return ret;
        }
    }

    c->users[user].started = TRUE;
    c->started_mask |= CAP_USER_BIT(user);

    (void)tkl_mutex_unlock(gl_cap_lock);

    TKL_AUD_CAP_LOG("cap: user %d started (mask=0x%x)\n", (int)user, c->started_mask);
    return OPRT_OK;
}

OPERATE_RET __t9_aud_cap_stop(TKL_AUD_CAP_USER_E user)
{
    cap_ctx_t *c;

    if (user >= TKL_AUD_CAP_USER_MAX) {
        return OPRT_INVALID_PARM;
    }

    if (__cap_lock_init() != OPRT_OK) {
        return OPRT_COM_ERROR;
    }
    (void)tkl_mutex_lock(gl_cap_lock);

    c = gl_cap;
    if ((c == NULL) || (c->users[user].started == FALSE)) {
        (void)tkl_mutex_unlock(gl_cap_lock);
        return OPRT_OK;
    }

    c->users[user].started = FALSE;
    c->started_mask &= ~CAP_USER_BIT(user);

    if (c->started_mask == 0U) {
        __cap_hw_stop(c);
    }

    (void)tkl_mutex_unlock(gl_cap_lock);

    TKL_AUD_CAP_LOG("cap: user %d stopped (mask=0x%x)\n", (int)user, c->started_mask);
    return OPRT_OK;
}

OPERATE_RET __t9_aud_cap_set_gain(TKL_AUD_CAP_USER_E user, UINT32_T lane,
                                 BOOL_T set_dig, FLOAT_T dig_db,
                                 BOOL_T set_ana, INT32_T ana_db)
{
    cap_ctx_t *c;
    bk_err_t ret;

    if ((user >= TKL_AUD_CAP_USER_MAX) || (lane >= CAP_LANES)) {
        return OPRT_INVALID_PARM;
    }

    if ((set_dig == FALSE) && (set_ana == FALSE)) {
        return OPRT_INVALID_PARM;
    }

    if (__cap_lock_init() != OPRT_OK) {
        return OPRT_COM_ERROR;
    }
    (void)tkl_mutex_lock(gl_cap_lock);

    c = gl_cap;
    if ((c == NULL) || (c->users[user].attached == FALSE)) {
        (void)tkl_mutex_unlock(gl_cap_lock);
        return OPRT_COM_ERROR;
    }

    if (!(c->users[user].gain_lane_mask & (1U << lane))) {
        TKL_AUD_CAP_LOG("cap: user %d does not own lane %u (mask=0x%x)\n",
                        (int)user, lane, c->users[user].gain_lane_mask);
        (void)tkl_mutex_unlock(gl_cap_lock);
        return OPRT_INVALID_PARM;
    }

    if (set_dig == TRUE) {
        if (dig_db > BK_AUD_ADC_DIG_GAIN_DB_MAX) {
            dig_db = BK_AUD_ADC_DIG_GAIN_DB_MAX;
        }
        ret = bk_aud_adc_set_dig_gain_db((aud_adc_chl_t)lane, dig_db);
        if (ret != BK_OK) {
            TKL_AUD_CAP_LOG("cap: set dig gain fail, ch=%u ret=%d\n", lane, ret);
            (void)tkl_mutex_unlock(gl_cap_lock);
            return OPRT_COM_ERROR;
        }
        c->dig_db[lane] = dig_db;
    }

    if (set_ana == TRUE) {
        if (ana_db > BK_AUD_ADC_ANA_GAIN_DB_MAX) {
            ana_db = BK_AUD_ADC_ANA_GAIN_DB_MAX;
        }
        if (ana_db < 0) {
            ana_db = 0;
        }
        ret = bk_aud_adc_set_ana_gain_db((aud_adc_chl_t)lane, ana_db);
        if (ret != BK_OK) {
            TKL_AUD_CAP_LOG("cap: set ana gain fail, ch=%u ret=%d\n", lane, ret);
            (void)tkl_mutex_unlock(gl_cap_lock);
            return OPRT_COM_ERROR;
        }
        c->ana_db[lane] = ana_db;
    }

    (void)tkl_mutex_unlock(gl_cap_lock);

    TKL_AUD_CAP_LOG("cap: gain ch=%u dig=%s%.1f ana=%s%d (user %d)\n",
                    lane,
                    (set_dig == TRUE) ? "" : "keep/",
                    (set_dig == TRUE) ? dig_db : c->dig_db[lane],
                    (set_ana == TRUE) ? "" : "keep/",
                    (set_ana == TRUE) ? ana_db : c->ana_db[lane],
                    (int)user);
    return OPRT_OK;
}

BOOL_T __t9_aud_cap_user_attached(TKL_AUD_CAP_USER_E user)
{
    BOOL_T attached = FALSE;

    if ((user >= TKL_AUD_CAP_USER_MAX) || (__cap_lock_init() != OPRT_OK)) {
        return FALSE;
    }

    (void)tkl_mutex_lock(gl_cap_lock);
    if (gl_cap != NULL) {
        attached = gl_cap->users[user].attached;
    }
    (void)tkl_mutex_unlock(gl_cap_lock);

    return attached;
}

#endif /* CONFIG_AUD_DRIVER_V2 */
