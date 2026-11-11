/**
 * @file tkl_audio.c
 * @brief T9 (BK7259) media audio TKL -- tkl_ai_* / tkl_ao_*.
 *
 * @copyright Copyright (c) 2025-2026 Tuya Inc. All Rights Reserved.
 *
 * This is the "media" audio interface consumed by src/peripherals/audio_codecs/.
 * tdd_audio.c uses it like this:
 *
 *   open : tkl_ai_init(&cfg, 0) -> tkl_ai_start(0, 0) [-> tkl_ao_set_vol()]
 *   play : tkl_ao_put_frame(0, 0, NULL, &frame)
 *   stop : tkl_ao_clear_buffer(...)
 *   close: tkl_ai_stop(0, 0) -> tkl_ai_uninit()
 *
 * tkl_ao_init()/tkl_ao_start() are only called by TDD on platforms that open
 * capture and playback separately, so on T9 both the ADC and the DAC are
 * brought up by tkl_ai_init() and torn down by tkl_ai_uninit(). The AO entry
 * points still work on their own for any caller that does use them.
 *
 * It is implemented on top of the split TKL audio drivers (tkl_aud_adc_* and
 * tkl_aud_dac_*), NOT on top of the Beken voice service:
 *
 *   - capture grabs all three ADC lanes (dual mic + speaker loopback) and runs
 *     them through tkl_audio_frontend.c, which applies the tal_audio_subsys
 *     DAS BF + FDAF AEC+NS pipeline (two mics steered into one beam, AEC
 *     against the loopback) and turns the 8 ms algorithm frames into the
 *     20 ms mono blocks this layer delivers. Without the algorithm library
 *     the front end passes mic0 through unchanged, so a build without
 *     AEC still works;
 *   - capture is the ADC's single FIFO. On BK7259 the analog ADC and the DMIC
 *     share one DMA, so tkl_aud_capture.c arbitrates the lanes and this file
 *     stays out of the way.
 *
 * Capture path: tkl_aud_capture.c delivers frames from the DMA ISR. The upper
 * layer's put_cb is not ISR-safe (TAL takes locks and posts events), so the ISR
 * side only copies into a single-producer/single-consumer byte ring and a
 * worker thread drains it, runs the front end, and makes one upper-layer
 * callback per 20 ms output block.
 *
 * Playback path: PCM is copied straight into the DAC driver's DMA ring. That
 * ring holds only six frames (a few tens of milliseconds), while decoded TTS
 * frames are much larger, so a frame is fed in DAC-frame-sized chunks and a
 * full ring is retried for a bounded time -- a stalled DAC then costs one
 * dropped frame instead of wedging the playback thread forever.
 */

#include "tuya_cloud_types.h"
#include "tuya_error_code.h"

#include "tkl_audio.h"
#include "tkl_aud_adc.h"
#include "tkl_aud_dac.h"
#include "tkl_audio_frontend.h"
#include "tkl_gpio.h"
#include "tkl_memory.h"
#include "tkl_system.h"

#include <os/os.h>
#include <string.h>

extern void bk_printf(const char *fmt, ...);

#define TKL_AUDIO_LOG(fmt, ...) bk_printf("[tkl_audio] " fmt, ##__VA_ARGS__)

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define TKL_AUDIO_FRAME_MS        (8U)
#define TKL_AUDIO_CAP_RING_FRAMES (16U) /* ~128 ms of overdue mic audio */
#define TKL_AUDIO_TASK_STACK      (4096U)
#define TKL_AUDIO_TASK_PRIO       (BEKEN_DEFAULT_WORKER_PRIORITY)

#define TKL_AUDIO_ADC_PORT (TUYA_AUDIO_ADC_PORT_0)
#define TKL_AUDIO_DAC_PORT (TUYA_AUDIO_DAC_PORT_0)

/*
 * All three ADC lanes are captured on every frame:
 *   lane 0/1 = the two analog mics, lane 2 = the speaker loopback used as the
 *   AEC reference. The front end decides which lane is which (the lane map in
 *   tkl_audio_frontend.c) and feeds a
 *   2 mic + 1 ref frame to the AEC interface; this file only asks for the bitmap.
 */
#define TKL_AUDIO_CAP_CHAN (TUYA_AUDIO_ADC_CHANNEL_0 | TUYA_AUDIO_ADC_CHANNEL_1 | TUYA_AUDIO_ADC_CHANNEL_2)

/*
 * A full DAC ring means the DMA is not consuming. The ring is six frames
 * (~48 ms at 8 ms per frame), so anything past ~200 ms of waiting is a stall,
 * not backpressure.
 */
#define TKL_AUDIO_SPK_WRITE_RETRY_MS   (10U)
#define TKL_AUDIO_SPK_WRITE_TIMEOUT_MS (200U)

#define TKL_AUDIO_DEFAULT_SPK_VOLUME (80)
#define TKL_AUDIO_MAX_VOLUME         (100)

/*
 * Volume mapping. The drivers take gain in dB, with a documented usable range:
 * ADC digital gain tops out at +18 dB (BK_AUD_DAC/ADC_*_DB_MAX in
 * aud_adc_types.h) and the DAC at +12 dB, but a small speaker distorts long
 * before that, so playback is mapped to [-36 dB, 0 dB] and mute.
 */
#define TKL_AUDIO_MIC_DB_MIN  (-12)
#define TKL_AUDIO_MIC_DB_MAX  (18)
#define TKL_AUDIO_SPK_DB_MIN  (-36)
#define TKL_AUDIO_SPK_DB_MUTE (-100)

/* ---------------------------------------------------------------------------
 * Type definitions
 * --------------------------------------------------------------------------- */
typedef struct {
    TKL_AUDIO_CONFIG_T cfg;

    uint32_t sample_rate;
    uint32_t sample_bits;
    uint32_t sample_ch;
    uint32_t cap_frame_size; /* one 3-lane capture frame from the ADC, bytes */
    uint32_t frame_size;     /* one delivered mono block (front-end output), bytes */

    /* capture ring: single producer (ADC ISR) / single consumer (task) */
    uint8_t          *ring;
    uint32_t          ring_size; /* power of two, mask arithmetic */
    volatile uint32_t ring_wr;
    volatile uint32_t ring_rd;

    beken_semaphore_t sem;      /* data available / wake the task */
    beken_semaphore_t stop_sem; /* task has exited */
    beken_thread_t    task;
    BOOL_T            task_created;
    volatile BOOL_T   task_run;

    BOOL_T ai_inited;
    BOOL_T ai_started;
    BOOL_T ao_inited;
    BOOL_T ao_started;

    uint8_t spk_volume;
    int32_t spk_gpio;     /* < 0: no amplifier pin */
    int32_t spk_polarity; /* level that MUTES the amplifier */
} TKL_AUDIO_CTX_T;

/* ---------------------------------------------------------------------------
 * File scope variables
 * --------------------------------------------------------------------------- */
static TKL_AUDIO_CTX_T *sg_audio_ctx = NULL;

/* ---------------------------------------------------------------------------
 * Function declarations
 * --------------------------------------------------------------------------- */
static void __tkl_audio_task(beken_thread_arg_t arg);

/* ---------------------------------------------------------------------------
 * Capture ring
 * --------------------------------------------------------------------------- */
static uint32_t __ring_write(TKL_AUDIO_CTX_T *ctx, const uint8_t *data, uint32_t len)
{
    uint32_t used;
    uint32_t free_len;
    uint32_t pos;
    uint32_t first;

    used     = ctx->ring_wr - ctx->ring_rd;
    free_len = ctx->ring_size - used;
    if (len > free_len) {
        len = free_len;
    }
    if (len == 0U) {
        return 0;
    }

    pos   = ctx->ring_wr & (ctx->ring_size - 1U);
    first = ctx->ring_size - pos;
    if (first > len) {
        first = len;
    }
    memcpy(ctx->ring + pos, data, first);
    if (len > first) {
        memcpy(ctx->ring, data + first, len - first);
    }
    ctx->ring_wr += len;

    return len;
}

static uint32_t __ring_read(TKL_AUDIO_CTX_T *ctx, uint8_t *data, uint32_t len)
{
    uint32_t used;
    uint32_t pos;
    uint32_t first;

    used = ctx->ring_wr - ctx->ring_rd;
    if (len > used) {
        len = used;
    }
    if (len == 0U) {
        return 0;
    }

    pos   = ctx->ring_rd & (ctx->ring_size - 1U);
    first = ctx->ring_size - pos;
    if (first > len) {
        first = len;
    }
    memcpy(data, ctx->ring + pos, first);
    if (len > first) {
        memcpy(data + first, ctx->ring, len - first);
    }
    ctx->ring_rd += len;

    return len;
}

static uint32_t __ring_size_for_frames(uint32_t frame_size, uint32_t frames)
{
    uint32_t size = frame_size * frames;
    uint32_t pow2 = 1U;

    while (pow2 < size) {
        pow2 <<= 1U;
    }

    return pow2;
}

/* ---------------------------------------------------------------------------
 * Capture delivery
 * --------------------------------------------------------------------------- */
/**
 * @brief ADC frame callback, runs in the DMA ISR
 *
 * @note Only copies into the ring and wakes the worker: put_cb belongs to the
 *       upper layer and must not run in interrupt context.
 */
static VOID_T __tkl_audio_adc_cb(TUYA_AUDIO_INPUT_BUFFER_T *buffer)
{
    TKL_AUDIO_CTX_T *ctx;

    if (buffer == NULL) {
        return;
    }

    ctx = (TKL_AUDIO_CTX_T *)buffer->args;
    if ((ctx == NULL) || (buffer->data == NULL) || (buffer->data_len == 0U)) {
        return;
    }
    if ((!ctx->task_run) || (ctx->cfg.put_cb == NULL)) {
        return;
    }

    /* Overflow costs the newest frame; blocking in the ISR would cost the
     * whole audio path. */
    if (__ring_write(ctx, buffer->data, buffer->data_len) == buffer->data_len) {
        (void)rtos_set_semaphore(&ctx->sem);
    }
}

static void __tkl_audio_deliver(TKL_AUDIO_CTX_T *ctx, uint8_t *buf, uint32_t len)
{
    TKL_AUDIO_FRAME_INFO_T frame;

    if (ctx->cfg.put_cb == NULL) {
        return;
    }

    memset(&frame, 0, sizeof(frame));
    frame.type      = TKL_AUDIO_FRAME;
    frame.codectype = TKL_CODEC_AUDIO_PCM;
    frame.sample    = (TKL_AUDIO_SAMPLE_E)ctx->sample_rate;
    frame.datebits  = (TKL_AUDIO_DATABITS_E)ctx->sample_bits;
    frame.channel   = (TKL_AUDIO_CHANNEL_E)ctx->sample_ch;
    frame.pbuf      = (CHAR_T *)buf;
    frame.buf_size  = len;
    frame.used_size = len;

    (void)ctx->cfg.put_cb(&frame);
}

static void __tkl_audio_task(beken_thread_arg_t arg)
{
    TKL_AUDIO_CTX_T *ctx = (TKL_AUDIO_CTX_T *)arg;
    uint8_t         *cap;
    uint8_t         *out;

    cap = (uint8_t *)tkl_system_malloc(ctx->cap_frame_size);
    if (cap == NULL) {
        TKL_AUDIO_LOG("mic capture buffer malloc %u failed\r\n", (unsigned)ctx->cap_frame_size);
    }
    out = (uint8_t *)tkl_system_malloc(ctx->frame_size);
    if (out == NULL) {
        TKL_AUDIO_LOG("mic output buffer malloc %u failed\r\n", (unsigned)ctx->frame_size);
    }

    while (ctx->task_run) {
        (void)rtos_get_semaphore(&ctx->sem, TKL_AUDIO_FRAME_MS);
        if (!ctx->task_run) {
            break;
        }
        if ((cap == NULL) || (out == NULL)) {
            continue;
        }
        while (ctx->task_run && (__ring_read(ctx, cap, ctx->cap_frame_size) == ctx->cap_frame_size)) {
            int len = tkl_af_process(cap, ctx->cap_frame_size, out, ctx->frame_size);

            if (len > 0) {
                __tkl_audio_deliver(ctx, out, (uint32_t)len);
            }
        }
    }

    if (cap != NULL) {
        tkl_system_free(cap);
    }
    if (out != NULL) {
        tkl_system_free(out);
    }

    (void)rtos_set_semaphore(&ctx->stop_sem);
    rtos_delete_thread(NULL);
}

static void __tkl_audio_task_stop(TKL_AUDIO_CTX_T *ctx)
{
    if (!ctx->task_created) {
        return;
    }

    ctx->task_run = FALSE;
    (void)rtos_set_semaphore(&ctx->sem);
    (void)rtos_get_semaphore(&ctx->stop_sem, BEKEN_WAIT_FOREVER);
    ctx->task_created = FALSE;

    (void)rtos_deinit_semaphore(&ctx->stop_sem);
    (void)rtos_deinit_semaphore(&ctx->sem);
}

/* ---------------------------------------------------------------------------
 * Speaker amplifier
 * --------------------------------------------------------------------------- */
/**
 * @brief Translate the configured polarity into the pin level
 *
 * @param[in] on TRUE for "amplifier enabled"
 * @return the level to drive
 * @note spk_gpio_polarity is the level that keeps the amplifier MUTED, which
 *       is how the T5AI adapter reads the same field and how the boards fill
 *       it in (TUYA_GPIO_LEVEL_LOW -> active high).
 */
static TUYA_GPIO_LEVEL_E __tkl_audio_pa_level(TKL_AUDIO_CTX_T *ctx, BOOL_T on)
{
    TUYA_GPIO_LEVEL_E mute =
        (ctx->spk_polarity == (int32_t)TUYA_GPIO_LEVEL_HIGH) ? TUYA_GPIO_LEVEL_HIGH : TUYA_GPIO_LEVEL_LOW;

    if (!on) {
        return mute;
    }

    return (mute == TUYA_GPIO_LEVEL_LOW) ? TUYA_GPIO_LEVEL_HIGH : TUYA_GPIO_LEVEL_LOW;
}

static void __tkl_audio_pa_init(TKL_AUDIO_CTX_T *ctx)
{
    TUYA_GPIO_BASE_CFG_T gpio_cfg;

    if (ctx->spk_gpio < 0) {
        return;
    }

    memset(&gpio_cfg, 0, sizeof(gpio_cfg));
    gpio_cfg.direct    = TUYA_GPIO_OUTPUT;
    gpio_cfg.mode      = TUYA_GPIO_PUSH_PULL;
    gpio_cfg.level     = __tkl_audio_pa_level(ctx, FALSE);
    gpio_cfg.drive_cap = TUYA_GPIO_DRIVE_CAP_0;

    if (tkl_gpio_init((TUYA_GPIO_NUM_E)ctx->spk_gpio, &gpio_cfg) != OPRT_OK) {
        TKL_AUDIO_LOG("spk pa gpio %d init failed\r\n", (int)ctx->spk_gpio);
        ctx->spk_gpio = -1;
    }
}

static void __tkl_audio_pa_set(TKL_AUDIO_CTX_T *ctx, BOOL_T on)
{
    if (ctx->spk_gpio < 0) {
        return;
    }

    (void)tkl_gpio_write((TUYA_GPIO_NUM_E)ctx->spk_gpio, __tkl_audio_pa_level(ctx, on));
}

static void __tkl_audio_pa_deinit(TKL_AUDIO_CTX_T *ctx)
{
    if (ctx->spk_gpio < 0) {
        return;
    }

    (void)tkl_gpio_write((TUYA_GPIO_NUM_E)ctx->spk_gpio, __tkl_audio_pa_level(ctx, FALSE));
    (void)tkl_gpio_deinit((TUYA_GPIO_NUM_E)ctx->spk_gpio);
    ctx->spk_gpio = -1;
}

/* ---------------------------------------------------------------------------
 * Gain helpers
 * --------------------------------------------------------------------------- */
static int32_t __tkl_audio_spk_db(uint8_t volume)
{
    if (volume == 0U) {
        return TKL_AUDIO_SPK_DB_MUTE;
    }

    return TKL_AUDIO_SPK_DB_MIN + ((int32_t)volume * (-TKL_AUDIO_SPK_DB_MIN)) / TKL_AUDIO_MAX_VOLUME;
}

static void __tkl_audio_apply_spk_volume(TKL_AUDIO_CTX_T *ctx)
{
    INT32_T gain[1];

    if (!ctx->ao_inited) {
        return;
    }

    gain[0] = __tkl_audio_spk_db(ctx->spk_volume);
    (void)tkl_aud_dac_set_gain(TKL_AUDIO_DAC_PORT, TUYA_AUDIO_DAC_CHANNEL_0, gain, 1);
}

/* ---------------------------------------------------------------------------
 * ADC / DAC bring-up
 * --------------------------------------------------------------------------- */
static void __tkl_audio_ai_close(TKL_AUDIO_CTX_T *ctx)
{
    if (ctx->ai_started) {
        (void)tkl_aud_adc_stop(TKL_AUDIO_ADC_PORT);
        ctx->ai_started = FALSE;
    }
    if (ctx->ai_inited) {
        (void)tkl_aud_adc_deinit(TKL_AUDIO_ADC_PORT);
        ctx->ai_inited = FALSE;
    }

    __tkl_audio_task_stop(ctx);
    (void)tkl_af_deinit();

    if (ctx->ring != NULL) {
        tkl_system_free(ctx->ring);
        ctx->ring      = NULL;
        ctx->ring_size = 0U;
    }
}

static OPERATE_RET __tkl_audio_ai_open(TKL_AUDIO_CTX_T *ctx)
{
    TUYA_AUDIO_ADC_BASE_CFG_T cfg;
    uint8_t                  *ring;
    uint32_t                  ring_size;
    bk_err_t                  bk_ret;

    ring_size = __ring_size_for_frames(ctx->cap_frame_size, TKL_AUDIO_CAP_RING_FRAMES);

    ring = (uint8_t *)tkl_system_malloc(ring_size);
    if (ring == NULL) {
        TKL_AUDIO_LOG("mic ring malloc %u failed\r\n", (unsigned)ring_size);
        return OPRT_MALLOC_FAILED;
    }
    ctx->ring      = ring;
    ctx->ring_size = ring_size;
    ctx->ring_wr   = 0U;
    ctx->ring_rd   = 0U;

    /* Bring the front end up before the DMA starts, so the task never sees a
     * capture frame it cannot process. */
    if (tkl_af_init() != OPRT_OK) {
        TKL_AUDIO_LOG("audio front end init failed\r\n");
        goto __ERR;
    }

    bk_ret = rtos_init_semaphore_ex(&ctx->sem, 1, 0);
    if (bk_ret != kNoErr) {
        TKL_AUDIO_LOG("mic sem init failed %d\r\n", (int)bk_ret);
        goto __ERR;
    }
    bk_ret = rtos_init_semaphore_ex(&ctx->stop_sem, 1, 0);
    if (bk_ret != kNoErr) {
        TKL_AUDIO_LOG("mic stop sem init failed %d\r\n", (int)bk_ret);
        (void)rtos_deinit_semaphore(&ctx->sem);
        goto __ERR;
    }

    ctx->task_run = TRUE;
    bk_ret        = rtos_create_thread(&ctx->task, TKL_AUDIO_TASK_PRIO, "tkl_audio_mic", __tkl_audio_task,
                                       TKL_AUDIO_TASK_STACK, (beken_thread_arg_t)ctx);
    if (bk_ret != kNoErr) {
        TKL_AUDIO_LOG("mic task create failed %d\r\n", (int)bk_ret);
        ctx->task_run = FALSE;
        (void)rtos_deinit_semaphore(&ctx->stop_sem);
        (void)rtos_deinit_semaphore(&ctx->sem);
        goto __ERR;
    }
    ctx->task_created = TRUE;

    memset(&cfg, 0, sizeof(cfg));
    cfg.chan          = TKL_AUDIO_CAP_CHAN;
    cfg.sample_bits   = TUYA_AUDIO_SAMPLE_BITS_16;
    cfg.sample_rate   = ctx->sample_rate;
    cfg.frame_time_ms = TKL_AUDIO_FRAME_MS;
    cfg.upper_cb      = __tkl_audio_adc_cb;
    cfg.args          = ctx;

    if (tkl_aud_adc_init(TKL_AUDIO_ADC_PORT, &cfg) != OPRT_OK) {
        TKL_AUDIO_LOG("adc init failed\r\n");
        __tkl_audio_ai_close(ctx);
        return OPRT_COM_ERROR;
    }
    ctx->ai_inited = TRUE;

    return OPRT_OK;

__ERR:
    ctx->ring = NULL;
    tkl_system_free(ring);
    ctx->ring_size = 0U;
    return OPRT_COM_ERROR;
}

static OPERATE_RET __tkl_audio_ao_open(TKL_AUDIO_CTX_T *ctx)
{
    TUYA_AUDIO_DAC_BASE_CFG_T cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.dac_chan      = TUYA_AUDIO_DAC_CHANNEL_0;
    cfg.sample_bits   = TUYA_AUDIO_SAMPLE_BITS_16;
    cfg.sample_rate   = (ctx->cfg.spk_sample > 0) ? (uint32_t)ctx->cfg.spk_sample : ctx->sample_rate;
    cfg.frame_time_ms = TKL_AUDIO_FRAME_MS;
    cfg.frame_cb      = NULL;
    cfg.args          = NULL;

    if (tkl_aud_dac_init(TKL_AUDIO_DAC_PORT, &cfg) != OPRT_OK) {
        TKL_AUDIO_LOG("dac init failed\r\n");
        return OPRT_COM_ERROR;
    }
    ctx->ao_inited = TRUE;

    __tkl_audio_pa_init(ctx);

    if (tkl_aud_dac_start(TKL_AUDIO_DAC_PORT) != OPRT_OK) {
        TKL_AUDIO_LOG("dac start failed\r\n");
        __tkl_audio_pa_deinit(ctx);
        (void)tkl_aud_dac_deinit(TKL_AUDIO_DAC_PORT);
        ctx->ao_inited = FALSE;
        return OPRT_COM_ERROR;
    }
    ctx->ao_started = TRUE;

    /* The DAC driver stays muted until a gain is written. */
    __tkl_audio_apply_spk_volume(ctx);
    __tkl_audio_pa_set(ctx, TRUE);

    return OPRT_OK;
}

static void __tkl_audio_ao_close(TKL_AUDIO_CTX_T *ctx)
{
    if (ctx->ao_started) {
        (void)tkl_aud_dac_stop(TKL_AUDIO_DAC_PORT);
        ctx->ao_started = FALSE;
    }
    if (ctx->ao_inited) {
        (void)tkl_aud_dac_deinit(TKL_AUDIO_DAC_PORT);
        ctx->ao_inited = FALSE;
    }

    __tkl_audio_pa_deinit(ctx);
}

/* ---------------------------------------------------------------------------
 * TKL input API
 * --------------------------------------------------------------------------- */
/**
 * @brief ai init
 *
 * @param[in] pconfig: audio config
 * @param[in] count: count of pconfig (ignored, one card on this platform)
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 * @note Brings the DAC up as well, because TDD never calls tkl_ao_init() here.
 */
OPERATE_RET tkl_ai_init(TKL_AUDIO_CONFIG_T *pconfig, INT32_T count)
{
    TKL_AUDIO_CTX_T *ctx;
    OPERATE_RET      rt;

    (void)count;

    if (pconfig == NULL) {
        return OPRT_INVALID_PARM;
    }
    if (sg_audio_ctx != NULL) {
        return OPRT_COM_ERROR;
    }
    if ((pconfig->datebits != TKL_AUDIO_DATABITS_16) || (pconfig->channel != TKL_AUDIO_CHANNEL_MONO) ||
        (pconfig->sample == TKL_AUDIO_SAMPLE_MAX) || (pconfig->sample == 0)) {
        TKL_AUDIO_LOG("unsupported format: %uHz/%ubit/%uch\r\n", (unsigned)pconfig->sample, (unsigned)pconfig->datebits,
                      (unsigned)pconfig->channel);
        return OPRT_INVALID_PARM;
    }
    if ((uint32_t)pconfig->sample != TKL_AF_SAMPLE_RATE) {
        /* The capture lanes, the AEC frame length and the 20 ms output block
         * are all fixed at 16 kHz. */
        TKL_AUDIO_LOG("unsupported sample rate %u, only %u Hz\r\n", (unsigned)pconfig->sample,
                      (unsigned)TKL_AF_SAMPLE_RATE);
        return OPRT_INVALID_PARM;
    }

    ctx = (TKL_AUDIO_CTX_T *)tkl_system_malloc(sizeof(TKL_AUDIO_CTX_T));
    if (ctx == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    memset(ctx, 0, sizeof(TKL_AUDIO_CTX_T));

    memcpy(&ctx->cfg, pconfig, sizeof(TKL_AUDIO_CONFIG_T));
    ctx->sample_rate    = (uint32_t)pconfig->sample;
    ctx->sample_bits    = (uint32_t)pconfig->datebits;
    ctx->sample_ch      = (uint32_t)pconfig->channel;
    ctx->cap_frame_size = TKL_AF_CAPTURE_BYTES;
    ctx->frame_size     = ctx->sample_rate * (ctx->sample_bits / 8U) * ctx->sample_ch * TKL_AF_OUT_FRAME_MS / 1000U;
    if ((ctx->frame_size == 0U) || (ctx->frame_size != TKL_AF_OUT_BYTES)) {
        tkl_system_free(ctx);
        return OPRT_INVALID_PARM;
    }

    ctx->spk_gpio     = pconfig->spk_gpio;
    ctx->spk_polarity = pconfig->spk_gpio_polarity;
    ctx->spk_volume =
        (pconfig->spk_volume > 0)
            ? (uint8_t)((pconfig->spk_volume > TKL_AUDIO_MAX_VOLUME) ? TKL_AUDIO_MAX_VOLUME : pconfig->spk_volume)
            : TKL_AUDIO_DEFAULT_SPK_VOLUME;

    rt = __tkl_audio_ai_open(ctx);
    if (rt != OPRT_OK) {
        tkl_system_free(ctx);
        return rt;
    }

    rt = __tkl_audio_ao_open(ctx);
    if (rt != OPRT_OK) {
        __tkl_audio_ai_close(ctx);
        tkl_system_free(ctx);
        return rt;
    }

    sg_audio_ctx = ctx;

    if (pconfig->enable && (tkl_af_aec_enabled() != TRUE)) {
        TKL_AUDIO_LOG("aec requested but the algorithm library is not available\r\n");
    }

    TKL_AUDIO_LOG("init %uHz/%ubit/%uch, capture %u bytes -> %u bytes, aec %d\r\n", (unsigned)ctx->sample_rate,
                  (unsigned)ctx->sample_bits, (unsigned)ctx->sample_ch, (unsigned)ctx->cap_frame_size,
                  (unsigned)ctx->frame_size, (int)tkl_af_aec_enabled());

    return OPRT_OK;
}

/**
 * @brief ai start
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ai_start(INT32_T card, TKL_AI_CHN_E chn)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    (void)card;
    (void)chn;

    if ((ctx == NULL) || (!ctx->ai_inited)) {
        return OPRT_RESOURCE_NOT_READY;
    }
    if (ctx->ai_started) {
        return OPRT_OK;
    }

    if (tkl_aud_adc_start(TKL_AUDIO_ADC_PORT) != OPRT_OK) {
        TKL_AUDIO_LOG("adc start failed\r\n");
        return OPRT_COM_ERROR;
    }
    ctx->ai_started = TRUE;

    return OPRT_OK;
}

/**
 * @brief ai set mic volume
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 * @param[in] vol: mic volume,[0, 100]
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ai_set_vol(INT32_T card, TKL_AI_CHN_E chn, INT32_T vol)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;
    INT32_T          gain[1];

    (void)card;
    (void)chn;

    if ((ctx == NULL) || (!ctx->ai_inited)) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (vol < 0) {
        vol = 0;
    } else if (vol > TKL_AUDIO_MAX_VOLUME) {
        vol = TKL_AUDIO_MAX_VOLUME;
    }

    gain[0] = TKL_AUDIO_MIC_DB_MIN + ((vol * (TKL_AUDIO_MIC_DB_MAX - TKL_AUDIO_MIC_DB_MIN)) / TKL_AUDIO_MAX_VOLUME);

    /* set_gain takes one lane per call, and all three lanes are captured: the
     * talk mic needs it, and keeping mic1/ref at the same digital gain keeps
     * the AEC reference comparable to the mic signal. */
    {
        uint32_t    idx;
        OPERATE_RET first_err = OPRT_OK;

        for (idx = 0U; idx < TKL_AF_CAPTURE_LANES; idx++) {
            OPERATE_RET rt = tkl_aud_adc_set_gain(
                TKL_AUDIO_ADC_PORT, (TUYA_AUDIO_MIC_CHANNEL_E)(TUYA_AUDIO_ADC_CHANNEL_0 << idx), gain, 1U);
            if ((rt != OPRT_OK) && (first_err == OPRT_OK)) {
                first_err = rt;
            }
        }

        return first_err;
    }
}

/**
 * @brief ai get frame
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 * @param[out] pframe: audio frame, pframe->pbuf allocated by upper layer application
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 * @note Capture is delivered through put_cb; there is no pull mode.
 */
OPERATE_RET tkl_ai_get_frame(INT32_T card, TKL_AI_CHN_E chn, TKL_AUDIO_FRAME_INFO_T *pframe)
{
    (void)card;
    (void)chn;
    (void)pframe;

    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief ai set vqe param
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 * @param[in] type: vqe type
 * @param[in] pparam: vqe param
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ai_set_vqe(INT32_T card, TKL_AI_CHN_E chn, TKL_AUDIO_VQE_TYPE_E type, TKL_AUDIO_VQE_PARAM_T *pparam)
{
    (void)card;
    (void)chn;
    (void)type;
    (void)pparam;

    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief ai get vqe param
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 * @param[in] type: vqe type
 * @param[out] pparam: vqe param
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ai_get_vqe(INT32_T card, TKL_AI_CHN_E chn, TKL_AUDIO_VQE_TYPE_E type, TKL_AUDIO_VQE_PARAM_T *pparam)
{
    (void)card;
    (void)chn;
    (void)type;
    (void)pparam;

    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief ai stop
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ai_stop(INT32_T card, TKL_AI_CHN_E chn)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    (void)card;
    (void)chn;

    if ((ctx == NULL) || (!ctx->ai_started)) {
        return OPRT_OK;
    }

    (void)tkl_aud_adc_stop(TKL_AUDIO_ADC_PORT);
    ctx->ai_started = FALSE;

    return OPRT_OK;
}

/**
 * @brief ai uninit
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 * @note Also releases the DAC, which tkl_ai_init() brought up.
 */
OPERATE_RET tkl_ai_uninit(VOID)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    if (ctx == NULL) {
        return OPRT_OK;
    }

    __tkl_audio_ai_close(ctx);
    __tkl_audio_ao_close(ctx);

    sg_audio_ctx = NULL;
    tkl_system_free(ctx);

    return OPRT_OK;
}

/* ---------------------------------------------------------------------------
 * TKL output API
 * --------------------------------------------------------------------------- */
/**
 * @brief ao init
 *
 * @param[in] pconfig: audio config
 * @param[in] count: config count
 * @param[out] handle: handle of the output instance
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 * @note The DAC is already running when tkl_ai_init() succeeded; this only
 *       reports that. A caller that wants a DAC without capture still has to
 *       go through tkl_ai_init().
 */
OPERATE_RET tkl_ao_init(TKL_AUDIO_CONFIG_T *pconfig, INT32_T count, VOID **handle)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    (void)pconfig;
    (void)count;

    if (handle != NULL) {
        *handle = NULL;
    }

    if ((ctx == NULL) || (!ctx->ao_inited)) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (handle != NULL) {
        *handle = (VOID *)ctx;
    }

    return OPRT_OK;
}

/**
 * @brief ao start
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 * @param[in] handle: handle of start
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ao_start(INT32_T card, TKL_AO_CHN_E chn, VOID *handle)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    (void)card;
    (void)chn;
    (void)handle;

    if ((ctx == NULL) || (!ctx->ao_inited)) {
        return OPRT_RESOURCE_NOT_READY;
    }
    if (ctx->ao_started) {
        return OPRT_OK;
    }

    if (tkl_aud_dac_start(TKL_AUDIO_DAC_PORT) != OPRT_OK) {
        TKL_AUDIO_LOG("dac start failed\r\n");
        return OPRT_COM_ERROR;
    }
    ctx->ao_started = TRUE;

    __tkl_audio_apply_spk_volume(ctx);
    __tkl_audio_pa_set(ctx, TRUE);

    return OPRT_OK;
}

/**
 * @brief ao set volume
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 * @param[in] handle: handle of start
 * @param[in] vol: spk volume,[0, 100]
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ao_set_vol(INT32_T card, TKL_AO_CHN_E chn, VOID *handle, INT32_T vol)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    (void)card;
    (void)chn;
    (void)handle;

    if (ctx == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (vol < 0) {
        vol = 0;
    } else if (vol > TKL_AUDIO_MAX_VOLUME) {
        vol = TKL_AUDIO_MAX_VOLUME;
    }

    ctx->spk_volume = (uint8_t)vol;
    __tkl_audio_apply_spk_volume(ctx);

    return OPRT_OK;
}

/**
 * @brief ao get volume
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 * @param[in] handle: handle of start
 * @param[out] vol: spk volume
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ao_get_vol(INT32_T card, TKL_AO_CHN_E chn, VOID *handle, INT32_T *vol)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    (void)card;
    (void)chn;
    (void)handle;

    if ((ctx == NULL) || (vol == NULL)) {
        return OPRT_RESOURCE_NOT_READY;
    }

    *vol = ctx->spk_volume;

    return OPRT_OK;
}

/**
 * @brief ao output frame
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 * @param[in] handle: handle of start
 * @param[in] pframe: output frame
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ao_put_frame(INT32_T card, TKL_AO_CHN_E chn, VOID *handle, TKL_AUDIO_FRAME_INFO_T *pframe)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;
    const uint8_t   *buf;
    uint32_t         len;
    uint32_t         off;
    uint32_t         chunk_max;
    OPERATE_RET      rt;

    if ((pframe == NULL) || (pframe->pbuf == NULL)) {
        return OPRT_INVALID_PARM;
    }

    len = (pframe->used_size > 0U) ? pframe->used_size : pframe->buf_size;
    if (len == 0U) {
        return OPRT_INVALID_PARM;
    }

    if ((ctx == NULL) || (!ctx->ao_inited)) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* tkl_ao_clear_buffer() parks the DAC; bring it back for the next frame. */
    if (!ctx->ao_started) {
        rt = tkl_ao_start(card, chn, handle);
        if (rt != OPRT_OK) {
            return rt;
        }
    }

    /* The DAC ring holds only a handful of frames, so a decoded TTS frame can
     * never be written in one go. Feed it frame by frame. */
    chunk_max = tkl_aud_dac_get_frame_size(TKL_AUDIO_DAC_PORT);
    if (chunk_max == 0U) {
        chunk_max = len;
    }

    buf = (const uint8_t *)pframe->pbuf;
    off = 0U;
    while (off < len) {
        uint32_t chunk  = len - off;
        uint32_t waited = 0U;

        if (chunk > chunk_max) {
            chunk = chunk_max;
        }

        for (;;) {
            rt = tkl_aud_dac_write(TKL_AUDIO_DAC_PORT, (CONST UINT8_T *)buf + off, chunk);
            if (rt != OPRT_OS_ADAPTER_DAC_BUSY) {
                break;
            }
            if (waited >= TKL_AUDIO_SPK_WRITE_TIMEOUT_MS) {
                TKL_AUDIO_LOG("dac ring full for %ums, drop %u bytes\r\n", (unsigned)waited, (unsigned)(len - off));
                return OPRT_OK;
            }
            tkl_system_sleep(TKL_AUDIO_SPK_WRITE_RETRY_MS);
            waited += TKL_AUDIO_SPK_WRITE_RETRY_MS;
        }

        if (rt != OPRT_OK) {
            return rt;
        }

        off += chunk;
    }

    return OPRT_OK;
}

/**
 * @brief clear speaker buffer
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 * @note Stops the DAC, which drops everything queued. The next put_frame
 *       restarts it and the driver pre-fills silence, so no pop is audible.
 */
OPERATE_RET tkl_ao_clear_buffer(INT32_T card, TKL_AO_CHN_E chn)
{
    return tkl_ao_stop(card, chn, NULL);
}

/**
 * @brief ao stop
 *
 * @param[in] card: card number
 * @param[in] chn: channel number
 * @param[in] handle: handle of start
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ao_stop(INT32_T card, TKL_AO_CHN_E chn, VOID *handle)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;
    OPERATE_RET      rt;

    (void)card;
    (void)chn;
    (void)handle;

    if ((ctx == NULL) || (!ctx->ao_started)) {
        return OPRT_OK;
    }

    rt              = tkl_aud_dac_stop(TKL_AUDIO_DAC_PORT);
    ctx->ao_started = FALSE;
    __tkl_audio_pa_set(ctx, FALSE);

    return rt;
}

/**
 * @brief ao uninit
 *
 * @param[in] handle: handle of the output instance
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ao_uninit(VOID *handle)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    (void)handle;

    if (ctx == NULL) {
        return OPRT_OK;
    }

    __tkl_audio_ao_close(ctx);

    return OPRT_OK;
}

/* ---------------------------------------------------------------------------
 * TKL detect API
 * --------------------------------------------------------------------------- */
/**
 * @brief audio input detect start
 *
 * @param[in] card: card number
 * @param[in] type: detect type
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ai_detect_start(INT32_T card, TKL_MEDIA_DETECT_TYPE_E type)
{
    (void)card;
    (void)type;

    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief audio input detect stop
 *
 * @param[in] card: card number
 * @param[in] type: detect type
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ai_detect_stop(INT32_T card, TKL_MEDIA_DETECT_TYPE_E type)
{
    (void)card;
    (void)type;

    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief audio detect get result
 *
 * @param[in] card: card number
 * @param[in] type: detect type
 * @param[out] presult: audio detect result
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 */
OPERATE_RET tkl_ai_detect_get_result(INT32_T card, TKL_MEDIA_DETECT_TYPE_E type, TKL_AUDIO_DETECT_RESULT_T *presult)
{
    (void)card;
    (void)type;
    (void)presult;

    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief Install an AEC/VAD processing hook
 *
 * @param[in] user_process_fun upper-layer processing function
 *
 * @return OPRT_OK on success. Others on error, please refer to tkl_error_code.h
 * @note There is no AEC/VAD stage on the media TKL, so nothing can be hooked.
 */
OPERATE_RET tkl_ai_set_vad_aec_algorithm(aec_vad_process_fun user_process_fun)
{
    (void)user_process_fun;

    return OPRT_NOT_SUPPORTED;
}
