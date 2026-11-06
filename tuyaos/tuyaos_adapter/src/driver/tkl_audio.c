/**
 * @file tkl_audio.c
 * @brief TKL media audio (tkl_ai_* / tkl_ao_*) for T9 (BK7259), built on top of
 *        the tkl_aud_adc_* and tkl_aud_dac_* drivers.
 *
 * The media TKL is the interface consumed by src/peripherals/audio_codecs/
 * (tdd_audio.c). That layer only does:
 *
 *   open : tkl_ai_init(&cfg, 0) -> tkl_ai_start(0, 0) -> tkl_ao_set_vol(...)
 *   play : tkl_ao_put_frame(0, 0, NULL, &frame)
 *   stop : tkl_ao_clear_buffer(...)
 *   close: tkl_ai_stop(0, 0) -> tkl_ai_uninit()
 *
 * so the ADC and the DAC are both brought up here by tkl_ai_init() and torn
 * down by tkl_ai_uninit().
 *
 * Capture path: the ADC driver delivers PCM from its DMA ISR, while the upper
 * layer's frame callback takes a mutex and posts an event and must therefore
 * run in task context. Frames are copied into a small single-producer /
 * single-consumer byte ring from the ISR and drained by a worker thread.
 *
 * Playback path: PCM is written straight into the DAC DMA ring. A full ring
 * returns OPRT_OS_ADAPTER_DAC_BUSY, which is retried for a bounded time so a
 * stalled DAC can never wedge the playback thread forever.
 *
 * @copyright Copyright (c) 2025 Tuya Inc. All Rights Reserved.
 */

#include "tuya_cloud_types.h"
#include "tuya_error_code.h"

#include "tkl_audio.h"
#include "tkl_aud_adc.h"
#include "tkl_aud_dac.h"
#include "tkl_gpio.h"
#include "tkl_memory.h"
#include "tkl_system.h"

#include <os/os.h>
#include <string.h>

extern void bk_printf(const char *fmt, ...);

#define TKL_AUDIO_LOG(fmt, ...) \
    bk_printf("[tkl_audio] " fmt, ##__VA_ARGS__)

/***********************************************************
************************macro define************************
***********************************************************/
#define TKL_AUDIO_FRAME_MS              (20)
#define TKL_AUDIO_RING_FRAMES           (8)          /* ~160 ms of mic audio */
#define TKL_AUDIO_TASK_STACK            (4096)
#define TKL_AUDIO_TASK_PRIO             (BEKEN_DEFAULT_WORKER_PRIORITY)
#define TKL_AUDIO_ADC_PORT              (TUYA_AUDIO_ADC_PORT_0)
#define TKL_AUDIO_DAC_PORT              (TUYA_AUDIO_DAC_PORT_0)

#define TKL_AUDIO_SPK_WRITE_RETRY_MS    (10)
#define TKL_AUDIO_SPK_WRITE_TIMEOUT_MS  (200)

/* Speaker volume 0..100 -> DAC digital gain dB. 0 mutes (the DAC driver treats
 * anything at or below its silence threshold as mute); 100 is unity gain so a
 * small speaker is not driven into clipping. */
#define TKL_AUDIO_SPK_DB_MUTE           (-100)
#define TKL_AUDIO_SPK_DB_MIN            (-36)

/* Mic volume 0..100 -> ADC digital gain dB. */
#define TKL_AUDIO_MIC_DB_MIN            (-12)
#define TKL_AUDIO_MIC_DB_SPAN           (42)

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef struct {
    TKL_AUDIO_CONFIG_T cfg;

    uint32_t           frame_size;      /* one 20 ms mono PCM frame, bytes */
    uint32_t           sample_rate;
    uint32_t           sample_bits;
    uint32_t           sample_ch;

    /* capture ring, single producer (ADC ISR) / single consumer (task) */
    uint8_t           *ring;
    uint32_t           ring_size;       /* power of two */
    volatile uint32_t  ring_wr;
    volatile uint32_t  ring_rd;

    beken_semaphore_t  sem;
    beken_semaphore_t  stop_sem;
    beken_thread_t     task;
    BOOL_T             task_created;
    volatile BOOL_T    task_run;

    BOOL_T             ai_inited;
    BOOL_T             ai_started;

    BOOL_T             ao_inited;
    BOOL_T             ao_started;
    uint8_t            spk_volume;
    int32_t            spk_gpio;
    int32_t            spk_polarity;
} TKL_AUDIO_CTX_T;

/***********************************************************
***********************variable define**********************
***********************************************************/
static TKL_AUDIO_CTX_T *sg_audio_ctx = NULL;

/***********************************************************
********************function declaration********************
***********************************************************/
static void __tkl_audio_task(beken_thread_arg_t arg);

/***********************************************************
***********************function define**********************
***********************************************************/
static uint32_t __ring_write(TKL_AUDIO_CTX_T *ctx, const uint8_t *data, uint32_t len)
{
    uint32_t used = ctx->ring_wr - ctx->ring_rd;
    uint32_t free_len = ctx->ring_size - used;
    uint32_t pos, first;

    if (len > free_len) {
        len = free_len;
    }
    if (len == 0U) {
        return 0;
    }

    pos = ctx->ring_wr & (ctx->ring_size - 1U);
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
    uint32_t used = ctx->ring_wr - ctx->ring_rd;
    uint32_t pos, first;

    if (len > used) {
        len = used;
    }
    if (len == 0U) {
        return 0;
    }

    pos = ctx->ring_rd & (ctx->ring_size - 1U);
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

/* ADC DMA ISR: copy PCM into the ring and let the worker do the real work. */
static VOID_T __tkl_audio_adc_cb(TUYA_AUDIO_INPUT_BUFFER_T *buffer)
{
    TKL_AUDIO_CTX_T *ctx = (buffer != NULL) ? (TKL_AUDIO_CTX_T *)buffer->args : NULL;

    if ((ctx == NULL) || (buffer->event != TUYA_AUDIO_FRAME_EVENT_ADC_RX) ||
        (buffer->data == NULL) || (buffer->data_len == 0U)) {
        return;
    }
    if (!ctx->ai_started || (ctx->cfg.put_cb == NULL)) {
        return;
    }

    if (__ring_write(ctx, buffer->data, buffer->data_len) != buffer->data_len) {
        /* Overflow: drop the frame rather than block in the ISR. */
        return;
    }

    (void)rtos_set_semaphore(&ctx->sem);
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
    uint8_t *frame = NULL;

    frame = (uint8_t *)tkl_system_malloc(ctx->frame_size);
    if (frame == NULL) {
        TKL_AUDIO_LOG("mic task malloc %u failed\r\n", (unsigned)ctx->frame_size);
    }

    while (ctx->task_run) {
        (void)rtos_get_semaphore(&ctx->sem, TKL_AUDIO_FRAME_MS);
        if (!ctx->task_run) {
            break;
        }
        if (frame == NULL) {
            continue;
        }
        while (ctx->task_run &&
               (__ring_read(ctx, frame, ctx->frame_size) == ctx->frame_size)) {
            __tkl_audio_deliver(ctx, frame, ctx->frame_size);
        }
    }

    if (frame != NULL) {
        tkl_system_free(frame);
    }

    (void)rtos_set_semaphore(&ctx->stop_sem);
    rtos_delete_thread(NULL);
}

static void __tkl_audio_pa_init(TKL_AUDIO_CTX_T *ctx)
{
    TUYA_GPIO_BASE_CFG_T gpio_cfg;

    if (ctx->spk_gpio < 0) {
        return;
    }

    memset(&gpio_cfg, 0, sizeof(gpio_cfg));
    gpio_cfg.direct = TUYA_GPIO_OUTPUT;
    gpio_cfg.mode   = TUYA_GPIO_PUSH_PULL;
    /* polarity is the level that enables the amplifier: start muted. */
    gpio_cfg.level  = (ctx->spk_polarity == 0) ? TUYA_GPIO_LEVEL_LOW : TUYA_GPIO_LEVEL_HIGH;
    gpio_cfg.drive_cap = TUYA_GPIO_DRIVE_CAP_0;

    if (tkl_gpio_init((TUYA_GPIO_NUM_E)ctx->spk_gpio, &gpio_cfg) != OPRT_OK) {
        TKL_AUDIO_LOG("spk pa gpio %d init failed\r\n", (int)ctx->spk_gpio);
        ctx->spk_gpio = -1;
    }
}

static void __tkl_audio_pa_set(TKL_AUDIO_CTX_T *ctx, BOOL_T on)
{
    TUYA_GPIO_LEVEL_E level;

    if (ctx->spk_gpio < 0) {
        return;
    }

    if (on) {
        level = (ctx->spk_polarity == 0) ? TUYA_GPIO_LEVEL_HIGH : TUYA_GPIO_LEVEL_LOW;
    } else {
        level = (ctx->spk_polarity == 0) ? TUYA_GPIO_LEVEL_LOW : TUYA_GPIO_LEVEL_HIGH;
    }
    (void)tkl_gpio_write((TUYA_GPIO_NUM_E)ctx->spk_gpio, level);
}

static void __tkl_audio_apply_spk_volume(TKL_AUDIO_CTX_T *ctx)
{
    INT32_T gain[1];

    if (!ctx->ao_started) {
        return;
    }

    if (ctx->spk_volume == 0U) {
        gain[0] = TKL_AUDIO_SPK_DB_MUTE;
    } else {
        gain[0] = TKL_AUDIO_SPK_DB_MIN +
                  ((INT32_T)ctx->spk_volume * (-TKL_AUDIO_SPK_DB_MIN)) / 100;
    }

    (void)tkl_aud_dac_set_gain(TKL_AUDIO_DAC_PORT, TUYA_AUDIO_DAC_CHANNEL_0, gain, 1);
}

static OPERATE_RET __tkl_audio_ao_open(TKL_AUDIO_CTX_T *ctx)
{
    TUYA_AUDIO_DAC_BASE_CFG_T cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.dac_chan     = TUYA_AUDIO_DAC_CHANNEL_0;
    cfg.sample_bits  = TUYA_AUDIO_SAMPLE_BITS_16;
    cfg.sample_rate  = (ctx->cfg.spk_sample > 0) ? (uint32_t)ctx->cfg.spk_sample : ctx->sample_rate;
    cfg.frame_time_ms = TKL_AUDIO_FRAME_MS;
    cfg.frame_cb     = NULL;
    cfg.args         = NULL;

    if (tkl_aud_dac_init(TKL_AUDIO_DAC_PORT, &cfg) != OPRT_OK) {
        TKL_AUDIO_LOG("dac init failed\r\n");
        return OPRT_COM_ERROR;
    }
    ctx->ao_inited = TRUE;

    if (tkl_aud_dac_start(TKL_AUDIO_DAC_PORT) != OPRT_OK) {
        TKL_AUDIO_LOG("dac start failed\r\n");
        return OPRT_COM_ERROR;
    }
    ctx->ao_started = TRUE;

    __tkl_audio_pa_init(ctx);
    __tkl_audio_pa_set(ctx, TRUE);
    __tkl_audio_apply_spk_volume(ctx);

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
    __tkl_audio_pa_set(ctx, FALSE);
    if (ctx->spk_gpio >= 0) {
        (void)tkl_gpio_deinit((TUYA_GPIO_NUM_E)ctx->spk_gpio);
        ctx->spk_gpio = -1;
    }
}

static OPERATE_RET __tkl_audio_ai_open(TKL_AUDIO_CTX_T *ctx)
{
    TUYA_AUDIO_ADC_BASE_CFG_T cfg;
    uint8_t *ring = NULL;
    uint32_t ring_size;
    OPERATE_RET rt;
    BOOL_T sem_ok = FALSE;
    BOOL_T stop_sem_ok = FALSE;

    /* One capture lane; the ADC driver extracts it from its raw DMA frame.
     * TUYA_T9_EVB: MIC1 (ADC lane 0) is not wired/populated -- measured
     * min/max +-5 at dig 16dB/ana 20dB -- MIC2 (ADC lane 1) carries the talk
     * mic. chan is a bitmap now, so the gain call below must name the same
     * bit. */
    memset(&cfg, 0, sizeof(cfg));
    cfg.chan          = TUYA_AUDIO_ADC_CHANNEL_1;
    cfg.sample_bits   = TUYA_AUDIO_SAMPLE_BITS_16;
    cfg.sample_rate   = ctx->sample_rate;
    cfg.frame_time_ms = TKL_AUDIO_FRAME_MS;
    cfg.upper_cb      = __tkl_audio_adc_cb;
    cfg.args          = ctx;

    ring_size = ctx->frame_size * TKL_AUDIO_RING_FRAMES;
    /* Round up to a power of two; the ring uses mask arithmetic. */
    {
        uint32_t pow2 = 1U;
        while (pow2 < ring_size) {
            pow2 <<= 1;
        }
        ring_size = pow2;
    }

    ring = (uint8_t *)tkl_system_malloc(ring_size);
    if (ring == NULL) {
        TKL_AUDIO_LOG("mic ring malloc %u failed\r\n", (unsigned)ring_size);
        return OPRT_MALLOC_FAILED;
    }
    ctx->ring = ring;
    ctx->ring_size = ring_size;
    ctx->ring_wr = 0;
    ctx->ring_rd = 0;

    rt = rtos_init_semaphore_ex(&ctx->sem, 1, 0);
    if (rt != BK_OK) {
        TKL_AUDIO_LOG("mic sem init failed %d\r\n", rt);
        goto __ERR;
    }
    sem_ok = TRUE;
    rt = rtos_init_semaphore_ex(&ctx->stop_sem, 1, 0);
    if (rt != BK_OK) {
        TKL_AUDIO_LOG("mic stop sem init failed %d\r\n", rt);
        goto __ERR;
    }
    stop_sem_ok = TRUE;

    ctx->task_run = TRUE;
    rt = rtos_create_thread(&ctx->task, TKL_AUDIO_TASK_PRIO, "tkl_audio_mic",
                            __tkl_audio_task, TKL_AUDIO_TASK_STACK,
                            (beken_thread_arg_t)ctx);
    if (rt != BK_OK) {
        TKL_AUDIO_LOG("mic task create failed %d\r\n", rt);
        ctx->task_run = FALSE;
        goto __ERR;
    }
    ctx->task_created = TRUE;

    if (tkl_aud_adc_init(TKL_AUDIO_ADC_PORT, &cfg) != OPRT_OK) {
        TKL_AUDIO_LOG("adc init failed\r\n");
        goto __ERR;
    }
    ctx->ai_inited = TRUE;

    return OPRT_OK;

__ERR:
    if (ctx->task_created) {
        ctx->task_run = FALSE;
        (void)rtos_set_semaphore(&ctx->sem);
        (void)rtos_get_semaphore(&ctx->stop_sem, BEKEN_WAIT_FOREVER);
        ctx->task_created = FALSE;
    }
    if (stop_sem_ok) {
        (void)rtos_deinit_semaphore(&ctx->stop_sem);
    }
    if (sem_ok) {
        (void)rtos_deinit_semaphore(&ctx->sem);
    }
    ctx->ring = NULL;
    tkl_system_free(ring);
    return OPRT_COM_ERROR;
}

static void __tkl_audio_ai_close(TKL_AUDIO_CTX_T *ctx)
{
    if (ctx->ai_started) {
        (void)tkl_aud_adc_stop(TKL_AUDIO_ADC_PORT);
        ctx->ai_started = FALSE;
    }
    if (ctx->ai_inited) {
        tkl_aud_adc_deinit(TKL_AUDIO_ADC_PORT);
        ctx->ai_inited = FALSE;
    }

    if (ctx->task_created) {
        ctx->task_run = FALSE;
        (void)rtos_set_semaphore(&ctx->sem);
        (void)rtos_get_semaphore(&ctx->stop_sem, BEKEN_WAIT_FOREVER);
        ctx->task_created = FALSE;
        (void)rtos_deinit_semaphore(&ctx->stop_sem);
        (void)rtos_deinit_semaphore(&ctx->sem);
    }

    if (ctx->ring != NULL) {
        tkl_system_free(ctx->ring);
        ctx->ring = NULL;
        ctx->ring_size = 0;
    }
}

/***********************************************************
***********************TKL input API************************
***********************************************************/
OPERATE_RET tkl_ai_init(TKL_AUDIO_CONFIG_T *pconfig, INT32_T count)
{
    TKL_AUDIO_CTX_T *ctx = NULL;
    OPERATE_RET rt;

    (void)count;

    if (pconfig == NULL) {
        return OPRT_INVALID_PARM;
    }
    if (sg_audio_ctx != NULL) {
        return OPRT_OK;
    }
    if ((pconfig->datebits != TKL_AUDIO_DATABITS_16) ||
        (pconfig->channel != TKL_AUDIO_CHANNEL_MONO) ||
        (pconfig->sample == TKL_AUDIO_SAMPLE_MAX) || (pconfig->sample == 0)) {
        TKL_AUDIO_LOG("unsupported format: %uHz/%ubit/%uch\r\n",
                      (unsigned)pconfig->sample, (unsigned)pconfig->datebits,
                      (unsigned)pconfig->channel);
        return OPRT_INVALID_PARM;
    }

    ctx = (TKL_AUDIO_CTX_T *)tkl_system_malloc(sizeof(TKL_AUDIO_CTX_T));
    if (ctx == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    memset(ctx, 0, sizeof(TKL_AUDIO_CTX_T));

    memcpy(&ctx->cfg, pconfig, sizeof(TKL_AUDIO_CONFIG_T));
    ctx->sample_rate = (uint32_t)pconfig->sample;
    ctx->sample_bits = (uint32_t)pconfig->datebits;
    ctx->sample_ch   = (uint32_t)pconfig->channel;
    ctx->frame_size  = ctx->sample_rate * (ctx->sample_bits / 8U) * ctx->sample_ch *
                       TKL_AUDIO_FRAME_MS / 1000U;
    if (ctx->frame_size == 0U) {
        tkl_system_free(ctx);
        return OPRT_INVALID_PARM;
    }

    ctx->spk_gpio     = pconfig->spk_gpio;
    ctx->spk_polarity = pconfig->spk_gpio_polarity;
    ctx->spk_volume   = (pconfig->spk_volume > 0) ? (uint8_t)pconfig->spk_volume : 80U;
    if (ctx->spk_volume > 100U) {
        ctx->spk_volume = 100U;
    }

    rt = __tkl_audio_ai_open(ctx);
    if (rt != OPRT_OK) {
        goto __ERR;
    }

    rt = __tkl_audio_ao_open(ctx);
    if (rt != OPRT_OK) {
        __tkl_audio_ai_close(ctx);
        __tkl_audio_ao_close(ctx);
        goto __ERR;
    }

    sg_audio_ctx = ctx;

    TKL_AUDIO_LOG("init %uHz/%ubit/%uch, frame %u\r\n",
                  (unsigned)ctx->sample_rate, (unsigned)ctx->sample_bits,
                  (unsigned)ctx->sample_ch, (unsigned)ctx->frame_size);

    return OPRT_OK;

__ERR:
    tkl_system_free(ctx);
    return (rt != OPRT_OK) ? rt : OPRT_COM_ERROR;
}

OPERATE_RET tkl_ai_start(INT32_T card, TKL_AI_CHN_E chn)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    (void)card;
    (void)chn;

    if (ctx == NULL || !ctx->ai_inited) {
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

OPERATE_RET tkl_ai_set_vol(INT32_T card, TKL_AI_CHN_E chn, INT32_T vol)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;
    INT32_T gain[1];

    (void)card;
    (void)chn;

    if (ctx == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }
    if (vol < 0) {
        vol = 0;
    } else if (vol > 100) {
        vol = 100;
    }

    gain[0] = TKL_AUDIO_MIC_DB_MIN + (vol * TKL_AUDIO_MIC_DB_SPAN) / 100;

    return tkl_aud_adc_set_gain(TKL_AUDIO_ADC_PORT, TUYA_AUDIO_ADC_CHANNEL_1, gain, 1);
}

OPERATE_RET tkl_ai_get_frame(INT32_T card, TKL_AI_CHN_E chn, TKL_AUDIO_FRAME_INFO_T *pframe)
{
    (void)card;
    (void)chn;
    (void)pframe;

    /* Capture is delivered through put_cb, pull mode is not implemented. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_ai_set_vqe(INT32_T card, TKL_AI_CHN_E chn, TKL_AUDIO_VQE_TYPE_E type,
                           TKL_AUDIO_VQE_PARAM_T *pparam)
{
    (void)card;
    (void)chn;
    (void)type;
    (void)pparam;

    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_ai_get_vqe(INT32_T card, TKL_AI_CHN_E chn, TKL_AUDIO_VQE_TYPE_E type,
                           TKL_AUDIO_VQE_PARAM_T *pparam)
{
    (void)card;
    (void)chn;
    (void)type;
    (void)pparam;

    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_ai_stop(INT32_T card, TKL_AI_CHN_E chn)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    (void)card;
    (void)chn;

    if (ctx == NULL) {
        return OPRT_OK;
    }
    if (!ctx->ai_started) {
        return OPRT_OK;
    }

    (void)tkl_aud_adc_stop(TKL_AUDIO_ADC_PORT);
    ctx->ai_started = FALSE;

    return OPRT_OK;
}

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

/***********************************************************
***********************TKL output API***********************
***********************************************************/
OPERATE_RET tkl_ao_init(TKL_AUDIO_CONFIG_T *pconfig, INT32_T count, VOID **handle)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    (void)pconfig;
    (void)count;
    if (handle != NULL) {
        *handle = NULL;
    }

    /* tkl_ai_init() already brought the DAC up. */
    if (ctx != NULL && ctx->ao_inited) {
        return OPRT_OK;
    }
    return OPRT_RESOURCE_NOT_READY;
}

OPERATE_RET tkl_ao_start(INT32_T card, TKL_AO_CHN_E chn, VOID *handle)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;
    OPERATE_RET rt;

    (void)card;
    (void)chn;
    (void)handle;

    if (ctx == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }
    if (!ctx->ao_inited) {
        return OPRT_COM_ERROR;
    }
    if (ctx->ao_started) {
        return OPRT_OK;
    }

    rt = tkl_aud_dac_start(TKL_AUDIO_DAC_PORT);
    if (rt != OPRT_OK) {
        TKL_AUDIO_LOG("dac start failed %d\r\n", rt);
        return rt;
    }
    ctx->ao_started = TRUE;
    __tkl_audio_pa_set(ctx, TRUE);
    __tkl_audio_apply_spk_volume(ctx);

    return OPRT_OK;
}

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
    } else if (vol > 100) {
        vol = 100;
    }

    ctx->spk_volume = (uint8_t)vol;
    __tkl_audio_apply_spk_volume(ctx);

    return OPRT_OK;
}

OPERATE_RET tkl_ao_get_vol(INT32_T card, TKL_AO_CHN_E chn, VOID *handle, INT32_T *vol)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;

    (void)card;
    (void)chn;
    (void)handle;

    if (ctx == NULL || vol == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    *vol = ctx->spk_volume;

    return OPRT_OK;
}

OPERATE_RET tkl_ao_put_frame(INT32_T card, TKL_AO_CHN_E chn, VOID *handle,
                             TKL_AUDIO_FRAME_INFO_T *pframe)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;
    OPERATE_RET rt;
    uint32_t len;
    uint32_t waited = 0;

    (void)card;
    (void)chn;
    (void)handle;

    if (ctx == NULL || pframe == NULL || pframe->pbuf == NULL) {
        return OPRT_INVALID_PARM;
    }
    len = (pframe->used_size > 0U) ? pframe->used_size : pframe->buf_size;
    if (len == 0U) {
        return OPRT_INVALID_PARM;
    }
    if (!ctx->ao_inited) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* A previous PLAY_STOP may have parked the DAC; bring it back. */
    if (!ctx->ao_started) {
        rt = tkl_ao_start(card, chn, handle);
        if (rt != OPRT_OK) {
            return rt;
        }
    }

    /* The DAC driver's internal ring only holds a few 20 ms frames (~3.8 KB
     * here). Decoded TTS/MP3 frames arrive much larger (e.g. 8064 B), so a
     * single all-or-nothing write can never succeed and the whole buffer used
     * to be dropped. Feed the ring in frame-sized chunks instead, waiting for
     * space between them. */
    {
        const uint8_t *buf = (const uint8_t *)pframe->pbuf;
        uint32_t       off = 0;
        uint32_t       chunk_max = ctx->frame_size;

        if (chunk_max == 0U) {
            chunk_max = len;
        }

        while (off < len) {
            uint32_t chunk = len - off;
            if (chunk > chunk_max) {
                chunk = chunk_max;
            }

            rt = tkl_aud_dac_write(TKL_AUDIO_DAC_PORT, (UINT8_T *)buf + off, chunk);
            if (rt == OPRT_OS_ADAPTER_DAC_BUSY) {
                if (waited >= TKL_AUDIO_SPK_WRITE_TIMEOUT_MS) {
                    TKL_AUDIO_LOG("dac ring full for %ums, drop %u bytes\r\n",
                                  (unsigned)waited, (unsigned)(len - off));
                    return OPRT_OK;
                }
                tkl_system_sleep(TKL_AUDIO_SPK_WRITE_RETRY_MS);
                waited += TKL_AUDIO_SPK_WRITE_RETRY_MS;
                continue;
            }
            if (rt != OPRT_OK) {
                return rt;
            }

            off += chunk;
            /* Per-chunk timeout: a long frame legitimately takes its own
             * duration to drain through the ring. */
            waited = 0;
        }
    }

    return rt;
}

OPERATE_RET tkl_ao_stop(INT32_T card, TKL_AO_CHN_E chn, VOID *handle)
{
    TKL_AUDIO_CTX_T *ctx = sg_audio_ctx;
    OPERATE_RET rt = OPRT_OK;

    (void)card;
    (void)chn;
    (void)handle;

    if (ctx == NULL || !ctx->ao_started) {
        return OPRT_OK;
    }

    rt = tkl_aud_dac_stop(TKL_AUDIO_DAC_PORT);
    ctx->ao_started = FALSE;
    __tkl_audio_pa_set(ctx, FALSE);

    return rt;
}

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

OPERATE_RET tkl_ao_clear_buffer(INT32_T card, TKL_AO_CHN_E chn)
{
    (void)card;
    (void)chn;

    /* Stop playback and drop the queued PCM; the next put_frame restarts the
     * DAC, which pre-fills silence before enabling the output. */
    return tkl_ao_stop(card, chn, NULL);
}

/***********************************************************
***********************TKL detect API***********************
***********************************************************/
OPERATE_RET tkl_ai_detect_start(INT32_T card, TKL_MEDIA_DETECT_TYPE_E type)
{
    (void)card;
    (void)type;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_ai_detect_stop(INT32_T card, TKL_MEDIA_DETECT_TYPE_E type)
{
    (void)card;
    (void)type;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_ai_detect_get_result(INT32_T card, TKL_MEDIA_DETECT_TYPE_E type,
                                     TKL_AUDIO_DETECT_RESULT_T *presult)
{
    (void)card;
    (void)type;
    (void)presult;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_ai_set_vad_aec_algorithm(aec_vad_process_fun user_process_fun)
{
    (void)user_process_fun;
    return OPRT_NOT_SUPPORTED;
}
