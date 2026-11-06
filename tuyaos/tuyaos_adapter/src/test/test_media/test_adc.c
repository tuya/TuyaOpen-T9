#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/mem.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_aud_adc.h"
#include "tkl_aud_dac.h"
#include "tkl_gpio.h"
#include "tkl_memory.h"
#include "tkl_output.h"
#include "tkl_system.h"

#define ADC_TEST_DEFAULT_PORT          TUYA_AUDIO_ADC_PORT_0
#define ADC_TEST_DEFAULT_CHAN          ADC_TEST_MODE_L
#define ADC_TEST_DEFAULT_SAMPLE_RATE   16000U
#define ADC_TEST_DEFAULT_SAMPLE_BITS   TUYA_AUDIO_SAMPLE_BITS_16
#define ADC_TEST_DEFAULT_FRAME_TIME_MS 20U
#define ADC_TEST_DEFAULT_DIG_DB        (16)     /* OEM DEFAULT dig_gain */
#define ADC_TEST_DEFAULT_ANA_DB        (20)     /* OEM DEFAULT ana_gain */
#define ADC_TEST_GAIN_NUM              (2U)
#define ADC_TEST_DEFAULT_REC_MS        2000U
#define ADC_TEST_DEFAULT_DAC_PORT      TUYA_AUDIO_DAC_PORT_0
#define ADC_TEST_DEFAULT_DAC_DIG_DB    (-7)
#define ADC_TEST_DEFAULT_DAC_ANA_DB    (4)
#define ADC_TEST_DEFAULT_LOOP_REF_MIC  (1U)   /* MIC1..MIC3, used by RAW3 loop */
#define ADC_TEST_DUMP_MAX_SAMPLES      32U
#define ADC_TEST_DUMP_DEFAULT_SAMPLES  12U

/* The CLI's ADC mode selector. TKL replaced its L/R/LR/MIC3/RAW3/MIC23 enum
 * with a channel bitmap (TUYA_AUDIO_MIC_CHANNEL_E), so the mode label lives
 * here now: it is a command-line convenience, not an ABI type. Values must stay
 * contiguous from 0 - the CLI parses a bare integer and casts to this type. */
typedef enum {
    ADC_TEST_MODE_L = 0,
    ADC_TEST_MODE_R,
    ADC_TEST_MODE_LR,
    ADC_TEST_MODE_MIC3,
    ADC_TEST_MODE_RAW3,
    ADC_TEST_MODE_MIC23,
    ADC_TEST_MODE_MAX
} ADC_TEST_MODE_E;

typedef struct {
    bool initialized;
    bool started;
    TUYA_AUDIO_ADC_PORT_E port;
    ADC_TEST_MODE_E chan;
    uint32_t sample_rate;
    uint8_t sample_bits;
    uint32_t frame_time_ms;
    INT32_T dig_db;
    INT32_T ana_db;
    uint32_t frame_size;
    volatile uint32_t frame_cnt;
    volatile uint32_t total_bytes;
    volatile uint32_t last_frame_bytes;
    uint32_t last_rec_frames;
    uint32_t last_rec_bytes;
    bool loop_on;
    bool dac_inited;
    bool dac_started;
    TUYA_AUDIO_DAC_PORT_E dac_port;
    INT32_T dac_dig_db;
    INT32_T dac_ana_db;
    uint8_t loop_ref_mic;       /**< RAW3 loop: 1..3 which MIC is 回采 (dropped) */
    uint8_t *loop_mix_buf;
    uint32_t loop_mix_buf_size;
    volatile uint32_t loop_push_bytes;
    volatile uint32_t loop_drop_bytes;
    volatile uint32_t loop_busy_cnt;
    volatile uint32_t loop_write_err_cnt;
    volatile uint32_t last_pcm_sample_cnt;
    volatile int16_t last_pcm_min;
    volatile int16_t last_pcm_max;
    int16_t last_pcm_dump[ADC_TEST_DUMP_MAX_SAMPLES];
} test_adc_ctx_t;

static test_adc_ctx_t s_adc_ctx = {
    .initialized = false,
    .started = false,
    .port = ADC_TEST_DEFAULT_PORT,
    .chan = ADC_TEST_DEFAULT_CHAN,
    .sample_rate = ADC_TEST_DEFAULT_SAMPLE_RATE,
    .sample_bits = ADC_TEST_DEFAULT_SAMPLE_BITS,
    .frame_time_ms = ADC_TEST_DEFAULT_FRAME_TIME_MS,
    .dig_db = ADC_TEST_DEFAULT_DIG_DB,
    .ana_db = ADC_TEST_DEFAULT_ANA_DB,
    .frame_size = 0U,
    .frame_cnt = 0U,
    .total_bytes = 0U,
    .last_frame_bytes = 0U,
    .last_rec_frames = 0U,
    .last_rec_bytes = 0U,
    .loop_on = false,
    .dac_inited = false,
    .dac_started = false,
    .dac_port = ADC_TEST_DEFAULT_DAC_PORT,
    .dac_dig_db = ADC_TEST_DEFAULT_DAC_DIG_DB,
    .dac_ana_db = ADC_TEST_DEFAULT_DAC_ANA_DB,
    .loop_ref_mic = ADC_TEST_DEFAULT_LOOP_REF_MIC,
    .loop_mix_buf = NULL,
    .loop_mix_buf_size = 0U,
    .loop_push_bytes = 0U,
    .loop_drop_bytes = 0U,
    .loop_busy_cnt = 0U,
    .loop_write_err_cnt = 0U,
    .last_pcm_sample_cnt = 0U,
    .last_pcm_min = 0,
    .last_pcm_max = 0,
    .last_pcm_dump = {0},
};

static OPERATE_RET test_adc_init(uint32_t port, uint32_t chan, uint32_t sample_rate, uint32_t frame_time_ms, INT32_T dig_db, INT32_T ana_db);
static OPERATE_RET test_adc_start(void);
static OPERATE_RET test_adc_deinit(void);
static TUYA_AUDIO_MIC_CHANNEL_E test_adc_lane_bit(UINT32_T lane);
static TUYA_AUDIO_MIC_CHANNEL_E test_adc_chan_to_bitmap(ADC_TEST_MODE_E chan);

/**
 * @brief Apply dig/ana gain to every lane the current mode owns
 * @param[in] port ADC port
 * @param[in] dig_db digital gain dB
 * @param[in] ana_db analog gain dB
 * @return OPRT_OK on success
 * @note Iterates the mode's own lane bitmap. A fixed lane list is wrong: the
 *       capture core rejects (OPRT_INVALID_PARM) any lane the user does not
 *       own, so LR (lane0+1) must not touch lane2, and MIC3 (lane2 only) must
 *       not start from lane0.
 */
static OPERATE_RET test_adc_apply_gain_active(TUYA_AUDIO_ADC_PORT_E port, INT32_T dig_db, INT32_T ana_db)
{
    OPERATE_RET ret;
    INT32_T gain[2];
    UINT32_T lane;
    UINT32_T mask;

    gain[0] = dig_db;
    gain[1] = ana_db;

    mask = (UINT32_T)test_adc_chan_to_bitmap(s_adc_ctx.chan);
    if ((mask == 0U) || (mask > 0x7U)) {
        return OPRT_INVALID_PARM;
    }

    for (lane = 0U; lane < 3U; lane++) {
        if ((mask & (1U << lane)) == 0U) {
            continue;
        }

        ret = tkl_aud_adc_set_gain(port, test_adc_lane_bit(lane), gain, ADC_TEST_GAIN_NUM);
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    return OPRT_OK;
}

static uint32_t test_adc_channel_count(ADC_TEST_MODE_E chan)
{
    if (chan == ADC_TEST_MODE_L ||
        chan == ADC_TEST_MODE_R ||
        chan == ADC_TEST_MODE_MIC3) {
        return 1U;
    }
    if (chan == ADC_TEST_MODE_LR ||
        chan == ADC_TEST_MODE_MIC23) {
        return 2U;
    }
    if (chan == ADC_TEST_MODE_RAW3) {
        return 3U;
    }
    return 0U;
}

/**
 * @brief Convert an ADC lane index 0..2 into its TUYA_AUDIO_MIC_CHANNEL_E bit
 * @param[in] lane lane index
 * @return the channel bit, or TUYA_AUDIO_MIC_CHANNEL_MAX if out of range
 */
static TUYA_AUDIO_MIC_CHANNEL_E test_adc_lane_bit(UINT32_T lane)
{
    if (lane > 2U) {
        return TUYA_AUDIO_MIC_CHANNEL_MAX;
    }

    return (TUYA_AUDIO_MIC_CHANNEL_E)((UINT32_T)TUYA_AUDIO_ADC_CHANNEL_0 << lane);
}

/**
 * @brief Convert the test CLI's mode selector into the TKL channel bitmap
 * @param[in] chan test mode (ADC_TEST_MODE_E)
 * @return the matching TUYA_AUDIO_MIC_CHANNEL_E bitmap
 */
static TUYA_AUDIO_MIC_CHANNEL_E test_adc_chan_to_bitmap(ADC_TEST_MODE_E chan)
{
    switch (chan) {
        case ADC_TEST_MODE_L:
            return TUYA_AUDIO_ADC_CHANNEL_0;
        case ADC_TEST_MODE_R:
            return TUYA_AUDIO_ADC_CHANNEL_1;
        case ADC_TEST_MODE_LR:
            return (TUYA_AUDIO_MIC_CHANNEL_E)((UINT32_T)TUYA_AUDIO_ADC_CHANNEL_0 |
                                              (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_1);
        case ADC_TEST_MODE_MIC3:
            return TUYA_AUDIO_ADC_CHANNEL_2;
        case ADC_TEST_MODE_RAW3:
            return (TUYA_AUDIO_MIC_CHANNEL_E)((UINT32_T)TUYA_AUDIO_ADC_CHANNEL_0 |
                                              (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_1 |
                                              (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_2);
        case ADC_TEST_MODE_MIC23:
            return (TUYA_AUDIO_MIC_CHANNEL_E)((UINT32_T)TUYA_AUDIO_ADC_CHANNEL_1 |
                                              (UINT32_T)TUYA_AUDIO_ADC_CHANNEL_2);
        default:
            return TUYA_AUDIO_MIC_CHANNEL_MAX;
    }
}

static uint32_t test_adc_calc_frame_size(uint32_t sample_rate, uint8_t sample_bits, uint32_t frame_ms, uint32_t chan_num)
{
    if (sample_rate == 0U || sample_bits == 0U || frame_ms == 0U || chan_num == 0U) {
        return 0U;
    }

    return ((sample_rate * (sample_bits / 8U) * chan_num) / 1000U) * frame_ms;
}

static void test_adc_runtime_reset(void)
{
    s_adc_ctx.initialized = false;
    s_adc_ctx.started = false;
    s_adc_ctx.port = ADC_TEST_DEFAULT_PORT;
    s_adc_ctx.chan = ADC_TEST_DEFAULT_CHAN;
    s_adc_ctx.sample_rate = ADC_TEST_DEFAULT_SAMPLE_RATE;
    s_adc_ctx.sample_bits = ADC_TEST_DEFAULT_SAMPLE_BITS;
    s_adc_ctx.frame_time_ms = ADC_TEST_DEFAULT_FRAME_TIME_MS;
    s_adc_ctx.dig_db = ADC_TEST_DEFAULT_DIG_DB;
    s_adc_ctx.ana_db = ADC_TEST_DEFAULT_ANA_DB;
    s_adc_ctx.frame_size = 0U;
    s_adc_ctx.frame_cnt = 0U;
    s_adc_ctx.total_bytes = 0U;
    s_adc_ctx.last_frame_bytes = 0U;
    s_adc_ctx.last_rec_frames = 0U;
    s_adc_ctx.last_rec_bytes = 0U;
    s_adc_ctx.loop_on = false;
    s_adc_ctx.dac_inited = false;
    s_adc_ctx.dac_started = false;
    s_adc_ctx.dac_port = ADC_TEST_DEFAULT_DAC_PORT;
    s_adc_ctx.dac_dig_db = ADC_TEST_DEFAULT_DAC_DIG_DB;
    s_adc_ctx.dac_ana_db = ADC_TEST_DEFAULT_DAC_ANA_DB;
    s_adc_ctx.loop_ref_mic = ADC_TEST_DEFAULT_LOOP_REF_MIC;
    s_adc_ctx.loop_mix_buf = NULL;
    s_adc_ctx.loop_mix_buf_size = 0U;
    s_adc_ctx.loop_push_bytes = 0U;
    s_adc_ctx.loop_drop_bytes = 0U;
    s_adc_ctx.loop_busy_cnt = 0U;
    s_adc_ctx.loop_write_err_cnt = 0U;
    s_adc_ctx.last_pcm_sample_cnt = 0U;
    s_adc_ctx.last_pcm_min = 0;
    s_adc_ctx.last_pcm_max = 0;
    (void)os_memset(s_adc_ctx.last_pcm_dump, 0, sizeof(s_adc_ctx.last_pcm_dump));
}

static void test_adc_snapshot_pcm(const uint8_t *buf, uint32_t buf_len)
{
    const int16_t *pcm = (const int16_t *)buf;
    uint32_t sample_cnt = buf_len / 2U;
    uint32_t dump_cnt;
    uint32_t i;
    int16_t min_v;
    int16_t max_v;

    if ((buf == NULL) || (buf_len < 2U) || ((buf_len & 0x1U) != 0U)) {
        s_adc_ctx.last_pcm_sample_cnt = 0U;
        return;
    }

    min_v = pcm[0];
    max_v = pcm[0];
    for (i = 1U; i < sample_cnt; i++) {
        if (pcm[i] < min_v) {
            min_v = pcm[i];
        } else if (pcm[i] > max_v) {
            max_v = pcm[i];
        }
    }

    dump_cnt = sample_cnt;
    if (dump_cnt > ADC_TEST_DUMP_MAX_SAMPLES) {
        dump_cnt = ADC_TEST_DUMP_MAX_SAMPLES;
    }
    for (i = 0U; i < dump_cnt; i++) {
        s_adc_ctx.last_pcm_dump[i] = pcm[i];
    }

    s_adc_ctx.last_pcm_min = min_v;
    s_adc_ctx.last_pcm_max = max_v;
    s_adc_ctx.last_pcm_sample_cnt = sample_cnt;
}

static void test_adc_mix_stereo_to_mono_16(const uint8_t *in_buf, uint32_t in_len, uint8_t *out_buf)
{
    const int16_t *in_pcm = (const int16_t *)in_buf;
    int16_t *out_pcm = (int16_t *)out_buf;
    uint32_t pair_cnt = in_len / 4U;
    uint32_t i;

    for (i = 0U; i < pair_cnt; i++) {
        int32_t l = in_pcm[i * 2U];
        int32_t r = in_pcm[i * 2U + 1U];
        out_pcm[i] = (int16_t)((l + r) / 2);
    }
}

/**
 * @brief Drop one MIC from RAW3 interleaved frame and mix the other two to mono
 * @param[in] in_buf 3ch interleaved PCM [mic1,mic2,mic3,...]
 * @param[in] in_len input bytes (must be multiple of 6)
 * @param[out] out_buf mono PCM
 * @param[in] ref_mic 回采 MIC number 1..3 (dropped)
 * @return none
 */
static void test_adc_mix_raw3_drop_ref_to_mono_16(const uint8_t *in_buf, uint32_t in_len,
                                                   uint8_t *out_buf, uint8_t ref_mic)
{
    const int16_t *in_pcm = (const int16_t *)in_buf;
    int16_t *out_pcm = (int16_t *)out_buf;
    uint32_t frame_pts = in_len / (sizeof(int16_t) * 3U);
    uint32_t ref_idx = (ref_mic >= 1U && ref_mic <= 3U) ? (uint32_t)(ref_mic - 1U) : 0U;
    uint32_t talk0 = (ref_idx == 0U) ? 1U : 0U;
    uint32_t talk1 = (ref_idx == 2U) ? 1U : 2U;
    uint32_t i;

    for (i = 0U; i < frame_pts; i++) {
        int32_t a = in_pcm[i * 3U + talk0];
        int32_t b = in_pcm[i * 3U + talk1];
        out_pcm[i] = (int16_t)((a + b) / 2);
    }
}

static void test_adc_input_cb(TUYA_AUDIO_INPUT_BUFFER_T *buffer)
{
    uint8_t *buf;
    uint32_t buf_len;
    uint32_t mono_len;
    OPERATE_RET ret;

    if ((buffer == NULL) || (buffer->event != TUYA_AUDIO_FRAME_EVENT_ADC_RX)) {
        return;
    }

    buf = buffer->data;
    buf_len = buffer->data_len;

    s_adc_ctx.frame_cnt++;
    s_adc_ctx.total_bytes += buf_len;
    s_adc_ctx.last_frame_bytes = buf_len;
    test_adc_snapshot_pcm(buf, buf_len);

    if (!s_adc_ctx.loop_on || !s_adc_ctx.dac_started) {
        return;
    }

    if (buf == NULL || buf_len == 0U || (buf_len & 0x1U)) {
        s_adc_ctx.loop_write_err_cnt++;
        return;
    }

    if (s_adc_ctx.chan == ADC_TEST_MODE_LR ||
        s_adc_ctx.chan == ADC_TEST_MODE_MIC23) {
        if (buf_len & 0x3U) {
            s_adc_ctx.loop_write_err_cnt++;
            return;
        }
        mono_len = buf_len / 2U;
        if (s_adc_ctx.loop_mix_buf == NULL || mono_len > s_adc_ctx.loop_mix_buf_size) {
            s_adc_ctx.loop_write_err_cnt++;
            return;
        }
        test_adc_mix_stereo_to_mono_16(buf, buf_len, s_adc_ctx.loop_mix_buf);
        ret = tkl_aud_dac_write(s_adc_ctx.dac_port, s_adc_ctx.loop_mix_buf, mono_len);
    } else if (s_adc_ctx.chan == ADC_TEST_MODE_RAW3) {
        /* 3ch interleaved; each sample triple is 6 bytes. */
        if ((buf_len % 6U) != 0U) {
            s_adc_ctx.loop_write_err_cnt++;
            return;
        }
        mono_len = buf_len / 3U;
        if (s_adc_ctx.loop_mix_buf == NULL || mono_len > s_adc_ctx.loop_mix_buf_size) {
            s_adc_ctx.loop_write_err_cnt++;
            return;
        }
        test_adc_mix_raw3_drop_ref_to_mono_16(buf, buf_len, s_adc_ctx.loop_mix_buf,
                                              s_adc_ctx.loop_ref_mic);
        ret = tkl_aud_dac_write(s_adc_ctx.dac_port, s_adc_ctx.loop_mix_buf, mono_len);
    } else {
        mono_len = buf_len;
        ret = tkl_aud_dac_write(s_adc_ctx.dac_port, buf, mono_len);
    }

    if (ret == OPRT_OK) {
        s_adc_ctx.loop_push_bytes += mono_len;
    } else if (ret == OPRT_OS_ADAPTER_DAC_BUSY) {
        s_adc_ctx.loop_busy_cnt++;
        s_adc_ctx.loop_drop_bytes += mono_len;
    } else {
        s_adc_ctx.loop_write_err_cnt++;
        s_adc_ctx.loop_drop_bytes += mono_len;
    }
}

static OPERATE_RET test_adc_stop(void)
{
    OPERATE_RET ret;

    if (!s_adc_ctx.initialized || !s_adc_ctx.started) {
        return OPRT_OK;
    }

    ret = tkl_aud_adc_stop(s_adc_ctx.port);
    if (ret != OPRT_OK) {
        tkl_log_output("adc stop failed: %d\r\n", ret);
        return ret;
    }

    s_adc_ctx.started = false;
    return OPRT_OK;
}


/* --------------------------------------------------------------------------
 * Speaker PA enable (vendor EVB P28).
 *
 * The adapter no longer drives this pin: in production the TAL layer owns it
 * (tal_audio_dac.c). A standalone test that only talks to tkl_aud_dac_* must
 * drive it itself, otherwise the DAC runs but stays silent.
 * ------------------------------------------------------------------------ */
#define TEST_ADC_PA_PIN  ((TUYA_GPIO_NUM_E)28)

static void test_adc_pa_init(void)
{
    TUYA_GPIO_BASE_CFG_T cfg;

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.direct = TUYA_GPIO_OUTPUT;
    cfg.mode   = TUYA_GPIO_PUSH_PULL;
    cfg.level  = TUYA_GPIO_LEVEL_LOW;   /* disabled until playback starts */
    (void)tkl_gpio_init(TEST_ADC_PA_PIN, &cfg);
}

static void test_adc_pa_set(bool enable)
{
    (void)tkl_gpio_write(TEST_ADC_PA_PIN,
                         enable ? TUYA_GPIO_LEVEL_HIGH : TUYA_GPIO_LEVEL_LOW);
}

static void test_adc_pa_deinit(void)
{
    (void)tkl_gpio_deinit(TEST_ADC_PA_PIN);
}

static void test_adc_dac_deinit(void)
{
    if (s_adc_ctx.dac_inited) {
        if (s_adc_ctx.dac_started) {
            test_adc_pa_set(false);
            (void)tkl_aud_dac_stop(s_adc_ctx.dac_port);
        }
        (void)tkl_aud_dac_deinit(s_adc_ctx.dac_port);
    }
    test_adc_pa_deinit();

    if (s_adc_ctx.loop_mix_buf != NULL) {
        tkl_system_free(s_adc_ctx.loop_mix_buf);
        s_adc_ctx.loop_mix_buf = NULL;
    }

    s_adc_ctx.loop_mix_buf_size = 0U;
    s_adc_ctx.loop_on = false;
    s_adc_ctx.dac_inited = false;
    s_adc_ctx.dac_started = false;
    s_adc_ctx.loop_push_bytes = 0U;
    s_adc_ctx.loop_drop_bytes = 0U;
    s_adc_ctx.loop_busy_cnt = 0U;
    s_adc_ctx.loop_write_err_cnt = 0U;
}

static OPERATE_RET test_adc_dac_init_start(uint32_t dac_port, uint32_t sample_rate,
                                           uint32_t frame_time_ms, INT32_T dig_db, INT32_T ana_db,
                                           uint32_t mono_frame_size)
{
    OPERATE_RET ret;
    TUYA_AUDIO_DAC_BASE_CFG_T cfg;

    if (dac_port >= TUYA_AUDIO_DAC_PORT_MAX || sample_rate == 0U || frame_time_ms == 0U) {
        return OPRT_INVALID_PARM;
    }

    if ((mono_frame_size == 0U) || (mono_frame_size & 0x1U)) {
        return OPRT_INVALID_PARM;
    }

    test_adc_dac_deinit();

    if ((s_adc_ctx.chan == ADC_TEST_MODE_LR) ||
        (s_adc_ctx.chan == ADC_TEST_MODE_MIC23) ||
        (s_adc_ctx.chan == ADC_TEST_MODE_RAW3)) {
        s_adc_ctx.loop_mix_buf = (uint8_t *)tkl_system_malloc(mono_frame_size);
        if (s_adc_ctx.loop_mix_buf == NULL) {
            return OPRT_MALLOC_FAILED;
        }
        s_adc_ctx.loop_mix_buf_size = mono_frame_size;
    }

    (void)os_memset(&cfg, 0, sizeof(cfg));
    /* The CLI still takes a DAC port; map it to the channel bitmap here so
     * the port never decides the side inside the driver. */
    cfg.dac_chan = (dac_port == (uint32_t)TUYA_AUDIO_DAC_PORT_0)
                 ? TUYA_AUDIO_DAC_CHANNEL_0
                 : TUYA_AUDIO_DAC_CHANNEL_1;
    cfg.sample_bits = TUYA_AUDIO_SAMPLE_BITS_16;
    cfg.sample_rate = sample_rate;
    cfg.frame_time_ms = frame_time_ms;
    cfg.frame_cb = NULL;
    cfg.args = NULL;

    ret = tkl_aud_dac_init((TUYA_AUDIO_DAC_PORT_E)dac_port, &cfg);
    if (ret != OPRT_OK) {
        test_adc_dac_deinit();
        return ret;
    }

    test_adc_pa_init();
    {
        INT32_T gain[2];

        gain[0] = dig_db;
        gain[1] = ana_db;
        ret = tkl_aud_dac_set_gain((TUYA_AUDIO_DAC_PORT_E)dac_port, TUYA_AUDIO_DAC_CHANNEL_0,
                                   gain, 2U);
        if (ret != OPRT_OK) {
            test_adc_dac_deinit();
            return ret;
        }
    }
    s_adc_ctx.dac_inited = true;
    s_adc_ctx.dac_port = (TUYA_AUDIO_DAC_PORT_E)dac_port;
    s_adc_ctx.dac_dig_db = dig_db;
    s_adc_ctx.dac_ana_db = ana_db;

    ret = tkl_aud_dac_start(s_adc_ctx.dac_port);
    if (ret != OPRT_OK) {
        test_adc_dac_deinit();
        return ret;
    }
    test_adc_pa_set(true);
    s_adc_ctx.dac_started = true;
    return OPRT_OK;
}

static OPERATE_RET test_adc_loop_start(uint32_t adc_port, uint32_t chan, uint32_t dac_port,
                                       uint32_t sample_rate, uint32_t frame_time_ms,
                                       INT32_T dig_db, INT32_T ana_db,
                                       INT32_T adc_dig_db, INT32_T adc_ana_db,
                                       uint32_t ref_mic)
{
    OPERATE_RET ret;
    uint32_t mono_frame_size;

    if ((chan == (uint32_t)ADC_TEST_MODE_RAW3) &&
        ((ref_mic < 1U) || (ref_mic > 3U))) {
        return OPRT_INVALID_PARM;
    }

    ret = test_adc_init(adc_port, chan, sample_rate, frame_time_ms, adc_dig_db, adc_ana_db);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (s_adc_ctx.chan == ADC_TEST_MODE_RAW3) {
        s_adc_ctx.loop_ref_mic = (uint8_t)ref_mic;
    }

    /*
     * Init DAC before ADC start. tkl_aud_dac_init() calls bk_aud_hardware_reset();
     * if ADC is already started, that wipe leaves capture as silence / distortion.
     * ADC start re-applies rate/mode/gain after any shared reset.
     */
    mono_frame_size = s_adc_ctx.frame_size;
    if (s_adc_ctx.chan == ADC_TEST_MODE_LR ||
        s_adc_ctx.chan == ADC_TEST_MODE_MIC23) {
        mono_frame_size = mono_frame_size / 2U;
    } else if (s_adc_ctx.chan == ADC_TEST_MODE_RAW3) {
        /* Drop 回采 lane, mix remaining 2 talk mics -> mono. */
        mono_frame_size = mono_frame_size / 3U;
    }
    ret = test_adc_dac_init_start(dac_port, sample_rate, frame_time_ms, dig_db, ana_db, mono_frame_size);
    if (ret != OPRT_OK) {
        (void)test_adc_deinit();
        return ret;
    }

    ret = test_adc_start();
    if (ret != OPRT_OK) {
        test_adc_dac_deinit();
        (void)test_adc_deinit();
        return ret;
    }

    s_adc_ctx.loop_push_bytes = 0U;
    s_adc_ctx.loop_drop_bytes = 0U;
    s_adc_ctx.loop_busy_cnt = 0U;
    s_adc_ctx.loop_write_err_cnt = 0U;
    s_adc_ctx.loop_on = true;
    return OPRT_OK;
}

static OPERATE_RET test_adc_loop_stop(void)
{
    s_adc_ctx.loop_on = false;
    test_adc_dac_deinit();
    (void)test_adc_stop();
    return test_adc_deinit();
}

static OPERATE_RET test_adc_deinit(void)
{
    s_adc_ctx.loop_on = false;
    test_adc_dac_deinit();

    if (s_adc_ctx.started) {
        (void)test_adc_stop();
    }

    tkl_aud_adc_deinit(s_adc_ctx.port);
    test_adc_runtime_reset();
    return OPRT_OK;
}

static OPERATE_RET test_adc_init(uint32_t port, uint32_t chan, uint32_t sample_rate, uint32_t frame_time_ms,
                                 INT32_T dig_db, INT32_T ana_db)
{
    OPERATE_RET ret;
    TUYA_AUDIO_ADC_BASE_CFG_T cfg;
    uint32_t chan_num;

    if (port >= TUYA_AUDIO_ADC_PORT_MAX || chan >= ADC_TEST_MODE_MAX ||
        sample_rate == 0U || frame_time_ms == 0U) {
        return OPRT_INVALID_PARM;
    }

    chan_num = test_adc_channel_count((ADC_TEST_MODE_E)chan);
    if (chan_num == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (s_adc_ctx.started) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_adc_ctx.initialized) {
        ret = test_adc_deinit();
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.chan = test_adc_chan_to_bitmap((ADC_TEST_MODE_E)chan);
    cfg.sample_bits = ADC_TEST_DEFAULT_SAMPLE_BITS;
    cfg.sample_rate = sample_rate;
    cfg.frame_time_ms = frame_time_ms;
    cfg.upper_cb = test_adc_input_cb;
    cfg.args = NULL;

    ret = tkl_aud_adc_init((TUYA_AUDIO_ADC_PORT_E)port, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("adc init failed: %d\r\n", ret);
        return ret;
    }

    /* Chan must be stored before apply_gain_active selects MIC targets. */
    s_adc_ctx.chan = (ADC_TEST_MODE_E)chan;
    ret = test_adc_apply_gain_active((TUYA_AUDIO_ADC_PORT_E)port, dig_db, ana_db);
    if (ret != OPRT_OK) {
        tkl_log_output("adc set_gain failed: %d\r\n", ret);
        tkl_aud_adc_deinit((TUYA_AUDIO_ADC_PORT_E)port);
        return ret;
    }

    s_adc_ctx.initialized = true;
    s_adc_ctx.started = false;
    s_adc_ctx.port = (TUYA_AUDIO_ADC_PORT_E)port;
    s_adc_ctx.chan = (ADC_TEST_MODE_E)chan;
    s_adc_ctx.sample_rate = sample_rate;
    s_adc_ctx.sample_bits = ADC_TEST_DEFAULT_SAMPLE_BITS;
    s_adc_ctx.frame_time_ms = frame_time_ms;
    s_adc_ctx.dig_db = dig_db;
    s_adc_ctx.ana_db = ana_db;
    s_adc_ctx.frame_size = test_adc_calc_frame_size(sample_rate, ADC_TEST_DEFAULT_SAMPLE_BITS, frame_time_ms, chan_num);
    s_adc_ctx.frame_cnt = 0U;
    s_adc_ctx.total_bytes = 0U;
    s_adc_ctx.last_frame_bytes = 0U;
    s_adc_ctx.last_rec_frames = 0U;
    s_adc_ctx.last_rec_bytes = 0U;
    return OPRT_OK;
}

static OPERATE_RET test_adc_start(void)
{
    OPERATE_RET ret;

    if (!s_adc_ctx.initialized) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_adc_ctx.started) {
        return OPRT_OK;
    }

    ret = tkl_aud_adc_start(s_adc_ctx.port);
    if (ret != OPRT_OK) {
        tkl_log_output("adc start failed: %d\r\n", ret);
        return ret;
    }

    s_adc_ctx.started = true;
    return OPRT_OK;
}

static OPERATE_RET test_adc_record(uint32_t rec_ms)
{
    uint32_t frame_before;
    uint32_t bytes_before;

    if (!s_adc_ctx.initialized || !s_adc_ctx.started || rec_ms == 0U) {
        return OPRT_INVALID_PARM;
    }

    frame_before = s_adc_ctx.frame_cnt;
    bytes_before = s_adc_ctx.total_bytes;
    tkl_system_sleep(rec_ms);
    s_adc_ctx.last_rec_frames = s_adc_ctx.frame_cnt - frame_before;
    s_adc_ctx.last_rec_bytes = s_adc_ctx.total_bytes - bytes_before;
    return OPRT_OK;
}

static OPERATE_RET test_adc_set_gain(INT32_T dig_db, INT32_T ana_db)
{
    OPERATE_RET ret;

    if (!s_adc_ctx.initialized) {
        return OPRT_INVALID_PARM;
    }

    ret = test_adc_apply_gain_active(s_adc_ctx.port, dig_db, ana_db);
    if (ret == OPRT_OK) {
        s_adc_ctx.dig_db = dig_db;
        s_adc_ctx.ana_db = ana_db;
    }
    return ret;
}

/**
 * @brief Set gain for one physical MIC (0=MIC1, 1=MIC2, 2=MIC3)
 * @param[in] mic_ch physical MIC channel
 * @param[in] dig_db digital gain, or AUDIO_ADC_INVALID_GAIN_VALUE to keep
 * @param[in] ana_db analog gain, or AUDIO_ADC_INVALID_GAIN_VALUE to keep
 * @return OPRT_OK on success
 */
static OPERATE_RET test_adc_set_gain_mic(UINT32_T mic_ch, INT32_T dig_db, INT32_T ana_db)
{
    INT32_T gain[2];

    if (!s_adc_ctx.initialized || (mic_ch >= 3U)) {
        return OPRT_INVALID_PARM;
    }

    gain[0] = dig_db;
    gain[1] = ana_db;
    return tkl_aud_adc_set_gain(s_adc_ctx.port, test_adc_lane_bit(mic_ch), gain, ADC_TEST_GAIN_NUM);
}

static void test_adc_dump_last_pcm(char *pcWriteBuffer, int xWriteBufferLen, uint32_t req_cnt)
{
    uint32_t dump_cnt;
    int used;
    int n;
    uint32_t i;

    if (s_adc_ctx.last_pcm_sample_cnt == 0U) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc dump no_frame\r\n");
        return;
    }

    dump_cnt = req_cnt;
    if (dump_cnt == 0U) {
        dump_cnt = ADC_TEST_DUMP_DEFAULT_SAMPLES;
    }
    if (dump_cnt > ADC_TEST_DUMP_MAX_SAMPLES) {
        dump_cnt = ADC_TEST_DUMP_MAX_SAMPLES;
    }
    if (dump_cnt > s_adc_ctx.last_pcm_sample_cnt) {
        dump_cnt = s_adc_ctx.last_pcm_sample_cnt;
    }

    used = snprintf(pcWriteBuffer, xWriteBufferLen,
                    "xadc dump frame_cnt=%u last_frame=%u sample_cnt=%u min=%d max=%d data=",
                    (unsigned int)s_adc_ctx.frame_cnt,
                    (unsigned int)s_adc_ctx.last_frame_bytes,
                    (unsigned int)s_adc_ctx.last_pcm_sample_cnt,
                    (int)s_adc_ctx.last_pcm_min,
                    (int)s_adc_ctx.last_pcm_max);
    if (used < 0 || used >= xWriteBufferLen) {
        return;
    }

    for (i = 0U; i < dump_cnt; i++) {
        n = snprintf(pcWriteBuffer + used, xWriteBufferLen - used,
                     "%s%d",
                     (i == 0U) ? "[" : ",",
                     (int)s_adc_ctx.last_pcm_dump[i]);
        if (n < 0 || n >= (xWriteBufferLen - used)) {
            return;
        }
        used += n;
    }

    (void)snprintf(pcWriteBuffer + used, xWriteBufferLen - used, "]\r\n");
}

/**
 * @brief Parse signed decimal from CLI argv
 * @param[in] s input string
 * @return INT32_T value
 */
static INT32_T test_parse_s32(const char *s)
{
    int neg = 0;
    UINT32_T v;

    if ((s == NULL) || (s[0] == '\0')) {
        return 0;
    }
    if (s[0] == '-') {
        neg = 1;
        s++;
    }
    v = os_strtoul(s, NULL, 10);
    return neg ? -(INT32_T)v : (INT32_T)v;
}

void cli_adc_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    uint32_t port = ADC_TEST_DEFAULT_PORT;
    uint32_t chan = ADC_TEST_DEFAULT_CHAN;
    uint32_t sample_rate = ADC_TEST_DEFAULT_SAMPLE_RATE;
    uint32_t frame_time_ms = ADC_TEST_DEFAULT_FRAME_TIME_MS;
    INT32_T dig_db = ADC_TEST_DEFAULT_DIG_DB;
    INT32_T ana_db = ADC_TEST_DEFAULT_ANA_DB;
    uint32_t rec_ms = ADC_TEST_DEFAULT_REC_MS;
    uint32_t dac_port = ADC_TEST_DEFAULT_DAC_PORT;
    INT32_T dac_dig_db = ADC_TEST_DEFAULT_DAC_DIG_DB;
    INT32_T dac_ana_db = ADC_TEST_DEFAULT_DAC_ANA_DB;
    uint32_t ref_mic = ADC_TEST_DEFAULT_LOOP_REF_MIC;
    uint32_t dump_cnt = ADC_TEST_DUMP_DEFAULT_SAMPLES;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xadc <init [port] [chan 0=L 1=R 2=LR 3=MIC3 4=RAW3 5=MIC23] [rate] [frame_ms] [dig_db] [ana_db]|start|rec [ms]|gain <dig_db> [ana_db]|loop_start [adc_port] [chan] [dac_port] [rate] [frame_ms] [dac_dig] [dac_ana] [adc_dig] [adc_ana] [ref_mic 1..3 for RAW3]|loop_stop|stop|deinit|close|status|stat|dump [n]>");
        return;
    }

    cmd = argv[1];
    if (os_strcmp(cmd, "init") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            chan = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            sample_rate = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            frame_time_ms = os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            dig_db = test_parse_s32(argv[6]);
        }
        if (argc >= 8) {
            ana_db = test_parse_s32(argv[7]);
        }

        ret = test_adc_init(port, chan, sample_rate, frame_time_ms, dig_db, ana_db);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc init %s (%d), port=%u chan=%u rate=%u frame_ms=%u dig=%d ana=%d frame_size=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port,
                 (unsigned int)chan,
                 (unsigned int)sample_rate,
                 (unsigned int)frame_time_ms,
                 (int)dig_db,
                 (int)ana_db,
                 (unsigned int)s_adc_ctx.frame_size);
    } else if (os_strcmp(cmd, "start") == 0) {
        ret = test_adc_start();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc start %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "rec") == 0 || os_strcmp(cmd, "record") == 0) {
        if (argc >= 3) {
            rec_ms = os_strtoul(argv[2], NULL, 10);
        }

        ret = test_adc_record(rec_ms);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc rec %s (%d), ms=%u frames=%u bytes=%u total_frames=%u total_bytes=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)rec_ms,
                 (unsigned int)s_adc_ctx.last_rec_frames,
                 (unsigned int)s_adc_ctx.last_rec_bytes,
                 (unsigned int)s_adc_ctx.frame_cnt,
                 (unsigned int)s_adc_ctx.total_bytes);
    } else if (os_strcmp(cmd, "gain") == 0) {
        UINT32_T mic_ch = 0xFFU;
        BOOL_T dig_keep = FALSE;
        BOOL_T ana_keep = FALSE;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xadc gain <dig_db|x> [ana_db|x] [mic 0|1|2]\r\n"
                     "  x = keep (AUDIO_ADC_INVALID_GAIN_VALUE); mic omit = all active MICs\r\n");
            return;
        }
        if ((argv[2][0] == 'x' || argv[2][0] == 'X') && argv[2][1] == '\0') {
            dig_keep = TRUE;
            dig_db = (INT32_T)AUDIO_ADC_INVALID_GAIN_VALUE;
        } else {
            dig_db = test_parse_s32(argv[2]);
        }
        if (argc >= 4) {
            if ((argv[3][0] == 'x' || argv[3][0] == 'X') && argv[3][1] == '\0') {
                ana_keep = TRUE;
                ana_db = (INT32_T)AUDIO_ADC_INVALID_GAIN_VALUE;
            } else {
                ana_db = test_parse_s32(argv[3]);
            }
        } else {
            ana_keep = TRUE;
            ana_db = (INT32_T)AUDIO_ADC_INVALID_GAIN_VALUE;
        }
        if (argc >= 5) {
            mic_ch = os_strtoul(argv[4], NULL, 10);
        }

        if (mic_ch != 0xFFU) {
            ret = test_adc_set_gain_mic(mic_ch, dig_db, ana_db);
        } else if ((dig_keep == TRUE) || (ana_keep == TRUE)) {
            /* Partial update (one side is "keep") on all active MICs. Reuse the
             * shared helper so the lane set comes from the mode's bitmap - the
             * old inline copy here looped lanes 0..2 unconditionally, which the
             * capture core rejects for L/R/LR/MIC3 (they own only some lanes). */
            ret = test_adc_apply_gain_active(s_adc_ctx.port, dig_db, ana_db);
        } else {
            ret = test_adc_set_gain(dig_db, ana_db);
        }
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc gain %s (%d), dig=%s%d ana=%s%d mic=%s\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (dig_keep == TRUE) ? "keep/" : "",
                 (int)((dig_keep == TRUE) ? 0 : dig_db),
                 (ana_keep == TRUE) ? "keep/" : "",
                 (int)((ana_keep == TRUE) ? 0 : ana_db),
                 (mic_ch == 0xFFU) ? "all" : "one");
    } else if (os_strcmp(cmd, "loop_start") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            chan = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            dac_port = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            sample_rate = os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            frame_time_ms = os_strtoul(argv[6], NULL, 10);
        }
        if (argc >= 8) {
            dac_dig_db = test_parse_s32(argv[7]);
        }
        if (argc >= 9) {
            dac_ana_db = test_parse_s32(argv[8]);
        }
        if (argc >= 10) {
            dig_db = test_parse_s32(argv[9]);
        }
        if (argc >= 11) {
            ana_db = test_parse_s32(argv[10]);
        }
        if (argc >= 12) {
            ref_mic = os_strtoul(argv[11], NULL, 10);
        }

        ret = test_adc_loop_start(port, chan, dac_port, sample_rate, frame_time_ms,
                                  dac_dig_db, dac_ana_db, dig_db, ana_db, ref_mic);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc loop_start %s (%d), adc_port=%u chan=%u dac_port=%u rate=%u frame_ms=%u dac_dig=%d dac_ana=%d adc_dig=%d adc_ana=%d ref_mic=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port,
                 (unsigned int)chan,
                 (unsigned int)dac_port,
                 (unsigned int)sample_rate,
                 (unsigned int)frame_time_ms,
                 (int)dac_dig_db,
                 (int)dac_ana_db,
                 (int)dig_db,
                 (int)ana_db,
                 (unsigned int)((chan == (uint32_t)ADC_TEST_MODE_RAW3) ?
                                ref_mic : 0U));
    } else if (os_strcmp(cmd, "loop_stop") == 0) {
        ret = test_adc_loop_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc loop_stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "stop") == 0) {
        ret = test_adc_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "deinit") == 0 || os_strcmp(cmd, "close") == 0) {
        ret = test_adc_deinit();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc %s %s (%d)\r\n",
                 cmd,
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "status") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc status init=%u start=%u port=%u chan=%u rate=%u bits=%u frame_ms=%u dig=%d ana=%d frame_size=%u frame_cnt=%u total_bytes=%u last_frame=%u loop_on=%u ref_mic=%u dac_inited=%u dac_started=%u dac_port=%u dac_dig=%d dac_ana=%d loop_push=%u loop_drop=%u loop_busy=%u loop_err=%u\r\n",
                 (unsigned int)s_adc_ctx.initialized,
                 (unsigned int)s_adc_ctx.started,
                 (unsigned int)s_adc_ctx.port,
                 (unsigned int)s_adc_ctx.chan,
                 (unsigned int)s_adc_ctx.sample_rate,
                 (unsigned int)s_adc_ctx.sample_bits,
                 (unsigned int)s_adc_ctx.frame_time_ms,
                 (int)s_adc_ctx.dig_db,
                 (int)s_adc_ctx.ana_db,
                 (unsigned int)s_adc_ctx.frame_size,
                 (unsigned int)s_adc_ctx.frame_cnt,
                 (unsigned int)s_adc_ctx.total_bytes,
                 (unsigned int)s_adc_ctx.last_frame_bytes,
                 (unsigned int)s_adc_ctx.loop_on,
                 (unsigned int)s_adc_ctx.loop_ref_mic,
                 (unsigned int)s_adc_ctx.dac_inited,
                 (unsigned int)s_adc_ctx.dac_started,
                 (unsigned int)s_adc_ctx.dac_port,
                 (int)s_adc_ctx.dac_dig_db,
                 (int)s_adc_ctx.dac_ana_db,
                 (unsigned int)s_adc_ctx.loop_push_bytes,
                 (unsigned int)s_adc_ctx.loop_drop_bytes,
                 (unsigned int)s_adc_ctx.loop_busy_cnt,
                 (unsigned int)s_adc_ctx.loop_write_err_cnt);
    } else if (os_strcmp(cmd, "stat") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc stat init=%u start=%u fc=%u bytes=%u lf=%u loop=%u push=%u drop=%u busy=%u err=%u min=%d max=%d\r\n",
                 (unsigned int)s_adc_ctx.initialized,
                 (unsigned int)s_adc_ctx.started,
                 (unsigned int)s_adc_ctx.frame_cnt,
                 (unsigned int)s_adc_ctx.total_bytes,
                 (unsigned int)s_adc_ctx.last_frame_bytes,
                 (unsigned int)s_adc_ctx.loop_on,
                 (unsigned int)s_adc_ctx.loop_push_bytes,
                 (unsigned int)s_adc_ctx.loop_drop_bytes,
                 (unsigned int)s_adc_ctx.loop_busy_cnt,
                 (unsigned int)s_adc_ctx.loop_write_err_cnt,
                 (int)s_adc_ctx.last_pcm_min,
                 (int)s_adc_ctx.last_pcm_max);
    } else if (os_strcmp(cmd, "dump") == 0) {
        if (argc >= 3) {
            dump_cnt = os_strtoul(argv[2], NULL, 10);
        }
        test_adc_dump_last_pcm(pcWriteBuffer, xWriteBufferLen, dump_cnt);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xadc <init [port] [chan 0=L 1=R 2=LR 3=MIC3 4=RAW3 5=MIC23] [rate] [frame_ms] [dig_db] [ana_db]|start|rec [ms]|gain <dig_db> [ana_db]|loop_start [adc_port] [chan] [dac_port] [rate] [frame_ms] [dac_dig] [dac_ana] [adc_dig] [adc_ana] [ref_mic 1..3 for RAW3]|loop_stop|stop|deinit|close|status|stat|dump [n]>");
    }
}
