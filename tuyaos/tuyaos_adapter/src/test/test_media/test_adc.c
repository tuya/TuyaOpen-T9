#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/mem.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_aud_adc.h"
#include "tkl_aud_dac.h"
#include "tkl_output.h"
#include "tkl_system.h"

#define ADC_TEST_DEFAULT_PORT          TUYA_AUDIO_ADC_PORT_0
#define ADC_TEST_DEFAULT_CHAN          TUYA_AUDIO_ADC_CHANNEL_L
#define ADC_TEST_DEFAULT_SAMPLE_RATE   16000U
#define ADC_TEST_DEFAULT_SAMPLE_BITS   TUYA_AUDIO_SAMPLE_BITS_16
#define ADC_TEST_DEFAULT_FRAME_TIME_MS 20U
#define ADC_TEST_DEFAULT_GAIN          0x2DU
#define ADC_TEST_DEFAULT_REC_MS        2000U
#define ADC_TEST_DEFAULT_DAC_PORT      TUYA_AUDIO_DAC_PORT_0
#define ADC_TEST_DEFAULT_DAC_DIG_DB    (-7)
#define ADC_TEST_DEFAULT_DAC_ANA_DB    (4)
#define ADC_TEST_DUMP_MAX_SAMPLES      32U
#define ADC_TEST_DUMP_DEFAULT_SAMPLES  12U

typedef struct {
    bool initialized;
    bool started;
    TUYA_AUDIO_ADC_PORT_E port;
    TUYA_AUDIO_ADC_CHAN_E chan;
    uint32_t sample_rate;
    uint8_t sample_bits;
    uint32_t frame_time_ms;
    uint8_t gain;
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
    .gain = ADC_TEST_DEFAULT_GAIN,
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

static OPERATE_RET test_adc_init(uint32_t port, uint32_t chan, uint32_t sample_rate, uint32_t frame_time_ms, uint32_t gain);
static OPERATE_RET test_adc_start(void);
static OPERATE_RET test_adc_deinit(void);

static uint32_t test_adc_channel_count(TUYA_AUDIO_ADC_CHAN_E chan)
{
    if (chan == TUYA_AUDIO_ADC_CHANNEL_L || chan == TUYA_AUDIO_ADC_CHANNEL_R) {
        return 1U;
    }
    if (chan == TUYA_AUDIO_ADC_CHANNEL_LR) {
        return 2U;
    }
    return 0U;
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
    s_adc_ctx.gain = ADC_TEST_DEFAULT_GAIN;
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

static void test_adc_input_cb(TUYA_AUDIO_FRAME_EVT_E event, uint8_t *buf, uint32_t buf_len, void *args)
{
    uint32_t mono_len;
    OPERATE_RET ret;

    (void)args;

    if (event != TUYA_AUDIO_FRAME_EVENT_ADC_RX) {
        return;
    }

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

    if (s_adc_ctx.chan == TUYA_AUDIO_ADC_CHANNEL_LR) {
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

static void test_adc_dac_deinit(void)
{
    if (s_adc_ctx.dac_inited) {
        if (s_adc_ctx.dac_started) {
            (void)tkl_aud_dac_stop(s_adc_ctx.dac_port);
        }
        (void)tkl_aud_dac_deinit(s_adc_ctx.dac_port);
    }

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
    TKL_AUD_DAC_CFG_T cfg;

    if (dac_port >= TUYA_AUDIO_DAC_PORT_MAX || sample_rate == 0U || frame_time_ms == 0U) {
        return OPRT_INVALID_PARM;
    }

    if ((mono_frame_size == 0U) || (mono_frame_size & 0x1U)) {
        return OPRT_INVALID_PARM;
    }

    test_adc_dac_deinit();

    if (s_adc_ctx.chan == TUYA_AUDIO_ADC_CHANNEL_LR) {
        s_adc_ctx.loop_mix_buf = (uint8_t *)tkl_system_malloc(mono_frame_size);
        if (s_adc_ctx.loop_mix_buf == NULL) {
            return OPRT_MALLOC_FAILED;
        }
        s_adc_ctx.loop_mix_buf_size = mono_frame_size;
    }

    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.gain[0] = dig_db;
    cfg.gain[1] = ana_db;
    cfg.gain_num = 2U;
    cfg.chan_num = 1U;
    cfg.sample_bits = TUYA_AUDIO_SAMPLE_BITS_16;
    cfg.sample_rate = sample_rate;
    cfg.frame_time_ms = frame_time_ms;
    cfg.frame_cb = NULL;
    cfg.args = NULL;
    cfg.pa_pin = 60;

    ret = tkl_aud_dac_init((TUYA_AUDIO_DAC_PORT_E)dac_port, &cfg);
    if (ret != OPRT_OK) {
        test_adc_dac_deinit();
        return ret;
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
    s_adc_ctx.dac_started = true;
    return OPRT_OK;
}

static OPERATE_RET test_adc_loop_start(uint32_t adc_port, uint32_t chan, uint32_t dac_port,
                                       uint32_t sample_rate, uint32_t frame_time_ms,
                                       INT32_T dig_db, INT32_T ana_db, uint32_t adc_gain)
{
    OPERATE_RET ret;
    uint32_t mono_frame_size;

    ret = test_adc_init(adc_port, chan, sample_rate, frame_time_ms, adc_gain);
    if (ret != OPRT_OK) {
        return ret;
    }

    /*
     * Init DAC before ADC start. tkl_aud_dac_init() calls bk_aud_hardware_reset();
     * if ADC is already started, that wipe leaves capture as silence / distortion.
     * ADC start re-applies rate/mode/gain after any shared reset.
     */
    mono_frame_size = s_adc_ctx.frame_size;
    if (s_adc_ctx.chan == TUYA_AUDIO_ADC_CHANNEL_LR) {
        mono_frame_size = mono_frame_size / 2U;
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

static OPERATE_RET test_adc_init(uint32_t port, uint32_t chan, uint32_t sample_rate, uint32_t frame_time_ms, uint32_t gain)
{
    OPERATE_RET ret;
    TKL_AUD_ADC_CFG_T cfg;
    uint32_t chan_num;

    if (port >= TUYA_AUDIO_ADC_PORT_MAX || chan >= TUYA_AUDIO_ADC_CHANNEL_MAX ||
        sample_rate == 0U || frame_time_ms == 0U || gain > 0x3FU) {
        return OPRT_INVALID_PARM;
    }

    chan_num = test_adc_channel_count((TUYA_AUDIO_ADC_CHAN_E)chan);
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
    cfg.chan = (TUYA_AUDIO_ADC_CHAN_E)chan;
    cfg.vol = (uint8_t)gain;
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

    s_adc_ctx.initialized = true;
    s_adc_ctx.started = false;
    s_adc_ctx.port = (TUYA_AUDIO_ADC_PORT_E)port;
    s_adc_ctx.chan = (TUYA_AUDIO_ADC_CHAN_E)chan;
    s_adc_ctx.sample_rate = sample_rate;
    s_adc_ctx.sample_bits = ADC_TEST_DEFAULT_SAMPLE_BITS;
    s_adc_ctx.frame_time_ms = frame_time_ms;
    s_adc_ctx.gain = (uint8_t)gain;
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

static OPERATE_RET test_adc_set_gain(uint32_t gain)
{
    OPERATE_RET ret;

    if (!s_adc_ctx.initialized || gain > 0x3FU) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_aud_adc_set_vol(s_adc_ctx.port, gain);
    if (ret == OPRT_OK) {
        s_adc_ctx.gain = (uint8_t)gain;
    }
    return ret;
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
    uint32_t gain = ADC_TEST_DEFAULT_GAIN;
    uint32_t rec_ms = ADC_TEST_DEFAULT_REC_MS;
    uint32_t dac_port = ADC_TEST_DEFAULT_DAC_PORT;
    INT32_T dig_db = ADC_TEST_DEFAULT_DAC_DIG_DB;
    INT32_T ana_db = ADC_TEST_DEFAULT_DAC_ANA_DB;
    uint32_t dump_cnt = ADC_TEST_DUMP_DEFAULT_SAMPLES;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xadc <init [port] [chan] [rate] [frame_ms] [gain]|start|rec [ms]|vol [gain]|loop_start [adc_port] [chan] [dac_port] [rate] [frame_ms] [dig_db] [ana_db] [adc_gain]|loop_stop|stop|deinit|close|status|stat|dump [n]>");
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
            gain = os_strtoul(argv[6], NULL, 10);
        }

        ret = test_adc_init(port, chan, sample_rate, frame_time_ms, gain);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc init %s (%d), port=%u chan=%u rate=%u frame_ms=%u gain=%u frame_size=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port,
                 (unsigned int)chan,
                 (unsigned int)sample_rate,
                 (unsigned int)frame_time_ms,
                 (unsigned int)gain,
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
    } else if (os_strcmp(cmd, "vol") == 0 || os_strcmp(cmd, "gain") == 0) {
        if (argc >= 3) {
            gain = os_strtoul(argv[2], NULL, 10);
        }

        ret = test_adc_set_gain(gain);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc vol %s (%d), gain=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)gain);
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
            dig_db = test_parse_s32(argv[7]);
        }
        if (argc >= 9) {
            ana_db = test_parse_s32(argv[8]);
        }
        if (argc >= 10) {
            gain = os_strtoul(argv[9], NULL, 10);
        }

        ret = test_adc_loop_start(port, chan, dac_port, sample_rate, frame_time_ms, dig_db, ana_db, gain);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xadc loop_start %s (%d), adc_port=%u chan=%u dac_port=%u rate=%u frame_ms=%u dig=%d ana=%d adc_gain=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port,
                 (unsigned int)chan,
                 (unsigned int)dac_port,
                 (unsigned int)sample_rate,
                 (unsigned int)frame_time_ms,
                 (int)dig_db,
                 (int)ana_db,
                 (unsigned int)gain);
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
                 "xadc status init=%u start=%u port=%u chan=%u rate=%u bits=%u frame_ms=%u gain=%u frame_size=%u frame_cnt=%u total_bytes=%u last_frame=%u loop_on=%u dac_inited=%u dac_started=%u dac_port=%u dig=%d ana=%d loop_push=%u loop_drop=%u loop_busy=%u loop_err=%u\r\n",
                 (unsigned int)s_adc_ctx.initialized,
                 (unsigned int)s_adc_ctx.started,
                 (unsigned int)s_adc_ctx.port,
                 (unsigned int)s_adc_ctx.chan,
                 (unsigned int)s_adc_ctx.sample_rate,
                 (unsigned int)s_adc_ctx.sample_bits,
                 (unsigned int)s_adc_ctx.frame_time_ms,
                 (unsigned int)s_adc_ctx.gain,
                 (unsigned int)s_adc_ctx.frame_size,
                 (unsigned int)s_adc_ctx.frame_cnt,
                 (unsigned int)s_adc_ctx.total_bytes,
                 (unsigned int)s_adc_ctx.last_frame_bytes,
                 (unsigned int)s_adc_ctx.loop_on,
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
                 "Usage: xadc <init [port] [chan] [rate] [frame_ms] [gain]|start|rec [ms]|vol [gain]|loop_start [adc_port] [chan] [dac_port] [rate] [frame_ms] [dig_db] [ana_db] [adc_gain]|loop_stop|stop|deinit|close|status|stat|dump [n]>");
    }
}
