#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/mem.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_aud_dac.h"
#include "tkl_aud_dmic.h"
#include "tkl_output.h"
#include "tkl_system.h"

#define DMIC_TEST_DEFAULT_PORT          TUYA_AUDIO_DMIC_PORT_0
#define DMIC_TEST_DEFAULT_CHAN          TUYA_AUDIO_DMIC_CHANNEL_LR
#define DMIC_TEST_DEFAULT_SAMPLE_RATE   16000U
#define DMIC_TEST_DEFAULT_SAMPLE_BITS   TUYA_AUDIO_SAMPLE_BITS_16
#define DMIC_TEST_DEFAULT_FRAME_TIME_MS 20U
#define DMIC_TEST_DEFAULT_DIG_DB        (16)     /* OEM DEFAULT dig_gain */
#define DMIC_TEST_DEFAULT_ANA_DB        (20)     /* OEM DEFAULT ana_gain */
#define DMIC_TEST_GAIN_NUM              (2U)
#define DMIC_TEST_DEFAULT_REC_MS        2000U
#define DMIC_TEST_NO_GPIO               ((uint32_t)0xFFFFFFFF)
#define DMIC_TEST_DEFAULT_DAC_PORT      TUYA_AUDIO_DAC_PORT_0
#define DMIC_TEST_DEFAULT_DAC_DIG_DB    (-7)
#define DMIC_TEST_DEFAULT_DAC_ANA_DB    (4)

typedef struct {
    bool initialized;
    bool started;
    TUYA_AUDIO_DMIC_PORT_E port;
    TUYA_AUDIO_DMIC_CHAN_E chan;
    uint32_t sample_rate;
    uint8_t sample_bits;
    uint32_t frame_time_ms;
    INT32_T dig_db;
    INT32_T ana_db;
    uint32_t frame_size;
    bool pinmux_set;
    uint32_t clk_gpio;
    uint32_t data_gpio;
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
} test_dmic_ctx_t;

static test_dmic_ctx_t s_dmic_ctx = {
    .initialized = false,
    .started = false,
    .port = DMIC_TEST_DEFAULT_PORT,
    .chan = DMIC_TEST_DEFAULT_CHAN,
    .sample_rate = DMIC_TEST_DEFAULT_SAMPLE_RATE,
    .sample_bits = DMIC_TEST_DEFAULT_SAMPLE_BITS,
    .frame_time_ms = DMIC_TEST_DEFAULT_FRAME_TIME_MS,
    .dig_db = DMIC_TEST_DEFAULT_DIG_DB,
    .ana_db = DMIC_TEST_DEFAULT_ANA_DB,
    .frame_size = 0U,
    .pinmux_set = false,
    .clk_gpio = DMIC_TEST_NO_GPIO,
    .data_gpio = DMIC_TEST_NO_GPIO,
    .frame_cnt = 0U,
    .total_bytes = 0U,
    .last_frame_bytes = 0U,
    .last_rec_frames = 0U,
    .last_rec_bytes = 0U,
    .loop_on = false,
    .dac_inited = false,
    .dac_started = false,
    .dac_port = DMIC_TEST_DEFAULT_DAC_PORT,
    .dac_dig_db = DMIC_TEST_DEFAULT_DAC_DIG_DB,
    .dac_ana_db = DMIC_TEST_DEFAULT_DAC_ANA_DB,
    .loop_mix_buf = NULL,
    .loop_mix_buf_size = 0U,
    .loop_push_bytes = 0U,
    .loop_drop_bytes = 0U,
    .loop_busy_cnt = 0U,
    .loop_write_err_cnt = 0U,
};

static OPERATE_RET test_dmic_init(uint32_t port, uint32_t sample_rate, uint32_t frame_time_ms,
                                  uint32_t clk_gpio, uint32_t data_gpio, bool use_pinmux);
static OPERATE_RET test_dmic_deinit(void);
static OPERATE_RET test_dmic_start(void);

static uint32_t test_dmic_calc_frame_size(uint32_t sample_rate, uint8_t sample_bits, uint32_t frame_ms)
{
    if (sample_rate == 0U || sample_bits == 0U || frame_ms == 0U) {
        return 0U;
    }

    return ((sample_rate * (sample_bits / 8U) * 2U) / 1000U) * frame_ms;
}

static void test_dmic_runtime_reset(void)
{
    s_dmic_ctx.initialized = false;
    s_dmic_ctx.started = false;
    s_dmic_ctx.port = DMIC_TEST_DEFAULT_PORT;
    s_dmic_ctx.chan = DMIC_TEST_DEFAULT_CHAN;
    s_dmic_ctx.sample_rate = DMIC_TEST_DEFAULT_SAMPLE_RATE;
    s_dmic_ctx.sample_bits = DMIC_TEST_DEFAULT_SAMPLE_BITS;
    s_dmic_ctx.frame_time_ms = DMIC_TEST_DEFAULT_FRAME_TIME_MS;
    s_dmic_ctx.dig_db = DMIC_TEST_DEFAULT_DIG_DB;
    s_dmic_ctx.ana_db = DMIC_TEST_DEFAULT_ANA_DB;
    s_dmic_ctx.frame_size = 0U;
    s_dmic_ctx.pinmux_set = false;
    s_dmic_ctx.clk_gpio = DMIC_TEST_NO_GPIO;
    s_dmic_ctx.data_gpio = DMIC_TEST_NO_GPIO;
    s_dmic_ctx.frame_cnt = 0U;
    s_dmic_ctx.total_bytes = 0U;
    s_dmic_ctx.last_frame_bytes = 0U;
    s_dmic_ctx.last_rec_frames = 0U;
    s_dmic_ctx.last_rec_bytes = 0U;
    s_dmic_ctx.loop_on = false;
    s_dmic_ctx.dac_inited = false;
    s_dmic_ctx.dac_started = false;
    s_dmic_ctx.dac_port = DMIC_TEST_DEFAULT_DAC_PORT;
    s_dmic_ctx.dac_dig_db = DMIC_TEST_DEFAULT_DAC_DIG_DB;
    s_dmic_ctx.dac_ana_db = DMIC_TEST_DEFAULT_DAC_ANA_DB;
    s_dmic_ctx.loop_mix_buf = NULL;
    s_dmic_ctx.loop_mix_buf_size = 0U;
    s_dmic_ctx.loop_push_bytes = 0U;
    s_dmic_ctx.loop_drop_bytes = 0U;
    s_dmic_ctx.loop_busy_cnt = 0U;
    s_dmic_ctx.loop_write_err_cnt = 0U;
}

static void test_dmic_loop_mix_stereo_to_mono_16(const uint8_t *in_buf, uint32_t in_len, uint8_t *out_buf)
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

static void test_dmic_input_cb(TUYA_AUDIO_FRAME_EVT_E event, uint8_t *buf, uint32_t buf_len, void *args)
{
    uint32_t mono_len;
    OPERATE_RET ret;

    (void)buf;
    (void)args;

    if (event != TUYA_AUDIO_FRAME_EVENT_DMIC_RX) {
        return;
    }

    s_dmic_ctx.frame_cnt++;
    s_dmic_ctx.total_bytes += buf_len;
    s_dmic_ctx.last_frame_bytes = buf_len;

    if (!s_dmic_ctx.loop_on || !s_dmic_ctx.dac_started || s_dmic_ctx.loop_mix_buf == NULL) {
        return;
    }

    if ((buf == NULL) || (buf_len == 0U) || ((buf_len & 0x3U) != 0U)) {
        s_dmic_ctx.loop_write_err_cnt++;
        return;
    }

    if ((buf_len / 2U) > s_dmic_ctx.loop_mix_buf_size) {
        s_dmic_ctx.loop_write_err_cnt++;
        return;
    }

    test_dmic_loop_mix_stereo_to_mono_16(buf, buf_len, s_dmic_ctx.loop_mix_buf);
    mono_len = buf_len / 2U;
    ret = tkl_aud_dac_write(s_dmic_ctx.dac_port, s_dmic_ctx.loop_mix_buf, mono_len);
    if (ret == OPRT_OK) {
        s_dmic_ctx.loop_push_bytes += mono_len;
    } else if (ret == OPRT_OS_ADAPTER_DAC_BUSY) {
        s_dmic_ctx.loop_busy_cnt++;
        s_dmic_ctx.loop_drop_bytes += mono_len;
    } else {
        s_dmic_ctx.loop_write_err_cnt++;
        s_dmic_ctx.loop_drop_bytes += mono_len;
    }
}

static OPERATE_RET test_dmic_stop(void)
{
    OPERATE_RET ret;

    if (!s_dmic_ctx.initialized || !s_dmic_ctx.started) {
        return OPRT_OK;
    }

    ret = tkl_aud_dmic_stop(s_dmic_ctx.port);
    if (ret != OPRT_OK) {
        tkl_log_output("dmic stop failed: %d\r\n", ret);
        return ret;
    }

    s_dmic_ctx.started = false;
    return OPRT_OK;
}

static void test_dmic_dac_deinit(void)
{
    if (s_dmic_ctx.dac_inited) {
        if (s_dmic_ctx.dac_started) {
            (void)tkl_aud_dac_stop(s_dmic_ctx.dac_port);
        }
        (void)tkl_aud_dac_deinit(s_dmic_ctx.dac_port);
    }

    if (s_dmic_ctx.loop_mix_buf != NULL) {
        tkl_system_free(s_dmic_ctx.loop_mix_buf);
        s_dmic_ctx.loop_mix_buf = NULL;
    }

    s_dmic_ctx.loop_mix_buf_size = 0U;
    s_dmic_ctx.loop_on = false;
    s_dmic_ctx.dac_inited = false;
    s_dmic_ctx.dac_started = false;
    s_dmic_ctx.loop_push_bytes = 0U;
    s_dmic_ctx.loop_drop_bytes = 0U;
    s_dmic_ctx.loop_busy_cnt = 0U;
    s_dmic_ctx.loop_write_err_cnt = 0U;
}

static OPERATE_RET test_dmic_dac_init_start(uint32_t dac_port, uint32_t sample_rate,
                                            uint32_t frame_time_ms, INT32_T dig_db, INT32_T ana_db,
                                            uint32_t mono_frame_size)
{
    OPERATE_RET ret;
    TKL_AUD_DAC_CFG_T cfg;

    if (dac_port >= TUYA_AUDIO_DAC_PORT_MAX || sample_rate == 0U || frame_time_ms == 0U) {
        return OPRT_INVALID_PARM;
    }

    if ((mono_frame_size == 0U) || ((mono_frame_size & 0x1U) != 0U)) {
        return OPRT_INVALID_PARM;
    }

    test_dmic_dac_deinit();

    s_dmic_ctx.loop_mix_buf = (uint8_t *)tkl_system_malloc(mono_frame_size);
    if (s_dmic_ctx.loop_mix_buf == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    s_dmic_ctx.loop_mix_buf_size = mono_frame_size;

    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.chan_num = 1U;
    cfg.sample_bits = TUYA_AUDIO_SAMPLE_BITS_16;
    cfg.sample_rate = sample_rate;
    cfg.frame_time_ms = frame_time_ms;
    cfg.frame_cb = NULL;
    cfg.args = NULL;
    cfg.pa_pin = 60;

    ret = tkl_aud_dac_init((TUYA_AUDIO_DAC_PORT_E)dac_port, &cfg);
    if (ret != OPRT_OK) {
        test_dmic_dac_deinit();
        return ret;
    }
    {
        INT32_T gain[2];

        gain[0] = dig_db;
        gain[1] = ana_db;
        ret = tkl_aud_dac_set_gain((TUYA_AUDIO_DAC_PORT_E)dac_port, gain, 2U);
        if (ret != OPRT_OK) {
            test_dmic_dac_deinit();
            return ret;
        }
    }
    s_dmic_ctx.dac_inited = true;
    s_dmic_ctx.dac_port = (TUYA_AUDIO_DAC_PORT_E)dac_port;
    s_dmic_ctx.dac_dig_db = dig_db;
    s_dmic_ctx.dac_ana_db = ana_db;

    ret = tkl_aud_dac_start(s_dmic_ctx.dac_port);
    if (ret != OPRT_OK) {
        test_dmic_dac_deinit();
        return ret;
    }
    s_dmic_ctx.dac_started = true;

    return OPRT_OK;
}

static OPERATE_RET test_dmic_loop_start(uint32_t dmic_port, uint32_t dac_port, uint32_t sample_rate,
                                        uint32_t frame_time_ms, INT32_T dig_db, INT32_T ana_db,
                                        uint32_t clk_gpio, uint32_t data_gpio, bool use_pinmux)
{
    OPERATE_RET ret;
    uint32_t mono_frame_size;

    ret = test_dmic_init(dmic_port, sample_rate, frame_time_ms, clk_gpio, data_gpio, use_pinmux);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = test_dmic_start();
    if (ret != OPRT_OK) {
        (void)test_dmic_deinit();
        return ret;
    }

    mono_frame_size = s_dmic_ctx.frame_size / 2U;
    ret = test_dmic_dac_init_start(dac_port, sample_rate, frame_time_ms, dig_db, ana_db, mono_frame_size);
    if (ret != OPRT_OK) {
        (void)test_dmic_stop();
        (void)test_dmic_deinit();
        return ret;
    }

    s_dmic_ctx.loop_push_bytes = 0U;
    s_dmic_ctx.loop_drop_bytes = 0U;
    s_dmic_ctx.loop_busy_cnt = 0U;
    s_dmic_ctx.loop_write_err_cnt = 0U;
    s_dmic_ctx.loop_on = true;
    return OPRT_OK;
}

static OPERATE_RET test_dmic_loop_stop(void)
{
    s_dmic_ctx.loop_on = false;
    test_dmic_dac_deinit();
    (void)test_dmic_stop();
    return test_dmic_deinit();
}

static OPERATE_RET test_dmic_deinit(void)
{
    s_dmic_ctx.loop_on = false;
    test_dmic_dac_deinit();

    if (s_dmic_ctx.started) {
        (void)test_dmic_stop();
    }

    tkl_aud_dmic_deinit(s_dmic_ctx.port);
    test_dmic_runtime_reset();
    return OPRT_OK;
}

static OPERATE_RET test_dmic_init(uint32_t port, uint32_t sample_rate, uint32_t frame_time_ms,
                                  uint32_t clk_gpio, uint32_t data_gpio, bool use_pinmux)
{
    OPERATE_RET ret;
    TKL_AUD_DMIC_CFG_T cfg;

    if (port >= TUYA_AUDIO_DMIC_PORT_MAX || sample_rate == 0U || frame_time_ms == 0U) {
        return OPRT_INVALID_PARM;
    }

    if (s_dmic_ctx.started) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_dmic_ctx.initialized) {
        ret = test_dmic_deinit();
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    if (use_pinmux) {
        ret = tkl_aud_dmic_pinmux_config(clk_gpio, data_gpio);
        if (ret != OPRT_OK) {
            tkl_log_output("dmic pinmux config failed: %d\r\n", ret);
            return ret;
        }
    }

    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.chan = DMIC_TEST_DEFAULT_CHAN;
    cfg.sample_bits = DMIC_TEST_DEFAULT_SAMPLE_BITS;
    cfg.sample_rate = sample_rate;
    cfg.frame_time_ms = frame_time_ms;
    cfg.upper_cb = test_dmic_input_cb;
    cfg.args = NULL;

    ret = tkl_aud_dmic_init((TUYA_AUDIO_DMIC_PORT_E)port, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("dmic init failed: %d\r\n", ret);
        return ret;
    }

    {
        INT32_T gain[2];

        gain[0] = DMIC_TEST_DEFAULT_DIG_DB;
        gain[1] = DMIC_TEST_DEFAULT_ANA_DB;
        ret = tkl_aud_dmic_set_gain((TUYA_AUDIO_DMIC_PORT_E)port, gain, DMIC_TEST_GAIN_NUM);
        if (ret != OPRT_OK) {
            tkl_log_output("dmic set_gain failed: %d\r\n", ret);
            tkl_aud_dmic_deinit((TUYA_AUDIO_DMIC_PORT_E)port);
            return ret;
        }
    }

    s_dmic_ctx.initialized = true;
    s_dmic_ctx.started = false;
    s_dmic_ctx.port = (TUYA_AUDIO_DMIC_PORT_E)port;
    s_dmic_ctx.chan = cfg.chan;
    s_dmic_ctx.sample_rate = sample_rate;
    s_dmic_ctx.sample_bits = DMIC_TEST_DEFAULT_SAMPLE_BITS;
    s_dmic_ctx.frame_time_ms = frame_time_ms;
    s_dmic_ctx.dig_db = DMIC_TEST_DEFAULT_DIG_DB;
    s_dmic_ctx.ana_db = DMIC_TEST_DEFAULT_ANA_DB;
    s_dmic_ctx.frame_size = test_dmic_calc_frame_size(sample_rate, DMIC_TEST_DEFAULT_SAMPLE_BITS, frame_time_ms);
    s_dmic_ctx.pinmux_set = use_pinmux;
    s_dmic_ctx.clk_gpio = use_pinmux ? clk_gpio : DMIC_TEST_NO_GPIO;
    s_dmic_ctx.data_gpio = use_pinmux ? data_gpio : DMIC_TEST_NO_GPIO;
    s_dmic_ctx.frame_cnt = 0U;
    s_dmic_ctx.total_bytes = 0U;
    s_dmic_ctx.last_frame_bytes = 0U;
    s_dmic_ctx.last_rec_frames = 0U;
    s_dmic_ctx.last_rec_bytes = 0U;
    return OPRT_OK;
}

static OPERATE_RET test_dmic_start(void)
{
    OPERATE_RET ret;

    if (!s_dmic_ctx.initialized) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (s_dmic_ctx.started) {
        return OPRT_OK;
    }

    ret = tkl_aud_dmic_start(s_dmic_ctx.port);
    if (ret != OPRT_OK) {
        tkl_log_output("dmic start failed: %d\r\n", ret);
        return ret;
    }

    s_dmic_ctx.started = true;
    return OPRT_OK;
}

static OPERATE_RET test_dmic_record(uint32_t rec_ms)
{
    uint32_t frame_before;
    uint32_t byte_before;

    if (!s_dmic_ctx.initialized || !s_dmic_ctx.started || rec_ms == 0U) {
        return OPRT_INVALID_PARM;
    }

    frame_before = s_dmic_ctx.frame_cnt;
    byte_before = s_dmic_ctx.total_bytes;

    tkl_system_sleep(rec_ms);

    s_dmic_ctx.last_rec_frames = s_dmic_ctx.frame_cnt - frame_before;
    s_dmic_ctx.last_rec_bytes = s_dmic_ctx.total_bytes - byte_before;
    return OPRT_OK;
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

void cli_tuya_dmic_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    uint32_t port = DMIC_TEST_DEFAULT_PORT;
    uint32_t sample_rate = DMIC_TEST_DEFAULT_SAMPLE_RATE;
    uint32_t frame_time_ms = DMIC_TEST_DEFAULT_FRAME_TIME_MS;
    uint32_t rec_ms = DMIC_TEST_DEFAULT_REC_MS;
    uint32_t dac_port = DMIC_TEST_DEFAULT_DAC_PORT;
    INT32_T dig_db = DMIC_TEST_DEFAULT_DAC_DIG_DB;
    INT32_T ana_db = DMIC_TEST_DEFAULT_DAC_ANA_DB;
    uint32_t clk_gpio = DMIC_TEST_NO_GPIO;
    uint32_t data_gpio = DMIC_TEST_NO_GPIO;
    bool use_pinmux = false;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xdmic <init [port] [rate] [frame_ms] [clk_gpio data_gpio]|start|rec [ms]|gain <dig_db> [ana_db]|loop_start [dmic_port] [dac_port] [rate] [frame_ms] [dig_db] [ana_db] [clk_gpio data_gpio]|loop_stop|stop|deinit|close|status>");
        return;
    }

    cmd = argv[1];
    if (os_strcmp(cmd, "init") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            sample_rate = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            frame_time_ms = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 7) {
            clk_gpio = os_strtoul(argv[5], NULL, 10);
            data_gpio = os_strtoul(argv[6], NULL, 10);
            use_pinmux = true;
        }

        ret = test_dmic_init(port, sample_rate, frame_time_ms, clk_gpio, data_gpio, use_pinmux);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdmic init %s (%d), port=%u rate=%u frame_ms=%u pinmux=%u clk=%u data=%u frame_size=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port,
                 (unsigned int)sample_rate,
                 (unsigned int)frame_time_ms,
                 (unsigned int)use_pinmux,
                 (unsigned int)(use_pinmux ? clk_gpio : 0U),
                 (unsigned int)(use_pinmux ? data_gpio : 0U),
                 (unsigned int)s_dmic_ctx.frame_size);
    } else if (os_strcmp(cmd, "start") == 0) {
        ret = test_dmic_start();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdmic start %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "rec") == 0 || os_strcmp(cmd, "record") == 0) {
        if (argc >= 3) {
            rec_ms = os_strtoul(argv[2], NULL, 10);
        }

        ret = test_dmic_record(rec_ms);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdmic rec %s (%d), ms=%u frames=%u bytes=%u total_frames=%u total_bytes=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)rec_ms,
                 (unsigned int)s_dmic_ctx.last_rec_frames,
                 (unsigned int)s_dmic_ctx.last_rec_bytes,
                 (unsigned int)s_dmic_ctx.frame_cnt,
                 (unsigned int)s_dmic_ctx.total_bytes);
    } else if (os_strcmp(cmd, "gain") == 0) {
        INT32_T dig = DMIC_TEST_DEFAULT_DIG_DB;
        INT32_T ana = DMIC_TEST_DEFAULT_ANA_DB;
        INT32_T garr[2];

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xdmic gain <dig_db> [ana_db]\r\n");
            return;
        }
        dig = test_parse_s32(argv[2]);
        if (argc >= 4) {
            ana = test_parse_s32(argv[3]);
        } else if (s_dmic_ctx.initialized) {
            ana = s_dmic_ctx.ana_db;
        }
        garr[0] = dig;
        garr[1] = ana;
        ret = tkl_aud_dmic_set_gain(s_dmic_ctx.port, garr, DMIC_TEST_GAIN_NUM);
        if (ret == OPRT_OK) {
            s_dmic_ctx.dig_db = dig;
            s_dmic_ctx.ana_db = ana;
        }
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdmic gain %s (%d), dig=%d ana=%d\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (int)dig,
                 (int)ana);
    } else if (os_strcmp(cmd, "loop_start") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            dac_port = os_strtoul(argv[3], NULL, 10);
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
        if (argc >= 10) {
            clk_gpio = os_strtoul(argv[8], NULL, 10);
            data_gpio = os_strtoul(argv[9], NULL, 10);
            use_pinmux = true;
        }

        ret = test_dmic_loop_start(port, dac_port, sample_rate, frame_time_ms, dig_db, ana_db,
                                   clk_gpio, data_gpio, use_pinmux);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdmic loop_start %s (%d), dmic_port=%u dac_port=%u rate=%u frame_ms=%u dig=%d ana=%d pinmux=%u clk=%u data=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port,
                 (unsigned int)dac_port,
                 (unsigned int)sample_rate,
                 (unsigned int)frame_time_ms,
                 (int)dig_db,
                 (int)ana_db,
                 (unsigned int)use_pinmux,
                 (unsigned int)(use_pinmux ? clk_gpio : 0U),
                 (unsigned int)(use_pinmux ? data_gpio : 0U));
    } else if (os_strcmp(cmd, "loop_stop") == 0) {
        ret = test_dmic_loop_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdmic loop_stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "stop") == 0) {
        ret = test_dmic_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdmic stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "deinit") == 0 || os_strcmp(cmd, "close") == 0) {
        ret = test_dmic_deinit();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdmic %s %s (%d)\r\n",
                 cmd,
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "status") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdmic status init=%u start=%u port=%u chan=%u rate=%u bits=%u frame_ms=%u frame_size=%u pinmux=%u clk=%u data=%u frame_cnt=%u total_bytes=%u last_frame=%u loop_on=%u dac_inited=%u dac_started=%u dac_port=%u dig=%d ana=%d loop_push=%u loop_drop=%u loop_busy=%u loop_err=%u\r\n",
                 (unsigned int)s_dmic_ctx.initialized,
                 (unsigned int)s_dmic_ctx.started,
                 (unsigned int)s_dmic_ctx.port,
                 (unsigned int)s_dmic_ctx.chan,
                 (unsigned int)s_dmic_ctx.sample_rate,
                 (unsigned int)s_dmic_ctx.sample_bits,
                 (unsigned int)s_dmic_ctx.frame_time_ms,
                 (unsigned int)s_dmic_ctx.frame_size,
                 (unsigned int)s_dmic_ctx.pinmux_set,
                 (unsigned int)(s_dmic_ctx.pinmux_set ? s_dmic_ctx.clk_gpio : 0U),
                 (unsigned int)(s_dmic_ctx.pinmux_set ? s_dmic_ctx.data_gpio : 0U),
                 (unsigned int)s_dmic_ctx.frame_cnt,
                 (unsigned int)s_dmic_ctx.total_bytes,
                 (unsigned int)s_dmic_ctx.last_frame_bytes,
                 (unsigned int)s_dmic_ctx.loop_on,
                 (unsigned int)s_dmic_ctx.dac_inited,
                 (unsigned int)s_dmic_ctx.dac_started,
                 (unsigned int)s_dmic_ctx.dac_port,
                 (int)s_dmic_ctx.dac_dig_db,
                 (int)s_dmic_ctx.dac_ana_db,
                 (unsigned int)s_dmic_ctx.loop_push_bytes,
                 (unsigned int)s_dmic_ctx.loop_drop_bytes,
                 (unsigned int)s_dmic_ctx.loop_busy_cnt,
                 (unsigned int)s_dmic_ctx.loop_write_err_cnt);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xdmic <init [port] [rate] [frame_ms] [clk_gpio data_gpio]|start|rec [ms]|gain <dig_db> [ana_db]|loop_start [dmic_port] [dac_port] [rate] [frame_ms] [dig_db] [ana_db] [clk_gpio data_gpio]|loop_stop|stop|deinit|close|status>");
    }
}
