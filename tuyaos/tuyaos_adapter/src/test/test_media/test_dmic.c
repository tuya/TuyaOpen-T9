#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/mem.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_aud_dac.h"
#include "tkl_gpio.h"
#include "tkl_aud_dmic.h"
#include "tkl_output.h"
#include "tkl_system.h"

#define DMIC_TEST_DEFAULT_PORT          TUYA_AUDIO_DMIC_PORT_0
#define DMIC_TEST_DEFAULT_CHAN          ((TUYA_AUDIO_MIC_CHANNEL_E)TUYA_AUDIO_DMIC_CHANNEL_0)
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
    TUYA_AUDIO_MIC_CHANNEL_E chan;
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

/**
 * @brief Lanes a DMIC port contributes to the shared capture frame.
 * @note  PORT_0 -> MODE_0 -> lane0 (1 lane); PORT_1 -> MODE_1 -> lane1+2 (2).
 */
static uint32_t test_dmic_lane_count(uint32_t port)
{
    return (port == (uint32_t)TUYA_AUDIO_DMIC_PORT_1) ? 2U : 1U;
}

static uint32_t test_dmic_calc_frame_size(uint32_t port, uint32_t sample_rate,
                                          uint8_t sample_bits, uint32_t frame_ms)
{
    if (sample_rate == 0U || sample_bits == 0U || frame_ms == 0U) {
        return 0U;
    }

    return ((sample_rate * (sample_bits / 8U) * test_dmic_lane_count(port)) / 1000U) * frame_ms;
}

/**
 * @brief Convert a DMIC lane index 0..2 into its TUYA_AUDIO_MIC_CHANNEL_E bit
 * @param[in] lane lane index
 * @return the channel bit, or TUYA_AUDIO_MIC_CHANNEL_MAX if out of range
 */
static TUYA_AUDIO_MIC_CHANNEL_E test_dmic_lane_bit(uint32_t lane)
{
    if (lane > 2U) {
        return TUYA_AUDIO_MIC_CHANNEL_MAX;
    }

    return (TUYA_AUDIO_MIC_CHANNEL_E)((UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_0 << lane);
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

static void test_dmic_input_cb(TUYA_AUDIO_INPUT_BUFFER_T *buffer)
{
    uint8_t *buf;
    uint32_t buf_len;
    uint32_t mono_len;
    OPERATE_RET ret;

    if ((buffer == NULL) || (buffer->event != TUYA_AUDIO_FRAME_EVENT_DMIC_RX)) {
        return;
    }

    buf = buffer->data;
    buf_len = buffer->data_len;

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


/* --------------------------------------------------------------------------
 * Speaker PA enable (vendor EVB P28).
 *
 * The adapter no longer drives this pin: in production the TAL layer owns it
 * (tal_audio_dac.c). A standalone test that only talks to tkl_aud_dac_* must
 * drive it itself, otherwise the DAC runs but stays silent.
 * ------------------------------------------------------------------------ */
#define TEST_DMIC_PA_PIN  ((TUYA_GPIO_NUM_E)28)

static void test_dmic_pa_init(void)
{
    TUYA_GPIO_BASE_CFG_T cfg;

    os_memset(&cfg, 0, sizeof(cfg));
    cfg.direct = TUYA_GPIO_OUTPUT;
    cfg.mode   = TUYA_GPIO_PUSH_PULL;
    cfg.level  = TUYA_GPIO_LEVEL_LOW;   /* disabled until playback starts */
    (void)tkl_gpio_init(TEST_DMIC_PA_PIN, &cfg);
}

static void test_dmic_pa_set(bool enable)
{
    (void)tkl_gpio_write(TEST_DMIC_PA_PIN,
                         enable ? TUYA_GPIO_LEVEL_HIGH : TUYA_GPIO_LEVEL_LOW);
}

static void test_dmic_pa_deinit(void)
{
    (void)tkl_gpio_deinit(TEST_DMIC_PA_PIN);
}

static void test_dmic_dac_deinit(void)
{
    if (s_dmic_ctx.dac_inited) {
        if (s_dmic_ctx.dac_started) {
            test_dmic_pa_set(false);
            (void)tkl_aud_dac_stop(s_dmic_ctx.dac_port);
        }
        (void)tkl_aud_dac_deinit(s_dmic_ctx.dac_port);
    }
    test_dmic_pa_deinit();

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
    TUYA_AUDIO_DAC_BASE_CFG_T cfg;

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
        test_dmic_dac_deinit();
        return ret;
    }

    test_dmic_pa_init();
    {
        INT32_T gain[2];

        gain[0] = dig_db;
        gain[1] = ana_db;
        ret = tkl_aud_dac_set_gain((TUYA_AUDIO_DAC_PORT_E)dac_port, TUYA_AUDIO_DAC_CHANNEL_0,
                                   gain, 2U);
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
    test_dmic_pa_set(true);
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

    /* One lane's worth of bytes: the DAC loop plays a single channel. */
    mono_frame_size = s_dmic_ctx.frame_size / test_dmic_lane_count(s_dmic_ctx.port);
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
    TUYA_AUDIO_DMIC_BASE_CFG_T cfg;

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
        ret = tkl_aud_dmic_pinmux_config((TUYA_AUDIO_DMIC_PORT_E)port, clk_gpio, data_gpio);
        if (ret != OPRT_OK) {
            tkl_log_output("dmic pinmux config failed: %d\r\n", ret);
            return ret;
        }
    }

    (void)os_memset(&cfg, 0, sizeof(cfg));
    /* PORT_0 owns DMIC CH0 (BIT(8)); PORT_1 owns CH1/CH2 (BIT(9)|BIT(10)). */
    cfg.chan = ((TUYA_AUDIO_DMIC_PORT_E)port == TUYA_AUDIO_DMIC_PORT_1)
             ? (TUYA_AUDIO_MIC_CHANNEL_E)((UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_1 |
                                          (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_2)
             : (TUYA_AUDIO_MIC_CHANNEL_E)TUYA_AUDIO_DMIC_CHANNEL_0;
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
        UINT32_T ch;

        gain[0] = DMIC_TEST_DEFAULT_DIG_DB;
        gain[1] = DMIC_TEST_DEFAULT_ANA_DB;
        /* PORT_0 (MODE_0) owns CH0; PORT_1 (MODE_1) owns CH1/CH2. CH0 on PORT_1
         * is the analog lane and belongs to tkl_aud_adc. */
        if ((TUYA_AUDIO_DMIC_PORT_E)port == TUYA_AUDIO_DMIC_PORT_1) {
            for (ch = 1U; ch <= 2U; ch++) {
                ret = tkl_aud_dmic_set_gain((TUYA_AUDIO_DMIC_PORT_E)port,
                                            test_dmic_lane_bit(ch),
                                            gain, DMIC_TEST_GAIN_NUM);
                if (ret != OPRT_OK) {
                    tkl_log_output("dmic set_gain failed: %d ch=%u\r\n", ret, (unsigned)ch);
                    tkl_aud_dmic_deinit((TUYA_AUDIO_DMIC_PORT_E)port);
                    return ret;
                }
            }
        } else {
            ret = tkl_aud_dmic_set_gain((TUYA_AUDIO_DMIC_PORT_E)port,
                                        TUYA_AUDIO_DMIC_CHANNEL_0,
                                        gain, DMIC_TEST_GAIN_NUM);
            if (ret != OPRT_OK) {
                tkl_log_output("dmic set_gain failed: %d ch=0\r\n", ret);
                tkl_aud_dmic_deinit((TUYA_AUDIO_DMIC_PORT_E)port);
                return ret;
            }
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
    s_dmic_ctx.frame_size = test_dmic_calc_frame_size(port, sample_rate, DMIC_TEST_DEFAULT_SAMPLE_BITS,
                                                      frame_time_ms);
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

/* --------------------------------------------------------------------------
 * `xdmic raw <en> [ms]` — the digmic_en hardware experiment
 *
 * Opens the port(s) selected by the en bitmask (0x1 = port 0 only, 0x6 = port 1
 * only, 0x7 = both), lets the stream run and reports per-lane energy plus a
 * pairwise independence number. That answers the one question the two-port
 * design rests on: does digmic_en = 0x7 really feed three independent lanes?
 * ------------------------------------------------------------------------ */
#define DMIC_RAW_LANES      (3U)

typedef struct {
    uint32_t frames;
    uint32_t lanes;
    /* Per-lane sample counter. NOT derivable from `frames`: with en=0x7 the
     * two users deliver 1 and 2 lanes respectively, so one "frame" adds a
     * different number of samples to each lane. */
    uint64_t samples[DMIC_RAW_LANES];
    uint64_t sum_sq[DMIC_RAW_LANES];
    uint64_t sum_abs[DMIC_RAW_LANES];
    uint64_t sum_abs_diff[DMIC_RAW_LANES][DMIC_RAW_LANES];
    /* How many times lane pair (a,b) was seen in the SAME buffer. 0 means the
     * pair is not comparable at all - the two lanes come from different users
     * and never appear together, so their diff is meaningless. */
    uint32_t pair_cnt[DMIC_RAW_LANES][DMIC_RAW_LANES];
    /* Shape of the waveform, which rms/mean_abs alone cannot tell apart:
     *   dc ~ 0, min/-max symmetric, mean_abs/rms > 0.5 -> real audio
     *   dc != 0, min ~ max                             -> stuck / constant
     *   min ~ 0, max spikes, mean_abs/rms << 0.5       -> mostly silence
     *                                                     with rare impulses
     */
    int64_t  sum_v[DMIC_RAW_LANES];
    int32_t  vmin[DMIC_RAW_LANES];
    int32_t  vmax[DMIC_RAW_LANES];
} test_dmic_raw_stats_t;

static test_dmic_raw_stats_t s_raw_stats;

static void test_dmic_raw_reset(void)
{
    uint32_t i;

    (void)os_memset(&s_raw_stats, 0, sizeof(s_raw_stats));
    for (i = 0U; i < DMIC_RAW_LANES; i++) {
        s_raw_stats.vmin[i] = 32767;
        s_raw_stats.vmax[i] = -32768;
    }
}

static void test_dmic_raw_cb(TUYA_AUDIO_INPUT_BUFFER_T *buffer)
{
    const int16_t *pcm;
    uint32_t lanes;
    uint32_t first_lane;
    uint32_t samples;
    uint32_t i;
    uint32_t a;
    uint32_t b;
    UINT32_T bit;

    if ((buffer == NULL) || (buffer->event != TUYA_AUDIO_FRAME_EVENT_DMIC_RX) ||
        (buffer->data == NULL)) {
        return;
    }

    /* Each user is handed only its own lanes; map them back to global lanes. */
    lanes = buffer->chan_num;
    if ((lanes == 0U) || (lanes > DMIC_RAW_LANES)) {
        return;
    }

    bit = (UINT32_T)buffer->chan[0];
    for (first_lane = 0U; first_lane < DMIC_RAW_LANES; first_lane++) {
        if (bit == ((UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_0 << first_lane)) {
            break;
        }
    }
    if ((first_lane + lanes) > DMIC_RAW_LANES) {
        return;
    }

    pcm = (const int16_t *)buffer->data;
    samples = buffer->data_len / 2U;
    if ((samples == 0U) || ((samples % lanes) != 0U)) {
        return;
    }

    s_raw_stats.frames++;

    for (i = 0U; i < samples; i++) {
        int32_t v = pcm[i];
        int64_t mag = (v < 0) ? -(int64_t)v : (int64_t)v;
        uint32_t lane = first_lane + (i % lanes);

        s_raw_stats.sum_sq[lane] += (uint64_t)((int64_t)v * (int64_t)v);
        s_raw_stats.sum_abs[lane] += (uint64_t)mag;
        s_raw_stats.sum_v[lane] += (int64_t)v;
        if (v < s_raw_stats.vmin[lane]) {
            s_raw_stats.vmin[lane] = v;
        }
        if (v > s_raw_stats.vmax[lane]) {
            s_raw_stats.vmax[lane] = v;
        }
        s_raw_stats.samples[lane]++;
    }

    for (i = 0U; i < (samples / lanes); i++) {
        for (a = 0U; a < lanes; a++) {
            for (b = a + 1U; b < lanes; b++) {
                int32_t va = pcm[i * lanes + a];
                int32_t vb = pcm[i * lanes + b];
                int32_t d = va - vb;

                s_raw_stats.sum_abs_diff[first_lane + a][first_lane + b] += (uint64_t)((d < 0) ? -d : d);
                s_raw_stats.pair_cnt[first_lane + a][first_lane + b]++;
            }
        }
    }
}

/* Integer sqrt, kept local so the test never needs libm. */
static uint32_t test_dmic_isqrt(uint64_t v)
{
    uint64_t r = 0U;
    uint64_t bit = (uint64_t)1U << 62;

    while (bit > v) {
        bit >>= 2;
    }

    while (bit != 0U) {
        if (v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }

    return (uint32_t)r;
}

/**
 * @brief `xdmic raw <en> [ms]` experiment driver
 * @param[in] en digmic_en to realise (0x1 / 0x6 / 0x7)
 * @param[in] ms how long to run the stream
 * @param[out] out reaction buffer
 * @param[in] out_len reaction buffer length
 */
static void test_dmic_raw_run(uint32_t en, uint32_t ms, char *out, int out_len)
{
    TUYA_AUDIO_DMIC_BASE_CFG_T cfg;
    OPERATE_RET ret;
    uint32_t lanes = 0U;
    uint32_t opened = 0U;
    uint32_t i;
    uint32_t a;
    uint32_t b;

    if ((en != 0x1U) && (en != 0x6U) && (en != 0x7U)) {
        snprintf(out, out_len, "xdmic raw: en must be 0x1, 0x6 or 0x7\r\n");
        return;
    }

    if (s_dmic_ctx.initialized || s_dmic_ctx.started) {
        snprintf(out, out_len, "xdmic raw: xdmic is open, run 'xdmic deinit' first\r\n");
        return;
    }

    for (i = 0U; i < DMIC_RAW_LANES; i++) {
        if (en & (1U << i)) {
            lanes++;
        }
    }

    test_dmic_raw_reset();
    s_raw_stats.lanes = lanes;

    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.chan = (TUYA_AUDIO_MIC_CHANNEL_E)TUYA_AUDIO_DMIC_CHANNEL_0;
    cfg.sample_bits = DMIC_TEST_DEFAULT_SAMPLE_BITS;
    cfg.sample_rate = DMIC_TEST_DEFAULT_SAMPLE_RATE;
    cfg.frame_time_ms = DMIC_TEST_DEFAULT_FRAME_TIME_MS;
    cfg.upper_cb = test_dmic_raw_cb;
    cfg.args = NULL;

    if (en & 0x1U) {
        cfg.chan = (TUYA_AUDIO_MIC_CHANNEL_E)TUYA_AUDIO_DMIC_CHANNEL_0;
        ret = tkl_aud_dmic_init(TUYA_AUDIO_DMIC_PORT_0, &cfg);
        if (ret != OPRT_OK) {
            snprintf(out, out_len, "xdmic raw: port0 init failed (%d)\r\n", ret);
            return;
        }
        opened++;
    }

    if (en & 0x6U) {
        cfg.chan = (TUYA_AUDIO_MIC_CHANNEL_E)((UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_1 |
                                              (UINT32_T)TUYA_AUDIO_DMIC_CHANNEL_2);
        ret = tkl_aud_dmic_init(TUYA_AUDIO_DMIC_PORT_1, &cfg);
        if (ret != OPRT_OK) {
            if (en & 0x1U) {
                tkl_aud_dmic_deinit(TUYA_AUDIO_DMIC_PORT_0);
                snprintf(out, out_len,
                         "xdmic raw: port1 init failed (%d) with port0 open"
                         " - pads shared? use 'xdmic pinmux 1 <clk> <dat>'\r\n", ret);
            } else {
                snprintf(out, out_len, "xdmic raw: port1 init failed (%d)\r\n", ret);
            }
            return;
        }
        opened++;
    }

    /* Each started port receives its own lanes; start every opened port. */
    ret = OPRT_OK;
    if (en & 0x1U) {
        ret = tkl_aud_dmic_start(TUYA_AUDIO_DMIC_PORT_0);
    }
    if ((ret == OPRT_OK) && (en & 0x6U)) {
        ret = tkl_aud_dmic_start(TUYA_AUDIO_DMIC_PORT_1);
    }
    if (ret != OPRT_OK) {
        snprintf(out, out_len, "xdmic raw: start failed (%d)\r\n", ret);
    } else {
        tkl_system_sleep(ms);
        if (en & 0x1U) {
            (void)tkl_aud_dmic_stop(TUYA_AUDIO_DMIC_PORT_0);
        }
        if (en & 0x6U) {
            (void)tkl_aud_dmic_stop(TUYA_AUDIO_DMIC_PORT_1);
        }
    }

    if (en & 0x1U) {
        tkl_aud_dmic_deinit(TUYA_AUDIO_DMIC_PORT_0);
    }
    if (en & 0x6U) {
        tkl_aud_dmic_deinit(TUYA_AUDIO_DMIC_PORT_1);
    }

    if (ret != OPRT_OK) {
        return;
    }

    {
        /* The shell hands us SHELL_RSP_BUF_LEN - 4 == 252 bytes; anything longer
         * is silently truncated, so keep the whole report compact. */
        int n = snprintf(out, out_len,
                         "raw en=0x%x lanes=%u frames=%u\r\n",
                         (unsigned int)en, (unsigned int)lanes,
                         (unsigned int)s_raw_stats.frames);
        int printed_pair = 0;

        /* Index the stats by GLOBAL lane number: with en=0x6 the only user
         * owns lanes 1 and 2, so printing i = 0..lanes-1 would label real data
         * as lane0/lane1 and invent a phantom lane0. */
        for (i = 0U; (i < DMIC_RAW_LANES) && (n > 0) && (n < out_len); i++) {
            uint64_t count = s_raw_stats.samples[i];
            uint32_t rms;
            uint32_t mean_abs;
            int32_t dc;
            int32_t vmin;
            int32_t vmax;

            if (count == 0U) {
                continue;
            }

            rms = test_dmic_isqrt(s_raw_stats.sum_sq[i] / count);
            mean_abs = (uint32_t)(s_raw_stats.sum_abs[i] / count);
            dc = (int32_t)(s_raw_stats.sum_v[i] / (int64_t)count);
            vmin = s_raw_stats.vmin[i];
            vmax = s_raw_stats.vmax[i];

            n += snprintf(out + n, out_len - n,
                          "lane%u rms=%u mean=%u dc=%d min=%d max=%d\r\n",
                          (unsigned int)i, (unsigned int)rms, (unsigned int)mean_abs,
                          (int)dc, (int)vmin, (int)vmax);
        }

        for (a = 0U; (a < DMIC_RAW_LANES) && (n > 0) && (n < out_len); a++) {
            if (s_raw_stats.samples[a] == 0U) {
                continue;
            }
            for (b = a + 1U; (b < DMIC_RAW_LANES) && (n > 0) && (n < out_len); b++) {
                uint64_t diff = s_raw_stats.sum_abs_diff[a][b];
                uint64_t base = s_raw_stats.sum_abs[a] + s_raw_stats.sum_abs[b];
                uint32_t permille;

                if (s_raw_stats.samples[b] == 0U) {
                    continue;
                }
                if (printed_pair == 0) {
                    n += snprintf(out + n, out_len - n, "diff");
                    printed_pair = 1;
                }

                if (s_raw_stats.pair_cnt[a][b] == 0U) {
                    n += snprintf(out + n, out_len - n, " %d%d=n/a",
                                  (unsigned int)a, (unsigned int)b);
                    continue;
                }

                permille = (base != 0U) ? (uint32_t)((diff * 1000U) / base) : 0U;

                n += snprintf(out + n, out_len - n, " %d%d=%u",
                              (unsigned int)a, (unsigned int)b, (unsigned int)permille);
            }
        }

        if ((printed_pair != 0) && (n > 0) && (n < out_len)) {
            (void)snprintf(out + n, out_len - n, "\r\n");
        }
    }
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

/**
 * @brief Parse unsigned decimal or 0x-prefixed hex from CLI argv
 * @param[in] s input string
 * @return uint32_t value
 */
static uint32_t test_parse_u32_auto(const char *s)
{
    if ((s == NULL) || (s[0] == '\0')) {
        return 0U;
    }

    if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X'))) {
        return (uint32_t)os_strtoul(s + 2, NULL, 16);
    }

    return (uint32_t)os_strtoul(s, NULL, 10);
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
                 "Usage: xdmic <init [port] [rate] [frame_ms] [clk_gpio data_gpio]|start|rec [ms]|pinmux <port> <clk> <dat>|raw <en> [ms]|gain <dig|x> [ana|x] [ch 0|1|2]|loop_start [dmic_port] [dac_port] [rate] [frame_ms] [dig_db] [ana_db] [clk_gpio data_gpio]|loop_stop|stop|deinit|close|status>");
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
    } else if (os_strcmp(cmd, "pinmux") == 0) {
        if (argc < 5) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xdmic pinmux <port> <clk_gpio> <data_gpio>\r\n");
            return;
        }

        port = os_strtoul(argv[2], NULL, 10);
        clk_gpio = os_strtoul(argv[3], NULL, 10);
        data_gpio = os_strtoul(argv[4], NULL, 10);

        ret = tkl_aud_dmic_pinmux_config((TUYA_AUDIO_DMIC_PORT_E)port, clk_gpio, data_gpio);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdmic pinmux %s (%d), port=%u clk=%u data=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port,
                 (unsigned int)clk_gpio,
                 (unsigned int)data_gpio);
    } else if (os_strcmp(cmd, "raw") == 0) {
        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xdmic raw <en> [ms]   en = 0x1 (port0) | 0x6 (port1) | 0x7 (both)\r\n"
                     "  opens the selected ports, runs, prints per-lane energy + pairwise diff\r\n"
                     "  configure each port's pads first: xdmic pinmux <port> <clk> <dat>\r\n");
            return;
        }

        {
            UINT32_T en = test_parse_u32_auto(argv[2]);
            UINT32_T run_ms = DMIC_TEST_DEFAULT_REC_MS;

            if (argc >= 4) {
                run_ms = (UINT32_T)os_strtoul(argv[3], NULL, 10);
            }

            test_dmic_raw_run(en, run_ms, pcWriteBuffer, xWriteBufferLen);
        }
    } else if (os_strcmp(cmd, "gain") == 0) {
        INT32_T dig = DMIC_TEST_DEFAULT_DIG_DB;
        INT32_T ana = DMIC_TEST_DEFAULT_ANA_DB;
        INT32_T garr[2];
        UINT32_T mic_ch = 0xFFU;
        BOOL_T dig_keep = FALSE;
        BOOL_T ana_keep = FALSE;
        UINT32_T ch;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: xdmic gain <dig_db|x> [ana_db|x] [ch 0|1|2]\r\n"
                     "  x = keep (AUDIO_DMIC_INVALID_GAIN_VALUE); ch omit = all open ch\r\n");
            return;
        }
        if ((argv[2][0] == 'x' || argv[2][0] == 'X') && argv[2][1] == '\0') {
            dig_keep = TRUE;
            dig = (INT32_T)AUDIO_DMIC_INVALID_GAIN_VALUE;
        } else {
            dig = test_parse_s32(argv[2]);
        }
        if (argc >= 4) {
            if ((argv[3][0] == 'x' || argv[3][0] == 'X') && argv[3][1] == '\0') {
                ana_keep = TRUE;
                ana = (INT32_T)AUDIO_DMIC_INVALID_GAIN_VALUE;
            } else {
                ana = test_parse_s32(argv[3]);
            }
        } else {
            ana_keep = TRUE;
            ana = (INT32_T)AUDIO_DMIC_INVALID_GAIN_VALUE;
        }
        if (argc >= 5) {
            mic_ch = (UINT32_T)test_parse_s32(argv[4]);
        }

        if (!s_dmic_ctx.initialized) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "xdmic gain failed: not init\r\n");
            return;
        }

        garr[0] = dig;
        garr[1] = ana;
        ret = OPRT_OK;
        if (mic_ch != 0xFFU) {
            ret = tkl_aud_dmic_set_gain(s_dmic_ctx.port, test_dmic_lane_bit(mic_ch),
                                        garr, DMIC_TEST_GAIN_NUM);
        } else {
            /* PORT_0 owns CH0; PORT_1 owns CH1/CH2 (CH0 is the analog lane). */
            UINT32_T first = (s_dmic_ctx.port == TUYA_AUDIO_DMIC_PORT_1) ? 1U : 0U;
            UINT32_T last  = (s_dmic_ctx.port == TUYA_AUDIO_DMIC_PORT_1) ? 2U : 0U;

            for (ch = first; ch <= last; ch++) {
                ret = tkl_aud_dmic_set_gain(s_dmic_ctx.port, test_dmic_lane_bit(ch),
                                            garr, DMIC_TEST_GAIN_NUM);
                if (ret != OPRT_OK) {
                    break;
                }
            }
        }
        if (ret == OPRT_OK) {
            if (dig_keep != TRUE) {
                s_dmic_ctx.dig_db = dig;
            }
            if (ana_keep != TRUE) {
                s_dmic_ctx.ana_db = ana;
            }
        }
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xdmic gain %s (%d), ch=%s dig=%s%d ana=%s%d\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (mic_ch != 0xFFU) ? "one" : "all",
                 (dig_keep == TRUE) ? "keep/" : "",
                 (dig_keep == TRUE) ? (int)s_dmic_ctx.dig_db : (int)dig,
                 (ana_keep == TRUE) ? "keep/" : "",
                 (ana_keep == TRUE) ? (int)s_dmic_ctx.ana_db : (int)ana);
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
                 "Usage: xdmic <init [port] [rate] [frame_ms] [clk_gpio data_gpio]|start|rec [ms]|pinmux <port> <clk> <dat>|raw <en> [ms]|gain <dig|x> [ana|x] [ch 0|1|2]|loop_start [dmic_port] [dac_port] [rate] [frame_ms] [dig_db] [ana_db] [clk_gpio data_gpio]|loop_stop|stop|deinit|close|status>");
    }
}
