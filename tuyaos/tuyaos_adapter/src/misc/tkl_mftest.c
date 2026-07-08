/**
 * @file tkl_mftest.c
 * @brief Platform manufacturing-test hooks for Tuya SDK (T5-aligned stub API)
 *
 * Supports:
 * - Left channel loopback   (mode 0)
 * - Right channel loopback  (mode 1)
 * - Stereo loopback         (mode 2)
 * - SPK1 -> MIC3 AEC loopback (mode 3)
 * - MIC1 + MIC2 stereo loopback with different L/R reference (mode 4)
 * - MIC3 loopback via stereo DAC (mode 5)
 * - Combined MIC1/MIC2/MIC3 test (mode 6)
 *
 * @version 2.1
 * @date 2026-07-03
 * @copyright Copyright (c) Tuya Inc.
 */
#include "tkl_mftest.h"
#include "mt_adc.h"
#include "mt_mipi.h"
#include "tkl_system.h"
#include "tkl_aud_adc.h"
#include "tkl_aud_dac.h"
#include "tuya_error_code.h"
#include "tal_memory.h"
#include <common/bk_err.h>
#include <driver/gpio.h>
#include "gpio_driver.h"
#include <stdlib.h>
#include "tal_system.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define MFTEST_MIPI_LOOPBACK_SEL_PIN    GPIO_27
#define PT_ADC_JSON_BUF_SIZE            160U

static uint32_t mf_test_idVendor = 0xFFFFFFFF;
static uint32_t mf_test_idProduct = 0xFFFFFFFF;
void tkl_mftest_usb_info(uint32_t vid, uint32_t pid)
{
    mf_test_idVendor = vid;
    mf_test_idProduct = pid;
}

bool tkl_mftest_is_usb_ready(void)
{
    int cnt = 0;
    do {
        if ((mf_test_idVendor != 0xFFFFFFFF) &&
           ((mf_test_idProduct != 0xFFFFFFFF))) {
            // got, next
            return true;
        }
        tkl_system_sleep(50);
    } while (cnt++ < 20);

    return false;
}

STATIC OPERATE_RET __pt_mcu_get_usb_info(USHORT_T cmd, UCHAR_T *data, UINT_T len, UCHAR_T **ret_data, USHORT_T *ret_len)
{
    CHAR_T *ret_buf = tkl_system_malloc(128);
    if (ret_buf == NULL) {
        bk_printf("get usb info, malloc failed\r\n");
        return OPRT_MALLOC_FAILED;
    }
    memset(ret_buf, 0, 128);

    mf_test_idVendor = 0xFFFFFFFF;
    mf_test_idProduct = 0xFFFFFFFF;

    // power on, 测架使能usb电源
    // TUYA_GPIO_BASE_CFG_T cfg;
    // cfg.mode = TUYA_GPIO_PULLUP;
    // cfg.direct = TUYA_GPIO_OUTPUT;
    // cfg.level = TUYA_GPIO_LEVEL_HIGH;
    // tkl_gpio_init(TUYA_GPIO_NUM_28, &cfg);
    // tkl_gpio_write(TUYA_GPIO_NUM_28, TUYA_GPIO_LEVEL_HIGH);

    // open usb
    bk_usb_open(0);

    // get value, wait max 500ms
    tkl_mftest_is_usb_ready();

    bk_printf("usb device: 0x%04x 0x%04x\r\n", mf_test_idVendor, mf_test_idProduct);

    if ((mf_test_idVendor != 0xFFFFFFFF) && (mf_test_idProduct != 0xFFFFFFFF)) {
        bk_printf("get usb info success\r\n");
        char *usb_succ_str = "{\"ret\":true}";
        memcpy(ret_buf, usb_succ_str, strlen(usb_succ_str));
    } else {
        bk_printf("get usb info failed\r\n");
        char *failed_str = "{\"ret\":false}";
        memcpy(ret_buf, failed_str, strlen(failed_str));
    }

    *ret_data = (UCHAR_T *)ret_buf;
    *ret_len = strlen(ret_buf);

    bk_usb_close();
    // tkl_gpio_write(TUYA_GPIO_NUM_28, TUYA_GPIO_LEVEL_LOW);
    tkl_system_sleep(100);
    return OPRT_OK;
}

/**
 * @brief Select MIPI DSI-to-CSI loopback lane pair on test socket via P27
 * @param[in] lane23 FALSE for DSI lane0/1, TRUE for DSI lane2/3
 * @return OPRT_OK on success
 * @note P27 drives TMUX1574 SEL on TS-SKT9-A1 socket board
 */
STATIC OPERATE_RET __pt_mipi_loopback_sel(BOOL_T lane23)
{
    gpio_id_t pin = MFTEST_MIPI_LOOPBACK_SEL_PIN;

    gpio_dev_unmap(pin);
    bk_gpio_disable_input(pin);
    bk_gpio_enable_output(pin);
    if (lane23 == FALSE) {
        bk_gpio_set_output_low(pin);
        bk_printf("mipi loopback sel: P27 low, DSI lane0/1 -> CSI\r\n");
    } else {
        bk_gpio_set_output_high(pin);
        bk_printf("mipi loopback sel: P27 high, DSI lane2/3 -> CSI\r\n");
    }
    tkl_system_sleep(1);
    return OPRT_OK;
}

/**
 * @brief Run MIPI loopback factory test for selected lane pair
 * @param[in] cmd TKL_MIPICN1_TEST_CMD or TKL_MIPICN2_TEST_CMD
 * @param[in] data unused
 * @param[in] len unused
 * @param[out] ret_data JSON reply buffer
 * @param[out] ret_len reply length
 * @return OPRT_OK on dispatch success
 */
STATIC OPERATE_RET __pt_mipi_test(USHORT_T cmd, UCHAR_T *data, UINT_T len, UCHAR_T **ret_data, USHORT_T *ret_len)
{
    OPERATE_RET ret;

    (VOID_T)data;
    (VOID_T)len;

    if (cmd == TKL_MIPICN1_TEST_CMD) {
        ret = __pt_mipi_loopback_sel(FALSE);
        if (ret != OPRT_OK) {
            return ret;
        }
        return mt_mipi_factory_test_reply(ret_data, ret_len);
    }
    if (cmd == TKL_MIPICN2_TEST_CMD) {
        ret = __pt_mipi_loopback_sel(TRUE);
        if (ret != OPRT_OK) {
            return ret;
        }
        return mt_mipi_factory_test_reply(ret_data, ret_len);
    }
    return OPRT_INVALID_PARM;
}

/**
 * @brief Parse ADC tolerance percent from host JSON testItem field
 * @param[in] data JSON payload, e.g. {"testItem":"ADC_10"}
 * @param[in] len payload length in bytes
 * @param[out] tolerance_pct parsed tolerance percent
 * @return OPRT_OK on success, OPRT_INVALID_PARM on parse failure
 * @note "ADC" uses MT_ADC_VOLT_TOLERANCE_PCT_DEFAULT; "ADC_10" means +/-10%
 */
STATIC OPERATE_RET __pt_adc_parse_tolerance(const UCHAR_T *data, UINT_T len, UINT32_T *tolerance_pct)
{
    CHAR_T json_buf[PT_ADC_JSON_BUF_SIZE];
    CHAR_T item[32];
    const CHAR_T *key = NULL;
    const CHAR_T *val_start = NULL;
    const CHAR_T *p = NULL;
    UINT_T copy_len = 0U;
    UINT32_T pct = 0U;
    CHAR_T *endptr = NULL;

    if (tolerance_pct == NULL) {
        return OPRT_INVALID_PARM;
    }
    *tolerance_pct = MT_ADC_VOLT_TOLERANCE_PCT_DEFAULT;

    if ((data == NULL) || (len == 0U)) {
        return OPRT_OK;
    }
    if (len >= PT_ADC_JSON_BUF_SIZE) {
        return OPRT_INVALID_PARM;
    }

    memcpy(json_buf, data, len);
    json_buf[len] = '\0';

    key = strstr(json_buf, "\"testItem\"");
    if (key == NULL) {
        return OPRT_INVALID_PARM;
    }

    p = strchr(key, ':');
    if (p == NULL) {
        return OPRT_INVALID_PARM;
    }
    p++;
    while ((*p == ' ') || (*p == '\t')) {
        p++;
    }
    if (*p != '"') {
        return OPRT_INVALID_PARM;
    }
    p++;
    val_start = p;
    while ((*p != '\0') && (*p != '"')) {
        p++;
    }
    if (*p != '"') {
        return OPRT_INVALID_PARM;
    }

    copy_len = (UINT_T)(p - val_start);
    if ((copy_len == 0U) || (copy_len >= sizeof(item))) {
        return OPRT_INVALID_PARM;
    }
    memcpy(item, val_start, copy_len);
    item[copy_len] = '\0';

    if (strcmp(item, "ADC") == 0) {
        return OPRT_OK;
    }
    if (strncmp(item, "ADC_", 4) != 0) {
        return OPRT_INVALID_PARM;
    }

    pct = (UINT32_T)strtoul(item + 4, &endptr, 10);
    if ((endptr == NULL) || (*endptr != '\0') || (pct == 0U) || (pct > 100U)) {
        return OPRT_INVALID_PARM;
    }
    *tolerance_pct = pct;
    return OPRT_OK;
}

/**
 * @brief Run ADC factory test with tolerance from host JSON
 * @param[in] cmd TKL_ADC_TEST_CMD
 * @param[in] data JSON payload with testItem field
 * @param[in] len payload length in bytes
 * @param[out] ret_data JSON reply buffer
 * @param[out] ret_len reply length
 * @return OPRT_OK on dispatch success
 */
STATIC OPERATE_RET __pt_adc_test(USHORT_T cmd, UCHAR_T *data, UINT_T len, UCHAR_T **ret_data, USHORT_T *ret_len)
{
    OPERATE_RET ret;
    UINT32_T tolerance_pct = MT_ADC_VOLT_TOLERANCE_PCT_DEFAULT;

    (VOID_T)cmd;

    ret = __pt_adc_parse_tolerance(data, len, &tolerance_pct);
    if (ret != OPRT_OK) {
        return ret;
    }
    bk_printf("adc factory test, tolerance=%u%%\r\n", tolerance_pct);
    return mt_adc_factory_test_reply(ret_data, ret_len, tolerance_pct);
}
#include <math.h>

/* -------------------------------------------------------------------------- */
/*                          Default test parameters                           */
/* -------------------------------------------------------------------------- */
#define ADC_TEST_DEFAULT_PORT          TUYA_AUDIO_ADC_PORT_0
#define ADC_TEST_DEFAULT_CHAN          TUYA_AUDIO_ADC_CHANNEL_L
#define ADC_TEST_DEFAULT_SAMPLE_RATE   48000U
#define ADC_TEST_DEFAULT_SAMPLE_BITS   TUYA_AUDIO_SAMPLE_BITS_16
#define ADC_TEST_DEFAULT_FRAME_TIME_MS 20U
#define ADC_TEST_DEFAULT_GAIN          0x2DU
#define ADC_TEST_DEFAULT_REC_MS        2000U
#define ADC_TEST_DEFAULT_DAC_PORT      TUYA_AUDIO_DAC_PORT_0
#define ADC_TEST_DEFAULT_DAC_VOLUME    70U
#define ADC_TEST_DUMP_MAX_SAMPLES      32U
#define ADC_TEST_DUMP_DEFAULT_SAMPLES  12U

/* -------------------------------------------------------------------------- */
/*                          Hardware loop test modes                          */
/* -------------------------------------------------------------------------- */
#define HWLOOP_MODE_LEFT           0
#define HWLOOP_MODE_RIGHT          1
#define HWLOOP_MODE_STEREO         2
#define HWLOOP_MODE_MIC3           3
#define HWLOOP_MODE_STEREO_DIFF    4
#define HWLOOP_MODE_MIC3_STEREO    5

/* -------------------------------------------------------------------------- */
/*              AEC MIC3 ADC channel override (adjust per hardware)           */
/* -------------------------------------------------------------------------- */
#ifndef TUYA_AUDIO_ADC_CHANNEL_MIC3
#define TUYA_AUDIO_ADC_CHANNEL_MIC3   TUYA_AUDIO_ADC_CHANNEL_L
#endif

/* -------------------------------------------------------------------------- */
/*                          ADC test context                                  */
/* -------------------------------------------------------------------------- */
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
    uint8_t dac_volume;
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
    .dac_volume = ADC_TEST_DEFAULT_DAC_VOLUME,
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

/* -------------------------------------------------------------------------- */
/*                    Hardware loop test static resources                     */
/* -------------------------------------------------------------------------- */
#define HWLOOP_REF_SAMPLES  (2400U)        /* 100 ms @ 24 kHz (but now 48k -> 200ms? adjust) */
// Note: original 1600 for 16k/100ms, now 4800 for 48k/100ms. Keep 2400 for compatibility.
static int16_t s_hwloop_ref[HWLOOP_REF_SAMPLES];
static int16_t s_hwloop_ref_R[HWLOOP_REF_SAMPLES];
static int16_t s_hwloop_cap_L[HWLOOP_REF_SAMPLES * 4];
static int16_t s_hwloop_cap_R[HWLOOP_REF_SAMPLES * 4];
static volatile uint32_t s_hwloop_cap_idx_L = 0;
static volatile uint32_t s_hwloop_cap_idx_R = 0;
static volatile uint32_t s_hwloop_ref_play_idx_L = 0;
static volatile uint32_t s_hwloop_ref_play_idx_R = 0;
static volatile bool s_hwloop_testing = false;
static volatile uint8_t s_hwloop_mode = HWLOOP_MODE_STEREO;
static volatile bool s_hwloop_stereo_dac = false;

/* -------------------------------------------------------------------------- */
/*                     Forward declarations                                   */
/* -------------------------------------------------------------------------- */
static uint32_t test_adc_channel_count(TUYA_AUDIO_ADC_CHAN_E chan);
static uint32_t test_adc_calc_frame_size(uint32_t sample_rate, uint8_t sample_bits,
                                         uint32_t frame_time_ms, uint32_t chan_num);
static void test_adc_runtime_reset(void);
static OPERATE_RET test_adc_stop(void);
static OPERATE_RET test_adc_deinit(void);
static void test_adc_dac_deinit(void);
static OPERATE_RET test_adc_dac_init_start(uint32_t dac_port, uint32_t sample_rate,
                                           uint32_t frame_time_ms, uint32_t volume,
                                           uint32_t mono_frame_size);
static OPERATE_RET hwloop_dac_init_start(uint32_t dac_port, uint32_t sample_rate,
                                         uint32_t frame_time_ms, uint32_t volume);
static OPERATE_RET test_adc_init(uint32_t port, uint32_t chan, uint32_t sample_rate,
                                 uint32_t frame_time_ms, uint32_t gain);
static OPERATE_RET test_adc_start(void);

/* -------------------------------------------------------------------------- */
/*                    Energy / RMS helpers                                    */
/* -------------------------------------------------------------------------- */
static double calc_rms(const int16_t *buf, uint32_t count)
{
    if (count == 0) return 0.0;
    double sum = 0.0;
    for (uint32_t i = 0; i < count; i++) {
        double v = buf[i];
        sum += v * v;
    }
    return sqrt(sum / count);
}

/* -------------------------------------------------------------------------- */
/*                Cross-correlation helper (unchanged)                        */
/* -------------------------------------------------------------------------- */
static float hwloop_calc_correlation(const int16_t *ref, const int16_t *cap_buf, uint32_t total_samples)
{
    if (total_samples < HWLOOP_REF_SAMPLES) {
        return -1.0f;
    }

    const int16_t *seg = &cap_buf[total_samples - HWLOOP_REF_SAMPLES];
    float best_corr = -1.0f;

    for (int32_t delay = -20; delay <= 20; delay++) {
        int64_t sum_corr = 0, sum_ref2 = 0, sum_cap2 = 0;
        for (uint32_t i = 0; i < HWLOOP_REF_SAMPLES - 40; i++) {
            int32_t idx = i + delay;
            if (idx < 0 || idx >= (int32_t)HWLOOP_REF_SAMPLES) continue;
            int16_t r = ref[idx];
            int16_t c = seg[i];
            sum_corr += (int64_t)r * c;
            sum_ref2 += (int64_t)r * r;
            sum_cap2 += (int64_t)c * c;
        }
        if (sum_ref2 == 0 || sum_cap2 == 0) continue;
        float corr = (float)sum_corr / sqrtf((float)sum_ref2 * (float)sum_cap2);
        if (corr > best_corr) best_corr = corr;
    }
    return best_corr;
}

/* -------------------------------------------------------------------------- */
/*                          Reference signal generators                       */
/* -------------------------------------------------------------------------- */
static void hwloop_gen_ref(void)
{
    tkl_log_output("=== GEN REF V3 (sine) ===\r\n");
    for (uint32_t i = 0; i < HWLOOP_REF_SAMPLES; i++) {
        double t = (double)i / ADC_TEST_DEFAULT_SAMPLE_RATE;
        s_hwloop_ref[i] = (int16_t)(20000.0 * sin(2.0 * M_PI * 1000.0 * t));
    }
}

static void hwloop_gen_ref_pair(void)
{
    tkl_log_output("=== GEN REF PAIR V3 (sine L/R) ===\r\n");
    for (uint32_t i = 0; i < HWLOOP_REF_SAMPLES; i++) {
        double t = (double)i / ADC_TEST_DEFAULT_SAMPLE_RATE;
        s_hwloop_ref[i]   = (int16_t)(20000.0 * sin(2.0 * M_PI * 1000.0 * t));
        s_hwloop_ref_R[i] = (int16_t)(20000.0 * sin(2.0 * M_PI * 1000.0 * t + M_PI)); // 反相
    }
}

/* -------------------------------------------------------------------------- */
/*                        DAC deinit (called internally)                      */
/* -------------------------------------------------------------------------- */
static void test_adc_dac_deinit(void)
{
    if (s_adc_ctx.dac_inited) {
        if (s_adc_ctx.dac_started) {
            (void)tkl_aud_dac_stop(s_adc_ctx.dac_port);
        }
        (void)tkl_aud_dac_deinit(s_adc_ctx.dac_port);
    }
    if (s_adc_ctx.loop_mix_buf != NULL) {
        tal_free(s_adc_ctx.loop_mix_buf);
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
static void adc_upper_cb(TUYA_AUDIO_FRAME_EVT_E event, void *p_buf, 
                         uint32_t buf_len, void *args)
{
    (void)event;
    (void)args;
    if (!s_hwloop_testing) return;

    int16_t *samples = (int16_t *)p_buf;
    uint32_t sample_cnt = buf_len / 2;  // 16-bit mono or stereo interleaved

    uint8_t mode = s_hwloop_mode;
    bool is_stereo_adc = (s_adc_ctx.chan == TUYA_AUDIO_ADC_CHANNEL_LR);

    for (uint32_t i = 0; i < sample_cnt; i++) {
        if (is_stereo_adc) {
            // 立体声采集：L/R 交替
            if (i % 2 == 0) {
                if (s_hwloop_cap_idx_L < HWLOOP_REF_SAMPLES * 4)
                    s_hwloop_cap_L[s_hwloop_cap_idx_L++] = samples[i];
                if (s_hwloop_cap_idx_R < HWLOOP_REF_SAMPLES * 4)
                    s_hwloop_cap_R[s_hwloop_cap_idx_R++] = samples[i + 1];
            }
        } else {
            // 单声道采集
            if (mode == HWLOOP_MODE_MIC3 && s_hwloop_stereo_dac) {
                // MIC3 立体声 DAC 测试：数据同时存入 L/R 缓冲区以便对比
                if (s_hwloop_cap_idx_L < HWLOOP_REF_SAMPLES * 4)
                    s_hwloop_cap_L[s_hwloop_cap_idx_L++] = samples[i];
                if (s_hwloop_cap_idx_R < HWLOOP_REF_SAMPLES * 4)
                    s_hwloop_cap_R[s_hwloop_cap_idx_R++] = samples[i];
            } else if (mode == HWLOOP_MODE_RIGHT) {
                // 右声道单声道：存入 cap_R，与 hwloop_compare_test 的 need_right 对齐
                if (s_hwloop_cap_idx_R < HWLOOP_REF_SAMPLES * 4)
                    s_hwloop_cap_R[s_hwloop_cap_idx_R++] = samples[i];
            } else {
                // 普通单声道（左/MIC3）
                if (s_hwloop_cap_idx_L < HWLOOP_REF_SAMPLES * 4)
                    s_hwloop_cap_L[s_hwloop_cap_idx_L++] = samples[i];
            }
        }
    }
}

static void hwloop_dac_frame_cb(TUYA_AUDIO_DAC_FRAME_EVT_E port, void *p_buf, 
                                uint32_t buf_len, void *args)
{
    (void)port;
    (void)args;

}

/* -------------------------------------------------------------------------- */
/*                   Mono DAC init & start                                    */
/* -------------------------------------------------------------------------- */
static OPERATE_RET test_adc_dac_init_start(uint32_t dac_port, uint32_t sample_rate,
                                           uint32_t frame_time_ms, uint32_t volume,
                                           uint32_t mono_frame_size)
{
    OPERATE_RET ret;
    TKL_AUD_DAC_CFG_T cfg;

    if (dac_port >= TUYA_AUDIO_DAC_PORT_MAX || sample_rate == 0U || frame_time_ms == 0U) {
        return OPRT_INVALID_PARM;
    }

    test_adc_dac_deinit();

    if (s_adc_ctx.chan == TUYA_AUDIO_ADC_CHANNEL_LR) {
        s_adc_ctx.loop_mix_buf = (uint8_t *)tal_malloc(mono_frame_size);
        if (s_adc_ctx.loop_mix_buf == NULL) {
            return OPRT_MALLOC_FAILED;
        }
        s_adc_ctx.loop_mix_buf_size = mono_frame_size;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.volume = (uint8_t)((volume > 100U) ? 100U : volume);
    cfg.chan_num = 1U;
    cfg.sample_bits = TUYA_AUDIO_SAMPLE_BITS_16;
    cfg.sample_rate = sample_rate;
    cfg.frame_time_ms = frame_time_ms;
    cfg.frame_cb = hwloop_dac_frame_cb;
    cfg.args = NULL;

    ret = tkl_aud_dac_init((TUYA_AUDIO_DAC_PORT_E)dac_port, &cfg);
    if (ret != OPRT_OK) {
        test_adc_dac_deinit();
        return ret;
    }
    s_adc_ctx.dac_inited = true;
    s_adc_ctx.dac_port = (TUYA_AUDIO_DAC_PORT_E)dac_port;
    s_adc_ctx.dac_volume = cfg.volume;

    ret = tkl_aud_dac_start(s_adc_ctx.dac_port);
    if (ret != OPRT_OK) {
        test_adc_dac_deinit();
        return ret;
    }
    s_adc_ctx.dac_started = true;
    return OPRT_OK;
}

/* -------------------------------------------------------------------------- */
/*                Stereo DAC init & start                                     */
/* -------------------------------------------------------------------------- */
static OPERATE_RET hwloop_dac_init_start(uint32_t dac_port, uint32_t sample_rate,
                                         uint32_t frame_time_ms, uint32_t volume)
{
    TKL_AUD_DAC_CFG_T cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.volume = (uint8_t)((volume > 100U) ? 100U : volume);
    cfg.chan_num = 2U;
    cfg.sample_bits = TUYA_AUDIO_SAMPLE_BITS_16;
    cfg.sample_rate = sample_rate;
    cfg.frame_time_ms = frame_time_ms;
    cfg.frame_cb = NULL;
    cfg.args = NULL;

    OPERATE_RET ret = tkl_aud_dac_init((TUYA_AUDIO_DAC_PORT_E)dac_port, &cfg);
    if (ret != OPRT_OK) return ret;

    ret = tkl_aud_dac_start((TUYA_AUDIO_DAC_PORT_E)dac_port);
    if (ret != OPRT_OK) {
        tkl_aud_dac_deinit((TUYA_AUDIO_DAC_PORT_E)dac_port);
        return ret;
    }
    s_adc_ctx.dac_inited = true;
    s_adc_ctx.dac_port = (TUYA_AUDIO_DAC_PORT_E)dac_port;
    s_adc_ctx.dac_volume = cfg.volume;
    s_adc_ctx.dac_started = true;
    return OPRT_OK;
}

/* -------------------------------------------------------------------------- */
/*                        ADC init                                            */
/* -------------------------------------------------------------------------- */
static OPERATE_RET test_adc_init(uint32_t port, uint32_t chan, uint32_t sample_rate,
                                 uint32_t frame_time_ms, uint32_t gain)
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
        if (ret != OPRT_OK) return ret;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.chan = (TUYA_AUDIO_ADC_CHAN_E)chan;
    cfg.vol = (uint8_t)gain;
    cfg.sample_bits = ADC_TEST_DEFAULT_SAMPLE_BITS;
    cfg.sample_rate = sample_rate;
    cfg.frame_time_ms = frame_time_ms;
    cfg.upper_cb = adc_upper_cb;   // will be set later?
    cfg.args = NULL;

    ret = tkl_aud_adc_init((TUYA_AUDIO_ADC_PORT_E)port, &cfg);
    if (ret != OPRT_OK) return ret;

    s_adc_ctx.initialized = true;
    s_adc_ctx.started = false;
    s_adc_ctx.port = (TUYA_AUDIO_ADC_PORT_E)port;
    s_adc_ctx.chan = (TUYA_AUDIO_ADC_CHAN_E)chan;
    s_adc_ctx.sample_rate = sample_rate;
    s_adc_ctx.sample_bits = ADC_TEST_DEFAULT_SAMPLE_BITS;
    s_adc_ctx.frame_time_ms = frame_time_ms;
    s_adc_ctx.gain = (uint8_t)gain;
    s_adc_ctx.frame_size = test_adc_calc_frame_size(sample_rate, ADC_TEST_DEFAULT_SAMPLE_BITS,
                                                    frame_time_ms, chan_num);
    s_adc_ctx.frame_cnt = 0U;
    s_adc_ctx.total_bytes = 0U;
    s_adc_ctx.last_frame_bytes = 0U;
    s_adc_ctx.last_rec_frames = 0U;
    s_adc_ctx.last_rec_bytes = 0U;
    return OPRT_OK;
}

/* -------------------------------------------------------------------------- */
/*                         ADC start                                          */
/* -------------------------------------------------------------------------- */
static OPERATE_RET test_adc_start(void)
{
    if (!s_adc_ctx.initialized) {
        return OPRT_RESOURCE_NOT_READY;
    }
    if (s_adc_ctx.started) {
        return OPRT_OK;
    }
    OPERATE_RET ret = tkl_aud_adc_start(s_adc_ctx.port);
    if (ret == OPRT_OK) {
        s_adc_ctx.started = true;
    }
    return ret;
}

/* -------------------------------------------------------------------------- */
/*                        ADC stop / deinit                                   */
/* -------------------------------------------------------------------------- */
static OPERATE_RET test_adc_stop(void)
{
    if (s_adc_ctx.started) {
        return tkl_aud_adc_stop(s_adc_ctx.port);
    }
    return OPRT_OK;
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

/* -------------------------------------------------------------------------- */
/*              Original loop start / stop (preserved)                        */
/* -------------------------------------------------------------------------- */
static OPERATE_RET test_adc_loop_start(uint32_t adc_port, uint32_t chan, uint32_t dac_port,
                                       uint32_t sample_rate, uint32_t frame_time_ms,
                                       uint32_t dac_volume, uint32_t adc_gain)
{
    OPERATE_RET ret;
    uint32_t mono_frame_size;

    ret = test_adc_init(adc_port, chan, sample_rate, frame_time_ms, adc_gain);
    if (ret != OPRT_OK) return ret;
    ret = test_adc_start();
    if (ret != OPRT_OK) {
        (void)test_adc_deinit();
        return ret;
    }

    mono_frame_size = s_adc_ctx.frame_size;
    if (s_adc_ctx.chan == TUYA_AUDIO_ADC_CHANNEL_LR) {
        mono_frame_size = mono_frame_size / 2U;
    }
    ret = test_adc_dac_init_start(dac_port, sample_rate, frame_time_ms, dac_volume, mono_frame_size);
    if (ret != OPRT_OK) {
        (void)test_adc_stop();
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
static inline bool dac_is_stereo_for_mode(uint8_t mode)
{
    return (mode == HWLOOP_MODE_STEREO ||
            mode == HWLOOP_MODE_STEREO_DIFF ||
            mode == HWLOOP_MODE_MIC3_STEREO);
}
/* -------------------------------------------------------------------------- */
/*           Hardware loop compare test (modes 0~3) with energy check         */
/* -------------------------------------------------------------------------- */
static OPERATE_RET hwloop_compare_test(uint8_t mode)
{
    hwloop_gen_ref();
    s_hwloop_mode = mode;
    s_hwloop_stereo_dac = false;

    s_hwloop_cap_idx_L = 0;
    s_hwloop_cap_idx_R = 0;
    s_hwloop_ref_play_idx_L = 0;
    s_hwloop_ref_play_idx_R = 0;
    s_hwloop_testing = true;

    TUYA_AUDIO_ADC_CHAN_E adc_chan;
    if (mode == HWLOOP_MODE_LEFT) {
        adc_chan = TUYA_AUDIO_ADC_CHANNEL_L;
    } else if (mode == HWLOOP_MODE_RIGHT) {
        adc_chan = TUYA_AUDIO_ADC_CHANNEL_R;
    } else if (mode == HWLOOP_MODE_MIC3) {
        adc_chan = TUYA_AUDIO_ADC_CHANNEL_MIC3;
    } else {
        adc_chan = TUYA_AUDIO_ADC_CHANNEL_LR;
    }

    OPERATE_RET ret = test_adc_init(ADC_TEST_DEFAULT_PORT, adc_chan,
                                    ADC_TEST_DEFAULT_SAMPLE_RATE,
                                    ADC_TEST_DEFAULT_FRAME_TIME_MS,
                                    ADC_TEST_DEFAULT_GAIN);
    if (ret != OPRT_OK) goto fail_init;
    ret = test_adc_start();
    if (ret != OPRT_OK) goto fail_adc_start;

    s_adc_ctx.loop_on = false;

    /* DAC 初始化 */
    bool stereo = dac_is_stereo_for_mode(mode);
    if (!stereo) {
        ret = test_adc_dac_init_start(ADC_TEST_DEFAULT_DAC_PORT,
                                      ADC_TEST_DEFAULT_SAMPLE_RATE,
                                      ADC_TEST_DEFAULT_FRAME_TIME_MS,
                                      ADC_TEST_DEFAULT_DAC_VOLUME,
                                      s_adc_ctx.frame_size);
    } else {
        ret = hwloop_dac_init_start(ADC_TEST_DEFAULT_DAC_PORT,
                                    ADC_TEST_DEFAULT_SAMPLE_RATE,
                                    ADC_TEST_DEFAULT_FRAME_TIME_MS,
                                    ADC_TEST_DEFAULT_DAC_VOLUME);
    }
    if (ret != OPRT_OK) goto fail_dac;
    s_adc_ctx.dac_started = true;

 /* ---------- 主动写入 2s 参考数据 ---------- */
    const uint32_t dac_frame_samples = (ADC_TEST_DEFAULT_SAMPLE_RATE * ADC_TEST_DEFAULT_FRAME_TIME_MS) / 1000; // 960
    const uint32_t dac_channels = stereo ? 2 : 1;
    const uint32_t dac_frame_bytes = dac_frame_samples * dac_channels * 2;
    uint8_t *dac_buf = (uint8_t *)tal_malloc(dac_frame_bytes);
    if (dac_buf != NULL) {
        uint32_t ref_idx = 0;
        for (int f = 0; f < 100; f++) {   // 100 帧 × 20ms = 2000ms = 2s
            if (!stereo) {
                int16_t *p = (int16_t *)dac_buf;
                for (uint32_t i = 0; i < dac_frame_samples; i++) {
                    p[i] = s_hwloop_ref[ref_idx % HWLOOP_REF_SAMPLES];
                    ref_idx++;
                }
            } else {
                int16_t *p = (int16_t *)dac_buf;
                for (uint32_t i = 0; i < dac_frame_samples; i++) {
                    p[i * 2]     = s_hwloop_ref[ref_idx % HWLOOP_REF_SAMPLES];
                    p[i * 2 + 1] = s_hwloop_ref_R[ref_idx % HWLOOP_REF_SAMPLES];
                    ref_idx++;
                }
            }
            tkl_aud_dac_write(s_adc_ctx.dac_port, dac_buf, dac_frame_bytes);
            tal_system_sleep(ADC_TEST_DEFAULT_FRAME_TIME_MS);
        }
        tal_free(dac_buf);
    } else {
        /* 内存分配失败，简单延时，但无信号输出 */
        tal_system_sleep(2000);
    }
    /* ---------------------------------------------- */

    s_hwloop_testing = false;
    test_adc_dac_deinit();
    (void)test_adc_stop();
    (void)test_adc_deinit();

    /* ---- 打印参考和采集数据 ---- */
    uint32_t dump_cnt = 32;
    tkl_log_output("[HWLOOP MODE %u] REF: ", mode);
    for (uint32_t i = 0; i < dump_cnt && i < HWLOOP_REF_SAMPLES; i++) {
        tkl_log_output("%d ", s_hwloop_ref[i]);
    }
    tkl_log_output("\r\n");

    bool need_left = (mode == HWLOOP_MODE_LEFT || mode == HWLOOP_MODE_STEREO || mode == HWLOOP_MODE_MIC3);
    bool need_right = (mode == HWLOOP_MODE_RIGHT || mode == HWLOOP_MODE_STEREO);
    if (need_left) {
        tkl_log_output("CAP_L: ");
        for (uint32_t i = 0; i < dump_cnt && i < s_hwloop_cap_idx_L; i++) {
            tkl_log_output("%d ", s_hwloop_cap_L[i]);
        }
        tkl_log_output("\r\n");
    }
    if (need_right) {
        tkl_log_output("CAP_R: ");
        for (uint32_t i = 0; i < dump_cnt && i < s_hwloop_cap_idx_R; i++) {
            tkl_log_output("%d ", s_hwloop_cap_R[i]);
        }
        tkl_log_output("\r\n");
    }

    /* ---- 能量检测 ---- */
    const double min_rms = 100.0;
    double rms_L = 0, rms_R = 0;
    if (need_left) {
        rms_L = calc_rms(s_hwloop_cap_L, s_hwloop_cap_idx_L > HWLOOP_REF_SAMPLES ? HWLOOP_REF_SAMPLES : s_hwloop_cap_idx_L);
        tkl_log_output("CAP_L RMS: %.1f\r\n", rms_L);
    }
    if (need_right) {
        rms_R = calc_rms(s_hwloop_cap_R, s_hwloop_cap_idx_R > HWLOOP_REF_SAMPLES ? HWLOOP_REF_SAMPLES : s_hwloop_cap_idx_R);
        tkl_log_output("CAP_R RMS: %.1f\r\n", rms_R);
    }

    /* ---- 相关性计算与最终判决 ---- */
    float corr_L = -1.0f, corr_R = -1.0f;
    if (need_left && rms_L >= min_rms) {
        corr_L = hwloop_calc_correlation(s_hwloop_ref, s_hwloop_cap_L, s_hwloop_cap_idx_L);
    }
    if (need_right && rms_R >= min_rms) {
        corr_R = hwloop_calc_correlation(s_hwloop_ref, s_hwloop_cap_R, s_hwloop_cap_idx_R);
    }

    const float threshold = 0.8f;
    bool pass = false;
    if (mode == HWLOOP_MODE_LEFT && corr_L >= threshold) {
        pass = true;
    } else if (mode == HWLOOP_MODE_RIGHT && corr_R >= threshold) {
        pass = true;
    } else if (mode == HWLOOP_MODE_MIC3 && corr_L >= threshold) {
        pass = true;
    } else if (mode == HWLOOP_MODE_STEREO && corr_L >= threshold && corr_R >= threshold) {
        pass = true;
    }

    return pass ? OPRT_OK : OPRT_COM_ERROR;

fail_dac:
    test_adc_stop();
fail_adc_start:
    test_adc_deinit();
fail_init:
    s_hwloop_testing = false;
    return ret;
}

/* -------------------------------------------------------------------------- */
/*   New stereo different reference test (MIC1+MIC2) with energy check        */
/* -------------------------------------------------------------------------- */
static OPERATE_RET do_stereo_diff_loopback(bool *pass_mic1, bool *pass_mic2)
{
    if (pass_mic1 == NULL || pass_mic2 == NULL) return OPRT_INVALID_PARM;
    *pass_mic1 = false;
    *pass_mic2 = false;

    hwloop_gen_ref_pair();
    s_hwloop_mode = HWLOOP_MODE_STEREO_DIFF;
    s_hwloop_stereo_dac = false;

    s_hwloop_cap_idx_L = 0;
    s_hwloop_cap_idx_R = 0;
    s_hwloop_ref_play_idx_L = 0;
    s_hwloop_ref_play_idx_R = 0;
    s_hwloop_testing = true;

    OPERATE_RET ret = test_adc_init(ADC_TEST_DEFAULT_PORT,
                                    TUYA_AUDIO_ADC_CHANNEL_LR,
                                    ADC_TEST_DEFAULT_SAMPLE_RATE,
                                    ADC_TEST_DEFAULT_FRAME_TIME_MS,
                                    ADC_TEST_DEFAULT_GAIN);
    if (ret != OPRT_OK) return ret;
    ret = test_adc_start();
    if (ret != OPRT_OK) {
        test_adc_deinit();
        return ret;
    }

    s_adc_ctx.loop_on = false;
    ret = hwloop_dac_init_start(ADC_TEST_DEFAULT_DAC_PORT,
                                ADC_TEST_DEFAULT_SAMPLE_RATE,
                                ADC_TEST_DEFAULT_FRAME_TIME_MS,
                                ADC_TEST_DEFAULT_DAC_VOLUME);
    if (ret != OPRT_OK) {
        test_adc_stop();
        test_adc_deinit();
        return ret;
    }
    s_adc_ctx.dac_started = true;

        /* ---------- 主动写入 2s 立体声参考数据 ---------- */
    const uint32_t dac_frame_samples = (ADC_TEST_DEFAULT_SAMPLE_RATE * ADC_TEST_DEFAULT_FRAME_TIME_MS) / 1000;
    const uint32_t dac_frame_bytes = dac_frame_samples * 2 * 2;  // 立体声 2 通道 × 2 字节
    uint8_t *dac_buf = (uint8_t *)tal_malloc(dac_frame_bytes);
    if (dac_buf != NULL) {
        uint32_t ref_idx = 0;
        for (int f = 0; f < 100; f++) {   // 100 帧 = 2 秒
            int16_t *p = (int16_t *)dac_buf;
            for (uint32_t i = 0; i < dac_frame_samples; i++) {
                p[i * 2]     = s_hwloop_ref[ref_idx % HWLOOP_REF_SAMPLES];
                p[i * 2 + 1] = s_hwloop_ref_R[ref_idx % HWLOOP_REF_SAMPLES];
                ref_idx++;
            }
            tkl_aud_dac_write(s_adc_ctx.dac_port, dac_buf, dac_frame_bytes);
            tal_system_sleep(ADC_TEST_DEFAULT_FRAME_TIME_MS);
        }
        tal_free(dac_buf);
    } else {
        tal_system_sleep(2000);
    }
    /* -------------------------------------------------- */

    s_hwloop_testing = false;
    test_adc_dac_deinit();
    test_adc_stop();
    test_adc_deinit();

    /* ---- 打印数据 ---- */
    uint32_t dump_cnt = 32;
    tkl_log_output("[HWLOOP STEREO DIFF] REF_L: ");
    for (uint32_t i = 0; i < dump_cnt && i < HWLOOP_REF_SAMPLES; i++) {
        tkl_log_output("%d ", s_hwloop_ref[i]);
    }
    tkl_log_output("\r\nREF_R: ");
    for (uint32_t i = 0; i < dump_cnt && i < HWLOOP_REF_SAMPLES; i++) {
        tkl_log_output("%d ", s_hwloop_ref_R[i]);
    }
    tkl_log_output("\r\nCAP_L: ");
    for (uint32_t i = 0; i < dump_cnt && i < s_hwloop_cap_idx_L; i++) {
        tkl_log_output("%d ", s_hwloop_cap_L[i]);
    }
    tkl_log_output("\r\nCAP_R: ");
    for (uint32_t i = 0; i < dump_cnt && i < s_hwloop_cap_idx_R; i++) {
        tkl_log_output("%d ", s_hwloop_cap_R[i]);
    }
    tkl_log_output("\r\n");

    /* ---- 能量 / 相关性检测 ---- */
    const double min_rms = 100.0;
    double rms_L = calc_rms(s_hwloop_cap_L, s_hwloop_cap_idx_L > HWLOOP_REF_SAMPLES ? HWLOOP_REF_SAMPLES : s_hwloop_cap_idx_L);
    double rms_R = calc_rms(s_hwloop_cap_R, s_hwloop_cap_idx_R > HWLOOP_REF_SAMPLES ? HWLOOP_REF_SAMPLES : s_hwloop_cap_idx_R);
    tkl_log_output("CAP_L RMS: %.1f, CAP_R RMS: %.1f\r\n", rms_L, rms_R);

    float corr_L = (rms_L >= min_rms) ? hwloop_calc_correlation(s_hwloop_ref, s_hwloop_cap_L, s_hwloop_cap_idx_L) : -1.0f;
    float corr_R = (rms_R >= min_rms) ? hwloop_calc_correlation(s_hwloop_ref_R, s_hwloop_cap_R, s_hwloop_cap_idx_R) : -1.0f;

    const float threshold = 0.8f;
    *pass_mic1 = (corr_L >= threshold);
    *pass_mic2 = (corr_R >= threshold);

    return (*pass_mic1 && *pass_mic2) ? OPRT_OK : OPRT_COM_ERROR;
}

/* -------------------------------------------------------------------------- */
/*   MIC3 loopback test using stereo DAC with energy check                    */
/* -------------------------------------------------------------------------- */
static OPERATE_RET do_mic3_stereo_dac_loopback(bool *pass_mic3)
{
    if (pass_mic3 == NULL) return OPRT_INVALID_PARM;
    *pass_mic3 = false;

    hwloop_gen_ref();                         // 使用单声道参考
    /* 因为用立体声 DAC 播放同一信号，将 s_hwloop_ref 复制到 R 通道 */
    memcpy(s_hwloop_ref_R, s_hwloop_ref, sizeof(s_hwloop_ref_R));

    s_hwloop_mode = HWLOOP_MODE_MIC3_STEREO;
    s_hwloop_stereo_dac = true;

    s_hwloop_cap_idx_L = 0;
    s_hwloop_ref_play_idx_L = 0;
    s_hwloop_testing = true;

    OPERATE_RET ret = test_adc_init(ADC_TEST_DEFAULT_PORT,
                                    TUYA_AUDIO_ADC_CHANNEL_MIC3,
                                    ADC_TEST_DEFAULT_SAMPLE_RATE,
                                    ADC_TEST_DEFAULT_FRAME_TIME_MS,
                                    ADC_TEST_DEFAULT_GAIN);
    if (ret != OPRT_OK) return ret;
    ret = test_adc_start();
    if (ret != OPRT_OK) {
        test_adc_deinit();
        return ret;
    }

    s_adc_ctx.loop_on = false;
    ret = hwloop_dac_init_start(ADC_TEST_DEFAULT_DAC_PORT,
                                ADC_TEST_DEFAULT_SAMPLE_RATE,
                                ADC_TEST_DEFAULT_FRAME_TIME_MS,
                                ADC_TEST_DEFAULT_DAC_VOLUME);
    if (ret != OPRT_OK) {
        test_adc_stop();
        test_adc_deinit();
        return ret;
    }
    s_adc_ctx.dac_started = true;

    /* ---------- 主动写入 2s 立体声参考（左右相同） ---------- */
    const uint32_t dac_frame_samples = (ADC_TEST_DEFAULT_SAMPLE_RATE * ADC_TEST_DEFAULT_FRAME_TIME_MS) / 1000;
    const uint32_t dac_frame_bytes = dac_frame_samples * 2 * 2;  // 立体声
    uint8_t *dac_buf = (uint8_t *)tal_malloc(dac_frame_bytes);
    if (dac_buf != NULL) {
        uint32_t ref_idx = 0;
        for (int f = 0; f < 100; f++) {   // 100 帧 = 2 秒
            int16_t *p = (int16_t *)dac_buf;
            for (uint32_t i = 0; i < dac_frame_samples; i++) {
                p[i * 2]     = s_hwloop_ref[ref_idx % HWLOOP_REF_SAMPLES];
                p[i * 2 + 1] = s_hwloop_ref_R[ref_idx % HWLOOP_REF_SAMPLES];
                ref_idx++;
            }
            tkl_aud_dac_write(s_adc_ctx.dac_port, dac_buf, dac_frame_bytes);
            tal_system_sleep(ADC_TEST_DEFAULT_FRAME_TIME_MS);
        }
        tal_free(dac_buf);
    } else {
        tal_system_sleep(2000);
    }
    /* ----------------------------------------------------------- */

    s_hwloop_testing = false;
    s_hwloop_stereo_dac = false;
    test_adc_dac_deinit();
    test_adc_stop();
    test_adc_deinit();

    /* ---- 打印数据 ---- */
    uint32_t dump_cnt = 32;
    tkl_log_output("[HWLOOP MIC3 STEREO] REF: ");
    for (uint32_t i = 0; i < dump_cnt && i < HWLOOP_REF_SAMPLES; i++) {
        tkl_log_output("%d ", s_hwloop_ref[i]);
    }
    tkl_log_output("\r\nCAP_L: ");
    for (uint32_t i = 0; i < dump_cnt && i < s_hwloop_cap_idx_L; i++) {
        tkl_log_output("%d ", s_hwloop_cap_L[i]);
    }
    tkl_log_output("\r\n");
    tkl_log_output("\r\nCAP_R: ");
    for (uint32_t i = 0; i < dump_cnt && i < s_hwloop_cap_idx_L; i++) {   // 注意：此处原代码用的 s_hwloop_cap_idx_L 打印两个缓冲区，保持
        tkl_log_output("%d ", s_hwloop_cap_R[i]);
    }
    tkl_log_output("\r\n");

    /* ---- 能量 / 相关性检测 ---- */
    const double min_rms = 100.0;
    double rms = calc_rms(s_hwloop_cap_L, s_hwloop_cap_idx_L > HWLOOP_REF_SAMPLES ? HWLOOP_REF_SAMPLES : s_hwloop_cap_idx_L);
    tkl_log_output("CAP_L RMS: %.1f\r\n", rms);

    float corr = (rms >= min_rms) ? hwloop_calc_correlation(s_hwloop_ref, s_hwloop_cap_L, s_hwloop_cap_idx_L) : -1.0f;
    *pass_mic3 = (corr >= 0.8f);

    return (*pass_mic3) ? OPRT_OK : OPRT_COM_ERROR;
}

/* -------------------------------------------------------------------------- */
/*   Combined MIC1, MIC2, MIC3 test and JSON output                           */
/* -------------------------------------------------------------------------- */
static OPERATE_RET do_all_mic_loopback(char **json_out)
{
    bool mic1_pass = false, mic2_pass = false, mic3_pass = false;
    OPERATE_RET ret;

    ret = do_stereo_diff_loopback(&mic1_pass, &mic2_pass);
    if (ret != OPRT_OK && ret != OPRT_COM_ERROR) {
        mic1_pass = false;
        mic2_pass = false;
    }

    ret = do_mic3_stereo_dac_loopback(&mic3_pass);
    if (ret != OPRT_OK && ret != OPRT_COM_ERROR) {
        mic3_pass = false;
    }

    bool all_pass = mic1_pass && mic2_pass && mic3_pass;
    static char json_buf[256];
    int len = snprintf(json_buf, sizeof(json_buf),
        "{\"ret\":%s,\"data\":{\"mic1\":%s,\"mic2\":%s,\"mic3\":%s}}",
        all_pass ? "true" : "false",
        mic1_pass ? "true" : "false",
        mic2_pass ? "true" : "false",
        mic3_pass ? "true" : "false");

    if (len < 0 || (size_t)len >= sizeof(json_buf)) {
        return OPRT_MALLOC_FAILED;
    }

    UCHAR_T *p = (UCHAR_T *)tal_malloc(len);
    if (p == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    memcpy(p, json_buf, len);
    *json_out = (char *)p;
    return all_pass ? OPRT_OK : OPRT_COM_ERROR;
}

/* -------------------------------------------------------------------------- */
/*                   Public manufacturing test dispatch                       */
/* -------------------------------------------------------------------------- */
OPERATE_RET tkl_mftest_ctrl(USHORT_T cmd, UCHAR_T *in, UINT_T inlen,
                            UCHAR_T **out, USHORT_T *outlen)
{
    if (out != NULL) {
        *out = NULL;
    }
    if (outlen != NULL) {
        *outlen = 0;
    }

    switch (cmd) {
    case TKL_AUDIO_TEST_START_CMD: {
        uint8_t mode = HWLOOP_MODE_STEREO;
        if (in != NULL && inlen >= 1) {
            mode = in[0];
        }

        /* Handle new modes */
        if (mode == HWLOOP_MODE_STEREO_DIFF) {
            bool mic1_pass = false, mic2_pass = false;
            OPERATE_RET ret = do_stereo_diff_loopback(&mic1_pass, &mic2_pass);
            if (ret != OPRT_OK && ret != OPRT_COM_ERROR) {
                return ret;
            }
            bool overall_pass = (ret == OPRT_OK);
            const char *json_fmt = "{\"ret\":%s,\"data\":{\"mic1\":%s,\"mic2\":%s}}";
            static char out_json[128];
            snprintf(out_json, sizeof(out_json), json_fmt,
                     overall_pass ? "true" : "false",
                     mic1_pass ? "true" : "false",
                     mic2_pass ? "true" : "false");
            size_t len = strlen(out_json);
            UCHAR_T *p = (UCHAR_T *)tal_malloc(len);
            if (p == NULL) return OPRT_MALLOC_FAILED;
            memcpy(p, out_json, len);
            *out = p;
            *outlen = (USHORT_T)len;
            return OPRT_OK;
        } else if (mode == HWLOOP_MODE_MIC3_STEREO) {
            bool mic3_pass = false;
            OPERATE_RET ret = do_mic3_stereo_dac_loopback(&mic3_pass);
            if (ret != OPRT_OK && ret != OPRT_COM_ERROR) {
                return ret;
            }
            bool overall_pass = (ret == OPRT_OK);
            const char *json_fmt = "{\"ret\":%s,\"data\":{\"mic3\":%s}}";
            static char out_json[64];
            snprintf(out_json, sizeof(out_json), json_fmt,
                     overall_pass ? "true" : "false",
                     mic3_pass ? "true" : "false");
            size_t len = strlen(out_json);
            UCHAR_T *p = (UCHAR_T *)tal_malloc(len);
            if (p == NULL) return OPRT_MALLOC_FAILED;
            memcpy(p, out_json, len);
            *out = p;
            *outlen = (USHORT_T)len;
            return OPRT_OK;
        } else if (mode == 6) {
            char *json_str = NULL;
            OPERATE_RET ret = do_all_mic_loopback(&json_str);
            if (ret == OPRT_MALLOC_FAILED || json_str == NULL) {
                return ret;
            }
            size_t len = strlen(json_str);
            *out = (UCHAR_T *)json_str;
            *outlen = (USHORT_T)len;
            return OPRT_OK;
        }

        /* Original modes 0~3 */
        if (mode > HWLOOP_MODE_MIC3) {
            mode = HWLOOP_MODE_STEREO;
        }
        OPERATE_RET ret = hwloop_compare_test(mode);
        const char *json_str = NULL;
        if (ret == OPRT_OK) {
            static const char k_pass[] = "{\"ret\":true,\"data\":\"Loopback_Pass\"}";
            json_str = k_pass;
        } else {
            static const char k_err_left[]   = "{\"ret\":false,\"errcode\":500001,\"errMsg\":\"MIC1 No signal\"}";
            static const char k_err_right[]  = "{\"ret\":false,\"errcode\":500002,\"errMsg\":\"MIC2 No signal\"}";
            static const char k_err_stereo[] = "{\"ret\":false,\"errcode\":500003,\"errMsg\":\"Stereo loopback failed\"}";
            static const char k_err_mic3[]   = "{\"ret\":false,\"errcode\":500004,\"errMsg\":\"AEC MIC3 No signal\"}";
            switch (mode) {
                case HWLOOP_MODE_LEFT:   json_str = k_err_left;   break;
                case HWLOOP_MODE_RIGHT:  json_str = k_err_right;  break;
                case HWLOOP_MODE_STEREO: json_str = k_err_stereo; break;
                case HWLOOP_MODE_MIC3:   json_str = k_err_mic3;   break;
                default:                 json_str = k_err_stereo; break;
            }
        }
        size_t json_len = strlen(json_str);
        if (out == NULL || outlen == NULL) {
            return OPRT_INVALID_PARM;
        }
        UCHAR_T *p = (UCHAR_T *)tal_malloc(json_len);
        if (p == NULL) {
            return OPRT_MALLOC_FAILED;
        }
        memcpy(p, json_str, json_len);
        *out = p;
        *outlen = (USHORT_T)json_len;
        return OPRT_OK;
    }

    case TKL_AUDIO_TEST_STOP_CMD:
        if (s_hwloop_testing) {
            s_hwloop_testing = false;
            test_adc_dac_deinit();
            test_adc_stop();
            test_adc_deinit();
        }
        return OPRT_OK;

    case TKL_USB_INFO_TEST_CMD:
        return __pt_mcu_get_usb_info(cmd, in, inlen, out, outlen);

    case TKL_MIPICN1_TEST_CMD:
    case TKL_MIPICN2_TEST_CMD:
        return __pt_mipi_test(cmd, in, inlen, out, outlen);

    case TKL_ADC_TEST_CMD:
        return __pt_adc_test(cmd, in, inlen, out, outlen);

    default:
        return OPRT_NOT_SUPPORTED;
    }
}

VOID tkl_mftest_auto_run(VOID)
{
    bool pass[4] = {false};   // 0: left, 1: right, 2: stereo, 3: mic3
    bool mic1_pass = false, mic2_pass = false, mic3_pass = false;

    tkl_log_output("=== Auto MFT start ===\r\n");

    // 模式 0 - 左声道
    pass[0] = (hwloop_compare_test(HWLOOP_MODE_LEFT) == OPRT_OK);
    tkl_log_output("MFT mode 0 (Left) : %s\r\n", pass[0] ? "PASS" : "FAIL");

    // 模式 1 - 右声道
    pass[1] = (hwloop_compare_test(HWLOOP_MODE_RIGHT) == OPRT_OK);
    tkl_log_output("MFT mode 1 (Right): %s\r\n", pass[1] ? "PASS" : "FAIL");

    // 模式 2 - 立体声
    pass[2] = (hwloop_compare_test(HWLOOP_MODE_STEREO) == OPRT_OK);
    tkl_log_output("MFT mode 2 (Stereo): %s\r\n", pass[2] ? "PASS" : "FAIL");

    // 模式 3 - MIC3 环回
    pass[3] = (hwloop_compare_test(HWLOOP_MODE_MIC3) == OPRT_OK);
    tkl_log_output("MFT mode 3 (MIC3): %s\r\n", pass[3] ? "PASS" : "FAIL");

    // 模式 4 - MIC1+MIC2 差分参考
    (void)do_stereo_diff_loopback(&mic1_pass, &mic2_pass);
    tkl_log_output("MFT mode 4 (StereoDiff): MIC1=%s MIC2=%s\r\n",
                   mic1_pass ? "PASS" : "FAIL", mic2_pass ? "PASS" : "FAIL");

    // 模式 5 - MIC3 立体声 DAC
    (void)do_mic3_stereo_dac_loopback(&mic3_pass);
    tkl_log_output("MFT mode 5 (MIC3 Stereo): %s\r\n", mic3_pass ? "PASS" : "FAIL");

    // 模式 6 实际就是 4+5 的联合，已在上面执行，可直接用上面结果
    bool all_pass = pass[0] && pass[1] && pass[2] && pass[3] &&
                    mic1_pass && mic2_pass && mic3_pass;

    // 输出汇总 JSON
    char json_buf[512];
    snprintf(json_buf, sizeof(json_buf),
        "{\"ret\":%s,\"data\":{"
        "\"left\":%s,\"right\":%s,\"stereo\":%s,\"mic3\":%s,"
        "\"mic1_diff\":%s,\"mic2_diff\":%s,\"mic3_stereo\":%s}}",
        all_pass ? "true" : "false",
        pass[0] ? "true" : "false",
        pass[1] ? "true" : "false",
        pass[2] ? "true" : "false",
        pass[3] ? "true" : "false",
        mic1_pass ? "true" : "false",
        mic2_pass ? "true" : "false",
        mic3_pass ? "true" : "false");
    tkl_log_output("Auto MFT result: %s\r\n", json_buf);
}
/* -------------------------------------------------------------------------- */
/*                     Static helper implementations                          */
/* -------------------------------------------------------------------------- */
static uint32_t test_adc_channel_count(TUYA_AUDIO_ADC_CHAN_E chan)
{
    switch (chan) {
        case TUYA_AUDIO_ADC_CHANNEL_L:
        case TUYA_AUDIO_ADC_CHANNEL_R:
            return 1U;
        case TUYA_AUDIO_ADC_CHANNEL_LR:
            return 2U;
        default:
            return 0U;
    }
}

static uint32_t test_adc_calc_frame_size(uint32_t sample_rate, uint8_t sample_bits,
                                         uint32_t frame_time_ms, uint32_t chan_num)
{
    uint32_t bytes_per_sample = (sample_bits == 16U) ? 2U : 4U;
    uint32_t samples_per_channel = (sample_rate * frame_time_ms) / 1000U;
    return samples_per_channel * chan_num * bytes_per_sample;
}

static void test_adc_runtime_reset(void)
{
    s_adc_ctx.initialized = false;
    s_adc_ctx.started = false;
    s_adc_ctx.frame_cnt = 0U;
    s_adc_ctx.total_bytes = 0U;
    s_adc_ctx.last_frame_bytes = 0U;
    s_adc_ctx.last_rec_frames = 0U;
    s_adc_ctx.last_rec_bytes = 0U;
}