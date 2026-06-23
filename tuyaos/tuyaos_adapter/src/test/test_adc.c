/*
 * test_adc.c
 * Copyright (C) 2024 cc <cc@tuya>
 *
 * Distributed under terms of the MIT license.
 */

#include <stdint.h>
#include <stdbool.h>

#include <os/mem.h>
#include <os/str.h>

#include "FreeRTOS.h"
#include "task.h"

#include "cli_tuya_test.h"
#include "tkl_adc.h"
#include "tkl_pinmux.h"
#include "tkl_system.h"
#include "tuya_error_code.h"

extern void bk_printf(const char *fmt, ...);

#define TKL_ADC_TEST_DEFAULT_PORT       TUYA_ADC_NUM_0
#define TKL_ADC_TEST_DEFAULT_PIN        25U
#define TKL_ADC_TEST_DEFAULT_PIN_2      24U
#define TKL_ADC_TEST_DEFAULT_CONV_CNT   8U
#define TKL_ADC_TEST_MAX_CONV_CNT       32U
#define TKL_ADC_TEST_MAX_CH_NUM         4U
#define TKL_ADC_TEST_DEFAULT_PERIOD_MS  1000U

typedef enum {
    TKL_ADC_TEST_READ_RAW = 0,
    TKL_ADC_TEST_READ_VOLTAGE,
} TKL_ADC_TEST_READ_MODE_E;

typedef struct {
    TaskHandle_t handle;
    volatile bool stop;
    uint32_t pin;
    uint32_t conv_cnt;
    uint32_t period_ms;
    TKL_ADC_TEST_READ_MODE_E mode;
} TKL_ADC_TEST_LOOP_T;

static TKL_ADC_TEST_LOOP_T s_tkl_adc_loop = {
    .handle = NULL,
    .stop = false,
    .pin = TKL_ADC_TEST_DEFAULT_PIN,
    .conv_cnt = TKL_ADC_TEST_DEFAULT_CONV_CNT,
    .period_ms = TKL_ADC_TEST_DEFAULT_PERIOD_MS,
    .mode = TKL_ADC_TEST_READ_VOLTAGE,
};

static void tkl_adc_test_usage(void)
{
    bk_printf("Usage:\r\n");
    bk_printf("  xtkladc map [pin]\r\n");
    bk_printf("  xtkladc info\r\n");
    bk_printf("  xtkladc raw [pin] [cnt]\r\n");
    bk_printf("  xtkladc single [pin] [cnt]\r\n");
    bk_printf("  xtkladc volt [pin] [cnt]\r\n");
    bk_printf("  xtkladc multi [cnt] [pin0] [pin1] ...\r\n");
    bk_printf("  xtkladc div <pin> <open|close>\r\n");
    bk_printf("  xtkladc loop_start [pin] [period_ms] [cnt] [raw|volt]\r\n");
    bk_printf("  xtkladc loop_stop\r\n");
    bk_printf("  xtkladc errtest [pin]\r\n");
    bk_printf("  xtkladc selftest [pin]\r\n");
}

static uint32_t tkl_adc_test_arg_u32(int argc, char **argv, int index, uint32_t def)
{
    if ((index >= argc) || (argv[index] == NULL)) {
        return def;
    }

    return (uint32_t)os_strtoul(argv[index], NULL, 0);
}

static uint32_t tkl_adc_test_sanitize_cnt(uint32_t cnt)
{
    if (cnt == 0U) {
        return 1U;
    }

    if (cnt > TKL_ADC_TEST_MAX_CONV_CNT) {
        return TKL_ADC_TEST_MAX_CONV_CNT;
    }

    return cnt;
}

static INT32_T tkl_adc_test_pin_to_channel(uint32_t pin)
{
    INT32_T chan = tkl_io_pin_to_func((TUYA_PIN_NAME_E)pin, TUYA_IO_TYPE_ADC);

    if ((chan < 0) || (chan >= 16)) {
        return -1;
    }

    return chan;
}

static void tkl_adc_test_print_ret(const char *op, OPERATE_RET ret)
{
    bk_printf("%s %s (%d)\r\n", op, (ret == OPRT_OK) ? "ok" : "fail", ret);
}

static void tkl_adc_test_print_samples(const char *tag, uint32_t pin, INT32_T chan,
                                       const INT32_T *samples, uint32_t cnt,
                                       const char *unit)
{
    INT32_T min_v;
    INT32_T max_v;
    int64_t sum = 0;

    if ((samples == NULL) || (cnt == 0U)) {
        bk_printf("%s pin=%u ch=%d no data\r\n", tag, pin, chan);
        return;
    }

    min_v = samples[0];
    max_v = samples[0];
    bk_printf("%s pin=%u ch=%d cnt=%u data=", tag, pin, chan, cnt);
    for (uint32_t i = 0; i < cnt; i++) {
        if (samples[i] < min_v) {
            min_v = samples[i];
        }
        if (samples[i] > max_v) {
            max_v = samples[i];
        }
        sum += samples[i];
        bk_printf("%s%d", (i == 0U) ? "" : ",", samples[i]);
    }
    bk_printf(" %s min=%d max=%d avg=%d\r\n", unit, min_v, max_v, (INT32_T)(sum / cnt));
}

static OPERATE_RET tkl_adc_test_init_channels(const uint32_t *pins, uint32_t ch_num,
                                              uint32_t conv_cnt, TUYA_ADC_MODE_E mode,
                                              TUYA_ADC_BASE_CFG_T *cfg)
{
    if ((pins == NULL) || (cfg == NULL) || (ch_num == 0U) ||
        (ch_num > TKL_ADC_TEST_MAX_CH_NUM)) {
        return OPRT_INVALID_PARM;
    }

    os_memset(cfg, 0, sizeof(*cfg));
    cfg->ch_nums = ch_num;
    cfg->type = TUYA_ADC_INNER_SAMPLE_VOL;
    cfg->width = 12;
    cfg->mode = mode;
    cfg->conv_cnt = (UINT16_T)tkl_adc_test_sanitize_cnt(conv_cnt);
    cfg->ref_vol = tkl_adc_ref_voltage_get(TKL_ADC_TEST_DEFAULT_PORT);

    for (uint32_t i = 0; i < ch_num; i++) {
        INT32_T chan = tkl_adc_test_pin_to_channel(pins[i]);
        if (chan < 0) {
            bk_printf("invalid adc pin:%u\r\n", pins[i]);
            return OPRT_INVALID_PARM;
        }
        cfg->ch_list.data |= (1UL << (uint32_t)chan);
    }

    return tkl_adc_init(TKL_ADC_TEST_DEFAULT_PORT, cfg);
}

static OPERATE_RET tkl_adc_test_read_one(uint32_t pin, uint32_t conv_cnt,
                                         TKL_ADC_TEST_READ_MODE_E mode,
                                         bool use_single_api)
{
    TUYA_ADC_BASE_CFG_T cfg;
    INT32_T samples[TKL_ADC_TEST_MAX_CONV_CNT] = {0};
    INT32_T chan = tkl_adc_test_pin_to_channel(pin);
    OPERATE_RET ret;
    uint32_t cnt = tkl_adc_test_sanitize_cnt(conv_cnt);

    if (chan < 0) {
        bk_printf("invalid adc pin:%u\r\n", pin);
        return OPRT_INVALID_PARM;
    }

    ret = tkl_adc_test_init_channels(&pin, 1U, cnt, TUYA_ADC_CONTINUOUS, &cfg);
    if (ret != OPRT_OK) {
        tkl_adc_test_print_ret("init", ret);
        return ret;
    }

    if (use_single_api) {
        ret = tkl_adc_read_single_channel(TKL_ADC_TEST_DEFAULT_PORT, (UINT8_T)chan, samples);
    } else if (mode == TKL_ADC_TEST_READ_VOLTAGE) {
        ret = tkl_adc_read_voltage(TKL_ADC_TEST_DEFAULT_PORT, samples, (UINT16_T)cnt);
    } else {
        ret = tkl_adc_read_data(TKL_ADC_TEST_DEFAULT_PORT, samples, (UINT16_T)cnt);
    }

    if (ret == OPRT_OK) {
        tkl_adc_test_print_samples(use_single_api ? "single" :
                                   ((mode == TKL_ADC_TEST_READ_VOLTAGE) ? "volt" : "raw"),
                                   pin, chan, samples, cnt,
                                   (mode == TKL_ADC_TEST_READ_VOLTAGE) ? "mv" : "raw");
    } else {
        tkl_adc_test_print_ret("read", ret);
    }

    tkl_adc_deinit(TKL_ADC_TEST_DEFAULT_PORT);

    return ret;
}

static OPERATE_RET tkl_adc_test_multi(int argc, char **argv)
{
    TUYA_ADC_BASE_CFG_T cfg;
    uint32_t cnt = tkl_adc_test_sanitize_cnt(tkl_adc_test_arg_u32(argc, argv, 2,
                                                                  TKL_ADC_TEST_DEFAULT_CONV_CNT));
    uint32_t pins[TKL_ADC_TEST_MAX_CH_NUM] = {
        TKL_ADC_TEST_DEFAULT_PIN,
        TKL_ADC_TEST_DEFAULT_PIN_2,
    };
    uint32_t ch_num = 2U;
    INT32_T raw[TKL_ADC_TEST_MAX_CH_NUM * TKL_ADC_TEST_MAX_CONV_CNT] = {0};
    INT32_T volt[TKL_ADC_TEST_MAX_CH_NUM * TKL_ADC_TEST_MAX_CONV_CNT] = {0};
    OPERATE_RET ret;

    if (argc > 3) {
        ch_num = (uint32_t)(argc - 3);
        if (ch_num > TKL_ADC_TEST_MAX_CH_NUM) {
            ch_num = TKL_ADC_TEST_MAX_CH_NUM;
        }
        for (uint32_t i = 0; i < ch_num; i++) {
            pins[i] = tkl_adc_test_arg_u32(argc, argv, (int)(3U + i), pins[i]);
        }
    }

    ret = tkl_adc_test_init_channels(pins, ch_num, cnt, TUYA_ADC_CONTINUOUS, &cfg);
    if (ret != OPRT_OK) {
        tkl_adc_test_print_ret("multi init", ret);
        return ret;
    }

    ret = tkl_adc_read_data(TKL_ADC_TEST_DEFAULT_PORT, raw,
                            (UINT16_T)(ch_num * cnt));
    if (ret != OPRT_OK) {
        tkl_adc_test_print_ret("multi raw", ret);
        tkl_adc_deinit(TKL_ADC_TEST_DEFAULT_PORT);
        return ret;
    }

    ret = tkl_adc_read_voltage(TKL_ADC_TEST_DEFAULT_PORT, volt,
                               (UINT16_T)(ch_num * cnt));
    if (ret != OPRT_OK) {
        tkl_adc_test_print_ret("multi volt", ret);
        tkl_adc_deinit(TKL_ADC_TEST_DEFAULT_PORT);
        return ret;
    }

    for (uint32_t i = 0; i < ch_num; i++) {
        INT32_T chan = tkl_adc_test_pin_to_channel(pins[i]);
        uint32_t offset = i * cnt;
        tkl_adc_test_print_samples("multi raw", pins[i], chan, &raw[offset], cnt, "raw");
        tkl_adc_test_print_samples("multi volt", pins[i], chan, &volt[offset], cnt, "mv");
    }

    tkl_adc_deinit(TKL_ADC_TEST_DEFAULT_PORT);

    return OPRT_OK;
}

static void tkl_adc_test_print_pin_map(uint32_t pin)
{
    INT32_T chan = tkl_adc_test_pin_to_channel(pin);

    if (chan < 0) {
        bk_printf("pin=%u adc=unsupported\r\n", pin);
    } else {
        bk_printf("pin=%u adc_ch=%d\r\n", pin, chan);
    }
}

static void tkl_adc_test_print_all_maps(void)
{
    static const uint32_t adc_pins[] = {25, 24, 23, 28, 22, 21, 8, 0, 1, 12, 13};

    for (uint32_t i = 0; i < sizeof(adc_pins) / sizeof(adc_pins[0]); i++) {
        tkl_adc_test_print_pin_map(adc_pins[i]);
    }
}

static OPERATE_RET tkl_adc_test_div(uint32_t pin, const char *state)
{
    INT32_T chan = tkl_adc_test_pin_to_channel(pin);
    ADC_IOCTL_DIV_RESIS_T div_cfg = {0};
    OPERATE_RET ret;

    if ((chan < 0) || (state == NULL)) {
        return OPRT_INVALID_PARM;
    }

    /* tkl_adc_ioctl() expects an ADC_IOCTL_DIV_RESIS_T (port/channel/data), per
     * tkl_adc.h. Passing a bare channel byte makes the adapter read garbage for
     * port and reject with OPRT_INVALID_PARM. */
    div_cfg.port = TKL_ADC_TEST_DEFAULT_PORT;
    div_cfg.channel = (UINT8_T)chan;

    if (os_strcmp(state, "close") == 0) {
        div_cfg.data = 0;
        ret = tkl_adc_ioctl(ADC_DIV_RESIS_CLOSE, &div_cfg);
    } else if (os_strcmp(state, "open") == 0) {
        div_cfg.data = 1;
        ret = tkl_adc_ioctl(ADC_DIV_RESIS_OPEN, &div_cfg);
    } else {
        return OPRT_INVALID_PARM;
    }

    bk_printf("div pin=%u ch=%u state=%s ret=%d\r\n", pin, div_cfg.channel, state, ret);
    return ret;
}

static OPERATE_RET tkl_adc_test_errtest(uint32_t pin)
{
    TUYA_ADC_BASE_CFG_T cfg;
    INT32_T sample = 0;
    INT32_T chan = tkl_adc_test_pin_to_channel(pin);
    UINT8_T unconfigured_chan = (chan == 15) ? 14U : 15U;
    OPERATE_RET ret;
    OPERATE_RET final_ret = OPRT_OK;

    bk_printf("errtest begin pin=%u ch=%d\r\n", pin, chan);

    ret = tkl_adc_init(TUYA_ADC_NUM_MAX, NULL);
    bk_printf("errtest init invalid ret=%d expect=%d\r\n", ret, OPRT_INVALID_PARM);
    if (ret != OPRT_INVALID_PARM) {
        final_ret = OPRT_COM_ERROR;
    }

    ret = tkl_adc_read_data(TKL_ADC_TEST_DEFAULT_PORT, &sample, 1);
    bk_printf("errtest read before init ret=%d expect=%d\r\n", ret, OPRT_INVALID_PARM);
    if (ret != OPRT_INVALID_PARM) {
        final_ret = OPRT_COM_ERROR;
    }

    ret = tkl_adc_test_init_channels(&pin, 1U, 2U, TUYA_ADC_CONTINUOUS, &cfg);
    if (ret != OPRT_OK) {
        bk_printf("errtest valid init failed ret=%d\r\n", ret);
        return ret;
    }

    ret = tkl_adc_read_data(TKL_ADC_TEST_DEFAULT_PORT, &sample, 1);
    bk_printf("errtest short raw buffer ret=%d expect=%d\r\n", ret, OPRT_INVALID_PARM);
    if (ret != OPRT_INVALID_PARM) {
        final_ret = OPRT_COM_ERROR;
    }

    ret = tkl_adc_read_voltage(TKL_ADC_TEST_DEFAULT_PORT, NULL, 1);
    bk_printf("errtest null voltage buffer ret=%d expect=%d\r\n", ret, OPRT_INVALID_PARM);
    if (ret != OPRT_INVALID_PARM) {
        final_ret = OPRT_COM_ERROR;
    }

    ret = tkl_adc_read_single_channel(TKL_ADC_TEST_DEFAULT_PORT, unconfigured_chan, &sample);
    bk_printf("errtest unconfigured single ch=%u ret=%d expect=%d\r\n",
              unconfigured_chan, ret, OPRT_INVALID_PARM);
    if (ret != OPRT_INVALID_PARM) {
        final_ret = OPRT_COM_ERROR;
    }

    tkl_adc_deinit(TKL_ADC_TEST_DEFAULT_PORT);

    tkl_adc_test_print_ret("errtest", final_ret);
    return final_ret;
}

static OPERATE_RET tkl_adc_test_selftest(uint32_t pin)
{
    OPERATE_RET ret;

    bk_printf("selftest begin pin=%u\r\n", pin);
    bk_printf("width=%u ref=%u mv temp=%d\r\n",
              tkl_adc_width_get(TKL_ADC_TEST_DEFAULT_PORT),
              tkl_adc_ref_voltage_get(TKL_ADC_TEST_DEFAULT_PORT),
              tkl_adc_temperature_get());
    tkl_adc_test_print_pin_map(pin);

    ret = tkl_adc_test_read_one(pin, 1U, TKL_ADC_TEST_READ_RAW, false);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_adc_test_read_one(pin, 1U, TKL_ADC_TEST_READ_RAW, true);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_adc_test_read_one(pin, 1U, TKL_ADC_TEST_READ_VOLTAGE, false);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_adc_test_errtest(pin);
    tkl_adc_test_print_ret("selftest", ret);

    return ret;
}

static void tkl_adc_test_loop_task(void *arg)
{
    (void)arg;

    while (!s_tkl_adc_loop.stop) {
        (void)tkl_adc_test_read_one(s_tkl_adc_loop.pin,
                                    s_tkl_adc_loop.conv_cnt,
                                    s_tkl_adc_loop.mode,
                                    false);
        tkl_system_sleep(s_tkl_adc_loop.period_ms);
    }

    s_tkl_adc_loop.handle = NULL;
    s_tkl_adc_loop.stop = false;
    vTaskDelete(NULL);
}

static OPERATE_RET tkl_adc_test_loop_start(uint32_t pin, uint32_t period_ms,
                                           uint32_t conv_cnt,
                                           TKL_ADC_TEST_READ_MODE_E mode)
{
    BaseType_t ret;

    if (s_tkl_adc_loop.handle != NULL) {
        bk_printf("loop already running\r\n");
        return OPRT_COM_ERROR;
    }

    s_tkl_adc_loop.stop = false;
    s_tkl_adc_loop.pin = pin;
    s_tkl_adc_loop.period_ms = (period_ms == 0U) ? TKL_ADC_TEST_DEFAULT_PERIOD_MS : period_ms;
    s_tkl_adc_loop.conv_cnt = tkl_adc_test_sanitize_cnt(conv_cnt);
    s_tkl_adc_loop.mode = mode;

    ret = xTaskCreate(tkl_adc_test_loop_task, "tkl_adc", 4096, NULL, 5,
                      &s_tkl_adc_loop.handle);
    if (ret != pdPASS) {
        s_tkl_adc_loop.handle = NULL;
        return OPRT_COM_ERROR;
    }

    bk_printf("loop_start pin=%u period=%u cnt=%u mode=%s\r\n",
              s_tkl_adc_loop.pin, s_tkl_adc_loop.period_ms,
              s_tkl_adc_loop.conv_cnt,
              (s_tkl_adc_loop.mode == TKL_ADC_TEST_READ_VOLTAGE) ? "volt" : "raw");

    return OPRT_OK;
}

static OPERATE_RET tkl_adc_test_loop_stop(void)
{
    if (s_tkl_adc_loop.handle == NULL) {
        bk_printf("loop not running\r\n");
        return OPRT_OK;
    }

    s_tkl_adc_loop.stop = true;
    bk_printf("loop_stop requested\r\n");

    return OPRT_OK;
}

void cli_tkl_adc_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    const char *cmd;
    uint32_t pin;
    uint32_t cnt;
    OPERATE_RET ret = OPRT_OK;

    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    if ((argc < 2) || (argv == NULL) || (argv[1] == NULL)) {
        tkl_adc_test_usage();
        return;
    }

    cmd = argv[1];

    if ((os_strcmp(cmd, "help") == 0) || (os_strcmp(cmd, "?") == 0)) {
        tkl_adc_test_usage();
    } else if (os_strcmp(cmd, "map") == 0) {
        if (argc >= 3) {
            pin = tkl_adc_test_arg_u32(argc, argv, 2, TKL_ADC_TEST_DEFAULT_PIN);
            tkl_adc_test_print_pin_map(pin);
        } else {
            tkl_adc_test_print_all_maps();
        }
    } else if (os_strcmp(cmd, "info") == 0) {
        bk_printf("width=%u ref=%u mv temp=%d\r\n",
                  tkl_adc_width_get(TKL_ADC_TEST_DEFAULT_PORT),
                  tkl_adc_ref_voltage_get(TKL_ADC_TEST_DEFAULT_PORT),
                  tkl_adc_temperature_get());
    } else if (os_strcmp(cmd, "raw") == 0) {
        pin = tkl_adc_test_arg_u32(argc, argv, 2, TKL_ADC_TEST_DEFAULT_PIN);
        cnt = tkl_adc_test_arg_u32(argc, argv, 3, TKL_ADC_TEST_DEFAULT_CONV_CNT);
        ret = tkl_adc_test_read_one(pin, cnt, TKL_ADC_TEST_READ_RAW, false);
    } else if (os_strcmp(cmd, "single") == 0) {
        pin = tkl_adc_test_arg_u32(argc, argv, 2, TKL_ADC_TEST_DEFAULT_PIN);
        cnt = tkl_adc_test_arg_u32(argc, argv, 3, 1U);
        ret = tkl_adc_test_read_one(pin, cnt, TKL_ADC_TEST_READ_RAW, true);
    } else if (os_strcmp(cmd, "volt") == 0) {
        pin = tkl_adc_test_arg_u32(argc, argv, 2, TKL_ADC_TEST_DEFAULT_PIN);
        cnt = tkl_adc_test_arg_u32(argc, argv, 3, TKL_ADC_TEST_DEFAULT_CONV_CNT);
        ret = tkl_adc_test_read_one(pin, cnt, TKL_ADC_TEST_READ_VOLTAGE, false);
    } else if (os_strcmp(cmd, "multi") == 0) {
        ret = tkl_adc_test_multi(argc, argv);
    } else if (os_strcmp(cmd, "div") == 0) {
        pin = tkl_adc_test_arg_u32(argc, argv, 2, TKL_ADC_TEST_DEFAULT_PIN);
        ret = tkl_adc_test_div(pin, (argc >= 4) ? argv[3] : NULL);
    } else if (os_strcmp(cmd, "loop_start") == 0) {
        TKL_ADC_TEST_READ_MODE_E mode = TKL_ADC_TEST_READ_VOLTAGE;
        pin = tkl_adc_test_arg_u32(argc, argv, 2, TKL_ADC_TEST_DEFAULT_PIN);
        uint32_t period_ms = tkl_adc_test_arg_u32(argc, argv, 3,
                                                  TKL_ADC_TEST_DEFAULT_PERIOD_MS);
        cnt = tkl_adc_test_arg_u32(argc, argv, 4, TKL_ADC_TEST_DEFAULT_CONV_CNT);
        if ((argc >= 6) && (argv[5] != NULL) && (os_strcmp(argv[5], "raw") == 0)) {
            mode = TKL_ADC_TEST_READ_RAW;
        }
        ret = tkl_adc_test_loop_start(pin, period_ms, cnt, mode);
    } else if (os_strcmp(cmd, "loop_stop") == 0) {
        ret = tkl_adc_test_loop_stop();
    } else if (os_strcmp(cmd, "errtest") == 0) {
        pin = tkl_adc_test_arg_u32(argc, argv, 2, TKL_ADC_TEST_DEFAULT_PIN);
        ret = tkl_adc_test_errtest(pin);
    } else if (os_strcmp(cmd, "selftest") == 0) {
        pin = tkl_adc_test_arg_u32(argc, argv, 2, TKL_ADC_TEST_DEFAULT_PIN);
        ret = tkl_adc_test_selftest(pin);
    } else {
        bk_printf("unknown cmd:%s\r\n", cmd);
        tkl_adc_test_usage();
        return;
    }

    if (ret != OPRT_OK) {
        bk_printf("xtkladc %s failed:%d\r\n", cmd, ret);
    }
}
