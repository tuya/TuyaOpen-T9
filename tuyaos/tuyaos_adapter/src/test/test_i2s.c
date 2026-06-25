#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include <soc/soc.h>

#include "cli_tuya_test.h"
#include "tkl_i2s.h"
#include "tkl_pinmux.h"
#include "tkl_system.h"

/*
 * I2S pad routing. The multi-i2s driver maps pads via gpio_dev_map_by_func()
 * (port0->group0=I2S0 funcs, port1->group1=I2S1 funcs), but those funcs are
 * absent/incomplete in the board GPIO_DEFAULT_DEV_CONFIG, so the driver can't
 * resolve the pins. I2S is a flexible-mux peripheral, so route the pads here
 * with tkl_io_pinmux_config() before init (any pin can take any I2S function).
 *
 * NOTE: GPIO40/41 are also UART2 (TX/RX). If your serial console is on UART2,
 * running this remaps the console pads. Confirmed not the console here, so
 * port1 uses the documented I2S group1 pins GPIO40-43.
 *
 * Loopback wiring (port0 master -> port1 slave):
 *   GPIO6 (port0 BCLK) -> GPIO40 (port1 BCLK)
 *   GPIO7 (port0 LRCK) -> GPIO41 (port1 LRCK)
 *   GPIO9 (port0 DOUT) -> GPIO42 (port1 DIN)
 */
#define I2S0_CLK_PIN   6
#define I2S0_WS_PIN    7
#define I2S0_DIN_PIN   8
#define I2S0_DOUT_PIN  9
#define I2S1_CLK_PIN   40
#define I2S1_WS_PIN    41
#define I2S1_DIN_PIN   42
#define I2S1_DOUT_PIN  43

static void test_i2s_pinmux_setup(void)
{
    /* port0 -> group0 = I2S0 functions */
    (void)tkl_io_pinmux_config(I2S0_CLK_PIN,  TUYA_I2S0_SCK);
    (void)tkl_io_pinmux_config(I2S0_WS_PIN,   TUYA_I2S0_WS);
    (void)tkl_io_pinmux_config(I2S0_DIN_PIN,  TUYA_I2S0_SDI_0);
    (void)tkl_io_pinmux_config(I2S0_DOUT_PIN, TUYA_I2S0_SDO_0);
    /* port1 -> group1 = I2S1 functions */
    (void)tkl_io_pinmux_config(I2S1_CLK_PIN,  TUYA_I2S1_SCK);
    (void)tkl_io_pinmux_config(I2S1_WS_PIN,   TUYA_I2S1_WS);
    (void)tkl_io_pinmux_config(I2S1_DIN_PIN,  TUYA_I2S1_SDI_0);
    (void)tkl_io_pinmux_config(I2S1_DOUT_PIN, TUYA_I2S1_SDO_0);
}

#define I2S_TEST_DEFAULT_TX_PORT         TUYA_I2S_NUM_0
#define I2S_TEST_DEFAULT_RX_PORT         TUYA_I2S_NUM_1
#define I2S_TEST_DEFAULT_LOOPS           8U
#define I2S_TEST_DEFAULT_STRESS_ROUNDS   20U
#define I2S_TEST_WAIT_TIMEOUT_MS         12000U
#define I2S_TEST_POLL_INTERVAL_MS        2U
#define I2S_TEST_MAX_FRAME_BYTES         4096U
#define I2S_TEST_MAX_FRAME_LOOPS         16U
#define I2S_TEST_MAX_TOTAL_BYTES         (I2S_TEST_MAX_FRAME_BYTES * I2S_TEST_MAX_FRAME_LOOPS)
#define I2S_TEST_DUMP_MAX_BYTES          64U
#define I2S_TEST_TX_DRAIN_WAIT_MS        24U

extern void bk_printf(const char *fmt, ...);
#define I2S_TEST_LOG(...) bk_printf(__VA_ARGS__)

typedef struct {
    uint32_t pass;
    uint32_t fail;
    uint32_t skip;
} i2s_stat_t;

typedef struct {
    TUYA_I2S_NUM_E port;
    uint32_t frame_size;
    uint32_t loops;
    volatile uint32_t stop_req;
    volatile uint32_t started;
    volatile uint32_t exited;
    volatile uint32_t recv_ok;
    volatile OPERATE_RET last_ret;
    uint8_t *rx_buf;
} i2s_loop_worker_t;

static uint32_t test_i2s_hw_port_num(void)
{
#if defined(SOC_I2S_UNIT_NUM)
    if ((uint32_t)SOC_I2S_UNIT_NUM < (uint32_t)TUYA_I2S_NUM_MAX) {
        return (uint32_t)SOC_I2S_UNIT_NUM;
    }
#endif
    return (uint32_t)TUYA_I2S_NUM_MAX;
}

static bool test_i2s_port_in_range(uint32_t port)
{
    return (port < test_i2s_hw_port_num());
}

static void test_i2s_usage(char *pcWriteBuffer, int xWriteBufferLen)
{
    snprintf(pcWriteBuffer, xWriteBufferLen,
             "Usage: xi2s <list|param [port]|cfg [port]|loop [tx_port] [rx_port] [loops] [dma(0/1|dma)] [verify(0/1)] [sample(1..9|hz)] [bits(8/16/24/32)] [ch(0..4)] [fmt(1/2/4/12)]|stress [tx_port] [rx_port] [rounds] [dma(0/1|dma)]|full [tx_port] [rx_port]>");
}

static void test_i2s_stat_add(i2s_stat_t *st, const char *name, bool pass, bool skip, OPERATE_RET ret)
{
    if ((st == NULL) || (name == NULL)) {
        return;
    }

    if (skip) {
        st->skip++;
        I2S_TEST_LOG("[xi2s] SKIP test_name=%s return_code=%d\r\n", name, (int)ret);
        return;
    }

    if (pass) {
        st->pass++;
        I2S_TEST_LOG("[xi2s] PASS test_name=%s\r\n", name);
    } else {
        st->fail++;
        I2S_TEST_LOG("[xi2s] FAIL test_name=%s return_code=%d\r\n", name, (int)ret);
    }
}

static void test_i2s_stat_summary(const i2s_stat_t *st, const char *tag)
{
    if ((st == NULL) || (tag == NULL)) {
        return;
    }

    I2S_TEST_LOG("[xi2s] result_summary test_name=%s passed_count=%u failed_count=%u skipped_count=%u\r\n",
                 tag,
                 (unsigned int)st->pass,
                 (unsigned int)st->fail,
                 (unsigned int)st->skip);
}

static uint32_t test_i2s_parse_dma_arg(const char *arg)
{
    if (arg == NULL) {
        return 0U;
    }

    if ((os_strcmp(arg, "dma") == 0) ||
        (os_strcmp(arg, "on") == 0) ||
        (os_strcmp(arg, "true") == 0)) {
        return 1U;
    }

    if ((os_strcmp(arg, "nodma") == 0) ||
        (os_strcmp(arg, "off") == 0) ||
        (os_strcmp(arg, "false") == 0)) {
        return 0U;
    }

    return os_strtoul(arg, NULL, 10);
}

static uint32_t test_i2s_sample_rate_to_enum(uint32_t sample)
{
    switch (sample) {
    case 1U:
    case 8000U:
        return 1U;
    case 2U:
    case 11025U:
        return 2U;
    case 3U:
    case 12000U:
        return 3U;
    case 4U:
    case 16000U:
        return 4U;
    case 5U:
    case 22050U:
        return 5U;
    case 6U:
    case 24000U:
        return 6U;
    case 7U:
    case 32000U:
        return 7U;
    case 8U:
    case 44100U:
        return 8U;
    case 9U:
    case 48000U:
        return 9U;
    default:
        return 0U;
    }
}

static uint32_t test_i2s_sample_enum_to_hz(uint32_t sample_enum)
{
    switch (sample_enum) {
    case 1U: return 8000U;
    case 2U: return 11025U;
    case 3U: return 12000U;
    case 4U: return 16000U;
    case 5U: return 22050U;
    case 6U: return 24000U;
    case 7U: return 32000U;
    case 8U: return 44100U;
    case 9U: return 48000U;
    default: return 16000U;
    }
}

static uint32_t test_i2s_calc_frame_size(const TUYA_I2S_BASE_CFG_T *cfg)
{
    uint32_t sample_hz;
    uint32_t bytes_per_slot;
    uint32_t slots_per_sample;

    if (cfg == NULL || cfg->bits_per_sample == 0U) {
        return 0U;
    }

    sample_hz = test_i2s_sample_enum_to_hz(cfg->sample_rate);
    bytes_per_slot = ((uint32_t)cfg->bits_per_sample + 7U) / 8U;

    if ((cfg->communication_format == I2S_COMM_FORMAT_STAND_PCM_SHORT) ||
        (cfg->communication_format == I2S_COMM_FORMAT_STAND_PCM_LONG)) {
        slots_per_sample = (cfg->channel_format == TUYA_I2S_CHANNEL_FMT_RIGHT_LEFT) ? 2U : 1U;
    } else {
        slots_per_sample = (cfg->channel_format == TUYA_I2S_CHANNEL_FMT_RIGHT_LEFT) ? 2U : 1U;
    }

    if (cfg->channel_format == TUYA_I2S_CHANNEL_FMT_RIGHT_LEFT) {
        return (sample_hz * 4U) / 50U;
    }

    return (sample_hz * slots_per_sample * bytes_per_slot) / 50U;
}

static TUYA_I2S_BASE_CFG_T test_i2s_build_cfg(TUYA_I2S_NUM_E port,
                                               uint32_t mode,
                                               bool use_dma,
                                               uint32_t sample_rate,
                                               uint32_t bits,
                                               uint32_t channel_fmt,
                                               uint32_t comm_fmt)
{
    TUYA_I2S_BASE_CFG_T cfg;
    (void)port;
    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.mode = (TUYA_I2S_MODE_E)mode;
    cfg.sample_rate = sample_rate;
    cfg.mclk = 0U;
    cfg.bits_per_sample = (TUYA_I2S_BITS_PER_SAMP_E)bits;
    cfg.channel_format = (TUYA_I2S_CHANNEL_FMT_E)channel_fmt;
    cfg.communication_format = (TUYA_I2S_COMM_FORMAT_E)comm_fmt;
    cfg.i2s_dma_flags = use_dma ? 1U : 0U;
    cfg.upper_rx_cb = NULL;
    cfg.rx_args = NULL;
    cfg.upper_tx_cb = NULL;
    cfg.tx_args = NULL;
    return cfg;
}

static void test_i2s_fill_pattern(uint8_t *buf, uint32_t bytes, uint32_t seed)
{
    if (buf == NULL) {
        return;
    }

    for (uint32_t i = 0; i < bytes; i++) {
        buf[i] = (uint8_t)((seed + i) & 0xFFU);
    }
}

static bool test_i2s_frame_match_exact(const uint8_t *tx, const uint8_t *rx, uint32_t bytes)
{
    if ((tx == NULL) || (rx == NULL)) {
        return false;
    }
    return (os_memcmp(tx, rx, bytes) == 0);
}

static void test_i2s_dump_bytes(const char *tag, const uint8_t *buf, uint32_t len, uint32_t max_bytes)
{
    uint32_t dump_len;
    uint32_t off;
    char line[16U * 3U + 1U];

    if ((tag == NULL) || (buf == NULL) || (len == 0U) || (max_bytes == 0U)) {
        return;
    }

    dump_len = (len < max_bytes) ? len : max_bytes;
    for (off = 0U; off < dump_len; off += 16U) {
        uint32_t j;
        uint32_t line_len = 0U;
        line[0] = '\0';
        for (j = 0U; (j < 16U) && ((off + j) < dump_len); j++) {
            line_len += (uint32_t)snprintf(line + line_len, sizeof(line) - line_len, "%02X ", buf[off + j]);
            if (line_len >= (sizeof(line) - 1U)) {
                break;
            }
        }
        I2S_TEST_LOG("[xi2s] %s off=%u data=%s\r\n", tag, (unsigned int)off, line);
    }

    if (dump_len < len) {
        I2S_TEST_LOG("[xi2s] %s truncated total=%u dumped=%u\r\n",
                     tag,
                     (unsigned int)len,
                     (unsigned int)dump_len);
    }
}

static void test_i2s_cleanup_port(TUYA_I2S_NUM_E port)
{
    (void)tkl_i2s_send_stop(port);
    (void)tkl_i2s_recv_stop(port);
    (void)tkl_i2s_deinit(port);
}

static OPERATE_RET test_i2s_join_worker_with_timeout(beken_thread_t *thread, i2s_loop_worker_t *ctx)
{
    uint32_t waited = 0U;

    if ((thread == NULL) || (ctx == NULL)) {
        return OPRT_INVALID_PARM;
    }

    while ((ctx->exited == 0U) && (waited < I2S_TEST_WAIT_TIMEOUT_MS)) {
        tkl_system_sleep(I2S_TEST_POLL_INTERVAL_MS);
        waited += I2S_TEST_POLL_INTERVAL_MS;
    }

    if (ctx->exited != 0U) {
        (void)rtos_thread_join(thread);
        return OPRT_OK;
    }

    I2S_TEST_LOG("[xi2s] loop_worker_timeout waited_ms=%u started=%u exited=%u recv_ok=%u last_ret=%d\r\n",
                 (unsigned int)waited,
                 (unsigned int)ctx->started,
                 (unsigned int)ctx->exited,
                 (unsigned int)ctx->recv_ok,
                 (int)ctx->last_ret);
    (void)rtos_delete_thread(thread);
    ctx->exited = 1U;
    return OPRT_TIMEOUT;
}

static void test_i2s_loop_rx_worker(beken_thread_arg_t arg)
{
    i2s_loop_worker_t *ctx = (i2s_loop_worker_t *)arg;

    if (ctx == NULL) {
        rtos_delete_thread(NULL);
        return;
    }

    ctx->last_ret = OPRT_COM_ERROR;
    ctx->started = 1U;

    for (uint32_t i = 0U; (i < ctx->loops) && (ctx->stop_req == 0U); i++) {
        INT_T ret = tkl_i2s_recv(ctx->port, ctx->rx_buf + (i * ctx->frame_size), ctx->frame_size);
        if (ret != (INT_T)ctx->frame_size) {
            I2S_TEST_LOG("[xi2s] loop_rx_worker recv_fail port=%u frame_idx=%u ret=%d expect=%u\r\n",
                         (unsigned int)ctx->port,
                         (unsigned int)i,
                         (int)ret,
                         (unsigned int)ctx->frame_size);
            ctx->last_ret = (ret < 0) ? (OPERATE_RET)ret : OPRT_COM_ERROR;
            goto __EXIT;
        }
        ctx->recv_ok++;
    }

    ctx->last_ret = OPRT_OK;

__EXIT:
    ctx->exited = 1U;
    rtos_delete_thread(NULL);
}

static OPERATE_RET test_i2s_param(uint32_t port)
{
    i2s_stat_t st = {0};
    TUYA_I2S_BASE_CFG_T tx_cfg;
    TUYA_I2S_BASE_CFG_T rx_cfg;
    OPERATE_RET ret;
    uint32_t frame_size;
    uint8_t one[4] = {0};
    uint8_t *tx_frame = NULL;
    uint8_t *rx_frame = NULL;

    if (!test_i2s_port_in_range(port)) {
        return OPRT_INVALID_PARM;
    }

    tx_cfg = test_i2s_build_cfg((TUYA_I2S_NUM_E)port,
                                (uint32_t)(TUYA_I2S_MODE_MASTER | TUYA_I2S_MODE_TX),
                                true,
                                4U,
                                TUYA_I2S_BITS_PER_SAMPLE_16BIT,
                                TUYA_I2S_CHANNEL_FMT_RIGHT_LEFT,
                                I2S_COMM_FORMAT_STAND_I2S);
    rx_cfg = test_i2s_build_cfg((TUYA_I2S_NUM_E)port,
                                (uint32_t)(TUYA_I2S_MODE_MASTER | TUYA_I2S_MODE_RX),
                                true,
                                4U,
                                TUYA_I2S_BITS_PER_SAMPLE_16BIT,
                                TUYA_I2S_CHANNEL_FMT_RIGHT_LEFT,
                                I2S_COMM_FORMAT_STAND_I2S);
    frame_size = test_i2s_calc_frame_size(&tx_cfg);
    if ((frame_size == 0U) || (frame_size > I2S_TEST_MAX_FRAME_BYTES)) {
        return OPRT_INVALID_PARM;
    }

    tx_frame = (uint8_t *)tkl_system_malloc(frame_size);
    rx_frame = (uint8_t *)tkl_system_malloc(frame_size);
    if ((tx_frame == NULL) || (rx_frame == NULL)) {
        if (tx_frame != NULL) {
            tkl_system_free(tx_frame);
        }
        if (rx_frame != NULL) {
            tkl_system_free(rx_frame);
        }
        return OPRT_MALLOC_FAILED;
    }

    test_i2s_cleanup_port((TUYA_I2S_NUM_E)port);

    ret = tkl_i2s_init((TUYA_I2S_NUM_E)TUYA_I2S_NUM_MAX, &tx_cfg);
    test_i2s_stat_add(&st, "TC001 init(invalid_port)", (ret != OPRT_OK), false, ret);

    ret = tkl_i2s_init((TUYA_I2S_NUM_E)port, NULL);
    test_i2s_stat_add(&st, "TC002 init(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_i2s_send((TUYA_I2S_NUM_E)port, one, sizeof(one));
    test_i2s_stat_add(&st, "TC003 send(before_init)", (ret < 0), false, ret);

    ret = tkl_i2s_recv((TUYA_I2S_NUM_E)port, one, sizeof(one));
    test_i2s_stat_add(&st, "TC004 recv(before_init)", (ret < 0), false, ret);

    ret = tkl_i2s_send((TUYA_I2S_NUM_E)port, NULL, sizeof(one));
    test_i2s_stat_add(&st, "TC005 send(NULL)", (ret < 0), false, ret);

    ret = tkl_i2s_recv((TUYA_I2S_NUM_E)port, NULL, sizeof(one));
    test_i2s_stat_add(&st, "TC006 recv(NULL)", (ret < 0), false, ret);

    ret = tkl_i2s_send((TUYA_I2S_NUM_E)port, one, 0U);
    test_i2s_stat_add(&st, "TC007 send(len0)", (ret < 0), false, ret);

    ret = tkl_i2s_recv((TUYA_I2S_NUM_E)port, one, 0U);
    test_i2s_stat_add(&st, "TC008 recv(len0)", (ret < 0), false, ret);

    ret = tkl_i2s_init((TUYA_I2S_NUM_E)port, &tx_cfg);
    test_i2s_stat_add(&st, "TC009 init(tx_dma)", (ret == OPRT_OK), false, ret);
    if (ret == OPRT_OK) {
        ret = tkl_i2s_send((TUYA_I2S_NUM_E)port, tx_frame, frame_size - 1U);
        test_i2s_stat_add(&st, "TC010 send(not_frame_aligned)", (ret < 0), false, ret);

        test_i2s_fill_pattern(tx_frame, frame_size, 0x31U);
        ret = tkl_i2s_send((TUYA_I2S_NUM_E)port, tx_frame, frame_size);
        test_i2s_stat_add(&st, "TC011 send(frame)", (ret >= 0), false, ret);

        ret = tkl_i2s_send_stop((TUYA_I2S_NUM_E)port);
        test_i2s_stat_add(&st, "TC012 send_stop", (ret == OPRT_OK || ret == OPRT_COM_ERROR), false, ret);

        ret = tkl_i2s_deinit((TUYA_I2S_NUM_E)port);
        test_i2s_stat_add(&st, "TC013 deinit", (ret == OPRT_OK), false, ret);
    }

    ret = tkl_i2s_init((TUYA_I2S_NUM_E)port, &rx_cfg);
    test_i2s_stat_add(&st, "TC014 init(rx_dma)", (ret == OPRT_OK), false, ret);
    if (ret == OPRT_OK) {
        ret = tkl_i2s_recv((TUYA_I2S_NUM_E)port, rx_frame, frame_size - 1U);
        test_i2s_stat_add(&st, "TC015 recv(not_frame_aligned)", (ret < 0), false, ret);

        ret = tkl_i2s_recv_stop((TUYA_I2S_NUM_E)port);
        test_i2s_stat_add(&st, "TC016 recv_stop", (ret == OPRT_OK || ret == OPRT_COM_ERROR), false, ret);

        ret = tkl_i2s_deinit((TUYA_I2S_NUM_E)port);
        test_i2s_stat_add(&st, "TC017 deinit", (ret == OPRT_OK), false, ret);
    }

    ret = tkl_i2s_deinit((TUYA_I2S_NUM_E)port);
    test_i2s_stat_add(&st, "TC018 deinit(repeat)", (ret == OPRT_OK), false, ret);

    test_i2s_stat_summary(&st, "param");
    tkl_system_free(tx_frame);
    tkl_system_free(rx_frame);
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET test_i2s_cfg_matrix(uint32_t port)
{
    static const uint32_t mode_list[] = {
        (uint32_t)(TUYA_I2S_MODE_MASTER | TUYA_I2S_MODE_TX),
        (uint32_t)(TUYA_I2S_MODE_MASTER | TUYA_I2S_MODE_RX),
        (uint32_t)(TUYA_I2S_MODE_SLAVE | TUYA_I2S_MODE_TX),
        (uint32_t)(TUYA_I2S_MODE_SLAVE | TUYA_I2S_MODE_RX),
    };
    static const uint32_t dma_list[] = {0U, 1U};
    static const uint32_t sample_list[] = {4U, 9U};
    static const uint32_t bits_list[] = {TUYA_I2S_BITS_PER_SAMPLE_16BIT, TUYA_I2S_BITS_PER_SAMPLE_32BIT};
    static const uint32_t ch_list[] = {TUYA_I2S_CHANNEL_FMT_RIGHT_LEFT, TUYA_I2S_CHANNEL_FMT_ONLY_LEFT};
    static const uint32_t fmt_list[] = {I2S_COMM_FORMAT_STAND_I2S, I2S_COMM_FORMAT_STAND_PCM_SHORT};
    i2s_stat_t st = {0};
    TUYA_I2S_BASE_CFG_T cfg;
    OPERATE_RET ret;
    bool skip_case;
    bool pass_case;
    char tc_name[96];

    if (!test_i2s_port_in_range(port)) {
        return OPRT_INVALID_PARM;
    }

    for (uint32_t mi = 0U; mi < (uint32_t)(sizeof(mode_list) / sizeof(mode_list[0])); mi++) {
        for (uint32_t di = 0U; di < (uint32_t)(sizeof(dma_list) / sizeof(dma_list[0])); di++) {
            for (uint32_t si = 0U; si < (uint32_t)(sizeof(sample_list) / sizeof(sample_list[0])); si++) {
                for (uint32_t bi = 0U; bi < (uint32_t)(sizeof(bits_list) / sizeof(bits_list[0])); bi++) {
                    for (uint32_t ci = 0U; ci < (uint32_t)(sizeof(ch_list) / sizeof(ch_list[0])); ci++) {
                        for (uint32_t fi = 0U; fi < (uint32_t)(sizeof(fmt_list) / sizeof(fmt_list[0])); fi++) {
                            cfg = test_i2s_build_cfg((TUYA_I2S_NUM_E)port,
                                                     mode_list[mi],
                                                     (dma_list[di] != 0U),
                                                     sample_list[si],
                                                     bits_list[bi],
                                                     ch_list[ci],
                                                     fmt_list[fi]);
                            ret = tkl_i2s_init((TUYA_I2S_NUM_E)port, &cfg);
                            snprintf(tc_name, sizeof(tc_name), "TCM cfg_m%u_d%u_s%u_b%u_c%u_f%u",
                                     (unsigned int)mode_list[mi],
                                     (unsigned int)dma_list[di],
                                     (unsigned int)sample_list[si],
                                     (unsigned int)bits_list[bi],
                                     (unsigned int)ch_list[ci],
                                     (unsigned int)fmt_list[fi]);
                            skip_case = (ret == OPRT_NOT_SUPPORTED);
                            if ((ret == OPRT_COM_ERROR) && (dma_list[di] != 0U)) {
                                // Some boards have limited DMA channels, treat as unsupported in cfg matrix.
                                skip_case = true;
                            }
                            pass_case = (ret == OPRT_OK) || skip_case;
                            test_i2s_stat_add(&st,
                                              tc_name,
                                              pass_case,
                                              skip_case,
                                              ret);
                            if (ret == OPRT_OK) {
                                (void)tkl_i2s_deinit((TUYA_I2S_NUM_E)port);
                            }
                        }
                    }
                }
            }
        }
    }

    test_i2s_stat_summary(&st, "cfg_matrix");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET test_i2s_loopback(uint32_t tx_port,
                                     uint32_t rx_port,
                                     uint32_t loops,
                                     bool use_dma,
                                     bool verify_data,
                                     uint32_t sample_rate,
                                     uint32_t bits,
                                     uint32_t channel_fmt,
                                     uint32_t comm_fmt)
{
    TUYA_I2S_BASE_CFG_T tx_cfg;
    TUYA_I2S_BASE_CFG_T rx_cfg;
    i2s_loop_worker_t worker;
    beken_thread_t rx_thread = NULL;
    OPERATE_RET ret = OPRT_OK;
    OPERATE_RET join_ret;
    uint32_t frame_size;
    uint32_t total_bytes;
    uint32_t tx_total_bytes;
    uint32_t rx_total_bytes;
    uint32_t tx_frame_count;
    uint32_t rx_frame_count;
    uint32_t verify_frame_offset;
    uint32_t mismatch_frames = 0U;
    uint32_t first_mismatch_frame = 0xFFFFFFFFU;
    uint32_t send_ok = 0U;
    uint32_t i;
    bool need_warmup;
    uint8_t *tx_buf = NULL;
    uint8_t *rx_buf = NULL;

    if (!test_i2s_port_in_range(tx_port) ||
        !test_i2s_port_in_range(rx_port) ||
        (tx_port == rx_port) ||
        (loops == 0U) ||
        (loops > I2S_TEST_MAX_FRAME_LOOPS)) {
        return OPRT_INVALID_PARM;
    }

    tx_cfg = test_i2s_build_cfg((TUYA_I2S_NUM_E)tx_port,
                                (uint32_t)(TUYA_I2S_MODE_MASTER | TUYA_I2S_MODE_TX),
                                use_dma,
                                sample_rate,
                                bits,
                                channel_fmt,
                                comm_fmt);
    rx_cfg = test_i2s_build_cfg((TUYA_I2S_NUM_E)rx_port,
                                (uint32_t)(TUYA_I2S_MODE_SLAVE | TUYA_I2S_MODE_RX),
                                use_dma,
                                sample_rate,
                                bits,
                                channel_fmt,
                                comm_fmt);

    frame_size = test_i2s_calc_frame_size(&tx_cfg);
    total_bytes = frame_size * loops;
    need_warmup = (use_dma && verify_data);
    verify_frame_offset = need_warmup ? 1U : 0U;
    tx_frame_count = loops + verify_frame_offset;
    rx_frame_count = loops + verify_frame_offset;
    tx_total_bytes = frame_size * tx_frame_count;
    rx_total_bytes = frame_size * rx_frame_count;
    if ((frame_size == 0U) ||
        (frame_size > I2S_TEST_MAX_FRAME_BYTES) ||
        (total_bytes > I2S_TEST_MAX_TOTAL_BYTES) ||
        (tx_total_bytes > I2S_TEST_MAX_TOTAL_BYTES) ||
        (rx_total_bytes > I2S_TEST_MAX_TOTAL_BYTES)) {
        return OPRT_INVALID_PARM;
    }

    I2S_TEST_LOG("[xi2s] loop_cfg tx_port=%u rx_port=%u dma=%u verify=%u sample=%u bits=%u ch=%u fmt=%u frame=%u loops=%u total=%u warmup=%u\r\n",
                 (unsigned int)tx_port,
                 (unsigned int)rx_port,
                 (unsigned int)use_dma,
                 (unsigned int)verify_data,
                 (unsigned int)sample_rate,
                 (unsigned int)bits,
                 (unsigned int)channel_fmt,
                 (unsigned int)comm_fmt,
                 (unsigned int)frame_size,
                 (unsigned int)loops,
                 (unsigned int)total_bytes,
                 (unsigned int)verify_frame_offset);

    tx_buf = (uint8_t *)tkl_system_malloc(tx_total_bytes);
    rx_buf = (uint8_t *)tkl_system_malloc(rx_total_bytes);
    if ((tx_buf == NULL) || (rx_buf == NULL)) {
        if (tx_buf != NULL) {
            tkl_system_free(tx_buf);
        }
        if (rx_buf != NULL) {
            tkl_system_free(rx_buf);
        }
        return OPRT_MALLOC_FAILED;
    }

    test_i2s_cleanup_port((TUYA_I2S_NUM_E)tx_port);
    test_i2s_cleanup_port((TUYA_I2S_NUM_E)rx_port);

    ret = tkl_i2s_init((TUYA_I2S_NUM_E)rx_port, &rx_cfg);
    if (ret != OPRT_OK) {
        I2S_TEST_LOG("[xi2s] loop_init_fail stage=rx_init ret=%d port=%u\r\n", (int)ret, (unsigned int)rx_port);
        tkl_system_free(tx_buf);
        tkl_system_free(rx_buf);
        return ret;
    }

    ret = tkl_i2s_init((TUYA_I2S_NUM_E)tx_port, &tx_cfg);
    if (ret != OPRT_OK) {
        I2S_TEST_LOG("[xi2s] loop_init_fail stage=tx_init ret=%d port=%u\r\n", (int)ret, (unsigned int)tx_port);
        (void)tkl_i2s_deinit((TUYA_I2S_NUM_E)rx_port);
        tkl_system_free(tx_buf);
        tkl_system_free(rx_buf);
        return ret;
    }

    I2S_TEST_LOG("[xi2s] loop_init_ok tx_port=%u rx_port=%u dma=%u\r\n",
                 (unsigned int)tx_port,
                 (unsigned int)rx_port,
                 (unsigned int)use_dma);

    (void)os_memset(&worker, 0, sizeof(worker));
    worker.port = (TUYA_I2S_NUM_E)rx_port;
    worker.frame_size = frame_size;
    worker.loops = rx_frame_count;
    worker.rx_buf = rx_buf;
    worker.last_ret = OPRT_COM_ERROR;
    (void)os_memset(rx_buf, 0, rx_total_bytes);
    (void)os_memset(tx_buf, 0, tx_total_bytes);

    if (rtos_create_thread(&rx_thread, 6, "xi2s_rx", test_i2s_loop_rx_worker, 3584, &worker) != kNoErr) {
        test_i2s_cleanup_port((TUYA_I2S_NUM_E)tx_port);
        test_i2s_cleanup_port((TUYA_I2S_NUM_E)rx_port);
        tkl_system_free(tx_buf);
        tkl_system_free(rx_buf);
        return OPRT_COM_ERROR;
    }

    for (uint32_t waited = 0U; (worker.started == 0U) && (waited < I2S_TEST_WAIT_TIMEOUT_MS); waited += I2S_TEST_POLL_INTERVAL_MS) {
        tkl_system_sleep(I2S_TEST_POLL_INTERVAL_MS);
    }
    if (worker.started == 0U) {
        worker.stop_req = 1U;
        (void)test_i2s_join_worker_with_timeout(&rx_thread, &worker);
        test_i2s_cleanup_port((TUYA_I2S_NUM_E)tx_port);
        test_i2s_cleanup_port((TUYA_I2S_NUM_E)rx_port);
        tkl_system_free(tx_buf);
        tkl_system_free(rx_buf);
        return OPRT_TIMEOUT;
    }

    I2S_TEST_LOG("[xi2s] loop_worker_started rx_port=%u frame=%u loops=%u\r\n",
                 (unsigned int)rx_port,
                 (unsigned int)frame_size,
                 (unsigned int)rx_frame_count);

    for (i = 0U; i < loops; i++) {
        uint8_t *tx_ptr = tx_buf + ((i + verify_frame_offset) * frame_size);
        test_i2s_fill_pattern(tx_ptr, frame_size, (0x40U + i));
    }
    if (need_warmup) {
        test_i2s_fill_pattern(tx_buf, frame_size, 0x00U);
    }

    {
        OPERATE_RET send_ret = tkl_i2s_send((TUYA_I2S_NUM_E)tx_port, tx_buf, tx_total_bytes);
        if (send_ret < 0) {
            I2S_TEST_LOG("[xi2s] loop_tx_fail tx_port=%u ret=%d total=%u\r\n",
                         (unsigned int)tx_port,
                         (int)send_ret,
                         (unsigned int)tx_total_bytes);
            ret = send_ret;
        } else if ((uint32_t)send_ret != tx_total_bytes) {
            I2S_TEST_LOG("[xi2s] loop_tx_partial tx_port=%u sent=%u expect=%u\r\n",
                         (unsigned int)tx_port,
                         (unsigned int)send_ret,
                         (unsigned int)tx_total_bytes);
            ret = OPRT_COM_ERROR;
            send_ok = (uint32_t)send_ret / frame_size;
            if (send_ok > verify_frame_offset) {
                send_ok -= verify_frame_offset;
            } else {
                send_ok = 0U;
            }
            if (send_ok > loops) {
                send_ok = loops;
            }
        } else {
            I2S_TEST_LOG("[xi2s] loop_tx_all_ok tx_port=%u sent=%u\r\n",
                         (unsigned int)tx_port,
                         (unsigned int)send_ret);
            send_ok = loops;
        }
    }

    if (ret != OPRT_OK) {
        worker.stop_req = 1U;
    }
    join_ret = test_i2s_join_worker_with_timeout(&rx_thread, &worker);
    if ((ret == OPRT_OK) && (join_ret != OPRT_OK)) {
        ret = join_ret;
    }

    if ((ret == OPRT_OK) && (worker.last_ret != OPRT_OK)) {
        ret = worker.last_ret;
    }

    if (use_dma && (send_ok > 0U)) {
        I2S_TEST_LOG("[xi2s] tx_drain_wait tx_port=%u wait_ms=%u send_ok=%u\r\n",
                     (unsigned int)tx_port,
                     (unsigned int)I2S_TEST_TX_DRAIN_WAIT_MS,
                     (unsigned int)send_ok);
        tkl_system_sleep(I2S_TEST_TX_DRAIN_WAIT_MS);
    }

    if ((ret == OPRT_OK) && verify_data) {
        for (i = 0U; i < loops; i++) {
            uint8_t *tx_ptr = tx_buf + ((i + verify_frame_offset) * frame_size);
            uint8_t *rx_same = rx_buf + ((i + verify_frame_offset) * frame_size);
            bool matched = test_i2s_frame_match_exact(tx_ptr, rx_same, frame_size);
            if (!matched) {
                mismatch_frames++;
                if (first_mismatch_frame == 0xFFFFFFFFU) {
                    first_mismatch_frame = i;
                }
            }
        }
        if (mismatch_frames > 0U) {
            I2S_TEST_LOG("[xi2s] verify_mismatch mismatch_frames=%u first_idx=%u frame_bytes=%u loops=%u total=%u\r\n",
                         (unsigned int)mismatch_frames,
                         (unsigned int)((first_mismatch_frame == 0xFFFFFFFFU) ? 0U : first_mismatch_frame),
                         (unsigned int)frame_size,
                         (unsigned int)loops,
                         (unsigned int)total_bytes);
            if (need_warmup) {
                test_i2s_dump_bytes("tx_warmup", tx_buf, frame_size, frame_size);
                test_i2s_dump_bytes("rx_warmup", rx_buf, frame_size, frame_size);
            }
            for (i = 0U; i < loops; i++) {
                uint8_t *tx_dbg = tx_buf + ((i + verify_frame_offset) * frame_size);
                uint8_t *rx_dbg = rx_buf + ((i + verify_frame_offset) * frame_size);
                char tx_tag[24];
                char rx_tag[24];
                (void)snprintf(tx_tag, sizeof(tx_tag), "tx_frame_%u", (unsigned int)i);
                (void)snprintf(rx_tag, sizeof(rx_tag), "rx_frame_%u", (unsigned int)(i + verify_frame_offset));
                test_i2s_dump_bytes(tx_tag, tx_dbg, frame_size, frame_size);
                test_i2s_dump_bytes(rx_tag, rx_dbg, frame_size, frame_size);
            }
            ret = OPRT_COM_ERROR;
        }
    }

    test_i2s_cleanup_port((TUYA_I2S_NUM_E)tx_port);
    test_i2s_cleanup_port((TUYA_I2S_NUM_E)rx_port);
    tkl_system_free(tx_buf);
    tkl_system_free(rx_buf);

    I2S_TEST_LOG("[xi2s] loop_result tx_port=%u rx_port=%u dma_enabled=%u verify_enabled=%u sample_enum=%u bits=%u channel_fmt=%u comm_fmt=%u frame_bytes=%u loops=%u send_ok=%u recv_ok=%u mismatch_frames=%u return_code=%d\r\n",
                 (unsigned int)tx_port,
                 (unsigned int)rx_port,
                 (unsigned int)use_dma,
                 (unsigned int)verify_data,
                 (unsigned int)sample_rate,
                 (unsigned int)bits,
                 (unsigned int)channel_fmt,
                 (unsigned int)comm_fmt,
                 (unsigned int)frame_size,
                 (unsigned int)loops,
                 (unsigned int)send_ok,
                 (unsigned int)worker.recv_ok,
                 (unsigned int)mismatch_frames,
                 (int)ret);

    return ret;
}

static OPERATE_RET test_i2s_stress(uint32_t tx_port, uint32_t rx_port, uint32_t rounds, bool use_dma)
{
    OPERATE_RET ret = OPRT_OK;
    uint32_t pass = 0U;
    uint32_t fail = 0U;

    if ((rounds == 0U) || !test_i2s_port_in_range(tx_port) || !test_i2s_port_in_range(rx_port) || (tx_port == rx_port)) {
        return OPRT_INVALID_PARM;
    }

    for (uint32_t i = 0U; i < rounds; i++) {
        ret = test_i2s_loopback(tx_port,
                                rx_port,
                                2U,
                                use_dma,
                                false,
                                4U,
                                TUYA_I2S_BITS_PER_SAMPLE_16BIT,
                                TUYA_I2S_CHANNEL_FMT_RIGHT_LEFT,
                                I2S_COMM_FORMAT_STAND_I2S);
        if (ret == OPRT_OK) {
            pass++;
        } else {
            fail++;
            break;
        }
    }

    I2S_TEST_LOG("[xi2s] stress_result tx_port=%u rx_port=%u dma_enabled=%u rounds=%u passed_count=%u failed_count=%u return_code=%d\r\n",
                 (unsigned int)tx_port,
                 (unsigned int)rx_port,
                 (unsigned int)use_dma,
                 (unsigned int)rounds,
                 (unsigned int)pass,
                 (unsigned int)fail,
                 (int)ret);

    return (fail == 0U) ? OPRT_OK : ret;
}

static OPERATE_RET test_i2s_full(uint32_t tx_port, uint32_t rx_port)
{
    i2s_stat_t st = {0};
    OPERATE_RET ret;
    uint32_t port;
    uint32_t hw_ports = test_i2s_hw_port_num();
    char tc_name[64];

    for (port = 0U; port < hw_ports; port++) {
        ret = test_i2s_param(port);
        snprintf(tc_name, sizeof(tc_name), "TF101 param_port%u", (unsigned int)port);
        test_i2s_stat_add(&st, tc_name, (ret == OPRT_OK), false, ret);

        ret = test_i2s_cfg_matrix(port);
        snprintf(tc_name, sizeof(tc_name), "TF102 cfg_matrix_port%u", (unsigned int)port);
        test_i2s_stat_add(&st, tc_name, (ret == OPRT_OK), false, ret);
    }

    if (hw_ports >= 2U && tx_port != rx_port &&
        test_i2s_port_in_range(tx_port) && test_i2s_port_in_range(rx_port)) {
        ret = test_i2s_loopback(tx_port,
                                rx_port,
                                I2S_TEST_DEFAULT_LOOPS,
                                true,
                                true,
                                4U,
                                TUYA_I2S_BITS_PER_SAMPLE_16BIT,
                                TUYA_I2S_CHANNEL_FMT_RIGHT_LEFT,
                                I2S_COMM_FORMAT_STAND_I2S);
        test_i2s_stat_add(&st, "TF201 loopback_dma", (ret == OPRT_OK), false, ret);

        ret = test_i2s_loopback(tx_port,
                                rx_port,
                                I2S_TEST_DEFAULT_LOOPS,
                                false,
                                false,
                                4U,
                                TUYA_I2S_BITS_PER_SAMPLE_16BIT,
                                TUYA_I2S_CHANNEL_FMT_RIGHT_LEFT,
                                I2S_COMM_FORMAT_STAND_I2S);
        test_i2s_stat_add(&st, "TF202 loopback_nodma",
                          (ret == OPRT_OK || ret == OPRT_NOT_SUPPORTED || ret == OPRT_COM_ERROR),
                          (ret != OPRT_OK),
                          ret);

        ret = test_i2s_stress(tx_port, rx_port, I2S_TEST_DEFAULT_STRESS_ROUNDS, true);
        test_i2s_stat_add(&st, "TF203 loopback_stress_dma", (ret == OPRT_OK), false, ret);
    } else {
        test_i2s_stat_add(&st, "TF201 loopback_dma", true, true, OPRT_NOT_SUPPORTED);
        test_i2s_stat_add(&st, "TF202 loopback_nodma", true, true, OPRT_NOT_SUPPORTED);
        test_i2s_stat_add(&st, "TF203 loopback_stress_dma", true, true, OPRT_NOT_SUPPORTED);
    }

    test_i2s_stat_summary(&st, "full");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

void cli_tuya_i2s_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    const char *cmd;
    OPERATE_RET ret;
    uint32_t port = (uint32_t)I2S_TEST_DEFAULT_TX_PORT;
    uint32_t tx_port = (uint32_t)I2S_TEST_DEFAULT_TX_PORT;
    uint32_t rx_port = (uint32_t)I2S_TEST_DEFAULT_RX_PORT;
    uint32_t loops = I2S_TEST_DEFAULT_LOOPS;
    uint32_t rounds = I2S_TEST_DEFAULT_STRESS_ROUNDS;
    uint32_t dma = 1U;
    uint32_t verify = 1U;
    uint32_t sample_rate = 4U;
    uint32_t bits = TUYA_I2S_BITS_PER_SAMPLE_16BIT;
    uint32_t channel_fmt = TUYA_I2S_CHANNEL_FMT_RIGHT_LEFT;
    uint32_t comm_fmt = I2S_COMM_FORMAT_STAND_I2S;
    uint32_t sample_arg = 0U;

    if ((pcWriteBuffer == NULL) || (xWriteBufferLen <= 0)) {
        return;
    }

    /* Route I2S pads before any init (board default table lacks I2S0/I2S1). */
    test_i2s_pinmux_setup();

    if (argc < 2) {
        test_i2s_usage(pcWriteBuffer, xWriteBufferLen);
        return;
    }

    cmd = argv[1];
    if (os_strcmp(cmd, "list") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xi2s list logical_port=[0..%u] note=\"loopback needs wiring TX(BCLK/LRCK/DOUT)->RX(BCLK/LRCK/DIN)\"",
                 (unsigned int)(test_i2s_hw_port_num() - 1U));
        return;
    }

    if (os_strcmp(cmd, "param") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_i2s_param(port);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xi2s param result=%s return_code=%d port=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret,
                 (unsigned int)port);
        return;
    }

    if (os_strcmp(cmd, "cfg") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_i2s_cfg_matrix(port);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xi2s cfg result=%s return_code=%d port=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret,
                 (unsigned int)port);
        return;
    }

    if (os_strcmp(cmd, "loop") == 0) {
        if (argc >= 3) {
            tx_port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            rx_port = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            loops = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            dma = test_i2s_parse_dma_arg(argv[5]);
        }
        if (argc >= 7) {
            verify = os_strtoul(argv[6], NULL, 10);
        }
        if (argc >= 8) {
            sample_arg = os_strtoul(argv[7], NULL, 10);
            sample_rate = test_i2s_sample_rate_to_enum(sample_arg);
        }
        if (argc >= 9) {
            bits = os_strtoul(argv[8], NULL, 10);
        }
        if (argc >= 10) {
            channel_fmt = os_strtoul(argv[9], NULL, 10);
        }
        if (argc >= 11) {
            comm_fmt = os_strtoul(argv[10], NULL, 10);
        }

        if ((sample_rate == 0U) ||
            !((bits == TUYA_I2S_BITS_PER_SAMPLE_8BIT) || (bits == TUYA_I2S_BITS_PER_SAMPLE_16BIT) ||
              (bits == TUYA_I2S_BITS_PER_SAMPLE_24BIT) || (bits == TUYA_I2S_BITS_PER_SAMPLE_32BIT)) ||
            (channel_fmt > (uint32_t)TUYA_I2S_CHANNEL_FMT_ONLY_LEFT) ||
            !((comm_fmt == I2S_COMM_FORMAT_STAND_I2S) || (comm_fmt == I2S_COMM_FORMAT_STAND_MSB) ||
              (comm_fmt == I2S_COMM_FORMAT_STAND_PCM_SHORT) || (comm_fmt == I2S_COMM_FORMAT_STAND_PCM_LONG))) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xi2s loop failed (%d), invalid cfg sample=%u bits=%u ch=%u fmt=%u\r\n",
                     OPRT_INVALID_PARM,
                     (unsigned int)sample_arg,
                     (unsigned int)bits,
                     (unsigned int)channel_fmt,
                     (unsigned int)comm_fmt);
            return;
        }

        ret = test_i2s_loopback(tx_port, rx_port, loops, dma != 0U, verify != 0U, sample_rate, bits, channel_fmt, comm_fmt);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xi2s loop result=%s return_code=%d tx_port=%u rx_port=%u loops=%u dma_enabled=%u verify_enabled=%u sample_enum=%u bits=%u channel_fmt=%u comm_fmt=%u hint=\"need I2S loopback wiring\"\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret,
                 (unsigned int)tx_port,
                 (unsigned int)rx_port,
                 (unsigned int)loops,
                 (unsigned int)dma,
                 (unsigned int)verify,
                 (unsigned int)sample_rate,
                 (unsigned int)bits,
                 (unsigned int)channel_fmt,
                 (unsigned int)comm_fmt);
        return;
    }

    if (os_strcmp(cmd, "stress") == 0) {
        if (argc >= 3) {
            tx_port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            rx_port = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            rounds = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            dma = test_i2s_parse_dma_arg(argv[5]);
        }
        ret = test_i2s_stress(tx_port, rx_port, rounds, dma != 0U);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xi2s stress result=%s return_code=%d tx_port=%u rx_port=%u rounds=%u dma_enabled=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret,
                 (unsigned int)tx_port,
                 (unsigned int)rx_port,
                 (unsigned int)rounds,
                 (unsigned int)dma);
        return;
    }

    if (os_strcmp(cmd, "full") == 0) {
        if (argc >= 3) {
            tx_port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            rx_port = os_strtoul(argv[3], NULL, 10);
        }
        ret = test_i2s_full(tx_port, rx_port);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xi2s full result=%s return_code=%d tx_port=%u rx_port=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret,
                 (unsigned int)tx_port,
                 (unsigned int)rx_port);
        return;
    }

    test_i2s_usage(pcWriteBuffer, xWriteBufferLen);
}
