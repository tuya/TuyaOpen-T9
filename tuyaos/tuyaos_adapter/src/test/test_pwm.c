#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/mem.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_output.h"
#include "tkl_pwm.h"
#include "tkl_system.h"

#define PWM_TEST_DEFAULT_CH            TUYA_PWM_NUM_0
#define PWM_TEST_DEFAULT_FREQ_HZ       1000U
#define PWM_TEST_DEFAULT_DUTY          5000U
#define PWM_TEST_DEFAULT_CYCLE         10000U
#define PWM_TEST_DEFAULT_POLARITY      TUYA_PWM_POSITIVE
#define PWM_TEST_DEFAULT_RUN_MS        2000U

typedef struct {
    bool initialized;
    bool started;
    bool capture_mode;
    TUYA_PWM_NUM_E channel;
    TUYA_PWM_BASE_CFG_T cfg;
    uint32_t cap_irq_cnt;
    TUYA_PWM_CAPTURE_DATA_T last_cap;
} test_pwm_ctx_t;

static test_pwm_ctx_t s_pwm_ctx = {
    .initialized = false,
    .started = false,
    .capture_mode = false,
    .channel = PWM_TEST_DEFAULT_CH,
    .cfg = {
        .polarity = PWM_TEST_DEFAULT_POLARITY,
        .count_mode = TUYA_PWM_CNT_UP,
        .duty = PWM_TEST_DEFAULT_DUTY,
        .cycle = PWM_TEST_DEFAULT_CYCLE,
        .frequency = PWM_TEST_DEFAULT_FREQ_HZ,
    },
    .cap_irq_cnt = 0U,
    .last_cap = {0},
};

static void test_pwm_runtime_reset(void)
{
    (void)os_memset(&s_pwm_ctx, 0, sizeof(s_pwm_ctx));
    s_pwm_ctx.channel = PWM_TEST_DEFAULT_CH;
    s_pwm_ctx.cfg.polarity = PWM_TEST_DEFAULT_POLARITY;
    s_pwm_ctx.cfg.count_mode = TUYA_PWM_CNT_UP;
    s_pwm_ctx.cfg.duty = PWM_TEST_DEFAULT_DUTY;
    s_pwm_ctx.cfg.cycle = PWM_TEST_DEFAULT_CYCLE;
    s_pwm_ctx.cfg.frequency = PWM_TEST_DEFAULT_FREQ_HZ;
}

static OPERATE_RET test_pwm_validate_cfg(uint32_t ch, uint32_t freq_hz, uint32_t duty, uint32_t cycle, uint32_t polarity)
{
    if (ch >= TUYA_PWM_NUM_MAX || freq_hz == 0U || cycle == 0U) {
        return OPRT_INVALID_PARM;
    }
    if (duty > cycle) {
        return OPRT_INVALID_PARM;
    }
    if (polarity > (uint32_t)TUYA_PWM_POSITIVE) {
        return OPRT_INVALID_PARM;
    }
    return OPRT_OK;
}

static OPERATE_RET test_pwm_deinit(void)
{
    OPERATE_RET ret = OPRT_OK;
    TUYA_PWM_NUM_E ch = s_pwm_ctx.initialized ? s_pwm_ctx.channel : PWM_TEST_DEFAULT_CH;

    if (s_pwm_ctx.capture_mode) {
        (void)tkl_pwm_cap_stop(ch);
    } else {
        (void)tkl_pwm_stop(ch);
    }

    ret = tkl_pwm_deinit(ch);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm deinit failed: %d\r\n", ret);
        return ret;
    }

    test_pwm_runtime_reset();
    return OPRT_OK;
}

static OPERATE_RET test_pwm_init(uint32_t ch, uint32_t freq_hz, uint32_t duty, uint32_t cycle, uint32_t polarity)
{
    OPERATE_RET ret;
    TUYA_PWM_BASE_CFG_T cfg;

    ret = test_pwm_validate_cfg(ch, freq_hz, duty, cycle, polarity);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (s_pwm_ctx.initialized) {
        ret = test_pwm_deinit();
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.polarity = (TUYA_PWM_POLARITY_E)polarity;
    cfg.count_mode = TUYA_PWM_CNT_UP;
    cfg.duty = duty;
    cfg.cycle = cycle;
    cfg.frequency = freq_hz;

    ret = tkl_pwm_init((TUYA_PWM_NUM_E)ch, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm init failed: %d\r\n", ret);
        return ret;
    }

    s_pwm_ctx.initialized = true;
    s_pwm_ctx.started = false;
    s_pwm_ctx.capture_mode = false;
    s_pwm_ctx.channel = (TUYA_PWM_NUM_E)ch;
    s_pwm_ctx.cfg = cfg;
    s_pwm_ctx.cap_irq_cnt = 0U;
    (void)os_memset(&s_pwm_ctx.last_cap, 0, sizeof(s_pwm_ctx.last_cap));
    return OPRT_OK;
}

static OPERATE_RET test_pwm_start(void)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || s_pwm_ctx.capture_mode) {
        return OPRT_RESOURCE_NOT_READY;
    }
    if (s_pwm_ctx.started) {
        return OPRT_OK;
    }

    ret = tkl_pwm_start(s_pwm_ctx.channel);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm start failed: %d\r\n", ret);
        return ret;
    }

    s_pwm_ctx.started = true;
    return OPRT_OK;
}

static OPERATE_RET test_pwm_stop(void)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || s_pwm_ctx.capture_mode) {
        return OPRT_OK;
    }
    if (!s_pwm_ctx.started) {
        return OPRT_OK;
    }

    ret = tkl_pwm_stop(s_pwm_ctx.channel);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm stop failed: %d\r\n", ret);
        return ret;
    }

    s_pwm_ctx.started = false;
    return OPRT_OK;
}

static OPERATE_RET test_pwm_set_duty(uint32_t duty)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || s_pwm_ctx.capture_mode) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (duty > s_pwm_ctx.cfg.cycle) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_pwm_duty_set(s_pwm_ctx.channel, duty);
    if (ret == OPRT_OK) {
        s_pwm_ctx.cfg.duty = duty;
    }

    return ret;
}

static OPERATE_RET test_pwm_set_frequency(uint32_t freq_hz)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || s_pwm_ctx.capture_mode) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (freq_hz == 0U) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_pwm_frequency_set(s_pwm_ctx.channel, freq_hz);
    if (ret == OPRT_OK) {
        s_pwm_ctx.cfg.frequency = freq_hz;
    }

    return ret;
}

static OPERATE_RET test_pwm_set_polarity(uint32_t polarity)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || s_pwm_ctx.capture_mode) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (polarity > (uint32_t)TUYA_PWM_POSITIVE) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_pwm_polarity_set(s_pwm_ctx.channel, (TUYA_PWM_POLARITY_E)polarity);
    if (ret == OPRT_OK) {
        s_pwm_ctx.cfg.polarity = (TUYA_PWM_POLARITY_E)polarity;
    }

    return ret;
}

static void test_pwm_cap_cb(TUYA_PWM_NUM_E port, TUYA_PWM_CAPTURE_DATA_T data, VOID_T *arg)
{
    (void)port;
    (void)arg;

    s_pwm_ctx.cap_irq_cnt++;
    s_pwm_ctx.last_cap = data;
}

static OPERATE_RET test_pwm_cap_start(uint32_t ch, uint32_t edge, uint32_t mode, uint32_t clk)
{
    OPERATE_RET ret;
    TUYA_PWM_CAP_IRQ_T cfg;

    if (ch >= TUYA_PWM_NUM_MAX || edge > (uint32_t)TUYA_PWM_POSITIVE || mode > 1U) {
        return OPRT_INVALID_PARM;
    }

    if (s_pwm_ctx.initialized) {
        ret = test_pwm_deinit();
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.cap_mode = (mode == 0U) ? TUYA_PWM_CAPTURE_MODE_ONCE : TUYA_PWM_CAPTURE_MODE_PERIOD;
    cfg.trigger_level = (TUYA_PWM_POLARITY_E)edge;
    cfg.clk = clk;
    cfg.cb = test_pwm_cap_cb;
    cfg.arg = NULL;

    ret = tkl_pwm_cap_start((TUYA_PWM_NUM_E)ch, &cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm cap start failed: %d\r\n", ret);
        return ret;
    }

    s_pwm_ctx.initialized = true;
    s_pwm_ctx.started = true;
    s_pwm_ctx.capture_mode = true;
    s_pwm_ctx.channel = (TUYA_PWM_NUM_E)ch;
    s_pwm_ctx.cap_irq_cnt = 0U;
    (void)os_memset(&s_pwm_ctx.last_cap, 0, sizeof(s_pwm_ctx.last_cap));
    return OPRT_OK;
}

static OPERATE_RET test_pwm_cap_stop(void)
{
    OPERATE_RET ret;

    if (!s_pwm_ctx.initialized || !s_pwm_ctx.capture_mode) {
        return OPRT_OK;
    }

    ret = tkl_pwm_cap_stop(s_pwm_ctx.channel);
    if (ret != OPRT_OK) {
        tkl_log_output("pwm cap stop failed: %d\r\n", ret);
        return ret;
    }

    test_pwm_runtime_reset();
    return OPRT_OK;
}

static OPERATE_RET test_pwm_run(uint32_t run_ms, uint32_t freq_hz, uint32_t duty, uint32_t cycle)
{
    OPERATE_RET ret;

    if (run_ms == 0U) {
        return OPRT_INVALID_PARM;
    }

    ret = test_pwm_init((uint32_t)PWM_TEST_DEFAULT_CH, freq_hz, duty, cycle, (uint32_t)PWM_TEST_DEFAULT_POLARITY);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = test_pwm_start();
    if (ret != OPRT_OK) {
        return ret;
    }

    tkl_system_sleep(run_ms);
    return test_pwm_stop();
}

/* ---------------------------------------------------------------------------
 * Single-channel automated test suite (API / state / boundary checks).
 * No external wiring required.
 * --------------------------------------------------------------------------- */
typedef struct {
    uint32_t pass;
    uint32_t fail;
    uint32_t skip;
} pwm_stat_t;

static void test_pwm_stat_add(pwm_stat_t *st, const char *name, bool pass, bool skip, OPERATE_RET ret)
{
    if ((st == NULL) || (name == NULL)) {
        return;
    }
    if (skip) {
        st->skip++;
        tkl_log_output("[xpwm] SKIP test_name=%s return_code=%d\r\n", name, (int)ret);
        return;
    }
    if (pass) {
        st->pass++;
        tkl_log_output("[xpwm] PASS test_name=%s\r\n", name);
    } else {
        st->fail++;
        tkl_log_output("[xpwm] FAIL test_name=%s return_code=%d\r\n", name, (int)ret);
    }
}

static void test_pwm_stat_summary(const pwm_stat_t *st, const char *tag)
{
    tkl_log_output("[xpwm] result_summary test_name=%s passed_count=%u failed_count=%u skipped_count=%u\r\n",
                   tag,
                   (unsigned int)st->pass,
                   (unsigned int)st->fail,
                   (unsigned int)st->skip);
}

static OPERATE_RET test_pwm_param(uint32_t ch)
{
    pwm_stat_t st = {0};
    TUYA_PWM_BASE_CFG_T base;
    TUYA_PWM_BASE_CFG_T bad;
    TUYA_PWM_BASE_CFG_T got;
    OPERATE_RET ret;

    if (ch >= TUYA_PWM_NUM_MAX) {
        return OPRT_INVALID_PARM;
    }

    /* Clean slate: drop the shared single-channel context and this channel. */
    (void)test_pwm_deinit();
    (void)tkl_pwm_deinit((TUYA_PWM_NUM_E)ch);

    (void)os_memset(&base, 0, sizeof(base));
    base.polarity = TUYA_PWM_POSITIVE;
    base.count_mode = TUYA_PWM_CNT_UP;
    base.cycle = 10000U;
    base.duty = 5000U;
    base.frequency = 1000U;

    /* ---- init parameter validation (all expected to be rejected) ---- */
    ret = tkl_pwm_init((TUYA_PWM_NUM_E)TUYA_PWM_NUM_MAX, &base);
    test_pwm_stat_add(&st, "TC001 init(invalid_ch)", (ret != OPRT_OK), false, ret);

    ret = tkl_pwm_init((TUYA_PWM_NUM_E)ch, NULL);
    test_pwm_stat_add(&st, "TC002 init(NULL_cfg)", (ret != OPRT_OK), false, ret);

    bad = base;
    bad.cycle = 0U;
    ret = tkl_pwm_init((TUYA_PWM_NUM_E)ch, &bad);
    test_pwm_stat_add(&st, "TC003 init(cycle0)", (ret != OPRT_OK), false, ret);

    bad = base;
    bad.frequency = 0U;
    ret = tkl_pwm_init((TUYA_PWM_NUM_E)ch, &bad);
    test_pwm_stat_add(&st, "TC004 init(freq0)", (ret != OPRT_OK), false, ret);

    bad = base;
    bad.duty = bad.cycle + 1U;
    ret = tkl_pwm_init((TUYA_PWM_NUM_E)ch, &bad);
    test_pwm_stat_add(&st, "TC005 init(duty>cycle)", (ret != OPRT_OK), false, ret);

    bad = base;
    bad.count_mode = TUYA_PWM_CNT_UP_AND_DOWN; /* driver supports UP only */
    ret = tkl_pwm_init((TUYA_PWM_NUM_E)ch, &bad);
    test_pwm_stat_add(&st, "TC006 init(count_mode_unsupported)", (ret != OPRT_OK), false, ret);

    /* ---- operations before init (expected to be rejected) ---- */
    ret = tkl_pwm_start((TUYA_PWM_NUM_E)ch);
    test_pwm_stat_add(&st, "TC007 start(before_init)", (ret != OPRT_OK), false, ret);

    ret = tkl_pwm_stop((TUYA_PWM_NUM_E)ch);
    test_pwm_stat_add(&st, "TC008 stop(before_init)", (ret != OPRT_OK), false, ret);

    ret = tkl_pwm_duty_set((TUYA_PWM_NUM_E)ch, 3000U);
    test_pwm_stat_add(&st, "TC009 duty_set(before_init)", (ret != OPRT_OK), false, ret);

    ret = tkl_pwm_frequency_set((TUYA_PWM_NUM_E)ch, 2000U);
    test_pwm_stat_add(&st, "TC010 freq_set(before_init)", (ret != OPRT_OK), false, ret);

    ret = tkl_pwm_polarity_set((TUYA_PWM_NUM_E)ch, TUYA_PWM_NEGATIVE);
    test_pwm_stat_add(&st, "TC011 pol_set(before_init)", (ret != OPRT_OK), false, ret);

    ret = tkl_pwm_info_get((TUYA_PWM_NUM_E)ch, &got);
    test_pwm_stat_add(&st, "TC012 info_get(before_init)", (ret != OPRT_OK), false, ret);

    ret = tkl_pwm_info_get((TUYA_PWM_NUM_E)ch, NULL);
    test_pwm_stat_add(&st, "TC013 info_get(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_pwm_deinit((TUYA_PWM_NUM_E)ch);
    test_pwm_stat_add(&st, "TC014 deinit(before_init)", (ret == OPRT_OK), false, ret);

    ret = tkl_pwm_deinit((TUYA_PWM_NUM_E)TUYA_PWM_NUM_MAX);
    test_pwm_stat_add(&st, "TC015 deinit(invalid_ch)", (ret != OPRT_OK), false, ret);

    /* ---- valid lifecycle ---- */
    ret = tkl_pwm_init((TUYA_PWM_NUM_E)ch, &base);
    test_pwm_stat_add(&st, "TC016 init(valid)", (ret == OPRT_OK), false, ret);
    if (ret != OPRT_OK) {
        test_pwm_stat_summary(&st, "param");
        return OPRT_COM_ERROR;
    }

    ret = tkl_pwm_init((TUYA_PWM_NUM_E)ch, &base);
    test_pwm_stat_add(&st, "TC017 init(repeat)", (ret == OPRT_OK), false, ret);

    (void)os_memset(&got, 0, sizeof(got));
    ret = tkl_pwm_info_get((TUYA_PWM_NUM_E)ch, &got);
    test_pwm_stat_add(&st, "TC018 info_get",
                      (ret == OPRT_OK) && (got.frequency == base.frequency) && (got.cycle == base.cycle),
                      false, ret);

    ret = tkl_pwm_start((TUYA_PWM_NUM_E)ch);
    test_pwm_stat_add(&st, "TC019 start", (ret == OPRT_OK), false, ret);

    ret = tkl_pwm_duty_set((TUYA_PWM_NUM_E)ch, 0U);
    test_pwm_stat_add(&st, "TC020 duty_set(0)", (ret == OPRT_OK), false, ret);

    ret = tkl_pwm_duty_set((TUYA_PWM_NUM_E)ch, base.cycle);
    test_pwm_stat_add(&st, "TC021 duty_set(=cycle)", (ret == OPRT_OK), false, ret);

    ret = tkl_pwm_duty_set((TUYA_PWM_NUM_E)ch, base.cycle + 1U);
    test_pwm_stat_add(&st, "TC022 duty_set(>cycle)", (ret != OPRT_OK), false, ret);

    ret = tkl_pwm_duty_set((TUYA_PWM_NUM_E)ch, 2500U);
    test_pwm_stat_add(&st, "TC023 duty_set(valid)", (ret == OPRT_OK), false, ret);

    ret = tkl_pwm_frequency_set((TUYA_PWM_NUM_E)ch, 5000U);
    test_pwm_stat_add(&st, "TC024 freq_set(valid)", (ret == OPRT_OK), false, ret);

    ret = tkl_pwm_frequency_set((TUYA_PWM_NUM_E)ch, 0U);
    test_pwm_stat_add(&st, "TC025 freq_set(0)", (ret != OPRT_OK), false, ret);

    ret = tkl_pwm_polarity_set((TUYA_PWM_NUM_E)ch, TUYA_PWM_NEGATIVE);
    test_pwm_stat_add(&st, "TC026 pol_set(neg)", (ret == OPRT_OK), false, ret);

    ret = tkl_pwm_polarity_set((TUYA_PWM_NUM_E)ch, TUYA_PWM_POSITIVE);
    test_pwm_stat_add(&st, "TC027 pol_set(pos)", (ret == OPRT_OK), false, ret);

    ret = tkl_pwm_polarity_set((TUYA_PWM_NUM_E)ch, (TUYA_PWM_POLARITY_E)0xFF);
    test_pwm_stat_add(&st, "TC028 pol_set(invalid)", (ret != OPRT_OK), false, ret);

    tkl_system_sleep(20U); /* let the output run briefly */

    ret = tkl_pwm_stop((TUYA_PWM_NUM_E)ch);
    test_pwm_stat_add(&st, "TC029 stop", (ret == OPRT_OK), false, ret);

    ret = tkl_pwm_deinit((TUYA_PWM_NUM_E)ch);
    test_pwm_stat_add(&st, "TC030 deinit", (ret == OPRT_OK), false, ret);

    ret = tkl_pwm_deinit((TUYA_PWM_NUM_E)ch);
    test_pwm_stat_add(&st, "TC031 deinit(repeat)", (ret == OPRT_OK), false, ret);

    /* ---- multichannel API guards (single element) ---- */
    ret = tkl_pwm_multichannel_start(NULL, 1U);
    test_pwm_stat_add(&st, "TC032 multi_start(NULL)", (ret != OPRT_OK), false, ret);

    {
        TUYA_PWM_NUM_E chs[1] = { (TUYA_PWM_NUM_E)ch };
        ret = tkl_pwm_multichannel_start(chs, 0U);
        test_pwm_stat_add(&st, "TC033 multi_start(num0)", (ret != OPRT_OK), false, ret);
    }

    test_pwm_stat_summary(&st, "param");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

/* GPIO that each PWM channel maps to (GPIO_PWM_MAP_TABLE), for the wiring hint. */
static const uint8_t s_pwm_ch_gpio[12] = {47, 48, 49, 50, 51, 52, 53, 54, 55, 8, 28, 9};

typedef struct {
    volatile uint32_t irq_cnt;
    volatile uint32_t last_value;
    volatile uint32_t last_edge;
} pwm_capself_ctx_t;

static pwm_capself_ctx_t s_pwm_capself;

static void test_pwm_capself_cb(TUYA_PWM_NUM_E port, TUYA_PWM_CAPTURE_DATA_T data, VOID_T *arg)
{
    (void)port;
    (void)arg;
    s_pwm_capself.irq_cnt++;
    s_pwm_capself.last_value = data.cap_value;
    s_pwm_capself.last_edge = (uint32_t)data.cap_edge;
}

/*
 * Self-contained capture test: gen_ch outputs a known square wave that must be
 * jumpered to cap_ch's input pin; cap_ch captures it. Validates capture without
 * an external signal generator.
 *
 * Verdict is clock-independent: the same source is measured at f and 2*f, and
 * the captured period count must (roughly) halve. This avoids depending on the
 * exact capture-timer clock, which is not assumed here.
 *
 * Uses tkl_pwm directly (independent per-channel driver state), so it does not
 * disturb / is not limited by the single-channel s_pwm_ctx used by the other
 * xpwm subcommands.
 */
static OPERATE_RET test_pwm_cap_selftest(uint32_t gen_ch, uint32_t cap_ch, uint32_t freq_hz)
{
    OPERATE_RET ret;
    TUYA_PWM_BASE_CFG_T gen_cfg;
    TUYA_PWM_CAP_IRQ_T cap_cfg;
    uint32_t v_f;
    uint32_t v_2f;
    uint32_t cnt_after_f;
    bool edges;
    bool nonzero;
    bool ratio_ok;

    if (gen_ch >= TUYA_PWM_NUM_MAX || cap_ch >= TUYA_PWM_NUM_MAX ||
        gen_ch == cap_ch || freq_hz == 0U) {
        return OPRT_INVALID_PARM;
    }

    /* Release the shared single-channel context so it cannot collide. */
    (void)test_pwm_deinit();
    (void)os_memset(&s_pwm_capself, 0, sizeof(s_pwm_capself));

    /* Capture side: continuous, rising-to-rising (one period per capture). */
    (void)os_memset(&cap_cfg, 0, sizeof(cap_cfg));
    cap_cfg.cap_mode = TUYA_PWM_CAPTURE_MODE_PERIOD;
    cap_cfg.trigger_level = TUYA_PWM_POSITIVE;
    cap_cfg.cb = test_pwm_capself_cb;
    cap_cfg.arg = NULL;
    ret = tkl_pwm_cap_start((TUYA_PWM_NUM_E)cap_ch, &cap_cfg);
    if (ret != OPRT_OK) {
        tkl_log_output("[xpwm] capself cap_start failed cap_ch=%u ret=%d\r\n",
                       (unsigned int)cap_ch, ret);
        return ret;
    }

    /* Generator side at freq_hz, 50%% duty. */
    (void)os_memset(&gen_cfg, 0, sizeof(gen_cfg));
    gen_cfg.polarity = TUYA_PWM_POSITIVE;
    gen_cfg.count_mode = TUYA_PWM_CNT_UP;
    gen_cfg.cycle = 10000U;
    gen_cfg.duty = 5000U;
    gen_cfg.frequency = freq_hz;
    ret = tkl_pwm_init((TUYA_PWM_NUM_E)gen_ch, &gen_cfg);
    if (ret == OPRT_OK) {
        ret = tkl_pwm_start((TUYA_PWM_NUM_E)gen_ch);
    }
    if (ret != OPRT_OK) {
        tkl_log_output("[xpwm] capself gen start failed gen_ch=%u ret=%d\r\n",
                       (unsigned int)gen_ch, ret);
        (void)tkl_pwm_cap_stop((TUYA_PWM_NUM_E)cap_ch);
        return ret;
    }

    tkl_system_sleep(50U);
    cnt_after_f = s_pwm_capself.irq_cnt;
    v_f = s_pwm_capself.last_value;

    /* Second measurement point at 2*f for the clock-independent ratio check. */
    (void)tkl_pwm_frequency_set((TUYA_PWM_NUM_E)gen_ch, freq_hz * 2U);
    tkl_system_sleep(50U);
    v_2f = s_pwm_capself.last_value;

    (void)tkl_pwm_stop((TUYA_PWM_NUM_E)gen_ch);
    (void)tkl_pwm_deinit((TUYA_PWM_NUM_E)gen_ch);
    (void)tkl_pwm_cap_stop((TUYA_PWM_NUM_E)cap_ch);

    edges = (cnt_after_f > 0U) && (s_pwm_capself.irq_cnt > cnt_after_f);
    nonzero = (v_f > 0U) && (v_2f > 0U);
    /* Doubling f halves the period count: expect v_f / v_2f ~ 2 (accept 1.6..2.5). */
    ratio_ok = nonzero && (v_f * 10U >= v_2f * 16U) && (v_f * 10U <= v_2f * 25U);

    tkl_log_output("[xpwm] capself gen_ch=%u(GPIO%u) cap_ch=%u(GPIO%u) freq=%u cap@f=%u cap@2f=%u irq=%u edges=%u ratio_ok=%u result=%s\r\n",
                   (unsigned int)gen_ch, (unsigned int)s_pwm_ch_gpio[gen_ch],
                   (unsigned int)cap_ch, (unsigned int)s_pwm_ch_gpio[cap_ch],
                   (unsigned int)freq_hz,
                   (unsigned int)v_f, (unsigned int)v_2f,
                   (unsigned int)s_pwm_capself.irq_cnt,
                   (unsigned int)edges, (unsigned int)ratio_ok,
                   (!edges) ? "NO_SIGNAL(skip)" : (ratio_ok ? "PASS" : "FAIL"));

    if (!edges) {
        /* No capture interrupts at all -> no signal reached cap_ch, almost
         * certainly the gen->cap jumper is not wired. Report as unsupported so
         * callers can record it as SKIP rather than FAIL. */
        return OPRT_NOT_SUPPORTED;
    }
    return ratio_ok ? OPRT_OK : OPRT_COM_ERROR;
}

/*
 * Wiring-dependent capture test case. Requires a jumper between gen_ch's output
 * GPIO and cap_ch's input GPIO (default GPIO28 <-> GPIO8). Recorded as:
 *   PASS  - signal captured and the f / 2f period ratio is correct;
 *   SKIP  - no edges captured (jumper not present);
 *   FAIL  - edges captured but the measured ratio is wrong.
 */
static OPERATE_RET test_pwm_capture(uint32_t gen_ch, uint32_t cap_ch, uint32_t freq_hz)
{
    pwm_stat_t st = {0};
    char tc_name[48];
    OPERATE_RET ret;

    if (gen_ch >= TUYA_PWM_NUM_MAX || cap_ch >= TUYA_PWM_NUM_MAX) {
        return OPRT_INVALID_PARM;
    }

    ret = test_pwm_cap_selftest(gen_ch, cap_ch, freq_hz);
    (void)snprintf(tc_name, sizeof(tc_name), "TC101 capture_loopback_ch%uto%u",
                   (unsigned int)gen_ch, (unsigned int)cap_ch);
    test_pwm_stat_add(&st, tc_name, (ret == OPRT_OK), (ret == OPRT_NOT_SUPPORTED), ret);

    test_pwm_stat_summary(&st, "capture");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

void cli_pwm_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;
    uint32_t ch = (uint32_t)PWM_TEST_DEFAULT_CH;
    uint32_t freq_hz = PWM_TEST_DEFAULT_FREQ_HZ;
    uint32_t duty = PWM_TEST_DEFAULT_DUTY;
    uint32_t cycle = PWM_TEST_DEFAULT_CYCLE;
    uint32_t polarity = (uint32_t)PWM_TEST_DEFAULT_POLARITY;
    uint32_t run_ms = PWM_TEST_DEFAULT_RUN_MS;
    uint32_t edge = (uint32_t)TUYA_PWM_POSITIVE;
    uint32_t mode = (uint32_t)TUYA_PWM_CAPTURE_MODE_PERIOD;
    uint32_t clk = 0U;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xpwm <init [ch] [freq] [duty] [cycle] [pol]|start|duty [v]|freq [hz]|pol [0|1]|run [ms] [freq] [duty] [cycle]|cap_start [ch] [edge] [mode] [clk]|cap_stop|capself [gen_ch] [cap_ch] [freq]|capcase [gen_ch] [cap_ch] [freq]|param [ch]|full|stop|deinit|status>");
        return;
    }

    cmd = argv[1];
    if (os_strcmp(cmd, "init") == 0) {
        if (argc >= 3) {
            ch = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            freq_hz = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            duty = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            cycle = os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            polarity = os_strtoul(argv[6], NULL, 10);
        }

        ret = test_pwm_init(ch, freq_hz, duty, cycle, polarity);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm init %s (%d), ch=%u freq=%u duty=%u cycle=%u pol=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)ch,
                 (unsigned int)freq_hz,
                 (unsigned int)duty,
                 (unsigned int)cycle,
                 (unsigned int)polarity);
    } else if (os_strcmp(cmd, "start") == 0) {
        ret = test_pwm_start();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm start %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "duty") == 0) {
        if (argc >= 3) {
            duty = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_pwm_set_duty(duty);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm duty %s (%d), duty=%u cycle=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)duty,
                 (unsigned int)s_pwm_ctx.cfg.cycle);
    } else if (os_strcmp(cmd, "freq") == 0) {
        if (argc >= 3) {
            freq_hz = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_pwm_set_frequency(freq_hz);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm freq %s (%d), freq=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)freq_hz);
    } else if (os_strcmp(cmd, "pol") == 0 || os_strcmp(cmd, "polarity") == 0) {
        if (argc >= 3) {
            polarity = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_pwm_set_polarity(polarity);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm pol %s (%d), polarity=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)polarity);
    } else if (os_strcmp(cmd, "run") == 0) {
        if (argc >= 3) {
            run_ms = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            freq_hz = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            duty = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            cycle = os_strtoul(argv[5], NULL, 10);
        }

        ret = test_pwm_run(run_ms, freq_hz, duty, cycle);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm run %s (%d), ms=%u freq=%u duty=%u cycle=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)run_ms,
                 (unsigned int)freq_hz,
                 (unsigned int)duty,
                 (unsigned int)cycle);
    } else if (os_strcmp(cmd, "cap_start") == 0) {
        if (argc >= 3) {
            ch = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            edge = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            mode = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            clk = os_strtoul(argv[5], NULL, 10);
        }

        ret = test_pwm_cap_start(ch, edge, mode, clk);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm cap_start %s (%d), ch=%u edge=%u mode=%u clk=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)ch,
                 (unsigned int)edge,
                 (unsigned int)mode,
                 (unsigned int)clk);
    } else if (os_strcmp(cmd, "cap_stop") == 0) {
        ret = test_pwm_cap_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm cap_stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "param") == 0) {
        uint32_t param_ch = (uint32_t)PWM_TEST_DEFAULT_CH;
        if (argc >= 3) {
            param_ch = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_pwm_param(param_ch);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm param %s (%d), ch=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)param_ch);
    } else if (os_strcmp(cmd, "full") == 0) {
        uint32_t c;
        uint32_t fail = 0U;
        for (c = 0U; c < (uint32_t)TUYA_PWM_NUM_MAX; c++) {
            if (test_pwm_param(c) != OPRT_OK) {
                fail++;
            }
        }
        /* Wiring-dependent capture case (default GPIO28<->GPIO8); SKIP if no jumper. */
        if (test_pwm_capture(10U, 9U, 1000U) != OPRT_OK) {
            fail++;
        }
        ret = (fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm full %s (%d), param_channels=%u failed=%u (capture needs GPIO28<->GPIO8 jumper)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)TUYA_PWM_NUM_MAX,
                 (unsigned int)fail);
    } else if (os_strcmp(cmd, "capcase") == 0) {
        uint32_t gen_ch = 10U; /* GPIO28 */
        uint32_t cap_ch = 9U;  /* GPIO8  */
        uint32_t case_freq = 1000U;
        if (argc >= 3) {
            gen_ch = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            cap_ch = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            case_freq = os_strtoul(argv[4], NULL, 10);
        }
        ret = test_pwm_capture(gen_ch, cap_ch, case_freq);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm capcase %s (%d), gen_ch=%u cap_ch=%u freq=%u (jumper the two GPIOs)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)gen_ch,
                 (unsigned int)cap_ch,
                 (unsigned int)case_freq);
    } else if (os_strcmp(cmd, "capself") == 0 || os_strcmp(cmd, "cap_selftest") == 0) {
        uint32_t gen_ch = 10U; /* GPIO28 */
        uint32_t cap_ch = 9U;  /* GPIO8  */
        uint32_t self_freq = 1000U;
        if (argc >= 3) {
            gen_ch = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            cap_ch = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            self_freq = os_strtoul(argv[4], NULL, 10);
        }
        ret = test_pwm_cap_selftest(gen_ch, cap_ch, self_freq);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm capself %s (%d), gen_ch=%u cap_ch=%u freq=%u (jumper the two GPIOs together)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)gen_ch,
                 (unsigned int)cap_ch,
                 (unsigned int)self_freq);
    } else if (os_strcmp(cmd, "stop") == 0) {
        ret = test_pwm_stop();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm stop %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "deinit") == 0 || os_strcmp(cmd, "close") == 0) {
        ret = test_pwm_deinit();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm %s %s (%d)\r\n",
                 cmd,
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
    } else if (os_strcmp(cmd, "status") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xpwm status init=%u start=%u cap=%u ch=%u freq=%u duty=%u cycle=%u pol=%u cap_irq=%u last_cap=%u last_edge=%u\r\n",
                 (unsigned int)s_pwm_ctx.initialized,
                 (unsigned int)s_pwm_ctx.started,
                 (unsigned int)s_pwm_ctx.capture_mode,
                 (unsigned int)s_pwm_ctx.channel,
                 (unsigned int)s_pwm_ctx.cfg.frequency,
                 (unsigned int)s_pwm_ctx.cfg.duty,
                 (unsigned int)s_pwm_ctx.cfg.cycle,
                 (unsigned int)s_pwm_ctx.cfg.polarity,
                 (unsigned int)s_pwm_ctx.cap_irq_cnt,
                 (unsigned int)s_pwm_ctx.last_cap.cap_value,
                 (unsigned int)s_pwm_ctx.last_cap.cap_edge);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: xpwm <init [ch] [freq] [duty] [cycle] [pol]|start|duty [v]|freq [hz]|pol [0|1]|run [ms] [freq] [duty] [cycle]|cap_start [ch] [edge] [mode] [clk]|cap_stop|capself [gen_ch] [cap_ch] [freq]|capcase [gen_ch] [cap_ch] [freq]|param [ch]|full|stop|deinit|status>");
    }
}

