#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_system.h"
#include "tkl_timer.h"

#define TIMER_TEST_DEFAULT_PORT        TUYA_TIMER_NUM_0
#define TIMER_TEST_MULTI_BASE_PORT     TUYA_TIMER_NUM_0
#define TIMER_TEST_MULTI_COUNT         3U
#define TIMER_TEST_ONCE_US             50000U
#define TIMER_TEST_PERIOD_US           30000U
#define TIMER_TEST_PERIOD_EXPECT_HIT   2U
#define TIMER_TEST_WAIT_TIMEOUT_MS     1200U
#define TIMER_TEST_POLL_INTERVAL_MS    2U
#define TIMER_TEST_STOP_SETTLE_MS      120U
#define TIMER_TEST_MAX_PERIOD_US       165191000U
#define TIMER_TEST_OVER_MAX_PERIOD_US  (TIMER_TEST_MAX_PERIOD_US + 1000U)

extern void bk_printf(const char *fmt, ...);
#define TIMER_TEST_LOG(...) bk_printf(__VA_ARGS__)

typedef struct {
    uint32_t pass;
    uint32_t fail;
    uint32_t skip;
} timer_stat_t;

typedef struct {
    volatile uint32_t hit_cnt;
} timer_cb_ctx_t;

static timer_cb_ctx_t s_timer_cb_ctx[TUYA_TIMER_NUM_MAX];
static volatile uint32_t s_timer_noarg_hit_cnt = 0U;

static bool test_timer_port_in_range(uint32_t port)
{
    return (port < (uint32_t)TUYA_TIMER_NUM_MAX);
}

static void test_timer_usage(char *pcWriteBuffer, int xWriteBufferLen)
{
    snprintf(pcWriteBuffer, xWriteBufferLen,
             "Usage: xtimer <list|api [port]|param [port]|multi [base_port] [count]|full>");
}

static void test_timer_stat_add(timer_stat_t *st, const char *name, bool pass, bool skip, OPERATE_RET ret)
{
    if ((st == NULL) || (name == NULL)) {
        return;
    }

    if (skip) {
        st->skip++;
        TIMER_TEST_LOG("[xtimer] SKIP %s, ret=%d\r\n", name, ret);
        return;
    }

    if (pass) {
        st->pass++;
        TIMER_TEST_LOG("[xtimer] PASS %s\r\n", name);
    } else {
        st->fail++;
        TIMER_TEST_LOG("[xtimer] FAIL %s, ret=%d\r\n", name, ret);
    }
}

static void test_timer_stat_summary(const timer_stat_t *st, const char *tag)
{
    if ((st == NULL) || (tag == NULL)) {
        return;
    }

    TIMER_TEST_LOG("[xtimer] %s summary: pass=%u fail=%u skip=%u\r\n",
                   tag,
                   (unsigned int)st->pass,
                   (unsigned int)st->fail,
                   (unsigned int)st->skip);
}

static OPERATE_RET test_timer_wait_hit(volatile uint32_t *hit_cnt, uint32_t expect, uint32_t timeout_ms)
{
    uint32_t waited = 0U;

    if (hit_cnt == NULL) {
        return OPRT_INVALID_PARM;
    }

    while (waited < timeout_ms) {
        if (*hit_cnt >= expect) {
            return OPRT_OK;
        }
        tkl_system_sleep(TIMER_TEST_POLL_INTERVAL_MS);
        waited += TIMER_TEST_POLL_INTERVAL_MS;
    }

    return OPRT_TIMEOUT;
}

static VOID test_timer_cb(VOID *args)
{
    timer_cb_ctx_t *ctx = (timer_cb_ctx_t *)args;

    if (ctx != NULL) {
        ctx->hit_cnt++;
    }
}

static VOID test_timer_noarg_cb(VOID *args)
{
    (void)args;
    s_timer_noarg_hit_cnt++;
}

static OPERATE_RET test_timer_cleanup(uint32_t port)
{
    (void)tkl_timer_stop((TUYA_TIMER_NUM_E)port);
    return tkl_timer_deinit((TUYA_TIMER_NUM_E)port);
}

static OPERATE_RET test_timer_param(uint32_t port)
{
    timer_stat_t st = {0};
    TUYA_TIMER_BASE_CFG_T cfg;
    TUYA_TIMER_BASE_CFG_T bad_cfg;
    TUYA_TIMER_BASE_CFG_T null_cb_cfg;
    TUYA_TIMER_BASE_CFG_T null_arg_cfg;
    UINT_T us_cfg = 0U;
    OPERATE_RET ret;

    if (!test_timer_port_in_range(port)) {
        return OPRT_INVALID_PARM;
    }

    (void)test_timer_cleanup(port);

    cfg.mode = TUYA_TIMER_MODE_ONCE;
    cfg.cb = test_timer_cb;
    cfg.args = (VOID *)&s_timer_cb_ctx[port];

    bad_cfg = cfg;
    bad_cfg.mode = (TUYA_TIMER_MODE_E)0xFF;

    ret = tkl_timer_init((TUYA_TIMER_NUM_E)port, NULL);
    test_timer_stat_add(&st, "TC001 init(NULL cfg)", (ret != OPRT_OK), false, ret);

    ret = tkl_timer_init((TUYA_TIMER_NUM_E)TUYA_TIMER_NUM_MAX, &cfg);
    test_timer_stat_add(&st, "TC002 init(invalid port)", (ret != OPRT_OK), false, ret);

    ret = tkl_timer_init((TUYA_TIMER_NUM_E)port, &bad_cfg);
    test_timer_stat_add(&st, "TC003 init(invalid mode)", (ret != OPRT_OK), false, ret);

    ret = tkl_timer_start((TUYA_TIMER_NUM_E)port, TIMER_TEST_ONCE_US);
    test_timer_stat_add(&st, "TC004 start(before init)", (ret != OPRT_OK), false, ret);

    ret = tkl_timer_start((TUYA_TIMER_NUM_E)port, 0U);
    test_timer_stat_add(&st, "TC005 start(us=0)", (ret != OPRT_OK), false, ret);

    ret = tkl_timer_get((TUYA_TIMER_NUM_E)port, NULL);
    test_timer_stat_add(&st, "TC006 get(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_timer_get_current_value((TUYA_TIMER_NUM_E)port, NULL);
    test_timer_stat_add(&st, "TC007 get_current(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_timer_stop((TUYA_TIMER_NUM_E)port);
    test_timer_stat_add(&st, "TC008 stop(before init)", (ret != OPRT_OK), false, ret);

    ret = tkl_timer_deinit((TUYA_TIMER_NUM_E)TUYA_TIMER_NUM_MAX);
    test_timer_stat_add(&st, "TC009 deinit(invalid port)", (ret != OPRT_OK), false, ret);

    ret = tkl_timer_get((TUYA_TIMER_NUM_E)port, &us_cfg);
    test_timer_stat_add(&st, "TC010 get(before init)", (ret != OPRT_OK), false, ret);

    null_cb_cfg = cfg;
    null_cb_cfg.cb = NULL;
    null_cb_cfg.args = NULL;
    ret = tkl_timer_init((TUYA_TIMER_NUM_E)port, &null_cb_cfg);
    test_timer_stat_add(&st, "TC011 init(cb=NULL)", (ret == OPRT_OK), false, ret);
    if (ret == OPRT_OK) {
        ret = tkl_timer_start((TUYA_TIMER_NUM_E)port, TIMER_TEST_ONCE_US);
        test_timer_stat_add(&st, "TC012 start(cb=NULL)", (ret == OPRT_OK), false, ret);
        if (ret == OPRT_OK) {
            tkl_system_sleep(TIMER_TEST_STOP_SETTLE_MS);
            ret = tkl_timer_stop((TUYA_TIMER_NUM_E)port);
            test_timer_stat_add(&st, "TC013 stop(cb=NULL)", (ret == OPRT_OK), false, ret);
        }
        ret = tkl_timer_deinit((TUYA_TIMER_NUM_E)port);
        test_timer_stat_add(&st, "TC014 deinit(cb=NULL)", (ret == OPRT_OK), false, ret);
    }

    null_arg_cfg = cfg;
    null_arg_cfg.cb = test_timer_noarg_cb;
    null_arg_cfg.args = NULL;
    s_timer_noarg_hit_cnt = 0U;
    ret = tkl_timer_init((TUYA_TIMER_NUM_E)port, &null_arg_cfg);
    test_timer_stat_add(&st, "TC015 init(args=NULL)", (ret == OPRT_OK), false, ret);
    if (ret == OPRT_OK) {
        ret = tkl_timer_start((TUYA_TIMER_NUM_E)port, TIMER_TEST_ONCE_US);
        test_timer_stat_add(&st, "TC016 start(args=NULL)", (ret == OPRT_OK), false, ret);
        if (ret == OPRT_OK) {
            ret = test_timer_wait_hit(&s_timer_noarg_hit_cnt, 1U, TIMER_TEST_WAIT_TIMEOUT_MS);
            test_timer_stat_add(&st, "TC017 wait_cb(args=NULL)", (ret == OPRT_OK), false, ret);
        }
        ret = tkl_timer_deinit((TUYA_TIMER_NUM_E)port);
        test_timer_stat_add(&st, "TC018 deinit(args=NULL)", (ret == OPRT_OK), false, ret);
    }

    null_cb_cfg.mode = TUYA_TIMER_MODE_PERIOD;
    ret = tkl_timer_init((TUYA_TIMER_NUM_E)port, &null_cb_cfg);
    test_timer_stat_add(&st, "TC019 init(period,cb=NULL)", (ret == OPRT_OK), false, ret);
    if (ret == OPRT_OK) {
        ret = tkl_timer_start((TUYA_TIMER_NUM_E)port, TIMER_TEST_PERIOD_US);
        test_timer_stat_add(&st, "TC020 start(period,cb=NULL)", (ret == OPRT_OK), false, ret);
        if (ret == OPRT_OK) {
            tkl_system_sleep(TIMER_TEST_STOP_SETTLE_MS);
            ret = tkl_timer_stop((TUYA_TIMER_NUM_E)port);
            test_timer_stat_add(&st, "TC021 stop(period,cb=NULL)", (ret == OPRT_OK), false, ret);
        }
        ret = tkl_timer_deinit((TUYA_TIMER_NUM_E)port);
        test_timer_stat_add(&st, "TC022 deinit(period,cb=NULL)", (ret == OPRT_OK), false, ret);
    }

    null_arg_cfg.mode = TUYA_TIMER_MODE_PERIOD;
    s_timer_noarg_hit_cnt = 0U;
    ret = tkl_timer_init((TUYA_TIMER_NUM_E)port, &null_arg_cfg);
    test_timer_stat_add(&st, "TC023 init(period,args=NULL)", (ret == OPRT_OK), false, ret);
    if (ret == OPRT_OK) {
        ret = tkl_timer_start((TUYA_TIMER_NUM_E)port, TIMER_TEST_PERIOD_US);
        test_timer_stat_add(&st, "TC024 start(period,args=NULL)", (ret == OPRT_OK), false, ret);
        if (ret == OPRT_OK) {
            ret = test_timer_wait_hit(&s_timer_noarg_hit_cnt, TIMER_TEST_PERIOD_EXPECT_HIT, TIMER_TEST_WAIT_TIMEOUT_MS);
            test_timer_stat_add(&st, "TC025 wait_cb(period,args=NULL)", (ret == OPRT_OK), false, ret);
            ret = tkl_timer_stop((TUYA_TIMER_NUM_E)port);
            test_timer_stat_add(&st, "TC026 stop(period,args=NULL)", (ret == OPRT_OK), false, ret);
        }
        ret = tkl_timer_deinit((TUYA_TIMER_NUM_E)port);
        test_timer_stat_add(&st, "TC027 deinit(period,args=NULL)", (ret == OPRT_OK), false, ret);
    }

    ret = tkl_timer_init((TUYA_TIMER_NUM_E)port, &cfg);
    test_timer_stat_add(&st, "TC028 init(max_us_check)", (ret == OPRT_OK), false, ret);
    if (ret == OPRT_OK) {
        ret = tkl_timer_start((TUYA_TIMER_NUM_E)port, TIMER_TEST_MAX_PERIOD_US);
        test_timer_stat_add(&st, "TC029 start(us=max)", (ret == OPRT_OK), false, ret);
        if (ret == OPRT_OK) {
            ret = tkl_timer_stop((TUYA_TIMER_NUM_E)port);
            test_timer_stat_add(&st, "TC030 stop(us=max)", (ret == OPRT_OK), false, ret);
        }

        ret = tkl_timer_start((TUYA_TIMER_NUM_E)port, TIMER_TEST_OVER_MAX_PERIOD_US);
        test_timer_stat_add(&st, "TC031 start(us>max)", (ret != OPRT_OK), false, ret);

        ret = tkl_timer_deinit((TUYA_TIMER_NUM_E)port);
        test_timer_stat_add(&st, "TC032 deinit(max_us_check)", (ret == OPRT_OK), false, ret);
    }

    test_timer_stat_summary(&st, "param");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET test_timer_api(uint32_t port)
{
    timer_stat_t st = {0};
    TUYA_TIMER_BASE_CFG_T cfg;
    UINT_T us_cfg = 0U;
    UINT_T us_cur = 0U;
    uint32_t hit_before = 0U;
    uint32_t hit_after = 0U;
    OPERATE_RET ret;

    if (!test_timer_port_in_range(port)) {
        return OPRT_INVALID_PARM;
    }

    (void)test_timer_cleanup(port);
    s_timer_cb_ctx[port].hit_cnt = 0U;

    cfg.mode = TUYA_TIMER_MODE_ONCE;
    cfg.cb = test_timer_cb;
    cfg.args = (VOID *)&s_timer_cb_ctx[port];

    ret = tkl_timer_init((TUYA_TIMER_NUM_E)port, &cfg);
    test_timer_stat_add(&st, "TC101 init", (ret == OPRT_OK), false, ret);
    if (ret != OPRT_OK) {
        test_timer_stat_summary(&st, "api");
        return OPRT_COM_ERROR;
    }

    ret = tkl_timer_init((TUYA_TIMER_NUM_E)port, &cfg);
    test_timer_stat_add(&st, "TC102 init(repeat)", (ret == OPRT_OK), false, ret);

    ret = tkl_timer_start((TUYA_TIMER_NUM_E)port, TIMER_TEST_ONCE_US);
    test_timer_stat_add(&st, "TC103 start(once)", (ret == OPRT_OK), false, ret);

    ret = tkl_timer_get((TUYA_TIMER_NUM_E)port, &us_cfg);
    test_timer_stat_add(&st, "TC104 get(period)", (ret == OPRT_OK && us_cfg >= TIMER_TEST_ONCE_US), false, ret);

    ret = test_timer_wait_hit(&s_timer_cb_ctx[port].hit_cnt, 1U, TIMER_TEST_WAIT_TIMEOUT_MS);
    test_timer_stat_add(&st, "TC105 wait cb(once)", (ret == OPRT_OK), false, ret);

    hit_before = s_timer_cb_ctx[port].hit_cnt;
    tkl_system_sleep(TIMER_TEST_STOP_SETTLE_MS);
    hit_after = s_timer_cb_ctx[port].hit_cnt;
    test_timer_stat_add(&st, "TC106 once no more cb", (hit_after == hit_before), false, OPRT_OK);

    ret = tkl_timer_get_current_value((TUYA_TIMER_NUM_E)port, &us_cur);
    test_timer_stat_add(&st, "TC107 get_current", (ret == OPRT_OK), false, ret);

    ret = tkl_timer_stop((TUYA_TIMER_NUM_E)port);
    test_timer_stat_add(&st, "TC108 stop(after once)", (ret == OPRT_OK), false, ret);

    ret = tkl_timer_deinit((TUYA_TIMER_NUM_E)port);
    test_timer_stat_add(&st, "TC109 deinit", (ret == OPRT_OK), false, ret);

    ret = tkl_timer_deinit((TUYA_TIMER_NUM_E)port);
    test_timer_stat_add(&st, "TC110 deinit(repeat)", (ret == OPRT_OK), false, ret);

    test_timer_stat_summary(&st, "once_api");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET test_timer_multi(uint32_t base_port, uint32_t count)
{
    timer_stat_t st = {0};
    TUYA_TIMER_BASE_CFG_T cfg;
    UINT_T us_cfg = 0U;
    uint32_t i;
    uint32_t hit_before[TUYA_TIMER_NUM_MAX] = {0};
    uint32_t hit_after[TUYA_TIMER_NUM_MAX] = {0};
    OPERATE_RET ret = OPRT_OK;
    bool inited[TUYA_TIMER_NUM_MAX] = {false};

    if (!test_timer_port_in_range(base_port) || count == 0U) {
        return OPRT_INVALID_PARM;
    }
    if ((base_port + count) > (uint32_t)TUYA_TIMER_NUM_MAX) {
        return OPRT_INVALID_PARM;
    }

    cfg.mode = TUYA_TIMER_MODE_PERIOD;
    cfg.cb = NULL;
    cfg.args = NULL;

    for (i = 0U; i < count; i++) {
        uint32_t port = base_port + i;
        (void)test_timer_cleanup(port);
        s_timer_cb_ctx[port].hit_cnt = 0U;

        cfg.args = (VOID *)&s_timer_cb_ctx[port];
        cfg.cb = test_timer_cb;

        ret = tkl_timer_init((TUYA_TIMER_NUM_E)port, &cfg);
        test_timer_stat_add(&st, "TC201 init(period)", (ret == OPRT_OK), false, ret);
        if (ret != OPRT_OK) {
            goto __EXIT;
        }
        inited[port] = true;
    }

    for (i = 0U; i < count; i++) {
        uint32_t port = base_port + i;
        uint32_t period_us = TIMER_TEST_PERIOD_US + (i * 10000U);
        ret = tkl_timer_start((TUYA_TIMER_NUM_E)port, period_us);
        test_timer_stat_add(&st, "TC202 start(period)", (ret == OPRT_OK), false, ret);
        if (ret != OPRT_OK) {
            goto __EXIT;
        }

        ret = tkl_timer_get((TUYA_TIMER_NUM_E)port, &us_cfg);
        test_timer_stat_add(&st, "TC203 get(period)", (ret == OPRT_OK && us_cfg >= period_us), false, ret);
        if (ret != OPRT_OK) {
            goto __EXIT;
        }
    }

    for (i = 0U; i < count; i++) {
        uint32_t port = base_port + i;
        ret = test_timer_wait_hit(&s_timer_cb_ctx[port].hit_cnt, TIMER_TEST_PERIOD_EXPECT_HIT, TIMER_TEST_WAIT_TIMEOUT_MS);
        test_timer_stat_add(&st, "TC204 wait >=2 cb(period)", (ret == OPRT_OK), false, ret);
        if (ret != OPRT_OK) {
            goto __EXIT;
        }
    }

    tkl_system_sleep(TIMER_TEST_STOP_SETTLE_MS);

    for (i = 0U; i < count; i++) {
        uint32_t port = base_port + i;
        hit_before[port] = s_timer_cb_ctx[port].hit_cnt;
        ret = tkl_timer_stop((TUYA_TIMER_NUM_E)port);
        test_timer_stat_add(&st, "TC205 stop(period)", (ret == OPRT_OK), false, ret);
    }

    tkl_system_sleep(TIMER_TEST_STOP_SETTLE_MS);

    for (i = 0U; i < count; i++) {
        uint32_t port = base_port + i;
        hit_after[port] = s_timer_cb_ctx[port].hit_cnt;
        test_timer_stat_add(&st, "TC206 no continuous cb after stop",
                            (hit_after[port] <= (hit_before[port] + 1U)), false, OPRT_OK);
    }

__EXIT:
    for (i = 0U; i < count; i++) {
        uint32_t port = base_port + i;
        if (inited[port]) {
            ret = tkl_timer_deinit((TUYA_TIMER_NUM_E)port);
            test_timer_stat_add(&st, "TC207 deinit(period)", (ret == OPRT_OK), false, ret);
        }
    }

    test_timer_stat_summary(&st, "period_multi");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET test_timer_full(void)
{
    OPERATE_RET ret;

    /* one-shot suite */
    ret = test_timer_param((uint32_t)TIMER_TEST_DEFAULT_PORT);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = test_timer_api((uint32_t)TIMER_TEST_DEFAULT_PORT);
    if (ret != OPRT_OK) {
        return ret;
    }

    /* periodic suite (multi timer) */
    return test_timer_multi((uint32_t)TIMER_TEST_MULTI_BASE_PORT, TIMER_TEST_MULTI_COUNT);
}

void cli_tkl_timer_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    const char *cmd;
    OPERATE_RET ret;
    uint32_t port = (uint32_t)TIMER_TEST_DEFAULT_PORT;
    uint32_t base_port = (uint32_t)TIMER_TEST_MULTI_BASE_PORT;
    uint32_t count = TIMER_TEST_MULTI_COUNT;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        test_timer_usage(pcWriteBuffer, xWriteBufferLen);
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "list") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xtimer list logical_port=[0..%u] suites=\"api,param,multi,full\"",
                 (unsigned int)((uint32_t)TUYA_TIMER_NUM_MAX - 1U));
        return;
    }

    if (os_strcmp(cmd, "api") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_timer_api(port);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xtimer api %s (%d), port=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port);
        return;
    }

    if (os_strcmp(cmd, "param") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_timer_param(port);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xtimer param %s (%d), port=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port);
        return;
    }

    if (os_strcmp(cmd, "multi") == 0) {
        if (argc >= 3) {
            base_port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            count = os_strtoul(argv[3], NULL, 10);
        }
        ret = test_timer_multi(base_port, count);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xtimer multi %s (%d), base=%u count=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)base_port,
                 (unsigned int)count);
        return;
    }

    if ((os_strcmp(cmd, "full") == 0) || (os_strcmp(cmd, "auto") == 0)) {
        ret = test_timer_full();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xtimer full %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
        return;
    }

    test_timer_usage(pcWriteBuffer, xWriteBufferLen);
}
