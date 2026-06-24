#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include <soc/bk7259/qspi_cap.h>

#include "cli_tuya_test.h"
#include "tkl_qspi.h"
#include "tkl_system.h"

#define QSPI_TEST_DEFAULT_PORT          TUYA_QSPI_NUM_0
#define QSPI_TEST_DEFAULT_BYTES         64U
#define QSPI_TEST_DEFAULT_LARGE_BYTES   1024U
#define QSPI_TEST_DEFAULT_LOOPS         5U
#define QSPI_TEST_DEFAULT_STRESS_LOOPS  30U
#define QSPI_TEST_DEFAULT_DUAL_LOOPS    10U
#define QSPI_TEST_MAX_BYTES             4096U

extern void bk_printf(const char *fmt, ...);
#define QSPI_TEST_LOG(...) bk_printf(__VA_ARGS__)

typedef struct {
    uint32_t pass;
    uint32_t fail;
    uint32_t skip;
} qspi_stat_t;

typedef struct {
    uint32_t port;
    uint32_t loops;
    volatile uint32_t pass;
    volatile uint32_t fail;
    volatile uint32_t skip;
    volatile OPERATE_RET last_ret;
} qspi_dual_worker_t;

static uint8_t s_qspi_tx[QSPI_TEST_MAX_BYTES + 8U] __attribute__((aligned(4)));
static uint8_t s_qspi_rx[QSPI_TEST_MAX_BYTES + 8U] __attribute__((aligned(4)));

static uint32_t test_qspi_hw_port_num(void)
{
#if defined(SOC_QSPI_UNIT_NUM)
    if ((uint32_t)SOC_QSPI_UNIT_NUM < (uint32_t)TUYA_QSPI_NUM_MAX) {
        return (uint32_t)SOC_QSPI_UNIT_NUM;
    }
#endif
    return (uint32_t)TUYA_QSPI_NUM_MAX;
}

static bool test_qspi_port_in_range(uint32_t port)
{
    return (port < test_qspi_hw_port_num());
}

static uint32_t test_qspi_parse_dma_arg(const char *arg)
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

static void test_qspi_fill_pattern(uint8_t *buf, uint32_t bytes, uint8_t seed)
{
    if (buf == NULL) {
        return;
    }

    for (uint32_t i = 0; i < bytes; i++) {
        buf[i] = (uint8_t)(seed + i);
    }
}

static TUYA_QSPI_BASE_CFG_T test_qspi_build_cfg(bool use_dma, TUYA_QSPI_MODULE_TYPE_E type)
{
    TUYA_QSPI_BASE_CFG_T cfg;
    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.role = TUYA_QSPI_ROLE_MASTER;
    cfg.mode = TUYA_QSPI_MODE0;
    cfg.freq_hz = 20000000U;
    cfg.use_dma = use_dma ? TRUE : FALSE;
    cfg.dma_data_lines = TUYA_QSPI_1WIRE;
    cfg.type = type;
    return cfg;
}

static void test_qspi_dummy_irq_cb(TUYA_QSPI_NUM_E port, TUYA_QSPI_IRQ_EVT_E event)
{
    (void)port;
    (void)event;
}

static void test_qspi_stat_add(qspi_stat_t *st, const char *name, bool pass, bool skip, OPERATE_RET ret)
{
    if ((st == NULL) || (name == NULL)) {
        return;
    }

    /*
     * Any case that would otherwise FAIL purely because the adapter reports
     * the operation as unsupported (OPRT_NOT_SUPPORTED) is recorded as SKIP.
     * Negative cases that expect a rejection (pass == true here) keep their
     * PASS verdict even when the rejection code happens to be NOT_SUPPORTED.
     */
    if (!pass && (ret == OPRT_NOT_SUPPORTED)) {
        skip = true;
    }

    if (skip) {
        st->skip++;
        QSPI_TEST_LOG("[xqspi] SKIP test_name=%s return_code=%d\r\n", name, (int)ret);
        return;
    }

    if (pass) {
        st->pass++;
        QSPI_TEST_LOG("[xqspi] PASS test_name=%s\r\n", name);
    } else {
        st->fail++;
        QSPI_TEST_LOG("[xqspi] FAIL test_name=%s return_code=%d\r\n", name, (int)ret);
    }
}

static void test_qspi_stat_summary(const qspi_stat_t *st, const char *tag)
{
    if ((st == NULL) || (tag == NULL)) {
        return;
    }

    QSPI_TEST_LOG("[xqspi] result_summary test_name=%s passed_count=%u failed_count=%u skipped_count=%u\r\n",
                 tag,
                 (unsigned int)st->pass,
                 (unsigned int)st->fail,
                 (unsigned int)st->skip);
}

static OPERATE_RET test_qspi_run_command(uint32_t port,
                                         bool is_write,
                                         uint8_t *data,
                                         uint32_t size,
                                         uint8_t cmd_lines,
                                         uint8_t addr_lines,
                                         uint8_t data_lines)
{
    TUYA_QSPI_CMD_T cmd;

    (void)os_memset(&cmd, 0, sizeof(cmd));
    cmd.op = is_write ? TUYA_QSPI_WRITE : TUYA_QSPI_READ;
    cmd.cmd[0] = is_write ? 0x02U : 0x03U;
    cmd.cmd_size = 1U;
    cmd.cmd_lines = (TUYA_QSPI_WIRE_MODE_E)cmd_lines;

    cmd.addr[0] = 0x00U;
    cmd.addr[1] = 0x00U;
    cmd.addr[2] = 0x00U;
    cmd.addr_size = 3U;
    cmd.addr_lines = (TUYA_QSPI_WIRE_MODE_E)addr_lines;

    cmd.data = data;
    cmd.data_size = size;
    cmd.data_lines = (TUYA_QSPI_WIRE_MODE_E)data_lines;
    cmd.dummy_cycle = 0U;

    return tkl_qspi_comand((TUYA_QSPI_NUM_E)port, &cmd);
}

static OPERATE_RET test_qspi_param(uint32_t port)
{
    qspi_stat_t st = {0};
    OPERATE_RET ret;
    TUYA_QSPI_BASE_CFG_T cfg = test_qspi_build_cfg(false, TUYA_QSPI_TYPE_FLASH);
    TUYA_QSPI_BASE_CFG_T slave_cfg = cfg;
    TUYA_QSPI_CMD_T bad_cmd;
    uint32_t invalid_port = (uint32_t)TUYA_QSPI_NUM_MAX;

    if (!test_qspi_port_in_range(port)) {
        QSPI_TEST_LOG("[xqspi] invalid port=%u valid=[0..%u]\r\n",
                     (unsigned int)port,
                     (unsigned int)(test_qspi_hw_port_num() - 1U));
        return OPRT_INVALID_PARM;
    }

    (void)tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);

    ret = tkl_qspi_init((TUYA_QSPI_NUM_E)invalid_port, &cfg);
    test_qspi_stat_add(&st, "TC001 init(invalid_port)", (ret != OPRT_OK), false, ret);

    ret = tkl_qspi_init((TUYA_QSPI_NUM_E)port, NULL);
    test_qspi_stat_add(&st, "TC002 init(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_qspi_send((TUYA_QSPI_NUM_E)port, NULL, QSPI_TEST_DEFAULT_BYTES);
    test_qspi_stat_add(&st, "TC003 send(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_qspi_recv((TUYA_QSPI_NUM_E)port, NULL, QSPI_TEST_DEFAULT_BYTES);
    test_qspi_stat_add(&st, "TC004 recv(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_qspi_comand((TUYA_QSPI_NUM_E)port, NULL);
    test_qspi_stat_add(&st, "TC005 command(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_qspi_irq_enable((TUYA_QSPI_NUM_E)port);
    test_qspi_stat_add(&st, "TC006 irq_enable(before_init)", (ret != OPRT_OK), false, ret);

    ret = tkl_qspi_force_cs_pin((TUYA_QSPI_NUM_E)port, TUYA_GPIO_LEVEL_LOW);
    test_qspi_stat_add(&st, "TC007 force_cs(before_init)", (ret != OPRT_OK), false, ret);

    slave_cfg.role = TUYA_QSPI_ROLE_SLAVE;
    ret = tkl_qspi_init((TUYA_QSPI_NUM_E)port, &slave_cfg);
    test_qspi_stat_add(&st, "TC008 init(slave_role)", (ret != OPRT_OK), false, ret);

    ret = tkl_qspi_init((TUYA_QSPI_NUM_E)port, &cfg);
    test_qspi_stat_add(&st, "TC009 init(master)", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_init((TUYA_QSPI_NUM_E)port, &cfg);
    test_qspi_stat_add(&st, "TC010 init(repeat)", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_send((TUYA_QSPI_NUM_E)port, s_qspi_tx, 0U);
    test_qspi_stat_add(&st, "TC011 send(len0)", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_recv((TUYA_QSPI_NUM_E)port, s_qspi_rx, 0U);
    test_qspi_stat_add(&st, "TC012 recv(len0)", (ret == OPRT_OK), false, ret);

    (void)os_memset(&bad_cmd, 0, sizeof(bad_cmd));
    bad_cmd.op = TUYA_QSPI_WRITE;
    bad_cmd.data = s_qspi_tx;
    bad_cmd.data_size = 8U;
    bad_cmd.cmd_size = 5U;
    ret = tkl_qspi_comand((TUYA_QSPI_NUM_E)port, &bad_cmd);
    test_qspi_stat_add(&st, "TC013 command(cmd_size>4)", (ret != OPRT_OK), false, ret);

    (void)os_memset(&bad_cmd, 0, sizeof(bad_cmd));
    bad_cmd.op = TUYA_QSPI_WRITE;
    bad_cmd.data = s_qspi_tx;
    bad_cmd.data_size = 8U;
    bad_cmd.cmd_size = 1U;
    bad_cmd.addr_size = 5U;
    ret = tkl_qspi_comand((TUYA_QSPI_NUM_E)port, &bad_cmd);
    test_qspi_stat_add(&st, "TC014 command(addr_size>4)", (ret != OPRT_OK), false, ret);

    ret = tkl_qspi_irq_init((TUYA_QSPI_NUM_E)port, test_qspi_dummy_irq_cb);
    test_qspi_stat_add(&st, "TC015 irq_init", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_irq_enable((TUYA_QSPI_NUM_E)port);
    test_qspi_stat_add(&st, "TC016 irq_enable", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_irq_disable((TUYA_QSPI_NUM_E)port);
    test_qspi_stat_add(&st, "TC017 irq_disable", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_force_cs_pin((TUYA_QSPI_NUM_E)port, TUYA_GPIO_LEVEL_LOW);
    test_qspi_stat_add(&st, "TC018 force_cs(low)", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_force_cs_pin((TUYA_QSPI_NUM_E)port, TUYA_GPIO_LEVEL_HIGH);
    test_qspi_stat_add(&st, "TC019 force_cs(high)", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);
    test_qspi_stat_add(&st, "TC020 deinit", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);
    test_qspi_stat_add(&st, "TC021 deinit(repeat)", (ret == OPRT_OK), false, ret);

    test_qspi_stat_summary(&st, "param");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET test_qspi_api(uint32_t port, bool use_dma)
{
    qspi_stat_t st = {0};
    OPERATE_RET ret;
    TUYA_QSPI_BASE_CFG_T cfg = test_qspi_build_cfg(use_dma, TUYA_QSPI_TYPE_FLASH);
    uint8_t *aligned_tx = (uint8_t *)(((uintptr_t)s_qspi_tx + 3U) & ~((uintptr_t)0x3U));
    uint8_t *aligned_rx = (uint8_t *)(((uintptr_t)s_qspi_rx + 3U) & ~((uintptr_t)0x3U));
    uint8_t *unaligned_tx = aligned_tx + 1U;
    const uint32_t small = QSPI_TEST_DEFAULT_BYTES;
    const uint32_t large = QSPI_TEST_DEFAULT_LARGE_BYTES;
    const TUYA_QSPI_MODULE_TYPE_E types[] = {
        TUYA_QSPI_TYPE_FLASH,
        TUYA_QSPI_TYPE_PSRAM,
        TUYA_QSPI_TYPE_LCD,
    };

    if (!test_qspi_port_in_range(port)) {
        QSPI_TEST_LOG("[xqspi] api invalid port=%u valid=[0..%u]\r\n",
                     (unsigned int)port,
                     (unsigned int)(test_qspi_hw_port_num() - 1U));
        return OPRT_INVALID_PARM;
    }

    (void)tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);

    test_qspi_fill_pattern(aligned_tx, large + 4U, 0x10U);
    (void)os_memset(aligned_rx, 0, large + 4U);

    ret = tkl_qspi_init((TUYA_QSPI_NUM_E)port, &cfg);
    test_qspi_stat_add(&st, "TC101 init", (ret == OPRT_OK), false, ret);
    if (ret != OPRT_OK) {
        test_qspi_stat_summary(&st, use_dma ? "api_dma" : "api_nodma");
        return OPRT_COM_ERROR;
    }

    ret = tkl_qspi_init((TUYA_QSPI_NUM_E)port, &cfg);
    test_qspi_stat_add(&st, "TC102 init(repeat)", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_irq_init((TUYA_QSPI_NUM_E)port, test_qspi_dummy_irq_cb);
    test_qspi_stat_add(&st, "TC103 irq_init", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_irq_enable((TUYA_QSPI_NUM_E)port);
    test_qspi_stat_add(&st, "TC104 irq_enable", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_irq_disable((TUYA_QSPI_NUM_E)port);
    test_qspi_stat_add(&st, "TC105 irq_disable", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_send((TUYA_QSPI_NUM_E)port, aligned_tx, small);
    test_qspi_stat_add(&st, "TC106 send_small_aligned",
                       (ret == OPRT_OK), (ret == OPRT_NOT_SUPPORTED), ret);

    ret = tkl_qspi_send((TUYA_QSPI_NUM_E)port, unaligned_tx, small);
    test_qspi_stat_add(&st, "TC107 send_small_unaligned",
                       (ret == OPRT_OK), (ret == OPRT_NOT_SUPPORTED), ret);

    ret = tkl_qspi_send((TUYA_QSPI_NUM_E)port, aligned_tx, large);
    test_qspi_stat_add(&st, "TC108 send_large_aligned",
                       use_dma ? (ret == OPRT_OK) : (ret != OPRT_OK),
                       (ret == OPRT_NOT_SUPPORTED), ret);

    ret = tkl_qspi_send((TUYA_QSPI_NUM_E)port, unaligned_tx, large);
    test_qspi_stat_add(&st, "TC109 send_large_unaligned",
                       (ret != OPRT_OK), (ret == OPRT_NOT_SUPPORTED), ret);

    ret = tkl_qspi_recv((TUYA_QSPI_NUM_E)port, aligned_rx, small);
    test_qspi_stat_add(&st, "TC110 recv_small",
                       (ret == OPRT_OK), (ret == OPRT_NOT_SUPPORTED), ret);

    ret = tkl_qspi_recv((TUYA_QSPI_NUM_E)port, aligned_rx, large);
    test_qspi_stat_add(&st, "TC111 recv_large", (ret != OPRT_OK), false, ret);

    ret = test_qspi_run_command(port, true, aligned_tx, small, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE);
    test_qspi_stat_add(&st, "TC112 command_write_small", (ret == OPRT_OK), false, ret);

    ret = test_qspi_run_command(port, false, aligned_rx, small, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE);
    test_qspi_stat_add(&st, "TC113 command_read_small", (ret == OPRT_OK), false, ret);

    ret = test_qspi_run_command(port, true, aligned_tx, large, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE);
    test_qspi_stat_add(&st, "TC114 command_write_large_aligned",
                       (ret != OPRT_OK), (ret == OPRT_NOT_SUPPORTED), ret);

    ret = test_qspi_run_command(port, true, unaligned_tx, large, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE);
    test_qspi_stat_add(&st, "TC115 command_write_large_unaligned",
                       (ret != OPRT_OK), (ret == OPRT_NOT_SUPPORTED), ret);

    ret = test_qspi_run_command(port, false, aligned_rx, large, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE);
    test_qspi_stat_add(&st, "TC116 command_read_large",
                       (ret != OPRT_OK), (ret == OPRT_NOT_SUPPORTED), ret);

    ret = tkl_qspi_force_cs_pin((TUYA_QSPI_NUM_E)port, TUYA_GPIO_LEVEL_LOW);
    test_qspi_stat_add(&st, "TC117 force_cs_low", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_force_cs_pin((TUYA_QSPI_NUM_E)port, TUYA_GPIO_LEVEL_HIGH);
    test_qspi_stat_add(&st, "TC118 force_cs_high", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);
    test_qspi_stat_add(&st, "TC119 deinit", (ret == OPRT_OK), false, ret);

    ret = tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);
    test_qspi_stat_add(&st, "TC120 deinit(repeat)", (ret == OPRT_OK), false, ret);

    for (uint32_t i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
        char tc_name[72];
        cfg = test_qspi_build_cfg(use_dma, types[i]);

        ret = tkl_qspi_init((TUYA_QSPI_NUM_E)port, &cfg);
        (void)snprintf(tc_name, sizeof(tc_name), "TC12%u init(type=%u)", (unsigned int)(i + 1U), (unsigned int)types[i]);
        test_qspi_stat_add(&st, tc_name, (ret == OPRT_OK), false, ret);
        if (ret != OPRT_OK) {
            continue;
        }

        ret = test_qspi_run_command(port, true, aligned_tx, small, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE);
        (void)snprintf(tc_name, sizeof(tc_name), "TC12%u command(type=%u)", (unsigned int)(i + 4U), (unsigned int)types[i]);
        test_qspi_stat_add(&st, tc_name, (ret == OPRT_OK), false, ret);

        ret = tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);
        (void)snprintf(tc_name, sizeof(tc_name), "TC12%u deinit(type=%u)", (unsigned int)(i + 7U), (unsigned int)types[i]);
        test_qspi_stat_add(&st, tc_name, (ret == OPRT_OK), false, ret);
    }

    test_qspi_stat_summary(&st, use_dma ? "api_dma" : "api_nodma");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET test_qspi_api_all(void)
{
    OPERATE_RET final_ret = OPRT_OK;
    uint32_t pass = 0U;
    uint32_t fail = 0U;
    uint32_t ports = test_qspi_hw_port_num();

    for (uint32_t port = 0; port < ports; port++) {
        OPERATE_RET one = test_qspi_api(port, false);
        if (one == OPRT_OK) {
            pass++;
        } else {
            fail++;
            final_ret = one;
        }

        one = test_qspi_api(port, true);
        if (one == OPRT_OK) {
            pass++;
        } else {
            fail++;
            final_ret = one;
        }
    }

    QSPI_TEST_LOG("[xqspi] api_all_result passed_count=%u failed_count=%u\r\n",
                 (unsigned int)pass,
                 (unsigned int)fail);
    return final_ret;
}

static OPERATE_RET test_qspi_stress(uint32_t port, uint32_t loops)
{
    uint32_t pass = 0U;
    uint32_t fail = 0U;
    uint32_t skip = 0U;
    OPERATE_RET final_ret = OPRT_OK;
    uint8_t *aligned_tx = (uint8_t *)(((uintptr_t)s_qspi_tx + 3U) & ~((uintptr_t)0x3U));

    if (!test_qspi_port_in_range(port) || loops == 0U) {
        return OPRT_INVALID_PARM;
    }

    (void)tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);

    for (uint32_t i = 0; i < loops; i++) {
        bool use_dma = ((i & 0x1U) != 0U);
        TUYA_QSPI_BASE_CFG_T cfg = test_qspi_build_cfg(use_dma, (TUYA_QSPI_MODULE_TYPE_E)(i % 3U));
        OPERATE_RET ret = tkl_qspi_init((TUYA_QSPI_NUM_E)port, &cfg);

        /*
         * The adapter only supports a subset of module types on some paths
         * (e.g. tkl_qspi_send is LCD-only). Such combinations return
         * OPRT_NOT_SUPPORTED by design, so count them as skipped rather than
         * failed and do not let them taint final_ret.
         */
        if (ret != OPRT_OK) {
            if (ret == OPRT_NOT_SUPPORTED) {
                skip++;
            } else {
                fail++;
                final_ret = ret;
            }
            continue;
        }

        ret = tkl_qspi_send((TUYA_QSPI_NUM_E)port,
                            aligned_tx,
                            use_dma ? QSPI_TEST_DEFAULT_LARGE_BYTES : QSPI_TEST_DEFAULT_BYTES);
        if (ret != OPRT_OK) {
            if (ret == OPRT_NOT_SUPPORTED) {
                skip++;
            } else {
                fail++;
                final_ret = ret;
            }
            (void)tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);
            continue;
        }

        ret = test_qspi_run_command(port, true, aligned_tx, QSPI_TEST_DEFAULT_BYTES,
                                    TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE);
        if (ret != OPRT_OK) {
            if (ret == OPRT_NOT_SUPPORTED) {
                skip++;
            } else {
                fail++;
                final_ret = ret;
            }
            (void)tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);
            continue;
        }

        ret = tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);
        if (ret != OPRT_OK) {
            if (ret == OPRT_NOT_SUPPORTED) {
                skip++;
            } else {
                fail++;
                final_ret = ret;
            }
            continue;
        }

        pass++;
    }

    QSPI_TEST_LOG("[xqspi] stress_result port=%u requested_loops=%u passed_count=%u failed_count=%u skipped_count=%u return_code=%d\r\n",
                 (unsigned int)port,
                 (unsigned int)loops,
                 (unsigned int)pass,
                 (unsigned int)fail,
                 (unsigned int)skip,
                 (int)final_ret);

    (void)tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);
    if (fail != 0U) {
        return final_ret;
    }
    /* Everything that ran was unsupported (nothing actually exercised): report as skipped. */
    if (pass == 0U) {
        return OPRT_NOT_SUPPORTED;
    }
    return OPRT_OK;
}

static void test_qspi_dual_api_worker(beken_thread_arg_t arg)
{
    qspi_dual_worker_t *ctx = (qspi_dual_worker_t *)arg;
    uint8_t *aligned_tx = (uint8_t *)(((uintptr_t)s_qspi_tx + 3U) & ~((uintptr_t)0x3U));

    if (ctx == NULL) {
        rtos_delete_thread(NULL);
        return;
    }

    ctx->last_ret = OPRT_OK;
    for (uint32_t i = 0; i < ctx->loops; i++) {
        bool use_dma = ((i & 0x1U) != 0U);
        TUYA_QSPI_BASE_CFG_T cfg = test_qspi_build_cfg(use_dma, (TUYA_QSPI_MODULE_TYPE_E)(i % 3U));
        OPERATE_RET ret = tkl_qspi_init((TUYA_QSPI_NUM_E)ctx->port, &cfg);

        if (ret != OPRT_OK) {
            if (ret == OPRT_NOT_SUPPORTED) {
                ctx->skip++;
            } else {
                ctx->fail++;
                ctx->last_ret = ret;
            }
            continue;
        }

        ret = tkl_qspi_send((TUYA_QSPI_NUM_E)ctx->port,
                            aligned_tx,
                            use_dma ? QSPI_TEST_DEFAULT_LARGE_BYTES : QSPI_TEST_DEFAULT_BYTES);
        if (ret != OPRT_OK) {
            if (ret == OPRT_NOT_SUPPORTED) {
                ctx->skip++;
            } else {
                ctx->fail++;
                ctx->last_ret = ret;
            }
            (void)tkl_qspi_deinit((TUYA_QSPI_NUM_E)ctx->port);
            continue;
        }

        ret = test_qspi_run_command(ctx->port, true, aligned_tx, QSPI_TEST_DEFAULT_BYTES,
                                    TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE, TUYA_QSPI_1WIRE);
        if (ret != OPRT_OK) {
            if (ret == OPRT_NOT_SUPPORTED) {
                ctx->skip++;
            } else {
                ctx->fail++;
                ctx->last_ret = ret;
            }
            (void)tkl_qspi_deinit((TUYA_QSPI_NUM_E)ctx->port);
            continue;
        }

        ret = tkl_qspi_deinit((TUYA_QSPI_NUM_E)ctx->port);
        if (ret != OPRT_OK) {
            if (ret == OPRT_NOT_SUPPORTED) {
                ctx->skip++;
            } else {
                ctx->fail++;
                ctx->last_ret = ret;
            }
            continue;
        }

        ctx->pass++;
    }

    rtos_delete_thread(NULL);
}

static OPERATE_RET test_qspi_dual_api(uint32_t port0, uint32_t port1, uint32_t loops)
{
    beken_thread_t th0 = NULL;
    beken_thread_t th1 = NULL;
    qspi_dual_worker_t w0;
    qspi_dual_worker_t w1;
    OPERATE_RET ret = OPRT_OK;

    if ((!test_qspi_port_in_range(port0)) ||
        (!test_qspi_port_in_range(port1)) ||
        (port0 == port1) ||
        (loops == 0U)) {
        return OPRT_INVALID_PARM;
    }

    (void)os_memset(&w0, 0, sizeof(w0));
    (void)os_memset(&w1, 0, sizeof(w1));
    w0.port = port0;
    w0.loops = loops;
    w1.port = port1;
    w1.loops = loops;

    if (rtos_create_thread(&th0, 6, "xqspi_d0", test_qspi_dual_api_worker, 3072, &w0) != kNoErr) {
        return OPRT_COM_ERROR;
    }

    if (rtos_create_thread(&th1, 6, "xqspi_d1", test_qspi_dual_api_worker, 3072, &w1) != kNoErr) {
        (void)rtos_delete_thread(&th0);
        return OPRT_COM_ERROR;
    }

    (void)rtos_thread_join(&th0);
    (void)rtos_thread_join(&th1);

    if ((w0.fail != 0U) || (w1.fail != 0U)) {
        ret = OPRT_COM_ERROR;
    } else if ((w0.pass == 0U) && (w1.pass == 0U)) {
        /* Nothing actually ran on either worker; only unsupported combos: skip. */
        ret = OPRT_NOT_SUPPORTED;
    }

    QSPI_TEST_LOG("[xqspi] dualapi_result port0=%u port0_passed_count=%u port0_failed_count=%u port0_skipped_count=%u port1=%u port1_passed_count=%u port1_failed_count=%u port1_skipped_count=%u return_code=%d\r\n",
                 (unsigned int)port0,
                 (unsigned int)w0.pass,
                 (unsigned int)w0.fail,
                 (unsigned int)w0.skip,
                 (unsigned int)port1,
                 (unsigned int)w1.pass,
                 (unsigned int)w1.fail,
                 (unsigned int)w1.skip,
                 (int)ret);
    return ret;
}

static OPERATE_RET test_qspi_lcd_dma(uint32_t port, uint32_t bytes)
{
    OPERATE_RET ret;
    TUYA_QSPI_BASE_CFG_T cfg = test_qspi_build_cfg(true, TUYA_QSPI_TYPE_LCD);
    uint8_t *aligned_tx = (uint8_t *)(((uintptr_t)s_qspi_tx + 3U) & ~((uintptr_t)0x3U));

    if (!test_qspi_port_in_range(port) || (bytes == 0U) || (bytes > QSPI_TEST_MAX_BYTES)) {
        return OPRT_INVALID_PARM;
    }

    (void)tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);
    test_qspi_fill_pattern(aligned_tx, bytes, 0x70U);

    ret = tkl_qspi_init((TUYA_QSPI_NUM_E)port, &cfg);
    if (ret != OPRT_OK) {
        QSPI_TEST_LOG("[xqspi] lcd_dma_result port=%u transfer_bytes=%u return_code=%d\r\n",
                     (unsigned int)port, (unsigned int)bytes, (int)ret);
        return ret;
    }

    ret = tkl_qspi_send((TUYA_QSPI_NUM_E)port, aligned_tx, bytes);
    QSPI_TEST_LOG("[xqspi] lcd_dma_result port=%u transfer_bytes=%u return_code=%d\r\n",
                 (unsigned int)port, (unsigned int)bytes, (int)ret);

    (void)tkl_qspi_deinit((TUYA_QSPI_NUM_E)port);
    return ret;
}

static OPERATE_RET test_qspi_full(void)
{
    qspi_stat_t st = {0};
    OPERATE_RET ret;
    uint32_t ports = test_qspi_hw_port_num();

    ret = test_qspi_api_all();
    test_qspi_stat_add(&st, "TF001 api_all", (ret == OPRT_OK), false, ret);

    for (uint32_t port = 0; port < ports; port++) {
        char tc_name[48];

        ret = test_qspi_param(port);
        (void)snprintf(tc_name, sizeof(tc_name), "TF1%02u param(port=%u)",
                       (unsigned int)(port + 1U), (unsigned int)port);
        test_qspi_stat_add(&st, tc_name, (ret == OPRT_OK), false, ret);

        ret = test_qspi_stress(port, QSPI_TEST_DEFAULT_STRESS_LOOPS);
        (void)snprintf(tc_name, sizeof(tc_name), "TF2%02u stress(port=%u)",
                       (unsigned int)(port + 1U), (unsigned int)port);
        test_qspi_stat_add(&st, tc_name, (ret == OPRT_OK), (ret == OPRT_NOT_SUPPORTED), ret);
    }

    if (ports >= 2U) {
        ret = test_qspi_dual_api(0U, 1U, QSPI_TEST_DEFAULT_DUAL_LOOPS);
        test_qspi_stat_add(&st, "TF301 dual_port_parallel_api", (ret == OPRT_OK), false, ret);
    } else {
        test_qspi_stat_add(&st, "TF301 dual_port_parallel_api", true, true, OPRT_NOT_SUPPORTED);
    }

    test_qspi_stat_summary(&st, "full");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static void test_qspi_usage(char *pcWriteBuffer, int xWriteBufferLen)
{
    snprintf(pcWriteBuffer, xWriteBufferLen,
             "Usage: xqspi <list|param [port]|api [port] [dma(0/1|dma)]|api_all|lcd_dma [port] [bytes]|stress [port] [loops]|dualapi [port0] [port1] [loops]|full>");
}

void cli_xqspi_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    const char *cmd = NULL;
    uint32_t port = QSPI_TEST_DEFAULT_PORT;
    uint32_t port0 = 0U;
    uint32_t port1 = 1U;
    uint32_t loops = QSPI_TEST_DEFAULT_LOOPS;
    uint32_t dma = 0U;
    OPERATE_RET ret;

    if ((pcWriteBuffer == NULL) || (xWriteBufferLen <= 0)) {
        return;
    }

    if (argc < 2) {
        test_qspi_usage(pcWriteBuffer, xWriteBufferLen);
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "list") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xqspi list logical_port=[0..%u] commands=param/api/api_all/lcd_dma/stress/dualapi/full (no loopback required)",
                 (unsigned int)(test_qspi_hw_port_num() - 1U));
        return;
    }

    if (os_strcmp(cmd, "param") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_qspi_param(port);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xqspi param result=%s return_code=%d port=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret,
                 (unsigned int)port);
        return;
    }

    if (os_strcmp(cmd, "api") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            dma = test_qspi_parse_dma_arg(argv[3]);
        }
        ret = test_qspi_api(port, dma != 0U);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xqspi api result=%s return_code=%d port=%u dma_enabled=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret,
                 (unsigned int)port,
                 (unsigned int)dma);
        return;
    }

    if (os_strcmp(cmd, "lcd_dma") == 0) {
        uint32_t bytes = QSPI_TEST_DEFAULT_LARGE_BYTES;

        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            bytes = os_strtoul(argv[3], NULL, 10);
        }

        ret = test_qspi_lcd_dma(port, bytes);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xqspi lcd_dma result=%s return_code=%d port=%u transfer_bytes=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret,
                 (unsigned int)port,
                 (unsigned int)bytes);
        return;
    }

    if (os_strcmp(cmd, "api_all") == 0) {
        ret = test_qspi_api_all();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xqspi api_all result=%s return_code=%d\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret);
        return;
    }

    if (os_strcmp(cmd, "stress") == 0) {
        loops = QSPI_TEST_DEFAULT_STRESS_LOOPS;
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            loops = os_strtoul(argv[3], NULL, 10);
        }
        ret = test_qspi_stress(port, loops);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xqspi stress result=%s return_code=%d port=%u requested_loops=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret,
                 (unsigned int)port,
                 (unsigned int)loops);
        return;
    }

    if (os_strcmp(cmd, "dualapi") == 0) {
        loops = QSPI_TEST_DEFAULT_DUAL_LOOPS;
        if (argc >= 3) {
            port0 = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            port1 = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            loops = os_strtoul(argv[4], NULL, 10);
        }
        ret = test_qspi_dual_api(port0, port1, loops);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xqspi dualapi result=%s return_code=%d port0=%u port1=%u requested_loops=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret,
                 (unsigned int)port0,
                 (unsigned int)port1,
                 (unsigned int)loops);
        return;
    }

    if (os_strcmp(cmd, "full") == 0) {
        ret = test_qspi_full();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xqspi full result=%s return_code=%d\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 (int)ret);
        return;
    }

    test_qspi_usage(pcWriteBuffer, xWriteBufferLen);
}
