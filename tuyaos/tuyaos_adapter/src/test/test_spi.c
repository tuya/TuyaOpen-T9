#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include <soc/soc.h>
#include <soc/bk7259/spi_cap.h>

#include "cli_tuya_test.h"
#include "tkl_spi.h"
#include "tkl_pinmux.h"
#include "tkl_system.h"

/*
 * SPI pad routing. tkl_spi maps its pads in spi_init_gpio() from configurable
 * pin getters; route SPI0 (master, GPIO14-17) and SPI1 (slave, GPIO2-5) here
 * via tkl_io_pinmux_config() before init so the pins are explicit and the
 * master/slave loopback wiring is well-defined.
 *
 * Loopback wiring (master SPI0 <-> slave SPI1):
 *   GPIO14 SCK <-> GPIO2,  GPIO15 CS <-> GPIO3,
 *   GPIO16 MOSI <-> GPIO4, GPIO17 MISO <-> GPIO5
 */
#define SPI0_CLK_PIN   14
#define SPI0_CS_PIN    15
#define SPI0_MOSI_PIN  16
#define SPI0_MISO_PIN  17
#define SPI1_CLK_PIN   2
#define SPI1_CS_PIN    3
#define SPI1_MOSI_PIN  4
#define SPI1_MISO_PIN  5

static void test_spi_pinmux_setup(void)
{
    (void)tkl_io_pinmux_config(SPI0_CLK_PIN,  TUYA_SPI0_CLK);
    (void)tkl_io_pinmux_config(SPI0_CS_PIN,   TUYA_SPI0_CS);
    (void)tkl_io_pinmux_config(SPI0_MOSI_PIN, TUYA_SPI0_MOSI);
    (void)tkl_io_pinmux_config(SPI0_MISO_PIN, TUYA_SPI0_MISO);
    (void)tkl_io_pinmux_config(SPI1_CLK_PIN,  TUYA_SPI1_CLK);
    (void)tkl_io_pinmux_config(SPI1_CS_PIN,   TUYA_SPI1_CS);
    (void)tkl_io_pinmux_config(SPI1_MOSI_PIN, TUYA_SPI1_MOSI);
    (void)tkl_io_pinmux_config(SPI1_MISO_PIN, TUYA_SPI1_MISO);
}

#define SPI_TEST_DEFAULT_MASTER_PORT      TUYA_SPI_NUM_0
#define SPI_TEST_DEFAULT_SLAVE_PORT       TUYA_SPI_NUM_1
#define SPI_TEST_DEFAULT_BYTES            32U
#define SPI_TEST_DEFAULT_LOOPS            5U
#define SPI_TEST_DEFAULT_DUAL_LOOPS       3U
#define SPI_TEST_DEFAULT_STRESS_LOOPS     50U
#define SPI_TEST_FULL_SMALL_LOOPS         2U
#define SPI_TEST_FULL_LARGE_LOOPS         1U
#define SPI_TEST_MAX_XFER_BYTES           8192U
#define SPI_TEST_WAIT_TIMEOUT_MS          10000U
#define SPI_TEST_POLL_INTERVAL_MS         2U
#define SPI_TEST_ROLE_SWITCH_DELAY_MS     5U
#define SPI_TEST_INTER_LOOP_DELAY_MS      5U
#define SPI_TEST_TEMP_DEBUG               0
#define SPI_WORKER_STAGE_NONE             0U
#define SPI_WORKER_STAGE_INIT             1U
#define SPI_WORKER_STAGE_RECV_ARM         2U
#define SPI_WORKER_STAGE_SEND_ARM         3U
#define SPI_WORKER_STAGE_DEINIT           4U
#define SPI_WORKER_STAGE_DONE             5U
#define SPI_WORKER_RET_PENDING            ((OPERATE_RET)0x7FFFFFFF)

extern void bk_printf(const char *fmt, ...);
#define SPI_TEST_LOG(...) bk_printf(__VA_ARGS__)
#if SPI_TEST_TEMP_DEBUG
#define SPI_TEST_DBG(...) bk_printf("[xspi][dbg] " __VA_ARGS__)
#else
#define SPI_TEST_DBG(...) do { } while (0)
#endif

typedef struct {
    uint32_t pass;
    uint32_t fail;
    uint32_t skip;
} spi_stat_t;

typedef struct {
    TUYA_SPI_NUM_E port;
    bool is_master;
    bool slave_send;
    bool use_dma;
    bool verify_data;
    uint32_t bytes;
    uint32_t loops;
    volatile uint32_t stop_req;
    volatile uint32_t started;
    volatile uint32_t exited;
    volatile uint32_t ready_seq;
    volatile uint32_t resp_ready_seq;
    volatile uint32_t recv_ok;
    volatile uint32_t send_ok;
    volatile uint32_t mismatch;
    volatile uint32_t stage;
    volatile uint32_t loop_idx;
    volatile OPERATE_RET last_ret;
} spi_loop_worker_t;

typedef struct {
    uint32_t port;
    uint32_t loops;
    beken_semaphore_t *start_sem;
    beken_semaphore_t *done_sem;
    volatile uint32_t pass;
    volatile uint32_t fail;
} spi_dual_worker_t;

static uint8_t s_spi_master_tx[SPI_TEST_MAX_XFER_BYTES] __attribute__((aligned(4)));
static uint8_t s_spi_master_rx[SPI_TEST_MAX_XFER_BYTES] __attribute__((aligned(4)));
static uint8_t s_spi_slave_rx[SPI_TEST_MAX_XFER_BYTES] __attribute__((aligned(4)));
static uint8_t s_spi_slave_tx[SPI_TEST_MAX_XFER_BYTES] __attribute__((aligned(4)));

typedef struct {
    volatile uint32_t transfer_done_cnt;
    volatile uint32_t tx_done_cnt;
    volatile uint32_t rx_done_cnt;
    volatile uint32_t last_event;
} spi_evt_stat_t;

static spi_evt_stat_t s_spi_evt_stat[TUYA_SPI_NUM_MAX];

static uint32_t test_spi_hw_port_num(void)
{
#if defined(SOC_SPI_UNIT_NUM)
    if ((uint32_t)SOC_SPI_UNIT_NUM < (uint32_t)TUYA_SPI_NUM_MAX) {
        return (uint32_t)SOC_SPI_UNIT_NUM;
    }
#endif
    return (uint32_t)TUYA_SPI_NUM_MAX;
}

static bool test_spi_port_in_range(uint32_t port)
{
    return (port < test_spi_hw_port_num());
}

static uint32_t test_spi_parse_dma_arg(const char *arg)
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

static bool test_spi_parse_role_arg(const char *arg, TUYA_SPI_ROLE_E *role)
{
    if ((arg == NULL) || (role == NULL)) {
        return false;
    }

    if ((os_strcmp(arg, "master") == 0) || (os_strcmp(arg, "m") == 0) || (os_strcmp(arg, "0") == 0)) {
        *role = TUYA_SPI_ROLE_MASTER;
        return true;
    }

    if ((os_strcmp(arg, "slave") == 0) || (os_strcmp(arg, "s") == 0) || (os_strcmp(arg, "1") == 0)) {
        *role = TUYA_SPI_ROLE_SLAVE;
        return true;
    }

    return false;
}

static void test_spi_irq_cb(TUYA_SPI_NUM_E port, TUYA_SPI_IRQ_EVT_E event)
{
    if ((uint32_t)port >= (uint32_t)TUYA_SPI_NUM_MAX) {
        return;
    }

    s_spi_evt_stat[port].last_event = (uint32_t)event;
    switch (event) {
        case TUYA_SPI_EVENT_TRANSFER_COMPLETE:
            s_spi_evt_stat[port].transfer_done_cnt++;
            break;
        case TUYA_SPI_EVENT_TX_COMPLETE:
            s_spi_evt_stat[port].tx_done_cnt++;
            break;
        case TUYA_SPI_EVENT_RX_COMPLETE:
            s_spi_evt_stat[port].rx_done_cnt++;
            break;
        default:
            break;
    }
}

static void test_spi_evt_reset(TUYA_SPI_NUM_E port)
{
    if ((uint32_t)port >= (uint32_t)TUYA_SPI_NUM_MAX) {
        return;
    }

    s_spi_evt_stat[port].transfer_done_cnt = 0U;
    s_spi_evt_stat[port].tx_done_cnt = 0U;
    s_spi_evt_stat[port].rx_done_cnt = 0U;
    s_spi_evt_stat[port].last_event = (uint32_t)TUYA_SPI_EVENT_TRANSFER_COMPLETE;
}

static uint32_t test_spi_evt_count_get(TUYA_SPI_NUM_E port, TUYA_SPI_IRQ_EVT_E event)
{
    if ((uint32_t)port >= (uint32_t)TUYA_SPI_NUM_MAX) {
        return 0U;
    }

    switch (event) {
        case TUYA_SPI_EVENT_TRANSFER_COMPLETE:
            return s_spi_evt_stat[port].transfer_done_cnt;
        case TUYA_SPI_EVENT_TX_COMPLETE:
            return s_spi_evt_stat[port].tx_done_cnt;
        case TUYA_SPI_EVENT_RX_COMPLETE:
            return s_spi_evt_stat[port].rx_done_cnt;
        default:
            return 0U;
    }
}

static void test_spi_dump_status(const char *tag, TUYA_SPI_NUM_E port)
{
    TUYA_SPI_STATUS_T status;
    OPERATE_RET ret;

    (void)os_memset(&status, 0, sizeof(status));
    ret = tkl_spi_get_status(port, &status);
    if (ret == OPRT_OK) {
        SPI_TEST_DBG("%s p=%u busy=%u lost=%u mf=%u\r\n",
                     tag,
                     (unsigned int)port,
                     (unsigned int)status.busy,
                     (unsigned int)status.data_lost,
                     (unsigned int)status.mode_fault);
    } else {
        SPI_TEST_DBG("%s p=%u get_status ret=%d\r\n",
                     tag,
                     (unsigned int)port,
                     (int)ret);
    }
}

static OPERATE_RET test_spi_wait_event(TUYA_SPI_NUM_E port,
                                       TUYA_SPI_IRQ_EVT_E event,
                                       uint32_t base_count,
                                       uint32_t timeout_ms,
                                       const char *tag)
{
    uint32_t waited = 0U;

    if ((uint32_t)port >= (uint32_t)TUYA_SPI_NUM_MAX) {
        return OPRT_INVALID_PARM;
    }

    while (waited < timeout_ms) {
        if (test_spi_evt_count_get(port, event) > base_count) {
            return OPRT_OK;
        }
        tkl_system_sleep(SPI_TEST_POLL_INTERVAL_MS);
        waited += SPI_TEST_POLL_INTERVAL_MS;
    }

    SPI_TEST_LOG("[xspi] wait_event timeout tag=%s port=%u event=%u waited_ms=%u tx_done=%u rx_done=%u tr_done=%u last_event=%u\r\n",
                 (tag != NULL) ? tag : "evt",
                 (unsigned int)port,
                 (unsigned int)event,
                 (unsigned int)waited,
                 (unsigned int)s_spi_evt_stat[port].tx_done_cnt,
                 (unsigned int)s_spi_evt_stat[port].rx_done_cnt,
                 (unsigned int)s_spi_evt_stat[port].transfer_done_cnt,
                 (unsigned int)s_spi_evt_stat[port].last_event);
    test_spi_dump_status("wait_event_timeout", port);
    return OPRT_TIMEOUT;
}

static OPERATE_RET test_spi_join_worker_with_timeout(const char *tag,
                                                     beken_thread_t *thread,
                                                     spi_loop_worker_t *ctx,
                                                     TUYA_SPI_NUM_E master_port,
                                                     TUYA_SPI_NUM_E slave_port)
{
    uint32_t waited = 0U;

    if ((thread == NULL) || (ctx == NULL)) {
        return OPRT_INVALID_PARM;
    }

    while ((ctx->exited == 0U) && (waited < SPI_TEST_WAIT_TIMEOUT_MS)) {
        tkl_system_sleep(SPI_TEST_POLL_INTERVAL_MS);
        waited += SPI_TEST_POLL_INTERVAL_MS;
    }

    if (ctx->exited != 0U) {
        (void)rtos_thread_join(thread);
        return OPRT_OK;
    }

    SPI_TEST_LOG("[xspi] worker_join_timeout tag=%s waited_ms=%u stage=%u loop_idx=%u started=%u exited=%u stop_req=%u ready_seq=%u resp_ready_seq=%u recv_ok=%u send_ok=%u mismatch=%u last_ret=%d\r\n",
                 (tag != NULL) ? tag : "worker",
                 (unsigned int)waited,
                 (unsigned int)ctx->stage,
                 (unsigned int)ctx->loop_idx,
                 (unsigned int)ctx->started,
                 (unsigned int)ctx->exited,
                 (unsigned int)ctx->stop_req,
                 (unsigned int)ctx->ready_seq,
                 (unsigned int)ctx->resp_ready_seq,
                 (unsigned int)ctx->recv_ok,
                 (unsigned int)ctx->send_ok,
                 (unsigned int)ctx->mismatch,
                 (int)ctx->last_ret);
    test_spi_dump_status("join_timeout_master", master_port);
    test_spi_dump_status("join_timeout_slave", slave_port);
    (void)rtos_delete_thread(thread);
    ctx->exited = 1U;
    return OPRT_TIMEOUT;
}

static void test_spi_usage(char *pcWriteBuffer, int xWriteBufferLen)
{
    snprintf(pcWriteBuffer, xWriteBufferLen,
             "Usage: xspi <list|api [port] [dma(0/1|dma)]|api_all|param [port]|stress [port] [loops]|dualapi [port0] [port1] [loops]|send [master_port] [slave_port] [bytes] [loops] [dma(0/1|dma)]|recv [master_port] [slave_port] [bytes] [loops] [dma(0/1|dma)]|io [port] [master/slave] [send/recv] [bytes] [loops] [dma(0/1|dma)] [verify(0/1)]|dualrw [master_port] [slave_port] [bytes] [loops] [dma(0/1|dma)]|loop [master_port] [slave_port] [bytes] [loops] [dma(0/1|dma)]|full(note: matrix covers dma/nodma send+recv align/non-align <4096/>4096)>");
}

static void test_spi_stat_add(spi_stat_t *st, const char *name, bool pass, bool skip, OPERATE_RET ret)
{
    if ((st == NULL) || (name == NULL)) {
        return;
    }

    if (skip) {
        st->skip++;
        SPI_TEST_LOG("[xspi] SKIP test_name=%s return_code=%d\r\n", name, ret);
        return;
    }

    if (pass) {
        st->pass++;
        SPI_TEST_LOG("[xspi] PASS test_name=%s\r\n", name);
    } else {
        st->fail++;
        SPI_TEST_LOG("[xspi] FAIL test_name=%s return_code=%d\r\n", name, ret);
    }
}

static void test_spi_stat_summary(const spi_stat_t *st, const char *tag)
{
    if ((st == NULL) || (tag == NULL)) {
        return;
    }

    SPI_TEST_LOG("[xspi] result_summary test_name=%s passed_count=%u failed_count=%u skipped_count=%u\r\n",
                 tag,
                 (unsigned int)st->pass,
                 (unsigned int)st->fail,
                 (unsigned int)st->skip);
}

static uint8_t test_spi_pattern(uint32_t loop, uint32_t idx)
{
    (void)loop;
    return (uint8_t)(idx & 0xFFU);
}

static uint8_t test_spi_resp_from_req(uint8_t req)
{
    return (uint8_t)((req + 0x80U) & 0xFFU);
}

static TUYA_SPI_BASE_CFG_T test_spi_build_cfg_ex(TUYA_SPI_ROLE_E role,
                                                 TUYA_SPI_MODE_E mode,
                                                 TUYA_SPI_DATABITS_E databits,
                                                 TUYA_SPI_BIT_ORDER_E bitorder,
                                                 TUYA_SPI_TYPE_E type,
                                                 uint32_t freq_hz,
                                                 bool use_dma)
{
    TUYA_SPI_BASE_CFG_T cfg;
    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.role = role;
    cfg.mode = mode;
    cfg.type = type;
    cfg.databits = databits;
    cfg.bitorder = bitorder;
    cfg.freq_hz = freq_hz;
    cfg.spi_dma_flags = use_dma ? 1U : 0U;
    return cfg;
}

static TUYA_SPI_BASE_CFG_T test_spi_build_cfg(TUYA_SPI_ROLE_E role, bool use_dma)
{
    return test_spi_build_cfg_ex(role,
                                 TUYA_SPI_MODE0,
                                 TUYA_SPI_DATA_BIT8,
                                 TUYA_SPI_ORDER_MSB2LSB,
                                 TUYA_SPI_AUTO_TYPE,
                                 1000000U,
                                 use_dma);
}

static bool test_spi_expect_role_not_supported(uint32_t port, TUYA_SPI_ROLE_E role)
{
    if (port < (uint32_t)TUYA_SPI_NUM_2) {
        return false;
    }

    return ((role == TUYA_SPI_ROLE_SLAVE) || (role == TUYA_SPI_ROLE_SLAVE_SIMPLEX));
}

static void test_spi_cfg_matrix_add(spi_stat_t *st, const char *name, OPERATE_RET ret, bool expect_not_supported)
{
    bool pass;
    bool skip = false;

    if (expect_not_supported) {
        pass = (ret == OPRT_NOT_SUPPORTED);
        skip = (ret == OPRT_NOT_SUPPORTED);
    } else {
        pass = ((ret == OPRT_OK) || (ret == OPRT_NOT_SUPPORTED));
        skip = (ret == OPRT_NOT_SUPPORTED);
    }

    test_spi_stat_add(st, name, pass, skip, ret);
}

static OPERATE_RET test_spi_cfg_matrix(uint32_t port, bool use_dma)
{
    static const TUYA_SPI_ROLE_E role_list[] = {
        TUYA_SPI_ROLE_MASTER,
        TUYA_SPI_ROLE_SLAVE,
        TUYA_SPI_ROLE_MASTER_SIMPLEX,
        TUYA_SPI_ROLE_SLAVE_SIMPLEX,
    };
    static const TUYA_SPI_MODE_E mode_list[] = {
        TUYA_SPI_MODE0, TUYA_SPI_MODE1, TUYA_SPI_MODE2, TUYA_SPI_MODE3,
    };
    static const TUYA_SPI_DATABITS_E databits_list[] = {
        TUYA_SPI_DATA_BIT8, TUYA_SPI_DATA_BIT16,
    };
    static const TUYA_SPI_BIT_ORDER_E bitorder_list[] = {
        TUYA_SPI_ORDER_MSB2LSB, TUYA_SPI_ORDER_LSB2MSB,
    };
    static const TUYA_SPI_TYPE_E type_list[] = {
        TUYA_SPI_AUTO_TYPE, TUYA_SPI_SOFT_TYPE, TUYA_SPI_SOFT_ONE_WIRE_TYPE,
    };
    static const uint32_t freq_list[] = {100000U, 1000000U, 4000000U, 8000000U, 20000000U};
    spi_stat_t st = {0};
    TUYA_SPI_BASE_CFG_T cfg;
    OPERATE_RET ret;
    uint32_t i;
    char tc_name[112];

    if (!test_spi_port_in_range(port)) {
        return OPRT_INVALID_PARM;
    }

    for (i = 0U; i < (uint32_t)(sizeof(role_list) / sizeof(role_list[0])); i++) {
        bool expect_not_supported = test_spi_expect_role_not_supported(port, role_list[i]);
        cfg = test_spi_build_cfg_ex(role_list[i],
                                    TUYA_SPI_MODE0,
                                    TUYA_SPI_DATA_BIT8,
                                    TUYA_SPI_ORDER_MSB2LSB,
                                    TUYA_SPI_AUTO_TYPE,
                                    1000000U,
                                    use_dma);
        ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &cfg);
        snprintf(tc_name, sizeof(tc_name), "TCM001 cfg_role_%u_dma%u",
                 (unsigned int)role_list[i], (unsigned int)use_dma);
        test_spi_cfg_matrix_add(&st, tc_name, ret, expect_not_supported);
        if (ret == OPRT_OK) {
            (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);
        }
    }

    for (i = 0U; i < (uint32_t)(sizeof(mode_list) / sizeof(mode_list[0])); i++) {
        cfg = test_spi_build_cfg_ex(TUYA_SPI_ROLE_MASTER,
                                    mode_list[i],
                                    TUYA_SPI_DATA_BIT8,
                                    TUYA_SPI_ORDER_MSB2LSB,
                                    TUYA_SPI_AUTO_TYPE,
                                    1000000U,
                                    use_dma);
        ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &cfg);
        snprintf(tc_name, sizeof(tc_name), "TCM002 cfg_mode_%u_dma%u",
                 (unsigned int)mode_list[i], (unsigned int)use_dma);
        test_spi_cfg_matrix_add(&st, tc_name, ret, false);
        if (ret == OPRT_OK) {
            (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);
        }
    }

    for (i = 0U; i < (uint32_t)(sizeof(databits_list) / sizeof(databits_list[0])); i++) {
        cfg = test_spi_build_cfg_ex(TUYA_SPI_ROLE_MASTER,
                                    TUYA_SPI_MODE0,
                                    databits_list[i],
                                    TUYA_SPI_ORDER_MSB2LSB,
                                    TUYA_SPI_AUTO_TYPE,
                                    1000000U,
                                    use_dma);
        ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &cfg);
        snprintf(tc_name, sizeof(tc_name), "TCM003 cfg_databits_%u_dma%u",
                 (unsigned int)databits_list[i], (unsigned int)use_dma);
        test_spi_cfg_matrix_add(&st, tc_name, ret, false);
        if (ret == OPRT_OK) {
            (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);
        }
    }

    for (i = 0U; i < (uint32_t)(sizeof(bitorder_list) / sizeof(bitorder_list[0])); i++) {
        cfg = test_spi_build_cfg_ex(TUYA_SPI_ROLE_MASTER,
                                    TUYA_SPI_MODE0,
                                    TUYA_SPI_DATA_BIT8,
                                    bitorder_list[i],
                                    TUYA_SPI_AUTO_TYPE,
                                    1000000U,
                                    use_dma);
        ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &cfg);
        snprintf(tc_name, sizeof(tc_name), "TCM004 cfg_bitorder_%u_dma%u",
                 (unsigned int)bitorder_list[i], (unsigned int)use_dma);
        test_spi_cfg_matrix_add(&st, tc_name, ret, false);
        if (ret == OPRT_OK) {
            (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);
        }
    }

    for (i = 0U; i < (uint32_t)(sizeof(type_list) / sizeof(type_list[0])); i++) {
        cfg = test_spi_build_cfg_ex(TUYA_SPI_ROLE_MASTER,
                                    TUYA_SPI_MODE0,
                                    TUYA_SPI_DATA_BIT8,
                                    TUYA_SPI_ORDER_MSB2LSB,
                                    type_list[i],
                                    1000000U,
                                    use_dma);
        ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &cfg);
        snprintf(tc_name, sizeof(tc_name), "TCM005 cfg_type_%u_dma%u",
                 (unsigned int)type_list[i], (unsigned int)use_dma);
        test_spi_cfg_matrix_add(&st, tc_name, ret, false);
        if (ret == OPRT_OK) {
            (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);
        }
    }

    for (i = 0U; i < (uint32_t)(sizeof(freq_list) / sizeof(freq_list[0])); i++) {
        cfg = test_spi_build_cfg_ex(TUYA_SPI_ROLE_MASTER,
                                    TUYA_SPI_MODE0,
                                    TUYA_SPI_DATA_BIT8,
                                    TUYA_SPI_ORDER_MSB2LSB,
                                    TUYA_SPI_AUTO_TYPE,
                                    freq_list[i],
                                    use_dma);
        ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &cfg);
        snprintf(tc_name, sizeof(tc_name), "TCM006 cfg_freq_%u_dma%u",
                 (unsigned int)freq_list[i], (unsigned int)use_dma);
        test_spi_cfg_matrix_add(&st, tc_name, ret, false);
        if (ret == OPRT_OK) {
            (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);
        }
    }

    test_spi_stat_summary(&st, use_dma ? "cfg_matrix_dma" : "cfg_matrix_nodma");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET test_spi_enable_async_mode(TUYA_SPI_NUM_E port)
{
    OPERATE_RET ret;

    test_spi_evt_reset(port);
    ret = tkl_spi_irq_init(port, test_spi_irq_cb);
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = tkl_spi_irq_enable(port);
    if (ret != OPRT_OK) {
        (void)tkl_spi_irq_disable(port);
        return ret;
    }

    test_spi_evt_reset(port);
    return ret;
}

static void test_spi_loop_slave_worker(beken_thread_arg_t arg)
{
    spi_loop_worker_t *ctx = (spi_loop_worker_t *)arg;
    TUYA_SPI_BASE_CFG_T cfg;
    uint32_t loop;
    uint32_t i;
    uint32_t rx_evt_base;
    uint32_t tx_evt_base;
    OPERATE_RET ret;

    if (ctx == NULL) {
        rtos_delete_thread(NULL);
        return;
    }

    ctx->last_ret = SPI_WORKER_RET_PENDING;
    ctx->stage = SPI_WORKER_STAGE_INIT;
    ctx->loop_idx = 0U;
    cfg = test_spi_build_cfg(TUYA_SPI_ROLE_SLAVE, ctx->use_dma);

    ret = tkl_spi_init(ctx->port, &cfg);
    if (ret != OPRT_OK) {
        ctx->last_ret = ret;
        ctx->exited = 1U;
        SPI_TEST_LOG("[xspi] slave_init_failed port=%u dma_enabled=%u return_code=%d\r\n",
                     (unsigned int)ctx->port, (unsigned int)ctx->use_dma, ret);
        rtos_delete_thread(NULL);
        return;
    }

    ret = test_spi_enable_async_mode(ctx->port);
    if (ret != OPRT_OK) {
        ctx->last_ret = ret;
        ctx->exited = 1U;
        SPI_TEST_LOG("[xspi] slave_async_enable_failed port=%u dma_enabled=%u return_code=%d\r\n",
                     (unsigned int)ctx->port, (unsigned int)ctx->use_dma, ret);
        (void)tkl_spi_deinit(ctx->port);
        rtos_delete_thread(NULL);
        return;
    }

    SPI_TEST_DBG("slave worker start p=%u dma=%u loops=%u bytes=%u\r\n",
                 (unsigned int)ctx->port,
                 (unsigned int)ctx->use_dma,
                 (unsigned int)ctx->loops,
                 (unsigned int)ctx->bytes);

    ctx->started = 1U;
    for (loop = 0; (loop < ctx->loops) && (ctx->stop_req == 0U); loop++) {
        ctx->loop_idx = loop;
        ctx->ready_seq = loop + 1U;
        ctx->stage = SPI_WORKER_STAGE_RECV_ARM;
        SPI_TEST_DBG("slave loop=%u arm recv\r\n", (unsigned int)loop);
        rx_evt_base = test_spi_evt_count_get(ctx->port, TUYA_SPI_EVENT_RX_COMPLETE);
        ret = tkl_spi_recv(ctx->port, s_spi_slave_rx, ctx->bytes);
        if (ret != OPRT_OK) {
            SPI_TEST_LOG("[xspi] slave_recv failed, loop=%u ret=%d\r\n",
                         (unsigned int)loop, ret);
            test_spi_dump_status("slave recv fail", ctx->port);
            ctx->last_ret = ret;
            goto __EXIT;
        }
        ret = test_spi_wait_event(ctx->port,
                                  TUYA_SPI_EVENT_RX_COMPLETE,
                                  rx_evt_base,
                                  SPI_TEST_WAIT_TIMEOUT_MS,
                                  "slave_wait_rx_done");
        if (ret != OPRT_OK) {
            SPI_TEST_LOG("[xspi] slave_wait_rx_done timeout, loop=%u ret=%d\r\n",
                         (unsigned int)loop, ret);
            ctx->last_ret = ret;
            goto __EXIT;
        }
        ctx->recv_ok++;

        for (i = 0; i < ctx->bytes; i++) {
            uint8_t expected = test_spi_pattern(loop, i);
            if ((ctx->verify_data == true) && (s_spi_slave_rx[i] != expected)) {
                ctx->mismatch++;
            }
            s_spi_slave_tx[i] = test_spi_resp_from_req(s_spi_slave_rx[i]);
        }

        ctx->resp_ready_seq = loop + 1U;
        ctx->stage = SPI_WORKER_STAGE_SEND_ARM;
        SPI_TEST_DBG("slave loop=%u arm send\r\n", (unsigned int)loop);
        tx_evt_base = test_spi_evt_count_get(ctx->port, TUYA_SPI_EVENT_TX_COMPLETE);
        ret = tkl_spi_send(ctx->port, s_spi_slave_tx, ctx->bytes);
        if (ret != OPRT_OK) {
            SPI_TEST_LOG("[xspi] slave_send failed, loop=%u ret=%d\r\n",
                         (unsigned int)loop, ret);
            test_spi_dump_status("slave send fail", ctx->port);
            ctx->last_ret = ret;
            goto __EXIT;
        }
        ret = test_spi_wait_event(ctx->port,
                                  TUYA_SPI_EVENT_TX_COMPLETE,
                                  tx_evt_base,
                                  SPI_TEST_WAIT_TIMEOUT_MS,
                                  "slave_wait_tx_done");
        if (ret != OPRT_OK) {
            SPI_TEST_LOG("[xspi] slave_wait_tx_done timeout, loop=%u ret=%d\r\n",
                         (unsigned int)loop, ret);
            ctx->last_ret = ret;
            goto __EXIT;
        }
        ctx->send_ok++;
        if ((loop + 1U) < ctx->loops) {
            tkl_system_sleep(SPI_TEST_INTER_LOOP_DELAY_MS);
        }
    }

    ctx->last_ret = OPRT_OK;

__EXIT:
    SPI_TEST_DBG("slave worker exit p=%u last_ret=%d recv_ok=%u send_ok=%u\r\n",
                 (unsigned int)ctx->port,
                 (int)ctx->last_ret,
                 (unsigned int)ctx->recv_ok,
                 (unsigned int)ctx->send_ok);
    ctx->stage = SPI_WORKER_STAGE_DEINIT;
    (void)tkl_spi_deinit(ctx->port);
    ctx->stage = SPI_WORKER_STAGE_DONE;
    ctx->exited = 1U;
    rtos_delete_thread(NULL);
}

static OPERATE_RET test_spi_param(uint32_t port)
{
    spi_stat_t st = {0};
    TUYA_SPI_BASE_CFG_T cfg = test_spi_build_cfg(TUYA_SPI_ROLE_MASTER, false);
    TUYA_SPI_BASE_CFG_T bad_cfg = cfg;
    TUYA_SPI_STATUS_T status;
    uint8_t one = 0x5AU;
    OPERATE_RET ret;

    if (!test_spi_port_in_range(port)) {
        return OPRT_INVALID_PARM;
    }

    (void)os_memset(&status, 0, sizeof(status));
    (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);

    ret = tkl_spi_init((TUYA_SPI_NUM_E)TUYA_SPI_NUM_MAX, &cfg);
    test_spi_stat_add(&st, "TC001 init(invalid_port)", (ret != OPRT_OK), false, ret);

    ret = tkl_spi_init((TUYA_SPI_NUM_E)port, NULL);
    test_spi_stat_add(&st, "TC002 init(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_spi_send((TUYA_SPI_NUM_E)port, NULL, 1);
    test_spi_stat_add(&st, "TC003 send(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_spi_recv((TUYA_SPI_NUM_E)port, NULL, 1);
    test_spi_stat_add(&st, "TC004 recv(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_spi_transfer((TUYA_SPI_NUM_E)port, NULL, NULL, 1);
    test_spi_stat_add(&st, "TC005 transfer(NULL,NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_spi_get_status((TUYA_SPI_NUM_E)port, NULL);
    test_spi_stat_add(&st, "TC006 get_status(NULL)", (ret != OPRT_OK), false, ret);

    ret = tkl_spi_send((TUYA_SPI_NUM_E)port, &one, 0);
    test_spi_stat_add(&st, "TC007 send(len0)", (ret != OPRT_OK), false, ret);

    ret = tkl_spi_recv((TUYA_SPI_NUM_E)port, &one, 0);
    test_spi_stat_add(&st, "TC008 recv(len0)", (ret != OPRT_OK), false, ret);

    bad_cfg.role = TUYA_SPI_ROLE_INACTIVE;
    ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &bad_cfg);
    test_spi_stat_add(&st, "TC009 init(role=inactive->master_default)", (ret == OPRT_OK), false, ret);

    ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &cfg);
    test_spi_stat_add(&st, "TC010 init(master)", (ret == OPRT_OK), false, ret);
    if (ret == OPRT_OK) {
        ret = tkl_spi_get_status((TUYA_SPI_NUM_E)port, &status);
        test_spi_stat_add(&st, "TC011 get_status", (ret == OPRT_OK), false, ret);

        ret = tkl_spi_ioctl((TUYA_SPI_NUM_E)port, 0x1234U, &one);
        test_spi_stat_add(&st, "TC012 ioctl(unknown)", (ret != OPRT_OK), false, ret);

        ret = tkl_spi_deinit((TUYA_SPI_NUM_E)port);
        test_spi_stat_add(&st, "TC013 deinit", (ret == OPRT_OK), false, ret);
    }

    ret = tkl_spi_deinit((TUYA_SPI_NUM_E)port);
    test_spi_stat_add(&st, "TC014 deinit(repeat)", (ret == OPRT_OK), false, ret);

    test_spi_stat_summary(&st, "param");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET test_spi_api(uint32_t port, bool use_dma)
{
    spi_stat_t st = {0};
    TUYA_SPI_BASE_CFG_T cfg = test_spi_build_cfg(TUYA_SPI_ROLE_MASTER, use_dma);
    TUYA_SPI_STATUS_T status;
    OPERATE_RET ret;

    if (!test_spi_port_in_range(port)) {
        return OPRT_INVALID_PARM;
    }

    (void)os_memset(&status, 0, sizeof(status));
    (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);

    ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &cfg);
    test_spi_stat_add(&st, "TC101 init", (ret == OPRT_OK), false, ret);
    if (ret != OPRT_OK) {
        test_spi_stat_summary(&st, use_dma ? "api_dma" : "api_nodma");
        return OPRT_COM_ERROR;
    }

    ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &cfg);
    test_spi_stat_add(&st, "TC102 init(repeat)", (ret == OPRT_OK), false, ret);

    ret = tkl_spi_get_status((TUYA_SPI_NUM_E)port, &status);
    test_spi_stat_add(&st, "TC103 get_status", (ret == OPRT_OK), false, ret);

    ret = tkl_spi_irq_init((TUYA_SPI_NUM_E)port, test_spi_irq_cb);
    test_spi_stat_add(&st, "TC104 irq_init", (ret == OPRT_OK || ret == OPRT_NOT_SUPPORTED), (ret == OPRT_NOT_SUPPORTED), ret);

    ret = tkl_spi_irq_enable((TUYA_SPI_NUM_E)port);
    test_spi_stat_add(&st, "TC105 irq_enable", (ret == OPRT_OK || ret == OPRT_NOT_SUPPORTED), (ret == OPRT_NOT_SUPPORTED), ret);

    ret = tkl_spi_irq_disable((TUYA_SPI_NUM_E)port);
    test_spi_stat_add(&st, "TC106 irq_disable", (ret == OPRT_OK || ret == OPRT_NOT_SUPPORTED), (ret == OPRT_NOT_SUPPORTED), ret);

    ret = tkl_spi_abort_transfer((TUYA_SPI_NUM_E)port);
    test_spi_stat_add(&st, "TC107 abort", (ret == OPRT_OK), false, ret);

    if (use_dma) {
        uint32_t max_len = tkl_spi_get_max_dma_data_length();
        test_spi_stat_add(&st, "TC108 get_max_dma_len", (max_len > 0U), false, (max_len > 0U) ? OPRT_OK : OPRT_COM_ERROR);
    }

    ret = tkl_spi_deinit((TUYA_SPI_NUM_E)port);
    test_spi_stat_add(&st, "TC109 deinit", (ret == OPRT_OK), false, ret);

    ret = tkl_spi_deinit((TUYA_SPI_NUM_E)port);
    test_spi_stat_add(&st, "TC110 deinit(repeat)", (ret == OPRT_OK), false, ret);

    test_spi_stat_summary(&st, use_dma ? "api_dma" : "api_nodma");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET test_spi_api_all(void)
{
    uint32_t port;
    uint32_t hw_ports = test_spi_hw_port_num();
    uint32_t pass = 0U;
    uint32_t fail = 0U;
    OPERATE_RET ret = OPRT_OK;

    for (port = 0; port < hw_ports; port++) {
        OPERATE_RET one = test_spi_api(port, false);
        if (one == OPRT_OK) {
            pass++;
        } else {
            fail++;
            ret = OPRT_COM_ERROR;
        }

        one = test_spi_api(port, true);
        if (one == OPRT_OK) {
            pass++;
        } else {
            fail++;
            ret = OPRT_COM_ERROR;
        }
    }

    SPI_TEST_LOG("[xspi] api_all_result passed_count=%u failed_count=%u\r\n",
                 (unsigned int)pass,
                 (unsigned int)fail);
    return ret;
}

static OPERATE_RET test_spi_stress(uint32_t port, uint32_t loops)
{
    TUYA_SPI_BASE_CFG_T cfg;
    OPERATE_RET ret = OPRT_OK;
    uint32_t pass = 0U;
    uint32_t fail = 0U;
    uint32_t i;

    if (!test_spi_port_in_range(port) || loops == 0U) {
        return OPRT_INVALID_PARM;
    }

    (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);

    for (i = 0; i < loops; i++) {
        bool use_dma = ((i & 1U) != 0U);
        cfg = test_spi_build_cfg(TUYA_SPI_ROLE_MASTER, use_dma);

        ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &cfg);
        if (ret != OPRT_OK) {
            fail++;
            break;
        }

        ret = tkl_spi_abort_transfer((TUYA_SPI_NUM_E)port);
        if (ret != OPRT_OK) {
            fail++;
            break;
        }

        ret = tkl_spi_deinit((TUYA_SPI_NUM_E)port);
        if (ret != OPRT_OK) {
            fail++;
            break;
        }
        pass++;
    }

    (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);
    SPI_TEST_LOG("[xspi] stress_result port=%u requested_loops=%u passed_count=%u failed_count=%u return_code=%d\r\n",
                 (unsigned int)port,
                 (unsigned int)loops,
                 (unsigned int)pass,
                 (unsigned int)fail,
                 ret);

    return (fail == 0U) ? OPRT_OK : ((ret != OPRT_OK) ? ret : OPRT_COM_ERROR);
}

static void test_spi_dual_api_worker(beken_thread_arg_t arg)
{
    spi_dual_worker_t *ctx = (spi_dual_worker_t *)arg;
    uint32_t i;

    if ((ctx == NULL) || (ctx->start_sem == NULL) || (ctx->done_sem == NULL)) {
        rtos_delete_thread(NULL);
        return;
    }

    if (rtos_get_semaphore(ctx->start_sem, BEKEN_WAIT_FOREVER) != kNoErr) {
        (void)rtos_set_semaphore(ctx->done_sem);
        rtos_delete_thread(NULL);
        return;
    }

    for (i = 0U; i < ctx->loops; i++) {
        bool use_dma = ((i & 1U) != 0U);
        OPERATE_RET ret = test_spi_api(ctx->port, use_dma);
        if (ret == OPRT_OK) {
            ctx->pass++;
        } else {
            ctx->fail++;
        }
    }

    (void)rtos_set_semaphore(ctx->done_sem);
    rtos_delete_thread(NULL);
}

static OPERATE_RET test_spi_dual_api(uint32_t port0, uint32_t port1, uint32_t loops)
{
    beken_thread_t th0 = NULL;
    beken_thread_t th1 = NULL;
    beken_semaphore_t start_sem = NULL;
    beken_semaphore_t done_sem = NULL;
    spi_dual_worker_t w0;
    spi_dual_worker_t w1;
    OPERATE_RET ret = OPRT_OK;

    if (!test_spi_port_in_range(port0) || !test_spi_port_in_range(port1) || loops == 0U || port0 == port1) {
        return OPRT_INVALID_PARM;
    }

    if (rtos_init_semaphore_ex(&start_sem, 2, 0) != kNoErr) {
        return OPRT_COM_ERROR;
    }
    if (rtos_init_semaphore_ex(&done_sem, 2, 0) != kNoErr) {
        (void)rtos_deinit_semaphore(&start_sem);
        return OPRT_COM_ERROR;
    }

    (void)os_memset(&w0, 0, sizeof(w0));
    (void)os_memset(&w1, 0, sizeof(w1));
    w0.port = port0;
    w1.port = port1;
    w0.loops = loops;
    w1.loops = loops;
    w0.start_sem = &start_sem;
    w1.start_sem = &start_sem;
    w0.done_sem = &done_sem;
    w1.done_sem = &done_sem;

    if (rtos_create_thread(&th0, 6, "xspi_d0", test_spi_dual_api_worker, 3584, &w0) != kNoErr) {
        (void)rtos_deinit_semaphore(&start_sem);
        (void)rtos_deinit_semaphore(&done_sem);
        return OPRT_COM_ERROR;
    }
    if (rtos_create_thread(&th1, 6, "xspi_d1", test_spi_dual_api_worker, 3584, &w1) != kNoErr) {
        (void)rtos_set_semaphore(&start_sem);
        (void)rtos_get_semaphore(&done_sem, SPI_TEST_WAIT_TIMEOUT_MS);
        (void)rtos_thread_join(&th0);
        (void)rtos_deinit_semaphore(&start_sem);
        (void)rtos_deinit_semaphore(&done_sem);
        return OPRT_COM_ERROR;
    }

    (void)rtos_set_semaphore(&start_sem);
    (void)rtos_set_semaphore(&start_sem);
    if (rtos_get_semaphore(&done_sem, SPI_TEST_WAIT_TIMEOUT_MS) != kNoErr ||
        rtos_get_semaphore(&done_sem, SPI_TEST_WAIT_TIMEOUT_MS) != kNoErr) {
        ret = OPRT_TIMEOUT;
    }

    (void)rtos_thread_join(&th0);
    (void)rtos_thread_join(&th1);

    SPI_TEST_LOG("[xspi] dualapi_result port0=%u port0_passed_count=%u port0_failed_count=%u port1=%u port1_passed_count=%u port1_failed_count=%u return_code=%d\r\n",
                 (unsigned int)port0,
                 (unsigned int)w0.pass,
                 (unsigned int)w0.fail,
                 (unsigned int)port1,
                 (unsigned int)w1.pass,
                 (unsigned int)w1.fail,
                 ret);

    (void)rtos_deinit_semaphore(&start_sem);
    (void)rtos_deinit_semaphore(&done_sem);
    if (ret != OPRT_OK) {
        return ret;
    }
    if ((w0.fail != 0U) || (w1.fail != 0U)) {
        return OPRT_COM_ERROR;
    }
    return OPRT_OK;
}

static OPERATE_RET test_spi_loopback(uint32_t master_port, uint32_t slave_port,
                                     uint32_t bytes, uint32_t loops,
                                     bool use_dma, bool verify_data)
{
    TUYA_SPI_BASE_CFG_T master_cfg = test_spi_build_cfg(TUYA_SPI_ROLE_MASTER, use_dma);
    spi_loop_worker_t worker;
    beken_thread_t slave_thread = NULL;
    OPERATE_RET ret = OPRT_OK;
    uint32_t wait_ms = 0U;
    bk_err_t os_ret;
    uint32_t loop;
    uint32_t i;
    uint32_t tx_evt_base;
    uint32_t rx_evt_base;
    uint32_t master_mismatch = 0U;

    if (!test_spi_port_in_range(master_port) || !test_spi_port_in_range(slave_port)) {
        return OPRT_INVALID_PARM;
    }
    if (master_port == slave_port) {
        return OPRT_INVALID_PARM;
    }
    if ((bytes == 0U) || (bytes > SPI_TEST_MAX_XFER_BYTES) || (loops == 0U)) {
        return OPRT_INVALID_PARM;
    }

    (void)os_memset(&worker, 0, sizeof(worker));
    worker.port = (TUYA_SPI_NUM_E)slave_port;
    worker.use_dma = use_dma;
    worker.verify_data = verify_data;
    worker.bytes = bytes;
    worker.loops = loops;
    worker.last_ret = SPI_WORKER_RET_PENDING;

    if (rtos_create_thread(&slave_thread, 6, "xspi_slv", test_spi_loop_slave_worker, 3072, &worker) != kNoErr) {
        return OPRT_COM_ERROR;
    }

    ret = tkl_spi_init((TUYA_SPI_NUM_E)master_port, &master_cfg);
    if (ret != OPRT_OK) {
        worker.stop_req = 1U;
        (void)test_spi_join_worker_with_timeout("loop_init_fail", &slave_thread, &worker,
                                                (TUYA_SPI_NUM_E)master_port, (TUYA_SPI_NUM_E)slave_port);
        return ret;
    }

    ret = test_spi_enable_async_mode((TUYA_SPI_NUM_E)master_port);
    if (ret != OPRT_OK) {
        worker.stop_req = 1U;
        (void)test_spi_join_worker_with_timeout("loop_async_enable_fail", &slave_thread, &worker,
                                                (TUYA_SPI_NUM_E)master_port, (TUYA_SPI_NUM_E)slave_port);
        (void)tkl_spi_deinit((TUYA_SPI_NUM_E)master_port);
        return ret;
    }

    SPI_TEST_DBG("master init ok p=%u dma=%u loops=%u bytes=%u\r\n",
                 (unsigned int)master_port,
                 (unsigned int)use_dma,
                 (unsigned int)loops,
                 (unsigned int)bytes);

    while ((worker.started == 0U) && (worker.exited == 0U) && (wait_ms < SPI_TEST_WAIT_TIMEOUT_MS)) {
        tkl_system_sleep(SPI_TEST_POLL_INTERVAL_MS);
        wait_ms += SPI_TEST_POLL_INTERVAL_MS;
    }
    if (worker.started == 0U) {
        ret = (worker.last_ret != SPI_WORKER_RET_PENDING) ? worker.last_ret : OPRT_TIMEOUT;
        goto __EXIT;
    }

    for (loop = 0U; loop < loops; loop++) {
        SPI_TEST_DBG("master loop=%u wait slave ready_seq>%u\r\n",
                     (unsigned int)loop,
                     (unsigned int)loop);
        wait_ms = 0U;
        while ((worker.ready_seq <= loop) && (worker.exited == 0U) && (wait_ms < SPI_TEST_WAIT_TIMEOUT_MS)) {
            tkl_system_sleep(SPI_TEST_POLL_INTERVAL_MS);
            wait_ms += SPI_TEST_POLL_INTERVAL_MS;
        }
        if (worker.ready_seq <= loop) {
            ret = OPRT_TIMEOUT;
            SPI_TEST_LOG("[xspi] wait slave ready timeout, loop=%u last_slave_ret=%d\r\n",
                         (unsigned int)loop, (int)worker.last_ret);
            test_spi_dump_status("wait slave ready timeout master", (TUYA_SPI_NUM_E)master_port);
            test_spi_dump_status("wait slave ready timeout slave", (TUYA_SPI_NUM_E)slave_port);
            break;
        }
        tkl_system_sleep(SPI_TEST_ROLE_SWITCH_DELAY_MS);
        test_spi_dump_status("master before send master", (TUYA_SPI_NUM_E)master_port);
        test_spi_dump_status("master before send slave", (TUYA_SPI_NUM_E)slave_port);

        for (i = 0U; i < bytes; i++) {
            s_spi_master_tx[i] = test_spi_pattern(loop, i);
            s_spi_master_rx[i] = 0U;
        }

        tx_evt_base = test_spi_evt_count_get((TUYA_SPI_NUM_E)master_port, TUYA_SPI_EVENT_TX_COMPLETE);
        ret = tkl_spi_send((TUYA_SPI_NUM_E)master_port, s_spi_master_tx, bytes);
        if (ret != OPRT_OK) {
            SPI_TEST_LOG("[xspi] master_send failed, loop=%u ret=%d\r\n",
                         (unsigned int)loop, ret);
            test_spi_dump_status("master send fail master", (TUYA_SPI_NUM_E)master_port);
            test_spi_dump_status("master send fail slave", (TUYA_SPI_NUM_E)slave_port);
            break;
        }
        ret = test_spi_wait_event((TUYA_SPI_NUM_E)master_port,
                                  TUYA_SPI_EVENT_TX_COMPLETE,
                                  tx_evt_base,
                                  SPI_TEST_WAIT_TIMEOUT_MS,
                                  "master_wait_tx_done");
        if (ret != OPRT_OK) {
            SPI_TEST_LOG("[xspi] wait master tx done timeout, loop=%u\r\n",
                         (unsigned int)loop);
            break;
        }

        SPI_TEST_DBG("master loop=%u wait slave resp_ready_seq>%u\r\n",
                     (unsigned int)loop,
                     (unsigned int)loop);
        wait_ms = 0U;
        while ((worker.resp_ready_seq <= loop) && (worker.exited == 0U) && (wait_ms < SPI_TEST_WAIT_TIMEOUT_MS)) {
            tkl_system_sleep(SPI_TEST_POLL_INTERVAL_MS);
            wait_ms += SPI_TEST_POLL_INTERVAL_MS;
        }
        if (worker.resp_ready_seq <= loop) {
            ret = OPRT_TIMEOUT;
            SPI_TEST_LOG("[xspi] wait slave resp_ready timeout, loop=%u last_slave_ret=%d\r\n",
                         (unsigned int)loop, (int)worker.last_ret);
            test_spi_dump_status("wait slave resp_ready timeout master", (TUYA_SPI_NUM_E)master_port);
            test_spi_dump_status("wait slave resp_ready timeout slave", (TUYA_SPI_NUM_E)slave_port);
            break;
        }
        tkl_system_sleep(SPI_TEST_ROLE_SWITCH_DELAY_MS);
        test_spi_dump_status("master before recv master", (TUYA_SPI_NUM_E)master_port);
        test_spi_dump_status("master before recv slave", (TUYA_SPI_NUM_E)slave_port);

        rx_evt_base = test_spi_evt_count_get((TUYA_SPI_NUM_E)master_port, TUYA_SPI_EVENT_RX_COMPLETE);
        ret = tkl_spi_recv((TUYA_SPI_NUM_E)master_port, s_spi_master_rx, bytes);
        if (ret != OPRT_OK) {
            SPI_TEST_LOG("[xspi] master_recv failed, loop=%u ret=%d\r\n",
                         (unsigned int)loop, ret);
            test_spi_dump_status("master recv fail master", (TUYA_SPI_NUM_E)master_port);
            test_spi_dump_status("master recv fail slave", (TUYA_SPI_NUM_E)slave_port);
            break;
        }
        ret = test_spi_wait_event((TUYA_SPI_NUM_E)master_port,
                                  TUYA_SPI_EVENT_RX_COMPLETE,
                                  rx_evt_base,
                                  SPI_TEST_WAIT_TIMEOUT_MS,
                                  "master_wait_rx_done");
        if (ret != OPRT_OK) {
            SPI_TEST_LOG("[xspi] wait master rx done timeout, loop=%u\r\n",
                         (unsigned int)loop);
            break;
        }
        wait_ms = 0U;
        while ((worker.send_ok <= loop) && (worker.exited == 0U) && (wait_ms < SPI_TEST_WAIT_TIMEOUT_MS)) {
            tkl_system_sleep(SPI_TEST_POLL_INTERVAL_MS);
            wait_ms += SPI_TEST_POLL_INTERVAL_MS;
        }
        if (worker.send_ok <= loop) {
            ret = OPRT_TIMEOUT;
            SPI_TEST_LOG("[xspi] wait slave send done timeout, loop=%u last_slave_ret=%d\r\n",
                         (unsigned int)loop, (int)worker.last_ret);
            test_spi_dump_status("wait slave send done timeout master", (TUYA_SPI_NUM_E)master_port);
            test_spi_dump_status("wait slave send done timeout slave", (TUYA_SPI_NUM_E)slave_port);
            break;
        }

        if (verify_data == true) {
            for (i = 0U; i < bytes; i++) {
                uint8_t expected = test_spi_resp_from_req(s_spi_master_tx[i]);
                if (s_spi_master_rx[i] != expected) {
                    master_mismatch++;
                    SPI_TEST_LOG("[xspi] resp mismatch, loop=%u idx=%u tx=%02x rx=%02x exp=%02x\r\n",
                                 (unsigned int)loop,
                                 (unsigned int)i,
                                 (unsigned int)s_spi_master_tx[i],
                                 (unsigned int)s_spi_master_rx[i],
                                 (unsigned int)expected);
                    ret = OPRT_COM_ERROR;
                    break;
                }
            }
            if (ret != OPRT_OK) {
                break;
            }
        }
    }

__EXIT:
    worker.stop_req = 1U;
    os_ret = test_spi_join_worker_with_timeout("loop_exit", &slave_thread, &worker,
                                               (TUYA_SPI_NUM_E)master_port, (TUYA_SPI_NUM_E)slave_port);
    if ((os_ret != OPRT_OK) && (ret == OPRT_OK)) {
        ret = os_ret;
    }

    (void)tkl_spi_deinit((TUYA_SPI_NUM_E)master_port);

    if ((ret == OPRT_OK || ret == OPRT_TIMEOUT) &&
        (worker.last_ret != OPRT_OK) &&
        (worker.last_ret != SPI_WORKER_RET_PENDING)) {
        ret = worker.last_ret;
    }
    if ((ret == OPRT_OK) && ((worker.mismatch != 0U) || (master_mismatch != 0U)) && (verify_data == true)) {
        ret = OPRT_COM_ERROR;
    }

    SPI_TEST_LOG("[xspi] %s_result master_port=%u slave_port=%u dma_enabled=%u transfer_bytes=%u requested_loops=%u slave_receive_success_count=%u slave_send_success_count=%u slave_mismatch_count=%u master_mismatch_count=%u return_code=%d\r\n",
                 (verify_data == true) ? "loop" : "dualrw",
                 (unsigned int)master_port,
                 (unsigned int)slave_port,
                 (unsigned int)use_dma,
                 (unsigned int)bytes,
                 (unsigned int)loops,
                 (unsigned int)worker.recv_ok,
                 (unsigned int)worker.send_ok,
                 (unsigned int)worker.mismatch,
                 (unsigned int)master_mismatch,
                 ret);

    return ret;
}

static void test_spi_oneway_slave_worker(beken_thread_arg_t arg)
{
    spi_loop_worker_t *ctx = (spi_loop_worker_t *)arg;
    TUYA_SPI_BASE_CFG_T cfg;
    uint32_t loop;
    uint32_t i;
    uint32_t rx_evt_base;
    uint32_t tx_evt_base;
    OPERATE_RET ret;

    if (ctx == NULL) {
        rtos_delete_thread(NULL);
        return;
    }

    ctx->last_ret = SPI_WORKER_RET_PENDING;
    ctx->stage = SPI_WORKER_STAGE_INIT;
    ctx->loop_idx = 0U;
    cfg = test_spi_build_cfg(TUYA_SPI_ROLE_SLAVE, ctx->use_dma);

    ret = tkl_spi_init(ctx->port, &cfg);
    if (ret != OPRT_OK) {
        ctx->last_ret = ret;
        ctx->exited = 1U;
        SPI_TEST_LOG("[xspi] oneway_slave_init_failed port=%u dma_enabled=%u return_code=%d\r\n",
                     (unsigned int)ctx->port, (unsigned int)ctx->use_dma, ret);
        rtos_delete_thread(NULL);
        return;
    }

    ret = test_spi_enable_async_mode(ctx->port);
    if (ret != OPRT_OK) {
        ctx->last_ret = ret;
        ctx->exited = 1U;
        SPI_TEST_LOG("[xspi] oneway_slave_async_enable_failed port=%u dma_enabled=%u return_code=%d\r\n",
                     (unsigned int)ctx->port, (unsigned int)ctx->use_dma, ret);
        (void)tkl_spi_deinit(ctx->port);
        rtos_delete_thread(NULL);
        return;
    }

    SPI_TEST_DBG("oneway slave worker start p=%u dma=%u mode=%s loops=%u bytes=%u\r\n",
                 (unsigned int)ctx->port,
                 (unsigned int)ctx->use_dma,
                 ctx->slave_send ? "send" : "recv",
                 (unsigned int)ctx->loops,
                 (unsigned int)ctx->bytes);

	ctx->started = 1U;
	for (loop = 0; (loop < ctx->loops) && (ctx->stop_req == 0U); loop++) {
		ctx->loop_idx = loop;
		if (ctx->slave_send) {
			for (i = 0; i < ctx->bytes; i++) {
				s_spi_slave_tx[i] = test_spi_pattern(loop, i);
			}
            ctx->stage = SPI_WORKER_STAGE_SEND_ARM;
            SPI_TEST_DBG("oneway slave loop=%u arm send\r\n", (unsigned int)loop);
            tx_evt_base = test_spi_evt_count_get(ctx->port, TUYA_SPI_EVENT_TX_COMPLETE);
            ret = tkl_spi_send(ctx->port, s_spi_slave_tx, ctx->bytes);
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] oneway slave_send failed, loop=%u ret=%d\r\n",
                             (unsigned int)loop, ret);
                test_spi_dump_status("oneway slave send fail", ctx->port);
				ctx->last_ret = ret;
				goto __EXIT;
			}
			ctx->ready_seq = loop + 1U;
			ret = test_spi_wait_event(ctx->port,
			                          TUYA_SPI_EVENT_TX_COMPLETE,
			                          tx_evt_base,
                                      SPI_TEST_WAIT_TIMEOUT_MS,
                                      "oneway_slave_wait_tx_done");
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] oneway slave wait tx done timeout, loop=%u ret=%d\r\n",
                             (unsigned int)loop, ret);
                ctx->last_ret = ret;
                goto __EXIT;
            }
            ctx->send_ok++;
        } else {
            ctx->stage = SPI_WORKER_STAGE_RECV_ARM;
            SPI_TEST_DBG("oneway slave loop=%u arm recv\r\n", (unsigned int)loop);
            rx_evt_base = test_spi_evt_count_get(ctx->port, TUYA_SPI_EVENT_RX_COMPLETE);
            ret = tkl_spi_recv(ctx->port, s_spi_slave_rx, ctx->bytes);
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] oneway slave_recv failed, loop=%u ret=%d\r\n",
                             (unsigned int)loop, ret);
                test_spi_dump_status("oneway slave recv fail", ctx->port);
				ctx->last_ret = ret;
				goto __EXIT;
			}
			ctx->ready_seq = loop + 1U;
			ret = test_spi_wait_event(ctx->port,
			                          TUYA_SPI_EVENT_RX_COMPLETE,
			                          rx_evt_base,
                                      SPI_TEST_WAIT_TIMEOUT_MS,
                                      "oneway_slave_wait_rx_done");
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] oneway slave wait rx done timeout, loop=%u ret=%d\r\n",
                             (unsigned int)loop, ret);
                ctx->last_ret = ret;
                goto __EXIT;
            }
            ctx->recv_ok++;
            if (ctx->verify_data) {
                for (i = 0; i < ctx->bytes; i++) {
                    uint8_t expected = test_spi_pattern(loop, i);
                    if (s_spi_slave_rx[i] != expected) {
                        ctx->mismatch++;
                    }
                }
            }
        }
        if ((loop + 1U) < ctx->loops) {
            tkl_system_sleep(SPI_TEST_INTER_LOOP_DELAY_MS);
        }
    }

    ctx->last_ret = OPRT_OK;

__EXIT:
    SPI_TEST_DBG("oneway slave worker exit p=%u last_ret=%d recv_ok=%u send_ok=%u mismatch=%u\r\n",
                 (unsigned int)ctx->port,
                 (int)ctx->last_ret,
                 (unsigned int)ctx->recv_ok,
                 (unsigned int)ctx->send_ok,
                 (unsigned int)ctx->mismatch);
    ctx->stage = SPI_WORKER_STAGE_DEINIT;
    (void)tkl_spi_deinit(ctx->port);
    ctx->stage = SPI_WORKER_STAGE_DONE;
    ctx->exited = 1U;
    rtos_delete_thread(NULL);
}

static OPERATE_RET test_spi_oneway(uint32_t master_port, uint32_t slave_port,
                                   uint32_t bytes, uint32_t loops,
                                   bool use_dma, bool master_send)
{
    TUYA_SPI_BASE_CFG_T master_cfg = test_spi_build_cfg(TUYA_SPI_ROLE_MASTER, use_dma);
    spi_loop_worker_t worker;
    beken_thread_t slave_thread = NULL;
    OPERATE_RET ret = OPRT_OK;
    uint32_t wait_ms = 0U;
    bk_err_t os_ret;
    uint32_t loop;
    uint32_t i;
    uint32_t tx_evt_base;
    uint32_t rx_evt_base;
    uint32_t master_mismatch = 0U;

    if (!test_spi_port_in_range(master_port) || !test_spi_port_in_range(slave_port)) {
        return OPRT_INVALID_PARM;
    }
    if (master_port == slave_port) {
        return OPRT_INVALID_PARM;
    }
    if ((bytes == 0U) || (bytes > SPI_TEST_MAX_XFER_BYTES) || (loops == 0U)) {
        return OPRT_INVALID_PARM;
    }

    (void)os_memset(&worker, 0, sizeof(worker));
    worker.port = (TUYA_SPI_NUM_E)slave_port;
    worker.slave_send = !master_send;
    worker.use_dma = use_dma;
    worker.verify_data = master_send;
    worker.bytes = bytes;
    worker.loops = loops;
    worker.last_ret = SPI_WORKER_RET_PENDING;

    if (rtos_create_thread(&slave_thread, 6, "xspi_1w", test_spi_oneway_slave_worker, 3072, &worker) != kNoErr) {
        return OPRT_COM_ERROR;
    }

    ret = tkl_spi_init((TUYA_SPI_NUM_E)master_port, &master_cfg);
    if (ret != OPRT_OK) {
        worker.stop_req = 1U;
        (void)test_spi_join_worker_with_timeout("oneway_init_fail", &slave_thread, &worker,
                                                (TUYA_SPI_NUM_E)master_port, (TUYA_SPI_NUM_E)slave_port);
        return ret;
    }

    ret = test_spi_enable_async_mode((TUYA_SPI_NUM_E)master_port);
    if (ret != OPRT_OK) {
        worker.stop_req = 1U;
        (void)test_spi_join_worker_with_timeout("oneway_async_enable_fail", &slave_thread, &worker,
                                                (TUYA_SPI_NUM_E)master_port, (TUYA_SPI_NUM_E)slave_port);
        (void)tkl_spi_deinit((TUYA_SPI_NUM_E)master_port);
        return ret;
    }

    SPI_TEST_DBG("oneway master init ok p=%u dma=%u mode=%s loops=%u bytes=%u\r\n",
                 (unsigned int)master_port,
                 (unsigned int)use_dma,
                 master_send ? "send" : "recv",
                 (unsigned int)loops,
                 (unsigned int)bytes);

    while ((worker.started == 0U) && (worker.exited == 0U) && (wait_ms < SPI_TEST_WAIT_TIMEOUT_MS)) {
        tkl_system_sleep(SPI_TEST_POLL_INTERVAL_MS);
        wait_ms += SPI_TEST_POLL_INTERVAL_MS;
    }
    if (worker.started == 0U) {
        ret = (worker.last_ret != SPI_WORKER_RET_PENDING) ? worker.last_ret : OPRT_TIMEOUT;
        goto __EXIT;
    }

    for (loop = 0U; loop < loops; loop++) {
        SPI_TEST_DBG("oneway master loop=%u wait slave ready_seq>%u\r\n",
                     (unsigned int)loop,
                     (unsigned int)loop);
        wait_ms = 0U;
        while ((worker.ready_seq <= loop) && (worker.exited == 0U) && (wait_ms < SPI_TEST_WAIT_TIMEOUT_MS)) {
            tkl_system_sleep(SPI_TEST_POLL_INTERVAL_MS);
            wait_ms += SPI_TEST_POLL_INTERVAL_MS;
        }
        if (worker.ready_seq <= loop) {
            ret = OPRT_TIMEOUT;
            SPI_TEST_LOG("[xspi] oneway wait slave ready timeout, loop=%u last_slave_ret=%d mode=%s\r\n",
                         (unsigned int)loop, (int)worker.last_ret, master_send ? "send" : "recv");
            test_spi_dump_status("oneway wait ready timeout master", (TUYA_SPI_NUM_E)master_port);
            test_spi_dump_status("oneway wait ready timeout slave", (TUYA_SPI_NUM_E)slave_port);
            break;
        }
        tkl_system_sleep(SPI_TEST_ROLE_SWITCH_DELAY_MS);

        if (master_send) {
            test_spi_dump_status("oneway before send master", (TUYA_SPI_NUM_E)master_port);
            test_spi_dump_status("oneway before send slave", (TUYA_SPI_NUM_E)slave_port);
            for (i = 0U; i < bytes; i++) {
                s_spi_master_tx[i] = test_spi_pattern(loop, i);
            }
            tx_evt_base = test_spi_evt_count_get((TUYA_SPI_NUM_E)master_port, TUYA_SPI_EVENT_TX_COMPLETE);
            ret = tkl_spi_send((TUYA_SPI_NUM_E)master_port, s_spi_master_tx, bytes);
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] oneway master_send failed, loop=%u ret=%d\r\n",
                             (unsigned int)loop, ret);
                test_spi_dump_status("oneway master send fail master", (TUYA_SPI_NUM_E)master_port);
                test_spi_dump_status("oneway master send fail slave", (TUYA_SPI_NUM_E)slave_port);
                break;
            }
            ret = test_spi_wait_event((TUYA_SPI_NUM_E)master_port,
                                      TUYA_SPI_EVENT_TX_COMPLETE,
                                      tx_evt_base,
                                      SPI_TEST_WAIT_TIMEOUT_MS,
                                      "oneway_master_wait_tx_done");
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] oneway wait master tx done timeout, loop=%u\r\n",
                             (unsigned int)loop);
                break;
            }

            wait_ms = 0U;
            while ((worker.recv_ok <= loop) && (worker.exited == 0U) && (wait_ms < SPI_TEST_WAIT_TIMEOUT_MS)) {
                tkl_system_sleep(SPI_TEST_POLL_INTERVAL_MS);
                wait_ms += SPI_TEST_POLL_INTERVAL_MS;
            }
            if (worker.recv_ok <= loop) {
                ret = OPRT_TIMEOUT;
                SPI_TEST_LOG("[xspi] oneway wait slave recv done timeout, loop=%u last_slave_ret=%d\r\n",
                             (unsigned int)loop, (int)worker.last_ret);
                test_spi_dump_status("oneway wait slave recv done timeout", (TUYA_SPI_NUM_E)slave_port);
                break;
            }
        } else {
            test_spi_dump_status("oneway before recv master", (TUYA_SPI_NUM_E)master_port);
            test_spi_dump_status("oneway before recv slave", (TUYA_SPI_NUM_E)slave_port);
            for (i = 0U; i < bytes; i++) {
                s_spi_master_rx[i] = 0U;
            }
            rx_evt_base = test_spi_evt_count_get((TUYA_SPI_NUM_E)master_port, TUYA_SPI_EVENT_RX_COMPLETE);
            ret = tkl_spi_recv((TUYA_SPI_NUM_E)master_port, s_spi_master_rx, bytes);
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] oneway master_recv failed, loop=%u ret=%d\r\n",
                             (unsigned int)loop, ret);
                test_spi_dump_status("oneway master recv fail master", (TUYA_SPI_NUM_E)master_port);
                test_spi_dump_status("oneway master recv fail slave", (TUYA_SPI_NUM_E)slave_port);
                break;
            }
            ret = test_spi_wait_event((TUYA_SPI_NUM_E)master_port,
                                      TUYA_SPI_EVENT_RX_COMPLETE,
                                      rx_evt_base,
                                      SPI_TEST_WAIT_TIMEOUT_MS,
                                      "oneway_master_wait_rx_done");
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] oneway wait master rx done timeout, loop=%u\r\n",
                             (unsigned int)loop);
                break;
            }
            wait_ms = 0U;
            while ((worker.send_ok <= loop) && (worker.exited == 0U) && (wait_ms < SPI_TEST_WAIT_TIMEOUT_MS)) {
                tkl_system_sleep(SPI_TEST_POLL_INTERVAL_MS);
                wait_ms += SPI_TEST_POLL_INTERVAL_MS;
            }
            if (worker.send_ok <= loop) {
                ret = OPRT_TIMEOUT;
                SPI_TEST_LOG("[xspi] oneway wait slave send done timeout, loop=%u last_slave_ret=%d\r\n",
                             (unsigned int)loop, (int)worker.last_ret);
                test_spi_dump_status("oneway wait slave send done timeout master", (TUYA_SPI_NUM_E)master_port);
                test_spi_dump_status("oneway wait slave send done timeout slave", (TUYA_SPI_NUM_E)slave_port);
                break;
            }

            for (i = 0U; i < bytes; i++) {
                uint8_t expected = test_spi_pattern(loop, i);
                if (s_spi_master_rx[i] != expected) {
                    master_mismatch++;
                    SPI_TEST_LOG("[xspi] oneway recv mismatch, loop=%u idx=%u rx=%02x exp=%02x\r\n",
                                 (unsigned int)loop,
                                 (unsigned int)i,
                                 (unsigned int)s_spi_master_rx[i],
                                 (unsigned int)expected);
                    ret = OPRT_COM_ERROR;
                    break;
                }
            }
            if (ret != OPRT_OK) {
                break;
            }
        }
    }

__EXIT:
    worker.stop_req = 1U;
    os_ret = test_spi_join_worker_with_timeout("oneway_exit", &slave_thread, &worker,
                                               (TUYA_SPI_NUM_E)master_port, (TUYA_SPI_NUM_E)slave_port);
    if ((os_ret != OPRT_OK) && (ret == OPRT_OK)) {
        ret = os_ret;
    }

    (void)tkl_spi_deinit((TUYA_SPI_NUM_E)master_port);

    if ((ret == OPRT_OK || ret == OPRT_TIMEOUT) &&
        (worker.last_ret != OPRT_OK) &&
        (worker.last_ret != SPI_WORKER_RET_PENDING)) {
        ret = worker.last_ret;
    }
    if ((ret == OPRT_OK) && ((worker.mismatch != 0U) || (master_mismatch != 0U))) {
        ret = OPRT_COM_ERROR;
    }

    SPI_TEST_LOG("[xspi] %s_result master_port=%u slave_port=%u dma_enabled=%u transfer_bytes=%u requested_loops=%u slave_receive_success_count=%u slave_send_success_count=%u slave_mismatch_count=%u master_mismatch_count=%u return_code=%d\r\n",
                 master_send ? "send" : "recv",
                 (unsigned int)master_port,
                 (unsigned int)slave_port,
                 (unsigned int)use_dma,
                 (unsigned int)bytes,
                 (unsigned int)loops,
                 (unsigned int)worker.recv_ok,
                 (unsigned int)worker.send_ok,
                 (unsigned int)worker.mismatch,
                 (unsigned int)master_mismatch,
                 ret);

    return ret;
}

static OPERATE_RET test_spi_single_io(uint32_t port, TUYA_SPI_ROLE_E role, bool do_send,
                                      uint32_t bytes, uint32_t loops,
                                      bool use_dma, bool verify_data)
{
    TUYA_SPI_BASE_CFG_T cfg = test_spi_build_cfg(role, use_dma);
    OPERATE_RET ret = OPRT_OK;
    uint32_t loop;
    uint32_t i;
    uint32_t tx_evt_base;
    uint32_t rx_evt_base;
    uint32_t mismatch = 0U;

    if (!test_spi_port_in_range(port)) {
        return OPRT_INVALID_PARM;
    }
    if ((bytes == 0U) || (bytes > SPI_TEST_MAX_XFER_BYTES) || (loops == 0U)) {
        return OPRT_INVALID_PARM;
    }

    (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);
    ret = tkl_spi_init((TUYA_SPI_NUM_E)port, &cfg);
    if (ret != OPRT_OK) {
        SPI_TEST_LOG("[xspi] single_io_init_failed port=%u role=%u dma_enabled=%u return_code=%d\r\n",
                     (unsigned int)port,
                     (unsigned int)role,
                     (unsigned int)use_dma,
                     ret);
        return ret;
    }

    ret = test_spi_enable_async_mode((TUYA_SPI_NUM_E)port);
    if (ret != OPRT_OK) {
        SPI_TEST_LOG("[xspi] single_io_async_enable_failed port=%u role=%u dma_enabled=%u return_code=%d\r\n",
                     (unsigned int)port,
                     (unsigned int)role,
                     (unsigned int)use_dma,
                     ret);
        (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);
        return ret;
    }

    SPI_TEST_DBG("io start p=%u role=%s op=%s dma=%u loops=%u bytes=%u verify=%u\r\n",
                 (unsigned int)port,
                 (role == TUYA_SPI_ROLE_MASTER) ? "master" : "slave",
                 do_send ? "send" : "recv",
                 (unsigned int)use_dma,
                 (unsigned int)loops,
                 (unsigned int)bytes,
                 (unsigned int)verify_data);

    for (loop = 0U; loop < loops; loop++) {
        if (do_send) {
            for (i = 0U; i < bytes; i++) {
                s_spi_master_tx[i] = test_spi_pattern(loop, i);
            }
            tx_evt_base = test_spi_evt_count_get((TUYA_SPI_NUM_E)port, TUYA_SPI_EVENT_TX_COMPLETE);
            ret = tkl_spi_send((TUYA_SPI_NUM_E)port, s_spi_master_tx, bytes);
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] io send failed, loop=%u ret=%d\r\n",
                             (unsigned int)loop, ret);
                test_spi_dump_status("io send fail", (TUYA_SPI_NUM_E)port);
                break;
            }
            ret = test_spi_wait_event((TUYA_SPI_NUM_E)port,
                                      TUYA_SPI_EVENT_TX_COMPLETE,
                                      tx_evt_base,
                                      SPI_TEST_WAIT_TIMEOUT_MS,
                                      "single_io_wait_tx_done");
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] io wait tx done timeout, loop=%u ret=%d\r\n",
                             (unsigned int)loop, ret);
                break;
            }
        } else {
            for (i = 0U; i < bytes; i++) {
                s_spi_master_rx[i] = 0U;
            }
            rx_evt_base = test_spi_evt_count_get((TUYA_SPI_NUM_E)port, TUYA_SPI_EVENT_RX_COMPLETE);
            ret = tkl_spi_recv((TUYA_SPI_NUM_E)port, s_spi_master_rx, bytes);
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] io recv failed, loop=%u ret=%d\r\n",
                             (unsigned int)loop, ret);
                test_spi_dump_status("io recv fail", (TUYA_SPI_NUM_E)port);
                break;
            }
            ret = test_spi_wait_event((TUYA_SPI_NUM_E)port,
                                      TUYA_SPI_EVENT_RX_COMPLETE,
                                      rx_evt_base,
                                      SPI_TEST_WAIT_TIMEOUT_MS,
                                      "single_io_wait_rx_done");
            if (ret != OPRT_OK) {
                SPI_TEST_LOG("[xspi] io wait rx done timeout, loop=%u ret=%d\r\n",
                             (unsigned int)loop, ret);
                break;
            }

            if (verify_data) {
                for (i = 0U; i < bytes; i++) {
                    uint8_t expected = test_spi_pattern(loop, i);
                    if (s_spi_master_rx[i] != expected) {
                        mismatch++;
                        SPI_TEST_LOG("[xspi] io recv mismatch, loop=%u idx=%u rx=%02x exp=%02x\r\n",
                                     (unsigned int)loop,
                                     (unsigned int)i,
                                     (unsigned int)s_spi_master_rx[i],
                                     (unsigned int)expected);
                        ret = OPRT_COM_ERROR;
                        break;
                    }
                }
                if (ret != OPRT_OK) {
                    break;
                }
            }
        }
    }

    (void)tkl_spi_deinit((TUYA_SPI_NUM_E)port);
    if ((ret == OPRT_OK) && (mismatch != 0U)) {
        ret = OPRT_COM_ERROR;
    }

    SPI_TEST_LOG("[xspi] single_io_result port=%u role=%s operation=%s dma_enabled=%u transfer_bytes=%u requested_loops=%u verify_enabled=%u mismatch_count=%u return_code=%d\r\n",
                 (unsigned int)port,
                 (role == TUYA_SPI_ROLE_MASTER) ? "master" : "slave",
                 do_send ? "send" : "recv",
                 (unsigned int)use_dma,
                 (unsigned int)bytes,
                 (unsigned int)loops,
                 (unsigned int)verify_data,
                 (unsigned int)mismatch,
                 ret);
    return ret;
}

static OPERATE_RET test_spi_full_xfer_matrix(uint32_t master_port, uint32_t slave_port)
{
    typedef struct {
        uint32_t bytes;
        bool expect_success;
    } spi_xfer_case_t;
    static const spi_xfer_case_t xfer_cases_nodma[] = {
        {32U, true},
        {252U, true},
        {253U, true},
        {4095U, true},
        {4096U, false},
    };
    static const spi_xfer_case_t xfer_cases_dma[] = {
        {32U, true},
        {252U, true},
        {253U, true},
        {4095U, true},
        {4096U, true},
        {4104U, true},
        {4105U, true},
        {8192U, true},
    };
    spi_stat_t st = {0};
    uint32_t dma_idx;
    uint32_t len_idx;
    char tc_name[96];
    OPERATE_RET ret;

    for (dma_idx = 0U; dma_idx < 2U; dma_idx++) {
        bool use_dma = (dma_idx != 0U);
        const spi_xfer_case_t *xfer_cases = use_dma ? xfer_cases_dma : xfer_cases_nodma;
        uint32_t xfer_count = use_dma ? (uint32_t)(sizeof(xfer_cases_dma) / sizeof(xfer_cases_dma[0]))
                                      : (uint32_t)(sizeof(xfer_cases_nodma) / sizeof(xfer_cases_nodma[0]));
        for (len_idx = 0U; len_idx < xfer_count; len_idx++) {
            uint32_t bytes = xfer_cases[len_idx].bytes;
            bool expect_success = xfer_cases[len_idx].expect_success;
            uint32_t loops = (bytes > 4096U) ? SPI_TEST_FULL_LARGE_LOOPS : SPI_TEST_FULL_SMALL_LOOPS;
            const char *align_tag = ((bytes & 0x3U) == 0U) ? "align4" : "unalign4";
            const char *range_tag = (bytes <= 4095U) ? "le4095" : "gt4095";
            bool pass;
            bool skip = false;

            ret = test_spi_oneway(master_port, slave_port, bytes, loops, use_dma, true);
            if (expect_success) {
                pass = (ret == OPRT_OK);
                skip = (ret == OPRT_TIMEOUT);
            } else {
                pass = ((ret != OPRT_OK) || (ret == OPRT_TIMEOUT));
                skip = (ret == OPRT_TIMEOUT);
            }
            snprintf(tc_name, sizeof(tc_name), "TF301 oneway_send_%s_%s_%s_len%u",
                     use_dma ? "dma" : "nodma",
                     align_tag,
                     range_tag,
                     (unsigned int)bytes);
            test_spi_stat_add(&st, tc_name, pass, skip, ret);

            if ((!use_dma) && (bytes >= 253U)) {
                /*
                 * bk7259 non-DMA slave->master long receive path is unstable and may
                 * trigger persistent tx_underflow/rx_overflow interrupts, so skip it.
                 */
                ret = OPRT_TIMEOUT;
            } else {
                ret = test_spi_oneway(master_port, slave_port, bytes, loops, use_dma, false);
            }
            if (expect_success) {
                pass = (ret == OPRT_OK);
                skip = (ret == OPRT_TIMEOUT);
            } else {
                pass = ((ret != OPRT_OK) || (ret == OPRT_TIMEOUT));
                skip = (ret == OPRT_TIMEOUT);
            }
            snprintf(tc_name, sizeof(tc_name), "TF302 oneway_recv_%s_%s_%s_len%u",
                     use_dma ? "dma" : "nodma",
                     align_tag,
                     range_tag,
                     (unsigned int)bytes);
            test_spi_stat_add(&st, tc_name, pass, skip, ret);
        }
    }

    test_spi_stat_summary(&st, "full_xfer_matrix");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

static OPERATE_RET test_spi_full(void)
{
    spi_stat_t st = {0};
    OPERATE_RET ret;
    uint32_t port;
    uint32_t hw_ports = test_spi_hw_port_num();
    char tc_name[64];

    ret = test_spi_api_all();
    test_spi_stat_add(&st, "TF001 api_all", (ret == OPRT_OK), false, ret);

    for (port = 0U; port < hw_ports; port++) {
        ret = test_spi_param(port);
        snprintf(tc_name, sizeof(tc_name), "TF101 param_check_port%u", (unsigned int)port);
        test_spi_stat_add(&st, tc_name, (ret == OPRT_OK), false, ret);

        ret = test_spi_cfg_matrix(port, false);
        snprintf(tc_name, sizeof(tc_name), "TF103 cfg_matrix_nodma_port%u", (unsigned int)port);
        test_spi_stat_add(&st, tc_name, (ret == OPRT_OK), false, ret);

        ret = test_spi_cfg_matrix(port, true);
        snprintf(tc_name, sizeof(tc_name), "TF104 cfg_matrix_dma_port%u", (unsigned int)port);
        test_spi_stat_add(&st, tc_name, (ret == OPRT_OK), false, ret);

        ret = test_spi_stress(port, SPI_TEST_DEFAULT_STRESS_LOOPS);
        snprintf(tc_name, sizeof(tc_name), "TF102 repeat_init_deinit_port%u", (unsigned int)port);
        test_spi_stat_add(&st, tc_name, (ret == OPRT_OK), false, ret);
    }

    if (hw_ports >= 2U) {
        ret = test_spi_dual_api(0U, 1U, SPI_TEST_DEFAULT_DUAL_LOOPS);
        test_spi_stat_add(&st, "TF201 dual_port_parallel_api", (ret == OPRT_OK), false, ret);

        ret = test_spi_full_xfer_matrix(0U, 1U);
        test_spi_stat_add(&st, "TF202 xfer_matrix_master_slave_dma_align_len", (ret == OPRT_OK), false, ret);
    } else {
        test_spi_stat_add(&st, "TF201 dual_port_parallel_api", true, true, OPRT_NOT_SUPPORTED);
        test_spi_stat_add(&st, "TF202 xfer_matrix_master_slave_dma_align_len", true, true, OPRT_NOT_SUPPORTED);
    }

    test_spi_stat_summary(&st, "full");
    return (st.fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

void cli_tkl_spi_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    const char *cmd;
    OPERATE_RET ret;
    uint32_t port = (uint32_t)SPI_TEST_DEFAULT_MASTER_PORT;
    uint32_t master_port = (uint32_t)SPI_TEST_DEFAULT_MASTER_PORT;
    uint32_t slave_port = (uint32_t)SPI_TEST_DEFAULT_SLAVE_PORT;
    uint32_t bytes = SPI_TEST_DEFAULT_BYTES;
    uint32_t loops = SPI_TEST_DEFAULT_LOOPS;
    uint32_t dma = 0U;
    uint32_t port0 = 0U;
    uint32_t port1 = 1U;
    TUYA_SPI_ROLE_E role = TUYA_SPI_ROLE_MASTER;
    bool io_send = true;
    uint32_t verify = 0U;

    if ((pcWriteBuffer == NULL) || (xWriteBufferLen <= 0)) {
        return;
    }

    /* Route SPI pads before any init. */
    test_spi_pinmux_setup();

    if (argc < 2) {
        test_spi_usage(pcWriteBuffer, xWriteBufferLen);
        return;
    }

    cmd = argv[1];
    if (os_strcmp(cmd, "list") == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi list logical_port=[0..%u] note=\"dual-port need jumper for send/recv/dualrw/loop; single-port external test use: xspi io [port] [master/slave] [send/recv] ...\"",
                 (unsigned int)(test_spi_hw_port_num() - 1U));
        return;
    }

    if (os_strcmp(cmd, "api") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            dma = test_spi_parse_dma_arg(argv[3]);
        }
        if (!test_spi_port_in_range(port)) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xspi api failed (%d), invalid port=%u valid=[0..%u]\r\n",
                     OPRT_INVALID_PARM,
                     (unsigned int)port,
                     (unsigned int)(test_spi_hw_port_num() - 1U));
            return;
        }
        ret = test_spi_api(port, dma != 0U);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi api result=%s return_code=%d port=%u dma_enabled=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port,
                 (unsigned int)dma);
        return;
    }

    if (os_strcmp(cmd, "api_all") == 0) {
        ret = test_spi_api_all();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi api_all result=%s return_code=%d\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
        return;
    }

    if (os_strcmp(cmd, "param") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        ret = test_spi_param(port);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi param result=%s return_code=%d port=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port);
        return;
    }

    if (os_strcmp(cmd, "stress") == 0) {
        loops = SPI_TEST_DEFAULT_STRESS_LOOPS;
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            loops = os_strtoul(argv[3], NULL, 10);
        }
        ret = test_spi_stress(port, loops);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi stress result=%s return_code=%d port=%u requested_loops=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port,
                 (unsigned int)loops);
        return;
    }

    if (os_strcmp(cmd, "dualapi") == 0) {
        loops = SPI_TEST_DEFAULT_DUAL_LOOPS;
        if (argc >= 3) {
            port0 = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            port1 = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            loops = os_strtoul(argv[4], NULL, 10);
        }
        ret = test_spi_dual_api(port0, port1, loops);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi dualapi result=%s return_code=%d port0=%u port1=%u requested_loops=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port0,
                 (unsigned int)port1,
                 (unsigned int)loops);
        return;
    }

    if (os_strcmp(cmd, "dualrw") == 0) {
        if (argc >= 3) {
            master_port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            slave_port = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            bytes = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            loops = os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            dma = test_spi_parse_dma_arg(argv[6]);
        }
        ret = test_spi_loopback(master_port, slave_port, bytes, loops, dma != 0U, false);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi dualrw result=%s return_code=%d master_port=%u slave_port=%u transfer_bytes=%u requested_loops=%u dma_enabled=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)master_port,
                 (unsigned int)slave_port,
                 (unsigned int)bytes,
                 (unsigned int)loops,
                 (unsigned int)dma);
        return;
    }

    if (os_strcmp(cmd, "io") == 0) {
        if (argc >= 3) {
            port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            if (!test_spi_parse_role_arg(argv[3], &role)) {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xspi io failed (%d), invalid role=%s expect=master/slave\r\n",
                         OPRT_INVALID_PARM, argv[3]);
                return;
            }
        }
        if (argc >= 5) {
            if ((os_strcmp(argv[4], "send") == 0) || (os_strcmp(argv[4], "tx") == 0)) {
                io_send = true;
            } else if ((os_strcmp(argv[4], "recv") == 0) || (os_strcmp(argv[4], "rx") == 0)) {
                io_send = false;
            } else {
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "xspi io failed (%d), invalid op=%s expect=send/recv\r\n",
                         OPRT_INVALID_PARM, argv[4]);
                return;
            }
        }
        if (argc >= 6) {
            bytes = os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            loops = os_strtoul(argv[6], NULL, 10);
        }
        if (argc >= 8) {
            dma = test_spi_parse_dma_arg(argv[7]);
        }
        if (argc >= 9) {
            verify = os_strtoul(argv[8], NULL, 10);
        }
        if (!test_spi_port_in_range(port)) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "xspi io failed (%d), invalid port=%u valid=[0..%u]\r\n",
                     OPRT_INVALID_PARM,
                     (unsigned int)port,
                     (unsigned int)(test_spi_hw_port_num() - 1U));
            return;
        }

        ret = test_spi_single_io(port, role, io_send, bytes, loops, dma != 0U, verify != 0U);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi io result=%s return_code=%d port=%u role=%s operation=%s transfer_bytes=%u requested_loops=%u dma_enabled=%u verify_enabled=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)port,
                 (role == TUYA_SPI_ROLE_MASTER) ? "master" : "slave",
                 io_send ? "send" : "recv",
                 (unsigned int)bytes,
                 (unsigned int)loops,
                 (unsigned int)dma,
                 (unsigned int)verify);
        return;
    }

    if (os_strcmp(cmd, "send") == 0 || os_strcmp(cmd, "tx") == 0) {
        if (argc >= 3) {
            master_port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            slave_port = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            bytes = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            loops = os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            dma = test_spi_parse_dma_arg(argv[6]);
        }
        ret = test_spi_oneway(master_port, slave_port, bytes, loops, dma != 0U, true);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi send result=%s return_code=%d master_port=%u slave_port=%u transfer_bytes=%u requested_loops=%u dma_enabled=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)master_port,
                 (unsigned int)slave_port,
                 (unsigned int)bytes,
                 (unsigned int)loops,
                 (unsigned int)dma);
        return;
    }

    if (os_strcmp(cmd, "recv") == 0 || os_strcmp(cmd, "rx") == 0) {
        if (argc >= 3) {
            master_port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            slave_port = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            bytes = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            loops = os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            dma = test_spi_parse_dma_arg(argv[6]);
        }
        ret = test_spi_oneway(master_port, slave_port, bytes, loops, dma != 0U, false);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi recv result=%s return_code=%d master_port=%u slave_port=%u transfer_bytes=%u requested_loops=%u dma_enabled=%u\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)master_port,
                 (unsigned int)slave_port,
                 (unsigned int)bytes,
                 (unsigned int)loops,
                 (unsigned int)dma);
        return;
    }

    if (os_strcmp(cmd, "loop") == 0 || os_strcmp(cmd, "loopback") == 0) {
        if (argc >= 3) {
            master_port = os_strtoul(argv[2], NULL, 10);
        }
        if (argc >= 4) {
            slave_port = os_strtoul(argv[3], NULL, 10);
        }
        if (argc >= 5) {
            bytes = os_strtoul(argv[4], NULL, 10);
        }
        if (argc >= 6) {
            loops = os_strtoul(argv[5], NULL, 10);
        }
        if (argc >= 7) {
            dma = test_spi_parse_dma_arg(argv[6]);
        }

        ret = test_spi_loopback(master_port, slave_port, bytes, loops, dma != 0U, true);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi loop result=%s return_code=%d master_port=%u slave_port=%u transfer_bytes=%u requested_loops=%u dma_enabled=%u hint=\"need cross-wire between two SPI ports\"\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret,
                 (unsigned int)master_port,
                 (unsigned int)slave_port,
                 (unsigned int)bytes,
                 (unsigned int)loops,
                 (unsigned int)dma);
        return;
    }

    if (os_strcmp(cmd, "full") == 0) {
        ret = test_spi_full();
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "xspi full result=%s return_code=%d\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed",
                 ret);
        return;
    }

    test_spi_usage(pcWriteBuffer, xWriteBufferLen);
}
