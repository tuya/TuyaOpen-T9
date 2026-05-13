/**
 * @file test_i2c.c
 * @brief I2C self-test for T9 (BK7259 AP); external entry is cli_xi2c_cmd only.
 * @version 1.0
 * @date 2026-05-13
 *
 * CLI (registered name "xi2c"):
 *   xi2c single [port]
 *   xi2c smoke [port] [loops]
 *   xi2c full [master_port] [slave_port] [slave_addr_hex] [loops] [bytes]
 *   xi2c pajid [port] [dev_addr_hex]
 *
 * Loopback cases are disabled in current suite.
 *
 * Log format:
 *   [xi2c] PASS|FAIL|SKIP test_name=<item> return_code=...
 * End banner:
 *   TOTAL PASS/FAIL/SKIP COUNT and FINAL RESULT in uppercase labels.
 */

#include "cli.h"
#include "tkl_i2c.h"
#include "tkl_system.h"

#include <os/mem.h>
#include <os/os.h>
#include <soc/soc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern void bk_printf(const char *fmt, ...);

#define I2C_SMOKE_LOOPS_DEFAULT       300U
#define I2C_SMOKE_LOOPS_MAX           50000U
#define I2C_LOOPBACK_LOOPS_DEFAULT    20U
#define I2C_LOOPBACK_LOOPS_MAX        2000U
#define I2C_LOOPBACK_BYTES_DEFAULT    16U
#define I2C_LOOPBACK_BYTES_MAX        256U
#define I2C_DEFAULT_SLAVE_ADDR        0x42U
#define PAJ7620_DEFAULT_ADDR          0x73U
#define PAJ7620_REG_BANK_SEL          0xEFU
#define PAJ7620_BANK_0                0x00U
#define PAJ7620_REG_ID_L              0x00U
#define PAJ7620_REG_ID_H              0x01U
#define PAJ7620_EXPECT_ID_L           0x20U
#define PAJ7620_EXPECT_ID_H           0x76U
#define I2C_WAIT_MS                   10000U
#define I2C_POLL_INTERVAL_MS          2U
#define I2C_SLAVE_ARM_WAIT_MS         10U

static UINT32_T s_pass;
static UINT32_T s_fail;
static UINT32_T s_skip;
static UINT8_T s_loopback_case_enable = 0U;

static UINT8_T s_loop_slave_rx[I2C_LOOPBACK_BYTES_MAX];
static UINT8_T s_loop_slave_tx[I2C_LOOPBACK_BYTES_MAX];
static UINT8_T s_loop_master_tx[I2C_LOOPBACK_BYTES_MAX];
static UINT8_T s_loop_master_rx[I2C_LOOPBACK_BYTES_MAX];

typedef struct {
    TUYA_I2C_NUM_E slave_port;
    UINT16_T slave_addr;
    UINT32_T bytes;
    UINT32_T loops;
    volatile UINT32_T stop_req;
    volatile UINT32_T started;
    volatile UINT32_T exited;
    volatile UINT32_T ready_seq;
    volatile UINT32_T resp_ready_seq;
    volatile UINT32_T recv_ok;
    volatile UINT32_T send_ok;
    volatile UINT32_T data_mismatch;
    volatile OPERATE_RET last_ret;
} I2C_LOOP_WORKER_T;

typedef struct {
    TUYA_I2C_NUM_E port;
    UINT32_T loops;
    beken_semaphore_t *start_sem;
    beken_semaphore_t *done_sem;
    volatile UINT32_T pass;
    volatile UINT32_T fail;
    volatile OPERATE_RET last_ret;
} I2C_DUAL_WORKER_T;

static void test_i2c_dummy_irq_cb(TUYA_I2C_NUM_E port, TUYA_IIC_IRQ_EVT_E event)
{
    (void)port;
    (void)event;
}

static UINT32_T test_i2c_hw_port_num(void)
{
#if defined(SOC_I2C_UNIT_NUM)
    if ((UINT32_T)SOC_I2C_UNIT_NUM < (UINT32_T)TUYA_I2C_NUM_MAX) {
        return (UINT32_T)SOC_I2C_UNIT_NUM;
    }
#endif
    return (UINT32_T)TUYA_I2C_NUM_MAX;
}

static int test_i2c_port_ok(int port)
{
    return ((port >= 0) && ((UINT32_T)port < test_i2c_hw_port_num()));
}

static void test_i2c_stats_reset(void)
{
    s_pass = 0;
    s_fail = 0;
    s_skip = 0;
}

static void test_i2c_item_pass_ret(const char *test_name, OPERATE_RET ret)
{
    s_pass++;
    bk_printf("[xi2c] PASS test_name=%s return_code=%d\r\n", test_name, (int)ret);
}

static void test_i2c_item_pass_plain(const char *test_name)
{
    s_pass++;
    bk_printf("[xi2c] PASS test_name=%s\r\n", test_name);
}

static void test_i2c_item_fail_ret(const char *test_name, OPERATE_RET got, OPERATE_RET want)
{
    s_fail++;
    bk_printf("[xi2c] FAIL test_name=%s return_code=%d expect_return_code=%d\r\n", test_name, (int)got,
              (int)want);
}

static void test_i2c_item_fail_msg(const char *test_name, const char *detail)
{
    s_fail++;
    bk_printf("[xi2c] FAIL test_name=%s detail=%s\r\n", test_name, detail);
}

static void test_i2c_item_skip_reason(const char *test_name, const char *reason)
{
    s_skip++;
    bk_printf("[xi2c] SKIP test_name=%s reason=%s\r\n", test_name, reason);
}

static void test_i2c_stats_print(const char *suite_tag)
{
    const char *final = (s_fail == 0U) ? "PASS" : "FAIL";

    bk_printf("\r\n");
    bk_printf("****************************************\r\n");
    bk_printf("*      I2C SELF-TEST SUMMARY (END)     *\r\n");
    bk_printf("****************************************\r\n");
    bk_printf("SUITE TAG:          %s\r\n", suite_tag);
    bk_printf("TOTAL PASS COUNT:  %8u\r\n", (unsigned)s_pass);
    bk_printf("TOTAL FAIL COUNT:  %8u\r\n", (unsigned)s_fail);
    bk_printf("TOTAL SKIP COUNT:  %8u\r\n", (unsigned)s_skip);
    bk_printf("FINAL RESULT:      %s\r\n", final);
    bk_printf("****************************************\r\n");
}

static void test_i2c_expect(const char *test_name, OPERATE_RET got, OPERATE_RET want)
{
    if (got == want) {
        test_i2c_item_pass_ret(test_name, got);
    } else {
        test_i2c_item_fail_ret(test_name, got, want);
    }
}

static void test_i2c_expect_ok_or_ns(const char *test_name, OPERATE_RET got)
{
    if ((got == OPRT_OK) || (got == OPRT_NOT_SUPPORTED)) {
        if (got == OPRT_NOT_SUPPORTED) {
            test_i2c_item_skip_reason(test_name, "not supported");
        } else {
            test_i2c_item_pass_ret(test_name, got);
        }
    } else {
        test_i2c_item_fail_msg(test_name, "expect OPRT_OK or OPRT_NOT_SUPPORTED");
    }
}

static void test_i2c_skip(const char *test_name, const char *reason)
{
    test_i2c_item_skip_reason(test_name, reason);
}

static TUYA_IIC_BASE_CFG_T test_i2c_build_cfg(TUYA_IIC_ROLE_E role, TUYA_IIC_SPEED_E speed,
                                               TUYA_IIC_ADDR_MODE_E addr_width)
{
    TUYA_IIC_BASE_CFG_T cfg;
    (void)os_memset(&cfg, 0, sizeof(cfg));
    cfg.role = role;
    cfg.speed = speed;
    cfg.addr_width = addr_width;
    return cfg;
}

static UINT8_T test_i2c_pattern(UINT32_T loop, UINT32_T idx)
{
    return (UINT8_T)((0x5AU + loop * 17U + idx * 3U) & 0xFFU);
}

static UINT8_T test_i2c_resp_from_req(UINT8_T req)
{
    return (UINT8_T)(req ^ 0xA5U);
}

static OPERATE_RET test_i2c_reg_write8(TUYA_I2C_NUM_E port, UINT16_T dev_addr, UINT8_T reg, UINT8_T val)
{
    UINT8_T buf[2];
    buf[0] = reg;
    buf[1] = val;
    return tkl_i2c_master_send(port, dev_addr, buf, sizeof(buf), FALSE);
}

static OPERATE_RET test_i2c_reg_read8(TUYA_I2C_NUM_E port, UINT16_T dev_addr, UINT8_T reg, UINT8_T *out)
{
    OPERATE_RET ret;

    if (out == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_i2c_master_send(port, dev_addr, &reg, 1U, TRUE);
    if (ret != OPRT_OK) {
        bk_printf("[xi2c] INFO test_name=PAJ_READ_STEP step=send_reg_pending port=%d dev_addr=0x%02x reg=0x%02x ret=%d\r\n",
                  (int)port, (unsigned)dev_addr, (unsigned)reg, (int)ret);
        return ret;
    }

    ret = tkl_i2c_master_receive(port, dev_addr, out, 1U, FALSE);
    if (ret != OPRT_OK) {
        bk_printf("[xi2c] INFO test_name=PAJ_READ_STEP step=recv_data port=%d dev_addr=0x%02x reg=0x%02x ret=%d\r\n",
                  (int)port, (unsigned)dev_addr, (unsigned)reg, (int)ret);
    }
    return ret;
}

static OPERATE_RET test_i2c_case_pajid(TUYA_I2C_NUM_E port, UINT16_T dev_addr)
{
    TUYA_IIC_BASE_CFG_T cfg;
    REGADDR_WIDTH_T reg_cfg;
    OPERATE_RET ret;
    UINT8_T id_l = 0U;
    UINT8_T id_h = 0U;

    (void)tkl_i2c_deinit(port);
    cfg = test_i2c_build_cfg(TUYA_IIC_MODE_MASTER, TUYA_IIC_BUS_SPEED_100K, TUYA_IIC_ADDRESS_7BIT);
    ret = tkl_i2c_init(port, &cfg);
    if (ret != OPRT_OK) {
        test_i2c_item_fail_ret("PAJ00_init", ret, OPRT_OK);
        return ret;
    }

    reg_cfg.dev_addr = dev_addr;
    reg_cfg.reg_addr_width = 1U;
    ret = tkl_i2c_ioctl(port, I2C_IOCTL_SET_REGADDR_WIDTH, &reg_cfg);
    if (ret != OPRT_OK) {
        test_i2c_item_fail_ret("PAJ00_set_regw8", ret, OPRT_OK);
        (void)tkl_i2c_deinit(port);
        return ret;
    }
    test_i2c_item_pass_ret("PAJ00_set_regw8", ret);

    ret = test_i2c_reg_write8(port, dev_addr, PAJ7620_REG_BANK_SEL, PAJ7620_BANK_0);
    if (ret != OPRT_OK) {
        test_i2c_item_fail_ret("PAJ01_select_bank0", ret, OPRT_OK);
        (void)tkl_i2c_deinit(port);
        return ret;
    }
    test_i2c_item_pass_ret("PAJ01_select_bank0", ret);

    ret = test_i2c_reg_read8(port, dev_addr, PAJ7620_REG_ID_L, &id_l);
    if (ret != OPRT_OK) {
        test_i2c_item_fail_ret("PAJ02_read_id_l", ret, OPRT_OK);
        (void)tkl_i2c_deinit(port);
        return ret;
    }
    test_i2c_item_pass_ret("PAJ02_read_id_l", ret);

    ret = test_i2c_reg_read8(port, dev_addr, PAJ7620_REG_ID_H, &id_h);
    if (ret != OPRT_OK) {
        test_i2c_item_fail_ret("PAJ03_read_id_h", ret, OPRT_OK);
        (void)tkl_i2c_deinit(port);
        return ret;
    }
    test_i2c_item_pass_ret("PAJ03_read_id_h", ret);

    bk_printf("[xi2c] INFO test_name=PAJ_ID_VALUE id_l=0x%02x id_h=0x%02x expect_l=0x%02x expect_h=0x%02x\r\n",
              (unsigned)id_l, (unsigned)id_h, (unsigned)PAJ7620_EXPECT_ID_L, (unsigned)PAJ7620_EXPECT_ID_H);

    if ((id_l == PAJ7620_EXPECT_ID_L) && (id_h == PAJ7620_EXPECT_ID_H)) {
        test_i2c_item_pass_plain("PAJ04_id_match");
        ret = OPRT_OK;
    } else {
        test_i2c_item_fail_msg("PAJ04_id_match", "id mismatch");
        ret = OPRT_COM_ERROR;
    }

    (void)tkl_i2c_deinit(port);
    return ret;
}

/* @note Loopback case requires wiring in file header. */
static void test_i2c_loop_slave_worker(beken_thread_arg_t arg)
{
    I2C_LOOP_WORKER_T *ctx = (I2C_LOOP_WORKER_T *)arg;
    TUYA_IIC_BASE_CFG_T cfg;
    UINT32_T loop;
    UINT32_T i;
    OPERATE_RET ret;

    if (ctx == NULL) {
        rtos_delete_thread(NULL);
        return;
    }

    ctx->last_ret = OPRT_OK;
    cfg = test_i2c_build_cfg(TUYA_IIC_MODE_SLAVE, TUYA_IIC_BUS_SPEED_100K, TUYA_IIC_ADDRESS_7BIT);

    ret = tkl_i2c_init(ctx->slave_port, &cfg);
    if (ret != OPRT_OK) {
        ctx->last_ret = ret;
        ctx->exited = 1U;
        rtos_delete_thread(NULL);
        return;
    }

    ret = tkl_i2c_set_slave_addr(ctx->slave_port, ctx->slave_addr);
    if (ret != OPRT_OK) {
        ctx->last_ret = ret;
        (void)tkl_i2c_deinit(ctx->slave_port);
        ctx->exited = 1U;
        rtos_delete_thread(NULL);
        return;
    }

    ctx->started = 1U;
    for (loop = 0; loop < ctx->loops && ctx->stop_req == 0U; loop++) {
        ctx->ready_seq = loop + 1U;
        ret = tkl_i2c_slave_receive(ctx->slave_port, s_loop_slave_rx, ctx->bytes);
        if (ret != OPRT_OK) {
            ctx->last_ret = ret;
            break;
        }
        ctx->recv_ok++;

        for (i = 0; i < ctx->bytes; i++) {
            UINT8_T expected = test_i2c_pattern(loop, i);
            if (s_loop_slave_rx[i] != expected) {
                ctx->data_mismatch++;
            }
            s_loop_slave_tx[i] = test_i2c_resp_from_req(s_loop_slave_rx[i]);
        }

        ctx->resp_ready_seq = loop + 1U;
        ret = tkl_i2c_slave_send(ctx->slave_port, s_loop_slave_tx, ctx->bytes);
        if (ret != OPRT_OK) {
            ctx->last_ret = ret;
            break;
        }
        ctx->send_ok++;
    }

    (void)tkl_i2c_deinit(ctx->slave_port);
    ctx->exited = 1U;
    rtos_delete_thread(NULL);
}

static OPERATE_RET test_i2c_case_cfg_matrix(TUYA_I2C_NUM_E port)
{
    static const TUYA_IIC_ROLE_E role_list[] = {
        TUYA_IIC_MODE_MASTER,
        TUYA_IIC_MODE_SLAVE,
    };
    static const TUYA_IIC_SPEED_E speed_list[] = {
        TUYA_IIC_BUS_SPEED_100K,
        TUYA_IIC_BUS_SPEED_400K,
        TUYA_IIC_BUS_SPEED_1M,
        TUYA_IIC_BUS_SPEED_3_4M,
    };
    static const TUYA_IIC_ADDR_MODE_E addr_list[] = {
        TUYA_IIC_ADDRESS_7BIT,
        TUYA_IIC_ADDRESS_10BIT,
    };
    UINT32_T r;
    UINT32_T s;
    UINT32_T a;
    char test_name[48];

    for (r = 0; r < (UINT32_T)(sizeof(role_list) / sizeof(role_list[0])); r++) {
        for (s = 0; s < (UINT32_T)(sizeof(speed_list) / sizeof(speed_list[0])); s++) {
            for (a = 0; a < (UINT32_T)(sizeof(addr_list) / sizeof(addr_list[0])); a++) {
                TUYA_IIC_BASE_CFG_T cfg = test_i2c_build_cfg(role_list[r], speed_list[s], addr_list[a]);
                OPERATE_RET ret = tkl_i2c_init(port, &cfg);

                (void)snprintf(test_name, sizeof(test_name), "M01_r%u_s%u_a%u", (unsigned)r, (unsigned)s,
                               (unsigned)a);

                if (speed_list[s] == TUYA_IIC_BUS_SPEED_3_4M) {
                    test_i2c_expect(test_name, ret, OPRT_NOT_SUPPORTED);
                } else if (ret == OPRT_OK) {
                    test_i2c_item_pass_ret(test_name, ret);
                    (void)tkl_i2c_deinit(port);
                } else if (ret == OPRT_NOT_SUPPORTED) {
                    test_i2c_skip(test_name, "mode not supported");
                } else {
                    test_i2c_item_fail_msg(test_name, "unexpected return in cfg matrix");
                }
            }
        }
    }

    return OPRT_OK;
}

static void test_i2c_run_single_core(TUYA_I2C_NUM_E port)
{
    TUYA_IIC_BASE_CFG_T cfg;
    TUYA_IIC_STATUS_T status;
    REGADDR_WIDTH_T reg_cfg;
    UINT8_T one = 0x5A;
    OPERATE_RET ret;

    (void)tkl_i2c_deinit(port);

    cfg = test_i2c_build_cfg(TUYA_IIC_MODE_MASTER, TUYA_IIC_BUS_SPEED_100K, TUYA_IIC_ADDRESS_7BIT);
    test_i2c_expect("P01_invalid_port", tkl_i2c_init(TUYA_I2C_NUM_MAX, &cfg), OPRT_INVALID_PARM);
    test_i2c_expect("P02_null_cfg", tkl_i2c_init(port, NULL), OPRT_INVALID_PARM);

    (void)os_memset(&status, 0, sizeof(status));
    test_i2c_expect("P03_send_not_ready", tkl_i2c_master_send(port, I2C_DEFAULT_SLAVE_ADDR, &one, 1U, FALSE),
                    OPRT_RESOURCE_NOT_READY);
    test_i2c_expect("P03_recv_not_ready", tkl_i2c_master_receive(port, I2C_DEFAULT_SLAVE_ADDR, &one, 1U, FALSE),
                    OPRT_RESOURCE_NOT_READY);
    test_i2c_expect("P03_slave_send_not_ready", tkl_i2c_slave_send(port, &one, 1U), OPRT_RESOURCE_NOT_READY);
    test_i2c_expect("P03_slave_recv_not_ready", tkl_i2c_slave_receive(port, &one, 1U), OPRT_RESOURCE_NOT_READY);
    test_i2c_expect("P03_status_not_ready", tkl_i2c_get_status(port, &status), OPRT_RESOURCE_NOT_READY);
    test_i2c_expect("P03_reset_not_ready", tkl_i2c_reset(port), OPRT_RESOURCE_NOT_READY);
    test_i2c_expect("P03_irq_en_not_ready", tkl_i2c_irq_enable(port), OPRT_RESOURCE_NOT_READY);
    test_i2c_expect("P03_irq_dis_not_ready", tkl_i2c_irq_disable(port), OPRT_RESOURCE_NOT_READY);

    test_i2c_expect("P04_irq_init_null_cb", tkl_i2c_irq_init(port, NULL), OPRT_INVALID_PARM);

    ret = tkl_i2c_init(port, &cfg);
    test_i2c_expect("P05_init_master", ret, OPRT_OK);
    if (ret != OPRT_OK) {
        return;
    }
    test_i2c_expect("P05_init_master_repeat", tkl_i2c_init(port, &cfg), OPRT_OK);

    ret = tkl_i2c_irq_init(port, test_i2c_dummy_irq_cb);
    test_i2c_expect_ok_or_ns("P06_irq_init_cb", ret);

    reg_cfg.dev_addr = I2C_DEFAULT_SLAVE_ADDR;
    reg_cfg.reg_addr_width = 3U;
    test_i2c_expect("P07_ioctl_width3", tkl_i2c_ioctl(port, I2C_IOCTL_SET_REGADDR_WIDTH, &reg_cfg),
                    OPRT_NOT_SUPPORTED);
    reg_cfg.reg_addr_width = 1U;
    test_i2c_expect("P07_ioctl_width1", tkl_i2c_ioctl(port, I2C_IOCTL_SET_REGADDR_WIDTH, &reg_cfg), OPRT_OK);
    reg_cfg.reg_addr_width = 2U;
    test_i2c_expect("P07_ioctl_width2", tkl_i2c_ioctl(port, I2C_IOCTL_SET_REGADDR_WIDTH, &reg_cfg), OPRT_OK);
    test_i2c_expect("P07_ioctl_unknown", tkl_i2c_ioctl(port, 0xFFFFFFFFU, &reg_cfg), OPRT_NOT_SUPPORTED);

    test_i2c_expect("P08_recv_pending_true", tkl_i2c_master_receive(port, I2C_DEFAULT_SLAVE_ADDR, &one, 1U, TRUE),
                    OPRT_NOT_SUPPORTED);
    test_i2c_expect("P09_size0_send", tkl_i2c_master_send(port, I2C_DEFAULT_SLAVE_ADDR, &one, 0U, FALSE),
                    OPRT_INVALID_PARM);
    test_i2c_expect("P09_size0_recv", tkl_i2c_master_receive(port, I2C_DEFAULT_SLAVE_ADDR, &one, 0U, FALSE),
                    OPRT_INVALID_PARM);
    test_i2c_expect("P09_size0_slave_send", tkl_i2c_slave_send(port, &one, 0U), OPRT_INVALID_PARM);
    test_i2c_expect("P09_size0_slave_recv", tkl_i2c_slave_receive(port, &one, 0U), OPRT_INVALID_PARM);

    test_i2c_expect("P10_slave_addr_400", tkl_i2c_set_slave_addr(port, 0x400U), OPRT_INVALID_PARM);
    ret = tkl_i2c_set_slave_addr(port, I2C_DEFAULT_SLAVE_ADDR);
    test_i2c_expect_ok_or_ns("P10_slave_addr_ok", ret);

    test_i2c_expect("P11_status_after_init", tkl_i2c_get_status(port, &status), OPRT_OK);
    test_i2c_expect("P11_reset_after_init", tkl_i2c_reset(port), OPRT_OK);

    test_i2c_expect("P12_deinit", tkl_i2c_deinit(port), OPRT_OK);
    test_i2c_expect("P12_deinit_repeat", tkl_i2c_deinit(port), OPRT_OK);

    cfg = test_i2c_build_cfg(TUYA_IIC_MODE_MASTER, TUYA_IIC_BUS_SPEED_3_4M, TUYA_IIC_ADDRESS_7BIT);
    test_i2c_expect("P13_speed_3_4m", tkl_i2c_init(port, &cfg), OPRT_NOT_SUPPORTED);
}

static void test_i2c_dual_basic_worker(beken_thread_arg_t arg)
{
    I2C_DUAL_WORKER_T *ctx = (I2C_DUAL_WORKER_T *)arg;
    TUYA_IIC_BASE_CFG_T cfg;
    TUYA_IIC_STATUS_T status;
    UINT32_T i;
    OPERATE_RET ret;

    if ((ctx == NULL) || (ctx->start_sem == NULL) || (ctx->done_sem == NULL)) {
        rtos_delete_thread(NULL);
        return;
    }

    if (rtos_get_semaphore(ctx->start_sem, BEKEN_WAIT_FOREVER) != kNoErr) {
        ctx->last_ret = OPRT_TIMEOUT;
        (void)rtos_set_semaphore(ctx->done_sem);
        rtos_delete_thread(NULL);
        return;
    }

    cfg = test_i2c_build_cfg(TUYA_IIC_MODE_MASTER, TUYA_IIC_BUS_SPEED_100K, TUYA_IIC_ADDRESS_7BIT);
    for (i = 0U; i < ctx->loops; i++) {
        ret = tkl_i2c_init(ctx->port, &cfg);
        if (ret != OPRT_OK) {
            ctx->fail++;
            ctx->last_ret = ret;
            continue;
        }

        ret = tkl_i2c_get_status(ctx->port, &status);
        if (ret != OPRT_OK) {
            ctx->fail++;
            ctx->last_ret = ret;
            (void)tkl_i2c_deinit(ctx->port);
            continue;
        }

        ret = tkl_i2c_reset(ctx->port);
        if (ret != OPRT_OK) {
            ctx->fail++;
            ctx->last_ret = ret;
            (void)tkl_i2c_deinit(ctx->port);
            continue;
        }

        ret = tkl_i2c_deinit(ctx->port);
        if (ret != OPRT_OK) {
            ctx->fail++;
            ctx->last_ret = ret;
            continue;
        }
        ctx->pass++;
    }

    (void)rtos_set_semaphore(ctx->done_sem);
    rtos_delete_thread(NULL);
}

static OPERATE_RET test_i2c_case_dual_port_parallel(TUYA_I2C_NUM_E p0, TUYA_I2C_NUM_E p1, UINT32_T loops)
{
    beken_thread_t t0 = NULL;
    beken_thread_t t1 = NULL;
    beken_semaphore_t start_sem = NULL;
    beken_semaphore_t done_sem = NULL;
    I2C_DUAL_WORKER_T w0;
    I2C_DUAL_WORKER_T w1;

    if (loops == 0U) {
        return OPRT_INVALID_PARM;
    }

    (void)os_memset(&w0, 0, sizeof(w0));
    (void)os_memset(&w1, 0, sizeof(w1));
    w0.port = p0;
    w1.port = p1;
    w0.loops = loops;
    w1.loops = loops;

    if (rtos_init_semaphore_ex(&start_sem, 2, 0) != kNoErr) {
        return OPRT_COM_ERROR;
    }
    if (rtos_init_semaphore_ex(&done_sem, 2, 0) != kNoErr) {
        (void)rtos_deinit_semaphore(&start_sem);
        return OPRT_COM_ERROR;
    }
    w0.start_sem = &start_sem;
    w1.start_sem = &start_sem;
    w0.done_sem = &done_sem;
    w1.done_sem = &done_sem;

    if (rtos_create_thread(&t0, 6, "xi2c_p0", test_i2c_dual_basic_worker, 3072, &w0) != kNoErr) {
        (void)rtos_deinit_semaphore(&start_sem);
        (void)rtos_deinit_semaphore(&done_sem);
        return OPRT_COM_ERROR;
    }
    if (rtos_create_thread(&t1, 6, "xi2c_p1", test_i2c_dual_basic_worker, 3072, &w1) != kNoErr) {
        (void)rtos_set_semaphore(&start_sem);
        (void)rtos_get_semaphore(&done_sem, I2C_WAIT_MS);
        (void)rtos_thread_join(&t0);
        (void)rtos_deinit_semaphore(&start_sem);
        (void)rtos_deinit_semaphore(&done_sem);
        return OPRT_COM_ERROR;
    }

    (void)rtos_set_semaphore(&start_sem);
    (void)rtos_set_semaphore(&start_sem);
    if ((rtos_get_semaphore(&done_sem, I2C_WAIT_MS) != kNoErr) ||
        (rtos_get_semaphore(&done_sem, I2C_WAIT_MS) != kNoErr)) {
        (void)rtos_thread_join(&t0);
        (void)rtos_thread_join(&t1);
        (void)rtos_deinit_semaphore(&start_sem);
        (void)rtos_deinit_semaphore(&done_sem);
        return OPRT_TIMEOUT;
    }

    (void)rtos_thread_join(&t0);
    (void)rtos_thread_join(&t1);
    (void)rtos_deinit_semaphore(&start_sem);
    (void)rtos_deinit_semaphore(&done_sem);

    bk_printf("[xi2c] INFO test_name=M02_dual_port_parallel p0=%u pass=%u fail=%u p1=%u pass=%u fail=%u\r\n",
              (unsigned)p0, (unsigned)w0.pass, (unsigned)w0.fail, (unsigned)p1, (unsigned)w1.pass,
              (unsigned)w1.fail);

    if ((w0.fail != 0U) || (w1.fail != 0U)) {
        return OPRT_COM_ERROR;
    }
    return OPRT_OK;
}

/* @note Loopback case requires wiring in file header. */
static OPERATE_RET test_i2c_case_loopback(TUYA_I2C_NUM_E master_port, TUYA_I2C_NUM_E slave_port,
                                          UINT16_T slave_addr, UINT32_T bytes, UINT32_T loops,
                                          const char *test_name)
{
    TUYA_IIC_BASE_CFG_T master_cfg;
    I2C_LOOP_WORKER_T worker;
    beken_thread_t slave_thread = NULL;
    UINT32_T loop;
    UINT32_T i;
    UINT32_T wait_ms;
    UINT32_T master_mismatch = 0U;
    OPERATE_RET ret = OPRT_OK;

    if (s_loopback_case_enable == 0U) {
        (void)master_port;
        (void)slave_port;
        (void)slave_addr;
        (void)bytes;
        (void)loops;
        test_i2c_skip(test_name, "loopback case removed");
        return OPRT_NOT_SUPPORTED;
    }

    if ((bytes == 0U) || (bytes > I2C_LOOPBACK_BYTES_MAX) || (loops == 0U)) {
        return OPRT_INVALID_PARM;
    }

    (void)os_memset(&worker, 0, sizeof(worker));
    worker.slave_port = slave_port;
    worker.slave_addr = slave_addr;
    worker.bytes = bytes;
    worker.loops = loops;
    worker.last_ret = OPRT_COM_ERROR;

    if (rtos_create_thread(&slave_thread, 4, "xi2c_slv", test_i2c_loop_slave_worker, 3072, &worker) != kNoErr) {
        return OPRT_COM_ERROR;
    }

    master_cfg = test_i2c_build_cfg(TUYA_IIC_MODE_MASTER, TUYA_IIC_BUS_SPEED_100K, TUYA_IIC_ADDRESS_7BIT);
    ret = tkl_i2c_init(master_port, &master_cfg);
    if (ret != OPRT_OK) {
        worker.stop_req = 1U;
        (void)rtos_thread_join(&slave_thread);
        return ret;
    }

    wait_ms = 0U;
    while ((worker.started == 0U) && (worker.exited == 0U) && (wait_ms < I2C_WAIT_MS)) {
        tkl_system_sleep(I2C_POLL_INTERVAL_MS);
        wait_ms += I2C_POLL_INTERVAL_MS;
    }
    if (worker.started == 0U) {
        ret = (worker.last_ret != OPRT_COM_ERROR) ? worker.last_ret : OPRT_TIMEOUT;
        goto __EXIT;
    }

    for (loop = 0U; loop < loops; loop++) {
        wait_ms = 0U;
        while ((worker.ready_seq <= loop) && (worker.exited == 0U) && (wait_ms < I2C_WAIT_MS)) {
            tkl_system_sleep(I2C_POLL_INTERVAL_MS);
            wait_ms += I2C_POLL_INTERVAL_MS;
        }
        if (worker.ready_seq <= loop) {
            ret = OPRT_TIMEOUT;
            break;
        }
        /*
         * ready_seq is published before slave_receive/slave_send enters blocking wait.
         * Leave enough scheduling window for slave side to arm transaction state.
         */
        tkl_system_sleep(I2C_SLAVE_ARM_WAIT_MS);

        for (i = 0U; i < bytes; i++) {
            s_loop_master_tx[i] = test_i2c_pattern(loop, i);
            s_loop_master_rx[i] = 0U;
        }

        ret = tkl_i2c_master_send(master_port, slave_addr, s_loop_master_tx, bytes, FALSE);
        if (ret != OPRT_OK) {
            break;
        }

        wait_ms = 0U;
        while ((worker.resp_ready_seq <= loop) && (worker.exited == 0U) && (wait_ms < I2C_WAIT_MS)) {
            tkl_system_sleep(I2C_POLL_INTERVAL_MS);
            wait_ms += I2C_POLL_INTERVAL_MS;
        }
        if (worker.resp_ready_seq <= loop) {
            ret = OPRT_TIMEOUT;
            break;
        }
        /*
         * resp_ready_seq is published before slave_send blocks, keep symmetric wait
         * to reduce cross-thread race when master starts read phase.
         */
        tkl_system_sleep(I2C_SLAVE_ARM_WAIT_MS);

        ret = tkl_i2c_master_receive(master_port, slave_addr, s_loop_master_rx, bytes, FALSE);
        if (ret != OPRT_OK) {
            break;
        }

        for (i = 0U; i < bytes; i++) {
            UINT8_T expected = test_i2c_resp_from_req(s_loop_master_tx[i]);
            if (s_loop_master_rx[i] != expected) {
                master_mismatch++;
                ret = OPRT_COM_ERROR;
                break;
            }
        }
        if (ret != OPRT_OK) {
            break;
        }
    }

__EXIT:
    worker.stop_req = 1U;
    (void)rtos_thread_join(&slave_thread);
    (void)tkl_i2c_deinit(master_port);

    if ((ret == OPRT_OK) && (worker.last_ret != OPRT_OK)) {
        ret = worker.last_ret;
    }
    if ((ret == OPRT_OK) && ((worker.data_mismatch != 0U) || (master_mismatch != 0U))) {
        ret = OPRT_COM_ERROR;
    }

    if (ret == OPRT_OK) {
        s_pass++;
        bk_printf("[xi2c] PASS test_name=%s return_code=%d slave_recv_ok=%u slave_send_ok=%u\r\n", test_name,
                  (int)ret, (unsigned)worker.recv_ok, (unsigned)worker.send_ok);
    } else if (ret == OPRT_NOT_SUPPORTED) {
        test_i2c_skip(test_name, "loopback path not supported");
    } else {
        bk_printf("[xi2c] FAIL test_name=%s return_code=%d slave_ret=%d slave_mismatch=%u master_mismatch=%u\r\n",
                  test_name, (int)ret, (int)worker.last_ret, (unsigned)worker.data_mismatch,
                  (unsigned)master_mismatch);
        s_fail++;
    }

    return ret;
}

static void test_i2c_run_single(int port)
{
    if (!test_i2c_port_ok(port)) {
        bk_printf("[xi2c] ERROR invalid_port=%d hw_ports=%u\r\n", port, (unsigned)test_i2c_hw_port_num());
        return;
    }

    test_i2c_stats_reset();
    bk_printf("[xi2c] SUITE BEGIN test_name=SINGLE port=%d\r\n", port);
    test_i2c_run_single_core((TUYA_I2C_NUM_E)port);
    test_i2c_stats_print("single");
}

static void test_i2c_run_smoke(int port, UINT32_T loops)
{
    TUYA_IIC_BASE_CFG_T cfg;
    TUYA_IIC_STATUS_T status;
    OPERATE_RET ret;
    UINT32_T i;

    if (!test_i2c_port_ok(port)) {
        bk_printf("[xi2c] ERROR invalid_port=%d hw_ports=%u\r\n", port, (unsigned)test_i2c_hw_port_num());
        return;
    }

    if (loops == 0U) {
        loops = I2C_SMOKE_LOOPS_DEFAULT;
    }
    if (loops > I2C_SMOKE_LOOPS_MAX) {
        loops = I2C_SMOKE_LOOPS_MAX;
    }

    cfg = test_i2c_build_cfg(TUYA_IIC_MODE_MASTER, TUYA_IIC_BUS_SPEED_100K, TUYA_IIC_ADDRESS_7BIT);
    test_i2c_stats_reset();
    bk_printf("[xi2c] SUITE BEGIN test_name=SMOKE port=%d planned_loops=%u\r\n", port, (unsigned)loops);

    for (i = 0U; i < loops; i++) {
        ret = tkl_i2c_init((TUYA_I2C_NUM_E)port, &cfg);
        if (ret != OPRT_OK) {
            s_fail++;
            bk_printf("[xi2c] FAIL test_name=SMOKE_ITER_INIT iter=%u return_code=%d\r\n", (unsigned)i, (int)ret);
            break;
        }

        ret = tkl_i2c_set_slave_addr((TUYA_I2C_NUM_E)port, (UINT16_T)(0x30U + (i & 0x3FU)));
        if ((ret != OPRT_OK) && (ret != OPRT_NOT_SUPPORTED)) {
            s_fail++;
            bk_printf("[xi2c] FAIL test_name=SMOKE_ITER_SET_ADDR iter=%u return_code=%d\r\n", (unsigned)i, (int)ret);
            (void)tkl_i2c_deinit((TUYA_I2C_NUM_E)port);
            break;
        }

        ret = tkl_i2c_get_status((TUYA_I2C_NUM_E)port, &status);
        if (ret != OPRT_OK) {
            s_fail++;
            bk_printf("[xi2c] FAIL test_name=SMOKE_ITER_STATUS iter=%u return_code=%d\r\n", (unsigned)i, (int)ret);
            (void)tkl_i2c_deinit((TUYA_I2C_NUM_E)port);
            break;
        }

        ret = tkl_i2c_reset((TUYA_I2C_NUM_E)port);
        if (ret != OPRT_OK) {
            s_fail++;
            bk_printf("[xi2c] FAIL test_name=SMOKE_ITER_RESET iter=%u return_code=%d\r\n", (unsigned)i, (int)ret);
            (void)tkl_i2c_deinit((TUYA_I2C_NUM_E)port);
            break;
        }

        ret = tkl_i2c_deinit((TUYA_I2C_NUM_E)port);
        if (ret != OPRT_OK) {
            s_fail++;
            bk_printf("[xi2c] FAIL test_name=SMOKE_ITER_DEINIT iter=%u return_code=%d\r\n", (unsigned)i, (int)ret);
            break;
        }

        if ((i & 0x3FU) == 0U) {
            bk_printf("[xi2c] INFO test_name=SMOKE_PROGRESS iter=%u total=%u\r\n", (unsigned)i, (unsigned)loops);
        }
    }

    if (s_fail == 0U) {
        test_i2c_item_pass_plain("SMOKE_ALL_ITERATIONS_PASS");
    }
    test_i2c_stats_print("smoke");
}

static void test_i2c_run_full(int master_port, int slave_port, UINT16_T slave_addr, UINT32_T loops, UINT32_T bytes)
{
    OPERATE_RET ret;

    if (!test_i2c_port_ok(master_port)) {
        bk_printf("[xi2c] ERROR invalid_master_port=%d hw_ports=%u\r\n", master_port, (unsigned)test_i2c_hw_port_num());
        return;
    }
    if (!test_i2c_port_ok(slave_port)) {
        bk_printf("[xi2c] ERROR invalid_slave_port=%d hw_ports=%u\r\n", slave_port, (unsigned)test_i2c_hw_port_num());
        return;
    }
    if (master_port == slave_port) {
        bk_printf("[xi2c] ERROR same_port_not_allowed master=%d slave=%d\r\n", master_port, slave_port);
        return;
    }
    if (loops == 0U) {
        loops = I2C_LOOPBACK_LOOPS_DEFAULT;
    }
    if (loops > I2C_LOOPBACK_LOOPS_MAX) {
        loops = I2C_LOOPBACK_LOOPS_MAX;
    }
    if (bytes == 0U) {
        bytes = I2C_LOOPBACK_BYTES_DEFAULT;
    }
    if (bytes > I2C_LOOPBACK_BYTES_MAX) {
        bytes = I2C_LOOPBACK_BYTES_MAX;
    }

    test_i2c_stats_reset();
    bk_printf("[xi2c] SUITE BEGIN test_name=FULL master_port=%d slave_port=%d slave_addr=0x%02x loops=%u bytes=%u\r\n",
              master_port, slave_port, (unsigned)slave_addr, (unsigned)loops, (unsigned)bytes);

    test_i2c_run_single_core((TUYA_I2C_NUM_E)master_port);
    test_i2c_case_cfg_matrix((TUYA_I2C_NUM_E)master_port);

    if (test_i2c_hw_port_num() >= 2U) {
        ret = test_i2c_case_dual_port_parallel((TUYA_I2C_NUM_E)master_port, (TUYA_I2C_NUM_E)slave_port, 20U);
        if (ret == OPRT_OK) {
            test_i2c_item_pass_ret("M02_dual_port_parallel", ret);
        } else {
            test_i2c_item_fail_msg("M02_dual_port_parallel", "parallel init/reset/deinit failed");
        }
    } else {
        test_i2c_skip("M02_dual_port_parallel", "single i2c port build");
    }

    ret = test_i2c_case_pajid((TUYA_I2C_NUM_E)master_port, (UINT16_T)PAJ7620_DEFAULT_ADDR);
    (void)ret;

    ret = test_i2c_case_loopback((TUYA_I2C_NUM_E)master_port, (TUYA_I2C_NUM_E)slave_port, slave_addr, bytes, loops,
                                 "L01_loopback_master_to_slave");
    (void)ret;
    ret = test_i2c_case_loopback((TUYA_I2C_NUM_E)slave_port, (TUYA_I2C_NUM_E)master_port, slave_addr, bytes, loops,
                                 "L02_loopback_slave_to_master");
    (void)ret;

    test_i2c_stats_print("full");
}

static void test_i2c_run_pajid(int port, UINT16_T dev_addr)
{
    if (!test_i2c_port_ok(port)) {
        bk_printf("[xi2c] ERROR invalid_port=%d hw_ports=%u\r\n", port, (unsigned)test_i2c_hw_port_num());
        return;
    }

    test_i2c_stats_reset();
    bk_printf("[xi2c] SUITE BEGIN test_name=PAJ7620_ID port=%d dev_addr=0x%02x\r\n", port, (unsigned)dev_addr);
    (void)test_i2c_case_pajid((TUYA_I2C_NUM_E)port, dev_addr);
    test_i2c_stats_print("pajid");
}

void cli_xi2c_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int port = 0;
    int master_port = 0;
    int slave_port = 1;
    UINT32_T loops = I2C_SMOKE_LOOPS_DEFAULT;
    UINT32_T bytes = I2C_LOOPBACK_BYTES_DEFAULT;
    UINT16_T slave_addr = (UINT16_T)I2C_DEFAULT_SLAVE_ADDR;

    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    if ((argc < 2) || (argv == NULL) || (argv[1] == NULL)) {
        bk_printf("[xi2c] USAGE: xi2c single [port]\r\n");
        bk_printf("[xi2c] USAGE: xi2c smoke [port] [loops]\r\n");
        bk_printf("[xi2c] USAGE: xi2c full [master_port] [slave_port] [slave_addr_hex] [loops] [bytes]\r\n");
        bk_printf("[xi2c] USAGE: xi2c pajid [port] [dev_addr_hex]\r\n");
        bk_printf("[xi2c] NOTE: loopback cases are removed in current build.\r\n");
        return;
    }

    if (strcmp(argv[1], "single") == 0) {
        if ((argc >= 3) && (argv[2] != NULL)) {
            port = atoi(argv[2]);
        }
        test_i2c_run_single(port);
        return;
    }

    if (strcmp(argv[1], "smoke") == 0) {
        if ((argc >= 3) && (argv[2] != NULL)) {
            port = atoi(argv[2]);
        }
        if ((argc >= 4) && (argv[3] != NULL)) {
            loops = (UINT32_T)strtoul(argv[3], NULL, 10);
        }
        test_i2c_run_smoke(port, loops);
        return;
    }

    if (strcmp(argv[1], "full") == 0) {
        if ((argc >= 3) && (argv[2] != NULL)) {
            master_port = atoi(argv[2]);
        }
        if ((argc >= 4) && (argv[3] != NULL)) {
            slave_port = atoi(argv[3]);
        }
        if ((argc >= 5) && (argv[4] != NULL)) {
            slave_addr = (UINT16_T)strtoul(argv[4], NULL, 16);
        }
        if ((argc >= 6) && (argv[5] != NULL)) {
            loops = (UINT32_T)strtoul(argv[5], NULL, 10);
        } else {
            loops = I2C_LOOPBACK_LOOPS_DEFAULT;
        }
        if ((argc >= 7) && (argv[6] != NULL)) {
            bytes = (UINT32_T)strtoul(argv[6], NULL, 10);
        }
        test_i2c_run_full(master_port, slave_port, slave_addr, loops, bytes);
        return;
    }

    if (strcmp(argv[1], "pajid") == 0) {
        UINT16_T dev_addr = (UINT16_T)PAJ7620_DEFAULT_ADDR;
        if ((argc >= 3) && (argv[2] != NULL)) {
            port = atoi(argv[2]);
        }
        if ((argc >= 4) && (argv[3] != NULL)) {
            dev_addr = (UINT16_T)strtoul(argv[3], NULL, 16);
        }
        test_i2c_run_pajid(port, dev_addr);
        return;
    }

    bk_printf("[xi2c] ERROR unknown_subcommand=%s\r\n", argv[1]);
}
