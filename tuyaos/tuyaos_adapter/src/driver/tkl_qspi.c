/**
 * @file tkl_qspi.c
 * @brief TKL QSPI adapter implementation for T9
 * @version 0.1
 * @date 2026-05-06
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_qspi.h"

#include <common/bk_include.h>
#include <driver/gpio.h>
#include <driver/int.h>
#include <modules/pm.h>
#include <os/mem.h>
#include <os/os.h>
#include "gpio_driver.h"
#include "qspi_hal.h"
#include "sys_driver.h"
#include "tuya_dma.h"
#include "bk_misc.h"   /* bk_delay_us() prototype */

extern void bk_delay_us(UINT32 us);

#define TKL_QSPI_DIV_FACTOR_NUM 5U
#define TKL_QSPI_SRC_DIV_MAX    15U
#define TKL_QSPI_CLK_DIV_MAX    4U
#define TKL_QSPI_DMA_POLL_US    10U
#define TKL_QSPI_DMA_TIMEOUT_US 10000000U
#define TKL_QSPI_LCD_WRITE_CMD  0x2CU

typedef struct {
    qspi_hal_t hal;
    TUYA_QSPI_BASE_CFG_T cfg;
    TUYA_QSPI_IRQ_CB irq_cb;
    beken_mutex_t port_lock;
    dma_id_t dma_tx_chan;
    BOOL_T inited;
    BOOL_T irq_enable;
    BOOL_T dma_inited;
    BOOL_T lock_inited;
} TKL_QSPI_PORT_CTX_T;

static TKL_QSPI_PORT_CTX_T s_qspi_ctx[SOC_QSPI_UNIT_NUM];
static beken_mutex_t s_qspi_driver_lock = NULL;
static BOOL_T s_qspi_driver_ready = FALSE;

static OPERATE_RET tkl_qspi_map_bk_err(bk_err_t ret)
{
    return (ret == BK_OK) ? OPRT_OK : OPRT_COM_ERROR;
}

static BOOL_T tkl_qspi_is_valid_port(TUYA_QSPI_NUM_E port)
{
    return (port < TUYA_QSPI_NUM_MAX) && ((uint32_t)port < SOC_QSPI_UNIT_NUM);
}

static OPERATE_RET tkl_qspi_driver_lock(void)
{
    if (s_qspi_driver_lock == NULL) {
        return OPRT_NOT_FOUND;
    }

    return (rtos_lock_mutex(&s_qspi_driver_lock) == BK_OK) ? OPRT_OK : OPRT_COM_ERROR;
}

static void tkl_qspi_driver_unlock(void)
{
    if (s_qspi_driver_lock != NULL) {
        rtos_unlock_mutex(&s_qspi_driver_lock);
    }
}

static OPERATE_RET tkl_qspi_port_lock(TKL_QSPI_PORT_CTX_T *ctx)
{
    if ((ctx == NULL) || (ctx->lock_inited == FALSE) || (ctx->port_lock == NULL)) {
        return OPRT_NOT_FOUND;
    }

    return (rtos_lock_mutex(&ctx->port_lock) == BK_OK) ? OPRT_OK : OPRT_COM_ERROR;
}

static void tkl_qspi_port_unlock(TKL_QSPI_PORT_CTX_T *ctx)
{
    if ((ctx != NULL) && (ctx->lock_inited == TRUE) && (ctx->port_lock != NULL)) {
        rtos_unlock_mutex(&ctx->port_lock);
    }
}

static void tkl_qspi_map_gpio(qspi_id_t id)
{
    switch (id) {
    case QSPI_ID_0:
        gpio_dev_unmap(QSPI0_LL_CSN_PIN);
        gpio_dev_unmap(QSPI0_LL_CLK_PIN);
        gpio_dev_unmap(QSPI0_LL_IO0_PIN);
        gpio_dev_unmap(QSPI0_LL_IO1_PIN);
        gpio_dev_unmap(QSPI0_LL_IO2_PIN);
        gpio_dev_unmap(QSPI0_LL_IO3_PIN);

        gpio_dev_map(QSPI0_LL_CSN_PIN, GPIO_DEV_QSPI0_CSN);
        gpio_dev_map(QSPI0_LL_CLK_PIN, GPIO_DEV_QSPI0_CLK);
        gpio_dev_map(QSPI0_LL_IO0_PIN, GPIO_DEV_QSPI0_IO0);
        gpio_dev_map(QSPI0_LL_IO1_PIN, GPIO_DEV_QSPI0_IO1);
        gpio_dev_map(QSPI0_LL_IO2_PIN, GPIO_DEV_QSPI0_IO2);
        gpio_dev_map(QSPI0_LL_IO3_PIN, GPIO_DEV_QSPI0_IO3);

        bk_gpio_pull_up(QSPI0_LL_CSN_PIN);
        bk_gpio_pull_up(QSPI0_LL_IO0_PIN);
        bk_gpio_pull_up(QSPI0_LL_IO1_PIN);
        bk_gpio_pull_up(QSPI0_LL_IO2_PIN);
        bk_gpio_pull_up(QSPI0_LL_IO3_PIN);

        bk_gpio_set_capacity(QSPI0_LL_CSN_PIN, 3);
        bk_gpio_set_capacity(QSPI0_LL_CLK_PIN, 3);
        bk_gpio_set_capacity(QSPI0_LL_IO0_PIN, 3);
        bk_gpio_set_capacity(QSPI0_LL_IO1_PIN, 3);
        bk_gpio_set_capacity(QSPI0_LL_IO2_PIN, 3);
        bk_gpio_set_capacity(QSPI0_LL_IO3_PIN, 3);
        break;

#if (SOC_QSPI_UNIT_NUM > 1)
    case QSPI_ID_1:
        gpio_dev_unmap(QSPI1_LL_CSN_PIN);
        gpio_dev_unmap(QSPI1_LL_CLK_PIN);
        gpio_dev_unmap(QSPI1_LL_IO0_PIN);
        gpio_dev_unmap(QSPI1_LL_IO1_PIN);
        gpio_dev_unmap(QSPI1_LL_IO2_PIN);
        gpio_dev_unmap(QSPI1_LL_IO3_PIN);

        gpio_dev_map(QSPI1_LL_CSN_PIN, GPIO_DEV_QSPI1_CSN);
        gpio_dev_map(QSPI1_LL_CLK_PIN, GPIO_DEV_QSPI1_CLK);
        gpio_dev_map(QSPI1_LL_IO0_PIN, GPIO_DEV_QSPI1_IO0);
        gpio_dev_map(QSPI1_LL_IO1_PIN, GPIO_DEV_QSPI1_IO1);
        gpio_dev_map(QSPI1_LL_IO2_PIN, GPIO_DEV_QSPI1_IO2);
        gpio_dev_map(QSPI1_LL_IO3_PIN, GPIO_DEV_QSPI1_IO3);

        bk_gpio_pull_up(QSPI1_LL_CSN_PIN);
        bk_gpio_pull_up(QSPI1_LL_IO0_PIN);
        bk_gpio_pull_up(QSPI1_LL_IO1_PIN);
        bk_gpio_pull_up(QSPI1_LL_IO2_PIN);
        bk_gpio_pull_up(QSPI1_LL_IO3_PIN);

        bk_gpio_set_capacity(QSPI1_LL_CSN_PIN, 3);
        bk_gpio_set_capacity(QSPI1_LL_CLK_PIN, 3);
        bk_gpio_set_capacity(QSPI1_LL_IO0_PIN, 3);
        bk_gpio_set_capacity(QSPI1_LL_IO1_PIN, 3);
        bk_gpio_set_capacity(QSPI1_LL_IO2_PIN, 3);
        bk_gpio_set_capacity(QSPI1_LL_IO3_PIN, 3);
        break;
#endif

    default:
        break;
    }
}

static void tkl_qspi_clock_power_ctrl(qspi_id_t id, BOOL_T enable)
{
    if (id == QSPI_ID_0) {
        bk_pm_module_vote_power_ctrl(PM_POWER_SUB_MODULE_NAME_AHBP_QSPI,
                                     enable ? PM_POWER_MODULE_STATE_ON : PM_POWER_MODULE_STATE_OFF);
        sys_drv_dev_clk_pwr_up(CLK_PWR_ID_QSPI0, enable ? CLK_PWR_CTRL_PWR_UP : CLK_PWR_CTRL_PWR_DOWN);
        sys_drv_set_int_en(0, INT_SRC_QSPI0, 0);
        return;
    }

#if (SOC_QSPI_UNIT_NUM > 1)
    if (id == QSPI_ID_1) {
        bk_pm_module_vote_power_ctrl(PM_POWER_SUB_MODULE_NAME_AHBP_QSPI1,
                                     enable ? PM_POWER_MODULE_STATE_ON : PM_POWER_MODULE_STATE_OFF);
        sys_drv_dev_clk_pwr_up(CLK_PWR_ID_QSPI1, enable ? CLK_PWR_CTRL_PWR_UP : CLK_PWR_CTRL_PWR_DOWN);
        sys_drv_set_int_en(0, INT_SRC_QSPI1, 0);
    }
#endif
}

static void tkl_qspi_pick_clock_cfg(uint32_t target_hz,
                                    qspi_src_clk_t *src_clk,
                                    uint32_t *src_div,
                                    uint32_t *clk_div)
{
    static const uint32_t s_src_hz[] = {160000000U, 240000000U};
    static const qspi_src_clk_t s_src_sel[] = {QSPI_SCLK_160M, QSPI_SCLK_240M};
    static const uint32_t s_clk_div_factor[TKL_QSPI_DIV_FACTOR_NUM] = {1U, 2U, 4U, 6U, 8U};
    uint32_t best_under_hz = 0U;
    uint32_t best_under_src = 0U;
    uint32_t best_under_src_div = 0U;
    uint32_t best_under_clk_div = 0U;

    uint32_t min_hz = 0xFFFFFFFFU;
    uint32_t min_src = 0U;
    uint32_t min_src_div = 0U;
    uint32_t min_clk_div = 0U;
    uint32_t hz = (target_hz == 0U) ? 20000000U : target_hz;

    for (uint32_t si = 0; si < sizeof(s_src_hz) / sizeof(s_src_hz[0]); si++) {
        for (uint32_t sdiv = 0; sdiv <= TKL_QSPI_SRC_DIV_MAX; sdiv++) {
            for (uint32_t cdiv = 0; cdiv <= TKL_QSPI_CLK_DIV_MAX; cdiv++) {
                uint32_t calc_hz = s_src_hz[si] / ((sdiv + 1U) * s_clk_div_factor[cdiv]);

                if (calc_hz < min_hz) {
                    min_hz = calc_hz;
                    min_src = si;
                    min_src_div = sdiv;
                    min_clk_div = cdiv;
                }

                if ((calc_hz <= hz) &&
                    ((calc_hz > best_under_hz) ||
                     ((calc_hz == best_under_hz) && (cdiv < best_under_clk_div)))) {
                    best_under_hz = calc_hz;
                    best_under_src = si;
                    best_under_src_div = sdiv;
                    best_under_clk_div = cdiv;
                }
            }
        }
    }

    if (best_under_hz > 0U) {
        *src_clk = s_src_sel[best_under_src];
        *src_div = best_under_src_div;
        *clk_div = best_under_clk_div;
    } else {
        *src_clk = s_src_sel[min_src];
        *src_div = min_src_div;
        *clk_div = min_clk_div;
    }
}

static qspi_device_t tkl_qspi_map_device(TUYA_QSPI_MODULE_TYPE_E type)
{
    switch (type) {
    case TUYA_QSPI_TYPE_FLASH:
        return QSPI_FLASH;
    case TUYA_QSPI_TYPE_PSRAM:
        return QSPI_PSRAM;
    case TUYA_QSPI_TYPE_LCD:
    default:
        return QSPI_OLED;
    }
}

static uint32_t tkl_qspi_dma_dst_addr(TKL_QSPI_PORT_CTX_T *ctx)
{
    if ((ctx != NULL) && (ctx->cfg.type == TUYA_QSPI_TYPE_LCD)) {
        return QSPI_DATA_LL_REG_BASE(ctx->hal.id);
    }

    return (uint32_t)(uintptr_t)&ctx->hal.hw->fifo_data[0];
}

static OPERATE_RET tkl_qspi_map_wire_mode(TUYA_QSPI_WIRE_MODE_E mode, qspi_wire_mode_t *wire_mode)
{
    if (wire_mode == NULL) {
        return OPRT_INVALID_PARM;
    }

    switch (mode) {
    case TUYA_QSPI_1WIRE:
        *wire_mode = QSPI_1WIRE;
        return OPRT_OK;
    case TUYA_QSPI_2WIRE:
        *wire_mode = QSPI_2WIRE;
        return OPRT_OK;
    case TUYA_QSPI_4WIRE:
        *wire_mode = QSPI_4WIRE;
        return OPRT_OK;
    default:
        return OPRT_NOT_SUPPORTED;
    }
}

static OPERATE_RET tkl_qspi_driver_prepare(void)
{
    OPERATE_RET ret = OPRT_OK;

    if (s_qspi_driver_lock == NULL) {
        if (rtos_init_mutex(&s_qspi_driver_lock) != BK_OK) {
            return OPRT_COM_ERROR;
        }
    }

    ret = tkl_qspi_driver_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    if (s_qspi_driver_ready == TRUE) {
        tkl_qspi_driver_unlock();
        return OPRT_OK;
    }

    os_memset(s_qspi_ctx, 0, sizeof(s_qspi_ctx));
    for (uint32_t id = 0; id < SOC_QSPI_UNIT_NUM; id++) {
        if (rtos_init_mutex(&s_qspi_ctx[id].port_lock) != BK_OK) {
            for (uint32_t j = 0; j < id; j++) {
                rtos_deinit_mutex(&s_qspi_ctx[j].port_lock);
                s_qspi_ctx[j].lock_inited = FALSE;
            }
            tkl_qspi_driver_unlock();
            return OPRT_COM_ERROR;
        }
        s_qspi_ctx[id].lock_inited = TRUE;
        s_qspi_ctx[id].hal.id = (qspi_id_t)id;
        qspi_hal_init(&s_qspi_ctx[id].hal);
        s_qspi_ctx[id].dma_tx_chan = DMA_ID_MAX;
    }

    s_qspi_driver_ready = TRUE;
    tkl_qspi_driver_unlock();
    return OPRT_OK;
}

static OPERATE_RET tkl_qspi_dma_init_locked(TKL_QSPI_PORT_CTX_T *ctx)
{
    TKL_DMA_CONFIG_T dma_cfg = {0};
    OPERATE_RET ret = OPRT_OK;

    if ((ctx == NULL) || (ctx->cfg.use_dma == FALSE)) {
        return OPRT_OK;
    }

    if (ctx->dma_inited == TRUE) {
        return OPRT_OK;
    }

    dma_cfg.mode = TKL_DMA_WORK_MODE_SINGLE;
    dma_cfg.src.dev = DMA_DEV_DTCM;
    dma_cfg.src.width = TKL_DMA_DATA_WIDTH_32BITS;
    dma_cfg.src.addr_inc_en = TKL_DMA_ADDR_INC_ENABLE;
    dma_cfg.src.addr_loop_en = TKL_DMA_ADDR_LOOP_DISABLE;
    dma_cfg.src.start_addr = 0U;
    dma_cfg.src.end_addr = 0U;
    dma_cfg.src.brust_len = BURST_LEN_INC16;

    dma_cfg.dst.dev = DMA_DEV_DTCM;
    dma_cfg.dst.width = TKL_DMA_DATA_WIDTH_32BITS;
    dma_cfg.dst.addr_inc_en = TKL_DMA_ADDR_INC_ENABLE;
    dma_cfg.dst.addr_loop_en = TKL_DMA_ADDR_LOOP_DISABLE;
    dma_cfg.dst.start_addr = tkl_qspi_dma_dst_addr(ctx);
    dma_cfg.dst.end_addr = 0U;
    dma_cfg.dst.brust_len = BURST_LEN_INC16;

    dma_cfg.dev_id = DMA_DEV_DTCM;

    ret = tkl_dma_init(&ctx->dma_tx_chan, &dma_cfg);
    if (ret != OPRT_OK) {
        ctx->dma_tx_chan = DMA_ID_MAX;
        return ret;
    }

    ctx->dma_inited = TRUE;
    return OPRT_OK;
}

static void tkl_qspi_dma_deinit_locked(TKL_QSPI_PORT_CTX_T *ctx)
{
    if ((ctx == NULL) || (ctx->dma_inited == FALSE)) {
        return;
    }

    tkl_dma_stop(ctx->dma_tx_chan);
    tkl_dma_deinit(ctx->dma_tx_chan);
    ctx->dma_tx_chan = DMA_ID_MAX;
    ctx->dma_inited = FALSE;
}

static OPERATE_RET tkl_qspi_dma_wait_done_locked(dma_id_t dma_id)
{
    for (uint32_t waited = 0; waited < TKL_QSPI_DMA_TIMEOUT_US; waited += TKL_QSPI_DMA_POLL_US) {
        if (tkl_dma_get_enable_status(dma_id) == 0U) {
            return OPRT_OK;
        }
        bk_delay_us(TKL_QSPI_DMA_POLL_US);
    }

    tkl_dma_stop(dma_id);
    return OPRT_TIMEOUT;
}

static OPERATE_RET tkl_qspi_dma_write_locked(TKL_QSPI_PORT_CTX_T *ctx, const uint8_t *data, uint32_t size)
{
    OPERATE_RET ret = OPRT_OK;

    if ((ctx == NULL) || (ctx->dma_inited == FALSE)) {
        return OPRT_NOT_SUPPORTED;
    }

    if ((((uintptr_t)data & 0x3U) != 0U) || ((size & 0x3U) != 0U)) {
        return OPRT_INVALID_PARM;
    }

    if (ctx->cfg.type != TUYA_QSPI_TYPE_LCD) {
        ctx->hal.hw->rst_cfg.fifo_io_wr = 1;
        ctx->hal.hw->rst_cfg.clk_man_en = 1;
        ctx->hal.hw->rst_cfg.clk_man_sel = 0;
        bk_delay_us(1000);
    }

    ret = tkl_dma_write(ctx->dma_tx_chan, data, size);
    if (ret != OPRT_OK) {
        if (ctx->cfg.type != TUYA_QSPI_TYPE_LCD) {
            ctx->hal.hw->rst_cfg.fifo_io_wr = 0;
            ctx->hal.hw->rst_cfg.clk_man_en = 1;
            ctx->hal.hw->rst_cfg.clk_man_sel = 1;
        }
        return ret;
    }

    ret = tkl_qspi_dma_wait_done_locked(ctx->dma_tx_chan);
    if (ctx->cfg.type != TUYA_QSPI_TYPE_LCD) {
        ctx->hal.hw->rst_cfg.fifo_io_wr = 0;
        ctx->hal.hw->rst_cfg.clk_man_en = 1;
        ctx->hal.hw->rst_cfg.clk_man_sel = 1;
        bk_delay_us(1000);
    }

    return ret;
}

static OPERATE_RET tkl_qspi_io_write_locked(TKL_QSPI_PORT_CTX_T *ctx, const uint8_t *data, uint32_t size)
{
    uint32_t tmp_buf[(MAX_QSPI_FIFO_SIZE + 3U) / 4U] = {0};
    uint8_t *tmp_ptr = (uint8_t *)tmp_buf;

    if (size > MAX_QSPI_FIFO_SIZE) {
        return OPRT_NOT_SUPPORTED;
    }

    if ((((uintptr_t)data & 0x3U) == 0U) && ((size & 0x3U) == 0U)) {
        return tkl_qspi_map_bk_err(qspi_hal_io_write(&ctx->hal, data, size));
    }

    os_memcpy(tmp_ptr, data, size);
    return tkl_qspi_map_bk_err(qspi_hal_io_write(&ctx->hal, tmp_ptr, size));
}

static OPERATE_RET tkl_qspi_send_data_locked(TKL_QSPI_PORT_CTX_T *ctx, const uint8_t *data, uint32_t size)
{
    if ((ctx == NULL) || (data == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (size == 0U) {
        return OPRT_OK;
    }

    if ((ctx->cfg.use_dma == TRUE) && (ctx->dma_inited == TRUE)) {
        if ((((uintptr_t)data & 0x3U) == 0U) && ((size & 0x3U) == 0U)) {
            if ((ctx->cfg.type != TUYA_QSPI_TYPE_LCD) && (size > MAX_QSPI_FIFO_SIZE)) {
                return OPRT_NOT_SUPPORTED;
            }
            return tkl_qspi_dma_write_locked(ctx, data, size);
        }

        if (size > MAX_QSPI_FIFO_SIZE) {
            return OPRT_INVALID_PARM;
        }
    }

    return tkl_qspi_io_write_locked(ctx, data, size);
}

static OPERATE_RET tkl_qspi_io_read_locked(TKL_QSPI_PORT_CTX_T *ctx, uint8_t *data, uint32_t size)
{
    uint32_t tmp_buf[(MAX_QSPI_FIFO_SIZE + 3U) / 4U] = {0};
    uint8_t *tmp_ptr = (uint8_t *)tmp_buf;
    OPERATE_RET ret = OPRT_OK;

    if (size > MAX_QSPI_FIFO_SIZE) {
        return OPRT_NOT_SUPPORTED;
    }

    if ((((uintptr_t)data & 0x3U) == 0U) && ((size & 0x3U) == 0U)) {
        return tkl_qspi_map_bk_err(qspi_hal_io_read(&ctx->hal, data, size));
    }

    ret = tkl_qspi_map_bk_err(qspi_hal_io_read(&ctx->hal, tmp_ptr, size));
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memcpy(data, tmp_ptr, size);
    return OPRT_OK;
}

static OPERATE_RET tkl_qspi_lcd_write_start_locked(TKL_QSPI_PORT_CTX_T *ctx)
{
    OPERATE_RET ret = OPRT_OK;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_qspi_map_bk_err(qspi_hal_force_spi_cs_low_enable(&ctx->hal));
    if (ret != OPRT_OK) {
        return ret;
    }

    qspi_hal_set_cmd_c_l(&ctx->hal, 0);
    qspi_hal_set_cmd_c_h(&ctx->hal, TKL_QSPI_LCD_WRITE_CMD);
    qspi_hal_set_cmd_c_cfg1(&ctx->hal, 0x300);
    qspi_hal_set_cmd_c_cfg2(&ctx->hal, 0);
    qspi_hal_cmd_c_start(&ctx->hal);
    qspi_hal_wait_cmd_done(&ctx->hal);
    qspi_hal_io_cpu_mem_select(&ctx->hal, 1);
    qspi_hal_disable_cmd_sck_enable(&ctx->hal);

    return OPRT_OK;
}

static void tkl_qspi_lcd_write_stop_locked(TKL_QSPI_PORT_CTX_T *ctx)
{
    if (ctx == NULL) {
        return;
    }

    qspi_hal_disable_cmd_sck_disable(&ctx->hal);
    qspi_hal_force_spi_cs_low_disable(&ctx->hal);
    qspi_hal_io_cpu_mem_select(&ctx->hal, 0);
}

static OPERATE_RET tkl_qspi_wait_tx_idle_locked(TKL_QSPI_PORT_CTX_T *ctx)
{
    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    for (uint32_t waited = 0; waited < TKL_QSPI_DMA_TIMEOUT_US; waited += TKL_QSPI_DMA_POLL_US) {
        if ((ctx->hal.hw->status.tx_busy == 0U) && (ctx->hal.hw->status.fifo_empty != 0U)) {
            return OPRT_OK;
        }
        bk_delay_us(TKL_QSPI_DMA_POLL_US);
    }

    return OPRT_TIMEOUT;
}

OPERATE_RET tkl_qspi_init(TUYA_QSPI_NUM_E port, const TUYA_QSPI_BASE_CFG_T *cfg)
{
    OPERATE_RET ret = OPRT_OK;
    qspi_src_clk_t src_clk = QSPI_SCLK_240M;
    uint32_t src_div = 0U;
    uint32_t clk_div = 0U;
    TKL_QSPI_PORT_CTX_T *ctx = NULL;

    if (!tkl_qspi_is_valid_port(port) || (cfg == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->role != TUYA_QSPI_ROLE_MASTER) {
        return OPRT_NOT_SUPPORTED;
    }

    ret = tkl_qspi_driver_prepare();
    if (ret != OPRT_OK) {
        return ret;
    }

    ctx = &s_qspi_ctx[port];
    ret = tkl_qspi_port_lock(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }

    /*
     * The hardware bring-up below touches registers SHARED across both QSPI
     * ports: PMU/clock power, GPIO mux (tkl_qspi_map_gpio), QSPI common regs
     * (qspi_hal_init_common) and the sys_drv clock source/divider. The per-port
     * lock does NOT serialize these, so port0/port1 bring-up on the two SMP cores
     * races on the shared registers and a port can come up half-configured.
     * Serialize the whole bring-up with the cross-port global lock. The data path
     * (tkl_qspi_send) still uses only the per-port lock, so the two ports keep
     * transferring in parallel. Lock order is always port_lock -> driver_lock
     * (tkl_qspi_driver_prepare took/released driver_lock alone, before the port
     * lock), so this cannot deadlock.
     */
    ret = tkl_qspi_driver_lock();
    if (ret != OPRT_OK) {
        tkl_qspi_port_unlock(ctx);
        return ret;
    }

    tkl_qspi_dma_deinit_locked(ctx);

    tkl_qspi_clock_power_ctrl((qspi_id_t)port, TRUE);
    tkl_qspi_map_gpio((qspi_id_t)port);
    qspi_hal_init_common(&ctx->hal);

    tkl_qspi_pick_clock_cfg(cfg->freq_hz, &src_clk, &src_div, &clk_div);
    qspi_hal_set_clock_source((qspi_id_t)port, src_clk);
    sys_drv_qspi_set_src_clk_div((qspi_id_t)port, src_div);
    qspi_hal_set_clk_div(&ctx->hal, clk_div);

    os_memcpy(&ctx->cfg, cfg, sizeof(TUYA_QSPI_BASE_CFG_T));
    if (ctx->cfg.type == TUYA_QSPI_TYPE_LCD) {
        qspi_hal_set_cmd_a_cfg2(&ctx->hal, 0x80008000);
    }
    ret = tkl_qspi_dma_init_locked(ctx);
    if (ret != OPRT_OK) {
        tkl_qspi_clock_power_ctrl((qspi_id_t)port, FALSE);
        qspi_hal_deinit_common(&ctx->hal);
        tkl_qspi_driver_unlock();
        tkl_qspi_port_unlock(ctx);
        return ret;
    }
    ctx->inited = TRUE;

    tkl_qspi_driver_unlock();
    tkl_qspi_port_unlock(ctx);
    return OPRT_OK;
}

OPERATE_RET tkl_qspi_deinit(TUYA_QSPI_NUM_E port)
{
    OPERATE_RET ret = OPRT_OK;
    TKL_QSPI_PORT_CTX_T *ctx = NULL;

    if (!tkl_qspi_is_valid_port(port)) {
        return OPRT_INVALID_PARM;
    }

    if (s_qspi_driver_ready != TRUE) {
        return OPRT_OK;
    }

    ctx = &s_qspi_ctx[port];
    ret = tkl_qspi_port_lock(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }
    if (ctx->inited == FALSE) {
        tkl_qspi_port_unlock(ctx);
        return OPRT_OK;
    }

    /* Serialize shared-register teardown with the cross-port global lock; see
     * tkl_qspi_init for the rationale and lock-ordering note. */
    ret = tkl_qspi_driver_lock();
    if (ret != OPRT_OK) {
        tkl_qspi_port_unlock(ctx);
        return ret;
    }

    tkl_qspi_dma_deinit_locked(ctx);
    qspi_hal_deinit_common(&ctx->hal);
    tkl_qspi_clock_power_ctrl((qspi_id_t)port, FALSE);
    ctx->inited = FALSE;
    ctx->irq_enable = FALSE;
    os_memset(&ctx->cfg, 0, sizeof(ctx->cfg));

    tkl_qspi_driver_unlock();
    tkl_qspi_port_unlock(ctx);
    return OPRT_OK;
}

OPERATE_RET tkl_qspi_send(TUYA_QSPI_NUM_E port, VOID_T *data, UINT32_T size)
{
    OPERATE_RET ret = OPRT_OK;
    TKL_QSPI_PORT_CTX_T *ctx = NULL;
    TUYA_QSPI_IRQ_CB cb = NULL;

    if (!tkl_qspi_is_valid_port(port) || (data == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (size == 0U) {
        return OPRT_OK;
    }

    if (s_qspi_driver_ready != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ctx = &s_qspi_ctx[port];
    ret = tkl_qspi_port_lock(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }
    if (ctx->inited == FALSE) {
        tkl_qspi_port_unlock(ctx);
        return OPRT_RESOURCE_NOT_READY;
    }
    if (ctx->cfg.type != TUYA_QSPI_TYPE_LCD) {
        tkl_qspi_port_unlock(ctx);
        return OPRT_NOT_SUPPORTED;
    }

    ret = tkl_qspi_lcd_write_start_locked(ctx);
    if (ret == OPRT_OK) {
        ret = tkl_qspi_send_data_locked(ctx, (const uint8_t *)data, size);
        if (ret == OPRT_OK) {
            ret = tkl_qspi_wait_tx_idle_locked(ctx);
        }
        tkl_qspi_lcd_write_stop_locked(ctx);
    }
    if ((ret == OPRT_OK) && (ctx->irq_enable == TRUE)) {
        cb = ctx->irq_cb;
    }

    tkl_qspi_port_unlock(ctx);

    if (cb != NULL) {
        cb(port, TUYA_QSPI_EVENT_TX);
    }

    return ret;
}

OPERATE_RET tkl_qspi_recv(TUYA_QSPI_NUM_E port, VOID_T *data, UINT32_T size)
{
    OPERATE_RET ret = OPRT_OK;
    TKL_QSPI_PORT_CTX_T *ctx = NULL;
    TUYA_QSPI_IRQ_CB cb = NULL;

    if (!tkl_qspi_is_valid_port(port) || (data == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (size == 0U) {
        return OPRT_OK;
    }

    if (s_qspi_driver_ready != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ctx = &s_qspi_ctx[port];
    ret = tkl_qspi_port_lock(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }
    if (ctx->inited == FALSE) {
        tkl_qspi_port_unlock(ctx);
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = tkl_qspi_io_read_locked(ctx, (uint8_t *)data, size);
    if ((ret == OPRT_OK) && (ctx->irq_enable == TRUE)) {
        cb = ctx->irq_cb;
    }

    tkl_qspi_port_unlock(ctx);

    if (cb != NULL) {
        cb(port, TUYA_QSPI_EVENT_RX);
    }

    return ret;
}

OPERATE_RET tkl_qspi_comand(TUYA_QSPI_NUM_E port, TUYA_QSPI_CMD_T *command)
{
    OPERATE_RET ret = OPRT_OK;
    qspi_cmd_t qcmd = {0};
    TKL_QSPI_PORT_CTX_T *ctx = NULL;
    TUYA_QSPI_IRQ_CB cb = NULL;
    qspi_wire_mode_t wire_mode = QSPI_1WIRE;
    uint32_t max_wire = 0;
    uint32_t cmd = 0;
    uint32_t addr = 0;

    if (!tkl_qspi_is_valid_port(port) || (command == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if ((command->cmd_size > 4U) || (command->addr_size > 4U)) {
        return OPRT_INVALID_PARM;
    }

    if ((command->data == NULL) && (command->data_size > 0U)) {
        return OPRT_INVALID_PARM;
    }

    if (s_qspi_driver_ready != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ctx = &s_qspi_ctx[port];
    ret = tkl_qspi_port_lock(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }
    if (ctx->inited == FALSE) {
        tkl_qspi_port_unlock(ctx);
        return OPRT_RESOURCE_NOT_READY;
    }

    if ((command->data_size > MAX_QSPI_FIFO_SIZE) && (ctx->cfg.type != TUYA_QSPI_TYPE_LCD)) {
        tkl_qspi_port_unlock(ctx);
        return OPRT_NOT_SUPPORTED;
    }

    max_wire = ctx->cfg.dma_data_lines;
    if ((command->cmd_size > 0U) && (command->cmd_lines > max_wire)) {
        max_wire = command->cmd_lines;
    }
    if ((command->addr_size > 0U) && (command->addr_lines > max_wire)) {
        max_wire = command->addr_lines;
    }
    if ((command->data_size > 0U) && (command->data_lines > max_wire)) {
        max_wire = command->data_lines;
    }

    ret = tkl_qspi_map_wire_mode((TUYA_QSPI_WIRE_MODE_E)max_wire, &wire_mode);
    if (ret != OPRT_OK) {
        tkl_qspi_port_unlock(ctx);
        return ret;
    }

    for (uint32_t i = 0; i < command->cmd_size; i++) {
        cmd |= ((uint32_t)command->cmd[i] << (8U * i));
    }

    for (uint32_t i = 0; i < command->addr_size; i++) {
        addr <<= 8U;
        addr |= command->addr[i];
    }

    qcmd.op = (command->op == TUYA_QSPI_READ) ? QSPI_READ : QSPI_WRITE;
    qcmd.wire_mode = wire_mode;
    qcmd.work_mode = INDIRECT_MODE;
    qcmd.cmd = cmd;
    qcmd.addr = addr;
    qcmd.dummy_cycle = command->dummy_cycle;
    qcmd.data_len = command->data_size;
    qcmd.device = tkl_qspi_map_device(ctx->cfg.type);

    if ((qcmd.op == QSPI_WRITE) && (command->data_size > 0U)) {
        ret = tkl_qspi_send_data_locked(ctx, command->data, command->data_size);
        if (ret != OPRT_OK) {
            tkl_qspi_port_unlock(ctx);
            return ret;
        }
    }

    ret = tkl_qspi_map_bk_err(qspi_hal_command(&ctx->hal, &qcmd));
    if (ret != OPRT_OK) {
        tkl_qspi_port_unlock(ctx);
        return ret;
    }

    if ((qcmd.op == QSPI_READ) && (command->data_size > 0U)) {
        ret = tkl_qspi_io_read_locked(ctx, command->data, command->data_size);
        if (ret != OPRT_OK) {
            tkl_qspi_port_unlock(ctx);
            return ret;
        }
    }

    if ((ret == OPRT_OK) && (ctx->irq_enable == TRUE)) {
        cb = ctx->irq_cb;
    }

    tkl_qspi_port_unlock(ctx);

    if (cb != NULL) {
        cb(port, (qcmd.op == QSPI_WRITE) ? TUYA_QSPI_EVENT_TX : TUYA_QSPI_EVENT_RX);
    }

    return ret;
}

OPERATE_RET tkl_qspi_irq_init(TUYA_QSPI_NUM_E port, TUYA_QSPI_IRQ_CB cb)
{
    OPERATE_RET ret = OPRT_OK;
    TKL_QSPI_PORT_CTX_T *ctx = NULL;

    if (!tkl_qspi_is_valid_port(port)) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_qspi_driver_prepare();
    if (ret != OPRT_OK) {
        return ret;
    }

    ctx = &s_qspi_ctx[port];
    ret = tkl_qspi_port_lock(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }
    ctx->irq_cb = cb;
    ctx->irq_enable = FALSE;

    tkl_qspi_port_unlock(ctx);
    return OPRT_OK;
}

OPERATE_RET tkl_qspi_irq_enable(TUYA_QSPI_NUM_E port)
{
    OPERATE_RET ret = OPRT_OK;
    TKL_QSPI_PORT_CTX_T *ctx = NULL;

    if (!tkl_qspi_is_valid_port(port)) {
        return OPRT_INVALID_PARM;
    }

    if (s_qspi_driver_ready != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ctx = &s_qspi_ctx[port];
    ret = tkl_qspi_port_lock(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }
    if (ctx->inited == FALSE) {
        tkl_qspi_port_unlock(ctx);
        return OPRT_RESOURCE_NOT_READY;
    }

    ctx->irq_enable = TRUE;
    tkl_qspi_port_unlock(ctx);
    return OPRT_OK;
}

OPERATE_RET tkl_qspi_irq_disable(TUYA_QSPI_NUM_E port)
{
    OPERATE_RET ret = OPRT_OK;
    TKL_QSPI_PORT_CTX_T *ctx = NULL;

    if (!tkl_qspi_is_valid_port(port)) {
        return OPRT_INVALID_PARM;
    }

    if (s_qspi_driver_ready != TRUE) {
        return OPRT_OK;
    }

    ctx = &s_qspi_ctx[port];
    ret = tkl_qspi_port_lock(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }
    ctx->irq_enable = FALSE;

    tkl_qspi_port_unlock(ctx);
    return OPRT_OK;
}

OPERATE_RET tkl_qspi_force_cs_pin(TUYA_QSPI_NUM_E port, TUYA_GPIO_LEVEL_E level)
{
    OPERATE_RET ret = OPRT_OK;
    TKL_QSPI_PORT_CTX_T *ctx = NULL;

    if (!tkl_qspi_is_valid_port(port)) {
        return OPRT_INVALID_PARM;
    }

    if (s_qspi_driver_ready != TRUE) {
        return OPRT_RESOURCE_NOT_READY;
    }

    ctx = &s_qspi_ctx[port];
    ret = tkl_qspi_port_lock(ctx);
    if (ret != OPRT_OK) {
        return ret;
    }
    if (ctx->inited == FALSE) {
        tkl_qspi_port_unlock(ctx);
        return OPRT_RESOURCE_NOT_READY;
    }

    if (level == TUYA_GPIO_LEVEL_LOW) {
        ret = tkl_qspi_map_bk_err(qspi_hal_force_spi_cs_low_enable(&ctx->hal));
    } else if (level == TUYA_GPIO_LEVEL_HIGH) {
        ret = tkl_qspi_map_bk_err(qspi_hal_force_spi_cs_low_disable(&ctx->hal));
    } else {
        ret = OPRT_INVALID_PARM;
    }

    tkl_qspi_port_unlock(ctx);
    return ret;
}
