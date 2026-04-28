/**
 * @file tkl_i2c.c
 * @brief TKL I2C adapter for T9 (Beken I2C driver)
 * @version 0.1
 * @date 2026-04-28
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_i2c.h"

#include <soc/soc.h>
#include <driver/i2c.h>
#include <string.h>

#include "tkl_mutex.h"

extern void bk_printf(const char *fmt, ...);

#define TKL_I2C_DEFAULT_SLAVE_ADDR       0x42u
#define TKL_I2C_XFER_TIMEOUT_MS          500u
#define TKL_I2C_REG_WIDTH_8BIT           1u
#define TKL_I2C_REG_WIDTH_16BIT          2u

typedef struct {
    BOOL_T              inited;
    BOOL_T              busy;
    TUYA_IIC_BASE_CFG_T cfg;
    UINT16_T            slave_addr;
    UINT32_T            reg_addr_width;
    UINT16_T            reg_addr_dev_addr;
    BOOL_T              pending_reg_valid;
    UINT16_T            pending_dev_addr;
    UINT32_T            pending_reg_addr;
    UINT32_T            pending_reg_width;
    INT32_T             last_count;
    bk_err_t            last_bk_ret;
    TUYA_I2C_IRQ_CB     irq_cb;
    TUYA_IIC_ROLE_E     last_mode;
    UINT8_T             last_direction; /* 0: write, 1: read */
} TKL_I2C_PORT_CTX_T;

STATIC TKL_MUTEX_HANDLE s_i2c_mutex = NULL;
STATIC BOOL_T s_i2c_driver_inited = FALSE;
STATIC UINT32_T s_i2c_init_port_count = 0;
STATIC TKL_I2C_PORT_CTX_T s_i2c_ctx[TUYA_I2C_NUM_MAX];

STATIC OPERATE_RET __tkl_i2c_lock(void)
{
    OPERATE_RET ret;

    if (s_i2c_mutex == NULL) {
        ret = tkl_mutex_create_init(&s_i2c_mutex);
        if (ret != OPRT_OK) {
            return ret;
        }
    }

    return tkl_mutex_lock(s_i2c_mutex);
}

STATIC VOID_T __tkl_i2c_unlock(void)
{
    if (s_i2c_mutex != NULL) {
        (VOID_T)tkl_mutex_unlock(s_i2c_mutex);
    }
}

STATIC OPERATE_RET __tkl_i2c_port_to_id(TUYA_I2C_NUM_E port, i2c_id_t *id)
{
    if (id == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (port >= TUYA_I2C_NUM_MAX) {
        return OPRT_INVALID_PARM;
    }

#if defined(SOC_I2C_UNIT_NUM)
    if ((UINT32_T)port >= (UINT32_T)SOC_I2C_UNIT_NUM) {
#else
    if ((UINT32_T)port >= (UINT32_T)I2C_ID_MAX) {
#endif
        return OPRT_NOT_SUPPORTED;
    }

    *id = (i2c_id_t)port;
    return OPRT_OK;
}

STATIC VOID_T __tkl_i2c_clear_pending(TKL_I2C_PORT_CTX_T *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->pending_reg_valid = FALSE;
    ctx->pending_dev_addr = 0;
    ctx->pending_reg_addr = 0;
    ctx->pending_reg_width = 0;
}

STATIC OPERATE_RET __tkl_i2c_map_bk_err(bk_err_t bk_ret, OPERATE_RET default_ret)
{
    if (bk_ret == BK_OK) {
        return OPRT_OK;
    }

    if ((bk_ret == BK_ERR_NULL_PARAM) || (bk_ret == BK_ERR_PARAM)) {
        return OPRT_INVALID_PARM;
    }

    if (bk_ret == BK_ERR_NO_MEM) {
        return OPRT_MALLOC_FAILED;
    }

    if (bk_ret == BK_ERR_NOT_SUPPORT) {
        return OPRT_NOT_SUPPORTED;
    }

    if ((bk_ret == BK_ERR_TIMEOUT) ||
        (bk_ret == BK_ERR_I2C_ACK_TIMEOUT) ||
        (bk_ret == BK_ERR_I2C_SCL_TIMEOUT)) {
        return OPRT_TIMEOUT;
    }

    if ((bk_ret == BK_ERR_I2C_NOT_INIT) || (bk_ret == BK_ERR_I2C_ID_NOT_INIT)) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if ((bk_ret == BK_ERR_I2C_SM_BUS_BUSY) || (bk_ret == BK_ERR_BUSY)) {
        return OPRT_COM_ERROR;
    }

    return default_ret;
}

STATIC OPERATE_RET __tkl_i2c_build_bk_cfg(CONST TUYA_IIC_BASE_CFG_T *cfg, UINT16_T slave_addr, i2c_config_t *bk_cfg)
{
    if ((cfg == NULL) || (bk_cfg == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if ((cfg->role != TUYA_IIC_MODE_MASTER) && (cfg->role != TUYA_IIC_MODE_SLAVE)) {
        return OPRT_INVALID_PARM;
    }

    (VOID_T)memset(bk_cfg, 0, sizeof(i2c_config_t));

    switch (cfg->speed) {
    case TUYA_IIC_BUS_SPEED_100K:
        bk_cfg->baud_rate = I2C_BAUD_RATE_100KHZ;
        break;
    case TUYA_IIC_BUS_SPEED_400K:
        bk_cfg->baud_rate = I2C_BAUD_RATE_400KHZ;
        break;
    case TUYA_IIC_BUS_SPEED_1M:
        bk_cfg->baud_rate = 1000000u;
        break;
    case TUYA_IIC_BUS_SPEED_3_4M:
    default:
        return OPRT_NOT_SUPPORTED;
    }

    switch (cfg->addr_width) {
    case TUYA_IIC_ADDRESS_7BIT:
        bk_cfg->addr_mode = I2C_ADDR_MODE_7BIT;
        break;
    case TUYA_IIC_ADDRESS_10BIT:
        bk_cfg->addr_mode = I2C_ADDR_MODE_10BIT;
        break;
    default:
        return OPRT_INVALID_PARM;
    }

    bk_cfg->slave_addr = slave_addr;
    bk_cfg->src_clk = I2C_SCLK_XTAL;

    return OPRT_OK;
}

STATIC VOID_T __tkl_i2c_update_result(TKL_I2C_PORT_CTX_T *ctx, bk_err_t bk_ret,
                                      INT32_T ok_count, UINT8_T direction, TUYA_IIC_ROLE_E mode)
{
    if (ctx == NULL) {
        return;
    }

    ctx->busy = FALSE;
    ctx->last_bk_ret = bk_ret;
    ctx->last_direction = direction;
    ctx->last_mode = mode;
    ctx->last_count = (bk_ret == BK_OK) ? ok_count : 0;
}

STATIC OPERATE_RET __tkl_i2c_parse_mem_addr(CONST UINT8_T *data, UINT32_T len, UINT32_T *mem_addr)
{
    if ((data == NULL) || (mem_addr == NULL) || (len == 0)) {
        return OPRT_INVALID_PARM;
    }

    if (len == TKL_I2C_REG_WIDTH_8BIT) {
        *mem_addr = data[0];
        return OPRT_OK;
    }

    if (len == TKL_I2C_REG_WIDTH_16BIT) {
        *mem_addr = ((UINT32_T)data[0] << 8) | (UINT32_T)data[1];
        return OPRT_OK;
    }

    return OPRT_NOT_SUPPORTED;
}

STATIC OPERATE_RET __tkl_i2c_to_mem_addr_size(UINT32_T width, i2c_mem_addr_size_t *size_out)
{
    if (size_out == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (width == TKL_I2C_REG_WIDTH_8BIT) {
        *size_out = I2C_MEM_ADDR_SIZE_8BIT;
        return OPRT_OK;
    }

    if (width == TKL_I2C_REG_WIDTH_16BIT) {
        *size_out = I2C_MEM_ADDR_SIZE_16BIT;
        return OPRT_OK;
    }

    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_i2c_init(TUYA_I2C_NUM_E port, CONST TUYA_IIC_BASE_CFG_T *cfg)
{
    OPERATE_RET ret;
    i2c_id_t id;
    i2c_config_t bk_cfg;
    bk_err_t bk_ret;
    TKL_I2C_PORT_CTX_T *ctx;

    if (cfg == NULL) {
        bk_printf("[tkl_i2c] init failed: cfg null\r\n");
        return OPRT_INVALID_PARM;
    }

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
#if defined(SOC_I2C_UNIT_NUM)
        bk_printf("[tkl_i2c] init failed: invalid port=%u soc_i2c_num=%u ret=%d\r\n",
                  (unsigned int)port, (unsigned int)SOC_I2C_UNIT_NUM, ret);
#else
        bk_printf("[tkl_i2c] init failed: invalid port=%u i2c_id_max=%u ret=%d\r\n",
                  (unsigned int)port, (unsigned int)I2C_ID_MAX, ret);
#endif
        goto __EXIT;
    }

    ctx = &s_i2c_ctx[port];
    if (ctx->slave_addr == 0) {
        ctx->slave_addr = TKL_I2C_DEFAULT_SLAVE_ADDR;
    }

    if (ctx->inited) {
        if (ctx->busy) {
            bk_printf("[tkl_i2c] init failed: port=%u busy, skip reinit\r\n", (unsigned int)port);
            ret = OPRT_COM_ERROR;
            goto __EXIT;
        }

        if (memcmp(&ctx->cfg, cfg, sizeof(TUYA_IIC_BASE_CFG_T)) == 0) {
            ret = OPRT_OK;
            goto __EXIT;
        }

        bk_ret = bk_i2c_deinit(id);
        ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_OS_ADAPTER_I2C_CLOSE_FAILED);
        if (ret != OPRT_OK) {
            goto __EXIT;
        }

        ctx->inited = FALSE;
        if (s_i2c_init_port_count > 0) {
            s_i2c_init_port_count--;
        }
    }

    if (!s_i2c_driver_inited) {
        bk_ret = bk_i2c_driver_init();
        ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_OS_ADAPTER_I2C_OPEN_FAILED);
        if (ret != OPRT_OK) {
            bk_printf("[tkl_i2c] init failed: driver_init port=%u id=%u bk_ret=%d ret=%d\r\n",
                      (unsigned int)port, (unsigned int)id, (int)bk_ret, ret);
            goto __EXIT;
        }
        s_i2c_driver_inited = TRUE;
    }

    ret = __tkl_i2c_build_bk_cfg(cfg, ctx->slave_addr, &bk_cfg);
    if (ret != OPRT_OK) {
        bk_printf("[tkl_i2c] init failed: cfg_build port=%u id=%u role=%d speed=%d addrw=%d ret=%d\r\n",
                  (unsigned int)port,
                  (unsigned int)id,
                  (int)cfg->role,
                  (int)cfg->speed,
                  (int)cfg->addr_width,
                  ret);
        if ((s_i2c_init_port_count == 0) && s_i2c_driver_inited) {
            (VOID_T)bk_i2c_driver_deinit();
            s_i2c_driver_inited = FALSE;
        }
        goto __EXIT;
    }

    bk_ret = bk_i2c_init(id, &bk_cfg);
    ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_OS_ADAPTER_I2C_OPEN_FAILED);
    if (ret != OPRT_OK) {
        bk_printf("[tkl_i2c] init failed: bk_i2c_init port=%u id=%u baud=%u mode=%u slave=0x%02x bk_ret=%d ret=%d\r\n",
                  (unsigned int)port,
                  (unsigned int)id,
                  (unsigned int)bk_cfg.baud_rate,
                  (unsigned int)bk_cfg.addr_mode,
                  (unsigned int)bk_cfg.slave_addr,
                  (int)bk_ret,
                  ret);
        if ((s_i2c_init_port_count == 0) && s_i2c_driver_inited) {
            (VOID_T)bk_i2c_driver_deinit();
            s_i2c_driver_inited = FALSE;
        }
        goto __EXIT;
    }

    ctx->inited = TRUE;
    ctx->busy = FALSE;
    ctx->cfg = *cfg;
    ctx->last_bk_ret = BK_OK;
    ctx->last_count = 0;
    ctx->last_mode = cfg->role;
    ctx->last_direction = 0;
    __tkl_i2c_clear_pending(ctx);
    s_i2c_init_port_count++;
    ret = OPRT_OK;

__EXIT:
    __tkl_i2c_unlock();
    return ret;
}

OPERATE_RET tkl_i2c_deinit(TUYA_I2C_NUM_E port)
{
    OPERATE_RET ret;
    i2c_id_t id;
    bk_err_t bk_ret;
    TKL_I2C_PORT_CTX_T *ctx;

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    ctx = &s_i2c_ctx[port];
    if (!ctx->inited) {
        ret = OPRT_OK;
        goto __EXIT;
    }

    if (ctx->busy) {
        bk_printf("[tkl_i2c] deinit failed: port=%u busy\r\n", (unsigned int)port);
        ret = OPRT_COM_ERROR;
        goto __EXIT;
    }

    bk_ret = bk_i2c_deinit(id);
    ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_OS_ADAPTER_I2C_CLOSE_FAILED);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    ctx->inited = FALSE;
    ctx->busy = FALSE;
    ctx->irq_cb = NULL;
    ctx->last_bk_ret = BK_OK;
    ctx->last_count = 0;
    __tkl_i2c_clear_pending(ctx);
    if (s_i2c_init_port_count > 0) {
        s_i2c_init_port_count--;
    }

    if ((s_i2c_init_port_count == 0) && s_i2c_driver_inited) {
        bk_ret = bk_i2c_driver_deinit();
        ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_OS_ADAPTER_I2C_CLOSE_FAILED);
        if (ret != OPRT_OK) {
            goto __EXIT;
        }
        s_i2c_driver_inited = FALSE;
    }

    ret = OPRT_OK;

__EXIT:
    __tkl_i2c_unlock();
    return ret;
}

OPERATE_RET tkl_i2c_irq_init(TUYA_I2C_NUM_E port, TUYA_I2C_IRQ_CB cb)
{
    OPERATE_RET ret;
    i2c_id_t id;

    if (cb == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret == OPRT_OK) {
        s_i2c_ctx[port].irq_cb = cb;
        ret = OPRT_NOT_SUPPORTED;
    }

    __tkl_i2c_unlock();
    return ret;
}

OPERATE_RET tkl_i2c_irq_enable(TUYA_I2C_NUM_E port)
{
    OPERATE_RET ret;
    i2c_id_t id;
    bk_err_t bk_ret;

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    if (!s_i2c_ctx[port].inited) {
        ret = OPRT_RESOURCE_NOT_READY;
        goto __EXIT;
    }

    bk_ret = bk_i2c_enable_interrupt(id);
    ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_COM_ERROR);

__EXIT:
    __tkl_i2c_unlock();
    return ret;
}

OPERATE_RET tkl_i2c_irq_disable(TUYA_I2C_NUM_E port)
{
    OPERATE_RET ret;
    i2c_id_t id;
    bk_err_t bk_ret;

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    if (!s_i2c_ctx[port].inited) {
        ret = OPRT_RESOURCE_NOT_READY;
        goto __EXIT;
    }

    bk_ret = bk_i2c_disable_interrupt(id);
    ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_COM_ERROR);

__EXIT:
    __tkl_i2c_unlock();
    return ret;
}

OPERATE_RET tkl_i2c_master_send(TUYA_I2C_NUM_E port, UINT16_T dev_addr, CONST VOID_T *data, UINT32_T size, BOOL_T xfer_pending)
{
    OPERATE_RET ret;
    OPERATE_RET lock_ret;
    i2c_id_t id;
    bk_err_t bk_ret;
    BOOL_T do_vendor_xfer = FALSE;
    TKL_I2C_PORT_CTX_T *ctx;

    if ((data == NULL) || (size == 0)) {
        return OPRT_INVALID_PARM;
    }

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    ctx = &s_i2c_ctx[port];
    if (!ctx->inited) {
        ret = OPRT_RESOURCE_NOT_READY;
        goto __EXIT;
    }

    ctx->busy = TRUE;
    if (ctx->pending_reg_valid && (ctx->pending_dev_addr != dev_addr)) {
        __tkl_i2c_clear_pending(ctx);
    }

    /*
     * xfer_pending 在当前平台仅用于常见寄存器读前缀：
     * 先发送寄存器地址（不立即落总线），后续 receive 使用 memory_read 完成重复起始事务。
     */
    if (xfer_pending) {
        CONST UINT8_T *bytes = (CONST UINT8_T *)data;
        UINT32_T width = ctx->reg_addr_width ? ctx->reg_addr_width : size;
        UINT32_T mem_addr = 0;

        if (width != size) {
            ctx->busy = FALSE;
            ctx->last_bk_ret = BK_ERR_PARAM;
            ctx->last_count = 0;
            ret = OPRT_INVALID_PARM;
            goto __EXIT;
        }

        ret = __tkl_i2c_parse_mem_addr(bytes, width, &mem_addr);
        if (ret != OPRT_OK) {
            ctx->busy = FALSE;
            ctx->last_bk_ret = (ret == OPRT_NOT_SUPPORTED) ? BK_ERR_NOT_SUPPORT : BK_ERR_PARAM;
            ctx->last_count = 0;
            goto __EXIT;
        }

        ctx->pending_reg_valid = TRUE;
        ctx->pending_dev_addr = dev_addr;
        ctx->pending_reg_addr = mem_addr;
        ctx->pending_reg_width = width;
        __tkl_i2c_update_result(ctx, BK_OK, (INT32_T)size, 0, TUYA_IIC_MODE_MASTER);
        ret = OPRT_OK;
        goto __EXIT;
    }

    __tkl_i2c_clear_pending(ctx);
    do_vendor_xfer = TRUE;
    ret = OPRT_OK;

__EXIT:
    __tkl_i2c_unlock();

    if (!do_vendor_xfer) {
        return ret;
    }

    bk_ret = bk_i2c_master_write(id, (uint32_t)dev_addr, (const uint8_t *)data, size, TKL_I2C_XFER_TIMEOUT_MS);
    ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_OS_ADAPTER_I2C_WRITE_FAILED);

    lock_ret = __tkl_i2c_lock();
    if (lock_ret == OPRT_OK) {
        __tkl_i2c_update_result(&s_i2c_ctx[port], bk_ret, (INT32_T)size, 0, TUYA_IIC_MODE_MASTER);
        __tkl_i2c_unlock();
    }

    return ret;
}

OPERATE_RET tkl_i2c_master_receive(TUYA_I2C_NUM_E port, UINT16_T dev_addr, VOID *data, UINT32_T size, BOOL_T xfer_pending)
{
    OPERATE_RET ret;
    OPERATE_RET lock_ret;
    i2c_id_t id;
    bk_err_t bk_ret;
    BOOL_T do_mem_read = FALSE;
    BOOL_T do_master_read = FALSE;
    UINT32_T mem_addr = 0;
    i2c_mem_addr_size_t mem_addr_size = I2C_MEM_ADDR_SIZE_8BIT;
    TKL_I2C_PORT_CTX_T *ctx;

    if ((data == NULL) || (size == 0)) {
        return OPRT_INVALID_PARM;
    }

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    ctx = &s_i2c_ctx[port];
    if (!ctx->inited) {
        ret = OPRT_RESOURCE_NOT_READY;
        goto __EXIT;
    }

    ctx->busy = TRUE;
    if (xfer_pending) {
        __tkl_i2c_update_result(ctx, BK_ERR_NOT_SUPPORT, 0, 1, TUYA_IIC_MODE_MASTER);
        ret = OPRT_NOT_SUPPORTED;
        goto __EXIT;
    }

    if (ctx->pending_reg_valid && (ctx->pending_dev_addr == dev_addr)) {
        ret = __tkl_i2c_to_mem_addr_size(ctx->pending_reg_width, &mem_addr_size);
        if (ret != OPRT_OK) {
            __tkl_i2c_clear_pending(ctx);
            __tkl_i2c_update_result(ctx, BK_ERR_NOT_SUPPORT, 0, 1, TUYA_IIC_MODE_MASTER);
            goto __EXIT;
        }

        mem_addr = ctx->pending_reg_addr;
        __tkl_i2c_clear_pending(ctx);
        do_mem_read = TRUE;
        ret = OPRT_OK;
        goto __EXIT;
    }

    __tkl_i2c_clear_pending(ctx);
    do_master_read = TRUE;
    ret = OPRT_OK;

__EXIT:
    __tkl_i2c_unlock();

    if (ret != OPRT_OK) {
        return ret;
    }

    if (do_mem_read) {
        i2c_mem_param_t mem_param;
        (VOID_T)memset(&mem_param, 0, sizeof(i2c_mem_param_t));
        mem_param.dev_addr = (uint32_t)dev_addr;
        mem_param.mem_addr = mem_addr;
        mem_param.mem_addr_size = mem_addr_size;
        mem_param.data = (uint8_t *)data;
        mem_param.data_size = size;
        mem_param.timeout_ms = TKL_I2C_XFER_TIMEOUT_MS;
        bk_ret = bk_i2c_memory_read(id, &mem_param);
    } else if (do_master_read) {
        bk_ret = bk_i2c_master_read(id, (uint32_t)dev_addr, (uint8_t *)data, size, TKL_I2C_XFER_TIMEOUT_MS);
    } else {
        return OPRT_COM_ERROR;
    }

    ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_OS_ADAPTER_I2C_READ_FAILED);

    lock_ret = __tkl_i2c_lock();
    if (lock_ret == OPRT_OK) {
        __tkl_i2c_update_result(&s_i2c_ctx[port], bk_ret, (INT32_T)size, 1, TUYA_IIC_MODE_MASTER);
        __tkl_i2c_unlock();
    }

    return ret;
}

OPERATE_RET tkl_i2c_set_slave_addr(TUYA_I2C_NUM_E port, UINT16_T dev_addr)
{
    OPERATE_RET ret;
    i2c_id_t id;
    bk_err_t bk_ret;
    TKL_I2C_PORT_CTX_T *ctx;

    if (dev_addr > 0x3FFu) {
        return OPRT_INVALID_PARM;
    }

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    ctx = &s_i2c_ctx[port];
    ctx->slave_addr = dev_addr;
    if (!ctx->inited) {
        ret = OPRT_OK;
        goto __EXIT;
    }

    ctx->busy = TRUE;
    bk_ret = bk_i2c_set_slave_address(id, dev_addr);
    __tkl_i2c_update_result(ctx, bk_ret, 0, 0, TUYA_IIC_MODE_SLAVE);
    ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_COM_ERROR);

__EXIT:
    __tkl_i2c_unlock();
    return ret;
}

OPERATE_RET tkl_i2c_slave_send(TUYA_I2C_NUM_E port, CONST VOID *data, UINT32_T size)
{
    OPERATE_RET ret;
    OPERATE_RET lock_ret;
    i2c_id_t id;
    bk_err_t bk_ret;
    BOOL_T do_vendor_xfer = FALSE;
    TKL_I2C_PORT_CTX_T *ctx;

    if ((data == NULL) || (size == 0)) {
        return OPRT_INVALID_PARM;
    }

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    ctx = &s_i2c_ctx[port];
    if (!ctx->inited) {
        ret = OPRT_RESOURCE_NOT_READY;
        goto __EXIT;
    }

    ctx->busy = TRUE;
    do_vendor_xfer = TRUE;
    ret = OPRT_OK;

__EXIT:
    __tkl_i2c_unlock();

    if (!do_vendor_xfer) {
        return ret;
    }

    bk_ret = bk_i2c_slave_write(id, (const uint8_t *)data, size, TKL_I2C_XFER_TIMEOUT_MS);
    ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_OS_ADAPTER_I2C_WRITE_FAILED);

    lock_ret = __tkl_i2c_lock();
    if (lock_ret == OPRT_OK) {
        __tkl_i2c_update_result(&s_i2c_ctx[port], bk_ret, (INT32_T)size, 0, TUYA_IIC_MODE_SLAVE);
        __tkl_i2c_unlock();
    }

    return ret;
}

OPERATE_RET tkl_i2c_slave_receive(TUYA_I2C_NUM_E port, VOID *data, UINT32_T size)
{
    OPERATE_RET ret;
    OPERATE_RET lock_ret;
    i2c_id_t id;
    bk_err_t bk_ret;
    BOOL_T do_vendor_xfer = FALSE;
    TKL_I2C_PORT_CTX_T *ctx;

    if ((data == NULL) || (size == 0)) {
        return OPRT_INVALID_PARM;
    }

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    ctx = &s_i2c_ctx[port];
    if (!ctx->inited) {
        ret = OPRT_RESOURCE_NOT_READY;
        goto __EXIT;
    }

    ctx->busy = TRUE;
    do_vendor_xfer = TRUE;
    ret = OPRT_OK;

__EXIT:
    __tkl_i2c_unlock();

    if (!do_vendor_xfer) {
        return ret;
    }

    bk_ret = bk_i2c_slave_read(id, (uint8_t *)data, size, TKL_I2C_XFER_TIMEOUT_MS);
    ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_OS_ADAPTER_I2C_READ_FAILED);

    lock_ret = __tkl_i2c_lock();
    if (lock_ret == OPRT_OK) {
        __tkl_i2c_update_result(&s_i2c_ctx[port], bk_ret, (INT32_T)size, 1, TUYA_IIC_MODE_SLAVE);
        __tkl_i2c_unlock();
    }

    return ret;
}

OPERATE_RET tkl_i2c_get_status(TUYA_I2C_NUM_E port, TUYA_IIC_STATUS_T *status)
{
    OPERATE_RET ret;
    i2c_id_t id;
    TKL_I2C_PORT_CTX_T *ctx;
    BOOL_T hw_busy;

    if (status == NULL) {
        return OPRT_INVALID_PARM;
    }

    (VOID_T)memset(status, 0, sizeof(TUYA_IIC_STATUS_T));

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    ctx = &s_i2c_ctx[port];
    if (!ctx->inited) {
        ret = OPRT_RESOURCE_NOT_READY;
        goto __EXIT;
    }

    hw_busy = bk_i2c_is_bus_busy(id) ? TRUE : FALSE;
    status->busy = (ctx->busy || hw_busy) ? 1u : 0u;
    status->mode = (ctx->last_mode == TUYA_IIC_MODE_MASTER) ? 1u : 0u;
    status->direction = (ctx->last_direction != 0u) ? 1u : 0u;
    status->general_call = 0u;
    status->arbitration_lost = 0u;
    status->bus_error = ((ctx->last_bk_ret == BK_ERR_I2C_ACK_TIMEOUT) ||
                         (ctx->last_bk_ret == BK_ERR_I2C_SCL_TIMEOUT) ||
                         (ctx->last_bk_ret == BK_ERR_I2C_SM_BUS_BUSY)) ? 1u : 0u;
    ret = OPRT_OK;

__EXIT:
    __tkl_i2c_unlock();
    return ret;
}

OPERATE_RET tkl_i2c_reset(TUYA_I2C_NUM_E port)
{
    OPERATE_RET ret;
    i2c_id_t id;
    i2c_config_t bk_cfg;
    bk_err_t bk_ret;
    TKL_I2C_PORT_CTX_T *ctx;

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    ctx = &s_i2c_ctx[port];
    if (!ctx->inited) {
        ret = OPRT_RESOURCE_NOT_READY;
        goto __EXIT;
    }

    ret = __tkl_i2c_build_bk_cfg(&ctx->cfg, ctx->slave_addr, &bk_cfg);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    ctx->busy = TRUE;
    bk_ret = bk_i2c_deinit(id);
    if (bk_ret != BK_OK) {
        __tkl_i2c_update_result(ctx, bk_ret, 0, ctx->last_direction, ctx->last_mode);
        ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_OS_ADAPTER_I2C_CLOSE_FAILED);
        goto __EXIT;
    }

    bk_ret = bk_i2c_init(id, &bk_cfg);
    if (bk_ret != BK_OK) {
        ctx->inited = FALSE;
        if (s_i2c_init_port_count > 0) {
            s_i2c_init_port_count--;
        }
        __tkl_i2c_update_result(ctx, bk_ret, 0, ctx->last_direction, ctx->last_mode);
        __tkl_i2c_clear_pending(ctx);
        if ((s_i2c_init_port_count == 0) && s_i2c_driver_inited) {
            if (bk_i2c_driver_deinit() == BK_OK) {
                s_i2c_driver_inited = FALSE;
            }
        }
        ret = __tkl_i2c_map_bk_err(bk_ret, OPRT_OS_ADAPTER_I2C_OPEN_FAILED);
        goto __EXIT;
    }

    __tkl_i2c_clear_pending(ctx);
    __tkl_i2c_update_result(ctx, BK_OK, 0, 0, ctx->cfg.role);
    ret = OPRT_OK;

__EXIT:
    __tkl_i2c_unlock();
    return ret;
}

INT32_T tkl_i2c_get_data_count(TUYA_I2C_NUM_E port)
{
    OPERATE_RET op_ret;
    i2c_id_t id;
    INT32_T ret;

    op_ret = __tkl_i2c_lock();
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    op_ret = __tkl_i2c_port_to_id(port, &id);
    if (op_ret != OPRT_OK) {
        __tkl_i2c_unlock();
        return op_ret;
    }

    (void)id;
    if (!s_i2c_ctx[port].inited) {
        __tkl_i2c_unlock();
        return OPRT_RESOURCE_NOT_READY;
    }

    ret = s_i2c_ctx[port].last_count;
    __tkl_i2c_unlock();
    return ret;
}

OPERATE_RET tkl_i2c_ioctl(TUYA_I2C_NUM_E port, UINT32_T cmd, VOID *args)
{
    OPERATE_RET ret;
    i2c_id_t id;
    TKL_I2C_PORT_CTX_T *ctx;

    ret = __tkl_i2c_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    ret = __tkl_i2c_port_to_id(port, &id);
    if (ret != OPRT_OK) {
        goto __EXIT;
    }

    (void)id;
    ctx = &s_i2c_ctx[port];
    switch (cmd) {
    case I2C_IOCTL_SET_REGADDR_WIDTH: {
        REGADDR_WIDTH_T *cfg = (REGADDR_WIDTH_T *)args;
        if (cfg == NULL) {
            ret = OPRT_INVALID_PARM;
            break;
        }

        if ((cfg->reg_addr_width != TKL_I2C_REG_WIDTH_8BIT) &&
            (cfg->reg_addr_width != TKL_I2C_REG_WIDTH_16BIT)) {
            ret = OPRT_NOT_SUPPORTED;
            break;
        }

        ctx->reg_addr_width = cfg->reg_addr_width;
        ctx->reg_addr_dev_addr = (UINT16_T)cfg->dev_addr;
        __tkl_i2c_clear_pending(ctx);
        ret = OPRT_OK;
        break;
    }
    default:
        ret = OPRT_NOT_SUPPORTED;
        break;
    }

__EXIT:
    __tkl_i2c_unlock();
    return ret;
}
