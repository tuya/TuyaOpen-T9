/**
 * @file tkl_uart.c
 * @brief Auto-generated TKL adapter stub. Candidate UART adapter for T9.
 * @version 0.1
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_uart.h"

OPERATE_RET tkl_uart_init(TUYA_UART_NUM_E port_id, TUYA_UART_BASE_CFG_T *cfg)
{
    /* 通过端口号和基础配置初始化对应的uart实例，返回初始化结果 。 */
    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)port_id;
    (void)cfg;

    /* TODO: map `tkl_uart_init` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uart_deinit(TUYA_UART_NUM_E port_id)
{
    /* uart反初始化。该接口会停止uart。 */
    (void)port_id;

    /* TODO: map `tkl_uart_deinit` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

INT_T tkl_uart_write(TUYA_UART_NUM_E port_id, VOID_T *buff, UINT16_T len)
{
    /* uart 发送数据。 */
    if (buff == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)port_id;
    (void)buff;
    (void)len;

    /* TODO: map `tkl_uart_write` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

VOID_T tkl_uart_rx_irq_cb_reg(TUYA_UART_NUM_E port_id, TUYA_UART_IRQ_CB rx_cb)
{
    /* 注册uart 接受中断回调函数。 */
    (void)port_id;
    (void)rx_cb;

    /* TODO: map `tkl_uart_rx_irq_cb_reg` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return;
}

VOID_T tkl_uart_tx_irq_cb_reg(TUYA_UART_NUM_E port_id, TUYA_UART_IRQ_CB tx_cb)
{
    /* 注册uart 接受中断回调函数。 */
    (void)port_id;
    (void)tx_cb;

    /* TODO: map `tkl_uart_tx_irq_cb_reg` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return;
}

INT_T tkl_uart_read(TUYA_UART_NUM_E port_id, VOID_T *buff, UINT16_T len)
{
    /* 读取uart中的数据。 */
    if (buff == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)port_id;
    (void)buff;
    (void)len;

    /* TODO: map `tkl_uart_read` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uart_set_tx_int(TUYA_UART_NUM_E port_id, BOOL_T enable)
{
    /* enabe/disable uart 发送中断 */
    (void)port_id;
    (void)enable;

    /* TODO: map `tkl_uart_set_tx_int` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uart_set_rx_flowctrl(TUYA_UART_NUM_E port_id, BOOL_T enable)
{
    /* enable/disable uart 接受流控制。 */
    (void)port_id;
    (void)enable;

    /* TODO: map `tkl_uart_set_rx_flowctrl` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uart_wait_for_data(TUYA_UART_NUM_E port_id, INT_T timeout_ms)
{
    (void)port_id;
    (void)timeout_ms;

    /* TODO: map `tkl_uart_wait_for_data` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_uart_ioctl(TUYA_UART_NUM_E port_id, UINT32_T cmd, VOID *arg)
{
    (void)port_id;
    (void)cmd;
    (void)arg;

    /* TODO: map `tkl_uart_ioctl` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}
