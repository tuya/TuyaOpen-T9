/**
 * @file tkl_qspi.c
 * @brief Auto-generated TKL adapter stub. Candidate QSPI adapter for T9.
 * @version 0.1
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_qspi.h"

OPERATE_RET tkl_qspi_init(TUYA_QSPI_NUM_E port, const TUYA_QSPI_BASE_CFG_T *cfg)
{
    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)port;
    (void)cfg;

    /* TODO: map `tkl_qspi_init` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_qspi_deinit(TUYA_QSPI_NUM_E port)
{
    (void)port;

    /* TODO: map `tkl_qspi_deinit` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_qspi_send(TUYA_QSPI_NUM_E port, VOID_T *data, UINT32_T size)
{
    if (data == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)port;
    (void)data;
    (void)size;

    /* TODO: map `tkl_qspi_send` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_qspi_recv(TUYA_QSPI_NUM_E port, VOID_T *data, UINT32_T size)
{
    if (data == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)port;
    (void)data;
    (void)size;

    /* TODO: map `tkl_qspi_recv` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_qspi_comand(TUYA_QSPI_NUM_E port, TUYA_QSPI_CMD_T *command)
{
    if (command == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)port;
    (void)command;

    /* TODO: map `tkl_qspi_comand` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_qspi_irq_init(TUYA_QSPI_NUM_E port, TUYA_QSPI_IRQ_CB cb)
{
    (void)port;
    (void)cb;

    /* TODO: map `tkl_qspi_irq_init` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_qspi_irq_enable(TUYA_QSPI_NUM_E port)
{
    (void)port;

    /* TODO: map `tkl_qspi_irq_enable` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_qspi_irq_disable(TUYA_QSPI_NUM_E port)
{
    (void)port;

    /* TODO: map `tkl_qspi_irq_disable` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_qspi_force_cs_pin(TUYA_QSPI_NUM_E port, TUYA_GPIO_LEVEL_E level)
{
    (void)port;
    (void)level;

    /* TODO: map `tkl_qspi_force_cs_pin` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}
