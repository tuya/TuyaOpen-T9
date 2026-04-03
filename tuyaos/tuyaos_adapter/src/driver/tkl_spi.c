/**
 * @file tkl_spi.c
 * @brief Auto-generated TKL adapter stub. Candidate SPI adapter for T9.
 * @version 0.1
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_spi.h"

OPERATE_RET tkl_spi_init(TUYA_SPI_NUM_E port, CONST TUYA_SPI_BASE_CFG_T *cfg)
{
    /* 功能描述: */
    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)port;
    (void)cfg;

    /* TODO: map `tkl_spi_init` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_spi_deinit(TUYA_SPI_NUM_E port)
{
    /* 功能描述: */
    (void)port;

    /* TODO: map `tkl_spi_deinit` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_spi_send(TUYA_SPI_NUM_E port, VOID_T *data, UINT32_T size)
{
    /* 功能描述: */
    if (data == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)port;
    (void)data;
    (void)size;

    /* TODO: map `tkl_spi_send` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_spi_recv(TUYA_SPI_NUM_E port, VOID_T *data, UINT32_T size)
{
    /* 功能描述: */
    if (data == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)port;
    (void)data;
    (void)size;

    /* TODO: map `tkl_spi_recv` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_spi_transfer(TUYA_SPI_NUM_E port, VOID_T* send_buf, VOID_T* receive_buf, UINT32_T length)
{
    /* 功能描述: */
    (void)port;
    (void)send_buf;
    (void)receive_buf;
    (void)length;

    /* TODO: map `tkl_spi_transfer` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_spi_transfer_with_length(TUYA_SPI_NUM_E port, void* send_buf, uint32_t send_len, void* receive_buf, uint32_t receive_len)
{
    (void)port;
    (void)send_buf;
    (void)send_len;
    (void)receive_buf;
    (void)receive_len;

    /* TODO: map `tkl_spi_transfer_with_length` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_spi_abort_transfer(TUYA_SPI_NUM_E port)
{
    /* 功能描述: */
    (void)port;

    /* TODO: map `tkl_spi_abort_transfer` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_spi_get_status(TUYA_SPI_NUM_E port, TUYA_SPI_STATUS_T *status)
{
    /* 功能描述: */
    if (status == NULL) {
        return OPRT_INVALID_PARM;
    }

    (void)port;
    (void)status;

    /* TODO: map `tkl_spi_get_status` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_spi_irq_init(TUYA_SPI_NUM_E port, TUYA_SPI_IRQ_CB cb)
{
    /* 功能描述: */
    (void)port;
    (void)cb;

    /* TODO: map `tkl_spi_irq_init` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_spi_irq_enable(TUYA_SPI_NUM_E port)
{
    /* 功能描述: */
    (void)port;

    /* TODO: map `tkl_spi_irq_enable` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_spi_irq_disable(TUYA_SPI_NUM_E port)
{
    /* 功能描述: */
    (void)port;

    /* TODO: map `tkl_spi_irq_disable` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

INT32_T tkl_spi_get_data_count(TUYA_SPI_NUM_E port)
{
    /* 功能描述: */
    (void)port;

    /* TODO: map `tkl_spi_get_data_count` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_spi_ioctl(TUYA_SPI_NUM_E port, UINT32_T cmd, VOID *args)
{
    (void)port;
    (void)cmd;
    (void)args;

    /* TODO: map `tkl_spi_ioctl` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}

UINT32_T tkl_spi_get_max_dma_data_length(VOID_T)
{
    /* TODO: map `tkl_spi_get_max_dma_data_length` to the vendor SDK implementation. */
    /* TODO: confirm pinmux, clock, interrupt, DMA, and error-code mapping. */
    return OPRT_NOT_SUPPORTED;
}
