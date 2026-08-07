/****************************************************************************
 * @file tkl_pinmux.c
 * @brief this module is used to tkl_pinmux
 * @version 0.0.1
 * @date 2023-06-07
 *
 * @copyright Copyright(C) 2021-2022 Tuya Inc. All Rights Reserved.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "tkl_pinmux.h"
#include "driver/hal/hal_adc_types.h"
#include <driver/gpio.h>
#include "gpio_driver.h"

/*
 * Map a Tuya pin function to the SoC GPIO device function. Returns
 * GPIO_DEV_INVALID for funcs that are handled by per-driver setters below
 * (spi/sdio/...) or that have no direct GPIO device mapping.
 *
 * This lets tkl_io_pinmux_config() bind a *specific* pin to a peripheral
 * function via gpio_dev_unprotect_map() before the peripheral is initialised.
 * UART default ownership remains GPIO_DEFAULT_DEV_CONFIG (usr_gpio_cfg).
 */
static gpio_dev_t __tkl_pinmux_func_to_gpio_dev(TUYA_PIN_FUNC_E pin_func)
{
    switch (pin_func) {
        case TUYA_IIC0_SCL:  return GPIO_DEV_I2C0_SCL;
        case TUYA_IIC0_SDA:  return GPIO_DEV_I2C0_SDA;
        case TUYA_IIC1_SCL:  return GPIO_DEV_I2C1_SCL;
        case TUYA_IIC1_SDA:  return GPIO_DEV_I2C1_SDA;
        case TUYA_IIC2_SCL:  return GPIO_DEV_I2C2_SCL;
        case TUYA_IIC2_SDA:  return GPIO_DEV_I2C2_SDA;
        /* SPI is NOT handled here: tkl_spi re-maps its pads in spi_init_gpio()
         * using its own (configurable) pin getters, which would override a
         * gpio_dev_unprotect_map() done here. SPI pins are set via the
         * __tkl_spi_set_*_pin() hooks in the switch below instead. */
#ifdef TUYA_QSPI0_CLK
        case TUYA_QSPI0_CLK: return GPIO_DEV_QSPI0_CLK;
        case TUYA_QSPI0_CS:  return GPIO_DEV_QSPI0_CSN;
        case TUYA_QSPI0_IO0: return GPIO_DEV_QSPI0_IO0;
        case TUYA_QSPI0_IO1: return GPIO_DEV_QSPI0_IO1;
        case TUYA_QSPI0_IO2: return GPIO_DEV_QSPI0_IO2;
        case TUYA_QSPI0_IO3: return GPIO_DEV_QSPI0_IO3;
        case TUYA_QSPI1_CLK: return GPIO_DEV_QSPI1_CLK;
        case TUYA_QSPI1_CS:  return GPIO_DEV_QSPI1_CSN;
        case TUYA_QSPI1_IO0: return GPIO_DEV_QSPI1_IO0;
        case TUYA_QSPI1_IO1: return GPIO_DEV_QSPI1_IO1;
        case TUYA_QSPI1_IO2: return GPIO_DEV_QSPI1_IO2;
        case TUYA_QSPI1_IO3: return GPIO_DEV_QSPI1_IO3;
#endif
        case TUYA_PWM0:      return GPIO_DEV_PWM0;
        case TUYA_PWM1:      return GPIO_DEV_PWM1;
        case TUYA_PWM2:      return GPIO_DEV_PWM2;
        case TUYA_PWM3:      return GPIO_DEV_PWM3;
        case TUYA_PWM4:      return GPIO_DEV_PWM4;
        case TUYA_PWM5:      return GPIO_DEV_PWM5;
        case TUYA_PWM6:      return GPIO_DEV_PWM6;
        case TUYA_PWM7:      return GPIO_DEV_PWM7;
        case TUYA_PWM8:      return GPIO_DEV_PWM8;
        case TUYA_PWM9:      return GPIO_DEV_PWM9;
        case TUYA_PWM10:     return GPIO_DEV_PWM10;
        case TUYA_PWM11:     return GPIO_DEV_PWM11;
        case TUYA_I2S0_SCK:   return GPIO_DEV_I2S0_CLK;
        case TUYA_I2S0_WS:    return GPIO_DEV_I2S0_SYNC;
        case TUYA_I2S0_SDO_0: return GPIO_DEV_I2S0_DOUT;
        case TUYA_I2S0_SDI_0: return GPIO_DEV_I2S0_DIN;
        case TUYA_I2S1_SCK:   return GPIO_DEV_I2S1_CLK;
        case TUYA_I2S1_WS:    return GPIO_DEV_I2S1_SYNC;
        case TUYA_I2S1_SDO_0: return GPIO_DEV_I2S1_DOUT;
        case TUYA_I2S1_SDI_0: return GPIO_DEV_I2S1_DIN;
#ifdef TUYA_DMIC0_CLK
        case TUYA_DMIC0_CLK: return GPIO_DEV_DMIC0_CLK;
        case TUYA_DMIC0_DAT: return GPIO_DEV_DMIC0_DAT;
        case TUYA_DMIC1_CLK: return GPIO_DEV_DMIC1_CLK;
        case TUYA_DMIC1_DAT: return GPIO_DEV_DMIC1_DAT;
#endif
        /*
         * ADC is an ANALOG function (datasheet Table 3-5), not a digital IOMUX
         * function: the AP-side IOMUX map has no ADC func code, and the SARADC +
         * its pad routing live on the CP side. So ADC is NOT mapped via
         * gpio_dev_unprotect_map() here; it is delegated to bk_adc_chan_init_gpio()
         * (CP IPC) in tkl_io_pinmux_config() instead.
         */
        /*
         * UART pads follow usr_gpio_cfg + driver uart_map_func(); runtime
         * pinmux uses the same gpio_dev_unprotect_map() path as I2C/PWM.
         */
        case TUYA_UART0_TX:  return GPIO_DEV_UART0_TXD;
        case TUYA_UART0_RX:  return GPIO_DEV_UART0_RXD;
        case TUYA_UART1_TX:  return GPIO_DEV_UART1_TXD;
        case TUYA_UART1_RX:  return GPIO_DEV_UART1_RXD;
        case TUYA_UART2_TX:  return GPIO_DEV_UART2_TXD;
        case TUYA_UART2_RX:  return GPIO_DEV_UART2_RXD;
        /*
         * SPI/SDIO are NOT mapped here: their drivers re-map the pads in
         * their own init() using configurable pin getters, which would override
         * a gpio_dev_unprotect_map() done here (and leave two pads on the same
         * function). They are routed through their __tkl_*_set_*_pin() hooks in
         * tkl_io_pinmux_config() instead.
         * DAC / analog-MIC have no GPIO pinmux (fixed analog pads).
         */
        default:             return GPIO_DEV_INVALID;
    }
}

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
// extern VOID_T __tkl_i2c_set_scl_pin(TUYA_I2C_NUM_E port, const TUYA_PIN_NAME_E scl_pin);
// extern VOID_T __tkl_i2c_set_sda_pin(TUYA_I2C_NUM_E port, const TUYA_PIN_NAME_E sda_pin);

extern void __tkl_spi_set_cs_pin(TUYA_SPI_NUM_E port, TUYA_PIN_NAME_E pin);
extern void __tkl_spi_set_clk_pin(TUYA_SPI_NUM_E port, TUYA_PIN_NAME_E pin);
extern void __tkl_spi_set_mosi_pin(TUYA_SPI_NUM_E port, TUYA_PIN_NAME_E pin);
extern void __tkl_spi_set_miso_pin(TUYA_SPI_NUM_E port, TUYA_PIN_NAME_E pin);

extern void __tkl_sdio_set_clk_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin);
extern void __tkl_sdio_set_cmd_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin);
extern void __tkl_sdio_set_d0_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin);
extern void __tkl_sdio_set_d1_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin);
extern void __tkl_sdio_set_d2_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin);
extern void __tkl_sdio_set_d3_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin);

/**
 * @brief tuya io pinmux func
 *
 * @param[in] pin: pin number
 * @param[in] pin_func: pin function
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_io_pinmux_config(TUYA_PIN_NAME_E pin, TUYA_PIN_FUNC_E pin_func)
{
    /*
     * Bind a specific pin to a peripheral function directly, bypassing the
     * board's GPIO_DEFAULT_DEV_CONFIG table. Call this BEFORE the peripheral's
     * init(): the pad is already routed, so the driver's own
     * gpio_dev_map_by_func() lookup becoming a no-op / harmless warning does
     * not matter (it never unmaps an already-mapped pad).
     */
    gpio_dev_t dev = __tkl_pinmux_func_to_gpio_dev(pin_func);
    if (dev != GPIO_DEV_INVALID) {
        /*
         * Same mapping as gpio_dev_map_by_func() — unmap the pad first, then
         * map it to the requested function — but WITHOUT its
         * gpio_find_row_by_func()/GPIO_DEFAULT_DEV_CONFIG lookup, so any pin can
         * be routed to any function regardless of the board default table.
         * gpio_dev_unprotect_map() resolves dev->func_code via the IOMX map
         * itself (no default-table dependency).
         */
        gpio_dev_unmap((gpio_id_t)pin);
        if (gpio_dev_unprotect_map((gpio_id_t)pin, dev) != BK_OK) {
            return OPRT_COM_ERROR;
        }
        /*
         * I2C is open-drain: without a pull-up the bus floats and the slave
         * never ACKs. The board default table doesn't route I2C, so no pull was
         * applied; enable the internal pull-up here. (External 4.7k~10k pull-ups
         * are still recommended for reliable/fast I2C.)
         */
        if ((pin_func == TUYA_IIC0_SCL) || (pin_func == TUYA_IIC0_SDA) ||
            (pin_func == TUYA_IIC1_SCL) || (pin_func == TUYA_IIC1_SDA) ||
            (pin_func == TUYA_IIC2_SCL) || (pin_func == TUYA_IIC2_SDA)) {
            (void)bk_gpio_pull_up((gpio_id_t)pin);
        }
        return OPRT_OK;
    }

    /*
     * SPI / SDIO drivers re-map their pads in their own init() from a
     * configurable pin. Set that pin here so the driver picks it up; a bare
     * gpio_dev_unprotect_map() would be overridden by the driver.
     * UART pins are owned by usr_gpio_cfg + the path above.
     */
    switch (pin_func) {
        case TUYA_SPI0_CLK:  __tkl_spi_set_clk_pin(TUYA_SPI_NUM_0, pin);  break;
        case TUYA_SPI0_MOSI: __tkl_spi_set_mosi_pin(TUYA_SPI_NUM_0, pin); break;
        case TUYA_SPI0_MISO: __tkl_spi_set_miso_pin(TUYA_SPI_NUM_0, pin); break;
        case TUYA_SPI0_CS:   __tkl_spi_set_cs_pin(TUYA_SPI_NUM_0, pin);   break;
        case TUYA_SPI1_CLK:  __tkl_spi_set_clk_pin(TUYA_SPI_NUM_1, pin);  break;
        case TUYA_SPI1_MOSI: __tkl_spi_set_mosi_pin(TUYA_SPI_NUM_1, pin); break;
        case TUYA_SPI1_MISO: __tkl_spi_set_miso_pin(TUYA_SPI_NUM_1, pin); break;
        case TUYA_SPI1_CS:   __tkl_spi_set_cs_pin(TUYA_SPI_NUM_1, pin);   break;
        case TUYA_SDIO_CLK:   __tkl_sdio_set_clk_pin(TUYA_SDIO_NUM_0, pin); break;
        case TUYA_SDIO_CMD:   __tkl_sdio_set_cmd_pin(TUYA_SDIO_NUM_0, pin); break;
        case TUYA_SDIO_DATA0: __tkl_sdio_set_d0_pin(TUYA_SDIO_NUM_0, pin);  break;
        case TUYA_SDIO_DATA1: __tkl_sdio_set_d1_pin(TUYA_SDIO_NUM_0, pin);  break;
        case TUYA_SDIO_DATA2: __tkl_sdio_set_d2_pin(TUYA_SDIO_NUM_0, pin);  break;
        case TUYA_SDIO_DATA3: __tkl_sdio_set_d3_pin(TUYA_SDIO_NUM_0, pin);  break;
        default:
            return OPRT_NOT_SUPPORTED;
    }

    return OPRT_OK;
}
INT32_T tkl_io_pin_to_func(UINT32_T pin, TUYA_PIN_TYPE_E pin_type)
{
    INT32_T port_channel = OPRT_NOT_SUPPORTED;

    switch (pin_type) {
        case TUYA_IO_TYPE_PWM:                  // all pwm channels belong to one port
            if (TUYA_IO_PIN_18 == pin) {
                port_channel = 0;
            } else if (TUYA_IO_PIN_24 == pin) {
                port_channel = 1;
            } else if (TUYA_IO_PIN_32 == pin) {
                port_channel = 2;
            } else if (TUYA_IO_PIN_34 == pin) {
                port_channel = 3;
            } else if (TUYA_IO_PIN_36 == pin) {
                port_channel = 4;
            } else if (TUYA_IO_PIN_19 == pin) {
                port_channel = 5;
            } else if (TUYA_IO_PIN_8 == pin) {
                port_channel = 6;
            } else if (TUYA_IO_PIN_9 == pin) {
                port_channel = 7;
            } else if (TUYA_IO_PIN_25 == pin) {
                port_channel = 8;
            } else if (TUYA_IO_PIN_33 == pin) {
                port_channel = 9;
            } else if (TUYA_IO_PIN_35 == pin) {
                port_channel = 10;
            } else if (TUYA_IO_PIN_37 == pin) {
                port_channel = 11;
            }
            break;
        case TUYA_IO_TYPE_ADC:
            if (TUYA_IO_PIN_25 == pin) {
                port_channel = ADC_1;
            } else if (TUYA_IO_PIN_24 == pin) {
                port_channel = ADC_2;
            } else if (TUYA_IO_PIN_23 == pin) {
                port_channel = ADC_3;
            } else if (TUYA_IO_PIN_28 == pin) {
                port_channel = ADC_4;
            } else if (TUYA_IO_PIN_22 == pin) {
                port_channel = ADC_5;
            } else if (TUYA_IO_PIN_21 == pin) {
                port_channel = ADC_6;
            } else if (TUYA_IO_PIN_8 == pin) {
                port_channel = ADC_10;
            } else if (TUYA_IO_PIN_1 == pin) {
                port_channel = ADC_13;
            } else if (TUYA_IO_PIN_0 == pin) {
                port_channel = ADC_12;
            }
            break;
        case TUYA_IO_TYPE_DAC:
            break;
        case TUYA_IO_TYPE_UART:
            break;
        case TUYA_IO_TYPE_SPI:
            break;
        case TUYA_IO_TYPE_I2C:
            break;
        case TUYA_IO_TYPE_I2S:
            break;
        case TUYA_IO_TYPE_GPIO:
            break;
        default:
            break;
    }

    return port_channel;
}


