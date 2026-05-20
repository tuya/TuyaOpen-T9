/*
 * tkl_sdio.c
 * Copyright (C) 2025 cc <cc@tuya>
 *
 * Distributed under terms of the TUYA license.
 */


#include <driver/gpio.h>
#include "gpio_driver.h"
#include "tuya_cloud_types.h"
#include "sdkconfig.h"

#define TKL_SDIO_WIDTH_1BIT      0
#define TKL_SDIO_WIDTH_4BIT      1
#define TKL_SDIO_WIDTH_8BIT      2

#define TKL_SDIO_G0_CLK         GPIO_2
#define TKL_SDIO_G0_CMD         GPIO_3
#define TKL_SDIO_G0_D0          GPIO_4
#define TKL_SDIO_G0_D1          GPIO_5
#define TKL_SDIO_G0_D2          GPIO_10
#define TKL_SDIO_G0_D3          GPIO_11

#define TKL_SDIO_G1_CLK         GPIO_14
#define TKL_SDIO_G1_CMD         GPIO_15
#define TKL_SDIO_G1_D0          GPIO_16
#define TKL_SDIO_G1_D1          GPIO_17
#define TKL_SDIO_G1_D2          GPIO_18
#define TKL_SDIO_G1_D3          GPIO_19

struct tkl_sdio_pin_s {
    gpio_id_t clk;
    gpio_id_t cmd;
    gpio_id_t d0;
    gpio_id_t d1;
    gpio_id_t d2;
    gpio_id_t d3;
};

static struct tkl_sdio_pin_s current_sdio_gpio = {
    .clk = TKL_SDIO_G0_CLK,
    .cmd = TKL_SDIO_G0_CMD,
    .d0  = TKL_SDIO_G0_D0,
    .d1  = TKL_SDIO_G0_D1,
    .d2  = TKL_SDIO_G0_D2,
    .d3  = TKL_SDIO_G0_D3,
};

static uint8_t current_sdio_width = TKL_SDIO_WIDTH_1BIT;
static uint8_t last_sdio_width = TKL_SDIO_WIDTH_1BIT;
static uint8_t last_sdio_gpio_valid = 0;
static struct tkl_sdio_pin_s last_sdio_gpio = {0};

void __tkl_sdio_set_clk_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.clk = pin;
}

void __tkl_sdio_set_cmd_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.cmd = pin;
}

void __tkl_sdio_set_d0_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.d0 = pin;
}

void __tkl_sdio_set_d1_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.d1 = pin;
}

void __tkl_sdio_set_d2_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.d2 = pin;
}

void __tkl_sdio_set_d3_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.d3 = pin;
}

void user_sdio_set_width(uint8_t width_sel)
{
    current_sdio_width = width_sel;
}

static const struct tkl_sdio_pin_s s_valid_pin_groups[] = {
    {TKL_SDIO_G0_CLK, TKL_SDIO_G0_CMD, TKL_SDIO_G0_D0, TKL_SDIO_G0_D1, TKL_SDIO_G0_D2, TKL_SDIO_G0_D3},
    {TKL_SDIO_G1_CLK, TKL_SDIO_G1_CMD, TKL_SDIO_G1_D0, TKL_SDIO_G1_D1, TKL_SDIO_G1_D2, TKL_SDIO_G1_D3},
};

static const struct tkl_sdio_pin_s *__tkl_sdio_gpio_get_group(void);

static int __tkl_sdio_is_group0(const struct tkl_sdio_pin_s *group)
{
    if (group == NULL) {
        return 0;
    }

    return (group->clk == TKL_SDIO_G0_CLK) &&
           (group->cmd == TKL_SDIO_G0_CMD) &&
           (group->d0 == TKL_SDIO_G0_D0) &&
           (group->d1 == TKL_SDIO_G0_D1) &&
           (group->d2 == TKL_SDIO_G0_D2) &&
           (group->d3 == TKL_SDIO_G0_D3);
}

uint8_t sdio_board_use_sdio1(void)
{
    const struct tkl_sdio_pin_s *group = __tkl_sdio_gpio_get_group();

    return (group != NULL) && !__tkl_sdio_is_group0(group);
}

static const struct tkl_sdio_pin_s *__tkl_sdio_gpio_get_group(void)
{
    for (size_t i = 0; i < sizeof(s_valid_pin_groups) / sizeof(s_valid_pin_groups[0]); i++) {
        if ((current_sdio_gpio.clk == s_valid_pin_groups[i].clk) &&
            (current_sdio_gpio.cmd == s_valid_pin_groups[i].cmd) &&
            (current_sdio_gpio.d0  == s_valid_pin_groups[i].d0)
            && ((current_sdio_width != TKL_SDIO_WIDTH_4BIT) ||
                ((current_sdio_gpio.d1  == s_valid_pin_groups[i].d1) &&
                 (current_sdio_gpio.d2  == s_valid_pin_groups[i].d2) &&
                 (current_sdio_gpio.d3  == s_valid_pin_groups[i].d3)))
        ) {
            return &s_valid_pin_groups[i];
        }
    }

    return NULL;
}

static void __tkl_sdio_gpio_unmap_group(const struct tkl_sdio_pin_s *group, uint8_t width)
{
    if (group == NULL) {
        return;
    }

    gpio_dev_unmap(group->clk);
    gpio_dev_unmap(group->cmd);
    gpio_dev_unmap(group->d0);
    if (width == TKL_SDIO_WIDTH_4BIT) {
        gpio_dev_unmap(group->d1);
        gpio_dev_unmap(group->d2);
        gpio_dev_unmap(group->d3);
    } else if ((width == TKL_SDIO_WIDTH_1BIT) && (!__tkl_sdio_is_group0(group))) {
        gpio_dev_unmap(group->d3);
    }
}

static void __tkl_sdio_gpio_map_all(const struct tkl_sdio_pin_s *group, uint8_t width)
{
    if (group == NULL) {
        return;
    }

    if (__tkl_sdio_is_group0(group)) {
        gpio_dev_map(group->clk, GPIO_DEV_SDIO_HOST_CLK);
        gpio_dev_map(group->cmd, GPIO_DEV_SDIO_HOST_CMD);
        gpio_dev_map(group->d0, GPIO_DEV_SDIO_HOST_DATA0);
        if (width == TKL_SDIO_WIDTH_4BIT) {
            gpio_dev_map(group->d1, GPIO_DEV_SDIO_HOST_DATA1);
            gpio_dev_map(group->d2, GPIO_DEV_SDIO_HOST_DATA2);
            gpio_dev_map(group->d3, GPIO_DEV_SDIO_HOST_DATA3);
        }
    } else {
        gpio_dev_map(group->clk, GPIO_DEV_SDIO1_HOST_CLK);
        gpio_dev_map(group->cmd, GPIO_DEV_SDIO1_HOST_CMD);
        gpio_dev_map(group->d0, GPIO_DEV_SDIO1_HOST_DATA0);
        if (width == TKL_SDIO_WIDTH_4BIT) {
            gpio_dev_map(group->d1, GPIO_DEV_SDIO1_HOST_DATA1);
            gpio_dev_map(group->d2, GPIO_DEV_SDIO1_HOST_DATA2);
            gpio_dev_map(group->d3, GPIO_DEV_SDIO1_HOST_DATA3);
        }
    }
}

static void __tkl_sdio_gpio_config_data3_high(const struct tkl_sdio_pin_s *group)
{
    if (group == NULL) {
        return;
    }

    gpio_dev_unmap(group->d3);
    bk_gpio_enable_output(group->d3);
    bk_gpio_set_output_high(group->d3);
    bk_gpio_set_capacity(group->d3, GPIO_DRIVER_CAPACITY_3);
}

static void __tkl_sdio_gpio_config_all(const struct tkl_sdio_pin_s *group, uint8_t width)
{
    if (group == NULL) {
        return;
    }

    bk_gpio_pull_up(group->cmd);
    bk_gpio_pull_up(group->clk);
    bk_gpio_pull_up(group->d0);

    bk_gpio_set_capacity(group->cmd, GPIO_DRIVER_CAPACITY_3);
    bk_gpio_set_capacity(group->clk, GPIO_DRIVER_CAPACITY_3);
    bk_gpio_set_capacity(group->d0, GPIO_DRIVER_CAPACITY_3);

    if (width == TKL_SDIO_WIDTH_4BIT) {
        bk_gpio_pull_up(group->d1);
        bk_gpio_pull_up(group->d2);
        bk_gpio_pull_up(group->d3);

        bk_gpio_set_capacity(group->d1, GPIO_DRIVER_CAPACITY_3);
        bk_gpio_set_capacity(group->d2, GPIO_DRIVER_CAPACITY_3);
        bk_gpio_set_capacity(group->d3, GPIO_DRIVER_CAPACITY_3);
    } else if ((width == TKL_SDIO_WIDTH_1BIT) && (!__tkl_sdio_is_group0(group))) {
        __tkl_sdio_gpio_config_data3_high(group);
    }
}

void user_sdio_gpio_init(void)
{
    const struct tkl_sdio_pin_s *group = __tkl_sdio_gpio_get_group();

    if (group == NULL) {
        tkl_log_output("invalid sdio pin group clk=%d cmd=%d d0=%d d1=%d d2=%d d3=%d\r\n",
                current_sdio_gpio.clk, current_sdio_gpio.cmd, current_sdio_gpio.d0,
                current_sdio_gpio.d1, current_sdio_gpio.d2, current_sdio_gpio.d3);
        return;
    }

    if (current_sdio_width > TKL_SDIO_WIDTH_4BIT) {
        tkl_log_output("unsupported sdio width=%d, fallback to 1bit\r\n", current_sdio_width);
        current_sdio_width = TKL_SDIO_WIDTH_1BIT;
    }

    if (last_sdio_gpio_valid) {
        __tkl_sdio_gpio_unmap_group(&last_sdio_gpio, last_sdio_width);
    }

    __tkl_sdio_gpio_unmap_group(group, current_sdio_width);
    __tkl_sdio_gpio_map_all(group, current_sdio_width);
    __tkl_sdio_gpio_config_all(group, current_sdio_width);
    last_sdio_gpio = *group;
    last_sdio_width = current_sdio_width;
    last_sdio_gpio_valid = 1;
}
