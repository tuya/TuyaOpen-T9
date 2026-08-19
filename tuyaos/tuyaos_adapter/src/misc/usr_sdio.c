/*
 * tkl_sdio.c
 * Copyright (C) 2025 cc <cc@tuya>
 *
 * Distributed under terms of the TUYA license.
 */


#include "tkl_gpio.h"
#include "tkl_output.h"
#include "tuya_cloud_types.h"
#include <driver/gpio.h>
#include "gpio_driver.h"
#include "sdkconfig.h"

#define TKL_SDIO_WIDTH_1BIT      0
#define TKL_SDIO_WIDTH_4BIT      1
#define TKL_SDIO_WIDTH_8BIT      2

#define TKL_SDIO_G0_CLK         TUYA_GPIO_NUM_2
#define TKL_SDIO_G0_CMD         TUYA_GPIO_NUM_3
#define TKL_SDIO_G0_D0          TUYA_GPIO_NUM_4
#define TKL_SDIO_G0_D1          TUYA_GPIO_NUM_5
#define TKL_SDIO_G0_D2          TUYA_GPIO_NUM_10
#define TKL_SDIO_G0_D3          TUYA_GPIO_NUM_11

#define TKL_SDIO_G1_CLK         TUYA_GPIO_NUM_14
#define TKL_SDIO_G1_CMD         TUYA_GPIO_NUM_15
#define TKL_SDIO_G1_D0          TUYA_GPIO_NUM_16
#define TKL_SDIO_G1_D1          TUYA_GPIO_NUM_17
#define TKL_SDIO_G1_D2          TUYA_GPIO_NUM_18
#define TKL_SDIO_G1_D3          TUYA_GPIO_NUM_19

struct tkl_sdio_pin_s {
    TUYA_GPIO_NUM_E clk;
    TUYA_GPIO_NUM_E cmd;
    TUYA_GPIO_NUM_E d0;
    TUYA_GPIO_NUM_E d1;
    TUYA_GPIO_NUM_E d2;
    TUYA_GPIO_NUM_E d3;
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
/* Default matches previous hardcoded CAP_3. Board may override via
 * user_sdio_set_drive_capability() before sdio_gpio_init(). */
static TUYA_GPIO_DRIVE_CAP_E s_sdio_drive_cap = TUYA_GPIO_DRIVE_CAP_3;

static VOID_T __tkl_sdio_gpio_apply_drive(const struct tkl_sdio_pin_s *group, uint8_t width);

void __tkl_sdio_set_clk_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.clk = (TUYA_GPIO_NUM_E)pin;
}

void __tkl_sdio_set_cmd_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.cmd = (TUYA_GPIO_NUM_E)pin;
}

void __tkl_sdio_set_d0_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.d0 = (TUYA_GPIO_NUM_E)pin;
}

void __tkl_sdio_set_d1_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.d1 = (TUYA_GPIO_NUM_E)pin;
}

void __tkl_sdio_set_d2_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.d2 = (TUYA_GPIO_NUM_E)pin;
}

void __tkl_sdio_set_d3_pin(TUYA_SDIO_NUM_E port, TUYA_PIN_NAME_E pin)
{
    (void)port;
    current_sdio_gpio.d3 = (TUYA_GPIO_NUM_E)pin;
}

void user_sdio_set_width(uint8_t width_sel)
{
    current_sdio_width = width_sel;
}

/**
 * @brief Check whether the SDIO pad drive level is supported on this SoC.
 * @param[in] strength requested drive capability
 * @return TRUE if strength is CAP_0..CAP_3, FALSE otherwise
 */
static BOOL_T __tkl_sdio_drive_cap_valid(TUYA_GPIO_DRIVE_CAP_E strength)
{
    if ((strength >= TUYA_GPIO_DRIVE_CAP_MAX) || (strength > TUYA_GPIO_DRIVE_CAP_3)) {
        return FALSE;
    }

    return TRUE;
}

/**
 * @brief Set SDIO bus pad drive capability for the current board.
 * @param[in] strength TUYA_GPIO_DRIVE_CAP_0 .. TUYA_GPIO_DRIVE_CAP_3
 * @return none
 * @note Call before SDIO host init (same timing as pinmux). If GPIO has
 *       already been mapped, the new level is applied immediately. CAP_4 is
 *       not supported on BK7259 and is ignored.
 */
void user_sdio_set_drive_capability(TUYA_GPIO_DRIVE_CAP_E strength)
{
    if (!__tkl_sdio_drive_cap_valid(strength)) {
        tkl_log_output("unsupported sdio drive_cap=%d, keep %d\r\n",
                       (INT_T)strength, (INT_T)s_sdio_drive_cap);
        return;
    }

    s_sdio_drive_cap = strength;
    if (last_sdio_gpio_valid) {
        __tkl_sdio_gpio_apply_drive(&last_sdio_gpio, last_sdio_width);
    }
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

/**
 * @brief Release an SDIO pad from its previous mux (GPIO input floating).
 * @param[in] pin GPIO pin id
 * @return none
 */
static VOID_T __tkl_sdio_gpio_unmap_pin(TUYA_GPIO_NUM_E pin)
{
    (VOID_T)tkl_gpio_deinit(pin);
}

/**
 * @brief Configure an SDIO pad as input pull-up with the board drive level.
 * @param[in] pin GPIO pin id
 * @return none
 * @note Must run before gpio_dev_map(). tkl_gpio_init() unmaps the pad;
 *       mapping afterwards only changes the function select and keeps pull/cap.
 */
static VOID_T __tkl_sdio_gpio_cfg_input_pullup(TUYA_GPIO_NUM_E pin)
{
    TUYA_GPIO_BASE_CFG_T cfg;

    cfg.mode = TUYA_GPIO_PULLUP;
    cfg.direct = TUYA_GPIO_INPUT;
    cfg.level = TUYA_GPIO_LEVEL_HIGH;
    cfg.drive_cap = s_sdio_drive_cap;
    (VOID_T)tkl_gpio_init(pin, &cfg);
}

static void __tkl_sdio_gpio_unmap_group(const struct tkl_sdio_pin_s *group, uint8_t width)
{
    if (group == NULL) {
        return;
    }

    __tkl_sdio_gpio_unmap_pin(group->clk);
    __tkl_sdio_gpio_unmap_pin(group->cmd);
    __tkl_sdio_gpio_unmap_pin(group->d0);
    if (width == TKL_SDIO_WIDTH_4BIT) {
        __tkl_sdio_gpio_unmap_pin(group->d1);
        __tkl_sdio_gpio_unmap_pin(group->d2);
        __tkl_sdio_gpio_unmap_pin(group->d3);
    } else if ((width == TKL_SDIO_WIDTH_1BIT) && (!__tkl_sdio_is_group0(group))) {
        __tkl_sdio_gpio_unmap_pin(group->d3);
    }
}

static void __tkl_sdio_gpio_map_all(const struct tkl_sdio_pin_s *group, uint8_t width)
{
    if (group == NULL) {
        return;
    }

    if (__tkl_sdio_is_group0(group)) {
        gpio_dev_map((gpio_id_t)group->clk, GPIO_DEV_SDIO_HOST_CLK);
        gpio_dev_map((gpio_id_t)group->cmd, GPIO_DEV_SDIO_HOST_CMD);
        gpio_dev_map((gpio_id_t)group->d0, GPIO_DEV_SDIO_HOST_DATA0);
        if (width == TKL_SDIO_WIDTH_4BIT) {
            gpio_dev_map((gpio_id_t)group->d1, GPIO_DEV_SDIO_HOST_DATA1);
            gpio_dev_map((gpio_id_t)group->d2, GPIO_DEV_SDIO_HOST_DATA2);
            gpio_dev_map((gpio_id_t)group->d3, GPIO_DEV_SDIO_HOST_DATA3);
        }
    } else {
        gpio_dev_map((gpio_id_t)group->clk, GPIO_DEV_SDIO1_HOST_CLK);
        gpio_dev_map((gpio_id_t)group->cmd, GPIO_DEV_SDIO1_HOST_CMD);
        gpio_dev_map((gpio_id_t)group->d0, GPIO_DEV_SDIO1_HOST_DATA0);
        if (width == TKL_SDIO_WIDTH_4BIT) {
            gpio_dev_map((gpio_id_t)group->d1, GPIO_DEV_SDIO1_HOST_DATA1);
            gpio_dev_map((gpio_id_t)group->d2, GPIO_DEV_SDIO1_HOST_DATA2);
            gpio_dev_map((gpio_id_t)group->d3, GPIO_DEV_SDIO1_HOST_DATA3);
        }
    }
}

/**
 * @brief Hold SDIO DATA3 high as GPIO output (1-bit mode on group1).
 * @param[in] group SDIO pin group
 * @return none
 */
static void __tkl_sdio_gpio_config_data3_high(const struct tkl_sdio_pin_s *group)
{
    TUYA_GPIO_BASE_CFG_T cfg;

    if (group == NULL) {
        return;
    }

    cfg.mode = TUYA_GPIO_PUSH_PULL;
    cfg.direct = TUYA_GPIO_OUTPUT;
    cfg.level = TUYA_GPIO_LEVEL_HIGH;
    cfg.drive_cap = s_sdio_drive_cap;
    (VOID_T)tkl_gpio_init(group->d3, &cfg);
    (VOID_T)tkl_gpio_write(group->d3, TUYA_GPIO_LEVEL_HIGH);
}

static void __tkl_sdio_gpio_config_all(const struct tkl_sdio_pin_s *group, uint8_t width)
{
    if (group == NULL) {
        return;
    }

    __tkl_sdio_gpio_cfg_input_pullup(group->cmd);
    __tkl_sdio_gpio_cfg_input_pullup(group->clk);
    __tkl_sdio_gpio_cfg_input_pullup(group->d0);

    if (width == TKL_SDIO_WIDTH_4BIT) {
        __tkl_sdio_gpio_cfg_input_pullup(group->d1);
        __tkl_sdio_gpio_cfg_input_pullup(group->d2);
        __tkl_sdio_gpio_cfg_input_pullup(group->d3);
    } else if ((width == TKL_SDIO_WIDTH_1BIT) && (!__tkl_sdio_is_group0(group))) {
        __tkl_sdio_gpio_config_data3_high(group);
    }
}

/**
 * @brief Re-apply the board SDIO drive level after mux.
 * @param[in] group SDIO pin group
 * @param[in] width bus width
 * @return none
 */
static VOID_T __tkl_sdio_gpio_apply_drive(const struct tkl_sdio_pin_s *group, uint8_t width)
{
    if (group == NULL) {
        return;
    }

    (VOID_T)tkl_gpio_set_drive_capability(group->cmd, s_sdio_drive_cap);
    (VOID_T)tkl_gpio_set_drive_capability(group->clk, s_sdio_drive_cap);
    (VOID_T)tkl_gpio_set_drive_capability(group->d0, s_sdio_drive_cap);
    if (width == TKL_SDIO_WIDTH_4BIT) {
        (VOID_T)tkl_gpio_set_drive_capability(group->d1, s_sdio_drive_cap);
        (VOID_T)tkl_gpio_set_drive_capability(group->d2, s_sdio_drive_cap);
        (VOID_T)tkl_gpio_set_drive_capability(group->d3, s_sdio_drive_cap);
    } else if ((width == TKL_SDIO_WIDTH_1BIT) && (!__tkl_sdio_is_group0(group))) {
        (VOID_T)tkl_gpio_set_drive_capability(group->d3, s_sdio_drive_cap);
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
    /* Pad pull-up / drive first: tkl_gpio_init() unmaps. Then mux to SDIO. */
    __tkl_sdio_gpio_config_all(group, current_sdio_width);
    __tkl_sdio_gpio_map_all(group, current_sdio_width);
    __tkl_sdio_gpio_apply_drive(group, current_sdio_width);
    last_sdio_gpio = *group;
    last_sdio_width = current_sdio_width;
    last_sdio_gpio_valid = 1;
}
