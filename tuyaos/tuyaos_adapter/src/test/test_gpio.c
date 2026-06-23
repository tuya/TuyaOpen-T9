/*
 * test_gpio.c
 * Copyright (C) 2024 cc <cc@tuya>
 *
 * Distributed under terms of the MIT license.
 */

#include "cli.h"
#include "cli_tuya_test.h"
#include "tkl_gpio.h"
#include "tkl_thread.h"
#include "tkl_system.h"
#include "sys_driver.h"
#include <stdint.h>

#define GPIO_TEST_PIN_MIN            0
#define GPIO_TEST_PIN_MAX            71
#define GPIO_TEST_LEVEL_WAIT_MS      5
#define GPIO_TEST_IRQ_TIMEOUT_MS     100
#define GPIO_TEST_RELAY_PERIOD_MS    (10 * 1000)
#define GPIO_TEST_CONCURRENT_LOOPS   100
#define GPIO_TEST_CONCURRENT_WORKERS 3
#define GPIO_TEST_CONCURRENT_TIMEOUT_MS 2000

/* P0/P1 are reserved for the UART1 log console (tx=P0/rx=P1). tkl_gpio blocks
 * both, so the API self-test drives two free pins instead of the old P1/P2. */
#define GPIO_API_TEST_PIN_A          TUYA_GPIO_NUM_2
#define GPIO_API_TEST_PIN_B          TUYA_GPIO_NUM_3

#define SYS_ANA_REG_BASE            (0x44010000)
#define LDO_ANA_REG                 (0x69)

extern void bk_printf(const char *fmt, ...);

static volatile uint32_t s_gpio_irq_count = 0;
static volatile TUYA_GPIO_NUM_E s_gpio_irq_last_pin = TUYA_GPIO_NUM_MAX;

static TaskHandle_t s_relay_test_thread = NULL;
static TUYA_GPIO_NUM_E s_relay_pin = TUYA_GPIO_NUM_MAX;

typedef struct {
    volatile uint32_t start;
    volatile uint32_t done;
    volatile uint32_t init_ok;
    volatile uint32_t init_busy;
    volatile uint32_t init_fail;
    TUYA_GPIO_NUM_E pin;
} gpio_test_init_race_ctx_t;

typedef struct {
    volatile uint32_t start;
    volatile uint32_t done;
    volatile uint32_t write_ok;
    volatile uint32_t write_fail;
    volatile uint32_t read_ok;
    volatile uint32_t read_fail;
    TUYA_GPIO_NUM_E pin;
} gpio_test_rw_race_ctx_t;

static gpio_test_init_race_ctx_t s_gpio_init_race_ctx;
static gpio_test_rw_race_ctx_t s_gpio_rw_race_ctx;

static void test_gpio_auxldo_enable(void)
{
    uint32_t reg = REG_READ(SYS_ANA_REG_BASE + LDO_ANA_REG * 4);

    reg &= ~((0x7 << 23) | (0xF << 19));
    reg |= (0x2 << 23) | (0x7 << 19) | (0x3 << 30);
    REG_WRITE(SYS_ANA_REG_BASE + LDO_ANA_REG * 4, reg);
}

static int gpio_test_is_num_str(const char *s)
{
    int i = 0;

    if ((s == NULL) || (s[0] == '\0')) {
        return 0;
    }

    while (s[i] != '\0') {
        if ((s[i] < '0') || (s[i] > '9')) {
            return 0;
        }
        i++;
    }

    return 1;
}

static int gpio_test_parse_pin(const char *s, TUYA_GPIO_NUM_E *pin)
{
    unsigned long val;

    if ((pin == NULL) || !gpio_test_is_num_str(s)) {
        return -1;
    }

    val = os_strtoul(s, NULL, 10);
    if (val > GPIO_TEST_PIN_MAX) {
        return -1;
    }

    *pin = (TUYA_GPIO_NUM_E)val;
    return 0;
}

static int gpio_test_is_auto_skip(TUYA_GPIO_NUM_E pin)
{
    /* Only P0/P1 are reserved (UART1 log console). P10/P11 are plain GPIOs
     * here even though the GPIO config still pins them to UART0. */
    return (pin == TUYA_GPIO_NUM_0) || (pin == TUYA_GPIO_NUM_1);
}

static void gpio_test_dump_pin_state(const char *tag, TUYA_GPIO_NUM_E pin, OPERATE_RET ret)
{
    bk_printf("[%s] pin=%d ret=%d\r\n", tag, pin, ret);
}

static void gpio_test_api_expect(const char *name, OPERATE_RET ret, OPERATE_RET expect,
                                 uint32_t *pass, uint32_t *fail)
{
    if (ret == expect) {
        (*pass)++;
        bk_printf("[api] PASS %-24s ret=%d\r\n", name, ret);
    } else {
        (*fail)++;
        bk_printf("[api] FAIL %-24s ret=%d expect=%d\r\n", name, ret, expect);
    }
}

static const char *gpio_test_irq_mode_name(TUYA_GPIO_IRQ_E mode)
{
    switch (mode) {
        case TUYA_GPIO_IRQ_RISE:
            return "rise";
        case TUYA_GPIO_IRQ_FALL:
            return "fall";
        case TUYA_GPIO_IRQ_LOW:
            return "low";
        case TUYA_GPIO_IRQ_HIGH:
            return "high";
        case TUYA_GPIO_IRQ_RISE_FALL:
            return "rise_fall";
        default:
            return "unknown";
    }
}

static void gpio_test_usage(void)
{
    bk_printf("xgpio list\r\n");
    bk_printf("xgpio api\r\n");
    bk_printf("xgpio concurrent <pin>\r\n");
    bk_printf("xgpio smoke <pin>\r\n");
    bk_printf("xgpio smoke all [start] [end]\r\n");
    bk_printf("xgpio pair <out_pin> <in_pin>\r\n");
    bk_printf("xgpio irq <drive_pin> <irq_pin> <rise|fall|high|low>\r\n");
    bk_printf("xgpio all [start] [end]\r\n");
    bk_printf("xgpio relay <pin>\r\n");
    bk_printf("compat: xgpio <pin> input|output <0|1|2>\r\n");
    bk_printf("compat: xgpio <pin> irq <rise|fall|low|high|start|stop>\r\n");
}

static void gpio_test_list(void)
{
    bk_printf("gpio test range: P%d ~ P%d\r\n", GPIO_TEST_PIN_MIN, GPIO_TEST_PIN_MAX);
    bk_printf("auto skip pins in xgpio all: P0 P1 (P0/P1=UART1 log)\r\n");
    bk_printf("re-init on same pin is not allowed before deinit\r\n");
    bk_printf("write/read/irq require pin init first\r\n");
    bk_printf("pair/all/irq electrical tests require external wiring\r\n");
    bk_printf("concurrent tests verify same-pin reentry and multi-thread access\r\n");
    bk_printf("recommended adjacent shorting: P2<->P3, P4<->P5, ...\r\n");
}

static void gpio_test_irq_cb(void *args)
{
    s_gpio_irq_count++;
    s_gpio_irq_last_pin = (TUYA_GPIO_NUM_E)(uintptr_t)args;
}

static void gpio_test_reset_irq_state(void)
{
    s_gpio_irq_count = 0;
    s_gpio_irq_last_pin = TUYA_GPIO_NUM_MAX;
}

static void gpio_test_input_cfg(TUYA_GPIO_BASE_CFG_T *cfg, TUYA_GPIO_MODE_E mode)
{
    cfg->direct = TUYA_GPIO_INPUT;
    cfg->mode = mode;
    cfg->level = TUYA_GPIO_LEVEL_NONE;
}

static void gpio_test_output_cfg(TUYA_GPIO_BASE_CFG_T *cfg, TUYA_GPIO_LEVEL_E level)
{
    cfg->direct = TUYA_GPIO_OUTPUT;
    cfg->mode = TUYA_GPIO_PUSH_PULL;
    cfg->level = level;
}

static int gpio_test_wait_irq(uint32_t timeout_ms)
{
    uint32_t waited = 0;

    while (waited < timeout_ms) {
        if (s_gpio_irq_count > 0) {
            return 0;
        }
        tkl_system_sleep(5);
        waited += 5;
    }

    return -1;
}

static void gpio_test_atomic_inc(volatile uint32_t *value)
{
    UINT_T irq_mask = tkl_system_enter_critical();
    (*value)++;
    tkl_system_exit_critical(irq_mask);
}

static int gpio_test_wait_counter(volatile uint32_t *value, uint32_t target, uint32_t timeout_ms)
{
    uint32_t waited = 0;

    while (waited < timeout_ms) {
        if (*value >= target) {
            return 0;
        }
        tkl_system_sleep(5);
        waited += 5;
    }

    return -1;
}

static void gpio_test_cleanup_pin(TUYA_GPIO_NUM_E pin)
{
    OPERATE_RET ret = tkl_gpio_deinit(pin);
    (void)ret;
}

static int gpio_test_smoke_one(TUYA_GPIO_NUM_E pin)
{
    OPERATE_RET ret;
    TUYA_GPIO_BASE_CFG_T cfg;
    int rc = -1;

    gpio_test_input_cfg(&cfg, TUYA_GPIO_PULLUP);
    ret = tkl_gpio_init(pin, &cfg);
    if (ret != OPRT_OK) {
        gpio_test_dump_pin_state("smoke-input-up-fail", pin, ret);
        return -1;
    }
    gpio_test_cleanup_pin(pin);

    gpio_test_input_cfg(&cfg, TUYA_GPIO_PULLDOWN);
    ret = tkl_gpio_init(pin, &cfg);
    if (ret != OPRT_OK) {
        gpio_test_dump_pin_state("smoke-input-down-fail", pin, ret);
        return -1;
    }
    gpio_test_cleanup_pin(pin);

    gpio_test_input_cfg(&cfg, TUYA_GPIO_FLOATING);
    ret = tkl_gpio_init(pin, &cfg);
    if (ret != OPRT_OK) {
        gpio_test_dump_pin_state("smoke-input-float-fail", pin, ret);
        return -1;
    }
    gpio_test_cleanup_pin(pin);

    gpio_test_output_cfg(&cfg, TUYA_GPIO_LEVEL_LOW);
    ret = tkl_gpio_init(pin, &cfg);
    if (ret != OPRT_OK) {
        gpio_test_dump_pin_state("smoke-output-low-fail", pin, ret);
        goto cleanup;
    }

    ret = tkl_gpio_write(pin, TUYA_GPIO_LEVEL_HIGH);
    if (ret != OPRT_OK) {
        gpio_test_dump_pin_state("smoke-write-high-fail", pin, ret);
        goto cleanup;
    }

    ret = tkl_gpio_write(pin, TUYA_GPIO_LEVEL_LOW);
    if (ret != OPRT_OK) {
        gpio_test_dump_pin_state("smoke-write-low-fail", pin, ret);
        goto cleanup;
    }

    ret = tkl_gpio_deinit(pin);
    if (ret != OPRT_OK) {
        gpio_test_dump_pin_state("smoke-deinit-fail", pin, ret);
        goto cleanup;
    }

    bk_printf("[smoke] PASS P%d\r\n", pin);
    rc = 0;

cleanup:
    if (rc != 0) {
        gpio_test_cleanup_pin(pin);
    }
    return rc;
}

static void gpio_test_smoke_range(TUYA_GPIO_NUM_E start, TUYA_GPIO_NUM_E end)
{
    uint32_t pass = 0;
    uint32_t fail = 0;
    uint32_t skip = 0;

    for (TUYA_GPIO_NUM_E pin = start; pin <= end; pin++) {
        if (gpio_test_is_auto_skip(pin)) {
            bk_printf("[smoke] skip P%d\r\n", pin);
            skip++;
            continue;
        }

        int ret = gpio_test_smoke_one(pin);
        if (ret == 0) {
            pass++;
        } else if (ret > 0) {
            skip++;
        } else {
            fail++;
        }
    }

    bk_printf("[smoke] summary pass=%u fail=%u skip=%u\r\n", pass, fail, skip);
}

static int gpio_test_pair_direction(TUYA_GPIO_NUM_E out_pin, TUYA_GPIO_NUM_E in_pin)
{
    TUYA_GPIO_LEVEL_E level = TUYA_GPIO_LEVEL_NONE;
    OPERATE_RET ret;
    TUYA_GPIO_BASE_CFG_T cfg;
    int rc = -1;

    gpio_test_input_cfg(&cfg, TUYA_GPIO_FLOATING);
    ret = tkl_gpio_init(in_pin, &cfg);
    if (ret != OPRT_OK) {
        bk_printf("[pair] init input fail P%d ret=%d\r\n", in_pin, ret);
        goto cleanup;
    }

    gpio_test_output_cfg(&cfg, TUYA_GPIO_LEVEL_LOW);
    ret = tkl_gpio_init(out_pin, &cfg);
    if (ret != OPRT_OK) {
        bk_printf("[pair] init output fail P%d ret=%d\r\n", out_pin, ret);
        goto cleanup;
    }

    tkl_system_sleep(GPIO_TEST_LEVEL_WAIT_MS);
    ret = tkl_gpio_read(in_pin, &level);
    if ((ret != OPRT_OK) || (level != TUYA_GPIO_LEVEL_LOW)) {
        bk_printf("[pair] low verify fail out=P%d in=P%d ret=%d level=%d\r\n", out_pin, in_pin, ret, level);
        goto cleanup;
    }

    ret = tkl_gpio_write(out_pin, TUYA_GPIO_LEVEL_HIGH);
    if (ret != OPRT_OK) {
        bk_printf("[pair] write high fail out=P%d ret=%d\r\n", out_pin, ret);
        goto cleanup;
    }

    tkl_system_sleep(GPIO_TEST_LEVEL_WAIT_MS);
    ret = tkl_gpio_read(in_pin, &level);
    if ((ret != OPRT_OK) || (level != TUYA_GPIO_LEVEL_HIGH)) {
        bk_printf("[pair] high verify fail out=P%d in=P%d ret=%d level=%d\r\n", out_pin, in_pin, ret, level);
        goto cleanup;
    }

    ret = tkl_gpio_write(out_pin, TUYA_GPIO_LEVEL_LOW);
    if (ret != OPRT_OK) {
        bk_printf("[pair] write low fail out=P%d ret=%d\r\n", out_pin, ret);
        goto cleanup;
    }

    tkl_system_sleep(GPIO_TEST_LEVEL_WAIT_MS);
    ret = tkl_gpio_read(in_pin, &level);
    if ((ret != OPRT_OK) || (level != TUYA_GPIO_LEVEL_LOW)) {
        bk_printf("[pair] low verify 2 fail out=P%d in=P%d ret=%d level=%d\r\n", out_pin, in_pin, ret, level);
        goto cleanup;
    }

    rc = 0;

cleanup:
    gpio_test_cleanup_pin(out_pin);
    gpio_test_cleanup_pin(in_pin);
    return rc;
}

static int gpio_test_pair(TUYA_GPIO_NUM_E pin_a, TUYA_GPIO_NUM_E pin_b)
{
    if (pin_a == pin_b) {
        bk_printf("[pair] invalid pair P%d P%d\r\n", pin_a, pin_b);
        return -1;
    }

    if (gpio_test_pair_direction(pin_a, pin_b) != 0) {
        tkl_gpio_deinit(pin_a);
        tkl_gpio_deinit(pin_b);
        return -1;
    }

    if (gpio_test_pair_direction(pin_b, pin_a) != 0) {
        tkl_gpio_deinit(pin_a);
        tkl_gpio_deinit(pin_b);
        return -1;
    }

    bk_printf("[pair] PASS P%d <-> P%d\r\n", pin_a, pin_b);
    return 0;
}

static int gpio_test_irq_mode(TUYA_GPIO_NUM_E drive_pin, TUYA_GPIO_NUM_E irq_pin, TUYA_GPIO_IRQ_E mode)
{
    TUYA_GPIO_IRQ_T irq_cfg;
    OPERATE_RET ret;
    TUYA_GPIO_BASE_CFG_T cfg;
    int rc = -1;

    if (drive_pin == irq_pin) {
        bk_printf("[irq] invalid pair drive=P%d irq=P%d\r\n", drive_pin, irq_pin);
        return -1;
    }
    gpio_test_reset_irq_state();

    irq_cfg.mode = mode;
    irq_cfg.cb = gpio_test_irq_cb;
    irq_cfg.arg = (void *)(uintptr_t)irq_pin;

    if ((mode == TUYA_GPIO_IRQ_FALL) || (mode == TUYA_GPIO_IRQ_HIGH)) {
        gpio_test_output_cfg(&cfg, TUYA_GPIO_LEVEL_HIGH);
    } else {
        gpio_test_output_cfg(&cfg, TUYA_GPIO_LEVEL_LOW);
    }
    ret = tkl_gpio_init(drive_pin, &cfg);

    if (ret != OPRT_OK) {
        bk_printf("[irq] drive init fail pin=%d ret=%d\r\n", drive_pin, ret);
        return -1;
    }

    gpio_test_input_cfg(&cfg, TUYA_GPIO_FLOATING);
    ret = tkl_gpio_init(irq_pin, &cfg);
    if (ret != OPRT_OK) {
        bk_printf("[irq] irq pin init fail pin=%d ret=%d\r\n", irq_pin, ret);
        goto cleanup;
    }

    ret = tkl_gpio_irq_init(irq_pin, &irq_cfg);
    if (ret != OPRT_OK) {
        bk_printf("[irq] irq init fail pin=%d mode=%s ret=%d\r\n",
                irq_pin, gpio_test_irq_mode_name(mode), ret);
        goto cleanup;
    }

    ret = tkl_gpio_irq_enable(irq_pin);
    if (ret != OPRT_OK) {
        bk_printf("[irq] irq enable fail pin=%d ret=%d\r\n", irq_pin, ret);
        goto cleanup;
    }

    tkl_system_sleep(GPIO_TEST_LEVEL_WAIT_MS);

    switch (mode) {
        case TUYA_GPIO_IRQ_RISE:
            ret = tkl_gpio_write(drive_pin, TUYA_GPIO_LEVEL_HIGH);
            break;
        case TUYA_GPIO_IRQ_FALL:
            ret = tkl_gpio_write(drive_pin, TUYA_GPIO_LEVEL_LOW);
            break;
        case TUYA_GPIO_IRQ_LOW:
            ret = tkl_gpio_write(drive_pin, TUYA_GPIO_LEVEL_LOW);
            break;
        case TUYA_GPIO_IRQ_HIGH:
            ret = tkl_gpio_write(drive_pin, TUYA_GPIO_LEVEL_HIGH);
            break;
        default:
            ret = OPRT_NOT_SUPPORTED;
            break;
    }

    if (ret != OPRT_OK) {
        bk_printf("[irq] trigger fail drive=%d mode=%s ret=%d\r\n",
                drive_pin, gpio_test_irq_mode_name(mode), ret);
        goto cleanup;
    }

    if (gpio_test_wait_irq(GPIO_TEST_IRQ_TIMEOUT_MS) != 0) {
        bk_printf("[irq] timeout drive=%d irq=%d mode=%s count=%u\r\n",
                drive_pin, irq_pin, gpio_test_irq_mode_name(mode), (unsigned int)s_gpio_irq_count);
        goto cleanup;
    }

    bk_printf("[irq] PASS drive=P%d irq=P%d mode=%s count=%u last=%d\r\n",
            drive_pin, irq_pin, gpio_test_irq_mode_name(mode),
            (unsigned int)s_gpio_irq_count, s_gpio_irq_last_pin);
    rc = 0;

cleanup:
    tkl_gpio_irq_disable(irq_pin);
    tkl_gpio_deinit(drive_pin);
    tkl_gpio_deinit(irq_pin);
    return rc;
}

static void gpio_test_api(void)
{
    TUYA_GPIO_BASE_CFG_T input_cfg;
    TUYA_GPIO_BASE_CFG_T output_cfg;
    TUYA_GPIO_LEVEL_E level = TUYA_GPIO_LEVEL_NONE;
    TUYA_GPIO_IRQ_T irq_cfg;
    OPERATE_RET ret;
    uint32_t pass = 0;
    uint32_t fail = 0;

    gpio_test_output_cfg(&output_cfg, TUYA_GPIO_LEVEL_LOW);
    gpio_test_input_cfg(&input_cfg, TUYA_GPIO_FLOATING);

    irq_cfg.mode = TUYA_GPIO_IRQ_RISE_FALL;
    irq_cfg.cb = gpio_test_irq_cb;
    irq_cfg.arg = NULL;

    gpio_test_cleanup_pin(GPIO_API_TEST_PIN_A);
    gpio_test_cleanup_pin(GPIO_API_TEST_PIN_B);

    ret = tkl_gpio_init(GPIO_API_TEST_PIN_A, NULL);
    gpio_test_api_expect("init null cfg", ret, OPRT_INVALID_PARM, &pass, &fail);

    ret = tkl_gpio_irq_init(GPIO_API_TEST_PIN_A, NULL);
    gpio_test_api_expect("irq init null cfg", ret, OPRT_INVALID_PARM, &pass, &fail);

    ret = tkl_gpio_deinit(TUYA_GPIO_NUM_MAX);
    gpio_test_api_expect("deinit invalid pin", ret, OPRT_INVALID_PARM, &pass, &fail);

    ret = tkl_gpio_deinit(GPIO_API_TEST_PIN_A);
    gpio_test_api_expect("deinit before init", ret, OPRT_RESOURCE_NOT_READY, &pass, &fail);

    ret = tkl_gpio_read(TUYA_GPIO_NUM_MAX, &level);
    gpio_test_api_expect("read invalid pin", ret, OPRT_INVALID_PARM, &pass, &fail);

    ret = tkl_gpio_read(GPIO_API_TEST_PIN_A, NULL);
    gpio_test_api_expect("read null level", ret, OPRT_INVALID_PARM, &pass, &fail);

    ret = tkl_gpio_write(TUYA_GPIO_NUM_MAX, TUYA_GPIO_LEVEL_HIGH);
    gpio_test_api_expect("write invalid pin", ret, OPRT_INVALID_PARM, &pass, &fail);

    ret = tkl_gpio_write(GPIO_API_TEST_PIN_A, TUYA_GPIO_LEVEL_HIGH);
    gpio_test_api_expect("write before init", ret, OPRT_RESOURCE_NOT_READY, &pass, &fail);

    ret = tkl_gpio_read(GPIO_API_TEST_PIN_A, &level);
    gpio_test_api_expect("read before init", ret, OPRT_RESOURCE_NOT_READY, &pass, &fail);

    ret = tkl_gpio_irq_enable(TUYA_GPIO_NUM_MAX);
    gpio_test_api_expect("irq enable invalid", ret, OPRT_INVALID_PARM, &pass, &fail);

    ret = tkl_gpio_irq_disable(TUYA_GPIO_NUM_MAX);
    gpio_test_api_expect("irq disable invalid", ret, OPRT_INVALID_PARM, &pass, &fail);

    ret = tkl_gpio_irq_init(GPIO_API_TEST_PIN_A, &irq_cfg);
    gpio_test_api_expect("irq init before init", ret, OPRT_RESOURCE_NOT_READY, &pass, &fail);

    ret = tkl_gpio_irq_enable(GPIO_API_TEST_PIN_A);
    gpio_test_api_expect("irq enable before init", ret, OPRT_RESOURCE_NOT_READY, &pass, &fail);

    ret = tkl_gpio_irq_disable(GPIO_API_TEST_PIN_A);
    gpio_test_api_expect("irq disable before init", ret, OPRT_RESOURCE_NOT_READY, &pass, &fail);

    input_cfg.mode = TUYA_GPIO_HIGH_IMPEDANCE;
    ret = tkl_gpio_init(GPIO_API_TEST_PIN_A, &input_cfg);
    gpio_test_api_expect("input high impedance", ret, OPRT_NOT_SUPPORTED, &pass, &fail);
    gpio_test_cleanup_pin(GPIO_API_TEST_PIN_A);

    gpio_test_input_cfg(&input_cfg, TUYA_GPIO_FLOATING);
    ret = tkl_gpio_init(GPIO_API_TEST_PIN_A, &input_cfg);
    gpio_test_api_expect("irq test input init", ret, OPRT_OK, &pass, &fail);

    ret = tkl_gpio_irq_init(GPIO_API_TEST_PIN_A, &irq_cfg);
    gpio_test_api_expect("irq rise_fall", ret, OPRT_NOT_SUPPORTED, &pass, &fail);
    gpio_test_cleanup_pin(GPIO_API_TEST_PIN_A);

    gpio_test_output_cfg(&output_cfg, TUYA_GPIO_LEVEL_LOW);
    ret = tkl_gpio_init(GPIO_API_TEST_PIN_A, &output_cfg);
    gpio_test_api_expect("output init low", ret, OPRT_OK, &pass, &fail);

    ret = tkl_gpio_init(GPIO_API_TEST_PIN_A, &output_cfg);
    gpio_test_api_expect("output init twice", ret, OPRT_INIT_MORE_THAN_ONCE, &pass, &fail);

    ret = tkl_gpio_write(GPIO_API_TEST_PIN_A, TUYA_GPIO_LEVEL_HIGH);
    gpio_test_api_expect("output write high", ret, OPRT_OK, &pass, &fail);

    ret = tkl_gpio_write(GPIO_API_TEST_PIN_A, TUYA_GPIO_LEVEL_LOW);
    gpio_test_api_expect("output write low", ret, OPRT_OK, &pass, &fail);

    ret = tkl_gpio_deinit(GPIO_API_TEST_PIN_A);
    gpio_test_api_expect("output deinit", ret, OPRT_OK, &pass, &fail);

    gpio_test_output_cfg(&output_cfg, TUYA_GPIO_LEVEL_HIGH);
    ret = tkl_gpio_init(GPIO_API_TEST_PIN_A, &output_cfg);
    gpio_test_api_expect("output init high", ret, OPRT_OK, &pass, &fail);

    ret = tkl_gpio_deinit(GPIO_API_TEST_PIN_A);
    gpio_test_api_expect("output deinit 2", ret, OPRT_OK, &pass, &fail);

    gpio_test_input_cfg(&input_cfg, TUYA_GPIO_FLOATING);
    ret = tkl_gpio_init(GPIO_API_TEST_PIN_B, &input_cfg);
    gpio_test_api_expect("input init float", ret, OPRT_OK, &pass, &fail);

    ret = tkl_gpio_write(GPIO_API_TEST_PIN_B, TUYA_GPIO_LEVEL_HIGH);
    gpio_test_api_expect("write on input pin", ret, OPRT_COM_ERROR, &pass, &fail);

    ret = tkl_gpio_read(GPIO_API_TEST_PIN_B, &level);
    gpio_test_api_expect("read valid pin", ret, OPRT_OK, &pass, &fail);

    irq_cfg.mode = TUYA_GPIO_IRQ_RISE;
    irq_cfg.arg = (void *)(uintptr_t)GPIO_API_TEST_PIN_B;
    ret = tkl_gpio_irq_init(GPIO_API_TEST_PIN_B, &irq_cfg);
    gpio_test_api_expect("irq init rise", ret, OPRT_OK, &pass, &fail);

    ret = tkl_gpio_irq_enable(GPIO_API_TEST_PIN_B);
    gpio_test_api_expect("irq enable", ret, OPRT_OK, &pass, &fail);

    ret = tkl_gpio_irq_disable(GPIO_API_TEST_PIN_B);
    gpio_test_api_expect("irq disable", ret, OPRT_OK, &pass, &fail);

    ret = tkl_gpio_deinit(GPIO_API_TEST_PIN_B);
    gpio_test_api_expect("input deinit", ret, OPRT_OK, &pass, &fail);

    ret = tkl_gpio_deinit(GPIO_API_TEST_PIN_B);
    gpio_test_api_expect("input deinit twice", ret, OPRT_RESOURCE_NOT_READY, &pass, &fail);

    bk_printf("[api] summary pass=%u fail=%u\r\n", pass, fail);
}

static void gpio_test_init_race_task(void *arg)
{
    TUYA_GPIO_BASE_CFG_T cfg;
    OPERATE_RET ret;

    (void)arg;

    while (!s_gpio_init_race_ctx.start) {
        tkl_system_sleep(1);
    }

    gpio_test_output_cfg(&cfg, TUYA_GPIO_LEVEL_LOW);
    ret = tkl_gpio_init(s_gpio_init_race_ctx.pin, &cfg);
    if (ret == OPRT_OK) {
        gpio_test_atomic_inc(&s_gpio_init_race_ctx.init_ok);
    } else if (ret == OPRT_INIT_MORE_THAN_ONCE) {
        gpio_test_atomic_inc(&s_gpio_init_race_ctx.init_busy);
    } else {
        gpio_test_atomic_inc(&s_gpio_init_race_ctx.init_fail);
    }

    gpio_test_atomic_inc(&s_gpio_init_race_ctx.done);
    vTaskDelete(NULL);
}

static void gpio_test_rw_write_task(void *arg)
{
    TUYA_GPIO_LEVEL_E level = (TUYA_GPIO_LEVEL_E)(uintptr_t)arg;

    while (!s_gpio_rw_race_ctx.start) {
        tkl_system_sleep(1);
    }

    for (uint32_t i = 0; i < GPIO_TEST_CONCURRENT_LOOPS; i++) {
        if (tkl_gpio_write(s_gpio_rw_race_ctx.pin, level) == OPRT_OK) {
            gpio_test_atomic_inc(&s_gpio_rw_race_ctx.write_ok);
        } else {
            gpio_test_atomic_inc(&s_gpio_rw_race_ctx.write_fail);
        }
        level = (level == TUYA_GPIO_LEVEL_HIGH) ? TUYA_GPIO_LEVEL_LOW : TUYA_GPIO_LEVEL_HIGH;
    }

    gpio_test_atomic_inc(&s_gpio_rw_race_ctx.done);
    vTaskDelete(NULL);
}

static void gpio_test_rw_read_task(void *arg)
{
    TUYA_GPIO_LEVEL_E level = TUYA_GPIO_LEVEL_NONE;

    (void)arg;

    while (!s_gpio_rw_race_ctx.start) {
        tkl_system_sleep(1);
    }

    for (uint32_t i = 0; i < GPIO_TEST_CONCURRENT_LOOPS; i++) {
        if (tkl_gpio_read(s_gpio_rw_race_ctx.pin, &level) == OPRT_OK) {
            gpio_test_atomic_inc(&s_gpio_rw_race_ctx.read_ok);
        } else {
            gpio_test_atomic_inc(&s_gpio_rw_race_ctx.read_fail);
        }
    }

    gpio_test_atomic_inc(&s_gpio_rw_race_ctx.done);
    vTaskDelete(NULL);
}

static void gpio_test_concurrent(TUYA_GPIO_NUM_E pin)
{
    TUYA_GPIO_BASE_CFG_T cfg;
    OPERATE_RET ret;
    uint32_t created = 0;

    if (gpio_test_is_auto_skip(pin)) {
        bk_printf("[concurrent] skip P%d\r\n", pin);
        return;
    }

    gpio_test_cleanup_pin(pin);
    os_memset((void *)&s_gpio_init_race_ctx, 0, sizeof(s_gpio_init_race_ctx));
    s_gpio_init_race_ctx.pin = pin;

    for (uint32_t i = 0; i < GPIO_TEST_CONCURRENT_WORKERS; i++) {
        if (xTaskCreate(gpio_test_init_race_task, "gpio_i", 2048, NULL, 5, NULL) != pdPASS) {
            bk_printf("[concurrent] init race task create fail idx=%u\r\n", (unsigned int)i);
            s_gpio_init_race_ctx.start = 1;
            gpio_test_wait_counter(&s_gpio_init_race_ctx.done, created, GPIO_TEST_CONCURRENT_TIMEOUT_MS);
            return;
        }
        created++;
    }

    tkl_system_sleep(10);
    s_gpio_init_race_ctx.start = 1;

    if (gpio_test_wait_counter(&s_gpio_init_race_ctx.done, GPIO_TEST_CONCURRENT_WORKERS,
                               GPIO_TEST_CONCURRENT_TIMEOUT_MS) != 0) {
        bk_printf("[concurrent] init race timeout pin=P%d done=%u\r\n",
                  pin, (unsigned int)s_gpio_init_race_ctx.done);
        gpio_test_cleanup_pin(pin);
        return;
    }

    if ((s_gpio_init_race_ctx.init_ok != 1) ||
        (s_gpio_init_race_ctx.init_busy != (GPIO_TEST_CONCURRENT_WORKERS - 1)) ||
        (s_gpio_init_race_ctx.init_fail != 0)) {
        bk_printf("[concurrent] init race fail pin=P%d ok=%u busy=%u fail=%u\r\n",
                  pin,
                  (unsigned int)s_gpio_init_race_ctx.init_ok,
                  (unsigned int)s_gpio_init_race_ctx.init_busy,
                  (unsigned int)s_gpio_init_race_ctx.init_fail);
        gpio_test_cleanup_pin(pin);
        return;
    }

    bk_printf("[concurrent] init race PASS pin=P%d ok=%u busy=%u\r\n",
              pin,
              (unsigned int)s_gpio_init_race_ctx.init_ok,
              (unsigned int)s_gpio_init_race_ctx.init_busy);

    gpio_test_cleanup_pin(pin);

    gpio_test_output_cfg(&cfg, TUYA_GPIO_LEVEL_LOW);
    ret = tkl_gpio_init(pin, &cfg);
    if (ret != OPRT_OK) {
        bk_printf("[concurrent] base init fail pin=P%d ret=%d\r\n", pin, ret);
        return;
    }

    os_memset((void *)&s_gpio_rw_race_ctx, 0, sizeof(s_gpio_rw_race_ctx));
    s_gpio_rw_race_ctx.pin = pin;
    created = 0;

    if (xTaskCreate(gpio_test_rw_write_task, "gpio_wh", 2048,
                    (void *)(uintptr_t)TUYA_GPIO_LEVEL_HIGH, 5, NULL) != pdPASS) {
        bk_printf("[concurrent] write task high create fail\r\n");
        gpio_test_cleanup_pin(pin);
        return;
    }
    created++;

    if (xTaskCreate(gpio_test_rw_write_task, "gpio_wl", 2048,
                    (void *)(uintptr_t)TUYA_GPIO_LEVEL_LOW, 5, NULL) != pdPASS) {
        bk_printf("[concurrent] write task low create fail\r\n");
        s_gpio_rw_race_ctx.start = 1;
        gpio_test_wait_counter(&s_gpio_rw_race_ctx.done, created, GPIO_TEST_CONCURRENT_TIMEOUT_MS);
        gpio_test_cleanup_pin(pin);
        return;
    }
    created++;

    if (xTaskCreate(gpio_test_rw_read_task, "gpio_r", 2048, NULL, 5, NULL) != pdPASS) {
        bk_printf("[concurrent] read task create fail\r\n");
        s_gpio_rw_race_ctx.start = 1;
        gpio_test_wait_counter(&s_gpio_rw_race_ctx.done, created, GPIO_TEST_CONCURRENT_TIMEOUT_MS);
        gpio_test_cleanup_pin(pin);
        return;
    }
    created++;

    tkl_system_sleep(10);
    s_gpio_rw_race_ctx.start = 1;

    if (gpio_test_wait_counter(&s_gpio_rw_race_ctx.done, 3, GPIO_TEST_CONCURRENT_TIMEOUT_MS) != 0) {
        bk_printf("[concurrent] rw race timeout pin=P%d done=%u\r\n",
                  pin, (unsigned int)s_gpio_rw_race_ctx.done);
        gpio_test_cleanup_pin(pin);
        return;
    }

    if ((s_gpio_rw_race_ctx.write_fail != 0) || (s_gpio_rw_race_ctx.read_fail != 0)) {
        bk_printf("[concurrent] rw race fail pin=P%d w_ok=%u w_fail=%u r_ok=%u r_fail=%u\r\n",
                  pin,
                  (unsigned int)s_gpio_rw_race_ctx.write_ok,
                  (unsigned int)s_gpio_rw_race_ctx.write_fail,
                  (unsigned int)s_gpio_rw_race_ctx.read_ok,
                  (unsigned int)s_gpio_rw_race_ctx.read_fail);
        gpio_test_cleanup_pin(pin);
        return;
    }

    bk_printf("[concurrent] rw race PASS pin=P%d w_ok=%u r_ok=%u\r\n",
              pin,
              (unsigned int)s_gpio_rw_race_ctx.write_ok,
              (unsigned int)s_gpio_rw_race_ctx.read_ok);

    gpio_test_cleanup_pin(pin);
}

static void gpio_test_pair_range(TUYA_GPIO_NUM_E start, TUYA_GPIO_NUM_E end, int with_irq)
{
    uint32_t pass = 0;
    uint32_t fail = 0;
    uint32_t skip = 0;

    for (TUYA_GPIO_NUM_E pin = start; pin < end; pin += 2) {
        TUYA_GPIO_NUM_E next = pin + 1;

        if (next > end) {
            break;
        }

        if (gpio_test_is_auto_skip(pin) || gpio_test_is_auto_skip(next)) {
            bk_printf("[pair-range] skip P%d P%d\r\n", pin, next);
            skip++;
            continue;
        }

        if (gpio_test_pair(pin, next) == 0) {
            pass++;
        } else {
            fail++;
            continue;
        }

        if (with_irq) {
            if (gpio_test_irq_mode(pin, next, TUYA_GPIO_IRQ_RISE) != 0) {
                fail++;
                continue;
            }
            if (gpio_test_irq_mode(pin, next, TUYA_GPIO_IRQ_FALL) != 0) {
                fail++;
                continue;
            }
            if (gpio_test_irq_mode(pin, next, TUYA_GPIO_IRQ_LOW) != 0) {
                fail++;
                continue;
            }
            if (gpio_test_irq_mode(pin, next, TUYA_GPIO_IRQ_HIGH) != 0) {
                fail++;
                continue;
            }
        }
    }

    bk_printf("[pair-range] summary pass=%u fail=%u skip=%u\r\n", pass, fail, skip);
}

static void relay_test_task(void *arg)
{
    TUYA_GPIO_BASE_CFG_T cfg;

    (void)arg;

    gpio_test_output_cfg(&cfg, TUYA_GPIO_LEVEL_LOW);

    if (gpio_test_is_auto_skip(s_relay_pin)) {
        bk_printf("[relay] invalid pin P%d\r\n", s_relay_pin);
        s_relay_test_thread = NULL;
        vTaskDelete(NULL);
        return;
    }

    if (tkl_gpio_init(s_relay_pin, &cfg) != OPRT_OK) {
        bk_printf("[relay] init fail P%d\r\n", s_relay_pin);
        s_relay_test_thread = NULL;
        vTaskDelete(NULL);
        return;
    }

    bk_printf("[relay] start pin=P%d period=%dms\r\n", s_relay_pin, GPIO_TEST_RELAY_PERIOD_MS);

    while (1) {
        tkl_gpio_write(s_relay_pin, TUYA_GPIO_LEVEL_HIGH);
        bk_printf("[relay] P%d=1\r\n", s_relay_pin);
        tkl_system_sleep(GPIO_TEST_RELAY_PERIOD_MS);

        tkl_gpio_write(s_relay_pin, TUYA_GPIO_LEVEL_LOW);
        bk_printf("[relay] P%d=0\r\n", s_relay_pin);
        tkl_system_sleep(GPIO_TEST_RELAY_PERIOD_MS);
    }
}

static void gpio_test_start_relay(TUYA_GPIO_NUM_E pin)
{
    if (s_relay_test_thread != NULL) {
        bk_printf("[relay] task already running on P%d\r\n", s_relay_pin);
        return;
    }

    s_relay_pin = pin;
    xTaskCreate(relay_test_task, "relay", 4096, NULL, 5, &s_relay_test_thread);
}

static void gpio_test_legacy_cmd(int argc, char **argv)
{
    TUYA_GPIO_NUM_E pin_id;
    TUYA_GPIO_BASE_CFG_T cfg;

    if ((argc < 3) || (gpio_test_parse_pin(argv[1], &pin_id) != 0)) {
        gpio_test_usage();
        return;
    }

    if (!os_strcmp(argv[2], "relay")) {
        gpio_test_start_relay(pin_id);
        return;
    }

    if (!os_strcmp(argv[2], "irq")) {
        TUYA_GPIO_IRQ_T irq_cfg;

        if (argc < 4) {
            gpio_test_usage();
            return;
        }

        if (!os_strcmp(argv[3], "start")) {
            tkl_gpio_irq_enable(pin_id);
            return;
        }

        if (!os_strcmp(argv[3], "stop")) {
            tkl_gpio_irq_disable(pin_id);
            return;
        }

        irq_cfg.cb = gpio_test_irq_cb;
        irq_cfg.arg = (void *)(uintptr_t)pin_id;

        if (!os_strcmp(argv[3], "rise")) {
            irq_cfg.mode = TUYA_GPIO_IRQ_RISE;
        } else if (!os_strcmp(argv[3], "fall")) {
            irq_cfg.mode = TUYA_GPIO_IRQ_FALL;
        } else if (!os_strcmp(argv[3], "high")) {
            irq_cfg.mode = TUYA_GPIO_IRQ_HIGH;
        } else if (!os_strcmp(argv[3], "low")) {
            irq_cfg.mode = TUYA_GPIO_IRQ_LOW;
        } else {
            gpio_test_usage();
            return;
        }
        bk_printf("[legacy] irq init pin=P%d mode=%s ret=%d\r\n",
                pin_id, gpio_test_irq_mode_name(irq_cfg.mode), tkl_gpio_irq_init(pin_id, &irq_cfg));
        return;
    }

    if (argc < 4) {
        gpio_test_usage();
        return;
    }

    if (!os_strcmp(argv[2], "input")) {
        cfg.direct = TUYA_GPIO_INPUT;
    } else if (!os_strcmp(argv[2], "output")) {
        cfg.direct = TUYA_GPIO_OUTPUT;
    } else {
        gpio_test_usage();
        return;
    }

    switch (os_strtoul(argv[3], NULL, 10)) {
        case 0:
            cfg.mode = TUYA_GPIO_PULLDOWN;
            cfg.level = TUYA_GPIO_LEVEL_LOW;
            break;
        case 1:
            cfg.mode = TUYA_GPIO_PULLUP;
            cfg.level = TUYA_GPIO_LEVEL_HIGH;
            break;
        default:
            cfg.mode = TUYA_GPIO_FLOATING;
            cfg.level = TUYA_GPIO_LEVEL_NONE;
            break;
    }
    bk_printf("[legacy] init pin=P%d ret=%d\r\n", pin_id, tkl_gpio_init(pin_id, &cfg));
}

void cli_gpio_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    TUYA_GPIO_NUM_E pin_a;
    TUYA_GPIO_NUM_E pin_b;
    TUYA_GPIO_NUM_E start = TUYA_GPIO_NUM_0;
    TUYA_GPIO_NUM_E end = (TUYA_GPIO_NUM_E)GPIO_TEST_PIN_MAX;

    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    test_gpio_auxldo_enable();

    if (argc < 2) {
        gpio_test_usage();
        return;
    }

    if (gpio_test_is_num_str(argv[1])) {
        gpio_test_legacy_cmd(argc, argv);
        return;
    }

    if (!os_strcmp(argv[1], "help") || !os_strcmp(argv[1], "?")) {
        gpio_test_usage();
    } else if (!os_strcmp(argv[1], "list")) {
        gpio_test_list();
    } else if (!os_strcmp(argv[1], "api")) {
        gpio_test_api();
    } else if (!os_strcmp(argv[1], "concurrent")) {
        if ((argc < 3) || (gpio_test_parse_pin(argv[2], &pin_a) != 0)) {
            gpio_test_usage();
            return;
        }
        gpio_test_concurrent(pin_a);
    } else if (!os_strcmp(argv[1], "smoke")) {
        if (argc < 3) {
            gpio_test_usage();
            return;
        }

        if (!os_strcmp(argv[2], "all")) {
            if ((argc >= 5) &&
                (gpio_test_parse_pin(argv[3], &start) == 0) &&
                (gpio_test_parse_pin(argv[4], &end) == 0) &&
                (start <= end)) {
                gpio_test_smoke_range(start, end);
            } else {
                gpio_test_smoke_range(start, end);
            }
            return;
        }

        if (gpio_test_parse_pin(argv[2], &pin_a) != 0) {
            gpio_test_usage();
            return;
        }

        gpio_test_smoke_one(pin_a);
    } else if (!os_strcmp(argv[1], "pair")) {
        if ((argc < 4) ||
            (gpio_test_parse_pin(argv[2], &pin_a) != 0) ||
            (gpio_test_parse_pin(argv[3], &pin_b) != 0)) {
            gpio_test_usage();
            return;
        }

        gpio_test_pair(pin_a, pin_b);
    } else if (!os_strcmp(argv[1], "irq")) {
        if ((argc < 5) ||
            (gpio_test_parse_pin(argv[2], &pin_a) != 0) ||
            (gpio_test_parse_pin(argv[3], &pin_b) != 0)) {
            gpio_test_usage();
            return;
        }

        if (!os_strcmp(argv[4], "rise")) {
            gpio_test_irq_mode(pin_a, pin_b, TUYA_GPIO_IRQ_RISE);
        } else if (!os_strcmp(argv[4], "fall")) {
            gpio_test_irq_mode(pin_a, pin_b, TUYA_GPIO_IRQ_FALL);
        } else if (!os_strcmp(argv[4], "low")) {
            gpio_test_irq_mode(pin_a, pin_b, TUYA_GPIO_IRQ_LOW);
        } else if (!os_strcmp(argv[4], "high")) {
            gpio_test_irq_mode(pin_a, pin_b, TUYA_GPIO_IRQ_HIGH);
        } else {
            gpio_test_usage();
        }
    } else if (!os_strcmp(argv[1], "all")) {
        if ((argc >= 4) &&
            (gpio_test_parse_pin(argv[2], &start) == 0) &&
            (gpio_test_parse_pin(argv[3], &end) == 0) &&
            (start <= end)) {
            bk_printf("[all] running range P%d~P%d\r\n", start, end);
        } else {
            start = TUYA_GPIO_NUM_0;
            end = (TUYA_GPIO_NUM_E)GPIO_TEST_PIN_MAX;
            bk_printf("[all] running default range P%d~P%d\r\n", start, end);
        }

        bk_printf("[all] step1 smoke test\r\n");
        gpio_test_smoke_range(start, end);
        bk_printf("[all] step2 pair+irq test, requires adjacent wiring\r\n");
        gpio_test_pair_range(start, end, 1);
    } else if (!os_strcmp(argv[1], "relay")) {
        if ((argc < 3) || (gpio_test_parse_pin(argv[2], &pin_a) != 0)) {
            gpio_test_usage();
            return;
        }
        gpio_test_start_relay(pin_a);
    } else {
        gpio_test_usage();
    }
}
