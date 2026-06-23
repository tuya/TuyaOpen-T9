#include "tkl_gpio.h"
#include "tkl_mutex.h"
#include "tkl_system.h"
#include <driver/gpio.h>
#include "gpio_driver.h"
#include "sdkconfig.h"
#include <driver/uart.h>



typedef struct {
    gpio_id_t           gpio;
    TUYA_GPIO_IRQ_CB    cb;
    void                *args;
    bool                initialized;
    bool                irq_initialized;
    TUYA_GPIO_DRCT_E    direct;
    TUYA_GPIO_MODE_E    mode;
    TUYA_GPIO_LEVEL_E   level;
} pin_dev_map_t;

#define PINMAP_ENTRY(id) {GPIO_##id, NULL, NULL, false, false, TUYA_GPIO_INPUT, TUYA_GPIO_FLOATING, TUYA_GPIO_LEVEL_NONE}

static pin_dev_map_t pinmap[] = {
    PINMAP_ENTRY(0),  PINMAP_ENTRY(1),  PINMAP_ENTRY(2),  PINMAP_ENTRY(3),
    PINMAP_ENTRY(4),  PINMAP_ENTRY(5),  PINMAP_ENTRY(6),  PINMAP_ENTRY(7),
    PINMAP_ENTRY(8),  PINMAP_ENTRY(9),  PINMAP_ENTRY(10), PINMAP_ENTRY(11),
    PINMAP_ENTRY(12), PINMAP_ENTRY(13), PINMAP_ENTRY(14), PINMAP_ENTRY(15),
    PINMAP_ENTRY(16), PINMAP_ENTRY(17), PINMAP_ENTRY(18), PINMAP_ENTRY(19),
    PINMAP_ENTRY(20), PINMAP_ENTRY(21), PINMAP_ENTRY(22), PINMAP_ENTRY(23),
    PINMAP_ENTRY(24), PINMAP_ENTRY(25), PINMAP_ENTRY(26), PINMAP_ENTRY(27),
    PINMAP_ENTRY(28), PINMAP_ENTRY(29), PINMAP_ENTRY(30), PINMAP_ENTRY(31),
    PINMAP_ENTRY(32), PINMAP_ENTRY(33), PINMAP_ENTRY(34), PINMAP_ENTRY(35),
    PINMAP_ENTRY(36), PINMAP_ENTRY(37), PINMAP_ENTRY(38), PINMAP_ENTRY(39),
    PINMAP_ENTRY(40), PINMAP_ENTRY(41), PINMAP_ENTRY(42), PINMAP_ENTRY(43),
    PINMAP_ENTRY(44), PINMAP_ENTRY(45), PINMAP_ENTRY(46), PINMAP_ENTRY(47),
    PINMAP_ENTRY(48), PINMAP_ENTRY(49), PINMAP_ENTRY(50), PINMAP_ENTRY(51),
    PINMAP_ENTRY(52), PINMAP_ENTRY(53), PINMAP_ENTRY(54), PINMAP_ENTRY(55),
    PINMAP_ENTRY(56), PINMAP_ENTRY(57), PINMAP_ENTRY(58), PINMAP_ENTRY(59),
    PINMAP_ENTRY(60), PINMAP_ENTRY(61), PINMAP_ENTRY(62), PINMAP_ENTRY(63),
    PINMAP_ENTRY(64), PINMAP_ENTRY(65), PINMAP_ENTRY(66), PINMAP_ENTRY(67),
    PINMAP_ENTRY(68), PINMAP_ENTRY(69), PINMAP_ENTRY(70), PINMAP_ENTRY(71),
};

#define PINMAP_COUNT (sizeof(pinmap) / sizeof(pinmap[0]))

static TKL_MUTEX_HANDLE s_tkl_gpio_mutex = NULL;

static bool tkl_gpio_is_valid_pin(TUYA_GPIO_NUM_E pin_id)
{
    if ((uint32_t)pin_id >= PINMAP_COUNT) {
        return false;
    }

    if ((pin_id == TUYA_GPIO_NUM_0) || (pin_id == TUYA_GPIO_NUM_1)) {
        return false;
    }

    return true;
}

#define PIN_DEV_CHECK_ERROR_RETURN(__PIN, __ERROR) \
    do { \
        if (!tkl_gpio_is_valid_pin(__PIN)) { \
            return __ERROR; \
        } \
    } while (0)

extern void bk_printf(const char *fmt, ...);
extern uint32_t platform_is_in_interrupt_context(void);

static OPERATE_RET tkl_gpio_ensure_mutex(void)
{
    TKL_MUTEX_HANDLE new_mutex = NULL;
    UINT_T irq_mask;
    OPERATE_RET ret;

    if (s_tkl_gpio_mutex != NULL) {
        return OPRT_OK;
    }

    ret = tkl_mutex_create_init(&new_mutex);
    if (ret != OPRT_OK) {
        return ret;
    }

    irq_mask = tkl_system_enter_critical();
    if (s_tkl_gpio_mutex == NULL) {
        s_tkl_gpio_mutex = new_mutex;
        new_mutex = NULL;
    }
    tkl_system_exit_critical(irq_mask);

    if (new_mutex != NULL) {
        tkl_mutex_release(new_mutex);
    }

    return OPRT_OK;
}

static OPERATE_RET tkl_gpio_lock(void)
{
    OPERATE_RET ret = tkl_gpio_ensure_mutex();
    if (ret != OPRT_OK) {
        return ret;
    }

    return tkl_mutex_lock(s_tkl_gpio_mutex);
}

static void tkl_gpio_unlock(void)
{
    if (s_tkl_gpio_mutex != NULL) {
        tkl_mutex_unlock(s_tkl_gpio_mutex);
    }
}

/**
 * @brief gpio init
 *
 * @param[in] pin_id: gpio pin id
 * @param[in] cfg:  gpio config
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_init(TUYA_GPIO_NUM_E pin_id, CONST TUYA_GPIO_BASE_CFG_T *cfg)
{
    OPERATE_RET ret = OPRT_OK;

    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    PIN_DEV_CHECK_ERROR_RETURN(pin_id, OPRT_INVALID_PARM);

    ret = tkl_gpio_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    if (pinmap[pin_id].initialized) {
        tkl_gpio_unlock();
        return OPRT_INIT_MORE_THAN_ONCE;
    }

    if (pin_id == TUYA_GPIO_NUM_0) {
        bk_printf("WARNING: Init gpio0, LOG will be disabled\r\n");
    }

    bk_gpio_driver_init();
    gpio_dev_unmap(pinmap[pin_id].gpio);
    //! set pin direction
    switch (cfg->direct) {
        case TUYA_GPIO_INPUT:
            bk_gpio_disable_output(pinmap[pin_id].gpio);
            bk_gpio_enable_input(pinmap[pin_id].gpio);

            if(cfg->mode == TUYA_GPIO_PULLUP) {
                bk_gpio_enable_pull(pinmap[pin_id].gpio);
                bk_gpio_pull_up(pinmap[pin_id].gpio);
            } else if(cfg->mode == TUYA_GPIO_PULLDOWN) {
                bk_gpio_enable_pull(pinmap[pin_id].gpio);
                bk_gpio_pull_down(pinmap[pin_id].gpio);
            } else if(cfg->mode == TUYA_GPIO_FLOATING) {
                bk_gpio_disable_pull(pinmap[pin_id].gpio);
            } else {
                bk_printf("set direct error \r\n");
                tkl_gpio_unlock();
                return OPRT_NOT_SUPPORTED;
            }
            break;
        case TUYA_GPIO_OUTPUT:
            bk_gpio_disable_input(pinmap[pin_id].gpio);
            bk_gpio_enable_output(pinmap[pin_id].gpio);
            //! set pin init level
            if(TUYA_GPIO_LEVEL_LOW == cfg->level) {
                bk_gpio_set_output_low(pinmap[pin_id].gpio);
            } else {
                bk_gpio_set_output_high(pinmap[pin_id].gpio);
            }
            break;
        default:
            tkl_gpio_unlock();
            return OPRT_NOT_SUPPORTED;
    }

    pinmap[pin_id].initialized = true;
    pinmap[pin_id].irq_initialized = false;
    pinmap[pin_id].direct = cfg->direct;
    pinmap[pin_id].mode = cfg->mode;
    pinmap[pin_id].level = cfg->level;
    pinmap[pin_id].cb = NULL;
    pinmap[pin_id].args = NULL;

    tkl_gpio_unlock();
    return ret;
}

/**
 * @brief gpio deinit
 *
 * @param[in] pin_id: gpio pin id
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_deinit(TUYA_GPIO_NUM_E pin_id)
{
    OPERATE_RET ret;

    PIN_DEV_CHECK_ERROR_RETURN(pin_id, OPRT_INVALID_PARM);

    ret = tkl_gpio_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    if (!pinmap[pin_id].initialized) {
        tkl_gpio_unlock();
        return OPRT_RESOURCE_NOT_READY;
    }

    gpio_dev_unmap(pinmap[pin_id].gpio);
    //Set gpio to input floating mode
    bk_gpio_disable_interrupt(pinmap[pin_id].gpio);
    bk_gpio_disable_output(pinmap[pin_id].gpio);
    bk_gpio_enable_input(pinmap[pin_id].gpio);
    bk_gpio_disable_pull(pinmap[pin_id].gpio);

    pinmap[pin_id].initialized = false;
    pinmap[pin_id].irq_initialized = false;
    pinmap[pin_id].direct = TUYA_GPIO_INPUT;
    pinmap[pin_id].mode = TUYA_GPIO_FLOATING;
    pinmap[pin_id].level = TUYA_GPIO_LEVEL_NONE;
    pinmap[pin_id].cb = NULL;
    pinmap[pin_id].args = NULL;

    tkl_gpio_unlock();
    return OPRT_OK;
}

/**
 * @brief gpio write
 *
 * @param[in] pin_id: gpio pin id
 * @param[in] level: gpio output level value
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_write(TUYA_GPIO_NUM_E pin_id, TUYA_GPIO_LEVEL_E level)
{
    OPERATE_RET ret;

    PIN_DEV_CHECK_ERROR_RETURN(pin_id, OPRT_INVALID_PARM);

    ret = tkl_gpio_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    if (!pinmap[pin_id].initialized) {
        tkl_gpio_unlock();
        return OPRT_RESOURCE_NOT_READY;
    }

    if (pinmap[pin_id].direct != TUYA_GPIO_OUTPUT) {
        tkl_gpio_unlock();
        return OPRT_COM_ERROR;
    }

    if (level == TUYA_GPIO_LEVEL_HIGH) {
        bk_gpio_set_output_high(pinmap[pin_id].gpio);
    } else {
        bk_gpio_set_output_low(pinmap[pin_id].gpio);
    }

    pinmap[pin_id].level = level;
    tkl_gpio_unlock();
    return OPRT_OK;
}

/**
 * @brief gpio read
 *
 * @param[in] pin_id: gpio pin id
 * @param[out] level: gpio output level
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_read(TUYA_GPIO_NUM_E pin_id, TUYA_GPIO_LEVEL_E *level)
{
    OPERATE_RET ret;

    if (level == NULL) {
        return OPRT_INVALID_PARM;
    }

    PIN_DEV_CHECK_ERROR_RETURN(pin_id, OPRT_INVALID_PARM);

    ret = tkl_gpio_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    if (!pinmap[pin_id].initialized) {
        tkl_gpio_unlock();
        return OPRT_RESOURCE_NOT_READY;
    }

    if(bk_gpio_get_input(pinmap[pin_id].gpio)) {
        *level = TUYA_GPIO_LEVEL_HIGH;
    } else {
        *level = TUYA_GPIO_LEVEL_LOW;
    }

    tkl_gpio_unlock();
    return OPRT_OK;
}

/**
 * @brief gpio irq cb
 * NOTE: this API will call irq callback
 *
 * @param[in] arg: gpio pin id
 * @return none
 */
static void __tkl_gpio_irq_cb(gpio_id_t gpio_id)
{
    if ((uint32_t)gpio_id >= PINMAP_COUNT) {
        return;
    }

    if(pinmap[gpio_id].cb){
        pinmap[gpio_id].cb(pinmap[gpio_id].args);
    }
    bk_gpio_clear_interrupt(gpio_id);
    return;
}

/**
 * @brief gpio irq init
 * NOTE: call this API will not enable interrupt
 *
 * @param[in] pin_id: gpio pin id
 * @param[in] cfg:  gpio irq config
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_irq_init(TUYA_GPIO_NUM_E pin_id, CONST TUYA_GPIO_IRQ_T *cfg)
{
    OPERATE_RET ret;
    PIN_DEV_CHECK_ERROR_RETURN(pin_id, OPRT_INVALID_PARM);

    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    ret = tkl_gpio_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    if (!pinmap[pin_id].initialized) {
        tkl_gpio_unlock();
        return OPRT_RESOURCE_NOT_READY;
    }

    if (pin_id == TUYA_GPIO_NUM_0) {
        bk_printf("WARNING: Init gpio0 irq, LOG will be disabled\r\n");
    }

    gpio_int_type_t trigger;

    switch (cfg->mode) {
        case TUYA_GPIO_IRQ_RISE:
            trigger = GPIO_INT_TYPE_RISING_EDGE;
            break;
        case TUYA_GPIO_IRQ_FALL:
            trigger = GPIO_INT_TYPE_FALLING_EDGE;
            break;
        case TUYA_GPIO_IRQ_LOW:
            trigger = GPIO_INT_TYPE_LOW_LEVEL;
            break;
        case TUYA_GPIO_IRQ_HIGH:
            trigger = GPIO_INT_TYPE_HIGH_LEVEL;
            break;
        case TUYA_GPIO_IRQ_RISE_FALL:
        default:
            bk_printf("GPIO_INT_TYPE_HIGH_LEVEL Not supported yet !!!\r\n");
            tkl_gpio_unlock();
            return OPRT_NOT_SUPPORTED;
    }

    pinmap[pin_id].cb = (TUYA_GPIO_IRQ_CB)cfg->cb;
    pinmap[pin_id].args = cfg->arg;
    pinmap[pin_id].irq_initialized = true;
    bk_gpio_register_isr(pinmap[pin_id].gpio, (gpio_isr_t)__tkl_gpio_irq_cb);
    bk_gpio_set_interrupt_type(pinmap[pin_id].gpio, trigger);
    tkl_gpio_unlock();
    return OPRT_OK;
}

/**
 * @brief gpio irq enable
 *
 * @param[in] pin_id: gpio pin id
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_irq_enable(TUYA_GPIO_NUM_E pin_id)
{
    OPERATE_RET ret;
    UINT_T irq_mask;

    PIN_DEV_CHECK_ERROR_RETURN(pin_id, OPRT_INVALID_PARM);

    if (platform_is_in_interrupt_context() != 0) {
        irq_mask = tkl_system_enter_critical();
        if (!pinmap[pin_id].initialized || !pinmap[pin_id].irq_initialized) {
            tkl_system_exit_critical(irq_mask);
            return OPRT_RESOURCE_NOT_READY;
        }

        bk_gpio_enable_interrupt(pinmap[pin_id].gpio);
        tkl_system_exit_critical(irq_mask);
        return OPRT_OK;
    }

    ret = tkl_gpio_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    if (!pinmap[pin_id].initialized || !pinmap[pin_id].irq_initialized) {
        tkl_gpio_unlock();
        return OPRT_RESOURCE_NOT_READY;
    }

    bk_gpio_enable_interrupt(pinmap[pin_id].gpio);
    tkl_gpio_unlock();
    return OPRT_OK;
}

/**
 * @brief gpio irq disable
 *
 * @param[in] pin_id: gpio pin id
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_irq_disable(TUYA_GPIO_NUM_E pin_id)
{
    OPERATE_RET ret;
    UINT_T irq_mask;

    PIN_DEV_CHECK_ERROR_RETURN(pin_id, OPRT_INVALID_PARM);

    if (platform_is_in_interrupt_context() != 0) {
        irq_mask = tkl_system_enter_critical();
        if (!pinmap[pin_id].initialized || !pinmap[pin_id].irq_initialized) {
            tkl_system_exit_critical(irq_mask);
            return OPRT_RESOURCE_NOT_READY;
        }

        bk_gpio_disable_interrupt(pinmap[pin_id].gpio);
        tkl_system_exit_critical(irq_mask);
        return OPRT_OK;
    }

    ret = tkl_gpio_lock();
    if (ret != OPRT_OK) {
        return ret;
    }

    if (!pinmap[pin_id].initialized || !pinmap[pin_id].irq_initialized) {
        tkl_gpio_unlock();
        return OPRT_RESOURCE_NOT_READY;
    }

    bk_gpio_disable_interrupt(pinmap[pin_id].gpio);
    tkl_gpio_unlock();
    return OPRT_OK;
}
