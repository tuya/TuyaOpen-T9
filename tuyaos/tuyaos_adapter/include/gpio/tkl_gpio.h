/**
 * @file tkl_gpio.h
 * @brief Common process - adapter the gpio api
 * @version 0.1
 * @date 2021-08-06
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_GPIO_H__
#define __TKL_GPIO_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief gpio init
 * 
 * @param[in] pin_id: gpio pin id, id index starts at 0
 * @param[in] cfg:  gpio config
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_init(TUYA_GPIO_NUM_E pin_id, CONST TUYA_GPIO_BASE_CFG_T *cfg);

/**
 * @brief gpio deinit
 * 
 * @param[in] pin_id: gpio pin id, id index starts at 0
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_deinit(TUYA_GPIO_NUM_E pin_id);

/**
 * @brief gpio write
 * 
 * @param[in] pin_id: gpio pin id, id index starts at 0
 * @param[in] level: gpio output level value
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_write(TUYA_GPIO_NUM_E pin_id, TUYA_GPIO_LEVEL_E level);

/**
 * @brief gpio read
 * 
 * @param[in] pin_id: gpio pin id, id index starts at 0
 * @param[out] level: gpio output level
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_read(TUYA_GPIO_NUM_E pin_id, TUYA_GPIO_LEVEL_E *level);

/**
 * @brief gpio irq init
 * NOTE: call this API will not enable interrupt
 * 
 * @param[in] pin_id: gpio pin id, id index starts at 0
 * @param[in] cfg:  gpio irq config
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_irq_init(TUYA_GPIO_NUM_E pin_id, CONST TUYA_GPIO_IRQ_T *cfg);

/**
 * @brief gpio irq enable
 * 
 * @param[in] pin_id: gpio pin id, id index starts at 0
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_irq_enable(TUYA_GPIO_NUM_E pin_id);

/**
 * @brief gpio irq disable
 * 
 * @param[in] pin_id: gpio pin id, id index starts at 0
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_gpio_irq_disable(TUYA_GPIO_NUM_E pin_id);

/**
 * @brief Set GPIO drive capability (drive strength)
 * @param[in] pin_id GPIO pin id, id index starts at 0
 * @param[in] strength Desired drive capability level
 *                     (TUYA_GPIO_DRIVE_CAP_0 .. TUYA_GPIO_DRIVE_CAP_4)
 * @return OPRT_OK on success, OPRT_INVALID_PARM if pin_id is invalid or strength
 *         is out of range,
 *         OPRT_NOT_SUPPORTED if drive-strength control is unavailable on this
 *         pin/platform, or the requested level exceeds what the pin supports
 * @note TUYA_GPIO_DRIVE_CAP_E is an abstract ordinal level, NOT an mA value:
 *       TUYA_GPIO_DRIVE_CAP_0 is the weakest and TUYA_GPIO_DRIVE_CAP_4 the
 *       strongest. It conceptually bounds the maximum current the pad can
 *       source or sink, but callers pass the abstract level, not a raw mA
 *       figure; the actual mA each level maps to is SoC/pad specific and must
 *       be documented in the pinmap of tkl_gpio.c. A platform that cannot
 *       configure drive strength, or a level beyond what a pin supports, must
 *       return OPRT_NOT_SUPPORTED rather than silently clamping to a lower
 *       level. TUYA_GPIO_DRIVE_CAP_MAX is a sentinel and is not a valid level.
 */
OPERATE_RET tkl_gpio_set_drive_capability(TUYA_GPIO_NUM_E pin_id, CONST TUYA_GPIO_DRIVE_CAP_E strength);

/**
 * @brief Get GPIO drive capability (drive strength)
 * @param[in] pin_id GPIO pin id, id index starts at 0
 * @param[out] strength Pointer to receive the current drive capability level;
 *                      must not be NULL
 * @return OPRT_OK on success, OPRT_INVALID_PARM if pin_id or strength is invalid,
 *         OPRT_NOT_SUPPORTED if drive-strength readback is unavailable on this
 *         pin/platform
 * @note On success *strength is set to one of TUYA_GPIO_DRIVE_CAP_0 ..
 *       TUYA_GPIO_DRIVE_CAP_4. As with tkl_gpio_set_drive_capability(), the
 *       value is an abstract ordinal level, NOT an mA figure; the mA-per-level
 *       mapping is SoC/pad specific (see the pinmap in tkl_gpio.c).
 */
OPERATE_RET tkl_gpio_get_drive_capability(TUYA_GPIO_NUM_E pin_id, TUYA_GPIO_DRIVE_CAP_E *strength);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

