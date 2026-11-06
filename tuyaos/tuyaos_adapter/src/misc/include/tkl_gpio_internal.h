/**
 * @file tkl_gpio_internal.h
 * @brief Adapter-internal GPIO pin validation, T9-specific.
 *
 * @details
 * The public TUYA_GPIO_NUM_E is a cross-platform pin numbering space, so its
 * TUYA_GPIO_NUM_MAX bound says nothing about how many pins this SoC actually
 * has. On BK7259 that is 72 (SOC_GPIO_NUM, and pinmap[] in tkl_gpio.c), while
 * the enum now runs to 171.
 *
 * Adapter code that accepts a pin from an upper layer must therefore ask this
 * module rather than compare against TUYA_GPIO_NUM_MAX: the enum bound would
 * let 72..170 through, and the vendor drivers below do not re-check. tkl_gpio.c
 * itself validates every entry point already; this header exists so the other
 * TKL modules (wakeup, sdio, ...) can apply the same rule.
 *
 * @version 0.1
 * @date 2026-11-04
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 */

#ifndef __TKL_GPIO_INTERNAL_H__
#define __TKL_GPIO_INTERNAL_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Whether the pin exists on this SoC and is routed out on this board.
 *
 * @param[in] pin_id pin to check
 * @return TRUE when the pin may be handed to a vendor driver, FALSE otherwise
 */
BOOL_T __t9_gpio_is_valid_pin(TUYA_GPIO_NUM_E pin_id);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_GPIO_INTERNAL_H__ */
