/**
 * @file tkl_watchdog.c
 * @brief TKL watchdog adapter for T9 AP (Beken WWDT)
 * @version 0.1
 * @date 2026-08-03
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_watchdog.h"

// #include "sdkconfig.h"

#if defined(CONFIG_SUPPORT_WWDT) && (CONFIG_SUPPORT_WWDT)
#include <driver/wwdt.h>
#include <common/bk_err.h>
#endif

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
/* Matches WWDT_F_PERIOD_V in soc/wwdt_reg.h (timeout_ms upper bound). */
#define TKL_WWDT_PERIOD_MAX_MS    (0xffffu)

#if defined(CONFIG_SUPPORT_WWDT) && (CONFIG_SUPPORT_WWDT)
#ifndef CONFIG_INT_WWDT_PERIOD_MS
#define CONFIG_INT_WWDT_PERIOD_MS 2000
#endif
#endif

/* ---------------------------------------------------------------------------
 * File scope variables
 * --------------------------------------------------------------------------- */
#if defined(CONFIG_SUPPORT_WWDT) && (CONFIG_SUPPORT_WWDT)
STATIC UINT8_T s_wdog_inited = 0;
STATIC UINT32_T s_wdog_period_ms = 0;
#endif

/* ---------------------------------------------------------------------------
 * Function implementations
 * --------------------------------------------------------------------------- */
/**
 * @brief watchdog init
 *
 * @param[in] cfg: watchdog config
 *
 * @return 0, init error; >0 : the actually watchdog interval
 * @note Maps to AP WWDT (bk_wwdt_*). Window feature is not used.
 *       Boot may already start WWDT; this call reconfigures the period.
 */
UINT32_T tkl_watchdog_init(TUYA_WDOG_BASE_CFG_T *cfg)
{
#if defined(CONFIG_SUPPORT_WWDT) && (CONFIG_SUPPORT_WWDT)
    UINT32_T timeout_ms;
    bk_err_t bk_ret;

    if (cfg == NULL) {
        return 0;
    }

    timeout_ms = (UINT32_T)cfg->interval_ms;
    if (timeout_ms == 0) {
        timeout_ms = (UINT32_T)CONFIG_INT_WWDT_PERIOD_MS;
    }
    if (timeout_ms > TKL_WWDT_PERIOD_MAX_MS) {
        return 0;
    }

    bk_ret = bk_wwdt_driver_init();
    if (bk_ret != BK_OK) {
        return 0;
    }

    /* Disable window mode; TKL only exposes a plain timeout. */
    bk_ret = bk_wwdt_start(timeout_ms, false, 0);
    if (bk_ret != BK_OK) {
        return 0;
    }

    s_wdog_inited = 1;
    s_wdog_period_ms = timeout_ms;
    return timeout_ms;
#else
    (void)cfg;
    return 0;
#endif
}

/**
 * @brief watchdog deinit
 *
 * @param[in] none
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 * @note Stops current-core WWDT and disables SysTick auto-restart for this core.
 */
OPERATE_RET tkl_watchdog_deinit(VOID_T)
{
#if defined(CONFIG_SUPPORT_WWDT) && (CONFIG_SUPPORT_WWDT)
    bk_err_t bk_ret;

    if (s_wdog_inited == 0) {
        return OPRT_OK;
    }

    bk_ret = bk_wwdt_stop();
    if (bk_ret != BK_OK) {
        return OPRT_COM_ERROR;
    }

    s_wdog_inited = 0;
    s_wdog_period_ms = 0;
    return OPRT_OK;
#else
    return OPRT_NOT_SUPPORTED;
#endif
}

/**
 * @brief refresh watch dog
 *
 * @param[in] none
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 * @note Feeds current-core WWDT via bk_wwdt_feed() (no software throttle).
 *       Safe for long blocking paths such as OTA flash erase/write.
 */
OPERATE_RET tkl_watchdog_refresh(VOID_T)
{
#if defined(CONFIG_SUPPORT_WWDT) && (CONFIG_SUPPORT_WWDT)
    bk_err_t bk_ret;

    /*
     * Boot path may have started WWDT before tkl_watchdog_init().
     * Allow refresh in that case as long as the driver is ready.
     */
    if (!bk_wwdt_is_driver_inited()) {
        return OPRT_COM_ERROR;
    }

    bk_ret = bk_wwdt_feed();
    if (bk_ret == BK_ERR_WWDT_NOT_INIT) {
        /* Core WWDT not active yet — start with last/default period then feed. */
        UINT32_T timeout_ms = s_wdog_period_ms ? s_wdog_period_ms
                                               : (UINT32_T)CONFIG_INT_WWDT_PERIOD_MS;
        bk_ret = bk_wwdt_start(timeout_ms, false, 0);
        if (bk_ret != BK_OK) {
            return OPRT_COM_ERROR;
        }
        s_wdog_inited = 1;
        s_wdog_period_ms = timeout_ms;
        bk_ret = bk_wwdt_feed();
    }

    if (bk_ret != BK_OK) {
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
#else
    return OPRT_NOT_SUPPORTED;
#endif
}
