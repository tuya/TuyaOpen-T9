/**
 * @file tkl_pwm.c
 * @brief TKL PWM adapter for T9 (Beken PWM v2p2 driver)
 * @version 0.1
 * @date 2026-04-23
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_pwm.h"

#include <common/bk_err.h>
#include <driver/pwm.h>
#include <driver/hal/hal_pwm_types.h>
#include <string.h>

#include "pwm_hal_v2p2.h"

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define TKL_PWM_XTAL_HZ        26000000u
#define TKL_PWM_PSC_MAX        255u
#define TKL_PWM_PERIOD_MAX     65535u

/* ---------------------------------------------------------------------------
 * Type definitions
 * --------------------------------------------------------------------------- */
typedef struct {
    UINT8_T             inited;
    UINT8_T             running;
    UINT8_T             capture_mode;
    TUYA_PWM_BASE_CFG_T cfg;
    UINT32_T           psc;
    UINT32_T           period_cycle;
} tkl_pwm_ch_ctx_t;

/* ---------------------------------------------------------------------------
 * File scope variables
 * --------------------------------------------------------------------------- */
STATIC tkl_pwm_ch_ctx_t s_pwm_ctx[TUYA_PWM_NUM_MAX];
STATIC TUYA_PWM_CAP_IRQ_T s_cap_user_cfg[TUYA_PWM_NUM_MAX];

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
STATIC void __tkl_pwm_cap_isr(pwm_chan_t chan);

/* ---------------------------------------------------------------------------
 * Function implementations
 * --------------------------------------------------------------------------- */
/**
 * @brief Map Beken error code to Tuya OPERATE_RET
 * @param[in] bk_ret vendor return code
 * @return OPRT_OK on BK_OK, else mapped error
 */
STATIC OPERATE_RET __tkl_pwm_map_bk_err(bk_err_t bk_ret)
{
    if (bk_ret == BK_OK) {
        return OPRT_OK;
    }
    if ((bk_ret == BK_ERR_NULL_PARAM) || (bk_ret == BK_ERR_PARAM)) {
        return OPRT_INVALID_PARM;
    }
    if (bk_ret == BK_ERR_NO_MEM) {
        return OPRT_MALLOC_FAILED;
    }
    if (bk_ret == BK_ERR_NOT_SUPPORT) {
        return OPRT_NOT_SUPPORTED;
    }
    if (bk_ret == BK_ERR_PWM_CHAN_ID) {
        return OPRT_INVALID_PARM;
    }
    if (bk_ret == BK_ERR_PWM_PERIOD_DUTY) {
        return OPRT_INVALID_PARM;
    }
    if (bk_ret == BK_ERR_PWM_NOT_INIT) {
        return OPRT_COM_ERROR;
    }
    if (bk_ret == BK_ERR_PWM_CHAN_NOT_INIT) {
        return OPRT_COM_ERROR;
    }
    return OPRT_COM_ERROR;
}

/**
 * @brief Validate logical PWM channel id
 * @param[in] ch_id Tuya PWM channel
 * @return OPRT_OK if valid
 */
STATIC OPERATE_RET __tkl_pwm_ch_check(TUYA_PWM_NUM_E ch_id)
{
    if (ch_id >= TUYA_PWM_NUM_MAX) {
        return OPRT_INVALID_PARM;
    }
    return OPRT_OK;
}

/**
 * @brief Compute prescaler and period in PWM timer ticks for a target frequency
 * @param[in] frequency_hz output frequency in Hz
 * @param[out] psc_out prescaler (0..255)
 * @param[out] period_out period in counter cycles
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_pwm_calc_psc_period(UINT32_T frequency_hz, UINT32_T *psc_out, UINT32_T *period_out)
{
    UINT32_T psc;

    if ((frequency_hz == 0) || (psc_out == NULL) || (period_out == NULL)) {
        return OPRT_INVALID_PARM;
    }

    for (psc = 0; psc <= TKL_PWM_PSC_MAX; psc++) {
        UINT64_T tick_hz = (UINT64_T)TKL_PWM_XTAL_HZ / ((UINT64_T)psc + 1u);
        UINT64_T per = tick_hz / (UINT64_T)frequency_hz;

        if (per == 0u) {
            continue;
        }
        if (per > TKL_PWM_PERIOD_MAX) {
            continue;
        }
        if (per < 1u) {
            continue;
        }
        *psc_out = psc;
        *period_out = (UINT32_T)per;
        return OPRT_OK;
    }

    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief Convert Tuya duty/cycle ratio to high-time ticks for Beken init
 * @param[in] duty Tuya duty numerator
 * @param[in] cycle Tuya cycle denominator
 * @param[in] period_cycle hardware period ticks
 * @param[out] duty_high_ticks output high time in ticks
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_pwm_ratio_to_ticks(UINT32_T duty, UINT32_T cycle, UINT32_T period_cycle,
                                            UINT32_T *duty_high_ticks)
{
    UINT64_T high;

    if ((cycle == 0) || (duty_high_ticks == NULL)) {
        return OPRT_INVALID_PARM;
    }

    high = (UINT64_T)period_cycle * (UINT64_T)duty;
    high /= (UINT64_T)cycle;
    if (high > (UINT64_T)period_cycle) {
        return OPRT_INVALID_PARM;
    }
    *duty_high_ticks = (UINT32_T)high;
    return OPRT_OK;
}

/**
 * @brief Fill pwm_init_config_t from Tuya base config
 * @param[in] cfg Tuya PWM configuration
 * @param[out] init_cfg Beken init structure
 * @param[out] psc prescaler used
 * @param[out] period_cycle period ticks used
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_pwm_build_init_config(CONST TUYA_PWM_BASE_CFG_T *cfg, pwm_init_config_t *init_cfg,
                                               UINT32_T *psc, UINT32_T *period_cycle)
{
    OPERATE_RET op;
    UINT32_T high_ticks = 0;

    if ((cfg == NULL) || (init_cfg == NULL) || (psc == NULL) || (period_cycle == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if (cfg->count_mode != TUYA_PWM_CNT_UP) {
        return OPRT_NOT_SUPPORTED;
    }

    if (cfg->cycle == 0) {
        return OPRT_INVALID_PARM;
    }

    op = __tkl_pwm_calc_psc_period(cfg->frequency, psc, period_cycle);
    if (op != OPRT_OK) {
        return op;
    }

    op = __tkl_pwm_ratio_to_ticks(cfg->duty, cfg->cycle, *period_cycle, &high_ticks);
    if (op != OPRT_OK) {
        return op;
    }

    (void)memset(init_cfg, 0, sizeof(pwm_init_config_t));
    init_cfg->period_cycle = *period_cycle;
    init_cfg->duty_cycle = high_ticks;
    init_cfg->duty2_cycle = 0;
    init_cfg->duty3_cycle = 0;
    init_cfg->psc = *psc;
    return OPRT_OK;
}

/**
 * @brief Apply polarity to an initialized channel
 * @param[in] ch Beken channel
 * @param[in] pol Tuya polarity
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_pwm_apply_polarity(pwm_chan_t ch, TUYA_PWM_POLARITY_E pol)
{
    pwm_polarity_t hal_pol = PWM_POLARITY_POSITIVE;

    if (pol == TUYA_PWM_NEGATIVE) {
        hal_pol = PWM_POLARITY_NEGATIVE;
    } else if (pol == TUYA_PWM_POSITIVE) {
        hal_pol = PWM_POLARITY_POSITIVE;
    } else {
        return OPRT_INVALID_PARM;
    }

    return __tkl_pwm_map_bk_err(pwm_hal_set_polarity(ch, hal_pol));
}

/**
 * @brief Map capture trigger level to Beken capture edge
 * @param[in] level Tuya trigger level
 * @param[out] edge_out Beken edge mode
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_pwm_cap_edge_from_trigger(TUYA_PWM_POLARITY_E level, pwm_capture_edge_t *edge_out)
{
    if (edge_out == NULL) {
        return OPRT_INVALID_PARM;
    }
    if (level == TUYA_PWM_POSITIVE) {
        *edge_out = PWM_CAPTURE_POS;
        return OPRT_OK;
    }
    if (level == TUYA_PWM_NEGATIVE) {
        *edge_out = PWM_CAPTURE_NEG;
        return OPRT_OK;
    }
    return OPRT_INVALID_PARM;
}

/**
 * @brief PWM capture ISR bridge to Tuya callback
 * @param[in] chan Beken channel index
 * @return none
 */
STATIC void __tkl_pwm_cap_isr(pwm_chan_t chan)
{
    TUYA_PWM_NUM_E port;
    TUYA_PWM_CAPTURE_DATA_T data;

    if (chan >= (pwm_chan_t)TUYA_PWM_NUM_MAX) {
        return;
    }

    port = (TUYA_PWM_NUM_E)chan;
    (void)memset(&data, 0, sizeof(data));
    data.cap_value = bk_pwm_capture_get_value(chan);
    data.cap_edge = s_cap_user_cfg[chan].trigger_level;

    if (s_cap_user_cfg[chan].cb != NULL) {
        s_cap_user_cfg[chan].cb(port, data, s_cap_user_cfg[chan].arg);
    }
}

OPERATE_RET tkl_pwm_init(TUYA_PWM_NUM_E ch_id, CONST TUYA_PWM_BASE_CFG_T *cfg)
{
    pwm_init_config_t init_cfg;
    pwm_chan_t ch;
    OPERATE_RET op;
    bk_err_t bk_ret;
    UINT32_T psc = 0;
    UINT32_T period = 0;

    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    op = __tkl_pwm_ch_check(ch_id);
    if (op != OPRT_OK) {
        return op;
    }

    ch = (pwm_chan_t)ch_id;

    bk_ret = bk_pwm_driver_init();
    op = __tkl_pwm_map_bk_err(bk_ret);
    if (op != OPRT_OK) {
        return op;
    }

    if (s_pwm_ctx[ch_id].inited) {
        if (s_pwm_ctx[ch_id].capture_mode) {
            (void)bk_pwm_capture_stop(ch);
            (void)bk_pwm_capture_deinit(ch);
            s_pwm_ctx[ch_id].capture_mode = 0;
        } else {
            (void)bk_pwm_stop(ch);
            (void)bk_pwm_deinit(ch);
        }
        s_pwm_ctx[ch_id].inited = 0;
        s_pwm_ctx[ch_id].running = 0;
    }

    op = __tkl_pwm_build_init_config(cfg, &init_cfg, &psc, &period);
    if (op != OPRT_OK) {
        return op;
    }

    bk_ret = bk_pwm_init(ch, &init_cfg);
    op = __tkl_pwm_map_bk_err(bk_ret);
    if (op != OPRT_OK) {
        return op;
    }

    op = __tkl_pwm_apply_polarity(ch, cfg->polarity);
    if (op != OPRT_OK) {
        (void)bk_pwm_deinit(ch);
        return op;
    }

    (void)memcpy(&s_pwm_ctx[ch_id].cfg, cfg, sizeof(TUYA_PWM_BASE_CFG_T));
    s_pwm_ctx[ch_id].psc = psc;
    s_pwm_ctx[ch_id].period_cycle = period;
    s_pwm_ctx[ch_id].inited = 1;
    s_pwm_ctx[ch_id].running = 0;
    s_pwm_ctx[ch_id].capture_mode = 0;
    return OPRT_OK;
}

OPERATE_RET tkl_pwm_deinit(TUYA_PWM_NUM_E ch_id)
{
    pwm_chan_t ch;
    OPERATE_RET op;

    op = __tkl_pwm_ch_check(ch_id);
    if (op != OPRT_OK) {
        return op;
    }

    ch = (pwm_chan_t)ch_id;

    if (!s_pwm_ctx[ch_id].inited) {
        return OPRT_OK;
    }

    if (s_pwm_ctx[ch_id].capture_mode) {
        (void)bk_pwm_capture_stop(ch);
        (void)__tkl_pwm_map_bk_err(bk_pwm_capture_deinit(ch));
        (void)memset(&s_cap_user_cfg[ch_id], 0, sizeof(s_cap_user_cfg[ch_id]));
        s_pwm_ctx[ch_id].capture_mode = 0;
    } else {
        (void)bk_pwm_stop(ch);
        (void)__tkl_pwm_map_bk_err(bk_pwm_deinit(ch));
    }

    (void)memset(&s_pwm_ctx[ch_id], 0, sizeof(s_pwm_ctx[ch_id]));
    return OPRT_OK;
}

OPERATE_RET tkl_pwm_start(TUYA_PWM_NUM_E ch_id)
{
    pwm_chan_t ch;
    OPERATE_RET op;
    bk_err_t bk_ret;

    op = __tkl_pwm_ch_check(ch_id);
    if (op != OPRT_OK) {
        return op;
    }

    if (!s_pwm_ctx[ch_id].inited || s_pwm_ctx[ch_id].capture_mode) {
        return OPRT_COM_ERROR;
    }

    ch = (pwm_chan_t)ch_id;
    bk_ret = bk_pwm_start(ch);
    op = __tkl_pwm_map_bk_err(bk_ret);
    if (op == OPRT_OK) {
        s_pwm_ctx[ch_id].running = 1;
    }
    return op;
}

OPERATE_RET tkl_pwm_stop(TUYA_PWM_NUM_E ch_id)
{
    pwm_chan_t ch;
    OPERATE_RET op;
    bk_err_t bk_ret;

    op = __tkl_pwm_ch_check(ch_id);
    if (op != OPRT_OK) {
        return op;
    }

    if (!s_pwm_ctx[ch_id].inited || s_pwm_ctx[ch_id].capture_mode) {
        return OPRT_COM_ERROR;
    }

    ch = (pwm_chan_t)ch_id;
    bk_ret = bk_pwm_stop(ch);
    op = __tkl_pwm_map_bk_err(bk_ret);
    if (op == OPRT_OK) {
        s_pwm_ctx[ch_id].running = 0;
    }
    return op;
}

OPERATE_RET tkl_pwm_multichannel_start(TUYA_PWM_NUM_E *ch_id, UINT8_T num)
{
    UINT8_T i;
    OPERATE_RET op;

    if ((ch_id == NULL) || (num == 0)) {
        return OPRT_INVALID_PARM;
    }

    for (i = 0; i < num; i++) {
        op = tkl_pwm_start(ch_id[i]);
        if (op != OPRT_OK) {
            while (i > 0) {
                i--;
                (void)tkl_pwm_stop(ch_id[i]);
            }
            return op;
        }
    }
    return OPRT_OK;
}

OPERATE_RET tkl_pwm_multichannel_stop(TUYA_PWM_NUM_E *ch_id, UINT8_T num)
{
    UINT8_T i;
    OPERATE_RET op;
    OPERATE_RET last = OPRT_OK;

    if ((ch_id == NULL) || (num == 0)) {
        return OPRT_INVALID_PARM;
    }

    for (i = 0; i < num; i++) {
        op = tkl_pwm_stop(ch_id[i]);
        if (op != OPRT_OK) {
            last = op;
        }
    }
    return last;
}

OPERATE_RET tkl_pwm_duty_set(TUYA_PWM_NUM_E ch_id, UINT32_T duty)
{
    pwm_period_duty_config_t pd;
    pwm_chan_t ch;
    OPERATE_RET op;
    UINT32_T high_ticks = 0;
    bk_err_t bk_ret;

    op = __tkl_pwm_ch_check(ch_id);
    if (op != OPRT_OK) {
        return op;
    }

    if (!s_pwm_ctx[ch_id].inited || s_pwm_ctx[ch_id].capture_mode) {
        return OPRT_COM_ERROR;
    }

    if (s_pwm_ctx[ch_id].cfg.cycle == 0) {
        return OPRT_COM_ERROR;
    }

    ch = (pwm_chan_t)ch_id;
    op = __tkl_pwm_ratio_to_ticks(duty, s_pwm_ctx[ch_id].cfg.cycle, s_pwm_ctx[ch_id].period_cycle, &high_ticks);
    if (op != OPRT_OK) {
        return op;
    }

    (void)memset(&pd, 0, sizeof(pd));
    pd.period_cycle = s_pwm_ctx[ch_id].period_cycle;
    pd.duty_cycle = high_ticks;
    pd.duty2_cycle = 0;
    pd.duty3_cycle = 0;
    pd.psc = s_pwm_ctx[ch_id].psc;

    bk_ret = bk_pwm_set_period_duty(ch, &pd);
    op = __tkl_pwm_map_bk_err(bk_ret);
    if (op == OPRT_OK) {
        s_pwm_ctx[ch_id].cfg.duty = duty;
    }
    return op;
}

OPERATE_RET tkl_pwm_frequency_set(TUYA_PWM_NUM_E ch_id, UINT32_T frequency)
{
    pwm_period_duty_config_t pd;
    pwm_chan_t ch;
    OPERATE_RET op;
    UINT32_T psc = 0;
    UINT32_T period = 0;
    UINT32_T high_ticks = 0;
    bk_err_t bk_ret;

    op = __tkl_pwm_ch_check(ch_id);
    if (op != OPRT_OK) {
        return op;
    }

    if (!s_pwm_ctx[ch_id].inited || s_pwm_ctx[ch_id].capture_mode) {
        return OPRT_COM_ERROR;
    }

    if (s_pwm_ctx[ch_id].cfg.cycle == 0) {
        return OPRT_COM_ERROR;
    }

    op = __tkl_pwm_calc_psc_period(frequency, &psc, &period);
    if (op != OPRT_OK) {
        return op;
    }

    op = __tkl_pwm_ratio_to_ticks(s_pwm_ctx[ch_id].cfg.duty, s_pwm_ctx[ch_id].cfg.cycle, period, &high_ticks);
    if (op != OPRT_OK) {
        return op;
    }

    ch = (pwm_chan_t)ch_id;
    (void)memset(&pd, 0, sizeof(pd));
    pd.period_cycle = period;
    pd.duty_cycle = high_ticks;
    pd.duty2_cycle = 0;
    pd.duty3_cycle = 0;
    pd.psc = psc;

    bk_ret = bk_pwm_set_period_duty(ch, &pd);
    op = __tkl_pwm_map_bk_err(bk_ret);
    if (op == OPRT_OK) {
        s_pwm_ctx[ch_id].cfg.frequency = frequency;
        s_pwm_ctx[ch_id].psc = psc;
        s_pwm_ctx[ch_id].period_cycle = period;
    }
    return op;
}

OPERATE_RET tkl_pwm_polarity_set(TUYA_PWM_NUM_E ch_id, TUYA_PWM_POLARITY_E polarity)
{
    pwm_chan_t ch;
    OPERATE_RET op;

    op = __tkl_pwm_ch_check(ch_id);
    if (op != OPRT_OK) {
        return op;
    }

    if (!s_pwm_ctx[ch_id].inited || s_pwm_ctx[ch_id].capture_mode) {
        return OPRT_COM_ERROR;
    }

    ch = (pwm_chan_t)ch_id;
    op = __tkl_pwm_apply_polarity(ch, polarity);
    if (op == OPRT_OK) {
        s_pwm_ctx[ch_id].cfg.polarity = polarity;
    }
    return op;
}

OPERATE_RET tkl_pwm_info_set(TUYA_PWM_NUM_E ch_id, CONST TUYA_PWM_BASE_CFG_T *info)
{
    if (info == NULL) {
        return OPRT_INVALID_PARM;
    }
    return tkl_pwm_init(ch_id, info);
}

OPERATE_RET tkl_pwm_info_get(TUYA_PWM_NUM_E ch_id, TUYA_PWM_BASE_CFG_T *info)
{
    OPERATE_RET op;

    if (info == NULL) {
        return OPRT_INVALID_PARM;
    }

    op = __tkl_pwm_ch_check(ch_id);
    if (op != OPRT_OK) {
        return op;
    }

    if (!s_pwm_ctx[ch_id].inited) {
        return OPRT_COM_ERROR;
    }

    (void)memcpy(info, &s_pwm_ctx[ch_id].cfg, sizeof(TUYA_PWM_BASE_CFG_T));
    return OPRT_OK;
}

OPERATE_RET tkl_pwm_cap_start(TUYA_PWM_NUM_E ch_id, CONST TUYA_PWM_CAP_IRQ_T *cfg)
{
    pwm_capture_init_config_t cap_cfg;
    pwm_chan_t ch;
    OPERATE_RET op;
    bk_err_t bk_ret;

    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }

    op = __tkl_pwm_ch_check(ch_id);
    if (op != OPRT_OK) {
        return op;
    }

    if (cfg->cb == NULL) {
        return OPRT_INVALID_PARM;
    }

    ch = (pwm_chan_t)ch_id;

    bk_ret = bk_pwm_driver_init();
    op = __tkl_pwm_map_bk_err(bk_ret);
    if (op != OPRT_OK) {
        return op;
    }

    if (s_pwm_ctx[ch_id].inited && (!s_pwm_ctx[ch_id].capture_mode)) {
        (void)bk_pwm_stop(ch);
        (void)bk_pwm_deinit(ch);
        (void)memset(&s_pwm_ctx[ch_id], 0, sizeof(s_pwm_ctx[ch_id]));
    }

    if (s_pwm_ctx[ch_id].capture_mode) {
        (void)bk_pwm_capture_stop(ch);
        (void)bk_pwm_capture_deinit(ch);
        (void)memset(&s_cap_user_cfg[ch_id], 0, sizeof(s_cap_user_cfg[ch_id]));
        s_pwm_ctx[ch_id].capture_mode = 0;
        s_pwm_ctx[ch_id].inited = 0;
    }

    op = __tkl_pwm_cap_edge_from_trigger(cfg->trigger_level, &cap_cfg.edge);
    if (op != OPRT_OK) {
        return op;
    }

    cap_cfg.isr = __tkl_pwm_cap_isr;

    bk_ret = bk_pwm_capture_init(ch, &cap_cfg);
    op = __tkl_pwm_map_bk_err(bk_ret);
    if (op != OPRT_OK) {
        return op;
    }

    (void)memcpy(&s_cap_user_cfg[ch_id], cfg, sizeof(TUYA_PWM_CAP_IRQ_T));

    bk_ret = bk_pwm_capture_start(ch);
    op = __tkl_pwm_map_bk_err(bk_ret);
    if (op != OPRT_OK) {
        (void)bk_pwm_capture_deinit(ch);
        (void)memset(&s_cap_user_cfg[ch_id], 0, sizeof(s_cap_user_cfg[ch_id]));
        return op;
    }

    s_pwm_ctx[ch_id].inited = 1;
    s_pwm_ctx[ch_id].capture_mode = 1;
    s_pwm_ctx[ch_id].running = 1;

    return OPRT_OK;
}

OPERATE_RET tkl_pwm_cap_stop(TUYA_PWM_NUM_E ch_id)
{
    pwm_chan_t ch;
    OPERATE_RET op;
    bk_err_t bk_ret;

    op = __tkl_pwm_ch_check(ch_id);
    if (op != OPRT_OK) {
        return op;
    }

    if (!s_pwm_ctx[ch_id].inited || (!s_pwm_ctx[ch_id].capture_mode)) {
        return OPRT_COM_ERROR;
    }

    ch = (pwm_chan_t)ch_id;
    bk_ret = bk_pwm_capture_stop(ch);
    op = __tkl_pwm_map_bk_err(bk_ret);
    if (op != OPRT_OK) {
        return op;
    }

    bk_ret = bk_pwm_capture_deinit(ch);
    op = __tkl_pwm_map_bk_err(bk_ret);
    (void)memset(&s_cap_user_cfg[ch_id], 0, sizeof(s_cap_user_cfg[ch_id]));
    (void)memset(&s_pwm_ctx[ch_id], 0, sizeof(s_pwm_ctx[ch_id]));
    return op;
}
