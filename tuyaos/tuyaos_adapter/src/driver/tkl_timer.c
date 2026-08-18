/**
 * @file tkl_timer.c
 * @brief TKL timer adapter for T9 (Beken AP timer driver)
 * @version 0.1
 * @date 2026-04-28
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_timer.h"

#include <common/bk_err.h>
#include <driver/timer.h>
#include <string.h>

#define TKL_TIMER_BK_BASE_ID      TIMER_ID12
#define TKL_TIMER_BK_MAX_ID       TIMER_ID17
/*
 * AP TIMER4/5 count at bus clock. At 480M CPU the bus is 240MHz, so the
 * 32-bit end_count caps period to ~17.895s. Use this as a safe upper bound.
 */
#define TKL_TIMER_MAX_BUS_FREQ_KHZ  240000u
#define TKL_TIMER_MAX_PERIOD_US \
    ((UINT32_T)((0xFFFFFFFFull * 1000ull) / (UINT64_T)TKL_TIMER_MAX_BUS_FREQ_KHZ))

typedef struct {
    UINT8_T inited;
    UINT8_T running;
    TUYA_TIMER_MODE_E mode;
    TUYA_TIMER_ISR_CB cb;
    VOID *args;
    UINT_T period_us;
} tkl_timer_ctx_t;

STATIC tkl_timer_ctx_t s_timer_ctx[TUYA_TIMER_NUM_MAX];
STATIC UINT8_T s_timer_driver_inited = 0;
STATIC UINT8_T s_timer_init_count = 0;

STATIC OPERATE_RET __tkl_timer_map_bk_err(bk_err_t bk_ret)
{
    if (bk_ret == BK_OK) {
        return OPRT_OK;
    }
    if ((bk_ret == BK_ERR_NULL_PARAM) || (bk_ret == BK_ERR_PARAM) ||
        (bk_ret == BK_ERR_TIMER_ID) || (bk_ret == BK_ERR_TIMER_ID_ON_DEFCONFIG)) {
        return OPRT_INVALID_PARM;
    }
    if (bk_ret == BK_ERR_NOT_SUPPORT) {
        return OPRT_NOT_SUPPORTED;
    }
    return OPRT_COM_ERROR;
}

STATIC OPERATE_RET __tkl_timer_id_check(TUYA_TIMER_NUM_E timer_id)
{
    if (timer_id >= TUYA_TIMER_NUM_MAX) {
        return OPRT_INVALID_PARM;
    }
    return OPRT_OK;
}

STATIC OPERATE_RET __tkl_timer_cfg_check(CONST TUYA_TIMER_BASE_CFG_T *cfg)
{
    if (cfg == NULL) {
        return OPRT_INVALID_PARM;
    }
    if ((cfg->mode != TUYA_TIMER_MODE_ONCE) && (cfg->mode != TUYA_TIMER_MODE_PERIOD)) {
        return OPRT_INVALID_PARM;
    }
    return OPRT_OK;
}

STATIC timer_id_t __tkl_timer_to_bk_id(TUYA_TIMER_NUM_E timer_id)
{
    return (timer_id_t)((INT_T)TKL_TIMER_BK_BASE_ID + (INT_T)timer_id);
}

STATIC BOOL_T __tkl_timer_bk_id_valid(timer_id_t bk_timer_id)
{
    if ((bk_timer_id < TKL_TIMER_BK_BASE_ID) || (bk_timer_id > TKL_TIMER_BK_MAX_ID)) {
        return FALSE;
    }
    return TRUE;
}

STATIC VOID __tkl_timer_isr(timer_id_t bk_timer_id)
{
    TUYA_TIMER_NUM_E timer_id;
    timer_id_t real_bk_timer_id;
    INT_T idx;
    tkl_timer_ctx_t *ctx;
    TUYA_TIMER_ISR_CB user_cb;
    VOID *user_arg;

    /*
     * Depending on the vendor timer_driver version, the ISR callback argument
     * is either the hardware channel (0..5) or the full BK timer id
     * (TIMER_ID12..17 == TKL_TIMER_BK_BASE_ID + chan). Normalize both forms to
     * a logical tkl timer id (0..5); otherwise the full-id form is wrongly
     * rejected as out of range and the user callback never runs.
     */
    if ((bk_timer_id >= TKL_TIMER_BK_BASE_ID) && (bk_timer_id <= TKL_TIMER_BK_MAX_ID)) {
        idx = (INT_T)bk_timer_id - (INT_T)TKL_TIMER_BK_BASE_ID;
    } else {
        idx = (INT_T)bk_timer_id;
    }

    if ((idx < 0) || ((UINT_T)idx >= TUYA_TIMER_NUM_MAX)) {
        return;
    }

    timer_id = (TUYA_TIMER_NUM_E)idx;
    real_bk_timer_id = __tkl_timer_to_bk_id(timer_id);

    ctx = &s_timer_ctx[timer_id];
    if (ctx->inited == 0) {
        return;
    }

    if (ctx->mode == TUYA_TIMER_MODE_ONCE) {
        (void)bk_timer_stop(real_bk_timer_id);
        (void)bk_timer_cancel(real_bk_timer_id);
        ctx->running = 0;
    }

    user_cb = ctx->cb;
    user_arg = ctx->args;
    if (user_cb != NULL) {
        user_cb(user_arg);
    }
}

OPERATE_RET tkl_timer_init(TUYA_TIMER_NUM_E timer_id, TUYA_TIMER_BASE_CFG_T *cfg)
{
    OPERATE_RET op_ret;
    bk_err_t bk_ret;
    tkl_timer_ctx_t *ctx;

    op_ret = __tkl_timer_id_check(timer_id);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    op_ret = __tkl_timer_cfg_check(cfg);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    if (s_timer_driver_inited == 0) {
        bk_ret = bk_timer_driver_init();
        if (bk_ret != BK_OK) {
            return __tkl_timer_map_bk_err(bk_ret);
        }
        s_timer_driver_inited = 1;
    }

    ctx = &s_timer_ctx[timer_id];
    if (ctx->inited == 0) {
        s_timer_init_count++;
    }

    ctx->inited = 1;
    ctx->mode = cfg->mode;
    ctx->cb = cfg->cb;
    ctx->args = cfg->args;
    if (ctx->running == 0) {
        ctx->period_us = 0;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_timer_start(TUYA_TIMER_NUM_E timer_id, UINT_T us)
{
    OPERATE_RET op_ret;
    timer_id_t bk_timer_id;
    tkl_timer_ctx_t *ctx;
    timer_isr_t bk_cb = NULL;
    bk_err_t bk_ret;

    op_ret = __tkl_timer_id_check(timer_id);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }
    if ((us == 0u) || (us > TKL_TIMER_MAX_PERIOD_US)) {
        return OPRT_INVALID_PARM;
    }

    ctx = &s_timer_ctx[timer_id];
    if ((ctx->inited == 0) || (s_timer_driver_inited == 0)) {
        return OPRT_COM_ERROR;
    }

    bk_timer_id = __tkl_timer_to_bk_id(timer_id);
    if (__tkl_timer_bk_id_valid(bk_timer_id) == FALSE) {
        return OPRT_NOT_SUPPORTED;
    }

    if ((ctx->mode == TUYA_TIMER_MODE_ONCE) || (ctx->cb != NULL)) {
        bk_cb = __tkl_timer_isr;
    }

    bk_ret = bk_timer_start_us(bk_timer_id, (uint64_t)us, bk_cb);
    if (bk_ret != BK_OK) {
        return __tkl_timer_map_bk_err(bk_ret);
    }

    ctx->running = 1;
    ctx->period_us = us;
    return OPRT_OK;
}

OPERATE_RET tkl_timer_stop(TUYA_TIMER_NUM_E timer_id)
{
    OPERATE_RET op_ret;
    timer_id_t bk_timer_id;
    tkl_timer_ctx_t *ctx;
    bk_err_t bk_ret;

    op_ret = __tkl_timer_id_check(timer_id);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    ctx = &s_timer_ctx[timer_id];
    if ((ctx->inited == 0) || (s_timer_driver_inited == 0)) {
        return OPRT_COM_ERROR;
    }

    if (ctx->running == 0) {
        return OPRT_OK;
    }

    bk_timer_id = __tkl_timer_to_bk_id(timer_id);
    bk_ret = bk_timer_stop(bk_timer_id);
    if (bk_ret != BK_OK) {
        return __tkl_timer_map_bk_err(bk_ret);
    }

    (void)bk_timer_cancel(bk_timer_id);
    ctx->running = 0;
    return OPRT_OK;
}

OPERATE_RET tkl_timer_deinit(TUYA_TIMER_NUM_E timer_id)
{
    OPERATE_RET op_ret;
    timer_id_t bk_timer_id;
    tkl_timer_ctx_t *ctx;
    bk_err_t bk_ret;
    OPERATE_RET final_ret = OPRT_OK;

    op_ret = __tkl_timer_id_check(timer_id);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }

    ctx = &s_timer_ctx[timer_id];
    if ((ctx->inited == 0) || (s_timer_driver_inited == 0)) {
        return OPRT_OK;
    }

    bk_timer_id = __tkl_timer_to_bk_id(timer_id);
    if (ctx->running) {
        bk_ret = bk_timer_stop(bk_timer_id);
        if ((bk_ret != BK_OK) && (final_ret == OPRT_OK)) {
            final_ret = __tkl_timer_map_bk_err(bk_ret);
        }
        ctx->running = 0;
    }

    bk_ret = bk_timer_cancel(bk_timer_id);
    if ((bk_ret != BK_OK) && (final_ret == OPRT_OK)) {
        final_ret = __tkl_timer_map_bk_err(bk_ret);
    }

    (void)memset(ctx, 0, sizeof(*ctx));
    if (s_timer_init_count > 0) {
        s_timer_init_count--;
    }

    if ((s_timer_init_count == 0) && (s_timer_driver_inited != 0)) {
        bk_ret = bk_timer_driver_deinit();
        if ((bk_ret != BK_OK) && (final_ret == OPRT_OK)) {
            final_ret = __tkl_timer_map_bk_err(bk_ret);
        }
        if (bk_ret == BK_OK) {
            s_timer_driver_inited = 0;
        }
    }

    return final_ret;
}

OPERATE_RET tkl_timer_get_current_value(TUYA_TIMER_NUM_E timer_id, UINT_T *us)
{
    OPERATE_RET op_ret;
    timer_id_t bk_timer_id;
    uint64_t cur_us;

    op_ret = __tkl_timer_id_check(timer_id);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }
    if (us == NULL) {
        return OPRT_INVALID_PARM;
    }
    if ((s_timer_ctx[timer_id].inited == 0) || (s_timer_driver_inited == 0)) {
        return OPRT_COM_ERROR;
    }

    bk_timer_id = __tkl_timer_to_bk_id(timer_id);
    cur_us = bk_timer_get_time(bk_timer_id, 1, 0, TIMER_UNIT_US);
    if (cur_us > 0xFFFFFFFFull) {
        *us = 0xFFFFFFFFu;
    } else {
        *us = (UINT_T)cur_us;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_timer_get(TUYA_TIMER_NUM_E timer_id, UINT_T *us)
{
    OPERATE_RET op_ret;

    op_ret = __tkl_timer_id_check(timer_id);
    if (op_ret != OPRT_OK) {
        return op_ret;
    }
    if (us == NULL) {
        return OPRT_INVALID_PARM;
    }
    if ((s_timer_ctx[timer_id].inited == 0) || (s_timer_driver_inited == 0)) {
        return OPRT_COM_ERROR;
    }

    *us = s_timer_ctx[timer_id].period_us;
    return OPRT_OK;
}
