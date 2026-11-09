/**
 * @file tkl_kws.h
 * @brief keyword spotting module head file
 * @version 0.1
 * @date 2021-08-18
 *
 * T9 (BK7259) KWS is implemented in src/driver/tkl_kws.c on top of the
 * tal_audio_subsys KWS module. Audio is pushed in by tkl_audio_frontend.c as
 * 20 ms mono blocks and decoded on a dedicated thread. The embedded model is
 * the upstream WeKWS demo "hey tino"; that is the sequence the driver
 * registers and reports as TKL_KWS_WAKEUP_TINO.
 *
 * @copyright Copyright (c) 2021-2026 Tuya Inc. All Rights Reserved.
 *
 */
#ifndef __TKL_KWS_H__
#define __TKL_KWS_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TKL_KWS_WAKEUP_WORD_UNKNOWN,
    TKL_KWS_WAKEUP_NIHAO_TUYA,
    TKL_KWS_WAKEUP_NIHAO_XIAOZHI,
    TKL_KWS_WAKEUP_HEY_TUYA,
    TKL_KWS_WAKEUP_SMARTLIFE,
    TKL_KWS_WAKEUP_ZHINENGGUANJIA,
    TKL_KWS_WAKEUP_XIAOZHI_TONGXUE,
    TKL_KWS_WAKEUP_XIAOZHI_GUANJIA,
    TKL_KWS_WAKEUP_XIAOAI_XIAOAI,
    TKL_KWS_WAKEUP_XIAODU_XIAODU,
    TKL_KWS_WAKEUP_TINO,
    TKL_KWS_WAKEUP_WORD_MAX,
} TKL_KWS_WAKEUP_WORD_E;

typedef void (*TKL_KWS_WAKEUP_CB)(TKL_KWS_WAKEUP_WORD_E wakeup_word);

OPERATE_RET tkl_kws_init(void);

OPERATE_RET tkl_kws_reg_wakeup_cb(TKL_KWS_WAKEUP_CB wakeup_cb);

OPERATE_RET tkl_kws_enable(void);

OPERATE_RET tkl_kws_disable(void);

OPERATE_RET tkl_kws_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
