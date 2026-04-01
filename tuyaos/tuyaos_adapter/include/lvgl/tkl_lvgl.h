/**
 * @file tkl_timer.h
 * @brief Common process - adapter the hardware timer api
 * @version 0.1
 * @date 2021-08-06
 *
 * @copyright Copyright 2018-2021 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_LVGL_H__
#define __TKL_LVGL_H__

#include "tuya_cloud_types.h"
#include "tkl_display.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef VOID_T (*TKL_LVGL_EVENT_CB)(VOID *buf, UINT_T len, VOID *args);

typedef struct {
    TKL_LVGL_EVENT_CB   recv_cb;
    VOID                *recv_arg;
} TKL_LVGL_CFG_T;


/**
 * @brief lvgl init
 *
 * @param[in] cfg configure
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_lvgl_init(TKL_DISP_INFO_S *disp_info, TKL_LVGL_CFG_T *cfg);

/**
 * @brief lvgl message sync
 *
 * @param[in] msg message
 * @param[in] length
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_lvgl_msg_sync(UINT8_T *msg, UINT32_T length);

/**
 * @brief lvgl start
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_lvgl_start(VOID);

/**
 * @brief lvgl stop
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_lvgl_stop(VOID);

/**
 * @brief lvgl uninit
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_lvgl_uninit(VOID);



#ifdef __cplusplus
} // extern "C"
#endif

#endif /* !__TKL_LVGL_H__ */
