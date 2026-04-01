/**
* @file tkl_8080.h
* @brief Common process - 8080 display process
* @version 0.1
* @date 2025-03-27
*
* @copyright Copyright 2021-2025 Tuya Inc. All Rights Reserved.
*
*/
#ifndef __TKL_8080_H__
#define __TKL_8080_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tuya_cloud_types.h"

typedef enum {
    MCU8080_OUTPUT_FINISH = 0,
} TUYA_8080_EVENT_E;


typedef VOID_T (*tuya_8080_isr_t)(TUYA_8080_EVENT_E event);

/**
 * @brief 8080 init
 * 
 * @param[in] cfg: 8080 config
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_8080_init(TUYA_8080_BASE_CFG_T *cfg);

/**
 * @brief 8080 deinit
 * 
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_8080_deinit(VOID_T);

/**
 * @brief register 8080 cb
 * 
 * @param[in] cb: callback
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_8080_irq_cb_register(tuya_8080_isr_t cb);

/**
 * @brief ppi set
 * 
 * @param[in] width: ppi : width
 * @param[in] height: ppi : height
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_8080_ppi_set(UINT16_T width, UINT16_T height);

/**
 * @brief pixel mode set
 * 
 * @param[in] mode: mode, such as 565 or 888
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_8080_pixel_mode_set(DISPLAY_PIXEL_FORMAT_E mode);//input mode set

/**
 * @brief 8080 base addr set
 * 
 * @param[in] addr : base addr
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_8080_base_addr_set(UINT32_T addr);

/**
 * @brief  8080 transfer start
 * 
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_8080_transfer_start(VOID_T);

/**
 * @brief  8080 transfer stop
 * 
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_8080_transfer_stop(VOID_T);

/**
 * @brief 8080 cmd send
 * 
 * @param[in] cmd : cmd
 * 
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_8080_cmd_send(UINT32_T cmd);

/**
 * @brief 8080 cmd send(with param)
 * 
 *@param[in] cmd : cmd
 *@param[in] param : param data buf
 *@param[in] param_cnt : param cnt
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_8080_cmd_send_with_param(UINT32_T cmd, UINT32_T *param, UINT8_T param_cnt);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif