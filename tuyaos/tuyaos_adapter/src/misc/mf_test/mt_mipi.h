/**
 * @file mt_mipi.h
 * @brief MIPI DSI display + CSI BIST manufacturing test for T9 platform
 * @version 1.0
 * @date 2026-07-06
 * @copyright Copyright (c) Tuya Inc.
 */
#ifndef __MT_MIPI_H__
#define __MT_MIPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tuya_cloud_types.h"

/**
 * @brief Run MIPI factory test (DSI PRBS7 flush + CSI BIST)
 * @return OPRT_OK on pass, OPRT_COM_ERROR on test failure, OPRT_NOT_SUPPORTED if panel unavailable
 */
OPERATE_RET mt_mipi_factory_test(VOID_T);

/**
 * @brief Run MIPI factory test and fill JSON reply buffer for mftest framework
 * @param[out] ret_data heap buffer with JSON result (caller frees)
 * @param[out] ret_len length of @p ret_data
 * @return OPRT_OK on dispatch success, OPRT_MALLOC_FAILED on alloc failure
 */
OPERATE_RET mt_mipi_factory_test_reply(UCHAR_T **ret_data, USHORT_T *ret_len);

#ifdef __cplusplus
}
#endif

#endif /* __MT_MIPI_H__ */
