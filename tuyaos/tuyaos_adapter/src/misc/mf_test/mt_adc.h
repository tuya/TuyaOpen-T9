/**
 * @file mt_adc.h
 * @brief ADC voltage manufacturing test for T9 test socket (P24/P25)
 * @version 1.0
 * @date 2026-07-06
 * @copyright Copyright (c) Tuya Inc.
 */
#ifndef __MT_ADC_H__
#define __MT_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tuya_cloud_types.h"

/**
 * @brief Read P24(1V2) and P25(1V8) voltages and verify against expected rails
 * @return OPRT_OK on pass, OPRT_COM_ERROR on test failure
 */
OPERATE_RET mt_adc_factory_test(VOID_T);

/**
 * @brief Run ADC factory test and fill JSON reply buffer for mftest framework
 * @param[out] ret_data heap buffer with JSON result (caller frees)
 * @param[out] ret_len length of @p ret_data
 * @return OPRT_OK on dispatch success, OPRT_MALLOC_FAILED on alloc failure
 */
OPERATE_RET mt_adc_factory_test_reply(UCHAR_T **ret_data, USHORT_T *ret_len);

#ifdef __cplusplus
}
#endif

#endif /* __MT_ADC_H__ */
