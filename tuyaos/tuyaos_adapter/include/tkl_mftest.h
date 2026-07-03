/**
* Copyright (C) by Tuya Inc
* All rights reserved
*
* @file tkl_mftest.h
*/

#ifndef __TKL_MFTEST_H__
#define __TKL_MFTEST_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tuya_cloud_types.h"

typedef enum {
    TKL_AUDIO_TEST_START_CMD     = 0x0001,
    TKL_AUDIO_TEST_STOP_CMD      = 0x0002,
    TKL_USB_INFO_TEST_CMD        = 0x0003,
    TKL_AUDIO_AEC_TEST_START_CMD = 0x0004,
    TKL_AUDIO_AEC_GET_RESULT     = 0x0005,
} TKL_MFTEST_CMD_E;

#define AEC_TEST_INIT       0x74757961      // tuya
#define AEC_TEST_START      0x74757962
#define AEC_TEST_RUNNING    0x74757962

#define AEC_SAMPLE_RATE   (16000)   // 采样率 16k
#define AEC_FRAME_MS      (20)      // 单帧时长 20ms

// 一整帧字节数 = 采样率 * 时长(s) * 2通道(L/R) * 2字节(16bit)
// = 16000 * 20/1000 * 2 * 2 = 1280 字节 (= 640 个 int16 = L/R 各 320 点)
#define ADC_FRAME_SIZE    (AEC_SAMPLE_RATE * AEC_FRAME_MS / 1000 * 2 * 2)

OPERATE_RET tkl_mftest_ctrl(USHORT_T cmd, UCHAR_T *in, UINT_T inlen, UCHAR_T **out, USHORT_T *outlen);


#ifdef __cplusplus
}
#endif

#endif /* __TKL_MFTEST_H__ */
