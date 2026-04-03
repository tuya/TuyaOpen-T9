/**
 * @file tkl_mftest.c
 * @brief Platform manufacturing-test hooks for Tuya SDK (T5-aligned stub API)
 * @version 1.0
 * @date 2026-04-02
 * @copyright Copyright (c) Tuya Inc.
 */
#include "tkl_mftest.h"
#include "tuya_error_code.h"
#include "tal_memory.h"
#include <string.h>

/**
 * @brief Dispatch manufacturing test command (stub: link symbol for AP; extend for real audio/USB)
 * @param[in] cmd command from TKL_MFTEST_CMD_E
 * @param[in] in optional input payload
 * @param[in] inlen input length in bytes
 * @param[out] out optional heap buffer for JSON/text reply (caller frees with Free())
 * @param[out] outlen output length when @p out is set
 * @return OPRT_OK on success, OPRT_NOT_SUPPORTED if cmd unknown, OPRT_MALLOC_FAILED on alloc failure
 */
OPERATE_RET tkl_mftest_ctrl(USHORT_T cmd, UCHAR_T *in, UINT_T inlen, UCHAR_T **out, USHORT_T *outlen)
{
    (VOID_T)in;
    (VOID_T)inlen;

    if (out != NULL) {
        *out = NULL;
    }
    if (outlen != NULL) {
        *outlen = 0;
    }

    switch (cmd) {
    case TKL_AUDIO_TEST_START_CMD:
    case TKL_AUDIO_TEST_STOP_CMD:
        return OPRT_OK;

    case TKL_USB_INFO_TEST_CMD: {
        static const CHAR_T k_json[] = "{\"ret\":true,\"usb\":\"stub\"}";
        UINT_T len = (UINT_T)sizeof(k_json);
        UCHAR_T *p;

        if (out == NULL || outlen == NULL) {
            return OPRT_INVALID_PARM;
        }
        p = (UCHAR_T *)tal_malloc(len);
        if (p == NULL) {
            return OPRT_MALLOC_FAILED;
        }
        memcpy(p, k_json, len);
        *out = p;
        *outlen = (USHORT_T)(len - 1U);
        return OPRT_OK;
    }

    default:
        return OPRT_NOT_SUPPORTED;
    }
}
