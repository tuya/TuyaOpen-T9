/**
* @file tkl_symmetry.c
* @brief TKL symmetric crypto (AES ECB / CBC / GCM) backed by mbedTLS.
*
* Only compiled in when ENABLE_PLATFORM_AES is enabled (see
* vendor/T9/tuyaos/tuyaos_kernel.config). Disabled means the TuyaOS SDK uses its
* own implementation inside libtuyaos.a.
*
* Each entry point emits a "[tkl-sec] <name>" trace line, see tkl_security_ext.h.
*/

/* Must precede the ENABLE_PLATFORM_AES guard below: this header is what defines
 * that macro, so with the guard first the file silently compiles to an empty
 * translation unit and every tkl_aes* symbol goes missing at link. */
#include "tuya_iot_config.h"

#if defined(ENABLE_PLATFORM_AES)

#include "tkl_symmetry.h"
#include "tkl_security_ext.h"
#include "tkl_memory.h"

#include "mbedtls/aes.h"
#include "mbedtls/gcm.h"
#include "mbedtls/cipher.h"

#define TKL_SEC_RET(x) ((x) == 0 ? OPRT_OK : OPRT_COM_ERROR)

OPERATE_RET tkl_aes_create_init(TKL_SYMMETRY_HANDLE *ctx)
{
    mbedtls_aes_context *aes_ctx = NULL;

    TKL_SEC_TRACE("tkl_aes_create_init");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    aes_ctx = (mbedtls_aes_context *)tkl_system_malloc(sizeof(mbedtls_aes_context));
    if (NULL == aes_ctx) {
        return OPRT_MALLOC_FAILED;
    }

    mbedtls_aes_init(aes_ctx);
    *ctx = aes_ctx;

    return OPRT_OK;
}

OPERATE_RET tkl_aes_free(TKL_SYMMETRY_HANDLE ctx)
{
    TKL_SEC_TRACE("tkl_aes_free");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    mbedtls_aes_free((mbedtls_aes_context *)ctx);
    tkl_system_free(ctx);

    return OPRT_OK;
}

OPERATE_RET tkl_aes_setkey_enc(TKL_SYMMETRY_HANDLE ctx, const UINT8_T *key, UINT32_T keybits)
{
    TKL_SEC_TRACE("tkl_aes_setkey_enc");

    if (NULL == ctx || NULL == key) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_aes_setkey_enc((mbedtls_aes_context *)ctx, key, (unsigned int)keybits));
}

OPERATE_RET tkl_aes_setkey_dec(TKL_SYMMETRY_HANDLE ctx, const UINT8_T *key, UINT32_T keybits)
{
    TKL_SEC_TRACE("tkl_aes_setkey_dec");

    if (NULL == ctx || NULL == key) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_aes_setkey_dec((mbedtls_aes_context *)ctx, key, (unsigned int)keybits));
}

OPERATE_RET tkl_aes_crypt_ecb(TKL_SYMMETRY_HANDLE ctx, INT32_T mode, size_t length,
                              const UINT8_T *input, UINT8_T *output)
{
    size_t offset;

    TKL_SEC_TRACE("tkl_aes_crypt_ecb");

    if (NULL == ctx || NULL == input || NULL == output) {
        return OPRT_INVALID_PARM;
    }

    if (0 != (length % 16)) {
        return OPRT_INVALID_PARM;
    }

    /* mbedtls_aes_crypt_ecb() handles exactly one 16-byte block per call while
     * the TKL interface accepts a multi-block buffer, so loop over the blocks. */
    for (offset = 0; offset < length; offset += 16) {
        if (0 != mbedtls_aes_crypt_ecb((mbedtls_aes_context *)ctx, (int)mode,
                                       input + offset, output + offset)) {
            return OPRT_COM_ERROR;
        }
    }

    return OPRT_OK;
}

OPERATE_RET tkl_aes_crypt_cbc(TKL_SYMMETRY_HANDLE ctx, INT32_T mode, size_t length,
                              UINT8_T iv[16], const UINT8_T *input, UINT8_T *output)
{
    TKL_SEC_TRACE("tkl_aes_crypt_cbc");

    if (NULL == ctx || NULL == iv || NULL == input || NULL == output) {
        return OPRT_INVALID_PARM;
    }

    if (0 != (length % 16)) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_aes_crypt_cbc((mbedtls_aes_context *)ctx, (int)mode,
                                             length, iv, input, output));
}

OPERATE_RET tkl_aes_gcm_encode(const UINT8_T *key, UINT32_T key_len,
                               const UINT8_T *nonce, UINT32_T nonce_len,
                               const UINT8_T *ad, UINT32_T ad_len,
                               const UINT8_T *input, UINT32_T input_len,
                               UINT8_T *output, UINT32_T *output_len,
                               UINT8_T *tag, UINT32_T tag_len)
{
    mbedtls_gcm_context *gcm = NULL;
    int ret;

    TKL_SEC_TRACE("tkl_aes_gcm_encode");

    if (NULL == key || 0 == key_len || NULL == nonce || 0 == nonce_len ||
        NULL == input || NULL == output || NULL == tag || 0 == tag_len) {
        return OPRT_INVALID_PARM;
    }

    gcm = (mbedtls_gcm_context *)tkl_system_malloc(sizeof(mbedtls_gcm_context));
    if (NULL == gcm) {
        return OPRT_MALLOC_FAILED;
    }

    mbedtls_gcm_init(gcm);

    ret = mbedtls_gcm_setkey(gcm, MBEDTLS_CIPHER_ID_AES, key, (unsigned int)key_len * 8);
    if (0 == ret) {
        ret = mbedtls_gcm_crypt_and_tag(gcm, MBEDTLS_GCM_ENCRYPT, (size_t)input_len,
                                        nonce, (size_t)nonce_len,
                                        ad, (size_t)ad_len,
                                        input, output,
                                        (size_t)tag_len, tag);
    }

    mbedtls_gcm_free(gcm);
    tkl_system_free(gcm);

    if (0 != ret) {
        return OPRT_COM_ERROR;
    }

    if (NULL != output_len) {
        *output_len = input_len;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_aes_gcm_decode(const UINT8_T *key, UINT32_T key_len,
                               const UINT8_T *nonce, UINT32_T nonce_len,
                               const UINT8_T *ad, UINT32_T ad_len,
                               const UINT8_T *input, UINT32_T input_len,
                               UINT8_T *output, UINT32_T *output_len,
                               UINT8_T *tag, UINT32_T tag_len)
{
    mbedtls_gcm_context *gcm = NULL;
    int ret;

    TKL_SEC_TRACE("tkl_aes_gcm_decode");

    if (NULL == key || 0 == key_len || NULL == nonce || 0 == nonce_len ||
        NULL == input || NULL == output || NULL == tag || 0 == tag_len) {
        return OPRT_INVALID_PARM;
    }

    gcm = (mbedtls_gcm_context *)tkl_system_malloc(sizeof(mbedtls_gcm_context));
    if (NULL == gcm) {
        return OPRT_MALLOC_FAILED;
    }

    mbedtls_gcm_init(gcm);

    ret = mbedtls_gcm_setkey(gcm, MBEDTLS_CIPHER_ID_AES, key, (unsigned int)key_len * 8);
    if (0 == ret) {
        /* mbedtls_gcm_auth_decrypt() already returns a failure code if the tag
         * does not match, no extra verification is required here. */
        ret = mbedtls_gcm_auth_decrypt(gcm, (size_t)input_len,
                                       nonce, (size_t)nonce_len,
                                       ad, (size_t)ad_len,
                                       tag, (size_t)tag_len,
                                       input, output);
    }

    mbedtls_gcm_free(gcm);
    tkl_system_free(gcm);

    if (0 != ret) {
        return OPRT_AUTHENTICATION_FAIL;
    }

    if (NULL != output_len) {
        *output_len = input_len;
    }

    return OPRT_OK;
}

#endif /* ENABLE_PLATFORM_AES */
