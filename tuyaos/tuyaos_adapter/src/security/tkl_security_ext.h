/**
* @file tkl_security_ext.h
* @brief Private header of the TKL security adapter (src/security).
*
* It carries two things the Tuya SDK headers do not provide:
*   1. the prototypes of tkl_aes_gcm_encode/decode;
*   2. the optional entry-point call trace used to prove at runtime that this
*      adapter, and not the copy shipped inside libtuyaos.a, is being called.
*
* @note tkl_aes_gcm_encode/decode are referenced by libtuyaos.a (member
*       tal_symmetry.c.o) but declared in none of the headers shipped in this
*       repository. The prototypes below were recovered from the DWARF of the
*       SDK's own implementation (components/tal_security/src/mbedtls/
*       mbedtls_symmetry.c) and are parameter-compatible with
*       tal_aes_gcm_encode/decode in include/components/tal_security/include/
*       tal_symmetry.h. Do not move them into tuyaos_adapter/include/security/:
*       that directory is Tuya SDK owned (see local.mk).
*/
#ifndef __TKL_SECURITY_EXT_H__
#define __TKL_SECURITY_EXT_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* Call trace switch.
*
* When enabled, every public entry point in the security adapter logs one line
* "[tkl-sec] <function name>". Seeing those lines on the console proves that the
* adapter in this repository is the implementation actually linked and called.
* If instead libtuyaos.a's built-in copy is used, no such line appears.
*
* Set TKL_SECURITY_TRACE to 0 (or define it to 0 in the build flags) to compile
* the trace out; it is not intended to stay enabled for production builds since
* it emits one UART line per crypto call.
*/
#ifndef TKL_SECURITY_TRACE
#define TKL_SECURITY_TRACE 0
#endif

#if TKL_SECURITY_TRACE
#include "tkl_output.h"
#define TKL_SEC_TRACE(fn)   tkl_log_output("[tkl-sec] %s\r\n", fn)
#else
#define TKL_SEC_TRACE(fn)   do { } while (0)
#endif

/**
* @brief AES-GCM encryption with an explicit key
*
* @param[in]  key        encryption key
* @param[in]  key_len    key length in bytes (16/24/32)
* @param[in]  nonce      nonce/IV buffer
* @param[in]  nonce_len  nonce length in bytes (typically 12)
* @param[in]  ad         additional authenticated data, may be NULL
* @param[in]  ad_len     AAD length in bytes
* @param[in]  input      plaintext buffer
* @param[in]  input_len  plaintext length in bytes
* @param[out] output     ciphertext buffer, at least input_len bytes
* @param[out] output_len actual ciphertext length
* @param[out] tag        authentication tag buffer
* @param[in]  tag_len    desired tag length in bytes (typically 16)
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_aes_gcm_encode(const UINT8_T *key, UINT32_T key_len,
                               const UINT8_T *nonce, UINT32_T nonce_len,
                               const UINT8_T *ad, UINT32_T ad_len,
                               const UINT8_T *input, UINT32_T input_len,
                               UINT8_T *output, UINT32_T *output_len,
                               UINT8_T *tag, UINT32_T tag_len);

/**
* @brief AES-GCM decryption with an explicit key
*
* @param[in]  key        decryption key
* @param[in]  key_len    key length in bytes (16/24/32)
* @param[in]  nonce      nonce/IV buffer
* @param[in]  nonce_len  nonce length in bytes (typically 12)
* @param[in]  ad         additional authenticated data, may be NULL
* @param[in]  ad_len     AAD length in bytes
* @param[in]  input      ciphertext buffer
* @param[in]  input_len  ciphertext length in bytes
* @param[out] output     plaintext buffer, at least input_len bytes
* @param[out] output_len actual plaintext length
* @param[in]  tag        authentication tag to verify
* @param[in]  tag_len    tag length in bytes (typically 16)
*
* @return OPRT_OK on success, OPRT_AUTHENTICATION_FAIL if the tag does not match
*/
OPERATE_RET tkl_aes_gcm_decode(const UINT8_T *key, UINT32_T key_len,
                               const UINT8_T *nonce, UINT32_T nonce_len,
                               const UINT8_T *ad, UINT32_T ad_len,
                               const UINT8_T *input, UINT32_T input_len,
                               UINT8_T *output, UINT32_T *output_len,
                               UINT8_T *tag, UINT32_T tag_len);

/**
* @brief Select the software or the hardware RSA path at run time.
*
* The RSA adapter always keeps both implementations: tkl_rsa_public/private use
* the Dubhe ACA engine when CONFIG_TRUSTENGINE is set, and the software mbedTLS
* routines otherwise. This switch forces either one, which is what the
* xsecurity self-test uses to time the two paths on the same key and message.
*
* @param[in] enable  non-zero for the hardware path, 0 for software
*
* @note Only referenced by the self-test. Removing it would leave the software
*       path reachable only when CONFIG_TRUSTENGINE is off.
*/
void tkl_rsa_set_hw_enabled(int enable);

/**
* @brief EC scalar multiplication Q = k * P on NIST P-256.
*
* The Tuya SDK defines no tkl_ecp/tkl_ecdsa interface, so this is a private
* entry point used by the xsecurity self-test. When CONFIG_TRUSTENGINE is set
* the point multiplication runs on the Dubhe ACA engine, otherwise on mbedtls'
* software ecp; both produce identical affine coordinates.
*
* @param[in]  k   scalar, big-endian, 32 bytes
* @param[in]  px  X coordinate of P, big-endian, 32 bytes
* @param[in]  py  Y coordinate of P, big-endian, 32 bytes
* @param[out] qx  X coordinate of k*P, big-endian, 32 bytes
* @param[out] qy  Y coordinate of k*P, big-endian, 32 bytes
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_ecp_p256_mul(const UINT8_T *k,
                             const UINT8_T *px, const UINT8_T *py,
                             UINT8_T *qx, UINT8_T *qy);

/**
* @brief X25519 scalar multiplication (RFC 7748) on Curve25519.
*
* Private entry point used by the xsecurity self-test, same idea as
* tkl_ecp_p256_mul(). The engine has no dedicated x25519 unit, so the
* Montgomery ladder itself is written in tkl_asymmetrical.c while its field
* arithmetic runs on the Dubhe ACA. Values are little-endian, as the RFC
* specifies; the scalar is clamped and the u-coordinate's top bit masked
* internally, so callers pass the raw RFC 7748 inputs.
*
* @param[in]  k    scalar, little-endian, 32 bytes
* @param[in]  u    u-coordinate of the input point, little-endian, 32 bytes
* @param[out] out  u-coordinate of k*U, little-endian, 32 bytes
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_ecp_x25519_mul(const UINT8_T *k, const UINT8_T *u, UINT8_T *out);

/**
* @brief Select the software or the hardware EC path at run time.
*
* Same purpose as tkl_rsa_set_hw_enabled(): the self-test forces either path to
* time them on the same input and cross-check their output.
*
* @param[in] enable  non-zero for the hardware path, 0 for software
*/
void tkl_ecp_set_hw_enabled(int enable);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_SECURITY_EXT_H__ */
