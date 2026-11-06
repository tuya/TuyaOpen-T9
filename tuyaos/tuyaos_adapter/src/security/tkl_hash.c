/**
* @file tkl_hash.c
* @brief TKL hash (SHA-256 / SHA-1 / MD5).
*
* This file is only compiled in when the corresponding ENABLE_PLATFORM_* option
* is enabled (see vendor/T9/tuyaos/tuyaos_kernel.config). When the options are
* disabled the TuyaOS SDK provides its own implementation inside libtuyaos.a.
* Each section is a separate translation unit per algorithm so that the symbol
* set exactly matches what the rebuilt library leaves undefined.
*
* SHA-256 and SHA-1 talk to the Dubhe engine directly when CONFIG_TRUSTENGINE is
* set. The mbedTLS ALT route is not usable here: the lib_tls 3.6.5 headers ship
* no ALT headers, so mbedtls_sha256_context is 108 bytes while
* arm_ce_hash_context_t is 136. Casting the former to the latter places
* finish_flag past the end of the allocation, which makes arm_ce_hash_finish()
* return 0 without writing the digest. Allocating the engine's own context here
* keeps the two layouts apart.
*
* MD5 has no hardware backend and always uses the software mbedTLS from
* libtuyaos.a.
*
* Each entry point emits a "[tkl-sec] <name>" trace line, see tkl_security_ext.h.
*/

/* Must precede the ENABLE_PLATFORM_* guard below: this header is what defines
 * those macros, so with the guard first the file silently compiles to an empty
 * translation unit and every tkl_sha and tkl_md5 symbol goes missing at link. */
#include "tuya_iot_config.h"

#if defined(ENABLE_PLATFORM_SHA256) || defined(ENABLE_PLATFORM_SHA1) || defined(ENABLE_PLATFORM_MD5)

#include <string.h>
#include <stdint.h>

/* Brings in CONFIG_TRUSTENGINE; the Tuya headers below do not include it. */
#include "sdkconfig.h"

#include "tkl_hash.h"
#include "tkl_memory.h"
#include "tkl_security_ext.h"

#include "mbedtls/sha256.h"
#include "mbedtls/sha1.h"
#include "mbedtls/md5.h"

#if CONFIG_TRUSTENGINE
#include "dubhe_hash.h"
#endif

#define TKL_SEC_RET(x) ((x) == 0 ? OPRT_OK : OPRT_COM_ERROR)

/* One line per call carrying what decides whether the digest reaches the
 * caller: the context address (must be DMA-reachable SRAM), the output address
 * (PSRAM or not) and the driver's return code. arm_ce_hash_finish() returns 0
 * without writing anything when its finish_flag is clear, so a "success" alone
 * proves nothing. */
#if CONFIG_TRUSTENGINE && TKL_SECURITY_TRACE
#define TKL_SEC_DBG(fmt, ...) tkl_log_output("[tkl-sec] " fmt "\r\n", ##__VA_ARGS__)
#else
#define TKL_SEC_DBG(fmt, ...) do { } while (0)
#endif

#if defined(ENABLE_PLATFORM_SHA256)

#if CONFIG_TRUSTENGINE

OPERATE_RET tkl_sha256_create_init(TKL_HASH_HANDLE *ctx)
{
    arm_ce_hash_context_t *sha256_ctx = NULL;

    TKL_SEC_TRACE("tkl_sha256_create_init");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    sha256_ctx = (arm_ce_hash_context_t *)tkl_system_malloc(sizeof(arm_ce_hash_context_t));
    if (NULL == sha256_ctx) {
        return OPRT_MALLOC_FAILED;
    }

    /* Zeroed so arm_ce_hash_init() cannot mistake heap garbage for a pending
     * finish_flag. */
    memset(sha256_ctx, 0, sizeof(arm_ce_hash_context_t));
    *ctx = sha256_ctx;
    TKL_SEC_DBG("sha256 ctx=0x%08x size=%u", (unsigned)(uintptr_t)sha256_ctx,
                (unsigned)sizeof(arm_ce_hash_context_t));

    return OPRT_OK;
}

OPERATE_RET tkl_sha256_free(TKL_HASH_HANDLE ctx)
{
    TKL_SEC_TRACE("tkl_sha256_free");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    /* Flushes the engine session if this context is still the active one; it
     * does not release the memory. */
    arm_ce_hash_free((arm_ce_hash_context_t *)ctx);
    tkl_system_free(ctx);

    return OPRT_OK;
}

OPERATE_RET tkl_sha256_starts_ret(TKL_HASH_HANDLE ctx, INT32_T is224)
{
    arm_ce_hash_context_t *sha256_ctx = (arm_ce_hash_context_t *)ctx;
    int ret = 0;

    TKL_SEC_TRACE("tkl_sha256_starts_ret");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    ret = arm_ce_hash_init(sha256_ctx);
    if (0 != ret) {
        return TKL_SEC_RET(ret);
    }

    ret = arm_ce_hash_start(sha256_ctx,
                            ARM_HASH_VECT_DEFAULT,
                            ARM_HASH_IV_IS_ADDR,
                            NULL,
                            0,
                            is224 ? ARM_HASH_MODE_SHA224 : ARM_HASH_MODE_SHA256);
    TKL_SEC_DBG("sha256 start: ctx=0x%08x extra=0x%08x ret=%d fin=%u mode=%u",
                (unsigned)(uintptr_t)sha256_ctx,
                (unsigned)(uintptr_t)sha256_ctx->extra_data,
                ret,
                (unsigned)sha256_ctx->finish_flag,
                (unsigned)sha256_ctx->mode);

    return TKL_SEC_RET(ret);
}

OPERATE_RET tkl_sha256_update_ret(TKL_HASH_HANDLE ctx, const UINT8_T *input, size_t ilen)
{
    int ret = 0;

    TKL_SEC_TRACE("tkl_sha256_update_ret");

    if (NULL == ctx || (NULL == input && 0 != ilen)) {
        return OPRT_INVALID_PARM;
    }

    ret = arm_ce_hash_update((arm_ce_hash_context_t *)ctx, input, ilen);
    TKL_SEC_DBG("sha256 update: in=0x%08x len=%u ret=%d cnt=%u bits=%u fin=%u",
                (unsigned)(uintptr_t)input,
                (unsigned)ilen,
                ret,
                (unsigned)((arm_ce_hash_context_t *)ctx)->count,
                (unsigned)((arm_ce_hash_context_t *)ctx)->total_bits_num,
                (unsigned)((arm_ce_hash_context_t *)ctx)->finish_flag);

    return TKL_SEC_RET(ret);
}

OPERATE_RET tkl_sha256_finish_ret(TKL_HASH_HANDLE ctx, UINT8_T output[32])
{
    int ret = 0;

    TKL_SEC_TRACE("tkl_sha256_finish_ret");

    if (NULL == ctx || NULL == output) {
        return OPRT_INVALID_PARM;
    }

    ret = arm_ce_hash_finish((arm_ce_hash_context_t *)ctx, output);
    TKL_SEC_DBG("sha256 finish: out=0x%08x ret=%d first=0x%02x%02x%02x%02x",
                (unsigned)(uintptr_t)output,
                ret,
                (unsigned)output[0],
                (unsigned)output[1],
                (unsigned)output[2],
                (unsigned)output[3]);

    return TKL_SEC_RET(ret);
}

#else /* CONFIG_TRUSTENGINE */

OPERATE_RET tkl_sha256_create_init(TKL_HASH_HANDLE *ctx)
{
    mbedtls_sha256_context *sha256_ctx = NULL;

    TKL_SEC_TRACE("tkl_sha256_create_init");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    sha256_ctx = (mbedtls_sha256_context *)tkl_system_malloc(sizeof(mbedtls_sha256_context));
    if (NULL == sha256_ctx) {
        return OPRT_MALLOC_FAILED;
    }

    mbedtls_sha256_init(sha256_ctx);
    *ctx = sha256_ctx;

    return OPRT_OK;
}

OPERATE_RET tkl_sha256_free(TKL_HASH_HANDLE ctx)
{
    TKL_SEC_TRACE("tkl_sha256_free");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    mbedtls_sha256_free((mbedtls_sha256_context *)ctx);
    tkl_system_free(ctx);

    return OPRT_OK;
}

OPERATE_RET tkl_sha256_starts_ret(TKL_HASH_HANDLE ctx, INT32_T is224)
{
    TKL_SEC_TRACE("tkl_sha256_starts_ret");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_sha256_starts((mbedtls_sha256_context *)ctx, (int)is224));
}

OPERATE_RET tkl_sha256_update_ret(TKL_HASH_HANDLE ctx, const UINT8_T *input, size_t ilen)
{
    TKL_SEC_TRACE("tkl_sha256_update_ret");

    if (NULL == ctx || (NULL == input && 0 != ilen)) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_sha256_update((mbedtls_sha256_context *)ctx, input, ilen));
}

OPERATE_RET tkl_sha256_finish_ret(TKL_HASH_HANDLE ctx, UINT8_T output[32])
{
    TKL_SEC_TRACE("tkl_sha256_finish_ret");

    if (NULL == ctx || NULL == output) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_sha256_finish((mbedtls_sha256_context *)ctx, output));
}

#endif /* CONFIG_TRUSTENGINE */

#endif /* ENABLE_PLATFORM_SHA256 */

#if defined(ENABLE_PLATFORM_SHA1)

#if CONFIG_TRUSTENGINE

OPERATE_RET tkl_sha1_create_init(TKL_HASH_HANDLE *ctx)
{
    arm_ce_hash_context_t *sha1_ctx = NULL;

    TKL_SEC_TRACE("tkl_sha1_create_init");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    sha1_ctx = (arm_ce_hash_context_t *)tkl_system_malloc(sizeof(arm_ce_hash_context_t));
    if (NULL == sha1_ctx) {
        return OPRT_MALLOC_FAILED;
    }

    memset(sha1_ctx, 0, sizeof(arm_ce_hash_context_t));
    *ctx = sha1_ctx;
    TKL_SEC_DBG("sha1 ctx=0x%08x size=%u", (unsigned)(uintptr_t)sha1_ctx,
                (unsigned)sizeof(arm_ce_hash_context_t));

    return OPRT_OK;
}

OPERATE_RET tkl_sha1_free(TKL_HASH_HANDLE ctx)
{
    TKL_SEC_TRACE("tkl_sha1_free");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    arm_ce_hash_free((arm_ce_hash_context_t *)ctx);
    tkl_system_free(ctx);

    return OPRT_OK;
}

OPERATE_RET tkl_sha1_starts_ret(TKL_HASH_HANDLE ctx)
{
    arm_ce_hash_context_t *sha1_ctx = (arm_ce_hash_context_t *)ctx;
    int ret = 0;

    TKL_SEC_TRACE("tkl_sha1_starts_ret");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    ret = arm_ce_hash_init(sha1_ctx);
    if (0 != ret) {
        return TKL_SEC_RET(ret);
    }

    ret = arm_ce_hash_start(sha1_ctx,
                            ARM_HASH_VECT_DEFAULT,
                            ARM_HASH_IV_IS_ADDR,
                            NULL,
                            0,
                            ARM_HASH_MODE_SHA1);
    TKL_SEC_DBG("sha1 start: ctx=0x%08x extra=0x%08x ret=%d fin=%u mode=%u",
                (unsigned)(uintptr_t)sha1_ctx,
                (unsigned)(uintptr_t)sha1_ctx->extra_data,
                ret,
                (unsigned)sha1_ctx->finish_flag,
                (unsigned)sha1_ctx->mode);

    return TKL_SEC_RET(ret);
}

OPERATE_RET tkl_sha1_update_ret(TKL_HASH_HANDLE ctx, const UINT8_T *input, size_t ilen)
{
    int ret = 0;

    TKL_SEC_TRACE("tkl_sha1_update_ret");

    if (NULL == ctx || (NULL == input && 0 != ilen)) {
        return OPRT_INVALID_PARM;
    }

    ret = arm_ce_hash_update((arm_ce_hash_context_t *)ctx, input, ilen);
    TKL_SEC_DBG("sha1 update: in=0x%08x len=%u ret=%d cnt=%u bits=%u fin=%u",
                (unsigned)(uintptr_t)input,
                (unsigned)ilen,
                ret,
                (unsigned)((arm_ce_hash_context_t *)ctx)->count,
                (unsigned)((arm_ce_hash_context_t *)ctx)->total_bits_num,
                (unsigned)((arm_ce_hash_context_t *)ctx)->finish_flag);

    return TKL_SEC_RET(ret);
}

OPERATE_RET tkl_sha1_finish_ret(TKL_HASH_HANDLE ctx, UINT8_T output[20])
{
    int ret = 0;

    TKL_SEC_TRACE("tkl_sha1_finish_ret");

    if (NULL == ctx || NULL == output) {
        return OPRT_INVALID_PARM;
    }

    ret = arm_ce_hash_finish((arm_ce_hash_context_t *)ctx, output);
    TKL_SEC_DBG("sha1 finish: out=0x%08x ret=%d first=0x%02x%02x%02x%02x",
                (unsigned)(uintptr_t)output,
                ret,
                (unsigned)output[0],
                (unsigned)output[1],
                (unsigned)output[2],
                (unsigned)output[3]);

    return TKL_SEC_RET(ret);
}

#else /* CONFIG_TRUSTENGINE */

OPERATE_RET tkl_sha1_create_init(TKL_HASH_HANDLE *ctx)
{
    mbedtls_sha1_context *sha1_ctx = NULL;

    TKL_SEC_TRACE("tkl_sha1_create_init");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    sha1_ctx = (mbedtls_sha1_context *)tkl_system_malloc(sizeof(mbedtls_sha1_context));
    if (NULL == sha1_ctx) {
        return OPRT_MALLOC_FAILED;
    }

    mbedtls_sha1_init(sha1_ctx);
    *ctx = sha1_ctx;

    return OPRT_OK;
}

OPERATE_RET tkl_sha1_free(TKL_HASH_HANDLE ctx)
{
    TKL_SEC_TRACE("tkl_sha1_free");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    mbedtls_sha1_free((mbedtls_sha1_context *)ctx);
    tkl_system_free(ctx);

    return OPRT_OK;
}

OPERATE_RET tkl_sha1_starts_ret(TKL_HASH_HANDLE ctx)
{
    TKL_SEC_TRACE("tkl_sha1_starts_ret");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_sha1_starts((mbedtls_sha1_context *)ctx));
}

OPERATE_RET tkl_sha1_update_ret(TKL_HASH_HANDLE ctx, const UINT8_T *input, size_t ilen)
{
    TKL_SEC_TRACE("tkl_sha1_update_ret");

    if (NULL == ctx || (NULL == input && 0 != ilen)) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_sha1_update((mbedtls_sha1_context *)ctx, input, ilen));
}

OPERATE_RET tkl_sha1_finish_ret(TKL_HASH_HANDLE ctx, UINT8_T output[20])
{
    TKL_SEC_TRACE("tkl_sha1_finish_ret");

    if (NULL == ctx || NULL == output) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_sha1_finish((mbedtls_sha1_context *)ctx, output));
}

#endif /* CONFIG_TRUSTENGINE */

#endif /* ENABLE_PLATFORM_SHA1 */

#if defined(ENABLE_PLATFORM_MD5)

OPERATE_RET tkl_md5_create_init(TKL_HASH_HANDLE *ctx)
{
    mbedtls_md5_context *md5_ctx = NULL;

    TKL_SEC_TRACE("tkl_md5_create_init");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    md5_ctx = (mbedtls_md5_context *)tkl_system_malloc(sizeof(mbedtls_md5_context));
    if (NULL == md5_ctx) {
        return OPRT_MALLOC_FAILED;
    }

    mbedtls_md5_init(md5_ctx);
    *ctx = md5_ctx;

    return OPRT_OK;
}

OPERATE_RET tkl_md5_free(TKL_HASH_HANDLE ctx)
{
    TKL_SEC_TRACE("tkl_md5_free");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    mbedtls_md5_free((mbedtls_md5_context *)ctx);
    tkl_system_free(ctx);

    return OPRT_OK;
}

OPERATE_RET tkl_md5_starts_ret(TKL_HASH_HANDLE ctx)
{
    TKL_SEC_TRACE("tkl_md5_starts_ret");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_md5_starts((mbedtls_md5_context *)ctx));
}

OPERATE_RET tkl_md5_update_ret(TKL_HASH_HANDLE ctx, const UINT8_T *input, size_t ilen)
{
    TKL_SEC_TRACE("tkl_md5_update_ret");

    if (NULL == ctx || (NULL == input && 0 != ilen)) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_md5_update((mbedtls_md5_context *)ctx, input, ilen));
}

OPERATE_RET tkl_md5_finish_ret(TKL_HASH_HANDLE ctx, UINT8_T output[16])
{
    TKL_SEC_TRACE("tkl_md5_finish_ret");

    if (NULL == ctx || NULL == output) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_md5_finish((mbedtls_md5_context *)ctx, output));
}

#endif /* ENABLE_PLATFORM_MD5 */

#endif /* ENABLE_PLATFORM_SHA256 || ENABLE_PLATFORM_SHA1 || ENABLE_PLATFORM_MD5 */
