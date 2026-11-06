/**
* @file tkl_asymmetrical.c
* @brief TKL asymmetric crypto (RSA).
*
* Only compiled in when ENABLE_PLATFORM_RSA is enabled (see
* vendor/T9/tuyaos/tuyaos_kernel.config).
*
* tkl_rsa_write_key_der(), tkl_rsa_gen_key() and tkl_rsa_create_init() stay on
* the software mbedTLS pk/RSA API: key generation happens once at provisioning
* time and DER encoding is not on any hot path.
*
* tkl_rsa_public() and tkl_rsa_private() (the two operations that actually run
* per handshake / per authentication) are done on the Dubhe (TE200) ACA engine
* when CONFIG_TRUSTENGINE is set, and fall back to software mbedTLS otherwise.
*
* @note No member of libtuyaos.a currently references tkl_rsa_*, the TAL layer
*       calls the mbedtls pk API directly (see tal_asymmetrical.c.o), so this
*       adapter only runs for its own self-test until the SDK is rebuilt with
*       ENABLE_PLATFORM_RSA=y and its TAL layer is routed through TKL.
*
* Each entry point emits a "[tkl-sec] <name>" trace line, see tkl_security_ext.h.
*/

/* Must be defined before any mbedtls header: the RSA context is reached
 * through mbedtls_pk_context::pk_ctx, which is only visible under this macro.
 * It changes member names only, not the layout, so it is ABI safe here. */
#define MBEDTLS_ALLOW_PRIVATE_ACCESS

/* Include this before the ENABLE_PLATFORM_RSA guard: it is what pulls in
 * tuya_iot_config.h, so the macro is still undefined if the guard comes first
 * and this file would silently compile to an empty translation unit. */
#include "tkl_asymmetrical.h"

#if defined(ENABLE_PLATFORM_RSA)

#include "tkl_security_ext.h"
#include "tkl_memory.h"
#include "tkl_system.h"

/* Brings in CONFIG_TRUSTENGINE; the Tuya headers below do not include it. */
#include "sdkconfig.h"

#include <string.h>

#include "mbedtls/rsa.h"
#include "mbedtls/pk.h"
#include "mbedtls/bignum.h"
#include "mbedtls/ecp.h"

#if CONFIG_TRUSTENGINE
#include "tkl_mutex.h"
#include "dubhe_aca.h"
#include "dubhe_aca_hw.h"
#include "dubhe_aca_hw_defs.h"
#endif

#define TKL_SEC_RET(x) ((x) == 0 ? OPRT_OK : OPRT_COM_ERROR)

#define TKL_PK_TO_RSA(handle) ((mbedtls_rsa_context *)((mbedtls_pk_context *)(handle))->pk_ctx)

#if CONFIG_TRUSTENGINE
/* Both implementations are always compiled in; this only selects which one runs.
 * It exists so the self-test can time hardware against software on the same key
 * and message, and cross-check that both produce identical output. */
static int s_aca_rsa_enabled = 1;
#endif

void tkl_rsa_set_hw_enabled(int enable)
{
#if CONFIG_TRUSTENGINE
    s_aca_rsa_enabled = (enable != 0) ? 1 : 0;
#else
    (void)enable;
#endif
}

#if CONFIG_TRUSTENGINE && TKL_SECURITY_TRACE
#define TKL_SEC_DBG(fmt, ...) tkl_log_output("[tkl-sec] " fmt "\r\n", ##__VA_ARGS__)
#else
#define TKL_SEC_DBG(fmt, ...) do { } while (0)
#endif

/**
 * @brief Random source handed to mbedTLS, backed by the TKL system RNG.
 *
 * @note tkl_system_get_random() must be called with a non-zero range, passing 0
 *       makes the underlying implementation fall back to a truncated range.
 */
static int tkl_rsa_rng(void *p_rng, unsigned char *output, size_t output_len)
{
    size_t i;

    (void)p_rng;

    for (i = 0; i < output_len; i++) {
        output[i] = (unsigned char)tkl_system_get_random(256);
    }

    return 0;
}

#if CONFIG_TRUSTENGINE
/* ------------------------------------------------------------------------ *
 *  ACA-backed raw RSA primitive (X = A^E mod N)
 *
 *  This replays the call sequence of dubhe_lib/bignum.c:_mbedtls_mpi_exp_mod()
 *  -- pick len_id -> operands into ACA registers -> Barrett tag NP -> MODEXP --
 *  using only the driver's public API. Unlike the vendor bignum.c it keeps no
 *  per-mpi register cache, so it does not need the "mhandle" field the vendor
 *  added to mbedtls_mpi: the stock mbedtls headers shipped with lib_tls stay
 *  usable and no ABI change is required anywhere.
 *
 *  Registers are allocated and released for the duration of one call. The ACA
 *  is a single engine instance shared by N/NP/T0/T1, so the whole sequence is
 *  serialised by a mutex.
 *
 *  RSADP uses CRT (two half-width exponentiations plus Garner recombination)
 *  whenever the key carries its primes, which holds for keys produced by
 *  tkl_rsa_gen_key() and for keys imported with mbedtls_rsa_import_raw().
 *
 *  @note No blinding: the result is correct, but do not treat this as
 *        side-channel hardened yet.
 * ------------------------------------------------------------------------ */

#define ACA_RSA_INVALID_REG      0xFFU

#define ACA_RSA_MAX3(a, b, c) \
    (((a) > (b) && (a) > (c)) ? (a) : ((b) > (c) ? (b) : (c)))
#define ACA_RSA_BITS_TO_LIMBS(i) ((i) / 32U + (((i) % 32U) != 0U))

#define ACA_RSA_WORD_BITS   DBH_ACA_WORD_SIZE_IN_BITS   /* 64 */
#define ACA_RSA_EXTRA_BITS  ACA_EXTRA_BITS              /* 8  */

static TKL_MUTEX_HANDLE s_aca_rsa_mutex = NULL;

static void aca_rsa_lock(void)
{
    if (NULL == s_aca_rsa_mutex) {
        (void)tkl_mutex_create_init(&s_aca_rsa_mutex);
    }
    if (NULL != s_aca_rsa_mutex) {
        (void)tkl_mutex_lock(s_aca_rsa_mutex);
    }
}

static void aca_rsa_unlock(void)
{
    if (NULL != s_aca_rsa_mutex) {
        (void)tkl_mutex_unlock(s_aca_rsa_mutex);
    }
}

/* Grab one ACA scratch register plus the SRAM backing it (len_id sized). */
static int aca_rsa_alloc_reg(unsigned char len_id, unsigned char *reg)
{
    unsigned int index = ACA_RSA_INVALID_REG;
    int ret;
    int tries = 0;

    do {
        ret = aca_reg_index_alloc(&index);
        if (DBH_ACA_REG_INDEX_ALLOC_FAIL_ERR == ret) {
            aca_reset_len_id();
            tries++;
        }
    } while ((DBH_ACA_REG_INDEX_ALLOC_FAIL_ERR == ret) && (tries <= 1));

    if (0 != ret) {
        return ret;
    }

    tries = 0;
    do {
        ret = aca_reg_sram_alloc_from_reg((unsigned char)index, len_id);
        if (DBH_ACA_REG_SRAM_ALLOC_FAIL_ERR == ret) {
            aca_reset_len_id();
            tries++;
        }
    } while ((DBH_ACA_REG_SRAM_ALLOC_FAIL_ERR == ret) && (tries <= 1));

    if (0 != ret) {
        aca_reg_index_free(index);
        return ret;
    }

    *reg = (unsigned char)index;

    return 0;
}

static void aca_rsa_free_reg(unsigned char reg)
{
    if (ACA_RSA_INVALID_REG != reg) {
        aca_reg_sram_free_from_reg(reg);
        aca_reg_index_free(reg);
    }
}

/* Copy one mbedtls_mpi into a freshly allocated ACA register. */
static int aca_rsa_mpi_to_reg(const mbedtls_mpi *X, unsigned char len_id,
                              unsigned char *reg)
{
    unsigned int full_len = aca_get_full_len(len_id);
    unsigned int wordsize = ACA_RSA_BITS_TO_LIMBS((unsigned int)mbedtls_mpi_bitlen(X));
    unsigned int copy_len = (full_len < wordsize) ? full_len : wordsize;
    unsigned char reg_index = ACA_RSA_INVALID_REG;
    int ret;

    ret = aca_rsa_alloc_reg(len_id, &reg_index);
    if (0 != ret) {
        return ret;
    }

    if (copy_len > 0 && NULL != X->p) {
        aca_copy_data_into_reg(reg_index, len_id, X->p, copy_len);
    }
    if (full_len > copy_len) {
        dubhe_aca_clear_reg_sram(reg_index, copy_len, full_len - copy_len);
    }

    *reg = reg_index;

    return 0;
}

/* Copy an ACA register back into an mbedtls_mpi (full_len 32-bit words). */
static int aca_rsa_reg_to_mpi(mbedtls_mpi *X, unsigned int full_len,
                              unsigned char reg)
{
    int ret;

    ret = mbedtls_mpi_grow(X, full_len);
    if (0 != ret) {
        return ret;
    }

    dubhe_aca_wait_on_aca_done();
    memset(X->p, 0, X->n * sizeof(mbedtls_mpi_uint));
    aca_copy_data_from_reg(X->p, full_len, reg);
    X->s = 1;

    return 0;
}

/* Mirrors mbedtls_get_len_id(): pick the register-length class for the op. */
static int aca_rsa_get_len_id(unsigned char *len_id,
                              unsigned int a_bits,
                              unsigned int e_bits,
                              unsigned int n_bits,
                              unsigned char op_code)
{
    unsigned int op_len_bits;
    unsigned int full_len = 0;
    int ret;
    int tries = 0;

    op_len_bits = ACA_RSA_MAX3(a_bits, e_bits, n_bits);

    ret = aca_get_full_len_by_opcode(op_code, &op_len_bits, &full_len);
    if (0 != ret) {
        return ret;
    }

    do {
        ret = aca_get_len_id(len_id, op_len_bits, full_len);
        if (0 != ret) {
            aca_reset_len_id();
            tries++;
        }
    } while ((0 != ret) && (tries <= 1));

    return ret;
}

/* Mirrors mbedtls_calc_np(): compute the Barrett tag of N into *reg_np.
 * On success the caller owns *reg_np and must release it. */
static int aca_rsa_calc_np(unsigned char len_id, unsigned char reg_n,
                           unsigned int n_bits, unsigned char *reg_np)
{
    unsigned char len_id1 = len_id;
    unsigned char hw_t0 = ACA_RSA_INVALID_REG;
    unsigned char hw_t1 = ACA_RSA_INVALID_REG;
    unsigned char sw_t0 = ACA_RSA_INVALID_REG;
    unsigned char sw_t1 = ACA_RSA_INVALID_REG;
    unsigned char np    = ACA_RSA_INVALID_REG;
    unsigned int tmp_bits;
    int ret = 0;
    int tries;

    if (n_bits <= (3U * ACA_RSA_WORD_BITS + 3U * ACA_RSA_EXTRA_BITS)) {
        if (n_bits > (2U * ACA_RSA_WORD_BITS + 2U * ACA_RSA_EXTRA_BITS)) {
            tmp_bits = 3U * ACA_RSA_WORD_BITS + 3U * ACA_RSA_EXTRA_BITS;
        } else {
            tmp_bits = n_bits + ACA_RSA_WORD_BITS + ACA_RSA_EXTRA_BITS;
        }

        tries = 0;
        do {
            ret = aca_get_len_id(&len_id1, tmp_bits,
                                 CALC_FULL_32BIT_WORDS(tmp_bits));
            if (0 != ret) {
                aca_reset_len_id();
                aca_set_len_id(len_id, n_bits,
                               CALC_FULL_32BIT_WORDS(n_bits + 2U));
                tries++;
            }
        } while ((0 != ret) && (tries <= 1));

        if (0 != ret) {
            return ret;
        }
    }

    ret = aca_rsa_alloc_reg(len_id1, &hw_t0);
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id1, &hw_t1);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id1, &np);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id1, &sw_t0);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id1, &sw_t1);
    }
    if (0 != ret) {
        goto cleanup;
    }

    dubhe_aca_set_n_np_t0_t1_reg(reg_n, np, hw_t0, hw_t1);
    dubhe_aca_calc_np(len_id, len_id1, np, reg_n, sw_t0, sw_t1, n_bits);

    *reg_np = np;
    np = ACA_RSA_INVALID_REG;   /* ownership passed to the caller */

cleanup:
    aca_rsa_free_reg(hw_t0);
    aca_rsa_free_reg(hw_t1);
    aca_rsa_free_reg(sw_t0);
    aca_rsa_free_reg(sw_t1);
    aca_rsa_free_reg(np);

    return ret;
}

/* X = A^E mod N on the ACA engine. N must be odd (RSA moduli are). */
static int aca_rsa_modexp(mbedtls_mpi *X, const mbedtls_mpi *A,
                          const mbedtls_mpi *E, const mbedtls_mpi *N)
{
    unsigned char len_id = 0;
    unsigned char reg_n = ACA_RSA_INVALID_REG;
    unsigned char reg_a = ACA_RSA_INVALID_REG;
    unsigned char reg_e = ACA_RSA_INVALID_REG;
    unsigned char reg_r = ACA_RSA_INVALID_REG;
    unsigned char hw_t0 = ACA_RSA_INVALID_REG;
    unsigned char hw_t1 = ACA_RSA_INVALID_REG;
    unsigned char np    = ACA_RSA_INVALID_REG;
    unsigned int n_bits;
    mbedtls_mpi a_red;
    const mbedtls_mpi *base = A;
    int ret;

    aca_rsa_lock();
    mbedtls_mpi_init(&a_red);

    n_bits = (unsigned int)mbedtls_mpi_bitlen(N);

    /* The engine does not reduce the base for us, and a base wider than the
     * modulus silently produces a non-reduced result. Reduce first, exactly as
     * mbedtls_mpi_exp_mod()'s wrapper does. The full-width path never needed
     * this because there A and N have the same bit length; CRT is the first
     * caller whose base (the ciphertext) is wider than the modulus (P or Q). */
    if (mbedtls_mpi_bitlen(A) > n_bits) {
        ret = mbedtls_mpi_mod_mpi(&a_red, A, N);
        if (0 != ret) {
            goto cleanup;
        }
        base = &a_red;
    }

    ret = aca_rsa_get_len_id(&len_id,
                             (unsigned int)mbedtls_mpi_bitlen(base),
                             (unsigned int)mbedtls_mpi_bitlen(E),
                             n_bits,
                             ACA_OPCODE_ID_MODEXP);
    if (0 != ret) {
        goto cleanup;
    }

    ret = aca_rsa_mpi_to_reg(N, len_id, &reg_n);
    if (0 != ret) {
        goto cleanup;
    }

    ret = aca_rsa_calc_np(len_id, reg_n, n_bits, &np);
    if (0 != ret) {
        goto cleanup;
    }

    ret = aca_rsa_alloc_reg(len_id, &hw_t0);
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id, &hw_t1);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(base, len_id, &reg_a);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(E, len_id, &reg_e);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id, &reg_r);
    }
    if (0 != ret) {
        goto cleanup;
    }

    /* The engine only writes the significant words of the result, while the
     * read-back below fetches full_len words. Clear the destination first,
     * otherwise the high words still hold whatever ran in this register SRAM
     * before (e.g. a wider modulus), and the value comes back inflated. The
     * vendor's _mbedtls_mpi_exp_mod() does the same via mbedtls_mpi_to_reg()
     * on its (empty) output MPI. */
    dubhe_aca_clear_reg_sram(reg_r, 0, aca_get_full_len(len_id));

    dubhe_aca_set_n_np_t0_t1_reg(reg_n, np, hw_t0, hw_t1);
    ret = dubhe_aca_mod_exp(len_id, reg_r, reg_a, reg_e);
    if (0 != ret) {
        goto cleanup;
    }

    ret = aca_rsa_reg_to_mpi(X, aca_get_full_len(len_id), reg_r);

cleanup:
    aca_rsa_free_reg(reg_r);
    aca_rsa_free_reg(reg_e);
    aca_rsa_free_reg(reg_a);
    aca_rsa_free_reg(hw_t1);
    aca_rsa_free_reg(hw_t0);
    aca_rsa_free_reg(np);
    aca_rsa_free_reg(reg_n);
    mbedtls_mpi_free(&a_red);
    aca_rsa_unlock();

    TKL_SEC_DBG("rsa modexp: nbits=%u len_id=%u ret=%d",
                n_bits, (unsigned)len_id, ret);

    return ret;
}

/* True when the key carries everything CRT needs. Keys without primes (public
 * only, or imported without P/Q) fall back to the full-width exponentiation. */
static int aca_rsa_has_crt(const mbedtls_rsa_context *rsa)
{
    return (rsa->MBEDTLS_PRIVATE(P).n  != 0) &&
           (rsa->MBEDTLS_PRIVATE(Q).n  != 0) &&
           (rsa->MBEDTLS_PRIVATE(DP).n != 0) &&
           (rsa->MBEDTLS_PRIVATE(DQ).n != 0) &&
           (rsa->MBEDTLS_PRIVATE(QP).n != 0);
}

/* RSADP with CRT (Garner):
 *   m1 = c^dP mod P,  m2 = c^dQ mod Q,
 *   h  = ((m1 - m2) mod P) * QP mod P,   m = m2 + h * Q.
 * The two exponentiations run on the engine; the recombination is cheap bignum
 * work on half-width operands and stays in software. */
static int aca_rsa_private_crt(mbedtls_rsa_context *rsa,
                               const UINT8_T *input, UINT8_T *output)
{
    const mbedtls_mpi *p  = &rsa->MBEDTLS_PRIVATE(P);
    const mbedtls_mpi *q  = &rsa->MBEDTLS_PRIVATE(Q);
    const mbedtls_mpi *dp = &rsa->MBEDTLS_PRIVATE(DP);
    const mbedtls_mpi *dq = &rsa->MBEDTLS_PRIVATE(DQ);
    const mbedtls_mpi *qp = &rsa->MBEDTLS_PRIVATE(QP);
    mbedtls_mpi c;
    mbedtls_mpi m1;
    mbedtls_mpi m2;
    mbedtls_mpi m2p;
    mbedtls_mpi diff;
    mbedtls_mpi difp;
    mbedtls_mpi prod;
    mbedtls_mpi h;
    mbedtls_mpi hq;
    mbedtls_mpi m;
    size_t len = mbedtls_rsa_get_len(rsa);
    int ret;

    mbedtls_mpi_init(&c);
    mbedtls_mpi_init(&m1);
    mbedtls_mpi_init(&m2);
    mbedtls_mpi_init(&m2p);
    mbedtls_mpi_init(&diff);
    mbedtls_mpi_init(&difp);
    mbedtls_mpi_init(&prod);
    mbedtls_mpi_init(&h);
    mbedtls_mpi_init(&hq);
    mbedtls_mpi_init(&m);

    /* Every destination below is a distinct object: mbedtls' div_mpi (used by
     * mod_mpi) is not documented to tolerate R aliasing A. */
    ret = mbedtls_mpi_read_binary(&c, input, len);
    if (0 == ret) {
        ret = aca_rsa_modexp(&m1, &c, dp, p);
    }
    if (0 == ret) {
        ret = aca_rsa_modexp(&m2, &c, dq, q);
    }
    if (0 == ret) {
        /* Bring m2 into [0, P) so m1 - m2p lies in (-P, P) and one conditional
         * add is enough, whether or not Q > P. */
        ret = mbedtls_mpi_mod_mpi(&m2p, &m2, p);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_sub_mpi(&diff, &m1, &m2p);
    }
    if (0 == ret) {
        if (mbedtls_mpi_cmp_int(&diff, 0) < 0) {
            ret = mbedtls_mpi_add_mpi(&difp, &diff, p);
        } else {
            ret = mbedtls_mpi_copy(&difp, &diff);
        }
    }
    if (0 == ret) {
        ret = mbedtls_mpi_mul_mpi(&prod, &difp, qp);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_mod_mpi(&h, &prod, p);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_mul_mpi(&hq, &h, q);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_add_mpi(&m, &m2, &hq);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_write_binary(&m, output, len);
    }

    TKL_SEC_DBG("crt: m1=%u m2=%u h=%u m=%u ret=%d",
                (unsigned)mbedtls_mpi_bitlen(&m1),
                (unsigned)mbedtls_mpi_bitlen(&m2),
                (unsigned)mbedtls_mpi_bitlen(&h),
                (unsigned)mbedtls_mpi_bitlen(&m), ret);

    mbedtls_mpi_free(&c);
    mbedtls_mpi_free(&m1);
    mbedtls_mpi_free(&m2);
    mbedtls_mpi_free(&m2p);
    mbedtls_mpi_free(&diff);
    mbedtls_mpi_free(&difp);
    mbedtls_mpi_free(&prod);
    mbedtls_mpi_free(&h);
    mbedtls_mpi_free(&hq);
    mbedtls_mpi_free(&m);

    return ret;
}

/* Raw RSAEP / RSADP on the engine: output = input^E mod N (or ^D). */
static OPERATE_RET tkl_rsa_raw_op(TKL_ASYMMETRICAY_HANDLE ctx,
                                  const UINT8_T *input, UINT8_T *output,
                                  int private_op)
{
    mbedtls_rsa_context *rsa;
    const mbedtls_mpi *exp;
    mbedtls_mpi a;
    mbedtls_mpi x;
    size_t len;
    int ret;

    if (NULL == ctx || NULL == input || NULL == output) {
        return OPRT_INVALID_PARM;
    }

    rsa = TKL_PK_TO_RSA(ctx);
    len = mbedtls_rsa_get_len(rsa);
    if (0 == len) {
        return OPRT_COM_ERROR;
    }

    /* RSADP goes through CRT when the key carries its primes; RSAEP and key
     * material without primes use the plain full-width exponentiation. */
    if (private_op && aca_rsa_has_crt(rsa)) {
        ret = aca_rsa_private_crt(rsa, input, output);
        TKL_SEC_DBG("rsa private via ACA/CRT: len=%u ret=%d", (unsigned)len, ret);
        return TKL_SEC_RET(ret);
    }

    exp = private_op ? &rsa->MBEDTLS_PRIVATE(D) : &rsa->MBEDTLS_PRIVATE(E);

    mbedtls_mpi_init(&a);
    mbedtls_mpi_init(&x);

    ret = mbedtls_mpi_read_binary(&a, input, len);
    if (0 == ret) {
        ret = aca_rsa_modexp(&x, &a, exp, &rsa->MBEDTLS_PRIVATE(N));
    }
    if (0 == ret) {
        ret = mbedtls_mpi_write_binary(&x, output, len);
    }

    mbedtls_mpi_free(&a);
    mbedtls_mpi_free(&x);

    TKL_SEC_DBG("rsa %s via ACA: len=%u ret=%d",
                private_op ? "private" : "public", (unsigned)len, ret);

    return TKL_SEC_RET(ret);
}
#endif /* CONFIG_TRUSTENGINE */

OPERATE_RET tkl_rsa_create_init(TKL_ASYMMETRICAY_HANDLE *ctx)
{
    mbedtls_pk_context *pk = NULL;

    TKL_SEC_TRACE("tkl_rsa_create_init");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    pk = (mbedtls_pk_context *)tkl_system_malloc(sizeof(mbedtls_pk_context));
    if (NULL == pk) {
        return OPRT_MALLOC_FAILED;
    }

    mbedtls_pk_init(pk);

    /* Let mbedTLS allocate the RSA context itself, so this adapter does not
     * depend on sizeof(mbedtls_rsa_context). */
    if (0 != mbedtls_pk_setup(pk, mbedtls_pk_info_from_type(MBEDTLS_PK_RSA))) {
        mbedtls_pk_free(pk);
        tkl_system_free(pk);
        return OPRT_COM_ERROR;
    }

    *ctx = pk;

    return OPRT_OK;
}

OPERATE_RET tkl_rsa_free(TKL_ASYMMETRICAY_HANDLE ctx)
{
    TKL_SEC_TRACE("tkl_rsa_free");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    mbedtls_pk_free((mbedtls_pk_context *)ctx);
    tkl_system_free(ctx);

    return OPRT_OK;
}

OPERATE_RET tkl_rsa_gen_key(TKL_ASYMMETRICAY_HANDLE ctx, UINT32_T nbits, INT32_T exponent)
{
    TKL_SEC_TRACE("tkl_rsa_gen_key");

    if (NULL == ctx) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_rsa_gen_key(TKL_PK_TO_RSA(ctx), tkl_rsa_rng, NULL,
                                           (unsigned int)nbits, (int)exponent));
}

OPERATE_RET tkl_rsa_write_key_der(TKL_ASYMMETRICAY_HANDLE ctx, UINT8_T *buf, size_t size)
{
    int ret;

    TKL_SEC_TRACE("tkl_rsa_write_key_der");

    if (NULL == ctx || NULL == buf) {
        return OPRT_INVALID_PARM;
    }

    ret = mbedtls_pk_write_key_der((mbedtls_pk_context *)ctx, buf, size);

    /* mbedTLS writes the DER at the END of buf and reports the number of bytes
     * written, so the key starts at buf + size - ret. */
    return (ret > 0) ? OPRT_OK : OPRT_COM_ERROR;
}

OPERATE_RET tkl_rsa_public(TKL_ASYMMETRICAY_HANDLE ctx, const UINT8_T *input, UINT8_T *output)
{
    TKL_SEC_TRACE("tkl_rsa_public");

#if CONFIG_TRUSTENGINE
    if (s_aca_rsa_enabled) {
        return tkl_rsa_raw_op(ctx, input, output, 0);
    }
#endif

    if (NULL == ctx || NULL == input || NULL == output) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_rsa_public(TKL_PK_TO_RSA(ctx), input, output));
}

OPERATE_RET tkl_rsa_private(TKL_ASYMMETRICAY_HANDLE ctx, const UINT8_T *input, UINT8_T *output)
{
    TKL_SEC_TRACE("tkl_rsa_private");

#if CONFIG_TRUSTENGINE
    if (s_aca_rsa_enabled) {
        return tkl_rsa_raw_op(ctx, input, output, 1);
    }
#endif

    if (NULL == ctx || NULL == input || NULL == output) {
        return OPRT_INVALID_PARM;
    }

    return TKL_SEC_RET(mbedtls_rsa_private(TKL_PK_TO_RSA(ctx), tkl_rsa_rng, NULL, input, output));
}

/* ======================================================================== *
 *  EC point multiplication (Q = k * P) on NIST P-256
 *
 *  The Tuya SDK ships no tkl_ecp/tkl_ecdsa interface, so unlike RSA there is
 *  no header to implement: the entry points below are declared in the private
 *  tkl_security_ext.h and exist for the xsecurity self-test (and as the
 *  reference sequence the lib_tls phase will replay inside mbedtls ecp).
 *
 *  The hardware path replays dubhe_lib/bignum.c:mbedtls_mpi_mul_ecp() -- NAF
 *  recoding of the scalar plus double_point_jj/point_add_ajj in Jacobian
 *  coordinates -- through the driver's public API and the ACA register
 *  plumbing already used by the RSA path above. It does not need the vendor's
 *  "mhandle" field, so the stock mbedtls headers stay usable. A final
 *  jac2affine converts the result to affine coordinates.
 *
 *  The software path is plain mbedtls ecp on the named P-256 curve.
 *
 *  @note No scalar blinding on the hardware path yet, so this is not
 *        side-channel hardened.
 * ======================================================================== */

/* Field prime and curve coefficient a (= p - 3) of NIST P-256, big-endian. */
#if CONFIG_TRUSTENGINE
static const UINT8_T s_ecp_p256_p[32] = {
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static const UINT8_T s_ecp_p256_a[32] = {
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc
};
#endif /* CONFIG_TRUSTENGINE */

/* Software reference: mbedtls ecp on the named curve. */
static int ecp_p256_sw_mul(const UINT8_T *k,
                           const UINT8_T *px, const UINT8_T *py,
                           UINT8_T *qx, UINT8_T *qy)
{
    mbedtls_ecp_group grp;
    mbedtls_ecp_point pt;
    mbedtls_ecp_point res;
    mbedtls_mpi scalar;
    UINT8_T ptbuf[1 + 32 + 32];
    int ret;

    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_point_init(&pt);
    mbedtls_ecp_point_init(&res);
    mbedtls_mpi_init(&scalar);

    ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1);
    if (0 == ret) {
        ret = mbedtls_mpi_read_binary(&scalar, k, 32);
    }
    if (0 == ret) {
        /* Uncompressed point, also validates that P lies on the curve. */
        ptbuf[0] = 0x04;
        memcpy(&ptbuf[1], px, 32);
        memcpy(&ptbuf[33], py, 32);
        ret = mbedtls_ecp_point_read_binary(&grp, &pt, ptbuf, sizeof(ptbuf));
    }
    if (0 == ret) {
        ret = mbedtls_ecp_mul(&grp, &res, &scalar, &pt, tkl_rsa_rng, NULL);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_write_binary(&res.X, qx, 32);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_write_binary(&res.Y, qy, 32);
    }

    mbedtls_ecp_group_free(&grp);
    mbedtls_ecp_point_free(&pt);
    mbedtls_ecp_point_free(&res);
    mbedtls_mpi_free(&scalar);

    return ret;
}

#if CONFIG_TRUSTENGINE
/* ------------------------------------------------------------------------ *
 *  ACA-backed EC point multiplication
 *
 *  Port of dubhe_lib/bignum.c:mbedtls_mpi_mul_ecp() (NAF) and
 *  mbedtls_mpi_jac2affine(), reusing the register plumbing and the mutex of
 *  the RSA path above. Both callers must already hold the ACA lock.
 * ------------------------------------------------------------------------ */

#define ACA_ECP_TMP_REG_NUM       16U   /* MUL_ECP_MAX_TMP_REG_NUM   */
#define ACA_ECP_JAC2AFF_REG_NUM    4U   /* JAC_2_AFFINE_MAX_TMP_REG_NUM */

/* mbedtls_mpi_uint is 32-bit on this target; the NAF builder and the ACA
 * length helpers both work in 32-bit words. */
#define ACA_ECP_BI_L              32U

static int s_aca_ecp_enabled = 1;

/* Carry-propagating add into a little-endian 32-bit vector (port of
 * _mbedtls_add_counter). Returns the carry out of the most significant word. */
static uint32_t aca_ecp_add_counter(uint32_t *cnt, uint32_t val, size_t cnt_size)
{
    uint32_t cur;
    size_t i;

    for (i = 0; i < cnt_size; i++) {
        cur = cnt[i];
        cnt[i] = cur + val;
        val = (cur > cnt[i]) ? 1U : 0U;
    }

    return val;
}

/* v >>= 1 over a little-endian 32-bit vector (port of _mbedtls_divide_by2). */
static void aca_ecp_div2(uint32_t *v, size_t n)
{
    size_t i;

    for (i = 0; i + 1 < n; i++) {
        v[i] = (v[i] >> 1) | ((v[i + 1] & 1U) << 31);
    }

    v[n - 1] >>= 1;
}

/* Recode the scalar in pbuf (psize_bits bits) into non-adjacent form
 * (port of _mbedtls_build_naf). On success *naf points at the most significant
 * NAF digit and *naf_size holds the digit count. */
static int aca_ecp_build_naf(char **naf, size_t *naf_size,
                             mbedtls_mpi_uint *pbuf, size_t psize_bits)
{
    uint32_t *w = (uint32_t *)pbuf;
    size_t wk;
    char *p;

    if ((psize_bits == 0) || ((psize_bits + 2) > *naf_size)) {
        return -1;
    }

    /* The NAF form requires the most significant bit to be set. */
    if ((w[(psize_bits - 1) / ACA_ECP_BI_L] >> ((psize_bits - 1) % ACA_ECP_BI_L)) != 1U) {
        return -1;
    }

    *naf_size = 0;
    p = *naf + psize_bits + 1;      /* one past the last digit, holds '\0' */
    *p = 0;
    wk = CALC_FULL_32BIT_WORDS(psize_bits);
    w[wk] = 0;                      /* zero the extra word the carry uses */

    while (psize_bits) {
        uint32_t carry;
        uint32_t msbit;

        (*naf_size)++;
        --p;
        if (p < *naf) {
            return -1;
        }

        *p = (w[0] & 1U) ? ((w[0] & 2U) ? '-' : '+') : '0';

        msbit = w[wk - 1] >> ((psize_bits - 1) % ACA_ECP_BI_L);
        if (*p == '-') {
            carry = aca_ecp_add_counter(w, 1U, wk);     /* k += 1 */
            if (carry) {
                w[wk] = 1;
                psize_bits++;
            } else if ((w[wk - 1] >> ((psize_bits - 1) % ACA_ECP_BI_L)) > msbit) {
                psize_bits++;
            }
        }

        aca_ecp_div2(w, wk + 1);                        /* k >>= 1 */
        psize_bits--;
        wk = CALC_FULL_32BIT_WORDS(psize_bits);
    }

    *naf = p;                       /* actual NAF vector begin (MS digit) */

    return 0;
}

/* Q = d * G on y^2 = x^3 + a*x + b. A is the curve coefficient a and P the
 * field prime; G/Q are Jacobian triples. The ACA lock must be held. */
static int aca_ecp_mul(mbedtls_mpi *Qx, mbedtls_mpi *Qy, mbedtls_mpi *Qz,
                       const mbedtls_mpi *d,
                       const mbedtls_mpi *Gx, const mbedtls_mpi *Gy,
                       const mbedtls_mpi *Gz,
                       const mbedtls_mpi *A, const mbedtls_mpi *P)
{
    unsigned char len_id = 0;
    unsigned char reg_n  = ACA_RSA_INVALID_REG;
    unsigned char np     = ACA_RSA_INVALID_REG;
    unsigned char hw_t0  = ACA_RSA_INVALID_REG;
    unsigned char hw_t1  = ACA_RSA_INVALID_REG;
    unsigned char reg_ta = ACA_RSA_INVALID_REG;
    unsigned char reg_x1 = ACA_RSA_INVALID_REG;
    unsigned char reg_y1 = ACA_RSA_INVALID_REG;
    unsigned char reg_z1 = ACA_RSA_INVALID_REG;
    unsigned char reg_x  = ACA_RSA_INVALID_REG;
    unsigned char reg_y  = ACA_RSA_INVALID_REG;
    unsigned char reg_z  = ACA_RSA_INVALID_REG;
    unsigned char reg_w;
    unsigned char reg_negy1;
    unsigned char tmp[ACA_ECP_TMP_REG_NUM];
    unsigned int n_bits;
    unsigned int d_bits;
    unsigned int full_len;
    size_t naf_alloc;
    size_t naf_size;
    char *naf_buf = NULL;
    char *naf;
    mbedtls_mpi tmp_d;
    unsigned int i;
    int ret;

    for (i = 0; i < ACA_ECP_TMP_REG_NUM; i++) {
        tmp[i] = ACA_RSA_INVALID_REG;
    }

    mbedtls_mpi_init(&tmp_d);

    n_bits = (unsigned int)mbedtls_mpi_bitlen(P);
    d_bits = (unsigned int)mbedtls_mpi_bitlen(d);

    /* The NAF builder needs one spare limb above the scalar. */
    naf_alloc = (size_t)((d_bits + ACA_ECP_BI_L - 1) / ACA_ECP_BI_L + 1) * ACA_ECP_BI_L;

    ret = mbedtls_mpi_copy(&tmp_d, d);
    if (0 == ret) {
        ret = mbedtls_mpi_grow(&tmp_d, (size_t)((d_bits + ACA_ECP_BI_L - 1) / ACA_ECP_BI_L + 1));
    }
    if (0 != ret) {
        goto cleanup;
    }

    ret = aca_rsa_get_len_id(&len_id, (unsigned int)mbedtls_mpi_bitlen(A), 0U,
                             n_bits, ACA_OPCODE_ID_MODMUL);
    if (0 != ret) {
        goto cleanup;
    }

    ret = aca_rsa_mpi_to_reg(P, len_id, &reg_n);
    if (0 == ret) {
        ret = aca_rsa_calc_np(len_id, reg_n, n_bits, &np);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(A, len_id, &reg_ta);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Gx, len_id, &reg_x1);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Gy, len_id, &reg_y1);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Gz, len_id, &reg_z1);
    }
    /* Q starts at (0,0,0): a zero mpi copy clears the freshly allocated regs. */
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Qx, len_id, &reg_x);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Qy, len_id, &reg_y);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Qz, len_id, &reg_z);
    }
    for (i = 0; (0 == ret) && (i < ACA_ECP_TMP_REG_NUM); i++) {
        ret = aca_rsa_alloc_reg(len_id, &tmp[i]);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id, &hw_t0);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id, &hw_t1);
    }
    if (0 != ret) {
        goto cleanup;
    }

    naf_buf = (char *)tkl_system_malloc(naf_alloc);
    if (NULL == naf_buf) {
        ret = OPRT_MALLOC_FAILED;
        goto cleanup;
    }
    memset(naf_buf, 0, naf_alloc);

    naf = naf_buf;
    naf_size = naf_alloc;
    ret = aca_ecp_build_naf(&naf, &naf_size, tmp_d.p, d_bits);
    if (0 != ret) {
        goto cleanup;
    }

    dubhe_aca_set_n_np_t0_t1_reg(reg_n, np, hw_t0, hw_t1);

    /* The last two temp registers double as the modified-Jacobian w (= a*Z^4,
     * updated by the engine) and as -G.y. */
    reg_negy1 = tmp[ACA_ECP_TMP_REG_NUM - 1];
    reg_w     = tmp[ACA_ECP_TMP_REG_NUM - 2];

    dubhe_aca_copy(len_id, reg_w, reg_ta);          /* w = a (Z == 1) */
    (void)dubhe_aca_sub(len_id, reg_negy1, reg_n, reg_y1);  /* -G.y mod p */

    /* Q = G to start the double-and-add; the leading '+' digit is skipped. */
    dubhe_aca_add_im(len_id, reg_x, reg_x1, 0);
    dubhe_aca_add_im(len_id, reg_y, reg_y1, 0);
    dubhe_aca_add_im(len_id, reg_z, reg_z1, 0);

    if (*naf == '+') {
        naf++;
    }

    while (*naf) {
        if (*naf == '0') {
            dubhe_aca_double_point_jj(len_id, tmp, reg_x, reg_y, reg_z, reg_w, 1);
        } else {
            dubhe_aca_double_point_jj(len_id, tmp, reg_x, reg_y, reg_z, reg_w, 0);
            if (*naf == '+') {
                dubhe_aca_point_add_ajj(len_id, tmp, reg_x, reg_y, reg_z,
                                        reg_x1, reg_y1, reg_ta, reg_w, 1);
            } else {
                dubhe_aca_point_add_ajj(len_id, tmp, reg_x, reg_y, reg_z,
                                        reg_x1, reg_negy1, reg_ta, reg_w, 1);
            }
        }
        naf++;
    }

    full_len = aca_get_full_len(len_id);
    ret = aca_rsa_reg_to_mpi(Qx, full_len, reg_x);
    if (0 == ret) {
        ret = aca_rsa_reg_to_mpi(Qy, full_len, reg_y);
    }
    if (0 == ret) {
        ret = aca_rsa_reg_to_mpi(Qz, full_len, reg_z);
    }

cleanup:
    if (naf_buf != NULL) {
        tkl_system_free(naf_buf);
    }
    aca_rsa_free_reg(reg_z);
    aca_rsa_free_reg(reg_y);
    aca_rsa_free_reg(reg_x);
    aca_rsa_free_reg(reg_z1);
    aca_rsa_free_reg(reg_y1);
    aca_rsa_free_reg(reg_x1);
    aca_rsa_free_reg(reg_ta);
    aca_rsa_free_reg(hw_t1);
    aca_rsa_free_reg(hw_t0);
    for (i = 0; i < ACA_ECP_TMP_REG_NUM; i++) {
        aca_rsa_free_reg(tmp[i]);
    }
    aca_rsa_free_reg(np);
    aca_rsa_free_reg(reg_n);
    mbedtls_mpi_free(&tmp_d);

    return ret;
}

/* (Tx, Ty, Tz) <- affine, i.e. (X/Z^2, Y/Z^3). The ACA lock must be held. */
static int aca_ecp_jac2affine(mbedtls_mpi *Tx, mbedtls_mpi *Ty, mbedtls_mpi *Tz,
                              const mbedtls_mpi *P)
{
    unsigned char len_id = 0;
    unsigned char reg_n  = ACA_RSA_INVALID_REG;
    unsigned char np     = ACA_RSA_INVALID_REG;
    unsigned char hw_t0  = ACA_RSA_INVALID_REG;
    unsigned char hw_t1  = ACA_RSA_INVALID_REG;
    unsigned char reg_x  = ACA_RSA_INVALID_REG;
    unsigned char reg_y  = ACA_RSA_INVALID_REG;
    unsigned char reg_z  = ACA_RSA_INVALID_REG;
    unsigned char tmp[ACA_ECP_JAC2AFF_REG_NUM];
    unsigned int n_bits;
    unsigned int full_len;
    unsigned int i;
    int ret;

    for (i = 0; i < ACA_ECP_JAC2AFF_REG_NUM; i++) {
        tmp[i] = ACA_RSA_INVALID_REG;
    }

    n_bits = (unsigned int)mbedtls_mpi_bitlen(P);

    ret = aca_rsa_get_len_id(&len_id, 0U, 0U, n_bits, ACA_OPCODE_ID_MODINV);
    if (0 != ret) {
        goto cleanup;
    }

    ret = aca_rsa_mpi_to_reg(P, len_id, &reg_n);
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Tx, len_id, &reg_x);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Ty, len_id, &reg_y);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Tz, len_id, &reg_z);
    }
    for (i = 0; (0 == ret) && (i < ACA_ECP_JAC2AFF_REG_NUM); i++) {
        ret = aca_rsa_alloc_reg(len_id, &tmp[i]);
    }
    if (0 == ret) {
        ret = aca_rsa_calc_np(len_id, reg_n, n_bits, &np);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id, &hw_t0);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id, &hw_t1);
    }
    if (0 != ret) {
        goto cleanup;
    }

    dubhe_aca_set_n_np_t0_t1_reg(reg_n, np, hw_t0, hw_t1);
    dubhe_aca_jac2affine(len_id, tmp, reg_x, reg_y, reg_z);

    full_len = aca_get_full_len(len_id);
    ret = aca_rsa_reg_to_mpi(Tx, full_len, reg_x);
    if (0 == ret) {
        ret = aca_rsa_reg_to_mpi(Ty, full_len, reg_y);
    }
    if (0 == ret) {
        ret = aca_rsa_reg_to_mpi(Tz, full_len, reg_z);
    }

cleanup:
    aca_rsa_free_reg(reg_z);
    aca_rsa_free_reg(reg_y);
    aca_rsa_free_reg(reg_x);
    aca_rsa_free_reg(hw_t1);
    aca_rsa_free_reg(hw_t0);
    for (i = 0; i < ACA_ECP_JAC2AFF_REG_NUM; i++) {
        aca_rsa_free_reg(tmp[i]);
    }
    aca_rsa_free_reg(np);
    aca_rsa_free_reg(reg_n);

    return ret;
}

/* R = P1 + P2 in Jacobian coordinates. Port of dubhe_lib:mbedtls_mpi_add_ecp(),
 * restricted to the register plumbing used above. P1 and P2 are Jacobian
 * triples, so this also covers the affine case by passing Z == 1. The ACA lock
 * must be held. A is the curve coefficient and P the field prime. */
#define ACA_ECP_ADD_TMP_REG_NUM 13U   /* point_add touches T[0] .. T[12] */

static int aca_ecp_point_add(mbedtls_mpi *Rx, mbedtls_mpi *Ry, mbedtls_mpi *Rz,
                             const mbedtls_mpi *X1, const mbedtls_mpi *Y1,
                             const mbedtls_mpi *Z1,
                             const mbedtls_mpi *X2, const mbedtls_mpi *Y2,
                             const mbedtls_mpi *Z2,
                             const mbedtls_mpi *A, const mbedtls_mpi *P)
{
    unsigned char len_id = 0;
    unsigned char reg_n  = ACA_RSA_INVALID_REG;
    unsigned char np     = ACA_RSA_INVALID_REG;
    unsigned char hw_t0  = ACA_RSA_INVALID_REG;
    unsigned char hw_t1  = ACA_RSA_INVALID_REG;
    unsigned char reg_x1 = ACA_RSA_INVALID_REG;
    unsigned char reg_y1 = ACA_RSA_INVALID_REG;
    unsigned char reg_z1 = ACA_RSA_INVALID_REG;
    unsigned char reg_x2 = ACA_RSA_INVALID_REG;
    unsigned char reg_y2 = ACA_RSA_INVALID_REG;
    unsigned char reg_z2 = ACA_RSA_INVALID_REG;
    unsigned char reg_x  = ACA_RSA_INVALID_REG;
    unsigned char reg_y  = ACA_RSA_INVALID_REG;
    unsigned char reg_z  = ACA_RSA_INVALID_REG;
    unsigned char tmp[ACA_ECP_ADD_TMP_REG_NUM];
    unsigned int n_bits;
    unsigned int full_len;
    unsigned int i;
    int ret;

    for (i = 0; i < ACA_ECP_ADD_TMP_REG_NUM; i++) {
        tmp[i] = ACA_RSA_INVALID_REG;
    }

    n_bits = (unsigned int)mbedtls_mpi_bitlen(P);

    ret = aca_rsa_get_len_id(&len_id, (unsigned int)mbedtls_mpi_bitlen(A), 0U,
                             n_bits, ACA_OPCODE_ID_MODMUL);
    if (0 != ret) {
        goto cleanup;
    }

    ret = aca_rsa_mpi_to_reg(P, len_id, &reg_n);
    if (0 == ret) {
        ret = aca_rsa_calc_np(len_id, reg_n, n_bits, &np);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(X1, len_id, &reg_x1);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Y1, len_id, &reg_y1);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Z1, len_id, &reg_z1);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(X2, len_id, &reg_x2);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Y2, len_id, &reg_y2);
    }
    if (0 == ret) {
        ret = aca_rsa_mpi_to_reg(Z2, len_id, &reg_z2);
    }
    /* The three result registers are pure outputs of point_add(), so they only
     * need to be allocated, not initialised. */
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id, &reg_x);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id, &reg_y);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id, &reg_z);
    }
    for (i = 0; (0 == ret) && (i < ACA_ECP_ADD_TMP_REG_NUM); i++) {
        ret = aca_rsa_alloc_reg(len_id, &tmp[i]);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id, &hw_t0);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(len_id, &hw_t1);
    }
    if (0 != ret) {
        goto cleanup;
    }

    dubhe_aca_set_n_np_t0_t1_reg(reg_n, np, hw_t0, hw_t1);

    /* The trailing register is a read-only source for the halving step; reg_n
     * is free to use for it because N/NP were already latched into the engine
     * by the call above (this is what the vendor driver does as well). */
    dubhe_aca_point_add(len_id, tmp, reg_n,
                        reg_x, reg_y, reg_z,
                        reg_x1, reg_y1, reg_z1,
                        reg_x2, reg_y2, reg_z2);

    full_len = aca_get_full_len(len_id);
    ret = aca_rsa_reg_to_mpi(Rx, full_len, reg_x);
    if (0 == ret) {
        ret = aca_rsa_reg_to_mpi(Ry, full_len, reg_y);
    }
    if (0 == ret) {
        ret = aca_rsa_reg_to_mpi(Rz, full_len, reg_z);
    }

cleanup:
    aca_rsa_free_reg(reg_z);
    aca_rsa_free_reg(reg_y);
    aca_rsa_free_reg(reg_x);
    aca_rsa_free_reg(reg_z2);
    aca_rsa_free_reg(reg_y2);
    aca_rsa_free_reg(reg_x2);
    aca_rsa_free_reg(reg_z1);
    aca_rsa_free_reg(reg_y1);
    aca_rsa_free_reg(reg_x1);
    aca_rsa_free_reg(hw_t1);
    aca_rsa_free_reg(hw_t0);
    for (i = 0; i < ACA_ECP_ADD_TMP_REG_NUM; i++) {
        aca_rsa_free_reg(tmp[i]);
    }
    aca_rsa_free_reg(np);
    aca_rsa_free_reg(reg_n);

    return ret;
}

/* Q = d * (Gx, Gy) with the base point given in affine form (Z == 1). The
 * result is left in Jacobian coordinates. Qx/Qy/Qz must be freshly initialised
 * (zero) mpis. The ACA lock must be held. */
static int aca_ecp_mul_affine(mbedtls_mpi *Qx, mbedtls_mpi *Qy, mbedtls_mpi *Qz,
                              const mbedtls_mpi *d,
                              const mbedtls_mpi *Gx, const mbedtls_mpi *Gy)
{
    mbedtls_mpi A;
    mbedtls_mpi P;
    mbedtls_mpi Z;
    int ret;

    mbedtls_mpi_init(&A);
    mbedtls_mpi_init(&P);
    mbedtls_mpi_init(&Z);

    ret = mbedtls_mpi_read_binary(&A, s_ecp_p256_a, sizeof(s_ecp_p256_a));
    if (0 == ret) {
        ret = mbedtls_mpi_read_binary(&P, s_ecp_p256_p, sizeof(s_ecp_p256_p));
    }
    if (0 == ret) {
        ret = mbedtls_mpi_lset(&Z, 1);
    }
    if (0 == ret) {
        ret = aca_ecp_mul(Qx, Qy, Qz, d, Gx, Gy, &Z, &A, &P);
    }

    mbedtls_mpi_free(&A);
    mbedtls_mpi_free(&P);
    mbedtls_mpi_free(&Z);

    return ret;
}

/* Q = k * P on P-256 through the engine. All buffers are big-endian, 32 bytes. */
static int aca_ecp_p256_mul(const UINT8_T *k,
                            const UINT8_T *px, const UINT8_T *py,
                            UINT8_T *qx, UINT8_T *qy)
{
    mbedtls_mpi K;
    mbedtls_mpi Gx, Gy, Gz;
    mbedtls_mpi A, N;
    mbedtls_mpi Qx, Qy, Qz;
    int ret;

    mbedtls_mpi_init(&K);
    mbedtls_mpi_init(&Gx);
    mbedtls_mpi_init(&Gy);
    mbedtls_mpi_init(&Gz);
    mbedtls_mpi_init(&A);
    mbedtls_mpi_init(&N);
    mbedtls_mpi_init(&Qx);
    mbedtls_mpi_init(&Qy);
    mbedtls_mpi_init(&Qz);

    aca_rsa_lock();

    ret = mbedtls_mpi_read_binary(&K, k, 32);
    if (0 == ret) {
        ret = mbedtls_mpi_read_binary(&Gx, px, 32);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_read_binary(&Gy, py, 32);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_lset(&Gz, 1);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_read_binary(&A, s_ecp_p256_a, 32);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_read_binary(&N, s_ecp_p256_p, 32);
    }

    if (0 == ret) {
        ret = aca_ecp_mul(&Qx, &Qy, &Qz, &K, &Gx, &Gy, &Gz, &A, &N);
    }
    /* Z == 0 means the result is the point at infinity, i.e. k is a multiple of
     * the group order; the affine conversion below is undefined there. */
    if ((0 == ret) && (mbedtls_mpi_cmp_int(&Qz, 0) == 0)) {
        ret = OPRT_COM_ERROR;
    }
    if (0 == ret) {
        ret = aca_ecp_jac2affine(&Qx, &Qy, &Qz, &N);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_write_binary(&Qx, qx, 32);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_write_binary(&Qy, qy, 32);
    }

    aca_rsa_unlock();

    mbedtls_mpi_free(&K);
    mbedtls_mpi_free(&Gx);
    mbedtls_mpi_free(&Gy);
    mbedtls_mpi_free(&Gz);
    mbedtls_mpi_free(&A);
    mbedtls_mpi_free(&N);
    mbedtls_mpi_free(&Qx);
    mbedtls_mpi_free(&Qy);
    mbedtls_mpi_free(&Qz);

    return ret;
}

/* R = k * P on P-256, result returned in affine form. Non-zero on any operand
 * the engine cannot take. The ACA lock is taken internally. */
static int aca_ecp_mul_hw(mbedtls_ecp_point *R, const mbedtls_mpi *k,
                          const mbedtls_ecp_point *P)
{
    mbedtls_mpi X, Y, Z, A, Pr;
    int ret;

    mbedtls_mpi_init(&X);
    mbedtls_mpi_init(&Y);
    mbedtls_mpi_init(&Z);
    mbedtls_mpi_init(&A);
    mbedtls_mpi_init(&Pr);

    aca_rsa_lock();

    ret = mbedtls_mpi_read_binary(&A, s_ecp_p256_a, sizeof(s_ecp_p256_a));
    if (0 == ret) {
        ret = mbedtls_mpi_read_binary(&Pr, s_ecp_p256_p, sizeof(s_ecp_p256_p));
    }
    if (0 == ret) {
        ret = aca_ecp_mul_affine(&X, &Y, &Z, k, &P->X, &P->Y);
    }
    /* Z == 0 means the result is the point at infinity, which has no affine
     * form; let the software path report it the way mbedtls expects. */
    if ((0 == ret) && (mbedtls_mpi_cmp_int(&Z, 0) == 0)) {
        ret = OPRT_COM_ERROR;
    }
    if (0 == ret) {
        ret = aca_ecp_jac2affine(&X, &Y, &Z, &Pr);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_copy(&R->X, &X);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_copy(&R->Y, &Y);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_lset(&R->Z, 1);
    }

    aca_rsa_unlock();

    mbedtls_mpi_free(&X);
    mbedtls_mpi_free(&Y);
    mbedtls_mpi_free(&Z);
    mbedtls_mpi_free(&A);
    mbedtls_mpi_free(&Pr);

    return ret;
}

/* R = m*P + n*Q on P-256: two NAF scalar multiplications and one Jacobian
 * point addition (the same decomposition dubhe_lib uses in
 * mbedtls_ecp_muladd_restartable). Non-zero on any operand the engine cannot
 * take. The ACA lock is taken internally. */
static int aca_ecp_muladd_hw(mbedtls_ecp_point *R,
                             const mbedtls_mpi *m, const mbedtls_ecp_point *P,
                             const mbedtls_mpi *n, const mbedtls_ecp_point *Q)
{
    mbedtls_mpi X1, Y1, Z1, X2, Y2, Z2, Rx, Ry, Rz, A, Pr;
    int ret;

    mbedtls_mpi_init(&X1);
    mbedtls_mpi_init(&Y1);
    mbedtls_mpi_init(&Z1);
    mbedtls_mpi_init(&X2);
    mbedtls_mpi_init(&Y2);
    mbedtls_mpi_init(&Z2);
    mbedtls_mpi_init(&Rx);
    mbedtls_mpi_init(&Ry);
    mbedtls_mpi_init(&Rz);
    mbedtls_mpi_init(&A);
    mbedtls_mpi_init(&Pr);

    aca_rsa_lock();

    ret = mbedtls_mpi_read_binary(&A, s_ecp_p256_a, sizeof(s_ecp_p256_a));
    if (0 == ret) {
        ret = mbedtls_mpi_read_binary(&Pr, s_ecp_p256_p, sizeof(s_ecp_p256_p));
    }
    if (0 == ret) {
        ret = aca_ecp_mul_affine(&X1, &Y1, &Z1, m, &P->X, &P->Y);
    }
    if ((0 == ret) && (mbedtls_mpi_cmp_int(&Z1, 0) == 0)) {
        ret = OPRT_COM_ERROR;
    }
    if (0 == ret) {
        ret = aca_ecp_mul_affine(&X2, &Y2, &Z2, n, &Q->X, &Q->Y);
    }
    if ((0 == ret) && (mbedtls_mpi_cmp_int(&Z2, 0) == 0)) {
        ret = OPRT_COM_ERROR;
    }
    if (0 == ret) {
        ret = aca_ecp_point_add(&Rx, &Ry, &Rz, &X1, &Y1, &Z1,
                                &X2, &Y2, &Z2, &A, &Pr);
    }
    if ((0 == ret) && (mbedtls_mpi_cmp_int(&Rz, 0) == 0)) {
        ret = OPRT_COM_ERROR;
    }
    if (0 == ret) {
        ret = aca_ecp_jac2affine(&Rx, &Ry, &Rz, &Pr);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_copy(&R->X, &Rx);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_copy(&R->Y, &Ry);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_lset(&R->Z, 1);
    }

    aca_rsa_unlock();

    mbedtls_mpi_free(&X1);
    mbedtls_mpi_free(&Y1);
    mbedtls_mpi_free(&Z1);
    mbedtls_mpi_free(&X2);
    mbedtls_mpi_free(&Y2);
    mbedtls_mpi_free(&Z2);
    mbedtls_mpi_free(&Rx);
    mbedtls_mpi_free(&Ry);
    mbedtls_mpi_free(&Rz);
    mbedtls_mpi_free(&A);
    mbedtls_mpi_free(&Pr);

    return ret;
}

/* ------------------------------------------------------------------------ *
 *  ACA-backed X25519 (Curve25519)
 *
 *  The engine has no dedicated x25519 unit, so the RFC 7748 Montgomery ladder
 *  is written here and only its field arithmetic runs on the ACA -- the same
 *  split the vendor uses in dubhe_lib (software ladder, hardware modular
 *  ops). All ladder state lives in ACA registers for the whole call, so a
 *  ladder step costs one register-to-register engine operation; only the
 *  32-byte input and output are copied in and out.
 *
 *  The ladder needs a conditional swap per step. Instead of swapping register
 *  contents (which would need engine ops for something the ISA cannot express
 *  branch-freely -- there is no XOR/OR/NOT register operation), the *register
 *  indices* are swapped in software. That is branch-free, costs nothing on the
 *  engine, and keeps the ladder constant-time in the scalar.
 *
 *  @note No coordinate randomisation and no scalar blinding, like the P-256
 *        path above. The loop bound itself is fixed (255 iterations).
 * ------------------------------------------------------------------------ */

#define ACA_X25519_BITS        255U
#define ACA_X25519_BYTES       32U
#define ACA_X25519_WORDS       8U    /* 2^255-19 fits in eight 32-bit words */
#define ACA_X25519_MAX_WORDS   16U

/* Constants in little-endian word order, which is also the ACA register
 * layout (and the mbedtls limb order on this little-endian target). */
static const uint32_t s_x25519_p[ACA_X25519_WORDS] = {
    0xFFFFFFEDu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
    0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0x7FFFFFFFu
};
static const uint32_t s_x25519_a24[ACA_X25519_WORDS] = {
    0x0001DB41u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u
};
static const uint32_t s_x25519_one[ACA_X25519_WORDS] = {
    0x00000001u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u
};
static const uint32_t s_x25519_zero[ACA_X25519_WORDS] = {
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u
};

/* Engine state shared by the modular operations below: the length class, the
 * modulus register plus its Barrett tag, and two scratch registers. */
typedef struct {
    unsigned char len_id;
    unsigned char sw0;
    unsigned char sw1;
} aca_x25519_ctx;

/* Branch-free conditional swap of two register indices. */
static void aca_x25519_cswap(unsigned char *a, unsigned char *b,
                             unsigned char flag)
{
    unsigned char mask = (unsigned char)(0U - (unsigned int)(flag & 1U));
    unsigned char d    = (unsigned char)(((*a) ^ (*b)) & mask);

    *a ^= d;
    *b ^= d;
}

/* Allocate a register and copy eight little-endian words into it. */
static int aca_x25519_load(const uint32_t *src, unsigned char len_id,
                           unsigned char *reg)
{
    unsigned int full_len = aca_get_full_len(len_id);
    int ret;

    ret = aca_rsa_alloc_reg(len_id, reg);
    if (0 != ret) {
        return ret;
    }

    aca_copy_data_into_reg(*reg, len_id, src, ACA_X25519_WORDS);
    if (full_len > ACA_X25519_WORDS) {
        /* Freshly allocated SRAM is not guaranteed to be zeroed. */
        dubhe_aca_clear_reg_sram(*reg, ACA_X25519_WORDS,
                                 full_len - ACA_X25519_WORDS);
    }

    return 0;
}

/* Leave the ACA_DONE interrupt masked. The driver unmasks it right before it
 * blocks in dubhe_aca_wait_on_aca_done() and the ISR re-masks it again, so it is
 * normally already masked; doing it explicitly here means the polling loop below
 * can never trigger that ISR. That matters: the ISR posts the very semaphore
 * dubhe_wait_event(DBH_EVENT_ACA_DONE) blocks on, and a post nobody consumed
 * would make the *next* dubhe_aca_wait_on_aca_done() (the P-256 and RSA paths
 * still use it) return before its own operation had finished. */
static void x25519_aca_mask_done_intr(void)
{
    uint32_t reg = DBH_READ_REGISTER(ACA, ACA_INTR_MASK);

    DBH_REG_FLD_SET(ACA_INTR_MASK, ACA_DONE, reg, 0x1);
    DBH_WRITE_REGISTER(ACA, ACA_INTR_MASK, reg);
}

/* Poll the engine status register until the opcode queue is empty and the engine
 * is idle.
 *
 * dubhe_aca_wait_on_aca_done() is unusable in this loop: the driver signals
 * completion through the ACA_DONE interrupt and a counting semaphore, so every
 * drain costs a full RTOS block/wake round trip (~19us measured) while one
 * 255-bit field operation only takes ~1.3us on the engine. That single call
 * dominated the whole ladder; polling straight on the status register removes
 * it and keeps the exact same ordering guarantees. */
static void x25519_aca_sync(void)
{
    uint32_t status;

    do {
        ACA_GET_ACA_DONE_STATUS(status);
    } while (!status);
}

/* The engine operations are queued asynchronously: ACA_EXEC_OPERATION() only
 * writes the opcode and waits for FIFO space. A read-after-write between two
 * dependent register-to-register operations is therefore NOT resolved by the
 * queue, so every operation has to be drained before the next one is issued.
 * The vendor's bignum layer gets this for free because it copies each operand
 * in and out of the register file around every single operation (there is even
 * an explicit ACA_WAIT_ON_ACA_DONE() in the middle of dubhe_aca_point_add()).
 * Draining here costs one status poll per operation and keeps the ladder
 * correct; the ladder is still far faster than the software path. */
static int x25519_mod_mul(const aca_x25519_ctx *ctx, unsigned char r,
                          unsigned char a, unsigned char b)
{
    int ret = dubhe_aca_mod_mul(ctx->len_id, r, a, b, ctx->sw0, ctx->sw1);

    x25519_aca_sync();

    return ret;
}

static int x25519_mod_add(const aca_x25519_ctx *ctx, unsigned char r,
                          unsigned char a, unsigned char b)
{
    int ret = dubhe_aca_mod_add(ctx->len_id, r, a, b);

    x25519_aca_sync();

    return ret;
}

static int x25519_mod_sub(const aca_x25519_ctx *ctx, unsigned char r,
                          unsigned char a, unsigned char b)
{
    int ret = dubhe_aca_mod_sub(ctx->len_id, r, a, b);

    x25519_aca_sync();

    return ret;
}

#define ACA_X25519_STEP(call) \
    do { if (0 == ret) { ret = (call); } } while (0)

/* out = X25519(k, u), all operands 32-byte little-endian. */
static int aca_x25519_mul(const UINT8_T k[ACA_X25519_BYTES],
                          const UINT8_T u[ACA_X25519_BYTES],
                          UINT8_T out[ACA_X25519_BYTES])
{
    aca_x25519_ctx ctx;
    unsigned char r_n     = ACA_RSA_INVALID_REG;
    unsigned char r_np    = ACA_RSA_INVALID_REG;
    unsigned char r_hw_t0 = ACA_RSA_INVALID_REG;
    unsigned char r_hw_t1 = ACA_RSA_INVALID_REG;
    unsigned char r_x1    = ACA_RSA_INVALID_REG;
    unsigned char r_x2    = ACA_RSA_INVALID_REG;
    unsigned char r_x3    = ACA_RSA_INVALID_REG;
    unsigned char r_z2    = ACA_RSA_INVALID_REG;
    unsigned char r_z3    = ACA_RSA_INVALID_REG;
    unsigned char r_a24   = ACA_RSA_INVALID_REG;
    unsigned char r_A     = ACA_RSA_INVALID_REG;
    unsigned char r_AA    = ACA_RSA_INVALID_REG;
    unsigned char r_B     = ACA_RSA_INVALID_REG;
    unsigned char r_BB    = ACA_RSA_INVALID_REG;
    unsigned char r_E     = ACA_RSA_INVALID_REG;
    unsigned char r_C     = ACA_RSA_INVALID_REG;
    unsigned char r_D     = ACA_RSA_INVALID_REG;
    unsigned char r_DA    = ACA_RSA_INVALID_REG;
    unsigned char r_CB    = ACA_RSA_INVALID_REG;
    unsigned char r_S     = ACA_RSA_INVALID_REG;
    unsigned char r_T     = ACA_RSA_INVALID_REG;
    unsigned char r_inv   = ACA_RSA_INVALID_REG;
    uint32_t kbuf[ACA_X25519_WORDS];
    uint32_t ubuf[ACA_X25519_WORDS];
    uint32_t obuf[ACA_X25519_MAX_WORDS];
    unsigned int full_len;
    unsigned int t;
    unsigned char swap = 0;
    unsigned char kt;
    int ret;

    ctx.len_id = 0;
    ctx.sw0    = ACA_RSA_INVALID_REG;
    ctx.sw1    = ACA_RSA_INVALID_REG;

    /* RFC 7748 section 5: clamp the scalar and mask the top bit of the
     * u-coordinate. Both buffers are little-endian, so they can be copied
     * straight into the register word array. */
    memcpy(kbuf, k, sizeof(kbuf));
    memcpy(ubuf, u, sizeof(ubuf));
    ((UINT8_T *)kbuf)[0]  &= 0xF8U;
    ((UINT8_T *)kbuf)[31] &= 0x7FU;
    ((UINT8_T *)kbuf)[31] |= 0x40U;
    ((UINT8_T *)ubuf)[31] &= 0x7FU;

    ret = aca_rsa_get_len_id(&ctx.len_id, ACA_X25519_BITS, 0U,
                             ACA_X25519_BITS, ACA_OPCODE_ID_MODMUL);
    if (0 != ret) {
        return ret;
    }
    full_len = aca_get_full_len(ctx.len_id);
    if ((full_len < ACA_X25519_WORDS) || (full_len > ACA_X25519_MAX_WORDS)) {
        return OPRT_COM_ERROR;
    }

    aca_rsa_lock();
    x25519_aca_mask_done_intr();

    /* Modulus p = 2^255-19 and its Barrett tag. */
    ret = aca_x25519_load(s_x25519_p, ctx.len_id, &r_n);
    if (0 == ret) {
        ret = aca_rsa_calc_np(ctx.len_id, r_n, ACA_X25519_BITS, &r_np);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(ctx.len_id, &r_hw_t0);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(ctx.len_id, &r_hw_t1);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(ctx.len_id, &ctx.sw0);
    }
    if (0 == ret) {
        ret = aca_rsa_alloc_reg(ctx.len_id, &ctx.sw1);
    }
    /* x1 = u, (x2,z2) = (1,0), (x3,z3) = (u,1) */
    if (0 == ret) {
        ret = aca_x25519_load(ubuf, ctx.len_id, &r_x1);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_one, ctx.len_id, &r_x2);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_z2);
    }
    if (0 == ret) {
        ret = aca_x25519_load(ubuf, ctx.len_id, &r_x3);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_one, ctx.len_id, &r_z3);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_a24, ctx.len_id, &r_a24);
    }
    /* The scratch registers must be written before they are first read as an
     * operand. aca_rsa_alloc_reg() only carves out SRAM, and the ACA does not
     * zero it: for a 255-bit modulus the length class spans ten 32-bit words
     * while the values only occupy the low 255 bits, so whatever the previous
     * user of that SRAM left in the upper words would take part in the very
     * first multiply -- making the result depend on the allocation history.
     * Zero them, which is also what the P-256 path gets for free because every
     * one of its registers is filled through aca_rsa_mpi_to_reg(). */
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_A);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_AA);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_B);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_BB);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_E);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_C);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_D);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_DA);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_CB);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_S);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_T);
    }
    if (0 == ret) {
        ret = aca_x25519_load(s_x25519_zero, ctx.len_id, &r_inv);
    }
    if (0 != ret) {
        goto cleanup;
    }

    dubhe_aca_set_n_np_t0_t1_reg(r_n, r_np, r_hw_t0, r_hw_t1);

    for (t = ACA_X25519_BITS; t-- > 0U; ) {
        kt = (unsigned char)((((const UINT8_T *)kbuf)[t >> 3] >> (t & 7U)) & 1U);

        swap ^= kt;
        aca_x25519_cswap(&r_x2, &r_x3, swap);
        aca_x25519_cswap(&r_z2, &r_z3, swap);
        swap = kt;

        /* r_x2/r_x3/r_z2/r_z3 may have been swapped above: always go through
         * the index variables, never cache their values before this point. */
        ACA_X25519_STEP(x25519_mod_add(&ctx, r_A, r_x2, r_z2));
        ACA_X25519_STEP(x25519_mod_mul(&ctx, r_AA, r_A, r_A));
        ACA_X25519_STEP(x25519_mod_sub(&ctx, r_B, r_x2, r_z2));
        ACA_X25519_STEP(x25519_mod_mul(&ctx, r_BB, r_B, r_B));
        ACA_X25519_STEP(x25519_mod_sub(&ctx, r_E, r_AA, r_BB));
        ACA_X25519_STEP(x25519_mod_add(&ctx, r_C, r_x3, r_z3));
        ACA_X25519_STEP(x25519_mod_sub(&ctx, r_D, r_x3, r_z3));
        ACA_X25519_STEP(x25519_mod_mul(&ctx, r_DA, r_D, r_A));
        ACA_X25519_STEP(x25519_mod_mul(&ctx, r_CB, r_C, r_B));
        ACA_X25519_STEP(x25519_mod_add(&ctx, r_S, r_DA, r_CB));
        ACA_X25519_STEP(x25519_mod_mul(&ctx, r_x3, r_S, r_S));
        ACA_X25519_STEP(x25519_mod_sub(&ctx, r_S, r_DA, r_CB));
        ACA_X25519_STEP(x25519_mod_mul(&ctx, r_T, r_S, r_S));
        ACA_X25519_STEP(x25519_mod_mul(&ctx, r_z3, r_x1, r_T));
        ACA_X25519_STEP(x25519_mod_mul(&ctx, r_x2, r_AA, r_BB));
        ACA_X25519_STEP(x25519_mod_mul(&ctx, r_T, r_a24, r_E));
        ACA_X25519_STEP(x25519_mod_add(&ctx, r_T, r_AA, r_T));
        ACA_X25519_STEP(x25519_mod_mul(&ctx, r_z2, r_E, r_T));
    }

    aca_x25519_cswap(&r_x2, &r_x3, swap);
    aca_x25519_cswap(&r_z2, &r_z3, swap);

    /* out = x2 * z2^(p-2) mod p. mod_inv is fed with NP = 0 (only N is
     * meaningful for an inversion), then NP is restored for the multiply. */
    dubhe_aca_set_n_np_t0_t1_reg(r_n, 0, r_hw_t0, r_hw_t1);
    ACA_X25519_STEP(dubhe_aca_mod_inv(ctx.len_id, r_inv, r_z2, ctx.sw0));
    x25519_aca_sync();
    dubhe_aca_set_n_np_t0_t1_reg(r_n, r_np, r_hw_t0, r_hw_t1);
    ACA_X25519_STEP(x25519_mod_mul(&ctx, r_T, r_x2, r_inv));

    if (0 == ret) {
        aca_copy_data_from_reg(obuf, full_len, r_T);
        memcpy(out, obuf, ACA_X25519_BYTES);
    }

cleanup:
    aca_rsa_free_reg(r_inv);
    aca_rsa_free_reg(r_T);
    aca_rsa_free_reg(r_S);
    aca_rsa_free_reg(r_CB);
    aca_rsa_free_reg(r_DA);
    aca_rsa_free_reg(r_D);
    aca_rsa_free_reg(r_C);
    aca_rsa_free_reg(r_E);
    aca_rsa_free_reg(r_BB);
    aca_rsa_free_reg(r_B);
    aca_rsa_free_reg(r_AA);
    aca_rsa_free_reg(r_A);
    aca_rsa_free_reg(r_a24);
    aca_rsa_free_reg(r_z3);
    aca_rsa_free_reg(r_x3);
    aca_rsa_free_reg(r_z2);
    aca_rsa_free_reg(r_x2);
    aca_rsa_free_reg(r_x1);
    aca_rsa_free_reg(ctx.sw1);
    aca_rsa_free_reg(ctx.sw0);
    aca_rsa_free_reg(r_hw_t1);
    aca_rsa_free_reg(r_hw_t0);
    aca_rsa_free_reg(r_np);
    aca_rsa_free_reg(r_n);

    aca_rsa_unlock();

    return ret;
}

#undef ACA_X25519_STEP
#endif /* CONFIG_TRUSTENGINE */

void tkl_ecp_set_hw_enabled(int enable)
{
#if CONFIG_TRUSTENGINE
    s_aca_ecp_enabled = (enable != 0) ? 1 : 0;
#else
    (void)enable;
#endif
}

OPERATE_RET tkl_ecp_p256_mul(const UINT8_T *k,
                             const UINT8_T *px, const UINT8_T *py,
                             UINT8_T *qx, UINT8_T *qy)
{
    TKL_SEC_TRACE("tkl_ecp_p256_mul");

    if (NULL == k || NULL == px || NULL == py || NULL == qx || NULL == qy) {
        return OPRT_INVALID_PARM;
    }

#if CONFIG_TRUSTENGINE
    if (s_aca_ecp_enabled) {
        int ret = aca_ecp_p256_mul(k, px, py, qx, qy);
        TKL_SEC_DBG("ecp p256 mul via ACA: ret=%d", ret);
        return TKL_SEC_RET(ret);
    }
#endif

    return TKL_SEC_RET(ecp_p256_sw_mul(k, px, py, qx, qy));
}

/* Software reference for X25519: mbedtls' own curve25519 scalar
 * multiplication. It is reached through __real_* on purpose -- a plain call to
 * mbedtls_ecp_mul() from this object is redirected by --wrap back into the ACA
 * path, which would make the self-test compare hardware against hardware. */
#if defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
extern int __real_mbedtls_ecp_mul(mbedtls_ecp_group *grp, mbedtls_ecp_point *R,
                                  const mbedtls_mpi *m, const mbedtls_ecp_point *P,
                                  mbedtls_f_rng_t *f_rng, void *p_rng);

static int ecp_x25519_sw_mul(const UINT8_T k[32], const UINT8_T u[32],
                             UINT8_T out[32])
{
    mbedtls_ecp_group grp;
    mbedtls_ecp_point P;
    mbedtls_ecp_point R;
    mbedtls_mpi d;
    UINT8_T kc[32];
    UINT8_T uc[32];
    int ret;

    memcpy(kc, k, sizeof(kc));
    memcpy(uc, u, sizeof(uc));
    kc[0]  &= 0xF8U;
    kc[31] &= 0x7FU;
    kc[31] |= 0x40U;
    uc[31] &= 0x7FU;

    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_point_init(&P);
    mbedtls_ecp_point_init(&R);
    mbedtls_mpi_init(&d);

    ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_CURVE25519);
    if (0 == ret) {
        ret = mbedtls_mpi_read_binary_le(&d, kc, sizeof(kc));
    }
    if (0 == ret) {
        ret = mbedtls_mpi_read_binary_le(&P.X, uc, sizeof(uc));
    }
    if (0 == ret) {
        ret = mbedtls_mpi_lset(&P.Z, 1);
    }
    if (0 == ret) {
        ret = __real_mbedtls_ecp_mul(&grp, &R, &d, &P, tkl_rsa_rng, NULL);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_write_binary_le(&R.X, out, 32);
    }

    mbedtls_ecp_group_free(&grp);
    mbedtls_ecp_point_free(&P);
    mbedtls_ecp_point_free(&R);
    mbedtls_mpi_free(&d);

    return ret;
}
#endif /* MBEDTLS_ECP_DP_CURVE25519_ENABLED */

OPERATE_RET tkl_ecp_x25519_mul(const UINT8_T *k, const UINT8_T *u, UINT8_T *out)
{
    TKL_SEC_TRACE("tkl_ecp_x25519_mul");

#if defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
    if (NULL == k || NULL == u || NULL == out) {
        return OPRT_INVALID_PARM;
    }

#if CONFIG_TRUSTENGINE
    if (s_aca_ecp_enabled) {
        int ret = aca_x25519_mul(k, u, out);
        TKL_SEC_DBG("x25519 mul via ACA: ret=%d", ret);
        if (0 == ret) {
            return OPRT_OK;
        }
        /* The engine refused this input: fall back to software. */
    }
#endif

    return TKL_SEC_RET(ecp_x25519_sw_mul(k, u, out));
#else
    (void)k;
    (void)u;
    (void)out;
    return OPRT_COM_ERROR;
#endif
}

/* ======================================================================== *
 *  Link-time shims (--wrap)
 *
 *  libtuyaos.a carries mbedtls' software RSA/ECC and is linked *before* this
 *  component, so a plain duplicate definition of mbedtls_mpi_exp_mod() would
 *  lose to the archive copy (or raise a multiple-definition error). The
 *  --wrap=... link option (see CMakeLists.txt) redirects every reference --
 *  including the ones made from inside libtuyaos.a -- to the __wrap_ variants
 *  below, while leaving the original definitions in place for __real_ to call.
 *  It is the same mechanism this build already uses for malloc/free/printf.
 *
 *  Reference sites, verified by disassembling libtuyaos.a's rsa.c.o:
 *    mbedtls_rsa_public  -> mbedtls_mpi_exp_mod_unsafe   (encrypt / verify)
 *    mbedtls_rsa_private -> mbedtls_mpi_exp_mod          (decrypt / sign)
 *
 *  Every wrapper below first tries the ACE engine through the primitive this
 *  file already uses for tkl_rsa_public/private (aca_rsa_modexp(), KAT
 *  verified), and falls back to the mbedtls software implementation for any
 *  operand the engine cannot take. The fallback is deliberate: the worst case
 *  is then exactly the behaviour this firmware had before the shims existed.
 *  s_aca_rsa_enabled is the very switch tkl_rsa_set_hw_enabled() drives, so
 *  turning it off makes the software path in the self-test genuinely software.
 *
 *  @note mbedtls_mpi_exp_mod_unsafe() is declared in mbedtls' internal
 *        bignum_internal.h, which lib_tls does not ship; its prototype below
 *        was recovered from the DWARF of the library's own bignum.c.o.
 *        Because of --wrap the call graph is __wrap_ -> __real_ only:
 *        bignum.c.o reaches its static helper directly, so the two wrappers
 *        can never call each other.
 * ======================================================================== */

extern int __real_mbedtls_mpi_exp_mod(mbedtls_mpi *X, const mbedtls_mpi *A,
                                      const mbedtls_mpi *E, const mbedtls_mpi *N,
                                      mbedtls_mpi *prec_RR);
extern int __real_mbedtls_mpi_exp_mod_unsafe(mbedtls_mpi *X, const mbedtls_mpi *A,
                                             const mbedtls_mpi *E, const mbedtls_mpi *N,
                                             mbedtls_mpi *prec_RR);

static unsigned int s_wrap_exp_mod_calls = 0;
static unsigned int s_wrap_exp_mod_unsafe_calls = 0;

#if CONFIG_TRUSTENGINE
/* The engine path needs N odd (dubhe_aca_mod_exp requires it, mbedtls too), N
 * within the engine's 4096-bit limit, and non-negative operands with E >= 1. */
static int aca_rsa_exp_mod_applicable(const mbedtls_mpi *A, const mbedtls_mpi *E,
                                      const mbedtls_mpi *N)
{
    return (mbedtls_mpi_cmp_int(N, 0) > 0) &&
           (mbedtls_mpi_get_bit(N, 0) == 1) &&
           (mbedtls_mpi_bitlen(N) <= 4096U) &&
           (mbedtls_mpi_cmp_int(A, 0) >= 0) &&
           (mbedtls_mpi_cmp_int(E, 1) >= 0);
}
#endif /* CONFIG_TRUSTENGINE */

int __wrap_mbedtls_mpi_exp_mod(mbedtls_mpi *X, const mbedtls_mpi *A,
                               const mbedtls_mpi *E, const mbedtls_mpi *N,
                               mbedtls_mpi *prec_RR)
{
    int ret;

    s_wrap_exp_mod_calls++;

#if CONFIG_TRUSTENGINE
    if (s_aca_rsa_enabled && (NULL != X) && (NULL != A) && (NULL != E) && (NULL != N) &&
        aca_rsa_exp_mod_applicable(A, E, N)) {
        ret = aca_rsa_modexp(X, A, E, N);
        if (0 == ret) {
            TKL_SEC_DBG("wrap exp_mod: #%u nbits=%u ACA",
                        (unsigned)s_wrap_exp_mod_calls, (unsigned)mbedtls_mpi_bitlen(N));
            return 0;
        }
        /* The engine refused these operands: fall through to software. */
    }
#endif

    ret = __real_mbedtls_mpi_exp_mod(X, A, E, N, prec_RR);

    TKL_SEC_DBG("wrap exp_mod: #%u nbits=%u sw ret=%d",
                (unsigned)s_wrap_exp_mod_calls,
                (NULL != N) ? (unsigned)mbedtls_mpi_bitlen(N) : 0U, ret);

    return ret;
}

int __wrap_mbedtls_mpi_exp_mod_unsafe(mbedtls_mpi *X, const mbedtls_mpi *A,
                                      const mbedtls_mpi *E, const mbedtls_mpi *N,
                                      mbedtls_mpi *prec_RR)
{
    int ret;

    s_wrap_exp_mod_unsafe_calls++;

#if CONFIG_TRUSTENGINE
    if (s_aca_rsa_enabled && (NULL != X) && (NULL != A) && (NULL != E) && (NULL != N) &&
        aca_rsa_exp_mod_applicable(A, E, N)) {
        ret = aca_rsa_modexp(X, A, E, N);
        if (0 == ret) {
            TKL_SEC_DBG("wrap exp_mod_unsafe: #%u nbits=%u ACA",
                        (unsigned)s_wrap_exp_mod_unsafe_calls,
                        (unsigned)mbedtls_mpi_bitlen(N));
            return 0;
        }
        /* The engine refused these operands: fall through to software. */
    }
#endif

    ret = __real_mbedtls_mpi_exp_mod_unsafe(X, A, E, N, prec_RR);

    TKL_SEC_DBG("wrap exp_mod_unsafe: #%u nbits=%u sw ret=%d",
                (unsigned)s_wrap_exp_mod_unsafe_calls,
                (NULL != N) ? (unsigned)mbedtls_mpi_bitlen(N) : 0U, ret);

    return ret;
}

/* ---- EC point multiplication -------------------------------------------
 *
 *  Reference sites, verified with nm/objdump on libtuyaos.a:
 *    ecdh.c.o  -> mbedtls_ecp_mul_restartable      (ECDHE gen_public / compute_shared)
 *    ecdsa.c.o -> mbedtls_ecp_muladd_restartable   (ECDSA verify / sign)
 *
 *  Both are the *public* entry points, so both are wrap-able. Wrapping only
 *  mbedtls_ecp_mul_restartable would miss ECDSA: mbedtls_ecp_muladd_restartable
 *  goes through the static mbedtls_ecp_mul_shortcuts(), which calls the static
 *  ecp_mul_restartable_internal() directly -- a local symbol the linker cannot
 *  redirect. Hence the second shim below.
 *
 *  mbedtls_ecp_mul() needs its own wrapper. --wrap redirects only references
 *  that are undefined in the referencing object, and mbedtls_ecp_mul() calls
 *  mbedtls_ecp_mul_restartable() from within the same ecp.c.o -- that inner
 *  call is therefore untouched. Cross-object callers of mbedtls_ecp_mul()
 *  (pk_ecc.c.o inside libtuyaos.a, plus this repository's own self-test) would
 *  otherwise be missed.
 *
 *  Each wrapper runs the engine path first (P-256 only, both operands affine,
 *  non-zero scalars) and falls back to the mbedtls software implementation for
 *  anything else -- x25519 (curve 9), P-384/P-521, degenerate scalars, a key
 *  that is not Z == 1, or any operand the ACA refuses. The worst case is then
 *  exactly the behaviour this firmware had before the shims existed.
 *  s_aca_ecp_enabled is the switch tkl_ecp_set_hw_enabled() drives.
 * ----------------------------------------------------------------------- */

extern int __real_mbedtls_ecp_mul(mbedtls_ecp_group *grp, mbedtls_ecp_point *R,
                                  const mbedtls_mpi *m, const mbedtls_ecp_point *P,
                                  mbedtls_f_rng_t *f_rng, void *p_rng);
extern int __real_mbedtls_ecp_mul_restartable(mbedtls_ecp_group *grp, mbedtls_ecp_point *R,
                                              const mbedtls_mpi *m, const mbedtls_ecp_point *P,
                                              mbedtls_f_rng_t *f_rng, void *p_rng,
                                              mbedtls_ecp_restart_ctx *rs_ctx);
extern int __real_mbedtls_ecp_muladd_restartable(mbedtls_ecp_group *grp, mbedtls_ecp_point *R,
                                                 const mbedtls_mpi *m, const mbedtls_ecp_point *P,
                                                 const mbedtls_mpi *n, const mbedtls_ecp_point *Q,
                                                 mbedtls_ecp_restart_ctx *rs_ctx);

static unsigned int s_wrap_ecp_mul_calls = 0;
static unsigned int s_wrap_ecp_muladd_calls = 0;

#if CONFIG_TRUSTENGINE
/* The engine path below only implements NIST P-256, and its NAF/scalar code
 * expects an affine (Z == 1) base point and a strictly positive scalar. */
static int aca_ecp_wrap_applicable(const mbedtls_ecp_group *grp,
                                   const mbedtls_mpi *k,
                                   const mbedtls_ecp_point *P)
{
    return (NULL != grp) && (NULL != k) && (NULL != P) &&
           (MBEDTLS_ECP_DP_SECP256R1 == grp->id) &&
           (mbedtls_mpi_cmp_int(k, 0) > 0) &&
           (mbedtls_mpi_cmp_int(&P->Z, 1) == 0);
}
#endif /* CONFIG_TRUSTENGINE */

#if CONFIG_TRUSTENGINE && defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
/* Curve25519 has its own ladder (aca_x25519_mul) and its own operand
 * convention: an affine x-only input and a non-zero scalar. Returns 0 only
 * when R was written in full by the engine. */
static int aca_ecp_wrap_x25519(mbedtls_ecp_point *R, const mbedtls_mpi *m,
                               const mbedtls_ecp_point *P)
{
    UINT8_T k[32];
    UINT8_T u[32];
    UINT8_T o[32];
    int ret;

    if ((NULL == R) || (NULL == m) || (NULL == P) ||
        (mbedtls_mpi_cmp_int(m, 0) <= 0) ||
        (mbedtls_mpi_cmp_int(&P->Z, 1) != 0)) {
        return OPRT_COM_ERROR;
    }

    ret = mbedtls_mpi_write_binary_le(m, k, sizeof(k));
    if (0 == ret) {
        ret = mbedtls_mpi_write_binary_le(&P->X, u, sizeof(u));
    }
    if (0 == ret) {
        ret = aca_x25519_mul(k, u, o);
    }
    if (0 == ret) {
        ret = mbedtls_mpi_read_binary_le(&R->X, o, sizeof(o));
    }
    if (0 == ret) {
        /* mbedtls returns Montgomery points normalised: X = u, Z = 1. */
        ret = mbedtls_mpi_lset(&R->Z, 1);
    }

    return ret;
}
#endif /* CONFIG_TRUSTENGINE && MBEDTLS_ECP_DP_CURVE25519_ENABLED */

int __wrap_mbedtls_ecp_mul(mbedtls_ecp_group *grp, mbedtls_ecp_point *R,
                           const mbedtls_mpi *m, const mbedtls_ecp_point *P,
                           mbedtls_f_rng_t *f_rng, void *p_rng)
{
    int ret;

    s_wrap_ecp_mul_calls++;

#if CONFIG_TRUSTENGINE
#if defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
    if (s_aca_ecp_enabled && (NULL != grp) &&
        (MBEDTLS_ECP_DP_CURVE25519 == grp->id)) {
        ret = aca_ecp_wrap_x25519(R, m, P);
        if (0 == ret) {
            TKL_SEC_DBG("wrap ecp_mul(nr): #%u curve=9 x25519 ACA",
                        (unsigned)s_wrap_ecp_mul_calls);
            return ret;
        }
    }
#endif
    if (s_aca_ecp_enabled && (NULL != R) && aca_ecp_wrap_applicable(grp, m, P)) {
        ret = aca_ecp_mul_hw(R, m, P);
        if (0 == ret) {
            TKL_SEC_DBG("wrap ecp_mul(nr): #%u curve=%d bits=%u ACA",
                        (unsigned)s_wrap_ecp_mul_calls, (int)grp->id,
                        (unsigned)mbedtls_mpi_bitlen(m));
            return ret;
        }
        /* The engine refused these operands: fall through to software. */
    }
#endif

    ret = __real_mbedtls_ecp_mul(grp, R, m, P, f_rng, p_rng);

    TKL_SEC_DBG("wrap ecp_mul(nr): #%u curve=%d bits=%u sw ret=%d",
                (unsigned)s_wrap_ecp_mul_calls,
                (NULL != grp) ? (int)grp->id : -1,
                (NULL != m) ? (unsigned)mbedtls_mpi_bitlen(m) : 0U,
                ret);

    return ret;
}

int __wrap_mbedtls_ecp_mul_restartable(mbedtls_ecp_group *grp, mbedtls_ecp_point *R,
                                       const mbedtls_mpi *m, const mbedtls_ecp_point *P,
                                       mbedtls_f_rng_t *f_rng, void *p_rng,
                                       mbedtls_ecp_restart_ctx *rs_ctx)
{
    int ret;

    s_wrap_ecp_mul_calls++;

#if CONFIG_TRUSTENGINE
#if defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
    if (s_aca_ecp_enabled && (NULL != grp) &&
        (MBEDTLS_ECP_DP_CURVE25519 == grp->id)) {
        ret = aca_ecp_wrap_x25519(R, m, P);
        if (0 == ret) {
            TKL_SEC_DBG("wrap ecp_mul: #%u curve=9 x25519 ACA",
                        (unsigned)s_wrap_ecp_mul_calls);
            return ret;
        }
    }
#endif
    if (s_aca_ecp_enabled && (NULL != R) && aca_ecp_wrap_applicable(grp, m, P)) {
        ret = aca_ecp_mul_hw(R, m, P);
        if (0 == ret) {
            TKL_SEC_DBG("wrap ecp_mul: #%u curve=%d bits=%u ACA",
                        (unsigned)s_wrap_ecp_mul_calls, (int)grp->id,
                        (unsigned)mbedtls_mpi_bitlen(m));
            return ret;
        }
        /* The engine refused these operands: fall through to software. */
    }
#endif

    ret = __real_mbedtls_ecp_mul_restartable(grp, R, m, P, f_rng, p_rng, rs_ctx);

    TKL_SEC_DBG("wrap ecp_mul: #%u curve=%d bits=%u rs=%d sw ret=%d",
                (unsigned)s_wrap_ecp_mul_calls,
                (NULL != grp) ? (int)grp->id : -1,
                (NULL != m) ? (unsigned)mbedtls_mpi_bitlen(m) : 0U,
                (NULL != rs_ctx) ? 1 : 0,
                ret);

    return ret;
}

int __wrap_mbedtls_ecp_muladd_restartable(mbedtls_ecp_group *grp, mbedtls_ecp_point *R,
                                          const mbedtls_mpi *m, const mbedtls_ecp_point *P,
                                          const mbedtls_mpi *n, const mbedtls_ecp_point *Q,
                                          mbedtls_ecp_restart_ctx *rs_ctx)
{
    int ret;

    s_wrap_ecp_muladd_calls++;

#if CONFIG_TRUSTENGINE
    if (s_aca_ecp_enabled && (NULL != R) && aca_ecp_wrap_applicable(grp, m, P) &&
        (NULL != n) && (NULL != Q) && (mbedtls_mpi_cmp_int(n, 0) > 0) &&
        (mbedtls_mpi_cmp_int(&Q->Z, 1) == 0)) {
        ret = aca_ecp_muladd_hw(R, m, P, n, Q);
        if (0 == ret) {
            TKL_SEC_DBG("wrap ecp_muladd: #%u curve=%d mbits=%u nbits=%u ACA",
                        (unsigned)s_wrap_ecp_muladd_calls, (int)grp->id,
                        (unsigned)mbedtls_mpi_bitlen(m),
                        (unsigned)mbedtls_mpi_bitlen(n));
            return ret;
        }
        /* The engine refused these operands: fall through to software. */
    }
#endif

    ret = __real_mbedtls_ecp_muladd_restartable(grp, R, m, P, n, Q, rs_ctx);

    TKL_SEC_DBG("wrap ecp_muladd: #%u curve=%d mbits=%u nbits=%u rs=%d sw ret=%d",
                (unsigned)s_wrap_ecp_muladd_calls,
                (NULL != grp) ? (int)grp->id : -1,
                (NULL != m) ? (unsigned)mbedtls_mpi_bitlen(m) : 0U,
                (NULL != n) ? (unsigned)mbedtls_mpi_bitlen(n) : 0U,
                (NULL != rs_ctx) ? 1 : 0,
                ret);

    return ret;
}

#endif /* ENABLE_PLATFORM_RSA */
