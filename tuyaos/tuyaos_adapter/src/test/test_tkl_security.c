/**
* @file test_tkl_security.c
* @brief CLI self-test for the TKL security backend (xsecurity).
*
* Exercises the TKL interfaces (not mbedTLS directly) so that it proves the
* vendor implementation is the one linked in. For a direct mbedTLS comparison
* use the xmbedtls command in test_mbedtls.c.
*
* Usage: xsecurity <sha|aes|gcm|rsa|all|help>
*/
/* The RSA known-answer test loads a fixed key through
 * mbedtls_pk_context::pk_ctx, which is only reachable under this macro. It
 * renames members only, so the struct layout (and therefore the TKL handle ABI)
 * is unchanged. */
#define MBEDTLS_ALLOW_PRIVATE_ACCESS

#include "cli.h"
#include "cli_tuya_test.h"

#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include <components/log.h>

#include "tkl_hash.h"
#include "tkl_memory.h"
#include "tkl_symmetry.h"
#include "tkl_security_ext.h"
#include "tkl_asymmetrical.h"

#if defined(ENABLE_PLATFORM_RSA)
#include "mbedtls/pk.h"
#include "mbedtls/rsa.h"
#endif

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define SEC_LOG(fmt, ...)       BK_RAW_LOGE("xsecurity", fmt, ##__VA_ARGS__)
#define SEC_TASK_PRIORITY       4
#define SEC_TASK_STACK_SIZE     (1024 * 5)

static int sec_failures = 0;

static void sec_result(const char *name, int ok)
{
    if (ok) {
        SEC_LOG("%-24s PASS\r\n", name);
    } else {
        sec_failures++;
        SEC_LOG("%-24s FAIL\r\n", name);
    }
}

static void sec_dump(const char *tag, const UINT8_T *buf, size_t len)
{
    char line[16 * 2 + 1];
    size_t i;

    for (i = 0; i < len; i++) {
        os_snprintf(&line[(i % 16) * 2], 3, "%02x", (unsigned int)buf[i]);
        if ((i % 16) == 15 || (i + 1) == len) {
            line[((i % 16) + 1) * 2] = '\0';
            SEC_LOG("%s %s\r\n", tag, line);
        }
    }
}

static int hex_nibble(char c, int *out)
{
    if (c >= '0' && c <= '9') {
        *out = c - '0';
    } else if (c >= 'a' && c <= 'f') {
        *out = c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
        *out = c - 'A' + 10;
    } else {
        return 0;
    }

    return 1;
}

static int hex2bin(const char *hex, UINT8_T *out, size_t out_max)
{
    size_t n = 0;
    int hi = 0;
    int lo = 0;

    while (n < out_max && hex[0] != '\0' && hex[1] != '\0') {
        if (!hex_nibble(hex[0], &hi) || !hex_nibble(hex[1], &lo)) {
            break;
        }

        out[n++] = (UINT8_T)((hi << 4) | lo);
        hex += 2;
    }

    return (int)n;
}

/* ------------------------------------------------------------------ hash */

static void sec_test_sha256(void)
{
    static const UINT8_T expect[32] = {
        0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
        0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad
    };
    TKL_HASH_HANDLE ctx = NULL;
    UINT8_T out[32];
    int ok = 0;

    if (tkl_sha256_create_init(&ctx) != OPRT_OK) {
        SEC_LOG("sha256: create_init failed\r\n");
        sec_result("sha256(abc)", 0);
        return;
    }

    ok = (tkl_sha256_starts_ret(ctx, 0) == OPRT_OK) &&
         (tkl_sha256_update_ret(ctx, (const UINT8_T *)"abc", 3) == OPRT_OK) &&
         (tkl_sha256_finish_ret(ctx, out) == OPRT_OK) &&
         (memcmp(out, expect, sizeof(expect)) == 0);

    tkl_sha256_free(ctx);
    sec_result("sha256(abc)", ok);
    if (!ok) {
        sec_dump("got     ", out, sizeof(out));
        sec_dump("expect  ", expect, sizeof(expect));
    }
}

static void sec_test_sha1(void)
{
    static const UINT8_T expect[20] = {
        0xa9, 0x99, 0x3e, 0x36, 0x47, 0x06, 0x81, 0x6a, 0xba, 0x3e,
        0x25, 0x71, 0x78, 0x50, 0xc2, 0x6c, 0x9c, 0xd0, 0xd8, 0x9d
    };
    TKL_HASH_HANDLE ctx = NULL;
    UINT8_T out[20];
    int ok = 0;

    if (tkl_sha1_create_init(&ctx) != OPRT_OK) {
        sec_result("sha1(abc)", 0);
        return;
    }

    ok = (tkl_sha1_starts_ret(ctx) == OPRT_OK) &&
         (tkl_sha1_update_ret(ctx, (const UINT8_T *)"abc", 3) == OPRT_OK) &&
         (tkl_sha1_finish_ret(ctx, out) == OPRT_OK) &&
         (memcmp(out, expect, sizeof(expect)) == 0);

    tkl_sha1_free(ctx);
    sec_result("sha1(abc)", ok);
    if (!ok) {
        sec_dump("got     ", out, sizeof(out));
        sec_dump("expect  ", expect, sizeof(expect));
    }
}

/* 64 x 'a'. Same input as `xsecurity raw 8`: long enough that
 * arm_ce_hash_update() cannot take its memcpy short path, so this exercises the
 * engine's PROCESS read DMA through mbedTLS + ALT instead of the raw API. */
static void sec_test_sha256_long(void)
{
    static const UINT8_T expect[32] = {
        0xff, 0xe0, 0x54, 0xfe, 0x7a, 0xe0, 0xcb, 0x6d, 0xc6, 0x5c, 0x3a, 0xf9, 0xb6, 0x1d, 0x52, 0x09,
        0xf4, 0x39, 0x85, 0x1d, 0xb4, 0x3d, 0x0b, 0xa5, 0x99, 0x73, 0x37, 0xdf, 0x15, 0x46, 0x68, 0xeb
    };
    TKL_HASH_HANDLE ctx = NULL;
    UINT8_T msg[64];
    UINT8_T out[32];
    int ok = 0;

    memset(msg, 'a', sizeof(msg));

    if (tkl_sha256_create_init(&ctx) != OPRT_OK) {
        sec_result("sha256(64*'a')", 0);
        return;
    }

    ok = (tkl_sha256_starts_ret(ctx, 0) == OPRT_OK) &&
         (tkl_sha256_update_ret(ctx, msg, sizeof(msg)) == OPRT_OK) &&
         (tkl_sha256_finish_ret(ctx, out) == OPRT_OK) &&
         (memcmp(out, expect, sizeof(expect)) == 0);

    tkl_sha256_free(ctx);
    sec_result("sha256(64*'a')", ok);
    if (!ok) {
        sec_dump("got     ", out, sizeof(out));
        sec_dump("expect  ", expect, sizeof(expect));
    }
}

static void sec_test_sha1_long(void)
{
    static const UINT8_T expect[20] = {
        0x00, 0x98, 0xba, 0x82, 0x4b, 0x5c, 0x16, 0x42, 0x7b, 0xd7,
        0xa1, 0x12, 0x2a, 0x5a, 0x44, 0x2a, 0x25, 0xec, 0x64, 0x4d
    };
    TKL_HASH_HANDLE ctx = NULL;
    UINT8_T msg[64];
    UINT8_T out[20];
    int ok = 0;

    memset(msg, 'a', sizeof(msg));

    if (tkl_sha1_create_init(&ctx) != OPRT_OK) {
        sec_result("sha1(64*'a')", 0);
        return;
    }

    ok = (tkl_sha1_starts_ret(ctx) == OPRT_OK) &&
         (tkl_sha1_update_ret(ctx, msg, sizeof(msg)) == OPRT_OK) &&
         (tkl_sha1_finish_ret(ctx, out) == OPRT_OK) &&
         (memcmp(out, expect, sizeof(expect)) == 0);

    tkl_sha1_free(ctx);
    sec_result("sha1(64*'a')", ok);
    if (!ok) {
        sec_dump("got     ", out, sizeof(out));
        sec_dump("expect  ", expect, sizeof(expect));
    }
}

static void sec_test_md5(void)
{
    static const UINT8_T expect[16] = {
        0x90, 0x01, 0x50, 0x98, 0x3c, 0xd2, 0x4f, 0xb0,
        0xd6, 0x96, 0x3f, 0x7d, 0x28, 0xe1, 0x7f, 0x72
    };
    TKL_HASH_HANDLE ctx = NULL;
    UINT8_T out[16];
    int ok = 0;

    if (tkl_md5_create_init(&ctx) != OPRT_OK) {
        sec_result("md5(abc)", 0);
        return;
    }

    ok = (tkl_md5_starts_ret(ctx) == OPRT_OK) &&
         (tkl_md5_update_ret(ctx, (const UINT8_T *)"abc", 3) == OPRT_OK) &&
         (tkl_md5_finish_ret(ctx, out) == OPRT_OK) &&
         (memcmp(out, expect, sizeof(expect)) == 0);

    tkl_md5_free(ctx);
    sec_result("md5(abc)", ok);
    if (!ok) {
        sec_dump("got     ", out, sizeof(out));
        sec_dump("expect  ", expect, sizeof(expect));
    }
}

/* ------------------------------------------------------------------- aes */

static void sec_test_aes_ecb(void)
{
    /* FIPS-197 C.1, applied to two identical blocks to cover the multi-block
     * loop in tkl_aes_crypt_ecb(). */
    UINT8_T key[16];
    UINT8_T pt[32];
    UINT8_T expect[32];
    UINT8_T out[32];
    TKL_SYMMETRY_HANDLE ctx = NULL;
    int ok = 0;

    hex2bin("000102030405060708090a0b0c0d0e0f", key, sizeof(key));
    hex2bin("00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff", pt, sizeof(pt));
    hex2bin("69c4e0d86a7b0430d8cdb78070b4c55a69c4e0d86a7b0430d8cdb78070b4c55a", expect, sizeof(expect));

    if (tkl_aes_create_init(&ctx) != OPRT_OK) {
        sec_result("aes-128-ecb", 0);
        return;
    }

    ok = (tkl_aes_setkey_enc(ctx, key, 128) == OPRT_OK) &&
         (tkl_aes_crypt_ecb(ctx, 1 /* encrypt */, sizeof(pt), pt, out) == OPRT_OK) &&
         (memcmp(out, expect, sizeof(expect)) == 0);

    tkl_aes_free(ctx);
    sec_result("aes-128-ecb (2 blocks)", ok);
    if (!ok) {
        sec_dump("got     ", out, sizeof(out));
        sec_dump("expect  ", expect, sizeof(expect));
    }
}

static void sec_test_aes_cbc(void)
{
    /* NIST SP800-38A F.2.1 */
    UINT8_T key[16];
    UINT8_T iv[16];
    UINT8_T pt[16];
    UINT8_T expect[16];
    UINT8_T out[16];
    TKL_SYMMETRY_HANDLE ctx = NULL;
    int ok = 0;

    hex2bin("2b7e151628aed2a6abf7158809cf4f3c", key, sizeof(key));
    hex2bin("000102030405060708090a0b0c0d0e0f", iv, sizeof(iv));
    hex2bin("6bc1bee22e409f96e93d7e117393172a", pt, sizeof(pt));
    hex2bin("7649abac8119b246cee98e9b12e9197d", expect, sizeof(expect));

    if (tkl_aes_create_init(&ctx) != OPRT_OK) {
        sec_result("aes-128-cbc", 0);
        return;
    }

    ok = (tkl_aes_setkey_enc(ctx, key, 128) == OPRT_OK) &&
         (tkl_aes_crypt_cbc(ctx, 1 /* encrypt */, sizeof(pt), iv, pt, out) == OPRT_OK) &&
         (memcmp(out, expect, sizeof(expect)) == 0);

    tkl_aes_free(ctx);
    sec_result("aes-128-cbc", ok);
    if (!ok) {
        sec_dump("got     ", out, sizeof(out));
        sec_dump("expect  ", expect, sizeof(expect));
    }
}

static void sec_test_gcm(void)
{
    /* NIST GCM test case 2: zero key, zero IV, one zero block. */
    UINT8_T key[16];
    UINT8_T iv[12];
    UINT8_T pt[16];
    UINT8_T expect_ct[16];
    UINT8_T expect_tag[16];
    UINT8_T ct[16];
    UINT8_T tag[16];
    UINT8_T back[16];
    UINT32_T out_len = 0;
    int ok;

    memset(key, 0, sizeof(key));
    memset(iv, 0, sizeof(iv));
    memset(pt, 0, sizeof(pt));
    hex2bin("0388dace60b6a392f328c2b971b2fe78", expect_ct, sizeof(expect_ct));
    hex2bin("ab6e47d42cec13bdf53a67b21257bddf", expect_tag, sizeof(expect_tag));

    memset(ct, 0, sizeof(ct));
    memset(tag, 0, sizeof(tag));

    ok = (tkl_aes_gcm_encode(key, sizeof(key), iv, sizeof(iv), NULL, 0,
                             pt, sizeof(pt), ct, &out_len, tag, sizeof(tag)) == OPRT_OK) &&
         (out_len == sizeof(pt)) &&
         (memcmp(ct, expect_ct, sizeof(expect_ct)) == 0);
    sec_result("gcm encode (NIST TC2)", ok);
    if (!ok) {
        sec_dump("ct got  ", ct, sizeof(ct));
        sec_dump("ct exp  ", expect_ct, sizeof(expect_ct));
    }

    sec_result("gcm tag (NIST TC2)", memcmp(tag, expect_tag, sizeof(expect_tag)) == 0);
    if (memcmp(tag, expect_tag, sizeof(expect_tag)) != 0) {
        sec_dump("tag got ", tag, sizeof(tag));
        sec_dump("tag exp ", expect_tag, sizeof(expect_tag));
    }

    /* round trip */
    memset(back, 0, sizeof(back));
    out_len = 0;
    ok = (tkl_aes_gcm_decode(key, sizeof(key), iv, sizeof(iv), NULL, 0,
                             ct, sizeof(ct), back, &out_len, tag, sizeof(tag)) == OPRT_OK) &&
         (out_len == sizeof(pt)) &&
         (memcmp(back, pt, sizeof(pt)) == 0);
    sec_result("gcm decode round trip", ok);

    /* Tampered tag must be rejected. Note the tkl_aes_gcm_decode() that ends up
     * linked is the one inside libtuyaos.a (mbedtls_symmetry.c), not the copy
     * in tuyaos_adapter: it returns OPRT_COM_ERROR for every mbedtls failure
     * instead of the OPRT_AUTHENTICATION_FAIL documented in tkl_security_ext.h,
     * so any non-zero return means the tag was rejected. */
    tag[0] ^= 0xff;
    ok = (tkl_aes_gcm_decode(key, sizeof(key), iv, sizeof(iv), NULL, 0,
                             ct, sizeof(ct), back, &out_len, tag, sizeof(tag))
          != OPRT_OK);
    sec_result("gcm tampered tag rejected", ok);
}

/* ------------------------------------------------------------------- rsa */

/* tkl_asymmetrical.c is an empty TU when ENABLE_PLATFORM_RSA is not set in
 * tuya_iot_config.h (the default for this board), so the test has to be
 * compiled out as well or the link fails on tkl_rsa_*. */
#if defined(ENABLE_PLATFORM_RSA)

/* Known-answer test against the ACA path.
 *
 * The key returned by tkl_rsa_gen_key() is unknown at compile time, so that
 * test alone cannot tell a correct modexp from a wrong one. This one pins a
 * throw-away 1024-bit key (generated once with `openssl genrsa 1024`) plus a
 * fixed plaintext and its expected ciphertext M^E mod N, computed offline with
 * an independent bignum implementation. It is a test vector, not a product key.
 *
 * N   = 0xe9c426c2...f665   E = 65537   D = 0x13a4a2e9...7141
 * M   = 0x01010203...7e7f   C = M^E mod N = 0x500ea7dc...2271
 */
static const char kat_rsa_n[] =
    "e9c426c2ccc7b77ea32a468a62d6e38790172cda691bfa686c812d968d742c44"
    "ac1b2fb282b19ee2a7bf8eb3b384a494d71ac370fcfd603f29194947e73aedc2"
    "9495284b4e09f15daa02f12c2609b44443b786558ff58631d610f94b6e3d0763"
    "a1aa7a0a28868b92c4291b6b0a31a730f4237a66d70a6c5709b7e73ff8c2f665";
static const char kat_rsa_e[] = "010001";
static const char kat_rsa_d[] =
    "13a4a2e92d91feae4a81abe9c7026d9e1f9282f75b6bb96ffae623f189b3606"
    "0e8e163f1a91fc9cd1bf171c951e56c7b01544366042c3a61344fe6a37f1fa98"
    "8ce15dab53ea226597a973e1b5aacd4459df0cbf844d774c47cad2d8d2546cb1"
    "840a434298dc8f31569cac721338f547bd825d0b122f75b952a03d18ab4237141";
static const char kat_rsa_p[] =
    "fd8e2bb6b74e688e02d34a5595de4998b0a86241be606a9762e305f9aff82204"
    "527ba78b9105197d580998f54e52a6d1f9b6e7ba2087c74601e79dd67c790c31";
static const char kat_rsa_q[] =
    "ec0523051dc1c3ae17be0da3586c8be31d9d943eeb94c10cf8b42552d20754b4"
    "701ed70fb976b6a253fce1867bbde6b3d53454fd56073ea20a3b519da871a475";
static const char kat_rsa_m[] =
    "010102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f"
    "202122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f"
    "404142434445464748494a4b4c4d4e4f505152535455565758595a5b5c5d5e5f"
    "606162636465666768696a6b6c6d6e6f707172737475767778797a7b7c7d7e7f";
static const char kat_rsa_c[] =
    "500ea7dcb07a9067368293db1d83bd8ea8d59d94ad8ecbb021282d1fdeee292c"
    "d7ea3128096e9252026dd15682af9ca96d1077a30579edb2e0889490a22ea1c0"
    "0cecc88d26be2beb9d9d8f974853d6683206adb817c4aee1b3b2869750e2e853"
    "b7932da883de42d1eced1975ec78f23017711f20a2dbf9ef18e585fa4bbb2271";

static void sec_test_rsa_kat(void)
{
    UINT8_T n[128];
    UINT8_T e[3];
    UINT8_T d[128];
    UINT8_T p[64];
    UINT8_T q[64];
    UINT8_T m[128];
    UINT8_T c[128];
    UINT8_T got[128];
    TKL_ASYMMETRICAY_HANDLE ctx = NULL;
    mbedtls_rsa_context *rsa;
    int ok;

    if (tkl_rsa_create_init(&ctx) != OPRT_OK) {
        sec_result("rsa kat create_init", 0);
        return;
    }

    rsa = (mbedtls_rsa_context *)((mbedtls_pk_context *)ctx)->pk_ctx;
    if (NULL == rsa) {
        sec_result("rsa kat ctx", 0);
        tkl_rsa_free(ctx);
        return;
    }

    hex2bin(kat_rsa_n, n, sizeof(n));
    hex2bin(kat_rsa_e, e, sizeof(e));
    hex2bin(kat_rsa_d, d, sizeof(d));
    hex2bin(kat_rsa_p, p, sizeof(p));
    hex2bin(kat_rsa_q, q, sizeof(q));
    hex2bin(kat_rsa_m, m, sizeof(m));
    hex2bin(kat_rsa_c, c, sizeof(c));

    if (0 != mbedtls_rsa_import_raw(rsa, n, sizeof(n), p, sizeof(p), q, sizeof(q),
                                    d, sizeof(d), e, sizeof(e))) {
        sec_result("rsa kat import key", 0);
        tkl_rsa_free(ctx);
        return;
    }
    sec_result("rsa kat import key", 1);

    memset(got, 0, sizeof(got));
    ok = (tkl_rsa_public(ctx, m, got) == OPRT_OK) && (memcmp(got, c, sizeof(c)) == 0);
    sec_result("rsa kat public(M)=C", ok);
    if (!ok) {
        sec_dump("got     ", got, sizeof(got));
        sec_dump("expect  ", c, sizeof(c));
    }

    memset(got, 0, sizeof(got));
    ok = (tkl_rsa_private(ctx, c, got) == OPRT_OK) && (memcmp(got, m, sizeof(m)) == 0);
    sec_result("rsa kat private(C)=M", ok);
    if (!ok) {
        sec_dump("got     ", got, sizeof(got));
        sec_dump("expect  ", m, sizeof(m));
    }

    tkl_rsa_free(ctx);
}

/* Runs the same key and message through both paths so the reported times are
 * comparable, and cross-checks that software and hardware agree byte for byte.
 * Times come from rtos_get_time() (ms); the per-op figure is derived from the
 * loop count so the timer granularity does not dominate a single fast call. */
#define RSA_BENCH_PUB_ITERS 8
#define RSA_BENCH_PRV_ITERS 3

static void sec_test_rsa_bench(TKL_ASYMMETRICAY_HANDLE ctx, const UINT8_T *msg)
{
    static UINT8_T sw_c[128];
    static UINT8_T sw_m[128];
    static UINT8_T hw_c[128];
    static UINT8_T hw_m[128];
    uint32_t sw_pub_ms, sw_prv_ms, hw_pub_ms, hw_prv_ms;
    uint32_t t0, t1;
    int i;
    int ok;

    /* Software reference: mbedTLS, with blinding via tkl_rsa_rng(). */
    tkl_rsa_set_hw_enabled(0);
    t0 = rtos_get_time();
    for (i = 0; i < RSA_BENCH_PUB_ITERS; i++) {
        (void)tkl_rsa_public(ctx, msg, sw_c);
    }
    t1 = rtos_get_time();
    sw_pub_ms = t1 - t0;

    t0 = rtos_get_time();
    for (i = 0; i < RSA_BENCH_PRV_ITERS; i++) {
        (void)tkl_rsa_private(ctx, sw_c, sw_m);
    }
    t1 = rtos_get_time();
    sw_prv_ms = t1 - t0;

    /* Hardware: Dubhe ACA. */
    tkl_rsa_set_hw_enabled(1);
    t0 = rtos_get_time();
    for (i = 0; i < RSA_BENCH_PUB_ITERS; i++) {
        (void)tkl_rsa_public(ctx, msg, hw_c);
    }
    t1 = rtos_get_time();
    hw_pub_ms = t1 - t0;

    t0 = rtos_get_time();
    for (i = 0; i < RSA_BENCH_PRV_ITERS; i++) {
        (void)tkl_rsa_private(ctx, hw_c, hw_m);
    }
    t1 = rtos_get_time();
    hw_prv_ms = t1 - t0;

    SEC_LOG("rsa public  x%d: sw=%ums (%uus/op) | hw=%ums (%uus/op)\r\n",
            RSA_BENCH_PUB_ITERS,
            (unsigned)sw_pub_ms,
            (unsigned)(sw_pub_ms * 1000U / RSA_BENCH_PUB_ITERS),
            (unsigned)hw_pub_ms,
            (unsigned)(hw_pub_ms * 1000U / RSA_BENCH_PUB_ITERS));
    SEC_LOG("rsa private x%d: sw=%ums (%uus/op) | hw=%ums (%uus/op)\r\n",
            RSA_BENCH_PRV_ITERS,
            (unsigned)sw_prv_ms,
            (unsigned)(sw_prv_ms * 1000U / RSA_BENCH_PRV_ITERS),
            (unsigned)hw_prv_ms,
            (unsigned)(hw_prv_ms * 1000U / RSA_BENCH_PRV_ITERS));

    ok = (memcmp(sw_c, hw_c, sizeof(sw_c)) == 0) &&
         (memcmp(sw_m, hw_m, sizeof(sw_m)) == 0);
    sec_result("rsa sw==hw output", ok);
    if (!ok) {
        sec_dump("sw c    ", sw_c, sizeof(sw_c));
        sec_dump("hw c    ", hw_c, sizeof(hw_c));
        sec_dump("sw m    ", sw_m, sizeof(sw_m));
        sec_dump("hw m    ", hw_m, sizeof(hw_m));
    }
}

static void sec_test_rsa(void)
{
    /* Raw RSA only (no padding): public() then private() must round trip.
     * The plaintext is deliberately non-trivial (0x01 || 0x23 * 127) so that a
     * modexp returning its input unchanged cannot pass. A 1024-bit modulus is
     * >= 2^1022, so this message is always smaller than N. */
    TKL_ASYMMETRICAY_HANDLE ctx = NULL;
    UINT8_T msg[128];
    UINT8_T enc[128];
    UINT8_T dec[128];
    UINT8_T der[1024];
    size_t i;
    int ok;

    if (tkl_rsa_create_init(&ctx) != OPRT_OK) {
        sec_result("rsa create_init", 0);
        return;
    }
    sec_result("rsa create_init", 1);

    if (tkl_rsa_gen_key(ctx, 1024, 65537) != OPRT_OK) {
        sec_result("rsa gen_key(1024)", 0);
        tkl_rsa_free(ctx);
        return;
    }
    sec_result("rsa gen_key(1024)", 1);

    msg[0] = 0x01;
    for (i = 1; i < sizeof(msg); i++) {
        msg[i] = 0x23;
    }

    memset(enc, 0, sizeof(enc));
    memset(dec, 0, sizeof(dec));

    ok = (tkl_rsa_public(ctx, msg, enc) == OPRT_OK) &&
         (memcmp(enc, msg, sizeof(msg)) != 0) &&
         (tkl_rsa_private(ctx, enc, dec) == OPRT_OK) &&
         (memcmp(dec, msg, sizeof(msg)) == 0);
    sec_result("rsa public/private round trip", ok);

    memset(der, 0, sizeof(der));
    sec_result("rsa write_key_der", tkl_rsa_write_key_der(ctx, der, sizeof(der)) == OPRT_OK);

    sec_test_rsa_bench(ctx, msg);

    tkl_rsa_free(ctx);

    sec_test_rsa_kat();
}
#endif /* ENABLE_PLATFORM_RSA */

/* ------------------------------------------------------------------- ecc */

/* tkl_ecp_* live in tkl_asymmetrical.c, so the test is compiled out with the
 * same switch as the RSA test above. */
#if defined(ENABLE_PLATFORM_RSA)

/* NIST P-256 base point, the group order minus one and the y coordinate of -G.
 * (n-1)*G == -G == (Gx, p - Gy) is a known answer that needs no EC arithmetic
 * to derive, so it pins the scalar multiplication without a second table. */
static const char ecp_gx[] =
    "6b17d1f2e12c4247f8bce6e563a440f277037d812deb33a0f4a13945d898c296";
static const char ecp_gy[] =
    "4fe342e2fe1a7f9b8ee7eb4a7c0f9e162bce33576b315ececbb6406837bf51f5";
static const char ecp_n_minus_1[] =
    "ffffffff00000000ffffffffffffffffbce6faada7179e84f3b9cac2fc632550";
static const char ecp_neg_gy[] =
    "b01cbd1c01e58065711814b583f061e9d431cca994cea1313449bf97c840ae0a";

static void sec_test_ecp_kat(void)
{
    UINT8_T gx[32];
    UINT8_T gy[32];
    UINT8_T negy[32];
    UINT8_T k[32];
    UINT8_T qx[32];
    UINT8_T qy[32];
    int ok;

    hex2bin(ecp_gx, gx, sizeof(gx));
    hex2bin(ecp_gy, gy, sizeof(gy));
    hex2bin(ecp_neg_gy, negy, sizeof(negy));

    /* k = 1: 1*G == G */
    memset(k, 0, sizeof(k));
    k[sizeof(k) - 1] = 1;
    memset(qx, 0, sizeof(qx));
    memset(qy, 0, sizeof(qy));
    ok = (tkl_ecp_p256_mul(k, gx, gy, qx, qy) == OPRT_OK) &&
         (memcmp(qx, gx, sizeof(gx)) == 0) &&
         (memcmp(qy, gy, sizeof(gy)) == 0);
    sec_result("ecp kat 1*G", ok);
    if (!ok) {
        sec_dump("qx      ", qx, sizeof(qx));
        sec_dump("qy      ", qy, sizeof(qy));
    }

    /* k = n-1: (n-1)*G == -G */
    hex2bin(ecp_n_minus_1, k, sizeof(k));
    memset(qx, 0, sizeof(qx));
    memset(qy, 0, sizeof(qy));
    ok = (tkl_ecp_p256_mul(k, gx, gy, qx, qy) == OPRT_OK) &&
         (memcmp(qx, gx, sizeof(gx)) == 0) &&
         (memcmp(qy, negy, sizeof(negy)) == 0);
    sec_result("ecp kat (n-1)*G", ok);
    if (!ok) {
        sec_dump("qx      ", qx, sizeof(qx));
        sec_dump("qy      ", qy, sizeof(qy));
        sec_dump("expect y", negy, sizeof(negy));
    }
}

#define ECP_BENCH_ITERS 4

static void sec_test_ecp_bench(void)
{
    static UINT8_T gx[32];
    static UINT8_T gy[32];
    static UINT8_T k[32];
    static UINT8_T sw_x[32];
    static UINT8_T sw_y[32];
    static UINT8_T hw_x[32];
    static UINT8_T hw_y[32];
    uint32_t sw_ms;
    uint32_t hw_ms;
    uint32_t t0;
    uint32_t t1;
    int i;
    int ok;

    hex2bin(ecp_gx, gx, sizeof(gx));
    hex2bin(ecp_gy, gy, sizeof(gy));
    hex2bin(ecp_n_minus_1, k, sizeof(k));

    tkl_ecp_set_hw_enabled(0);
    t0 = rtos_get_time();
    for (i = 0; i < ECP_BENCH_ITERS; i++) {
        (void)tkl_ecp_p256_mul(k, gx, gy, sw_x, sw_y);
    }
    t1 = rtos_get_time();
    sw_ms = t1 - t0;

    tkl_ecp_set_hw_enabled(1);
    t0 = rtos_get_time();
    for (i = 0; i < ECP_BENCH_ITERS; i++) {
        (void)tkl_ecp_p256_mul(k, gx, gy, hw_x, hw_y);
    }
    t1 = rtos_get_time();
    hw_ms = t1 - t0;

    SEC_LOG("ecp p256 mul x%d: sw=%ums (%uus/op) | hw=%ums (%uus/op)\r\n",
            ECP_BENCH_ITERS,
            (unsigned)sw_ms, (unsigned)(sw_ms * 1000U / ECP_BENCH_ITERS),
            (unsigned)hw_ms, (unsigned)(hw_ms * 1000U / ECP_BENCH_ITERS));

    ok = (memcmp(sw_x, hw_x, sizeof(sw_x)) == 0) &&
         (memcmp(sw_y, hw_y, sizeof(sw_y)) == 0);
    sec_result("ecp sw==hw output", ok);
    if (!ok) {
        sec_dump("sw x    ", sw_x, sizeof(sw_x));
        sec_dump("hw x    ", hw_x, sizeof(hw_x));
        sec_dump("sw y    ", sw_y, sizeof(sw_y));
        sec_dump("hw y    ", hw_y, sizeof(hw_y));
    }
}

/* RFC 7748 X25519 vectors. All values are little-endian byte strings exactly
 * as the RFC prints them, so hex2bin() maps them straight to the buffers.
 * Section 5.2 is the fixed KAT, section 6.1 the base point / Diffie-Hellman
 * exchange (which also cross-checks itself: both directions must agree). */
static const char x25519_v1_k[] =
    "a546e36bf0527c9d3b16154b82465edd62144c0ac1fc5a18506a2244ba449ac4";
static const char x25519_v1_u[] =
    "e6db6867583030db3594c1a424b15f7c726624ec26b3353b10a903a6d0ab1c4c";
static const char x25519_v1_out[] =
    "c3da55379de9c6908e94ea4df28d084f32eccf03491c71f754b4075577a28552";
static const char x25519_v2_k[] =
    "4b66e9d4d1b4673c5ad22691957d6af5c11b6421e0ea01d42ca4169e7918ba0d";
static const char x25519_v2_u[] =
    "e5210f12786811d3f4b7959d0538ae2c31dbe7106fc03c3efc4cd549c715a493";
static const char x25519_v2_out[] =
    "95cbde9476e8907d7aade45cb4b873f88b595a68799fa152e6f8f7647aac7957";
static const char x25519_base[] =
    "0900000000000000000000000000000000000000000000000000000000000000";
static const char x25519_ak[] =
    "77076d0a7318a57d3c16c17251b26645df4c2f87ebc0992ab177fba51db92c2a";
static const char x25519_apub[] =
    "8520f0098930a754748b7ddcb43ef75a0dbf3a0d26381af4eba4a98eaa9b4e6a";
static const char x25519_bk[] =
    "5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c2f8b27ff88e0eb";
static const char x25519_bpub[] =
    "de9edb7d7b7dc1b4d35b61c2ece435373f8343c85b78674dadfc7e146f882b4f";
static const char x25519_ss[] =
    "4a5d9d5ba4ce2de1728e3bf480350f25e07e21c947d19e3376f09b3c1e161742";

static void sec_test_ecp_x25519_kat(void)
{
    UINT8_T k[32];
    UINT8_T u[32];
    UINT8_T exp[32];
    UINT8_T out[32];
    int ok;

    /* RFC 7748 5.2 vector 1 */
    hex2bin(x25519_v1_k, k, sizeof(k));
    hex2bin(x25519_v1_u, u, sizeof(u));
    hex2bin(x25519_v1_out, exp, sizeof(exp));
    memset(out, 0, sizeof(out));
    ok = (tkl_ecp_x25519_mul(k, u, out) == OPRT_OK) &&
         (memcmp(out, exp, sizeof(exp)) == 0);
    sec_result("x25519 kat rfc7748-1", ok);
    if (!ok) {
        sec_dump("got     ", out, sizeof(out));
        sec_dump("expect  ", exp, sizeof(exp));
    }

    /* RFC 7748 5.2 vector 2 */
    hex2bin(x25519_v2_k, k, sizeof(k));
    hex2bin(x25519_v2_u, u, sizeof(u));
    hex2bin(x25519_v2_out, exp, sizeof(exp));
    memset(out, 0, sizeof(out));
    ok = (tkl_ecp_x25519_mul(k, u, out) == OPRT_OK) &&
         (memcmp(out, exp, sizeof(exp)) == 0);
    sec_result("x25519 kat rfc7748-2", ok);
    if (!ok) {
        sec_dump("got     ", out, sizeof(out));
        sec_dump("expect  ", exp, sizeof(exp));
    }

    /* RFC 7748 6.1: public keys from the base point */
    hex2bin(x25519_base, u, sizeof(u));
    hex2bin(x25519_ak, k, sizeof(k));
    hex2bin(x25519_apub, exp, sizeof(exp));
    memset(out, 0, sizeof(out));
    ok = (tkl_ecp_x25519_mul(k, u, out) == OPRT_OK) &&
         (memcmp(out, exp, sizeof(exp)) == 0);
    sec_result("x25519 kat apub", ok);
    if (!ok) {
        sec_dump("got     ", out, sizeof(out));
        sec_dump("expect  ", exp, sizeof(exp));
    }

    hex2bin(x25519_bk, k, sizeof(k));
    hex2bin(x25519_bpub, exp, sizeof(exp));
    memset(out, 0, sizeof(out));
    ok = (tkl_ecp_x25519_mul(k, u, out) == OPRT_OK) &&
         (memcmp(out, exp, sizeof(exp)) == 0);
    sec_result("x25519 kat bpub", ok);
    if (!ok) {
        sec_dump("got     ", out, sizeof(out));
        sec_dump("expect  ", exp, sizeof(exp));
    }

    /* Shared secret, both directions (must agree with each other and the RFC) */
    {
        UINT8_T ak[32];
        UINT8_T bk[32];
        UINT8_T apub[32];
        UINT8_T bpub[32];
        UINT8_T ss_a[32];
        UINT8_T ss_b[32];

        hex2bin(x25519_ak, ak, sizeof(ak));
        hex2bin(x25519_bk, bk, sizeof(bk));
        hex2bin(x25519_apub, apub, sizeof(apub));
        hex2bin(x25519_bpub, bpub, sizeof(bpub));
        hex2bin(x25519_ss, exp, sizeof(exp));
        memset(ss_a, 0, sizeof(ss_a));
        memset(ss_b, 0, sizeof(ss_b));

        ok = (tkl_ecp_x25519_mul(ak, bpub, ss_a) == OPRT_OK) &&
             (tkl_ecp_x25519_mul(bk, apub, ss_b) == OPRT_OK) &&
             (memcmp(ss_a, ss_b, sizeof(ss_a)) == 0) &&
             (memcmp(ss_a, exp, sizeof(exp)) == 0);
        sec_result("x25519 kat shared", ok);
        if (!ok) {
            sec_dump("a       ", ss_a, sizeof(ss_a));
            sec_dump("b       ", ss_b, sizeof(ss_b));
            sec_dump("expect  ", exp, sizeof(exp));
        }
    }
}

#define X25519_BENCH_ITERS 4

static void sec_test_ecp_x25519_bench(void)
{
    static UINT8_T k[32];
    static UINT8_T u[32];
    static UINT8_T sw[32];
    static UINT8_T hw[32];
    uint32_t sw_ms;
    uint32_t hw_ms;
    uint32_t t0;
    uint32_t t1;
    int i;
    int ok;

    hex2bin(x25519_base, u, sizeof(u));
    hex2bin(x25519_ak, k, sizeof(k));

    tkl_ecp_set_hw_enabled(0);
    t0 = rtos_get_time();
    for (i = 0; i < X25519_BENCH_ITERS; i++) {
        (void)tkl_ecp_x25519_mul(k, u, sw);
    }
    t1 = rtos_get_time();
    sw_ms = t1 - t0;

    tkl_ecp_set_hw_enabled(1);
    t0 = rtos_get_time();
    for (i = 0; i < X25519_BENCH_ITERS; i++) {
        (void)tkl_ecp_x25519_mul(k, u, hw);
    }
    t1 = rtos_get_time();
    hw_ms = t1 - t0;

    SEC_LOG("x25519 mul x%d: sw=%ums (%uus/op) | hw=%ums (%uus/op)\r\n",
            X25519_BENCH_ITERS,
            (unsigned)sw_ms, (unsigned)(sw_ms * 1000U / X25519_BENCH_ITERS),
            (unsigned)hw_ms, (unsigned)(hw_ms * 1000U / X25519_BENCH_ITERS));

    ok = (memcmp(sw, hw, sizeof(sw)) == 0);
    sec_result("x25519 sw==hw output", ok);
    if (!ok) {
        sec_dump("sw      ", sw, sizeof(sw));
        sec_dump("hw      ", hw, sizeof(hw));
    }
}

static void sec_test_ecp(void)
{
    sec_test_ecp_kat();
    sec_test_ecp_bench();
    sec_test_ecp_x25519_kat();
    sec_test_ecp_x25519_bench();
}
#endif /* ENABLE_PLATFORM_RSA */


/* ------------------------------------------------------------------ glue */

static void sec_test_all(void)
{
    sec_failures = 0;

    sec_test_sha256();
    sec_test_sha256_long();
    sec_test_sha1();
    sec_test_sha1_long();
    sec_test_md5();
    sec_test_aes_ecb();
    sec_test_aes_cbc();
    sec_test_gcm();
#if defined(ENABLE_PLATFORM_RSA)
    sec_test_rsa();
    sec_test_ecp();
#endif

    SEC_LOG("---------- %d failure(s) ----------\r\n", sec_failures);
}

static void sec_usage(void)
{
    SEC_LOG("usage: xsecurity <sha|aes|gcm|rsa|ecp|all>\r\n");
    SEC_LOG("  note: tkl_rsa_* / tkl_ecp_* are inactive until libtuyaos.a is\r\n");
    SEC_LOG("        rebuilt with ENABLE_PLATFORM_RSA=y\r\n");
    SEC_LOG("  sha:    sha256/sha1/md5 via mbedtls+ALT, 3 bytes and 64 bytes\r\n");
}

typedef struct {
    int argc;
    char *argv[4];
    char argv_buf[4][16];
} sec_task_arg_t;

static void sec_task(void *param)
{
    sec_task_arg_t *arg = (sec_task_arg_t *)param;
    const char *sub = arg->argv[1];

    if (os_strcmp(sub, "sha") == 0) {
        sec_failures = 0;
        sec_test_sha256();
        sec_test_sha256_long();
        sec_test_sha1();
        sec_test_sha1_long();
        sec_test_md5();
        SEC_LOG("---------- %d failure(s) ----------\r\n", sec_failures);
    } else if (os_strcmp(sub, "aes") == 0) {
        sec_failures = 0;
        sec_test_aes_ecb();
        sec_test_aes_cbc();
        SEC_LOG("---------- %d failure(s) ----------\r\n", sec_failures);
    } else if (os_strcmp(sub, "gcm") == 0) {
        sec_failures = 0;
        sec_test_gcm();
        SEC_LOG("---------- %d failure(s) ----------\r\n", sec_failures);
    } else if (os_strcmp(sub, "rsa") == 0) {
        sec_failures = 0;
#if defined(ENABLE_PLATFORM_RSA)
        sec_test_rsa();
#else
        SEC_LOG("rsa: skipped, ENABLE_PLATFORM_RSA is not set\r\n");
#endif
        SEC_LOG("---------- %d failure(s) ----------\r\n", sec_failures);
    } else if (os_strcmp(sub, "ecp") == 0) {
        sec_failures = 0;
#if defined(ENABLE_PLATFORM_RSA)
        sec_test_ecp();
#else
        SEC_LOG("ecp: skipped, ENABLE_PLATFORM_RSA is not set\r\n");
#endif
        SEC_LOG("---------- %d failure(s) ----------\r\n", sec_failures);
    } else if (os_strcmp(sub, "all") == 0) {
        sec_test_all();
    } else {
        sec_usage();
    }

    os_free(arg);
    rtos_delete_thread(NULL);
}

void cli_tuya_security_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    sec_task_arg_t *arg = NULL;
    int i;

    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    if (argc < 2 || argv == NULL || argv[1] == NULL) {
        sec_usage();
        return;
    }

    if (os_strcmp(argv[1], "help") == 0 || os_strcmp(argv[1], "?") == 0) {
        sec_usage();
        return;
    }

    /* RSA key generation is CPU heavy: always run in a background task so the
     * shell task is not blocked and no watchdog is triggered. */
    arg = os_zalloc(sizeof(sec_task_arg_t));
    if (arg == NULL) {
        SEC_LOG("no memory\r\n");
        return;
    }

    arg->argc = (argc > 4) ? 4 : argc;
    for (i = 0; i < arg->argc; i++) {
        if (argv[i] != NULL) {
            os_strlcpy(arg->argv_buf[i], argv[i], sizeof(arg->argv_buf[i]));
        }
        arg->argv[i] = arg->argv_buf[i];
    }

    if (rtos_create_thread(NULL, SEC_TASK_PRIORITY, "xsecurity", sec_task,
                           SEC_TASK_STACK_SIZE, (beken_thread_arg_t)arg) != kNoErr) {
        SEC_LOG("create task failed\r\n");
        os_free(arg);
    }
}
