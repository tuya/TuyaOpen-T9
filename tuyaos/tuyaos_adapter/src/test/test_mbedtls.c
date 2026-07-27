#include "cli.h"
#include "cli_tuya_test.h"

#include "modules/pm.h"

#include <driver/aon_rtc.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include <components/log.h>

/* bk_printf uses bk_printf_ext which has a runtime level filter;
 * BK_RAW_LOGE uses bk_printf_raw which bypasses it — always visible */
#ifdef bk_printf
#undef bk_printf
#endif
#define bk_printf(fmt, ...) BK_RAW_LOGE("xmbedtls", fmt, ##__VA_ARGS__)

#include "mbedtls/aes.h"
#include "mbedtls/base64.h"
#include "mbedtls/bignum.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/entropy.h"
#include "mbedtls/error.h"
#include "mbedtls/gcm.h"
#include "mbedtls/pk.h"
#include "mbedtls/platform.h"
#include "mbedtls/rsa.h"
#include "mbedtls/sha1.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include "mbedtls/x509_crt.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MBEDTLS_SHA_TEST_CNT        1U
#define MBEDTLS_TEST_FREQUENCY      PM_CPU_FRQ_240M
#define MBEDTLS_TEST_PRIORITY       4
#define MBEDTLS_TEST_STACK_SIZE     (1024 * 5)
#define MBEDTLS_AES_ECB_LOOP_CNT    10000U
#define MBEDTLS_AES_CBC_LOOP_CNT    10000U
#define MBEDTLS_THREAD_MAX_DELAY_MS 5U
#define MBEDTLS_TLS_RANDOM_LEN      32U
#define MBEDTLS_TLS_PARAMS_BUF_LEN  256U
#define MBEDTLS_TLS_KEY_BUF_LEN     256U
/* Match real cloud TLS default ECDHE group (mbedtls ssl_preset_default_groups). */
#if defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
#define MBEDTLS_TLS_TEST_ECDHE_CURVE  MBEDTLS_ECP_DP_CURVE25519
#define MBEDTLS_TLS_TEST_ECDHE_NAME   "x25519"
#else
#define MBEDTLS_TLS_TEST_ECDHE_CURVE  MBEDTLS_ECP_DP_SECP256R1
#define MBEDTLS_TLS_TEST_ECDHE_NAME   "secp256r1"
#endif
/* TLS record layer: match ECDHE_*_AES_128_GCM (tuya_tls preferred suite) */
#define MBEDTLS_TLS_GCM_KEY_BITS    128U
#define MBEDTLS_TLS_GCM_AAD_LEN     13U  /* seq(8)+type(1)+ver(2)+len(2) */

#define MBEDTLS_GCM_IV_LEN          12U
#define MBEDTLS_GCM_KEY_BITS        256U
#define MBEDTLS_GCM_AD_LEN          16U
#define MBEDTLS_GCM_TAG_LEN         16U

#define MBEDTLS_RSA_KEY_LEN         128U
#define MBEDTLS_RSA_PT_LEN          24U

extern int bk_rand(void);
extern int bk_fill_rand(void *buff, size_t len);

typedef struct {
    uint32_t pass;
    uint32_t fail;
    uint32_t skip;
} mbedtls_test_stat_t;

typedef int (*mbedtls_selftest_fn_t)(int verbose);

typedef struct {
    const char *name;
    mbedtls_selftest_fn_t fn;
} mbedtls_selftest_case_t;

static const uint32_t s_mbedtls_test_len[] = {32U, 1024U, 4096U, 32768U};
/* Typical / medium / TLS 1.2 max record plaintext sizes */
static const uint32_t s_mbedtls_tls_record_len[] = {1024U, 4096U, 16384U};
static uint32_t s_mbedtls_thread_loop_cnt;
/* Accumulated RNG wall time for TLS ECDHE path (us). */
static uint64_t s_mbedtls_tls_rng_us;

static const unsigned char s_aes_test_ecb_dec[3][16] = {
    {0x44, 0x41, 0x6A, 0xC2, 0xD1, 0xF5, 0x3C, 0x58, 0x33, 0x03, 0x91, 0x7E, 0x6B, 0xE9, 0xEB, 0xE0},
    {0x48, 0xE3, 0x1E, 0x9E, 0x25, 0x67, 0x18, 0xF2, 0x92, 0x29, 0x31, 0x9C, 0x19, 0xF1, 0x5B, 0xA4},
    {0x05, 0x8C, 0xCF, 0xFD, 0xBB, 0xCB, 0x38, 0x2D, 0x1F, 0x6F, 0x56, 0x58, 0x5D, 0x8A, 0x4A, 0xDE}
};

static const unsigned char s_aes_test_ecb_enc[3][16] = {
    {0xC3, 0x4C, 0x05, 0x2C, 0xC0, 0xDA, 0x8D, 0x73, 0x45, 0x1A, 0xFE, 0x5F, 0x03, 0xBE, 0x29, 0x7F},
    {0xF3, 0xF6, 0x75, 0x2A, 0xE8, 0xD7, 0x83, 0x11, 0x38, 0xF0, 0x41, 0x56, 0x06, 0x31, 0xB1, 0x14},
    {0x8B, 0x79, 0xEE, 0xCC, 0x93, 0xA0, 0xEE, 0x5D, 0xFF, 0x30, 0xB4, 0xEA, 0x21, 0x63, 0x6D, 0xA4}
};

static const unsigned char s_aes_test_cbc_dec[3][16] = {
    {0xFA, 0xCA, 0x37, 0xE0, 0xB0, 0xC8, 0x53, 0x73, 0xDF, 0x70, 0x6E, 0x73, 0xF7, 0xC9, 0xAF, 0x86},
    {0x5D, 0xF6, 0x78, 0xDD, 0x17, 0xBA, 0x4E, 0x75, 0xB6, 0x17, 0x68, 0xC6, 0xAD, 0xEF, 0x7C, 0x7B},
    {0x48, 0x04, 0xE1, 0x81, 0x8F, 0xE6, 0x29, 0x75, 0x19, 0xA3, 0xE8, 0x8C, 0x57, 0x31, 0x04, 0x13}
};

static const unsigned char s_aes_test_cbc_enc[3][16] = {
    {0x8A, 0x05, 0xFC, 0x5E, 0x09, 0x5A, 0xF4, 0x84, 0x8A, 0x08, 0xD3, 0x28, 0xD3, 0x68, 0x8E, 0x3D},
    {0x7B, 0xD9, 0x66, 0xD5, 0x3A, 0xD8, 0xC1, 0xBB, 0x85, 0xD2, 0xAD, 0xFA, 0xE8, 0x7B, 0xB1, 0x04},
    {0xFE, 0x3C, 0x53, 0x65, 0x3E, 0x2F, 0x45, 0xB5, 0x6F, 0xCD, 0x88, 0xB2, 0xCC, 0x89, 0x8F, 0xF0}
};

static const unsigned char s_aes_test_ctr_pt[3][48] = {
    {0x53, 0x69, 0x6E, 0x67, 0x6C, 0x65, 0x20, 0x62, 0x6C, 0x6F, 0x63, 0x6B, 0x20, 0x6D, 0x73, 0x67},
    {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
     0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F},
    {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
     0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
     0x20, 0x21, 0x22, 0x23}
};

static const unsigned char s_aes_test_ctr_ct[3][48] = {
    {0xE4, 0x09, 0x5D, 0x4F, 0xB7, 0xA7, 0xB3, 0x79, 0x2D, 0x61, 0x75, 0xA3, 0x26, 0x13, 0x11, 0xB8},
    {0x51, 0x04, 0xA1, 0x06, 0x16, 0x8A, 0x72, 0xD9, 0x79, 0x0D, 0x41, 0xEE, 0x8E, 0xDA, 0xD3, 0x88,
     0xEB, 0x2E, 0x1E, 0xFC, 0x46, 0xDA, 0x57, 0xC8, 0xFC, 0xE6, 0x30, 0xDF, 0x91, 0x41, 0xBE, 0x28},
    {0xC1, 0xCF, 0x48, 0xA8, 0x9F, 0x2F, 0xFD, 0xD9, 0xCF, 0x46, 0x52, 0xE9, 0xEF, 0xDB, 0x72, 0xD7,
     0x45, 0x40, 0xA4, 0x2B, 0xDE, 0x6D, 0x78, 0x36, 0xD5, 0x9A, 0x5C, 0xEA, 0xAE, 0xF3, 0x10, 0x53,
     0x25, 0xB2, 0x07, 0x2F}
};

static const unsigned char s_aes_test_ctr_nonce_counter[3][16] = {
    {0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0x00, 0x6C, 0xB6, 0xDB, 0xC0, 0x54, 0x3B, 0x59, 0xDA, 0x48, 0xD9, 0x0B, 0x00, 0x00, 0x00, 0x01},
    {0x00, 0xE0, 0x01, 0x7B, 0x27, 0x77, 0x7F, 0x3F, 0x4A, 0x17, 0x86, 0xF0, 0x00, 0x00, 0x00, 0x01}
};

static const unsigned char s_aes_test_ctr_key[3][16] = {
    {0xAE, 0x68, 0x52, 0xF8, 0x12, 0x10, 0x67, 0xCC, 0x4B, 0xF7, 0xA5, 0x76, 0x55, 0x77, 0xF3, 0x9E},
    {0x7E, 0x24, 0x06, 0x78, 0x17, 0xFA, 0xE0, 0xD7, 0x43, 0xD6, 0xCE, 0x1F, 0x32, 0x53, 0x91, 0x63},
    {0x76, 0x91, 0xBE, 0x03, 0x5E, 0x50, 0x20, 0xA8, 0xAC, 0x6E, 0x61, 0x85, 0x29, 0xF9, 0xA0, 0xDC}
};

static const int s_aes_test_ctr_len[3] = {16, 32, 36};

static const uint8_t s_ecdsa_test_message[] = {
    0xAB, 0xE1, 0x0A, 0xCE, 0x13, 0xE7, 0xE1, 0xD9, 0x18, 0x6C, 0x48, 0xF7, 0x88, 0x9D, 0x51, 0x47,
    0x3D, 0x3A, 0x09, 0x61, 0x98, 0x4B, 0xC8, 0x72, 0xDF, 0x70, 0x8E, 0xCC, 0x3E, 0xD3, 0xB8, 0x16,
    0x9D, 0x01, 0xE3, 0xD9, 0x6F, 0xC4, 0xF1, 0xD5, 0xEA, 0x00, 0xA0, 0x36, 0x92, 0xBC, 0xC5, 0xCF,
    0xFD, 0x53, 0x78, 0x7C, 0x88, 0xB9, 0x34, 0xAF, 0x40, 0x4C, 0x03, 0x9D, 0x32, 0x89, 0xB5, 0xBA,
    0xC5, 0xAE, 0x7D, 0xB1, 0x49, 0x68, 0x75, 0xB5, 0xDC, 0x73, 0xC3, 0x09, 0xF9, 0x25, 0xC1, 0x3D,
    0x1C, 0x01, 0xAB, 0xDA, 0xAF, 0xEB, 0xCD, 0xAC, 0x2C, 0xEE, 0x43, 0x39, 0x39, 0xCE, 0x8D, 0x4A,
    0x0A, 0x5D, 0x57, 0xBB, 0x70, 0x5F, 0x3B, 0xF6, 0xEC, 0x08, 0x47, 0x95, 0x11, 0xD4, 0xB4, 0xA3,
    0x21, 0x1F, 0x61, 0x64, 0x9A, 0xD6, 0x27, 0x43, 0x14, 0xBF, 0x0D, 0x43, 0x8A, 0x81, 0xE0, 0x60
};

static const char s_rsa_n[] =
    "9292758453063D803DD603D5E777D788"
    "8ED1D5BF35786190FA2F23EBC0848AEA"
    "DDA92CA6C3D80B32C4D109BE0F36D6AE"
    "7130B9CED7ACDF54CFC7555AC14EEBAB"
    "93A89813FBF3C4F8066D2D800F7C38A8"
    "1AE31942917403FF4946B0A83D3D3E05"
    "EE57C6F5F5606FB5D4BC6CD34EE0801A"
    "5E94BB77B07507233A0BC7BAC8F90F79";

static const char s_rsa_e[] = "10001";

static const char s_rsa_d[] =
    "24BF6185468786FDD303083D25E64EFC"
    "66CA472BC44D253102F8B4A9D3BFA750"
    "91386C0077937FE33FA3252D28855837"
    "AE1B484A8A9A45F7EE8C0C634F99E8CD"
    "DF79C5CE07EE72C7F123142198164234"
    "CABB724CF78B8173B9F880FC86322407"
    "AF1FEDFDDE2BEB674CA15F3E81A1521E"
    "071513A1E85B5DFA031F21ECAE91A34D";

static const char s_rsa_p[] =
    "C36D0EB7FCD285223CFB5AABA5BDA3D8"
    "2C01CAD19EA484A87EA4377637E75500"
    "FCB2005C5C7DD6EC4AC023CDA285D796"
    "C3D9E75E1EFC42488BB4F1D13AC30A57";

static const char s_rsa_q[] =
    "C000DF51A7C77AE8D7C7370C1FF55B69"
    "E211C2B9E5DB1ED0BF61D0D9899620F4"
    "910E4168387E3C30AA1E00C339A79508"
    "8452DD96A9A5EA5D9DCA68DA636032AF";

static const unsigned char s_rsa_pt[MBEDTLS_RSA_PT_LEN] = {
    0xAA, 0xBB, 0xCC, 0x03, 0x02, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x11, 0x22, 0x33, 0x0A, 0x0B, 0x0C, 0xCC, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD
};

static const char s_tls_test_key_pem[] =
    "-----BEGIN PRIVATE KEY-----\r\n"
    "MIGHAgEAMBMGByqGSM49AgEGCCqGSM49AwEHBG0wawIBAQQgSmg4ZmkOnnfbkr6e\r\n"
    "J9olSiOQcI3In90o7Z5mc5jBNNKhRANCAARI/h6GgB26LzS91sy44FOL1Xk3Uw28\r\n"
    "e6DKlLtz57PH6qT1HriuQbsqyJUGtwaNS2GuTjBRtUfU4pxtR3x8QvAp\r\n"
    "-----END PRIVATE KEY-----\r\n";

static const char s_tls_test_chain_pem[] =
    "-----BEGIN CERTIFICATE-----\r\n"
    "MIIB7zCCAZWgAwIBAgIEf5abBDAKBggqhkjOPQQDAjCBjjELMAkGA1UEBhMCVVMx\r\n"
    "EzARBgNVBAgMCldhc2hpbmd0b24xEDAOBgNVBAcMB1NlYXR0bGUxGDAWBgNVBAoM\r\n"
    "D0FtYXpvbi5jb20gSW5jLjEaMBgGA1UECwwRRGV2aWNlIE1hbmFnZW1lbnQxIjAg\r\n"
    "BgNVBAMMGURBSyBDQSBmb3IgQTEwMU5VVlZOUzc2MzQwHhcNMjAwNTIxMDQwODIy\r\n"
    "WhcNNDAwNTIxMDQwODIyWjAfMR0wGwYJKwYBBAGlawEDDA5BMTAxTlVWVk5TNzYz\r\n"
    "NDBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABEj+HoaAHbovNL3WzLjgU4vVeTdT\r\n"
    "Dbx7oMqUu3Pns8fqpPUeuK5BuyrIlQa3Bo1LYa5OMFG1R9TinG1HfHxC8CmjTzBN\r\n"
    "MB0GA1UdDgQWBBRXoaVEB5rcaYsf3zF/SIesWIoHxTAfBgNVHSMEGDAWgBTPYP0y\r\n"
    "cVxp8p67ybjjw1nHIeXPzjALBgNVHQ8EBAMCA6gwCgYIKoZIzj0EAwIDSAAwRQIh\r\n"
    "AIEuXNXw+A8RIJrbGIFf1TzlQ0/mYplWKwFuC27jpWdGAiAInPm8uB9QaTM3Nr8z\r\n"
    "zp4O5vZQrCYMRFcvUreNIglTrw==\r\n"
    "-----END CERTIFICATE-----\r\n"
    "-----BEGIN CERTIFICATE-----\r\n"
    "MIIDVDCCAvqgAwIBAgIOAMh0yNgkaQl0BGq/jbUwCgYIKoZIzj0EAwIwgaYxCzAJ\r\n"
    "BgNVBAYTAlVTMRMwEQYDVQQIDApXYXNoaW5ndG9uMRAwDgYDVQQHDAdTZWF0dGxl\r\n"
    "MRgwFgYDVQQKDA9BbWF6b24uY29tIEluYy4xGjAYBgNVBAsMEURldmljZSBNYW5h\r\n"
    "Z2VtZW50MTowOAYDVQQDDDFBbWF6b25ESEEgdHV5YV9Qcm9kdWN0X0ExMDFOVVZW\r\n"
    "TlM3NjM0IENlcnRpZmljYXRlMB4XDTIwMDQyOTE2MjU1NloXDTQwMDQyOTE2MjU1\r\n"
    "NlowgY4xCzAJBgNVBAYTAlVTMRMwEQYDVQQIDApXYXNoaW5ndG9uMRAwDgYDVQQH\r\n"
    "DAdTZWF0dGxlMRgwFgYDVQQKDA9BbWF6b24uY29tIEluYy4xGjAYBgNVBAsMEURl\r\n"
    "dmljZSBNYW5hZ2VtZW50MSIwIAYDVQQDDBlEQUsgQ0EgZm9yIEExMDFOVVZWTlM3\r\n"
    "NjM0MFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEiD3cM+GTfNsfcJSO1zHbafKe\r\n"
    "MbdsHCU5vHzdD3IURcteJgnvQqjSMce/A82EmfL8G6GcHgq63j1Cik7dYPL5qKOC\r\n"
    "ASAwggEcMBIGA1UdEwEB/wQIMAYBAf8CAQAwHQYDVR0OBBYEFM9g/TJxXGnynrvJ\r\n"
    "uOPDWcch5c/OMIHWBgNVHSMEgc4wgcuAFBKrjoX+A7DTDyRzm5td9WGz91TYoYGi\r\n"
    "pIGfMIGcMQswCQYDVQQGEwJVUzETMBEGA1UECAwKV2FzaGluZ3RvbjEQMA4GA1UE\r\n"
    "BwwHU2VhdHRsZTEYMBYGA1UECgwPQW1hem9uLmNvbSBJbmMuMRowGAYDVQQLDBFE\r\n"
    "ZXZpY2UgTWFuYWdlbWVudDEwMC4GA1UEAwwnQW1hem9uREhBIFR1eWFfTWFudWZh\r\n"
    "Y3R1cmVyIENlcnRpZmljYXRlgg4AyHTIzjPtVZWdZb0jGDAOBgNVHQ8BAf8EBAMC\r\n"
    "AoQwCgYIKoZIzj0EAwIDSAAwRQIgBA7X5ncLg8Uwht+segzN0zSi2AwDpcOWIa4D\r\n"
    "9DcA3WoCIQCgsqkBjYr60zfyF0irOgJrlh+HgulKudfc/ESTSJs0aQ==\r\n"
    "-----END CERTIFICATE-----\r\n"
    "-----BEGIN CERTIFICATE-----\r\n"
    "MIIDWDCCAv2gAwIBAgIOAMh0yM4z7VWVnWW9IxgwCgYIKoZIzj0EAwIwgZwxCzAJ\r\n"
    "BgNVBAYTAlVTMRMwEQYDVQQIDApXYXNoaW5ndG9uMRAwDgYDVQQHDAdTZWF0dGxl\r\n"
    "MRgwFgYDVQQKDA9BbWF6b24uY29tIEluYy4xGjAYBgNVBAsMEURldmljZSBNYW5h\r\n"
    "Z2VtZW50MTAwLgYDVQQDDCdBbWF6b25ESEEgVHV5YV9NYW51ZmFjdHVyZXIgQ2Vy\r\n"
    "dGlmaWNhdGUwHhcNMjAwNDI5MTYyNTUyWhcNNDAwNDI5MTYyNTUyWjCBpjELMAkG\r\n"
    "A1UEBhMCVVMxEzARBgNVBAgMCldhc2hpbmd0b24xEDAOBgNVBAcMB1NlYXR0bGUx\r\n"
    "GDAWBgNVBAoMD0FtYXpvbi5jb20gSW5jLjEaMBgGA1UECwwRRGV2aWNlIE1hbmFn\r\n"
    "ZW1lbnQxOjA4BgNVBAMMMUFtYXpvbkRIQSB0dXlhX1Byb2R1Y3RfQTEwMU5VVlZO\r\n"
    "Uzc2MzQgQ2VydGlmaWNhdGUwWTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAAQPXnAv\r\n"
    "MfkLIPa36y1tytRiQnM6EML+KAS4yuDjN+WiqsjzAbmmrJISd0FOdaiKZLRy1CqQ\r\n"
    "8LPlnLeWN+ez75N6o4IBFTCCAREwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQU\r\n"
    "EquOhf4DsNMPJHObm131YbP3VNgwgc4GA1UdIwSBxjCBw4AU8RzI5oQ+OsSvsUub\r\n"
    "ORRuOEETp/ShgZqkgZcwgZQxFDASBgNVBAoMC0FtYXpvbiBJbmMuMSMwIQYDVQQL\r\n"
    "DBpEZXZpY2UgTWFuYWdlbWVudCBTZXJ2aWNlczEQMA4GA1UEBwwHU2VhdHRsZTET\r\n"
    "MBEGA1UECAwKV2FzaGluZ3RvbjELMAkGA1UEBhMCVVMxIzAhBgNVBAMMGkFtYXpv\r\n"
    "bkRIQSBSb290IENlcnRpZmljYXRlgg4AxujSrs5VP8cNqAB3ZjAOBgNVHQ8BAf8E\r\n"
    "BAMCAYYwCgYIKoZIzj0EAwIDSQAwRgIhALak4BjOMpaXW+ZnrMnfJQMfhyfDCw4r\r\n"
    "n2k6AlIJOb6KAiEAtZd1Xbp/qn7h16tosTdbeJCO9AYBURUDpiR1f6tKI4Q=\r\n"
    "-----END CERTIFICATE-----\r\n"
    "-----BEGIN CERTIFICATE-----\r\n"
    "MIIDQDCCAuagAwIBAgIOAMbo0q7OVT/HDagAd2YwCgYIKoZIzj0EAwIwgZQxFDAS\r\n"
    "BgNVBAoMC0FtYXpvbiBJbmMuMSMwIQYDVQQLDBpEZXZpY2UgTWFuYWdlbWVudCBT\r\n"
    "ZXJ2aWNlczEQMA4GA1UEBwwHU2VhdHRsZTETMBEGA1UECAwKV2FzaGluZ3RvbjEL\r\n"
    "MAkGA1UEBhMCVVMxIzAhBgNVBAMMGkFtYXpvbkRIQSBSb290IENlcnRpZmljYXRl\r\n"
    "MB4XDTE5MTIwOTIwMjU0MFoXDTM5MTIwOTIwMjU0MFowgZwxCzAJBgNVBAYTAlVT\r\n"
    "MRMwEQYDVQQIDApXYXNoaW5ndG9uMRAwDgYDVQQHDAdTZWF0dGxlMRgwFgYDVQQK\r\n"
    "DA9BbWF6b24uY29tIEluYy4xGjAYBgNVBAsMEURldmljZSBNYW5hZ2VtZW50MTAw\r\n"
    "LgYDVQQDDCdBbWF6b25ESEEgVHV5YV9NYW51ZmFjdHVyZXIgQ2VydGlmaWNhdGUw\r\n"
    "WTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAARBmDa+6dYNG2/KBpLv08bn6e1lmoKf\r\n"
    "1hv0e8W3tQW97u1kFpiq3BF1MaQqrAiV0vqFItWOeWCXwV7oy3hz+6Q7o4IBEDCC\r\n"
    "AQwwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQU8RzI5oQ+OsSvsUubORRuOEET\r\n"
    "p/QwgckGA1UdIwSBwTCBvoAUYYvcTyRKpW7uCmW8vHSaUZXqq2ChgZqkgZcwgZQx\r\n"
    "FDASBgNVBAoMC0FtYXpvbiBJbmMuMSMwIQYDVQQLDBpEZXZpY2UgTWFuYWdlbWVu\r\n"
    "dCBTZXJ2aWNlczEQMA4GA1UEBwwHU2VhdHRsZTETMBEGA1UECAwKV2FzaGluZ3Rv\r\n"
    "bjELMAkGA1UEBhMCVVMxIzAhBgNVBAMMGkFtYXpvbkRIQSBSb290IENlcnRpZmlj\r\n"
    "YXRlggkAqkHFg3IfxuIwDgYDVR0PAQH/BAQDAgGGMAoGCCqGSM49BAMCA0gAMEUC\r\n"
    "IQDnRZkmmDemoO+Mk7yutOIpp0mF2Cj4+1Y35uHLnobBfQIgOuZdBZK4Pab8TD2g\r\n"
    "9TzvlsmQkQqm1VTNyqwykH0o+5g=\r\n"
    "-----END CERTIFICATE-----\r\n"
    "-----BEGIN CERTIFICATE-----\r\n"
    "MIIDNjCCAtugAwIBAgIJAKpBxYNyH8biMAoGCCqGSM49BAMCMIGUMRQwEgYDVQQK\r\n"
    "DAtBbWF6b24gSW5jLjEjMCEGA1UECwwaRGV2aWNlIE1hbmFnZW1lbnQgU2Vydmlj\r\n"
    "ZXMxEDAOBgNVBAcMB1NlYXR0bGUxEzARBgNVBAgMCldhc2hpbmd0b24xCzAJBgNV\r\n"
    "BAYTAlVTMSMwIQYDVQQDDBpBbWF6b25ESEEgUm9vdCBDZXJ0aWZpY2F0ZTAgFw0x\r\n"
    "NzExMTYwNjI3MDlaGA8yMTE3MTEwODA2MjcwOVowgZQxFDASBgNVBAoMC0FtYXpv\r\n"
    "biBJbmMuMSMwIQYDVQQLDBpEZXZpY2UgTWFuYWdlbWVudCBTZXJ2aWNlczEQMA4G\r\n"
    "A1UEBwwHU2VhdHRsZTETMBEGA1UECAwKV2FzaGluZ3RvbjELMAkGA1UEBhMCVVMx\r\n"
    "IzAhBgNVBAMMGkFtYXpvbkRIQSBSb290IENlcnRpZmljYXRlMFkwEwYHKoZIzj0C\r\n"
    "AQYIKoZIzj0DAQcDQgAEv7+Zfvkc+qvUaKgaGxQMZoDFHQ18Z5OSXB4BNkYRszgR\r\n"
    "PQ82o54KVb0RKkkq0e3niRn1gUZ8jozePEPrpPV5yaOCARAwggEMMA8GA1UdEwEB\r\n"
    "/wQFMAMBAf8wHQYDVR0OBBYEFGGL3E8kSqVu7gplvLx0mlGV6qtgMIHJBgNVHSME\r\n"
    "gcEwgb6AFGGL3E8kSqVu7gplvLx0mlGV6qtgoYGapIGXMIGUMRQwEgYDVQQKDAtB\r\n"
    "bWF6b24gSW5jLjEjMCEGA1UECwwaRGV2aWNlIE1hbmFnZW1lbnQgU2VydmljZXMx\r\n"
    "EDAOBgNVBAcMB1NlYXR0bGUxEzARBgNVBAgMCldhc2hpbmd0b24xCzAJBgNVBAYT\r\n"
    "AlVTMSMwIQYDVQQDDBpBbWF6b25ESEEgUm9vdCBDZXJ0aWZpY2F0ZYIJAKpBxYNy\r\n"
    "H8biMA4GA1UdDwEB/wQEAwIBhjAKBggqhkjOPQQDAgNJADBGAiEAoCM4t1cMuTeu\r\n"
    "8yIlw/1BUIUb1Q4MYXp+LyfjcbmVz8ECIQDuKW8gQZjUS8Z7GcgNYnFup3UjTznj\r\n"
    "5ja4/PmvDx0Glw==\r\n"
    "-----END CERTIFICATE-----\r\n";

static int mbedtls_test_rng(void *rng_state, unsigned char *output, size_t len)
{
    uint32_t rnd = 0;
    size_t i;

    (void)rng_state;

    if (output == NULL) {
        return -1;
    }

    for (i = 0; i < len; i++) {
        if ((i & 0x3U) == 0U) {
            rnd = (uint32_t)bk_rand();
        }
        output[i] = (unsigned char)((rnd >> ((i & 0x3U) * 8U)) & 0xFFU);
    }

    return 0;
}

/**
 * @brief TLS-path RNG using platform fill (HW TRNG when available), with timing
 * @param[in] rng_state unused
 * @param[out] output random bytes buffer
 * @param[in] len number of bytes to fill
 * @return 0 on success, negative on failure
 * @note Closer to tuya_tls __tuya_tls_random / uni_random_bytes cost than bk_rand packing.
 */
static int mbedtls_test_tls_rng(void *rng_state, unsigned char *output, size_t len)
{
    uint64_t t0;
    int ret;

    (void)rng_state;

    if (output == NULL) {
        return -1;
    }

    t0 = bk_aon_rtc_get_us();
    ret = bk_fill_rand(output, len);
    s_mbedtls_tls_rng_us += bk_aon_rtc_get_us() - t0;
    return ret;
}

/**
 * @brief Duplicate a contiguous PEM certificate range [from, to] (0-based, inclusive)
 * @param[in] pem PEM bundle string
 * @param[in] from first certificate index
 * @param[in] to last certificate index
 * @param[out] out malloc'd null-terminated PEM slice (caller frees)
 * @param[out] out_len length including terminating NUL (for mbedtls_x509_crt_parse)
 * @return 0 on success, -1 on failure
 */
static int mbedtls_test_pem_dup_range(const char *pem,
                                      unsigned int from,
                                      unsigned int to,
                                      unsigned char **out,
                                      size_t *out_len)
{
    const char *begin_mark = "-----BEGIN CERTIFICATE-----";
    const char *p;
    const char *range_start = NULL;
    const char *range_end = NULL;
    unsigned int idx = 0;
    size_t len;
    unsigned char *buf;

    if ((pem == NULL) || (out == NULL) || (out_len == NULL) || (from > to)) {
        return -1;
    }

    p = pem;
    while ((p = strstr(p, begin_mark)) != NULL) {
        if (idx == from) {
            range_start = p;
        }
        if (idx == (to + 1U)) {
            range_end = p;
            break;
        }
        idx++;
        p += 27; /* strlen(begin_mark) */
    }

    if (range_start == NULL) {
        return -1;
    }
    if (range_end == NULL) {
        range_end = range_start + strlen(range_start);
    }

    len = (size_t)(range_end - range_start);
    buf = (unsigned char *)os_malloc(len + 1U);
    if (buf == NULL) {
        return -1;
    }
    memcpy(buf, range_start, len);
    buf[len] = '\0';
    *out = buf;
    *out_len = len + 1U;
    return 0;
}

static double mbedtls_test_perf_mbps(size_t bytes, uint32_t loops, uint64_t elapsed_us)
{
    if (elapsed_us == 0U) {
        return 0.0;
    }

    return ((double)bytes * (double)loops) / (double)elapsed_us;
}

static double mbedtls_test_perf_ops(uint32_t loops, uint64_t elapsed_us)
{
    if (elapsed_us == 0U) {
        return 0.0;
    }

    return ((double)loops * 1000000.0) / (double)elapsed_us;
}

static void mbedtls_test_fill_pattern(unsigned char *buf, size_t len)
{
    size_t i;

    if (buf == NULL) {
        return;
    }

    for (i = 0; i < len; i++) {
        buf[i] = (unsigned char)(i & 0xFFU);
    }
}

static void mbedtls_test_fill_increment(unsigned char *buf, size_t len)
{
    size_t i;

    if (buf == NULL) {
        return;
    }

    for (i = 0; i < len; i++) {
        buf[i] = (unsigned char)i;
    }
}

static void mbedtls_test_usage(void)
{
    bk_printf("xmbedtls help\r\n");
    bk_printf("xmbedtls sha {256|512}\r\n");
    bk_printf("xmbedtls aes {ecb|cbc|ctr|gcm}\r\n");
    bk_printf("xmbedtls ecdsa [cnt]\r\n");
    bk_printf("xmbedtls tls [cnt]  (HS + AES-128-GCM record 1K/4K/16K)\r\n");
    bk_printf("xmbedtls rsa\r\n");
    bk_printf("xmbedtls selftest [--quiet|-q] [suite...]\r\n");
    bk_printf("xmbedtls thread create <cnt>\r\n");
}

static void mbedtls_test_vote_cpu_freq(void)
{
    /* PM_DEV_ID_SECURE_WORLD is only valid with TrustEngine; skip on SW-only builds */
}

static void mbedtls_test_restore_cpu_freq(void)
{
    /* same reason as above */
}

static void mbedtls_test_log_pass(mbedtls_test_stat_t *st, const char *test_name, int ret)
{
    if ((st == NULL) || (test_name == NULL)) {
        return;
    }

    st->pass++;
    bk_printf("[xmbedtls] PASS test_name=%s ret=%d\r\n", test_name, ret);
}

static void mbedtls_test_log_fail(mbedtls_test_stat_t *st, const char *test_name, int ret)
{
    if ((st == NULL) || (test_name == NULL)) {
        return;
    }

    st->fail++;
    bk_printf("[xmbedtls] FAIL test_name=%s ret=%d\r\n", test_name, ret);
}

static void mbedtls_test_log_result(mbedtls_test_stat_t *st, const char *test_name, int ret)
{
    if (ret == 0) {
        mbedtls_test_log_pass(st, test_name, ret);
    } else {
        mbedtls_test_log_fail(st, test_name, ret);
    }
}

static void mbedtls_test_log_summary(const char *suite_name, const mbedtls_test_stat_t *st)
{
    if ((suite_name == NULL) || (st == NULL)) {
        return;
    }

    bk_printf("[xmbedtls] %s summary: pass=%u fail=%u skip=%u final=%s\r\n",
              suite_name,
              (unsigned int)st->pass,
              (unsigned int)st->fail,
              (unsigned int)st->skip,
              (st->fail == 0U) ? "PASS" : "FAIL");
}

static int mbedtls_test_sha256_once(const unsigned char *input, size_t input_len)
{
    unsigned char digest[32];
    mbedtls_sha256_context ctx;
    int ret;

    mbedtls_sha256_init(&ctx);

    ret = mbedtls_sha256_starts(&ctx, 0);
    if (ret == 0) {
        ret = mbedtls_sha256_update(&ctx, input, input_len);
    }
    if (ret == 0) {
        ret = mbedtls_sha256_finish(&ctx, digest);
    }

    mbedtls_sha256_free(&ctx);
    return ret;
}

static int mbedtls_test_sha256_loop_test(uint32_t input_len, uint32_t loops)
{
    unsigned char *buf = NULL;
    uint64_t start_us;
    uint64_t elapsed_us;
    int ret = -1;

    buf = os_malloc(input_len);
    if (buf == NULL) {
        return -1;
    }

    mbedtls_test_fill_pattern(buf, input_len);
    start_us = bk_aon_rtc_get_us();

    for (uint32_t i = 0; i < loops; i++) {
        ret = mbedtls_test_sha256_once(buf, input_len);
        if (ret != 0) {
            break;
        }
    }

    elapsed_us = bk_aon_rtc_get_us() - start_us;
    if (ret == 0) {
        bk_printf("[xmbedtls] INFO sha256 loops=%u len=%u elapsed_us=%llu perf=%.2fMB/s\r\n",
                  (unsigned int)loops,
                  (unsigned int)input_len,
                  (unsigned long long)elapsed_us,
                  mbedtls_test_perf_mbps(input_len, loops, elapsed_us));
    }

    os_free(buf);
    return ret;
}

static int mbedtls_test_sha512_loop_test(uint32_t input_len, uint32_t loops)
{
    unsigned char *buf = NULL;
    unsigned char digest[64];
    uint64_t start_us;
    uint64_t elapsed_us;
    int ret = -1;

    buf = os_malloc(input_len);
    if (buf == NULL) {
        return -1;
    }

    mbedtls_test_fill_pattern(buf, input_len);
    start_us = bk_aon_rtc_get_us();

    for (uint32_t i = 0; i < loops; i++) {
        ret = mbedtls_sha512(buf, input_len, digest, 0);
        if (ret != 0) {
            break;
        }
    }

    elapsed_us = bk_aon_rtc_get_us() - start_us;
    if (ret == 0) {
        bk_printf("[xmbedtls] INFO sha512 loops=%u len=%u elapsed_us=%llu perf=%.2fMB/s\r\n",
                  (unsigned int)loops,
                  (unsigned int)input_len,
                  (unsigned long long)elapsed_us,
                  mbedtls_test_perf_mbps(input_len, loops, elapsed_us));
    }

    os_free(buf);
    return ret;
}

static int mbedtls_test_aes_ecb_impl(void)
{
    mbedtls_aes_context ctx;
    unsigned char key[32];
    unsigned char buf[64];
    uint64_t start_us;
    uint64_t elapsed_us;
    int ret = 0;

    mbedtls_aes_init(&ctx);
    memset(key, 0, sizeof(key));

    for (int i = 0; i < 6; i++) {
        unsigned int keybits = 128U + (unsigned int)(i >> 1) * 64U;
        int mode = i & 1;
        int idx = i >> 1;
        const unsigned char *expect;

        memset(buf, 0, 16);
        start_us = bk_aon_rtc_get_us();

        if (mode == MBEDTLS_AES_DECRYPT) {
            ret = mbedtls_aes_setkey_dec(&ctx, key, keybits);
            expect = s_aes_test_ecb_dec[idx];
        } else {
            ret = mbedtls_aes_setkey_enc(&ctx, key, keybits);
            expect = s_aes_test_ecb_enc[idx];
        }

        if (ret == MBEDTLS_ERR_PLATFORM_FEATURE_UNSUPPORTED && keybits == 192U) {
            continue;
        }
        if (ret != 0) {
            break;
        }

        for (uint32_t loop = 0; loop < MBEDTLS_AES_ECB_LOOP_CNT; loop++) {
            ret = mbedtls_aes_crypt_ecb(&ctx, mode, buf, buf);
            if (ret != 0) {
                break;
            }
        }
        if (ret != 0) {
            break;
        }

        elapsed_us = bk_aon_rtc_get_us() - start_us;
        bk_printf("[xmbedtls] INFO aes_ecb keybits=%u mode=%s elapsed_us=%llu perf=%.2fMB/s\r\n",
                  keybits,
                  (mode == MBEDTLS_AES_DECRYPT) ? "dec" : "enc",
                  (unsigned long long)elapsed_us,
                  mbedtls_test_perf_mbps(16U, MBEDTLS_AES_ECB_LOOP_CNT, elapsed_us));

        if (memcmp(buf, expect, 16) != 0) {
            ret = -1;
            break;
        }
    }

    mbedtls_aes_free(&ctx);
    return ret;
}

static int mbedtls_test_aes_cbc_impl(void)
{
    mbedtls_aes_context ctx;
    unsigned char key[32];
    unsigned char buf[64];
    unsigned char iv[16];
    unsigned char prev[16];
    uint64_t start_us;
    uint64_t elapsed_us;
    int ret = 0;

    mbedtls_aes_init(&ctx);
    memset(key, 0, sizeof(key));

    for (int i = 0; i < 6; i++) {
        unsigned int keybits = 128U + (unsigned int)(i >> 1) * 64U;
        int mode = i & 1;
        int idx = i >> 1;
        const unsigned char *expect;

        memset(iv, 0, sizeof(iv));
        memset(prev, 0, sizeof(prev));
        memset(buf, 0, 16);
        start_us = bk_aon_rtc_get_us();

        if (mode == MBEDTLS_AES_DECRYPT) {
            ret = mbedtls_aes_setkey_dec(&ctx, key, keybits);
            expect = s_aes_test_cbc_dec[idx];
        } else {
            ret = mbedtls_aes_setkey_enc(&ctx, key, keybits);
            expect = s_aes_test_cbc_enc[idx];
        }

        if (ret == MBEDTLS_ERR_PLATFORM_FEATURE_UNSUPPORTED && keybits == 192U) {
            continue;
        }
        if (ret != 0) {
            break;
        }

        for (uint32_t loop = 0; loop < MBEDTLS_AES_CBC_LOOP_CNT; loop++) {
            if (mode == MBEDTLS_AES_ENCRYPT) {
                unsigned char tmp[16];

                memcpy(tmp, prev, 16);
                memcpy(prev, buf, 16);
                memcpy(buf, tmp, 16);
            }

            ret = mbedtls_aes_crypt_cbc(&ctx, mode, 16, iv, buf, buf);
            if (ret != 0) {
                break;
            }
        }
        if (ret != 0) {
            break;
        }

        elapsed_us = bk_aon_rtc_get_us() - start_us;
        bk_printf("[xmbedtls] INFO aes_cbc keybits=%u mode=%s elapsed_us=%llu perf=%.2fMB/s\r\n",
                  keybits,
                  (mode == MBEDTLS_AES_DECRYPT) ? "dec" : "enc",
                  (unsigned long long)elapsed_us,
                  mbedtls_test_perf_mbps(16U, MBEDTLS_AES_CBC_LOOP_CNT, elapsed_us));

        if (memcmp(buf, expect, 16) != 0) {
            ret = -1;
            break;
        }
    }

    mbedtls_aes_free(&ctx);
    return ret;
}

static int mbedtls_test_aes_cbc_large_data(uint32_t input_len)
{
    mbedtls_aes_context ctx;
    unsigned char key[32];
    unsigned char iv[16];
    unsigned char *plain = NULL;
    unsigned char *cipher = NULL;
    uint64_t start_us;
    uint64_t elapsed_us;
    int ret = 0;

    plain = os_malloc(input_len);
    cipher = os_malloc(input_len);
    if ((plain == NULL) || (cipher == NULL)) {
        ret = -1;
        goto cleanup;
    }

    mbedtls_test_fill_pattern(plain, input_len);
    memset(cipher, 0, input_len);
    memset(iv, 0x5A, sizeof(iv));
    mbedtls_test_fill_increment(key, sizeof(key));

    mbedtls_aes_init(&ctx);
    start_us = bk_aon_rtc_get_us();
    ret = mbedtls_aes_setkey_enc(&ctx, key, 256);
    if (ret == 0) {
        ret = mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_ENCRYPT, input_len, iv, plain, cipher);
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    mbedtls_aes_free(&ctx);
    if (ret != 0) {
        goto cleanup;
    }

    bk_printf("[xmbedtls] INFO aes_cbc_encrypt len=%u elapsed_us=%llu perf=%.2fMB/s\r\n",
              (unsigned int)input_len,
              (unsigned long long)elapsed_us,
              mbedtls_test_perf_mbps(input_len, 1U, elapsed_us));

    memset(plain, 0, input_len);
    memset(iv, 0x5A, sizeof(iv));

    mbedtls_aes_init(&ctx);
    start_us = bk_aon_rtc_get_us();
    ret = mbedtls_aes_setkey_dec(&ctx, key, 256);
    if (ret == 0) {
        ret = mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_DECRYPT, input_len, iv, cipher, plain);
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    mbedtls_aes_free(&ctx);
    if (ret != 0) {
        goto cleanup;
    }

    bk_printf("[xmbedtls] INFO aes_cbc_decrypt len=%u elapsed_us=%llu perf=%.2fMB/s\r\n",
              (unsigned int)input_len,
              (unsigned long long)elapsed_us,
              mbedtls_test_perf_mbps(input_len, 1U, elapsed_us));

    for (uint32_t i = 0; i < input_len; i++) {
        if (plain[i] != (unsigned char)(i & 0xFFU)) {
            ret = -1;
            break;
        }
    }

cleanup:
    if (plain != NULL) {
        os_free(plain);
    }
    if (cipher != NULL) {
        os_free(cipher);
    }
    return ret;
}

static int mbedtls_test_aes_ctr_impl(void)
{
    mbedtls_aes_context ctx;
    unsigned char buf[64];
    unsigned char nonce_counter[16];
    unsigned char stream_block[16];
    uint64_t start_us;
    uint64_t elapsed_us;
    int ret = 0;

    mbedtls_aes_init(&ctx);

    for (int i = 0; i < 6; i++) {
        int idx = i >> 1;
        int mode = i & 1;
        size_t offset = 0;
        int len = s_aes_test_ctr_len[idx];
        const unsigned char *expect;

        memcpy(nonce_counter, s_aes_test_ctr_nonce_counter[idx], sizeof(nonce_counter));
        memset(stream_block, 0, sizeof(stream_block));

        start_us = bk_aon_rtc_get_us();
        ret = mbedtls_aes_setkey_enc(&ctx, s_aes_test_ctr_key[idx], 128);
        if (ret != 0) {
            break;
        }

        if (mode == MBEDTLS_AES_DECRYPT) {
            memcpy(buf, s_aes_test_ctr_ct[idx], (size_t)len);
            expect = s_aes_test_ctr_pt[idx];
        } else {
            memcpy(buf, s_aes_test_ctr_pt[idx], (size_t)len);
            expect = s_aes_test_ctr_ct[idx];
        }

        ret = mbedtls_aes_crypt_ctr(&ctx, (size_t)len, &offset, nonce_counter, stream_block, buf, buf);
        if (ret != 0) {
            break;
        }

        elapsed_us = bk_aon_rtc_get_us() - start_us;
        bk_printf("[xmbedtls] INFO aes_ctr mode=%s len=%d elapsed_us=%llu perf=%.2fMB/s\r\n",
                  (mode == MBEDTLS_AES_DECRYPT) ? "dec" : "enc",
                  len,
                  (unsigned long long)elapsed_us,
                  mbedtls_test_perf_mbps((size_t)len, 1U, elapsed_us));

        if (memcmp(buf, expect, (size_t)len) != 0) {
            ret = -1;
            break;
        }
    }

    mbedtls_aes_free(&ctx);
    return ret;
}

static int mbedtls_test_aes_gcm_large_data(uint32_t input_len)
{
    mbedtls_gcm_context ctx;
    unsigned char key[MBEDTLS_GCM_KEY_BITS / 8U];
    unsigned char iv[MBEDTLS_GCM_IV_LEN];
    unsigned char aad[MBEDTLS_GCM_AD_LEN];
    unsigned char tag[MBEDTLS_GCM_TAG_LEN];
    unsigned char *plain = NULL;
    unsigned char *cipher = NULL;
    uint64_t start_us;
    uint64_t elapsed_us;
    int ret = 0;

    plain = os_malloc(input_len);
    cipher = os_malloc(input_len);
    if ((plain == NULL) || (cipher == NULL)) {
        ret = -1;
        goto cleanup;
    }

    mbedtls_test_fill_pattern(plain, input_len);
    memset(cipher, 0, input_len);
    memset(iv, 0x5A, sizeof(iv));
    memset(aad, 0x5A, sizeof(aad));
    mbedtls_test_fill_increment(key, sizeof(key));

    mbedtls_gcm_init(&ctx);
    start_us = bk_aon_rtc_get_us();
    ret = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, MBEDTLS_GCM_KEY_BITS);
    if (ret == 0) {
        ret = mbedtls_gcm_crypt_and_tag(&ctx,
                                        MBEDTLS_GCM_ENCRYPT,
                                        input_len,
                                        iv,
                                        sizeof(iv),
                                        aad,
                                        sizeof(aad),
                                        plain,
                                        cipher,
                                        sizeof(tag),
                                        tag);
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    mbedtls_gcm_free(&ctx);
    if (ret != 0) {
        goto cleanup;
    }

    bk_printf("[xmbedtls] INFO aes_gcm_encrypt len=%u elapsed_us=%llu perf=%.2fMB/s\r\n",
              (unsigned int)input_len,
              (unsigned long long)elapsed_us,
              mbedtls_test_perf_mbps(input_len, 1U, elapsed_us));

    memset(iv, 0x5A, sizeof(iv));
    memset(aad, 0x5A, sizeof(aad));
    memset(plain, 0, input_len);

    mbedtls_gcm_init(&ctx);
    start_us = bk_aon_rtc_get_us();
    ret = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, MBEDTLS_GCM_KEY_BITS);
    if (ret == 0) {
        ret = mbedtls_gcm_auth_decrypt(&ctx,
                                       input_len,
                                       iv,
                                       sizeof(iv),
                                       aad,
                                       sizeof(aad),
                                       tag,
                                       sizeof(tag),
                                       cipher,
                                       plain);
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    mbedtls_gcm_free(&ctx);
    if (ret != 0) {
        goto cleanup;
    }

    bk_printf("[xmbedtls] INFO aes_gcm_decrypt len=%u elapsed_us=%llu perf=%.2fMB/s\r\n",
              (unsigned int)input_len,
              (unsigned long long)elapsed_us,
              mbedtls_test_perf_mbps(input_len, 1U, elapsed_us));

    for (uint32_t i = 0; i < input_len; i++) {
        if (plain[i] != (unsigned char)(i & 0xFFU)) {
            ret = -1;
            break;
        }
    }

cleanup:
    if (plain != NULL) {
        os_free(plain);
    }
    if (cipher != NULL) {
        os_free(cipher);
    }
    return ret;
}

static int mbedtls_test_ecdsa_impl(int verbose, uint32_t loops)
{
    mbedtls_ecdsa_context sign_ctx;
    mbedtls_ecdsa_context verify_ctx;
    unsigned char digest[32];
    unsigned char sig[MBEDTLS_ECDSA_MAX_LEN];
    size_t sig_len = 0;
    uint64_t start_us;
    uint64_t elapsed_us;
    int ret = 0;

    if (loops == 0U) {
        loops = 1U;
    }

    ret = mbedtls_sha256(s_ecdsa_test_message, sizeof(s_ecdsa_test_message), digest, 0);
    if (ret != 0) {
        return ret;
    }

    mbedtls_ecdsa_init(&sign_ctx);
    mbedtls_ecdsa_init(&verify_ctx);
    memset(sig, 0, sizeof(sig));

    if (verbose != 0) {
        bk_printf("[xmbedtls] INFO ecdsa genkey begin loops=%u\r\n", (unsigned int)loops);
    }

    start_us = bk_aon_rtc_get_us();
    for (uint32_t i = 0; i < loops; i++) {
        ret = mbedtls_ecdsa_genkey(&sign_ctx, MBEDTLS_ECP_DP_SECP256R1, mbedtls_test_rng, NULL);
        if (ret != 0) {
            goto cleanup;
        }
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    bk_printf("[xmbedtls] INFO ecdsa_genkey loops=%u elapsed_us=%llu rate=%.2fops/s\r\n",
              (unsigned int)loops,
              (unsigned long long)elapsed_us,
              (elapsed_us == 0U) ? 0.0 : ((double)loops * 1000000.0 / (double)elapsed_us));

    start_us = bk_aon_rtc_get_us();
    for (uint32_t i = 0; i < loops; i++) {
        ret = mbedtls_ecdsa_write_signature(&sign_ctx,
                                            MBEDTLS_MD_SHA256,
                                            digest,
                                            sizeof(digest),
                                            sig,
                                            sizeof(sig),
                                            &sig_len,
                                            mbedtls_test_rng,
                                            NULL);
        if (ret != 0) {
            goto cleanup;
        }
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    bk_printf("[xmbedtls] INFO ecdsa_sign loops=%u elapsed_us=%llu rate=%.2fops/s\r\n",
              (unsigned int)loops,
              (unsigned long long)elapsed_us,
              (elapsed_us == 0U) ? 0.0 : ((double)loops * 1000000.0 / (double)elapsed_us));

    ret = mbedtls_ecp_group_copy(&verify_ctx.MBEDTLS_PRIVATE(grp), &sign_ctx.MBEDTLS_PRIVATE(grp));
    if (ret != 0) {
        goto cleanup;
    }
    ret = mbedtls_ecp_copy(&verify_ctx.MBEDTLS_PRIVATE(Q), &sign_ctx.MBEDTLS_PRIVATE(Q));
    if (ret != 0) {
        goto cleanup;
    }

    start_us = bk_aon_rtc_get_us();
    for (uint32_t i = 0; i < loops; i++) {
        ret = mbedtls_ecdsa_read_signature(&verify_ctx, digest, sizeof(digest), sig, sig_len);
        if (ret != 0) {
            goto cleanup;
        }
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    bk_printf("[xmbedtls] INFO ecdsa_verify loops=%u elapsed_us=%llu rate=%.2fops/s\r\n",
              (unsigned int)loops,
              (unsigned long long)elapsed_us,
              (elapsed_us == 0U) ? 0.0 : ((double)loops * 1000000.0 / (double)elapsed_us));

cleanup:
    mbedtls_ecdsa_free(&verify_ctx);
    mbedtls_ecdsa_free(&sign_ctx);
    return ret;
}

static int mbedtls_test_rsa_impl(int verbose)
{
    mbedtls_rsa_context rsa;
    mbedtls_mpi k;
    unsigned char decrypted[MBEDTLS_RSA_PT_LEN];
    unsigned char ciphertext[MBEDTLS_RSA_KEY_LEN];
    unsigned char sha1sum[20];
    size_t decrypted_len = 0;
    uint64_t start_us;
    uint64_t elapsed_us;
    int ret = 0;

    mbedtls_mpi_init(&k);
    mbedtls_rsa_init(&rsa);

    ret = mbedtls_mpi_read_string(&k, 16, s_rsa_n);
    if (ret == 0) {
        ret = mbedtls_rsa_import(&rsa, &k, NULL, NULL, NULL, NULL);
    }
    if (ret == 0) {
        ret = mbedtls_mpi_read_string(&k, 16, s_rsa_p);
    }
    if (ret == 0) {
        ret = mbedtls_rsa_import(&rsa, NULL, &k, NULL, NULL, NULL);
    }
    if (ret == 0) {
        ret = mbedtls_mpi_read_string(&k, 16, s_rsa_q);
    }
    if (ret == 0) {
        ret = mbedtls_rsa_import(&rsa, NULL, NULL, &k, NULL, NULL);
    }
    if (ret == 0) {
        ret = mbedtls_mpi_read_string(&k, 16, s_rsa_d);
    }
    if (ret == 0) {
        ret = mbedtls_rsa_import(&rsa, NULL, NULL, NULL, &k, NULL);
    }
    if (ret == 0) {
        ret = mbedtls_mpi_read_string(&k, 16, s_rsa_e);
    }
    if (ret == 0) {
        ret = mbedtls_rsa_import(&rsa, NULL, NULL, NULL, NULL, &k);
    }
    if (ret == 0) {
        ret = mbedtls_rsa_complete(&rsa);
    }
    if (ret != 0) {
        goto cleanup;
    }

    if ((mbedtls_rsa_check_pubkey(&rsa) != 0) || (mbedtls_rsa_check_privkey(&rsa) != 0)) {
        ret = -1;
        goto cleanup;
    }

    if (verbose != 0) {
        bk_printf("[xmbedtls] INFO rsa key validation ok\r\n");
    }

    start_us = bk_aon_rtc_get_us();
    ret = mbedtls_rsa_pkcs1_encrypt(&rsa, mbedtls_test_rng, NULL, sizeof(s_rsa_pt), s_rsa_pt, ciphertext);
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    if (ret != 0) {
        goto cleanup;
    }
    bk_printf("[xmbedtls] INFO rsa_encrypt len=%u elapsed_us=%llu\r\n",
              (unsigned int)sizeof(s_rsa_pt),
              (unsigned long long)elapsed_us);

    start_us = bk_aon_rtc_get_us();
    ret = mbedtls_rsa_pkcs1_decrypt(&rsa,
                                    mbedtls_test_rng,
                                    NULL,
                                    &decrypted_len,
                                    ciphertext,
                                    decrypted,
                                    sizeof(decrypted));
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    if (ret != 0) {
        goto cleanup;
    }
    bk_printf("[xmbedtls] INFO rsa_decrypt len=%u elapsed_us=%llu\r\n",
              (unsigned int)decrypted_len,
              (unsigned long long)elapsed_us);

    if ((decrypted_len != sizeof(s_rsa_pt)) || (memcmp(decrypted, s_rsa_pt, sizeof(s_rsa_pt)) != 0)) {
        ret = -1;
        goto cleanup;
    }

    ret = mbedtls_sha1(s_rsa_pt, sizeof(s_rsa_pt), sha1sum);
    if (ret != 0) {
        goto cleanup;
    }

    start_us = bk_aon_rtc_get_us();
    ret = mbedtls_rsa_pkcs1_sign(&rsa, mbedtls_test_rng, NULL, MBEDTLS_MD_SHA1, 20U, sha1sum, ciphertext);
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    if (ret != 0) {
        goto cleanup;
    }
    bk_printf("[xmbedtls] INFO rsa_sign elapsed_us=%llu\r\n", (unsigned long long)elapsed_us);

    start_us = bk_aon_rtc_get_us();
    ret = mbedtls_rsa_pkcs1_verify(&rsa, MBEDTLS_MD_SHA1, 20U, sha1sum, ciphertext);
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    if (ret != 0) {
        goto cleanup;
    }
    bk_printf("[xmbedtls] INFO rsa_verify elapsed_us=%llu\r\n", (unsigned long long)elapsed_us);

cleanup:
    mbedtls_rsa_free(&rsa);
    mbedtls_mpi_free(&k);
    return ret;
}

static int mbedtls_test_tls_server_certificate_impl(uint32_t loops)
{
    mbedtls_x509_crt cacert;
    mbedtls_x509_crt srvcert;
    unsigned char *ca_pem = NULL;
    unsigned char *srv_pem = NULL;
    size_t ca_pem_len = 0;
    size_t srv_pem_len = 0;
    uint32_t flags = 0;
    uint64_t start_us;
    uint64_t elapsed_us;
    uint64_t parse_us = 0;
    uint64_t verify_us = 0;
    int ret = 0;

    if (loops == 0U) {
        loops = 1U;
    }

    mbedtls_x509_crt_init(&cacert);
    mbedtls_x509_crt_init(&srvcert);

    /*
     * Match real cloud path: present leaf + 1 intermediate, trust next CA.
     * Forces ~2 ECDSA signature verifications (was 1 when full chain was also CA).
     */
    ret = mbedtls_test_pem_dup_range(s_tls_test_chain_pem, 2U, 2U, &ca_pem, &ca_pem_len);
    if (ret != 0) {
        goto cleanup;
    }
    ret = mbedtls_test_pem_dup_range(s_tls_test_chain_pem, 0U, 1U, &srv_pem, &srv_pem_len);
    if (ret != 0) {
        goto cleanup;
    }

    ret = mbedtls_x509_crt_parse(&cacert, ca_pem, ca_pem_len);
    if (ret != 0) {
        goto cleanup;
    }

    bk_printf("[xmbedtls] INFO tls_server_certificate setup ca_pem_len=%u srv_pem_len=%u (leaf+1int / ca)\r\n",
              (unsigned int)ca_pem_len,
              (unsigned int)srv_pem_len);

    start_us = bk_aon_rtc_get_us();
    for (uint32_t i = 0; i < loops; i++) {
        uint64_t step_us;

        mbedtls_x509_crt_free(&srvcert);
        mbedtls_x509_crt_init(&srvcert);
        flags = 0;

        step_us = bk_aon_rtc_get_us();
        ret = mbedtls_x509_crt_parse(&srvcert, srv_pem, srv_pem_len);
        parse_us += bk_aon_rtc_get_us() - step_us;
        if (ret != 0) {
            goto cleanup;
        }

        step_us = bk_aon_rtc_get_us();
        ret = mbedtls_x509_crt_verify(&srvcert, &cacert, NULL, NULL, &flags, NULL, NULL);
        verify_us += bk_aon_rtc_get_us() - step_us;
        if (ret != 0) {
            goto cleanup;
        }
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;

    bk_printf("[xmbedtls] INFO tls_server_certificate loops=%u elapsed_us=%llu rate=%.2fops/s\r\n",
              (unsigned int)loops,
              (unsigned long long)elapsed_us,
              mbedtls_test_perf_ops(loops, elapsed_us));
    bk_printf("[xmbedtls] INFO tls_server_certificate_sub loops=%u parse_us=%llu verify_us=%llu srv_pem_len=%u ca_pem_len=%u\r\n",
              (unsigned int)loops,
              (unsigned long long)parse_us,
              (unsigned long long)verify_us,
              (unsigned int)srv_pem_len,
              (unsigned int)ca_pem_len);

cleanup:
    if ((ret != 0) && (flags != 0U)) {
        char vrfy_buf[192];

        mbedtls_x509_crt_verify_info(vrfy_buf, sizeof(vrfy_buf), "", flags);
        bk_printf("[xmbedtls] ERROR tls_server_certificate flags=0x%08x %s\r\n",
                  (unsigned int)flags,
                  vrfy_buf);
    }

    mbedtls_x509_crt_free(&srvcert);
    mbedtls_x509_crt_free(&cacert);
    if (srv_pem != NULL) {
        os_free(srv_pem);
    }
    if (ca_pem != NULL) {
        os_free(ca_pem);
    }
    return ret;
}

static int mbedtls_test_tls_server_key_exchange_impl(uint32_t loops)
{
    mbedtls_x509_crt srvcert;
    mbedtls_pk_context sign_key;
    mbedtls_ecdh_context server_ecdh;
    unsigned char client_random[MBEDTLS_TLS_RANDOM_LEN];
    unsigned char server_random[MBEDTLS_TLS_RANDOM_LEN];
    unsigned char params[MBEDTLS_TLS_PARAMS_BUF_LEN];
    unsigned char signed_data[(MBEDTLS_TLS_RANDOM_LEN * 2U) + MBEDTLS_TLS_PARAMS_BUF_LEN];
    unsigned char hash[32];
    unsigned char sig[MBEDTLS_PK_SIGNATURE_MAX_SIZE];
    size_t params_len = 0;
    size_t sig_len = 0;
    uint64_t start_us;
    uint64_t elapsed_us;
    uint64_t params_parse_us = 0;
    uint64_t hash_us = 0;
    uint64_t sig_verify_us = 0;
    int ret = 0;

    if (loops == 0U) {
        loops = 1U;
    }

    mbedtls_x509_crt_init(&srvcert);
    mbedtls_pk_init(&sign_key);
    mbedtls_ecdh_init(&server_ecdh);
    memset(sig, 0, sizeof(sig));
    mbedtls_test_fill_increment(client_random, sizeof(client_random));
    mbedtls_test_fill_pattern(server_random, sizeof(server_random));

    ret = mbedtls_x509_crt_parse(&srvcert,
                                 (const unsigned char *)s_tls_test_chain_pem,
                                 sizeof(s_tls_test_chain_pem));
    if (ret != 0) {
        goto cleanup;
    }

    ret = mbedtls_pk_parse_key(&sign_key,
                               (const unsigned char *)s_tls_test_key_pem,
                               sizeof(s_tls_test_key_pem),
                               NULL,
                               0,
                               mbedtls_test_tls_rng,
                               NULL);
    if (ret != 0) {
        goto cleanup;
    }

    ret = mbedtls_ecdh_setup(&server_ecdh, MBEDTLS_TLS_TEST_ECDHE_CURVE);
    if (ret != 0) {
        goto cleanup;
    }

    ret = mbedtls_ecdh_make_params(&server_ecdh,
                                   &params_len,
                                   params,
                                   sizeof(params),
                                   mbedtls_test_tls_rng,
                                   NULL);
    if (ret != 0) {
        goto cleanup;
    }

    memcpy(signed_data, client_random, sizeof(client_random));
    memcpy(signed_data + sizeof(client_random), server_random, sizeof(server_random));
    memcpy(signed_data + sizeof(client_random) + sizeof(server_random), params, params_len);

    ret = mbedtls_sha256(signed_data, (sizeof(client_random) + sizeof(server_random) + params_len), hash, 0);
    if (ret != 0) {
        goto cleanup;
    }

    ret = mbedtls_pk_sign(&sign_key,
                          MBEDTLS_MD_SHA256,
                          hash,
                          0,
                          sig,
                          sizeof(sig),
                          &sig_len,
                          mbedtls_test_tls_rng,
                          NULL);
    if (ret != 0) {
        goto cleanup;
    }

    start_us = bk_aon_rtc_get_us();
    for (uint32_t i = 0; i < loops; i++) {
        mbedtls_ecdh_context client_ecdh;
        const unsigned char *p = params;
        uint64_t step_us;

        mbedtls_ecdh_init(&client_ecdh);

        step_us = bk_aon_rtc_get_us();
        ret = mbedtls_ecdh_read_params(&client_ecdh, &p, params + params_len);
        params_parse_us += bk_aon_rtc_get_us() - step_us;
        if ((ret == 0) && (p != (params + params_len))) {
            ret = -1;
        }
        if (ret == 0) {
            step_us = bk_aon_rtc_get_us();
            ret = mbedtls_sha256(signed_data,
                                 (sizeof(client_random) + sizeof(server_random) + params_len),
                                 hash,
                                 0);
            hash_us += bk_aon_rtc_get_us() - step_us;
        }
        if (ret == 0) {
            step_us = bk_aon_rtc_get_us();
            ret = mbedtls_pk_verify(&srvcert.pk, MBEDTLS_MD_SHA256, hash, 0, sig, sig_len);
            sig_verify_us += bk_aon_rtc_get_us() - step_us;
        }

        mbedtls_ecdh_free(&client_ecdh);
        if (ret != 0) {
            goto cleanup;
        }
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;

    bk_printf("[xmbedtls] INFO tls_server_key_exchange loops=%u elapsed_us=%llu rate=%.2fops/s ecdhe=%s\r\n",
              (unsigned int)loops,
              (unsigned long long)elapsed_us,
              mbedtls_test_perf_ops(loops, elapsed_us),
              MBEDTLS_TLS_TEST_ECDHE_NAME);
    bk_printf("[xmbedtls] INFO tls_server_key_exchange_sub loops=%u params_parse_us=%llu hash_us=%llu sig_verify_us=%llu params_len=%u signed_len=%u\r\n",
              (unsigned int)loops,
              (unsigned long long)params_parse_us,
              (unsigned long long)hash_us,
              (unsigned long long)sig_verify_us,
              (unsigned int)params_len,
              (unsigned int)(sizeof(client_random) + sizeof(server_random) + params_len));

cleanup:
    mbedtls_ecdh_free(&server_ecdh);
    mbedtls_pk_free(&sign_key);
    mbedtls_x509_crt_free(&srvcert);
    return ret;
}

static int mbedtls_test_tls_client_key_exchange_impl(uint32_t loops)
{
    mbedtls_ecp_keypair server_key;
    unsigned char client_pub[MBEDTLS_TLS_KEY_BUF_LEN];
    unsigned char client_secret[MBEDTLS_MPI_MAX_SIZE];
    unsigned char server_secret[MBEDTLS_MPI_MAX_SIZE];
    size_t client_pub_len = 0;
    size_t client_secret_len = 0;
    size_t server_secret_len = 0;
    uint64_t start_us;
    uint64_t elapsed_us;
    uint64_t local_make_pub_us = 0;
    uint64_t local_secret_us = 0;
    int ret = 0;

    if (loops == 0U) {
        loops = 1U;
    }

    mbedtls_ecp_keypair_init(&server_key);
    memset(client_pub, 0, sizeof(client_pub));
    memset(client_secret, 0, sizeof(client_secret));
    memset(server_secret, 0, sizeof(server_secret));

    s_mbedtls_tls_rng_us = 0;
    ret = mbedtls_ecp_gen_key(MBEDTLS_TLS_TEST_ECDHE_CURVE, &server_key, mbedtls_test_tls_rng, NULL);
    if (ret != 0) {
        goto cleanup;
    }

    /* Reset RNG accumulator: only count RNG inside the timed client ECDHE loop. */
    s_mbedtls_tls_rng_us = 0;
    start_us = bk_aon_rtc_get_us();
    for (uint32_t i = 0; i < loops; i++) {
        mbedtls_ecdh_context client_ecdh;
        uint64_t step_us;

        mbedtls_ecdh_init(&client_ecdh);

        ret = mbedtls_ecdh_get_params(&client_ecdh, &server_key, MBEDTLS_ECDH_THEIRS);
        if (ret == 0) {
            step_us = bk_aon_rtc_get_us();
            ret = mbedtls_ecdh_make_public(&client_ecdh,
                                           &client_pub_len,
                                           client_pub,
                                           sizeof(client_pub),
                                           mbedtls_test_tls_rng,
                                           NULL);
            local_make_pub_us += bk_aon_rtc_get_us() - step_us;
        }
        if (ret == 0) {
            step_us = bk_aon_rtc_get_us();
            ret = mbedtls_ecdh_calc_secret(&client_ecdh,
                                           &client_secret_len,
                                           client_secret,
                                           sizeof(client_secret),
                                           mbedtls_test_tls_rng,
                                           NULL);
            local_secret_us += bk_aon_rtc_get_us() - step_us;
        }

        mbedtls_ecdh_free(&client_ecdh);
        if (ret != 0) {
            goto cleanup;
        }
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;

    bk_printf("[xmbedtls] INFO tls_client_key_exchange loops=%u elapsed_us=%llu rate=%.2fops/s ecdhe=%s\r\n",
              (unsigned int)loops,
              (unsigned long long)elapsed_us,
              mbedtls_test_perf_ops(loops, elapsed_us),
              MBEDTLS_TLS_TEST_ECDHE_NAME);
    bk_printf("[xmbedtls] INFO tls_client_key_exchange_sub loops=%u local_make_pub_us=%llu local_secret_us=%llu rng_us=%llu pub_len=%u secret_len=%u\r\n",
              (unsigned int)loops,
              (unsigned long long)local_make_pub_us,
              (unsigned long long)local_secret_us,
              (unsigned long long)s_mbedtls_tls_rng_us,
              (unsigned int)client_pub_len,
              (unsigned int)client_secret_len);

    if ((ret == 0) && (client_pub_len > 0U)) {
        mbedtls_ecdh_context server_ecdh;

        mbedtls_ecdh_init(&server_ecdh);
        ret = mbedtls_ecdh_get_params(&server_ecdh, &server_key, MBEDTLS_ECDH_OURS);
        if (ret == 0) {
            ret = mbedtls_ecdh_read_public(&server_ecdh, client_pub, client_pub_len);
        }
        if (ret == 0) {
            ret = mbedtls_ecdh_calc_secret(&server_ecdh,
                                           &server_secret_len,
                                           server_secret,
                                           sizeof(server_secret),
                                           mbedtls_test_tls_rng,
                                           NULL);
        }
        if ((ret == 0) &&
            ((server_secret_len != client_secret_len) ||
             (memcmp(server_secret, client_secret, client_secret_len) != 0))) {
            ret = -1;
        }
        mbedtls_ecdh_free(&server_ecdh);
    }

cleanup:
    mbedtls_ecp_keypair_free(&server_key);
    return ret;
}

/**
 * @brief TLS record-layer AES-128-GCM encrypt/decrypt for one plaintext length
 * @param[in] input_len plaintext length in bytes
 * @return 0 on success, negative on failure
 * @note Mimics TLS 1.2 AAD layout (13 bytes) and preferred suite AES_128_GCM.
 */
static int mbedtls_test_tls_record_aes_gcm_once(uint32_t input_len)
{
    mbedtls_gcm_context ctx;
    unsigned char key[MBEDTLS_TLS_GCM_KEY_BITS / 8U];
    unsigned char iv[MBEDTLS_GCM_IV_LEN];
    unsigned char aad[MBEDTLS_TLS_GCM_AAD_LEN];
    unsigned char tag[MBEDTLS_GCM_TAG_LEN];
    unsigned char *plain = NULL;
    unsigned char *cipher = NULL;
    uint64_t start_us;
    uint64_t elapsed_us;
    int ret = 0;

    plain = os_malloc(input_len);
    cipher = os_malloc(input_len);
    if ((plain == NULL) || (cipher == NULL)) {
        ret = -1;
        goto cleanup;
    }

    mbedtls_test_fill_pattern(plain, input_len);
    memset(cipher, 0, input_len);
    memset(iv, 0xA5, sizeof(iv));
    /* Fake TLS 1.2 record AAD: seq_num | type | version | length */
    memset(aad, 0x00, sizeof(aad));
    aad[8] = 0x17;  /* application_data */
    aad[9] = 0x03;
    aad[10] = 0x03;
    aad[11] = (unsigned char)((input_len >> 8) & 0xFFU);
    aad[12] = (unsigned char)(input_len & 0xFFU);
    mbedtls_test_fill_increment(key, sizeof(key));

    mbedtls_gcm_init(&ctx);
    start_us = bk_aon_rtc_get_us();
    ret = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, MBEDTLS_TLS_GCM_KEY_BITS);
    if (ret == 0) {
        ret = mbedtls_gcm_crypt_and_tag(&ctx,
                                        MBEDTLS_GCM_ENCRYPT,
                                        input_len,
                                        iv,
                                        sizeof(iv),
                                        aad,
                                        sizeof(aad),
                                        plain,
                                        cipher,
                                        sizeof(tag),
                                        tag);
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    mbedtls_gcm_free(&ctx);
    if (ret != 0) {
        goto cleanup;
    }

    bk_printf("[xmbedtls] INFO tls_record_aes128_gcm_encrypt len=%u elapsed_us=%llu perf=%.2fMB/s\r\n",
              (unsigned int)input_len,
              (unsigned long long)elapsed_us,
              mbedtls_test_perf_mbps(input_len, 1U, elapsed_us));

    memset(plain, 0, input_len);

    mbedtls_gcm_init(&ctx);
    start_us = bk_aon_rtc_get_us();
    ret = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, MBEDTLS_TLS_GCM_KEY_BITS);
    if (ret == 0) {
        ret = mbedtls_gcm_auth_decrypt(&ctx,
                                       input_len,
                                       iv,
                                       sizeof(iv),
                                       aad,
                                       sizeof(aad),
                                       tag,
                                       sizeof(tag),
                                       cipher,
                                       plain);
    }
    elapsed_us = bk_aon_rtc_get_us() - start_us;
    mbedtls_gcm_free(&ctx);
    if (ret != 0) {
        goto cleanup;
    }

    bk_printf("[xmbedtls] INFO tls_record_aes128_gcm_decrypt len=%u elapsed_us=%llu perf=%.2fMB/s\r\n",
              (unsigned int)input_len,
              (unsigned long long)elapsed_us,
              mbedtls_test_perf_mbps(input_len, 1U, elapsed_us));

    for (uint32_t i = 0; i < input_len; i++) {
        if (plain[i] != (unsigned char)(i & 0xFFU)) {
            ret = -1;
            break;
        }
    }

cleanup:
    if (plain != NULL) {
        os_free(plain);
    }
    if (cipher != NULL) {
        os_free(cipher);
    }
    return ret;
}

/**
 * @brief Run TLS record AES-128-GCM cases for all configured lengths
 * @return 0 on success, negative on first failure
 */
static int mbedtls_test_tls_record_aes_gcm_impl(void)
{
    int ret = 0;

    bk_printf("[xmbedtls] INFO tls_record_aes128_gcm begin keybits=%u aad_len=%u iv_len=%u\r\n",
              (unsigned int)MBEDTLS_TLS_GCM_KEY_BITS,
              (unsigned int)MBEDTLS_TLS_GCM_AAD_LEN,
              (unsigned int)MBEDTLS_GCM_IV_LEN);

    for (size_t i = 0; i < (sizeof(s_mbedtls_tls_record_len) / sizeof(s_mbedtls_tls_record_len[0])); i++) {
        ret = mbedtls_test_tls_record_aes_gcm_once(s_mbedtls_tls_record_len[i]);
        if (ret != 0) {
            bk_printf("[xmbedtls] ERROR tls_record_aes128_gcm len=%u ret=%d\r\n",
                      (unsigned int)s_mbedtls_tls_record_len[i],
                      ret);
            break;
        }
    }

    return ret;
}

static void mbedtls_test_random_delay(void)
{
    unsigned char delay = 0;

    if (mbedtls_test_rng(NULL, &delay, sizeof(delay)) != 0) {
        return;
    }

    rtos_delay_milliseconds(delay % MBEDTLS_THREAD_MAX_DELAY_MS);
}

static int mbedtls_test_thread_symmetric_once(void)
{
    mbedtls_aes_context ctx;
    unsigned char key[32];
    unsigned char input[32];
    unsigned char output[32];
    unsigned char iv[16];
    int ret = 0;

    memset(key, 0xA5, sizeof(key));
    memset(iv, 0x5A, sizeof(iv));
    mbedtls_test_fill_pattern(input, sizeof(input));

    mbedtls_aes_init(&ctx);
    ret = mbedtls_aes_setkey_enc(&ctx, key, 256);
    if (ret == 0) {
        mbedtls_test_random_delay();
        ret = mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_ENCRYPT, sizeof(input), iv, input, output);
    }
    mbedtls_aes_free(&ctx);
    if (ret != 0) {
        return ret;
    }

    memset(iv, 0x5A, sizeof(iv));
    memset(input, 0, sizeof(input));

    mbedtls_aes_init(&ctx);
    ret = mbedtls_aes_setkey_dec(&ctx, key, 256);
    if (ret == 0) {
        ret = mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_DECRYPT, sizeof(input), iv, output, input);
    }
    mbedtls_test_random_delay();
    mbedtls_aes_free(&ctx);
    if (ret != 0) {
        return ret;
    }

    for (size_t i = 0; i < sizeof(input); i++) {
        if (input[i] != (unsigned char)(i & 0xFFU)) {
            return -1;
        }
    }

    return 0;
}

static int mbedtls_test_thread_asymmetric_once(void)
{
    mbedtls_ecdsa_context sign_ctx;
    mbedtls_ecdsa_context verify_ctx;
    unsigned char digest[32];
    unsigned char sig[MBEDTLS_ECDSA_MAX_LEN];
    size_t sig_len = 0;
    int ret;

    ret = mbedtls_sha256(s_ecdsa_test_message, sizeof(s_ecdsa_test_message), digest, 0);
    if (ret != 0) {
        return ret;
    }

    memset(sig, 0, sizeof(sig));
    mbedtls_ecdsa_init(&sign_ctx);
    mbedtls_ecdsa_init(&verify_ctx);

    mbedtls_test_random_delay();
    ret = mbedtls_ecdsa_genkey(&sign_ctx, MBEDTLS_ECP_DP_SECP256R1, mbedtls_test_rng, NULL);
    if (ret == 0) {
        ret = mbedtls_ecdsa_write_signature(&sign_ctx,
                                            MBEDTLS_MD_SHA256,
                                            digest,
                                            sizeof(digest),
                                            sig,
                                            sizeof(sig),
                                            &sig_len,
                                            mbedtls_test_rng,
                                            NULL);
    }
    if (ret == 0) {
        ret = mbedtls_ecp_group_copy(&verify_ctx.MBEDTLS_PRIVATE(grp), &sign_ctx.MBEDTLS_PRIVATE(grp));
    }
    if (ret == 0) {
        ret = mbedtls_ecp_copy(&verify_ctx.MBEDTLS_PRIVATE(Q), &sign_ctx.MBEDTLS_PRIVATE(Q));
    }
    if (ret == 0) {
        mbedtls_test_random_delay();
        ret = mbedtls_ecdsa_read_signature(&verify_ctx, digest, sizeof(digest), sig, sig_len);
    }

    mbedtls_ecdsa_free(&verify_ctx);
    mbedtls_ecdsa_free(&sign_ctx);
    return ret;
}

static void te200_muti_task_test(void *arg)
{
    uint32_t count = 0;
    uint32_t max_count = (arg == NULL) ? 0U : *(uint32_t *)arg;

    while (1) {
        if (mbedtls_test_thread_symmetric_once() != 0) {
            break;
        }
        if (mbedtls_test_thread_asymmetric_once() != 0) {
            break;
        }

        if (++count > max_count) {
            bk_printf("[xmbedtls] INFO thread_worker completed count=%u\r\n", (unsigned int)count);
            break;
        }
    }

    rtos_delete_thread(NULL);
}

static int mbedtls_test_selftest_sha256(int verbose)
{
    int ret = 0;

    (void)verbose;

    for (size_t i = 0; i < (sizeof(s_mbedtls_test_len) / sizeof(s_mbedtls_test_len[0])); i++) {
        ret = mbedtls_test_sha256_loop_test(s_mbedtls_test_len[i], MBEDTLS_SHA_TEST_CNT);
        if (ret != 0) {
            return ret;
        }
    }

    return 0;
}

static int mbedtls_test_selftest_sha512(int verbose)
{
    int ret = 0;

    (void)verbose;

    for (size_t i = 0; i < (sizeof(s_mbedtls_test_len) / sizeof(s_mbedtls_test_len[0])); i++) {
        ret = mbedtls_test_sha512_loop_test(s_mbedtls_test_len[i], MBEDTLS_SHA_TEST_CNT);
        if (ret != 0) {
            return ret;
        }
    }

    return 0;
}

static int mbedtls_test_selftest_aes(int verbose)
{
    int ret = 0;

    (void)verbose;

    ret = mbedtls_test_aes_ecb_impl();
    if (ret != 0) {
        return ret;
    }

    ret = mbedtls_test_aes_cbc_impl();
    if (ret != 0) {
        return ret;
    }

    for (size_t i = 0; i < (sizeof(s_mbedtls_test_len) / sizeof(s_mbedtls_test_len[0])); i++) {
        ret = mbedtls_test_aes_cbc_large_data(s_mbedtls_test_len[i]);
        if (ret != 0) {
            return ret;
        }
    }

    ret = mbedtls_test_aes_ctr_impl();
    if (ret != 0) {
        return ret;
    }

    for (size_t i = 0; i < (sizeof(s_mbedtls_test_len) / sizeof(s_mbedtls_test_len[0])); i++) {
        ret = mbedtls_test_aes_gcm_large_data(s_mbedtls_test_len[i]);
        if (ret != 0) {
            return ret;
        }
    }

    return 0;
}

static int mbedtls_test_selftest_ecdsa(int verbose)
{
    return mbedtls_test_ecdsa_impl(verbose, 1U);
}

static int mbedtls_test_selftest_rsa(int verbose)
{
    return mbedtls_test_rsa_impl(verbose);
}

static const mbedtls_selftest_case_t s_mbedtls_selftests[] = {
    {"aes", mbedtls_test_selftest_aes},
    {"ecdsa", mbedtls_test_selftest_ecdsa},
    {"rsa", mbedtls_test_selftest_rsa},
    {"sha256", mbedtls_test_selftest_sha256},
    {"sha512", mbedtls_test_selftest_sha512}
};

static const mbedtls_selftest_case_t *mbedtls_test_find_selftest(const char *name)
{
    size_t i;

    if (name == NULL) {
        return NULL;
    }

    for (i = 0; i < (sizeof(s_mbedtls_selftests) / sizeof(s_mbedtls_selftests[0])); i++) {
        if (os_strcmp(name, s_mbedtls_selftests[i].name) == 0) {
            return &s_mbedtls_selftests[i];
        }
    }

    return NULL;
}

static void mbedtls_test_sha_cmd(int argc, char **argv)
{
    mbedtls_test_stat_t st = {0};
    char case_name[32];
    size_t i;

    if ((argc < 3) || (argv[2] == NULL)) {
        mbedtls_test_usage();
        return;
    }

    mbedtls_test_vote_cpu_freq();

    if (os_strcmp(argv[2], "256") == 0) {
        for (i = 0; i < (sizeof(s_mbedtls_test_len) / sizeof(s_mbedtls_test_len[0])); i++) {
            int ret = mbedtls_test_sha256_loop_test(s_mbedtls_test_len[i], MBEDTLS_SHA_TEST_CNT);
            (void)snprintf(case_name, sizeof(case_name), "sha256_len_%u", (unsigned int)s_mbedtls_test_len[i]);
            mbedtls_test_log_result(&st, case_name, ret);
        }
    } else if (os_strcmp(argv[2], "512") == 0) {
        for (i = 0; i < (sizeof(s_mbedtls_test_len) / sizeof(s_mbedtls_test_len[0])); i++) {
            int ret = mbedtls_test_sha512_loop_test(s_mbedtls_test_len[i], MBEDTLS_SHA_TEST_CNT);
            (void)snprintf(case_name, sizeof(case_name), "sha512_len_%u", (unsigned int)s_mbedtls_test_len[i]);
            mbedtls_test_log_result(&st, case_name, ret);
        }
    } else {
        mbedtls_test_restore_cpu_freq();
        mbedtls_test_usage();
        return;
    }

    mbedtls_test_restore_cpu_freq();
    mbedtls_test_log_summary("sha", &st);
}

static void mbedtls_test_aes_cmd(int argc, char **argv)
{
    mbedtls_test_stat_t st = {0};
    char case_name[32];
    size_t i;

    if ((argc < 3) || (argv[2] == NULL)) {
        mbedtls_test_usage();
        return;
    }

    mbedtls_test_vote_cpu_freq();

    if (os_strcmp(argv[2], "ecb") == 0) {
        mbedtls_test_log_result(&st, "aes_ecb", mbedtls_test_aes_ecb_impl());
    } else if (os_strcmp(argv[2], "cbc") == 0) {
        mbedtls_test_log_result(&st, "aes_cbc", mbedtls_test_aes_cbc_impl());
        for (i = 0; i < (sizeof(s_mbedtls_test_len) / sizeof(s_mbedtls_test_len[0])); i++) {
            int ret = mbedtls_test_aes_cbc_large_data(s_mbedtls_test_len[i]);
            (void)snprintf(case_name, sizeof(case_name), "aes_cbc_len_%u", (unsigned int)s_mbedtls_test_len[i]);
            mbedtls_test_log_result(&st, case_name, ret);
        }
    } else if (os_strcmp(argv[2], "ctr") == 0) {
        mbedtls_test_log_result(&st, "aes_ctr", mbedtls_test_aes_ctr_impl());
    } else if (os_strcmp(argv[2], "gcm") == 0) {
        for (i = 0; i < (sizeof(s_mbedtls_test_len) / sizeof(s_mbedtls_test_len[0])); i++) {
            int ret = mbedtls_test_aes_gcm_large_data(s_mbedtls_test_len[i]);
            (void)snprintf(case_name, sizeof(case_name), "aes_gcm_len_%u", (unsigned int)s_mbedtls_test_len[i]);
            mbedtls_test_log_result(&st, case_name, ret);
        }
    } else {
        mbedtls_test_restore_cpu_freq();
        mbedtls_test_usage();
        return;
    }

    mbedtls_test_restore_cpu_freq();
    mbedtls_test_log_summary("aes", &st);
}

static void mbedtls_test_ecdsa_cmd(int argc, char **argv)
{
    mbedtls_test_stat_t st = {0};
    uint32_t loop_cnt = 1U;

    if ((argc >= 3) && (argv[2] != NULL)) {
        loop_cnt = os_strtoul(argv[2], NULL, 10);
    }

    mbedtls_test_vote_cpu_freq();
    mbedtls_test_log_result(&st, "ecdsa", mbedtls_test_ecdsa_impl(1, loop_cnt));
    mbedtls_test_restore_cpu_freq();
    mbedtls_test_log_summary("ecdsa", &st);
}

void mbedtls_test_tls_cmd(int argc, char **argv)
{
    mbedtls_test_stat_t st = {0};
    uint32_t loop_cnt = 1U;

    if ((argc >= 3) && (argv[2] != NULL)) {
        loop_cnt = os_strtoul(argv[2], NULL, 10);
    }

    bk_printf("[xmbedtls] INFO tls hotspot begin loops=%u (HS + AES-128-GCM record, ecdhe=%s)\r\n",
              (unsigned int)loop_cnt,
              MBEDTLS_TLS_TEST_ECDHE_NAME);

    mbedtls_test_vote_cpu_freq();
    mbedtls_test_log_result(&st, "tls_server_certificate", mbedtls_test_tls_server_certificate_impl(loop_cnt));
    mbedtls_test_log_result(&st, "tls_server_key_exchange", mbedtls_test_tls_server_key_exchange_impl(loop_cnt));
    mbedtls_test_log_result(&st, "tls_client_key_exchange", mbedtls_test_tls_client_key_exchange_impl(loop_cnt));
    mbedtls_test_log_result(&st, "tls_record_aes128_gcm", mbedtls_test_tls_record_aes_gcm_impl());
    mbedtls_test_restore_cpu_freq();
    mbedtls_test_log_summary("tls", &st);
}

static void mbedtls_test_rsa_cmd(void)
{
    mbedtls_test_stat_t st = {0};

    mbedtls_test_vote_cpu_freq();
    mbedtls_test_log_result(&st, "rsa", mbedtls_test_rsa_impl(1));
    mbedtls_test_restore_cpu_freq();
    mbedtls_test_log_summary("rsa", &st);
}

static int mbedtls_test_run_selftest(int argc, char **argv)
{
    mbedtls_test_stat_t st = {0};
    const mbedtls_selftest_case_t *selftest = NULL;
    int verbose = 1;
    int selected_cnt = 0;
    int ret = 0;

    for (int i = 2; i < argc; i++) {
        if (argv[i] == NULL) {
            continue;
        }

        if ((os_strcmp(argv[i], "-q") == 0) || (os_strcmp(argv[i], "--quiet") == 0)) {
            verbose = 0;
            continue;
        }

        if ((os_strcmp(argv[i], "-x") == 0) || (os_strcmp(argv[i], "--exclude") == 0)) {
            bk_printf("[xmbedtls] ERROR selftest exclude mode is not supported in adapter CLI\r\n");
            return -1;
        }
    }

    for (int i = 2; i < argc; i++) {
        if ((argv[i] == NULL) || (argv[i][0] == '-')) {
            continue;
        }

        selftest = mbedtls_test_find_selftest(argv[i]);
        if (selftest == NULL) {
            bk_printf("[xmbedtls] ERROR unknown selftest suite=%s\r\n", argv[i]);
            return -1;
        }

        {
            int one_ret = selftest->fn(verbose);
            mbedtls_test_log_result(&st, selftest->name, one_ret);
            if (one_ret != 0) {
                ret = -1;
            }
        }
        selected_cnt++;
    }

    if (selected_cnt == 0) {
        for (size_t i = 0; i < (sizeof(s_mbedtls_selftests) / sizeof(s_mbedtls_selftests[0])); i++) {
            int one_ret = s_mbedtls_selftests[i].fn(verbose);
            mbedtls_test_log_result(&st, s_mbedtls_selftests[i].name, one_ret);
            if (one_ret != 0) {
                ret = -1;
            }
        }
    }

    mbedtls_test_log_summary("selftest", &st);
    return ret;
}

static void mbedtls_test_selftest_cmd(int argc, char **argv)
{
    mbedtls_test_stat_t st = {0};

    mbedtls_test_vote_cpu_freq();
    mbedtls_test_log_result(&st, "selftest", mbedtls_test_run_selftest(argc, argv));
    mbedtls_test_restore_cpu_freq();
}

static void mbedtls_test_thread_cmd(int argc, char **argv)
{
    mbedtls_test_stat_t st = {0};
    int ret;

    if ((argc < 4) || (argv[2] == NULL) || (argv[3] == NULL)) {
        mbedtls_test_usage();
        return;
    }

    if (os_strcmp(argv[2], "create") != 0) {
        mbedtls_test_usage();
        return;
    }

    s_mbedtls_thread_loop_cnt = os_strtoul(argv[3], NULL, 10);
    ret = rtos_create_thread(NULL,
                             MBEDTLS_TEST_PRIORITY,
                             "xmbedtls",
                             te200_muti_task_test,
                             MBEDTLS_TEST_STACK_SIZE,
                             (beken_thread_arg_t)(&s_mbedtls_thread_loop_cnt));
    mbedtls_test_log_result(&st, "thread_create", ret);
    if (ret == 0) {
        bk_printf("[xmbedtls] INFO thread_started loop_cnt=%u\r\n",
                  (unsigned int)s_mbedtls_thread_loop_cnt);
    }
    mbedtls_test_log_summary("thread", &st);
}

/* ---- background task wrapper ---- */
#define XMBEDTLS_TASK_ARGC_MAX  4
#define XMBEDTLS_ARG_LEN_MAX    32

typedef struct {
    int    argc;
    char   argv_buf[XMBEDTLS_TASK_ARGC_MAX][XMBEDTLS_ARG_LEN_MAX];
    char  *argv[XMBEDTLS_TASK_ARGC_MAX];
} xmbedtls_task_arg_t;

static void xmbedtls_task(void *param)
{
    xmbedtls_task_arg_t *arg = (xmbedtls_task_arg_t *)param;
    int   argc = arg->argc;
    char **argv = arg->argv;

    if (os_strcmp(argv[1], "sha") == 0) {
        mbedtls_test_sha_cmd(argc, argv);
    } else if (os_strcmp(argv[1], "aes") == 0) {
        mbedtls_test_aes_cmd(argc, argv);
    } else if (os_strcmp(argv[1], "ecdsa") == 0) {
        mbedtls_test_ecdsa_cmd(argc, argv);
    } else if (os_strcmp(argv[1], "tls") == 0) {
        mbedtls_test_tls_cmd(argc, argv);
    } else if (os_strcmp(argv[1], "rsa") == 0) {
        mbedtls_test_rsa_cmd();
    } else if (os_strcmp(argv[1], "selftest") == 0) {
        mbedtls_test_selftest_cmd(argc, argv);
    } else {
        bk_printf("unknown xmbedtls subcmd: %s\r\n", argv[1]);
        mbedtls_test_usage();
    }

    os_free(arg);
    rtos_delete_thread(NULL);
}

void cli_xmbedtls_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    if ((argc < 2) || (argv == NULL) || (argv[1] == NULL)) {
        mbedtls_test_usage();
        return;
    }

    if ((os_strcmp(argv[1], "help") == 0) || (os_strcmp(argv[1], "?") == 0)) {
        mbedtls_test_usage();
        return;
    }

    /* thread subcommand manages its own task, run inline */
    if (os_strcmp(argv[1], "thread") == 0) {
        mbedtls_test_thread_cmd(argc, argv);
        return;
    }

    /* all other subcommands are CPU-heavy; run in a background task
     * to avoid blocking the shell task and triggering watchdog */
    {
        xmbedtls_task_arg_t *arg = os_zalloc(sizeof(xmbedtls_task_arg_t));
        int i;

        if (arg == NULL) {
            bk_printf("[xmbedtls] no memory\r\n");
            return;
        }

        arg->argc = (argc > XMBEDTLS_TASK_ARGC_MAX) ? XMBEDTLS_TASK_ARGC_MAX : argc;
        for (i = 0; i < arg->argc; i++) {
            if (argv[i] != NULL) {
                os_strlcpy(arg->argv_buf[i], argv[i], XMBEDTLS_ARG_LEN_MAX);
            }
            arg->argv[i] = arg->argv_buf[i];
        }

        if (rtos_create_thread(NULL,
                               MBEDTLS_TEST_PRIORITY,
                               "xmbedtls",
                               xmbedtls_task,
                               MBEDTLS_TEST_STACK_SIZE,
                               (beken_thread_arg_t)arg) != kNoErr) {
            CLI_LOGE("xmbedtls: create task failed\r\n");
            os_free(arg);
        }
    }
}
