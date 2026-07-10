/**
 * @file test_ota.c
 * @brief OTA self-test: read Beken RBL UG from SD card and run tkl_ota flow
 * @version 1.0
 * @date 2026-07-03
 * @copyright Copyright (c) Tuya Inc.
 */
#include "cli.h"
#include "cli_tuya_test.h"
#include "sdkconfig.h"
#include "tkl_flash.h"
#include "tkl_fs.h"
#include "tkl_memory.h"
#include "tkl_ota.h"
#include "tkl_system.h"
#include <common/bk_include.h>
#include <driver/flash_partition.h>
#include <modules/ota.h>
#include <os/os.h>
#include <string.h>

extern INT_T test_fs_mount(CONST CHAR_T *path, FS_DEV_TYPE_T dev_type);
extern INT_T test_fs_unmount(CONST CHAR_T *path);

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define TEST_OTA_UG_FILE_NAME           "UG.bin"
#define TEST_OTA_CHUNK_SIZE             4096U
#define TEST_OTA_FILE_PATH_MAX          128U
#define TEST_OTA_PROGRESS_STEP_PCT      10U
#define TEST_OTA_YIELD_MS               2U
#define TEST_OTA_RBL_MAGIC              "RBL"
#define TEST_OTA_RBL_MAGIC_LEN          4U
#define TEST_OTA_RBL_HEAD_POS           0x1000U

/* Beken RBL header (see bk_ota_private.h struct ota_rbl_head) */
typedef struct {
    CHAR_T  magic[4];
    UINT32_T algo;
    UINT32_T timestamp;
    CHAR_T  name[16];
    CHAR_T  version[24];
    CHAR_T  sn[24];
    UINT32_T crc32;
    UINT32_T hash;
    UINT32_T size_raw;
    UINT32_T size_package;
    UINT32_T info_crc32;
} TEST_OTA_RBL_HEAD_T;

extern void bk_printf(const char *fmt, ...);

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
STATIC INT_T __test_ota_mount(VOID_T);
STATIC VOID_T __test_ota_umount(VOID_T);
STATIC INT_T __test_ota_get_file_size(CONST CHAR_T *file_path, UINT_T *file_size);
STATIC INT_T __test_ota_run(CONST CHAR_T *ug_name, BOOL_T reset);
STATIC VOID_T __test_ota_print_ability(VOID_T);
STATIC INT_T __test_ota_get_target_partition(bk_logic_partition_t **pt);
STATIC UINT_T __test_ota_get_rbl_head_addr(CONST bk_logic_partition_t *pt);
STATIC INT_T __test_ota_verify_rbl_magic(CONST bk_logic_partition_t *pt);
STATIC INT_T __test_ota_verify_flash_vs_file(CONST CHAR_T *file_path,
                                              CONST bk_logic_partition_t *pt,
                                              UINT_T file_size);
STATIC INT_T __test_ota_verify(CONST CHAR_T *ug_name);

/**
 * @brief Mount SD card for OTA test
 * @return 0 on success, non-zero on failure
 */
STATIC INT_T __test_ota_mount(VOID_T)
{
    return test_fs_mount(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD);
}

/**
 * @brief Unmount SD card after OTA test
 * @return none
 */
STATIC VOID_T __test_ota_umount(VOID_T)
{
    test_fs_unmount(TEST_SDCARD_MOUNT_POINT);
}

/**
 * @brief Query file size on mounted SD card
 * @param[in] file_path absolute or relative path under mount point
 * @param[out] file_size output file size in bytes
 * @return 0 on success, non-zero on failure
 */
STATIC INT_T __test_ota_get_file_size(CONST CHAR_T *file_path, UINT_T *file_size)
{
    TUYA_FILE fp = NULL;
    INT64_T size = 0;

    if ((file_path == NULL) || (file_size == NULL)) {
        return -1;
    }

    fp = tkl_fopen(file_path, "rb");
    if (fp == NULL) {
        bk_printf("[xota] open %s failed\r\n", file_path);
        return -1;
    }

    if (tkl_fseek(fp, 0, SEEK_END) != 0) {
        bk_printf("[xota] seek end failed: %s\r\n", file_path);
        tkl_fclose(fp);
        return -1;
    }

    size = tkl_ftell(fp);
    tkl_fclose(fp);

    if (size <= 0) {
        bk_printf("[xota] invalid file size: %lld\r\n", (long long)size);
        return -1;
    }

    *file_size = (UINT_T)size;
    return 0;
}

/**
 * @brief Log OTA progress every 10%
 * @param[in] received bytes received so far
 * @param[in] total total file size
 * @param[in,out] last_pct last logged percentage
 * @return none
 */
STATIC VOID_T __test_ota_log_progress(UINT_T received, UINT_T total, UINT_T *last_pct)
{
    UINT_T cur_pct;

    if ((total == 0) || (last_pct == NULL)) {
        return;
    }

    cur_pct = (received * 100U) / total;
    if ((cur_pct < *last_pct) ||
        ((cur_pct - *last_pct) < TEST_OTA_PROGRESS_STEP_PCT && cur_pct != 100U)) {
        return;
    }

    bk_printf("[xota] progress %u%% (%u/%u)\r\n", cur_pct, received, total);
    *last_pct = cur_pct;
}

/**
 * @brief Print tkl_ota capability
 * @return none
 */
STATIC VOID_T __test_ota_print_ability(VOID_T)
{
    OPERATE_RET ret;
    UINT_T max_size = 0;
    TUYA_OTA_TYPE_E ota_type = TUYA_OTA_FULL;

    ret = tkl_ota_get_ability(&max_size, &ota_type);
    if (ret != OPRT_OK) {
        bk_printf("[xota] get_ability failed: %d\r\n", ret);
        return;
    }

    bk_printf("[xota] max_image_size=%u type=0x%x\r\n", max_size, ota_type);
}

/**
 * @brief Get OTA target partition (same selection as tkl_ota)
 * @param[out] pt output partition info pointer
 * @return 0 on success, non-zero on failure
 */
STATIC INT_T __test_ota_get_target_partition(bk_logic_partition_t **pt)
{
    bk_logic_partition_t *part = NULL;

    if (pt == NULL) {
        return -1;
    }

#ifdef CONFIG_HTTP_AB_PARTITION
    exec_flag cur_part = (exec_flag)bk_ota_get_current_partition();

#if CONFIG_OTA_POSITION_INDEPENDENT_AB
    if (cur_part == EXEC_B_PART) {
        part = bk_flash_partition_get_info(BK_PARTITION_APPLICATION);
    } else {
        part = bk_flash_partition_get_info(BK_PARTITION_S_APP);
    }
#else
    part = bk_flash_partition_get_info(BK_PARTITION_S_APP);
    (void)cur_part;
#endif
#else
    part = bk_flash_partition_get_info(BK_PARTITION_OTA);
#endif

    if (part == NULL) {
        return -1;
    }

    *pt = part;
    return 0;
}

/**
 * @brief Get RBL header flash address inside OTA target partition
 * @param[in] pt target partition info
 * @return absolute flash address of RBL header
 * @note AB layout stores header at partition tail; non-AB at partition start
 */
STATIC UINT_T __test_ota_get_rbl_head_addr(CONST bk_logic_partition_t *pt)
{
    if (pt == NULL) {
        return 0;
    }

#ifdef CONFIG_HTTP_AB_PARTITION
    return pt->partition_start_addr + pt->partition_length - TEST_OTA_RBL_HEAD_POS;
#else
    return pt->partition_start_addr;
#endif
}

/**
 * @brief Verify RBL magic in OTA target partition
 * @param[in] pt target partition info
 * @return 0 on success, non-zero on failure
 */
STATIC INT_T __test_ota_verify_rbl_magic(CONST bk_logic_partition_t *pt)
{
    TEST_OTA_RBL_HEAD_T rbl_head;
    UINT_T head_addr = 0;
    OPERATE_RET ret;
    BOOL_T magic_ok = FALSE;

    if (pt == NULL) {
        return -1;
    }

    head_addr = __test_ota_get_rbl_head_addr(pt);
    memset(&rbl_head, 0, sizeof(rbl_head));

    ret = tkl_flash_read(head_addr, (UCHAR_T *)&rbl_head, sizeof(rbl_head));
    if (ret != OPRT_OK) {
        bk_printf("[xota] verify: flash read head failed ret=%d\r\n", ret);
        return -1;
    }

    if (memcmp(rbl_head.magic, TEST_OTA_RBL_MAGIC, TEST_OTA_RBL_MAGIC_LEN) == 0) {
        magic_ok = TRUE;
    }

    bk_printf("[xota] verify: part=%s head=0x%x magic=%.4s %s\r\n",
              pt->partition_description, head_addr, rbl_head.magic,
              magic_ok ? "OK" : "FAIL");

    if (magic_ok == FALSE) {
        bk_printf("[xota] verify: expected magic=RBL\r\n");
        return -1;
    }

    bk_printf("[xota] verify: algo=0x%x ver=%.24s raw=%u pkg=%u\r\n",
              rbl_head.algo, rbl_head.version, rbl_head.size_raw, rbl_head.size_package);
    return 0;
}

/**
 * @brief Compare SD OTA file with flash content byte-by-byte
 * @param[in] file_path SD file absolute path
 * @param[in] pt target partition info (same as tkl_ota write target)
 * @param[in] file_size OTA file size in bytes
 * @return 0 if identical, non-zero on mismatch or error
 * @note Flash base address matches tkl_ota: partition_start_addr, sequential write
 */
STATIC INT_T __test_ota_verify_flash_vs_file(CONST CHAR_T *file_path,
                                              CONST bk_logic_partition_t *pt,
                                              UINT_T file_size)
{
    TUYA_FILE fp = NULL;
    UCHAR_T *sd_buf = NULL;
    UCHAR_T *flash_buf = NULL;
    UINT_T flash_base = 0;
    UINT_T offset = 0;
    UINT_T last_pct = 0;
    INT_T read_len = 0;
    INT_T cmp_ret = 0;
    OPERATE_RET flash_ret;

    if ((file_path == NULL) || (pt == NULL) || (file_size == 0)) {
        return -1;
    }

    if (file_size > pt->partition_length) {
        bk_printf("[xota] verify: file %u > part len %u\r\n",
                  file_size, pt->partition_length);
        return -1;
    }

    flash_base = pt->partition_start_addr;
    fp = tkl_fopen(file_path, "rb");
    if (fp == NULL) {
        bk_printf("[xota] verify: open %s failed\r\n", file_path);
        return -1;
    }

    sd_buf = (UCHAR_T *)tkl_system_malloc(TEST_OTA_CHUNK_SIZE);
    flash_buf = (UCHAR_T *)tkl_system_malloc(TEST_OTA_CHUNK_SIZE);
    if ((sd_buf == NULL) || (flash_buf == NULL)) {
        bk_printf("[xota] verify: malloc compare buf failed\r\n");
        cmp_ret = -1;
        goto exit;
    }

    bk_printf("[xota] verify: compare file=%s size=%u flash=0x%x\r\n",
              file_path, file_size, flash_base);

    while (offset < file_size) {
        UINT_T want_len = file_size - offset;
        INT_T diff_idx = -1;
        UINT_T i;

        if (want_len > TEST_OTA_CHUNK_SIZE) {
            want_len = TEST_OTA_CHUNK_SIZE;
        }

        if ((flash_base + offset) > (UINT32_T)(-1) - want_len) {
            bk_printf("[xota] verify: flash addr overflow at offset=%u\r\n", offset);
            cmp_ret = -1;
            goto exit;
        }

        read_len = tkl_fread(sd_buf, (INT_T)want_len, fp);
        if (read_len <= 0) {
            bk_printf("[xota] verify: sd read failed at offset=%u\r\n", offset);
            cmp_ret = -1;
            goto exit;
        }

        flash_ret = tkl_flash_read(flash_base + offset, flash_buf, (UINT_T)read_len);
        if (flash_ret != OPRT_OK) {
            bk_printf("[xota] verify: flash read failed at offset=%u ret=%d\r\n",
                      offset, flash_ret);
            cmp_ret = -1;
            goto exit;
        }

        for (i = 0; i < (UINT_T)read_len; i++) {
            if (sd_buf[i] != flash_buf[i]) {
                diff_idx = (INT_T)i;
                break;
            }
        }

        if (diff_idx >= 0) {
            UINT_T abs_off = offset + (UINT_T)diff_idx;

            bk_printf("[xota] verify: MISMATCH at offset=%u (flash=0x%x)\r\n",
                      abs_off, flash_base + abs_off);
            bk_printf("[xota] verify: sd=0x%02x flash=0x%02x\r\n",
                      sd_buf[diff_idx], flash_buf[diff_idx]);
            cmp_ret = -1;
            goto exit;
        }

        offset += (UINT_T)read_len;
        __test_ota_log_progress(offset, file_size, &last_pct);
    }

    bk_printf("[xota] verify: flash content matches SD file (%u bytes)\r\n", file_size);
    cmp_ret = 0;

exit:
    if (fp != NULL) {
        tkl_fclose(fp);
    }
    if (sd_buf != NULL) {
        tkl_system_free(sd_buf);
    }
    if (flash_buf != NULL) {
        tkl_system_free(flash_buf);
    }
    return cmp_ret;
}

/**
 * @brief Verify OTA partition: RBL magic + full content vs SD UG file
 * @param[in] ug_name UG file name under /sdcard (NULL for default UG.bin)
 * @return 0 on success, non-zero on failure
 */
STATIC INT_T __test_ota_verify(CONST CHAR_T *ug_name)
{
    CHAR_T file_path[TEST_OTA_FILE_PATH_MAX] = {0};
    bk_logic_partition_t *pt = NULL;
    UINT_T file_size = 0;
    INT_T path_len = 0;
    INT_T ret = 0;

    if (ug_name == NULL) {
        ug_name = TEST_OTA_UG_FILE_NAME;
    }

    path_len = os_snprintf(file_path, sizeof(file_path), "%s/%s",
                           TEST_SDCARD_MOUNT_POINT, ug_name);
    if ((path_len < 0) || (path_len >= (INT_T)sizeof(file_path))) {
        bk_printf("[xota] verify: file path too long\r\n");
        return -1;
    }

    if (__test_ota_get_target_partition(&pt) != 0) {
        bk_printf("[xota] verify: get target partition failed\r\n");
        return -1;
    }

    if (__test_ota_mount() != 0) {
        bk_printf("[xota] verify: mount failed\r\n");
        return -1;
    }

    if (__test_ota_get_file_size(file_path, &file_size) != 0) {
        __test_ota_umount();
        return -1;
    }

    ret = __test_ota_verify_rbl_magic(pt);
    if (ret != 0) {
        __test_ota_umount();
        return ret;
    }

    ret = __test_ota_verify_flash_vs_file(file_path, pt, file_size);
    __test_ota_umount();
    return ret;
}

/**
 * @brief Run OTA flow: SD UG.bin -> tkl_ota (same as cloud path without MULTI_SECTION)
 * @param[in] ug_name UG file name under /sdcard
 * @param[in] reset whether to reset after OTA end notify
 * @return 0 on success, non-zero on failure
 * @note Mimics tuya_svc_upgrade __dev_upgrage_process_cb + tal_ota_* without header parse
 */
STATIC INT_T __test_ota_run(CONST CHAR_T *ug_name, BOOL_T reset)
{
    CHAR_T file_path[TEST_OTA_FILE_PATH_MAX] = {0};
    TUYA_FILE fp = NULL;
    UCHAR_T *chunk_buf = NULL;
    OPERATE_RET ret = OPRT_OK;
    UINT_T file_size = 0;
    UINT_T max_size = 0;
    UINT_T ota_type = TUYA_OTA_FULL;
    UINT_T offset = 0;
    UINT_T remain_len = 0;
    UINT_T last_pct = 0;
    BOOL_T upgrade_started = FALSE;
    INT_T read_len = 0;
    INT_T path_len = 0;

    if (ug_name == NULL) {
        ug_name = TEST_OTA_UG_FILE_NAME;
    }

    path_len = os_snprintf(file_path, sizeof(file_path), "%s/%s",
                           TEST_SDCARD_MOUNT_POINT, ug_name);
    if ((path_len < 0) || (path_len >= (INT_T)sizeof(file_path))) {
        bk_printf("[xota] file path too long\r\n");
        return -1;
    }

    if (__test_ota_mount() != 0) {
        bk_printf("[xota] mount failed\r\n");
        return -1;
    }

    if (__test_ota_get_file_size(file_path, &file_size) != 0) {
        __test_ota_umount();
        return -1;
    }

    ret = tkl_ota_get_ability(&max_size, (TUYA_OTA_TYPE_E *)&ota_type);
    if (ret != OPRT_OK) {
        bk_printf("[xota] get_ability failed: %d\r\n", ret);
        __test_ota_umount();
        return -1;
    }

    if ((ota_type & TUYA_OTA_FULL) == 0) {
        bk_printf("[xota] platform does not support TUYA_OTA_FULL\r\n");
        __test_ota_umount();
        return -1;
    }

    if (file_size > max_size) {
        bk_printf("[xota] file too large: %u > %u\r\n", file_size, max_size);
        __test_ota_umount();
        return -1;
    }

    fp = tkl_fopen(file_path, "rb");
    if (fp == NULL) {
        bk_printf("[xota] open %s failed\r\n", file_path);
        __test_ota_umount();
        return -1;
    }

    chunk_buf = (UCHAR_T *)tkl_system_malloc(TEST_OTA_CHUNK_SIZE);
    if (chunk_buf == NULL) {
        bk_printf("[xota] malloc chunk buf failed\r\n");
        tkl_fclose(fp);
        __test_ota_umount();
        return -1;
    }

    bk_printf("[xota] start file=%s size=%u reset=%u\r\n", file_path, file_size, reset);

    while (offset < file_size) {
        UINT_T want_len = file_size - offset;

        if (want_len > TEST_OTA_CHUNK_SIZE) {
            want_len = TEST_OTA_CHUNK_SIZE;
        }

        read_len = tkl_fread(chunk_buf, (INT_T)want_len, fp);
        if (read_len <= 0) {
            bk_printf("[xota] read failed at offset=%u\r\n", offset);
            ret = OPRT_COM_ERROR;
            break;
        }

        if (upgrade_started == FALSE) {
            ret = tkl_ota_start_notify(file_size, TUYA_OTA_FULL, TUYA_OTA_PATH_AIR);
            if (ret != OPRT_OK) {
                bk_printf("[xota] start_notify failed: %d\r\n", ret);
                break;
            }
            upgrade_started = TRUE;
        }

        remain_len = 0;
        {
            TUYA_OTA_DATA_T pack = {0};

            pack.total_len = file_size;
            pack.offset = offset;
            pack.data = chunk_buf;
            pack.len = (UINT_T)read_len;

            ret = tkl_ota_data_process(&pack, &remain_len);
            if (ret != OPRT_OK) {
                bk_printf("[xota] data_process failed at offset=%u ret=%d\r\n", offset, ret);
                break;
            }
        }

        rtos_delay_milliseconds(TEST_OTA_YIELD_MS);
        offset += (UINT_T)read_len;
        __test_ota_log_progress(offset, file_size, &last_pct);
    }

    tkl_fclose(fp);
    fp = NULL;
    tkl_system_free(chunk_buf);
    chunk_buf = NULL;
    __test_ota_umount();

    if (ret != OPRT_OK) {
        return -1;
    }

    if (offset != file_size) {
        bk_printf("[xota] size mismatch recv=%u expect=%u\r\n", offset, file_size);
        return -1;
    }

    ret = tkl_ota_end_notify(reset);
    if (ret != OPRT_OK) {
        bk_printf("[xota] end_notify failed: %d\r\n", ret);
        return -1;
    }

    bk_printf("[xota] success\r\n");
    return 0;
}

/**
 * @brief CLI handler for OTA self-test
 * @param[in] pcWriteBuffer CLI output buffer
 * @param[in] xWriteBufferLen buffer length
 * @param[in] argc argument count
 * @param[in] argv argument vector
 * @return none
 */
VOID_T cli_ota_test_cmd(CHAR_T *pcWriteBuffer, INT_T xWriteBufferLen, INT_T argc, CHAR_T **argv)
{
    (void)pcWriteBuffer;
    (void)xWriteBufferLen;

    if (argc < 2) {
        bk_printf("usage:\r\n");
        bk_printf("  xota ability          - show OTA capability\r\n");
        bk_printf("  xota run [UG.bin] [0|1] - OTA from SD (default UG.bin, reset=1)\r\n");
        bk_printf("  xota verify [UG.bin]  - RBL magic + compare flash vs SD file\r\n");
        return;
    }

    if (!os_strcmp(argv[1], "ability")) {
        __test_ota_print_ability();
    } else if (!os_strcmp(argv[1], "verify")) {
        CONST CHAR_T *ug_name = TEST_OTA_UG_FILE_NAME;

        if ((argc >= 3) && (argv[2] != NULL)) {
            ug_name = argv[2];
        }

        if (__test_ota_verify(ug_name) != 0) {
            bk_printf("[xota] verify failed\r\n");
        } else {
            bk_printf("[xota] verify ok\r\n");
        }
    } else if (!os_strcmp(argv[1], "run")) {
        CONST CHAR_T *ug_name = TEST_OTA_UG_FILE_NAME;
        BOOL_T reset = TRUE;

        if ((argc >= 3) && (argv[2] != NULL)) {
            ug_name = argv[2];
        }
        if ((argc >= 4) && (argv[3] != NULL)) {
            reset = (os_strtoul(argv[3], NULL, 10) != 0) ? TRUE : FALSE;
        }

        if (__test_ota_run(ug_name, reset) != 0) {
            bk_printf("[xota] run failed\r\n");
        }
    } else {
        bk_printf("[xota] unknown cmd: %s\r\n", argv[1]);
    }
}
