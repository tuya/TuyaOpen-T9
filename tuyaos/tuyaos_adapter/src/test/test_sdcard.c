/*
 * test_sdcard.c
 * Copyright (C) 2025 cc <cc@tuya>
 *
 * Distributed under terms of the MIT license.
 */

#include "cli.h"
#include "cli_tuya_test.h"
#include "tuya_cloud_types.h"

// #include "ff.h"         /* FatFS头文件 */
// #include "ffconf.h"     /* FatFS配置 */

#include <common/bk_include.h>
// #include "diskio.h"
#include "bk_posix.h"
#include "driver/sd_card_types.h"
#include "tkl_fs.h"
#include "tkl_memory.h"
/* tkl_system_get_millisecond() returns SYS_TIME_T (64-bit). Without this the
 * implicit declaration makes it int, truncating the timing source the speed
 * test reports from. */
#include "tkl_system.h"
#include <string.h>

#define TEST_SDCARD_WR_FILE_NAME      "test.txt"
#define TEST_SDCARD_WR_FILE_CONTENT   "hello_world"

static void __cli_sdcard_list_file(const char *path);

static int __cli_sdcard_mount(void)
{
    return test_fs_mount(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD);
}

static int __cli_sdcard_mount_once(void)
{
    int ret = tkl_fs_mount(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD);
    if (ret != 0) {
        bk_printf("mount %s failed, ret=%d\r\n", TEST_SDCARD_MOUNT_POINT, ret);
    }

    return ret;
}

static void __cli_sdcard_umount(void)
{
    test_fs_unmount(TEST_SDCARD_MOUNT_POINT);
}

static int __cli_sdcard_umount_once(void)
{
    int ret = tkl_fs_unmount(TEST_SDCARD_MOUNT_POINT);
    if (ret != 0) {
        bk_printf("unmount %s failed, ret=%d\r\n", TEST_SDCARD_MOUNT_POINT, ret);
    }

    return ret;
}

static void __cli_sdcard_unlink(const char *path)
{
    char buf[64] = {'\0'};
    char fp[64] = {'\0'};

    if (path == NULL) {
        bk_printf("rm failed, no file name spec\r\n");
        return;
    }

    sprintf(fp, "%s/%s", TEST_SDCARD_MOUNT_POINT, path);

    int ret = tkl_fs_remove(fp);
    if (ret != 0) {
        bk_printf("rm %s failed\r\n", path);
        return;
    }

    bk_printf("rm file: %s\r\n", path);
}

static void __cli_sdcard_read(const char *path)
{
    char buf[64] = {'\0'};
    char fp[64] = {'\0'};

    if (path == NULL) {
        bk_printf("read failed, no file name spec\r\n");
        return;
    }

    sprintf(fp, "%s/%s", TEST_SDCARD_MOUNT_POINT, path);

    TUYA_FILE f = tkl_fopen(fp, "r");
    if (f == NULL) {
        bk_printf("open %s failed\r\n", path);
        return;
    }

    tkl_fread(buf, 64, f);

    tkl_fclose(f);

    bk_printf("read: %s\r\n", buf);
}

static void __cli_sdcard_write(const char *path, const uint8_t *buf, uint32_t size)
{
    if (path == NULL) {
        bk_printf("write failed, no file name spec\r\n");
        return;
    }
    char fp[64] = {'\0'};

    sprintf(fp, "%s/%s", TEST_SDCARD_MOUNT_POINT, path);

    TUYA_FILE f = tkl_fopen(fp, "ab");
    if (f == NULL) {
        bk_printf("open %s failed\r\n", path);
        return;
    }

    tkl_fwrite(buf, size, f);

    tkl_fclose(f);
}

static int __cli_sdcard_write_file_overwrite(const char *path, const uint8_t *buf, uint32_t size)
{
    char fp[64] = {'\0'};
    TUYA_FILE f = NULL;
    int ret = 0;

    if ((path == NULL) || (buf == NULL) || (size == 0)) {
        bk_printf("write failed, invalid param\r\n");
        return -1;
    }

    os_snprintf(fp, sizeof(fp), "%s/%s", TEST_SDCARD_MOUNT_POINT, path);

    f = tkl_fopen(fp, "wb");
    if (f == NULL) {
        bk_printf("open %s failed\r\n", path);
        return -1;
    }

    ret = tkl_fwrite((void *)buf, size, f);
    if (ret != (int)size) {
        bk_printf("write %s failed, ret=%d expect=%u\r\n", path, ret, size);
        tkl_fclose(f);
        return -1;
    }

    ret = tkl_fsync((int)f);
    if (ret != 0) {
        bk_printf("fsync %s failed, ret=%d\r\n", path, ret);
        tkl_fclose(f);
        return -1;
    }

    tkl_fclose(f);
    bk_printf("write: %s\r\n", path);
    return 0;
}

static int __cli_sdcard_read_and_check(const char *path, const uint8_t *expect, uint32_t expect_size)
{
    char buf[64] = {'\0'};
    char fp[64] = {'\0'};
    TUYA_FILE f = NULL;
    int ret = 0;

    if ((path == NULL) || (expect == NULL) || (expect_size == 0) || (expect_size >= sizeof(buf))) {
        bk_printf("read check failed, invalid param\r\n");
        return -1;
    }

    os_snprintf(fp, sizeof(fp), "%s/%s", TEST_SDCARD_MOUNT_POINT, path);

    f = tkl_fopen(fp, "r");
    if (f == NULL) {
        bk_printf("open %s failed\r\n", path);
        return -1;
    }

    ret = tkl_fread(buf, expect_size, f);
    tkl_fclose(f);
    if (ret != (int)expect_size) {
        bk_printf("read %s failed, ret=%d expect=%u\r\n", path, ret, expect_size);
        return -1;
    }

    buf[expect_size] = '\0';
    if (memcmp(buf, expect, expect_size) != 0) {
        bk_printf("read check failed, expect=%s actual=%s\r\n", (const char *)expect, buf);
        return -1;
    }

    bk_printf("read: %s\r\n", buf);
    return 0;
}

static int __cli_sdcard_delete_file(const char *path)
{
    char fp[64] = {'\0'};
    int ret = 0;

    if (path == NULL) {
        bk_printf("delete failed, no file name spec\r\n");
        return -1;
    }

    os_snprintf(fp, sizeof(fp), "%s/%s", TEST_SDCARD_MOUNT_POINT, path);

    ret = tkl_fs_remove(fp);
    if (ret != 0) {
        bk_printf("rm %s failed, ret=%d\r\n", path, ret);
        return -1;
    }

    bk_printf("rm file: %s\r\n", path);
    return 0;
}

static int __cli_sdcard_is_exist(const char *path)
{
    char fp[64] = {'\0'};
    const char *check_path = NULL;
    BOOL_T is_exist = FALSE;
    int ret = 0;
    int len = 0;

    if (path == NULL) {
        bk_printf("usage: xsd exist <path>\r\n");
        return -1;
    }

    if (path[0] == '/') {
        check_path = path;
    } else {
        len = os_snprintf(fp, sizeof(fp), "%s/%s", TEST_SDCARD_MOUNT_POINT, path);
        if ((len < 0) || (len >= (int)sizeof(fp))) {
            bk_printf("[xsd exist] path too long: %s\r\n", path);
            return -1;
        }
        check_path = fp;
    }

    ret = tkl_fs_is_exist(check_path, &is_exist);
    bk_printf("[xsd exist] path=%s ret=%d exist=%u\r\n",
              check_path,
              ret,
              (unsigned int)is_exist);
    return ret;
}

static int __cli_sdcard_loop_wr(uint32_t loops)
{
    uint32_t i = 0;
    uint32_t pass = 0;
    uint32_t fail = 0;
    int ret = 0;

    if (loops == 0) {
        bk_printf("invalid loops: 0\r\n");
        return -1;
    }

    for (i = 0; i < loops; i++) {
        bk_printf("[xsd wr] loop %u/%u begin\r\n", i + 1, loops);

        ret = __cli_sdcard_mount_once();
        if (ret != 0) {
            bk_printf("[xsd wr] mount failed, ret=%d\r\n", ret);
            fail++;
            continue;
        }

        ret = __cli_sdcard_write_file_overwrite(TEST_SDCARD_WR_FILE_NAME,
                                                (const uint8_t *)TEST_SDCARD_WR_FILE_CONTENT,
                                                strlen(TEST_SDCARD_WR_FILE_CONTENT));
        if (ret != 0) {
            goto loop_end;
        }

        ret = __cli_sdcard_read_and_check(TEST_SDCARD_WR_FILE_NAME,
                                          (const uint8_t *)TEST_SDCARD_WR_FILE_CONTENT,
                                          strlen(TEST_SDCARD_WR_FILE_CONTENT));
        if (ret != 0) {
            goto loop_end;
        }

        __cli_sdcard_list_file(TEST_SDCARD_MOUNT_POINT);

        ret = __cli_sdcard_delete_file(TEST_SDCARD_WR_FILE_NAME);

loop_end:
        {
            int umount_ret = __cli_sdcard_umount_once();
            if ((ret == 0) && (umount_ret != 0)) {
                ret = umount_ret;
            }
        }

        if (ret == 0) {
            pass++;
            bk_printf("[xsd wr] loop %u/%u pass\r\n", i + 1, loops);
        } else {
            fail++;
            bk_printf("[xsd wr] loop %u/%u fail\r\n", i + 1, loops);
        }
    }

    bk_printf("[xsd wr] done, loops=%u, pass=%u, fail=%u\r\n", loops, pass, fail);
    return (fail == 0) ? 0 : -1;
}

static void __cli_sdcard_list_file(const char* path)
{
    INT_T res = 0, is_dir = 0;
    TUYA_DIR dir;
    TUYA_FILEINFO info;
    char *f_name = NULL;

    res = tkl_dir_open(path, &dir);                 /* Open the directory */
    if (res == 0) {
        bk_printf("%s\r\n", path);
        while (1) {
            res = tkl_dir_read(dir, &info);         /* Read a directory item */
            if (res != 0) {
                break;  /* Break on error */
            }

            tkl_dir_name(info, &f_name);

            if (f_name == NULL) {
                break;  /* Break on end of dir */
            }

            tkl_dir_is_directory(info, &is_dir);
            if (is_dir) {
                /* It is a directory */
                char *pathTemp = tkl_system_malloc(strlen(path)+strlen(f_name)+2);
                if(pathTemp == NULL) {
                    bk_printf("%s:os_malloc dir failed \r\n", __func__);
                    break;
                }
                sprintf(pathTemp, "%s/%s", path, f_name);
                __cli_sdcard_list_file(pathTemp);      /* Enter the directory */
                tkl_system_free(pathTemp);
            } else {
                /* It is a file. */
                bk_printf("%s/%s\r\n", path, f_name);
            }
        }
        tkl_dir_close(dir);
    } else {
        bk_printf("f_opendir failed\r\n");
    }

    return;
}

static TUYA_DIR sdcard_test_dirp = NULL;
static void __cli_sdcard_closedir(TUYA_DIR sdcard_test_dirp)
{
    if (sdcard_test_dirp == NULL)
        return;

    tkl_dir_close(sdcard_test_dirp);
}

static void __cli_sdcard_mkdir(const char *dir_path)
{
    char buff[128] = {0};

    if (dir_path == NULL) {
        return;
    }

    if (sdcard_test_dirp != NULL) {
        // close last opened dir
        __cli_sdcard_closedir(sdcard_test_dirp);
        sdcard_test_dirp = NULL;
    }

    tkl_dir_open(dir_path, &sdcard_test_dirp);
    if (sdcard_test_dirp == NULL) {
        bk_printf("[%s][%d] no %s found, mkdir\r\n", __FUNCTION__, __LINE__, dir_path);
        // mkdir
        int fd = tkl_fs_mkdir(dir_path);
        if(fd < 0)
        {
            bk_printf("[%s][%d] mkdir fail:%d\r\n", __FUNCTION__, __LINE__, fd);
            return ;
        }
        read(fd, buff, sizeof(buff));
        close(fd);
        bk_printf("[%s][%d] mkdir ok\r\n", __FUNCTION__, __LINE__);
    }
}



#define SPEED_TEST_FILE_SIZE    (12 * 1024 * 1024)
#define SPEED_TEST_CHUNK_SIZE   (256 * 1024)
/* Out of byte range on purpose: selects the pseudo-random fill. */
#define SPEED_TEST_FILL_RAND    0x100u
/* Where each round's data physically lands. See __cli_sdcard_speed_test_open(). */
#define SPEED_TEST_MODE_TRUNC   0
#define SPEED_TEST_MODE_INPLACE 1

/**
 * @brief Fill the write buffer with the requested payload pattern.
 * @param[out] buf buffer to fill
 * @param[in] size buffer size in bytes
 * @param[in] fill byte value, or SPEED_TEST_FILL_RAND for incompressible data
 * @return none
 * @note A constant memset() pattern is trivially compressed or deduplicated by
 *       some SD-NAND parts, so a constant fill cannot tell "the card collapses
 *       repetitive data" apart from "the card has a fast first-write window".
 *       xorshift32 gives a cheap non-repeating stream for that comparison, and
 *       is the better default anyway - real payloads (H.264, JPEG) are already
 *       compressed and behave like random data.
 */
static void __cli_sdcard_fill_buf(uint8_t *buf, uint32_t size, uint32_t fill)
{
    uint32_t x = 0x12345678u;
    uint32_t *p = (uint32_t *)buf;
    uint32_t words = size / sizeof(uint32_t);

    if (fill != SPEED_TEST_FILL_RAND) {
        memset(buf, (uint8_t)fill, size);
        return;
    }

    for (uint32_t i = 0; i < words; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        p[i] = x;
    }
}

/**
 * @brief Write the test file once at full length, untimed.
 * @param[in] fp full path
 * @param[in] buf payload buffer
 * @param[in] size bytes per tkl_fwrite() call
 * @param[in] test_times loop bound (see __cli_sdcard_speed_test_opt)
 * @return 0 on success, -1 on failure
 */
static int __cli_sdcard_speed_test_lay_file(const char *fp, uint8_t *buf,
                                            uint32_t size, uint32_t test_times)
{
    uint32_t cnt = 0;
    TUYA_FILE f = tkl_fopen(fp, "wb");

    if (f == NULL) {
        return -1;
    }

    do {
        tkl_fwrite(buf, size, f);
    } while (cnt++ < test_times);

    tkl_fsync((int)f);
    tkl_fclose(f);
    return 0;
}

/**
 * @brief Open the test file for the timed write.
 * @param[in] fp full path
 * @param[in] buf payload buffer, used only if the file has to be laid down
 * @param[in] size bytes per tkl_fwrite() call
 * @param[in] test_times loop bound
 * @param[in] mode SPEED_TEST_MODE_TRUNC or SPEED_TEST_MODE_INPLACE
 * @return open handle, or NULL on failure
 * @note TRUNC ("wb") makes FATFS free and re-allocate the cluster chain, so
 *       the file lands somewhere new each round - that placement drift is the
 *       suspected cause of rounds jumping between discrete speeds. INPLACE
 *       ("r+b" rewound to 0) rewrites the same physical sectors every round,
 *       which isolates cluster placement from the measurement.
 */
static TUYA_FILE __cli_sdcard_speed_test_open(const char *fp, uint8_t *buf,
                                              uint32_t size, uint32_t test_times,
                                              uint8_t mode)
{
    TUYA_FILE f;

    if (mode == SPEED_TEST_MODE_TRUNC) {
        return tkl_fopen(fp, "wb");
    }

    f = tkl_fopen(fp, "r+b");
    if (f != NULL) {
        /* A short file would grow during the timed write and allocate fresh
         * clusters, defeating the point of this mode. Re-lay it full length. */
        if ((tkl_fseek(f, 0, SEEK_END) != 0) ||
            (tkl_ftell(f) < (INT64_T)SPEED_TEST_FILE_SIZE)) {
            tkl_fclose(f);
            f = NULL;
        }
    }

    if (f == NULL) {
        bk_printf("[speed_test] laying down %s at full length (untimed)\r\n", fp);
        if (__cli_sdcard_speed_test_lay_file(fp, buf, size, test_times) != 0) {
            return NULL;
        }
        f = tkl_fopen(fp, "r+b");
        if (f == NULL) {
            return NULL;
        }
    }

    if (tkl_fseek(f, 0, SEEK_SET) != 0) {
        tkl_fclose(f);
        return NULL;
    }

    return f;
}

/**
 * @brief One write pass of SPEED_TEST_FILE_SIZE bytes.
 * @param[in] size bytes per tkl_fwrite() call
 * @param[in] fill byte value, or SPEED_TEST_FILL_RAND
 * @param[in] mode SPEED_TEST_MODE_TRUNC or SPEED_TEST_MODE_INPLACE
 * @return elapsed milliseconds (write + fsync), 0 on failure
 * @note The buffer is always filled. Leaving it uninitialized made the result
 *       depend on PSRAM residue - the first pass after boot hit mostly-zero
 *       memory while later passes saw random residue, so consecutive runs were
 *       not comparable (and it wrote leftover memory content to the card).
 */
static uint32_t __cli_sdcard_speed_test_opt(uint32_t size, uint32_t fill, uint8_t mode)
{
    uint32_t cnt = 0;
    char *path = "speed_file";
    char fp[64] = {'\0'};
    uint8_t *buf = NULL;
    uint32_t test_times = 0;
    TUYA_FILE f = NULL;
    SYS_TIME_T t1, t2;

    sprintf(fp, "%s/%s", TEST_SDCARD_MOUNT_POINT, path);

    buf = tkl_system_psram_malloc(size);
    if (buf == NULL) {
        bk_printf("alloc %u byte test buffer failed\r\n", size);
        return 0;
    }
    __cli_sdcard_fill_buf(buf, size, fill);

    /* One extra pass beyond test_times, so the loop writes exactly
     * SPEED_TEST_FILE_SIZE. */
    test_times = (SPEED_TEST_FILE_SIZE / size) - 1;

    f = __cli_sdcard_speed_test_open(fp, buf, size, test_times, mode);
    if (f == NULL) {
        bk_printf("open %s failed\r\n", path);
        tkl_system_psram_free(buf);
        return 0;
    }

    t1 = tkl_system_get_millisecond();
    do {
        tkl_fwrite(buf, size, f);
    } while (cnt++ < test_times);

    tkl_fsync((int)f);
    t2 = tkl_system_get_millisecond();

    tkl_fclose(f);
    tkl_system_psram_free(buf);

    return (uint32_t)(t2 - t1);
}

/**
 * @brief Run the write speed test for several rounds and print each round.
 * @param[in] rounds number of write passes
 * @param[in] mode SPEED_TEST_MODE_TRUNC or SPEED_TEST_MODE_INPLACE
 * @param[in] fill byte value, or SPEED_TEST_FILL_RAND
 * @param[in] idle_s seconds to idle between rounds, 0 for none
 * @return none
 * @note Mounts once for the whole run on purpose: re-mounting between rounds
 *       re-inits the SDIO host and would mix that into the per-round numbers.
 *       Rounds jump between a few discrete speeds in TRUNC mode, so a single
 *       round is not representative - use several. `idle_s` leaves the card
 *       mounted but idle, to check whether an internal GC or write-buffer
 *       flush changes the next round.
 */
static void __cli_sdcard_speed_test(uint32_t rounds, uint8_t mode,
                                    uint32_t fill, uint32_t idle_s)
{
    TUYA_FS_STAT_T stat = {0};
    const char *mode_str = (mode == SPEED_TEST_MODE_INPLACE) ? "inplace" : "trunc";
    char fill_str[8];

    if (__cli_sdcard_mount() != 0) {
        bk_printf("speed_test mount failed\r\n");
        return;
    }

    if (fill == SPEED_TEST_FILL_RAND) {
        strcpy(fill_str, "rand");
    } else {
        sprintf(fill_str, "0x%02x", (unsigned)fill);
    }

    /* Cluster size is the unit FATFS allocates in, so log it next to the
     * numbers - it is what the placement hypothesis is about. */
    if (tkl_fs_statfs(TEST_SDCARD_MOUNT_POINT, &stat) == 0) {
        bk_printf("[speed_test] mode=%s fill=%s chunk=%uKB block_size=%u free_blocks=%u\r\n",
                  mode_str, fill_str, SPEED_TEST_CHUNK_SIZE >> 10,
                  stat.block_size, stat.free_blocks);
    }

    for (uint32_t i = 0; i < rounds; i++) {
        uint32_t ms = __cli_sdcard_speed_test_opt(SPEED_TEST_CHUNK_SIZE, fill, mode);
        uint32_t speed;

        if (ms == 0) {
            break;
        }

        speed = ((SPEED_TEST_FILE_SIZE >> 20) * 1000 * 100) / ms;
        bk_printf("[speed_test] round %u/%u mode=%s fill=%s: %u ms, %u.%02u MB/s\r\n",
                  i + 1, rounds, mode_str, fill_str, ms, speed / 100, speed % 100);

        if ((idle_s != 0) && ((i + 1) < rounds)) {
            bk_printf("[speed_test] idle %u s\r\n", idle_s);
            tkl_system_sleep(idle_s * 1000);
        }
    }

    tkl_system_sleep(100);
    __cli_sdcard_umount();
}

extern OPERATE_RET tkl_io_pinmux_config(TUYA_PIN_NAME_E pin, TUYA_PIN_FUNC_E pin_func);
extern void user_sdio_set_drive_capability(TUYA_GPIO_DRIVE_CAP_E strength);
extern OPERATE_RET user_sdio_request_bus_width(uint8_t width_bits);
extern uint8_t user_sdio_get_bus_width(void);

/**
 * @brief Route the SDIO1 pin group for the EVB.
 * @param[in] width_bits 1 for CLK/CMD/DATA0 only, 4 to add DATA1~DATA3
 * @return none
 * @note Call before mount. 4-bit needs the full group muxed, otherwise
 *       sdio_board_bus_width() downgrades the bus back to 1-bit at init.
 */
static void __cli_sdcard_port_test(uint8_t width_bits)
{
    tkl_io_pinmux_config(TUYA_IO_PIN_14, TUYA_SDIO_CLK);
    tkl_io_pinmux_config(TUYA_IO_PIN_15, TUYA_SDIO_CMD);
    tkl_io_pinmux_config(TUYA_IO_PIN_16, TUYA_SDIO_DATA0);

    if (width_bits == 4) {
        tkl_io_pinmux_config(TUYA_IO_PIN_17, TUYA_SDIO_DATA1);
        tkl_io_pinmux_config(TUYA_IO_PIN_18, TUYA_SDIO_DATA2);
        tkl_io_pinmux_config(TUYA_IO_PIN_19, TUYA_SDIO_DATA3);
    }

    bk_printf("[xsd port] sdio1 group muxed for %d-bit\r\n", width_bits);
}

/**
 * @brief Select the SD bus width from CLI.
 * @param[in] width_str "1" or "4"
 * @return none
 * @note Takes effect on the next mount, so run it while unmounted. 4-bit also
 *       needs DATA1~DATA3 routed (`xsd port 4`).
 */
static void __cli_sdcard_set_width(const char *width_str)
{
    unsigned long width = 0;

    if (width_str == NULL) {
        bk_printf("usage: xsd width <1|4>  (current=%d bit)\r\n", user_sdio_get_bus_width());
        return;
    }

    width = os_strtoul(width_str, NULL, 10);
    if (user_sdio_request_bus_width((uint8_t)width) != OPRT_OK) {
        return;
    }

    bk_printf("[xsd width] request %lu-bit, effective on next mount\r\n", width);
}

/**
 * @brief Set SDIO pad drive capability from CLI.
 * @param[in] cap_str drive level string, "0".."3"
 * @return none
 * @note Call before mount, or after GPIO has been mapped to apply immediately.
 */
static void __cli_sdcard_set_drive(const char *cap_str)
{
    unsigned long cap = 0;

    if (cap_str == NULL) {
        bk_printf("usage: xsd drive <0..3>\r\n");
        return;
    }

    cap = os_strtoul(cap_str, NULL, 10);
    if (cap > (unsigned long)TUYA_GPIO_DRIVE_CAP_3) {
        bk_printf("invalid drive_cap=%s, valid 0..3\r\n", cap_str);
        return;
    }

    user_sdio_set_drive_capability((TUYA_GPIO_DRIVE_CAP_E)cap);
    bk_printf("[xsd drive] set drive_cap=%lu\r\n", cap);
}

void cli_sdcard_test_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if (argc < 2) {
        bk_printf("invalid argc num\r\n");
        return;
    }

    if (!os_strcmp(argv[1], "mount")) {
        __cli_sdcard_mount();
    } else if (!os_strcmp(argv[1], "umount")) {
        __cli_sdcard_umount();
    } else if (!os_strcmp(argv[1], "port")) {
        /* `xsd port` keeps the 1-bit group; `xsd port 4` adds DATA1~DATA3. */
        __cli_sdcard_port_test((argv[2] != NULL) ? (uint8_t)os_strtoul(argv[2], NULL, 10) : 1);
    } else if (!os_strcmp(argv[1], "width") || !os_strcmp(argv[1], "bus_width")) {
        __cli_sdcard_set_width(argv[2]);
    } else if (!os_strcmp(argv[1], "drive") || !os_strcmp(argv[1], "drv_cap")) {
        __cli_sdcard_set_drive(argv[2]);
    } else if (!os_strcmp(argv[1], "ls")) {
        __cli_sdcard_list_file(TEST_SDCARD_MOUNT_POINT);
    } else if (!os_strcmp(argv[1], "read")) {
        if (argv[2] == NULL) {
            bk_printf("no file name\r\n");
            return;
        }
        __cli_sdcard_read(argv[2]);
    } else if (!os_strcmp(argv[1], "write")) {
        if (argv[2] == NULL) {
            bk_printf("no file name\r\n");
            return;
        }
        if (argv[3] == NULL) {
            bk_printf("no data\r\n");
            return;
        }
        uint32_t data_len = strlen(argv[3]);
        if (data_len == 0) {
            bk_printf("data len is zero\r\n");
            return;
        }

        __cli_sdcard_write(argv[2], argv[3], data_len);
    } else if (!os_strcmp(argv[1], "rmdir")) {
        if (argv[2] == NULL) {
            bk_printf("no spec dir\r\n");
            return;
        }
        tkl_fs_remove(argv[2]);
    } else if (!os_strcmp(argv[1], "mkdir")) {
        if (argv[2] == NULL) {
            bk_printf("no spec dir\r\n");
            return;
        }
        __cli_sdcard_mkdir(argv[2]);
    } else if (!os_strcmp(argv[1], "delete")) {
        if (argv[2] == NULL) {
            bk_printf("no file name\r\n");
            return;
        }
        __cli_sdcard_unlink(argv[2]);
    } else if (!os_strcmp(argv[1], "exist") || !os_strcmp(argv[1], "is_exist")) {
        if (argv[2] == NULL) {
            bk_printf("usage: xsd exist <path>\r\n");
            return;
        }
        __cli_sdcard_is_exist(argv[2]);
    } else if (!os_strcmp(argv[1], "format")) {
        int ret = tkl_fs_unmount(TEST_SDCARD_MOUNT_POINT);
        if (ret != 0) {
            bk_printf("[xsd format] unmount %s ret=%d (ignored if not mounted)\r\n",
                      TEST_SDCARD_MOUNT_POINT, ret);
        }

        ret = tkl_fs_format(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD);
        if (ret != 0) {
            bk_printf("[xsd format] failed, ret=%d\r\n", ret);
        } else {
            bk_printf("[xsd format] ok\r\n");
        }
    } else if (!os_strcmp(argv[1], "df")) {
        TUYA_FS_STAT_T stat = {0};
        int ret = tkl_fs_statfs(TEST_SDCARD_MOUNT_POINT, &stat);
        if (ret != 0) {
            bk_printf("[xsd df] failed, ret=%d\r\n", ret);
        } else {
            uint32_t total_mb = (uint32_t)(stat.total_bytes >> 20);
            uint32_t free_mb  = (uint32_t)(stat.free_bytes  >> 20);
            uint32_t used_mb  = total_mb - free_mb;
            uint32_t used_pct = (stat.total_bytes == 0)
                                ? 0
                                : (uint32_t)((stat.total_bytes - stat.free_bytes) * 100ULL
                                             / stat.total_bytes);
            bk_printf("[xsd df] %s: total=%u MB, used=%u MB, free=%u MB, used=%u%%\r\n",
                      TEST_SDCARD_MOUNT_POINT, total_mb, used_mb, free_mb, used_pct);
            bk_printf("[xsd df] block_size=%u, blocks total/free/avail = %u / %u / %u\r\n",
                      stat.block_size, stat.total_blocks, stat.free_blocks, stat.avail_blocks);
        }
    } else if (!os_strcmp(argv[1], "auto")) {
        int ret = __cli_sdcard_mount();
        if (ret != 0)
            return;
        __cli_sdcard_list_file(TEST_SDCARD_MOUNT_POINT);
        __cli_sdcard_umount();
    } else if (!os_strcmp(argv[1], "speed_test")) {
        /* xsd speed_test [rounds] [trunc|inplace] [rand|<byte>] [idle_s] */
        uint32_t rounds = (argc > 2) ? (uint32_t)os_strtoul(argv[2], NULL, 0) : 1;
        uint8_t  mode   = SPEED_TEST_MODE_TRUNC;
        uint32_t fill   = SPEED_TEST_FILL_RAND;
        uint32_t idle_s = (argc > 5) ? (uint32_t)os_strtoul(argv[5], NULL, 0) : 0;

        if (argc > 3) {
            if (!os_strcmp(argv[3], "inplace")) {
                mode = SPEED_TEST_MODE_INPLACE;
            } else if (!os_strcmp(argv[3], "trunc")) {
                mode = SPEED_TEST_MODE_TRUNC;
            } else {
                bk_printf("usage: xsd speed_test [rounds] [trunc|inplace] [rand|<byte>] [idle_s]\r\n");
                return;
            }
        }
        if (argc > 4 && os_strcmp(argv[4], "rand")) {
            fill = (uint32_t)os_strtoul(argv[4], NULL, 0) & 0xFF;
        }
        if (rounds == 0) {
            rounds = 1;
        }
        __cli_sdcard_speed_test(rounds, mode, fill, idle_s);
    } else if (!os_strcmp(argv[1], "wr")) {
        uint32_t loops = 0;

        if (argv[2] == NULL) {
            bk_printf("usage: xsd wr <loops>\r\n");
            return;
        }

        loops = os_strtoul(argv[2], NULL, 10);
        __cli_sdcard_loop_wr(loops);
    } else {
        bk_printf("unknown cmd: %s\r\n", argv[1]);
    }
}
