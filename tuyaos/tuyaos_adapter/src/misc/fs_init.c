#include "os/os.h"
#include "bk_posix.h"
#include "driver/flash_partition.h"
#include "bk_partition.h"
#include "sdkconfig.h"
#include "tkl_system.h"

#include "driver/sd_card.h"
#include "driver/sd_card_types.h"

extern void user_sdio_gpio_init(void);

#if defined(BK_PARTITION_LITTLEFS)
#define BK_PARTITION_FS_ID BK_PARTITION_LITTLEFS
#else
#define BK_PARTITION_FS_ID BK_PARTITION_USR_CONFIG
#endif

int fatfs_mount(const char *mount_path, int type)
{
#if !(CONFIG_FATFS && CONFIG_SDCARD)
    (void)mount_path;
    (void)type;
    bk_printf("fatfs mount failed, CONFIG_FATFS or CONFIG_SDCARD is disabled\r\n");
    return BK_FAIL;
#else
    int ret;
    const char *fs_type = "fatfs";
    struct bk_fatfs_partition partition;
    (void)type;

    if (mount_path == NULL) {
        bk_printf("fatfs mount failed, no mount point\r\n");
        return -1;
    }

    os_memset(&partition, 0, sizeof(partition));
    partition.part_type = FATFS_DEVICE;
    partition.part_dev.device_name = FATFS_DEV_SDCARD;
    partition.mount_path = mount_path;

    // user_sdio_gpio_init();

    ret = mount("SOURCE_NONE", partition.mount_path, fs_type, 0, &partition);
    if (ret == 0) {
        sd_card_info_t card_info;
        bk_sd_card_get_card_info(&card_info);
        uint32_t cap = bk_sd_card_get_card_size();
        sd_card_state_t state = bk_sd_card_get_card_state();
        bk_printf("mount on sdcard, version: %d, type: %d, class: %d, rca: %d, size: %d, stat: %d\r\n",
                card_info.card_version, card_info.card_type, card_info.class,
                card_info.relative_card_addr, cap, state);
    }

    bk_printf("mount fatfs, %s %d\r\n", partition.mount_path, ret);

    return ret;
#endif
}

int littlefs_mount(const char *mount_path, int type)
{
#if CONFIG_LITTLEFS
    int ret;
    int retry = 0;
    struct bk_little_fs_partition partition;
    const char *fs_type = "littlefs";

    if (mount_path == NULL) {
        bk_printf("lfs mount failed, no mount point\r\n");
        return BK_FAIL;
    }

    os_memset(&partition, 0, sizeof(partition));

    if (type == LFS_FLASH) {
        bk_logic_partition_t *pt = bk_flash_partition_get_info(BK_PARTITION_FS_ID);

        if (pt == NULL) {
            bk_printf("lfs mount failed, no flash partition info\r\n");
            return BK_FAIL;
        }

        partition.part_type = LFS_FLASH;
        partition.part_flash.start_addr = pt->partition_start_addr;
        partition.part_flash.size = pt->partition_length;
    } else if (type == LFS_QSPI_FLASH) {
#if CONFIG_QSPI_MST_FLASH
        bk_printf("lfs mount failed, ext flash size is not configured in current adapter\r\n");
        return BK_FAIL;
#else
        bk_printf("lfs mount failed, CONFIG_QSPI_MST_FLASH is disabled\r\n");
        return BK_FAIL;
#endif
    } else {
        bk_printf("lfs mount failed, device type not support: %d\r\n", type);
        return BK_FAIL;
    }

    partition.mount_path = mount_path;

    do {
        ret = mount("SOURCE_NONE", partition.mount_path, fs_type, 0, &partition);
        if (ret == BK_OK) {
            bk_printf("mount littlefs %s ok\r\n", partition.mount_path);
            return BK_OK;
        }

        retry++;
        tkl_system_sleep(50);
    } while (retry < 3);

    bk_printf("mount littlefs %s failed, ret=%d\r\n", partition.mount_path, ret);
    return ret;
#else
    (void)mount_path;
    (void)type;
    bk_printf("littlefs mount failed, CONFIG_LITTLEFS is disabled\r\n");
    return BK_FAIL;
#endif
}
