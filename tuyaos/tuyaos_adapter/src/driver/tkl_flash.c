#include "tuya_cloud_types.h"
#include "tkl_flash.h"

#include "tkl_output.h"
#include <common/bk_typedef.h>
#include "driver/flash.h"
/* 4.0.1.11 moved flash protection into the shared flash core (flash_core_*). */
#include "bk_flash_core.h"
#include "tkl_mutex.h"

typedef struct
{
    BOOL            is_start;
    unsigned long   set_ms;
    unsigned long   start;
    unsigned long   current;
} TUYA_OS_STORAGE_TIMER;
//flash 最大持续处理时间
#define FLASH_MAX_HANDLE_KEEP_TIME 10000    //10s

/* TODO: need to consider whether to use locks at the TKL layer*/

// total: 800000
// Name     Begin       End         Length
// boot     0x0         0x10000     64k
// cp       0x10000     0x138000    1184k
// ap       0x138000    0x4D0000    3680k
// ota      0x4D0000    0x7E2000    3144k
// usr      0x7E2000    0x7E4000    8k
// kvp      0x7E4000    0x7E5000    4k
// res      0x7E5000    0x7ED000    32k
// sleep    0x7ED000    0x7EE000    4k
// uf       0x7EE000    0x7F6000    32k
// kv       0x7F6000    0x7FE000    32k
// rf       0x7FE000    0x7FF000    4k
// net      0x7FF000    0x800000    4k

#define PARTITION_SIZE         (1 << 12) /* 4KB */
#define FLH_BLOCK_SZ            PARTITION_SIZE

#if defined(KV_PROTECTED_ENABLE) && (KV_PROTECTED_ENABLE==1)
    #define TUYA_KV_PROTECTED           0x7E4000
    #define TUYA_KV_PROTECTED_SIZE      (4 * 1024)
#endif

#define TUYA_USER0                      0x7E5000
#define TUYA_USER0_SIZE                 (32 * 1024)

#define TUYA_UF                         0x7EE000
#define TUYA_UF_SIZE                    (32 * 1024)

#define TUYA_KV                         0x7F6000
#define TUYA_KV_SIZE                    (32 * 1024)

#define FACTORY_RF_DATA_START           0x7FE000
#define FACTORY_RF_DATA_SIZE            (4 * 1024)

#define FACTORY_FAST_CONNECT_DATA_START 0x7FF000
#define FACTORY_FAST_CONNECT_DATA_SIZE  (4 * 1024)

static TKL_MUTEX_HANDLE __tkl_flash_mutex = NULL;
static OPERATE_RET __tkl_flash_opt_lock(void)
{
    OPERATE_RET ret = 0;
    if (__tkl_flash_mutex == NULL) {
        ret = tkl_mutex_create_init(&__tkl_flash_mutex);
        if (ret != 0) {
            bk_printf("flash mutex create failed\r\n");
            return OPRT_COM_ERROR;
        }
    }

    if (__tkl_flash_mutex)
        tkl_mutex_lock(__tkl_flash_mutex);

    return OPRT_OK;
}

static OPERATE_RET __tkl_flash_opt_unlock(void)
{
    if (__tkl_flash_mutex)
        tkl_mutex_unlock(__tkl_flash_mutex);
    return OPRT_OK;
}
/**
 * @brief flash 设置保护,enable 设置ture为全保护，false为半保护
 *
 * @return OPRT_OK
 */
/* 4.0.1.11 removed bk_flash_get/set_protect_type() and moved protection into the
 * flash core: it re-asserts this "runtime" type around every op (PER_OP
 * re-protect) and there is no getter, so the requested type is tracked here. */
static unsigned int s_flash_protect_type = FLASH_PROTECT_ALL;

OPERATE_RET tkl_flash_set_protect(const BOOL_T enable)
{
    unsigned int  param;

    if (enable) {
        param = FLASH_PROTECT_ALL;
    } else {
        param = FLASH_PROTECT_NONE;
    }

    s_flash_protect_type = param;
    flash_core_set_runtime_protect_type((flash_protect_type_t)param);

    return OPRT_OK;
}


/**
* @brief read flash
*
* @param[in] addr: flash address
* @param[out] dst: pointer of buffer
* @param[in] size: size of buffer
*
* @note This API is used for reading flash.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_flash_read(UINT_T addr, UCHAR_T *dst, UINT_T size)
{
    if (NULL == dst) {
        return OPRT_INVALID_PARM;
    }

    /* TODO: need to consider whether to use locks at the TKL layer*/
    __tkl_flash_opt_lock();

    bk_flash_read_bytes(addr, (uint8_t *)dst, size);

    /* TODO: need to consider whether to use locks at the TKL layer*/
    __tkl_flash_opt_unlock();

    return OPRT_OK;
}

static unsigned int __uni_flash_is_protect_all(void)
{
    return (FLASH_PROTECT_ALL == s_flash_protect_type ||
            FLASH_UNPROTECT_LAST_BLOCK == s_flash_protect_type);
}

/**
* @brief write flash
*
* @param[in] addr: flash address
* @param[in] src: pointer of buffer
* @param[in] size: size of buffer
*
* @note This API is used for writing flash.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_flash_write(UINT_T addr, CONST UCHAR_T *src, UINT_T size)
{
    if (NULL == src) {
        return OPRT_INVALID_PARM;
    }

    /* TODO: need to consider whether to use locks at the TKL layer*/
    __tkl_flash_opt_lock();

    /* 4.0.1.11: the flash core brackets protection per op; no manual unprotect. */
    bk_flash_write_bytes(addr, (const uint8_t *)src, size);

    /* TODO: need to consider whether to use locks at the TKL layer*/
    __tkl_flash_opt_unlock();

    return OPRT_OK;
}

/**
* @brief erase flash
*
* @param[in] addr: flash address
* @param[in] size: size of flash block
*
* @note This API is used for erasing flash.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_flash_erase(UINT_T addr, UINT_T size)
{
    unsigned short start_sec = (addr / PARTITION_SIZE);
    unsigned short end_sec = ((addr + size - 1) / PARTITION_SIZE);
    unsigned int i = 0;
    unsigned int sector_addr;

    /* TODO: need to consider whether to use locks at the TKL layer*/
    __tkl_flash_opt_lock();

    /* 4.0.1.11: the flash core brackets protection per op; no manual unprotect. */
    for (i = start_sec; i <= end_sec; i++) {
        sector_addr = PARTITION_SIZE * i;
        bk_flash_erase_sector(sector_addr);
        tkl_system_sleep(2);
    }

    /* TODO: need to consider whether to use locks at the TKL layer*/
    __tkl_flash_opt_unlock();

    return OPRT_OK;
}

/**
* @brief lock flash
*
* @param[in] addr: lock begin addr
* @param[in] size: lock area size
*
* @note This API is used for lock flash.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_flash_lock(UINT_T addr, UINT_T size)
{
    return OPRT_NOT_SUPPORTED;
}

/**
* @brief unlock flash
*
* @param[in] addr: unlock begin addr
* @param[in] size: unlock area size
*
* @note This API is used for unlock flash.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_flash_unlock(UINT_T addr, UINT_T size)
{
    return OPRT_NOT_SUPPORTED;
}

/**
* @brief get flash information
*
* @param[out] info: the description of the flash
*
* @note This API is used to get description of storage.
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_flash_get_one_type_info(TUYA_FLASH_TYPE_E type, TUYA_FLASH_BASE_INFO_T* info)
{
    if ((type > TUYA_FLASH_TYPE_MAX) || (info == NULL)) {
        return OPRT_INVALID_PARM;
    }
    switch (type) {
        case TUYA_FLASH_TYPE_UF:
            info->partition_num = 1;
            info->partition[0].block_size =  PARTITION_SIZE;
            info->partition[0].start_addr = TUYA_UF;
            info->partition[0].size = TUYA_UF_SIZE;
            break;
       case TUYA_FLASH_TYPE_KV_DATA:
            info->partition_num = 1;
            info->partition[0].block_size = FLH_BLOCK_SZ;
            info->partition[0].start_addr = TUYA_KV;
            info->partition[0].size = TUYA_KV_SIZE;
            break;
#if defined(KV_PROTECTED_ENABLE) && (KV_PROTECTED_ENABLE==1)
        case TUYA_FLASH_TYPE_KV_PROTECT:
            info->partition_num = 1;
            info->partition[0].block_size = TUYA_KV_PROTECTED_SIZE;
            info->partition[0].start_addr = TUYA_KV_PROTECTED;
            info->partition[0].size = TUYA_KV_PROTECTED_SIZE;
            break;
#endif
        case TUYA_FLASH_TYPE_USER0:
            info->partition_num = 1;
            info->partition[0].block_size =  PARTITION_SIZE;
            info->partition[0].start_addr = TUYA_USER0;
            info->partition[0].size = TUYA_USER0_SIZE;
            break;
            break;

        default:
            return OPRT_INVALID_PARM;
            break;
    }

    return OPRT_OK;

}

