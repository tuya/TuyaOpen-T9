/**
 * @file tkl_ota.c
 * @brief default weak implements of tuya ota, it only used when OS=linux
 *
 * @copyright Copyright 2020-2021 Tuya Inc. All Rights Reserved.
 *
 */
#include <string.h>
#include "tkl_ota.h"
#include "tuya_error_code.h"
#include "tkl_memory.h"
#include "tkl_flash.h"
#include "tkl_system.h"
#include "tkl_watchdog.h"
#include "tuya_cloud_types.h"

#if CONFIG_CPU_INDEX == 1

#define FLASH_SECTOR_SIZE 4096
#define BLOCK_SZ FLASH_SECTOR_SIZE

#define FIRMWARE_AREA           1
#define PATCH_AREA              2
#define BACKUP_AREA             3
#define MANAGER_AREA            4
#define LAST_WRITE_FIRMWARE     5
#define INVALID_ARG             255

#define TO_PHYSICS   1
#define TO_VIRTUAL   0
#define BK_ADDR_CHANGE(_TYPE_,_ADDR_)  ((((_TYPE_) == TO_PHYSICS) ? ((_ADDR_) / 32) * 34 : ((_ADDR_) / 34) * 32))
#define v_to_p(a) ((a / 32) * 34 + a % 32)
#define VIRTUAL_TO_AB_PHYSICS(addr) (v_to_p(addr) - v_to_p(addr) % BLOCK_SZ)

#define len_v_to_p(a) ((a / 32) * 34 + ((a % 32) ? 34 : 0))

typedef enum {
    UGS_RECV_HEADER = 0,
    UGS_RECV_IMG_DATA,
    UGS_RECV_IMG_DATA_B,
    UGS_FINISH
}UG_STAT_E;

typedef enum {
    TYPE_DIFF = 0,
    TYPE_SEG_A,
    TYPE_SEG_B
}OTA_TYPE_E;


typedef struct {
    UINT_T start_addr;
    UINT_T cur_addr;
    UINT_T recv_data_cnt;
    UINT_T bin_len;
    UINT_T bin_type;
    UINT_T offset;
    UCHAR_T firmware_is_crc;  //0- 固件区不需要加crc校验， 1- 固件区需要加crc校验（bk平台）
    UG_STAT_E stat;
    OTA_TYPE_E ota_type;//0-diff; 1-seg_A; 2-seg_B
}UG_PROC_S;

#define OTA_GAP_SIZE                    (512 * 1024)

/**
 * @brief BK segmented OTA erase headroom check (32B virtual <-> 34B physical).
 * @note Legacy used linker symbols __vector_core0_table and _ty_section_start to get
 *       (tuya_section - vector_base) + 1MiB as raw_size. That ties the adapter to SMP/Tuya
 *       ld layout. Here raw_size is derived only from TUYA_FLASH_TYPE_APP partition size.
 */
static int inline __tkl_ota_erase_limit_check(UINT32_T patch_size)
{
    TUYA_FLASH_BASE_INFO_T info;
    UINT32_T part_sz;
    UINT32_T max_v;
    UINT32_T raw_size;
    UINT32_T occupy_size;
    UINT32_T available_space;
    UINT32_T max_available;

    if (tkl_flash_get_one_type_info(TUYA_FLASH_TYPE_APP, &info) != OPRT_OK) {
        return 1;
    }

    part_sz = info.partition[0].size;
    max_v = (part_sz / 34U) * 32U;
    raw_size = max_v;
    if (raw_size > 1048576U) {
        raw_size -= 1048576U;
    }
    occupy_size = (raw_size / 32U) * 34U;
    if (part_sz > occupy_size) {
        available_space = part_sz - occupy_size;
    } else {
        available_space = 0;
    }
    if (available_space > (UINT32_T)OTA_GAP_SIZE) {
        max_available = available_space - (UINT32_T)OTA_GAP_SIZE;
    } else {
        max_available = 0;
    }

    /* 纯压缩UG包在生成时候就已经校验通过，此处仅判断可覆盖分段包长度是否允许 */
    if (patch_size < max_available) {
        return 1;
    }

    bk_printf("ota package check error, max patch allowed: %d, actually: %d\r\n", max_available, patch_size);
    return 0;
}

/***********************************************************
*************************variable define********************
***********************************************************/
#define SEC_B_UNIT (1024)
#define FLASH_BASE_ADDR (0x02000000)

static UINT_T flash_area = INVALID_ARG;
static UG_PROC_S *ug_proc = NULL;
static UCHAR_T tkl_fist_flag = 0;
static UINT_T flash_crc32 = 0;

extern OPERATE_RET tkl_flash_set_protect(const BOOL_T enable);
extern void bk_printf(const char *fmt, ...);

__attribute__((section(".iram")))void *mem_cpy(void *dest, const void *src, size_t count)
{
	char 		*tmp = dest;
	const char 	*s = src;

	while (count--)
		*tmp++ = *s++;
	return dest;
}

extern unsigned int hash_crc32i_init(void);
extern unsigned int hash_crc32i_update(unsigned int hash, const void *data, unsigned int size);
extern unsigned int hash_crc32i_finish(unsigned int hash);
UINT_T  _flash_crc32_cal(UINT_T addr, UINT_T size)
{
    UINT_T read_block = 4096;

    UCHAR_T *read_buffer = tkl_system_malloc(read_block);
    if(read_buffer == NULL) {
        bk_printf("malloc error\r\n");
        return 0;
    }

    UINT_T read_len = 0;
    UINT_T pos = 0;

    UINT_T crc32 = hash_crc32i_init();

    while (1) {
        read_len = size - pos > read_block ? read_block : size - pos;
        tkl_flash_set_protect(FALSE);
        mem_cpy(read_buffer, (UCHAR_T *)(addr + pos)+FLASH_BASE_ADDR, read_len);
        tkl_flash_set_protect(TRUE);

        crc32 = hash_crc32i_update(crc32, read_buffer, read_len);
        pos += read_len;
        if (pos >= size) {
            break;
        }
    }
    crc32 = hash_crc32i_finish(crc32);

    tkl_system_free(read_buffer);
    return crc32;
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
OPERATE_RET tkl_ota_flash_erase(uint32_t addr, uint32_t size, void* arg)
{
    uint32_t param = *((uint32_t*)arg);
    uint32_t real_size = size;
    uint32_t address = addr;

    if(param == FIRMWARE_AREA) {
        uint32_t phy_size =  (size % 32) ? (size / 32 * 34 + 34) : (size / 32 * 34);
        uint32_t off_size = BK_ADDR_CHANGE(TO_PHYSICS, addr) % FLASH_SECTOR_SIZE;
        real_size = ((off_size+phy_size)%FLASH_SECTOR_SIZE) ? ((((off_size+phy_size)/FLASH_SECTOR_SIZE)+1)*FLASH_SECTOR_SIZE) : (off_size+phy_size);
        address = BK_ADDR_CHANGE(TO_PHYSICS, addr) - off_size;
    } else {
        real_size = (size%FLASH_SECTOR_SIZE) ? ((size/FLASH_SECTOR_SIZE) + 1)*FLASH_SECTOR_SIZE : size;
    }
    bk_printf("tkl_ota_flash_erase:%x %p, %d\r\n",address, param, real_size);
    tkl_watchdog_refresh();
    //tkl_flash_set_protect(FALSE);
    tkl_flash_erase(address, real_size);//这里的擦除的size 是否需要改为：off_size+len_v_to_p(size)
    //tkl_flash_set_protect(TRUE);
    tkl_watchdog_refresh();

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
OPERATE_RET tkl_ota_flash_read(uint32_t addr, uint8_t *dst, uint32_t size)
{
    return tkl_flash_read(addr, dst, size);
}

extern void ty_calc_crc(uint32_t *buf, uint32_t packet_num);
extern void ty_get_calc_buf(uint8_t *rx, uint8_t *tx, uint32_t num);
//extern void ty_encrypt(uint32_t *rx, uint16_t *tx, uint32_t num,uint32_t  addr0);

static uint8_t bbuf[1024 / 32 * 34 + 34];
static uint8_t tbuf[32];
OPERATE_RET tkl_ota_flash_write(uint32_t addr, uint8_t *buf, uint32_t len, void* arg)
{
    uint32_t param = *((uint32_t*)arg);
    static uint8_t last_data_cnt = 0;
    //bk_printf("tkl_ota_flash_write:%x, %d, %d\r\n",addr,len, param);

    if(param == FIRMWARE_AREA || param == LAST_WRITE_FIRMWARE) {//write firmware
        uint32_t off = 0;
        uint32_t encrypt_num = 0;
        uint32_t lcnt = len;
        uint32_t address = addr;
        //bk_printf("last cnt:%d, len:%d\r\n",last_data_cnt, len);
        if(last_data_cnt) {
            if((lcnt + last_data_cnt) >= 32) {
                memcpy(tbuf+last_data_cnt, buf, 32-last_data_cnt);
                ty_get_calc_buf(tbuf, bbuf, 1);
                ty_calc_crc((uint32_t *)(bbuf), 1);
                lcnt -= 32-last_data_cnt;
                off += 32-last_data_cnt;
                address -= last_data_cnt;
                last_data_cnt = 0;
                tkl_watchdog_refresh();
                //tkl_flash_set_protect(FALSE);
                if(tkl_flash_write(BK_ADDR_CHANGE(TO_PHYSICS, address), bbuf, 34)) {
                    //tkl_flash_set_protect(TRUE);
                    bk_printf("Write sector failed\r\n");
                    return OPRT_OS_ADAPTER_OTA_PROCESS_FAILED;
                }
                //tkl_flash_set_protect(TRUE);
                tkl_watchdog_refresh();
                address += 32;
            }
        }
        while(lcnt >= 32) {
            encrypt_num = (lcnt > SEC_B_UNIT) ? (SEC_B_UNIT / 32) : lcnt / 32;
            ty_get_calc_buf((buf+off), bbuf, encrypt_num);
            ty_calc_crc((uint32_t *)bbuf, encrypt_num);
            tkl_watchdog_refresh();

            //tkl_flash_set_protect(FALSE);
            if(tkl_flash_write(BK_ADDR_CHANGE(TO_PHYSICS, address), bbuf, encrypt_num*34)) {
                //tkl_flash_set_protect(TRUE);
                bk_printf("Write sector failed\r\n");
                return OPRT_OS_ADAPTER_OTA_PROCESS_FAILED;
            }
            //tkl_flash_set_protect(TRUE);
            tkl_watchdog_refresh();
            lcnt -= encrypt_num * 32;
            off += encrypt_num * 32;
            address += encrypt_num * 32;
        }

        if(lcnt && param != LAST_WRITE_FIRMWARE) {
            memcpy(tbuf + last_data_cnt, (buf+off), lcnt);
            last_data_cnt += lcnt;
        } else if(lcnt && param == LAST_WRITE_FIRMWARE) {
            bk_printf("last_data_cnt:%d\r\n", last_data_cnt);
            uint8_t restbuf[32];
            memset(restbuf, 0xff, 32);
            memcpy(restbuf, tbuf, last_data_cnt);
            memcpy(restbuf + last_data_cnt, buf+off, lcnt);

            ty_get_calc_buf(restbuf, bbuf, 1);
            ty_calc_crc((uint32_t *)bbuf, 1);
            tkl_watchdog_refresh();
            //tkl_flash_set_protect(FALSE);
            if(tkl_flash_write(BK_ADDR_CHANGE(TO_PHYSICS, address - last_data_cnt), bbuf, 34)) {
                //tkl_flash_set_protect(TRUE);
                bk_printf("Write sector failed\r\n");
                return OPRT_OS_ADAPTER_OTA_PROCESS_FAILED;
            }
            //tkl_flash_set_protect(TRUE);
            tkl_watchdog_refresh();
        }

    }else {
        tkl_watchdog_refresh();
        //tkl_flash_set_protect(FALSE);
        tkl_flash_write(addr, buf, len);
        //tkl_flash_set_protect(TRUE);
        tkl_watchdog_refresh();
    }

    return OPRT_OK;
}

/**
* @brief ota start notify with data
*
* @param[in] image_size:  image size
* @param[in] type:        ota type
* @param[in] path:        ota path
*
* @note This API is used for ota start notify
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_ota_start_notify_with_data(UINT_T image_size, TUYA_OTA_TYPE_E type, TUYA_OTA_PATH_E path)
{
    if(image_size == 0) {
        return OPRT_OS_ADAPTER_INVALID_PARM;
    }
    if(NULL == ug_proc) {
        ug_proc = tkl_system_malloc(sizeof(UG_PROC_S));
        if(NULL == ug_proc) {
            return OPRT_MALLOC_FAILED;
        }
    }
    memset(ug_proc, 0,sizeof(UG_PROC_S));
    ug_proc->bin_len = image_size;

    ug_proc->ota_type = TYPE_DIFF;
    if(path == TUYA_OTA_PATH_SEC_A) {
        ug_proc->ota_type = TYPE_SEG_A;
    } else if(path == TUYA_OTA_PATH_SEC_B) {
        ug_proc->ota_type = TYPE_SEG_B;
        ug_proc->stat = UGS_RECV_IMG_DATA_B;
    }

    // amp delete
    tkl_flash_erase(0x6c9000, 8192);

    bk_printf("%s %d, image_size %d, type %d, %d, %d\r\n", __func__, __LINE__, image_size, type, ug_proc->bin_len, ug_proc->ota_type,ug_proc->stat);
    return 0;

    //bk_printf("_ota_start_notify:%d, %d, %d, %d\r\n",type, ug_proc->bin_len, ug_proc->ota_type,ug_proc->stat);
}

/**
* @brief ota start notify
*
* @param[in] image_size:  image size
* @param[in] type:        ota type
* @param[in] path:        ota path
*
* @note This API is used for ota start notify
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_ota_start_notify(UINT_T image_size, TUYA_OTA_TYPE_E type, TUYA_OTA_PATH_E path)
{
    if(image_size == 0) {
        return OPRT_OS_ADAPTER_INVALID_PARM;
    }

    if(ug_proc == NULL) {
        ug_proc = tkl_system_malloc(sizeof(UG_PROC_S));
        if(NULL == ug_proc) {
            return OPRT_MALLOC_FAILED;
        }
    }
    memset(ug_proc, 0,sizeof(UG_PROC_S));
    ug_proc->bin_len = image_size;

    ug_proc->ota_type = TYPE_DIFF;
    if(path == TUYA_OTA_PATH_SEC_A) {
        ug_proc->ota_type = TYPE_SEG_A;
    } else if(path == TUYA_OTA_PATH_SEC_B) {
        ug_proc->ota_type = TYPE_SEG_B;
        ug_proc->stat = UGS_RECV_IMG_DATA_B;
    }

    return 0;

    //bk_printf("_ota_start_notify:%d, %d, %d, %d\r\n",type, ug_proc->bin_len, ug_proc->ota_type,ug_proc->stat);
}


/**
* @brief ota data process
*
* @param[in] pack:       point to ota pack
* @param[in] remain_len: ota pack remain len
*
* @note This API is used for ota data process
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_ota_data_process(TUYA_OTA_DATA_T *pack, UINT_T* remain_len)
{
    if(ug_proc == NULL) {
        bk_printf("ota don't start or start err,process error!\r\n");
        return OPRT_COM_ERROR;
    }

    if(ug_proc->ota_type == TYPE_SEG_A)
        return -1;

    //bk_printf("ug_proc->stat:%d\r\n",ug_proc->stat);
    if(ug_proc->stat == UGS_RECV_IMG_DATA_B) {
        pack->start_addr -= 65536;      // SMP覆盖分段升级时候，压缩包中包含64k bl2，bl2中升级A段时候，丢弃了这部分长度，但是sdk在计算时候未丢弃，此处减去
        if(!tkl_fist_flag) {
            tkl_fist_flag = 1;
            flash_crc32 = hash_crc32i_init();

            UCHAR_T *temp_buf = NULL;
            UINT_T off_size = BK_ADDR_CHANGE(pack->start_addr, TO_PHYSICS) % FLASH_SECTOR_SIZE;
            UINT_T address = BK_ADDR_CHANGE(pack->start_addr, TO_PHYSICS) - off_size;
            //bk_printf("off_size:%x,%x,%x\r\n",off_size, address, pack->start_addr);
            if(off_size != 0) {
                temp_buf = (UCHAR_T *) tkl_system_malloc(off_size);
                if(NULL == temp_buf) {
                    return OPRT_MALLOC_FAILED;
                }
                tkl_flash_read(address, temp_buf, off_size);
            }

            flash_area = FIRMWARE_AREA;

            bk_printf("%s %d, b-section, earse %x %x\r\n", __func__, __LINE__, pack->start_addr, ug_proc->bin_len);
            tkl_ota_flash_erase(pack->start_addr, ug_proc->bin_len, (VOID *)&flash_area);

            if(off_size != 0) {
                tkl_flash_write(address, temp_buf, off_size);
            }
            if(NULL != temp_buf) {
                tkl_system_free(temp_buf);
                temp_buf = NULL;
            }
        }

        flash_crc32 = hash_crc32i_update(flash_crc32, pack->data, pack->len);

        flash_area = FIRMWARE_AREA;
        if(ug_proc->recv_data_cnt + pack->len >= ug_proc->bin_len) {
            pack->len = ug_proc->bin_len - ug_proc->recv_data_cnt;
            bk_printf("recv finish222222\r\n");
            flash_area = LAST_WRITE_FIRMWARE;
        }

        //bk_printf("start_addr:%x\r\n",pack->start_addr);
        tkl_ota_flash_write(pack->start_addr, pack->data, pack->len, (VOID *)&flash_area);
        ug_proc->recv_data_cnt += pack->len;
        *remain_len = 0;

        if(flash_area == LAST_WRITE_FIRMWARE) {
            flash_crc32 = hash_crc32i_finish(flash_crc32);
            //bk_printf("crc:%x, %x, %x, %x\r\n",pack->start_addr, pack->len, ug_proc->bin_len, pack->start_addr + pack->len - ug_proc->bin_len);
            UINT_T result_crc32 = _flash_crc32_cal(pack->start_addr + pack->len - ug_proc->bin_len, ug_proc->bin_len);
            if(result_crc32 != flash_crc32) {
                bk_printf("Area B crc32 err: %x, %x\r\n", result_crc32, flash_crc32);
                tkl_system_reset();  //下载B段校验失败直接重启，重新拉取B段
            }
        }
    } else {
        if (!__tkl_ota_erase_limit_check(ug_proc->bin_len)) {
            return -1;
        }

        if(!tkl_fist_flag) {
            tkl_fist_flag = 1;
            flash_area = PATCH_AREA;
            bk_printf("%s %d, a-section, earse %x %x\r\n", __func__, __LINE__, pack->start_addr, ug_proc->bin_len);

            tkl_ota_flash_erase(pack->start_addr, ug_proc->bin_len, (VOID *)&flash_area);
        }
        flash_area = PATCH_AREA;
        tkl_ota_flash_write(pack->start_addr, pack->data, pack->len, (VOID *)&flash_area);
    }

    return OPRT_OK;
}

/**
 * @brief firmware ota packet transform success inform, can do verify in this funcion
 *
 * @param[in]        reset       need reset or not
 *
 * @return OPRT_OK on success, others on failed
 */
OPERATE_RET tkl_ota_end_notify(BOOL_T reset)
{
    tkl_fist_flag = 0;
    return OPRT_OK;
}


/**
* @brief get ota ability
*
* @param[out] image_size:  max image size
* @param[out] type:        bit0, 1 - support full package upgrade
                                 0 - dont support full package upgrade
*                          bit1, 1 - support compress package upgrade
                                 0 - dont support compress package upgrade
*                          bit2, 1 - support difference package upgrade
                                 0 - dont support difference package upgrade
* @note This API is used for get chip ota ability
*
* @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
*/
OPERATE_RET tkl_ota_get_ability(UINT_T *image_size, TUYA_OTA_TYPE_E *type)
{
    *image_size = (2040 + 288 + 1700) * 1024;

    *type = TUYA_OTA_FULL;

    return OPRT_OK;
}

#elif CONFIG_CPU_INDEX == 0

OPERATE_RET tkl_ota_get_ability(UINT_T *image_size, TUYA_OTA_TYPE_E *type)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_ota_start_notify(UINT_T image_size, TUYA_OTA_TYPE_E type, TUYA_OTA_PATH_E path)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_ota_data_process(TUYA_OTA_DATA_T *pack, UINT_T* remain_len)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_ota_end_notify(BOOL_T reset)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_ota_get_old_firmware_info(TUYA_OTA_FIRMWARE_INFO_T **info)
{
    return OPRT_NOT_SUPPORTED;
}

#endif
