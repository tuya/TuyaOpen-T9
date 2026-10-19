/**
 * @file tkl_ota.c
 * @brief TuyaOS OTA hardware abstraction for Beken T9 platform
 * @version 1.0
 * @date 2026-07-03
 * @copyright Copyright (c) Tuya Inc.
 */
#include "tkl_ota.h"
#include "tkl_flash.h"
#include "tkl_system.h"
#include "tkl_watchdog.h"
#include "sdkconfig.h"
#include <string.h>
#include <driver/flash.h>
#include <driver/flash_partition.h>
#include <modules/ota.h>
#if defined(CONFIG_OTA_HASH_FUNCTION) && defined(CONFIG_HTTP_AB_PARTITION)
#include "bk_private/bk_ota_private.h"
#endif

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define TKL_OTA_FLASH_SECTOR_SIZE       4096U
#define TKL_OTA_FLASH_BUFFER_LENGTH     1024U
#define TKL_OTA_PROGRESS_STEP_PCT       10U

#ifndef TKL_OTA_MIN
#define TKL_OTA_MIN(a, b)               (((a) < (b)) ? (a) : (b))
#endif

extern void bk_printf(const char *fmt, ...);

#ifdef CONFIG_HTTP_AB_PARTITION
extern part_flag update_part_flag;
int bk_ota_update_partition_flag(int input_val);
#if CONFIG_OTA_POSITION_INDEPENDENT_AB
uint32_t http_get_sapp_partition_length(bk_partition_t partition);
#endif
#endif

/* ---------------------------------------------------------------------------
 * Type definitions
 * --------------------------------------------------------------------------- */
typedef struct {
    UINT_T              image_size;
    UINT_T              received_total;
    UINT_T              wr_last_len;
    UINT_T              wr_address;
    UINT_T              partition_start;
    UINT_T              partition_length;
    UINT_T              last_progress_pct;
    UINT_T              flash_protect_type;
    BOOL_T              protect_saved;
    BOOL_T              first_packet;
    TUYA_OTA_TYPE_E     ota_type;
    TUYA_OTA_PATH_E     ota_path;
    UCHAR_T            *wr_buf;
    UCHAR_T            *rd_buf;
} TKL_OTA_CTX_T;

/* ---------------------------------------------------------------------------
 * File scope variables
 * --------------------------------------------------------------------------- */
STATIC TKL_OTA_CTX_T *s_ota_ctx = NULL;

#if CONFIG_CPU_INDEX == 0

/**
 * @brief Get OTA ability (CPU0 stub)
 * @param[out] image_size max firmware image size in bytes
 * @param[out] type supported OTA type
 * @return OPRT_NOT_SUPPORTED on CPU0
 */
OPERATE_RET tkl_ota_get_ability(UINT_T *image_size, TUYA_OTA_TYPE_E *type)
{
    (void)image_size;
    (void)type;
    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief OTA start notify (CPU0 stub)
 * @param[in] image_size firmware image size
 * @param[in] type OTA type
 * @param[in] path OTA path
 * @return OPRT_NOT_SUPPORTED on CPU0
 */
OPERATE_RET tkl_ota_start_notify(UINT_T image_size, TUYA_OTA_TYPE_E type, TUYA_OTA_PATH_E path)
{
    (void)image_size;
    (void)type;
    (void)path;
    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief OTA data process (CPU0 stub)
 * @param[in] pack OTA data packet
 * @param[out] remain_len unprocessed bytes in current packet
 * @return OPRT_NOT_SUPPORTED on CPU0
 */
OPERATE_RET tkl_ota_data_process(TUYA_OTA_DATA_T *pack, UINT_T *remain_len)
{
    (void)pack;
    (void)remain_len;
    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief OTA end notify (CPU0 stub)
 * @param[in] reset whether to reset after OTA
 * @return OPRT_NOT_SUPPORTED on CPU0
 */
OPERATE_RET tkl_ota_end_notify(BOOL_T reset)
{
    (void)reset;
    return OPRT_NOT_SUPPORTED;
}

/**
 * @brief Get old firmware info (CPU0 stub)
 * @param[out] info old firmware info
 * @return OPRT_NOT_SUPPORTED on CPU0
 */
OPERATE_RET tkl_ota_get_old_firmware_info(TUYA_OTA_FIRMWARE_INFO_T **info)
{
    (void)info;
    return OPRT_NOT_SUPPORTED;
}

#else /* CONFIG_CPU_INDEX == 1 */

#include <os/os.h>

/* ---------------------------------------------------------------------------
 * Macros
 * --------------------------------------------------------------------------- */
#define TKL_OTA_YIELD_MS                2U

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
STATIC VOID_T __tkl_ota_yield(VOID_T);
STATIC VOID_T __tkl_ota_ctx_free(VOID_T);
STATIC OPERATE_RET __tkl_ota_flush_buffer(TKL_OTA_CTX_T *ctx, BOOL_T force);
STATIC OPERATE_RET __tkl_ota_write_flash(TKL_OTA_CTX_T *ctx, UINT_T len);
STATIC VOID_T __tkl_ota_log_progress(TKL_OTA_CTX_T *ctx);
STATIC OPERATE_RET __tkl_ota_select_target_partition(TKL_OTA_CTX_T *ctx);
#if defined(CONFIG_OTA_HASH_FUNCTION) && defined(CONFIG_HTTP_AB_PARTITION)
STATIC OPERATE_RET __tkl_ota_verify_rbl_hash(VOID_T);
#endif

/**
 * @brief Yield CPU for IPC heartbeat and other low-priority tasks
 * @return none
 * @note Same interval as Beken ota_base_drv.c rtos_delay_milliseconds(2)
 */
STATIC VOID_T __tkl_ota_yield(VOID_T)
{
    rtos_delay_milliseconds(TKL_OTA_YIELD_MS);
}

/**
 * @brief Release OTA context and buffers
 * @return none
 */
STATIC VOID_T __tkl_ota_ctx_free(VOID_T)
{
    if (s_ota_ctx == NULL) {
        return;
    }

    if (s_ota_ctx->wr_buf != NULL) {
        tkl_system_free(s_ota_ctx->wr_buf);
        s_ota_ctx->wr_buf = NULL;
    }

    if (s_ota_ctx->rd_buf != NULL) {
        tkl_system_free(s_ota_ctx->rd_buf);
        s_ota_ctx->rd_buf = NULL;
    }

    tkl_system_free(s_ota_ctx);
    s_ota_ctx = NULL;
}

/**
 * @brief Log OTA progress every 10%
 * @param[in] ctx OTA context
 * @return none
 */
STATIC VOID_T __tkl_ota_log_progress(TKL_OTA_CTX_T *ctx)
{
    UINT_T cur_pct;

    if ((ctx == NULL) || (ctx->image_size == 0)) {
        return;
    }

    cur_pct = (ctx->received_total * 100U) / ctx->image_size;
    if ((cur_pct < ctx->last_progress_pct) ||
        ((cur_pct - ctx->last_progress_pct) < TKL_OTA_PROGRESS_STEP_PCT && cur_pct != 100U)) {
        return;
    }

    bk_printf("[tkl_ota] progress %u%%\r\n", cur_pct);
    ctx->last_progress_pct = cur_pct;
}

/**
 * @brief Select OTA target partition (AB swap or single OTA partition)
 * @param[in] ctx OTA context
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_ota_select_target_partition(TKL_OTA_CTX_T *ctx)
{
    bk_logic_partition_t *pt = NULL;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

#ifdef CONFIG_HTTP_AB_PARTITION
    exec_flag cur_part = (exec_flag)bk_ota_get_current_partition();

    if (cur_part == EXEC_B_PART) {
        update_part_flag = UPDATE_A_PART;
    } else {
        update_part_flag = UPDATE_B_PART;
    }

#if CONFIG_OTA_POSITION_INDEPENDENT_AB
    ctx->partition_length = http_get_sapp_partition_length(BK_PARTITION_S_APP);
    if (update_part_flag == UPDATE_B_PART) {
        pt = bk_flash_partition_get_info(BK_PARTITION_S_APP);
    } else {
        pt = bk_flash_partition_get_info(BK_PARTITION_APPLICATION);
    }
#else
    pt = bk_flash_partition_get_info(BK_PARTITION_S_APP);
    ctx->partition_length = pt->partition_length;
#endif
#else
    pt = bk_flash_partition_get_info(BK_PARTITION_OTA);
    if (pt != NULL) {
        ctx->partition_length = pt->partition_length;
    }
#endif

    if (pt == NULL) {
        return OPRT_COM_ERROR;
    }

    ctx->partition_start = pt->partition_start_addr;
    ctx->wr_address = pt->partition_start_addr;
    return OPRT_OK;
}

#if defined(CONFIG_OTA_HASH_FUNCTION) && defined(CONFIG_HTTP_AB_PARTITION)
/**
 * @brief Verify RBL compressed firmware hash after download
 * @return OPRT_OK on success
 * @note Requires CONFIG_OTA_HASH_FUNCTION and AB partition; bootloader decompresses RBL payload on reboot
 */
STATIC OPERATE_RET __tkl_ota_verify_rbl_hash(VOID_T)
{
    if (ota_do_hash_check() != BK_OK) {
        bk_printf("[tkl_ota] RBL hash verify failed\r\n");
        return OPRT_OS_ADAPTER_OTA_VERIFY_FAILED;
    }

    bk_printf("[tkl_ota] RBL hash verify ok\r\n");
    return OPRT_OK;
}
#endif

/**
 * @brief Erase sector and write buffered data to flash
 * @param[in] ctx OTA context
 * @param[in] len bytes to write from wr_buf
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_ota_write_flash(TKL_OTA_CTX_T *ctx, UINT_T len)
{
    OPERATE_RET ret;

    if ((ctx == NULL) || (len == 0) || (ctx->wr_buf == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if ((ctx->wr_address < ctx->partition_start) ||
        ((ctx->wr_address + len) > (ctx->partition_start + ctx->partition_length))) {
        return OPRT_OS_ADAPTER_OTA_PROCESS_FAILED;
    }

    if ((ctx->wr_address % TKL_OTA_FLASH_SECTOR_SIZE) == 0) {
        tkl_watchdog_refresh();
        ret = tkl_flash_erase(ctx->wr_address, TKL_OTA_FLASH_SECTOR_SIZE);
        tkl_watchdog_refresh();
        if (ret != OPRT_OK) {
            return OPRT_OS_ADAPTER_OTA_PROCESS_FAILED;
        }
        __tkl_ota_yield();
    }

    tkl_watchdog_refresh();
    ret = tkl_flash_write(ctx->wr_address, ctx->wr_buf, len);
    tkl_watchdog_refresh();
    if (ret != OPRT_OK) {
        return OPRT_OS_ADAPTER_OTA_PROCESS_FAILED;
    }
    __tkl_ota_yield();

    if (ctx->rd_buf != NULL) {
        ret = tkl_flash_read(ctx->wr_address, ctx->rd_buf, len);
        if ((ret != OPRT_OK) || (memcmp(ctx->wr_buf, ctx->rd_buf, len) != 0)) {
            return OPRT_OS_ADAPTER_OTA_PROCESS_FAILED;
        }
        __tkl_ota_yield();
    }

    ctx->wr_address += len;
    memset(ctx->wr_buf, 0, len);
    return OPRT_OK;
}

/**
 * @brief Flush internal write buffer to flash
 * @param[in] ctx OTA context
 * @param[in] force flush remaining bytes even if less than one block
 * @return OPRT_OK on success
 */
STATIC OPERATE_RET __tkl_ota_flush_buffer(TKL_OTA_CTX_T *ctx, BOOL_T force)
{
    OPERATE_RET ret = OPRT_OK;

    if (ctx == NULL) {
        return OPRT_INVALID_PARM;
    }

    while (ctx->wr_last_len >= TKL_OTA_FLASH_BUFFER_LENGTH) {
        ret = __tkl_ota_write_flash(ctx, TKL_OTA_FLASH_BUFFER_LENGTH);
        if (ret != OPRT_OK) {
            return ret;
        }
        ctx->wr_last_len -= TKL_OTA_FLASH_BUFFER_LENGTH;
        if (ctx->wr_last_len > 0) {
            memmove(ctx->wr_buf, ctx->wr_buf + TKL_OTA_FLASH_BUFFER_LENGTH, ctx->wr_last_len);
        }
    }

    if (force && (ctx->wr_last_len > 0)) {
        ret = __tkl_ota_write_flash(ctx, ctx->wr_last_len);
        if (ret != OPRT_OK) {
            return ret;
        }
        ctx->wr_last_len = 0;
    }

    return ret;
}

/**
 * @brief Get OTA ability
 * @param[out] image_size max firmware image size in bytes
 * @param[out] type supported OTA type
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_ota_get_ability(UINT_T *image_size, TUYA_OTA_TYPE_E *type)
{
    TUYA_FLASH_BASE_INFO_T info;
    UINT_T fw1_size = 0;
    UINT_T fw2_size = 0;
    bk_logic_partition_t *pt = NULL;

    if ((image_size == NULL) || (type == NULL)) {
        return OPRT_INVALID_PARM;
    }

#ifdef CONFIG_HTTP_AB_PARTITION
    pt = bk_flash_partition_get_info(BK_PARTITION_APPLICATION);
    if (pt != NULL) {
        fw1_size = pt->partition_length;
    }

    pt = bk_flash_partition_get_info(BK_PARTITION_S_APP);
    if (pt != NULL) {
        fw2_size = pt->partition_length;
    }

    if ((fw1_size == 0) && (fw2_size == 0)) {
        return OPRT_COM_ERROR;
    }

    if ((fw1_size == 0) || (fw2_size == 0)) {
        *image_size = (fw1_size > fw2_size) ? fw1_size : fw2_size;
    } else {
        *image_size = TKL_OTA_MIN(fw1_size, fw2_size);
    }
#else
    if (tkl_flash_get_one_type_info(TUYA_FLASH_TYPE_APP, &info) != OPRT_OK) {
        pt = bk_flash_partition_get_info(BK_PARTITION_OTA);
        if (pt == NULL) {
            return OPRT_COM_ERROR;
        }
        *image_size = pt->partition_length;
    } else {
        *image_size = info.partition[0].size;
    }
#endif

    *type = TUYA_OTA_FULL;
    return OPRT_OK;
}

/**
 * @brief OTA start notify
 * @param[in] image_size firmware image size
 * @param[in] type OTA type
 * @param[in] path OTA path
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_ota_start_notify(UINT_T image_size, TUYA_OTA_TYPE_E type, TUYA_OTA_PATH_E path)
{
    OPERATE_RET ret;
    UINT_T max_size = 0;
    TUYA_OTA_TYPE_E support_type = TUYA_OTA_FULL;

    if (image_size == 0) {
        return OPRT_INVALID_PARM;
    }

    if (type == TUYA_OTA_DIFF) {
        return OPRT_NOT_SUPPORTED;
    }

    ret = tkl_ota_get_ability(&max_size, &support_type);
    if ((ret != OPRT_OK) || (image_size > max_size)) {
        return OPRT_OS_ADAPTER_OTA_PKT_SIZE_FAILED;
    }

    __tkl_ota_ctx_free();

    s_ota_ctx = (TKL_OTA_CTX_T *)tkl_system_malloc(sizeof(TKL_OTA_CTX_T));
    if (s_ota_ctx == NULL) {
        return OPRT_MALLOC_FAILED;
    }
    memset(s_ota_ctx, 0, sizeof(TKL_OTA_CTX_T));

    s_ota_ctx->wr_buf = (UCHAR_T *)tkl_system_malloc(TKL_OTA_FLASH_BUFFER_LENGTH);
    s_ota_ctx->rd_buf = (UCHAR_T *)tkl_system_malloc(TKL_OTA_FLASH_BUFFER_LENGTH);
    if ((s_ota_ctx->wr_buf == NULL) || (s_ota_ctx->rd_buf == NULL)) {
        __tkl_ota_ctx_free();
        return OPRT_MALLOC_FAILED;
    }

    s_ota_ctx->image_size = image_size;
    s_ota_ctx->ota_type = type;
    s_ota_ctx->ota_path = path;
    s_ota_ctx->first_packet = TRUE;
    s_ota_ctx->last_progress_pct = 0;

    ret = __tkl_ota_select_target_partition(s_ota_ctx);
    if (ret != OPRT_OK) {
        __tkl_ota_ctx_free();
        return OPRT_OS_ADAPTER_OTA_START_INFORM_FAILED;
    }

    /* 4.0.1.11: the flash core owns the protect state and re-asserts it around
     * every op, so OTA no longer saves/unprotects/restores it here. */

    bk_printf("[tkl_ota] start size=%u target=0x%x len=0x%x path=%u\r\n",
              image_size, s_ota_ctx->partition_start, s_ota_ctx->partition_length, path);
    return OPRT_OK;
}

/**
 * @brief OTA data process
 * @param[in] pack OTA data packet
 * @param[out] remain_len unprocessed bytes in current packet
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_ota_data_process(TUYA_OTA_DATA_T *pack, UINT_T *remain_len)
{
    OPERATE_RET ret = OPRT_OK;
    UINT_T copy_len = 0;
    UINT_T offset = 0;

    if ((pack == NULL) || (remain_len == NULL)) {
        return OPRT_INVALID_PARM;
    }

    if ((pack->data == NULL) || (pack->len == 0)) {
        *remain_len = 0;
        return OPRT_INVALID_PARM;
    }

    if (s_ota_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    *remain_len = 0;

    if (s_ota_ctx->first_packet) {
        UINT_T base = s_ota_ctx->partition_start;

        if (pack->start_addr != 0) {
            base = pack->start_addr;
        }

        if ((base < s_ota_ctx->partition_start) ||
            (base >= (s_ota_ctx->partition_start + s_ota_ctx->partition_length))) {
            return OPRT_OS_ADAPTER_OTA_PROCESS_FAILED;
        }

        s_ota_ctx->wr_address = base + pack->offset;
        s_ota_ctx->first_packet = FALSE;
    } else if (pack->offset != s_ota_ctx->received_total) {
        UINT_T base = (pack->start_addr != 0) ? pack->start_addr : s_ota_ctx->partition_start;

        s_ota_ctx->wr_address = base + pack->offset;
        s_ota_ctx->wr_last_len = 0;
    }

    while (offset < pack->len) {
        copy_len = pack->len - offset;
        if (copy_len > (TKL_OTA_FLASH_BUFFER_LENGTH - s_ota_ctx->wr_last_len)) {
            copy_len = TKL_OTA_FLASH_BUFFER_LENGTH - s_ota_ctx->wr_last_len;
        }

        memcpy(s_ota_ctx->wr_buf + s_ota_ctx->wr_last_len, pack->data + offset, copy_len);
        offset += copy_len;
        s_ota_ctx->wr_last_len += copy_len;
        s_ota_ctx->received_total += copy_len;

        if (s_ota_ctx->wr_last_len >= TKL_OTA_FLASH_BUFFER_LENGTH) {
            ret = __tkl_ota_flush_buffer(s_ota_ctx, FALSE);
            if (ret != OPRT_OK) {
                return ret;
            }
        }

        __tkl_ota_log_progress(s_ota_ctx);

        if (s_ota_ctx->received_total >= s_ota_ctx->image_size) {
            ret = __tkl_ota_flush_buffer(s_ota_ctx, TRUE);
            if (ret != OPRT_OK) {
                return ret;
            }
            break;
        }
    }

    return OPRT_OK;
}

/**
 * @brief OTA end notify
 * @param[in] reset whether to reset after OTA
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_ota_end_notify(BOOL_T reset)
{
    OPERATE_RET ret = OPRT_OK;

    if (s_ota_ctx == NULL) {
        return OPRT_COM_ERROR;
    }

    if (s_ota_ctx->wr_last_len > 0) {
        ret = __tkl_ota_flush_buffer(s_ota_ctx, TRUE);
        if (ret != OPRT_OK) {
            __tkl_ota_ctx_free();
            return OPRT_OS_ADAPTER_OTA_PROCESS_FAILED;
        }
    }

    if (s_ota_ctx->received_total != s_ota_ctx->image_size) {
        bk_printf("[tkl_ota] size mismatch recv=%u expect=%u\r\n",
                  s_ota_ctx->received_total, s_ota_ctx->image_size);
        __tkl_ota_ctx_free();
        return OPRT_OS_ADAPTER_OTA_VERIFY_FAILED;
    }

#if defined(CONFIG_OTA_HASH_FUNCTION) && defined(CONFIG_HTTP_AB_PARTITION)
    ret = __tkl_ota_verify_rbl_hash();
    if (ret != OPRT_OK) {
        __tkl_ota_ctx_free();
        return ret;
    }
#endif

#ifdef CONFIG_HTTP_AB_PARTITION
    if (bk_ota_update_partition_flag(0) != 0) {
        __tkl_ota_ctx_free();
        return OPRT_OS_ADAPTER_OTA_END_INFORM_FAILED;
    }
#endif

    bk_printf("[tkl_ota] end success\r\n");
    __tkl_ota_ctx_free();

    if (reset) {
        tkl_system_reset();
    }

    return ret;
}

/**
 * @brief Get old firmware info for breakpoint resume
 * @param[out] info old firmware info
 * @return OPRT_NOT_SUPPORTED
 */
OPERATE_RET tkl_ota_get_old_firmware_info(TUYA_OTA_FIRMWARE_INFO_T **info)
{
    (void)info;
    return OPRT_NOT_SUPPORTED;
}

#endif /* CONFIG_CPU_INDEX */
