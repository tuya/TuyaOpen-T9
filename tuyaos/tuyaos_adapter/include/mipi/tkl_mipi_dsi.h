/**
 * @file tkl_mipi_dsi.h
 * @brief Common process - adapter the MIPI DSI/DPU api
 * @version 0.1
 * @date 2026-04-09
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_MIPI_H__
#define __TKL_MIPI_H__

#include "tuya_cloud_types.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    UINT8_T *buffer;
    void (*free_cb)(UINT8_T *frame_buff);
} tkl_mipi_dsi_flush_frame_t;

OPERATE_RET tkl_mipi_dsi_set_input_mode(BOOL_T decompress);
OPERATE_RET tkl_mipi_dsi_init(const TUYA_MIPI_BASE_CFG_T *base_cfg);
OPERATE_RET tkl_mipi_dsi_deinit(VOID_T);
OPERATE_RET tkl_mipi_dsi_tx_cmd(UINT8_T cmd, const VOID_T *param, UINT32_T size);
OPERATE_RET tkl_mipi_dsi_flush(tkl_mipi_dsi_flush_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
