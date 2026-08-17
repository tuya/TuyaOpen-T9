/**
 * @file tkl_dvp.c
 * @brief TKL adapter stub. DVP camera adapter for T9.
 *
 * T9 / BK7259 has no DVP capture interface -- the camera path is MIPI CSI,
 * adapted in tkl_camera_mipi.c (tkl_camera_mipi_*). These stubs exist only so
 * that tal_dvp (application_drivers/tal_dvp) and any DVP-based test case still
 * link on this platform; every entry point reports OPRT_NOT_SUPPORTED.
 *
 * New camera test cases should target tkl_camera_mipi_* instead of this file.
 *
 * @version 0.1
 * @date 2026-08-13
 * @copyright Copyright (c) Tuya Inc.
 */

#include "tuya_cloud_types.h"
#include "tkl_dvp.h"
#include "tkl_output.h"

OPERATE_RET tkl_dvp_init(TUYA_DVP_CFG_T *dvp_cfg)
{
    /* 初始化 DVP 摄像头。T9 无 DVP 接口，请使用 tkl_camera_mipi_init。 */
    (void)dvp_cfg;

    tkl_log_output("tkl_dvp_init: DVP not available on T9, use tkl_camera_mipi_*\r\n");
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_dvp_deinit(void)
{
    /* 去初始化 DVP 摄像头。 */
    tkl_log_output("tkl_dvp_deinit: DVP not available on T9, use tkl_camera_mipi_*\r\n");
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_dvp_start(void)
{
    /* 启动 DVP 采集。 */
    tkl_log_output("tkl_dvp_start: DVP not available on T9, use tkl_camera_mipi_*\r\n");
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_dvp_stop(void)
{
    /* 停止 DVP 采集。 */
    tkl_log_output("tkl_dvp_stop: DVP not available on T9, use tkl_camera_mipi_*\r\n");
    return OPRT_NOT_SUPPORTED;
}
