/**
 * @file test_mipi.c
 * @brief "xmipi" front door: routes to the MIPI camera or the MIPI display.
 *
 * @details
 * Two peripherals share the MIPI bus family and nothing else, so they get one
 * prefix and two words:
 *
 *   xmipi cam <sub> ...   test_camera_mipi.c
 *   xmipi dsi <sub> ...   test_mipi_dsi.c
 *
 * Each handler is passed argc-1 / argv+1, so its own name lands at argv[0] and
 * its subcommand at argv[1].
 *
 * @version 0.1
 * @date 2026-08-29
 * @copyright Copyright (c) Tuya Inc.
 */

#include <stdio.h>
#include <os/str.h>

#include "tuya_cloud_types.h"
#include "cli_tuya_test.h"

#define XMIPI_USAGE \
    "Usage: xmipi <cam|dsi> <subcommand> ...\r\n" \
    "  cam  MIPI CSI camera - open|close|frame|set_output|record|osd|status|...\r\n" \
    "  dsi  MIPI DSI panel  - open|open_gpu|flush|stop_flush|close\r\n" \
    "  run 'xmipi cam' or 'xmipi dsi' with no subcommand for its own usage\r\n"

/**
 * @brief "xmipi" command entry: pick a peripheral, then hand the rest over.
 *
 * @param[out] pcWriteBuffer   reply buffer
 * @param[in]  xWriteBufferLen its size
 * @param[in]  argc, argv      argv[1] selects the peripheral
 */
void cli_tuya_mipi_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen, XMIPI_USAGE);
        return;
    }

    /* argc - 1 / argv + 1: the handler sees "cam"/"dsi" at argv[0] and its own
     * subcommand at argv[1]. */
    if (os_strcmp(argv[1], "cam") == 0) {
        cli_tuya_mipi_cam_cmd(pcWriteBuffer, xWriteBufferLen, argc - 1, argv + 1);
    } else if (os_strcmp(argv[1], "dsi") == 0) {
        cli_tuya_mipi_dsi_cmd(pcWriteBuffer, xWriteBufferLen, argc - 1, argv + 1);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, XMIPI_USAGE);
    }
}
