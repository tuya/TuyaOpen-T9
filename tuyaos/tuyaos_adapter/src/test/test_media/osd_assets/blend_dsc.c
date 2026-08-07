/**
 * @file blend_dsc.c
 * @brief OSD blend_assets / blend_info tables
 * @note Ported from draw_osd_example/ap/assets for xgpu OSD CLI
 */
#include "components/bk_draw_osd.h"
#include "blend.h"

const blend_info_t blend_assets[] =
{
    {.name = "clock", .addr = &font_clock, .content = "12:30"},
    {.name = "date",  .addr = &font_dates, .content = "2025年2月26日周三"},
    {.name = "ver",   .addr = &font_ver, .content = "v 1.0.0"},
    {.name = "wifi" , .addr = &wifi_rssi_none , .content = "wifi_rssi_none" },
    {.name = "wifi" , .addr = &wifi_rssi_1 , .content = "wifi_rssi_1" },
    {.name = "wifi" , .addr = &wifi_rssi_2 , .content = "wifi_rssi_2" },
    {.name = "wifi" , .addr = &wifi_rssi_3 , .content = "wifi_rssi_3" },
    {.name = "wifi" , .addr = &wifi_rssi_full , .content = "wifi_rssi_full" },
    {.addr = NULL},
};

const blend_info_t blend_info[] =
{
    {.name = "clock" , .addr = &font_clock , .content = "12:00:00" },
    {.name = "wifi" , .addr = &wifi_rssi_none , .content = "wifi_rssi_none" },
    {.addr = NULL},
};
