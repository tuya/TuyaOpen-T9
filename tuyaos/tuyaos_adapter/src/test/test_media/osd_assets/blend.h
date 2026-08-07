#ifndef __BLEND_H__
#define __BLEND_H__

/**
 * @file blend.h
 * @brief OSD blend asset declarations (ported from draw_osd_example)
 */

#include "components/bk_draw_osd.h"

extern const bk_blend_t font_clock;
extern const bk_blend_t font_dates;
extern const bk_blend_t font_weather;
extern const bk_blend_t font_ver;
/* wifi: WiFi signal strength icon set (none/1/2/3/full, 60x60 ARGB8888) */
extern const bk_blend_t wifi_rssi_none;
extern const bk_blend_t wifi_rssi_1;
extern const bk_blend_t wifi_rssi_2;
extern const bk_blend_t wifi_rssi_3;
extern const bk_blend_t wifi_rssi_full;

extern const blend_info_t blend_assets[];
extern const blend_info_t blend_info[];

#endif
