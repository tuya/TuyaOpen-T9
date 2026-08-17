#ifndef __BLEND_H__
#define __BLEND_H__

/**
 * @file blend.h
 * @brief OSD blend asset declarations (ported from draw_osd_example)
 */

#include "components/bk_draw_osd.h"

/* ---------------- OSD 元素坐标空间 ----------------
 * 面板固定 1024x600（jd9165t_mipi_1024x600）。元素 xpos/ypos 落在哪个坐标系，
 * 由 `xgpu osd open <rotate>` 决定：
 *   rotate 0      → 最终显示 buffer 空间，1024x600（横屏）
 *   rotate 90/270 → 旋转前的观看者空间，宽高互换成 600x1024（竖屏）
 *
 * 引擎的边界判定（bk_osd_engine.c）：
 *   xpos + sprite_w <= OSD_VIEW_W，ypos + sprite_h <= OSD_VIEW_H
 * 越界元素直接丢弃并打 "OSD element off-screen" warning。
 *
 * 两套布局只改 OSD_LAYOUT_ROTATED，各元素坐标都由 OSD_VIEW_W/H 推导，不用逐个改。 */
#define OSD_PANEL_W          1024
#define OSD_PANEL_H          600

#ifndef OSD_LAYOUT_ROTATED
#define OSD_LAYOUT_ROTATED   1   /* 1: 配合 osd open 90/270；0: 配合 osd open 0 */
#endif

#if OSD_LAYOUT_ROTATED
#define OSD_VIEW_W           OSD_PANEL_H   /* 600  */
#define OSD_VIEW_H           OSD_PANEL_W   /* 1024 */
#else
#define OSD_VIEW_W           OSD_PANEL_W   /* 1024 */
#define OSD_VIEW_H           OSD_PANEL_H   /* 600  */
#endif

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
