#ifndef __BLEND_H__
#define __BLEND_H__

/**
 * @file blend.h
 * @brief OSD asset declarations and layout (ported from draw_osd_example)
 *
 * @note Raw asset data plus its placement, which is what TKL_OSD_ELEM_T
 *       consumes. The one platform dependency is the emWin glyph table type
 *       (modules/lcd_font.h) - that *is* the asset format, and TKL never looks
 *       inside it: TKL_OSD_ELEM_T::font is a const void *.
 */

/* modules/lcd_font.h uses uint32_t and NULL but includes neither header itself;
 * components/bk_draw_osd.h used to drag them in before the wrappers were removed. */
#include <stdint.h>
#include <stddef.h>
#include "modules/lcd_font.h"   /* gui_font_digit_struct, GUI_CONST_STORAGE */

/* ---------------- OSD 元素坐标空间 ----------------
 * 面板固定 1024x600（jd9165t_mipi_1024x600）。元素 x/y 落在哪个坐标系，
 * 由该通道的旋转角决定：
 *   rotate 0      → 最终显示 buffer 空间，1024x600（横屏）
 *   rotate 90/270 → 旋转前的观看者空间，宽高互换成 600x1024（竖屏）
 *
 * 越界元素会被 tkl_osd_engine 丢弃并打 "off-screen" 警告。
 *
 * 两套布局只改 OSD_LAYOUT_ROTATED，各元素坐标都由 OSD_VIEW_W/H 推导，不用逐个改。 */
#define OSD_PANEL_W          1024
#define OSD_PANEL_H          600

#ifndef OSD_LAYOUT_ROTATED
#define OSD_LAYOUT_ROTATED   1   /* 1: 配合 rotate 90/270；0: 配合 rotate 0 */
#endif

#if OSD_LAYOUT_ROTATED
#define OSD_VIEW_W           OSD_PANEL_H   /* 600  */
#define OSD_VIEW_H           OSD_PANEL_W   /* 1024 */
#else
#define OSD_VIEW_W           OSD_PANEL_W   /* 1024 */
#define OSD_VIEW_H           OSD_PANEL_H   /* 600  */
#endif

/* ---------------- 文本元素几何 ---------------- */

#define CLOCK_LOGO_W    220   /* enough for "HH:MM:SS" */
#define CLOCK_LOGO_H    44

#define DATA_LOGO_W     220
#define DATA_LOGO_H     24

#define VERSION_LOGO_W  180
#define VERSION_LOGO_H  24

/* 位置按 OSD_VIEW_W/H 的比例推导，横屏(1024x600)与竖屏(600x1024)共用同一套公式。 */

/* ver: horizontal center, slightly above vertical center */
#define VER_X           ((OSD_VIEW_W - VERSION_LOGO_W) / 2)  /* 竖屏 210 / 横屏 422 */
#define VER_Y           (OSD_VIEW_H / 3)                     /* 竖屏 341 / 横屏 200 */

/* clock + date stacked: horizontal center, slightly below vertical center */
#define CLOCK_DATE_GAP  8
#define CLOCK_DATE_STACK_H  (CLOCK_LOGO_H + CLOCK_DATE_GAP + DATA_LOGO_H) /* 76 */
/* 中线下方 OSD_VIEW_H/15（竖屏 68 / 横屏 40），再上移半个堆叠高度 */
#define CLOCK_DATE_STACK_TOP  ((OSD_VIEW_H / 2) + (OSD_VIEW_H / 15) - (CLOCK_DATE_STACK_H / 2))
#define CLOCK_X         ((OSD_VIEW_W - CLOCK_LOGO_W) / 2)    /* 竖屏 190 / 横屏 402 */
#define CLOCK_Y         CLOCK_DATE_STACK_TOP                 /* 竖屏 542 / 横屏 302 */
#define DATE_X          ((OSD_VIEW_W - DATA_LOGO_W) / 2)     /* 竖屏 190 / 横屏 402 */
#define DATE_Y          (CLOCK_Y + CLOCK_LOGO_H + CLOCK_DATE_GAP)

#define FONT_COLOR      0xFFFF

/* ---------------- 图标元素几何 ---------------- */

#define WIFI_MARGIN            16
#define WIFI_ICON_W            60
#define WIFI_ICON_H            60
/* Top-right corner */
#define WIFI_X                 (OSD_VIEW_W - WIFI_ICON_W - WIFI_MARGIN) /* 竖屏 524 / 横屏 948 */
#define WIFI_Y                 WIFI_MARGIN                              /* 16 */

#define wifi_rssi_full_X       WIFI_X
#define wifi_rssi_full_Y       WIFI_Y
#define wifi_rssi_full_WIDTH   WIFI_ICON_W
#define wifi_rssi_full_HEIGHT  WIFI_ICON_H

#define wifi_rssi_none_X       WIFI_X
#define wifi_rssi_none_Y       WIFI_Y
#define wifi_rssi_none_WIDTH   WIFI_ICON_W
#define wifi_rssi_none_HEIGHT  WIFI_ICON_H

#define wifi_rssi_1_X          WIFI_X
#define wifi_rssi_1_Y          WIFI_Y
#define wifi_rssi_1_WIDTH      WIFI_ICON_W
#define wifi_rssi_1_HEIGHT     WIFI_ICON_H

#define wifi_rssi_2_X          WIFI_X
#define wifi_rssi_2_Y          WIFI_Y
#define wifi_rssi_2_WIDTH      WIFI_ICON_W
#define wifi_rssi_2_HEIGHT     WIFI_ICON_H

#define wifi_rssi_3_X          WIFI_X
#define wifi_rssi_3_Y          WIFI_Y
#define wifi_rssi_3_WIDTH      WIFI_ICON_W
#define wifi_rssi_3_HEIGHT     WIFI_ICON_H

/* ---------------- 原始资产 ---------------- */

/* emWin glyph tables (bk_font.c) */
extern const gui_font_digit_struct *const font_digit_Roboto53;
extern const gui_font_digit_struct *const font_digit_black24;

/* ARGB8888 icons, 60x60 (bk_img.c) */
extern const unsigned char wifi_rssi_none_argb8888[];
extern const unsigned char wifi_rssi_1_argb8888[];
extern const unsigned char wifi_rssi_2_argb8888[];
extern const unsigned char wifi_rssi_3_argb8888[];
extern const unsigned char wifi_rssi_full_argb8888[];

#endif
