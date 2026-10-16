#ifndef __BLEND_DSC_H__
#define __BLEND_DSC_H__

/**
 * @file blend_dsc.h
 * @brief Ready-made TKL_OSD_ELEM_T groups for the OSD CLI tests
 *
 * @note Element x/y are offsets inside the slot's sprite; the OSD_*_DST_X/Y
 *       macros are what goes into TKL_OSD_T::dst_x/dst_y. Keep unrelated
 *       regions in separate slots - see blend_dsc.c for why.
 */

#include "tuya_cloud_types.h"
#include "tkl_media_types.h"
#include "blend.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Slot assignment used by the CLI tests. */
#define OSD_SLOT_CLOCK   0U
#define OSD_SLOT_WIFI    1U

/** Placement in the channel's coordinate space. */
#define OSD_CLOCK_DST_X  CLOCK_X
#define OSD_CLOCK_DST_Y  CLOCK_Y
#define OSD_WIFI_DST_X   WIFI_X
#define OSD_WIFI_DST_Y   WIFI_Y

/** Clock colour: 0x00RRGGBB, alpha comes from glyph coverage. */
#define OSD_CLOCK_ARGB   0x00FFFF00U   /* yellow */

/** Clock text, drawn at the sprite origin. */
extern TKL_OSD_ELEM_T osd_elems_clock[];
extern CONST UINT8_T osd_elems_clock_count;

/** Wifi icon, drawn at the sprite origin. */
extern TKL_OSD_ELEM_T osd_elems_wifi[];
extern CONST UINT8_T osd_elems_wifi_count;

/**
 * @brief Bind the font and icon pointers into the element tables.
 * @note Must run once before the tables are submitted.
 */
VOID_T osd_assets_init(VOID_T);

/**
 * @brief Point the clock element at @p text.
 * @param[in] text NUL-terminated string, must outlive the next submit
 */
VOID_T osd_assets_set_clock(CONST CHAR_T *text);

/**
 * @brief Pick the wifi icon bitmap for a signal level.
 * @param[in] level 0..3, anything else means full
 * @return ARGB8888 bitmap, 60x60
 */
CONST UINT8_T *osd_assets_wifi_icon(UINT8_T level);

/**
 * @brief Point the wifi element at the icon for @p level.
 * @param[in] level 0..3, anything else means full
 */
VOID_T osd_assets_set_wifi(UINT8_T level);

#ifdef __cplusplus
}
#endif

#endif
