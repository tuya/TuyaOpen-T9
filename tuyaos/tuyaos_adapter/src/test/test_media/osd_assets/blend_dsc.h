#ifndef __BLEND_DSC_H__
#define __BLEND_DSC_H__

/**
 * @file blend_dsc.h
 * @brief Business-side OSD bitmap composition helpers for the CLI tests.
 */

#include "tuya_cloud_types.h"
#include "blend.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Placement in the channel's coordinate space. */
#define OSD_CLOCK_DST_X CLOCK_X
#define OSD_CLOCK_DST_Y CLOCK_Y
#define OSD_WIFI_DST_X  WIFI_X
#define OSD_WIFI_DST_Y  WIFI_Y

/** Business-owned clock bitmap dimensions. */
#define OSD_CLOCK_BITMAP_W CLOCK_LOGO_W
#define OSD_CLOCK_BITMAP_H CLOCK_LOGO_H

/** Initialize the business-side bitmap storage. */
VOID_T osd_assets_init(VOID_T);

/** Compose the clock text into the business-owned ARGB8888 bitmap. */
VOID_T osd_assets_set_clock(CONST CHAR_T *text);

/** Return the business-owned clock bitmap and its dimensions. */
CONST UINT8_T *osd_assets_clock_bitmap(UINT16_T *width, UINT16_T *height);

/** Return the selected immutable ARGB8888 Wi-Fi bitmap, 60x60. */
CONST UINT8_T *osd_assets_wifi_icon(UINT8_T level);

#ifdef __cplusplus
}
#endif

#endif
