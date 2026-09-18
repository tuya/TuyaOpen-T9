#ifndef __TEST_OSD_H__
#define __TEST_OSD_H__

/**
 * @file test_osd.h
 * @brief Business-side OSD bitmap composition reference for media tests.
 *
 * This is test/application code, not a public TKL OSD API. The caller owns
 * the composition policy and submits the returned ARGB8888 bitmap through
 * tkl_mipi_csi_set_osd() or tkl_uvc_set_osd().
 */

#include "tuya_cloud_types.h"
#include "blend.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TEST_OSD_CLOCK_DST_X  ((OSD_VIEW_W - TEST_OSD_CLOCK_BITMAP_W) / 2U)
#define TEST_OSD_CLOCK_DST_Y  ((OSD_VIEW_H / 2U) + (OSD_VIEW_H / 15U) - \
                               (TEST_OSD_CLOCK_BITMAP_H / 2U))
#define TEST_OSD_WIFI_DST_X   (OSD_VIEW_W - TEST_OSD_WIFI_BITMAP_W - TEST_OSD_WIFI_MARGIN)
#define TEST_OSD_WIFI_DST_Y   TEST_OSD_WIFI_MARGIN

#define TEST_OSD_CLOCK_BITMAP_W 220U
#define TEST_OSD_CLOCK_BITMAP_H 44U
#define TEST_OSD_WIFI_BITMAP_W   60U
#define TEST_OSD_WIFI_BITMAP_H   60U
#define TEST_OSD_WIFI_MARGIN     16U

/** Initialize the business-owned clock canvas. */
VOID_T test_osd_init(VOID_T);

/** Clear and redraw the clock canvas with HH:MM:SS text. */
VOID_T test_osd_set_clock(CONST CHAR_T *text);

/** Return the business-owned clock ARGB8888 canvas and dimensions. NULL when
 *  the media slab has no room for the canvases - skip the submit then. */
CONST UINT8_T *test_osd_clock_bitmap(UINT16_T *width, UINT16_T *height);

/** Return an immutable ARGB8888 Wi-Fi asset selected by signal level. */
CONST UINT8_T *test_osd_wifi_bitmap(UINT8_T level);

#ifdef __cplusplus
}
#endif

#endif /* __TEST_OSD_H__ */
