/**
 * @file blend_dsc.c
 * @brief Ready-made TKL_OSD_ELEM_T groups built from the raw assets
 *
 * @details
 * **One slot per screen region, not one slot for the screen.** Element x/y are
 * offsets *inside* the slot's sprite, and TKL_OSD_T::dst_x/dst_y place that
 * sprite. Far-apart elements in one slot auto-size the sprite to span the gap:
 * a clock plus a corner wifi icon would cost 1.4 MB per refresh against 38 KB +
 * 14 KB in separate slots.
 */

#include "blend.h"
#include "blend_dsc.h"

/* Clock text, drawn at the sprite origin; placed by OSD_CLOCK_DST_X/Y. */
TKL_OSD_ELEM_T osd_elems_clock[] = {
    {
        .type = TKL_OSD_ELEM_TEXT,
        .x = 0U,
        .y = 0U,
        .font = NULL,                 /* filled by osd_assets_init() */
        .font_kind = TKL_OSD_FONT_BITMAP,
        .text = "00:00:00",
        .argb = OSD_CLOCK_ARGB,
    },
};

/* Wifi icon, drawn at the sprite origin; placed by OSD_WIFI_DST_X/Y. */
TKL_OSD_ELEM_T osd_elems_wifi[] = {
    {
        .type = TKL_OSD_ELEM_IMAGE,
        .x = 0U,
        .y = 0U,
        .data = NULL,                 /* filled by osd_assets_init() */
        .w = WIFI_ICON_W,
        .h = WIFI_ICON_H,
        .format = TKL_PIXEL_FMT_ARGB8888,
    },
};

CONST UINT8_T osd_elems_clock_count = (UINT8_T)(sizeof(osd_elems_clock) /
                                                sizeof(osd_elems_clock[0]));
CONST UINT8_T osd_elems_wifi_count = (UINT8_T)(sizeof(osd_elems_wifi) /
                                               sizeof(osd_elems_wifi[0]));

VOID_T osd_assets_init(VOID_T)
{
    /* font_digit_* and the icon arrays are objects, not compile-time constants
     * usable in a static initializer for a pointer field */
    osd_elems_clock[0].font = font_digit_Roboto53;
    osd_elems_wifi[0].data = wifi_rssi_full_argb8888;
}

VOID_T osd_assets_set_clock(CONST CHAR_T *text)
{
    if (text == NULL) {
        return;
    }

    osd_elems_clock[0].text = text;
}

CONST UINT8_T *osd_assets_wifi_icon(UINT8_T level)
{
    switch (level) {
    case 0U:
        return wifi_rssi_none_argb8888;
    case 1U:
        return wifi_rssi_1_argb8888;
    case 2U:
        return wifi_rssi_2_argb8888;
    case 3U:
        return wifi_rssi_3_argb8888;
    default:
        return wifi_rssi_full_argb8888;
    }
}

VOID_T osd_assets_set_wifi(UINT8_T level)
{
    osd_elems_wifi[0].data = osd_assets_wifi_icon(level);
}
