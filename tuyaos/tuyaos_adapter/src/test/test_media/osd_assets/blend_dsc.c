/**
 * @file blend_dsc.c
 * @brief Business-side ARGB8888 bitmap composition for the OSD CLI tests.
 */

#include "blend.h"
#include "blend_dsc.h"

#define OSD_CLOCK_BPP 4U
#define OSD_CLOCK_GLYPH_W 5U
#define OSD_CLOCK_GLYPH_H 7U
#define OSD_CLOCK_GLYPH_SCALE 5U
#define OSD_CLOCK_GLYPH_STEP 30U
#define OSD_CLOCK_ORIGIN_X 4U
#define OSD_CLOCK_ORIGIN_Y 3U
#define OSD_CLOCK_ARGB 0xFFFFFF00U

static UINT8_T s_clock_bitmap[OSD_CLOCK_BITMAP_W * OSD_CLOCK_BITMAP_H * OSD_CLOCK_BPP]
    __attribute__((aligned(64)));

static CONST UINT8_T s_digit_glyphs[11][OSD_CLOCK_GLYPH_H] = {
    {0x1EU, 0x21U, 0x23U, 0x25U, 0x29U, 0x31U, 0x1EU}, /* 0 */
    {0x04U, 0x0CU, 0x04U, 0x04U, 0x04U, 0x04U, 0x1FU}, /* 1 */
    {0x1EU, 0x21U, 0x01U, 0x06U, 0x18U, 0x20U, 0x3FU}, /* 2 */
    {0x1EU, 0x21U, 0x01U, 0x0EU, 0x01U, 0x21U, 0x1EU}, /* 3 */
    {0x02U, 0x06U, 0x0AU, 0x12U, 0x3FU, 0x02U, 0x02U}, /* 4 */
    {0x3FU, 0x20U, 0x20U, 0x3EU, 0x01U, 0x21U, 0x1EU}, /* 5 */
    {0x0EU, 0x10U, 0x20U, 0x3EU, 0x21U, 0x21U, 0x1EU}, /* 6 */
    {0x3FU, 0x01U, 0x02U, 0x04U, 0x08U, 0x08U, 0x08U}, /* 7 */
    {0x1EU, 0x21U, 0x21U, 0x1EU, 0x21U, 0x21U, 0x1EU}, /* 8 */
    {0x1EU, 0x21U, 0x21U, 0x1FU, 0x01U, 0x02U, 0x1CU}, /* 9 */
    {0x00U, 0x00U, 0x04U, 0x00U, 0x00U, 0x04U, 0x00U}, /* : */
};

static VOID_T osd_clock_pixel(UINT16_T x, UINT16_T y, UINT32_T argb)
{
    UINT32_T *pixel;

    if (x >= OSD_CLOCK_BITMAP_W || y >= OSD_CLOCK_BITMAP_H) {
        return;
    }
    pixel = (UINT32_T *)&s_clock_bitmap[((UINT32_T)y * OSD_CLOCK_BITMAP_W + x) * 4U];
    *pixel = argb;
}

static VOID_T osd_clock_fill(VOID_T)
{
    os_memset(s_clock_bitmap, 0, sizeof(s_clock_bitmap));
}

static VOID_T osd_clock_draw_text(CONST CHAR_T *text)
{
    UINT16_T origin_x = OSD_CLOCK_ORIGIN_X;
    UINT16_T i;

    if (text == NULL) {
        return;
    }

    for (i = 0U; text[i] != '\0' && i < 8U; i++) {
        UINT8_T glyph;
        UINT8_T row;
        UINT8_T col;

        if (text[i] >= '0' && text[i] <= '9') {
            glyph = (UINT8_T)(text[i] - '0');
        } else if (text[i] == ':') {
            glyph = 10U;
        } else {
            origin_x = (UINT16_T)(origin_x + OSD_CLOCK_GLYPH_STEP);
            continue;
        }

        for (row = 0U; row < OSD_CLOCK_GLYPH_H; row++) {
            for (col = 0U; col < OSD_CLOCK_GLYPH_W; col++) {
                if ((s_digit_glyphs[glyph][row] & (1U << (OSD_CLOCK_GLYPH_W - 1U - col))) != 0U) {
                    UINT8_T dx;
                    UINT8_T dy;

                    for (dy = 0U; dy < OSD_CLOCK_GLYPH_SCALE; dy++) {
                        for (dx = 0U; dx < OSD_CLOCK_GLYPH_SCALE; dx++) {
                            osd_clock_pixel((UINT16_T)(origin_x + col * OSD_CLOCK_GLYPH_SCALE + dx),
                                            (UINT16_T)(OSD_CLOCK_ORIGIN_Y + row * OSD_CLOCK_GLYPH_SCALE + dy),
                                            OSD_CLOCK_ARGB);
                        }
                    }
                }
            }
        }
        origin_x = (UINT16_T)(origin_x + OSD_CLOCK_GLYPH_STEP);
    }
}

VOID_T osd_assets_init(VOID_T)
{
    osd_assets_set_clock("00:00:00");
}

VOID_T osd_assets_set_clock(CONST CHAR_T *text)
{
    osd_clock_fill();
    osd_clock_draw_text(text);
}

CONST UINT8_T *osd_assets_clock_bitmap(UINT16_T *width, UINT16_T *height)
{
    if (width != NULL) {
        *width = OSD_CLOCK_BITMAP_W;
    }
    if (height != NULL) {
        *height = OSD_CLOCK_BITMAP_H;
    }
    return s_clock_bitmap;
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
