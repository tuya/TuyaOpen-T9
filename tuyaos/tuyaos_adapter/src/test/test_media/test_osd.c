/**
 * @file test_osd.c
 * @brief Business-side ARGB8888 OSD composition reference for media tests.
 *
 * This file deliberately owns the content work that TKL no longer performs:
 * text rasterization, transparent canvas management and alpha compositing.
 * The returned buffers are business-owned and are submitted as complete
 * ARGB8888 bitmaps. The clock renderer reuses the Roboto53 asset retained in
 * osd_assets/bk_font.c; TKL only receives the finished bitmap.
 */

#include "test_osd.h"

#include <os/mem.h>

#define TEST_OSD_BPP                 4U
#define TEST_OSD_CLOCK_ARGB         0xFFFFFF00U
#define TEST_OSD_CLOCK_BUFFER_COUNT 2U

/* Static, aligned storage is used so the example does not hide an allocator
 * or lifetime policy. Compose into the inactive canvas and publish it only
 * after drawing, so the bitmap handed to the backend is not modified by the
 * next clock tick. The business must still keep both canvases valid until the
 * backend has retired them; TKL never frees either canvas. */
static UINT8_T s_clock_bitmap[TEST_OSD_CLOCK_BUFFER_COUNT]
                         [TEST_OSD_CLOCK_BITMAP_W * TEST_OSD_CLOCK_BITMAP_H * TEST_OSD_BPP]
    __attribute__((aligned(64)));
static UINT8_T s_clock_active;
static UINT8_T s_clock_building;

static UINT32_T test_osd_argb_blend(UINT32_T dst, UINT32_T src)
{
    UINT32_T alpha = (src >> 24U) & 0xFFU;
    UINT32_T inv_alpha;
    UINT32_T dst_a;
    UINT32_T dst_r;
    UINT32_T dst_g;
    UINT32_T dst_b;
    UINT32_T src_r;
    UINT32_T src_g;
    UINT32_T src_b;
    UINT32_T out_a;

    if (alpha == 0U) {
        return dst;
    }
    if (alpha == 0xFFU) {
        return src;
    }

    inv_alpha = 0xFFU - alpha;
    dst_a = (dst >> 24U) & 0xFFU;
    dst_r = (dst >> 16U) & 0xFFU;
    dst_g = (dst >> 8U) & 0xFFU;
    dst_b = dst & 0xFFU;
    src_r = (src >> 16U) & 0xFFU;
    src_g = (src >> 8U) & 0xFFU;
    src_b = src & 0xFFU;
    out_a = alpha + ((dst_a * inv_alpha) / 0xFFU);

    return (out_a << 24U) |
           (((src_r * alpha + dst_r * inv_alpha) / 0xFFU) << 16U) |
           (((src_g * alpha + dst_g * inv_alpha) / 0xFFU) << 8U) |
           ((src_b * alpha + dst_b * inv_alpha) / 0xFFU);
}

static CONST gui_font_digit_struct *test_osd_font_glyph(UINT32_T codepoint)
{
    UINT32_T i;

    if (font_digit_Roboto53 == NULL) {
        return NULL;
    }

    for (i = 0U; font_digit_Roboto53[i].value != 0U; i++) {
        if ((UINT32_T)font_digit_Roboto53[i].value == codepoint &&
            font_digit_Roboto53[i].data != NULL) {
            return &font_digit_Roboto53[i];
        }
    }
    return NULL;
}

static CONST gui_font_digit_struct *test_osd_font_glyph_for_char(CHAR_T ch)
{
    return test_osd_font_glyph((UINT32_T)(UINT8_T)ch);
}

static UINT32_T test_osd_clock_text_width(CONST CHAR_T *text)
{
    UINT32_T width = 0U;
    UINT16_T i;

    if (text == NULL) {
        return 0U;
    }

    for (i = 0U; text[i] != '\0' && i < 8U; i++) {
        CONST gui_font_digit_struct *glyph = test_osd_font_glyph_for_char(text[i]);

        if (glyph != NULL) {
            width += glyph->width;
        }
    }
    return width;
}

static UINT8_T *test_osd_clock_canvas(void)
{
    return s_clock_bitmap[s_clock_building];
}

static VOID_T test_osd_put_pixel(INT_T x, INT_T y, UINT32_T argb)
{
    UINT32_T *pixel;
    UINT8_T *canvas = test_osd_clock_canvas();

    if (x < 0 || y < 0 || x >= (INT_T)TEST_OSD_CLOCK_BITMAP_W ||
        y >= (INT_T)TEST_OSD_CLOCK_BITMAP_H) {
        return;
    }
    pixel = (UINT32_T *)&canvas[
        ((UINT32_T)y * TEST_OSD_CLOCK_BITMAP_W + (UINT32_T)x) * TEST_OSD_BPP];
    *pixel = test_osd_argb_blend(*pixel, argb);
}

static VOID_T test_osd_clear_clock(VOID_T)
{
    os_memset(test_osd_clock_canvas(), 0,
              TEST_OSD_CLOCK_BITMAP_W * TEST_OSD_CLOCK_BITMAP_H * TEST_OSD_BPP);
}

/* Decode the same packed emWin glyph format used by the original OSD path.
 * Roboto53 stores 4-bit coverage values, so nibble*17 restores the full
 * 0..255 alpha range and preserves the original anti-aliased appearance. */
static VOID_T test_osd_draw_glyph(CONST gui_font_digit_struct *glyph,
                                  INT_T pen_x, INT_T origin_y)
{
    UINT8_T bit_point;
    UINT32_T stride;
    UINT16_T row;

    if (glyph == NULL || glyph->data == NULL) {
        return;
    }

    bit_point = glyph->bit_point != 0U ? glyph->bit_point : 4U;
    stride = ((UINT32_T)glyph->x_size * bit_point + (8U - bit_point)) / 8U;

    for (row = 0U; row < glyph->y_size; row++) {
        UINT16_T col;
        CONST UINT8_T *src = &glyph->data[(UINT32_T)row * stride];

        for (col = 0U; col < glyph->x_size; col++) {
            UINT8_T coverage;

            if (bit_point == 4U) {
                UINT8_T byte = src[col >> 1U];
                coverage = (col & 1U) != 0U ? (UINT8_T)(byte & 0x0FU)
                                           : (UINT8_T)((byte >> 4U) & 0x0FU);
                coverage = (UINT8_T)(coverage * 17U);
            } else if (bit_point == 2U) {
                coverage = (UINT8_T)((src[col >> 2U] >>
                                      (6U - 2U * (col & 3U))) & 0x03U);
                coverage = (UINT8_T)(coverage * 85U);
            } else if (bit_point == 1U) {
                coverage = (src[col >> 3U] & (UINT8_T)(0x80U >> (col & 7U))) != 0U
                               ? 0xFFU : 0U;
            } else {
                coverage = 0U;
            }

            if (coverage != 0U) {
                test_osd_put_pixel(pen_x + glyph->x_pos + (INT_T)col,
                                   origin_y + glyph->y_pos + (INT_T)row,
                                   ((UINT32_T)coverage << 24U) |
                                   (TEST_OSD_CLOCK_ARGB & 0x00FFFFFFU));
            }
        }
    }
}

static VOID_T test_osd_draw_clock(CONST CHAR_T *text)
{
    UINT32_T text_width;
    INT_T pen_x;
    UINT16_T i;

    if (text == NULL) {
        return;
    }

    /* The Roboto53 table advances digits by 25px and ':' by 11px, making
     * HH:MM:SS 172px wide. Centering the measured advance avoids the old
     * fixed-step overflow that clipped the last characters. origin_y=0 is
     * intentional: the glyph y_pos places the 32px digit body at y=10..42
     * inside the 44px business canvas, matching the original asset layout. */
    text_width = test_osd_clock_text_width(text);
    pen_x = ((INT_T)TEST_OSD_CLOCK_BITMAP_W - (INT_T)text_width) / 2;

    for (i = 0U; text[i] != '\0' && i < 8U; i++) {
        CONST gui_font_digit_struct *glyph = test_osd_font_glyph_for_char(text[i]);

        if (glyph == NULL) {
            continue;
        }
        test_osd_draw_glyph(glyph, pen_x, 0);
        pen_x += glyph->width;
    }
}

VOID_T test_osd_init(VOID_T)
{
    test_osd_set_clock("00:00:00");
}

VOID_T test_osd_set_clock(CONST CHAR_T *text)
{
    s_clock_building = (UINT8_T)(s_clock_active ^ 1U);
    test_osd_clear_clock();
    test_osd_draw_clock(text);
    s_clock_active = s_clock_building;
}

CONST UINT8_T *test_osd_clock_bitmap(UINT16_T *width, UINT16_T *height)
{
    if (width != NULL) {
        *width = TEST_OSD_CLOCK_BITMAP_W;
    }
    if (height != NULL) {
        *height = TEST_OSD_CLOCK_BITMAP_H;
    }
    return s_clock_bitmap[s_clock_active];
}

CONST UINT8_T *test_osd_wifi_bitmap(UINT8_T level)
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
