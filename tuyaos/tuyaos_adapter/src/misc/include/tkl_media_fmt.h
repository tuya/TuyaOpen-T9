/**
 * @file tkl_media_fmt.h
 * @brief TKL <-> vendor pixel-format mapping, adapter-internal.
 *
 * @details
 * Separate from tkl_media_buf.h because this is the half that needs the vendor
 * pixel enum. Include it only where a format has to be translated.
 *
 * @version 0.1
 * @date 2026-08-27
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 */

#ifndef __TKL_MEDIA_FMT_H__
#define __TKL_MEDIA_FMT_H__

#include "tuya_cloud_types.h"

#include <common/avdk_pixel_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Public pixel format -> platform pixel format.
 * @param[in] fmt TUYA_FRAME_FMT_E (pixel members only)
 * @return bk_pixel_format_t, BK_PIXEL_FORMAT_INVALID when unmapped
 */
STATIC INLINE UINT32_T __t9_media_fmt_to_bk(UINT32_T fmt)
{
    switch (fmt) {
    case TUYA_FRAME_FMT_NV12:
        return (UINT32_T)BK_PIXEL_FORMAT_NV12;
    case TUYA_FRAME_FMT_YUYV:
        return (UINT32_T)BK_PIXEL_FORMAT_YUYV;
    case TUYA_FRAME_FMT_RGB565:
        return (UINT32_T)BK_PIXEL_FORMAT_RGB565;
    case TUYA_FRAME_FMT_RGB888:
        return (UINT32_T)BK_PIXEL_FORMAT_RGB888;
    case TUYA_FRAME_FMT_ARGB8888:
        return (UINT32_T)BK_PIXEL_FORMAT_ARGB8888;
    case TUYA_FRAME_FMT_ABGR8888:
        return (UINT32_T)BK_PIXEL_FORMAT_ABGR8888;
    case TUYA_FRAME_FMT_BGGR10:
        return (UINT32_T)BK_PIXEL_FORMAT_BGGR10;
    case TUYA_FRAME_FMT_GBRG10:
        return (UINT32_T)BK_PIXEL_FORMAT_GBRG10;
    case TUYA_FRAME_FMT_GRBG10:
        return (UINT32_T)BK_PIXEL_FORMAT_GRBG10;
    case TUYA_FRAME_FMT_RGGB10:
        return (UINT32_T)BK_PIXEL_FORMAT_RGGB10;
    default:
        return (UINT32_T)BK_PIXEL_FORMAT_INVALID;
    }
}

/**
 * @brief Platform pixel format -> public pixel format.
 * @param[in] fmt bk_pixel_format_t
 * @return TUYA_FRAME_FMT_E pixel member, TUYA_FRAME_FMT_UNKNOWN when unmapped
 */
STATIC INLINE UINT32_T __t9_media_fmt_from_bk(UINT32_T fmt)
{
    switch (fmt) {
    case (UINT32_T)BK_PIXEL_FORMAT_NV12:
        return (UINT32_T)TUYA_FRAME_FMT_NV12;
    case (UINT32_T)BK_PIXEL_FORMAT_YUYV:
        return (UINT32_T)TUYA_FRAME_FMT_YUYV;
    case (UINT32_T)BK_PIXEL_FORMAT_RGB565:
        return (UINT32_T)TUYA_FRAME_FMT_RGB565;
    case (UINT32_T)BK_PIXEL_FORMAT_RGB888:
        return (UINT32_T)TUYA_FRAME_FMT_RGB888;
    case (UINT32_T)BK_PIXEL_FORMAT_ARGB8888:
        return (UINT32_T)TUYA_FRAME_FMT_ARGB8888;
    case (UINT32_T)BK_PIXEL_FORMAT_ABGR8888:
        return (UINT32_T)TUYA_FRAME_FMT_ABGR8888;
    case (UINT32_T)BK_PIXEL_FORMAT_BGGR10:
        return (UINT32_T)TUYA_FRAME_FMT_BGGR10;
    case (UINT32_T)BK_PIXEL_FORMAT_GBRG10:
        return (UINT32_T)TUYA_FRAME_FMT_GBRG10;
    case (UINT32_T)BK_PIXEL_FORMAT_GRBG10:
        return (UINT32_T)TUYA_FRAME_FMT_GRBG10;
    case (UINT32_T)BK_PIXEL_FORMAT_RGGB10:
        return (UINT32_T)TUYA_FRAME_FMT_RGGB10;
    default:
        return (UINT32_T)TUYA_FRAME_FMT_UNKNOWN;
    }
}

/**
 * @brief Public rotation enum -> degrees, the unit the vendor blit expects.
 *
 * @details
 * TUYA_FRAME_ROTATE_E is ordinal (0/1/2/3); bk_gpu_blit_config_t.rotate_degree
 * and its consumers compare against real angles (== 90, == 270). Passing the
 * enum straight through makes the vendor take its no-rotate path without any
 * error, so every crossing between the two layers must go through here.
 *
 * @param[in] rotate TUYA_FRAME_ROTATE_E
 * @return 0 / 90 / 180 / 270
 */
STATIC INLINE UINT16_T __t9_rotate_to_deg(TUYA_FRAME_ROTATE_E rotate)
{
    switch (rotate) {
    case TUYA_FRAME_ROTATE_90:
        return 90U;
    case TUYA_FRAME_ROTATE_180:
        return 180U;
    case TUYA_FRAME_ROTATE_270:
        return 270U;
    case TUYA_FRAME_ROTATE_0:
    default:
        return 0U;
    }
}

/**
 * @brief Degrees -> public rotation enum.
 * @param[in] deg 0 / 90 / 180 / 270; anything else maps to TUYA_FRAME_ROTATE_0
 * @return TUYA_FRAME_ROTATE_E
 */
STATIC INLINE TUYA_FRAME_ROTATE_E __t9_rotate_from_deg(UINT16_T deg)
{
    switch (deg) {
    case 90U:
        return TUYA_FRAME_ROTATE_90;
    case 180U:
        return TUYA_FRAME_ROTATE_180;
    case 270U:
        return TUYA_FRAME_ROTATE_270;
    case 0U:
    default:
        return TUYA_FRAME_ROTATE_0;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* __TKL_MEDIA_FMT_H__ */
