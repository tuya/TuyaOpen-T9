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
#include "tkl_media_types.h"

#include <common/avdk_pixel_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Public pixel format -> platform pixel format.
 * @param[in] fmt TUYA_FRAME_FMT_E (pixel members only)
 * @return bk_pixel_format_t, BK_PIXEL_FORMAT_INVALID when unmapped
 */
STATIC INLINE UINT32_T tkl_media_fmt_to_bk(UINT32_T fmt)
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
STATIC INLINE UINT32_T tkl_media_fmt_from_bk(UINT32_T fmt)
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

#ifdef __cplusplus
}
#endif

#endif /* __TKL_MEDIA_FMT_H__ */
