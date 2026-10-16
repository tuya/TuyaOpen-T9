/**
 * @file test_image.c
 * @brief CLI for the one-shot image operations (ximg)
 *
 * @details
 * Source pictures come from the camera's SP channel, so the camera must be up
 * in frame mode before encode / transform / dump_sp. probe and decode read a
 * file instead and need no camera.
 *
 * @version 0.1
 * @date 2026-08-27
 * @copyright Copyright (c) Tuya Inc.
 */

#include <stdio.h>

#include <common/avdk_pixel_types.h>
#include <os/mem.h>
#include <os/str.h>

#include "cli_tuya_test.h"
#include "tkl_camera_mipi.h"
#include "tkl_fs.h"
#include "tkl_image.h"
#include "tkl_media_buf.h"
#include "tkl_output.h"
#include "tkl_system.h"

#define IMG_TEST_DEFAULT_PREFIX   "ximg"
#define IMG_TEST_JPEG_MARGIN      4096U
#define IMG_TEST_CAPTURE_TIMEOUT  1000U

extern INT_T test_fs_mount(CONST CHAR_T *path, FS_DEV_TYPE_T dev_type);
/* The camera is brought up by "xmipi cam"; see test_camera.c. */
extern OPERATE_RET test_camera_get_sp_info(UINT16_T *width, UINT16_T *height,
                                           UINT32_T *format, UINT32_T *timeout_ms);

/**
 * @brief Map a CLI format word to TKL_PIXEL_FMT_E.
 * @param[in] name nv12 / yuyv / rgb565 / rgb888
 * @return TKL_PIXEL_FMT_E, TKL_PIXEL_FMT_UNKNOWN when unrecognised
 */
static UINT32_T test_image_fmt_parse(const char *name)
{
    if (name == NULL) {
        return (UINT32_T)TKL_PIXEL_FMT_UNKNOWN;
    }
    if (os_strcmp(name, "nv12") == 0) {
        return (UINT32_T)TKL_PIXEL_FMT_NV12;
    }
    if (os_strcmp(name, "yuyv") == 0) {
        return (UINT32_T)TKL_PIXEL_FMT_YUYV;
    }
    if (os_strcmp(name, "rgb565") == 0) {
        return (UINT32_T)TKL_PIXEL_FMT_RGB565;
    }
    if (os_strcmp(name, "rgb888") == 0) {
        return (UINT32_T)TKL_PIXEL_FMT_RGB888;
    }
    return (UINT32_T)TKL_PIXEL_FMT_UNKNOWN;
}

static const char *test_image_fmt_name(UINT32_T fmt)
{
    switch (fmt) {
    case (UINT32_T)TKL_PIXEL_FMT_NV12:
        return "nv12";
    case (UINT32_T)TKL_PIXEL_FMT_YUYV:
        return "yuyv";
    case (UINT32_T)TKL_PIXEL_FMT_RGB565:
        return "rgb565";
    case (UINT32_T)TKL_PIXEL_FMT_RGB888:
        return "rgb888";
    case (UINT32_T)TKL_PIXEL_FMT_ARGB8888:
        return "argb8888";
    default:
        return "unknown";
    }
}

/* Media buffers come from the frame-buffer slabs, not the general PSRAM heap:
 * the software JPEG path already spends ~614 KB of that heap on its NV12->YUYV
 * scratch, so taking the picture and the bitstream from it too overruns it.
 * Going through tkl_media_buf also puts them in its accounting. */
static UINT8_T *test_image_pixel_alloc(UINT32_T size)
{
    return (UINT8_T *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL, size, TKL_MEDIA_BUF_OWNER_TEST);
}

static UINT8_T *test_image_stream_alloc(UINT32_T size)
{
    return (UINT8_T *)tkl_media_buf_alloc(TKL_BUF_USE_STREAM, size, TKL_MEDIA_BUF_OWNER_TEST);
}

static void test_image_buf_free(UINT8_T *ptr)
{
    tkl_media_buf_free(TKL_MEDIA_BUF_OWNER_TEST, ptr);
}

static OPERATE_RET test_image_write_file(const char *path, const UINT8_T *data, UINT32_T size)
{
    TUYA_FILE file = NULL;
    INT_T write_len;

    if (path == NULL || data == NULL || size == 0U) {
        return OPRT_INVALID_PARM;
    }

    /* xvenc mounts the card in its own helpers; ximg has to do the same or a
     * cold-boot run fails at fopen with nothing to explain why */
    if (test_fs_mount(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD) != 0) {
        tkl_log_output("ximg: mount %s failed\r\n", TEST_SDCARD_MOUNT_POINT);
        return OPRT_COM_ERROR;
    }

    file = tkl_fopen(path, "wb");
    if (file == NULL) {
        tkl_log_output("ximg open failed: %s\r\n", path);
        return OPRT_COM_ERROR;
    }

    write_len = tkl_fwrite((VOID_T *)data, (INT_T)size, file);
    (void)tkl_fsync((INT_T)file);
    (void)tkl_fclose(file);
    if (write_len != (INT_T)size) {
        tkl_log_output("ximg write failed: expect=%u actual=%d\r\n",
                       (unsigned int)size, write_len);
        return OPRT_COM_ERROR;
    }

    tkl_log_output("ximg wrote %s (%u bytes)\r\n", path, (unsigned int)size);
    return OPRT_OK;
}

/**
 * @brief Grab one NV12 picture from the camera SP channel.
 *
 * @param[out] buf    receives a pixel-slab block the caller must free
 * @param[out] width  receives the picture width
 * @param[out] height receives the picture height
 * @param[out] size   receives the valid byte count
 *
 * @return OPRT_OK on success
 */
static OPERATE_RET test_image_capture_nv12(UINT8_T **buf,
                                           UINT16_T *width,
                                           UINT16_T *height,
                                           UINT32_T *size)
{
    TKL_MEDIA_BUF_T frame;
    UINT16_T sp_w = 0U;
    UINT16_T sp_h = 0U;
    UINT32_T sp_fmt = 0U;
    UINT32_T timeout = IMG_TEST_CAPTURE_TIMEOUT;
    UINT32_T alloc_size;
    UINT32_T copy_size;
    UINT8_T *nv12;
    OPERATE_RET ret;

    ret = test_camera_get_sp_info(&sp_w, &sp_h, &sp_fmt, &timeout);
    if (ret != OPRT_OK || sp_w == 0U || sp_h == 0U) {
        tkl_log_output("ximg: camera SP not ready. Bring it up in frame mode first:\r\n"
                       "  xmipi cam open display <mp_w> <mp_h> nv12 <sp_w> <sp_h>\r\n"
                       "  e.g. xmipi cam open display 640 1024 nv12 640 480\r\n");
        return OPRT_RESOURCE_NOT_READY;
    }

    /* The SP channel is usually opened RGB888 (it exists for face detection);
     * the format argument is what switches it. Encoding and transforming both
     * need NV12, so say exactly that rather than producing a wrong image. */
    if (sp_fmt != (UINT32_T)TKL_PIXEL_FMT_NV12) {
        tkl_log_output("ximg: camera SP format is %u, need NV12. Re-open with nv12:\r\n"
                       "  xmipi cam open display 640 1024 nv12 %u %u\r\n",
                       (unsigned int)sp_fmt,
                       (unsigned int)sp_w, (unsigned int)sp_h);
        return OPRT_NOT_SUPPORTED;
    }

    alloc_size = (UINT32_T)sp_w * sp_h * 3U / 2U;
    nv12 = test_image_pixel_alloc(alloc_size);
    if (nv12 == NULL) {
        return OPRT_MALLOC_FAILED;
    }

    /* Channel must be SP explicitly: MP carries display and/or h264, i.e. flexa,
     * and a flexa channel has no frame queue to pull from.
     *
     * The camera lends this buffer - it must go back through frame.free_cb, and
     * the pool is only two deep - so copy into our own block and release
     * straight away. */
    os_memset(&frame, 0, sizeof(frame));
    ret = tkl_camera_mipi_get_frame(&frame, TUYA_CAMERA_MIPI_CHANNEL_SP, timeout);
    if (ret != OPRT_OK) {
        test_image_buf_free(nv12);
        tkl_log_output("ximg: capture failed: %d\r\n", ret);
        return ret;
    }

    copy_size = (frame.size < alloc_size) ? frame.size : alloc_size;
    os_memcpy(nv12, frame.data, copy_size);
    *width = frame.width;
    *height = frame.height;
    frame.free_cb(frame.data);

    *buf = nv12;
    *size = copy_size;
    return OPRT_OK;
}

/**
 * @brief Read a whole file into a stream media buffer.
 *
 * @param[in]  path file on the SD card
 * @param[out] buf  receives a TKL_BUF_USE_STREAM block the caller releases with
 *                  test_image_buf_free()
 * @param[out] len  receives the byte count
 *
 * @note Stream slab rather than the PSRAM heap, for the same reason as
 *       test_image_pixel_alloc(): the software encoder needs ~600 KB of
 *       *contiguous* heap, and a JPEG parked in it is enough to break that.
 *
 * @return OPRT_OK on success
 */
static OPERATE_RET test_image_read_file(const char *path, UINT8_T **buf, UINT32_T *len)
{
    TUYA_FILE file = NULL;
    UINT8_T *stream;
    INT_T size;
    INT_T got;

    if (path == NULL || buf == NULL || len == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (test_fs_mount(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD) != 0) {
        tkl_log_output("ximg: mount %s failed\r\n", TEST_SDCARD_MOUNT_POINT);
        return OPRT_COM_ERROR;
    }

    file = tkl_fopen(path, "rb");
    if (file == NULL) {
        tkl_log_output("ximg: open %s failed\r\n", path);
        return OPRT_COM_ERROR;
    }

    (void)tkl_fseek(file, 0, SEEK_END);
    size = tkl_ftell(file);
    (void)tkl_fseek(file, 0, SEEK_SET);
    if (size <= 0) {
        (void)tkl_fclose(file);
        tkl_log_output("ximg: %s is empty\r\n", path);
        return OPRT_COM_ERROR;
    }

    stream = test_image_stream_alloc((UINT32_T)size);
    if (stream == NULL) {
        (void)tkl_fclose(file);
        return OPRT_MALLOC_FAILED;
    }

    got = tkl_fread(stream, size, file);
    (void)tkl_fclose(file);
    if (got != size) {
        test_image_buf_free(stream);
        tkl_log_output("ximg: short read on %s: expect=%d actual=%d\r\n",
                       path, (int)size, (int)got);
        return OPRT_COM_ERROR;
    }

    *buf = stream;
    *len = (UINT32_T)size;
    return OPRT_OK;
}

/**
 * @brief ximg probe <file> - read a JPEG's header
 * @param[in] path JPEG file on the SD card
 * @return OPRT_OK on success
 */
static OPERATE_RET test_image_probe_file(const char *path)
{
    TKL_MEDIA_BUF_T in;
    TKL_IMAGE_INFO_T info;
    UINT8_T *stream = NULL;
    UINT32_T len = 0U;
    OPERATE_RET ret;

    ret = test_image_read_file(path, &stream, &len);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&in, 0, sizeof(in));
    in.data = stream;
    in.size = len;
    in.kind = (UINT8_T)TKL_MEDIA_BUF_STREAM;
    in.codec = (UINT32_T)TKL_MEDIA_CODEC_JPEG;

    ret = tkl_image_probe(&in, &info);
    if (ret == OPRT_OK) {
        tkl_log_output("ximg probe: %ux%u codec=%u sampling=%s\r\n",
                       info.width, info.height,
                       (unsigned int)info.codec,
                       test_image_fmt_name(info.format));
    } else {
        tkl_log_output("ximg probe failed: %d\r\n", ret);
    }

    test_image_buf_free(stream);
    return ret;
}

/**
 * @brief Rows the post-processor actually stores.
 *
 * @param[in] fmt    TKL_PIXEL_FMT_E of the output
 * @param[in] height requested height
 *
 * @details
 * NV12 stores an even row count, RGB rounds up to a 16-row block -
 * tkl_pp_output_size() is where this lives for real. The file on the card
 * therefore holds more rows than were asked for, and ffplay has to be told the
 * stored count or the picture shears, which looks exactly like a decoder bug.
 *
 * @return stored row count
 */
static UINT16_T test_image_stored_rows(UINT32_T fmt, UINT16_T height)
{
    if (fmt == (UINT32_T)TKL_PIXEL_FMT_RGB565 ||
        fmt == (UINT32_T)TKL_PIXEL_FMT_RGB888) {
        return (UINT16_T)(((UINT32_T)height + 15U) & ~15U);
    }
    return (UINT16_T)(((UINT32_T)height + 1U) & ~1U);
}

/**
 * @brief The ffplay -pixel_format word for a decoded file.
 * @param[in] fmt TKL_PIXEL_FMT_E
 * @return the format word, NULL when there is no sensible one
 */
static const char *test_image_ffplay_fmt(UINT32_T fmt)
{
    switch (fmt) {
    case (UINT32_T)TKL_PIXEL_FMT_NV12:
        return "nv12";
    case (UINT32_T)TKL_PIXEL_FMT_RGB565:
        return "rgb565le";
    case (UINT32_T)TKL_PIXEL_FMT_RGB888:
        /* the PP writes 4 bytes per pixel for RGB888, so the file is not rgb24 */
        return "rgb0";
    default:
        return NULL;
    }
}

/**
 * @brief ximg decode <file> - JPEG file back to a raw picture on the card
 *
 * @param[in] path     JPEG on the SD card
 * @param[in] fmt      TKL_PIXEL_FMT_E of the result: NV12, RGB565 or RGB888
 * @param[in] dst_w    0 = the stream's own width
 * @param[in] dst_h    0 = the stream's own height
 * @param[in] out_path where to write the raw picture
 *
 * @details
 * Probes first, which is what makes the geometry arguments optional and lets
 * the log state what the stream carried *before* the decoder scaled it. Scaling
 * is free here because the decoder feeds the same post-processor that
 * tkl_image_transform() uses.
 *
 * @return OPRT_OK on success
 */
static OPERATE_RET test_image_decode_file(const char *path, UINT32_T fmt,
                                          UINT16_T dst_w, UINT16_T dst_h,
                                          const char *out_path)
{
    TKL_MEDIA_BUF_T in;
    TKL_MEDIA_BUF_T out;
    TKL_IMAGE_INFO_T info;
    UINT8_T *stream = NULL;
    UINT8_T *pixel = NULL;
    UINT32_T len = 0U;
    UINT32_T need;
    UINT32_T t_dec;
    CONST CHAR_T *ff;
    OPERATE_RET ret;

    /* YUYV parses as a format and has a non-zero buffer size, so the generic
     * size check below would let it through to a decoder that only speaks
     * NV12/RGB - say so here instead of relaying a driver-level -4 */
    if (fmt != (UINT32_T)TKL_PIXEL_FMT_NV12 &&
        fmt != (UINT32_T)TKL_PIXEL_FMT_RGB565 &&
        fmt != (UINT32_T)TKL_PIXEL_FMT_RGB888) {
        tkl_log_output("ximg decode: %s is not a decode target (nv12|rgb565|rgb888)\r\n",
                       test_image_fmt_name(fmt));
        return OPRT_NOT_SUPPORTED;
    }

    ret = test_image_read_file(path, &stream, &len);
    if (ret != OPRT_OK) {
        return ret;
    }

    os_memset(&in, 0, sizeof(in));
    in.data = stream;
    in.size = len;
    in.kind = (UINT8_T)TKL_MEDIA_BUF_STREAM;
    in.codec = (UINT32_T)TKL_MEDIA_CODEC_JPEG;

    os_memset(&info, 0, sizeof(info));
    ret = tkl_image_probe(&in, &info);
    if (ret != OPRT_OK) {
        tkl_log_output("ximg decode: %s is not a decodable JPEG (%d)\r\n", path, ret);
        test_image_buf_free(stream);
        return ret;
    }

    if (dst_w == 0U) {
        dst_w = info.width;
    }
    if (dst_h == 0U) {
        dst_h = info.height;
    }

    need = tkl_image_buffer_size(fmt, dst_w, dst_h);
    if (need == 0U) {
        test_image_buf_free(stream);
        return OPRT_INVALID_PARM;
    }

    pixel = test_image_pixel_alloc(need);
    if (pixel == NULL) {
        tkl_log_output("ximg decode: no room for %u bytes (%ux%u %s)\r\n",
                       (unsigned int)need, dst_w, dst_h, test_image_fmt_name(fmt));
        test_image_buf_free(stream);
        return OPRT_MALLOC_FAILED;
    }

    os_memset(&out, 0, sizeof(out));
    out.data = pixel;
    out.size = need;
    out.kind = (UINT8_T)TKL_MEDIA_BUF_PIXEL;
    out.width = dst_w;
    out.height = dst_h;
    out.format = fmt;

    /* timed around tkl_image_decode() only, the same way encode is, so the two
     * directions can be compared */
    t_dec = (UINT32_T)tkl_system_get_millisecond();
    ret = tkl_image_decode(&in, &out);
    t_dec = (UINT32_T)tkl_system_get_millisecond() - t_dec;

    if (ret == OPRT_OK) {
        tkl_log_output("ximg decode: %s (%u bytes, %ux%u %s jpeg) -> %ux%u %s, "
                       "%u bytes, %ums\r\n",
                       path, (unsigned int)len,
                       info.width, info.height, test_image_fmt_name(info.format),
                       dst_w, dst_h, test_image_fmt_name(fmt),
                       (unsigned int)out.size, (unsigned int)t_dec);

        ff = test_image_ffplay_fmt(fmt);
        if (ff != NULL) {
            UINT16_T rows = test_image_stored_rows(fmt, dst_h);

            tkl_log_output("  ffplay -f rawvideo -pixel_format %s -video_size %ux%u %s\r\n",
                           ff, dst_w, rows, out_path);
            if (rows != dst_h) {
                tkl_log_output("  (%u rows stored, not %u - the PP rounds the height up)\r\n",
                               rows, dst_h);
            }
            if (fmt == (UINT32_T)TKL_PIXEL_FMT_RGB888) {
                tkl_log_output("  (rgb888 is 4 bytes/pixel here; try bgr0 if the "
                               "channels come out swapped)\r\n");
            }
        }

        ret = test_image_write_file(out_path, out.data, out.size);
    } else {
        tkl_log_output("ximg decode failed: %d\r\n", ret);
    }

    test_image_buf_free(pixel);
    test_image_buf_free(stream);
    return ret;
}

/**
 * @brief ximg encode - capture NV12 then JPEG-encode it
 * @param[in] quality 1..100, 0 for default
 * @param[in] use_sw  TRUE forces the software path (byte comparable with xvenc jpegsw)
 * @param[in] path    output file
 * @return OPRT_OK on success
 */
static OPERATE_RET test_image_encode(UINT8_T quality, BOOL_T use_sw, const char *path)
{
    TKL_MEDIA_BUF_T in;
    TKL_MEDIA_BUF_T out;
    TKL_IMAGE_ENC_PARAM_T param;
    UINT8_T *nv12 = NULL;
    UINT8_T *jpeg = NULL;
    UINT16_T width = 0U;
    UINT16_T height = 0U;
    UINT32_T nv12_size = 0U;
    UINT32_T jpeg_cap;
    UINT32_T t_enc;
    OPERATE_RET ret;

    ret = test_image_capture_nv12(&nv12, &width, &height, &nv12_size);
    if (ret != OPRT_OK) {
        return ret;
    }

    jpeg_cap = ((UINT32_T)width * height) + IMG_TEST_JPEG_MARGIN;
    jpeg = test_image_stream_alloc(jpeg_cap);
    if (jpeg == NULL) {
        test_image_buf_free(nv12);
        return OPRT_MALLOC_FAILED;
    }

    os_memset(&in, 0, sizeof(in));
    in.data = nv12;
    in.size = nv12_size;
    in.kind = (UINT8_T)TKL_MEDIA_BUF_PIXEL;
    in.width = width;
    in.height = height;
    in.format = (UINT32_T)TKL_PIXEL_FMT_NV12;

    os_memset(&out, 0, sizeof(out));
    out.data = jpeg;
    out.size = jpeg_cap;
    out.kind = (UINT8_T)TKL_MEDIA_BUF_STREAM;

    os_memset(&param, 0, sizeof(param));
    param.quality = quality;
    param.codec = (UINT32_T)TKL_MEDIA_CODEC_JPEG;
    param.prefer_sw = use_sw;

    /* timed around tkl_image_encode() only - capture and file write are excluded
     * so the number is comparable across quality settings */
    t_enc = (UINT32_T)tkl_system_get_millisecond();
    ret = tkl_image_encode(&in, &param, &out);
    t_enc = (UINT32_T)tkl_system_get_millisecond() - t_enc;

    if (ret == OPRT_OK) {
        UINT32_T px = (UINT32_T)width * height;

        tkl_log_output("ximg encode: %ux%u nv12 -> jpeg %u bytes (%s, q=%u) %ums, %u.%02u bpp\r\n",
                       width, height, (unsigned int)out.size,
                       (use_sw == TRUE) ? "sw" : "hw",
                       (unsigned int)quality,
                       (unsigned int)t_enc,
                       (unsigned int)((out.size * 8U) / px),
                       (unsigned int)(((out.size * 800U) / px) % 100U));
        ret = test_image_write_file(path, out.data, out.size);
    } else {
        tkl_log_output("ximg encode failed: %d\r\n", ret);
    }

    test_image_buf_free(jpeg);
    test_image_buf_free(nv12);
    return ret;
}

/**
 * @brief ximg dump_sp <prefix> - dump one SP frame as raw NV12 *and* YUYV
 *
 * @param[in] prefix file prefix under /sdcard, no extension
 *
 * @details
 * Not an encode test - a **format-conversion forensics** tool. It writes the
 * same frame twice, before and after tkl_image_transform(), plus a .txt naming
 * the exact ffplay command for each. When a picture comes out with swapped
 * chroma or a half-line shift, this is what tells you which side did it.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
static OPERATE_RET test_image_dump_sp(const char *prefix)
{
    TKL_MEDIA_BUF_T in;
    TKL_MEDIA_BUF_T out;
    TKL_IMAGE_XFORM_T xform;
    UINT8_T *nv12 = NULL;
    UINT8_T *yuyv = NULL;
    UINT16_T width = 0U;
    UINT16_T height = 0U;
    UINT32_T nv12_size = 0U;
    UINT32_T yuyv_size;
    CHAR_T path[128];
    CHAR_T info[384];
    INT_T info_len;
    OPERATE_RET ret;

    if (prefix == NULL || prefix[0] == '\0') {
        prefix = IMG_TEST_DEFAULT_PREFIX;
    }

    ret = test_image_capture_nv12(&nv12, &width, &height, &nv12_size);
    if (ret != OPRT_OK) {
        return ret;
    }

    yuyv_size = (UINT32_T)width * height * 2U;
    yuyv = test_image_pixel_alloc(yuyv_size);
    if (yuyv == NULL) {
        test_image_buf_free(nv12);
        return OPRT_MALLOC_FAILED;
    }

    os_memset(&in, 0, sizeof(in));
    in.data = nv12;
    in.size = nv12_size;
    in.kind = (UINT8_T)TKL_MEDIA_BUF_PIXEL;
    in.width = width;
    in.height = height;
    in.format = (UINT32_T)TKL_PIXEL_FMT_NV12;

    os_memset(&out, 0, sizeof(out));
    out.data = yuyv;
    out.size = yuyv_size;

    os_memset(&xform, 0, sizeof(xform));
    xform.width = width;
    xform.height = height;
    xform.format = (UINT32_T)TKL_PIXEL_FMT_YUYV;

    ret = tkl_image_transform(&in, &xform, &out);
    if (ret != OPRT_OK) {
        tkl_log_output("ximg dump_sp nv12->yuyv failed: %d\r\n", ret);
        goto done;
    }

    snprintf(path, sizeof(path), "%s/%s.nv12", TEST_SDCARD_MOUNT_POINT, prefix);
    ret = test_image_write_file(path, nv12, nv12_size);
    if (ret != OPRT_OK) {
        goto done;
    }

    snprintf(path, sizeof(path), "%s/%s.yuyv", TEST_SDCARD_MOUNT_POINT, prefix);
    ret = test_image_write_file(path, yuyv, out.size);
    if (ret != OPRT_OK) {
        goto done;
    }

    /* The pixel order the software encoder wants is YVYU, not YUYV - the name
     * of the TKL format is the historical one. Spelling the ffplay line out
     * here is the whole point: guessing it wrong looks exactly like a bug in
     * the conversion. */
    info_len = snprintf(info, sizeof(info),
                        "width=%u\nheight=%u\nnv12_size=%u\nyuyv_size=%u\n"
                        "nv12=tight packed, stride=width\n"
                        "yuyv=soft-jpeg packing (Y0 V Y1 U / YVYU-like)\n"
                        "ffplay -f rawvideo -pixel_format nv12 -video_size %ux%u %s.nv12\n"
                        "ffplay -f rawvideo -pixel_format yvyu422 -video_size %ux%u %s.yuyv\n",
                        (unsigned int)width, (unsigned int)height,
                        (unsigned int)nv12_size, (unsigned int)out.size,
                        (unsigned int)width, (unsigned int)height, prefix,
                        (unsigned int)width, (unsigned int)height, prefix);
    if (info_len > 0) {
        snprintf(path, sizeof(path), "%s/%s.txt", TEST_SDCARD_MOUNT_POINT, prefix);
        (void)test_image_write_file(path, (CONST UINT8_T *)info, (UINT32_T)info_len);
    }

    tkl_log_output("ximg dump_sp ok: %ux%u nv12=%u yuyv=%u -> %s/%s.{nv12,yuyv,txt}\r\n",
                   width, height, (unsigned int)nv12_size, (unsigned int)out.size,
                   TEST_SDCARD_MOUNT_POINT, prefix);

done:
    test_image_buf_free(yuyv);
    test_image_buf_free(nv12);
    return ret;
}

/**
 * @brief ximg qsweep - encode ONE captured frame at several qualities
 *
 * @details
 * `ximg encode` grabs a fresh frame every time, so two runs differ in exposure
 * and content as well as in quality - on a live sensor with AE still converging
 * that swamps what is being measured. This captures once and reuses the same
 * NV12 buffer for every encode, so the only variable left is quality.
 *
 * @param[in] use_sw    TRUE for the software path
 * @param[in] qualities quality values to sweep
 * @param[in] count     number of entries in @p qualities
 * @return OPRT_OK if every encode succeeded
 */
static OPERATE_RET test_image_qsweep(BOOL_T use_sw, const UINT8_T *qualities, UINT8_T count)
{
    TKL_MEDIA_BUF_T in;
    UINT8_T *nv12 = NULL;
    UINT8_T *jpeg = NULL;
    UINT16_T width = 0U;
    UINT16_T height = 0U;
    UINT32_T nv12_size = 0U;
    UINT32_T jpeg_cap;
    UINT32_T px;
    OPERATE_RET ret;
    UINT8_T i;

    if (qualities == NULL || count == 0U) {
        return OPRT_INVALID_PARM;
    }

    ret = test_image_capture_nv12(&nv12, &width, &height, &nv12_size);
    if (ret != OPRT_OK) {
        return ret;
    }

    jpeg_cap = ((UINT32_T)width * height) + IMG_TEST_JPEG_MARGIN;
    jpeg = test_image_stream_alloc(jpeg_cap);
    if (jpeg == NULL) {
        test_image_buf_free(nv12);
        return OPRT_MALLOC_FAILED;
    }

    px = (UINT32_T)width * height;

    os_memset(&in, 0, sizeof(in));
    in.data = nv12;
    in.size = nv12_size;
    in.kind = (UINT8_T)TKL_MEDIA_BUF_PIXEL;
    in.width = width;
    in.height = height;
    in.format = (UINT32_T)TKL_PIXEL_FMT_NV12;

    tkl_log_output("ximg qsweep: %ux%u %s, one frame, %u qualities\r\n",
                   width, height, (use_sw == TRUE) ? "sw" : "hw", (unsigned int)count);

    for (i = 0U; i < count; i++) {
        TKL_IMAGE_ENC_PARAM_T param;
        TKL_MEDIA_BUF_T out;
        char path[64];
        UINT32_T t_enc;
        OPERATE_RET one;

        os_memset(&out, 0, sizeof(out));
        out.data = jpeg;
        out.size = jpeg_cap;
        out.kind = (UINT8_T)TKL_MEDIA_BUF_STREAM;

        os_memset(&param, 0, sizeof(param));
        param.quality = qualities[i];
        param.codec = (UINT32_T)TKL_MEDIA_CODEC_JPEG;
        param.prefer_sw = use_sw;

        t_enc = (UINT32_T)tkl_system_get_millisecond();
        one = tkl_image_encode(&in, &param, &out);
        t_enc = (UINT32_T)tkl_system_get_millisecond() - t_enc;

        if (one != OPRT_OK) {
            tkl_log_output("  q=%-3u FAILED %d\r\n", (unsigned int)qualities[i], one);
            ret = one;
            continue;
        }

        snprintf(path, sizeof(path), "%s/qs_q%u_%s.jpg",
                 TEST_SDCARD_MOUNT_POINT, (unsigned int)qualities[i],
                 (use_sw == TRUE) ? "sw" : "hw");

        tkl_log_output("  q=%-3u %7u bytes  %u.%02u bpp  %ums  -> %s\r\n",
                       (unsigned int)qualities[i],
                       (unsigned int)out.size,
                       (unsigned int)((out.size * 8U) / px),
                       (unsigned int)(((out.size * 800U) / px) % 100U),
                       (unsigned int)t_enc,
                       path);

        one = test_image_write_file(path, out.data, out.size);
        if (one != OPRT_OK) {
            ret = one;
        }
    }

    test_image_buf_free(jpeg);
    test_image_buf_free(nv12);
    return ret;
}

/**
 * @brief ximg transform - capture NV12 then scale / convert it
 * @param[in] dst_w target width, 0 keeps the source width
 * @param[in] dst_h target height, 0 keeps the source height
 * @param[in] fmt   TKL_PIXEL_FMT_E of the result
 * @param[in] path  output file
 * @return OPRT_OK on success
 */
static OPERATE_RET test_image_transform(UINT16_T dst_w, UINT16_T dst_h,
                                        UINT32_T fmt, const char *path)
{
    TKL_MEDIA_BUF_T in;
    TKL_MEDIA_BUF_T out;
    TKL_IMAGE_XFORM_T xform;
    UINT8_T *nv12 = NULL;
    UINT8_T *dst = NULL;
    UINT16_T width = 0U;
    UINT16_T height = 0U;
    UINT32_T nv12_size = 0U;
    UINT32_T dst_size;
    OPERATE_RET ret;

    ret = test_image_capture_nv12(&nv12, &width, &height, &nv12_size);
    if (ret != OPRT_OK) {
        return ret;
    }

    if (dst_w == 0U) {
        dst_w = width;
    }
    if (dst_h == 0U) {
        dst_h = height;
    }

    dst_size = tkl_image_buffer_size(fmt, dst_w, dst_h);
    if (dst_size == 0U) {
        test_image_buf_free(nv12);
        tkl_log_output("ximg: format %s not supported as a transform target\r\n",
                       test_image_fmt_name(fmt));
        return OPRT_NOT_SUPPORTED;
    }

    dst = test_image_pixel_alloc(dst_size);
    if (dst == NULL) {
        test_image_buf_free(nv12);
        return OPRT_MALLOC_FAILED;
    }

    os_memset(&in, 0, sizeof(in));
    in.data = nv12;
    in.size = nv12_size;
    in.kind = (UINT8_T)TKL_MEDIA_BUF_PIXEL;
    in.width = width;
    in.height = height;
    in.format = (UINT32_T)TKL_PIXEL_FMT_NV12;

    os_memset(&out, 0, sizeof(out));
    out.data = dst;
    out.size = dst_size;

    os_memset(&xform, 0, sizeof(xform));
    xform.width = dst_w;
    xform.height = dst_h;
    xform.format = fmt;

    ret = tkl_image_transform(&in, &xform, &out);
    if (ret == OPRT_OK) {
        tkl_log_output("ximg transform: %ux%u nv12 -> %ux%u %s (%u bytes)\r\n",
                       width, height, dst_w, dst_h,
                       test_image_fmt_name(fmt), (unsigned int)out.size);
        ret = test_image_write_file(path, out.data, out.size);
    } else {
        tkl_log_output("ximg transform failed: %d\r\n", ret);
    }

    test_image_buf_free(dst);
    test_image_buf_free(nv12);
    return ret;
}

void cli_tuya_image_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: ximg <probe <file>|decode <file> [fmt] [w] [h] [outfile]|"
                 "encode [quality] [sw|hw] [file]|qsweep [sw|hw] [q ...]|"
                 "transform <w> <h> <fmt> [file]|dump_sp [prefix]>\r\n"
                 "  fmt = nv12|yuyv|rgb565|rgb888 (decode: nv12|rgb565|rgb888)\r\n"
                 "  probe/decode read a JPEG from the card; the rest need the camera\r\n"
                 "  encode/qsweep/transform take their input from the camera SP channel\r\n"
                 "  qsweep encodes ONE captured frame at every quality given\r\n"
                 "    (default 70 80 90 95 100), so only quality varies - use it\r\n"
                 "    instead of repeated 'encode' when comparing quality\r\n"
                 "  dump_sp writes one frame as raw .nv12 AND .yuyv plus a .txt\r\n"
                 "    naming the ffplay line for each - conversion forensics\r\n");
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "probe") == 0) {
        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: ximg probe <file>\r\n");
            return;
        }
        ret = test_image_probe_file(argv[2]);
        snprintf(pcWriteBuffer, xWriteBufferLen, "ximg probe %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "decode") == 0) {
        UINT32_T fmt = (UINT32_T)TKL_PIXEL_FMT_NV12;
        UINT16_T dst_w = 0U;
        UINT16_T dst_h = 0U;
        CHAR_T out_default[64];
        const char *out_path = NULL;
        int dim = 0;
        int i;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: ximg decode <file> [fmt] [w] [h] [outfile]\r\n");
            tkl_log_output("  fmt = nv12 (default) | rgb565 | rgb888\r\n"
                           "  w/h default to the JPEG's own size; give them to scale\r\n"
                           "  e.g. ximg decode %s/%s_enc.jpg nv12\r\n",
                           TEST_SDCARD_MOUNT_POINT, IMG_TEST_DEFAULT_PREFIX);
            return;
        }

        /* order-free like the encode arm: a word that names a format is the
         * format, a leading '/' is the output file, anything else is w then h */
        for (i = 3; i < argc; i++) {
            UINT32_T parsed = test_image_fmt_parse(argv[i]);

            if (parsed != (UINT32_T)TKL_PIXEL_FMT_UNKNOWN) {
                fmt = parsed;
            } else if (argv[i][0] == '/') {
                out_path = argv[i];
            } else if (dim == 0) {
                dst_w = (UINT16_T)os_strtoul(argv[i], NULL, 10);
                dim = 1;
            } else if (dim == 1) {
                dst_h = (UINT16_T)os_strtoul(argv[i], NULL, 10);
                dim = 2;
            }
        }

        if (out_path == NULL) {
            snprintf(out_default, sizeof(out_default), "%s/%s_dec.%s",
                     TEST_SDCARD_MOUNT_POINT, IMG_TEST_DEFAULT_PREFIX,
                     test_image_fmt_name(fmt));
            out_path = out_default;
        }

        ret = test_image_decode_file(argv[2], fmt, dst_w, dst_h, out_path);
        snprintf(pcWriteBuffer, xWriteBufferLen, "ximg decode %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "encode") == 0) {
        UINT8_T quality = 0U;
        BOOL_T use_sw = FALSE;
        const char *path = TEST_SDCARD_MOUNT_POINT "/" IMG_TEST_DEFAULT_PREFIX "_enc.jpg";
        int i;

        for (i = 2; i < argc; i++) {
            if (os_strcmp(argv[i], "sw") == 0) {
                use_sw = TRUE;
            } else if (os_strcmp(argv[i], "hw") == 0) {
                use_sw = FALSE;
            } else if (argv[i][0] == '/') {
                path = argv[i];
            } else {
                quality = (UINT8_T)os_strtoul(argv[i], NULL, 10);
            }
        }

        ret = test_image_encode(quality, use_sw, path);
        snprintf(pcWriteBuffer, xWriteBufferLen, "ximg encode %s (%d), %s\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret,
                 (use_sw == TRUE) ? "sw" : "hw");

    } else if (os_strcmp(cmd, "qsweep") == 0) {
        /* comparing quality settings needs the same pixels in every encode */
        STATIC CONST UINT8_T k_default_q[] = { 70U, 80U, 90U, 95U, 100U };
        UINT8_T qualities[8];
        UINT8_T count = 0U;
        BOOL_T use_sw = FALSE;
        int i;

        for (i = 2; i < argc; i++) {
            if (os_strcmp(argv[i], "sw") == 0) {
                use_sw = TRUE;
            } else if (os_strcmp(argv[i], "hw") == 0) {
                use_sw = FALSE;
            } else if (count < (UINT8_T)(sizeof(qualities) / sizeof(qualities[0]))) {
                qualities[count] = (UINT8_T)os_strtoul(argv[i], NULL, 10);
                count++;
            }
        }

        if (count == 0U) {
            os_memcpy(qualities, k_default_q, sizeof(k_default_q));
            count = (UINT8_T)(sizeof(k_default_q) / sizeof(k_default_q[0]));
        }

        ret = test_image_qsweep(use_sw, qualities, count);
        snprintf(pcWriteBuffer, xWriteBufferLen, "ximg qsweep %s (%d), %s\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret,
                 (use_sw == TRUE) ? "sw" : "hw");

    } else if (os_strcmp(cmd, "transform") == 0) {
        UINT16_T dst_w = 0U;
        UINT16_T dst_h = 0U;
        UINT32_T fmt;
        const char *path = TEST_SDCARD_MOUNT_POINT "/" IMG_TEST_DEFAULT_PREFIX "_xfm.bin";

        if (argc < 5) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: ximg transform <w> <h> <fmt> [file]\r\n"
                     "  fmt = nv12|yuyv|rgb565|rgb888 (yuyv cannot scale)\r\n");
            return;
        }

        dst_w = (UINT16_T)os_strtoul(argv[2], NULL, 10);
        dst_h = (UINT16_T)os_strtoul(argv[3], NULL, 10);
        fmt = test_image_fmt_parse(argv[4]);
        if (fmt == (UINT32_T)TKL_PIXEL_FMT_UNKNOWN) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "ximg: unknown format '%s'\r\n", argv[4]);
            return;
        }
        if (argc > 5) {
            path = argv[5];
        }

        ret = test_image_transform(dst_w, dst_h, fmt, path);
        snprintf(pcWriteBuffer, xWriteBufferLen, "ximg transform %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "dump_sp") == 0) {
        ret = test_image_dump_sp((argc >= 3) ? argv[2] : NULL);
        snprintf(pcWriteBuffer, xWriteBufferLen, "ximg dump_sp %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: ximg <probe <file>|decode <file> [fmt] [w] [h]|"
                 "encode [quality] [sw|hw] [file]|"
                 "qsweep [sw|hw] [q ...]|transform <w> <h> <fmt> [file]|"
                 "dump_sp [prefix]>\r\n");
    }
}
