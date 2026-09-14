/**
 * @file test_image.c
 * @brief CLI for the one-shot image operations (ximg)
 *
 * @details
 * Source pictures come from the camera's SP channel, so the camera must be up
 * in frame mode before encode / transform / dump_sp. get_info and decode read
 * a file instead and need no camera.
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
#include "tkl_mipi_csi.h"
#include "tkl_fs.h"
#include "tkl_image_codec.h"
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
 * @brief Map a CLI format word to TUYA_FRAME_FMT_E.
 * @param[in] name nv12 / yuyv / rgb565 / rgb888
 * @return TUYA_FRAME_FMT_E, TUYA_FRAME_FMT_UNKNOWN when unrecognised
 */
static UINT32_T test_image_fmt_parse(const char *name)
{
    if (name == NULL) {
        return (UINT32_T)TUYA_FRAME_FMT_UNKNOWN;
    }
    if (os_strcmp(name, "nv12") == 0) {
        return (UINT32_T)TUYA_FRAME_FMT_NV12;
    }
    if (os_strcmp(name, "yuyv") == 0) {
        return (UINT32_T)TUYA_FRAME_FMT_YUYV;
    }
    if (os_strcmp(name, "rgb565") == 0) {
        return (UINT32_T)TUYA_FRAME_FMT_RGB565;
    }
    if (os_strcmp(name, "rgb888") == 0) {
        return (UINT32_T)TUYA_FRAME_FMT_RGB888;
    }
    return (UINT32_T)TUYA_FRAME_FMT_UNKNOWN;
}

static const char *test_image_fmt_name(UINT32_T fmt)
{
    switch (fmt) {
    case (UINT32_T)TUYA_FRAME_FMT_NV12:
        return "nv12";
    case (UINT32_T)TUYA_FRAME_FMT_YUYV:
        return "yuyv";
    case (UINT32_T)TUYA_FRAME_FMT_RGB565:
        return "rgb565";
    case (UINT32_T)TUYA_FRAME_FMT_RGB888:
        return "rgb888";
    case (UINT32_T)TUYA_FRAME_FMT_ARGB8888:
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
    return (UINT8_T *)tkl_media_buf_alloc(TKL_BUF_USE_PIXEL, size, TUYA_MEDIA_BUF_OWNER_TEST);
}

static UINT8_T *test_image_stream_alloc(UINT32_T size)
{
    return (UINT8_T *)tkl_media_buf_alloc(TKL_BUF_USE_STREAM, size, TUYA_MEDIA_BUF_OWNER_TEST);
}

static void test_image_buf_free(UINT8_T *ptr)
{
    tkl_media_buf_free(TUYA_MEDIA_BUF_OWNER_TEST, ptr);
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
    TUYA_MEDIA_BUF_T frame;
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
    if (sp_fmt != (UINT32_T)TUYA_FRAME_FMT_NV12) {
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
    ret = tkl_mipi_csi_get_frame(&frame, TUYA_MIPI_CSI_CHANNEL_SECONDARY, timeout);
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
 * @brief ximg get_info <file> - read a JPEG's header
 * @param[in] path JPEG file on the SD card
 * @return OPRT_OK on success
 */
static OPERATE_RET test_image_get_info_file(const char *path)
{
    TUYA_IMAGE_CODEC_STREAM_T bit;
    TUYA_IMAGE_CODEC_BUF_T probe;
    UINT8_T *stream = NULL;
    UINT32_T len = 0U;
    OPERATE_RET ret;

    ret = test_image_read_file(path, &stream, &len);
    if (ret != OPRT_OK) {
        return ret;
    }

    bit.data = stream;
    bit.size = len;

    ret = tkl_image_codec_get_info(&bit, &probe);
    if (ret == OPRT_OK) {
        tkl_log_output("ximg get_info: %ux%u sampling=%s nv12 size=%u\r\n",
                       probe.width, probe.height,
                       test_image_fmt_name(probe.format),
                       (unsigned int)probe.size);
    } else {
        tkl_log_output("ximg get_info failed: %d\r\n", ret);
    }

    test_image_buf_free(stream);
    return ret;
}

/**
 * @brief Rows the post-processor actually stores.
 *
 * @param[in] fmt    TUYA_FRAME_FMT_E of the output
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
    if (fmt == (UINT32_T)TUYA_FRAME_FMT_RGB565 ||
        fmt == (UINT32_T)TUYA_FRAME_FMT_RGB888) {
        return (UINT16_T)(((UINT32_T)height + 15U) & ~15U);
    }
    if (fmt == (UINT32_T)TUYA_FRAME_FMT_NV12) {
        /* UV plane = height/2 rows on 16-row DMA segments -> height to a
         * multiple of 32. 320x240 would store 256 rows (see the adapter's
         * tkl_pp_output_size note). */
        return (UINT16_T)(((UINT32_T)height + 31U) & ~31U);
    }
    return (UINT16_T)(((UINT32_T)height + 1U) & ~1U);
}

/* The buffer tkl_image_codec_convert() writes for a pixel output: same rounding the
 * adapter enforces (see stored_rows), RGB888 at 4 bytes/pixel. The test keeps
 * the rule so a wrong size on the card side is distinguishable from a wrong
 * size the adapter rejected. */
static UINT32_T test_image_pixel_bytes(UINT32_T fmt, UINT16_T width, UINT16_T height)
{
    UINT16_T rows = test_image_stored_rows(fmt, height);

    if (fmt == (UINT32_T)TUYA_FRAME_FMT_NV12) {
        return (UINT32_T)width * rows * 3U / 2U;
    }
    if (fmt == (UINT32_T)TUYA_FRAME_FMT_RGB565) {
        return (UINT32_T)width * rows * 2U;
    }
    if (fmt == (UINT32_T)TUYA_FRAME_FMT_RGB888) {
        return (UINT32_T)width * rows * 4U;
    }
    if (fmt == (UINT32_T)TUYA_FRAME_FMT_YUYV) {
        return (UINT32_T)width * height * 2U;
    }
    return 0U;
}

/**
 * @brief The ffplay -pixel_format word for a decoded file.
 * @param[in] fmt TUYA_FRAME_FMT_E
 * @return the format word, NULL when there is no sensible one
 */
static const char *test_image_ffplay_fmt(UINT32_T fmt)
{
    switch (fmt) {
    case (UINT32_T)TUYA_FRAME_FMT_NV12:
        return "nv12";
    case (UINT32_T)TUYA_FRAME_FMT_RGB565:
        return "rgb565le";
    case (UINT32_T)TUYA_FRAME_FMT_RGB888:
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
 * @param[in] fmt      TUYA_FRAME_FMT_E of the result: NV12, RGB565 or RGB888
 * @param[in] dst_w    0 = the stream's own width
 * @param[in] dst_h    0 = the stream's own height
 * @param[in] out_path where to write the raw picture
 *
 * @details
 * Probes first, which is what makes the geometry arguments optional and lets
 * the log state what the stream carried *before* the decoder scaled it. Scaling
 * is free here because the decoder feeds the same post-processor that the
 * adapter's pixel-to-pixel convert leg uses.
 *
 * @return OPRT_OK on success
 */
static OPERATE_RET test_image_decode_file(const char *path, UINT32_T fmt,
                                          UINT16_T dst_w, UINT16_T dst_h,
                                          const char *out_path)
{
    TUYA_IMAGE_CODEC_BUF_T in;
    TUYA_IMAGE_CODEC_BUF_T out;
    TUYA_IMAGE_CODEC_BUF_T probe;
    TUYA_IMAGE_CODEC_STREAM_T bit;
    UINT8_T *stream = NULL;
    UINT8_T *pixel = NULL;
    UINT32_T len = 0U;
    UINT32_T need;
    UINT32_T t_dec;
    CONST CHAR_T *ff;
    OPERATE_RET ret;

    /* YUYV is a decode target only for a 4:2:2 stream's native geometry - the
     * adapter packs the decoder's semi-planar bytes into true YUYV (see the
     * same restriction in the pixel_bytes size check below). */
    if (fmt != (UINT32_T)TUYA_FRAME_FMT_NV12 &&
        fmt != (UINT32_T)TUYA_FRAME_FMT_RGB565 &&
        fmt != (UINT32_T)TUYA_FRAME_FMT_RGB888 &&
        fmt != (UINT32_T)TUYA_FRAME_FMT_YUYV) {
        tkl_log_output("ximg decode: %s is not a decode target "
                       "(nv12|rgb565|rgb888|yuyv)\r\n",
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
    in.format = (UINT32_T)TUYA_FRAME_FMT_JPEG;

    bit.data = stream;
    bit.size = len;

    ret = tkl_image_codec_get_info(&bit, &probe);
    if (ret != OPRT_OK) {
        tkl_log_output("ximg decode: %s is not a decodable JPEG (%d)\r\n", path, ret);
        test_image_buf_free(stream);
        return ret;
    }

    if (dst_w == 0U) {
        dst_w = probe.width;
    }
    if (dst_h == 0U) {
        dst_h = probe.height;
    }

    need = test_image_pixel_bytes(fmt, dst_w, dst_h);
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
    out.width = dst_w;
    out.height = dst_h;
    out.format = fmt;

    /* timed around tkl_image_codec_convert() only, the same way encode is, so the
     * two directions can be compared */
    t_dec = (UINT32_T)tkl_system_get_millisecond();
    ret = tkl_image_codec_convert(&in, &out);
    t_dec = (UINT32_T)tkl_system_get_millisecond() - t_dec;

    if (ret == OPRT_OK) {
        tkl_log_output("ximg decode: %s (%u bytes, %ux%u %s jpeg) -> %ux%u %s, "
                       "%u bytes, %ums\r\n",
                       path, (unsigned int)len,
                       probe.width, probe.height, test_image_fmt_name(probe.format),
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
            if (fmt == (UINT32_T)TUYA_FRAME_FMT_RGB888) {
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
 *
 * @param[in] use_sw TRUE composes the software path through the public API:
 *                   NV12->YUYV (CPU leg) then YUYV->JPEG (always software);
 *                   FALSE takes the adapter's default hardware encode.
 *                   Byte-comparable with xvenc jpegsw/jpeg like before.
 * @param[in] path   output file
 * @return OPRT_OK on success
 */
static OPERATE_RET test_image_encode(BOOL_T use_sw, const char *path)
{
    TUYA_IMAGE_CODEC_BUF_T in;
    TUYA_IMAGE_CODEC_BUF_T out;
    TUYA_IMAGE_CODEC_BUF_T yuyv;
    UINT8_T *nv12 = NULL;
    UINT8_T *yuyv_buf = NULL;
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
    in.width = width;
    in.height = height;
    in.format = (UINT32_T)TUYA_FRAME_FMT_NV12;

    os_memset(&out, 0, sizeof(out));
    out.data = jpeg;
    out.size = jpeg_cap;
    out.format = (UINT32_T)TUYA_FRAME_FMT_JPEG;

    /* timed around the convert(s) only - capture and file write are excluded.
     * The sw number includes the NV12->YUYV leg, exactly as the old prefer_sw
     * path did internally. */
    t_enc = (UINT32_T)tkl_system_get_millisecond();

    if (use_sw == TRUE) {
        os_memset(&yuyv, 0, sizeof(yuyv));
        yuyv.data = test_image_pixel_alloc(test_image_pixel_bytes(
                        (UINT32_T)TUYA_FRAME_FMT_YUYV, width, height));
        if (yuyv.data == NULL) {
            test_image_buf_free(jpeg);
            test_image_buf_free(nv12);
            return OPRT_MALLOC_FAILED;
        }
        yuyv.size = test_image_pixel_bytes((UINT32_T)TUYA_FRAME_FMT_YUYV, width, height);
        yuyv.width = width;
        yuyv.height = height;
        yuyv.format = (UINT32_T)TUYA_FRAME_FMT_YUYV;
        yuyv_buf = yuyv.data;

        /* 1. NV12 -> YUYV, the CPU fallback leg of the pixel convert */
        ret = tkl_image_codec_convert(&in, &yuyv);
        if (ret == OPRT_OK) {
            /* 2. YUYV input always reaches the software JPEG encoder */
            ret = tkl_image_codec_convert(&yuyv, &out);
        }
    } else {
        ret = tkl_image_codec_convert(&in, &out);
    }
    t_enc = (UINT32_T)tkl_system_get_millisecond() - t_enc;

    if (ret == OPRT_OK) {
        UINT32_T px = (UINT32_T)width * height;

        tkl_log_output("ximg encode: %ux%u nv12 -> jpeg %u bytes (%s) %ums, %u.%02u bpp\r\n",
                       width, height, (unsigned int)out.size,
                       (use_sw == TRUE) ? "sw" : "hw",
                       (unsigned int)t_enc,
                       (unsigned int)((out.size * 8U) / px),
                       (unsigned int)(((out.size * 800U) / px) % 100U));
        ret = test_image_write_file(path, out.data, out.size);
    } else {
        tkl_log_output("ximg encode failed: %d\r\n", ret);
    }

    test_image_buf_free(jpeg);
    if (yuyv_buf != NULL) {
        test_image_buf_free(yuyv_buf);
    }
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
 * same frame twice, before and after the NV12-to-YUYV convert, plus a .txt naming
 * the exact ffplay command for each. When a picture comes out with swapped
 * chroma or a half-line shift, this is what tells you which side did it.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
static OPERATE_RET test_image_dump_sp(const char *prefix)
{
    TUYA_IMAGE_CODEC_BUF_T in;
    TUYA_IMAGE_CODEC_BUF_T out;
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
    in.width = width;
    in.height = height;
    in.format = (UINT32_T)TUYA_FRAME_FMT_NV12;

    os_memset(&out, 0, sizeof(out));
    out.data = yuyv;
    out.size = yuyv_size;
    out.width = width;
    out.height = height;
    out.format = (UINT32_T)TUYA_FRAME_FMT_YUYV;

    ret = tkl_image_codec_convert(&in, &out);
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

    /* Public YUYV is the standard packed order everywhere (Y0 U Y1 V).
     * Spelling the ffplay line out here is the whole point: guessing it wrong
     * looks exactly like a bug in the conversion. */
    info_len = snprintf(info, sizeof(info),
                        "width=%u\nheight=%u\nnv12_size=%u\nyuyv_size=%u\n"
                        "nv12=tight packed, stride=width\n"
                        "yuyv=packed Y0 U Y1 V\n"
                        "ffplay -f rawvideo -pixel_format nv12 -video_size %ux%u %s.nv12\n"
                        "ffplay -f rawvideo -pixel_format yuyv422 -video_size %ux%u %s.yuyv\n",
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
 * @brief ximg transform - capture NV12 then scale / convert it
 * @param[in] dst_w target width, 0 keeps the source width
 * @param[in] dst_h target height, 0 keeps the source height
 * @param[in] fmt   TUYA_FRAME_FMT_E of the result
 * @param[in] path  output file
 * @return OPRT_OK on success
 */
static OPERATE_RET test_image_transform(UINT16_T dst_w, UINT16_T dst_h,
                                        UINT32_T fmt, const char *path)
{
    TUYA_IMAGE_CODEC_BUF_T in;
    TUYA_IMAGE_CODEC_BUF_T out;
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

    dst_size = test_image_pixel_bytes(fmt, dst_w, dst_h);
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
    in.width = width;
    in.height = height;
    in.format = (UINT32_T)TUYA_FRAME_FMT_NV12;

    os_memset(&out, 0, sizeof(out));
    out.data = dst;
    out.size = dst_size;
    out.width = dst_w;
    out.height = dst_h;
    out.format = fmt;

    ret = tkl_image_codec_convert(&in, &out);
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

/* ---------------------------------------------------------------------------
 * ximg memcpy - same-format region copy: correctness suite + backend bench.
 * Pure synthetic frames, the camera is not needed.
 * ------------------------------------------------------------------------- */

/* Test-only backend override inside the adapter: 0 = auto, 1 = CPU,
 * 2 = general DMA. Not part of the public API. */
extern INT_T g_tkl_image_memcpy_force;

static UINT32_T test_image_mc_pass = 0U;
static UINT32_T test_image_mc_fail = 0U;

static void test_image_pattern_fill(UINT8_T *buf, UINT32_T len)
{
    UINT32_T i;

    for (i = 0U; i < len; i++) {
        buf[i] = (UINT8_T)((i * 31U) + ((i >> 7U) * 7U));
    }
}

static void test_image_frame_init(TUYA_IMAGE_CODEC_BUF_T *f, UINT8_T *data, UINT32_T size,
                                  UINT32_T fmt, UINT16_T w, UINT16_T h)
{
    os_memset(f, 0, sizeof(*f));
    f->data = data;
    f->size = size;
    f->format = fmt;
    f->width = w;
    f->height = h;
}

static void test_image_region_set(TUYA_IMAGE_CODEC_BUF_T *f, UINT16_T x, UINT16_T y,
                                  UINT16_T w, UINT16_T h)
{
    f->region.x = x;
    f->region.y = y;
    f->region.width = w;
    f->region.height = h;
}

static BOOL_T test_image_bytes_equal(CONST UINT8_T *a, CONST UINT8_T *b, UINT32_T len)
{
    UINT32_T i;

    for (i = 0U; i < len; i++) {
        if (a[i] != b[i]) {
            tkl_log_output("  mismatch at %u: %02x vs %02x\r\n",
                           (unsigned int)i, a[i], b[i]);
            return FALSE;
        }
    }
    return TRUE;
}

static void test_image_mc_case(const char *name, BOOL_T pass)
{
    if (pass == TRUE) {
        test_image_mc_pass++;
    } else {
        test_image_mc_fail++;
    }
    tkl_log_output("ximg memcpy %-16s %s\r\n", name, (pass == TRUE) ? "PASS" : "FAIL");
}

/**
 * @brief ximg memcpy - the correctness suite.
 *
 * @details
 * Every positive case builds an expected buffer with plain os_memcpy loops as
 * the reference, then checks what tkl_image_codec_memcpy() produced against it
 * byte-for-byte - both backends must agree with the CPU reference. The
 * rejection cases pin the exact error code the contract promises.
 *
 * @return OPRT_OK when every case passed
 */
static OPERATE_RET test_image_memcpy_suite(void)
{
    OPERATE_RET ret;

    test_image_mc_pass = 0U;
    test_image_mc_fail = 0U;

    /* full frame, YUYV 128x96 = 24 KB (auto picks the big-copy backend) */
    {
        CONST UINT32_T sz = 128U * 96U * 2U;
        UINT8_T *src = test_image_pixel_alloc(sz);
        UINT8_T *dst = test_image_pixel_alloc(sz);
        UINT8_T *ref = test_image_pixel_alloc(sz);
        TUYA_IMAGE_CODEC_BUF_T s;
        TUYA_IMAGE_CODEC_BUF_T d;
        BOOL_T ok = FALSE;

        if (src != NULL && dst != NULL && ref != NULL) {
            test_image_pattern_fill(src, sz);
            os_memset(dst, 0, sz);
            os_memcpy(ref, src, sz);
            test_image_frame_init(&s, src, sz, (UINT32_T)TUYA_FRAME_FMT_YUYV,
                                  128U, 96U);
            test_image_frame_init(&d, dst, sz, (UINT32_T)TUYA_FRAME_FMT_YUYV,
                                  128U, 96U);
            ret = tkl_image_codec_memcpy(&s, &d);
            ok = (ret == OPRT_OK) && test_image_bytes_equal(dst, ref, sz);
        }
        test_image_mc_case("full_yuyv", ok);
        test_image_buf_free(src);
        test_image_buf_free(dst);
        test_image_buf_free(ref);
    }

    /* full frame, NV12 128x96 = 18 KB (two planes, both moved) */
    {
        CONST UINT32_T sz = 128U * 96U * 3U / 2U;
        UINT8_T *src = test_image_pixel_alloc(sz);
        UINT8_T *dst = test_image_pixel_alloc(sz);
        UINT8_T *ref = test_image_pixel_alloc(sz);
        TUYA_IMAGE_CODEC_BUF_T s;
        TUYA_IMAGE_CODEC_BUF_T d;
        BOOL_T ok = FALSE;

        if (src != NULL && dst != NULL && ref != NULL) {
            test_image_pattern_fill(src, sz);
            os_memset(dst, 0, sz);
            os_memcpy(ref, src, sz);
            test_image_frame_init(&s, src, sz, (UINT32_T)TUYA_FRAME_FMT_NV12,
                                  128U, 96U);
            test_image_frame_init(&d, dst, sz, (UINT32_T)TUYA_FRAME_FMT_NV12,
                                  128U, 96U);
            ret = tkl_image_codec_memcpy(&s, &d);
            ok = (ret == OPRT_OK) && test_image_bytes_equal(dst, ref, sz);
        }
        test_image_mc_case("full_nv12", ok);
        test_image_buf_free(src);
        test_image_buf_free(dst);
        test_image_buf_free(ref);
    }

    /* sub-rect paste: 32x16 window from a 64x48 picture lands at (40,20)
     * of a 128x96 canvas; everything outside the window must survive */
    {
        CONST UINT32_T src_sz = 64U * 48U * 2U;
        CONST UINT32_T dst_sz = 128U * 96U * 2U;
        UINT8_T *src = test_image_pixel_alloc(src_sz);
        UINT8_T *dst = test_image_pixel_alloc(dst_sz);
        UINT8_T *ref = test_image_pixel_alloc(dst_sz);
        TUYA_IMAGE_CODEC_BUF_T s;
        TUYA_IMAGE_CODEC_BUF_T d;
        UINT32_T y;
        BOOL_T ok = FALSE;

        if (src != NULL && dst != NULL && ref != NULL) {
            test_image_pattern_fill(src, src_sz);
            test_image_pattern_fill(dst, dst_sz);
            os_memcpy(ref, dst, dst_sz);
            /* the reference: paste the same window with plain os_memcpy */
            for (y = 0U; y < 16U; y++) {
                os_memcpy(ref + ((20U + y) * 128U * 2U) + (40U * 2U),
                          src + ((4U + y) * 64U * 2U) + (8U * 2U), 32U * 2U);
            }
            test_image_frame_init(&s, src, src_sz, (UINT32_T)TUYA_FRAME_FMT_YUYV,
                                  64U, 48U);
            test_image_frame_init(&d, dst, dst_sz, (UINT32_T)TUYA_FRAME_FMT_YUYV,
                                  128U, 96U);
            test_image_region_set(&s, 8U, 4U, 32U, 16U);
            test_image_region_set(&d, 40U, 20U, 32U, 16U);
            ret = tkl_image_codec_memcpy(&s, &d);
            ok = (ret == OPRT_OK) && test_image_bytes_equal(dst, ref, dst_sz);
        }
        test_image_mc_case("rect_paste_yuyv", ok);
        test_image_buf_free(src);
        test_image_buf_free(dst);
        test_image_buf_free(ref);
    }

    /* overlap: one buffer, rows shifted down by 8 (memmove semantics) */
    {
        CONST UINT16_T w = 96U;
        CONST UINT16_T h = 64U;
        CONST UINT32_T row = (UINT32_T)w * 2U;
        CONST UINT32_T sz = row * h;
        UINT8_T *a = test_image_pixel_alloc(sz);
        UINT8_T *b = test_image_pixel_alloc(sz);
        UINT8_T *ref = test_image_pixel_alloc(sz);
        TUYA_IMAGE_CODEC_BUF_T s;
        TUYA_IMAGE_CODEC_BUF_T d;
        UINT32_T y;
        BOOL_T ok = FALSE;

        if (a != NULL && b != NULL && ref != NULL) {
            /* reference: rows 0..55 land on 8..63, moved bottom-up */
            test_image_pattern_fill(a, sz);
            os_memcpy(ref, a, sz);
            for (y = h - 9U; ; y--) {
                os_memcpy(ref + (y + 8U) * row, ref + y * row, row);
                if (y == 0U) {
                    break;
                }
            }
            /* the call itself: src and dst are the same buffer */
            os_memcpy(b, a, sz);
            test_image_frame_init(&s, b, sz, (UINT32_T)TUYA_FRAME_FMT_YUYV,
                                  w, h);
            test_image_frame_init(&d, b, sz, (UINT32_T)TUYA_FRAME_FMT_YUYV,
                                  w, h);
            test_image_region_set(&s, 0U, 0U, w, h - 8U);
            test_image_region_set(&d, 0U, 8U, w, h - 8U);
            ret = tkl_image_codec_memcpy(&s, &d);
            ok = (ret == OPRT_OK) && test_image_bytes_equal(b, ref, sz);
        }
        test_image_mc_case("overlap_yuyv", ok);
        test_image_buf_free(a);
        test_image_buf_free(b);
        test_image_buf_free(ref);
    }

    /* rejection family - each pins its contract error code */
    {
        CONST UINT32_T sz = 128U * 96U * 2U;
        CONST UINT32_T nsz = 128U * 96U * 3U / 2U;
        UINT8_T *src = test_image_pixel_alloc(sz);
        UINT8_T *dst = test_image_pixel_alloc(sz);
        UINT8_T *srcn = test_image_pixel_alloc(nsz);
        UINT8_T *dstn = test_image_pixel_alloc(nsz);
        TUYA_IMAGE_CODEC_BUF_T s;
        TUYA_IMAGE_CODEC_BUF_T d;
        TUYA_IMAGE_CODEC_BUF_T sn;
        TUYA_IMAGE_CODEC_BUF_T dn;

        if (src != NULL && dst != NULL && srcn != NULL && dstn != NULL) {
            test_image_pattern_fill(src, sz);
            test_image_pattern_fill(srcn, nsz);
            test_image_frame_init(&s, src, sz, (UINT32_T)TUYA_FRAME_FMT_YUYV,
                                  128U, 96U);
            test_image_frame_init(&d, dst, sz, (UINT32_T)TUYA_FRAME_FMT_YUYV,
                                  128U, 96U);
            test_image_frame_init(&sn, srcn, nsz, (UINT32_T)TUYA_FRAME_FMT_NV12,
                                  128U, 96U);
            test_image_frame_init(&dn, dstn, nsz, (UINT32_T)TUYA_FRAME_FMT_NV12,
                                  128U, 96U);

            d.format = (UINT32_T)TUYA_FRAME_FMT_RGB565;
            ret = tkl_image_codec_memcpy(&s, &d);
            d.format = (UINT32_T)TUYA_FRAME_FMT_YUYV;
            test_image_mc_case("rej_cross_format", (ret == OPRT_NOT_SUPPORTED));

            test_image_region_set(&s, 120U, 0U, 16U, 96U);   /* 120+16 > 128 */
            ret = tkl_image_codec_memcpy(&s, &d);
            test_image_mc_case("rej_src_oob", (ret == OPRT_INVALID_PARM));
            test_image_region_set(&s, 0U, 0U, 0U, 0U);

            test_image_region_set(&d, 120U, 0U, 16U, 96U);
            ret = tkl_image_codec_memcpy(&s, &d);
            test_image_mc_case("rej_dst_oob", (ret == OPRT_INVALID_PARM));
            test_image_region_set(&d, 0U, 0U, 0U, 0U);

            test_image_region_set(&sn, 1U, 0U, 126U, 96U);   /* odd x on NV12 */
            ret = tkl_image_codec_memcpy(&sn, &dn);
            test_image_mc_case("rej_nv12_odd_x", (ret == OPRT_INVALID_PARM));
            test_image_region_set(&sn, 0U, 0U, 0U, 0U);

            test_image_region_set(&s, 0U, 0U, 15U, 16U);     /* odd width */
            test_image_region_set(&d, 0U, 0U, 15U, 16U);
            ret = tkl_image_codec_memcpy(&s, &d);
            test_image_mc_case("rej_yuyv_odd_w", (ret == OPRT_INVALID_PARM));
            test_image_region_set(&s, 0U, 0U, 0U, 0U);
            test_image_region_set(&d, 0U, 0U, 0U, 0U);

            ret = tkl_image_codec_memcpy(NULL, &d);
            test_image_mc_case("rej_null", (ret == OPRT_INVALID_PARM));

            test_image_region_set(&s, 0U, 0U, 32U, 32U);
            test_image_region_set(&d, 0U, 0U, 16U, 32U);     /* size mismatch */
            ret = tkl_image_codec_memcpy(&s, &d);
            test_image_mc_case("rej_win_mismatch", (ret == OPRT_INVALID_PARM));
            test_image_region_set(&s, 0U, 0U, 0U, 0U);
            test_image_region_set(&d, 0U, 0U, 0U, 0U);

            d.size = sz / 2U;                                /* undersized dst */
            ret = tkl_image_codec_memcpy(&s, &d);
            d.size = sz;
            test_image_mc_case("rej_dst_small", (ret == OPRT_INVALID_PARM));
        } else {
            test_image_mc_case("rej_alloc", FALSE);
        }
        test_image_buf_free(src);
        test_image_buf_free(dst);
        test_image_buf_free(srcn);
        test_image_buf_free(dstn);
    }

    tkl_log_output("ximg memcpy suite: pass=%u fail=%u (force=%s)\r\n",
                   (unsigned int)test_image_mc_pass, (unsigned int)test_image_mc_fail,
                   (g_tkl_image_memcpy_force == 1U) ? "cpu" :
                   (g_tkl_image_memcpy_force == 2U) ? "gdma" : "auto");

    return (test_image_mc_fail == 0U) ? OPRT_OK : OPRT_COM_ERROR;
}

/**
 * @brief ximg memcpy bench - per-op timing vs the os_memcpy reference.
 *
 * @details
 * Each size runs enough repetitions to outlast the millisecond clock, so the
 * numbers are ~us/op averages. Run it three times with force unset, force=cpu
 * and force=gdma to see the crossover the auto threshold should sit at.
 */
static OPERATE_RET test_image_memcpy_bench(void)
{
    static CONST struct {
        UINT16_T w;
        UINT16_T h;
    } k_sizes[] = {
        { 64U,  64U},    /*   8 KB */
        {256U, 128U},    /*  64 KB */
        {512U, 256U},    /* 256 KB */
    };
    UINT32_T idx;

    for (idx = 0U; idx < sizeof(k_sizes) / sizeof(k_sizes[0]); idx++) {
        CONST UINT32_T sz = (UINT32_T)k_sizes[idx].w * k_sizes[idx].h * 2U;
        UINT8_T *src = test_image_pixel_alloc(sz);
        UINT8_T *dst = test_image_pixel_alloc(sz);
        TUYA_IMAGE_CODEC_BUF_T s;
        TUYA_IMAGE_CODEC_BUF_T d;
        UINT32_T reps;
        UINT32_T t0;
        UINT32_T t_ref;
        UINT32_T t_api;
        UINT32_T i;

        if (src == NULL || dst == NULL) {
            test_image_buf_free(src);
            test_image_buf_free(dst);
            return OPRT_MALLOC_FAILED;
        }

        reps = (2U * 1024U * 1024U) / sz;
        if (reps < 4U) {
            reps = 4U;
        }
        test_image_pattern_fill(src, sz);
        test_image_frame_init(&s, src, sz, (UINT32_T)TUYA_FRAME_FMT_YUYV,
                              k_sizes[idx].w, k_sizes[idx].h);
        test_image_frame_init(&d, dst, sz, (UINT32_T)TUYA_FRAME_FMT_YUYV,
                              k_sizes[idx].w, k_sizes[idx].h);

        t0 = (UINT32_T)tkl_system_get_millisecond();
        for (i = 0U; i < reps; i++) {
            os_memcpy(dst, src, sz);
        }
        t_ref = (UINT32_T)tkl_system_get_millisecond() - t0;

        t0 = (UINT32_T)tkl_system_get_millisecond();
        for (i = 0U; i < reps; i++) {
            (void)tkl_image_codec_memcpy(&s, &d);
        }
        t_api = (UINT32_T)tkl_system_get_millisecond() - t0;

        tkl_log_output("ximg memcpy bench %4ux%-4u %7u B x%u: os_memcpy %ums "
                       "(~%uus/op), api %ums (~%uus/op)\r\n",
                       k_sizes[idx].w, k_sizes[idx].h, (unsigned int)sz,
                       (unsigned int)reps,
                       (unsigned int)t_ref, (unsigned int)((t_ref * 1000U) / reps),
                       (unsigned int)t_api, (unsigned int)((t_api * 1000U) / reps));

        test_image_buf_free(src);
        test_image_buf_free(dst);
    }

    return OPRT_OK;
}

void cli_tuya_image_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    OPERATE_RET ret = OPRT_OK;
    const char *cmd = NULL;

    if (pcWriteBuffer == NULL || xWriteBufferLen <= 0) {
        return;
    }

    /* Every command allocates from the media slabs - the JPEG stream on the
     * CODED heap, pixel output on UNCODED - and on a cold start nothing has
     * brought those up yet ("xmipi dsi open_gpu" or "xuvc open" do it as a
     * side effect; ximg needs neither). Without this the first allocation
     * asserts with "type:0 not init". Same call, same reason, as test_uvc.c's
     * and test_camera.c's open. Idempotent: safe to repeat while the camera
     * or display already has the slabs up. */
    tkl_media_buf_init();

    if (argc < 2) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: ximg <get_info <file>|decode <file> [fmt] [w] [h] [outfile]|"
                 "encode [hw|sw] [file]|transform <w> <h> <fmt> [file]|dump_sp [prefix]|"
                 "memcpy [bench] [force=cpu|gdma]>\r\n"
                 "  fmt = nv12|yuyv|rgb565|rgb888 (decode: nv12|rgb565|rgb888|yuyv, yuyv native only)\r\n"
                 "  get_info/decode read a JPEG from the card; the rest need the camera\r\n"
                 "  encode/transform take their input from the camera SP channel\r\n"
                 "  encode: quality fixed at best; hw = one convert, sw = the public-API\r\n"
                 "    composition (NV12->YUYV + YUYV->JPEG), byte-comparable with xvenc\r\n"
                 "  dump_sp writes one frame as raw .nv12 AND .yuyv plus a .txt\r\n"
                 "    naming the ffplay line for each - conversion forensics\r\n"
                 "  memcpy runs the same-format region copy suite (synthetic frames,\r\n"
                 "    no camera); 'bench' adds a timing table, force=cpu|gdma pins it\r\n");
        return;
    }

    cmd = argv[1];

    if (os_strcmp(cmd, "get_info") == 0) {
        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: ximg get_info <file>\r\n");
            return;
        }
        ret = test_image_get_info_file(argv[2]);
        snprintf(pcWriteBuffer, xWriteBufferLen, "ximg get_info %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else if (os_strcmp(cmd, "decode") == 0) {
        UINT32_T fmt = (UINT32_T)TUYA_FRAME_FMT_NV12;
        UINT16_T dst_w = 0U;
        UINT16_T dst_h = 0U;
        CHAR_T out_default[64];
        const char *out_path = NULL;
        int dim = 0;
        int i;

        if (argc < 3) {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Usage: ximg decode <file> [fmt] [w] [h] [outfile]\r\n");
            tkl_log_output("  fmt = nv12 (default) | rgb565 | rgb888 | yuyv (native size only)\r\n"
                           "  w/h default to the JPEG's own size; give them to scale\r\n"
                           "  yuyv output is true packed YUYV (Y0 U Y1 V), native size only\r\n"
                           "  e.g. ximg decode %s/%s_enc.jpg nv12\r\n",
                           TEST_SDCARD_MOUNT_POINT, IMG_TEST_DEFAULT_PREFIX);
            return;
        }

        /* order-free like the encode arm: a word that names a format is the
         * format, a leading '/' is the output file, anything else is w then h */
        for (i = 3; i < argc; i++) {
            UINT32_T parsed = test_image_fmt_parse(argv[i]);

            if (parsed != (UINT32_T)TUYA_FRAME_FMT_UNKNOWN) {
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
                /* quality is fixed at best now - say so instead of silently
                 * ignoring what used to be the first argument */
                snprintf(pcWriteBuffer, xWriteBufferLen,
                         "ximg encode: quality is fixed (best); use 'ximg encode [hw|sw] [file]'\r\n");
                return;
            }
        }

        ret = test_image_encode(use_sw, path);
        snprintf(pcWriteBuffer, xWriteBufferLen, "ximg encode %s (%d), %s\r\n",
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
        if (fmt == (UINT32_T)TUYA_FRAME_FMT_UNKNOWN) {
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

    } else if (os_strcmp(cmd, "memcpy") == 0) {
        BOOL_T bench = FALSE;
        int i;

        g_tkl_image_memcpy_force = 0;
        for (i = 2; i < argc; i++) {
            if (os_strcmp(argv[i], "bench") == 0) {
                bench = TRUE;
            } else if (os_strcmp(argv[i], "force=cpu") == 0) {
                g_tkl_image_memcpy_force = 1;
            } else if (os_strcmp(argv[i], "force=gdma") == 0) {
                g_tkl_image_memcpy_force = 2;
            }
        }

        ret = (bench == TRUE) ? test_image_memcpy_bench() : test_image_memcpy_suite();
        g_tkl_image_memcpy_force = 0;
        snprintf(pcWriteBuffer, xWriteBufferLen, "ximg memcpy %s (%d)\r\n",
                 (ret == OPRT_OK) ? "ok" : "failed", ret);

    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: ximg <get_info <file>|decode <file> [fmt] [w] [h]|"
                 "encode [hw|sw] [file]|transform <w> <h> <fmt> [file]|"
                 "dump_sp [prefix]|memcpy [bench] [force=cpu|gdma]>\r\n");
    }
}
