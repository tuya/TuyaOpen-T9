/**
 * @file tkl_media_buf.h
 * @brief TKL-internal media buffer pool, accounting and free_cb factory.
 *
 * @details
 * Single exit point for every media buffer allocation and release inside the
 * TKL adapter. Media hardware (ISP / GPU / VENC / VDEC) cannot take memory from
 * the ordinary heap: raw frames are DMA targets that must live in non-cacheable
 * memory, and encoded bitstreams come from a separate slab pool.
 *
 * Free of any vendor header; pixel-format mapping needs the vendor enum and so
 * lives in tkl_media_fmt.h instead.
 *
 * TKL-internal: buffers reach business wrapped in TKL_MEDIA_BUF_T, which carries
 * its own free_cb.
 *
 * @version 0.1
 * @date 2026-08-27
 *
 * @copyright Copyright 2021-2026 Tuya Inc. All Rights Reserved.
 */

#ifndef __TKL_MEDIA_BUF_H__
#define __TKL_MEDIA_BUF_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief What the buffer is going to hold - decides pool and cache policy.
 */
typedef enum {
    TKL_BUF_USE_STREAM = 0, /**< encoded bitstream -> MEM_SLAB_HEAP_CODED   (PSRAM1) */
    TKL_BUF_USE_PIXEL,      /**< pixel frame       -> MEM_SLAB_HEAP_UNCODED (PSRAM0) */
    TKL_BUF_USE_PIXEL_WT,   /**< pixel frame + PSRAM write-through (CPU writes, HW reads) */
    TKL_BUF_USE_MAX,
} TKL_BUF_USE_E;

/**
 * @brief Buffer owners, used for in-flight accounting only.
 *
 * @note **STREAM vs PIXEL is capacity separation, not cacheability** - both
 *       heaps carry the same non-cacheable MPU attribute. Pick by what the
 *       buffer holds, so a 1.4 MB pixel frame cannot eat the bitstream budget.
 *       PIXEL_WT is the one that changes behaviour: a PSRAM write-through
 *       channel, needed only for CPU-writes -> HW-reads.
 * @note VENC / VDEC / GPU are charged to only when a sink leaves alloc_cb NULL.
 *       A peripheral facade installs its own so its buffers land in a bucket
 *       close() can drain.
 * @note Keep this list short: one static thunk is generated per entry.
 */
#define TKL_MEDIA_BUF_OWNER_LIST(X)             \
    X(TKL_MEDIA_BUF_OWNER_CAMERA_DISPLAY, "cam.disp") \
    X(TKL_MEDIA_BUF_OWNER_CAMERA_STREAM,  "cam.strm") \
    X(TKL_MEDIA_BUF_OWNER_CAMERA_FRAME,   "cam.frm")  \
    X(TKL_MEDIA_BUF_OWNER_UVC_DISPLAY,    "uvc.disp") \
    X(TKL_MEDIA_BUF_OWNER_UVC_STREAM,     "uvc.strm") \
    X(TKL_MEDIA_BUF_OWNER_UVC_FRAME,      "uvc.frm")  \
    X(TKL_MEDIA_BUF_OWNER_DVP_DISPLAY,    "dvp.disp") \
    X(TKL_MEDIA_BUF_OWNER_DVP_STREAM,     "dvp.strm") \
    X(TKL_MEDIA_BUF_OWNER_OSD,            "osd")      \
    X(TKL_MEDIA_BUF_OWNER_IMAGE,          "image")    \
    X(TKL_MEDIA_BUF_OWNER_STREAM,         "stream")   \
    X(TKL_MEDIA_BUF_OWNER_VENC,           "venc")     \
    X(TKL_MEDIA_BUF_OWNER_VDEC,           "vdec")     \
    X(TKL_MEDIA_BUF_OWNER_GPU,            "gpu")      \
    X(TKL_MEDIA_BUF_OWNER_TEST,           "test")

typedef enum {
#define TKL_MEDIA_BUF_OWNER_ENUM(sym, name) sym,
    TKL_MEDIA_BUF_OWNER_LIST(TKL_MEDIA_BUF_OWNER_ENUM)
#undef TKL_MEDIA_BUF_OWNER_ENUM
    TKL_MEDIA_BUF_OWNER_MAX,
} TKL_MEDIA_BUF_OWNER_E;

/** Release callback signature; identical to tkl_mipi_dsi_flush_frame_t.free_cb. */
typedef VOID_T (*TKL_MEDIA_BUF_FREE_CB)(UINT8_T *buffer);

/**
 * @brief Bring up the backing slab pools.
 *
 * @warning **Destructive.** It resets each heap to one free block and forgets
 *          every live allocation, including the ones media hardware made from
 *          the same pools - calling it while anything runs hands those addresses
 *          straight back out.
 * @note Call it once at system bring-up, before any media hardware, never from
 *       an allocation path. The guard inside only catches repeat calls from
 *       this module; other code calls the vendor function directly.
 */
VOID_T tkl_media_buf_init(VOID_T);

/**
 * @brief Allocate a media buffer and charge it to an owner.
 *
 * @param[in] use   pool and cache policy, see TKL_BUF_USE_E
 * @param[in] size  byte count; rounded up internally when @p use requires it
 * @param[in] owner accounting bucket, see TKL_MEDIA_BUF_OWNER_E
 *
 * @note TKL_BUF_USE_PIXEL_WT rounds @p size up to 64 bytes: the vendor pool
 *       refuses a write-through mapping whose requested size is not 64-byte
 *       aligned. The write-through channel is owned by the pool and released
 *       automatically when the buffer is freed.
 *
 * @return buffer pointer on success, NULL on error
 */
VOID_T *tkl_media_buf_alloc(TKL_BUF_USE_E use, UINT32_T size, TKL_MEDIA_BUF_OWNER_E owner);

/**
 * @brief Release a buffer obtained from tkl_media_buf_alloc().
 *
 * @param[in] owner  the same owner the buffer was allocated with
 * @param[in] buffer buffer pointer; NULL is ignored
 *
 * @note Safe from any context the vendor pool is safe from, including the GPU
 *       flexa thread and the encoder worker thread.
 */
VOID_T tkl_media_buf_free(TKL_MEDIA_BUF_OWNER_E owner, VOID_T *buffer);

/**
 * @brief Get the release callback to embed in a frame handed to business.
 *
 * @param[in] owner accounting bucket the buffer was allocated with
 *
 * @return a stable function pointer that frees the buffer and settles the
 *         owner's in-flight count; NULL if @p owner is out of range
 */
TKL_MEDIA_BUF_FREE_CB tkl_media_buf_free_cb(TKL_MEDIA_BUF_OWNER_E owner);

/**
 * @brief Number of buffers this owner allocated but has not got back yet.
 *
 * @param[in] owner accounting bucket
 *
 * @return in-flight count, 0 if @p owner is out of range
 */
UINT32_T tkl_media_buf_inflight(TKL_MEDIA_BUF_OWNER_E owner);

/**
 * @brief Wait for an owner's in-flight buffers to come back.
 *
 * @param[in] owner           accounting bucket
 * @param[in] expect_residual how many buffers are legitimately still out when
 *                            the pipeline has stopped; anything at or below
 *                            this is success, not a leak
 * @param[in] timeout_ms      how long to wait; 0 polls once
 *
 * @note Not every owner drains to zero. The panel is still scanning out the
 *       last display frame and nothing will replace it, so a display owner
 *       passes 1 here while an encoded-stream owner passes 0.
 * @note Stop production first, then drain. On timeout this warns and gives up -
 *       it never forces a free, which would turn a leak into a use-after-free.
 * @warning Sleeps. Never call from an ISR, from the GPU flexa thread, or from
 *          the encoder/decoder worker threads.
 *
 * @return OPRT_OK when in-flight has fallen to @p expect_residual or below,
 *         OPRT_TIMEOUT if more than that are still out, OPRT_INVALID_PARM if
 *         @p owner is out of range
 */
OPERATE_RET tkl_media_buf_drain(TKL_MEDIA_BUF_OWNER_E owner,
                                UINT32_T expect_residual,
                                UINT32_T timeout_ms);

/**
 * @brief Log every owner that still holds buffers. Leak-hunting aid.
 */
VOID_T tkl_media_buf_dump(VOID_T);

#ifdef __cplusplus
}
#endif

#endif /* __TKL_MEDIA_BUF_H__ */
