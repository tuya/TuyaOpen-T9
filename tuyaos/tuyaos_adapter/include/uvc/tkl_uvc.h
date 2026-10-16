/**
 * @file tkl_uvc.h
 * @brief Common process - adapter the USB video camera api
 * @version 0.1
 * @date 2026-05-09
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_UVC_H__
#define __TKL_UVC_H__

#include "tuya_cloud_types.h"
#include "tkl_media_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Device plug/unplug notification. Runs on a vendor USB thread. */
typedef VOID (*TKL_UVC_STATE_CB)(BOOL_T connected, VOID_T *arg);

/**
 * What the camera produces. Singleton: one USB camera pipeline.
 *
 * Pull and push do not mix. Pull takes exactly one of MJPEG / FRAME. Push takes
 * either or both of DISPLAY / H264 and must be fed by tkl_uvc_decode_once().
 */
#define TKL_UVC_OUT_NONE     0U
#define TKL_UVC_OUT_MJPEG    (1U << 0)  /**< pull: device bitstream as-is, zero copy */
#define TKL_UVC_OUT_FRAME    (1U << 1)  /**< pull: decoded to NV12 inside get_frame() */
#define TKL_UVC_OUT_DISPLAY  (1U << 2)  /**< push: decoded, scaled and rotated via display_cb */
#define TKL_UVC_OUT_H264     (1U << 3)  /**< push: transcoded to H264 via stream_cb */

/** Which output an OSD or encoder setting applies to. */
typedef enum {
    TKL_UVC_CHN_DISPLAY = 0, /**< the display output, in rotated panel coordinates */
    TKL_UVC_CHN_ENCODE,      /**< the H264 output, in un-rotated decoder coordinates */
} TKL_UVC_CHN_E;

/**
 * @brief Everything the USB camera needs, including what to do with its output.
 *
 * @note Deliberately absent, compared with the legacy TUYA_UVC_CFG_T (now
 *       static inside src/driver/tkl_uvc.c): the stream buffer
 *       size. The vendor driver always asks for CONFIG_UVC_FRAME_SIZE whatever
 *       the negotiated resolution is, so letting the caller pick a smaller one
 *       would only produce an allocator that refuses every frame.
 */
typedef struct {
    UINT8_T  port;                /**< USB hub port; 0 means 1 */
    UINT16_T width;
    UINT16_T height;
    UINT8_T  fps;                 /**< 0 means 30; renegotiated down if unsupported */
    UINT8_T  drop_num;            /**< discard N frames after open so AE can settle */
    UINT32_T src_codec;           /**< TKL_MEDIA_CODEC_E the device sends; 0 means JPEG */
    UINT32_T connect_timeout_ms;  /**< 0 means 4000 */

    /** Which outputs to produce. Exactly one of the pull bits, or either or
     *  both of the push bits - never a mix of the two families. */
    UINT32_T out_kinds;

    /* How deep the adapter's three buffer pools are is not configurable, on
     * purpose. Every one of those numbers comes from a platform fact the caller
     * cannot see - the decoder's 16-line slice granularity, CONFIG_UVC_FRAME_SIZE,
     * how long this chip's jpegd takes on one picture - and a wrong value does
     * not fail, it silently drops frames. Same judgement, and the same reason,
     * as the stream buffer size in the note above. tkl_camera_mipi exposes no
     * depth knob either. See TKL_UVC_OUT_*_DEPTH in src/driver/tkl_uvc.c. */

    /** Applies to TKL_UVC_OUT_DISPLAY. */
    struct {
        UINT16_T rotate;      /**< 0/90/180/270 */
        UINT16_T dst_w;       /**< panel buffer width */
        UINT16_T dst_h;       /**< panel buffer height */
        UINT32_T format;      /**< TKL_PIXEL_FMT_E of the delivered pixels */
    } display;

    /** Applies to TKL_UVC_OUT_H264. */
    struct {
        UINT8_T  fps;         /**< 0 takes the negotiated capture rate */
        UINT32_T gop;
        UINT32_T bitrate;     /**< 0 leaves the encoder in fixed-QP mode */
    } encode;

    /** @warning display_cb and stream_cb run on **different vendor worker
     *           threads** and can therefore overlap. Pointing both at one
     *           function means that function must be reentrant. */
    TKL_MEDIA_BUF_CB display_cb;  /**< called on the GPU worker thread */
    VOID_T          *display_arg;
    TKL_MEDIA_BUF_CB stream_cb;   /**< called on the encoder worker thread */
    VOID_T          *stream_arg;

    /** The device's own bitstream for the picture the display and H264 paths
     *  just consumed. TKL_UVC_OUT_DISPLAY / _H264 only. Called inside
     *  tkl_uvc_decode_once() on that caller's thread, so it is not concurrent
     *  with the two above. **Borrowed**: free_cb is NULL and the slot goes back
     *  when this returns, so copy what you need. */
    TKL_MEDIA_BUF_CB mjpeg_cb;
    VOID_T          *mjpeg_arg;

    TKL_UVC_STATE_CB  state_cb;   /**< plug/unplug; runs on a vendor USB thread */
    VOID_T           *state_arg;
} TKL_UVC_CFG_T;

/* ===========================================================================
 * Lifecycle: init -> start -> stop -> deinit
 *
 * init() carries USB enumeration, which dominates a full bring-up, so pausing
 * with stop()/start() is far cheaper than close+open. Run "xuvc reopen" and
 * "xuvc restart" for the per-phase timings on your own configuration.
 *
 * start() and stop() may be called repeatedly between one init() and one
 * deinit(). Both are idempotent.
 * =========================================================================== */

/**
 * @brief Enumerate the USB camera and build its capture pool. No stream yet.
 *
 * @param[in] cfg device description plus the wanted output
 *
 * @note Blocks until the device enumerates or @ref connect_timeout_ms expires.
 * @note Only one USB camera pipeline may be live at a time.
 * @note Nothing is produced until tkl_uvc_start(); every data-path call returns
 *       OPRT_RESOURCE_NOT_READY until then.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_uvc_init(CONST TKL_UVC_CFG_T *cfg);

/**
 * @brief Build the decoder and consumers, then start the stream.
 *
 * @note Idempotent: OPRT_OK if already started. OPRT_RESOURCE_NOT_READY if
 *       tkl_uvc_init() has not run.
 * @note On failure the pipeline stays initialised, not torn down - the device
 *       remains enumerated so a retry costs 177 ms, not 716 ms.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_uvc_start(VOID_T);

/**
 * @brief Stop the stream and tear the consumers down, keeping the device.
 *
 * @note Idempotent: OPRT_OK if already stopped.
 * @note Stop your own consumer thread first. This turns away new
 *       tkl_uvc_get_frame() / tkl_uvc_decode_once() calls, but it cannot help
 *       one already parked inside either.
 * @note Your pause latency includes joining that thread, and the join is
 *       bounded by the timeout you passed to decode_once() / get_frame().
 * @warning A buffer business still holds is **not** waited for here. Holding one
 *          across a stop is legal - the pool is one slot short until it comes
 *          back. tkl_uvc_deinit() waits.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_uvc_stop(VOID_T);

/**
 * @brief Stop if needed, then release the pools and the device.
 *
 * @note Waits briefly for frames handed to business to come back. A buffer
 *       still held when that expires leaves the whole pool allocated rather
 *       than freeing memory business still points at; the leak is logged.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_uvc_deinit(VOID_T);

/**
 * @brief Take the next frame, blocking until one arrives.
 *
 * @param[out] out        receives the frame and its release callback
 * @param[in]  timeout_ms how long to wait for one
 *
 * @note Single consumer. With TKL_UVC_OUT_FRAME the JPEG is decoded here, on
 *       **the calling thread** - TKL owns no threads.
 * @note Release with @c out->free_cb(out->data) when done; that returns the
 *       buffer to its pool rather than freeing it.
 *
 * @return OPRT_OK on success, OPRT_TIMEOUT if no frame arrived in time.
 *         Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_uvc_get_frame(TKL_MEDIA_BUF_T *out, UINT32_T timeout_ms);

/**
 * @brief Decode one captured frame, for TKL_UVC_OUT_DISPLAY / _H264.
 *
 * @param[in] timeout_ms how long to wait for a captured frame to decode
 *
 * @note Call it in a loop from a thread of your own - TKL owns none:
 *       @code while (running) { tkl_uvc_decode_once(100); } @endcode
 *       Nothing comes back; results arrive on display_cb / stream_cb.
 * @note Single caller. Blocks for the whole decode.
 * @note Set @ref TKL_UVC_CFG_T::mjpeg_cb to also see the device's own MJPEG for
 *       each decoded picture.
 *
 * @return OPRT_OK when a frame was decoded, OPRT_TIMEOUT when none arrived in
 *         time. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_uvc_decode_once(UINT32_T timeout_ms);

/**
 * @brief Adjust the encoder while it runs.
 *
 * @param[in] ctrl bitrate and/or IDR request
 *
 * @return OPRT_OK on success, OPRT_NOT_SUPPORTED without an H264 output.
 *         Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_uvc_set_encode(CONST TKL_MEDIA_ENC_CTRL_T *ctrl);

/**
 * @brief Set one overlay slot on one output.
 *
 * @param[in] chn which output
 * @param[in] osd slot contents; count == 0 or elems == NULL clears the slot
 *
 * @note The two channels do not share a coordinate system - see TKL_OSD_T.
 *       This call fills in @c osd->rotate for the display channel; the encode
 *       overlay cannot rotate at all.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_uvc_set_osd(TKL_UVC_CHN_E chn, CONST TKL_OSD_T *osd);

/**
 * @brief Report what one output's overlay hardware can do.
 *
 * @param[in]  chn  which output
 * @param[out] caps receives the capabilities
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_uvc_get_osd_caps(TKL_UVC_CHN_E chn, TKL_OSD_CAPS_T *caps);


#ifdef __cplusplus
}
#endif

#endif
