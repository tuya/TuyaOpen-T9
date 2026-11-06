/**
 * @file tkl_dvp.h
 * @brief Tuya DVP driver adaptation layer (TKL) interface -- shared across platforms
 *
 * This interface carries DVP for the unified camera abstraction: the input frame
 * config (TUYA_CAMERA_FRAME_CFG_T) and the output channel array
 * (outputs/output_num) share one shape across the three input sources
 * MIPI CSI / UVC / DVP; the DVP-specific part (sync_polarity) is carried by this
 * file alone, while UVC input is carried by tkl_uvc through an identically
 * shaped interface.
 * Common definitions (frame channels / events / bitrate levels / media buffers /
 * input frame config / output node config) live in tuya_cloud_types.h
 * (TUYA_CAMERA_* / TUYA_MEDIA_* / TUYA_FRAME_*).
 *
 */

#ifndef __TKL_DVP_H__
#define __TKL_DVP_H__

#include "tuya_cloud_types.h"



#ifdef __cplusplus
extern "C" {
#endif

/* flow map

    dvp sensor ------ yuv buffer
                        |------ rotate/scale ---------- ch1
                        |           |------ display --- ch2
                        |
                        |
                        |------ h264 ------------------ ch3
    channel 1: yuv frame
    channel 2: rgb frame
    channel 3: h264 stream
    Channel 1's yuv frame is process data and is read through the get interface;
    channels 2/3 are data streams and are handed to the upper layer via callback.

    Channels: this interface defines ch1 / ch2 / ch3 only. ch0 (no parking-pool
    semantics on DVP) and 4..9 are rejected as illegal channels by init(), as is
    any repeated channel.
*/

/*===========================================================================
* DVP-specific input config: hsync / vsync polarity
*==========================================================================*/

/**
 * @brief hsync / vsync polarity combinations.
 *
 * The value must match the sensor's actual output timing; a mismatch usually
 * shows up as no frame or a torn/garbled image rather than an error code.
 */
typedef enum {
    TUYA_DVP_SYNC_MODE_0 = 0,   /* hsync high valid, vsync high valid */
    TUYA_DVP_SYNC_MODE_1,       /* hsync high valid, vsync low  valid */
    TUYA_DVP_SYNC_MODE_2,       /* hsync low  valid, vsync high valid */
    TUYA_DVP_SYNC_MODE_3,       /* hsync low  valid, vsync low  valid */
} TUYA_DVP_SYNC_MODE_E;

/*===========================================================================
* TKL config
*==========================================================================*/

typedef struct {
    /* input */
    TUYA_CAMERA_FRAME_CFG_T dvp_cfg;          /* input frame config (unified shape: w/h/fps/format) */
    TUYA_DVP_SYNC_MODE_E     sync_polarity;  /* hsync/vsync polarity (DVP-specific).The caller must fill it according to the sensor configuration */

    /* output: output node array (provided by the APP; TKL deep-copies the array
    * itself; a node present in the array counts as enabled, and at least one
    * valid node is required; node structure: see tuya_cloud_types.h
    * TUYA_MEDIA_OUTPUT_CFG_T).
    *
    * How frames are fetched is **decided by the channel's role, not by whether
    * cb is set**:
    *   - CH1 (yuv process data) -- **pull mode**, read via tkl_dvp_get_frame;
    *   - CH2 (rgb) / CH3 (h264) -- **output streams**, pushed via cb.
    * gop/level take effect on encode channels only; rotate on pixel-conversion
    * channels only. */
    UINT8_T                 output_num;
    TUYA_MEDIA_OUTPUT_CFG_T *outputs;

    /* event: event notification. The callback signature is
    * VOID(TUYA_CAMERA_EVENT_E, VOID_T *), with **no failure return value**.
    * event_arg is passed **only** to event_cb and cannot stand in for each
    * output node's arg: when a pushed frame is delivered,
    * `buf->user_args == that output node's outputs[i].arg` (the registered
    * argument passed through unchanged), which has nothing to do with
    * event_arg. */
    TUYA_CAMERA_EVENT_CB    event_cb;
    VOID_T                 *event_arg;
} TUYA_DVP_BASE_CFG_T;

/*===========================================================================
* Lifecycle API
*===========================================================================*/

/**
 * @brief Initialize the DVP input and every requested output channel.
 *
 * No output is produced until tkl_dvp_start() is called.
 *
 * @param[in] cfg DVP input config and the output channel array; must not be
 *                NULL
 *
 * @return OPRT_OK on success;
 *         OPRT_INVALID_PARM      invalid parameter (unsupported input format /
 *                                output_num==0 or outputs==NULL /
 *                                illegal or duplicate channel (ch1/ch2/ch3 are
 *                                the only defined channels; ch0 and 4..9 are
 *                                rejected) /
 *                                sync_polarity out of 0..3 /
 *                                output-stream channel with mismatched fields /
 *                                a field the channel does not support set to
 *                                non-zero);
 *         OPRT_NOT_SUPPORTED     platform capability unsupported (e.g. rotate
 *                                angle / channel count) / conflicts with another
 *                                camera path (e.g. UVC);
 *         OPRT_INIT_MORE_THAN_ONCE repeated init without deinit;
 *         OPRT_COM_ERROR         lower-layer failure (sensor probe / memory /
 *                                driver start)
 * @note Unsupported channel / format / rotate / field combinations **must be
 *       rejected** (OPRT_INVALID_PARM) and **must not be silently ignored**.
 * @note A failure can be retried directly. Runtime errors have no callback.
 * @note **Deep-copy boundary**: only cfg and the outputs array itself are
 *       deep-copied; the business objects pointed to by outputs[i].cb/arg and by
 *       event_arg **belong to the APP -- TKL only stores and uses the pointers,
 *       copying and freeing nothing**. Those objects must stay valid until
 *       deinit returns successfully.
 * @note **Event reporting scope**: this interface **reports lifecycle events
 *       only** (INITIALIZED / STARTED / STOPPED / DEINITIALIZED). The public
 *       header's CONNECT / DISCONNECT (UVC-only) and all ERR_* (OVERFLOW /
 *       TIMEOUT / GPU / VENC / VDEC / SENSOR / CSI / USB / ALLOC / BAD_FORMAT)
 *       are **not reported for now** -- the mere existence of those enumerators
 *       in the public header does not require this implementation to support
 *       them.
 * @note After a successful init, TUYA_CAMERA_EVENT_INIT is reported via
 *       event_cb (asynchronous notification).
 */
OPERATE_RET tkl_dvp_init(CONST TUYA_DVP_BASE_CFG_T *cfg);

/**
 * @brief Start every enabled output channel.
 *
 * @pre inited==TRUE
 * @return OPRT_OK / OPRT_COM_ERROR (not inited)
 * @note The APP must not assume continuity of the first frame's timestamp after
 *       start.
 * @note After start completes, TUYA_CAMERA_EVENT_START is reported via
 *       event_cb (asynchronous notification).
 * @note **Lifecycle event ordering**: for one instance, lifecycle events are
 *       notified **serially, in the order the state transitions actually
 *       succeeded -- never in parallel and never out of order**;
 *       TUYA_CAMERA_EVENT_DEINIT is that instance's last lifecycle
 *       notification. So on a fast stop->start, the old STOPPED is never
 *       reported later than the new STARTED.
 */
OPERATE_RET tkl_dvp_start(VOID_T);

/**
 * @brief Stop streaming while keeping the initialized resources.
 *
 * Idempotent: returns OPRT_OK when not started.
 *
 * @pre inited==TRUE
 * @return OPRT_OK / OPRT_COM_ERROR (not inited)
 * @note stop does not release already-allocated buffers; they remain usable
 *       after start.
 * @note After stop completes, TUYA_CAMERA_EVENT_STOP is reported via
 *       event_cb (asynchronous notification; it is a lifecycle event, not one of
 *       the forbidden new media-frame deliveries).
 */
OPERATE_RET tkl_dvp_stop(VOID_T);

/**
 * @brief Stop and release all camera resources.
 *
 * @return OPRT_OK / OPRT_COM_ERROR (not inited)
 * @note Memory usage must stay level across repeated init->deinit cycles (no
 *       leak over long runs).
 * @note **Callback context lifetime**: the business objects pointed to by
 *       outputs[i].arg and event_arg are **owned by the APP**; TKL only stores
 *       and uses the pointers and **does not copy or free** those objects. They
 *       must stay valid until **deinit returns successfully**; by then TKL has
 *       stopped using them and the APP may free them itself. **TKL only cleans
 *       up its own config copies and registration references** (including the
 *       outputs array copy) and never touches the APP's business objects.
 * @note After deinit returns, no further event callbacks are made for this
 *       registration; DEINITIALIZED is that instance's **last** lifecycle
 *       notification.
 */
OPERATE_RET tkl_dvp_deinit(VOID_T);

/*===========================================================================
* Raw frame pull (process-data channels, e.g. CH1 YUV)
*===========================================================================*/

/**
 * @brief Pull one frame from the given process-data channel (e.g. CH1).
 *
 * TKL takes the frame from the channel cache and returns, through the out
 * parameter, the **original address** of the frame descriptor (TUYA_MEDIA_BUF_T)
 * together with the right to use its internal data buffer (zero-copy).
 *
 * @param[in]  channel    channel number (e.g. TUYA_CAMERA_FRAME_CHANNEL_1;
 *                        must be a process-data channel)
 * @param[out] out        frame descriptor output (**the original address of
 *                        TKL's internal descriptor**; metadata carries
 *                        buffer/length/format/channel/timestamp/width/height and
 *                        a valid free_cb; user_args is always NULL -- the pull
 *                        path has no callback delivery, unlike an output
 *                        stream's outputs[i].arg)
 * @param[in]  timeout_ms wait budget in ms; 0 = return immediately when no frame
 *
 * @return OPRT_OK            a frame was obtained (*out is valid)
 *         OPRT_TIMEOUT       timed out with no frame (always this value once
 *                            stopped)
 *         OPRT_INVALID_PARM  out==NULL / illegal channel
 *         OPRT_COM_ERROR     not inited / that channel has no process-data output
 *         OPRT_NOT_SUPPORTED that channel is an output-stream channel (not a
 *                            process-data channel)
 * @note Serial calls on the same channel are safe (concurrent reads of the same
 *       channel from multiple threads are the APP's responsibility to avoid).
 * @note **A delivered frame is never reclaimed**: a descriptor and buffer the
 *       APP has already obtained are **never** reclaimed, reused or overwritten
 *       by a timeout or eviction mechanism; when resources run short the
 *       behaviour is to refuse the fetch / apply backpressure, never to reclaim
 *       a delivered frame.
 * @note Field semantics: timestamp is **milliseconds since boot** (UINT64_T);
 *       priv is TKL-private and the APP **must not** modify it.
 * @note **const boundary**: this interface's output parameter is non-const; the
 *       output-stream callback (TUYA_MEDIA_BUF_CB) receives
 *       CONST TUYA_MEDIA_BUF_T *, and that const **only** restricts access to
 *       the descriptor -- it neither makes the bytes the buffer points to
 *       read-only nor grants permission to free a copy.
 */
OPERATE_RET tkl_dvp_get_frame(TUYA_CAMERA_FRAME_CHANNEL_E channel,
                            TUYA_MEDIA_BUF_T **out, UINT32_T timeout_ms);

/*===========================================================================
* DVP-specific API
*===========================================================================*/

/**
 * @brief Request the next H264 frame to be an IDR (I frame), for streaming
 *        random access / recovery.
 *
 * @pre inited and an H264 encode output channel (CH3) exists; accepted whether
 *      started or not (takes effect queued), otherwise returns
 *      OPRT_NOT_SUPPORTED
 * @return OPRT_OK / OPRT_COM_ERROR / OPRT_NOT_SUPPORTED
 * @note **Effect check**: the next delivered H264 frame's `format` is
 *       TUYA_FRAME_FMT_H264_I. That value **only says "I frame type"; it does
 *       not by itself prove the bitstream is an IDR**.
 * @note Currently applies to the single enabled H264 channel.
 */
OPERATE_RET tkl_dvp_request_idr(VOID_T);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __TKL_DVP_H__ */
