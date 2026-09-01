/**
 * @file tkl_camera_mipi.h
 * @brief Common process - adapter the MIPI camera api
 * @version 0.1
 * @date 2026-04-10
 *
 * @copyright Copyright 2021-2022 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TKL_CAMERA_MIPI_H__
#define __TKL_CAMERA_MIPI_H__

#include "tuya_cloud_types.h"
#include "tkl_media_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TUYA_CAMERA_MIPI_CHANNEL_MP 0U
#define TUYA_CAMERA_MIPI_CHANNEL_SP 1U

/** What a channel produces. Bitwise-OR them; a channel may feed several sinks. */
#define TKL_CAM_OUT_NONE     0U
#define TKL_CAM_OUT_FRAME    (1U << 0)  /**< pull with tkl_camera_mipi_get_frame() */
#define TKL_CAM_OUT_DISPLAY  (1U << 1)  /**< scaled/rotated pixels via display_cb; **MP only** */
#define TKL_CAM_OUT_H264     (1U << 2)  /**< H264 bitstream via stream_cb; **MP only** */

/** Values for TKL_CAMERA_MIPI_CFG_T.display.osd_rotate. */
#define TKL_CAM_OSD_ROTATE_FOLLOW  0U    /**< same angle as display.rotate; the default */
#define TKL_CAM_OSD_ROTATE_ZERO    360U  /**< no rotation, whatever the picture does */

/** Which output an OSD or encoder setting applies to. */
typedef enum {
    TKL_CAM_CHN_DISPLAY = 0, /**< the display output, in rotated panel coordinates */
    TKL_CAM_CHN_ENCODE,      /**< the H264 output, in un-rotated sensor coordinates */
} TKL_CAM_CHN_E;

/** @brief Everything the camera needs, including what to do with its output. */
typedef struct {
    INT_T    pin_scl;
    INT_T    pin_sda;
    INT_T    pin_reset;
    INT_T    pin_pwdn;
    INT_T    pin_xclk;
    UINT8_T  i2c_id;

    UINT16_T sensor_width;
    UINT16_T sensor_height;
    UINT8_T  sensor_fps;
    UINT8_T  hmirror;
    UINT8_T  vflip;
    UINT8_T  skip_frames;     /**< drop N frames after open so AE can settle */

    /** Raw Bayer phase the sensor delivers, a TKL_PIXEL_FMT_*10 value. Board
     *  wiring, exactly like the pins above, and just as unguessable: 0 leaves it
     *  to the driver, whose default is RGGB10, and feeding that to a BGGR sensor
     *  produces a picture with red and blue exchanged. */
    UINT32_T sensor_input_format;

    /** TKL_CAM_OUT_* bitmask for the main channel. Any combination, including
     *  DISPLAY and H264 together - MP is the only channel a flexa bond attaches
     *  to. */
    UINT32_T mp_out_kinds;
    UINT16_T mp_width;
    UINT16_T mp_height;

    /** TKL_CAM_OUT_NONE disables SP. **Only TKL_CAM_OUT_FRAME is accepted**:
     *  every ISP bond hardcodes the MP channel, so DISPLAY or H264 here returns
     *  OPRT_NOT_SUPPORTED. */
    UINT32_T sp_out_kinds;
    UINT16_T sp_width;
    UINT16_T sp_height;
    /** TKL_PIXEL_FMT_E the SP channel delivers; 0 means NV12. NV12 to
     *  JPEG-encode or transform the frame, RGB888 for the face model. */
    UINT32_T sp_format;

    /** Applies to whichever channel carries TKL_CAM_OUT_DISPLAY. */
    struct {
        UINT16_T rotate;      /**< 0/90/180/270 */
        UINT16_T dst_w;       /**< panel buffer width */
        UINT16_T dst_h;       /**< panel buffer height */
        UINT32_T format;      /**< TKL_PIXEL_FMT_E of the delivered pixels */
        /** Rotation of the display overlay, independent of @c rotate. FOLLOW
         *  keeps text upright on the panel; set it otherwise to pin the overlay
         *  to the frame. Only 0, 90 and 270 reach the hardware, and the encode
         *  channel ignores it. */
        UINT16_T osd_rotate;
    } display;

    /** Applies to whichever channel carries TKL_CAM_OUT_H264. */
    struct {
        UINT8_T  fps;
        UINT32_T gop;
        UINT32_T bitrate;
    } encode;

    TKL_MEDIA_BUF_CB display_cb;  /**< called on the GPU worker thread */
    VOID_T          *display_arg;
    TKL_MEDIA_BUF_CB stream_cb;   /**< called on the encoder worker thread */
    VOID_T          *stream_arg;
} TKL_CAMERA_MIPI_CFG_T;

/* ===========================================================================
 * Lifecycle: init -> start -> stop -> deinit
 *
 * start() and stop() may be called repeatedly between one init() and one
 * deinit(). Both are idempotent. Run "xmipi cam reopen" and "xmipi cam restart"
 * for the per-phase timings on your own configuration.
 * =========================================================================== */

/**
 * @brief Power and probe the sensor. No stream yet.
 *
 * @param[in] cfg sensor description plus the wanted outputs
 *
 * @note Nothing is produced until tkl_camera_mipi_start(). Until then
 *       tkl_camera_mipi_get_frame(), _set_output(), _set_encode() and
 *       _set_osd() all return OPRT_RESOURCE_NOT_READY.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_camera_mipi_init(CONST TKL_CAMERA_MIPI_CFG_T *cfg);

/**
 * @brief Start the ISP and build everything its output feeds.
 *
 * @warning @ref display_cb and @ref stream_cb run on **different vendor worker
 *          threads** and can therefore overlap. Pointing both at one function
 *          means that function must be reentrant.
 * @note With TKL_CAM_OUT_DISPLAY selected, only one camera or UVC display
 *       pipeline may be live at a time - the GPU's bring-up touches global
 *       state that is not reference counted.
 * @note Idempotent: OPRT_OK if already started. OPRT_RESOURCE_NOT_READY if
 *       tkl_camera_mipi_init() has not run.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_camera_mipi_start(VOID_T);

/**
 * @brief Tear down every output and stop the ISP, keeping the sensor powered.
 *
 * @note Idempotent: OPRT_OK if already stopped.
 * @note Stop your own puller thread first. This turns away new calls but cannot
 *       help one already parked inside tkl_camera_mipi_get_frame().
 * @warning A frame business still holds is **not** waited for here. Holding one
 *          across a stop is legal; tkl_camera_mipi_deinit() waits.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_camera_mipi_stop(VOID_T);

/**
 * @brief Stop if needed, then power the sensor down and release everything.
 *
 * @note Waits briefly for frames handed to business to come back; a buffer
 *       still held when that expires is logged rather than force-released.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_camera_mipi_deinit(VOID_T);

/**
 * @brief Change a channel's resolution and rebuild whatever it feeds.
 *
 * @param[in] channel_id TUYA_CAMERA_MIPI_CHANNEL_MP or _SP
 * @param[in] width      new output width
 * @param[in] height     new output height
 *
 * @note The consumers are torn down and rebuilt around the switch: with only
 *       the main channel open, changing its resolution restarts the ISP and
 *       **invalidates the handle every bond was built on**.
 * @note Frames in flight across the switch are dropped.
 * @note On failure the previous geometry is restored and the consumers rebuilt,
 *       so a rejected switch leaves a running pipeline rather than a dead one.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_camera_mipi_set_output(UINT8_T channel_id, UINT16_T width, UINT16_T height);

/**
 * @brief Adjust the encoder while it runs.
 *
 * @param[in] ctrl bitrate and/or IDR request
 *
 * @return OPRT_OK on success, OPRT_NOT_SUPPORTED without an H264 output.
 *         Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_camera_mipi_set_encode(CONST TKL_MEDIA_ENC_CTRL_T *ctrl);

/**
 * @brief Set one overlay slot on one output.
 *
 * @param[in] chn which output
 * @param[in] osd slot contents; count == 0 or elems == NULL clears the slot
 *
 * @note The two channels do not share a coordinate system - see TKL_OSD_T.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_camera_mipi_set_osd(TKL_CAM_CHN_E chn, CONST TKL_OSD_T *osd);

/**
 * @brief Report what one output's overlay hardware can do.
 *
 * @param[in]  chn  which output
 * @param[out] caps receives the capabilities
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_camera_mipi_get_osd_caps(TKL_CAM_CHN_E chn, TKL_OSD_CAPS_T *caps);

/**
 * @brief Pull one frame. The buffer comes from the camera and is lent, not given.
 *
 * @param[out] out        receives the frame; release with @c out->free_cb(out->data)
 * @param[in]  chn        TUYA_CAMERA_MIPI_CHANNEL_MP or _SP
 * @param[in]  timeout_ms how long to wait, 0 for the configured read timeout
 *
 * @note Frame mode only. A channel carrying display or H264 runs flexa and has
 *       no frame queue to read, returning OPRT_NOT_SUPPORTED; SP is the pull
 *       channel in that setup.
 * @note Hold it briefly - the driver's queue is only 2 frames deep behind it.
 *
 * @return OPRT_OK on success. Others on error, please refer to tuya_error_code.h
 */
OPERATE_RET tkl_camera_mipi_get_frame(TKL_MEDIA_BUF_T *out, UINT8_T chn, UINT32_T timeout_ms);

/* ===========================================================================
 * Sensor / ISP image quality
 *
 * Apply to the sensor and the ISP as a whole, so they take no channel argument.
 * All of them except set_mirror_flip require the camera to be running.
 * =========================================================================== */

/**
 * @brief ISP CPROC manual attributes (color processing)
 */
typedef struct {
    UINT8_T enable;       /* 0=disable, 1=enable */
    UINT8_T op_type;      /* 0=auto, 1=manual */
    INT8_T brightness;    /* [-127, 127] */
    UINT8_T contrast;     /* [0, 255] */
    UINT8_T saturation;   /* [0, 255] */
    INT8_T hue;           /* [-90, 90] */
} TKL_CAMERA_MIPI_CPROC_T;

/**
 * @brief Set sensor horizontal mirror and vertical flip
 * @param[in] hmirror 0=off, non-zero=on
 * @param[in] vflip 0=off, non-zero=on
 * @return OPRT_OK on success
 * @note If camera has started, applies immediately via sensor registers.
 *       If not started yet, values are saved and applied on next start.
 */
OPERATE_RET tkl_camera_mipi_set_mirror_flip(UINT8_T hmirror, UINT8_T vflip);
/**
 * @brief Get current mirror/flip config
 * @param[out] hmirror current hmirror (0/1), may be NULL
 * @param[out] vflip current vflip (0/1), may be NULL
 * @return OPRT_OK on success
 */
OPERATE_RET tkl_camera_mipi_get_mirror_flip(UINT8_T *hmirror, UINT8_T *vflip);
/**
 * @brief Get ISP exposure-normalized luminance
 * @param[out] luminance (meanLum * 1e6) / (exposure[0] * 39)
 * @return OPRT_OK on success
 * @note Requires camera started. Wraps BK_CAM_IOCTL_GET_EXPOSURE_LUMINANCE.
 */
OPERATE_RET tkl_camera_mipi_get_exposure_luminance(UINT32_T *luminance);
/**
 * @brief Switch ISP output between gray and color via CPROC saturation
 * @param[in] enable TRUE=gray (saturation 0, manual), FALSE=restore sensor default CPROC
 * @return OPRT_OK on success
 * @note Requires camera started. Gray keeps brightness/contrast/hue of the current CPROC.
 */
OPERATE_RET tkl_camera_mipi_set_color_to_gray(BOOL_T enable);
/**
 * @brief Switch ISP gamma-out between night curve and backed-up day curve
 * @param[in] enable TRUE=apply night gamma-out (backup current first), FALSE=restore backup
 * @return OPRT_OK on success
 * @note Requires camera started. Night curve fields are left for product fill-in.
 */
OPERATE_RET tkl_camera_mipi_set_night_gamma(BOOL_T enable);
/**
 * @brief Get current ISP CPROC attributes
 * @param[out] cproc CPROC snapshot (manual fields only)
 * @return OPRT_OK on success
 * @note Requires camera started. Wraps BK_CAM_IOCTL_GET_CPROC.
 */
OPERATE_RET tkl_camera_mipi_get_cproc(TKL_CAMERA_MIPI_CPROC_T *cproc);

#ifdef __cplusplus
}
#endif

#endif
