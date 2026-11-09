/**
 * @file tkl_audio_frontend.c
 * @brief T9 audio front-end: 3-lane capture -> BF -> AEC/NS -> 20 ms mono
 * @version 1.0
 * @date 2026-11-06
 * @copyright Copyright (c) 2025-2026 Tuya Inc. All Rights Reserved.
 *
 * Data flow per 8 ms capture frame (768 bytes, lanes interleaved per sample):
 *   deinterleave -> [DAS BF + AEC + NS, 128 samples] -> accumulate 320 ->
 *   emit 640 bytes
 * The AEC input is 2 mic + 1 ref (mic0, mic1, ref0). The library's BF
 * pre-stage steers the two mics with delay-and-sum and feeds the mono beam to
 * the AEC; its fixed output delay is compensated on the reference here, since
 * the library does not delay the reference itself.
 * The emitted 20 ms block is what the audio task hands to put_cb, and it is
 * also what VAD/KWS consume (they take exactly 320 int16 samples per call).
 */

#include "tkl_audio_frontend.h"

#include <string.h>

#include "tkl_output.h"
#include "tkl_memory.h"

#if defined(TKL_AF_WITH_VAD) && (TKL_AF_WITH_VAD == 1)
#include "tkl_vad.h"
extern OPERATE_RET tkl_kws_feed_with_vad(uint8_t *data, uint16_t datalen, uint8_t vadflag);
#elif defined(TKL_AF_WITH_KWS) && (TKL_AF_WITH_KWS == 1)
extern OPERATE_RET tkl_kws_feed_with_vad(uint8_t *data, uint16_t datalen, uint8_t vadflag);
#endif

#if defined(TKL_AF_WITH_TAL_LIB) && (TKL_AF_WITH_TAL_LIB == 1)
#include "pipeline/audio_subsys_fdaf_aecns.h"
#endif

/***********************************************************
************************macro define************************
***********************************************************/
/* Position of each signal inside the captured 3-lane frame, by capture order.
 * This mirrors the ADC lane order tkl_aud_capture delivers, which is board
 * wiring: verify on hardware before trusting a default. MIC0/MIC1 also fix the
 * order the beamformer sees the array in, so swap them if the steering angle
 * behaves mirrored. */
/* Fallbacks for a build that bypasses the Kconfig-generated -D switches; they
 * must match the boards/T9/TKL_Kconfig defaults, or the lanes swap silently. */
#ifndef TKL_AF_MIC0_LANE
#define TKL_AF_MIC0_LANE (0U)
#endif
#ifndef TKL_AF_MIC1_LANE
#define TKL_AF_MIC1_LANE (1U)
#endif
#ifndef TKL_AF_REF_LANE
#define TKL_AF_REF_LANE (2U)
#endif

/* Beamformer geometry. The DAS pre-stage assumes an equispaced linear array
 * with mic0 at index 0; 90 deg is endfire (talker on the mic0-to-mic1 axis)
 * and 0 deg is broadside. Both values are board geometry, not something the
 * algorithm can infer: verify them on the target. */
#ifndef TKL_AF_BF_AZIMUTH_DEG
#define TKL_AF_BF_AZIMUTH_DEG (90)
#endif
#ifndef TKL_AF_BF_MIC_SPACING_MM
#define TKL_AF_BF_MIC_SPACING_MM (30)
#endif
#define TKL_AF_BF_MIC_SPACING_M ((float)TKL_AF_BF_MIC_SPACING_MM / 1000.0f)

/* The AEC entry takes all mics first, then the reference, at each sample:
 *   mic[0], mic[1], ..., ref[0], ...
 * The BF combines both mics; ref[0] is the only reference used. */
#define TKL_AF_AEC_MIC_CHANNELS (2U)
#define TKL_AF_AEC_REF_CHANNELS (1U)
#define TKL_AF_AEC_CHANNELS     (TKL_AF_AEC_MIC_CHANNELS + TKL_AF_AEC_REF_CHANNELS)          /* 3 */
#define TKL_AF_AEC_IN_BYTES     (TKL_AF_CAPTURE_SAMPLES * TKL_AF_AEC_CHANNELS * sizeof(int16_t)) /* 768 */
#define TKL_AF_AEC_OUT_BYTES    (TKL_AF_CAPTURE_SAMPLES * sizeof(int16_t))                   /* 256 */

/* The beamformer's output delay (GET_INFO.bf_latency_samples) is compensated
 * by delaying the capture reference by the same number of samples. The DAS
 * delay line bounds it well below this cap (base + largest channel delay). */
#define TKL_AF_REF_DELAY_MAX (128U)

/* The accumulator holds one 8 ms frame beyond the 20 ms block, because 128
 * and 320 only align every 64 samples. */
#define TKL_AF_ACC_SAMPLES (TKL_AF_OUT_SAMPLES + TKL_AF_CAPTURE_SAMPLES)

/***********************************************************
***********************variable define**********************
***********************************************************/
static int16_t  s_aec_in[TKL_AF_CAPTURE_SAMPLES * TKL_AF_AEC_CHANNELS];
static int16_t  s_mono[TKL_AF_CAPTURE_SAMPLES];
static int16_t  s_acc[TKL_AF_ACC_SAMPLES];
static uint32_t s_acc_n    = 0;
static BOOL_T   s_af_ready = FALSE;
static BOOL_T   s_aec_on   = FALSE;

/* Capture-reference delay line keeping the AEC reference aligned with the
 * beamformer output. s_ref_delay == 0 means no BF compensation. The line holds
 * s_ref_delay + 1 samples: the write slot is reused only after a full turn,
 * so reading the slot just advanced past returns the value written exactly
 * s_ref_delay samples ago. */
static int16_t  s_ref_hist[TKL_AF_REF_DELAY_MAX + 1U];
static uint32_t s_ref_w     = 0;
static uint32_t s_ref_delay = 0;

#if defined(TKL_AF_WITH_TAL_LIB) && (TKL_AF_WITH_TAL_LIB == 1)
static audio_subsys_module_t s_aec_module;
static void                 *s_aec_self = NULL;
#endif

/***********************************************************
***********************function define**********************
***********************************************************/
#if defined(TKL_AF_WITH_TAL_LIB) && (TKL_AF_WITH_TAL_LIB == 1)
static OPERATE_RET __tkl_af_aec_create(void)
{
    audio_subsys_fdaf_aecns_config_t cfg;
    audio_subsys_aecns_info_t        info;

    audio_subsys_fdaf_aecns_config_default(&cfg);
    cfg.framelen         = TKL_AF_CAPTURE_SAMPLES;
    cfg.fs               = TKL_AF_SAMPLE_RATE;
    cfg.mic_num          = TKL_AF_AEC_MIC_CHANNELS;
    cfg.ref_num          = TKL_AF_AEC_REF_CHANNELS;
    cfg.bf_enable        = 1;
    cfg.bf_azimuth_deg   = (float)TKL_AF_BF_AZIMUTH_DEG;
    cfg.bf_mic_spacing_m = TKL_AF_BF_MIC_SPACING_M;

    s_aec_module.self         = NULL;
    s_aec_module.name         = "fdaf_aecns";
    s_aec_module.create       = audio_subsys_fdaf_aecns_create;
    s_aec_module.destroy      = audio_subsys_fdaf_aecns_destroy;
    s_aec_module.set_callback = audio_subsys_fdaf_aecns_set_callback;
    s_aec_module.ioctl        = audio_subsys_fdaf_aecns_ioctl;
    s_aec_module.process      = audio_subsys_fdaf_aecns_process;

    if (s_aec_module.create(&s_aec_self, &cfg) != 0) {
        s_aec_self = NULL;
        return OPRT_COM_ERROR;
    }

    s_ref_delay = 0;
    s_ref_w     = 0;
    memset(&info, 0, sizeof(info));
    if (s_aec_module.ioctl(s_aec_self, AUDIO_SUBSYS_AECNS_GET_INFO, &info) == 0 &&
        info.bf_latency_samples > 0 && (uint32_t)info.bf_latency_samples <= TKL_AF_REF_DELAY_MAX) {
        s_ref_delay = (uint32_t)info.bf_latency_samples;
    } else {
        tkl_log_output("tkl_af: BF latency unavailable, reference not aligned\r\n");
    }

    tkl_log_output("tkl_af: BF %d ch, azimuth %d deg, spacing %d mm, ref delay %u\r\n",
                   (int)TKL_AF_AEC_MIC_CHANNELS, (int)TKL_AF_BF_AZIMUTH_DEG,
                   (int)TKL_AF_BF_MIC_SPACING_MM, (unsigned)s_ref_delay);

    return OPRT_OK;
}
#endif

OPERATE_RET tkl_af_init(void)
{
    s_acc_n     = 0;
    s_ref_w     = 0;
    s_ref_delay = 0;

    if (s_af_ready) {
        return OPRT_OK;
    }

#if defined(TKL_AF_WITH_TAL_LIB) && (TKL_AF_WITH_TAL_LIB == 1)
    if (__tkl_af_aec_create() == OPRT_OK) {
        s_aec_on = TRUE;
    } else {
        s_aec_on = FALSE;
        tkl_log_output("tkl_af: AEC+NS create failed, running passthrough\r\n");
    }
#endif

    s_af_ready = TRUE;

    return OPRT_OK;
}

OPERATE_RET tkl_af_deinit(void)
{
#if defined(TKL_AF_WITH_TAL_LIB) && (TKL_AF_WITH_TAL_LIB == 1)
    if (s_aec_self) {
        s_aec_module.destroy(s_aec_self);
        s_aec_self = NULL;
    }
#endif

    s_aec_on    = FALSE;
    s_af_ready  = FALSE;
    s_acc_n     = 0;
    s_ref_w     = 0;
    s_ref_delay = 0;

    return OPRT_OK;
}

BOOL_T tkl_af_aec_enabled(void)
{
    return s_aec_on;
}

/**
 * @brief Split one interleaved 3-lane frame into the AEC input layout
 * @note The AEC stage takes all mics first, then the reference, per sample:
 *       mic[0], mic[1], ref[0]. The beamformer consumes both mics and the AEC
 *       consumes its mono output; the reference is delayed by the beamformer
 *       latency so the two paths stay time-aligned.
 */
static void __tkl_af_deinterleave(const int16_t *in)
{
    uint32_t i;

    for (i = 0; i < TKL_AF_CAPTURE_SAMPLES; i++) {
        const int16_t *frame = in + i * TKL_AF_CAPTURE_LANES;

        s_aec_in[i * TKL_AF_AEC_CHANNELS + 0] = frame[TKL_AF_MIC0_LANE];
        s_aec_in[i * TKL_AF_AEC_CHANNELS + 1] = frame[TKL_AF_MIC1_LANE];

        if (s_ref_delay == 0U) {
            s_aec_in[i * TKL_AF_AEC_CHANNELS + 2] = frame[TKL_AF_REF_LANE];
        } else {
            s_ref_hist[s_ref_w] = frame[TKL_AF_REF_LANE];
            if (++s_ref_w > s_ref_delay) {
                s_ref_w = 0U;
            }
            /* Reading the slot just advanced past returns the value from
             * exactly s_ref_delay samples ago. */
            s_aec_in[i * TKL_AF_AEC_CHANNELS + 2] = s_ref_hist[s_ref_w];
        }
    }
}

/**
 * @brief Copy the mic0 lane out of an interleaved 3-lane frame
 */
static void __tkl_af_mic0_plane(const int16_t *in, int16_t *out)
{
    uint32_t i;

    for (i = 0; i < TKL_AF_CAPTURE_SAMPLES; i++) {
        out[i] = in[i * TKL_AF_CAPTURE_LANES + TKL_AF_MIC0_LANE];
    }
}

int tkl_af_process(const uint8_t *in, uint32_t inlen, uint8_t *out, uint32_t outlen)
{
    int rc = 0;

    if (in == NULL || out == NULL || inlen < TKL_AF_CAPTURE_BYTES || outlen < TKL_AF_OUT_BYTES) {
        return -1;
    }

    if (!s_af_ready) {
        if (tkl_af_init() != OPRT_OK) {
            return -1;
        }
    }

    if (s_aec_on) {
        __tkl_af_deinterleave((const int16_t *)in);
        rc = s_aec_module.process(s_aec_self, s_aec_in, TKL_AF_AEC_IN_BYTES, s_mono, TKL_AF_AEC_OUT_BYTES);
        if (rc != TKL_AF_AEC_OUT_BYTES) {
            /* State is undefined after a failed frame: fall back to mic0 and
             * let the next open recreate the instance. */
            tkl_log_output("tkl_af: BF+AEC+NS failed %d, passthrough\r\n", rc);
            tkl_af_deinit();
            __tkl_af_mic0_plane((const int16_t *)in, s_mono);
        }
    } else {
        __tkl_af_mic0_plane((const int16_t *)in, s_mono);
    }

    {
        uint32_t i;

        for (i = 0; i < TKL_AF_CAPTURE_SAMPLES; i++) {
            s_acc[s_acc_n + i] = s_mono[i];
        }
        s_acc_n += TKL_AF_CAPTURE_SAMPLES;
    }

    if (s_acc_n < TKL_AF_OUT_SAMPLES) {
        return 0;
    }

    {
        uint32_t i;

        for (i = 0; i < TKL_AF_OUT_SAMPLES; i++) {
            ((int16_t *)out)[i] = s_acc[i];
        }

        s_acc_n -= TKL_AF_OUT_SAMPLES;
        for (i = 0; i < s_acc_n; i++) {
            s_acc[i] = s_acc[TKL_AF_OUT_SAMPLES + i];
        }
    }

#if defined(TKL_AF_WITH_VAD) && (TKL_AF_WITH_VAD == 1)
    if (tkl_vad_feed(out, TKL_AF_OUT_BYTES) != OPRT_OK) {
        /* VAD is optional for the uplink: keep audio flowing on failure. */
    }
    tkl_kws_feed_with_vad(out, TKL_AF_OUT_BYTES, (tkl_vad_get_status() == TKL_VAD_STATUS_SPEECH) ? 1 : 0);
#elif defined(TKL_AF_WITH_KWS) && (TKL_AF_WITH_KWS == 1)
    tkl_kws_feed_with_vad(out, TKL_AF_OUT_BYTES, 0);
#endif

    return (int)TKL_AF_OUT_BYTES;
}
