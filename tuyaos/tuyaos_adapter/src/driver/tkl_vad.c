/**
 * @file tkl_vad.c
 * @brief T9 (BK7259) VAD, backed by the tal_audio_subsys FSMN-VAD module
 * @version 1.0
 * @date 2026-11-06
 * @copyright Copyright (c) 2025-2026 Tuya Inc. All Rights Reserved.
 *
 * The algorithm library consumes exactly 320 int16 mono samples (640 bytes)
 * per call. tkl_audio_frontend.c buffers the 8 ms AEC output into those
 * blocks and calls tkl_vad_feed(); nothing else on T9 feeds this module.
 */

#include "tkl_vad.h"

#include "tkl_output.h"
#include "tkl_audio_frontend.h"

#if defined(TKL_AF_WITH_VAD) && (TKL_AF_WITH_VAD == 1)

#include "vad/audio_subsys_fsmn_vad.h"

/***********************************************************
************************macro define************************
***********************************************************/
#define TKL_VAD_FRAME_SAMPLES (320)
#define TKL_VAD_INPUT_BYTES   (TKL_VAD_FRAME_SAMPLES * sizeof(int16_t))
#define TKL_VAD_FRAME_MS      (20)

/* Endpoint defaults when the caller leaves them unset; same values ai_chat_main
 * passes through ai_audio_input.c. */
#define TKL_VAD_SPEECH_MIN_MS (200)
#define TKL_VAD_NOISE_MIN_MS  (1000)

/* The library reports a per-frame probability. Treat one frame as speech above
 * this value, then latch the published state only after the configured run of
 * like frames, so a single noisy frame cannot flip the uplink stream. */
#define TKL_VAD_FRAME_SPEECH_THD (0.6f)

/***********************************************************
***********************variable define**********************
***********************************************************/
static void            *s_vad_self    = NULL;
static TKL_VAD_STATUS_T s_vad_status  = TKL_VAD_STATUS_NONE;
static BOOL_T           s_vad_started = FALSE;
static uint32_t         s_vad_speech_min_ms = TKL_VAD_SPEECH_MIN_MS;
static uint32_t         s_vad_noise_min_ms  = TKL_VAD_NOISE_MIN_MS;
static uint32_t         s_vad_speech_ms     = 0; /* consecutive speech frames */
static uint32_t         s_vad_silence_ms    = 0; /* consecutive non-speech frames */

/***********************************************************
***********************function define**********************
***********************************************************/
OPERATE_RET tkl_vad_set_threshold(TKL_AUDIO_VAD_THRESHOLD_E level)
{
    float energy_thd;

    if (s_vad_self == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    /* The gate is a mean-absolute-amplitude dBFS floor: a higher value needs
     * louder input to report speech. Order matches T5AI: HIGH is strictest. */
    switch (level) {
    case TKL_AUDIO_VAD_HIGH:
        energy_thd = -40.0f;
        break;
    case TKL_AUDIO_VAD_MID:
        energy_thd = -60.0f;
        break;
    case TKL_AUDIO_VAD_LOW:
        energy_thd = -80.0f;
        break;
    default:
        return OPRT_INVALID_PARM;
    }

    if (audio_subsys_fsmn_vad_ioctl(s_vad_self, AUDIO_SUBSYS_VAD_SET_ENERGY_THRESHOLD, &energy_thd) != 0) {
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_vad_init(TKL_VAD_CONFIG_T *config)
{
    audio_subsys_fsmn_vad_config_t cfg;

    if (config == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (config->sample_rate != TKL_AF_SAMPLE_RATE || config->channel_num != 1) {
        tkl_log_output("tkl_vad: only 16 kHz mono is supported (%u Hz, %u ch)\r\n", config->sample_rate,
                       config->channel_num);
        return OPRT_INVALID_PARM;
    }

    /* config->frame_duration_ms is advisory: ai_audio_input passes 10 while
     * the T9 front end delivers 20 ms blocks, and the library itself works on
     * 10 ms feature frames inside a 320-sample call. tkl_vad_feed enforces the
     * only contract that matters, the 640-byte block size. */

    /* Endpointing parameters, in the same units as T5AI's rnn_vad:
     * speech_min_ms is how long speech must run before the uplink opens,
     * noise_min_ms is how long silence must run before it closes again. */
    s_vad_speech_min_ms = config->speech_min_ms ? config->speech_min_ms : TKL_VAD_SPEECH_MIN_MS;
    s_vad_noise_min_ms  = config->noise_min_ms ? config->noise_min_ms : TKL_VAD_NOISE_MIN_MS;
    s_vad_speech_ms     = 0;
    s_vad_silence_ms    = 0;

    tkl_log_output("tkl_vad: speech_min %u ms, noise_min %u ms\r\n", (unsigned)s_vad_speech_min_ms,
                   (unsigned)s_vad_noise_min_ms);

    if (s_vad_self) {
        s_vad_started = TRUE;
        return OPRT_OK;
    }

    audio_subsys_fsmn_vad_config_default(&cfg);
    if (audio_subsys_fsmn_vad_create(&s_vad_self, &cfg) != 0) {
        s_vad_self = NULL;
        tkl_log_output("tkl_vad: fsmn_vad create failed\r\n");
        return OPRT_MALLOC_FAILED;
    }

    s_vad_status  = TKL_VAD_STATUS_NONE;
    s_vad_started = TRUE;

    return OPRT_OK;
}

OPERATE_RET tkl_vad_feed(uint8_t *data, uint32_t len)
{
    audio_subsys_vad_result_t result;
    int                       rc;

    if (s_vad_self == NULL || data == NULL) {
        return OPRT_RESOURCE_NOT_READY;
    }

    if (!s_vad_started) {
        return OPRT_OK;
    }

    if (len != TKL_VAD_INPUT_BYTES) {
        return OPRT_INVALID_PARM;
    }

    rc = audio_subsys_fsmn_vad_process(s_vad_self, data, (int)len, &result, sizeof(result));
    if (rc != (int)sizeof(result)) {
        /* Streaming state is undefined after a failed frame; recreate instead
         * of retrying it. */
        tkl_log_output("tkl_vad: process failed %d, recreating\r\n", rc);
        audio_subsys_fsmn_vad_destroy(s_vad_self);
        s_vad_self   = NULL;
        s_vad_status = TKL_VAD_STATUS_NONE;
        return OPRT_COM_ERROR;
    }

    {
        /* Hysteresis on the library's per-frame probability. Opening the
         * uplink needs speech_min_ms of consecutive speech frames; closing it
         * needs noise_min_ms of consecutive silence. Without this the raw
         * windowed output toggles around the threshold and the upper layer
         * starts/stops the cloud stream every second. */
        if (result.probability > TKL_VAD_FRAME_SPEECH_THD) {
            s_vad_silence_ms = 0;
            if (s_vad_speech_ms < s_vad_speech_min_ms) {
                s_vad_speech_ms += TKL_VAD_FRAME_MS;
            }
            if (s_vad_status != TKL_VAD_STATUS_SPEECH && s_vad_speech_ms >= s_vad_speech_min_ms) {
                s_vad_status = TKL_VAD_STATUS_SPEECH;
            }
        } else {
            s_vad_speech_ms = 0;
            if (s_vad_silence_ms < s_vad_noise_min_ms) {
                s_vad_silence_ms += TKL_VAD_FRAME_MS;
            }
            if (s_vad_status != TKL_VAD_STATUS_NONE && s_vad_silence_ms >= s_vad_noise_min_ms) {
                s_vad_status = TKL_VAD_STATUS_NONE;
            }
        }
    }

    return OPRT_OK;
}

TKL_VAD_STATUS_T tkl_vad_get_status(void)
{
    return s_vad_status;
}

OPERATE_RET tkl_vad_start(void)
{
    /* Same contract as T5AI's rnn_vad start/stop: begin from silence, so the
     * uplink opens on the first real utterance rather than on whatever the
     * previous session left latched. ai_audio_input_reset() also drops its
     * cached flag, which keeps this transition from publishing an event. */
    s_vad_speech_ms  = 0;
    s_vad_silence_ms = 0;
    s_vad_status     = TKL_VAD_STATUS_NONE;
    s_vad_started    = TRUE;

    return OPRT_OK;
}

OPERATE_RET tkl_vad_stop(void)
{
    s_vad_speech_ms  = 0;
    s_vad_silence_ms = 0;
    s_vad_status     = TKL_VAD_STATUS_NONE;
    s_vad_started    = FALSE;

    return OPRT_OK;
}

OPERATE_RET tkl_vad_deinit(void)
{
    if (s_vad_self) {
        audio_subsys_fsmn_vad_destroy(s_vad_self);
        s_vad_self = NULL;
    }

    s_vad_status  = TKL_VAD_STATUS_NONE;
    s_vad_started = FALSE;

    return OPRT_OK;
}

#else /* TKL_AF_WITH_VAD */

/* Stubs keep the header contract linkable when the front end is built without
 * the algorithm library; ai_audio_input.c includes this header regardless. */
OPERATE_RET tkl_vad_set_threshold(TKL_AUDIO_VAD_THRESHOLD_E level)
{
    (void)level;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_vad_init(TKL_VAD_CONFIG_T *config)
{
    (void)config;
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_vad_feed(uint8_t *data, uint32_t len)
{
    (void)data;
    (void)len;
    return OPRT_NOT_SUPPORTED;
}

TKL_VAD_STATUS_T tkl_vad_get_status(void)
{
    return TKL_VAD_STATUS_NONE;
}

OPERATE_RET tkl_vad_start(void)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_vad_stop(void)
{
    return OPRT_NOT_SUPPORTED;
}

OPERATE_RET tkl_vad_deinit(void)
{
    return OPRT_OK;
}

#endif /* TKL_AF_WITH_VAD */
