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

/***********************************************************
***********************variable define**********************
***********************************************************/
static void            *s_vad_self    = NULL;
static TKL_VAD_STATUS_T s_vad_status  = TKL_VAD_STATUS_NONE;
static BOOL_T           s_vad_started = FALSE;

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

    if (s_vad_self) {
        s_vad_status  = TKL_VAD_STATUS_NONE;
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

    s_vad_status = result.speech ? TKL_VAD_STATUS_SPEECH : TKL_VAD_STATUS_NONE;

    return OPRT_OK;
}

TKL_VAD_STATUS_T tkl_vad_get_status(void)
{
    return s_vad_status;
}

OPERATE_RET tkl_vad_start(void)
{
    s_vad_status  = TKL_VAD_STATUS_NONE;
    s_vad_started = TRUE;

    return OPRT_OK;
}

OPERATE_RET tkl_vad_stop(void)
{
    s_vad_status  = TKL_VAD_STATUS_NONE;
    s_vad_started = FALSE;

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
