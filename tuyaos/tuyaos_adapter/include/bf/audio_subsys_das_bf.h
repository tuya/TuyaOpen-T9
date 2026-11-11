#ifndef AUDIO_SUBSYS_DAS_BF_H
#define AUDIO_SUBSYS_DAS_BF_H
#include "common/audio_subsys_module.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Delay-and-sum beamformer over an equispaced linear microphone array.
 * Ported from the T5 project; see README for provenance and limits.
 *
 * Contract:
 * - Native-endian int16_t PCM, 16 kHz (S16LE on little-endian targets).
 * - Input per process(): 128 interleaved samples per mic channel
 *   (mic0, mic1, ... mic[channels-1] at each sample time);
 *   input_bytes = 128 * channels * 2. Output: 128 mono samples (256 bytes).
 * - Returns 256 on success, -1 on failure. No internal I/O or thread.
 * - The stream is stateful: call process() in time order; calls on one
 *   instance are serialized by the caller.
 * - Output lags the input by a fixed latency that depends on the steering
 *   (base + max per-channel delay, <= ~128 samples). Query it via GET_INFO
 *   after create/SET_DIRECTION and use it to align the AEC reference when
 *   feeding a downstream AEC module.
 * - Optional callback fires synchronously after each frame: flag=0,
 *   dlen=256. It must not reenter/destroy/reconfigure the module.
 */

#define AUDIO_SUBSYS_DAS_BF_SAMPLE_RATE 16000
#define AUDIO_SUBSYS_DAS_BF_FRAME_SAMPLES 128
#define AUDIO_SUBSYS_DAS_BF_MAX_CHANNELS 4

/* ioctl(arg = audio_subsys_das_bf_info_t *) */
#define AUDIO_SUBSYS_DAS_BF_GET_INFO 0x4120
/* ioctl(arg = audio_subsys_das_bf_direction_t *); NULL arg is an error */
#define AUDIO_SUBSYS_DAS_BF_SET_DIRECTION 0x4121
/* ioctl(arg ignored/NULL): clear delay-line state, keep configuration */
#define AUDIO_SUBSYS_DAS_BF_RESET 0x4122

typedef struct audio_subsys_das_bf_config {
    int channels;        /* 1..4 mics, default 2 */
    int fs;              /* must be 16000 */
    float azimuth_deg;   /* steering angle, default 90 (endfire) */
    float mic_spacing_m; /* element spacing in meters, > 0, default 0.03 */
} audio_subsys_das_bf_config_t;

typedef struct audio_subsys_das_bf_info {
    int sample_rate;
    int frame_samples;
    int channels;
    int input_bytes;      /* 128 * channels * 2 */
    int output_bytes;     /* 256 */
    int latency_samples;  /* fixed output delay; align AEC ref with it */
    float azimuth_deg;    /* current steering */
    float mic_spacing_m;  /* current spacing */
} audio_subsys_das_bf_info_t;

typedef struct audio_subsys_das_bf_direction {
    float azimuth_deg;   /* new steering angle */
    float mic_spacing_m; /* > 0; both fields are required */
} audio_subsys_das_bf_direction_t;

void audio_subsys_das_bf_config_default(audio_subsys_das_bf_config_t *cfg);
void audio_subsys_das_bf_instance(audio_subsys_module_t *module);
int audio_subsys_das_bf_create(void **self, void *cfg);
int audio_subsys_das_bf_destroy(void *self);
int audio_subsys_das_bf_set_callback(void *self, audio_subsys_callback_t cb, void *userdata);
int audio_subsys_das_bf_ioctl(void *self, int cmd, void *arg);
int audio_subsys_das_bf_process(void *self, const void *in, int inlen, void *out, int outlen);

#ifdef __cplusplus
}
#endif
#endif
