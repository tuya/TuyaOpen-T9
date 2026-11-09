#ifndef AUDIO_SUBSYS_VAD_H
#define AUDIO_SUBSYS_VAD_H

#include "common/audio_subsys_module.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_SUBSYS_VAD_SAMPLE_RATE 16000
#define AUDIO_SUBSYS_VAD_FRAME_SAMPLES 320
#define AUDIO_SUBSYS_VAD_INPUT_BYTES 640
#define AUDIO_SUBSYS_VAD_GET_INFO 0x4200
#define AUDIO_SUBSYS_VAD_SET_ENERGY_THRESHOLD 0x4201

typedef struct audio_subsys_vad_config {
    float energy_thd; /* Mean-absolute-amplitude dBFS gate, [-120,0], default -100. */
} audio_subsys_vad_config_t;

typedef struct audio_subsys_vad_result {
    float probability; /* Smoothed, energy-gated probability; not raw model output. */
    int speech;        /* Current postprocessed state: 0 non-speech, 1 speech. */
} audio_subsys_vad_result_t;

typedef struct audio_subsys_vad_info {
    int sample_rate;
    int frame_samples;
    int input_bytes;
    int output_bytes;
} audio_subsys_vad_info_t;

/* Both VAD modules:
 * - Input: exactly 320 aligned int16_t mono samples (640 bytes), native endian,
 *   16 kHz / 20 ms. No WAV parsing, channel selection, resampling or internal FIFO.
 * - out: buffer for audio_subsys_vad_result_t; capacity >= sizeof(result).
 * - process returns sizeof(result) on success, -1 on failure, NOT the speech flag.
 *   Output remains untouched on failure. After inference failure recreate the
 *   instance: streaming state may already have advanced. Do not retry that frame.
 * - Callback is synchronous, data points to result, dlen=sizeof(result), flag=speech.
 *   Return value is ignored. Copy the borrowed result before returning if needed;
 *   do not modify it or reenter/destroy/reconfigure the instance in a callback.
 * - GET_INFO arg: audio_subsys_vad_info_t*. SET_ENERGY_THRESHOLD arg: float*.
 *   Unsupported commands return -1. Configuration NULL selects defaults.
 * - Serialize all calls on an instance. Reset by destroy/create. Models are
 *   embedded; no filesystem, audio capture thread, or AEC pipeline connection.
 * - Postprocessing retains the source model's history/hangover, not start/end
 *   events. AEC's 128-sample output must be buffered into 320-sample VAD frames
 *   by the caller; sending an AEC frame directly is invalid.
 */
#ifdef __cplusplus
}
#endif
#endif
