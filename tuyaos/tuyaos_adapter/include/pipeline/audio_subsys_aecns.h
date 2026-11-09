#ifndef AUDIO_SUBSYS_AECNS_H
#define AUDIO_SUBSYS_AECNS_H

#include "common/audio_subsys_module.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_SUBSYS_AECNS_SAMPLE_RATE 16000
#define AUDIO_SUBSYS_AECNS_FRAME_SAMPLES 128
/* Default 1 mic + 1 ref only. Query GET_INFO for configured input_bytes. */
#define AUDIO_SUBSYS_AECNS_INPUT_BYTES 512
#define AUDIO_SUBSYS_AECNS_OUTPUT_BYTES 256

/* ioctl(arg = audio_subsys_aecns_info_t *). Module-specific commands are
 * declared in the corresponding module header; unsupported commands return -1. */
#define AUDIO_SUBSYS_AECNS_GET_INFO 0x4100

typedef struct audio_subsys_aecns_info {
    int sample_rate;
    int frame_samples;
    int mic_num;
    int ref_num;
    int output_channels;
    int input_bytes;
    int output_bytes;
    int algorithm_delay_samples; /* -1: not characterized; not CPU runtime */
    int single_instance;
    int bf_enabled;          /* 1 when the optional delay-and-sum BF pre-stage runs */
    int bf_latency_samples;  /* BF output delay in samples; 0 when bf_enabled == 0.
                              * Delay the speaker reference by this amount to keep
                              * the AEC path aligned (caller-side alignment). */
} audio_subsys_aecns_info_t;

/* Contract for both pipelines:
 * - Native-endian int16_t PCM, 16 kHz (S16LE on little-endian targets).
 * - mic_num and ref_num configure input channels (both >= 1, default 1 + 1).
 * - For each sample time: mic[0],...,mic[mic_num-1],ref[0],...,ref[ref_num-1].
 * - Only mic[0] and ref[0] are used. Other channels are skipped, not mixed;
 *   this is not multichannel AEC or beamforming yet.
 *   Exception (fdaf_aecns only): with bf_enable=1 all mic channels feed the
 *   delay-and-sum beamformer and its mono output feeds the AEC; ref_num still
 *   selects ref[0] only. Query bf_latency_samples from GET_INFO and align the
 *   reference yourself; the pipeline does not delay it internally.
 * - Callers provide int16_t sample buffers with int16_t alignment; no byte swapping.
 * - Lengths remain byte counts: exactly 128 * (mic_num + ref_num) * sizeof(int16_t)
 *   input bytes per call; output capacity >= 256 bytes. GET_INFO reports this size.
 * - create rejects channel counts whose input byte count would exceed INT_MAX.
 * - Return 256 on success, -1 on failure. Output is final mono AEC+NS.
 * - No internal audio I/O, thread, resampler, reference alignment or FIFO.
 * - Calls on a given instance are serialized by the caller.
 * - Output may overlap input: the complete input frame is split first.
 * - Optional callback is synchronous, after output is produced: flag=0,
 *   dlen=256. Its return value is ignored (notification only). It must not
 *   reenter/destroy/reconfigure the module or modify the borrowed output.
 *   Copy data before returning if it is needed asynchronously.
 * - create(NULL cfg) uses defaults. With a non-NULL cfg, initialize it using
 *   the module's config_default() function, then override desired fields.
 * - Reconfiguration/reset is not implemented; destroy/create instead.
 */
#ifdef __cplusplus
}
#endif
#endif
