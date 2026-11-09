#ifndef AUDIO_SUBSYS_KWS_H
#define AUDIO_SUBSYS_KWS_H

#include "common/audio_subsys_module.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Streaming FSMN-CTC keyword spotting over TFLite Micro (vendored WeKWS). */
#define AUDIO_SUBSYS_KWS_SAMPLE_RATE 16000
#define AUDIO_SUBSYS_KWS_CHUNK_SAMPLES 320
#define AUDIO_SUBSYS_KWS_INPUT_BYTES 640
#define AUDIO_SUBSYS_KWS_NAME_MAX 32

/* audio_subsys_kws_config_t.search_type */
#define AUDIO_SUBSYS_KWS_SEARCH_BEAM 0   /* CTC prefix beam search (default) */
#define AUDIO_SUBSYS_KWS_SEARCH_GREEDY 1 /* per-frame argmax + edit distance */

#define AUDIO_SUBSYS_KWS_GET_INFO 0x4300
#define AUDIO_SUBSYS_KWS_ADD_KEYWORD 0x4301
#define AUDIO_SUBSYS_KWS_RESET 0x4302

/* One keyword: CTC token-ID sequence in the embedded model's 40-entry
 * vocabulary. name is UTF-8 and copied at registration; token_ids are copied
 * too, so both may be released after create()/ioctl() returns. threshold <= 0
 * selects config.threshold. */
typedef struct audio_subsys_kws_keyword {
    const char *name;
    const int *token_ids;
    int num_ids;
    float threshold;
} audio_subsys_kws_keyword_t;

typedef struct audio_subsys_kws_config {
    int num_bins;         /* mel bins, 1..128; must match the embedded model (80) */
    int search_type;      /* BEAM / GREEDY, default BEAM */
    int score_beam_size;  /* beam: hypothesis beam, default 5 */
    int path_beam_size;   /* beam: prefix path beam, default 20 */
    int min_frames;       /* shortest keyword span, in 10 ms frames, default 3 */
    int max_frames;       /* longest keyword span, default 250 */
    int batch_size;       /* frames per invoke, default 1 (keep 1) */
    int frame_skip;       /* decode every Nth frame, default 3 */
    int left_context;     /* context frames concatenated per feature, default 2 */
    int right_context;    /* default 2; model expects 80 bins x 5 frames */
    int arena_size;       /* TFLite tensor arena bytes, heap-allocated, default 128 KiB */
    float threshold;      /* default per-token score gate, default 0.00001 */
    const audio_subsys_kws_keyword_t *keywords; /* optional, registered at create */
    int keyword_count;
} audio_subsys_kws_config_t;

typedef struct audio_subsys_kws_result {
    char name[AUDIO_SUBSYS_KWS_NAME_MAX]; /* registered name, UTF-8; longer
                                              names are truncated to 31 bytes
                                              (may split a UTF-8 character) */
    float score;                          /* mean per-token probability */
    int start_frame;                      /* span start, 10 ms feature frames */
    int end_frame;                        /* span end, inclusive */
} audio_subsys_kws_result_t;

typedef struct audio_subsys_kws_info {
    int sample_rate;
    int chunk_samples;
    int input_bytes;
    int output_bytes;
    int num_bins;
    int vocab;         /* model CTC vocabulary size (40) */
    int feature_dim;   /* model input width = num_bins * (left+1+right) */
} audio_subsys_kws_info_t;

/* Contract:
 * - Input: exactly 320 aligned int16_t mono samples (640 bytes), 16 kHz,
 *   20 ms, native endian. Any other length fails; no FIFO, resampling,
 *   de-interleaving or WAV parsing. Callers buffer AEC/VAD output into
 *   320-sample chunks themselves.
 * - Output: audio_subsys_kws_result_t. process() returns 0 when the chunk was
 *   consumed and no keyword fired, sizeof(result) on a hit, -1 on failure.
 *   The result is also passed to the callback on a hit: data = result,
 *   dlen = sizeof(result), flag = 1. No callback without a hit.
 * - Detection is sparse: most calls return 0. After a hit the decoders
 *   suppress the same span; see the vendored WeKWS decoder notes.
 * - A model/inference failure returns -1; the stream state is undefined then.
 *   Stop and destroy/recreate the instance instead of retrying the frame.
 * - GET_INFO arg: audio_subsys_kws_info_t*.
 *   ADD_KEYWORD arg: audio_subsys_kws_keyword_t* (between process() calls).
 *   RESET arg: NULL; clears features, model cache and decoder state, keeps
 *   registered keywords. Unsupported commands return -1.
 * - The model is embedded (~77 KiB flash); no filesystem at runtime. The
 *   engine allocates the pipeline block via the OS allocator at create()
 *   (roughly 30-60 KiB, vocab/context dependent) and the TFLite Impl
 *   (arena + caches, ~100-140 KiB) through the C++ allocator.
 * - All calls on one instance are serialized by the caller. Reset via
 *   destroy/create or the RESET ioctl.
 */
void audio_subsys_kws_config_default(audio_subsys_kws_config_t *cfg);
void audio_subsys_kws_instance(audio_subsys_module_t *module);
int audio_subsys_kws_create(void **self, void *cfg);
int audio_subsys_kws_destroy(void *self);
int audio_subsys_kws_set_callback(void *self, audio_subsys_callback_t cb, void *userdata);
int audio_subsys_kws_ioctl(void *self, int cmd, void *arg);
int audio_subsys_kws_process(void *self, const void *in, int inlen, void *out, int outlen);

#ifdef __cplusplus
}
#endif
#endif
