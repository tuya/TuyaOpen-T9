#ifndef AUDIO_SUBSYS_FDAF_AECNS_H
#define AUDIO_SUBSYS_FDAF_AECNS_H
#include "pipeline/audio_subsys_aecns.h"
#include "bf/audio_subsys_das_bf.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct audio_subsys_fdaf_aecns_config {
    int framelen;              /* must be 128 */
    int fs;                    /* must be 16000 */
    int mic_num;               /* input mic channels >= 1; mic0 only, or all mics
                                 * through BF when bf_enable=1 (2..4 mics) */
    int ref_num;               /* input speaker reference channels >= 1; select ref0 */
    int bf_enable;             /* 0 (default): select mic0 only. 1: delay-and-sum BF
                                 * over ALL mic channels (requires 2..4 mics); the
                                 * BF output feeds AEC. Steering via bf_* fields and
                                 * AUDIO_SUBSYS_FDAF_AECNS_SET_BF_DIRECTION. */
    float bf_azimuth_deg;      /* BF steering angle, default 90 (endfire) */
    float bf_mic_spacing_m;    /* BF element spacing in meters, > 0, default 0.03 */
    int filterlen;             /* 128..16000 samples, rounded up by core */
    float mu;                  /* (0,1], default 0.1 */
    int residual_suppression;  /* 0..1000, core scale = value * 0.01; default 200 */
    int denoise_disable;       /* 0: enable, 1: disable noise suppression */
    int denoise_dB;            /* [-100,0], default -15 */
    int echo_suppress_dB;      /* [-100,0], default -40 */
    int echo_suppress_active_dB; /* [-100,0], default -15 */
} audio_subsys_fdaf_aecns_config_t;

/* Optional linear AEC dump, synchronously called after AEC and before NS.
 * pcm: borrowed, read-only native-endian int16_t mono samples; bytes: 256.
 * Copy/write the samples before returning; NS will overwrite this buffer.
 * Do not reenter, destroy or reconfigure the module from this callback.
 * Return 0 on success. Nonzero aborts process() with -1 before NS/output;
 * AEC state has already advanced, so stop the stream and recreate the instance.
 * The library does not open files. userdata remains owned by the caller.
 */
typedef int (*audio_subsys_fdaf_aecns_linear_dump_callback_t)(
    const int16_t *pcm, int bytes, void *userdata);

typedef struct audio_subsys_fdaf_aecns_linear_dump {
    audio_subsys_fdaf_aecns_linear_dump_callback_t callback;
    void *userdata;
} audio_subsys_fdaf_aecns_linear_dump_t;

/* ioctl arg: pointer to the struct above (copied), or NULL to disable.
 * A NULL callback also disables dumping. Default is disabled.
 * Register/unregister between process() calls; detach before closing userdata.
 */
#define AUDIO_SUBSYS_FDAF_AECNS_SET_LINEAR_DUMP 0x4110

/* ioctl(arg = audio_subsys_das_bf_direction_t *, both fields required):
 * resteer the BF pre-stage at runtime. Only valid when bf_enable=1,
 * -1 otherwise. Serial with process(). */
#define AUDIO_SUBSYS_FDAF_AECNS_SET_BF_DIRECTION 0x4111

void audio_subsys_fdaf_aecns_config_default(audio_subsys_fdaf_aecns_config_t *cfg);
void audio_subsys_fdaf_aecns_instance(audio_subsys_module_t *module);
int audio_subsys_fdaf_aecns_create(void **self, void *cfg);
int audio_subsys_fdaf_aecns_destroy(void *self);
int audio_subsys_fdaf_aecns_set_callback(void *self, audio_subsys_callback_t cb, void *userdata);
int audio_subsys_fdaf_aecns_ioctl(void *self, int cmd, void *arg);
int audio_subsys_fdaf_aecns_process(void *self, const void *in, int inlen, void *out, int outlen);
#ifdef __cplusplus
}
#endif
#endif
