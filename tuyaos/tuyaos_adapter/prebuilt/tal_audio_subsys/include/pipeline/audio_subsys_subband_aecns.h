#ifndef AUDIO_SUBSYS_SUBBAND_AECNS_H
#define AUDIO_SUBSYS_SUBBAND_AECNS_H
#include "pipeline/audio_subsys_aecns.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct audio_subsys_subband_aecns_config {
    int framelen;             /* must be 128 */
    int fs;                   /* must be 16000 */
    int mic_num;              /* input mic channels >= 1; currently select mic0 */
    int ref_num;              /* input speaker reference channels >= 1; select ref0 */
    int taps;                 /* 1..128, default 8; heap cost grows with taps */
    float epsilon;            /* [0,1], default 0.06 */
    float noise_suppress;     /* [-100,0] dB, default -25 */
    float echo_suppress;      /* [-100,0] dB, default -40 */
    float echo_suppress_active; /* [-100,0] dB, default -15 */
    float leak;               /* [0,10], default 1.5 */
} audio_subsys_subband_aecns_config_t;

/* Only ONE live subband instance. ALL subband lifecycle/process calls must
 * be serialized, including calls from different tasks. Static scratch is
 * shared. With AUDIO_SUBSYS_USE_DTCM, the application must map .dtcm_noinit
 * to accessible aligned RAM (~44 KiB, excluding heap and stack).
 * Algorithm delay: 896 samples at 16 kHz (56 ms), not processing time.
 */
void audio_subsys_subband_aecns_config_default(audio_subsys_subband_aecns_config_t *cfg);
void audio_subsys_subband_aecns_instance(audio_subsys_module_t *module);
int audio_subsys_subband_aecns_create(void **self, void *cfg);
int audio_subsys_subband_aecns_destroy(void *self);
int audio_subsys_subband_aecns_set_callback(void *self, audio_subsys_callback_t cb, void *userdata);
int audio_subsys_subband_aecns_ioctl(void *self, int cmd, void *arg);
int audio_subsys_subband_aecns_process(void *self, const void *in, int inlen, void *out, int outlen);
#ifdef __cplusplus
}
#endif
#endif
