#ifndef AUDIO_SUBSYS_FSMN_VAD_H
#define AUDIO_SUBSYS_FSMN_VAD_H
#include "vad/audio_subsys_vad.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef audio_subsys_vad_config_t audio_subsys_fsmn_vad_config_t;
void audio_subsys_fsmn_vad_config_default(audio_subsys_fsmn_vad_config_t *cfg);
void audio_subsys_fsmn_vad_instance(audio_subsys_module_t *module);
int audio_subsys_fsmn_vad_create(void **self, void *cfg);
int audio_subsys_fsmn_vad_destroy(void *self);
int audio_subsys_fsmn_vad_set_callback(void *self, audio_subsys_callback_t cb, void *userdata);
int audio_subsys_fsmn_vad_ioctl(void *self, int cmd, void *arg);
int audio_subsys_fsmn_vad_process(void *self, const void *in, int inlen, void *out, int outlen);

#ifdef __cplusplus
}
#endif
#endif
