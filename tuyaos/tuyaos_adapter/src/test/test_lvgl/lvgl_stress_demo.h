#ifndef TUYA_LVGL_STRESS_DEMO_H
#define TUYA_LVGL_STRESS_DEMO_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

void tuya_lvgl_stress_create(void);
void tuya_lvgl_stress_pause(void);
void tuya_lvgl_stress_resume(void);
void tuya_lvgl_stress_destroy(void);

#ifdef __cplusplus
}
#endif

#endif
