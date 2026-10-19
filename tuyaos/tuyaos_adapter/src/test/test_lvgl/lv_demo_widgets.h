/**
 * @file lv_demo_widgets.h
 *
 */

#ifndef LV_DEMO_WIDGETS_H
#define LV_DEMO_WIDGETS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"
#include "lv_conf_custom.h"

/*
 * The demo image assets vendored under ./assets are renamed with a tuya_ prefix
 * (tuya_img_clothes, tuya_img_demo_widgets_avatar, ...). 4.0.1.11 added
 * lvgl_v9/demos/widgets/assets/*.c to the LVGL component, and those upstream files
 * define the same img_* symbols without any LV_USE_DEMO_WIDGETS guard, so the
 * unprefixed names would collide at link time.
 */

#if LV_USE_DEMO_WIDGETS

#if LV_USE_GRID == 0
#error "LV_USE_GRID needs to be enabled"
#endif

#if LV_USE_FLEX == 0
#error "LV_USE_FLEX needs to be enabled"
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void lv_demo_widgets(void);
void lv_demo_widgets_start_slideshow(void);

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_DEMO_WIDGETS*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_DEMO_WIDGETS_H*/
