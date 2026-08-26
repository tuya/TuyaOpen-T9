/**
 * @file avdk_nn_types.h
 * @brief Shared NN memory / model types for face recognition demo
 * @version 1.0
 * @date 2026-08-25
 * @copyright Copyright (c) Tuya Inc.
 */
#pragma once

#include <stdint.h>

#include "common/avdk_pixel_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AVDK_NN_MEM_TYPE_FALSH = 0,
    AVDK_NN_MEM_TYPE_HSRAM,
    AVDK_NN_MEM_TYPE_SRAM,
    AVDK_NN_MEM_TYPE_PSRAM_HEAP,
    AVDK_NN_MEM_TYPE_PSRAM_SLAB,
    AVDK_NN_MEM_TYPE_PSRAM_SLAB_UNCODED,
} avdk_nn_mem_type_t;

typedef enum {
    AVDK_NN_MODEL_TYPE_CPU = 0,
    AVDK_NN_MODEL_TYPE_NPU,
} avdk_nn_model_type_t;

typedef enum {
    AVDK_NN_MODEL_LOAD_TYPE_FLASH = 0,
    AVDK_NN_MODEL_LOAD_TYPE_SD_FILE,
} avdk_nn_model_load_type_t;

typedef struct {
    float x;
    float y;
    float w;
    float h;
    float score;
} FaceRecogBox;

#ifdef __cplusplus
}
#endif
