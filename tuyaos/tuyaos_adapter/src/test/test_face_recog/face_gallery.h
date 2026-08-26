/**
 * @file face_gallery.h
 * @brief SD face gallery enroll / 1:N verify (tkl_fs)
 * @version 1.0
 * @date 2026-08-25
 * @copyright Copyright (c) Tuya Inc.
 */
#pragma once

#include <stdint.h>

#include "FaceRecognitionModel.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FACE_GALLERY_DIR_BASE        "/sdcard/faces"
#define FACE_GALLERY_SAMPLES_PER_ID  3
#define FACE_GALLERY_AVG_THRESHOLD   0.62f

/**
 * @brief Ensure mount point usable and create /sdcard/faces
 * @return 0 on success
 */
int face_gallery_prepare(void);

/**
 * @brief Save one enroll sample (ppm + feature.bin)
 * @param[in] result embedding
 * @param[in] aligned_rgb 112x112 RGB
 * @param[in] aligned_rgb_size bytes
 * @param[out] out_saved_count sample index after save (1..3)
 * @param[out] out_profile_id profile id
 * @return 0 on success
 */
int face_gallery_enroll_save(const FaceVerifyResult *result,
                             const uint8_t *aligned_rgb,
                             uint32_t aligned_rgb_size,
                             uint32_t *out_saved_count,
                             uint32_t *out_profile_id);

/**
 * @brief 1:N verify against gallery
 * @param[in] current query embedding
 * @param[out] out_best_score best single-sample cosine
 * @param[out] out_best_avg_score avg cosine of best profile
 * @param[out] out_best_profile matched profile id
 * @param[out] out_best_sample best sample index
 * @return 0 on success (at least one complete profile compared)
 */
int face_gallery_verify(const FaceVerifyResult *current,
                        float *out_best_score,
                        float *out_best_avg_score,
                        uint32_t *out_best_profile,
                        uint32_t *out_best_sample);

/**
 * @brief List profiles and sample counts
 * @return 0 on success
 */
int face_gallery_list(void);

/**
 * @brief Clear one profile or all
 * @param[in] profile_id 0 means all
 * @return 0 on success
 */
int face_gallery_clear(uint32_t profile_id);

#ifdef __cplusplus
}
#endif
