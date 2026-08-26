/**
 * @file FaceRecognitionModel.h
 * @brief Face detect + align + verify pipeline (ported from avdk tflm_face_recog)
 * @version 1.0
 * @date 2026-08-25
 * @copyright Copyright (c) Tuya Inc.
 */
#pragma once

#include <stdint.h>

#include "avdk_nn_types.h"
#include "common/avdk_pixel_types.h"

constexpr int kFaceEmbeddingDim = 512;
constexpr float kFaceVerifySameThreshold = 0.7f;
constexpr int kFaceDetectKeypointCount = 5;

struct FacePoint {
    float x;
    float y;
};

struct FaceDetection {
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int order;
    FacePoint keypoints[kFaceDetectKeypointCount];
};

struct FaceVerifyResult {
    bool valid;
    float norm;
    float normalized[kFaceEmbeddingDim];
};

typedef void (*faceBoxCallbackT)(FaceRecogBox *boxes, int count);
typedef void (*faceVerifyResultCallbackT)(const FaceVerifyResult *result);
typedef void (*faceEnrollResultCallbackT)(const FaceVerifyResult *result,
                                          const uint8_t *aligned_rgb,
                                          uint32_t aligned_rgb_size);

float face_recognition_cosine_similarity(const float *a, const float *b, int dim);
bool face_recognition_is_same_person(const float *a, const float *b, int dim, float threshold);

class FaceDetectionRuntime;
class FaceVerifyRuntime;

class FaceRecognitionModel {
public:
    FaceRecognitionModel();
    ~FaceRecognitionModel();

    void setDetectModelFilePath(const char *path);
    void setVerifyModelFilePath(const char *path);
    void setBoxCallback(faceBoxCallbackT cb);
    void setVerifyResultCallback(faceVerifyResultCallbackT cb);
    void setEnrollResultCallback(faceEnrollResultCallbackT cb);
    void setVerifyEnabled(bool enable);

    /**
     * @brief Force 4bpp channel order for preprocess
     * @param[in] mode 0=auto(BGRX like vendor), 1=force BGRX, 2=force RGBX
     * @return none
     */
    void setInputLayoutMode(int mode);

    /**
     * @brief Override detect score threshold
     * @param[in] thr score threshold
     * @return none
     */
    void setDetectThreshold(float thr);

    /**
     * @brief Last detect max score / pass count (after run)
     * @param[out] max_score max raw score
     * @param[out] pass_count anchors above threshold
     * @return none
     */
    void getLastDetectDebug(float *max_score, int *pass_count) const;

    int init(void);
    int deinit(void);
    /**
     * @brief Run detect (+ optional align/verify) on one 320x320 frame
     * @param[in] data pixel buffer (ISP RGB888/BGR888 stored as 4bpp X-padded)
     * @param[in] size buffer bytes (expect 320*320*4)
     * @param[in] pixel_format BK_PIXEL_FORMAT_RGB888 / BGR888 / BGRA8888 / ...
     * @return kept face count (>0) or 0
     */
    int run(uint8_t *data, uint32_t size, bk_pixel_format_t pixel_format);

    uint16_t getWidth(void) const { return 320; }
    uint16_t getHeight(void) const { return 320; }

    static void LogEnable(bool enable);

private:
    bool ensureDetectorInterpreter(void);
    void releaseDetectorInterpreter(void);

    FaceDetectionRuntime *detector_;
    FaceVerifyRuntime *verifier_;
    const char *detect_model_file_path_;
    const char *verify_model_file_path_;
    faceBoxCallbackT box_callback_;
    faceVerifyResultCallbackT verify_result_callback_;
    faceEnrollResultCallbackT enroll_result_callback_;
    bool verify_enabled_;
    bool runtimes_initialized_;
    int input_layout_mode_; /* 0=auto, 1=bgrx, 2=rgbx */
};
