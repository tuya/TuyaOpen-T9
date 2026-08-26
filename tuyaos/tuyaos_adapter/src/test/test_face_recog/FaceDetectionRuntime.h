/**
 * @file FaceDetectionRuntime.h
 * @brief Face detection runtime (SCRFD-style decode + NMS)
 * @version 1.0
 * @date 2026-08-25
 * @copyright Copyright (c) Tuya Inc.
 */
#pragma once

#include <stdint.h>

#include "AvdkMultiModel.h"
#include "FaceRecognitionModel.h"
#include "tensorflow/lite/micro/micro_interpreter.h"

class FaceDetectionRuntime {
public:
    FaceDetectionRuntime();
    ~FaceDetectionRuntime();

    void setModelFilePath(const char *path);
    bool init();
    void deinit();
    tflite::MicroInterpreter *prepareInterpreter();
    void releaseInterpreter();
    void resetOutputSpecs();

    bool buildOutputSpecs(tflite::MicroInterpreter *interpreter);
    int invokeAndDecode(tflite::MicroInterpreter *interpreter, FaceDetection *out, int max_out);

    /**
     * @brief Max raw score seen in last decode (before NMS), for debug
     * @return max score in [0,1] typically; -1 if not run yet
     */
    float lastMaxScore(void) const { return last_max_score_; }

    /**
     * @brief Count of anchors that passed threshold in last decode
     * @return pass count before NMS
     */
    int lastPassCount(void) const { return last_pass_count_; }

    /**
     * @brief Override detect score threshold (default 0.5, same as vendor)
     * @param[in] thr score threshold in [0,1]
     * @return none
     */
    void setDetectThreshold(float thr) { detect_threshold_ = thr; }

    /**
     * @brief Get current detect score threshold
     * @return threshold
     */
    float detectThreshold(void) const { return detect_threshold_; }

private:
    struct OutputSpec {
        int stride;
        int feature_count;
        TfLiteTensor *score;
        TfLiteTensor *bbox;
        TfLiteTensor *keypoints;
    };

    float tensorValue(const TfLiteTensor *tensor, int flat_index) const;
    float keypointTensorValue(const TfLiteTensor *tensor, int logical_idx, int channel, int stride) const;
    int decode(FaceDetection *candidates, int max_candidates);
    int nms(FaceDetection *candidates, int count, FaceDetection *out, int max_out);

    OutputSpec specs_[3];
    FaceDetection *candidates_;
    AvdkMultiModel model_;
    const char *model_file_path_;
    bool output_specs_ready_;
    float detect_threshold_;
    float last_max_score_;
    int last_pass_count_;
};
