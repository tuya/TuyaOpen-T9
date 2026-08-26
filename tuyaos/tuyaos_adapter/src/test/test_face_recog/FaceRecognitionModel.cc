/**
 * @file FaceRecognitionModel.cc
 * @brief Face detect + CPU affine align + verify pipeline
 * @version 1.0
 * @date 2026-08-25
 * @copyright Copyright (c) Tuya Inc.
 */
#include "FaceRecognitionModel.h"

#include "FaceDetectionRuntime.h"
#include "FaceVerifyRuntime.h"

#include <math.h>
#include <stdint.h>

#include <components/bk_frame_buffer.h>
#include <os/mem.h>

#include "tensorflow/lite/micro/cortex_m_generic/debug_log_callback.h"
#include "tensorflow/lite/micro/micro_log.h"

#if defined(__ARM_FEATURE_MVE)
#include <arm_mve.h>
#endif

#ifndef FACE_RECOG_DETECT_MODEL_SD_PATH
#define FACE_RECOG_DETECT_MODEL_SD_PATH "/sdcard/tflite/face_detection_int8_vela.tflite"
#endif

#ifndef FACE_RECOG_VERIFY_MODEL_SD_PATH
#define FACE_RECOG_VERIFY_MODEL_SD_PATH "/sdcard/tflite/face_verify_int8_vela.tflite"
#endif

static constexpr int kDetectInputW = 320;
static constexpr int kDetectInputH = 320;
static constexpr int kAlignedFaceW = 112;
static constexpr int kAlignedFaceH = 112;
static constexpr int kAlignedFaceC = 3;
static constexpr int kAlignedFaceBytes = kAlignedFaceW * kAlignedFaceH * kAlignedFaceC;
static constexpr int kMaxFaceDetections = 32;

static FaceRecogBox s_face_out_boxes[kMaxFaceDetections];

static const FacePoint kArcfaceDst[kFaceDetectKeypointCount] = {
    {38.2946f, 51.6963f},
    {73.5318f, 51.5014f},
    {56.0252f, 71.7366f},
    {41.5493f, 92.3655f},
    {70.7299f, 92.2041f},
};

static void debugLogCallback(const char *s)
{
    BK_LOGI((char *)"TFLM", "%s", s);
}

void FaceRecognitionModel::LogEnable(bool enable)
{
    if (enable) {
        RegisterDebugLogCallback(debugLogCallback);
    } else {
        RegisterDebugLogCallback(NULL);
    }
}

/**
 * @brief True when 4bpp buffer should be read as B,G,R,X
 * @note ISP RGB888/BGR888 are stored as 4bpp. Vendor FaceRecognition and
 *       test_face_recognize both treat RGB888 4bpp as BGRX (not RGBX).
 * @param[in] pixel_format camera format enum
 * @param[in] layout_mode 0=auto, 1=force BGRX, 2=force RGBX
 */
static bool face_recog_src_is_bgrx(bk_pixel_format_t pixel_format, int layout_mode)
{
    if (layout_mode == 1) {
        return true;
    }
    if (layout_mode == 2) {
        return false;
    }
    (void)pixel_format;
    /* auto: match vendor bgrx_to_rgb_int8 for ISP 4bpp RGB path */
    return true;
}

/**
 * @brief Convert 4bpp camera buffer to model RGB int8 (value - 128)
 * @param[in] src_is_bgrx true: BGRX layout; false: RGBX layout
 */
static bool xrgb_to_rgb_int8(const uint8_t *src,
                             int8_t *dst,
                             uint32_t width,
                             uint32_t height,
                             bool src_is_bgrx)
{
    if (src == nullptr || dst == nullptr) {
        return false;
    }

    const uint32_t pixels = width * height;
    uint32_t i = 0;

#if defined(__ARM_FEATURE_MVE)
    for (; i + 16 <= pixels; i += 16) {
        static const uint8_t kMveRgbOffsets[16] = {
            0, 3, 6, 9, 12, 15, 18, 21,
            24, 27, 30, 33, 36, 39, 42, 45,
        };
        const uint8x16x4_t px = vld4q_u8(src + i * 4U);
        const uint8x16_t offset = vld1q_u8(kMveRgbOffsets);
        uint8_t *d = (uint8_t *)dst + i * 3U;
        if (src_is_bgrx) {
            /* B,G,R,X -> R,G,B */
            vstrbq_scatter_offset_u8(d, offset, vsubq_n_u8(px.val[2], 128));
            vstrbq_scatter_offset_u8(d + 1, offset, vsubq_n_u8(px.val[1], 128));
            vstrbq_scatter_offset_u8(d + 2, offset, vsubq_n_u8(px.val[0], 128));
        } else {
            /* R,G,B,X -> R,G,B */
            vstrbq_scatter_offset_u8(d, offset, vsubq_n_u8(px.val[0], 128));
            vstrbq_scatter_offset_u8(d + 1, offset, vsubq_n_u8(px.val[1], 128));
            vstrbq_scatter_offset_u8(d + 2, offset, vsubq_n_u8(px.val[2], 128));
        }
    }
#endif

    for (; i < pixels; i++) {
        const uint8_t *s = src + i * 4U;
        int8_t *d = dst + i * 3U;
        if (src_is_bgrx) {
            d[0] = (int8_t)((int)s[2] - 128);
            d[1] = (int8_t)((int)s[1] - 128);
            d[2] = (int8_t)((int)s[0] - 128);
        } else {
            d[0] = (int8_t)((int)s[0] - 128);
            d[1] = (int8_t)((int)s[1] - 128);
            d[2] = (int8_t)((int)s[2] - 128);
        }
    }
    return true;
}

static bool estimate_similarity_transform(const FacePoint *src,
                                          const FacePoint *dst,
                                          int n,
                                          float matrix[6])
{
    if (src == nullptr || dst == nullptr || matrix == nullptr || n < 2) {
        return false;
    }

    float src_cx = 0.0f;
    float src_cy = 0.0f;
    float dst_cx = 0.0f;
    float dst_cy = 0.0f;
    for (int i = 0; i < n; i++) {
        src_cx += src[i].x;
        src_cy += src[i].y;
        dst_cx += dst[i].x;
        dst_cy += dst[i].y;
    }
    src_cx /= (float)n;
    src_cy /= (float)n;
    dst_cx /= (float)n;
    dst_cy /= (float)n;

    float denom = 0.0f;
    float a_num = 0.0f;
    float b_num = 0.0f;
    for (int i = 0; i < n; i++) {
        const float xs = src[i].x - src_cx;
        const float ys = src[i].y - src_cy;
        const float xd = dst[i].x - dst_cx;
        const float yd = dst[i].y - dst_cy;
        denom += xs * xs + ys * ys;
        a_num += xs * xd + ys * yd;
        b_num += xs * yd - ys * xd;
    }
    if (denom <= 1e-10f) {
        return false;
    }

    const float a = a_num / denom;
    const float b = b_num / denom;
    matrix[0] = a;
    matrix[1] = -b;
    matrix[2] = dst_cx - a * src_cx + b * src_cy;
    matrix[3] = b;
    matrix[4] = a;
    matrix[5] = dst_cy - b * src_cx - a * src_cy;
    return true;
}

/**
 * @brief Warp 4bpp 320 source to RGB112 using inverse similarity matrix
 * @param[in] src_is_bgrx true: BGRX; false: RGBX
 */
static bool cpu_align_rgb112(const uint8_t *src,
                             int src_width,
                             int src_height,
                             const float matrix[6],
                             uint8_t *dst_rgb,
                             bool src_is_bgrx)
{
    if (src == nullptr || matrix == nullptr || dst_rgb == nullptr) {
        return false;
    }

    const float a = matrix[0];
    const float tx = matrix[2];
    const float b = matrix[3];
    const float ty = matrix[5];

    const float det = a * a + b * b;
    if (det <= 1e-10f) {
        return false;
    }
    const float inv_det = 1.0f / det;

    for (int yd = 0; yd < kAlignedFaceH; yd++) {
        for (int xd = 0; xd < kAlignedFaceW; xd++) {
            const float xdn = (float)xd - tx;
            const float ydn = (float)yd - ty;
            /* inverse of [a -b; b a] */
            const float xs = (a * xdn + b * ydn) * inv_det;
            const float ys = (-b * xdn + a * ydn) * inv_det;

            int xi = (int)(xs + 0.5f);
            int yi = (int)(ys + 0.5f);
            uint8_t *d = dst_rgb + (yd * kAlignedFaceW + xd) * kAlignedFaceC;
            if (xi < 0 || yi < 0 || xi >= src_width || yi >= src_height) {
                d[0] = 0;
                d[1] = 0;
                d[2] = 0;
                continue;
            }
            const uint8_t *s = src + ((yi * src_width + xi) * 4);
            if (src_is_bgrx) {
                d[0] = s[2];
                d[1] = s[1];
                d[2] = s[0];
            } else {
                d[0] = s[0];
                d[1] = s[1];
                d[2] = s[2];
            }
        }
    }
    return true;
}

FaceRecognitionModel::FaceRecognitionModel()
    : detector_(new FaceDetectionRuntime()),
      verifier_(new FaceVerifyRuntime()),
      detect_model_file_path_(nullptr),
      verify_model_file_path_(nullptr),
      box_callback_(nullptr),
      verify_result_callback_(nullptr),
      enroll_result_callback_(nullptr),
      verify_enabled_(false),
      runtimes_initialized_(false),
      input_layout_mode_(0)
{
}

FaceRecognitionModel::~FaceRecognitionModel()
{
    deinit();
    delete detector_;
    detector_ = nullptr;
    delete verifier_;
    verifier_ = nullptr;
}

void FaceRecognitionModel::setDetectModelFilePath(const char *path)
{
    detect_model_file_path_ = path;
    if (detector_ != nullptr) {
        detector_->setModelFilePath(path);
    }
}

void FaceRecognitionModel::setVerifyModelFilePath(const char *path)
{
    verify_model_file_path_ = path;
    if (verifier_ != nullptr) {
        verifier_->setModelFilePath(path);
    }
}

void FaceRecognitionModel::setBoxCallback(faceBoxCallbackT cb)
{
    box_callback_ = cb;
}

void FaceRecognitionModel::setVerifyResultCallback(faceVerifyResultCallbackT cb)
{
    verify_result_callback_ = cb;
}

void FaceRecognitionModel::setEnrollResultCallback(faceEnrollResultCallbackT cb)
{
    enroll_result_callback_ = cb;
}

void FaceRecognitionModel::setVerifyEnabled(bool enable)
{
    verify_enabled_ = enable;
}

void FaceRecognitionModel::setInputLayoutMode(int mode)
{
    if (mode < 0 || mode > 2) {
        mode = 0;
    }
    input_layout_mode_ = mode;
}

void FaceRecognitionModel::setDetectThreshold(float thr)
{
    if (detector_ != nullptr) {
        detector_->setDetectThreshold(thr);
    }
}

void FaceRecognitionModel::getLastDetectDebug(float *max_score, int *pass_count) const
{
    if (max_score != nullptr) {
        *max_score = (detector_ != nullptr) ? detector_->lastMaxScore() : -1.0f;
    }
    if (pass_count != nullptr) {
        *pass_count = (detector_ != nullptr) ? detector_->lastPassCount() : 0;
    }
}

int FaceRecognitionModel::init(void)
{
    if (runtimes_initialized_) {
        return 0;
    }

    if (detect_model_file_path_ == nullptr || detect_model_file_path_[0] == '\0') {
        detect_model_file_path_ = FACE_RECOG_DETECT_MODEL_SD_PATH;
    }
    if (verify_model_file_path_ == nullptr || verify_model_file_path_[0] == '\0') {
        verify_model_file_path_ = FACE_RECOG_VERIFY_MODEL_SD_PATH;
    }
    if (detector_ != nullptr) {
        detector_->setModelFilePath(detect_model_file_path_);
    }
    if (verifier_ != nullptr) {
        verifier_->setModelFilePath(verify_model_file_path_);
    }

    if (detector_ == nullptr || !detector_->init()) {
        MicroPrintf("FaceRecognition detector runtime init failed\r\n");
        return -1;
    }
    if (verifier_ == nullptr || !verifier_->init()) {
        MicroPrintf("FaceRecognition verify runtime init failed\r\n");
        detector_->deinit();
        return -1;
    }

    runtimes_initialized_ = true;
    MicroPrintf("FaceRecognitionModel init detect=%s verify=%s\r\n",
                detect_model_file_path_, verify_model_file_path_);
    return 0;
}

int FaceRecognitionModel::deinit(void)
{
    releaseDetectorInterpreter();
    if (detector_ != nullptr) {
        detector_->deinit();
    }
    if (verifier_ != nullptr) {
        verifier_->deinit();
    }
    runtimes_initialized_ = false;
    return 0;
}

void FaceRecognitionModel::releaseDetectorInterpreter(void)
{
    if (detector_ != nullptr) {
        detector_->releaseInterpreter();
    }
}

bool FaceRecognitionModel::ensureDetectorInterpreter(void)
{
    if (detector_ == nullptr) {
        return false;
    }
    return detector_->prepareInterpreter() != nullptr;
}

int FaceRecognitionModel::run(uint8_t *data, uint32_t size, bk_pixel_format_t pixel_format)
{
#define FACE_RECOG_NOTIFY_ENROLL_FAIL()                            \
    do {                                                           \
        if (verify_enabled_ && enroll_result_callback_ != nullptr) { \
            enroll_result_callback_(nullptr, nullptr, 0);          \
        }                                                          \
    } while (0)

    const uint32_t expect = (uint32_t)kDetectInputW * kDetectInputH * 4U;
    const bool src_is_bgrx = face_recog_src_is_bgrx(pixel_format, input_layout_mode_);

    if (data == nullptr || size != expect || detector_ == nullptr || verifier_ == nullptr) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        return 0;
    }
    if (!runtimes_initialized_ && init() != 0) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        return 0;
    }
    if (!ensureDetectorInterpreter()) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        return 0;
    }

    tflite::MicroInterpreter *interpreter = detector_->prepareInterpreter();
    if (interpreter == nullptr) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        return 0;
    }

    TfLiteTensor *input = interpreter->input(0);
    if (input == nullptr || input->type != kTfLiteInt8) {
        MicroPrintf("FaceRecognition invalid detector input tensor\r\n");
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        return 0;
    }

    static bool s_fmt_logged = false;
    if (!s_fmt_logged) {
        uint32_t sum = 0;
        const uint32_t sample_n = 64;
        for (uint32_t i = 0; i < sample_n; i++) {
            const uint32_t off = (i * 503U) % (expect / 4U);
            sum += data[off * 4U] + data[off * 4U + 1U] + data[off * 4U + 2U];
        }
        MicroPrintf("FaceRecognition input fmt=%u layout=%s mode=%d mean~%u (4bpp)\r\n",
                    (unsigned)pixel_format,
                    src_is_bgrx ? "BGRX" : "RGBX",
                    input_layout_mode_,
                    (unsigned)(sum / (sample_n * 3U)));
        if (sum == 0U) {
            MicroPrintf("FaceRecognition warn: sampled pixels all zero\r\n");
        }
        s_fmt_logged = true;
    }

    if (!xrgb_to_rgb_int8(data, input->data.int8, kDetectInputW, kDetectInputH, src_is_bgrx)) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        return 0;
    }

    FaceDetection *detections = (FaceDetection *)bk_frame_buffer_malloc(
        MEM_SLAB_HEAP_UNCODED, sizeof(FaceDetection) * kMaxFaceDetections);
    if (detections == nullptr) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        return 0;
    }

    const int det_count = detector_->invokeAndDecode(interpreter, detections, kMaxFaceDetections);
    if (det_count <= 0) {
        if (box_callback_ != nullptr) {
            box_callback_(nullptr, 0);
        }
        bk_frame_buffer_free(detections);
        return 0;
    }

    int box_count = 0;
    for (int i = 0; i < det_count; i++) {
        float x1 = detections[i].x1;
        float y1 = detections[i].y1;
        float x2 = detections[i].x2;
        float y2 = detections[i].y2;
        float bw;
        float bh;
        if (x1 < 0.0f) x1 = 0.0f;
        if (y1 < 0.0f) y1 = 0.0f;
        if (x2 > (float)kDetectInputW) x2 = (float)kDetectInputW;
        if (y2 > (float)kDetectInputH) y2 = (float)kDetectInputH;
        bw = x2 - x1;
        bh = y2 - y1;
        /* Drop tiny boxes: SP 320 full-FOV often yields <30px noise */
        if (bw < 32.0f || bh < 32.0f) {
            continue;
        }
        s_face_out_boxes[box_count].x = x1;
        s_face_out_boxes[box_count].y = y1;
        s_face_out_boxes[box_count].w = bw;
        s_face_out_boxes[box_count].h = bh;
        s_face_out_boxes[box_count].score = detections[i].score;
        box_count++;
    }
    if (box_callback_ != nullptr) {
        box_callback_(box_count > 0 ? s_face_out_boxes : nullptr, box_count);
    }

    /* detect-only: skip align / feature extract */
    if (!verify_enabled_) {
        bk_frame_buffer_free(detections);
        return box_count;
    }
    if (box_count <= 0) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        bk_frame_buffer_free(detections);
        return 0;
    }

    /* Prefer first kept (size-filtered) face; map back via score/box match on detections */
    int best_i = 0;
    for (int i = 0; i < det_count; i++) {
        float x1 = detections[i].x1;
        float y1 = detections[i].y1;
        float x2 = detections[i].x2;
        float y2 = detections[i].y2;
        if (x1 < 0.0f) x1 = 0.0f;
        if (y1 < 0.0f) y1 = 0.0f;
        if (x2 > (float)kDetectInputW) x2 = (float)kDetectInputW;
        if (y2 > (float)kDetectInputH) y2 = (float)kDetectInputH;
        if ((x2 - x1) >= 32.0f && (y2 - y1) >= 32.0f) {
            best_i = i;
            break;
        }
    }

    float affine[6];
    if (!estimate_similarity_transform(detections[best_i].keypoints,
                                       kArcfaceDst,
                                       kFaceDetectKeypointCount,
                                       affine)) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        bk_frame_buffer_free(detections);
        return 0;
    }

    uint8_t *aligned_rgb = (uint8_t *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED,
                                                             kAlignedFaceBytes);
    if (aligned_rgb == nullptr) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        bk_frame_buffer_free(detections);
        return 0;
    }

    if (!cpu_align_rgb112(data, kDetectInputW, kDetectInputH, affine, aligned_rgb, src_is_bgrx)) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        bk_frame_buffer_free(aligned_rgb);
        bk_frame_buffer_free(detections);
        return 0;
    }

    FaceVerifyResult *result = (FaceVerifyResult *)bk_frame_buffer_malloc(
        MEM_SLAB_HEAP_UNCODED, sizeof(FaceVerifyResult));
    if (result == nullptr) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        bk_frame_buffer_free(aligned_rgb);
        bk_frame_buffer_free(detections);
        return 0;
    }
    os_memset(result, 0, sizeof(FaceVerifyResult));

    releaseDetectorInterpreter();
    bool verify_ok = verifier_->extract(aligned_rgb, kAlignedFaceBytes, result);
    verifier_->releaseInterpreter();

    if (!verify_ok) {
        FACE_RECOG_NOTIFY_ENROLL_FAIL();
        bk_frame_buffer_free(detections);
        bk_frame_buffer_free(aligned_rgb);
        bk_frame_buffer_free(result);
        return 0;
    }

    if (verify_enabled_ && enroll_result_callback_ != nullptr) {
        enroll_result_callback_(result, aligned_rgb, kAlignedFaceBytes);
    }
    if (verify_result_callback_ != nullptr) {
        verify_result_callback_(result);
    }

    bk_frame_buffer_free(detections);
    bk_frame_buffer_free(aligned_rgb);
    bk_frame_buffer_free(result);
#undef FACE_RECOG_NOTIFY_ENROLL_FAIL
    return box_count > 0 ? box_count : 1;
}
