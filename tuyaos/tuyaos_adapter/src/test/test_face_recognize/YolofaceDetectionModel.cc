/**
 * @file YolofaceDetectionModel.cc
 * @brief YoloFace post-processing for `yoloface_int8_vela.tflite` (NXP).
 */

#include "YolofaceDetectionModel.h"
#include "yoloface_detect_model_data.h"
#include "box.h"

#include <math.h>

constexpr int kGridSize = 7;
constexpr int kStride = 8;
constexpr int kNumAnchors = 3;
constexpr int kValuesPerAnchor = 6;
constexpr int kChannelsPerCell = kNumAnchors * kValuesPerAnchor;

static const float kAnchors[kNumAnchors][2] = {
    {9.0f, 14.0f},
    {12.0f, 17.0f},
    {22.0f, 21.0f},
};

constexpr float kObjScoreThreshold = 0.75f;
constexpr float kNmsIoUThreshold = 0.45f;
constexpr int kMaxCandidates = 32;

static inline float sigmoid(float x)
{
    if (x > 10.0f) return 1.0f;
    if (x < -10.0f) return 0.0f;
    return 1.0f / (1.0f + expf(-x));
}

static float box_iou(const Box &a, const Box &b)
{
    float ax2 = a.x + a.w;
    float ay2 = a.y + a.h;
    float bx2 = b.x + b.w;
    float by2 = b.y + b.h;

    float ix1 = (a.x > b.x) ? a.x : b.x;
    float iy1 = (a.y > b.y) ? a.y : b.y;
    float ix2 = (ax2 < bx2) ? ax2 : bx2;
    float iy2 = (ay2 < by2) ? ay2 : by2;

    float iw = ix2 - ix1;
    if (iw <= 0.0f) return 0.0f;
    float ih = iy2 - iy1;
    if (ih <= 0.0f) return 0.0f;

    float inter = iw * ih;
    float uni = a.w * a.h + b.w * b.h - inter;
    return (uni > 0.0f) ? (inter / uni) : 0.0f;
}

static int nms_in_place(Box *boxes, int count, float iou_thresh)
{
    for (int i = 1; i < count; i++) {
        Box key = boxes[i];
        int j = i - 1;
        while (j >= 0 && boxes[j].score < key.score) {
            boxes[j + 1] = boxes[j];
            j--;
        }
        boxes[j + 1] = key;
    }

    bool keep[kMaxCandidates];
    for (int i = 0; i < count; i++) {
        keep[i] = true;
    }

    for (int i = 0; i < count; i++) {
        if (!keep[i]) {
            continue;
        }
        for (int j = i + 1; j < count; j++) {
            if (!keep[j]) {
                continue;
            }
            if (box_iou(boxes[i], boxes[j]) > iou_thresh) {
                keep[j] = false;
            }
        }
    }

    int n = 0;
    for (int i = 0; i < count; i++) {
        if (keep[i]) {
            if (n != i) {
                boxes[n] = boxes[i];
            }
            n++;
        }
    }

    return n;
}

void YolofaceDetectionModel::resolverLoad(void)
{
    micro_op_resolver.AddEthosU();
    micro_op_resolver.AddPadV2();
    micro_op_resolver.AddTranspose();
}

void YolofaceDetectionModel::resourceLoad(void)
{
    name = "yolofaceDetection";
    width = 56;
    height = 56;
    format = BK_PIXEL_FORMAT_RGB888;

    model_type = AVDK_NN_MODEL_TYPE_NPU;
    model_ram_type = AVDK_NN_MEM_TYPE_FALSH;
    model_flash_data = (uint8_t *)g_yoloface_int8_vela_tflite;
    model_flash_data_size = YOLOFACE_MODEL_DATA_SIZE;
    model_data = (uint8_t *)g_yoloface_int8_vela_tflite;
    model_data_size = YOLOFACE_MODEL_DATA_SIZE;

    fast_ram_type = AVDK_NN_MEM_TYPE_HSRAM;
    fast_ram_data_size = 40 * 1024;
    fast_ram_data = NULL;

    arena_data_size = 80 * 1024;
    arena_ram_type = AVDK_NN_MEM_TYPE_HSRAM;
    arena_ram_data = NULL;
}

void YolofaceDetectionModel::resourceUnload(void)
{
}

int YolofaceDetectionModel::run(uint8_t *data, uint32_t size, bk_pixel_format_t format)
{
    uint32_t expected_size = 0;

    if (format == BK_PIXEL_FORMAT_RGB888) {
        expected_size = (uint32_t)width * (uint32_t)height * 3U;
    } else if (format == BK_PIXEL_FORMAT_BGRA8888) {
        expected_size = (uint32_t)width * (uint32_t)height * 4U;
    } else {
        MicroPrintf("YolofaceDetection: unsupported pixel format %u\r\n", (unsigned)format);
        return 0;
    }

    if (size != expected_size) {
        MicroPrintf("YolofaceDetection: invalid size %u, expected %u\r\n",
                    (unsigned)size, (unsigned)expected_size);
        return 0;
    }

    TfLiteTensor *input = pinterpreter->input(0);
    if (format == BK_PIXEL_FORMAT_BGRA8888) {
        for (int i = 0; i < width * height; i++) {
            input->data.int8[i * 3 + 0] = (int8_t)((int)data[i * 4 + 2] - 128);
            input->data.int8[i * 3 + 1] = (int8_t)((int)data[i * 4 + 1] - 128);
            input->data.int8[i * 3 + 2] = (int8_t)((int)data[i * 4 + 0] - 128);
        }
    } else {
        for (int i = 0; i < width * height; i++) {
            input->data.int8[i * 3 + 0] = (int8_t)((int)data[i * 3 + 0] - 128);
            input->data.int8[i * 3 + 1] = (int8_t)((int)data[i * 3 + 1] - 128);
            input->data.int8[i * 3 + 2] = (int8_t)((int)data[i * 3 + 2] - 128);
        }
    }

    if (kTfLiteOk != pinterpreter->Invoke()) {
        MicroPrintf("YolofaceDetection: invoke failed\r\n");
        return 0;
    }

    TfLiteTensor *output = pinterpreter->output(0);
    const float out_scale = output->params.scale;
    const int32_t out_zp = output->params.zero_point;

    static bool s_shape_logged = false;
    if (!s_shape_logged) {
        MicroPrintf("YolofaceDetection: output dims=%d shape=[", output->dims->size);
        for (int i = 0; i < output->dims->size; i++) {
            MicroPrintf("%d%s", output->dims->data[i],
                        (i == output->dims->size - 1) ? "" : ", ");
        }
        MicroPrintf("] scale=%.6f zp=%ld\r\n", out_scale, (long)out_zp);
        MicroPrintf("YolofaceDetection: expecting [1, %d, %d, %d] (%d-cell grid, %d anchors, %d vals)\r\n",
                    kGridSize, kGridSize, kChannelsPerCell,
                    kGridSize * kGridSize, kNumAnchors, kValuesPerAnchor);
        s_shape_logged = true;
    }

    int total_elems = 1;
    for (int i = 0; i < output->dims->size; i++) {
        total_elems *= output->dims->data[i];
    }

    int expected_elems = kGridSize * kGridSize * kChannelsPerCell;
    if (total_elems != expected_elems) {
        MicroPrintf("YolofaceDetection: output element count=%d, expected %d -- model mismatch?\r\n",
                    total_elems, expected_elems);
        box_detection_path_clear();
        return 0;
    }

    Box candidates[kMaxCandidates];
    int n_cand = 0;
    int n_obj_pass = 0;
    const int8_t *out_int8 = output->data.int8;

    for (int gy = 0; gy < kGridSize; gy++) {
        for (int gx = 0; gx < kGridSize; gx++) {
            for (int a = 0; a < kNumAnchors; a++) {
                const int base = (gy * kGridSize + gx) * kChannelsPerCell + a * kValuesPerAnchor;

                const float raw_obj = ((float)out_int8[base + 4] - (float)out_zp) * out_scale;
                const float obj = sigmoid(raw_obj);
                if (obj < kObjScoreThreshold) {
                    continue;
                }
                n_obj_pass++;

                if (n_cand >= kMaxCandidates) {
                    continue;
                }

                const float raw_cx = ((float)out_int8[base + 0] - (float)out_zp) * out_scale;
                const float raw_cy = ((float)out_int8[base + 1] - (float)out_zp) * out_scale;
                const float raw_w = ((float)out_int8[base + 2] - (float)out_zp) * out_scale;
                const float raw_h = ((float)out_int8[base + 3] - (float)out_zp) * out_scale;

                const float cx = (sigmoid(raw_cx) + (float)gx) * (float)kStride;
                const float cy = (sigmoid(raw_cy) + (float)gy) * (float)kStride;
                const float w = expf(raw_w) * kAnchors[a][0];
                const float h = expf(raw_h) * kAnchors[a][1];

                float x1 = cx - w * 0.5f;
                float y1 = cy - h * 0.5f;
                float x2 = cx + w * 0.5f;
                float y2 = cy + h * 0.5f;

                if (x1 < 0.0f) x1 = 0.0f;
                if (y1 < 0.0f) y1 = 0.0f;
                if (x2 > (float)this->width) x2 = (float)this->width;
                if (y2 > (float)this->height) y2 = (float)this->height;

                const float cw = x2 - x1;
                const float ch = y2 - y1;
                if (cw <= 0.0f || ch <= 0.0f) {
                    continue;
                }

                candidates[n_cand].x = x1;
                candidates[n_cand].y = y1;
                candidates[n_cand].w = cw;
                candidates[n_cand].h = ch;
                candidates[n_cand].score = obj;
                n_cand++;
            }
        }
    }

    if (n_cand == 0) {
        if (n_obj_pass == 0) {
            MicroPrintf("YolofaceDetection: no candidates above obj=%.2f\r\n", kObjScoreThreshold);
        }
        onBoxDetectionCallback(NULL, 0);
        return 0;
    }

    int n_kept = nms_in_place(candidates, n_cand, kNmsIoUThreshold);

    MicroPrintf("YolofaceDetection: cand=%d (obj_pass=%d, cap=%d), nms_kept=%d\r\n",
                n_cand, n_obj_pass, kMaxCandidates, n_kept);
    for (int i = 0; i < n_kept; i++) {
        MicroPrintf("  [%d] xywh=(%.1f, %.1f, %.1fx%.1f)  obj=%.3f\r\n",
                    i, candidates[i].x, candidates[i].y,
                    candidates[i].w, candidates[i].h, candidates[i].score);
    }

    onBoxDetectionCallback(candidates, n_kept);

    return n_kept;
}
