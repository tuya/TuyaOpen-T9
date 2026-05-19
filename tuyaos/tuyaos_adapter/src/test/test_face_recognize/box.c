#include <common/bk_err.h>
#include <common/bk_include.h>
#include <driver/int.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>

#include <avdk_check.h>
#include <components/bk_gpu.h>

#include "box.h"

#define TAG "box"

#define LOGI(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)
#define LOGD(...) BK_LOGD(TAG, ##__VA_ARGS__)

extern int tkl_gpu_get_handle(void **handle) __attribute__((weak));

__attribute__((weak)) bk_gpu_ctlr_handle_t app_gpu_handle_get(void)
{
    void *gpu_handle = NULL;

    if (tkl_gpu_get_handle != NULL && tkl_gpu_get_handle(&gpu_handle) == 0) {
        return (bk_gpu_ctlr_handle_t)gpu_handle;
    }

    return NULL;
}

static inline void rotate_point(int rot, float src_w, float src_h,
                                float x, float y, float *rx, float *ry)
{
    switch (rot) {
        case 90:
            *rx = src_h - y;
            *ry = x;
            break;
        case 180:
            *rx = src_w - x;
            *ry = src_h - y;
            break;
        case 270:
            *rx = y;
            *ry = src_w - x;
            break;
        case 0:
        default:
            *rx = x;
            *ry = y;
            break;
    }
}

int box_detection_path_build(Box *boxes, int count, int buffer_count, int rotate,
                             int src_width, int src_height,
                             int dst_width, int dst_height)
{
    bk_gpu_ctlr_handle_t gpu_handle = app_gpu_handle_get();

    if (gpu_handle == NULL) {
        return 0;
    }

    if (boxes == NULL || buffer_count <= 0 ||
        src_width <= 0 || src_height <= 0 ||
        dst_width <= 0 || dst_height <= 0) {
        LOGE("box_detection_path_build: bad args\n");
        return -1;
    }

    if (count < 0) {
        count = 0;
    }
    if (count > buffer_count) {
        count = buffer_count;
    }

    int rot = ((rotate % 360) + 360) % 360;
    if (rot != 0 && rot != 90 && rot != 180 && rot != 270) {
        LOGW("box_detection_path_build: unsupported rotate=%d, fallback to 0\n", rotate);
        rot = 0;
    }

    int rot_w = (rot == 90 || rot == 270) ? src_height : src_width;
    int rot_h = (rot == 90 || rot == 270) ? src_width : src_height;

    float xscale = (float)dst_width / (float)rot_w;
    float yscale = (float)dst_height / (float)rot_h;

    uint8_t cmd_buff[buffer_count * 6 + 1];
    int16_t dat_buff[buffer_count * 10];
    uint8_t *pcmd = cmd_buff;
    int16_t *pdat = dat_buff;

    const float src_w_f = (float)src_width;
    const float src_h_f = (float)src_height;

    for (int i = 0; i < count; i++) {
        float rx0;
        float ry0;
        float rx1;
        float ry1;
        float fxmin;
        float fymin;
        float fxmax;
        float fymax;
        int xmin;
        int ymin;
        int xmax;
        int ymax;

        rotate_point(rot, src_w_f, src_h_f, boxes[i].x, boxes[i].y, &rx0, &ry0);
        rotate_point(rot, src_w_f, src_h_f,
                     boxes[i].x + boxes[i].w, boxes[i].y + boxes[i].h,
                     &rx1, &ry1);

        fxmin = rx0 * xscale;
        fymin = ry0 * yscale;
        fxmax = rx1 * xscale;
        fymax = ry1 * yscale;

        if (fxmin > fxmax) {
            float t = fxmin;
            fxmin = fxmax;
            fxmax = t;
        }
        if (fymin > fymax) {
            float t = fymin;
            fymin = fymax;
            fymax = t;
        }
        if (fxmin < 0.0f) fxmin = 0.0f;
        if (fymin < 0.0f) fymin = 0.0f;
        if (fxmax > (float)dst_width) fxmax = (float)dst_width;
        if (fymax > (float)dst_height) fymax = (float)dst_height;

        xmin = (int)(fxmin + 0.5f);
        ymin = (int)(fymin + 0.5f);
        xmax = (int)(fxmax + 0.5f);
        ymax = (int)(fymax + 0.5f);

        LOGI("box_detection_path_build[%d]: rot=%d src=%dx%d dst=%dx%d -> (%d,%d)-(%d,%d)\n",
             i, rot, src_width, src_height, dst_width, dst_height,
             xmin, ymin, xmax, ymax);

        DRAW_RECTANGLE_PATH_BUILD(pcmd, pdat, xmin, ymin, xmax, ymax);
    }

    *pcmd = VLC_OP_END;

    bk_gpu_draw_path_set_t path_set = {
        .cmd = cmd_buff,
        .data = dat_buff,
        .size = (uint32_t)count * 6 + 1,
        .color = 0x00FF00,
    };

    bk_gpu_flex_overlay_path_build(gpu_handle, &path_set);

    return 0;
}

void box_detection_path_clear(void)
{
    bk_gpu_ctlr_handle_t gpu_handle = app_gpu_handle_get();

    if (gpu_handle == NULL) {
        return;
    }

    bk_gpu_flex_overlay_path_clear(gpu_handle);
}
