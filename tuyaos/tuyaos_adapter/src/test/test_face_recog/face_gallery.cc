/**
 * @file face_gallery.cc
 * @brief SD face gallery implementation via tkl_fs
 * @version 1.0
 * @date 2026-08-25
 * @copyright Copyright (c) Tuya Inc.
 */
#include "face_gallery.h"

#include <stdio.h>
#include <string.h>

#include "components/bk_frame_buffer.h"
#include "os/mem.h"
#include "os/str.h"
#include "tkl_fs.h"
#include "tuya_cloud_types.h"

extern "C" {
extern INT_T test_fs_mount(CONST CHAR_T *path, FS_DEV_TYPE_T dev_type);
}

#ifndef TEST_SDCARD_MOUNT_POINT
#define TEST_SDCARD_MOUNT_POINT "/sdcard"
#endif

#define FACE_GALLERY_PATH_LEN 96
#define FACE_GALLERY_MAX_ID   9999
#define FACE_GALLERY_DIR_PREFIX "face_"

static uint32_t s_enroll_profile_id = 0;
static uint32_t s_enroll_sample_index = 0;

static int face_gallery_write_file(const char *path, const void *data, uint32_t size)
{
    TUYA_FILE f;
    INT_T bw;

    if (path == NULL || data == NULL || size == 0) {
        return -1;
    }
    f = tkl_fopen(path, "wb");
    if (f == NULL) {
        bk_printf("[xrecog] write open %s failed\r\n", path);
        return -1;
    }
    bw = tkl_fwrite((void *)data, (INT_T)size, f);
    tkl_fclose(f);
    if (bw != (INT_T)size) {
        bk_printf("[xrecog] write %s failed bw=%d size=%u\r\n", path, bw, (unsigned)size);
        return -1;
    }
    return 0;
}

static int face_gallery_read_file(const char *path, void *data, uint32_t size)
{
    TUYA_FILE f;
    INT_T br;

    if (path == NULL || data == NULL || size == 0) {
        return -1;
    }
    f = tkl_fopen(path, "rb");
    if (f == NULL) {
        return -1;
    }
    br = tkl_fread(data, (INT_T)size, f);
    tkl_fclose(f);
    return (br == (INT_T)size) ? 0 : -1;
}

static bool face_gallery_parse_face_dir_id(const char *name, uint32_t *out_id)
{
    uint32_t id = 0;
    const char *p;

    if (name == NULL || out_id == NULL) {
        return false;
    }
    if (strncmp(name, FACE_GALLERY_DIR_PREFIX, 5) != 0) {
        return false;
    }
    p = name + 5;
    if (*p == '\0') {
        return false;
    }
    while (*p != '\0') {
        if (*p < '0' || *p > '9') {
            return false;
        }
        id = id * 10U + (uint32_t)(*p - '0');
        if (id > FACE_GALLERY_MAX_ID) {
            return false;
        }
        p++;
    }
    *out_id = id;
    return true;
}

static bool face_gallery_parse_feature_id(const char *name, uint32_t *out_id)
{
    const char prefix[] = "feature_";
    const char suffix[] = ".bin";
    int len = 0;
    int prefix_len = (int)(sizeof(prefix) - 1);
    int suffix_len = (int)(sizeof(suffix) - 1);
    uint32_t id = 0;
    const char *p;

    if (name == NULL || out_id == NULL || strncmp(name, prefix, (size_t)prefix_len) != 0) {
        return false;
    }
    while (name[len] != '\0') {
        len++;
    }
    if (len <= prefix_len + suffix_len ||
        strcmp(name + len - suffix_len, suffix) != 0) {
        return false;
    }
    p = name + prefix_len;
    if (p >= name + len - suffix_len) {
        return false;
    }
    while (p < name + len - suffix_len) {
        if (*p < '0' || *p > '9') {
            return false;
        }
        id = id * 10U + (uint32_t)(*p - '0');
        p++;
    }
    *out_id = id;
    return true;
}

static int face_gallery_count_features(const char *dir_path, uint32_t *out_count)
{
    TUYA_DIR dir = NULL;
    TUYA_FILEINFO info = NULL;
    CONST CHAR_T *name = NULL;
    BOOL_T is_dir = FALSE;
    uint32_t count = 0;
    uint32_t sid = 0;

    if (out_count == NULL) {
        return -1;
    }
    *out_count = 0;
    if (tkl_dir_open(dir_path, &dir) != 0) {
        return -1;
    }
    while (tkl_dir_read(dir, &info) == 0) {
        if (tkl_dir_name(info, &name) != 0 || name == NULL) {
            continue;
        }
        if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
            continue;
        }
        if (tkl_dir_is_directory(info, &is_dir) == 0 && is_dir) {
            continue;
        }
        if (face_gallery_parse_feature_id(name, &sid)) {
            count++;
        }
    }
    tkl_dir_close(dir);
    *out_count = count;
    return 0;
}

static int face_gallery_find_next_profile_id(uint32_t *out_profile_id)
{
    TUYA_DIR dir = NULL;
    TUYA_FILEINFO info = NULL;
    CONST CHAR_T *name = NULL;
    BOOL_T is_dir = FALSE;
    uint32_t max_id = 0;
    uint32_t id = 0;

    if (out_profile_id == NULL) {
        return -1;
    }
    if (tkl_dir_open(FACE_GALLERY_DIR_BASE, &dir) != 0) {
        *out_profile_id = 1;
        return 0;
    }
    while (tkl_dir_read(dir, &info) == 0) {
        if (tkl_dir_name(info, &name) != 0 || name == NULL) {
            continue;
        }
        if (tkl_dir_is_directory(info, &is_dir) != 0 || !is_dir) {
            continue;
        }
        if (face_gallery_parse_face_dir_id(name, &id) && id > max_id) {
            max_id = id;
        }
    }
    tkl_dir_close(dir);
    *out_profile_id = (max_id == 0) ? 1 : (max_id + 1);
    return 0;
}

static int face_gallery_pick_slot(uint32_t *out_profile_id, uint32_t *out_sample_index)
{
    uint32_t profile_id = 0;
    char dir_path[FACE_GALLERY_PATH_LEN];
    uint32_t feat_count = 0;

    if (out_profile_id == NULL || out_sample_index == NULL) {
        return -1;
    }

    if (s_enroll_profile_id != 0 &&
        s_enroll_sample_index < FACE_GALLERY_SAMPLES_PER_ID) {
        *out_profile_id = s_enroll_profile_id;
        *out_sample_index = s_enroll_sample_index + 1;
        return 0;
    }

    if (face_gallery_find_next_profile_id(&profile_id) != 0) {
        return -1;
    }

    if (profile_id > 1) {
        uint32_t last_profile = profile_id - 1;
        int n = snprintf(dir_path, sizeof(dir_path), "%s/face_%04u",
                         FACE_GALLERY_DIR_BASE, (unsigned)last_profile);
        if (n > 0 && n < (int)sizeof(dir_path) &&
            face_gallery_count_features(dir_path, &feat_count) == 0 &&
            feat_count < FACE_GALLERY_SAMPLES_PER_ID) {
            *out_profile_id = last_profile;
            *out_sample_index = feat_count + 1;
            return 0;
        }
    }

    *out_profile_id = profile_id;
    *out_sample_index = 1;
    return 0;
}

static int face_gallery_remove_dir(const char *dir_path)
{
    TUYA_DIR dir = NULL;
    TUYA_FILEINFO info = NULL;
    CONST CHAR_T *name = NULL;
    BOOL_T is_dir = FALSE;
    char file_path[FACE_GALLERY_PATH_LEN];

    if (dir_path == NULL) {
        return -1;
    }
    if (tkl_dir_open(dir_path, &dir) != 0) {
        return 0;
    }
    while (tkl_dir_read(dir, &info) == 0) {
        if (tkl_dir_name(info, &name) != 0 || name == NULL) {
            continue;
        }
        if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
            continue;
        }
        if (tkl_dir_is_directory(info, &is_dir) == 0 && is_dir) {
            continue;
        }
        if (snprintf(file_path, sizeof(file_path), "%s/%s", dir_path, name) > 0) {
            (void)tkl_fs_remove(file_path);
        }
    }
    tkl_dir_close(dir);
    (void)tkl_fs_remove(dir_path);
    return 0;
}

int face_gallery_prepare(void)
{
    if (test_fs_mount(TEST_SDCARD_MOUNT_POINT, DEV_SDCARD) != 0) {
        bk_printf("[xrecog] mount %s failed\r\n", TEST_SDCARD_MOUNT_POINT);
        return -1;
    }
    if (tkl_fs_mkdir(FACE_GALLERY_DIR_BASE) != 0) {
        /* may already exist */
    }
    return 0;
}

int face_gallery_enroll_save(const FaceVerifyResult *result,
                             const uint8_t *aligned_rgb,
                             uint32_t aligned_rgb_size,
                             uint32_t *out_saved_count,
                             uint32_t *out_profile_id)
{
    uint32_t profile_id = 0;
    uint32_t sample_index = 0;
    char dir_path[FACE_GALLERY_PATH_LEN];
    char file_path[FACE_GALLERY_PATH_LEN];
    char ppm_header[32];
    int n;
    TUYA_FILE f;
    INT_T bw;

    if (result == NULL || !result->valid || aligned_rgb == NULL ||
        aligned_rgb_size != (112U * 112U * 3U) ||
        out_saved_count == NULL || out_profile_id == NULL) {
        return -1;
    }
    if (face_gallery_prepare() != 0) {
        return -1;
    }
    if (face_gallery_pick_slot(&profile_id, &sample_index) != 0) {
        return -1;
    }

    n = snprintf(dir_path, sizeof(dir_path), "%s/face_%04u",
                 FACE_GALLERY_DIR_BASE, (unsigned)profile_id);
    if (n <= 0 || n >= (int)sizeof(dir_path)) {
        return -1;
    }
    (void)tkl_fs_mkdir(dir_path);

    n = snprintf(file_path, sizeof(file_path), "%s/face_%04u.ppm",
                 dir_path, (unsigned)sample_index);
    if (n <= 0 || n >= (int)sizeof(file_path)) {
        return -1;
    }
    f = tkl_fopen(file_path, "wb");
    if (f == NULL) {
        bk_printf("[xrecog] open %s failed\r\n", file_path);
        return -1;
    }
    n = snprintf(ppm_header, sizeof(ppm_header), "P6\n112 112\n255\n");
    bw = tkl_fwrite(ppm_header, n, f);
    if (bw != n) {
        tkl_fclose(f);
        return -1;
    }
    bw = tkl_fwrite((void *)aligned_rgb, (INT_T)aligned_rgb_size, f);
    tkl_fclose(f);
    if (bw != (INT_T)aligned_rgb_size) {
        return -1;
    }

    n = snprintf(file_path, sizeof(file_path), "%s/feature_%04u.bin",
                 dir_path, (unsigned)sample_index);
    if (n <= 0 || n >= (int)sizeof(file_path)) {
        return -1;
    }
    if (face_gallery_write_file(file_path, result->normalized,
                                sizeof(result->normalized)) != 0) {
        return -1;
    }

    s_enroll_profile_id = profile_id;
    s_enroll_sample_index = sample_index;
    *out_saved_count = sample_index;
    *out_profile_id = profile_id;
    if (sample_index >= FACE_GALLERY_SAMPLES_PER_ID) {
        s_enroll_profile_id = 0;
        s_enroll_sample_index = 0;
    }
    return 0;
}

int face_gallery_verify(const FaceVerifyResult *current,
                        float *out_best_score,
                        float *out_best_avg_score,
                        uint32_t *out_best_profile,
                        uint32_t *out_best_sample)
{
    TUYA_DIR base_dir = NULL;
    TUYA_DIR face_dir = NULL;
    TUYA_FILEINFO info = NULL;
    CONST CHAR_T *name = NULL;
    BOOL_T is_dir = FALSE;
    float *stored = NULL;
    char dir_path[FACE_GALLERY_PATH_LEN];
    char file_path[FACE_GALLERY_PATH_LEN];
    uint32_t best_profile = 0;
    uint32_t best_sample = 0;
    float best_score = -2.0f;
    float best_avg_score = -2.0f;
    uint32_t feature_count = 0;
    int rc = -1;

    if (current == NULL || !current->valid ||
        out_best_score == NULL || out_best_avg_score == NULL ||
        out_best_profile == NULL || out_best_sample == NULL) {
        return -1;
    }
    if (face_gallery_prepare() != 0) {
        return -1;
    }

    stored = (float *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED,
                                             sizeof(current->normalized));
    if (stored == NULL) {
        return -1;
    }

    if (tkl_dir_open(FACE_GALLERY_DIR_BASE, &base_dir) != 0) {
        bk_printf("[xrecog] opendir %s failed\r\n", FACE_GALLERY_DIR_BASE);
        goto out;
    }

    while (tkl_dir_read(base_dir, &info) == 0) {
        uint32_t profile_id = 0;
        uint32_t feat_count = 0;
        float profile_best_score = -2.0f;
        uint32_t profile_best_sample = 0;
        float profile_score_sum = 0.0f;
        uint32_t profile_feature_count = 0;
        int n;

        if (tkl_dir_name(info, &name) != 0 || name == NULL) {
            continue;
        }
        if (tkl_dir_is_directory(info, &is_dir) != 0 || !is_dir) {
            continue;
        }
        if (!face_gallery_parse_face_dir_id(name, &profile_id)) {
            continue;
        }

        n = snprintf(dir_path, sizeof(dir_path), "%s/%s", FACE_GALLERY_DIR_BASE, name);
        if (n <= 0 || n >= (int)sizeof(dir_path)) {
            continue;
        }
        if (face_gallery_count_features(dir_path, &feat_count) != 0 ||
            feat_count < FACE_GALLERY_SAMPLES_PER_ID) {
            continue;
        }
        if (tkl_dir_open(dir_path, &face_dir) != 0) {
            continue;
        }

        {
            TUYA_FILEINFO face_info = NULL;
            while (tkl_dir_read(face_dir, &face_info) == 0) {
                uint32_t sample_id = 0;
                float score;
                CONST CHAR_T *fname = NULL;

                if (tkl_dir_name(face_info, &fname) != 0 || fname == NULL) {
                    continue;
                }
                if (!face_gallery_parse_feature_id(fname, &sample_id)) {
                    continue;
                }
                n = snprintf(file_path, sizeof(file_path), "%s/%s", dir_path, fname);
                if (n <= 0 || n >= (int)sizeof(file_path)) {
                    continue;
                }
                if (face_gallery_read_file(file_path, stored,
                                           sizeof(current->normalized)) != 0) {
                    continue;
                }
                score = face_recognition_cosine_similarity(current->normalized,
                                                           stored,
                                                           kFaceEmbeddingDim);
                feature_count++;
                profile_feature_count++;
                profile_score_sum += score;
                if (score > profile_best_score) {
                    profile_best_score = score;
                    profile_best_sample = sample_id;
                }
            }
        }
        tkl_dir_close(face_dir);
        face_dir = NULL;

        if (profile_feature_count >= FACE_GALLERY_SAMPLES_PER_ID &&
            profile_best_score > best_score) {
            best_score = profile_best_score;
            best_avg_score = profile_score_sum / (float)profile_feature_count;
            best_profile = profile_id;
            best_sample = profile_best_sample;
        }
    }
    tkl_dir_close(base_dir);
    base_dir = NULL;

    if (feature_count == 0) {
        goto out;
    }

    *out_best_score = best_score;
    *out_best_avg_score = best_avg_score;
    *out_best_profile = best_profile;
    *out_best_sample = best_sample;
    rc = 0;

out:
    if (stored != NULL) {
        bk_frame_buffer_free(stored);
    }
    if (face_dir != NULL) {
        tkl_dir_close(face_dir);
    }
    if (base_dir != NULL) {
        tkl_dir_close(base_dir);
    }
    return rc;
}

int face_gallery_list(void)
{
    TUYA_DIR dir = NULL;
    TUYA_FILEINFO info = NULL;
    CONST CHAR_T *name = NULL;
    BOOL_T is_dir = FALSE;
    char dir_path[FACE_GALLERY_PATH_LEN];
    uint32_t total = 0;

    if (face_gallery_prepare() != 0) {
        return -1;
    }
    if (tkl_dir_open(FACE_GALLERY_DIR_BASE, &dir) != 0) {
        bk_printf("[xrecog] gallery empty or missing\r\n");
        return 0;
    }
    bk_printf("[xrecog] gallery %s:\r\n", FACE_GALLERY_DIR_BASE);
    while (tkl_dir_read(dir, &info) == 0) {
        uint32_t profile_id = 0;
        uint32_t feat_count = 0;
        int n;

        if (tkl_dir_name(info, &name) != 0 || name == NULL) {
            continue;
        }
        if (tkl_dir_is_directory(info, &is_dir) != 0 || !is_dir) {
            continue;
        }
        if (!face_gallery_parse_face_dir_id(name, &profile_id)) {
            continue;
        }
        n = snprintf(dir_path, sizeof(dir_path), "%s/%s", FACE_GALLERY_DIR_BASE, name);
        if (n <= 0 || n >= (int)sizeof(dir_path)) {
            continue;
        }
        (void)face_gallery_count_features(dir_path, &feat_count);
        bk_printf("  profile=%u features=%u/%u path=%s\r\n",
                  (unsigned)profile_id,
                  (unsigned)feat_count,
                  (unsigned)FACE_GALLERY_SAMPLES_PER_ID,
                  dir_path);
        total++;
    }
    tkl_dir_close(dir);
    bk_printf("[xrecog] profiles=%u\r\n", (unsigned)total);
    return 0;
}

int face_gallery_clear(uint32_t profile_id)
{
    TUYA_DIR dir = NULL;
    TUYA_FILEINFO info = NULL;
    CONST CHAR_T *name = NULL;
    BOOL_T is_dir = FALSE;
    char dir_path[FACE_GALLERY_PATH_LEN];

    if (face_gallery_prepare() != 0) {
        return -1;
    }

    if (profile_id != 0) {
        int n = snprintf(dir_path, sizeof(dir_path), "%s/face_%04u",
                         FACE_GALLERY_DIR_BASE, (unsigned)profile_id);
        if (n <= 0 || n >= (int)sizeof(dir_path)) {
            return -1;
        }
        face_gallery_remove_dir(dir_path);
        if (s_enroll_profile_id == profile_id) {
            s_enroll_profile_id = 0;
            s_enroll_sample_index = 0;
        }
        bk_printf("[xrecog] cleared profile %u\r\n", (unsigned)profile_id);
        return 0;
    }

    if (tkl_dir_open(FACE_GALLERY_DIR_BASE, &dir) != 0) {
        return 0;
    }
    while (tkl_dir_read(dir, &info) == 0) {
        uint32_t id = 0;
        int n;

        if (tkl_dir_name(info, &name) != 0 || name == NULL) {
            continue;
        }
        if (tkl_dir_is_directory(info, &is_dir) != 0 || !is_dir) {
            continue;
        }
        if (!face_gallery_parse_face_dir_id(name, &id)) {
            continue;
        }
        n = snprintf(dir_path, sizeof(dir_path), "%s/%s", FACE_GALLERY_DIR_BASE, name);
        if (n > 0 && n < (int)sizeof(dir_path)) {
            face_gallery_remove_dir(dir_path);
        }
    }
    tkl_dir_close(dir);
    s_enroll_profile_id = 0;
    s_enroll_sample_index = 0;
    bk_printf("[xrecog] cleared all profiles\r\n");
    return 0;
}
