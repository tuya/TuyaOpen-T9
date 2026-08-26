/**
 * @file AvdkMultiModel.cc
 * @brief Shared-arena dual-model NPU runtime implementation
 * @version 1.0
 * @date 2026-08-25
 * @copyright Copyright (c) Tuya Inc.
 */
#include "AvdkMultiModel.h"

#include "os/mem.h"
#include "os/str.h"
#include "os/os.h"
#include "components/bk_frame_buffer.h"
#include "tkl_fs.h"
#include "bk_ethosu.h"
#include "tensorflow/lite/schema/schema_generated.h"

static const char *TAG = "multi-model";
#define AVDK_MULTI_MODEL_IO_CHUNK 4096u

#define LOGI(...) BK_LOGW((char *)TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE((char *)TAG, ##__VA_ARGS__)

bool AvdkMultiModel::s_hw_inited_ = false;
uint8_t *AvdkMultiModel::s_fast_ram_data_ = NULL;
uint32_t AvdkMultiModel::s_fast_ram_data_size_ = 0;
avdk_nn_mem_type_t AvdkMultiModel::s_fast_ram_type_ = AVDK_NN_MEM_TYPE_HSRAM;
int AvdkMultiModel::s_model_count_ = 0;
uint8_t *AvdkMultiModel::s_shared_arena_data_ = NULL;
uint32_t AvdkMultiModel::s_shared_arena_size_ = 0;
avdk_nn_mem_type_t AvdkMultiModel::s_shared_arena_type_ = AVDK_NN_MEM_TYPE_PSRAM_SLAB_UNCODED;
AvdkMultiModel *AvdkMultiModel::s_active_model_ = NULL;

AvdkMultiModel::AvdkMultiModel()
    : name_(NULL),
      model_type_(AVDK_NN_MODEL_TYPE_NPU),
      modelLoadType_(AVDK_NN_MODEL_LOAD_TYPE_FLASH),
      model_ram_type_(AVDK_NN_MEM_TYPE_FALSH),
      model_data_(NULL),
      model_data_size_(0),
      model_flash_data_(NULL),
      model_flash_data_size_(0),
      modelFilePath_(NULL),
      fast_ram_type_(AVDK_NN_MEM_TYPE_HSRAM),
      fast_ram_data_size_(0),
      arena_ram_type_(AVDK_NN_MEM_TYPE_PSRAM_SLAB_UNCODED),
      arena_ram_data_(NULL),
      arena_data_size_(0),
      pinterpreter_(NULL),
      micro_op_resolver_(),
      resolver_ready_(false),
      initialized_(false)
{
}

AvdkMultiModel::~AvdkMultiModel()
{
    deinit();
}

void AvdkMultiModel::setName(const char *name) { name_ = name; }
void AvdkMultiModel::setModelType(avdk_nn_model_type_t type) { model_type_ = type; }
void AvdkMultiModel::setModelLoadType(avdk_nn_model_load_type_t type) { modelLoadType_ = type; }
void AvdkMultiModel::setModelRamType(avdk_nn_mem_type_t type) { model_ram_type_ = type; }
void AvdkMultiModel::setModelFilePath(const char *path) { modelFilePath_ = path; }
void AvdkMultiModel::setFlashModel(uint8_t *data, uint32_t size)
{
    model_flash_data_ = data;
    model_flash_data_size_ = size;
}
void AvdkMultiModel::setFastRam(avdk_nn_mem_type_t type, uint32_t size)
{
    fast_ram_type_ = type;
    fast_ram_data_size_ = size;
}
void AvdkMultiModel::setArenaRam(avdk_nn_mem_type_t type, uint32_t size)
{
    arena_ram_type_ = type;
    arena_data_size_ = size;
}

void *AvdkMultiModel::allocMemory(avdk_nn_mem_type_t type, uint32_t size)
{
    if (type == AVDK_NN_MEM_TYPE_HSRAM) {
        return hsram_malloc(size);
    } else if (type == AVDK_NN_MEM_TYPE_SRAM) {
        return os_sram_malloc(size);
    } else if (type == AVDK_NN_MEM_TYPE_PSRAM_HEAP) {
        return psram_malloc(size);
    } else if (type == AVDK_NN_MEM_TYPE_PSRAM_SLAB) {
        return bk_frame_buffer_malloc(MEM_SLAB_HEAP_CODED, size);
    } else if (type == AVDK_NN_MEM_TYPE_PSRAM_SLAB_UNCODED) {
        return bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, size);
    }
    LOGE("Unsupported memory type alloc\n");
    return NULL;
}

void AvdkMultiModel::freeMemory(avdk_nn_mem_type_t type, void *ptr)
{
    if (ptr == NULL) {
        return;
    }
    if (type == AVDK_NN_MEM_TYPE_HSRAM) {
        hsram_free(ptr);
    } else if (type == AVDK_NN_MEM_TYPE_SRAM) {
        os_sram_free(ptr);
    } else if (type == AVDK_NN_MEM_TYPE_PSRAM_HEAP) {
        psram_free(ptr);
    } else if (type == AVDK_NN_MEM_TYPE_PSRAM_SLAB ||
               type == AVDK_NN_MEM_TYPE_PSRAM_SLAB_UNCODED) {
        bk_frame_buffer_free(ptr);
    } else {
        LOGE("Unsupported memory type free\n");
    }
}

int AvdkMultiModel::LoadModel()
{
    if (initialized_) {
        return 0;
    }

    if (modelLoadType_ == AVDK_NN_MODEL_LOAD_TYPE_SD_FILE) {
        if (modelFilePath_ == NULL || modelFilePath_[0] == '\0') {
            LOGE("SD model path is empty\n");
            return -1;
        }

        INT_T file_size = tkl_fgetsize(modelFilePath_);
        if (file_size <= 0) {
            LOGE("tkl_fgetsize %s failed, size=%d\n", modelFilePath_, file_size);
            return -1;
        }

        model_data_size_ = (uint32_t)file_size;
        model_data_ = (uint8_t *)allocMemory(model_ram_type_, model_data_size_);
        if (model_data_ == NULL) {
            LOGE("Failed to allocate SD model data, size=%u\n", (unsigned)model_data_size_);
            return -1;
        }

        TUYA_FILE file = tkl_fopen(modelFilePath_, "rb");
        if (file == NULL) {
            LOGE("tkl_fopen %s failed\n", modelFilePath_);
            freeMemory(model_ram_type_, model_data_);
            model_data_ = NULL;
            model_data_size_ = 0;
            return -1;
        }

        uint32_t total = 0;
        while (total < model_data_size_) {
            uint32_t chunk = model_data_size_ - total;
            if (chunk > AVDK_MULTI_MODEL_IO_CHUNK) {
                chunk = AVDK_MULTI_MODEL_IO_CHUNK;
            }
            INT_T br = tkl_fread(model_data_ + total, (INT_T)chunk, file);
            if (br != (INT_T)chunk) {
                LOGE("read SD model failed at %u/%u, br=%d\n",
                     (unsigned)total, (unsigned)model_data_size_, br);
                tkl_fclose(file);
                freeMemory(model_ram_type_, model_data_);
                model_data_ = NULL;
                model_data_size_ = 0;
                return -1;
            }
            total += (uint32_t)br;
        }
        tkl_fclose(file);
        LOGI("loaded SD model %s, size=%u\n", modelFilePath_, (unsigned)model_data_size_);
    } else if (model_ram_type_ == AVDK_NN_MEM_TYPE_FALSH) {
        model_data_ = model_flash_data_;
        model_data_size_ = model_flash_data_size_;
    } else {
        model_data_size_ = model_flash_data_size_;
        model_data_ = (uint8_t *)allocMemory(model_ram_type_, model_data_size_);
        if (model_data_ == NULL) {
            LOGE("Failed to allocate model data\n");
            return -1;
        }
        os_memcpy(model_data_, model_flash_data_, model_flash_data_size_);
    }
    return 0;
}

int AvdkMultiModel::UnloadModel()
{
    if (model_ram_type_ != AVDK_NN_MEM_TYPE_FALSH && model_data_ != NULL) {
        freeMemory(model_ram_type_, model_data_);
    }
    model_data_ = NULL;
    model_data_size_ = 0;
    return 0;
}

int AvdkMultiModel::npuStartup()
{
    if (s_hw_inited_ && fast_ram_data_size_ <= s_fast_ram_data_size_) {
        s_model_count_++;
        return 0;
    }

    if (s_hw_inited_) {
        bk_ethosu_deinit();
        if (s_fast_ram_data_ != NULL) {
            freeMemory(s_fast_ram_type_, s_fast_ram_data_);
            s_fast_ram_data_ = NULL;
        }
        s_hw_inited_ = false;
    }

    s_fast_ram_type_ = fast_ram_type_;
    s_fast_ram_data_size_ = fast_ram_data_size_;
    s_fast_ram_data_ = (uint8_t *)allocMemory(s_fast_ram_type_, s_fast_ram_data_size_ + 16);
    if (s_fast_ram_data_ == NULL) {
        LOGE("Failed to allocate fast RAM\n");
        return -1;
    }

    void *aligned_ptr = (void *)(((uint32_t)s_fast_ram_data_ + 15) & ~15U);
    int ret = bk_ethosu_init(aligned_ptr, s_fast_ram_data_size_);
    if (ret != 0) {
        LOGE("Failed to initialize Ethos-U driver, ret=%d\n", ret);
        freeMemory(s_fast_ram_type_, s_fast_ram_data_);
        s_fast_ram_data_ = NULL;
        s_fast_ram_data_size_ = 0;
        return -1;
    }

    s_hw_inited_ = true;
    s_model_count_++;
    return 0;
}

int AvdkMultiModel::npuShutdown()
{
    if (s_model_count_ > 0) {
        s_model_count_--;
    }
    if (s_model_count_ == 0 && s_hw_inited_) {
        bk_ethosu_deinit();
        if (s_fast_ram_data_ != NULL) {
            freeMemory(s_fast_ram_type_, s_fast_ram_data_);
            s_fast_ram_data_ = NULL;
        }
        s_hw_inited_ = false;
        s_fast_ram_data_size_ = 0;
        freeSharedArena();
    }
    return 0;
}

int AvdkMultiModel::ensureSharedArena(avdk_nn_mem_type_t type, uint32_t size)
{
    if (s_shared_arena_data_ != NULL &&
        s_shared_arena_type_ == type &&
        s_shared_arena_size_ >= size) {
        return 0;
    }

    if (s_active_model_ != NULL) {
        s_active_model_->releaseInterpreter();
    }
    if (s_shared_arena_data_ != NULL) {
        freeMemory(s_shared_arena_type_, s_shared_arena_data_);
        s_shared_arena_data_ = NULL;
        s_shared_arena_size_ = 0;
    }

    s_shared_arena_data_ = (uint8_t *)allocMemory(type, size + 16);
    if (s_shared_arena_data_ == NULL) {
        LOGE("Failed to allocate shared arena RAM, size=%u\n", (unsigned)size);
        return -1;
    }
    s_shared_arena_type_ = type;
    s_shared_arena_size_ = size;
    return 0;
}

void AvdkMultiModel::freeSharedArena()
{
    if (s_active_model_ != NULL) {
        s_active_model_->releaseInterpreter();
    }
    if (s_shared_arena_data_ != NULL) {
        freeMemory(s_shared_arena_type_, s_shared_arena_data_);
        s_shared_arena_data_ = NULL;
        s_shared_arena_size_ = 0;
    }
}

int AvdkMultiModel::init()
{
    if (initialized_) {
        return 0;
    }

    if (LoadModel() != 0) {
        LOGE("Failed to load model\n");
        return -1;
    }

    if (model_type_ == AVDK_NN_MODEL_TYPE_NPU) {
        if (npuStartup() != 0) {
            UnloadModel();
            return -1;
        }
    }

    if (model_data_ == NULL) {
        LOGE("model_data is NULL\n");
        if (model_type_ == AVDK_NN_MODEL_TYPE_NPU) {
            npuShutdown();
        }
        UnloadModel();
        return -1;
    }

    const tflite::Model *model = ::tflite::GetModel(model_data_);
    if (model == NULL || TFLITE_SCHEMA_VERSION != model->version()) {
        LOGE("Model schema version mismatch\n");
        if (model_type_ == AVDK_NN_MODEL_TYPE_NPU) {
            npuShutdown();
        }
        UnloadModel();
        return -1;
    }

    if (!resolver_ready_) {
        if (micro_op_resolver_.AddEthosU() != kTfLiteOk) {
            LOGE("AddEthosU failed\n");
            if (model_type_ == AVDK_NN_MODEL_TYPE_NPU) {
                npuShutdown();
            }
            UnloadModel();
            return -1;
        }
        resolver_ready_ = true;
    }

    if (ensureSharedArena(arena_ram_type_, arena_data_size_) != 0) {
        if (model_type_ == AVDK_NN_MODEL_TYPE_NPU) {
            npuShutdown();
        }
        UnloadModel();
        return -1;
    }

    initialized_ = true;
    LOGI("AvdkMultiModel::init %s\n", name_ ? name_ : "unknown");
    return 0;
}

int AvdkMultiModel::prepareInterpreter()
{
    if (pinterpreter_ != NULL) {
        return 0;
    }
    if (model_data_ == NULL) {
        LOGE("model_data is NULL\n");
        return -1;
    }
    if (ensureSharedArena(arena_ram_type_, arena_data_size_) != 0) {
        return -1;
    }
    if (s_active_model_ != NULL && s_active_model_ != this) {
        s_active_model_->releaseInterpreter();
    }

    const tflite::Model *model = ::tflite::GetModel(model_data_);
    if (model == NULL || TFLITE_SCHEMA_VERSION != model->version()) {
        LOGE("Model schema version mismatch\n");
        return -1;
    }

    arena_ram_data_ = s_shared_arena_data_;
    pinterpreter_ = new tflite::MicroInterpreter(model, micro_op_resolver_,
                                                  arena_ram_data_, s_shared_arena_size_);
    if (pinterpreter_ == NULL) {
        return -1;
    }
    if (kTfLiteOk != pinterpreter_->AllocateTensors()) {
        releaseInterpreter();
        return -1;
    }
    s_active_model_ = this;
    return 0;
}

void AvdkMultiModel::releaseInterpreter()
{
    if (pinterpreter_ != NULL) {
        delete pinterpreter_;
        pinterpreter_ = NULL;
    }
    if (s_active_model_ == this) {
        s_active_model_ = NULL;
    }
    arena_ram_data_ = NULL;
}

int AvdkMultiModel::deinit()
{
    if (!initialized_ && model_data_ == NULL) {
        return 0;
    }
    releaseInterpreter();
    if (model_type_ == AVDK_NN_MODEL_TYPE_NPU) {
        npuShutdown();
    }
    UnloadModel();
    initialized_ = false;
    return 0;
}

tflite::MicroInterpreter *AvdkMultiModel::interpreter()
{
    return pinterpreter_;
}
