// Copyright 2020-2021 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <stdint.h>

#include "AvdkDetectionModel.h"

extern "C" {
extern const unsigned char person_detection_vela_tflite[];
extern const unsigned int person_detection_vela_tflite_len;
}

class PersonDetectModel : public AvdkDetectionModel {
public:
    PersonDetectModel();

    void resolverLoad(void) override;
    void resourceLoad(void) override;
    void resourceUnload(void) override;
    int run(uint8_t *data, uint32_t size, bk_pixel_format_t format) override;

private:
    bool buildOutputTensors(void);
    bool output_tensors_ready;
};