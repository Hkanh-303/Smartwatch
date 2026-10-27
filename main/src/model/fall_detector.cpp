#include "model/fall_detector.h"

#include <cmath>
#include <cstring>
#include <new>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "fall_model_config.h"
#include "fall_model_data.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

namespace {

constexpr size_t kTensorArenaSize = 96 * 1024;
constexpr size_t kInputElements = WINDOW_SIZE * NUM_CHANNELS;
constexpr size_t kExpectedInputBytes = kInputElements * sizeof(int8_t);
constexpr float kQuantizationTolerance = 1.0e-6f;

alignas(16) uint8_t s_tensor_arena[kTensorArenaSize];
alignas(tflite::MicroInterpreter)
uint8_t s_interpreter_storage[sizeof(tflite::MicroInterpreter)];

tflite::MicroMutableOpResolver<7> s_resolver;
tflite::MicroInterpreter *s_interpreter;
TfLiteTensor *s_input;
TfLiteTensor *s_output;
bool s_initialized;
size_t s_arena_used;

const char *TAG = "fall_ai";

bool tensor_shape_is(const TfLiteTensor *tensor, const int *shape, int rank)
{
    if (tensor == nullptr || tensor->dims == nullptr || tensor->dims->size != rank) {
        return false;
    }
    for (int i = 0; i < rank; ++i) {
        if (tensor->dims->data[i] != shape[i]) {
            return false;
        }
    }
    return true;
}

bool quantization_matches(const TfLiteTensor *tensor, float scale, int zero_point)
{
    return tensor != nullptr &&
           std::fabs(tensor->params.scale - scale) <= kQuantizationTolerance &&
           tensor->params.zero_point == zero_point;
}

int8_t quantize_normalized(float normalized)
{
    long quantized = std::lround(normalized / INPUT_SCALE) + INPUT_ZERO_POINT;
    if (quantized < -128) {
        quantized = -128;
    } else if (quantized > 127) {
        quantized = 127;
    }
    return static_cast<int8_t>(quantized);
}

}  // namespace

extern "C" esp_err_t fall_detector_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    const tflite::Model *model = tflite::GetModel(g_fall_model);
    if (model == nullptr || model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema không tương thích: model=%lu, runtime=%lu",
                 static_cast<unsigned long>(model == nullptr ? 0 : model->version()),
                 static_cast<unsigned long>(TFLITE_SCHEMA_VERSION));
        return ESP_ERR_INVALID_VERSION;
    }

    if (s_resolver.AddExpandDims() != kTfLiteOk ||
        s_resolver.AddConv2D() != kTfLiteOk ||
        s_resolver.AddReshape() != kTfLiteOk ||
        s_resolver.AddMaxPool2D() != kTfLiteOk ||
        s_resolver.AddMean() != kTfLiteOk ||
        s_resolver.AddFullyConnected() != kTfLiteOk ||
        s_resolver.AddLogistic() != kTfLiteOk) {
        ESP_LOGE(TAG, "Không đăng ký được operator của model");
        return ESP_FAIL;
    }

    s_interpreter = new (s_interpreter_storage) tflite::MicroInterpreter(
        model, s_resolver, s_tensor_arena, sizeof(s_tensor_arena));
    if (s_interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors thất bại (arena=%u bytes)",
                 static_cast<unsigned>(sizeof(s_tensor_arena)));
        s_interpreter->~MicroInterpreter();
        s_interpreter = nullptr;
        return ESP_ERR_NO_MEM;
    }

    s_input = s_interpreter->input(0);
    s_output = s_interpreter->output(0);
    const int expected_input_shape[] = {1, WINDOW_SIZE, NUM_CHANNELS};
    const int expected_output_shape[] = {1, 1};
    if (s_input == nullptr || s_output == nullptr ||
        s_input->type != kTfLiteInt8 || s_output->type != kTfLiteInt8 ||
        !tensor_shape_is(s_input, expected_input_shape, 3) ||
        !tensor_shape_is(s_output, expected_output_shape, 2) ||
        s_input->bytes != kExpectedInputBytes ||
        s_output->bytes != sizeof(int8_t) ||
        !quantization_matches(s_input, INPUT_SCALE, INPUT_ZERO_POINT) ||
        !quantization_matches(s_output, OUTPUT_SCALE, OUTPUT_ZERO_POINT)) {
        ESP_LOGE(TAG, "Tensor model không khớp manifest/config (shape, type, bytes hoặc quantization)");
        s_interpreter->~MicroInterpreter();
        s_interpreter = nullptr;
        s_input = nullptr;
        s_output = nullptr;
        return ESP_ERR_INVALID_RESPONSE;
    }

    s_arena_used = s_interpreter->arena_used_bytes();
    ESP_LOGI(TAG, "Model INT8 OK: input [1,%d,%d], output [1,1], operators=7",
             WINDOW_SIZE, NUM_CHANNELS);
    ESP_LOGI(TAG, "Quantization: input scale=%.12f zp=%ld; output scale=%.8f zp=%ld",
             s_input->params.scale,
             static_cast<long>(s_input->params.zero_point),
             s_output->params.scale,
             static_cast<long>(s_output->params.zero_point));
    ESP_LOGI(TAG, "Tensor Arena: used=%u / %u bytes; free heap=%u bytes",
             static_cast<unsigned>(s_arena_used),
             static_cast<unsigned>(sizeof(s_tensor_arena)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)));

    /* ZIP sanity test: all-zero normalized input maps to the input zero point. */
    std::memset(s_input->data.int8, INPUT_ZERO_POINT, s_input->bytes);
    if (s_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Model sanity Invoke thất bại");
        s_interpreter->~MicroInterpreter();
        s_interpreter = nullptr;
        s_input = nullptr;
        s_output = nullptr;
        return ESP_FAIL;
    }
    const int8_t sanity_output = s_output->data.int8[0];
    const float sanity_probability =
        (static_cast<int>(sanity_output) - OUTPUT_ZERO_POINT) * OUTPUT_SCALE;
    ESP_LOGI(TAG, "Sanity test (normalized zeros, không phải dữ liệu realtime): output=%d, P(FALL)=%.6f",
             static_cast<int>(sanity_output), sanity_probability);

    s_initialized = true;
    return ESP_OK;
}

extern "C" esp_err_t fall_detector_predict_circular(
    const float ring[WINDOW_SIZE][NUM_CHANNELS],
    size_t oldest_index,
    fall_inference_result_t *result)
{
    if (ring == nullptr || result == nullptr || oldest_index >= WINDOW_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized || s_interpreter == nullptr || s_input == nullptr || s_output == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t source_sample = oldest_index;
    for (size_t sample = 0; sample < WINDOW_SIZE; ++sample) {
        for (size_t channel = 0; channel < NUM_CHANNELS; ++channel) {
            const size_t index = sample * NUM_CHANNELS + channel;
            const float value = ring[source_sample][channel];
            if (!std::isfinite(value)) {
                return ESP_ERR_INVALID_ARG;
            }
            const float normalized =
                (value - CHANNEL_MEAN[channel]) / CHANNEL_STD[channel];
            s_input->data.int8[index] = quantize_normalized(normalized);
        }
        if (++source_sample == WINDOW_SIZE) {
            source_sample = 0;
        }
    }

    const int64_t start_us = esp_timer_get_time();
    if (s_interpreter->Invoke() != kTfLiteOk) {
        return ESP_FAIL;
    }
    const int64_t elapsed_us = esp_timer_get_time() - start_us;

    result->output_q = s_output->data.int8[0];
    result->probability =
        (static_cast<int>(result->output_q) - OUTPUT_ZERO_POINT) * OUTPUT_SCALE;
    result->inference_ms = static_cast<float>(elapsed_us) / 1000.0f;
    return ESP_OK;
}

extern "C" size_t fall_detector_tensor_arena_used(void)
{
    return s_arena_used;
}
