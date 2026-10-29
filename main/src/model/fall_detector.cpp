#include "fall_detector.h"

#include "fall_model_config.h"
#include "fall_model_data.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include <cmath>
#include <cstdint>
#include <new>

namespace {
constexpr size_t kTensorArenaSize = 96 * 1024;
constexpr float kQuantizationTolerance = 0.000001f;

alignas(16) uint8_t s_tensor_arena[kTensorArenaSize];
alignas(tflite::MicroInterpreter)
uint8_t s_interpreter_storage[sizeof(tflite::MicroInterpreter)];

tflite::MicroMutableOpResolver<7> s_resolver;
tflite::MicroInterpreter *s_interpreter = nullptr;
TfLiteTensor *s_input = nullptr;
TfLiteTensor *s_output = nullptr;
bool s_temporal_candidates[TEMPORAL_WINDOW_COUNT] = {};
size_t s_temporal_count = 0;
size_t s_temporal_next = 0;

const char *TAG = "fall_detector";

bool quantization_matches(const TfLiteTensor *tensor,
                          float expected_scale,
                          int expected_zero_point)
{
    return tensor != nullptr &&
           std::fabs(tensor->params.scale - expected_scale) <=
               kQuantizationTolerance &&
           tensor->params.zero_point == expected_zero_point;
}
}  // namespace

extern "C" esp_err_t fall_detector_init(void)
{
    if (s_interpreter != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    if (g_fall_model_len == 0) {
        ESP_LOGE(TAG, "Model data rỗng");
        return ESP_FAIL;
    }

    const tflite::Model *model = tflite::GetModel(g_fall_model);
    if (model == nullptr || model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema không tương thích");
        return ESP_FAIL;
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

    const size_t free_heap_before =
        heap_caps_get_free_size(MALLOC_CAP_8BIT);

    s_interpreter = new (s_interpreter_storage) tflite::MicroInterpreter(
        model, s_resolver, s_tensor_arena, sizeof(s_tensor_arena));

    if (s_interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors thất bại; thử tăng Tensor Arena");
        return ESP_FAIL;
    }

    s_input = s_interpreter->input(0);
    s_output = s_interpreter->output(0);

    if (s_input == nullptr || s_output == nullptr ||
        s_input->type != kTfLiteInt8 || s_output->type != kTfLiteInt8) {
        ESP_LOGE(TAG, "Input/output tensor không hợp lệ hoặc không phải INT8");
        return ESP_FAIL;
    }

    const bool input_shape_ok =
        s_input->dims != nullptr && s_input->dims->size == 3 &&
        s_input->dims->data[0] == 1 &&
        s_input->dims->data[1] == WINDOW_SIZE &&
        s_input->dims->data[2] == NUM_CHANNELS;

    const bool output_shape_ok =
        s_output->dims != nullptr && s_output->dims->size == 2 &&
        s_output->dims->data[0] == 1 &&
        s_output->dims->data[1] == 1;

    const size_t expected_input_bytes =
        static_cast<size_t>(WINDOW_SIZE) *
        static_cast<size_t>(NUM_CHANNELS);

    if (!input_shape_ok || !output_shape_ok ||
        s_input->bytes != expected_input_bytes ||
        s_output->bytes != sizeof(int8_t)) {
        ESP_LOGE(TAG, "Shape hoặc kích thước tensor không khớp cấu hình");
        return ESP_FAIL;
    }

    if (!quantization_matches(s_input, INPUT_SCALE, INPUT_ZERO_POINT) ||
        !quantization_matches(s_output, OUTPUT_SCALE, OUTPUT_ZERO_POINT)) {
        ESP_LOGE(TAG, "Quantization của tensor không khớp manifest");
        return ESP_FAIL;
    }

    // Input sanity là normalized zero, nên mỗi phần tử INT8 bằng zero point.
    for (size_t i = 0; i < s_input->bytes; ++i) {
        s_input->data.int8[i] =
            static_cast<int8_t>(s_input->params.zero_point);
    }

    const int64_t start_us = esp_timer_get_time();
    if (s_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Sanity Invoke thất bại");
        return ESP_FAIL;
    }
    const int64_t elapsed_us = esp_timer_get_time() - start_us;

    const int8_t output_q = s_output->data.int8[0];
    const float p_fall =
        (static_cast<int>(output_q) - s_output->params.zero_point) *
        s_output->params.scale;

    if (!std::isfinite(p_fall) || p_fall < 0.0f || p_fall > 1.0f) {
        ESP_LOGE(TAG, "Output sau giải lượng tử hóa không hợp lệ: %.6f",
                 p_fall);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Model OK: %u bytes; input [1,%d,%d] INT8",
             static_cast<unsigned>(g_fall_model_len),
             WINDOW_SIZE, NUM_CHANNELS);
    ESP_LOGI(TAG, "Tensor Arena used=%u bytes; free heap before=%u bytes",
             static_cast<unsigned>(s_interpreter->arena_used_bytes()),
             static_cast<unsigned>(free_heap_before));
    ESP_LOGI(TAG, "Sanity output INT8=%d, P(FALL)=%.6f, Invoke=%.3f ms",
             static_cast<int>(output_q), p_fall,
             static_cast<float>(elapsed_us) / 1000.0f);

    return ESP_OK;
}

extern "C" esp_err_t fall_detector_prepare_input(
    const float *circular_buffer,
    size_t oldest_index)
{
    if (circular_buffer == nullptr || oldest_index >= WINDOW_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_interpreter == nullptr || s_input == nullptr ||
        s_input->type != kTfLiteInt8) {
        return ESP_ERR_INVALID_STATE;
    }

    const size_t expected_elements =
        static_cast<size_t>(WINDOW_SIZE) * static_cast<size_t>(NUM_CHANNELS);
    const float input_scale = s_input->params.scale;
    const int input_zero_point = s_input->params.zero_point;
    if (s_input->bytes != expected_elements ||
        !std::isfinite(input_scale) || input_scale <= 0.0f ||
        s_input->data.raw == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t i = 0; i < expected_elements; ++i) {
        if (!std::isfinite(circular_buffer[i])) {
            ESP_LOGE(TAG, "Mẫu SI không hữu hạn tại phần tử %u",
                     static_cast<unsigned>(i));
            return ESP_ERR_INVALID_ARG;
        }
    }

    size_t saturated_count = 0;
    int quantized_min = 127;
    int quantized_max = -128;

    for (size_t row = 0; row < static_cast<size_t>(WINDOW_SIZE); ++row) {
        const size_t source_row =
            (oldest_index + row) % static_cast<size_t>(WINDOW_SIZE);

        for (size_t channel = 0;
             channel < static_cast<size_t>(NUM_CHANNELS); ++channel) {
            const size_t output_index =
                row * static_cast<size_t>(NUM_CHANNELS) + channel;
            const float raw_value =
                circular_buffer[source_row * static_cast<size_t>(NUM_CHANNELS) +
                                channel];
            if (!std::isfinite(CHANNEL_MEAN[channel]) ||
                !std::isfinite(CHANNEL_STD[channel]) ||
                CHANNEL_STD[channel] <= 0.0f) {
                return ESP_ERR_INVALID_STATE;
            }
            const float normalized =
                (raw_value - CHANNEL_MEAN[channel]) / CHANNEL_STD[channel];
            if (!std::isfinite(normalized)) {
                return ESP_ERR_INVALID_ARG;
            }

            const double scaled =
                static_cast<double>(normalized) / static_cast<double>(input_scale);
            const double lower_saturation = -128.5 - input_zero_point;
            const double upper_saturation = 127.5 - input_zero_point;
            int quantized;
            if (scaled <= lower_saturation) {
                quantized = -128;
                ++saturated_count;
            } else if (scaled >= upper_saturation) {
                quantized = 127;
                ++saturated_count;
            } else {
                quantized = static_cast<int>(std::lround(scaled)) +
                            input_zero_point;
            }

            s_input->data.int8[output_index] =
                static_cast<int8_t>(quantized);
            if (quantized < quantized_min) {
                quantized_min = quantized;
            }
            if (quantized > quantized_max) {
                quantized_max = quantized;
            }
        }
    }

    ESP_LOGI(TAG,
             "Input INT8 prepared: range=[%d,%d], saturated=%u/%u elements",
             quantized_min, quantized_max,
             static_cast<unsigned>(saturated_count),
             static_cast<unsigned>(expected_elements));
    return ESP_OK;
}

extern "C" esp_err_t fall_detector_invoke(float *p_fall,
                                          float *inference_ms)
{
    if (p_fall == nullptr || inference_ms == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_interpreter == nullptr || s_output == nullptr ||
        s_output->type != kTfLiteInt8 || s_output->data.raw == nullptr ||
        s_output->bytes != sizeof(int8_t)) {
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t start_us = esp_timer_get_time();
    if (s_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke thất bại");
        return ESP_FAIL;
    }
    const int64_t elapsed_us = esp_timer_get_time() - start_us;

    const int output_q = static_cast<int>(s_output->data.int8[0]);
    const float probability =
        static_cast<float>(output_q - s_output->params.zero_point) *
        s_output->params.scale;
    if (!std::isfinite(probability) || probability < 0.0f ||
        probability > 1.0f) {
        ESP_LOGE(TAG, "P(FALL) sau giải lượng tử hóa không hợp lệ: %.6f",
                 probability);
        return ESP_FAIL;
    }

    *p_fall = probability;
    *inference_ms = static_cast<float>(elapsed_us) / 1000.0f;
    return ESP_OK;
}

extern "C" bool fall_detector_is_fall_candidate(float p_fall)
{
    return std::isfinite(p_fall) && p_fall >= FALL_THRESHOLD;
}

extern "C" esp_err_t fall_detector_temporal_update(
    bool candidate,
    bool *fall_detected,
    uint8_t *fall_votes,
    uint8_t *history_count)
{
    if (fall_detected == nullptr || fall_votes == nullptr ||
        history_count == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    s_temporal_candidates[s_temporal_next] = candidate;
    s_temporal_next = (s_temporal_next + 1U) % TEMPORAL_WINDOW_COUNT;
    if (s_temporal_count < TEMPORAL_WINDOW_COUNT) {
        ++s_temporal_count;
    }

    size_t votes = 0;
    for (size_t i = 0; i < s_temporal_count; ++i) {
        votes += s_temporal_candidates[i] ? 1U : 0U;
    }

    *fall_votes = static_cast<uint8_t>(votes);
    *history_count = static_cast<uint8_t>(s_temporal_count);
    *fall_detected =
        s_temporal_count == TEMPORAL_WINDOW_COUNT &&
        votes >= TEMPORAL_REQUIRED_FALLS;
    return ESP_OK;
}

extern "C" void fall_detector_temporal_reset(void)
{
    for (size_t i = 0; i < TEMPORAL_WINDOW_COUNT; ++i) {
        s_temporal_candidates[i] = false;
    }
    s_temporal_count = 0;
    s_temporal_next = 0;
}
