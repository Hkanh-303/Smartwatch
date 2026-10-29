#include "config.h"
#include "model/fall_detector.h"
#include "model/fall_preprocessing.h"
#include "mpu6050/mpu.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define SENSOR_WINDOW_SIZE       125U
#define SENSOR_WINDOW_STEP_SIZE   62U

typedef struct {
    float channels[FALL_SENSOR_CHANNEL_COUNT];
    uint32_t sequence;
} sensor_sample_t;

static const char *TAG = "smartwatch";
static QueueHandle_t s_sample_queue;

static void sensor_task(void *arg)
{
    (void)arg;

    const TickType_t period_ticks = pdMS_TO_TICKS(SENSOR_SAMPLE_PERIOD_MS);
    const TickType_t retry_ticks = pdMS_TO_TICKS(SENSOR_RETRY_INTERVAL_MS);
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t retry_after = last_wake;
    bool sensor_ready = false;
    int64_t last_log_us = 0;
    uint32_t samples_since_log = 0;
    uint32_t queue_drops_since_log = 0;
    uint32_t sequence = 0;

    for (;;) {
        const uint32_t current_sequence = sequence++;
        const TickType_t now_ticks = xTaskGetTickCount();

        if (!sensor_ready && (int32_t)(now_ticks - retry_after) >= 0) {
            const esp_err_t init_err = mpu6050_init();
            sensor_ready = (init_err == ESP_OK);

            if (sensor_ready) {
                ESP_LOGI(TAG, "Đọc MPU-6050/MPU-6500 ở %d Hz",
                         SENSOR_SAMPLE_RATE_HZ);
                last_wake = xTaskGetTickCount();
                last_log_us = 0;
                samples_since_log = 0;
                queue_drops_since_log = 0;
            } else {
                retry_after = xTaskGetTickCount() + retry_ticks;
            }
        }

        if (sensor_ready) {
            mpu6050_data_t data;
            const esp_err_t read_err = mpu6050_read(&data);

            if (read_err == ESP_OK) {
                float si_channels[FALL_SENSOR_CHANNEL_COUNT];
                const esp_err_t convert_err =
                    fall_preprocessing_to_si(&data, si_channels);

                if (convert_err != ESP_OK) {
                    ESP_LOGE(TAG, "Đổi đơn vị MPU thất bại: %s",
                             esp_err_to_name(convert_err));
                } else {
                    sensor_sample_t sample = {0};
                    for (size_t channel = 0;
                         channel < FALL_SENSOR_CHANNEL_COUNT; ++channel) {
                        sample.channels[channel] = si_channels[channel];
                    }
                    sample.sequence = current_sequence;
                    ++samples_since_log;

                    if (xQueueSend(s_sample_queue, &sample, 0) != pdTRUE) {
                        ++queue_drops_since_log;
                    }

                    const int64_t now_us = esp_timer_get_time();
                    if (last_log_us == 0 ||
                        now_us - last_log_us >=
                            (int64_t)SENSOR_LOG_INTERVAL_MS * 1000) {
                        const float accel_magnitude = sqrtf(
                            si_channels[0] * si_channels[0] +
                            si_channels[1] * si_channels[1] +
                            si_channels[2] * si_channels[2]);
                        const float gyro_magnitude = sqrtf(
                            si_channels[3] * si_channels[3] +
                            si_channels[4] * si_channels[4] +
                            si_channels[5] * si_channels[5]);

                        ESP_LOGI(TAG,
                                 "Samples/1s=%u qdrop=%u | "
                                 "Accel[m/s^2]: X=%.3f Y=%.3f Z=%.3f | "
                                 "Gyro[rad/s]: X=%.3f Y=%.3f Z=%.3f | "
                                 "|a|=%.3f | |g|=%.3f",
                                 (unsigned)samples_since_log,
                                 (unsigned)queue_drops_since_log,
                                 si_channels[0], si_channels[1], si_channels[2],
                                 si_channels[3], si_channels[4], si_channels[5],
                                 accel_magnitude, gyro_magnitude);

                        last_log_us = now_us;
                        samples_since_log = 0;
                        queue_drops_since_log = 0;
                    }
                }
            } else {
                ESP_LOGW(TAG, "Đọc MPU thất bại: %s; sẽ thử kết nối lại",
                         esp_err_to_name(read_err));
                sensor_ready = false;
                retry_after = xTaskGetTickCount() + retry_ticks;
            }
        }

        if (xTaskDelayUntil(&last_wake, period_ticks) == pdFALSE) {
            last_wake = xTaskGetTickCount();
        }
    }
}

static void inference_task(void *arg)
{
    (void)arg;

    static float sample_ring[SENSOR_WINDOW_SIZE][FALL_SENSOR_CHANNEL_COUNT];
    size_t write_index = 0;
    size_t samples_in_window = 0;
    size_t samples_after_window = 0;
    bool have_previous_sequence = false;
    uint32_t previous_sequence = 0;

    ESP_LOGI(TAG, "[AI] Inference worker: window=%u, step=%u, target=%u Hz",
             (unsigned)SENSOR_WINDOW_SIZE,
             (unsigned)SENSOR_WINDOW_STEP_SIZE,
             (unsigned)SENSOR_SAMPLE_RATE_HZ);

    for (;;) {
        sensor_sample_t sample;
        if (xQueueReceive(s_sample_queue, &sample, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (have_previous_sequence) {
            const uint32_t sequence_gap = sample.sequence - previous_sequence;
            if (sequence_gap != 1U) {
                ESP_LOGW(TAG,
                         "Sample gap=%lu; xóa cửa sổ để giữ dữ liệu liên tục",
                         (unsigned long)(sequence_gap - 1U));
                write_index = 0;
                samples_in_window = 0;
                samples_after_window = 0;
                fall_detector_temporal_reset();
            }
        }
        previous_sequence = sample.sequence;
        have_previous_sequence = true;

        for (size_t channel = 0;
             channel < FALL_SENSOR_CHANNEL_COUNT; ++channel) {
            sample_ring[write_index][channel] = sample.channels[channel];
        }
        write_index = (write_index + 1U) % SENSOR_WINDOW_SIZE;

        bool window_ready = false;
        if (samples_in_window < SENSOR_WINDOW_SIZE) {
            ++samples_in_window;
            window_ready = (samples_in_window == SENSOR_WINDOW_SIZE);
        } else {
            ++samples_after_window;
            if (samples_after_window >= SENSOR_WINDOW_STEP_SIZE) {
                samples_after_window = 0;
                window_ready = true;
            }
        }

        if (!window_ready) {
            continue;
        }

        ESP_LOGI(TAG,
                 "[BUFFER] Window ready: %u samples x %u channels; "
                 "oldest_idx=%u Ax=%.3f; next window after %u new samples",
                 (unsigned)SENSOR_WINDOW_SIZE,
                 (unsigned)FALL_SENSOR_CHANNEL_COUNT,
                 (unsigned)write_index, sample_ring[write_index][0],
                 (unsigned)SENSOR_WINDOW_STEP_SIZE);

        const esp_err_t prepare_err =
            fall_detector_prepare_input(&sample_ring[0][0], write_index);
        if (prepare_err != ESP_OK) {
            ESP_LOGE(TAG, "Chuẩn bị input model thất bại: %s",
                     esp_err_to_name(prepare_err));
            fall_detector_temporal_reset();
            continue;
        }

        float p_fall = 0.0f;
        float inference_ms = 0.0f;
        const esp_err_t infer_err =
            fall_detector_invoke(&p_fall, &inference_ms);
        if (infer_err != ESP_OK) {
            ESP_LOGE(TAG, "Inference thất bại: %s", esp_err_to_name(infer_err));
            fall_detector_temporal_reset();
            continue;
        }

        const bool candidate = fall_detector_is_fall_candidate(p_fall);
        bool fall_detected = false;
        uint8_t fall_votes = 0;
        uint8_t history_count = 0;
        const esp_err_t temporal_err = fall_detector_temporal_update(
            candidate, &fall_detected, &fall_votes, &history_count);
        if (temporal_err != ESP_OK) {
            ESP_LOGE(TAG, "Cập nhật quy tắc 2-of-3 thất bại: %s",
                     esp_err_to_name(temporal_err));
            fall_detector_temporal_reset();
            continue;
        }

        const char *status = fall_detected
                                 ? "FALL DETECTED"
                                 : (candidate ? "FALL CANDIDATE" : "NORMAL");
        ESP_LOGI(TAG,
                 "[AI] P(FALL)=%.4f | 2-of-3 votes=%u/3 history=%u/3 | "
                 "Status=%s | Invoke=%.3f ms",
                 p_fall, (unsigned)fall_votes, (unsigned)history_count,
                 status, inference_ms);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Khởi chạy ESP32-C3 với cảm biến MPU-6050/MPU-6500");
    ESP_LOGI(TAG, "Bắt đầu sanity test model");

    const esp_err_t model_err = fall_detector_init();
    if (model_err != ESP_OK) {
        ESP_LOGE(TAG, "Sanity test model thất bại: %s",
                 esp_err_to_name(model_err));
        return;
    }
    ESP_LOGI(TAG, "Sanity test model PASS");

    s_sample_queue = xQueueCreate(SENSOR_SAMPLE_QUEUE_LENGTH,
                                  sizeof(sensor_sample_t));
    if (s_sample_queue == NULL) {
        ESP_LOGE(TAG, "Không tạo được sample queue");
        return;
    }

    TaskHandle_t inference_handle = NULL;
    if (xTaskCreate(inference_task, "inference_task",
                    INFERENCE_TASK_STACK_SIZE, NULL,
                    INFERENCE_TASK_PRIORITY, &inference_handle) != pdPASS) {
        ESP_LOGE(TAG, "Không tạo được inference task");
        vQueueDelete(s_sample_queue);
        s_sample_queue = NULL;
        return;
    }

    if (xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK_SIZE,
                    NULL, SENSOR_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Không tạo được sensor task");
        vTaskDelete(inference_handle);
        vQueueDelete(s_sample_queue);
        s_sample_queue = NULL;
    }
}
