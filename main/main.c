#include "config.h"
#include "model/fall_detector.h"
#include "model/fall_model_config.h"
#include "model/fall_preprocessing.h"
#include "mpu6050/mpu.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#if SENSOR_SAMPLE_RATE_HZ != SAMPLE_RATE_HZ
#error "MPU sampling frequency must match the model training frequency"
#endif

typedef struct {
    float channels[NUM_CHANNELS];
    int64_t timestamp_us;
    uint32_t sequence;
} sensor_sample_t;

typedef struct {
    bool candidates[TEMPORAL_WINDOW_COUNT];
    size_t count;
    size_t next;
    bool latched;
    uint32_t consecutive_normal;
} fall_temporal_state_t;

static const char *TAG = "smartwatch";
#if !FALL_SIMULATION_TEST_ENABLED
static QueueHandle_t s_sample_queue;
static bool s_ai_ready;
#endif

static void clear_temporal_state(fall_temporal_state_t *state)
{
    memset(state, 0, sizeof(*state));
}

static const char *update_temporal_decision(fall_temporal_state_t *state,
                                            bool candidate,
                                            bool *fall_detected_now)
{
    *fall_detected_now = false;
    state->candidates[state->next] = candidate;
    state->next = (state->next + 1U) % TEMPORAL_WINDOW_COUNT;
    if (state->count < TEMPORAL_WINDOW_COUNT) {
        ++state->count;
    }

    if (candidate) {
        state->consecutive_normal = 0;
    } else if (state->consecutive_normal < UINT32_MAX) {
        ++state->consecutive_normal;
    }

    size_t fall_count = 0;
    for (size_t i = 0; i < state->count; ++i) {
        fall_count += state->candidates[i] ? 1U : 0U;
    }

    if (state->latched && state->consecutive_normal >= TEMPORAL_WINDOW_COUNT) {
        clear_temporal_state(state);
        return "NORMAL (rearmed)";
    }

    if (!state->latched && state->count >= TEMPORAL_REQUIRED_FALLS &&
        fall_count >= TEMPORAL_REQUIRED_FALLS) {
        state->latched = true;
        *fall_detected_now = true;
        return "FALL DETECTED!";
    }

    if (state->latched) {
        return "FALL DETECTED (latched)";
    }
    if (candidate) {
        return "FALL CANDIDATE";
    }
    return "NORMAL";
}

#if FALL_SIMULATION_TEST_ENABLED
static bool run_temporal_simulation_test(void)
{
    typedef struct {
        float probability;
        const char *expected_status;
        bool expected_event;
    } simulation_step_t;

    /* Synthetic scores test only the temporal rule; the CNN is never invoked here. */
    static const simulation_step_t steps[] = {
        {0.10f, "NORMAL", false},
        {0.80f, "FALL CANDIDATE", false},
        {0.15f, "NORMAL", false},
        {0.10f, "NORMAL", false},
        {0.10f, "NORMAL", false},
        {0.10f, "NORMAL", false},
        {0.80f, "FALL CANDIDATE", false},
        {0.90f, "FALL DETECTED!", true},
        {0.10f, "FALL DETECTED (latched)", false},
        {0.10f, "FALL DETECTED (latched)", false},
        {0.10f, "NORMAL (rearmed)", false},
    };

    fall_temporal_state_t state = {0};
    bool all_steps_passed = true;
    size_t detected_events = 0;

    ESP_LOGW(TAG, "[SIM] Rule-only simulation: synthetic scores; AI/MPU are not evaluated");
    for (size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); ++i) {
        bool detected_now = false;
        const bool candidate = steps[i].probability >= FALL_THRESHOLD;
        const char *status = update_temporal_decision(&state, candidate, &detected_now);
        const bool passed = (strcmp(status, steps[i].expected_status) == 0) &&
                            (detected_now == steps[i].expected_event);
        all_steps_passed = all_steps_passed && passed;
        detected_events += detected_now ? 1U : 0U;
        ESP_LOGI(TAG, "[SIM] step=%u score=%.2f -> %s | event=%s | %s",
                 (unsigned)(i + 1U), steps[i].probability, status,
                 detected_now ? "yes" : "no", passed ? "PASS" : "FAIL");
    }

    all_steps_passed = all_steps_passed && (detected_events == 1U);
    ESP_LOGI(TAG, "[SIM] Result: %s; expected one event and rearm after 3 normals",
             all_steps_passed ? "PASS" : "FAIL");
    return all_steps_passed;
}
#endif

#if !FALL_SIMULATION_TEST_ENABLED
static float measure_window_rate(const int64_t timestamps[WINDOW_SIZE],
                                 size_t oldest_index,
                                 float *min_interval_ms,
                                 float *max_interval_ms)
{
    int64_t min_interval_us = INT64_MAX;
    int64_t max_interval_us = 0;

    for (size_t i = 0; i + 1U < WINDOW_SIZE; ++i) {
        const size_t current = (oldest_index + i) % WINDOW_SIZE;
        const size_t next = (current + 1U) % WINDOW_SIZE;
        const int64_t interval_us = timestamps[next] - timestamps[current];
        if (interval_us < min_interval_us) {
            min_interval_us = interval_us;
        }
        if (interval_us > max_interval_us) {
            max_interval_us = interval_us;
        }
    }

    *min_interval_ms = (float)min_interval_us / 1000.0f;
    *max_interval_ms = (float)max_interval_us / 1000.0f;

    const size_t newest_index =
        (oldest_index + WINDOW_SIZE - 1U) % WINDOW_SIZE;
    const int64_t duration_us = timestamps[newest_index] - timestamps[oldest_index];
    if (duration_us <= 0) {
        return 0.0f;
    }
    return (float)(WINDOW_SIZE - 1U) * 1000000.0f / (float)duration_us;
}

/* High priority: collect MPU samples independently of the lower-priority TFLM worker. */
static void sensor_task(void *arg)
{
    (void)arg;
    const TickType_t period_ticks = pdMS_TO_TICKS(SENSOR_SAMPLE_PERIOD_MS);
    const TickType_t retry_ticks = pdMS_TO_TICKS(500);
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t retry_after = 0;
    bool sensor_ready = false;
    uint32_t sequence = 0;
    uint32_t queue_drop_count = 0;
    int64_t last_error_log_us = 0;
    int64_t last_queue_log_us = 0;

    for (;;) {
        if (!sensor_ready) {
            const TickType_t now_ticks = xTaskGetTickCount();
            if ((int32_t)(now_ticks - retry_after) >= 0) {
                const esp_err_t init_err = mpu6050_init();
                sensor_ready = (init_err == ESP_OK);
                if (sensor_ready) {
                    ESP_LOGI(TAG, "[INIT] MPU6050/6500: OK");
                    last_wake = xTaskGetTickCount();
                } else {
                    retry_after = xTaskGetTickCount() + retry_ticks;
                }
            }
            if (!sensor_ready) {
                vTaskDelayUntil(&last_wake, period_ticks);
                continue;
            }
        }

        const uint32_t sample_sequence = sequence++;
        mpu6050_data_t sensor_data;
        const esp_err_t read_err = mpu6050_read(&sensor_data);
        if (read_err == ESP_OK) {
            sensor_sample_t sample = {0};
            fall_preprocessing_from_mpu(&sensor_data, sample.channels);
            sample.timestamp_us = esp_timer_get_time();
            sample.sequence = sample_sequence;

            if (xQueueSend(s_sample_queue, &sample, 0) != pdTRUE) {
                ++queue_drop_count;
                const int64_t now_us = sample.timestamp_us;
                if (now_us - last_queue_log_us >= 1000000) {
                    ESP_LOGW(TAG, "Sample queue đầy; đã bỏ %lu mẫu",
                             (unsigned long)queue_drop_count);
                    last_queue_log_us = now_us;
                }
            }
        } else {
            sensor_ready = false;
            retry_after = xTaskGetTickCount() + retry_ticks;
            const int64_t now_us = esp_timer_get_time();
            if (now_us - last_error_log_us >= 1000000) {
                ESP_LOGW(TAG, "Đọc MPU thất bại: %s; sẽ thử kết nối lại",
                         esp_err_to_name(read_err));
                last_error_log_us = now_us;
            }
        }

        if (xTaskDelayUntil(&last_wake, period_ticks) == pdFALSE) {
            /* Mark the missing time slot so the consumer discards a discontinuous window. */
            ++sequence;
            last_wake = xTaskGetTickCount();
        }
    }
}

static void inference_task(void *arg)
{
    (void)arg;
    sensor_sample_t sample;

    if (!s_ai_ready) {
        ESP_LOGE(TAG, "AI chưa sẵn sàng; inference worker chỉ nhận và bỏ mẫu để tránh nghẽn queue");
        for (;;) {
            (void)xQueueReceive(s_sample_queue, &sample, portMAX_DELAY);
        }
    }

    float ring[WINDOW_SIZE][NUM_CHANNELS] = {{0}};
    int64_t timestamp_ring[WINDOW_SIZE] = {0};
    size_t write_index = 0;
    size_t samples_in_window = 0;
    size_t samples_since_prediction = 0;
    bool first_prediction_done = false;
    bool have_previous_sequence = false;
    uint32_t previous_sequence = 0;
    fall_temporal_state_t temporal_state = {0};
#if SENSOR_DEBUG_ENABLED
    int64_t debug_window_start_us = 0;
    uint32_t debug_sample_count = 0;
#endif

    ESP_LOGI(TAG, "[INIT] Sampling target: %d Hz; window=%d; step=%d",
             SAMPLE_RATE_HZ, WINDOW_SIZE, STEP_SIZE);

    for (;;) {
        if (xQueueReceive(s_sample_queue, &sample, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (have_previous_sequence) {
            const uint32_t sequence_gap = sample.sequence - previous_sequence;
            if (sequence_gap != 1U) {
                const uint32_t lost_samples = sequence_gap - 1U;
                ESP_LOGW(TAG, "Sample gap=%lu; xóa cửa sổ và lịch sử quyết định",
                         (unsigned long)lost_samples);
                write_index = 0;
                samples_in_window = 0;
                samples_since_prediction = 0;
                first_prediction_done = false;
                clear_temporal_state(&temporal_state);
#if SENSOR_DEBUG_ENABLED
                debug_window_start_us = 0;
                debug_sample_count = 0;
#endif
            }
        }
        previous_sequence = sample.sequence;
        have_previous_sequence = true;

#if SENSOR_DEBUG_ENABLED
        if (debug_window_start_us == 0) {
            debug_window_start_us = sample.timestamp_us;
            debug_sample_count = 1;
        } else {
            ++debug_sample_count;
            const int64_t elapsed_us = sample.timestamp_us - debug_window_start_us;
            if (elapsed_us >= (int64_t)SENSOR_DEBUG_INTERVAL_MS * 1000) {
                const float debug_hz = (float)(debug_sample_count - 1U) *
                                       1000000.0f / (float)elapsed_us;
                ESP_LOGI(TAG,
                         "[DEBUG] Ax=%.2f Ay=%.2f Az=%.2f m/s^2 | "
                         "Gx=%.3f Gy=%.3f Gz=%.3f rad/s | fs=%.2f Hz",
                         sample.channels[0], sample.channels[1], sample.channels[2],
                         sample.channels[3], sample.channels[4], sample.channels[5],
                         debug_hz);
                debug_window_start_us = sample.timestamp_us;
                debug_sample_count = 1;
            }
        }
#endif

        memcpy(ring[write_index], sample.channels, sizeof(ring[write_index]));
        timestamp_ring[write_index] = sample.timestamp_us;
        write_index = (write_index + 1U) % WINDOW_SIZE;
        if (samples_in_window < WINDOW_SIZE) {
            ++samples_in_window;
        }
        if (samples_in_window < WINDOW_SIZE) {
            continue;
        }

        if (first_prediction_done) {
            ++samples_since_prediction;
            if (samples_since_prediction < STEP_SIZE) {
                continue;
            }
            samples_since_prediction = 0;
        }

        fall_inference_result_t result;
        /* write_index is the oldest row when the circular window is full. */
        const esp_err_t infer_err =
            fall_detector_predict_circular(ring, write_index, &result);
        if (infer_err != ESP_OK) {
            ESP_LOGE(TAG, "Inference thất bại: %s", esp_err_to_name(infer_err));
            first_prediction_done = true;
            continue;
        }
        first_prediction_done = true;

        const bool candidate = result.probability >= FALL_THRESHOLD;
        bool detected_now = false;
        const char *status =
            update_temporal_decision(&temporal_state, candidate, &detected_now);
        float min_interval_ms = 0.0f;
        float max_interval_ms = 0.0f;
        const float measured_hz = measure_window_rate(timestamp_ring, write_index,
                                                     &min_interval_ms,
                                                     &max_interval_ms);

        ESP_LOGI(TAG,
                 "[AI] P(FALL): %.3f | Status: %s | Inference: %.2f ms | "
                 "Sampling: %.2f Hz (dt %.2f..%.2f ms, q=%u)",
                 result.probability, status, result.inference_ms, measured_hz,
                 min_interval_ms, max_interval_ms,
                 (unsigned)uxQueueMessagesWaiting(s_sample_queue));
        if (detected_now) {
            const size_t newest_index = (write_index + WINDOW_SIZE - 1U) % WINDOW_SIZE;
            const int64_t event_window_end_ms = timestamp_ring[newest_index] / 1000;
            const int64_t detected_at_ms = esp_timer_get_time() / 1000;
            ESP_LOGW(TAG,
                     "[EVENT] FALL DETECTED | detected_at=%lld ms | window_end=%lld ms | "
                     "P(FALL)=%.3f | inference=%.2f ms | rule=2-of-3",
                     (long long)detected_at_ms, (long long)event_window_end_ms,
                     result.probability, result.inference_ms);
        }
        if (max_interval_ms > (1000.0f / SAMPLE_RATE_HZ) * 1.5f) {
            ESP_LOGW(TAG, "Sampling interval lệch mục tiêu 50 Hz; kiểm tra I2C/tải CPU");
        }
    }
}
#endif

void app_main(void)
{
#if FALL_SIMULATION_TEST_ENABLED
    const bool simulation_passed = run_temporal_simulation_test();
    if (!simulation_passed) {
        ESP_LOGE(TAG, "[SIM] Temporal decision test failed");
    }
    return;
#else
    ESP_LOGI(TAG, "[INIT] Mode: REALTIME FALL DETECTION TEST");
    ESP_LOGI(TAG, "[INIT] Sampling: %d Hz; window=%d x %d; step=%d",
             SAMPLE_RATE_HZ, WINDOW_SIZE, NUM_CHANNELS, STEP_SIZE);

    const esp_err_t ai_err = fall_detector_init();
    s_ai_ready = (ai_err == ESP_OK);
    ESP_LOGI(TAG, "[INIT] AI Model: %s", s_ai_ready ? "OK" : "FAILED");

    s_sample_queue = xQueueCreate(SENSOR_SAMPLE_QUEUE_LENGTH,
                                  sizeof(sensor_sample_t));
    if (s_sample_queue == NULL) {
        ESP_LOGE(TAG, "Không tạo được sample queue");
        return;
    }

    if (xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK_SIZE,
                    NULL, SENSOR_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Không tạo được sensor task");
        return;
    }
    if (xTaskCreate(inference_task, "inference_task", INFERENCE_TASK_STACK_SIZE,
                    NULL, INFERENCE_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Không tạo được inference task");
        return;
    }

    ESP_LOGI(TAG, "[INIT] Runtime RAM: free heap=%u bytes; Tensor Arena used=%u bytes",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)fall_detector_tensor_arena_used());
#endif
}
