#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "fall_model_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int8_t output_q;
    float probability;
    float inference_ms;
} fall_inference_result_t;

/* Initialize TFLite Micro, validate the embedded model, and run its zero-input sanity test. */
esp_err_t fall_detector_init(void);

/*
 * Run inference directly from a circular [125, 6] buffer in training units.
 * oldest_index identifies the oldest row; the buffer is read chronologically
 * without creating a second, linear copy of the window.
 */
esp_err_t fall_detector_predict_circular(
                                         const float ring[WINDOW_SIZE][NUM_CHANNELS],
                                         size_t oldest_index,
                                         fall_inference_result_t *result);

size_t fall_detector_tensor_arena_used(void);

#ifdef __cplusplus
}
#endif
