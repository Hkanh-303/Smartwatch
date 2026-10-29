#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t fall_detector_init(void);

/* Prepare INT8 model input from a flat [WINDOW_SIZE][NUM_CHANNELS] SI ring.
 * oldest_index identifies the first row in chronological order. */
esp_err_t fall_detector_prepare_input(const float *circular_buffer,
                                      size_t oldest_index);

/* Invoke the model on the currently prepared INT8 input tensor. */
esp_err_t fall_detector_invoke(float *p_fall, float *inference_ms);

/* Apply the configured single-window fall threshold. */
bool fall_detector_is_fall_candidate(float p_fall);

/* Track candidates over the configured temporal window (2-of-3). */
esp_err_t fall_detector_temporal_update(bool candidate,
                                        bool *fall_detected,
                                        uint8_t *fall_votes,
                                        uint8_t *history_count);
void fall_detector_temporal_reset(void);

#ifdef __cplusplus
}
#endif
