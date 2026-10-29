#pragma once

#include "mpu6050/mpu.h"

enum { FALL_SENSOR_CHANNEL_COUNT = 6 };

/* Trả về Ax, Ay, Az, Gx, Gy, Gz theo đơn vị SI, giữ nguyên trọng lực. */
esp_err_t fall_preprocessing_to_si(
    const mpu6050_data_t *sample,
    float si_channels[FALL_SENSOR_CHANNEL_COUNT]);
