#include "model/fall_preprocessing.h"

#define STANDARD_GRAVITY_M_S2 9.80665f
#define DEGREES_TO_RADIANS    0.01745329251994329577f

esp_err_t fall_preprocessing_to_si(
    const mpu6050_data_t *sample,
    float si_channels[FALL_SENSOR_CHANNEL_COUNT])
{
    if (sample == NULL || si_channels == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    si_channels[0] = sample->accel_x_g * STANDARD_GRAVITY_M_S2;
    si_channels[1] = sample->accel_y_g * STANDARD_GRAVITY_M_S2;
    si_channels[2] = sample->accel_z_g * STANDARD_GRAVITY_M_S2;

    si_channels[3] = sample->gyro_x_dps * DEGREES_TO_RADIANS;
    si_channels[4] = sample->gyro_y_dps * DEGREES_TO_RADIANS;
    si_channels[5] = sample->gyro_z_dps * DEGREES_TO_RADIANS;

    return ESP_OK;
}
