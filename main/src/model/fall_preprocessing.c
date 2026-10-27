#include "model/fall_preprocessing.h"

#include <stddef.h>

#define STANDARD_GRAVITY_M_S2 9.80665f
#define DEGREES_TO_RADIANS    0.01745329251994329577f

void fall_preprocessing_from_mpu(const mpu6050_data_t *sample,
                                 float training_units[6])
{
    if (sample == NULL || training_units == NULL) {
        return;
    }

    /* The accelerometer values include gravity; do not subtract 1 g or estimate it away. */
    training_units[0] = sample->accel_x_g * STANDARD_GRAVITY_M_S2;
    training_units[1] = sample->accel_y_g * STANDARD_GRAVITY_M_S2;
    training_units[2] = sample->accel_z_g * STANDARD_GRAVITY_M_S2;
    training_units[3] = sample->gyro_x_dps * DEGREES_TO_RADIANS;
    training_units[4] = sample->gyro_y_dps * DEGREES_TO_RADIANS;
    training_units[5] = sample->gyro_z_dps * DEGREES_TO_RADIANS;
}
