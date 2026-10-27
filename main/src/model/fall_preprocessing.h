#pragma once

#include "mpu6050/mpu.h"

/* Convert driver units to the training units, preserving the gravity component. */
void fall_preprocessing_from_mpu(const mpu6050_data_t *sample,
                                 float training_units[6]);
