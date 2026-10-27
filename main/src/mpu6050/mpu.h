#pragma once

#include <stdint.h>

#include "esp_err.h"

/* Accel dùng g và vẫn bao gồm trọng lực; gyro dùng độ/giây. */
typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_deg;
    float pitch_deg;
} mpu6050_data_t;

/* Khởi tạo I2C, nhận diện MPU-6050/MPU-6500 và cấu hình dải đo cảm biến. */
esp_err_t mpu6050_init(void);

/* Đọc gia tốc, vận tốc góc và tính góc nghiêng roll/pitch. */
esp_err_t mpu6050_read(mpu6050_data_t *data);
