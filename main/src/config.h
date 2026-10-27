#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"

/* Cấu hình chân I2C và tốc độ bus cho ESP32-C3. */
#define APP_I2C_PORT             I2C_NUM_0
#define APP_I2C_SDA_GPIO         GPIO_NUM_4
#define APP_I2C_SCL_GPIO         GPIO_NUM_5
#define APP_I2C_FREQUENCY_HZ     100000
#define APP_I2C_TIMEOUT_MS       100

/* MPU-6050/MPU-6500 dùng địa chỉ 0x68 khi chân AD0 được nối mức thấp. */
#define MPU6050_I2C_ADDRESS      0x68

/* Lấy mẫu MPU6050 ở 50 Hz; chia sẻ hàng đợi để không chặn acquisition khi infer. */
#define SENSOR_SAMPLE_RATE_HZ        50
#define SENSOR_SAMPLE_PERIOD_MS      (1000 / SENSOR_SAMPLE_RATE_HZ)
#define SENSOR_SAMPLE_QUEUE_LENGTH   128
#define SENSOR_TASK_STACK_SIZE       4096
#define SENSOR_TASK_PRIORITY         8
#define INFERENCE_TASK_STACK_SIZE    12288
#define INFERENCE_TASK_PRIORITY      5

/* Print one SI-unit sample per second from the low-priority worker; 0 keeps logs quiet. */
#define SENSOR_DEBUG_ENABLED         0
#define SENSOR_DEBUG_INTERVAL_MS     1000

/* Rule-only simulation. Set to 1 to skip MPU/TFLM and exercise 2-of-3 logic. */
#define FALL_SIMULATION_TEST_ENABLED 0

/* Cấu hình I2C được định nghĩa một lần trong config.c. */
extern const i2c_config_t app_i2c_config;
