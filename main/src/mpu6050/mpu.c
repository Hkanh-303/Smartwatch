#include "mpu6050/mpu.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_ID               0x68
#define MPU6500_ID               0x70

#define MPU6050_ACCEL_LSB_PER_G  16384.0f
#define MPU6050_GYRO_LSB_PER_DPS 131.0f
#define RAD_TO_DEG               57.2957795f

static const char *TAG = "mpu6050";
static bool s_i2c_driver_installed;

/* Ghi một byte cấu hình vào thanh ghi của MPU6050. */
static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    const uint8_t tx_data[] = {reg, value};

    return i2c_master_write_to_device(APP_I2C_PORT,
                                      MPU6050_I2C_ADDRESS,
                                      tx_data,
                                      sizeof(tx_data),
                                      pdMS_TO_TICKS(APP_I2C_TIMEOUT_MS));
}

/* Đọc liên tiếp các byte bắt đầu từ một thanh ghi. */
static esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_master_write_read_device(APP_I2C_PORT,
                                        MPU6050_I2C_ADDRESS,
                                        &reg,
                                        sizeof(reg),
                                        data,
                                        length,
                                        pdMS_TO_TICKS(APP_I2C_TIMEOUT_MS));
}

/* Ghép hai byte theo định dạng số có dấu 16 bit của MPU6050. */
static int16_t decode_signed_word(const uint8_t *bytes)
{
    const uint16_t word = ((uint16_t)bytes[0] << 8) | (uint16_t)bytes[1];
    return (int16_t)word;
}

/* Cài đặt bus I2C một lần; lỗi được trả về để ứng dụng có thể thử lại. */
static esp_err_t initialize_i2c(void)
{
    if (s_i2c_driver_installed) {
        return ESP_OK;
    }

    esp_err_t err = i2c_param_config(APP_I2C_PORT, &app_i2c_config);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_driver_install(APP_I2C_PORT, app_i2c_config.mode, 0, 0, 0);
    if (err == ESP_OK) {
        s_i2c_driver_installed = true;
    }
    return err;
}

esp_err_t mpu6050_init(void)
{
    esp_err_t err = initialize_i2c();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Không khởi tạo được I2C: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t device_id = 0;
    err = read_registers(MPU6050_REG_WHO_AM_I, &device_id, sizeof(device_id));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Không đọc được WHO_AM_I: %s", esp_err_to_name(err));
        return err;
    }
    if (device_id != MPU6050_ID && device_id != MPU6500_ID) {
        ESP_LOGE(TAG,
                 "WHO_AM_I không được hỗ trợ: nhận 0x%02X (hỗ trợ MPU-6050 0x%02X hoặc MPU-6500 0x%02X)",
                 (unsigned)device_id,
                 (unsigned)MPU6050_ID,
                 (unsigned)MPU6500_ID);
        return ESP_ERR_NOT_FOUND;
    }
    const char *device_name = (device_id == MPU6050_ID) ? "MPU-6050" : "MPU-6500";

    /* Đặt lại cảm biến rồi chờ dao động nội ổn định trước khi cấu hình. */
    err = write_register(MPU6050_REG_PWR_MGMT_1, 0x80);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Chọn đồng hồ gyro, gia tốc ±2 g, gyro ±250 độ/giây và lọc thông thấp. */
    const struct {
        uint8_t reg;
        uint8_t value;
    } settings[] = {
        {MPU6050_REG_PWR_MGMT_1, 0x01},
        {MPU6050_REG_CONFIG, 0x03},
        /* DLPF=3 đặt base rate 1 kHz; divider 19 cho output 50 Hz. */
        {MPU6050_REG_SMPLRT_DIV, (uint8_t)(1000 / SENSOR_SAMPLE_RATE_HZ - 1)},
        {MPU6050_REG_GYRO_CONFIG, 0x00},
        {MPU6050_REG_ACCEL_CONFIG, 0x00},
    };

    for (size_t i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        err = write_register(settings[i].reg, settings[i].value);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Lỗi ghi thanh ghi 0x%02X: %s",
                     settings[i].reg, esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGI(TAG, "Đã khởi tạo %s tại địa chỉ I2C 0x%02X",
             device_name, (unsigned)MPU6050_I2C_ADDRESS);
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_i2c_driver_installed) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Đọc một burst 14 byte gồm gia tốc, nhiệt độ và gyro để giữ đồng bộ mẫu. */
    uint8_t raw[14];
    esp_err_t err = read_registers(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    const int16_t accel_x = decode_signed_word(&raw[0]);
    const int16_t accel_y = decode_signed_word(&raw[2]);
    const int16_t accel_z = decode_signed_word(&raw[4]);
    const int16_t gyro_x = decode_signed_word(&raw[8]);
    const int16_t gyro_y = decode_signed_word(&raw[10]);
    const int16_t gyro_z = decode_signed_word(&raw[12]);

    data->accel_x_g = (float)accel_x / MPU6050_ACCEL_LSB_PER_G;
    data->accel_y_g = (float)accel_y / MPU6050_ACCEL_LSB_PER_G;
    data->accel_z_g = (float)accel_z / MPU6050_ACCEL_LSB_PER_G;
    data->gyro_x_dps = (float)gyro_x / MPU6050_GYRO_LSB_PER_DPS;
    data->gyro_y_dps = (float)gyro_y / MPU6050_GYRO_LSB_PER_DPS;
    data->gyro_z_dps = (float)gyro_z / MPU6050_GYRO_LSB_PER_DPS;

    /* Tính góc nghiêng từ gia tốc; kết quả hợp lệ khi cảm biến gần tĩnh. */
    data->roll_deg = atan2f(data->accel_y_g, data->accel_z_g) * RAD_TO_DEG;
    data->pitch_deg = atan2f(-data->accel_x_g,
                             sqrtf(data->accel_y_g * data->accel_y_g +
                                   data->accel_z_g * data->accel_z_g)) * RAD_TO_DEG;

    return ESP_OK;
}
