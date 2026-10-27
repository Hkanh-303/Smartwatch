#include "config.h"

/* Khởi tạo bus I2C master trên GPIO4 (SDA) và GPIO5 (SCL). */
const i2c_config_t app_i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = APP_I2C_SDA_GPIO,
    .scl_io_num = APP_I2C_SCL_GPIO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = APP_I2C_FREQUENCY_HZ,
};
