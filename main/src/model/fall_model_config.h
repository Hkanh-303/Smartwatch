#pragma once

#include <stdint.h>

// ============================================================
// CẤU HÌNH CUỐI CÙNG - FALL DETECTION CNN v2 INT8
// ============================================================

// Tần số lấy mẫu: 50 mẫu mỗi giây
#define SAMPLE_RATE_HZ 50

// Một cửa sổ = 125 mẫu x 6 kênh
#define WINDOW_SIZE 125
#define NUM_CHANNELS 6

// Sau mỗi 62 mẫu mới, chạy suy luận một lần
#define STEP_SIZE 62

// Thứ tự kênh:
// 0=Ax, 1=Ay, 2=Az, 3=Gx, 4=Gy, 5=Gz
// Đơn vị train: accel m/s^2 (giữ trọng lực), gyro rad/s.

// Mean/Std chỉ được tính từ tập Train.
// Không tính lại các giá trị này trên ESP32-C3.
#ifdef __cplusplus
static const float CHANNEL_MEAN[NUM_CHANNELS] = {
    -4.315098300f,
    -3.462477400f,
     2.905352600f,
    -0.0018140064f,
    -0.0049290424f,
    -0.0099359490f
};

static const float CHANNEL_STD[NUM_CHANNELS] = {
    6.062491400f,
    5.307693000f,
    5.512110700f,
    1.594423500f,
    1.121074900f,
    1.245484100f
};
#endif

// Lượng tử hóa input INT8
#define INPUT_SCALE 0.11888055503368378f
#define INPUT_ZERO_POINT 14

// Giải lượng tử hóa output INT8
#define OUTPUT_SCALE 0.00390625f
#define OUTPUT_ZERO_POINT (-128)

// Ngưỡng quyết định
#define FALL_THRESHOLD 0.30f

// Giá trị INT8 gần nhất tương ứng với ngưỡng 0.30
#define FALL_THRESHOLD_INT8 ((int8_t)-51)

// Quy tắc thời gian 2-of-3
#define TEMPORAL_WINDOW_COUNT 3
#define TEMPORAL_REQUIRED_FALLS 2
