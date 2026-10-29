#pragma once

#include <cstdint>

// ============================================================
// CẤU HÌNH CUỐI CÙNG - FALL DETECTION CNN v2 INT8
// ============================================================

// Tần số lấy mẫu: 50 mẫu mỗi giây
constexpr int SAMPLE_RATE_HZ = 50;

// Một cửa sổ = 125 mẫu x 6 kênh
constexpr int WINDOW_SIZE = 125;
constexpr int NUM_CHANNELS = 6;

// Sau mỗi 62 mẫu mới, chạy suy luận một lần
constexpr int STEP_SIZE = 62;

// Thứ tự kênh:
// 0=Ax, 1=Ay, 2=Az, 3=Gx, 4=Gy, 5=Gz

// Mean/Std chỉ được tính từ tập Train.
// Không tính lại các giá trị này trên ESP32-C3.
constexpr float CHANNEL_MEAN[NUM_CHANNELS] = {
    -4.315098300f,
    -3.462477400f,
     2.905352600f,
    -0.0018140064f,
    -0.0049290424f,
    -0.0099359490f
};

constexpr float CHANNEL_STD[NUM_CHANNELS] = {
    6.062491400f,
    5.307693000f,
    5.512110700f,
    1.594423500f,
    1.121074900f,
    1.245484100f
};

// Lượng tử hóa input INT8
constexpr float INPUT_SCALE = 0.11888055503368378f;
constexpr int INPUT_ZERO_POINT = 14;

// Giải lượng tử hóa output INT8
constexpr float OUTPUT_SCALE = 0.00390625f;
constexpr int OUTPUT_ZERO_POINT = -128;

// Ngưỡng quyết định
constexpr float FALL_THRESHOLD = 0.30f;

// Giá trị INT8 gần nhất tương ứng với ngưỡng 0.30
constexpr int8_t FALL_THRESHOLD_INT8 = -51;

// Quy tắc thời gian 2-of-3
constexpr int TEMPORAL_WINDOW_COUNT = 3;
constexpr int TEMPORAL_REQUIRED_FALLS = 2;
