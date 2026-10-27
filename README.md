# Đồng hồ thông minh ESP32-C3 — Thử nghiệm phát hiện té ngã

Đây là dự án ESP-IDF dành cho ESP32-C3, sử dụng cảm biến MPU-6050/MPU-6500 và
mô hình phân loại té ngã INT8 chạy bằng TensorFlow Lite Micro.

## Chế độ phát hiện té ngã thời gian thực

Cấu hình mặc định chạy chế độ `REALTIME FALL DETECTION TEST`:

- I2C0: SDA ở GPIO4, SCL ở GPIO5, tốc độ 100 kHz, địa chỉ cảm biến `0x68`.
- Tần số lấy mẫu 50 Hz; dữ liệu đầu vào gồm 125 mẫu × 6 kênh theo thứ tự Ax,
  Ay, Az, Gx, Gy, Gz.
- Dữ liệu cảm biến được đổi sang m/s² và rad/s, giữ nguyên thành phần trọng lực,
  sau đó chuẩn hóa bằng mean/std cố định của mô hình đã huấn luyện.
- Lần suy luận đầu tiên chạy sau khi thu đủ 125 mẫu; các lần tiếp theo chạy sau
  mỗi 62 mẫu mới.
- `P(FALL) >= 0.30` được xem là một dự đoán té ngã tiềm năng. Sự kiện được xác
  nhận khi có ít nhất 2 trong 3 dự đoán gần nhất vượt ngưỡng. Trạng thái sự kiện
  được giữ cho đến khi có 3 dự đoán bình thường liên tiếp.
- Tác vụ lấy mẫu ưu tiên cao đưa dữ liệu vào hàng đợi độc lập với tác vụ suy luận
  và việc ghi Serial. Nếu phát hiện mẫu bị gián đoạn, cửa sổ dữ liệu hiện tại sẽ
  được xóa.

Thông tin về mô hình, tham số chuẩn hóa, ngưỡng và quy tắc phát hiện nằm trong
[ghi chú về mô hình](main/src/model/README.md).

## Build, nạp firmware và xem log

Mở terminal ESP-IDF tại thư mục gốc của dự án rồi chạy:

```sh
idf.py set-target esp32c3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Thay `/dev/ttyACM0` bằng cổng ESP32-C3 trên máy của bạn. Serial Monitor hiển
thị trạng thái khởi tạo, dung lượng RAM/arena, tần số lấy mẫu, thời gian suy luận,
`P(FALL)` và trạng thái `NORMAL`, `FALL CANDIDATE` hoặc `FALL DETECTED`.

## Các chế độ tùy chọn

Trong `main/src/config.h`:

- Đặt `SENSOR_DEBUG_ENABLED` thành `1` để in sáu kênh dữ liệu cảm biến theo đơn
  vị SI và tần số lấy mẫu thực tế khoảng mỗi giây. Mặc định là `0`.
- Đặt `FALL_SIMULATION_TEST_ENABLED` thành `1` để chạy mô phỏng bằng các điểm số
  giả lập, kiểm tra trạng thái dự đoán tiềm năng, xác nhận 2-of-3, giữ trạng thái
  sự kiện và kích hoạt lại. Chế độ này bỏ qua việc khởi tạo MPU và TFLM; kết quả
  không dùng để đánh giá độ chính xác của mô hình. Mặc định là `0`.

Đặt cả hai tùy chọn về `0` để chạy suy luận thời gian thực với cảm biến.
