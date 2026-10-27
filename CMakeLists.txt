# Tài liệu hệ thống build ESP-IDF: https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# Giữ đúng thứ tự các lệnh khởi tạo dự án như yêu cầu của ESP-IDF.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(smartwatch)
