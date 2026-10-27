idf_component_register(
    SRCS
        "main.c"
        "src/config.c"
        "src/mpu6050/mpu.c"
        "src/model/fall_preprocessing.c"
        "src/model/fall_detector.cpp"
        "src/model/fall_model_data.cc"
    INCLUDE_DIRS
        "."
        "src"
        "src/model"
    REQUIRES
        driver
        esp-tflite-micro
        esp_timer
        esp_system
)

# Liên kết thư viện toán học dùng để tính góc nghiêng từ dữ liệu gia tốc.
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
