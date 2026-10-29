idf_component_register(
    SRCS
        "main.c"
        "src/config.c"
        "src/mpu6050/mpu.c"
        "src/model/fall_model_data.cc"
         "src/model/fall_detector.cpp"
                 "src/model/fall_preprocessing.c"
    INCLUDE_DIRS
        "."
        "src"
        "src/model"
    REQUIRES
        driver
        esp_timer
        esp_system
        esp-tflite-micro
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)