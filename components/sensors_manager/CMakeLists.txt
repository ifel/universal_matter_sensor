idf_component_register(
        SRCS "src/sensors_manager.cpp" "src/sensors/bmp280_bme280.cpp" "src/sensors/sht4x.cpp" "src/sensors/scd30.cpp" "src/sensors/veml7700.cpp" "src/sensors/ens160.cpp" "src/sensors/bmp581.cpp" "src/sensors/tsl2591.cpp" "src/sensors/vl53l1x.cpp" "src/sensors/tmp117.cpp" "src/sensors/max17048.cpp" "src/sensors/ld2410s.cpp"
        INCLUDE_DIRS "include"
        REQUIRES common_types
        PRIV_REQUIRES log esp-idf-lib__bmp280 esp-idf-lib__sht4x esp-idf-lib__scd30 esp-idf-lib__veml7700 esp-idf-lib__tsl2591 esp-idf-lib__max1704x esp-idf-lib__i2cdev grrtzm__vl53l1x_library ens160 bmp5 tmp117 ld2410s esp_timer
)
