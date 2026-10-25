idf_component_register(
        SRCS "src/sensors_manager.cpp" "src/sensors/bmp280_bme280.cpp"
        INCLUDE_DIRS "include"
        REQUIRES common_types
        PRIV_REQUIRES log esp-idf-lib__bmp280
)
