idf_component_register(
        SRCS "src/sensors_manager.cpp" "src/sensors/bmp280_bme280.cpp" "src/sensors/sht4x.cpp" "src/sensors/scd30.cpp" "src/sensors/veml7700.cpp" "src/sensors/ens160.cpp" "src/sensors/bmp581.cpp"
        INCLUDE_DIRS "include"
        REQUIRES common_types
        PRIV_REQUIRES log esp-idf-lib__bmp280 esp-idf-lib__sht4x esp-idf-lib__scd30 esp-idf-lib__veml7700 ens160 bmp5
)
