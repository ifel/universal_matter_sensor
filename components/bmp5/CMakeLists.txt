if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/third_party/bmp5.c")
    message(FATAL_ERROR "BMP5 SensorAPI submodule is missing: run `git submodule update --init`")
endif()

idf_component_register(
        SRCS "bmp5_i2c.c" "third_party/bmp5.c"
        INCLUDE_DIRS "include" "third_party"
        REQUIRES esp-idf-lib__i2cdev
        PRIV_REQUIRES esp_rom esp-idf-lib__esp_idf_lib_helpers
)
