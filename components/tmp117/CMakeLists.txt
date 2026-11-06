idf_component_register(
        SRCS "tmp117.c"
        INCLUDE_DIRS "include"
        REQUIRES esp-idf-lib__i2cdev
        PRIV_REQUIRES log freertos esp-idf-lib__esp_idf_lib_helpers
)
