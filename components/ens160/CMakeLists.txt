idf_component_register(
        SRCS "ens160.c"
        INCLUDE_DIRS "include"
        REQUIRES esp-idf-lib__i2cdev
        PRIV_REQUIRES log esp_timer esp-idf-lib__esp_idf_lib_helpers
)
