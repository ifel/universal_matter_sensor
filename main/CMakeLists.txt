idf_component_register(SRCS "app_main.cpp"
        PRIV_REQUIRES esp_driver_gpio nvs_flash esp_matter matter_interface sensors_manager status_led
        PRIV_INCLUDE_DIRS "."
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")
