idf_component_register(SRCS "app_main.cpp"
        PRIV_REQUIRES nvs_flash esp_matter matter_interface
        PRIV_INCLUDE_DIRS "."
)

set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 17)
target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")
