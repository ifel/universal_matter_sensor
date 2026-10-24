idf_component_register(
	PRIV_REQUIRES esp_matter_console esp_driver_gpio esp_timer
        SRCS "src/matter_manager.cpp"
        INCLUDE_DIRS "include"
        REQUIRES esp_matter
)
