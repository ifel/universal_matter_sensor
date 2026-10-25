idf_component_register(
	PRIV_REQUIRES esp_matter_console common_types
        SRCS "src/matter_manager.cpp"
        INCLUDE_DIRS "include"
        REQUIRES esp_matter common_types
)
