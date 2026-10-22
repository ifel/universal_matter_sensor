idf_component_register(
	PRIV_REQUIRES esp_matter_console
        SRCS "src/matter_manager.cpp" "src/matter_temperature_sensor.cpp" "src/matter_pressure_sensor.cpp" "src/matter_relative_humidity_sensor.cpp"
        INCLUDE_DIRS "include"
        REQUIRES esp_matter
)
