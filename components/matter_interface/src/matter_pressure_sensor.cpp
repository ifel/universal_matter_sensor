#include "matter_pressure_sensor.h"

#include <esp_log.h>
#include <esp_matter.h>

constexpr const char* tag = "   ***matter_interface***   ";

/// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t createPressureSensorEndpoint(esp_matter::node_t* matter_node,
                                       uint16_t* endpoint_id,
                                       const int16_t min_pressure,
                                       const int16_t max_pressure,
                                       const int16_t initial_pressure) {

    // Validate that inputs are within the int16_t range
    if (min_pressure > max_pressure) {
        ESP_LOGE(tag, "Min pressure cannot be greater than max pressure.");
        return ESP_FAIL;
    }
    if (initial_pressure < min_pressure || initial_pressure > max_pressure) {
        ESP_LOGE(tag, "Initial pressure must be within min and max pressure range.");
        return ESP_FAIL;
    }

    // Set up the pressure sensor endpoint
    esp_matter::endpoint::pressure_sensor::config_t pressure_sensor_endpoint_config;
    pressure_sensor_endpoint_config.pressure_measurement.min_measured_value =
        static_cast<nullable<int16_t>>(min_pressure);
    pressure_sensor_endpoint_config.pressure_measurement.max_measured_value =
        static_cast<nullable<int16_t>>(max_pressure);
    pressure_sensor_endpoint_config.pressure_measurement.measured_value =
        static_cast<nullable<int16_t>>(initial_pressure);

    esp_matter::endpoint_t* pressure_sensor_endpoint =
        esp_matter::endpoint::pressure_sensor::create(
            matter_node, &pressure_sensor_endpoint_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    if (pressure_sensor_endpoint == nullptr) {
        ESP_LOGE(tag, "Failed to create pressure sensor endpoint.");
        return ESP_FAIL;
    }

    *endpoint_id = esp_matter::endpoint::get_id(pressure_sensor_endpoint);
    ESP_LOGI(tag, "Pressure sensor endpoint created with id %d", *endpoint_id);

    esp_matter::cluster::binding::config_t config;
    esp_matter::cluster::binding::create(
        pressure_sensor_endpoint, &config, esp_matter::CLUSTER_FLAG_SERVER);

    return ESP_OK;
}
