#include "matter_temperature_sensor.h"

#include <esp_log.h>
#include <esp_matter.h>

static auto TAG = "   ***matter_interface***   ";

esp_err_t create_temperature_sensor_endpoint(esp_matter::node_t*     matter_node,
                                             uint16_t*               endpoint_id,
                                             const nullable<int16_t> scaled_min_temperature,
                                             const nullable<int16_t> scaled_max_temperature,
                                             const nullable<int16_t> scaled_initial_temperature) {

    esp_matter::endpoint::temperature_sensor::config_t temperature_sensor_endpoint_config;
    temperature_sensor_endpoint_config.temperature_measurement.min_measured_value =
        scaled_min_temperature;
    temperature_sensor_endpoint_config.temperature_measurement.max_measured_value =
        scaled_max_temperature;
    temperature_sensor_endpoint_config.temperature_measurement.measured_value =
        scaled_initial_temperature;

    esp_matter::endpoint_t* temperature_sensor_endpoint =
        esp_matter::endpoint::temperature_sensor::create(matter_node,
                                                         &temperature_sensor_endpoint_config,
                                                         esp_matter::ENDPOINT_FLAG_NONE,
                                                         nullptr);
    if (temperature_sensor_endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create temperature sensor endpoint.");
        return ESP_FAIL;
    }

    *endpoint_id = esp_matter::endpoint::get_id(temperature_sensor_endpoint);
    ESP_LOGI(TAG, "Temperature sensor endpoint created with id %d", *endpoint_id);

    esp_matter::cluster::binding::config_t config;
    esp_matter::cluster::binding::create(
        temperature_sensor_endpoint, &config, esp_matter::CLUSTER_FLAG_SERVER);

    return ESP_OK;
}
