#include "matter_relative_humidity_sensor.h"

#include <esp_log.h>
#include <esp_matter.h>

static auto TAG = "   ***matter_interface***   ";

esp_err_t
create_relative_humidity_sensor_endpoint(esp_matter::node_t*      matter_node,
                                         uint16_t*                endpoint_id,
                                         const nullable<uint16_t> min_relative_humidity,
                                         const nullable<uint16_t> max_relative_humidity,
                                         const nullable<uint16_t> initial_relative_humidity) {

    esp_matter::endpoint::humidity_sensor::config_t humidity_sensor_endpoint_config;
    humidity_sensor_endpoint_config.relative_humidity_measurement.min_measured_value =
        min_relative_humidity;
    humidity_sensor_endpoint_config.relative_humidity_measurement.max_measured_value =
        max_relative_humidity;
    humidity_sensor_endpoint_config.relative_humidity_measurement.measured_value =
        initial_relative_humidity;

    esp_matter::endpoint_t* humidity_sensor_endpoint =
        esp_matter::endpoint::humidity_sensor::create(
            matter_node, &humidity_sensor_endpoint_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    if (humidity_sensor_endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create relative humidity sensor endpoint.");
        return ESP_FAIL;
    }

    *endpoint_id = esp_matter::endpoint::get_id(humidity_sensor_endpoint);
    ESP_LOGI(TAG, "Relative humidity sensor endpoint created with id %d", *endpoint_id);

    esp_matter::cluster::binding::config_t config;
    esp_matter::cluster::binding::create(
        humidity_sensor_endpoint, &config, esp_matter::CLUSTER_FLAG_SERVER);

    return ESP_OK;
}