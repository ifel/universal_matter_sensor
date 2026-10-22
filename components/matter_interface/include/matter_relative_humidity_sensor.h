#ifndef MATTER_RELATIVE_HUMIDITY_SENSOR_H
#define MATTER_RELATIVE_HUMIDITY_SENSOR_H

#include <esp_err.h>
#include <esp_matter.h>

/**
 * @brief Create a Matter relative humidity sensor endpoint.
 *
 * This function sets up a new Matter relative humidity sensor endpoint with the specified properties.
 *
 * @param[in]  matter_node               Pointer to the Node handle.
 * @param[out] endpoint_id               Pointer to store the assigned endpoint ID.
 * @param[in]  min_relative_humidity     Minimum allowable relative humidity (scaled in 0.01 %).
 * @param[in]  max_relative_humidity     Maximum allowable relative humidity (scaled in 0.01 %).
 * @param[in]  initial_relative_humidity Initial relative humidity value (scaled in 0.01 %).
 * @return
 *      - ESP_OK on success.
 *      - ESP_FAIL on failure.
 */
esp_err_t create_relative_humidity_sensor_endpoint(esp_matter::node_t *matter_node, uint16_t *endpoint_id,
                                                   nullable<uint16_t> min_relative_humidity,
                                                   nullable<uint16_t> max_relative_humidity,
                                                   nullable<uint16_t> initial_relative_humidity);

#endif // MATTER_RELATIVE_HUMIDITY_SENSOR_H
