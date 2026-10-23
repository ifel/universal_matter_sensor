#ifndef MATTER_TEMPERATURE_SENSOR_H
#define MATTER_TEMPERATURE_SENSOR_H

#include <esp_err.h>
#include <esp_matter.h>

/**
 * @brief Create a Matter temperature sensor endpoint.
 *
 * This function sets up a new Matter temperature sensor endpoint with the specified properties.
 *
 * @param[in]  matter_node                Pointer to the Node handle.
 * @param[out] endpoint_id                Pointer to store the assigned endpoint ID.
 * @param[in]  scaled_min_temperature     Minimum allowable temperature (scaled in 0.01 °C).
 * @param[in]  scaled_max_temperature     Maximum allowable temperature (scaled in 0.01 °C).
 * @param[in]  scaled_initial_temperature Initial temperature value (scaled in 0.01 °C).
 * @return
 *      - ESP_OK on success.
 *      - ESP_FAIL on failure.
 */
esp_err_t createTemperatureSensorEndpoint(esp_matter::node_t* matter_node,
                                          uint16_t* endpoint_id,
                                          nullable<int16_t> scaled_min_temperature,
                                          nullable<int16_t> scaled_max_temperature,
                                          nullable<int16_t> scaled_initial_temperature);

#endif // MATTER_TEMPERATURE_SENSOR_H
