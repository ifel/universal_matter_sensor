#ifndef MATTER_PRESSURE_SENSOR_H
#define MATTER_PRESSURE_SENSOR_H

#include <esp_err.h>
#include <esp_matter.h>

/**
 * @brief Create a Matter pressure sensor endpoint.
 *
 * This function sets up a new Matter pressure sensor endpoint with the specified properties.
 *
 * @param[in]  matter_node      Pointer to the Node handle.
 * @param[out] endpoint_id      Pointer to store the assigned endpoint ID.
 * @param[in]  min_pressure     Minimum allowable pressure (scaled in 0.1 kPa).
 * @param[in]  max_pressure     Maximum allowable pressure (scaled in 0.1 kPa).
 * @param[in]  initial_pressure Initial pressure value (scaled in 0.1 kPa).
 * @return
 *      - ESP_OK on success.
 *      - ESP_FAIL on failure.
 */
esp_err_t createPressureSensorEndpoint(esp_matter::node_t* matter_node,
                                       uint16_t* endpoint_id,
                                       int16_t min_pressure,
                                       int16_t max_pressure,
                                       int16_t initial_pressure);

#endif // MATTER_PRESSURE_SENSOR_H
