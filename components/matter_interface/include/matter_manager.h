#ifndef MATTER_MANAGER_H
#define MATTER_MANAGER_H

#include <esp_err.h>
#include <esp_matter.h>

esp_err_t matterInit(uint16_t* temperature_sensor_endpoint_id,
                     uint16_t* pressure_sensor_endpoint_id,
                     uint16_t* relative_humidity_sensor_endpoint_id,
                     float min_temperature,
                     float max_temperature,
                     float initial_temperature,
                     float min_pressure,
                     float max_pressure,
                     float initial_pressure,
                     float min_relative_humidity,
                     float max_relative_humidity,
                     float initial_relative_humidity);

esp_err_t matterUpdateValues(uint16_t temperature_endpoint_id,
                             uint16_t pressure_sensor_endpoint_id,
                             const uint16_t* relative_humidity_endpoint_id,
                             float new_temperature,
                             float new_pressure,
                             float new_relative_humidity);

#endif // MATTER_MANAGER_H
