#ifndef MATTER_MANAGER_H
#define MATTER_MANAGER_H

#include <esp_err.h>
#include <esp_matter.h>

class MatterManager {
  public:
    MatterManager() = default;
    ~MatterManager() = default;
    // Prevent copying
    MatterManager(const MatterManager&) = delete;
    MatterManager& operator=(const MatterManager&) = delete;

    // Prevent moving
    MatterManager(MatterManager&&) = delete;
    MatterManager& operator=(MatterManager&&) = delete;

    esp_err_t init(float initial_temperature,
                   float initial_pressure,
                   float initial_relative_humidity,
                   bool is_bme280);
    esp_err_t updateValues(float new_temperature, float new_pressure, float new_relative_humidity);

  private:
    /**
     * @brief Create a Matter temperature sensor endpoint.
     *
     * This function sets up a new Matter temperature sensor endpoint with the specified properties.
     *
     * @param[in]  matter_node                Pointer to the Node handle.
     * @param[in]  scaled_min_temperature     Minimum allowable temperature (scaled in 0.01 °C).
     * @param[in]  scaled_max_temperature     Maximum allowable temperature (scaled in 0.01 °C).
     * @param[in]  scaled_initial_temperature Initial temperature value (scaled in 0.01 °C).
     * @return
     *      - ESP_OK on success.
     *      - ESP_FAIL on failure.
     */
    esp_err_t createTemperatureSensorEndpoint(esp_matter::node_t* matter_node,
                                              nullable<int16_t> scaled_min_temperature,
                                              nullable<int16_t> scaled_max_temperature,
                                              nullable<int16_t> scaled_initial_temperature);
    /**
     * @brief Create a Matter relative humidity sensor endpoint.
     *
     * This function sets up a new Matter relative humidity sensor endpoint with the specified
     * properties.
     *
     * @param[in]  matter_node               Pointer to the Node handle.
     * @param[in]  min_relative_humidity     Minimum allowable relative humidity (scaled in 0.01 %).
     * @param[in]  max_relative_humidity     Maximum allowable relative humidity (scaled in 0.01 %).
     * @param[in]  initial_relative_humidity Initial relative humidity value (scaled in 0.01 %).
     * @return
     *      - ESP_OK on success.
     *      - ESP_FAIL on failure.
     */
    esp_err_t createRelativeHumiditySensorEndpoint(esp_matter::node_t* matter_node,
                                                   nullable<uint16_t> min_relative_humidity,
                                                   nullable<uint16_t> max_relative_humidity,
                                                   nullable<uint16_t> initial_relative_humidity);
    /**
     * @brief Create a Matter pressure sensor endpoint.
     *
     * This function sets up a new Matter pressure sensor endpoint with the specified properties.
     *
     * @param[in]  matter_node      Pointer to the Node handle.
     * @param[in]  min_pressure     Minimum allowable pressure (scaled in 0.1 kPa).
     * @param[in]  max_pressure     Maximum allowable pressure (scaled in 0.1 kPa).
     * @param[in]  initial_pressure Initial pressure value (scaled in 0.1 kPa).
     * @return
     *      - ESP_OK on success.
     *      - ESP_FAIL on failure.
     */
    esp_err_t createPressureSensorEndpoint(esp_matter::node_t* matter_node,
                                           int16_t min_pressure,
                                           int16_t max_pressure,
                                           int16_t initial_pressure);
    uint16_t temperature_sensor_endpoint_id_ = 0;
    uint16_t pressure_sensor_endpoint_id_ = 0;
    std::optional<uint16_t> relative_humidity_sensor_endpoint_id_ = std::nullopt;
};

#endif // MATTER_MANAGER_H
