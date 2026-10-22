#ifndef MATTER_MANAGER_H
#define MATTER_MANAGER_H

#include <esp_err.h>
#include <esp_matter.h>

/**
 * @brief Handle Matter device events.
 *
 * This callback is invoked to process various events related to the Matter stack.
 *
 * @param[in] event The Matter device event structure.
 * @param[in] arg   Additional argument provided by the caller.
 */
void matter_event_callback(const ChipDeviceEvent *event, intptr_t arg);

/**
 * @brief Handle identification cluster commands for a Matter endpoint.
 *
 * This callback is used to execute identification effects, such as blinking lights or beeping,
 * based on the effect ID and variant provided by the cluster command.
 *
 * @param[in] type           The type of identification callback (e.g., start or stop).
 * @param[in] endpoint_id    The endpoint ID where the command is received.
 * @param[in] effect_id      The effect ID specifying the type of identification effect.
 * @param[in] effect_variant The effect variant for fine-tuned behavior.
 * @param[in] priv_data      User-defined private data.
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_ARG if arguments are invalid.
 *      - Other error codes on failure.
 */
esp_err_t identification_callback(esp_matter::identification::callback_type_t type, uint16_t endpoint_id,
                                  uint8_t effect_id, uint8_t effect_variant, void *priv_data);

/**
 * @brief Handle attribute updates for Matter clusters.
 *
 * This callback is invoked when an attribute value is updated by the Matter framework.
 *
 * @param[in] type        The type of attribute update (e.g., write or read).
 * @param[in] endpoint_id The endpoint ID where the update occurred.
 * @param[in] cluster_id  The cluster ID associated with the attribute.
 * @param[in] attribute_id The attribute ID being updated.
 * @param[in] val          Pointer to the updated attribute value.
 * @param[in] priv_data    User-defined private data.
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_ARG if arguments are invalid.
 *      - Other error codes on failure.
 */
esp_err_t attribute_update_callback(esp_matter::attribute::callback_type_t type, uint16_t endpoint_id,
                                    uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val,
                                    void *priv_data);

esp_err_t matter_init(uint16_t *temperature_sensor_endpoint_id, uint16_t *pressure_sensor_endpoint_id,
                      uint16_t *relative_humidity_sensor_endpoint_id, float min_temperature, float max_temperature,
                      float initial_temperature, float min_pressure, float max_pressure, float initial_pressure,
                      float min_relative_humidity, float max_relative_humidity, float initial_relative_humidity);

esp_err_t matter_update_values(uint16_t temperature_endpoint_id, uint16_t pressure_sensor_endpoint_id,
                               const uint16_t *relative_humidity_endpoint_id, float new_temperature, float new_pressure,
                               float new_relative_humidity);

#endif // MATTER_MANAGER_H
