#ifndef SENSOR_H
#define SENSOR_H

#include <optional>
#include <span>

#include "common_types/common_types.h"
#include "esp_err.h"

class Sensor {
  public:
    Sensor() = default;
    virtual ~Sensor() = default;
    // Prevent copying
    Sensor(const Sensor&) = delete;
    Sensor& operator=(const Sensor&) = delete;

    // Prevent moving
    Sensor(Sensor&&) = delete;
    Sensor& operator=(Sensor&&) = delete;

    virtual std::span<const SensorType> getTypes() const = 0;
    virtual const char* getModelName() const = 0;
    // Returns valid measurement range in °C (Temperature), Pa (Pressure), or %RH (Humidity).
    virtual std::optional<SensorValueRange> getValueRangeForType(SensorType type) const = 0;
    virtual esp_err_t init() = 0;
    // Fills readings (°C, Pa, %RH) for every type in getTypes(); nullopt on failure.
    virtual esp_err_t read(SensorReadings& readings) = 0;
};

#endif // SENSOR_H
