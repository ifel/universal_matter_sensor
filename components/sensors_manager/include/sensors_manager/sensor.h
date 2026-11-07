#ifndef SENSOR_H
#define SENSOR_H

#include <functional>
#include <optional>
#include <span>

#include "common_types/common_types.h"
#include "esp_err.h"

class Sensor {
  public:
    // Reading pushed between polls (e.g. presence changes); may be called from the sensor's task.
    using ReadingListener = std::function<void(SensorType, std::optional<float>)>;

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
    virtual esp_err_t init(int64_t read_interval_ms) = 0;
    // Fills readings (°C, Pa, %RH) for every type in getTypes(); nullopt on failure.
    virtual esp_err_t read(SensorReadings& readings, SensorReadings& compensations) = 0;
    // Event-driven sensors push changes through it. Default: polled only.
    // NOLINTNEXTLINE(performance-unnecessary-value-param): by value so overrides can move it.
    virtual void setReadingListener(ReadingListener /*listener*/) {}
};

#endif // SENSOR_H
