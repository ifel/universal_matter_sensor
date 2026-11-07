#ifndef SENSOR_H
#define SENSOR_H

#include <algorithm>
#include <functional>
#include <optional>
#include <span>

#include "common_types/common_types.h"
#include "esp_err.h"

// One type a sensor reports and its range (in the type's unit, see SensorValueRange).
struct SensorTypeRange {
    SensorType type;
    SensorValueRange range;
};

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

    // Types this sensor reports, with their ranges (may depend on the chip found by init()).
    virtual std::span<const SensorTypeRange> getTypeRanges() const = 0;
    virtual const char* getModelName() const = 0;
    virtual esp_err_t init(int64_t read_interval_ms) = 0;
    // Fills readings for every type in getTypeRanges(); nullopt on failure. compensations holds
    // the previous cycle's readings.
    virtual esp_err_t read(SensorReadings& readings, const SensorReadings& compensations) = 0;
    // Event-driven sensors push changes through it. Default: polled only.
    // NOLINTNEXTLINE(performance-unnecessary-value-param): by value so overrides can move it.
    virtual void setReadingListener(ReadingListener /*listener*/) {}

    bool providesType(SensorType type) const {
        return getValueRangeForType(type).has_value();
    }

    std::optional<SensorValueRange> getValueRangeForType(SensorType type) const {
        const auto ranges = getTypeRanges();
        const auto found = std::ranges::find(ranges, type, &SensorTypeRange::type);
        return found == ranges.end() ? std::nullopt : std::optional(found->range);
    }
};

#endif // SENSOR_H
