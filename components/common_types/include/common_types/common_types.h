#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

// Physical measurement range and reporting thresholds
// (Temperature: °C, Pressure: Pa, Relative Humidity: %RH).
struct SensorValueRange {
    float min{};
    float max{};
    float abs_threshold{
        0.0F}; // Minimum absolute change to report immediately (same unit as min/max)
    float rel_threshold{0.0F}; // Minimum relative change as a fraction of |last_reported_value|
};

// Sensor measurement types (Temperature: °C, Pressure: Pa, Relative Humidity: %RH, Carbon Dioxide
// Concentration, ppm, Illuminance, Lux). Keep ILLUMINANCE last, or update sensor_type_count.
enum class SensorType : uint8_t {
    TEMPERATURE,
    PRESSURE,
    RELATIVE_HUMIDITY,
    CARBON_DIOXIDE_CONCENTRATION,
    ILLUMINANCE
};

inline constexpr size_t sensor_type_count = static_cast<size_t>(SensorType::ILLUMINANCE) + 1;

// Per-type containers indexed by SensorType.
template <typename T> using PerSensorType = std::array<T, sensor_type_count>;
using SensorRanges = PerSensorType<std::optional<SensorValueRange>>;
using SensorReadings = PerSensorType<std::optional<float>>;

constexpr size_t toIndex(SensorType type) {
    return static_cast<size_t>(type);
}

#endif // COMMON_TYPES_H
