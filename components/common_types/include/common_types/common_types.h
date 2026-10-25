#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

// Physical measurement range (Temperature: °C, Pressure: Pa, Relative Humidity: %RH).
struct SensorValueRange {
    float min{};
    float max{};
};

// Sensor measurement types (Temperature: °C, Pressure: Pa, Relative Humidity: %RH).
// Keep RELATIVE_HUMIDITY last, or update sensor_type_count.
enum class SensorType : uint8_t { TEMPERATURE, PRESSURE, RELATIVE_HUMIDITY };

inline constexpr size_t sensor_type_count = static_cast<size_t>(SensorType::RELATIVE_HUMIDITY) + 1;

// Per-type containers indexed by SensorType.
template <typename T> using PerSensorType = std::array<T, sensor_type_count>;
using SensorRanges = PerSensorType<std::optional<SensorValueRange>>;
using SensorReadings = PerSensorType<std::optional<float>>;

constexpr size_t toIndex(SensorType type) {
    return static_cast<size_t>(type);
}

#endif // COMMON_TYPES_H
