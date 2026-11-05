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
// Concentration, ppm, Illuminance, Lux, TVOC Concentration, ppb, Air Quality, AirQualityLevel).
// Keep AIR_QUALITY last, or update sensor_type_count.
enum class SensorType : uint8_t {
    TEMPERATURE,
    PRESSURE,
    RELATIVE_HUMIDITY,
    CARBON_DIOXIDE_CONCENTRATION,
    ILLUMINANCE,
    TVOC_CONCENTRATION,
    AIR_QUALITY
};

inline constexpr size_t sensor_type_count = static_cast<size_t>(SensorType::AIR_QUALITY) + 1;

// AIR_QUALITY reading values; numerically equal to Matter's AirQualityEnum.
enum class AirQualityLevel : uint8_t {
    UNKNOWN,
    GOOD,
    FAIR,
    MODERATE,
    POOR,
    VERY_POOR,
    EXTREMELY_POOR
};

// Per-type containers indexed by SensorType.
template <typename T> using PerSensorType = std::array<T, sensor_type_count>;
using SensorRanges = PerSensorType<std::optional<SensorValueRange>>;
using SensorReadings = PerSensorType<std::optional<float>>;

constexpr size_t toIndex(SensorType type) {
    return static_cast<size_t>(type);
}

// Compile-time bounds-checked access to a PerSensorType container.
template <SensorType Type, typename Container> constexpr auto& at(Container& container) {
    return std::get<toIndex(Type)>(container);
}

#endif // COMMON_TYPES_H
