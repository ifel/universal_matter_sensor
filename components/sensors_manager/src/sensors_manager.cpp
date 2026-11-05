#include "sensors_manager/sensors_manager.h"

#include <algorithm>
#include <array>

#include "esp_log.h"
#include "sensors/bmp280_bme280.h"
#include "sensors/bmp581.h"
#include "sensors/ens160.h"
#include "sensors/scd30.h"
#include "sensors/sht4x.h"
#include "sensors/tsl2591.h"
#include "sensors/veml7700.h"

constexpr const char* tag = "   ***sensors_manager***   ";

namespace {
bool providesType(const Sensor& sensor, SensorType type) {
    const auto types = sensor.getTypes();
    return std::ranges::find(types, type) != types.end();
}

// True when every type except the aggregated AIR_QUALITY has an owner.
bool allOwnedTypesCovered(const PerSensorType<Sensor*>& owners) {
    for (size_t i = 0; i < sensor_type_count; ++i) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        if (static_cast<SensorType>(i) != SensorType::AIR_QUALITY && owners[i] == nullptr) {
            return false;
        }
    }
    return true;
}
} // namespace

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t SensorsManager::init(int64_t read_interval_ms) {
    if (initialized_) {
        ESP_LOGW(tag, "Already initialized");
        return ESP_OK;
    }

    // Add new sensor drivers here.
    std::array<std::unique_ptr<Sensor>, 7> candidates = {std::make_unique<Sht4x>(),
                                                         std::make_unique<Bmp581>(),
                                                         std::make_unique<Bmp280Bme280>(),
                                                         std::make_unique<Scd30>(),
                                                         std::make_unique<Ens160>(),
                                                         std::make_unique<Veml7700>(),
                                                         std::make_unique<Tsl2591>()};

    for (auto& sensor : candidates) {
        if (sensor->init(read_interval_ms) != ESP_OK) {
            ESP_LOGD(tag, "%s sensor not found.", sensor->getModelName());
            continue;
        }

        // First sensor providing a type owns it.
        bool used = false;
        for (const auto provided_type : sensor->getTypes()) {
            if (provided_type == SensorType::AIR_QUALITY) { continue; } // aggregated, not owned
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            auto& slot = sensors_by_type_[toIndex(provided_type)];
            if (slot == nullptr) {
                slot = sensor.get();
                used = true;
            }
        }
        if (used) { active_sensors_.push_back(std::move(sensor)); }

        if (allOwnedTypesCovered(sensors_by_type_)) {
            break; // All types are covered
        }
    }

    // Air quality comes from every sensor that reports it and owns an Air Quality endpoint metric.
    for (const auto& sensor : active_sensors_) {
        const bool owns_endpoint_type =
            std::ranges::any_of(sensor->getTypes(), [&](const SensorType type) {
                return isAirQualityEndpointType(type) &&
                       // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
                       sensors_by_type_[toIndex(type)] == sensor.get();
            });
        if (owns_endpoint_type && providesType(*sensor, SensorType::AIR_QUALITY)) {
            air_quality_sensors_.push_back(sensor.get());
            if (!air_quality_model_names_.empty()) { air_quality_model_names_ += "+"; }
            air_quality_model_names_ += sensor->getModelName();
        }
    }
    if (!air_quality_sensors_.empty()) {
        ESP_LOGI(tag, "Air quality: worst of %s", air_quality_model_names_.c_str());
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t SensorsManager::readValues(SensorReadings& compensations) {
    if (!initialized_) { return ESP_ERR_INVALID_STATE; }

    auto ret = ESP_OK;
    std::optional<float> worst_air_quality;
    for (const auto& sensor : active_sensors_) {
        SensorReadings sensor_readings{};
        const auto err = sensor->read(sensor_readings, compensations);
        if (err != ESP_OK) { ret = err; }
        // Take only the types this sensor owns; failed reads clear them.
        for (const auto type : sensor->getTypes()) {
            const auto index = toIndex(type);
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            if (sensors_by_type_[index] == sensor.get()) {
                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
                readings_[index] = sensor_readings[index];
            }
        }
        // Higher AirQualityLevel is worse; contributors without a value are skipped.
        if (const auto& air_quality = at<SensorType::AIR_QUALITY>(sensor_readings);
            air_quality.has_value() &&
            std::ranges::find(air_quality_sensors_, sensor.get()) != air_quality_sensors_.end()) {
            worst_air_quality = std::max(worst_air_quality.value_or(*air_quality), *air_quality);
        }
    }
    at<SensorType::AIR_QUALITY>(readings_) = worst_air_quality;
    return ret;
}

SensorRanges SensorsManager::getRanges() const {
    SensorRanges ranges{};
    for (size_t i = 0; i < sensor_type_count; ++i) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        if (const auto* sensor = sensors_by_type_[i]; sensor != nullptr) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            ranges[i] = sensor->getValueRangeForType(static_cast<SensorType>(i));
        }
    }

    // Aggregated air quality covers the union of the contributors' ranges.
    auto& air_quality_range = at<SensorType::AIR_QUALITY>(ranges);
    for (const auto* sensor : air_quality_sensors_) {
        const auto range = sensor->getValueRangeForType(SensorType::AIR_QUALITY);
        if (!range.has_value()) { continue; }
        if (!air_quality_range.has_value()) {
            air_quality_range = range;
            continue;
        }
        air_quality_range->min = std::min(air_quality_range->min, range->min);
        air_quality_range->max = std::max(air_quality_range->max, range->max);
        air_quality_range->abs_threshold =
            std::min(air_quality_range->abs_threshold, range->abs_threshold);
        air_quality_range->rel_threshold =
            std::min(air_quality_range->rel_threshold, range->rel_threshold);
    }
    return ranges;
}

const char* SensorsManager::getSensorModelNameForType(const SensorType sensor_type) const {
    if (sensor_type == SensorType::AIR_QUALITY) {
        return air_quality_model_names_.empty() ? "Unknown" : air_quality_model_names_.c_str();
    }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    if (const auto* sensor = sensors_by_type_[toIndex(sensor_type)]; sensor != nullptr) {
        return sensor->getModelName();
    }
    return "Unknown";
}

bool SensorsManager::isSensorsDetected() const {
    return !active_sensors_.empty();
}
