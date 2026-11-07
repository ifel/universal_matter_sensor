#include "sensors_manager/sensors_manager.h"

#include <algorithm>
#include <array>

#include <driver/i2c_master.h>
#include <i2cdev.h>

#include "esp_log.h"
#include "sdkconfig.h"
#include "sensors/bmp280_bme280.h"
#include "sensors/bmp581.h"
#include "sensors/ens160.h"
#include "sensors/i2c_bus.h"
#if CONFIG_LD2410S_ENABLED
#    include "sensors/ld2410s.h"
#endif
#include "sensors/max17048.h"
#include "sensors/scd30.h"
#include "sensors/sht4x.h"
#include "sensors/tmp117.h"
#include "sensors/tsl2591.h"
#include "sensors/veml7700.h"
#include "sensors/vl53l1x.h"

constexpr const char* tag = "sensors_manager";

namespace {
// Consecutive failed reads before a sensor is re-initialized. ESP_ERR_NOT_FINISHED (no result
// yet: warm-up, data not ready, no target) is not a failure.
constexpr uint8_t failures_before_reinit = 3;

template <typename T> std::unique_ptr<Sensor> makeSensor() {
    return std::make_unique<T>();
}

// Add new sensor drivers here, in priority order (first detected owner of a type wins).
constexpr auto sensor_factories = std::to_array<std::unique_ptr<Sensor> (*)()>({
    &makeSensor<Tmp117>,
    &makeSensor<Sht4x>,
    &makeSensor<Bmp581>,
    &makeSensor<Bmp280Bme280>,
    &makeSensor<Scd30>,
    &makeSensor<Ens160>,
    &makeSensor<Veml7700>,
    // Before Tsl2591: both use 0x29, and the VL53L1X ID check writes to the bus.
    &makeSensor<Vl53l1x>,
    &makeSensor<Tsl2591>,
    &makeSensor<Max17048>,
#if CONFIG_LD2410S_ENABLED
    &makeSensor<Ld2410s>,
#endif
});

// True when every type except the aggregated AIR_QUALITY has an owner.
bool allOwnedTypesCovered(const PerSensorType<int8_t>& owners) {
    for (size_t i = 0; i < sensor_type_count; ++i) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        if (static_cast<SensorType>(i) != SensorType::AIR_QUALITY && owners[i] < 0) {
            return false;
        }
    }
    return true;
}

bool isFailure(esp_err_t err) {
    return err != ESP_OK && err != ESP_ERR_NOT_FINISHED;
}

// Clocks out a device holding SDA low, so the re-init that follows can reach the bus.
void resetI2cBus() {
    void* bus = nullptr;
    esp_err_t err = i2cdev_get_shared_handle(i2c_port, &bus);
    if (err == ESP_OK) { err = i2c_master_bus_reset(static_cast<i2c_master_bus_handle_t>(bus)); }
    if (err != ESP_OK) { ESP_LOGW(tag, "I2C bus reset failed: %s", esp_err_to_name(err)); }
}
} // namespace

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t SensorsManager::init(int64_t read_interval_ms) {
    if (initialized_) {
        ESP_LOGW(tag, "Already initialized");
        return ESP_OK;
    }
    read_interval_ms_ = read_interval_ms;

    for (const auto factory : sensor_factories) {
        auto sensor = factory();
        if (sensor->init(read_interval_ms) != ESP_OK) {
            ESP_LOGD(tag, "%s sensor not found.", sensor->getModelName());
            continue;
        }

        // First sensor providing a type owns it.
        const auto index = static_cast<int8_t>(active_sensors_.size());
        bool used = false;
        for (const auto& [provided_type, range] : sensor->getTypeRanges()) {
            if (provided_type == SensorType::AIR_QUALITY) { continue; } // aggregated, not owned
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            auto& owner = owner_by_type_[toIndex(provided_type)];
            if (owner == no_owner) {
                owner = index;
                used = true;
            }
        }
        if (used) {
            active_sensors_.push_back(
                ActiveSensor{.factory = factory, .sensor = std::move(sensor)});
        }

        if (allOwnedTypesCovered(owner_by_type_)) { break; }
    }

    // Air quality comes from every sensor that reports it and owns an Air Quality endpoint metric.
    for (size_t i = 0; i < active_sensors_.size(); ++i) {
        const auto& sensor = *active_sensors_[i].sensor;
        const bool owns_endpoint_type =
            std::ranges::any_of(sensor.getTypeRanges(), [&](const SensorTypeRange& type_range) {
                return isAirQualityEndpointType(type_range.type) &&
                       // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
                       owner_by_type_[toIndex(type_range.type)] == static_cast<int8_t>(i);
            });
        if (owns_endpoint_type && sensor.providesType(SensorType::AIR_QUALITY)) {
            air_quality_sensors_.push_back(i);
            if (!air_quality_model_names_.empty()) { air_quality_model_names_ += "+"; }
            air_quality_model_names_ += sensor.getModelName();
        }
    }
    if (!air_quality_sensors_.empty()) {
        ESP_LOGI(tag, "Air quality: worst of %s", air_quality_model_names_.c_str());
    }

    // With nothing found, init() can be called again to re-probe.
    initialized_ = !active_sensors_.empty();
    return ESP_OK;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t SensorsManager::readValues(const SensorReadings& compensations) {
    if (!initialized_) { return ESP_ERR_INVALID_STATE; }

    auto ret = ESP_OK;
    bool bus_reset_done = false;
    std::optional<float> worst_air_quality;
    for (size_t i = 0; i < active_sensors_.size(); ++i) {
        auto& active = active_sensors_[i];
        const auto owner = static_cast<int8_t>(i);

        for (size_t type = 0; type < sensor_type_count; ++type) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            if (owner_by_type_[type] == owner && calibration_requested_[type].exchange(false)) {
                active.sensor->requestCalibration(static_cast<SensorType>(type));
            }
        }

        SensorReadings sensor_readings{};
        const auto err = active.sensor->read(sensor_readings, compensations);
        if (err != ESP_OK) { ret = err; }
        active.failures = isFailure(err) ? static_cast<uint8_t>(active.failures + 1) : 0;
        if (active.failures >= failures_before_reinit) { reinit(i, bus_reset_done); }

        // Take only the types this sensor owns; failed reads clear them.
        for (size_t type = 0; type < sensor_type_count; ++type) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            if (owner_by_type_[type] == owner) { readings_[type] = sensor_readings[type]; }
        }
        // Higher AirQualityLevel is worse; contributors without a value are skipped.
        if (const auto& air_quality = at<SensorType::AIR_QUALITY>(sensor_readings);
            air_quality.has_value() &&
            std::ranges::find(air_quality_sensors_, i) != air_quality_sensors_.end()) {
            worst_air_quality = std::max(worst_air_quality.value_or(*air_quality), *air_quality);
        }
    }
    at<SensorType::AIR_QUALITY>(readings_) = worst_air_quality;
    return ret;
}

// Replaces a failing sensor with a fresh instance. The old one goes first, so its bus device or
// UART is released; a failed init leaves an uninitialized one, retried after more failures.
void SensorsManager::reinit(size_t index, bool& bus_reset_done) {
    auto& active = active_sensors_[index];
    ESP_LOGW(tag,
             "%s failed %u reads in a row, re-initializing",
             active.sensor->getModelName(),
             static_cast<unsigned int>(active.failures));
    active.failures = 0;
    active.sensor.reset();
    if (!bus_reset_done) {
        resetI2cBus();
        bus_reset_done = true;
    }

    active.sensor = active.factory();
    if (active.sensor->init(read_interval_ms_) != ESP_OK) {
        ESP_LOGW(tag, "%s re-init failed, retrying later", active.sensor->getModelName());
    }
    attachListener(index);
}

SensorRanges SensorsManager::getRanges() const {
    SensorRanges ranges{};
    for (size_t i = 0; i < sensor_type_count; ++i) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        if (const auto owner = owner_by_type_[i]; owner != no_owner) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            ranges[i] = active_sensors_[static_cast<size_t>(owner)].sensor->getValueRangeForType(
                static_cast<SensorType>(i));
        }
    }

    // Aggregated air quality covers the union of the contributors' ranges.
    auto& air_quality_range = at<SensorType::AIR_QUALITY>(ranges);
    for (const auto index : air_quality_sensors_) {
        const auto range =
            active_sensors_[index].sensor->getValueRangeForType(SensorType::AIR_QUALITY);
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
    if (const auto owner = owner_by_type_[toIndex(sensor_type)]; owner != no_owner) {
        return active_sensors_[static_cast<size_t>(owner)].sensor->getModelName();
    }
    return "Unknown";
}

void SensorsManager::setReadingListener(const Sensor::ReadingListener& listener) {
    listener_ = listener;
    for (size_t i = 0; i < active_sensors_.size(); ++i) {
        attachListener(i);
    }
}

// Forwards only the types this sensor owns.
void SensorsManager::attachListener(size_t index) {
    if (!listener_) { return; }
    active_sensors_[index].sensor->setReadingListener(
        [this, owner = static_cast<int8_t>(index), listener = listener_](
            SensorType type, std::optional<float> value) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            if (owner_by_type_[toIndex(type)] == owner) { listener(type, value); }
        });
}

void SensorsManager::requestCalibration(const SensorType sensor_type) {
    // Applied by readValues(): only the read loop's task touches the sensor objects.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    calibration_requested_[toIndex(sensor_type)] = true;
}

bool SensorsManager::isSensorsDetected() const {
    return !active_sensors_.empty();
}
