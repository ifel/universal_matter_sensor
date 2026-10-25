#include "sensors_manager/sensors_manager.h"

#include <algorithm>
#include <array>

#include "esp_log.h"
#include "sensors/bmp280_bme280.h"

constexpr const char* tag = "   ***sensors_manager***   ";

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t SensorsManager::init() {
    if (initialized_) {
        ESP_LOGW(tag, "Already initialized");
        return ESP_OK;
    }

    // Add new sensor drivers here.
    std::array<std::unique_ptr<Sensor>, 1> candidates = {
        std::make_unique<Bmp280Bme280>(),
    };

    for (auto& sensor : candidates) {
        if (sensor->init() != ESP_OK) {
            ESP_LOGD(tag, "%s sensor not found.", sensor->getModelName());
            continue;
        }

        // First sensor providing a type owns it.
        bool used = false;
        for (const auto provided_type : sensor->getTypes()) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            auto& slot = sensors_by_type_[toIndex(provided_type)];
            if (slot == nullptr) {
                slot = sensor.get();
                used = true;
            }
        }
        if (used) { active_sensors_.push_back(std::move(sensor)); }

        if (std::ranges::all_of(sensors_by_type_,
                                [](const auto* owner) { return owner != nullptr; })) {
            break; // All types are covered
        }
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t SensorsManager::readValues() {
    if (!initialized_) { return ESP_ERR_INVALID_STATE; }

    auto ret = ESP_OK;
    for (const auto& sensor : active_sensors_) {
        SensorReadings sensor_readings{};
        const auto err = sensor->read(sensor_readings);
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
    }
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
    return ranges;
}

bool SensorsManager::isSensorsDetected() const {
    return !active_sensors_.empty();
}
