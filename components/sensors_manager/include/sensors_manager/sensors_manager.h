#ifndef SENSORS_MANAGER_H
#define SENSORS_MANAGER_H

#include <memory>
#include <string>
#include <vector>

#include "common_types/common_types.h"
#include "esp_err.h"
#include "sensors_manager/sensor.h"

class SensorsManager {
  public:
    SensorsManager() = default;
    ~SensorsManager() = default;
    // Prevent copying
    SensorsManager(const SensorsManager&) = delete;
    SensorsManager& operator=(const SensorsManager&) = delete;

    // Prevent moving
    SensorsManager(SensorsManager&&) = delete;
    SensorsManager& operator=(SensorsManager&&) = delete;

    esp_err_t init(int64_t read_interval_ms);
    esp_err_t readValues(SensorReadings& compensations);
    // Latest readings in °C (Temperature), Pa (Pressure), %RH (Humidity); nullopt if unavailable.
    const SensorReadings& getReadings() const {
        return readings_;
    }
    // Valid measurement ranges in the same units; nullopt if no sensor provides the type.
    SensorRanges getRanges() const;
    const char* getSensorModelNameForType(SensorType sensor_type) const;
    bool isSensorsDetected() const;

  private:
    std::vector<std::unique_ptr<Sensor>> active_sensors_;
    PerSensorType<Sensor*> sensors_by_type_{}; // owner per type; AIR_QUALITY is never owned
    std::vector<Sensor*> air_quality_sensors_; // AIR_QUALITY contributors (worst value wins)
    std::string air_quality_model_names_;
    SensorReadings readings_{};
    bool initialized_ = false;
};

#endif // SENSORS_MANAGER_H
