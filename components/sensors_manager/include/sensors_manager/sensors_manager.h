#ifndef SENSORS_MANAGER_H
#define SENSORS_MANAGER_H

#include <atomic>
#include <cstdint>
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

    // Probes the sensors; may be called again while isSensorsDetected() is false.
    esp_err_t init(int64_t read_interval_ms);
    // Reads every sensor; one that keeps failing is re-initialized (after an I2C bus reset).
    esp_err_t readValues(const SensorReadings& compensations);
    // Latest readings in °C (Temperature), Pa (Pressure), %RH (Humidity); nullopt if unavailable.
    const SensorReadings& getReadings() const {
        return readings_;
    }
    // Valid measurement ranges in the same units; nullopt if no sensor provides the type.
    SensorRanges getRanges() const;
    const char* getSensorModelNameForType(SensorType sensor_type) const;
    // Asks the type's owner to calibrate on its next read; callable from any task.
    void requestCalibration(SensorType sensor_type);
    bool isSensorsDetected() const;
    // Forwards pushed readings of types each sensor owns (re-attached after a re-init); call once,
    // after init().
    void setReadingListener(const Sensor::ReadingListener& listener);

  private:
    using SensorFactory = std::unique_ptr<Sensor> (*)();

    struct ActiveSensor {
        SensorFactory factory; // recreates the sensor for a re-init
        std::unique_ptr<Sensor> sensor;
        uint8_t failures{}; // consecutive failed reads
    };

    static constexpr int8_t no_owner = -1;

    void reinit(size_t index, bool& bus_reset_done);
    void attachListener(size_t index);

    // Only the read loop's task touches the sensor objects (a re-init replaces them); other tasks
    // use the fixed owner indices and the calibration flags.
    std::vector<ActiveSensor> active_sensors_;
    PerSensorType<int8_t> owner_by_type_ = [] {
        PerSensorType<int8_t> owners{};
        owners.fill(no_owner);
        return owners;
    }(); // index into active_sensors_; AIR_QUALITY is never owned
    std::vector<size_t> air_quality_sensors_; // AIR_QUALITY contributors (worst value wins)
    std::string air_quality_model_names_;
    PerSensorType<std::atomic<bool>> calibration_requested_{};
    Sensor::ReadingListener listener_;
    SensorReadings readings_{};
    int64_t read_interval_ms_{};
    bool initialized_ = false;
};

#endif // SENSORS_MANAGER_H
