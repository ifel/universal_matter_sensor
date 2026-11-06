#ifndef SENSORS_VL53L1X_H
#define SENSORS_VL53L1X_H

#include <array>
#include <atomic>

// ST's ULP header has no extern "C" guards and the wrapper includes it outside its own; include it
// first with C linkage so its include guard keeps the wrapper from redeclaring it as C++.
extern "C" {
#include <VL53L1X_ULP_api.h>
}
#include <vl53l1x.h>

#include "sensors_manager/sensor.h"

// Time-of-flight distance sensor reporting how much of a consumable (e.g. a salt block) is left:
// 100% at the "full" distance (Kconfig default, or taught via requestCalibration and kept in NVS),
// 0% a configured height further away.
class Vl53l1x : public Sensor {
  public:
    Vl53l1x() = default;
    ~Vl53l1x() override;
    // Prevent copying
    Vl53l1x(const Vl53l1x&) = delete;
    Vl53l1x& operator=(const Vl53l1x&) = delete;

    // Prevent moving
    Vl53l1x(Vl53l1x&&) = delete;
    Vl53l1x& operator=(Vl53l1x&&) = delete;

    const char* getModelName() const override {
        return "VL53L1X";
    }
    std::span<const SensorTypeRange> getTypeRanges() const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, const SensorReadings& compensations) override;
    // Next valid distance becomes the "full" (100%) point, saved to NVS.
    void requestCalibration(SensorType type) override;

  private:
    esp_err_t configure();
    void loadFullDistance();
    void saveFullDistance() const;

    vl53l1x_t sensor_{};
    uint16_t full_distance_mm_ = CONFIG_VL53L1X_FULL_DISTANCE_MM; // Kconfig default until taught
    std::atomic<bool> calibration_requested_{false};
    bool initialized_ = false; // true only after a successful init()
};

#endif // SENSORS_VL53L1X_H
