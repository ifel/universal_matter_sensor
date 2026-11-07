#ifndef SENSORS_MAX17048_H
#define SENSORS_MAX17048_H

#include <array>

#include <max1704x.h>

#include "sensors_manager/sensor.h"

// Analog Devices MAX17048 / MAX17049 single-cell Li-ion fuel gauge (the device's own battery).
class Max17048 : public Sensor {
  public:
    Max17048() = default;
    ~Max17048() override;
    // Prevent copying
    Max17048(const Max17048&) = delete;
    Max17048& operator=(const Max17048&) = delete;

    // Prevent moving
    Max17048(Max17048&&) = delete;
    Max17048& operator=(Max17048&&) = delete;

    const char* getModelName() const override {
        return "MAX17048";
    }
    std::span<const SensorTypeRange> getTypeRanges() const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, const SensorReadings& compensations) override;

  private:
    esp_err_t updateRcomp(const SensorReadings& compensations);

    max1704x_t sensor_{};
    uint8_t rcomp_ = 0;        // last written; 0 = none yet (the gauge keeps it across ESP reboots)
    bool initialized_ = false; // true only after a successful init()
};

#endif // SENSORS_MAX17048_H
