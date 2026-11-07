#ifndef SENSORS_SHT4X_H
#define SENSORS_SHT4X_H

#include <array>

#include <sht4x.h>

#include "sensors_manager/sensor.h"

class Sht4x : public Sensor {
  public:
    Sht4x() = default;
    ~Sht4x() override;
    // Prevent copying
    Sht4x(const Sht4x&) = delete;
    Sht4x& operator=(const Sht4x&) = delete;

    // Prevent moving
    Sht4x(Sht4x&&) = delete;
    Sht4x& operator=(Sht4x&&) = delete;

    const char* getModelName() const override {
        return "SHT4X";
    }
    std::span<const SensorTypeRange> getTypeRanges() const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, const SensorReadings& compensations) override;

  private:
    sht4x_t sensor_{};
    bool initialized_ = false; // true only after a successful init()
};

#endif // SENSORS_SHT4X_H
