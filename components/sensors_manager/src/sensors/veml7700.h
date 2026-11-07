#ifndef SENSORS_VEML7700_H
#define SENSORS_VEML7700_H

#include <array>

#include <veml7700.h>

#include "sensors_manager/sensor.h"

class Veml7700 : public Sensor {
  public:
    Veml7700() = default;
    ~Veml7700() override;
    // Prevent copying
    Veml7700(const Veml7700&) = delete;
    Veml7700& operator=(const Veml7700&) = delete;

    // Prevent moving
    Veml7700(Veml7700&&) = delete;
    Veml7700& operator=(Veml7700&&) = delete;

    const char* getModelName() const override {
        return "VEML7700";
    }
    std::span<const SensorTypeRange> getTypeRanges() const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, const SensorReadings& compensations) override;

  private:
    esp_err_t applyConfigRange();

    i2c_dev_t sensor_{};
    veml7700_config_t config_{};
    uint8_t ladder_idx_ = 2;
    bool initialized_ = false; // true only after a successful init()
};

#endif // SENSORS_VEML7700_H
