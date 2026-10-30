#ifndef SENSORS_SCD30_H
#define SENSORS_SCD30_H

#include <array>

#include <scd30.h>

#include "sensors_manager/sensor.h"

class Scd30 : public Sensor {
  public:
    Scd30() = default;
    ~Scd30() override;
    // Prevent copying
    Scd30(const Scd30&) = delete;
    Scd30& operator=(const Scd30&) = delete;

    // Prevent moving
    Scd30(Scd30&&) = delete;
    Scd30& operator=(Scd30&&) = delete;

    std::span<const SensorType> getTypes() const override {
        static constexpr std::array k_types = {
            SensorType::TEMPERATURE,
            SensorType::RELATIVE_HUMIDITY,
            SensorType::CARBON_DIOXIDE_CONCENTRATION,
        };
        return std::span<const SensorType>{k_types};
    }
    const char* getModelName() const override {
        return "SCD30";
    }
    std::optional<SensorValueRange> getValueRangeForType(SensorType type) const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, SensorReadings& compensations) override;

  private:
    i2c_dev_t sensor_{};
    uint16_t pressure_comp_{}; // mBar
    bool initialized_ = false; // true only after a successful init()
};

#endif // SENSORS_SCD30_H
