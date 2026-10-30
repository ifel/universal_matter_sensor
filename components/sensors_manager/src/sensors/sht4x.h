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

    std::span<const SensorType> getTypes() const override {
        static constexpr std::array k_types = {
            SensorType::TEMPERATURE,
            SensorType::RELATIVE_HUMIDITY,
        };
        return std::span<const SensorType>(k_types);
    }
    const char* getModelName() const override {
        return "SHT4X";
    }
    std::optional<SensorValueRange> getValueRangeForType(SensorType type) const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, SensorReadings& compensations) override;

  private:
    sht4x_t sensor_{};
    bool initialized_ = false; // true only after a successful init()
};

#endif // SENSORS_SHT4X_H
