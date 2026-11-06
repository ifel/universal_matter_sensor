#ifndef SENSORS_TMP117_H
#define SENSORS_TMP117_H

#include <array>

#include <tmp117.h>

#include "sensors_manager/sensor.h"

// TI TMP119 (or register-compatible TMP117) high-accuracy temperature sensor.
class Tmp117 : public Sensor {
  public:
    Tmp117() = default;
    ~Tmp117() override;
    // Prevent copying
    Tmp117(const Tmp117&) = delete;
    Tmp117& operator=(const Tmp117&) = delete;

    // Prevent moving
    Tmp117(Tmp117&&) = delete;
    Tmp117& operator=(Tmp117&&) = delete;

    std::span<const SensorType> getTypes() const override {
        static constexpr std::array k_types = {
            SensorType::TEMPERATURE,
        };
        return std::span<const SensorType>(k_types);
    }
    const char* getModelName() const override {
        return sensor_.device_id == TMP119_DEVICE_ID_REV || !initialized_ ? "TMP119" : "TMP117";
    }
    std::optional<SensorValueRange> getValueRangeForType(SensorType type) const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, SensorReadings& compensations) override;

  private:
    esp_err_t initAt(uint8_t address);

    tmp117_t sensor_{};
    bool initialized_ = false; // true only after a successful init()
};

#endif // SENSORS_TMP117_H
