#ifndef SENSORS_TSL2591_H
#define SENSORS_TSL2591_H

#include <array>
#include <cstdint>

#include <tsl2591.h>

#include "sensors_manager/sensor.h"

class Tsl2591 : public Sensor {
  public:
    Tsl2591() = default;
    ~Tsl2591() override;
    // Prevent copying
    Tsl2591(const Tsl2591&) = delete;
    Tsl2591& operator=(const Tsl2591&) = delete;

    // Prevent moving
    Tsl2591(Tsl2591&&) = delete;
    Tsl2591& operator=(Tsl2591&&) = delete;

    const char* getModelName() const override {
        return "TSL2591";
    }
    std::span<const SensorTypeRange> getTypeRanges() const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, const SensorReadings& compensations) override;

  private:
    esp_err_t applyLadderStep();

    tsl2591_t sensor_{};
    uint8_t ladder_idx_ = 0;   // start least sensitive: no saturation in daylight
    bool initialized_ = false; // true only after a successful init()
};

#endif // SENSORS_TSL2591_H
