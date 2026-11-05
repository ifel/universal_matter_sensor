#ifndef SENSORS_ENS160_H
#define SENSORS_ENS160_H

#include <array>

#include <ens160.h>

#include "sensors_manager/sensor.h"

class Ens160 : public Sensor {
  public:
    Ens160() = default;
    ~Ens160() override;
    // Prevent copying
    Ens160(const Ens160&) = delete;
    Ens160& operator=(const Ens160&) = delete;

    // Prevent moving
    Ens160(Ens160&&) = delete;
    Ens160& operator=(Ens160&&) = delete;

    std::span<const SensorType> getTypes() const override {
        static constexpr std::array k_types = {
            SensorType::CARBON_DIOXIDE_CONCENTRATION,
            SensorType::TVOC_CONCENTRATION,
            SensorType::AIR_QUALITY,
        };
        return std::span<const SensorType>(k_types);
    }
    const char* getModelName() const override {
        return "ENS160";
    }
    std::optional<SensorValueRange> getValueRangeForType(SensorType type) const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, SensorReadings& compensations) override;

  private:
    esp_err_t updateCompensation(const SensorReadings& compensations);

    ens160_t sensor_{};
    std::optional<float> temperature_comp_; // last written compensation, °C
    std::optional<float> humidity_comp_;    // last written compensation, %RH
    bool initialized_ = false;              // true only after a successful init()
};

#endif // SENSORS_ENS160_H
