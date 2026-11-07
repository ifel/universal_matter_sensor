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

    const char* getModelName() const override {
        return "SCD30";
    }
    std::span<const SensorTypeRange> getTypeRanges() const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, const SensorReadings& compensations) override;

  private:
    struct Measurement {
        float co2{};
        float temperature{};
        float relative_humidity{};
        int64_t time_us{}; // esp_timer time of the read
    };

    void updatePressureCompensation(const SensorReadings& compensations);

    i2c_dev_t sensor_{};
    uint16_t pressure_comp_{}; // mBar, 0 = none
    uint16_t interval_s_{};    // measurement interval
    std::optional<Measurement> last_measurement_;
    bool initialized_ = false; // true only after a successful init()
};

#endif // SENSORS_SCD30_H
