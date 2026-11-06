#ifndef SENSORS_BMP581_H
#define SENSORS_BMP581_H

#include <array>

#include <bmp5_i2c.h>

#include "sensors_manager/sensor.h"

class Bmp581 : public Sensor {
  public:
    Bmp581() = default;
    ~Bmp581() override;
    // Prevent copying
    Bmp581(const Bmp581&) = delete;
    Bmp581& operator=(const Bmp581&) = delete;

    // Prevent moving
    Bmp581(Bmp581&&) = delete;
    Bmp581& operator=(Bmp581&&) = delete;

    std::span<const SensorType> getTypes() const override {
        static constexpr std::array k_types = {
            SensorType::TEMPERATURE,
            SensorType::PRESSURE,
        };
        return std::span<const SensorType>{k_types};
    }
    const char* getModelName() const override {
        return sensor_.dev.chip_id == BMP5_CHIP_ID_SEC ? "BMP585" : "BMP581";
    }
    std::optional<SensorValueRange> getValueRangeForType(SensorType type) const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, SensorReadings& compensations) override;

  private:
    esp_err_t initAt(uint8_t address);
    esp_err_t configure();

    bmp5_i2c_t sensor_{};
    bmp5_osr_odr_press_config osr_odr_press_cfg_{};
    bool initialized_ = false; // true only after a successful init()
};

#endif // SENSORS_BMP581_H
