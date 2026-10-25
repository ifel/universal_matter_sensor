#ifndef SENSORS_BMP280_BME280_H
#define SENSORS_BMP280_BME280_H

#include <array>

#include <bmp280.h>

#include "sensors_manager/sensor.h"

class Bmp280Bme280 : public Sensor {
  public:
    Bmp280Bme280() = default;
    ~Bmp280Bme280() override;
    // Prevent copying
    Bmp280Bme280(const Bmp280Bme280&) = delete;
    Bmp280Bme280& operator=(const Bmp280Bme280&) = delete;

    // Prevent moving
    Bmp280Bme280(Bmp280Bme280&&) = delete;
    Bmp280Bme280& operator=(Bmp280Bme280&&) = delete;

    std::span<const SensorType> getTypes() const override {
        static constexpr std::array k_bme280_types = {
            SensorType::TEMPERATURE,
            SensorType::PRESSURE,
            SensorType::RELATIVE_HUMIDITY,
        };
        static constexpr std::array k_bmp280_types = {
            SensorType::TEMPERATURE,
            SensorType::PRESSURE,
        };
        return is_bme280_ ? std::span<const SensorType>(k_bme280_types)
                          : std::span<const SensorType>(k_bmp280_types);
    }
    const char* getModelName() const override {
        return is_bme280_ ? "BME280" : "BMP280";
    }
    std::optional<SensorValueRange> getValueRangeForType(SensorType type) const override;
    esp_err_t init() override;
    esp_err_t read(SensorReadings& readings) override;

  private:
    bmp280_t sensor_{};
    bool initialized_ = false; // true only after a successful init()
    bool is_bme280_ = false;   // BME280 supports humidity, BMP280 does not
};

#endif // SENSORS_BMP280_BME280_H
