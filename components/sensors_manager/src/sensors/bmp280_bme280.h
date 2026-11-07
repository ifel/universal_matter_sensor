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

    const char* getModelName() const override {
        return is_bme280_ ? "BME280" : "BMP280";
    }
    std::span<const SensorTypeRange> getTypeRanges() const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, const SensorReadings& compensations) override;

  private:
    esp_err_t initAt(uint8_t address);

    bmp280_t sensor_{};
    bool initialized_ = false; // true only after a successful init()
    bool is_bme280_ = false;   // BME280 supports humidity, BMP280 does not
};

#endif // SENSORS_BMP280_BME280_H
