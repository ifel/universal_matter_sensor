#include "bmp280_bme280.h"

#include <array>

#include <esp_check.h>

#include "i2c_bus.h"

constexpr const char* tag = "sensor_bmp280_bme280";

// Pin-selectable I2C addresses, usual default first
constexpr std::array<uint8_t, 2> i2c_addresses = {0x77, 0x76}; // SDO high, SDO to GND

Bmp280Bme280::~Bmp280Bme280() {
    if (initialized_) { bmp280_free_desc(&sensor_); }
}

namespace {
// Temperature (°C), pressure (Pa), relative humidity (%RH)
constexpr std::array bme280_type_ranges = {
    SensorTypeRange{.type = SensorType::TEMPERATURE,
                    .range = {.min = -40.0F, .max = 85.0F, .abs_threshold = 0.10F}},
    SensorTypeRange{.type = SensorType::PRESSURE,
                    .range = {.min = 30000.0F, .max = 110000.0F, .abs_threshold = 50.0F}},
    SensorTypeRange{.type = SensorType::RELATIVE_HUMIDITY,
                    .range = {.min = 0.0F, .max = 100.0F, .abs_threshold = 0.50F}},
};
// BMP280: no humidity
constexpr std::array bmp280_type_ranges = {
    SensorTypeRange{.type = SensorType::TEMPERATURE,
                    .range = {.min = -40.0F, .max = 85.0F, .abs_threshold = 0.10F}},
    SensorTypeRange{.type = SensorType::PRESSURE,
                    .range = {.min = 30000.0F, .max = 110000.0F, .abs_threshold = 50.0F}},
};
} // namespace

std::span<const SensorTypeRange> Bmp280Bme280::getTypeRanges() const {
    return is_bme280_ ? std::span<const SensorTypeRange>(bme280_type_ranges)
                      : std::span<const SensorTypeRange>(bmp280_type_ranges);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Bmp280Bme280::init(int64_t /*read_interval_ms*/) {
    // idempotent, avoids leaking on a second call
    ESP_RETURN_ON_FALSE(!initialized_, ESP_OK, tag, "The sensor is already initialized");

    ESP_LOGI(tag, "Initializing sensor...");

    // Safe to call from every I2C driver: i2cdev_init() is a no-op after the first call.
    auto ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, tag, "I2C initialization failed: %s", esp_err_to_name(ret));

    // Address depends on the module's address pin: try the usual default first.
    for (const uint8_t address : i2c_addresses) {
        ret = initAt(address);
        if (ret == ESP_OK) { break; }
    }
    ESP_RETURN_ON_ERROR(ret,
                        tag,
                        "No %s found at 0x%02x or 0x%02x",
                        getModelName(),
                        i2c_addresses[0],
                        i2c_addresses[1]);

    is_bme280_ = sensor_.id == BME280_CHIP_ID;
    initialized_ = true;
    ESP_LOGI(tag,
             "Sensor initialized: %s detected at 0x%02x (chip ID: 0x%02x).",
             getModelName(),
             sensor_.i2c_dev.addr,
             sensor_.id);

    return ESP_OK;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Bmp280Bme280::initAt(uint8_t address) {
    bmp280_params_t params;
    auto ret = bmp280_init_default_params(&params);
    ESP_RETURN_ON_ERROR(
        ret, tag, "initialization failed on bmp280_init_default_params: %s", esp_err_to_name(ret));

    ret = bmp280_init_desc(&sensor_, address, i2c_port, sda_gpio, scl_gpio);
    ESP_RETURN_ON_ERROR(
        ret, tag, "initialization failed on bmp280_init_desc: %s", esp_err_to_name(ret));

    ret = bmp280_init(&sensor_, &params);
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on bmp280_init failed: %s", esp_err_to_name(ret));
    return ESP_OK;

cleanup:
    bmp280_free_desc(&sensor_); // desc was created above, so release it on failure
    return ret;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Bmp280Bme280::read(SensorReadings& readings, const SensorReadings& /*compensations*/) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    at<SensorType::TEMPERATURE>(readings) = std::nullopt;
    at<SensorType::PRESSURE>(readings) = std::nullopt;
    at<SensorType::RELATIVE_HUMIDITY>(readings) = std::nullopt;
    float temperature = 0.0F;
    float pressure = 0.0F;
    float relative_humidity = 0.0F;
    const auto ret = bmp280_read_float(
        &sensor_, &temperature, &pressure, is_bme280_ ? &relative_humidity : nullptr);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read sensor values: %s", esp_err_to_name(ret));

    at<SensorType::TEMPERATURE>(readings) = temperature;
    at<SensorType::PRESSURE>(readings) = pressure;
    at<SensorType::RELATIVE_HUMIDITY>(readings) =
        is_bme280_ ? std::optional<float>(relative_humidity) : std::nullopt;
    return ESP_OK;
}
