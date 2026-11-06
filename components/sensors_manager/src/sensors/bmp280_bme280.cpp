#include "bmp280_bme280.h"

#include <array>

#include <esp_check.h>

constexpr const char* tag = "   ***Sensor BMP280 BME280***   ";

// BMP280 / BME280 Sensor Configuration (from Kconfig)
constexpr std::array<uint8_t, 2> i2c_addresses = {0x77, 0x76}; // SDO high, SDO to GND
constexpr auto bmp280_sda_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SDA_GPIO);
constexpr auto bmp280_scl_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SCL_GPIO);
constexpr auto bmp280_i2c_port = static_cast<i2c_port_t>(CONFIG_SENSORS_I2C_PORT);

Bmp280Bme280::~Bmp280Bme280() {
    if (initialized_) { bmp280_free_desc(&sensor_); }
}

std::optional<SensorValueRange> Bmp280Bme280::getValueRangeForType(SensorType type) const {
    // Temperature (°C)
    constexpr float bmp_280_temp_min_deg_celsius = -40.0F;
    constexpr float bmp_280_temp_max_deg_celsius = 85.0F;
    constexpr float bmp_280_temp_abs_threshold = 0.10F;

    // Pressure (Pa)
    constexpr float bmp_280_press_min_pa = 30000.0F;
    constexpr float bmp_280_press_max_pa = 110000.0F;
    constexpr float bmp_280_press_abs_threshold = 50.0F;

    // Relative Humidity (%RH)
    constexpr float bmp_280_humi_min_percent = 0.0F;
    constexpr float bmp_280_humi_max_percent = 100.0F;
    constexpr float bmp_280_humi_abs_threshold = 0.50F;

    switch (type) {
    case SensorType::TEMPERATURE:
        return SensorValueRange{.min = bmp_280_temp_min_deg_celsius,
                                .max = bmp_280_temp_max_deg_celsius,
                                .abs_threshold = bmp_280_temp_abs_threshold};
    case SensorType::PRESSURE:
        return SensorValueRange{.min = bmp_280_press_min_pa,
                                .max = bmp_280_press_max_pa,
                                .abs_threshold = bmp_280_press_abs_threshold};
    case SensorType::RELATIVE_HUMIDITY:
        if (is_bme280_) {
            return SensorValueRange{.min = bmp_280_humi_min_percent,
                                    .max = bmp_280_humi_max_percent,
                                    .abs_threshold = bmp_280_humi_abs_threshold};
        }
        break;
    case SensorType::CARBON_DIOXIDE_CONCENTRATION:
    case SensorType::ILLUMINANCE:
    case SensorType::TVOC_CONCENTRATION:
    case SensorType::AIR_QUALITY:
    case SensorType::REMAINING_LEVEL:
    case SensorType::BATTERY_PERCENT:
    case SensorType::BATTERY_VOLTAGE:
    case SensorType::BATTERY_CHARGE_STATE:
        return std::nullopt;
    }
    return std::nullopt;
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

    ret = bmp280_init_desc(&sensor_, address, bmp280_i2c_port, bmp280_sda_gpio, bmp280_scl_gpio);
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
esp_err_t Bmp280Bme280::read(SensorReadings& readings, SensorReadings& /*compensations*/) {
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
