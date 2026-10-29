#include "bmp280_bme280.h"

#include "esp_log.h"

constexpr const char* tag = "   ***Sensor BMP280 BME280***   ";

// BMP280 / BME280 Sensor Configuration (from Kconfig)
constexpr uint8_t bmp280_i2c_address = CONFIG_BMP280_I2C_ADDRESS;
constexpr auto bmp280_sda_gpio = static_cast<gpio_num_t>(CONFIG_BMP280_I2C_SDA_GPIO);
constexpr auto bmp280_scl_gpio = static_cast<gpio_num_t>(CONFIG_BMP280_I2C_SCL_GPIO);
constexpr auto bmp280_i2c_port = static_cast<i2c_port_t>(CONFIG_BMP280_I2C_PORT);

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
    }
    return std::nullopt;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Bmp280Bme280::init() {
    if (initialized_) {
        ESP_LOGW(tag, "The sensor is already initialized");
        return ESP_OK;
    } // idempotent, avoids leaking on a second call

    ESP_LOGI(tag, "Initializing sensor...");

    // Safe to call from every I2C driver: i2cdev_init() is a no-op after the first call.
    auto err = i2cdev_init();
    if (err != ESP_OK) {
        ESP_LOGE(tag, "I2C initialization failed: %s", esp_err_to_name(err));
        return err;
    }

    bmp280_params_t params;
    err = bmp280_init_default_params(&params);
    if (err != ESP_OK) {
        ESP_LOGE(tag,
                 "BMP280 initialization bmp280_init_default_params failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = bmp280_init_desc(
        &sensor_, bmp280_i2c_address, bmp280_i2c_port, bmp280_sda_gpio, bmp280_scl_gpio);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "BMP280 initialization bmp280_init_desc failed: %s", esp_err_to_name(err));
        return err;
    }

    err = bmp280_init(&sensor_, &params);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "BMP280 initialization bmp280_init failed: %s", esp_err_to_name(err));
        bmp280_free_desc(&sensor_); // desc was created above, so release it on failure
        return err;
    }
    is_bme280_ = sensor_.id == BME280_CHIP_ID;
    initialized_ = true;
    ESP_LOGI(tag, "Sensor initialized: %s detected (chip ID: 0x%02x).", getModelName(), sensor_.id);

    return ESP_OK;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Bmp280Bme280::read(SensorReadings& readings) {
    if (!initialized_) {
        ESP_LOGW(tag, "Cannot read values on non initialized sensor");
        return ESP_ERR_INVALID_STATE;
    } // guard instead of a null deref

    float temperature = 0.0F;
    float pressure = 0.0F;
    float relative_humidity = 0.0F;
    const auto ret = bmp280_read_float(
        &sensor_, &temperature, &pressure, is_bme280_ ? &relative_humidity : nullptr);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to read sensor values: %s", esp_err_to_name(ret));
        readings[toIndex(SensorType::TEMPERATURE)] = std::nullopt;
        readings[toIndex(SensorType::PRESSURE)] = std::nullopt;
        readings[toIndex(SensorType::RELATIVE_HUMIDITY)] = std::nullopt;
        return ret;
    }

    readings[toIndex(SensorType::TEMPERATURE)] = temperature;
    readings[toIndex(SensorType::PRESSURE)] = pressure;
    readings[toIndex(SensorType::RELATIVE_HUMIDITY)] =
        is_bme280_ ? std::optional<float>(relative_humidity) : std::nullopt;
    return ESP_OK;
}
