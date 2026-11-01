#include "sht4x.h"

#include <esp_check.h>

constexpr const char* tag = "   ***Sensor SHT4X***   ";

// SHT4X Sensor Configuration (from Kconfig)
constexpr auto sda_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SDA_GPIO);
constexpr auto scl_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SCL_GPIO);
constexpr auto i2c_port = static_cast<i2c_port_t>(CONFIG_SENSORS_I2C_PORT);

Sht4x::~Sht4x() {
    if (initialized_) { sht4x_free_desc(&sensor_); }
}

std::optional<SensorValueRange> Sht4x::getValueRangeForType(SensorType type) const {
    // Temperature (°C)
    constexpr float temp_min_deg_celsius = -40.0F;
    constexpr float temp_max_deg_celsius = 125.0F;
    constexpr float temp_abs_threshold = 0.08F;

    // Relative Humidity (%RH)
    constexpr float humi_min_percent = 0.0F;
    constexpr float humi_max_percent = 100.0F;
    constexpr float humi_abs_threshold = 0.20F;

    switch (type) {
    case SensorType::TEMPERATURE:
        return SensorValueRange{.min = temp_min_deg_celsius,
                                .max = temp_max_deg_celsius,
                                .abs_threshold = temp_abs_threshold};
    case SensorType::PRESSURE:
        return std::nullopt;
    case SensorType::RELATIVE_HUMIDITY:
        return SensorValueRange{
            .min = humi_min_percent, .max = humi_max_percent, .abs_threshold = humi_abs_threshold};
    case SensorType::CARBON_DIOXIDE_CONCENTRATION:
    case SensorType::ILLUMINANCE:
        return std::nullopt;
    }
    return std::nullopt;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Sht4x::init(int64_t /*read_interval_ms*/) {
    // idempotent, avoids leaking on a second call
    ESP_RETURN_ON_FALSE(!initialized_, ESP_OK, tag, "The sensor is already initialized");

    ESP_LOGI(tag, "Initializing sensor...");

    // Safe to call from every I2C driver: i2cdev_init() is a no-op after the first call.
    auto ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, tag, "I2C initialization failed: %s", esp_err_to_name(ret));

    ret = sht4x_init_desc(&sensor_, i2c_port, sda_gpio, scl_gpio);
    ESP_RETURN_ON_ERROR(ret, tag, "initialization failed on init_desc: %s", esp_err_to_name(ret));

    ret = sht4x_init(&sensor_);
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on sht4x_init: %s", esp_err_to_name(ret));
    initialized_ = true;
    ESP_LOGI(tag,
             "Sensor initialized: %s (serial: 0x%08lx).",
             getModelName(),
             static_cast<unsigned long>(sensor_.serial));

    return ESP_OK;

cleanup:
    sht4x_free_desc(&sensor_); // desc was created above, so release it on failure
    return ret;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Sht4x::read(SensorReadings& readings, SensorReadings& /*compensations*/) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    readings[toIndex(SensorType::TEMPERATURE)] = std::nullopt;
    readings[toIndex(SensorType::RELATIVE_HUMIDITY)] = std::nullopt;

    float temperature = 0.0F;
    float relative_humidity = 0.0F;
    const auto ret = sht4x_measure(&sensor_, &temperature, &relative_humidity);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read sensor values: %s", esp_err_to_name(ret));

    readings[toIndex(SensorType::TEMPERATURE)] = temperature;
    readings[toIndex(SensorType::RELATIVE_HUMIDITY)] = relative_humidity;
    return ESP_OK;
}
