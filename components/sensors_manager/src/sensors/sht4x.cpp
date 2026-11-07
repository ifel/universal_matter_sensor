#include "sht4x.h"

#include <esp_check.h>

#include "i2c_bus.h"

constexpr const char* tag = "sensor_sht4x";

Sht4x::~Sht4x() {
    if (initialized_) { sht4x_free_desc(&sensor_); }
}

namespace {
// Temperature (°C), relative humidity (%RH)
constexpr std::array type_ranges = {
    SensorTypeRange{.type = SensorType::TEMPERATURE,
                    .range = {.min = -40.0F, .max = 125.0F, .abs_threshold = 0.08F}},
    SensorTypeRange{.type = SensorType::RELATIVE_HUMIDITY,
                    .range = {.min = 0.0F, .max = 100.0F, .abs_threshold = 0.20F}},
};
} // namespace

std::span<const SensorTypeRange> Sht4x::getTypeRanges() const {
    return type_ranges;
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
esp_err_t Sht4x::read(SensorReadings& readings, const SensorReadings& /*compensations*/) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    at<SensorType::TEMPERATURE>(readings) = std::nullopt;
    at<SensorType::RELATIVE_HUMIDITY>(readings) = std::nullopt;

    float temperature = 0.0F;
    float relative_humidity = 0.0F;
    const auto ret = sht4x_measure(&sensor_, &temperature, &relative_humidity);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read sensor values: %s", esp_err_to_name(ret));

    at<SensorType::TEMPERATURE>(readings) = temperature;
    at<SensorType::RELATIVE_HUMIDITY>(readings) = relative_humidity;
    return ESP_OK;
}
