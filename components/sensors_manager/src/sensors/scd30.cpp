#include "scd30.h"

#include <cmath>

#include <esp_check.h>

constexpr const char* tag = "   ***Sensor SCD30***   ";

// SHT4X Sensor Configuration (from Kconfig)
constexpr auto sda_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SDA_GPIO);
constexpr auto scl_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SCL_GPIO);
constexpr auto i2c_port = static_cast<i2c_port_t>(CONFIG_SENSORS_I2C_PORT);

namespace {
AirQualityLevel co2ToAirQuality(float co2_ppm) {
    if (!std::isfinite(co2_ppm)) { return AirQualityLevel::UNKNOWN; }
    if (co2_ppm < 800.0F) { return AirQualityLevel::GOOD; }
    if (co2_ppm < 1000.0F) { return AirQualityLevel::FAIR; }
    if (co2_ppm < 1500.0F) { return AirQualityLevel::MODERATE; }
    if (co2_ppm < 2000.0F) { return AirQualityLevel::POOR; }
    if (co2_ppm < 5000.0F) { return AirQualityLevel::VERY_POOR; }
    return AirQualityLevel::EXTREMELY_POOR;
}
} // namespace

Scd30::~Scd30() {
    if (initialized_) {
        scd30_stop_continuous_measurement(&sensor_);
        scd30_free_desc(&sensor_);
    }
}

std::optional<SensorValueRange> Scd30::getValueRangeForType(SensorType type) const {
    // Temperature (°C) from embedded SHT31
    constexpr float temp_min_deg_celsius = -40.0F;
    constexpr float temp_max_deg_celsius = 125.0F;
    constexpr float temp_abs_threshold = 0.08F;

    // Relative Humidity (%RH)
    constexpr float humi_min_percent = 0.0F;
    constexpr float humi_max_percent = 100.0F;
    constexpr float humi_abs_threshold = 0.20F;

    // CO2 (ppm)
    constexpr float co2_min_percent = 400.0F;
    constexpr float co2_max_percent = 10'000.0F;
    constexpr float co2_abs_threshold = 20.0F;
    constexpr float co2_rel_threshold = 0.02F;

    // Air quality derived from CO2 (AirQualityLevel)
    constexpr auto aq_min_level = static_cast<float>(AirQualityLevel::GOOD);
    constexpr auto aq_max_level = static_cast<float>(AirQualityLevel::EXTREMELY_POOR);
    constexpr float aq_abs_threshold = 1.0F;

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
        return SensorValueRange{.min = co2_min_percent,
                                .max = co2_max_percent,
                                .abs_threshold = co2_abs_threshold,
                                .rel_threshold = co2_rel_threshold};
    case SensorType::AIR_QUALITY:
        return SensorValueRange{
            .min = aq_min_level, .max = aq_max_level, .abs_threshold = aq_abs_threshold};
    case SensorType::REMAINING_LEVEL:
    case SensorType::BATTERY_PERCENT:
    case SensorType::BATTERY_VOLTAGE:
    case SensorType::BATTERY_CHARGE_STATE:
    case SensorType::ILLUMINANCE:
    case SensorType::TVOC_CONCENTRATION:
        return std::nullopt;
    }
    return std::nullopt;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Scd30::init(int64_t read_interval_ms) {
    if (initialized_) {
        ESP_LOGW(tag, "The sensor is already initialized");
        return ESP_OK;
    } // idempotent, avoids leaking on a second call

    ESP_LOGI(tag, "Initializing sensor...");

    // Safe to call from every I2C driver: i2cdev_init() is a no-op after the first call.
    auto ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, tag, "I2C initialization failed: %s", esp_err_to_name(ret));

    ret = scd30_init_desc(&sensor_, i2c_port, sda_gpio, scl_gpio);
    ESP_RETURN_ON_ERROR(ret, tag, "Initialization failed on init_desc: %s", esp_err_to_name(ret));

    ret = scd30_trigger_continuous_measurement(&sensor_, pressure_comp_);
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      tag,
                      "Initialization failed on scd30_trigger_continuous_measurement: %s",
                      esp_err_to_name(ret));

    ret = scd30_set_measurement_interval(&sensor_, read_interval_ms / 1000);
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      tag,
                      "Initialization failed on scd30_set_measurement_interval: %s",
                      esp_err_to_name(ret));
    initialized_ = true;
    ESP_LOGI(tag, "Sensor initialized: %s.", getModelName());

    return ESP_OK;

cleanup:
    scd30_free_desc(&sensor_); // desc was created above, so release it on failure
    return ret;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Scd30::read(SensorReadings& readings, SensorReadings& compensations) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    auto ret = ESP_OK;
    at<SensorType::TEMPERATURE>(readings) = std::nullopt;
    at<SensorType::RELATIVE_HUMIDITY>(readings) = std::nullopt;
    at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(readings) = std::nullopt;
    at<SensorType::AIR_QUALITY>(readings) = std::nullopt;

    uint16_t pressure_comp = 0;
    if (const auto new_pressure_comp = at<SensorType::PRESSURE>(compensations); new_pressure_comp) {
        pressure_comp = (uint16_t)std::roundf(new_pressure_comp.value() / 100.0F);
    }
    if (pressure_comp != pressure_comp_) {
        ESP_LOGD(
            tag, "Updating pressure compensation from %d to %d", pressure_comp_, pressure_comp);

        ret = scd30_trigger_continuous_measurement(&sensor_, pressure_comp);
        ESP_RETURN_ON_ERROR(
            ret,
            tag,
            "Failed to trigger continuous measurement with new pressure compensation %d: %s",
            pressure_comp,
            esp_err_to_name(ret));
        pressure_comp_ = pressure_comp;
    }

    bool data_ready = false;
    ret = scd30_get_data_ready_status(&sensor_, &data_ready);
    ESP_RETURN_ON_ERROR(
        ret, tag, "Failed to check sensor data readiness: %s", esp_err_to_name(ret));
    ESP_RETURN_ON_FALSE(data_ready, ESP_ERR_NOT_FINISHED, tag, "Sensor data not ready yet");

    float co2 = 0.0F;
    float temperature = 0.0F;
    float relative_humidity = 0.0F;
    ret = scd30_read_measurement(&sensor_, &co2, &temperature, &relative_humidity);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read sensor values: %s", esp_err_to_name(ret));

    at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(readings) = co2;
    if (const auto air_quality = co2ToAirQuality(co2); air_quality != AirQualityLevel::UNKNOWN) {
        at<SensorType::AIR_QUALITY>(readings) = static_cast<float>(air_quality);
    }
    at<SensorType::TEMPERATURE>(readings) = temperature;
    at<SensorType::RELATIVE_HUMIDITY>(readings) = relative_humidity;

    return ESP_OK;
}
