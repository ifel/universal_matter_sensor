#include "ens160.h"

#include <esp_check.h>

constexpr const char* tag = "   ***Sensor ENS160***   ";

// ENS160 Sensor Configuration (from Kconfig)
constexpr auto sda_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SDA_GPIO);
constexpr auto scl_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SCL_GPIO);
constexpr auto i2c_port = static_cast<i2c_port_t>(CONFIG_SENSORS_I2C_PORT);
constexpr auto i2c_address = static_cast<uint8_t>(CONFIG_ENS160_I2C_ADDRESS);

namespace {
std::optional<AirQualityLevel> ubaToAirQuality(ens160_aqi_uba_indexes_t uba_aqi) {
    switch (uba_aqi) {
    case ENS160_AQI_UBA_INDEX_1:
        return AirQualityLevel::GOOD;
    case ENS160_AQI_UBA_INDEX_2:
        return AirQualityLevel::FAIR;
    case ENS160_AQI_UBA_INDEX_3:
        return AirQualityLevel::MODERATE;
    case ENS160_AQI_UBA_INDEX_4:
        return AirQualityLevel::POOR;
    case ENS160_AQI_UBA_INDEX_5:
        return AirQualityLevel::VERY_POOR;
    case ENS160_AQI_UBA_INDEX_UNKNOWN:
        return std::nullopt;
    }
    return std::nullopt;
}
} // namespace

Ens160::~Ens160() {
    if (initialized_) {
        ens160_enable_deep_sleep_mode(&sensor_);
        ens160_free_desc(&sensor_);
    }
}

std::optional<SensorValueRange> Ens160::getValueRangeForType(SensorType type) const {
    // Equivalent CO2 (ppm)
    constexpr auto eco2_min_ppm = static_cast<float>(ENS160_ECO2_MIN);
    constexpr auto eco2_max_ppm = static_cast<float>(ENS160_ECO2_MAX);
    constexpr float eco2_abs_threshold = 20.0F;
    constexpr float eco2_rel_threshold = 0.02F;

    // TVOC (ppb)
    constexpr auto tvoc_min_ppb = static_cast<float>(ENS160_TVOC_MIN);
    constexpr auto tvoc_max_ppb = static_cast<float>(ENS160_TVOC_MAX);
    constexpr float tvoc_abs_threshold = 10.0F;
    constexpr float tvoc_rel_threshold = 0.05F;

    // Air quality from UBA index (AirQualityLevel)
    constexpr auto aq_min_level = static_cast<float>(AirQualityLevel::GOOD);
    constexpr auto aq_max_level = static_cast<float>(AirQualityLevel::VERY_POOR);
    constexpr float aq_abs_threshold = 1.0F;

    switch (type) {
    case SensorType::TEMPERATURE:
    case SensorType::PRESSURE:
    case SensorType::RELATIVE_HUMIDITY:
    case SensorType::ILLUMINANCE:
        return std::nullopt;
    case SensorType::CARBON_DIOXIDE_CONCENTRATION:
        return SensorValueRange{
            .min = eco2_min_ppm,
            .max = eco2_max_ppm,
            .abs_threshold = eco2_abs_threshold,
            .rel_threshold = eco2_rel_threshold,
        };
    case SensorType::TVOC_CONCENTRATION:
        return SensorValueRange{
            .min = tvoc_min_ppb,
            .max = tvoc_max_ppb,
            .abs_threshold = tvoc_abs_threshold,
            .rel_threshold = tvoc_rel_threshold,
        };
    case SensorType::AIR_QUALITY:
        return SensorValueRange{
            .min = aq_min_level,
            .max = aq_max_level,
            .abs_threshold = aq_abs_threshold,
        };
    }
    return std::nullopt;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Ens160::init(int64_t /*read_interval_ms*/) {
    // idempotent, avoids leaking on a second call
    ESP_RETURN_ON_FALSE(!initialized_, ESP_OK, tag, "The sensor is already initialized");

    ESP_LOGI(tag, "Initializing sensor...");

    // Safe to call from every I2C driver: i2cdev_init() is a no-op after the first call.
    auto ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, tag, "I2C initialization failed: %s", esp_err_to_name(ret));

    ret = ens160_init_desc(&sensor_, i2c_address, i2c_port, sda_gpio, scl_gpio);
    ESP_RETURN_ON_ERROR(ret, tag, "initialization failed on init_desc: %s", esp_err_to_name(ret));

    ret = ens160_init(&sensor_);
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on ens160_init: %s", esp_err_to_name(ret));
    initialized_ = true;
    ESP_LOGI(tag,
             "Sensor initialized: %s (part id: 0x%04x).",
             getModelName(),
             static_cast<unsigned int>(sensor_.part_id));

    return ESP_OK;

cleanup:
    ens160_free_desc(&sensor_); // desc was created above, so release it on failure
    return ret;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Ens160::read(SensorReadings& readings, SensorReadings& compensations) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(readings) = std::nullopt;
    at<SensorType::TVOC_CONCENTRATION>(readings) = std::nullopt;
    at<SensorType::AIR_QUALITY>(readings) = std::nullopt;

    // Not fatal: the sensor keeps working with its previous (or default) compensation.
    if (const auto ret = updateCompensation(compensations); ret != ESP_OK) {
        ESP_LOGW(tag, "Failed to update compensation: %s", esp_err_to_name(ret));
    }

    ens160_validity_flags_t validity = ENS160_VALFLAG_INVALID_OUTPUT;
    auto ret = ens160_get_validity_status(&sensor_, &validity);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read validity status: %s", esp_err_to_name(ret));
    if (validity == ENS160_VALFLAG_WARMUP || validity == ENS160_VALFLAG_INVALID_OUTPUT) {
        ESP_LOGI(tag,
                 "Sensor output is not valid yet (%s)",
                 validity == ENS160_VALFLAG_WARMUP ? "warm-up" : "invalid output");
        return ESP_OK;
    }

    ens160_air_quality_data_t data{};
    ret = ens160_get_measurement(&sensor_, &data);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read sensor values: %s", esp_err_to_name(ret));

    at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(readings) = static_cast<float>(data.eco2);
    at<SensorType::TVOC_CONCENTRATION>(readings) = static_cast<float>(data.tvoc);
    if (const auto air_quality = ubaToAirQuality(data.uba_aqi)) {
        at<SensorType::AIR_QUALITY>(readings) = static_cast<float>(*air_quality);
    }
    return ESP_OK;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Ens160::updateCompensation(const SensorReadings& compensations) {
    const auto& temperature = at<SensorType::TEMPERATURE>(compensations);
    const auto& humidity = at<SensorType::RELATIVE_HUMIDITY>(compensations);
    if (!temperature || !humidity) { return ESP_OK; } // keep previous compensation
    if (temperature == temperature_comp_ && humidity == humidity_comp_) { return ESP_OK; }

    ESP_LOGD(tag, "Updating compensation to %.2f°C, %.2f %%RH", *temperature, *humidity);
    ESP_RETURN_ON_ERROR(ens160_set_compensation_factors(&sensor_, *temperature, *humidity),
                        tag,
                        "Failed to set compensation factors");
    temperature_comp_ = temperature;
    humidity_comp_ = humidity;
    return ESP_OK;
}
