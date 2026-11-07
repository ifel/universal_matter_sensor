#include "ens160.h"

#include <array>

#include <esp_check.h>

#include "i2c_bus.h"

constexpr const char* tag = "sensor_ens160";

// Pin-selectable I2C addresses, usual default first
constexpr std::array<uint8_t, 2> i2c_addresses = {0x53, 0x52}; // ADDR high, ADDR to GND

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

namespace {
// Equivalent CO2 (ppm), TVOC (ppb), air quality (AirQualityLevel, from the UBA index)
constexpr std::array type_ranges = {
    SensorTypeRange{.type = SensorType::CARBON_DIOXIDE_CONCENTRATION,
                    .range = {.min = static_cast<float>(ENS160_ECO2_MIN),
                              .max = static_cast<float>(ENS160_ECO2_MAX),
                              .abs_threshold = 20.0F,
                              .rel_threshold = 0.02F}},
    SensorTypeRange{.type = SensorType::TVOC_CONCENTRATION,
                    .range = {.min = static_cast<float>(ENS160_TVOC_MIN),
                              .max = static_cast<float>(ENS160_TVOC_MAX),
                              .abs_threshold = 10.0F,
                              .rel_threshold = 0.05F}},
    SensorTypeRange{.type = SensorType::AIR_QUALITY,
                    .range = {.min = static_cast<float>(AirQualityLevel::GOOD),
                              .max = static_cast<float>(AirQualityLevel::VERY_POOR),
                              .abs_threshold = 1.0F}},
};
} // namespace

std::span<const SensorTypeRange> Ens160::getTypeRanges() const {
    return type_ranges;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Ens160::init(int64_t /*read_interval_ms*/) {
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

    initialized_ = true;
    ESP_LOGI(tag,
             "Sensor initialized: %s at 0x%02x (part id: 0x%04x).",
             getModelName(),
             sensor_.i2c_dev.addr,
             static_cast<unsigned int>(sensor_.part_id));

    return ESP_OK;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Ens160::initAt(uint8_t address) {
    auto ret = ens160_init_desc(&sensor_, address, i2c_port, sda_gpio, scl_gpio);
    ESP_RETURN_ON_ERROR(ret, tag, "initialization failed on init_desc: %s", esp_err_to_name(ret));

    ret = ens160_init(&sensor_);
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on ens160_init: %s", esp_err_to_name(ret));
    return ESP_OK;

cleanup:
    ens160_free_desc(&sensor_); // desc was created above, so release it on failure
    return ret;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Ens160::read(SensorReadings& readings, const SensorReadings& compensations) {
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
