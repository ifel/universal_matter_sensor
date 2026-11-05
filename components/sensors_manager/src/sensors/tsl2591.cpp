#include "tsl2591.h"

#include <algorithm>
#include <array>
#include <cstdint>

#include <esp_check.h>

constexpr const char* tag = "   ***Sensor TSL2591***   ";

// TSL2591 Sensor Configuration (from Kconfig)
constexpr auto sda_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SDA_GPIO);
constexpr auto scl_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SCL_GPIO);
constexpr auto i2c_port = static_cast<i2c_port_t>(CONFIG_SENSORS_I2C_PORT);

// Illuminance (Lux)
constexpr float illum_min_lux = 1.0F;
constexpr float illum_max_lux = 88'000.0F;

namespace {
using step_t = struct {
    tsl2591_gain_t gain;
    tsl2591_integration_time_t it;
    uint16_t it_ms;
    float gain_factor; // datasheet typical gain relative to LOW
};

// Least to most sensitive; the driver has no auto-gain.
constexpr std::array<step_t, 6> ladder{{
    {.gain = TSL2591_GAIN_LOW, .it = TSL2591_INTEGRATION_100MS, .it_ms = 100, .gain_factor = 1.0F},
    {.gain = TSL2591_GAIN_MEDIUM,
     .it = TSL2591_INTEGRATION_100MS,
     .it_ms = 100,
     .gain_factor = 25.0F},
    {.gain = TSL2591_GAIN_HIGH,
     .it = TSL2591_INTEGRATION_100MS,
     .it_ms = 100,
     .gain_factor = 428.0F},
    {.gain = TSL2591_GAIN_HIGH,
     .it = TSL2591_INTEGRATION_300MS,
     .it_ms = 300,
     .gain_factor = 428.0F},
    {.gain = TSL2591_GAIN_MAX,
     .it = TSL2591_INTEGRATION_300MS,
     .it_ms = 300,
     .gain_factor = 9876.0F},
    {.gain = TSL2591_GAIN_MAX,
     .it = TSL2591_INTEGRATION_600MS,
     .it_ms = 600,
     .gain_factor = 9876.0F},
}};

// Full-scale ADC count: 37888 at 100 ms, 65535 for longer integration (datasheet).
constexpr float maxCounts(const step_t& step) {
    return step.it_ms == 100 ? 37888.0F : 65535.0F;
}

constexpr float sensitivity(const step_t& step) {
    return step.gain_factor * static_cast<float>(step.it_ms);
}

constexpr float saturation_fraction = 0.9F; // step down above this share of full scale
constexpr float target_fraction = 0.5F;     // step up only if the result stays below this
} // namespace

Tsl2591::~Tsl2591() {
    if (initialized_) {
        tsl2591_set_power_status(&sensor_, TSL2591_POWER_OFF);
        tsl2591_free_desc(&sensor_);
    }
}

std::optional<SensorValueRange> Tsl2591::getValueRangeForType(SensorType type) const {
    constexpr float illum_abs_threshold = 1.0F;
    constexpr float illum_rel_threshold = 0.03F;

    switch (type) {
    case SensorType::TEMPERATURE:
    case SensorType::PRESSURE:
    case SensorType::RELATIVE_HUMIDITY:
    case SensorType::CARBON_DIOXIDE_CONCENTRATION:
    case SensorType::TVOC_CONCENTRATION:
    case SensorType::AIR_QUALITY:
        return std::nullopt;
    case SensorType::ILLUMINANCE:
        return SensorValueRange{.min = illum_min_lux,
                                .max = illum_max_lux,
                                .abs_threshold = illum_abs_threshold,
                                .rel_threshold = illum_rel_threshold};
    }
    return std::nullopt;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Tsl2591::init(int64_t /*read_interval_ms*/) {
    // idempotent, avoids leaking on a second call
    ESP_RETURN_ON_FALSE(!initialized_, ESP_OK, tag, "The sensor is already initialized");

    ESP_LOGI(tag, "Initializing sensor...");

    // Safe to call from every I2C driver: i2cdev_init() is a no-op after the first call.
    auto ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, tag, "I2C initialization failed: %s", esp_err_to_name(ret));

    ret = tsl2591_init_desc(&sensor_, i2c_port, sda_gpio, scl_gpio);
    ESP_RETURN_ON_ERROR(ret, tag, "initialization failed on init_desc: %s", esp_err_to_name(ret));

    ret = tsl2591_init(&sensor_);
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on tsl2591_init: %s", esp_err_to_name(ret));

    ret = applyLadderStep();
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on applyLadderStep: %s", esp_err_to_name(ret));

    initialized_ = true;
    ESP_LOGI(tag, "Sensor initialized: %s.", getModelName());

    return ESP_OK;

cleanup:
    tsl2591_free_desc(&sensor_); // desc was created above, so release it on failure
    return ret;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Tsl2591::read(SensorReadings& readings, SensorReadings& /*compensations*/) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    at<SensorType::ILLUMINANCE>(readings) = std::nullopt;

    for (size_t attempt = 0; attempt < ladder.size(); ++attempt) {
        uint16_t full_spectrum = 0;
        uint16_t infrared = 0;
        auto ret = tsl2591_get_channel_data(&sensor_, &full_spectrum, &infrared);
        ESP_RETURN_ON_ERROR(ret, tag, "Could not read channel data: %s", esp_err_to_name(ret));

        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        const auto& step = ladder[ladderIdx_];
        const auto counts = static_cast<float>(full_spectrum);
        const bool saturated = counts > saturation_fraction * maxCounts(step) ||
                               static_cast<float>(infrared) > saturation_fraction * maxCounts(step);
        if (saturated && ladderIdx_ > 0) {
            ladderIdx_--;
            ESP_LOGD(tag, "Counts %u saturated, ladder index %u", full_spectrum, ladderIdx_);
            ESP_RETURN_ON_ERROR(applyLadderStep(), tag, "read failed trying to change range");
            continue;
        }
        if (!saturated && ladderIdx_ + 1U < ladder.size()) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            const auto& next = ladder[ladderIdx_ + 1U];
            if (counts * sensitivity(next) / sensitivity(step) <
                target_fraction * maxCounts(next)) {
                ladderIdx_++;
                ESP_LOGD(tag, "Counts %u too low, ladder index %u", full_spectrum, ladderIdx_);
                ESP_RETURN_ON_ERROR(applyLadderStep(), tag, "read failed trying to change range");
                continue;
            }
        }

        if (saturated) { // brighter than the least sensitive step can measure
            at<SensorType::ILLUMINANCE>(readings) = illum_max_lux;
            return ESP_OK;
        }
        if (full_spectrum == 0) { // the driver's formula divides by it
            at<SensorType::ILLUMINANCE>(readings) = 0.0F;
            return ESP_OK;
        }
        float lux = 0.0F;
        ret = tsl2591_calculate_lux(&sensor_, full_spectrum, infrared, &lux);
        ESP_RETURN_ON_ERROR(ret, tag, "Could not calculate lux: %s", esp_err_to_name(ret));
        at<SensorType::ILLUMINANCE>(readings) = std::clamp(lux, 0.0F, illum_max_lux);
        return ESP_OK;
    }

    ESP_LOGW(tag, "Light level did not settle within the gain ladder");
    return ESP_ERR_NOT_FINISHED;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Tsl2591::applyLadderStep() {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    const auto& step = ladder[ladderIdx_];
    auto ret = tsl2591_set_gain(&sensor_, step.gain);
    ESP_RETURN_ON_ERROR(ret, tag, "Could not set gain: %s", esp_err_to_name(ret));
    ret = tsl2591_set_integration_time(&sensor_, step.it);
    ESP_RETURN_ON_ERROR(ret, tag, "Could not set integration time: %s", esp_err_to_name(ret));
    // A cycle already running finishes with the old settings, so wait for two.
    vTaskDelay(pdMS_TO_TICKS((2U * step.it_ms) + 10U));
    return ESP_OK;
}
