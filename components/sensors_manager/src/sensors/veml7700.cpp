#include "veml7700.h"

#include <array>
#include <cstdint>

#include <esp_check.h>

constexpr const char* tag = "   ***Sensor VEML7700***   ";

// VEML7700 Sensor Configuration (from Kconfig)
constexpr auto sda_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SDA_GPIO);
constexpr auto scl_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SCL_GPIO);
constexpr auto i2c_port = static_cast<i2c_port_t>(CONFIG_SENSORS_I2C_PORT);

using step_t = struct {
    uint16_t gain, it, it_ms;
    float res;
};

static constexpr std::array<step_t, 9> ladder{{
    {.gain = VEML7700_GAIN_DIV_8,
     .it = VEML7700_INTEGRATION_TIME_25MS,
     .it_ms = 25,
     .res = 1.8432F},
    {.gain = VEML7700_GAIN_DIV_8,
     .it = VEML7700_INTEGRATION_TIME_50MS,
     .it_ms = 50,
     .res = 0.9216F},
    {.gain = VEML7700_GAIN_DIV_8,
     .it = VEML7700_INTEGRATION_TIME_100MS,
     .it_ms = 100,
     .res = 0.4608F},
    {.gain = VEML7700_GAIN_DIV_4,
     .it = VEML7700_INTEGRATION_TIME_100MS,
     .it_ms = 100,
     .res = 0.2304F},
    {.gain = VEML7700_GAIN_1, .it = VEML7700_INTEGRATION_TIME_100MS, .it_ms = 100, .res = 0.0576F},
    {.gain = VEML7700_GAIN_2, .it = VEML7700_INTEGRATION_TIME_100MS, .it_ms = 100, .res = 0.0288F},
    {.gain = VEML7700_GAIN_2, .it = VEML7700_INTEGRATION_TIME_200MS, .it_ms = 200, .res = 0.0144F},
    {.gain = VEML7700_GAIN_2, .it = VEML7700_INTEGRATION_TIME_400MS, .it_ms = 400, .res = 0.0072F},
    {.gain = VEML7700_GAIN_2, .it = VEML7700_INTEGRATION_TIME_800MS, .it_ms = 800, .res = 0.0036F},
}};

namespace {
constexpr uint8_t als_result_register = 0x04;
constexpr uint16_t max_counts = 10000; // step down above this (well below saturation)
constexpr uint16_t min_counts = 100;   // step up at or below this (resolution)

// Vishay "Designing the VEML7700 Into an Application": at gain 1/8 and 1/4 the response is
// non-linear above 1000 lx; this polynomial corrects it.
constexpr float correction_min_lux = 1000.0F;

float correctHighLux(float lux) {
    return (((((6.0135e-13F * lux) - 9.3924e-9F) * lux + 8.1488e-5F) * lux) + 1.0023F) * lux;
}

bool isLowGain(uint16_t gain) {
    return gain == VEML7700_GAIN_DIV_8 || gain == VEML7700_GAIN_DIV_4;
}

esp_err_t readAlsCounts(i2c_dev_t* dev, uint16_t* counts) {
    std::array<uint8_t, 2> bytes{};
    I2C_DEV_TAKE_MUTEX(dev); // NOLINT(readability-identifier-length)
    auto err = i2c_dev_read_reg(dev, als_result_register, &bytes, 2);
    I2C_DEV_GIVE_MUTEX(dev); // NOLINT(readability-identifier-length)
    if (err == ESP_OK) {
        *counts = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8); // LSB first
    }
    return err;
}
} // namespace

Veml7700::~Veml7700() {
    if (initialized_) { veml7700_free_desc(&sensor_); }
}

std::optional<SensorValueRange> Veml7700::getValueRangeForType(SensorType type) const {
    // Illuminance (Lux)
    constexpr float illum_min_lux = 1.0F;
    constexpr float illum_max_lux = 140'000.0F;
    constexpr float illum_abs_threshold = 1.0F;
    constexpr float illum_rel_threshold = 0.03F;

    switch (type) {
    case SensorType::TEMPERATURE:
    case SensorType::PRESSURE:
    case SensorType::RELATIVE_HUMIDITY:
    case SensorType::CARBON_DIOXIDE_CONCENTRATION:
    case SensorType::TVOC_CONCENTRATION:
    case SensorType::AIR_QUALITY:
    case SensorType::REMAINING_LEVEL:
    case SensorType::BATTERY_PERCENT:
    case SensorType::BATTERY_VOLTAGE:
    case SensorType::BATTERY_CHARGE_STATE:
    case SensorType::OCCUPANCY:
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
esp_err_t Veml7700::init(int64_t /*read_interval_ms*/) {
    // idempotent, avoids leaking on a second call
    ESP_RETURN_ON_FALSE(!initialized_, ESP_OK, tag, "The sensor is already initialized");

    ESP_LOGI(tag, "Initializing sensor...");

    // Safe to call from every I2C driver: i2cdev_init() is a no-op after the first call.
    auto ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, tag, "I2C initialization failed: %s", esp_err_to_name(ret));

    ret = veml7700_init_desc(&sensor_, i2c_port, sda_gpio, scl_gpio);
    ESP_RETURN_ON_ERROR(ret, tag, "initialization failed on init_desc: %s", esp_err_to_name(ret));

    config_ = {
        .gain = VEML7700_GAIN_DIV_8, // 1/8
        .integration_time = VEML7700_INTEGRATION_TIME_100MS,
        .persistence_protect = VEML7700_PERSISTENCE_PROTECTION_1,
        .interrupt_enable = 0,
        .shutdown = 0, // 0 = wake/run, 1 = shutdown
        .threshold_high = 0xFFFF,
        .threshold_low = 0,
        .power_saving_mode = VEML7700_POWER_SAVING_MODE_500MS,
        .power_saving_enable = 0,
    };

    ret = applyConfigRange();
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on applyConfigRange: %s", esp_err_to_name(ret));

    initialized_ = true;
    ESP_LOGI(tag, "Sensor initialized: %s.", getModelName());

    return ESP_OK;

cleanup:
    veml7700_free_desc(&sensor_); // desc was created above, so release it on failure
    return ret;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Veml7700::read(SensorReadings& readings, SensorReadings& /*compensations*/) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    at<SensorType::ILLUMINANCE>(readings) = std::nullopt;

    for (size_t attempt = 0; attempt < ladder.size(); ++attempt) {
        uint16_t counts{};
        const auto ret = readAlsCounts(&sensor_, &counts);
        ESP_RETURN_ON_ERROR(ret, tag, "Could not read ALS counts %s", esp_err_to_name(ret));
        if (counts > max_counts && ladderIdx_ > 0) {
            ladderIdx_--;
            ESP_LOGD(
                tag, "Counts %d too high, decreasing gain to ladder index %d", counts, ladderIdx_);
            ESP_RETURN_ON_ERROR(applyConfigRange(), tag, "read failed trying to change range");
            continue;
        }
        if (counts <= min_counts && ladderIdx_ + 1U < ladder.size()) {
            ladderIdx_++;
            ESP_LOGD(
                tag, "Counts %d too low, increasing gain to ladder index %d", counts, ladderIdx_);
            ESP_RETURN_ON_ERROR(applyConfigRange(), tag, "read failed trying to change range");
            continue;
        }
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        const auto& step = ladder[ladderIdx_];
        float lux = static_cast<float>(counts) * step.res;
        if (isLowGain(step.gain) && lux > correction_min_lux) { lux = correctHighLux(lux); }
        at<SensorType::ILLUMINANCE>(readings) = lux;
        return ESP_OK;
    }

    ESP_LOGW(tag, "Light level did not settle within the gain ladder");
    return ESP_ERR_NOT_FINISHED;
}

esp_err_t Veml7700::applyConfigRange() {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    config_.gain = ladder[ladderIdx_].gain;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    config_.integration_time = ladder[ladderIdx_].it;
    const auto ret = veml7700_set_config(&sensor_, &config_);
    ESP_RETURN_ON_ERROR(ret, tag, "Could not apply config range: %s", esp_err_to_name(ret));
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    vTaskDelay(pdMS_TO_TICKS(ladder[ladderIdx_].it_ms + 120)); // one fresh conversion
    return ESP_OK;
}
