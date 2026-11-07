#include "max17048.h"

#include <algorithm>
#include <cmath>

#include <esp_check.h>

#include "i2c_bus.h"

constexpr const char* tag = "sensor_max17048";

namespace {
// VERSION register: 0x001x on MAX17048/49 (the driver does not check it).
constexpr uint16_t version_mask = 0xFFF0;
constexpr uint16_t max17048_version = 0x0010;

// Charge rate (%/h) above which the battery counts as charging, and below which as discharging;
// MAX17048 LSB is 0.208 %/h.
constexpr float charging_rate_threshold = 1.0F;
constexpr float discharging_rate_threshold = -1.0F;
constexpr float full_charge_percent = 99.0F;

// FULL needs a cell that is not discharging: on battery near 100 % the rate is negative.
BatteryChargeState toChargeState(float soc_percent, float rate_percent_per_hour) {
    if (rate_percent_per_hour > charging_rate_threshold) { return BatteryChargeState::CHARGING; }
    if (rate_percent_per_hour < discharging_rate_threshold) {
        return BatteryChargeState::NOT_CHARGING;
    }
    if (soc_percent >= full_charge_percent) { return BatteryChargeState::FULL; }
    return BatteryChargeState::NOT_CHARGING;
}

// ModelGauge temperature compensation (datasheet): RCOMP = RCOMP0 + (T - 20) * TempCo, with the
// generic model's RCOMP0 = 0x97, TempCoUp = -0.5 above 20 °C and TempCoDown = -5.0 below.
constexpr float rcomp0 = 0x97;
constexpr float rcomp_ref_celsius = 20.0F;
constexpr float temp_co_up = -0.5F;
constexpr float temp_co_down = -5.0F;

uint8_t rcompForTemperature(float temperature) {
    const float delta = temperature - rcomp_ref_celsius;
    const float rcomp = rcomp0 + (delta * (delta > 0.0F ? temp_co_up : temp_co_down));
    // 0 is excluded: max1704x_set_config treats it as "keep" and writes an uninitialized byte.
    return static_cast<uint8_t>(std::clamp(std::lround(rcomp), 1L, 255L));
}
} // namespace

Max17048::~Max17048() {
    if (initialized_) { max1704x_free_desc(&sensor_); }
}

namespace {
// Battery (%, Matter reports half-percent steps), voltage (V, VCELL full scale 5.12 V), charge
// state (BatteryChargeState)
constexpr std::array type_ranges = {
    SensorTypeRange{.type = SensorType::BATTERY_PERCENT,
                    .range = {.min = 0.0F, .max = 100.0F, .abs_threshold = 1.0F}},
    SensorTypeRange{.type = SensorType::BATTERY_VOLTAGE,
                    .range = {.min = 0.0F, .max = 5.12F, .abs_threshold = 0.01F}},
    SensorTypeRange{.type = SensorType::BATTERY_CHARGE_STATE,
                    .range = {.min = 0.0F,
                              .max = static_cast<float>(BatteryChargeState::NOT_CHARGING),
                              .abs_threshold = 1.0F}},
};
} // namespace

std::span<const SensorTypeRange> Max17048::getTypeRanges() const {
    return type_ranges;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Max17048::init(int64_t /*read_interval_ms*/) {
    // idempotent, avoids leaking on a second call
    ESP_RETURN_ON_FALSE(!initialized_, ESP_OK, tag, "The sensor is already initialized");

    ESP_LOGI(tag, "Initializing sensor...");

    // Safe to call from every I2C driver: i2cdev_init() is a no-op after the first call.
    auto ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, tag, "I2C initialization failed: %s", esp_err_to_name(ret));

    ret = max1704x_init_desc(&sensor_, i2c_port, sda_gpio, scl_gpio);
    ESP_RETURN_ON_ERROR(ret, tag, "initialization failed on init_desc: %s", esp_err_to_name(ret));
    sensor_.model = MAX17048_9; // selects VCELL scaling and enables CRATE

    uint16_t version = 0;
    ret = max1704x_get_version(&sensor_, &version);
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on reading version: %s", esp_err_to_name(ret));
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_FALSE((version & version_mask) == max17048_version,
                      ESP_ERR_NOT_FOUND,
                      cleanup,
                      tag,
                      "unexpected version 0x%04x at 0x36 (not a MAX17048/49)",
                      version);

    initialized_ = true;
    ESP_LOGI(tag, "Sensor initialized: %s (version: 0x%04x).", getModelName(), version);

    return ESP_OK;

cleanup:
    max1704x_free_desc(&sensor_); // desc was created above, so release it on failure
    return ret;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Max17048::read(SensorReadings& readings, const SensorReadings& compensations) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    at<SensorType::BATTERY_PERCENT>(readings) = std::nullopt;
    at<SensorType::BATTERY_VOLTAGE>(readings) = std::nullopt;
    at<SensorType::BATTERY_CHARGE_STATE>(readings) = std::nullopt;

    // Not fatal: the gauge keeps its previous compensation.
    if (const auto ret = updateRcomp(compensations); ret != ESP_OK) {
        ESP_LOGW(tag, "Failed to update RCOMP: %s", esp_err_to_name(ret));
    }

    float voltage = 0.0F;
    auto ret = max1704x_get_voltage(&sensor_, &voltage);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read voltage: %s", esp_err_to_name(ret));
    at<SensorType::BATTERY_VOLTAGE>(readings) = voltage;

    float soc = 0.0F;
    ret = max1704x_get_soc(&sensor_, &soc);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read state of charge: %s", esp_err_to_name(ret));
    at<SensorType::BATTERY_PERCENT>(readings) = soc;

    float rate = 0.0F;
    ret = max1704x_get_crate(&sensor_, &rate);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read charge rate: %s", esp_err_to_name(ret));
    ESP_LOGD(tag, "Charge rate %.2f %%/h", rate);
    at<SensorType::BATTERY_CHARGE_STATE>(readings) = static_cast<float>(toChargeState(soc, rate));
    return ESP_OK;
}

// The ambient temperature stands in for the cell's (the gauge has no sensor of its own).
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Max17048::updateRcomp(const SensorReadings& compensations) {
    const auto& temperature = at<SensorType::TEMPERATURE>(compensations);
    if (!temperature.has_value() || !std::isfinite(*temperature)) { return ESP_OK; }
    const uint8_t rcomp = rcompForTemperature(*temperature);
    if (rcomp == rcomp_) { return ESP_OK; }

    // Read-modify-write: RCOMP shares the CONFIG register with the alert settings.
    ESP_RETURN_ON_ERROR(max1704x_get_config(&sensor_), tag, "Failed to read CONFIG");
    max1704x_config_t config = sensor_.config;
    config.rcomp = rcomp;
    ESP_RETURN_ON_ERROR(max1704x_set_config(&sensor_, &config), tag, "Failed to write CONFIG");
    ESP_LOGD(tag, "RCOMP 0x%02x for %.1f °C", rcomp, *temperature);
    rcomp_ = rcomp;
    return ESP_OK;
}
