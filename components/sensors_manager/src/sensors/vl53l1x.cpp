#include "vl53l1x.h"

#include <algorithm>
#include <array>
#include <cstdint>

#include <esp_check.h>
#include <i2cdev.h>
#include <nvs.h>

#include "i2c_bus.h"

constexpr const char* tag = "sensor_vl53l1x";

// VL53L1X Sensor Configuration (from Kconfig)
constexpr uint8_t i2c_address = 0x29; // power-on default, shared with TSL2591
constexpr auto level_height_mm = static_cast<float>(CONFIG_VL53L1X_LEVEL_HEIGHT_MM);
static_assert(CONFIG_VL53L1X_LEVEL_HEIGHT_MM > 0, "VL53L1X level height must be positive");

namespace {
constexpr uint16_t vl53l1x_chip_id = 0xEACC;

// Narrowest field of view (~15° instead of 27°): the beam must stay on the target when it is far.
constexpr uint8_t roi_width = 4;
constexpr uint16_t macro_timing = 100; // longer integration: better accuracy
constexpr uint32_t inter_measurement_ms = 100;
constexpr uint16_t signal_threshold_kcps =
    1000;                                   // below ST's 1500: a 4x4 ROI gets 1/16 of the signal
constexpr uint16_t sigma_threshold_mm = 45; // ST default

constexpr size_t samples_per_read = 5;
constexpr size_t max_attempts = 3 * samples_per_read;
constexpr uint32_t sample_timeout_ms = 500;

constexpr const char* nvs_namespace = "vl53l1x";
constexpr const char* nvs_full_key = "full_mm";

float toRemainingPercent(float full_distance_mm, float distance_mm) {
    const float remaining = (full_distance_mm + level_height_mm - distance_mm) / level_height_mm;
    return std::clamp(remaining * 100.0F, 0.0F, 100.0F);
}
} // namespace

Vl53l1x::~Vl53l1x() {
    if (initialized_) {
        vl53l1x_stop(&sensor_);
        vl53l1x_deinit(&sensor_);
    }
}

namespace {
// Remaining level (%), published as whole percent
constexpr std::array type_ranges = {
    SensorTypeRange{.type = SensorType::REMAINING_LEVEL,
                    .range = {.min = 0.0F, .max = 100.0F, .abs_threshold = 1.0F}},
};
} // namespace

std::span<const SensorTypeRange> Vl53l1x::getTypeRanges() const {
    return type_ranges;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Vl53l1x::init(int64_t /*read_interval_ms*/) {
    // idempotent, avoids leaking on a second call
    ESP_RETURN_ON_FALSE(!initialized_, ESP_OK, tag, "The sensor is already initialized");

    ESP_LOGI(tag, "Initializing sensor...");
    loadFullDistance();

    // Safe to call from every I2C driver: i2cdev_init() is a no-op after the first call.
    auto ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, tag, "I2C initialization failed: %s", esp_err_to_name(ret));

    // The driver uses the ESP-IDF i2c_master bus directly; borrow i2cdev's so the bus is shared.
    // Probing through i2cdev also creates that bus (with our pins) if no other sensor did yet.
    i2c_dev_t probe{};
    probe.port = i2c_port;
    probe.addr = i2c_address;
    probe.cfg.sda_io_num = sda_gpio;
    probe.cfg.scl_io_num = scl_gpio;
    ret = i2c_dev_check_present(&probe);
    ESP_RETURN_ON_ERROR(ret, tag, "No device at 0x%02x: %s", i2c_address, esp_err_to_name(ret));

    void* bus_handle = nullptr;
    ret = i2cdev_get_shared_handle(i2c_port, &bus_handle);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to get I2C bus handle: %s", esp_err_to_name(ret));

    ret = vl53l1x_init(&sensor_, static_cast<i2c_master_bus_handle_t>(bus_handle), i2c_address);
    ESP_RETURN_ON_ERROR(
        ret, tag, "initialization failed on vl53l1x_init: %s", esp_err_to_name(ret));

    ret = configure();
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on configure: %s", esp_err_to_name(ret));

    initialized_ = true;
    ESP_LOGI(tag,
             "Sensor initialized: %s (full at %u mm, empty at %u mm).",
             getModelName(),
             full_distance_mm_,
             full_distance_mm_ + CONFIG_VL53L1X_LEVEL_HEIGHT_MM);

    return ESP_OK;

cleanup:
    vl53l1x_deinit(&sensor_); // device was added to the bus above, so remove it on failure
    return ret;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Vl53l1x::configure() {
    // 0x29 is also the TSL2591's address: make sure this is a VL53L1X.
    uint16_t chip_id = 0;
    auto ret = vl53l1x_get_sensor_id(&sensor_, &chip_id);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read chip ID: %s", esp_err_to_name(ret));
    ESP_RETURN_ON_FALSE(chip_id == vl53l1x_chip_id,
                        ESP_ERR_NOT_FOUND,
                        tag,
                        "unexpected chip ID 0x%04x at 0x29 (not a VL53L1X)",
                        chip_id);

    ret = vl53l1x_sensor_init(&sensor_);
    ESP_RETURN_ON_ERROR(ret, tag, "Sensor init failed: %s", esp_err_to_name(ret));

    // The wrapper only exposes timing; ROI and thresholds go through the ST ULP API (8-bit
    // address).
    const auto dev = static_cast<uint16_t>(i2c_address << 1);
    uint8_t status = VL53L1X_ULP_SetROI(dev, roi_width);
    status |= VL53L1X_ULP_SetMacroTiming(dev, macro_timing);
    status |= VL53L1X_ULP_SetInterMeasurementInMs(dev, inter_measurement_ms);
    status |= VL53L1X_ULP_SetSignalThreshold(dev, signal_threshold_kcps);
    status |= VL53L1X_ULP_SetSigmaThreshold(dev, sigma_threshold_mm);
    ESP_RETURN_ON_FALSE(
        status == 0, ESP_FAIL, tag, "Failed to configure ranging (status %u)", status);

    ret = vl53l1x_start(&sensor_);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to start ranging: %s", esp_err_to_name(ret));
    return ESP_OK;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Vl53l1x::read(SensorReadings& readings, const SensorReadings& /*compensations*/) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    at<SensorType::REMAINING_LEVEL>(readings) = std::nullopt;

    // Median of several valid samples smooths ToF noise and rejects outliers.
    std::array<uint16_t, samples_per_read> distances{};
    size_t valid = 0;
    for (size_t attempt = 0; attempt < max_attempts && valid < samples_per_read; ++attempt) {
        vl53l1x_result_t result{};
        const auto ret = vl53l1x_read(&sensor_, &result, sample_timeout_ms);
        ESP_RETURN_ON_ERROR(ret, tag, "Failed to read measurement: %s", esp_err_to_name(ret));
        ESP_LOGD(tag,
                 "status %u, distance %u mm, sigma %u mm, signal %u kcps, ambient %u kcps",
                 result.status,
                 result.distance_mm,
                 result.sigma_mm,
                 result.signal_kcps,
                 result.ambient_kcps);
        if (result.status == 0) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            distances[valid++] = result.distance_mm;
        }
    }
    ESP_RETURN_ON_FALSE(valid == samples_per_read,
                        ESP_ERR_INVALID_RESPONSE,
                        tag,
                        "Only %u of %u samples valid (target out of range or weak signal)",
                        static_cast<unsigned int>(valid),
                        static_cast<unsigned int>(samples_per_read));

    std::ranges::sort(distances);
    const auto median_mm = distances[samples_per_read / 2];
    if (calibration_requested_.exchange(false)) {
        full_distance_mm_ = median_mm;
        saveFullDistance();
        ESP_LOGI(tag, "Full point set to %u mm", full_distance_mm_);
    }
    const auto distance_mm = static_cast<float>(median_mm);
    const float remaining = toRemainingPercent(static_cast<float>(full_distance_mm_), distance_mm);
    ESP_LOGI(tag, "Distance %.0f mm -> remaining %.1f %%", distance_mm, remaining);

    at<SensorType::REMAINING_LEVEL>(readings) = remaining;
    return ESP_OK;
}

void Vl53l1x::requestCalibration(SensorType type) {
    if (type == SensorType::REMAINING_LEVEL) { calibration_requested_ = true; }
}

void Vl53l1x::loadFullDistance() {
    nvs_handle_t handle = 0;
    if (nvs_open(nvs_namespace, NVS_READONLY, &handle) != ESP_OK) { return; } // never taught
    uint16_t value = 0;
    if (nvs_get_u16(handle, nvs_full_key, &value) == ESP_OK) {
        full_distance_mm_ = value;
        ESP_LOGI(tag, "Full point %u mm from NVS", value);
    }
    nvs_close(handle);
}

void Vl53l1x::saveFullDistance() const {
    nvs_handle_t handle = 0;
    auto ret = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
    if (ret == ESP_OK) {
        ret = nvs_set_u16(handle, nvs_full_key, full_distance_mm_);
        if (ret == ESP_OK) { ret = nvs_commit(handle); }
        nvs_close(handle);
    }
    if (ret != ESP_OK) { ESP_LOGE(tag, "Failed to save the full point: %s", esp_err_to_name(ret)); }
}
