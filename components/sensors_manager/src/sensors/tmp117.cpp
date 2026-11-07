#include "tmp117.h"

#include <array>

#include <esp_check.h>

#include "i2c_bus.h"

constexpr const char* tag = "sensor_tmp117";

// ADD0 to GND (usual module default), V+, SDA, SCL
constexpr std::array<uint8_t, 4> i2c_addresses = {
    TMP117_I2C_ADDR_GND, TMP117_I2C_ADDR_VCC, TMP117_I2C_ADDR_SDA, TMP117_I2C_ADDR_SCL};

Tmp117::~Tmp117() {
    if (initialized_) { tmp117_free_desc(&sensor_); }
}

namespace {
// Temperature (°C)
constexpr std::array type_ranges = {
    SensorTypeRange{.type = SensorType::TEMPERATURE,
                    .range = {.min = -55.0F, .max = 150.0F, .abs_threshold = 0.05F}},
};
} // namespace

std::span<const SensorTypeRange> Tmp117::getTypeRanges() const {
    return type_ranges;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Tmp117::init(int64_t /*read_interval_ms*/) {
    // idempotent, avoids leaking on a second call
    ESP_RETURN_ON_FALSE(!initialized_, ESP_OK, tag, "The sensor is already initialized");

    ESP_LOGI(tag, "Initializing sensor...");

    // Safe to call from every I2C driver: i2cdev_init() is a no-op after the first call.
    auto ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, tag, "I2C initialization failed: %s", esp_err_to_name(ret));

    // Address depends on the module's ADD0 pin: try the usual default first.
    for (const uint8_t address : i2c_addresses) {
        ret = initAt(address);
        if (ret == ESP_OK) { break; }
    }
    ESP_RETURN_ON_ERROR(ret, tag, "No TMP117 / TMP119 found at 0x48..0x4b");

    initialized_ = true;
    ESP_LOGI(tag,
             "Sensor initialized: %s at 0x%02x (device id: 0x%04x).",
             getModelName(),
             sensor_.i2c_dev.addr,
             static_cast<unsigned int>(sensor_.device_id));

    return ESP_OK;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Tmp117::initAt(uint8_t address) {
    auto ret = tmp117_init_desc(&sensor_, address, i2c_port, sda_gpio, scl_gpio);
    ESP_RETURN_ON_ERROR(ret, tag, "initialization failed on init_desc: %s", esp_err_to_name(ret));

    ret = tmp117_init(&sensor_);
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on tmp117_init: %s", esp_err_to_name(ret));
    return ESP_OK;

cleanup:
    tmp117_free_desc(&sensor_); // desc was created above, so release it on failure
    return ret;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Tmp117::read(SensorReadings& readings, const SensorReadings& /*compensations*/) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    at<SensorType::TEMPERATURE>(readings) = std::nullopt;

    float temperature = 0.0F;
    const auto ret = tmp117_measure(&sensor_, &temperature);
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read sensor values: %s", esp_err_to_name(ret));

    at<SensorType::TEMPERATURE>(readings) = temperature;
    return ESP_OK;
}
