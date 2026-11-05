#include "bmp581.h"

#include <esp_check.h>

constexpr const char* tag = "   ***Sensor BMP581***   ";

// BMP581 Sensor Configuration (from Kconfig)
constexpr auto sda_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SDA_GPIO);
constexpr auto scl_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SCL_GPIO);
constexpr auto i2c_port = static_cast<i2c_port_t>(CONFIG_SENSORS_I2C_PORT);
constexpr auto i2c_address = static_cast<uint8_t>(CONFIG_BMP581_I2C_ADDRESS);

Bmp581::~Bmp581() {
    if (initialized_) {
        bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &sensor_.dev);
        bmp5_i2c_free_desc(&sensor_);
    }
}

std::optional<SensorValueRange> Bmp581::getValueRangeForType(SensorType type) const {
    // Temperature (°C)
    constexpr float temp_min_deg_celsius = -40.0F;
    constexpr float temp_max_deg_celsius = 85.0F;
    constexpr float temp_abs_threshold = 0.10F;

    // Pressure (Pa)
    constexpr float press_min_pa = 30000.0F;
    constexpr float press_max_pa = 125000.0F;
    constexpr float press_abs_threshold = 50.0F;

    switch (type) {
    case SensorType::TEMPERATURE:
        return SensorValueRange{.min = temp_min_deg_celsius,
                                .max = temp_max_deg_celsius,
                                .abs_threshold = temp_abs_threshold};
    case SensorType::PRESSURE:
        return SensorValueRange{
            .min = press_min_pa, .max = press_max_pa, .abs_threshold = press_abs_threshold};
    case SensorType::RELATIVE_HUMIDITY:
    case SensorType::CARBON_DIOXIDE_CONCENTRATION:
    case SensorType::ILLUMINANCE:
    case SensorType::TVOC_CONCENTRATION:
    case SensorType::AIR_QUALITY:
        return std::nullopt;
    }
    return std::nullopt;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Bmp581::init(int64_t /*read_interval_ms*/) {
    // idempotent, avoids leaking on a second call
    ESP_RETURN_ON_FALSE(!initialized_, ESP_OK, tag, "The sensor is already initialized");

    ESP_LOGI(tag, "Initializing sensor...");

    // Safe to call from every I2C driver: i2cdev_init() is a no-op after the first call.
    auto ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, tag, "I2C initialization failed: %s", esp_err_to_name(ret));

    ret = bmp5_i2c_init_desc(&sensor_, i2c_address, i2c_port, sda_gpio, scl_gpio);
    ESP_RETURN_ON_ERROR(ret, tag, "initialization failed on init_desc: %s", esp_err_to_name(ret));

    ret = bmp5_to_esp_err(bmp5_init(&sensor_.dev));
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on bmp5_init: %s", esp_err_to_name(ret));

    ret = configure();
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on configure: %s", esp_err_to_name(ret));

    initialized_ = true;
    ESP_LOGI(tag,
             "Sensor initialized: %s (chip ID: 0x%02x).",
             getModelName(),
             static_cast<unsigned int>(sensor_.dev.chip_id));

    return ESP_OK;

cleanup:
    bmp5_i2c_free_desc(&sensor_); // desc was created above, so release it on failure
    return ret;
}

// Normal mode at 1 Hz: the latest result is always ready when read() is called.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Bmp581::configure() {
    auto* dev = &sensor_.dev;

    // OSR/ODR/IIR registers can only be written in standby.
    auto ret = bmp5_to_esp_err(bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev));
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to enter standby: %s", esp_err_to_name(ret));

    ret = bmp5_to_esp_err(bmp5_get_osr_odr_press_config(&osr_odr_press_cfg_, dev));
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read OSR/ODR config: %s", esp_err_to_name(ret));
    osr_odr_press_cfg_.odr = BMP5_ODR_01_HZ;
    osr_odr_press_cfg_.press_en = BMP5_ENABLE;
    osr_odr_press_cfg_.osr_t = BMP5_OVERSAMPLING_2X;
    osr_odr_press_cfg_.osr_p = BMP5_OVERSAMPLING_16X;
    ret = bmp5_to_esp_err(bmp5_set_osr_odr_press_config(&osr_odr_press_cfg_, dev));
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to set OSR/ODR config: %s", esp_err_to_name(ret));

    bmp5_iir_config iir_cfg{};
    iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_3;
    iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_3;
    iir_cfg.shdw_set_iir_t = BMP5_ENABLE;
    iir_cfg.shdw_set_iir_p = BMP5_ENABLE;
    ret = bmp5_to_esp_err(bmp5_set_iir_config(&iir_cfg, dev));
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to set IIR config: %s", esp_err_to_name(ret));

    bmp5_osr_odr_eff osr_odr_eff{};
    ret = bmp5_to_esp_err(bmp5_get_osr_odr_eff(&osr_odr_eff, dev));
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read effective OSR/ODR: %s", esp_err_to_name(ret));
    ESP_RETURN_ON_FALSE(osr_odr_eff.odr_is_valid == BMP5_ENABLE,
                        ESP_ERR_INVALID_STATE,
                        tag,
                        "OSR/ODR combination is not valid");

    ret = bmp5_to_esp_err(bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, dev));
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to enter normal mode: %s", esp_err_to_name(ret));
    return ESP_OK;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Bmp581::read(SensorReadings& readings, SensorReadings& /*compensations*/) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    at<SensorType::TEMPERATURE>(readings) = std::nullopt;
    at<SensorType::PRESSURE>(readings) = std::nullopt;

    bmp5_sensor_data data{};
    const auto ret =
        bmp5_to_esp_err(bmp5_get_sensor_data(&data, &osr_odr_press_cfg_, &sensor_.dev));
    ESP_RETURN_ON_ERROR(ret, tag, "Failed to read sensor values: %s", esp_err_to_name(ret));
    // Result registers stay zero until the first conversion completes.
    ESP_RETURN_ON_FALSE(
        data.pressure > 0.0F, ESP_ERR_NOT_FINISHED, tag, "Sensor data not ready yet");

    at<SensorType::TEMPERATURE>(readings) = data.temperature;
    at<SensorType::PRESSURE>(readings) = data.pressure;
    return ESP_OK;
}
