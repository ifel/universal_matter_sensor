/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file ens160.c
 *
 * ESP-IDF driver for ScioSense ENS160 digital metal-oxide multi-gas sensor.
 *
 * Based on k0i05/esp_ens160 1.2.8, ported to the esp-idf-lib i2cdev bus layer.
 */
#include "ens160.h"

#include <math.h>

#include <esp_check.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define ENS160_REG_PART_ID_R 0x00       //!< part identifier (0x0160)
#define ENS160_REG_OPMODE_RW 0x10       //!< operating mode
#define ENS160_REG_INT_CONFIG_RW 0x11   //!< interrupt pin configuration
#define ENS160_REG_COMMAND_RW 0x12      //!< additional system commands
#define ENS160_REG_TEMP_IN_RW 0x13      //!< host ambient temperature
#define ENS160_REG_RH_IN_RW 0x15        //!< host relative humidity
#define ENS160_REG_DEVICE_STATUS_R 0x20 //!< operating status
#define ENS160_REG_DATA_AQI_R 0x21      //!< air quality index
#define ENS160_REG_DATA_TVOC_R 0x22     //!< TVOC concentration (ppb)
#define ENS160_REG_DATA_ETOH_R 0x22     //!< ethanol concentration (ppb), same register as TVOC
#define ENS160_REG_DATA_ECO2_R 0x24     //!< equivalent CO2 concentration (ppm)
#define ENS160_REG_DATA_BL_R 0x28       //!< baseline information
#define ENS160_REG_GPR_READ0_R 0x48     //!< general purpose read0 register

#define ENS160_STATUS_NEW_GPR BIT(0)
#define ENS160_STATUS_NEW_DATA BIT(1)
#define ENS160_STATUS_VALIDITY_SHIFT 2
#define ENS160_STATUS_VALIDITY_MASK 0x03
#define ENS160_STATUS_ERROR BIT(6)
#define ENS160_STATUS_MODE BIT(7)

#define ENS160_AQI_MASK 0x07

#define ENS160_TEMPERATURE_MAX (125.0f)
#define ENS160_TEMPERATURE_MIN (-40.0f)
#define ENS160_HUMIDITY_MAX (100.0f)
#define ENS160_HUMIDITY_MIN (0.0f)

#define ENS160_POWERUP_DELAY_MS 15
#define ENS160_APPSTART_DELAY_MS 25
#define ENS160_CMD_DELAY_MS 5
#define ENS160_MODE_DELAY_MS 10
#define ENS160_RESET_DELAY_MS 50
#define ENS160_CLEAR_GPR_DELAY_MS 10
#define ENS160_DATA_READY_DELAY_MS 1
#define ENS160_DATA_POLL_TIMEOUT_MS 1500

#define I2C_FREQ_HZ 100000

#define CHECK_ARG(VAL)                                                                             \
    do {                                                                                           \
        if (!(VAL)) return ESP_ERR_INVALID_ARG;                                                    \
    } while (0)

static const char *const TAG = "ens160";

// At least ms: pdMS_TO_TICKS rounds down (to 0 below one tick at 100 Hz), so add a tick.
static inline void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms) + 1);
}

static esp_err_t read_reg(ens160_t *dev, uint8_t reg, uint8_t *buf, size_t size) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, buf, size));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

static esp_err_t write_reg(ens160_t *dev, uint8_t reg, const uint8_t *buf, size_t size) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg, buf, size));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

static inline esp_err_t read_byte(ens160_t *dev, uint8_t reg, uint8_t *byte) {
    return read_reg(dev, reg, byte, 1);
}

static inline esp_err_t write_byte(ens160_t *dev, uint8_t reg, uint8_t byte) {
    return write_reg(dev, reg, &byte, 1);
}

// Registers are little-endian.
static esp_err_t read_word(ens160_t *dev, uint8_t reg, uint16_t *word) {
    uint8_t rx[2] = {0};
    ESP_RETURN_ON_ERROR(read_reg(dev, reg, rx, sizeof(rx)), TAG, "read word 0x%02x failed", reg);
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
    return ESP_OK;
}

static esp_err_t write_word(ens160_t *dev, uint8_t reg, uint16_t word) {
    const uint8_t tx[2] = {(uint8_t)(word & 0xff), (uint8_t)((word >> 8) & 0xff)};
    return write_reg(dev, reg, tx, sizeof(tx));
}

static inline float decode_temperature(uint16_t encoded) {
    return ((float)encoded / 64.0f) - 273.15f;
}

static inline uint16_t encode_temperature(float temperature) {
    return (uint16_t)((temperature + 273.15f) * 64.0f);
}

static inline float decode_humidity(uint16_t encoded) {
    return (float)encoded / 512.0f;
}

static inline uint16_t encode_humidity(float humidity) {
    return (uint16_t)(humidity * 512.0f);
}

static inline uint32_t convert_rs_raw_to_ohms(uint16_t raw) {
    return (uint32_t)pow(2, (float)raw / 2048);
}

static esp_err_t get_status_register(ens160_t *dev, uint8_t *status) {
    ESP_RETURN_ON_ERROR(read_byte(dev, ENS160_REG_DEVICE_STATUS_R, status), TAG,
                        "read device status register failed");
    return ESP_OK;
}

static esp_err_t set_mode(ens160_t *dev, ens160_operating_modes_t mode) {
    ESP_RETURN_ON_ERROR(write_byte(dev, ENS160_REG_OPMODE_RW, mode), TAG,
                        "write operating mode 0x%02x failed", mode);
    delay_ms(ENS160_MODE_DELAY_MS);
    return ESP_OK;
}

static esp_err_t set_command(ens160_t *dev, ens160_commands_t command) {
    ESP_RETURN_ON_ERROR(write_byte(dev, ENS160_REG_COMMAND_RW, command), TAG,
                        "write command 0x%02x failed", command);
    return ESP_OK;
}

static esp_err_t clear_general_purpose_registers(ens160_t *dev) {
    ESP_RETURN_ON_ERROR(set_command(dev, ENS160_CMD_NORMAL), TAG, "normal command failed");
    ESP_RETURN_ON_ERROR(set_command(dev, ENS160_CMD_CLEAR_GPR), TAG, "clear gpr command failed");
    delay_ms(ENS160_CLEAR_GPR_DELAY_MS);
    ESP_RETURN_ON_ERROR(set_command(dev, ENS160_CMD_NORMAL), TAG, "normal command failed");
    return ESP_OK;
}

static esp_err_t soft_reset(ens160_t *dev) {
    ESP_RETURN_ON_ERROR(set_mode(dev, ENS160_OPMODE_RESET), TAG, "soft-reset failed");
    delay_ms(ENS160_RESET_DELAY_MS);
    return ESP_OK;
}

static esp_err_t setup_registers(ens160_t *dev) {
    ESP_RETURN_ON_ERROR(read_word(dev, ENS160_REG_PART_ID_R, &dev->part_id), TAG,
                        "read part id failed");
    ESP_RETURN_ON_FALSE(dev->part_id == ENS160_PART_ID, ESP_ERR_NOT_FOUND, TAG,
                        "unexpected part id 0x%04x", dev->part_id);

    // Configuration registers are writable only in idle mode.
    ESP_RETURN_ON_ERROR(set_mode(dev, ENS160_OPMODE_IDLE), TAG, "enable idle mode failed");
    ESP_RETURN_ON_ERROR(clear_general_purpose_registers(dev), TAG, "clear gpr failed");

    // Interrupt pin disabled, open drain, active low.
    ESP_RETURN_ON_ERROR(write_byte(dev, ENS160_REG_INT_CONFIG_RW, 0x00), TAG,
                        "write interrupt configuration failed");

    ESP_RETURN_ON_ERROR(set_mode(dev, ENS160_OPMODE_STANDARD), TAG, "enable standard mode failed");
    return ESP_OK;
}

esp_err_t ens160_init_desc(ens160_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio,
                           gpio_num_t scl_gpio) {
    CHECK_ARG(dev);
    CHECK_ARG(addr == ENS160_I2C_ADDR_LO || addr == ENS160_I2C_ADDR_HI);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t ens160_free_desc(ens160_t *dev) {
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t ens160_init(ens160_t *dev) {
    CHECK_ARG(dev);

    delay_ms(ENS160_POWERUP_DELAY_MS);

    // Identify before the reset writes to the bus: 0x52/0x53 may hold another device.
    ESP_RETURN_ON_ERROR(read_word(dev, ENS160_REG_PART_ID_R, &dev->part_id), TAG,
                        "read part id failed");
    ESP_RETURN_ON_FALSE(dev->part_id == ENS160_PART_ID, ESP_ERR_NOT_FOUND, TAG,
                        "unexpected part id 0x%04x", dev->part_id);

    ESP_RETURN_ON_ERROR(soft_reset(dev), TAG, "soft-reset for init failed");
    ESP_RETURN_ON_ERROR(setup_registers(dev), TAG, "setup registers for init failed");

    delay_ms(ENS160_APPSTART_DELAY_MS);
    return ESP_OK;
}

static esp_err_t wait_for_status_bit(ens160_t *dev, uint8_t bit) {
    const int64_t start_time = esp_timer_get_time();
    uint8_t status = 0;
    do {
        ESP_RETURN_ON_ERROR(get_status_register(dev, &status), TAG, "poll status failed");
        if (status & bit) return ESP_OK;
        delay_ms(ENS160_DATA_READY_DELAY_MS);
    } while ((esp_timer_get_time() - start_time) < (ENS160_DATA_POLL_TIMEOUT_MS * 1000LL));
    return ESP_ERR_TIMEOUT;
}

esp_err_t ens160_get_measurement(ens160_t *dev, ens160_air_quality_data_t *data) {
    CHECK_ARG(dev && data);

    ESP_RETURN_ON_ERROR(wait_for_status_bit(dev, ENS160_STATUS_NEW_DATA), TAG,
                        "waiting for new data failed");

    uint8_t aqi = 0;
    uint16_t tvoc = 0;
    uint16_t etoh = 0;
    uint16_t eco2 = 0;
    ESP_RETURN_ON_ERROR(read_byte(dev, ENS160_REG_DATA_AQI_R, &aqi), TAG, "read aqi failed");
    ESP_RETURN_ON_ERROR(read_word(dev, ENS160_REG_DATA_TVOC_R, &tvoc), TAG, "read tvoc failed");
    ESP_RETURN_ON_ERROR(read_word(dev, ENS160_REG_DATA_ETOH_R, &etoh), TAG, "read etoh failed");
    ESP_RETURN_ON_ERROR(read_word(dev, ENS160_REG_DATA_ECO2_R, &eco2), TAG, "read eco2 failed");

    aqi &= ENS160_AQI_MASK;
    data->uba_aqi = (aqi >= ENS160_AQI_MIN && aqi <= ENS160_AQI_MAX)
                        ? (ens160_aqi_uba_indexes_t)aqi
                        : ENS160_AQI_UBA_INDEX_UNKNOWN;
    data->tvoc = tvoc;
    data->etoh = etoh;
    data->eco2 = eco2;

    delay_ms(ENS160_CMD_DELAY_MS);
    return ESP_OK;
}

esp_err_t ens160_get_raw_measurement(ens160_t *dev, ens160_air_quality_raw_data_t *data) {
    CHECK_ARG(dev && data);

    ESP_RETURN_ON_ERROR(wait_for_status_bit(dev, ENS160_STATUS_NEW_GPR), TAG,
                        "waiting for new gpr data failed");

    uint8_t rx[8] = {0};
    ESP_RETURN_ON_ERROR(read_reg(dev, ENS160_REG_GPR_READ0_R, rx, sizeof(rx)), TAG,
                        "read resistance gpr registers failed");
    data->hp0_ri = convert_rs_raw_to_ohms(rx[0] | ((uint16_t)rx[1] << 8));
    data->hp1_ri = convert_rs_raw_to_ohms(rx[2] | ((uint16_t)rx[3] << 8));
    data->hp2_ri = convert_rs_raw_to_ohms(rx[4] | ((uint16_t)rx[5] << 8));
    data->hp3_ri = convert_rs_raw_to_ohms(rx[6] | ((uint16_t)rx[7] << 8));

    delay_ms(ENS160_CMD_DELAY_MS);

    ESP_RETURN_ON_ERROR(read_reg(dev, ENS160_REG_DATA_BL_R, rx, sizeof(rx)), TAG,
                        "read baseline registers failed");
    data->hp0_bl = convert_rs_raw_to_ohms(rx[0] | ((uint16_t)rx[1] << 8));
    data->hp1_bl = convert_rs_raw_to_ohms(rx[2] | ((uint16_t)rx[3] << 8));
    data->hp2_bl = convert_rs_raw_to_ohms(rx[4] | ((uint16_t)rx[5] << 8));
    data->hp3_bl = convert_rs_raw_to_ohms(rx[6] | ((uint16_t)rx[7] << 8));

    delay_ms(ENS160_CMD_DELAY_MS);
    return ESP_OK;
}

esp_err_t ens160_get_status(ens160_t *dev, bool *data_ready, bool *gpr_data_ready,
                            ens160_validity_flags_t *state, bool *error, bool *mode) {
    CHECK_ARG(dev);

    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(get_status_register(dev, &status), TAG, "get status failed");

    if (data_ready) *data_ready = status & ENS160_STATUS_NEW_DATA;
    if (gpr_data_ready) *gpr_data_ready = status & ENS160_STATUS_NEW_GPR;
    if (state)
        *state = (ens160_validity_flags_t)((status >> ENS160_STATUS_VALIDITY_SHIFT) &
                                           ENS160_STATUS_VALIDITY_MASK);
    if (error) *error = status & ENS160_STATUS_ERROR;
    if (mode) *mode = status & ENS160_STATUS_MODE;
    return ESP_OK;
}

esp_err_t ens160_get_data_status(ens160_t *dev, bool *ready) {
    CHECK_ARG(ready);
    return ens160_get_status(dev, ready, NULL, NULL, NULL, NULL);
}

esp_err_t ens160_get_gpr_data_status(ens160_t *dev, bool *ready) {
    CHECK_ARG(ready);
    return ens160_get_status(dev, NULL, ready, NULL, NULL, NULL);
}

esp_err_t ens160_get_validity_status(ens160_t *dev, ens160_validity_flags_t *state) {
    CHECK_ARG(state);
    return ens160_get_status(dev, NULL, NULL, state, NULL, NULL);
}

esp_err_t ens160_get_error_status(ens160_t *dev, bool *error) {
    CHECK_ARG(error);
    return ens160_get_status(dev, NULL, NULL, NULL, error, NULL);
}

esp_err_t ens160_get_mode_status(ens160_t *dev, bool *mode) {
    CHECK_ARG(mode);
    return ens160_get_status(dev, NULL, NULL, NULL, NULL, mode);
}

esp_err_t ens160_get_compensation_factors(ens160_t *dev, float *temperature, float *humidity) {
    CHECK_ARG(dev && temperature && humidity);

    uint16_t t = 0;
    uint16_t h = 0;
    ESP_RETURN_ON_ERROR(read_word(dev, ENS160_REG_TEMP_IN_RW, &t), TAG,
                        "read temperature compensation failed");
    ESP_RETURN_ON_ERROR(read_word(dev, ENS160_REG_RH_IN_RW, &h), TAG,
                        "read humidity compensation failed");

    *temperature = decode_temperature(t);
    *humidity = decode_humidity(h);
    return ESP_OK;
}

esp_err_t ens160_set_compensation_factors(ens160_t *dev, float temperature, float humidity) {
    CHECK_ARG(dev);
    ESP_RETURN_ON_FALSE(temperature >= ENS160_TEMPERATURE_MIN &&
                            temperature <= ENS160_TEMPERATURE_MAX,
                        ESP_ERR_INVALID_ARG, TAG, "temperature compensation out of range");
    ESP_RETURN_ON_FALSE(humidity >= ENS160_HUMIDITY_MIN && humidity <= ENS160_HUMIDITY_MAX,
                        ESP_ERR_INVALID_ARG, TAG, "humidity compensation out of range");

    ESP_RETURN_ON_ERROR(write_word(dev, ENS160_REG_TEMP_IN_RW, encode_temperature(temperature)),
                        TAG, "write temperature compensation failed");
    ESP_RETURN_ON_ERROR(write_word(dev, ENS160_REG_RH_IN_RW, encode_humidity(humidity)), TAG,
                        "write humidity compensation failed");
    return ESP_OK;
}

esp_err_t ens160_enable_standard_mode(ens160_t *dev) {
    CHECK_ARG(dev);
    return set_mode(dev, ENS160_OPMODE_STANDARD);
}

esp_err_t ens160_enable_idle_mode(ens160_t *dev) {
    CHECK_ARG(dev);
    return set_mode(dev, ENS160_OPMODE_IDLE);
}

esp_err_t ens160_enable_deep_sleep_mode(ens160_t *dev) {
    CHECK_ARG(dev);
    return set_mode(dev, ENS160_OPMODE_DEEP_SLEEP);
}

esp_err_t ens160_reset(ens160_t *dev) {
    CHECK_ARG(dev);

    ESP_RETURN_ON_ERROR(soft_reset(dev), TAG, "soft-reset failed");
    ESP_RETURN_ON_ERROR(setup_registers(dev), TAG, "setup registers for soft-reset failed");
    return ESP_OK;
}
