#include "tmp117.h"

#include <esp_check.h>
#include <esp_idf_lib_helpers.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* const TAG = "tmp117";

#define I2C_FREQ_HZ 400000

#define REG_TEMP_RESULT 0x00
#define REG_CONFIG      0x01
#define REG_DEVICE_ID   0x0F

#define DEVICE_ID_MASK 0x0FFF // bits 15:12 are the revision

#define CFG_DATA_READY   (0x1 << 13)
#define CFG_MOD_SHUTDOWN (0x1 << 10)
#define CFG_MOD_ONE_SHOT (0x3 << 10)
#define CFG_AVG_8        (0x1 << 5)
#define CFG_SOFT_RESET   (0x1 << 1)

#define TEMP_LSB_CELSIUS 0.0078125f

#define RESET_TIME_MS         2
#define CONVERSION_TIME_MS    125 // one-shot, 8 averages
#define POLL_INTERVAL_MS      10
#define CONVERSION_TIMEOUT_MS 300

#define CHECK_ARG(VAL)                                                                             \
    do {                                                                                           \
        if (!(VAL)) return ESP_ERR_INVALID_ARG;                                                    \
    } while (0)

// +1 tick so short waits never round down to zero.
static inline void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms) + 1);
}

// Registers are 16-bit big-endian.
static esp_err_t read_word(tmp117_t* dev, uint8_t reg, uint16_t* word) {
    uint8_t rx[2] = {0};
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg, rx, sizeof(rx)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *word = ((uint16_t)rx[0] << 8) | rx[1];
    return ESP_OK;
}

static esp_err_t write_word(tmp117_t* dev, uint8_t reg, uint16_t word) {
    const uint8_t tx[2] = {(uint8_t)(word >> 8), (uint8_t)(word & 0xff)};
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg, tx, sizeof(tx)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t tmp117_init_desc(
    tmp117_t* dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio) {
    CHECK_ARG(dev);
    CHECK_ARG(addr >= TMP117_I2C_ADDR_GND && addr <= TMP117_I2C_ADDR_SCL);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t tmp117_free_desc(tmp117_t* dev) {
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t tmp117_init(tmp117_t* dev) {
    CHECK_ARG(dev);

    ESP_RETURN_ON_ERROR(
        read_word(dev, REG_DEVICE_ID, &dev->device_id), TAG, "read device id failed");
    ESP_RETURN_ON_FALSE((dev->device_id & DEVICE_ID_MASK) == TMP117_DEVICE_ID,
                        ESP_ERR_NOT_FOUND,
                        TAG,
                        "unexpected device id 0x%04x",
                        dev->device_id);

    // Reset reloads the EEPROM defaults (continuous mode); then stay idle between one-shots.
    ESP_RETURN_ON_ERROR(write_word(dev, REG_CONFIG, CFG_SOFT_RESET), TAG, "soft reset failed");
    delay_ms(RESET_TIME_MS);
    ESP_RETURN_ON_ERROR(
        write_word(dev, REG_CONFIG, CFG_MOD_SHUTDOWN | CFG_AVG_8), TAG, "set shutdown mode failed");
    return ESP_OK;
}

esp_err_t tmp117_measure(tmp117_t* dev, float* temperature) {
    CHECK_ARG(dev && temperature);

    ESP_RETURN_ON_ERROR(
        write_word(dev, REG_CONFIG, CFG_MOD_ONE_SHOT | CFG_AVG_8), TAG, "start one-shot failed");
    delay_ms(CONVERSION_TIME_MS);

    // Reading the configuration register clears Data_Ready, so read the result right after.
    uint16_t config = 0;
    uint32_t waited_ms = CONVERSION_TIME_MS;
    for (;;) {
        ESP_RETURN_ON_ERROR(read_word(dev, REG_CONFIG, &config), TAG, "read config failed");
        if (config & CFG_DATA_READY) break;
        ESP_RETURN_ON_FALSE(
            waited_ms < CONVERSION_TIMEOUT_MS, ESP_ERR_TIMEOUT, TAG, "conversion timed out");
        delay_ms(POLL_INTERVAL_MS);
        waited_ms += POLL_INTERVAL_MS;
    }

    uint16_t raw = 0;
    ESP_RETURN_ON_ERROR(read_word(dev, REG_TEMP_RESULT, &raw), TAG, "read temperature failed");
    *temperature = (float)(int16_t)raw * TEMP_LSB_CELSIUS;
    return ESP_OK;
}
