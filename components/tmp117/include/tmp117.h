/**
 * @file tmp117.h
 *
 * ESP-IDF driver for TI TMP117 / TMP119 digital temperature sensors (same register map), on the
 * esp-idf-lib i2cdev bus layer.
 */
#ifndef TMP117_H
#define TMP117_H

#include <esp_err.h>
#include <i2cdev.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TMP117_I2C_ADDR_GND UINT8_C(0x48) //!< ADD0 to GND
#define TMP117_I2C_ADDR_VCC UINT8_C(0x49) //!< ADD0 to V+
#define TMP117_I2C_ADDR_SDA UINT8_C(0x4A) //!< ADD0 to SDA
#define TMP117_I2C_ADDR_SCL UINT8_C(0x4B) //!< ADD0 to SCL

#define TMP117_DEVICE_ID     UINT16_C(0x0117) //!< Device_ID[11:0], shared by TMP117 and TMP119
#define TMP119_DEVICE_ID_REV UINT16_C(0x2117) //!< full Device_ID register reported by TMP119

typedef struct {
    i2c_dev_t i2c_dev;
    uint16_t device_id; //!< full Device_ID register (revision in bits 15:12), set by tmp117_init()
} tmp117_t;

esp_err_t tmp117_init_desc(
    tmp117_t* dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
esp_err_t tmp117_free_desc(tmp117_t* dev);

/**
 * Checks the device ID (ESP_ERR_NOT_FOUND if not a TMP117/TMP119), soft-resets the chip and puts
 * it into shutdown mode. Nothing is written before the ID matches.
 */
esp_err_t tmp117_init(tmp117_t* dev);

/**
 * One-shot conversion with 8x averaging (~125 ms), then back to shutdown (low self-heating).
 * Blocks until the result is ready.
 */
esp_err_t tmp117_measure(tmp117_t* dev, float* temperature);

#ifdef __cplusplus
}
#endif

#endif // TMP117_H
