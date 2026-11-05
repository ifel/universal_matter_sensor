/**
 * @file bmp5_i2c.h
 *
 * esp-idf-lib i2cdev bus glue for the Bosch BMP5 SensorAPI (BMP581 / BMP585).
 * Use the Bosch API (bmp5.h) with &desc->dev once the descriptor is initialized.
 */
#ifndef BMP5_I2C_H
#define BMP5_I2C_H

#include <esp_err.h>
#include <i2cdev.h>

#include "bmp5.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * BMP5 device bound to an i2cdev descriptor.
 */
typedef struct {
    i2c_dev_t i2c_dev;   //!< I2C device descriptor (bus access, mutex)
    struct bmp5_dev dev; //!< Bosch device, its callbacks use i2c_dev
} bmp5_i2c_t;

/**
 * @brief Initialize the descriptor and hook the Bosch callbacks to i2cdev.
 *
 * @param desc Descriptor
 * @param addr I2C address, BMP5_I2C_ADDR_PRIM (0x46) or BMP5_I2C_ADDR_SEC (0x47)
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t bmp5_i2c_init_desc(
    bmp5_i2c_t* desc, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free the descriptor.
 */
esp_err_t bmp5_i2c_free_desc(bmp5_i2c_t* desc);

/**
 * @brief Map a Bosch API result (BMP5_OK, BMP5_E_*) to esp_err_t.
 */
esp_err_t bmp5_to_esp_err(int8_t rslt);

#ifdef __cplusplus
}
#endif

#endif // BMP5_I2C_H
