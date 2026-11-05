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
 * @file ens160.h
 *
 * ESP-IDF driver for ScioSense ENS160 digital metal-oxide multi-gas sensor.
 *
 * Based on k0i05/esp_ens160 1.2.8, ported to the esp-idf-lib i2cdev bus layer
 * so it can share the I2C bus with other esp-idf-lib drivers.
 */
#ifndef ENS160_H
#define ENS160_H

#include <esp_err.h>
#include <i2cdev.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENS160_I2C_ADDR_LO UINT8_C(0x52) //!< ADDR pin low
#define ENS160_I2C_ADDR_HI UINT8_C(0x53) //!< ADDR pin high

#define ENS160_PART_ID UINT16_C(0x0160) //!< expected PART_ID register value

#define ENS160_TVOC_MIN UINT16_C(0)     //!< tvoc minimum in ppb (section 5.1)
#define ENS160_TVOC_MAX UINT16_C(65000) //!< tvoc maximum in ppb (section 5.1)
#define ENS160_ECO2_MIN UINT16_C(400)   //!< equivalent co2 minimum in ppm (table 5)
#define ENS160_ECO2_MAX UINT16_C(65000) //!< equivalent co2 maximum in ppm (table 5)
#define ENS160_AQI_MIN  UINT16_C(1)     //!< UBA air quality index minimum (table 6)
#define ENS160_AQI_MAX  UINT16_C(5)     //!< UBA air quality index maximum (table 6)

/**
 * Air quality index according to the UBA.
 */
typedef enum {
    ENS160_AQI_UBA_INDEX_UNKNOWN = 0,
    ENS160_AQI_UBA_INDEX_1 = 1, //!< excellent
    ENS160_AQI_UBA_INDEX_2 = 2, //!< good
    ENS160_AQI_UBA_INDEX_3 = 3, //!< moderate
    ENS160_AQI_UBA_INDEX_4 = 4, //!< poor
    ENS160_AQI_UBA_INDEX_5 = 5  //!< unhealthy
} ens160_aqi_uba_indexes_t;

/**
 * Operating modes.
 */
typedef enum {
    ENS160_OPMODE_DEEP_SLEEP = 0x00, //!< low-power standby
    ENS160_OPMODE_IDLE = 0x01,       //!< low-power (default)
    ENS160_OPMODE_STANDARD = 0x02,   //!< gas sensing
    ENS160_OPMODE_RESET = 0xf0       //!< reset
} ens160_operating_modes_t;

/**
 * Commands.
 */
typedef enum {
    ENS160_CMD_NORMAL = 0x00,        //!< normal operation (default)
    ENS160_CMD_GET_FW_APPVER = 0x0e, //!< get firmware version
    ENS160_CMD_CLEAR_GPR = 0xcc      //!< clear general purpose read registers
} ens160_commands_t;

/**
 * Validity flags from the status register.
 */
typedef enum {
    ENS160_VALFLAG_NORMAL = 0x00,          //!< normal operation
    ENS160_VALFLAG_WARMUP = 0x01,          //!< first 3 minutes after power-on
    ENS160_VALFLAG_INITIAL_STARTUP = 0x02, //!< first hour of operation, once in a lifetime
    ENS160_VALFLAG_INVALID_OUTPUT = 0x03   //!< invalid output
} ens160_validity_flags_t;

/**
 * Calculated air quality data.
 */
typedef struct {
    ens160_aqi_uba_indexes_t uba_aqi; //!< air quality index according to the UBA
    uint16_t tvoc;                    //!< tvoc in ppb
    uint16_t etoh;                    //!< ethanol in ppb
    uint16_t eco2;                    //!< equivalent co2 in ppm
} ens160_air_quality_data_t;

/**
 * Raw resistance data in ohms.
 */
typedef struct {
    uint32_t hp0_ri;
    uint32_t hp1_ri;
    uint32_t hp2_ri;
    uint32_t hp3_ri;
    uint32_t hp0_bl;
    uint32_t hp1_bl;
    uint32_t hp2_bl;
    uint32_t hp3_bl;
} ens160_air_quality_raw_data_t;

/**
 * Device descriptor.
 */
typedef struct {
    i2c_dev_t i2c_dev; //!< I2C device descriptor
    uint16_t part_id;  //!< part identifier read by ens160_init()
} ens160_t;

/**
 * @brief Initialize device descriptor.
 *
 * @param dev Device descriptor
 * @param addr I2C address, ENS160_I2C_ADDR_LO or ENS160_I2C_ADDR_HI
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t ens160_init_desc(
    ens160_t* dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor.
 */
esp_err_t ens160_free_desc(ens160_t* dev);

/**
 * @brief Reset the device, verify the part id, and start standard (gas sensing) mode.
 *
 * Interrupt pin is disabled.
 */
esp_err_t ens160_init(ens160_t* dev);

/**
 * @brief Wait for new data (up to 1.5 s) and read calculated air quality.
 */
esp_err_t ens160_get_measurement(ens160_t* dev, ens160_air_quality_data_t* data);

/**
 * @brief Wait for new GPR data (up to 1.5 s) and read raw resistances.
 */
esp_err_t ens160_get_raw_measurement(ens160_t* dev, ens160_air_quality_raw_data_t* data);

esp_err_t ens160_get_data_status(ens160_t* dev, bool* ready);
esp_err_t ens160_get_gpr_data_status(ens160_t* dev, bool* ready);
esp_err_t ens160_get_validity_status(ens160_t* dev, ens160_validity_flags_t* state);
esp_err_t ens160_get_error_status(ens160_t* dev, bool* error);
esp_err_t ens160_get_mode_status(ens160_t* dev, bool* mode);
esp_err_t ens160_get_status(ens160_t* dev,
                            bool* data_ready,
                            bool* gpr_data_ready,
                            ens160_validity_flags_t* state,
                            bool* error,
                            bool* mode);

/**
 * @brief Read ambient temperature (°C) and humidity (%RH) used for compensation.
 */
esp_err_t ens160_get_compensation_factors(ens160_t* dev, float* temperature, float* humidity);

/**
 * @brief Write ambient temperature (°C, -40..125) and humidity (%RH, 0..100) for compensation.
 */
esp_err_t ens160_set_compensation_factors(ens160_t* dev, float temperature, float humidity);

esp_err_t ens160_enable_standard_mode(ens160_t* dev);
esp_err_t ens160_enable_idle_mode(ens160_t* dev);
esp_err_t ens160_enable_deep_sleep_mode(ens160_t* dev);

/**
 * @brief Soft-reset the device and set it up again (same as ens160_init()).
 */
esp_err_t ens160_reset(ens160_t* dev);

#ifdef __cplusplus
}
#endif

#endif // ENS160_H
