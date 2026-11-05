/**
 * @file bmp5_i2c.c
 *
 * esp-idf-lib i2cdev bus glue for the Bosch BMP5 SensorAPI.
 */
#include "bmp5_i2c.h"

#include <esp_idf_lib_helpers.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define I2C_FREQ_HZ 400000

#define CHECK_ARG(VAL)                                                                             \
    do {                                                                                           \
        if (!(VAL)) return ESP_ERR_INVALID_ARG;                                                    \
    } while (0)

static BMP5_INTF_RET_TYPE bus_read(uint8_t reg_addr, uint8_t* data, uint32_t len, void* intf_ptr) {
    i2c_dev_t* i2c_dev = (i2c_dev_t*)intf_ptr;
    if (i2c_dev_take_mutex(i2c_dev) != ESP_OK) return BMP5_E_COM_FAIL;
    const esp_err_t err = i2c_dev_read_reg(i2c_dev, reg_addr, data, len);
    i2c_dev_give_mutex(i2c_dev);
    return err == ESP_OK ? BMP5_INTF_RET_SUCCESS : BMP5_E_COM_FAIL;
}

static BMP5_INTF_RET_TYPE
bus_write(uint8_t reg_addr, const uint8_t* data, uint32_t len, void* intf_ptr) {
    i2c_dev_t* i2c_dev = (i2c_dev_t*)intf_ptr;
    if (i2c_dev_take_mutex(i2c_dev) != ESP_OK) return BMP5_E_COM_FAIL;
    const esp_err_t err = i2c_dev_write_reg(i2c_dev, reg_addr, data, len);
    i2c_dev_give_mutex(i2c_dev);
    return err == ESP_OK ? BMP5_INTF_RET_SUCCESS : BMP5_E_COM_FAIL;
}

// Busy-wait below one RTOS tick (the API asks for e.g. 2 ms after reset), sleep otherwise.
static void delay_us(uint32_t period, void* intf_ptr) {
    (void)intf_ptr;
    const uint32_t tick_us = portTICK_PERIOD_MS * 1000U;
    if (period < tick_us) {
        esp_rom_delay_us(period);
    } else {
        vTaskDelay((period + tick_us - 1U) / tick_us);
    }
}

esp_err_t bmp5_i2c_init_desc(
    bmp5_i2c_t* desc, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio) {
    CHECK_ARG(desc);
    CHECK_ARG(addr == BMP5_I2C_ADDR_PRIM || addr == BMP5_I2C_ADDR_SEC);

    desc->i2c_dev.port = port;
    desc->i2c_dev.addr = addr;
    desc->i2c_dev.cfg.sda_io_num = sda_gpio;
    desc->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    desc->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    desc->dev.intf = BMP5_I2C_INTF;
    desc->dev.intf_ptr = &desc->i2c_dev;
    desc->dev.read = bus_read;
    desc->dev.write = bus_write;
    desc->dev.delay_us = delay_us;

    return i2c_dev_create_mutex(&desc->i2c_dev);
}

esp_err_t bmp5_i2c_free_desc(bmp5_i2c_t* desc) {
    CHECK_ARG(desc);

    return i2c_dev_delete_mutex(&desc->i2c_dev);
}

esp_err_t bmp5_to_esp_err(int8_t rslt) {
    switch (rslt) {
    case BMP5_OK:
        return ESP_OK;
    case BMP5_E_NULL_PTR:
        return ESP_ERR_INVALID_ARG;
    case BMP5_E_DEV_NOT_FOUND:
    case BMP5_E_INVALID_CHIP_ID:
        return ESP_ERR_NOT_FOUND;
    case BMP5_E_NVM_NOT_READY:
    case BMP5_E_POR_SOFTRESET:
        return ESP_ERR_INVALID_STATE;
    default:
        return ESP_FAIL;
    }
}
