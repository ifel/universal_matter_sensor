#ifndef SENSORS_I2C_BUS_H
#define SENSORS_I2C_BUS_H

#include <i2cdev.h>
#include <sdkconfig.h>

// Shared sensor I2C bus (from Kconfig).
inline constexpr auto sda_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SDA_GPIO);
inline constexpr auto scl_gpio = static_cast<gpio_num_t>(CONFIG_SENSORS_I2C_SCL_GPIO);
inline constexpr auto i2c_port = static_cast<i2c_port_t>(CONFIG_SENSORS_I2C_PORT);

#endif // SENSORS_I2C_BUS_H
