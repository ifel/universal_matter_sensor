#include <cstring>
#include <bmp280.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>

#include "matter_manager.h"

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

// Logging tag
static const char *TAG = "   ***app_main***   ";

// Temperature range
#define BMP280_TEMP_MIN_DEG_CELSIUS (-40.0)
#define BMP280_TEMP_MAX_DEG_CELSIUS 85.0

// Pressure range
#define BMP280_PRESS_MIN_PA 30000.0
#define BMP280_PRESS_MAX_PA 110000.0

// Relative Humidity range
#define BMP280_HUMI_MIN_PERCENT 0.0
#define BMP280_HUMI_MAX_PERCENT 100.0

// BMP280 / BME280 Sensor Configuration (from Kconfig)
#define BMP280_I2C_ADDRESS CONFIG_BMP280_I2C_ADDRESS
#define GPIO_NUM_SDA static_cast<gpio_num_t>(CONFIG_BMP280_I2C_SDA_GPIO)
#define GPIO_NUM_SCL static_cast<gpio_num_t>(CONFIG_BMP280_I2C_SCL_GPIO)
#define I2C_PORT_NUM static_cast<i2c_port_t>(CONFIG_BMP280_I2C_PORT)

// Matter Configuration
#define MATTER_TEMP_SENSOR_INITIAL_VALUE BMP280_TEMP_MAX_DEG_CELSIUS // Initial temperature value for Matter (in °C)
#define MATTER_TEMP_MIN BMP280_TEMP_MIN_DEG_CELSIUS                  // Minimum temperature value
#define MATTER_TEMP_MAX BMP280_TEMP_MAX_DEG_CELSIUS                  // Maximum temperature value
#define MATTER_PRESS_INITIAL_VALUE BMP280_PRESS_MAX_PA               // Initial pressure value for Matter (in Pa)
#define MATTER_PRESS_MIN BMP280_PRESS_MIN_PA                         // Minimum pressure value
#define MATTER_PRESS_MAX BMP280_PRESS_MAX_PA                         // Maximum pressure value
#define MATTER_HUMI_INITIAL_VALUE BMP280_HUMI_MAX_PERCENT            // Initial humidity value for Matter (in %)
#define MATTER_HUMI_MIN BMP280_HUMI_MIN_PERCENT                      // Minimum humidity value
#define MATTER_HUMI_MAX BMP280_HUMI_MAX_PERCENT                      // Maximum humidity value

// Task Configuration
#define SENSOR_READ_DELAY_MS CONFIG_SENSOR_READ_DELAY_MS // Delay between sensor readings (in ms)

extern "C" void app_main() {
    // Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS partition...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_LOGI(TAG, "NVS initialized.");

    ESP_LOGI(TAG, "Initializing I2C...");
    err = i2cdev_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "I2C initialized.");

    // Initialize BMP280
    ESP_LOGI(TAG, "Initializing BMP280 sensor...");
    static bmp280_t bmp280_sensor;

    memset(&bmp280_sensor, 0, sizeof(bmp280_t));

    bmp280_params_t params;
    err = bmp280_init_default_params(&params);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 initialization bmp280_init_default_params failed: %s", esp_err_to_name(err));
        return;
    }

    err = bmp280_init_desc(&bmp280_sensor, BMP280_I2C_ADDRESS, I2C_PORT_NUM, GPIO_NUM_SDA, GPIO_NUM_SCL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 initialization bmp280_init_desc failed: %s", esp_err_to_name(err));
        return;
    }

    err = bmp280_init(&bmp280_sensor, &params);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 initialization bmp280_init failed: %s", esp_err_to_name(err));
        return;
    }
    const bool is_bme280 = (bmp280_sensor.id == BME280_CHIP_ID);
    ESP_LOGI(TAG, "Sensor initialized: %s detected (chip ID: 0x%02x).", is_bme280 ? "BME280" : "BMP280",
             bmp280_sensor.id);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                                                          \
    {                                                                                                                  \
        .radio_mode = RADIO_MODE_NATIVE,                                                                               \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                                                           \
    {                                                                                                                  \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,                                                             \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                                                           \
    {                                                                                                                  \
        .storage_partition_name = "nvs",                                                                               \
        .netif_queue_size = 10,                                                                                        \
        .task_queue_size = 10,                                                                                         \
    }
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    // Initialize Matter
    ESP_LOGI(TAG, "Initializing Matter interface...");
    uint16_t temp_sensor_ep_id = 0, press_sensor_ep_id = 0, humi_sensor_ep_id = 0;
    uint16_t *humi_ep_ptr = is_bme280 ? &humi_sensor_ep_id : nullptr;
    err = matter_init(&temp_sensor_ep_id, &press_sensor_ep_id, humi_ep_ptr, MATTER_TEMP_MIN, MATTER_TEMP_MAX,
                      MATTER_TEMP_SENSOR_INITIAL_VALUE, MATTER_PRESS_MIN, MATTER_PRESS_MAX, MATTER_PRESS_INITIAL_VALUE,
                      MATTER_HUMI_MIN, MATTER_HUMI_MAX, MATTER_HUMI_INITIAL_VALUE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Matter initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Matter initialized.");

    // Sensor read loop
    float temperature = 0, pressure = 0, humidity = 0;
    while (true) {
        err = bmp280_read_float(&bmp280_sensor, &temperature, &pressure, is_bme280 ? &humidity : nullptr);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(err));
        } else {
            if (is_bme280) {
                ESP_LOGI(TAG, "Temperature = %.2f°C, Pressure = %.2f hPa, Humidity = %.2f %%", temperature,
                         pressure / 100.0, humidity);
            } else {
                ESP_LOGI(TAG, "Temperature = %.2f°C, Pressure = %.2f hPa", temperature, pressure / 100.0);
            }

            // Update Matter attributes
            err = matter_update_values(temp_sensor_ep_id, press_sensor_ep_id, humi_ep_ptr, temperature, pressure,
                                       humidity);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update Matter attributes: %s", esp_err_to_name(err));
            }
        }

        // Delay for next reading
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_DELAY_MS));
    }
}
