#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Provides CHIP_DEVICE_CONFIG_ENABLE_THREAD used below.
#include <esp_matter.h>
#include <esp_timer.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "matter_interface/matter_manager.h"
#include "nvs_flash.h"
#include "sensors_manager/sensors_manager.h"
#include "status_led/status_led.h"

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#    include <platform/ESP32/OpenthreadLauncher.h>
#endif

// Logging tag
constexpr const char* tag = "   ***app_main***   ";

// Task Configuration
constexpr int64_t sensor_read_delay_ms =
    CONFIG_SENSOR_READ_DELAY_MS; // Delay between sensor readings (in ms)

constexpr uint32_t restart_delay_ms = 10000;

namespace {
// Logs the reason and reboots so a transient fault (wiring, power) can recover.
[[noreturn]] void restartAfterFailure(const char* reason) {
    ESP_LOGE(
        tag, "%s. Restarting in %u ms...", reason, static_cast<unsigned int>(restart_delay_ms));
    vTaskDelay(pdMS_TO_TICKS(restart_delay_ms));
    esp_restart();
}
} // namespace

// NOLINTNEXTLINE(readability-identifier-naming, readability-function-cognitive-complexity)
extern "C" void app_main() {
    // Initialize NVS
    ESP_LOGI(tag, "Initializing NVS...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(tag, "Erasing NVS partition...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_LOGI(tag, "NVS initialized.");

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#    define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                                  \
        {                                                                                          \
            .radio_mode = RADIO_MODE_NATIVE,                                                       \
        }

#    define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                                   \
        {                                                                                          \
            .host_connection_mode = HOST_CONNECTION_MODE_NONE,                                     \
        }

#    define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                                   \
        {                                                                                          \
            .storage_partition_name = "nvs",                                                       \
            .netif_queue_size = 10,                                                                \
            .task_queue_size = 10,                                                                 \
        }

    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    // Initialize SensorsManager
    SensorsManager sensors_manager;
    if (sensors_manager.init() != ESP_OK) { restartAfterFailure("Sensors initialization failed"); }
    if (!sensors_manager.isSensorsDetected()) { restartAfterFailure("No sensors detected"); }

    // Initialize status LED and drive it from Matter state changes
    StatusLed status_led;
    status_led.init();

    // Initialize Matter
    ESP_LOGI(tag, "Initializing Matter interface...");
    MatterManager matter_manager;
    matter_manager.setListeners({
        .on_commissioning_window = [&status_led](bool open) { status_led.setCommissioning(open); },
        .on_identify = [&status_led](bool identifying) { status_led.setIdentifying(identifying); },
    });
    err = matter_manager.init(sensors_manager.getRanges());
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Matter initialization failed: %s", esp_err_to_name(err));
        restartAfterFailure("Matter initialization failed");
    }
    ESP_LOGI(tag, "Matter initialized.");
    status_led.setReady(true);

    // Sensor read loop
    while (true) {
        const int64_t cycle_start_us = esp_timer_get_time();
        err = sensors_manager.readValues();
        if (err != ESP_OK) {
            ESP_LOGE(tag, "Failed to read sensor data: %s", esp_err_to_name(err));
        }

        // Publish readings even on failure so unavailable values become null in Matter
        const auto& readings = sensors_manager.getReadings();
        const auto& temperature = readings[toIndex(SensorType::TEMPERATURE)];
        const auto& pressure = readings[toIndex(SensorType::PRESSURE)];
        const auto& relative_humidity = readings[toIndex(SensorType::RELATIVE_HUMIDITY)];
        if (temperature.has_value()) { ESP_LOGI(tag, "Temperature = %.2f°C", *temperature); }
        if (pressure.has_value()) { ESP_LOGI(tag, "Pressure = %.2f hPa", *pressure / 100.0F); }
        if (relative_humidity.has_value()) {
            ESP_LOGI(tag, "Relative Humidity = %.2f %%", *relative_humidity);
        }

        // Update Matter attributes
        err = matter_manager.updateValues(readings);
        if (err != ESP_OK) {
            ESP_LOGE(tag, "Failed to update Matter attributes: %s", esp_err_to_name(err));
        }

        const int64_t cycle_duration_ms = (esp_timer_get_time() - cycle_start_us) / 1000;
        if (sensor_read_delay_ms > cycle_duration_ms) {
            // Delay for next reading
            vTaskDelay(pdMS_TO_TICKS(sensor_read_delay_ms - cycle_duration_ms));
        }
    }
}
