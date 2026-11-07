#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Provides CHIP_DEVICE_CONFIG_ENABLE_THREAD used below.
#include <esp_matter.h>
#include <esp_task_wdt.h>
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

// Watchdog slack on top of one read interval (sensor init, slow reads).
constexpr uint32_t watchdog_margin_ms = 60000;

namespace {
// Logs the reason and reboots so a transient fault (wiring, power) can recover.
[[noreturn]] void restartAfterFailure(const char* reason) {
    ESP_LOGE(
        tag, "%s. Restarting in %u ms...", reason, static_cast<unsigned int>(restart_delay_ms));
    vTaskDelay(pdMS_TO_TICKS(restart_delay_ms));
    esp_restart();
}

// Reboots (via panic) if this task stops feeding the watchdog, e.g. a driver blocked on I2C.
void subscribeToWatchdog() {
    uint32_t idle_core_mask = 0;
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
    idle_core_mask |= 1U << 0;
#endif
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
    idle_core_mask |= 1U << 1;
#endif
    const esp_task_wdt_config_t config = {
        .timeout_ms = static_cast<uint32_t>(sensor_read_delay_ms) + watchdog_margin_ms,
        .idle_core_mask = idle_core_mask,
        .trigger_panic = true,
    };
    esp_err_t err = esp_task_wdt_reconfigure(&config);
    if (err == ESP_OK) { err = esp_task_wdt_add(nullptr); }
    if (err != ESP_OK) { ESP_LOGE(tag, "Task watchdog setup failed: %s", esp_err_to_name(err)); }
}
} // namespace

// NOLINTBEGIN(readability-identifier-naming, readability-function-cognitive-complexity)
// NOLINTBEGIN(readability-function-size)
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

    subscribeToWatchdog();

    // Initialize SensorsManager
    SensorsManager sensors_manager;
    if (sensors_manager.init(sensor_read_delay_ms) != ESP_OK) {
        restartAfterFailure("Sensors initialization failed");
    }
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
    // Event-driven readings (e.g. presence) skip the read interval.
    sensors_manager.setReadingListener(
        [&matter_manager](SensorType type, std::optional<float> value) {
            matter_manager.updateValue(type, value);
        });
    status_led.setReady(true);

    SensorReadings compensations{};
    // Sensor read loop
    while (true) {
        esp_task_wdt_reset();
        const int64_t cycle_start_us = esp_timer_get_time();
        err = sensors_manager.readValues(compensations);
        if (err != ESP_OK) {
            ESP_LOGE(tag, "Failed to read sensor data: %s", esp_err_to_name(err));
        }

        // Publish readings even on failure so unavailable values become null in Matter
        const auto& readings = sensors_manager.getReadings();
        const auto& temperature = at<SensorType::TEMPERATURE>(readings);
        const auto& pressure = at<SensorType::PRESSURE>(readings);
        const auto& relative_humidity = at<SensorType::RELATIVE_HUMIDITY>(readings);
        const auto& carbon_dioxide_concentration =
            at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(readings);
        const auto& illuminance = at<SensorType::ILLUMINANCE>(readings);
        const auto& tvoc_concentration = at<SensorType::TVOC_CONCENTRATION>(readings);
        const auto& air_quality = at<SensorType::AIR_QUALITY>(readings);
        const auto& remaining_level = at<SensorType::REMAINING_LEVEL>(readings);
        const auto& battery_percent = at<SensorType::BATTERY_PERCENT>(readings);
        const auto& battery_voltage = at<SensorType::BATTERY_VOLTAGE>(readings);
        const auto& battery_charge_state = at<SensorType::BATTERY_CHARGE_STATE>(readings);
        const auto& occupancy = at<SensorType::OCCUPANCY>(readings);
        if (temperature.has_value()) {
            ESP_LOGI(tag,
                     "Temperature = %.2f°C from %s",
                     *temperature,
                     sensors_manager.getSensorModelNameForType(SensorType::TEMPERATURE));
        }
        if (pressure.has_value()) {
            ESP_LOGI(tag,
                     "Pressure = %.2f hPa from %s",
                     *pressure / 100.0F,
                     sensors_manager.getSensorModelNameForType(SensorType::PRESSURE));
        }
        if (relative_humidity.has_value()) {
            ESP_LOGI(tag,
                     "Relative Humidity = %.2f %% from %s",
                     *relative_humidity,
                     sensors_manager.getSensorModelNameForType(SensorType::RELATIVE_HUMIDITY));
        }
        if (carbon_dioxide_concentration.has_value()) {
            ESP_LOGI(tag,
                     "Carbon Dioxide (CO2) Concentration = %.2f ppm from %s",
                     *carbon_dioxide_concentration,
                     sensors_manager.getSensorModelNameForType(
                         SensorType::CARBON_DIOXIDE_CONCENTRATION));
        }
        if (illuminance.has_value()) {
            ESP_LOGI(tag,
                     "Illuminance = %.2f lux from %s",
                     *illuminance,
                     sensors_manager.getSensorModelNameForType(SensorType::ILLUMINANCE));
        }
        if (tvoc_concentration.has_value()) {
            ESP_LOGI(tag,
                     "TVOC Concentration = %.0f ppb from %s",
                     *tvoc_concentration,
                     sensors_manager.getSensorModelNameForType(SensorType::TVOC_CONCENTRATION));
        }
        if (air_quality.has_value()) {
            ESP_LOGI(tag,
                     "Air Quality = %.0f (1 good .. 6 extremely poor) from %s",
                     *air_quality,
                     sensors_manager.getSensorModelNameForType(SensorType::AIR_QUALITY));
        }
        if (remaining_level.has_value()) {
            ESP_LOGI(tag,
                     "Remaining Level = %.1f %% from %s",
                     *remaining_level,
                     sensors_manager.getSensorModelNameForType(SensorType::REMAINING_LEVEL));
        }
        if (battery_percent.has_value()) {
            ESP_LOGI(tag,
                     "Battery = %.1f %% from %s",
                     *battery_percent,
                     sensors_manager.getSensorModelNameForType(SensorType::BATTERY_PERCENT));
        }
        if (battery_voltage.has_value()) {
            ESP_LOGI(tag,
                     "Battery Voltage = %.3f V from %s",
                     *battery_voltage,
                     sensors_manager.getSensorModelNameForType(SensorType::BATTERY_VOLTAGE));
        }
        if (battery_charge_state.has_value()) {
            ESP_LOGI(tag,
                     "Battery Charge State = %.0f (1 charging, 2 full, 3 not charging) from %s",
                     *battery_charge_state,
                     sensors_manager.getSensorModelNameForType(SensorType::BATTERY_CHARGE_STATE));
        }
        if (occupancy.has_value()) {
            ESP_LOGI(tag,
                     "Occupancy = %.0f from %s",
                     *occupancy,
                     sensors_manager.getSensorModelNameForType(SensorType::OCCUPANCY));
        }

        compensations = readings;

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
// NOLINTEND(readability-function-size)
// NOLINTEND(readability-identifier-naming, readability-function-cognitive-complexity)
