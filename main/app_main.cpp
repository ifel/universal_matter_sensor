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
constexpr const char* tag = "app_main";

// Task Configuration
constexpr int64_t sensor_read_delay_ms =
    CONFIG_SENSOR_READ_DELAY_MS; // Delay between sensor readings (in ms)

constexpr uint32_t restart_delay_ms = 10000;

// Loop watchdog slack on top of one read interval (sensor init, slow reads).
constexpr int64_t loop_watchdog_margin_ms = 60000;
constexpr uint64_t loop_watchdog_timeout_us =
    static_cast<uint64_t>(sensor_read_delay_ms + loop_watchdog_margin_ms) * 1000U;

namespace {
// Logs the reason and reboots so a transient fault (wiring, power) can recover.
[[noreturn]] void restartAfterFailure(const char* reason) {
    ESP_LOGE(
        tag, "%s. Restarting in %u ms...", reason, static_cast<unsigned int>(restart_delay_ms));
    vTaskDelay(pdMS_TO_TICKS(restart_delay_ms));
    esp_restart();
}

// Loop watchdog: a one-shot timer the read loop restarts every cycle. If the loop stops coming
// back (e.g. a driver blocked on I2C), it aborts, so the panic handler logs and reboots. It is
// separate from the task watchdog, whose short idle-task timeout must not depend on the read
// interval.
void loopStalled(void* /*arg*/) {
    esp_system_abort("Sensor read loop stalled");
}

esp_timer_handle_t startLoopWatchdog() {
    const esp_timer_create_args_t args = {.callback = loopStalled, .name = "loop_watchdog"};
    esp_timer_handle_t timer = nullptr;
    esp_err_t err = esp_timer_create(&args, &timer);
    if (err == ESP_OK) { err = esp_timer_start_once(timer, loop_watchdog_timeout_us); }
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Loop watchdog setup failed: %s", esp_err_to_name(err));
        return nullptr;
    }
    return timer;
}

void feedLoopWatchdog(esp_timer_handle_t timer) {
    if (timer != nullptr) { esp_timer_restart(timer, loop_watchdog_timeout_us); }
}

// How each reading is logged: value * scale with the given decimals, then the unit.
struct ReadingFormat {
    SensorType type;
    const char* label;
    float scale;
    int decimals;
    const char* unit;
};

// NOLINTBEGIN(modernize-use-designated-initializers): a table reads better positionally.
constexpr PerSensorType<ReadingFormat> reading_formats = {{
    {SensorType::TEMPERATURE, "Temperature", 1.0F, 2, "°C"},
    {SensorType::PRESSURE, "Pressure", 0.01F, 2, "hPa"},
    {SensorType::RELATIVE_HUMIDITY, "Relative Humidity", 1.0F, 2, "%"},
    {SensorType::CARBON_DIOXIDE_CONCENTRATION, "CO2 Concentration", 1.0F, 2, "ppm"},
    {SensorType::ILLUMINANCE, "Illuminance", 1.0F, 2, "lux"},
    {SensorType::TVOC_CONCENTRATION, "TVOC Concentration", 1.0F, 0, "ppb"},
    {SensorType::AIR_QUALITY, "Air Quality", 1.0F, 0, "(1 good .. 6 extremely poor)"},
    {SensorType::REMAINING_LEVEL, "Remaining Level", 1.0F, 1, "%"},
    {SensorType::BATTERY_PERCENT, "Battery", 1.0F, 1, "%"},
    {SensorType::BATTERY_VOLTAGE, "Battery Voltage", 1.0F, 3, "V"},
    {SensorType::BATTERY_CHARGE_STATE,
     "Battery Charge State",
     1.0F,
     0,
     "(1 charging, 2 full, 3 not charging)"},
    {SensorType::OCCUPANCY, "Occupancy", 1.0F, 0, ""},
}};
// NOLINTEND(modernize-use-designated-initializers)

// A missing entry would be value-initialized as TEMPERATURE.
constexpr bool readingFormatsInEnumOrder() {
    for (size_t i = 0; i < sensor_type_count; ++i) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        if (reading_formats[i].type != static_cast<SensorType>(i)) { return false; }
    }
    return true;
}
static_assert(readingFormatsInEnumOrder(), "reading_formats must list every SensorType in order");

void logReadings(const SensorsManager& sensors_manager, const SensorReadings& readings) {
    for (const auto& format : reading_formats) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        const auto& value = readings[toIndex(format.type)];
        if (!value.has_value()) { continue; }
        ESP_LOGI(tag,
                 "%s = %.*f %s from %s",
                 format.label,
                 format.decimals,
                 *value * format.scale,
                 format.unit,
                 sensors_manager.getSensorModelNameForType(format.type));
    }
}
} // namespace

// NOLINTBEGIN(readability-identifier-naming, readability-function-cognitive-complexity)
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

    esp_timer_handle_t loop_watchdog = startLoopWatchdog();

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
        .on_remaining_level_reset =
            [&sensors_manager] { sensors_manager.requestCalibration(SensorType::REMAINING_LEVEL); },
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
        feedLoopWatchdog(loop_watchdog);
        const int64_t cycle_start_us = esp_timer_get_time();
        err = sensors_manager.readValues(compensations);
        if (err != ESP_OK) {
            ESP_LOGE(tag, "Failed to read sensor data: %s", esp_err_to_name(err));
        }

        // Publish readings even on failure so unavailable values become null in Matter
        const auto& readings = sensors_manager.getReadings();
        logReadings(sensors_manager, readings);

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
// NOLINTEND(readability-identifier-naming, readability-function-cognitive-complexity)
