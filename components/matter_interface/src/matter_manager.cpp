#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_timer.h>

#include "matter_manager.h"
#include "matter_pressure_sensor.h"
#include "matter_relative_humidity_sensor.h"
#include "matter_temperature_sensor.h"

#define MATTER_TEMP_SCALE 100.0
#define MATTER_RELATIVE_HUMIDITY_SCALE 100.0
#define MATTER_PRESSURE_SCALE 10.0

#define LED_BLINK_COMMISSIONING_US (1000 * 1000)
#define LED_BLINK_IDENTIFY_US (200 * 1000)

static auto TAG = "   ***matter_interface***   ";

#if CONFIG_STATUS_LED_GPIO >= 0
static constexpr gpio_num_t kStatusLedGpio = static_cast<gpio_num_t>(CONFIG_STATUS_LED_GPIO);
#ifdef CONFIG_STATUS_LED_ACTIVE_LOW
static constexpr uint32_t kLedOnLevel = 0;
static constexpr uint32_t kLedOffLevel = 1;
#else
static constexpr uint32_t kLedOnLevel = 1;
static constexpr uint32_t kLedOffLevel = 0;
#endif

static esp_timer_handle_t s_led_timer = nullptr;
static bool s_led_on = false;
static bool s_commissioning_window_open = false;
static bool s_identifying = false;

static void status_led_set(bool on) {
    s_led_on = on;
    gpio_set_level(kStatusLedGpio, on ? kLedOnLevel : kLedOffLevel);
}

static void status_led_timer_cb(void *arg) {
    status_led_set(!s_led_on);
}

static void status_led_update_state() {
    if (s_led_timer == nullptr) {
        return;
    }
    esp_timer_stop(s_led_timer);
    if (s_identifying) {
        esp_timer_start_periodic(s_led_timer, LED_BLINK_IDENTIFY_US);
    } else if (s_commissioning_window_open) {
        esp_timer_start_periodic(s_led_timer, LED_BLINK_COMMISSIONING_US);
    } else {
        status_led_set(true);
    }
}

static void status_led_init() {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << kStatusLedGpio);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    status_led_set(false);

    esp_timer_create_args_t timer_args = {};
    timer_args.callback = &status_led_timer_cb;
    timer_args.name = "status_led";
    esp_timer_create(&timer_args, &s_led_timer);
}
#else
static void status_led_init() {}
static void status_led_update_state() {}
#endif

static inline int16_t matter_scale_temp(float value) {
    return static_cast<int16_t>(value * MATTER_TEMP_SCALE);
}

static inline int16_t matter_scale_pressure_to_kpa(float value_pa) {
    return static_cast<int16_t>(value_pa / 1000 * MATTER_PRESSURE_SCALE);
}

static inline uint16_t matter_scale_relative_humidity(float value) {
    return static_cast<uint16_t>(value * MATTER_RELATIVE_HUMIDITY_SCALE);
}

void matter_event_callback(const ChipDeviceEvent *event, intptr_t arg) {
    switch (event->Type) {
        // Signals a change in connectivity of the device's Wi-Fi station interface.
        case chip::DeviceLayer::DeviceEventType::kWiFiConnectivityChange:
            ESP_LOGI(TAG, "Wi-Fi connectivity change");
            break;

        // Signals a change in connectivity of the device's Thread interface.
        case chip::DeviceLayer::DeviceEventType::kThreadConnectivityChange:
            ESP_LOGI(TAG, "Thread connectivity change");
            break;

        // Signals a change in the device's ability to communicate via the internet.
        case chip::DeviceLayer::DeviceEventType::kInternetConnectivityChange:
            ESP_LOGI(TAG, "Internet connectivity change");
            break;

        // Signals a change in the device's ability to communicate with a chip-enabled service.
        case chip::DeviceLayer::DeviceEventType::kServiceConnectivityChange:
            ESP_LOGI(TAG, "Service connectivity change");
            break;

        // Signals a change to the device's service provisioning state.
        case chip::DeviceLayer::DeviceEventType::kServiceProvisioningChange:
            ESP_LOGI(TAG, "Service provisioning change");
            break;

        // Signals a change to the device's real-time clock synchronization state.
        case chip::DeviceLayer::DeviceEventType::kTimeSyncChange:
            ESP_LOGI(TAG, "Time synchronization change");
            break;

        // Signals that an external entity has established a new CHIPoBLE connection with the device.
        case chip::DeviceLayer::DeviceEventType::kCHIPoBLEConnectionEstablished:
            ESP_LOGI(TAG, "BLE connection established");
            break;

        // Signals that an external entity has closed an existing CHIPoBLE connection with the device.
        case chip::DeviceLayer::DeviceEventType::kCHIPoBLEConnectionClosed:
            ESP_LOGI(TAG, "BLE connection closed");
            break;

        // Request to close all BLE connections when concurrent connections are not supported.
        case chip::DeviceLayer::DeviceEventType::kCloseAllBleConnections:
            ESP_LOGI(TAG, "Close all BLE connections requested");
            break;

        // Indicates that the Wi-Fi device is available for connection.
        case chip::DeviceLayer::DeviceEventType::kWiFiDeviceAvailable:
            ESP_LOGI(TAG, "Wi-Fi device is available");
            break;

        // Indicates that the operational network has started.
        case chip::DeviceLayer::DeviceEventType::kOperationalNetworkStarted:
            ESP_LOGI(TAG, "Operational network started");
            break;

        // Signals a state change in the Thread stack.
        case chip::DeviceLayer::DeviceEventType::kThreadStateChange:
            ESP_LOGI(TAG, "Thread state change");
            break;

        // Indicates a change in the state of the Thread network interface.
        case chip::DeviceLayer::DeviceEventType::kThreadInterfaceStateChange:
            ESP_LOGI(TAG, "Thread interface state change");
            break;

        // Indicates a change in the CHIPoBLE advertising state.
        case chip::DeviceLayer::DeviceEventType::kCHIPoBLEAdvertisingChange:
            ESP_LOGI(TAG, "CHIPoBLE advertising state change");
            break;

        // Indicates that an IP address (IPv4 or IPv6) has changed for the interface.
        case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
            ESP_LOGI(TAG, "Interface IP address changed");
            break;

        // Signals that commissioning has completed via the general commissioning cluster command.
        case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
            ESP_LOGI(TAG, "Commissioning complete");
#if CONFIG_STATUS_LED_GPIO >= 0
            s_commissioning_window_open = false;
            status_led_update_state();
#endif
            break;

        // Indicates that the fail-safe timer expired before the CommissioningComplete command was invoked.
        case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
            ESP_LOGI(TAG, "Commissioning failed, fail-safe timer expired");
            break;

        // Indicates that the operational network is enabled.
        case chip::DeviceLayer::DeviceEventType::kOperationalNetworkEnabled:
            ESP_LOGI(TAG, "Operational network enabled");
            break;

        // Signals that DNS-SD has been initialized and is ready to operate.
        case chip::DeviceLayer::DeviceEventType::kDnssdInitialized:
            ESP_LOGI(TAG, "DNS-SD initialized");
            break;

        // Indicates that the DNS-SD backend was restarted and services must be published again.
        case chip::DeviceLayer::DeviceEventType::kDnssdRestartNeeded:
            ESP_LOGI(TAG, "DNS-SD restart needed");
            break;

        // Indicates that bindings were updated via cluster.
        case chip::DeviceLayer::DeviceEventType::kBindingsChangedViaCluster:
            ESP_LOGI(TAG, "Bindings updated via cluster");
            break;

        // Indicates a change in the OTA engine state.
        case chip::DeviceLayer::DeviceEventType::kOtaStateChanged:
            ESP_LOGI(TAG, "OTA state changed");
            break;

        // Indicates that server initialization has completed, and the node is ready for connections.
        case chip::DeviceLayer::DeviceEventType::kServerReady:
            ESP_LOGI(TAG, "Server is ready");
            break;

        // Signals that BLE has been deinitialized.
        case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
            ESP_LOGI(TAG, "BLE deinitialized");
            break;

        // Signals that a commissioning session has started.
        case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
            ESP_LOGI(TAG, "Commissioning session started");
            break;

        // Signals that a commissioning session has stopped.
        case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
            ESP_LOGI(TAG, "Commissioning session stopped");
            break;

        // Indicates that the commissioning window is now open.
        case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
            ESP_LOGI(TAG, "Commissioning window opened");
#if CONFIG_STATUS_LED_GPIO >= 0
            s_commissioning_window_open = true;
            status_led_update_state();
#endif
            break;

        // Indicates that the commissioning window is now closed.
        case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
            ESP_LOGI(TAG, "Commissioning window closed");
#if CONFIG_STATUS_LED_GPIO >= 0
            s_commissioning_window_open = false;
            status_led_update_state();
#endif
            break;

        // Indicates that a fabric is about to be removed.
        case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
            ESP_LOGI(TAG, "Fabric will be removed");
            break;

        // Indicates that a fabric has been removed.
        case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
            ESP_LOGI(TAG, "Fabric removed successfully");
            break;

        // Indicates that a fabric in the Fabric Table has been committed to storage.
        case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
            ESP_LOGI(TAG, "Fabric committed to storage");
            break;

        // Signals that operational credentials have been updated.
        case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
            ESP_LOGI(TAG, "Fabric updated");
            break;

        default:
            ESP_LOGI(TAG, "Unhandled Matter event type: %d", event->Type);
            break;
    }
}

esp_err_t identification_callback(esp_matter::identification::callback_type_t const type, uint16_t const endpoint_id,
                                  uint8_t const effect_id, uint8_t const effect_variant, void *priv_data) {
    ESP_LOGI(TAG, "Identification Callback Invoked: type=%d, endpoint_id=%u, effect_id=%u, effect_variant=%u", type,
             (unsigned int)endpoint_id, (unsigned int)effect_id, (unsigned int)effect_variant);
#if CONFIG_STATUS_LED_GPIO >= 0
    if (type == esp_matter::identification::START || type == esp_matter::identification::EFFECT) {
        s_identifying = true;
    } else if (type == esp_matter::identification::STOP) {
        s_identifying = false;
    }
    status_led_update_state();
#endif
    return ESP_OK;
}

esp_err_t attribute_update_callback(const esp_matter::attribute::callback_type_t type, const uint16_t endpoint_id,
                                    const uint32_t cluster_id, const uint32_t attribute_id, esp_matter_attr_val_t *val,
                                    void *priv_data) {
    ESP_LOGI(TAG, "Attribute update: type=%d, endpoint_id=%u, cluster_id=%u, attribute_id=%u", type,
             (unsigned int)endpoint_id, (unsigned int)cluster_id, (unsigned int)attribute_id);
    return ESP_OK;
}

esp_err_t matter_init(uint16_t *temperature_sensor_endpoint_id, uint16_t *pressure_sensor_endpoint_id,
                      uint16_t *relative_humidity_sensor_endpoint_id, const float min_temperature,
                      const float max_temperature, const float initial_temperature, const float min_pressure,
                      const float max_pressure, const float initial_pressure, const float min_relative_humidity,
                      const float max_relative_humidity, const float initial_relative_humidity) {
    status_led_init();

    // Initialize Matter Node, where
    // - node_config is the configuration object for the nodeset_callback
    // - app_attribute_update_cb is the callback for handling attribute updates for the temperature sensor
    // - app_identification_cb is the callback for handling identification queries
    esp_matter::node::config_t node_config;
    esp_matter::node_t *matter_node =
        esp_matter::node::create(&node_config, attribute_update_callback, identification_callback);
    if (matter_node == nullptr) {
        ESP_LOGE(TAG, "Failed to create Matter node.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Matter node created");

    // Initialize Matter temperature sensor endpoint
    if (create_temperature_sensor_endpoint(matter_node, temperature_sensor_endpoint_id,
                                           matter_scale_temp(min_temperature), matter_scale_temp(max_temperature),
                                           matter_scale_temp(initial_temperature)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize temperature sensor.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Temperature sensor endpoint initialized");

    // Initialize Matter pressure sensor endpoint
    // Pass scaled values to the endpoint creation function
    if (create_pressure_sensor_endpoint(
            matter_node, pressure_sensor_endpoint_id, matter_scale_pressure_to_kpa(min_pressure),
            matter_scale_pressure_to_kpa(max_pressure), matter_scale_pressure_to_kpa(initial_pressure)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize pressure sensor");
        return ESP_FAIL;
    }

    // Initialize Matter relative humidity sensor endpoint (optional, only when sensor supports humidity)
    if (relative_humidity_sensor_endpoint_id != nullptr) {
        if (create_relative_humidity_sensor_endpoint(matter_node, relative_humidity_sensor_endpoint_id,
                                                     matter_scale_relative_humidity(min_relative_humidity),
                                                     matter_scale_relative_humidity(max_relative_humidity),
                                                     matter_scale_relative_humidity(initial_relative_humidity)) !=
            ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize relative humidity sensor.");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Relative humidity sensor endpoint initialized");
    }

    // Start the Matter thread, using an optional event callback with private data.
    // It returns `ESP_OK` if successful or an error if it fails.
    esp_err_t matter_err = esp_matter::start(matter_event_callback);
    if (matter_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Matter, error: %d", matter_err);
        return matter_err;
    }
    ESP_LOGI(TAG, "Matter started");
    status_led_update_state();

    // Console shell is helpful when developing/debugging the application.
    // Set CONFIG_ENABLE_CHIP_SHELL=n in sdkconfig.defaults in production.
    // https://docs.espressif.com/projects/esp-matter/en/latest/esp32/optimizations.html
#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
#if CONFIG_OPENTHREAD_CLI
    console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif

    return ESP_OK;
}

esp_err_t matter_update_values(const uint16_t temperature_endpoint_id, const uint16_t pressure_sensor_endpoint_id,
                               const uint16_t *relative_humidity_endpoint_id, const float new_temperature,
                               const float new_pressure, const float new_relative_humidity) {
    // Update temperature
    esp_matter_attr_val_t new_temperature_int = esp_matter_nullable_int16(matter_scale_temp(new_temperature));
    const esp_err_t temperature_ret = esp_matter::attribute::update(
        temperature_endpoint_id, chip::app::Clusters::TemperatureMeasurement::Id,
        chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id, &new_temperature_int);
    if (temperature_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update temperature sensor endpoint with new temperature value");
        return temperature_ret;
    }

    // Update pressure
    esp_matter_attr_val_t new_pressure_int = esp_matter_nullable_int16(matter_scale_pressure_to_kpa(new_pressure));
    const esp_err_t pressure_ret = esp_matter::attribute::update(
        pressure_sensor_endpoint_id, chip::app::Clusters::PressureMeasurement::Id,
        chip::app::Clusters::PressureMeasurement::Attributes::MeasuredValue::Id, &new_pressure_int);
    if (pressure_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update pressure sensor endpoint with new pressure value");
        return pressure_ret;
    }

    // Update relative humidity if endpoint is enabled
    if (relative_humidity_endpoint_id != nullptr) {
        esp_matter_attr_val_t new_relative_humidity_int =
            esp_matter_nullable_uint16(matter_scale_relative_humidity(new_relative_humidity));
        const esp_err_t relative_humidity_ret = esp_matter::attribute::update(
            *relative_humidity_endpoint_id, chip::app::Clusters::RelativeHumidityMeasurement::Id,
            chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
            &new_relative_humidity_int);
        if (relative_humidity_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update relative humidity sensor endpoint with new relative humidity value");
            return relative_humidity_ret;
        }
    }

    return ESP_OK;
}