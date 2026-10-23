#include "matter_manager.h"

#include <cmath>

#include <app/clusters/pressure-measurement-server/PressureMeasurementCluster.h>
#include <app/clusters/relative-humidity-measurement-server/RelativeHumidityMeasurementCluster.h>
#include <app/clusters/temperature-measurement-server/TemperatureMeasurementCluster.h>
#include <data_model_provider/esp_matter_data_model_provider.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_data_model.h>
#include <esp_timer.h>
#include <setup_payload/OnboardingCodesUtil.h>

#include "matter_pressure_sensor.h"
#include "matter_relative_humidity_sensor.h"
#include "matter_temperature_sensor.h"

// Define the rendezvous mode for Matter commissioning if missing from Kconfig.
// This tells the commissioner (phone) how to connect for the initial Wi-Fi handover.
// 2 = BLE (Bluetooth Low Energy) - Standard for ESP32-C6
// 1 = SoftAP (Device creates a Wi-Fi hotspot)
// 4 = On-Network (Device is already wired to the network)
#ifndef CONFIG_RENDEZVOUS_MODE
constexpr uint8_t CONFIG_RENDEZVOUS_MODE = 2; // NOLINT(readability-identifier-naming)
#endif

#define CHIP_RETURN_ON_FAIL(expr, error_tag, msg)                                                  \
    do {                                                                                           \
        const CHIP_ERROR _macro_err = (expr);                                                      \
        if (_macro_err != CHIP_NO_ERROR) {                                                         \
            ESP_LOGE((error_tag), "%s", (msg));                                                    \
            return ESP_FAIL;                                                                       \
        }                                                                                          \
    } while (0)

using namespace chip::app;
using namespace chip::app::Clusters;

constexpr float matter_temp_scale = 100.0F;
constexpr float matter_relative_humidity_scale = 100.0F;
constexpr float matter_pressure_scale = 10.0F;

constexpr uint32_t led_blink_commissioning_us = 1000 * 1000;
constexpr uint32_t led_blink_identify_us = 200 * 1000;

constexpr const char* tag = "   ***matter_interface***   ";

#if CONFIG_STATUS_LED_GPIO >= 0
static constexpr gpio_num_t k_status_led_gpio = static_cast<gpio_num_t>(CONFIG_STATUS_LED_GPIO);
#    ifdef CONFIG_STATUS_LED_ACTIVE_LOW
static constexpr uint32_t k_led_on_level = 0;
static constexpr uint32_t k_led_off_level = 1;
#    else
static constexpr uint32_t k_led_on_level = 1;
static constexpr uint32_t k_led_off_level = 0;
#    endif

namespace {
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
esp_timer_handle_t s_led_timer = nullptr;
bool s_led_on = false; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
bool s_commissioning_window_open = false;
bool s_identifying = false; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

void statusLedSet(bool is_on) {
    s_led_on = is_on;
    gpio_set_level(k_status_led_gpio, is_on ? k_led_on_level : k_led_off_level);
}

void statusLedTimerCallback(void* /*arg*/) {
    statusLedSet(!s_led_on);
}

void statusLedUpdateState() {
    if (s_led_timer == nullptr) { return; }
    esp_timer_stop(s_led_timer);
    if (s_identifying) {
        esp_timer_start_periodic(s_led_timer, led_blink_identify_us);
    } else if (s_commissioning_window_open) {
        esp_timer_start_periodic(s_led_timer, led_blink_commissioning_us);
    } else {
        statusLedSet(true);
    }
}

void statusLedInit() {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << k_status_led_gpio);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    statusLedSet(false);

    esp_timer_create_args_t timer_args = {};
    timer_args.callback = &statusLedTimerCallback;
    timer_args.name = "status_led";
    esp_timer_create(&timer_args, &s_led_timer);
}
#else
void statusLedInit() {}
void statusLedUpdateState() {}
#endif

/**
 * @brief Handle Matter device events.
 *
 * This callback is invoked to process various events related to the Matter stack.
 *
 * @param[in] event The Matter device event structure.
 * @param[in] arg   Additional argument provided by the caller.
 */
// NOLINTNEXTLINE(readability-function-cognitive-complexity,readability-function-size)
void matterEventCallback(const ChipDeviceEvent* event, intptr_t /*arg*/) {
    switch (event->Type) {
    // Signals a change in connectivity of the device's Wi-Fi station interface.
    case chip::DeviceLayer::DeviceEventType::kWiFiConnectivityChange:
        ESP_LOGI(tag, "Wi-Fi connectivity change");
        break;

    // Signals a change in connectivity of the device's Thread interface.
    case chip::DeviceLayer::DeviceEventType::kThreadConnectivityChange:
        ESP_LOGI(tag, "Thread connectivity change");
        break;

    // Signals a change in the device's ability to communicate via the internet.
    case chip::DeviceLayer::DeviceEventType::kInternetConnectivityChange:
        ESP_LOGI(tag, "Internet connectivity change");
        break;

    // Signals a change in the device's ability to communicate with a chip-enabled service.
    case chip::DeviceLayer::DeviceEventType::kServiceConnectivityChange:
        ESP_LOGI(tag, "Service connectivity change");
        break;

    // Signals a change to the device's service provisioning state.
    case chip::DeviceLayer::DeviceEventType::kServiceProvisioningChange:
        ESP_LOGI(tag, "Service provisioning change");
        break;

    // Signals a change to the device's real-time clock synchronization state.
    case chip::DeviceLayer::DeviceEventType::kTimeSyncChange:
        ESP_LOGI(tag, "Time synchronization change");
        break;

    // Signals that an external entity has established a new CHIPoBLE connection with the device.
    case chip::DeviceLayer::DeviceEventType::kCHIPoBLEConnectionEstablished:
        ESP_LOGI(tag, "BLE connection established");
        break;

    // Signals that an external entity has closed an existing CHIPoBLE connection with the device.
    case chip::DeviceLayer::DeviceEventType::kCHIPoBLEConnectionClosed:
        ESP_LOGI(tag, "BLE connection closed");
        break;

    // Request to close all BLE connections when concurrent connections are not supported.
    case chip::DeviceLayer::DeviceEventType::kCloseAllBleConnections:
        ESP_LOGI(tag, "Close all BLE connections requested");
        break;

    // Indicates that the Wi-Fi device is available for connection.
    case chip::DeviceLayer::DeviceEventType::kWiFiDeviceAvailable:
        ESP_LOGI(tag, "Wi-Fi device is available");
        break;

    // Indicates that the operational network has started.
    case chip::DeviceLayer::DeviceEventType::kOperationalNetworkStarted:
        ESP_LOGI(tag, "Operational network started");
        break;

    // Signals a state change in the Thread stack.
    case chip::DeviceLayer::DeviceEventType::kThreadStateChange:
        ESP_LOGI(tag, "Thread state change");
        break;

    // Indicates a change in the state of the Thread network interface.
    case chip::DeviceLayer::DeviceEventType::kThreadInterfaceStateChange:
        ESP_LOGI(tag, "Thread interface state change");
        break;

    // Indicates a change in the CHIPoBLE advertising state.
    case chip::DeviceLayer::DeviceEventType::kCHIPoBLEAdvertisingChange:
        ESP_LOGI(tag, "CHIPoBLE advertising state change");
        break;

    // Indicates that an IP address (IPv4 or IPv6) has changed for the interface.
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(tag, "Interface IP address changed");
        break;

    // Signals that commissioning has completed via the general commissioning cluster command.
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(tag, "Commissioning complete");
#if CONFIG_STATUS_LED_GPIO >= 0
        s_commissioning_window_open = false;
        statusLedUpdateState();
#endif
        break;

    // Indicates that the fail-safe timer expired before the CommissioningComplete command was
    // invoked.
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(tag, "Commissioning failed, fail-safe timer expired");
        break;

    // Indicates that the operational network is enabled.
    case chip::DeviceLayer::DeviceEventType::kOperationalNetworkEnabled:
        ESP_LOGI(tag, "Operational network enabled");
        break;

    // Signals that DNS-SD has been initialized and is ready to operate.
    case chip::DeviceLayer::DeviceEventType::kDnssdInitialized:
        ESP_LOGI(tag, "DNS-SD initialized");
        break;

    // Indicates that the DNS-SD backend was restarted and services must be published again.
    case chip::DeviceLayer::DeviceEventType::kDnssdRestartNeeded:
        ESP_LOGI(tag, "DNS-SD restart needed");
        break;

    // Indicates that bindings were updated via cluster.
    case chip::DeviceLayer::DeviceEventType::kBindingsChangedViaCluster:
        ESP_LOGI(tag, "Bindings updated via cluster");
        break;

    // Indicates a change in the OTA engine state.
    case chip::DeviceLayer::DeviceEventType::kOtaStateChanged:
        ESP_LOGI(tag, "OTA state changed");
        break;

    // Indicates that server initialization has completed, and the node is ready for connections.
    case chip::DeviceLayer::DeviceEventType::kServerReady:
        ESP_LOGI(tag, "Server is ready");
        break;

    // Signals that BLE has been deinitialized.
    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(tag, "BLE deinitialized");
        break;

    // Signals that a commissioning session has started.
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(tag, "Commissioning session started");
        break;

    // Signals that a commissioning session has stopped.
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(tag, "Commissioning session stopped");
        break;

    // Indicates that the commissioning window is now open.
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(tag, "Commissioning window opened");
#if CONFIG_STATUS_LED_GPIO >= 0
        s_commissioning_window_open = true;
        statusLedUpdateState();
#endif
        break;

    // Indicates that the commissioning window is now closed.
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(tag, "Commissioning window closed");
#if CONFIG_STATUS_LED_GPIO >= 0
        s_commissioning_window_open = false;
        statusLedUpdateState();
#endif
        break;

    // Indicates that a fabric is about to be removed.
    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(tag, "Fabric will be removed");
        break;

    // Indicates that a fabric has been removed.
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(tag, "Fabric removed successfully");
        break;

    // Indicates that a fabric in the Fabric Table has been committed to storage.
    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(tag, "Fabric committed to storage");
        break;

    // Signals that operational credentials have been updated.
    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(tag, "Fabric updated");
        break;

    default:
        ESP_LOGI(tag, "Unhandled Matter event type: %d", event->Type);
        break;
    }
}

/**
 * @brief Handle identification cluster commands for a Matter endpoint.
 *
 * This callback is used to execute identification effects, such as blinking lights or beeping,
 * based on the effect ID and variant provided by the cluster command.
 *
 * @param[in] type           The type of identification callback (e.g., start or stop).
 * @param[in] endpoint_id    The endpoint ID where the command is received.
 * @param[in] effect_id      The effect ID specifying the type of identification effect.
 * @param[in] effect_variant The effect variant for fine-tuned behavior.
 * @param[in] priv_data      User-defined private data.
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_ARG if arguments are invalid.
 *      - Other error codes on failure.
 */
esp_err_t identificationCallback(esp_matter::identification::callback_type_t const type,
                                 uint16_t const endpoint_id,
                                 uint8_t const effect_id,
                                 uint8_t const effect_variant,
                                 void* /*priv_data*/) {
    ESP_LOGI(
        tag,
        "Identification Callback Invoked: type=%d, endpoint_id=%u, effect_id=%u, effect_variant=%u",
        type,
        (unsigned int)endpoint_id,
        (unsigned int)effect_id,
        (unsigned int)effect_variant);
#if CONFIG_STATUS_LED_GPIO >= 0
    if (type == esp_matter::identification::START || type == esp_matter::identification::EFFECT) {
        s_identifying = true;
    } else if (type == esp_matter::identification::STOP) {
        s_identifying = false;
    }
    statusLedUpdateState();
#endif
    return ESP_OK;
}

/**
 * @brief Handle attribute updates for Matter clusters.
 *
 * This callback is invoked when an attribute value is updated by the Matter framework.
 *
 * @param[in] type        The type of attribute update (e.g., write or read).
 * @param[in] endpoint_id The endpoint ID where the update occurred.
 * @param[in] cluster_id  The cluster ID associated with the attribute.
 * @param[in] attribute_id The attribute ID being updated.
 * @param[in] val          Pointer to the updated attribute value.
 * @param[in] priv_data    User-defined private data.
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_ARG if arguments are invalid.
 *      - Other error codes on failure.
 */
esp_err_t attributeUpdateCallback(const esp_matter::attribute::callback_type_t type,
                                  const uint16_t endpoint_id,
                                  const uint32_t cluster_id,
                                  const uint32_t attribute_id,
                                  esp_matter_attr_val_t* /*val*/,
                                  void* /*priv_data*/) {
    ESP_LOGI(tag,
             "Attribute update: type=%d, endpoint_id=%u, cluster_id=%u, attribute_id=%u",
             type,
             (unsigned int)endpoint_id,
             (unsigned int)cluster_id,
             (unsigned int)attribute_id);
    return ESP_OK;
}

inline int16_t matterScaleTemp(float value) {
    // NOLINTNEXTLINE(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
    return static_cast<int16_t>(std::round(value * matter_temp_scale));
}

inline int16_t matterScalePressureToKpa(float value_pa) {
    // NOLINTNEXTLINE(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
    return static_cast<int16_t>(std::round(value_pa / 1000 * matter_pressure_scale));
}

inline uint16_t matterScaleRelativeHumidity(float value) {
    // NOLINTNEXTLINE(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
    return static_cast<uint16_t>(std::round(value * matter_relative_humidity_scale));
}
} // namespace

class RAIIUpdateMatterValues {
  public:
    RAIIUpdateMatterValues() { // NOLINT(modernize-use-equals-default)
        chip::DeviceLayer::PlatformMgr().LockChipStack();
    }
    ~RAIIUpdateMatterValues() { // NOLINT(modernize-use-equals-default)
        chip::DeviceLayer::PlatformMgr().UnlockChipStack();
    }

    // Prevent copying
    RAIIUpdateMatterValues(const RAIIUpdateMatterValues&) = delete;
    RAIIUpdateMatterValues& operator=(const RAIIUpdateMatterValues&) = delete;

    // Prevent moving
    RAIIUpdateMatterValues(RAIIUpdateMatterValues&&) = delete;
    RAIIUpdateMatterValues& operator=(RAIIUpdateMatterValues&&) = delete;

    // NOLINTNEXTLINE(readability-convert-member-functions-to-static,readability-function-cognitive-complexity)
    esp_err_t updateValues(const uint16_t temperature_endpoint_id,
                           const uint16_t pressure_sensor_endpoint_id,
                           const uint16_t* relative_humidity_endpoint_id,
                           const float new_temperature,
                           const float new_pressure,
                           const float new_relative_humidity) {
        // Update temperature
        auto* temperature_cluster = esp_matter::data_model::provider::get_instance().registry().Get(
            ConcreteClusterPath(temperature_endpoint_id, TemperatureMeasurement::Id));
        if (temperature_cluster != nullptr) {
            CHIP_RETURN_ON_FAIL(
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
                static_cast<TemperatureMeasurementCluster*>(temperature_cluster)
                    ->SetMeasuredValue(
                        DataModel::MakeNullable<int16_t>((matterScaleTemp(new_temperature)))),
                tag,
                "Failed to update temperature sensor endpoint with new temperature value");
        }

        // Update pressure
        auto* pressure_cluster = esp_matter::data_model::provider::get_instance().registry().Get(
            ConcreteClusterPath(pressure_sensor_endpoint_id, PressureMeasurement::Id));
        if (pressure_cluster != nullptr) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
            CHIP_RETURN_ON_FAIL(
                static_cast<PressureMeasurementCluster*>(pressure_cluster)
                    ->SetMeasuredValue(
                        DataModel::MakeNullable<int16_t>((matterScalePressureToKpa(new_pressure)))),
                tag,
                "Failed to update pressure sensor endpoint with new pressure value");
        }

        // Update relative humidity if endpoint is enabled
        if (relative_humidity_endpoint_id != nullptr) {
            auto* relative_humidity_cluster =
                esp_matter::data_model::provider::get_instance().registry().Get(ConcreteClusterPath(
                    *relative_humidity_endpoint_id, RelativeHumidityMeasurement::Id));
            if (relative_humidity_cluster != nullptr) {
                CHIP_RETURN_ON_FAIL(
                    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
                    static_cast<RelativeHumidityMeasurementCluster*>(relative_humidity_cluster)
                        ->SetMeasuredValue(DataModel::MakeNullable<uint16_t>(
                            (matterScaleRelativeHumidity(new_relative_humidity)))),
                    tag,
                    "Failed to update relative humidity sensor endpoint with new relative humidity "
                    "value");
            }
        }

        return ESP_OK;
    }
};

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t matterInit(
    uint16_t* temperature_sensor_endpoint_id,
    uint16_t* pressure_sensor_endpoint_id,
    uint16_t* relative_humidity_sensor_endpoint_id, // NOLINT(readability-non-const-parameter)
    const float min_temperature,
    const float max_temperature,
    const float initial_temperature,
    const float min_pressure,
    const float max_pressure,
    const float initial_pressure,
    const float min_relative_humidity,
    const float max_relative_humidity,
    const float initial_relative_humidity) {
    statusLedInit();

    // Initialize Matter Node, where
    // - node_config is the configuration object for the nodeset_callback
    // - app_attribute_update_cb is the callback for handling attribute updates for the temperature
    // sensor
    // - app_identification_cb is the callback for handling identification queries
    esp_matter::node::config_t node_config;
    esp_matter::node_t* matter_node =
        esp_matter::node::create(&node_config, attributeUpdateCallback, identificationCallback);
    if (matter_node == nullptr) {
        ESP_LOGE(tag, "Failed to create Matter node.");
        return ESP_FAIL;
    }
    ESP_LOGI(tag, "Matter node created");

    // Initialize Matter temperature sensor endpoint
    if (createTemperatureSensorEndpoint(matter_node,
                                        temperature_sensor_endpoint_id,
                                        matterScaleTemp(min_temperature),
                                        matterScaleTemp(max_temperature),
                                        matterScaleTemp(initial_temperature)) != ESP_OK) {
        ESP_LOGE(tag, "Failed to initialize temperature sensor.");
        return ESP_FAIL;
    }
    ESP_LOGI(tag, "Temperature sensor endpoint initialized");

    // Initialize Matter pressure sensor endpoint
    // Pass scaled values to the endpoint creation function
    if (createPressureSensorEndpoint(matter_node,
                                     pressure_sensor_endpoint_id,
                                     matterScalePressureToKpa(min_pressure),
                                     matterScalePressureToKpa(max_pressure),
                                     matterScalePressureToKpa(initial_pressure)) != ESP_OK) {
        ESP_LOGE(tag, "Failed to initialize pressure sensor");
        return ESP_FAIL;
    }

    // Initialize Matter relative humidity sensor endpoint (optional, only when sensor supports
    // humidity)
    if (relative_humidity_sensor_endpoint_id != nullptr) {
        if (createRelativeHumiditySensorEndpoint(
                matter_node,
                relative_humidity_sensor_endpoint_id,
                matterScaleRelativeHumidity(min_relative_humidity),
                matterScaleRelativeHumidity(max_relative_humidity),
                matterScaleRelativeHumidity(initial_relative_humidity)) != ESP_OK) {
            ESP_LOGE(tag, "Failed to initialize relative humidity sensor.");
            return ESP_FAIL;
        }
        ESP_LOGI(tag, "Relative humidity sensor endpoint initialized");
    }

    // Start the Matter thread, using an optional event callback with private data.
    // It returns `ESP_OK` if successful or an error if it fails.
    esp_err_t matter_err = esp_matter::start(matterEventCallback);
    if (matter_err != ESP_OK) {
        ESP_LOGE(tag, "Failed to start Matter, error: %d", matter_err);
        return matter_err;
    }
    ESP_LOGI(tag, "Matter started");
    PrintOnboardingCodes(chip::RendezvousInformationFlags(CONFIG_RENDEZVOUS_MODE));

    statusLedUpdateState();

    // Console shell is helpful when developing/debugging the application.
    // Set CONFIG_ENABLE_CHIP_SHELL=n in sdkconfig.defaults in production.
    // https://docs.espressif.com/projects/esp-matter/en/latest/esp32/optimizations.html
#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
#    if CONFIG_OPENTHREAD_CLI
    console::otcli_register_commands();
#    endif
    esp_matter::console::init();
#endif

    return ESP_OK;
}

esp_err_t matterUpdateValues(const uint16_t temperature_endpoint_id,
                             const uint16_t pressure_sensor_endpoint_id,
                             const uint16_t* relative_humidity_endpoint_id,
                             const float new_temperature,
                             const float new_pressure,
                             const float new_relative_humidity) {
    return RAIIUpdateMatterValues().updateValues(temperature_endpoint_id,
                                                 pressure_sensor_endpoint_id,
                                                 relative_humidity_endpoint_id,
                                                 new_temperature,
                                                 new_pressure,
                                                 new_relative_humidity);
}