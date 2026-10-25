#include "matter_interface/matter_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

#include <app/clusters/pressure-measurement-server/PressureMeasurementCluster.h>
#include <app/clusters/relative-humidity-measurement-server/RelativeHumidityMeasurementCluster.h>
#include <app/clusters/temperature-measurement-server/TemperatureMeasurementCluster.h>
#include <data_model_provider/esp_matter_data_model_provider.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_data_model.h>
#include <setup_payload/OnboardingCodesUtil.h>

// Define the rendezvous mode for Matter commissioning if missing from Kconfig.
// This tells the commissioner (phone) how to connect for the initial Wi-Fi handover.
// 2 = BLE (Bluetooth Low Energy) - Standard for ESP32-C6
// 1 = SoftAP (Device creates a Wi-Fi hotspot)
// 4 = On-Network (Device is already wired to the network)
#ifndef CONFIG_RENDEZVOUS_MODE
constexpr uint8_t CONFIG_RENDEZVOUS_MODE = 2; // NOLINT(readability-identifier-naming)
#endif

// Returns ESP_ERR_NOT_FINISHED and logs fmt plus the CHIP error when expr fails.
#define RETURN_ESP_ERR_ON_CHIP_FAIL(expr, error_tag, fmt, ...)                                     \
    do {                                                                                           \
        const CHIP_ERROR chip_err = (expr);                                                        \
        if (chip_err != CHIP_NO_ERROR) {                                                           \
            ESP_LOGE((error_tag), fmt ": %s", ##__VA_ARGS__, chip_err.Format());                   \
            return ESP_ERR_NOT_FINISHED;                                                           \
        }                                                                                          \
    } while (0)

using namespace chip::app;
using namespace chip::app::Clusters;

constexpr float matter_temp_scale = 100.0F;
constexpr float matter_relative_humidity_scale = 100.0F;
constexpr float matter_pressure_scale = 0.01F; // Scale - 10.0F and PA -> kPa => 10/1000

constexpr const char* tag = "   ***matter_interface***   ";

namespace {
// Set by MatterManager::init(); the Matter callbacks carry no context we can rely on.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
const MatterManager::Listeners* s_listeners = nullptr;

void notifyCommissioningWindow(bool open) {
    if (s_listeners != nullptr && s_listeners->on_commissioning_window) {
        s_listeners->on_commissioning_window(open);
    }
}

void notifyIdentify(bool identifying) {
    if (s_listeners != nullptr && s_listeners->on_identify) {
        s_listeners->on_identify(identifying);
    }
}

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
        notifyCommissioningWindow(false);
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
        notifyCommissioningWindow(true);
        break;

    // Indicates that the commissioning window is now closed.
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(tag, "Commissioning window closed");
        notifyCommissioningWindow(false);
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

    // Signals that secure session is established.
    case chip::DeviceLayer::DeviceEventType::kSecureSessionEstablished:
        ESP_LOGI(tag, "Secure session is established");
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
    if (type == esp_matter::identification::START || type == esp_matter::identification::EFFECT) {
        notifyIdentify(true);
    } else if (type == esp_matter::identification::STOP) {
        notifyIdentify(false);
    }
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

// Scale a physical value to a Matter integer attribute, saturating at the type limits.
// Matter reserves one extreme as the "null" sentinel (INT16_MIN for signed, 0xFFFF for
// unsigned), so stay one step inside it.
template <typename T>
nullable<T> scaleToMatter(float value, float scale, SensorValueRange sensor_value_range) {
    if (!std::isfinite(value)) {
        return nullable<T>();
    } // NaN/inf -> null; clamp(NaN) would still be UB on cast

    constexpr float low =
        std::is_signed_v<T> ? static_cast<float>(std::numeric_limits<T>::min()) + 1.0F : 0.0F;
    constexpr float high = std::is_signed_v<T>
                               ? static_cast<float>(std::numeric_limits<T>::max())
                               : static_cast<float>(std::numeric_limits<T>::max()) - 1.0F;

    return nullable<T>(
        static_cast<T>(std::lround(std::clamp(value * scale,
                                              std::max(low, sensor_value_range.min * scale),
                                              std::min(high, sensor_value_range.max * scale)))));
}

nullable<int16_t> matterScaleTemp(float temp_c, SensorValueRange sensor_value_range) {
    return scaleToMatter<int16_t>(temp_c, matter_temp_scale, sensor_value_range);
}

nullable<int16_t> matterScalePressureToKpa(float pressure_pa, SensorValueRange sensor_value_range) {
    return scaleToMatter<int16_t>(pressure_pa, matter_pressure_scale, sensor_value_range);
}

nullable<uint16_t> matterScaleRelativeHumidity(float relative_humidity_pct,
                                               SensorValueRange sensor_value_range) {
    return scaleToMatter<uint16_t>(
        relative_humidity_pct, matter_relative_humidity_scale, sensor_value_range);
}

template <typename Cluster, typename ConvFn>
esp_err_t
updateValue(const std::optional<float> value, // NOLINT(readability-function-cognitive-complexity)
            const std::optional<uint16_t> endpoint_id,
            const std::optional<SensorValueRange> range,
            ConvFn value_conv_func,
            const char* type_name,
            esp_matter::data_model::ClusterId cluster_id) {
    if (!endpoint_id.has_value()) {
        if (!value.has_value()) { return ESP_OK; } // type not exposed, nothing to publish
        ESP_LOGE(tag, "%s sensor endpoint ID is not set", type_name);
        return ESP_ERR_INVALID_ARG;
    }

    if (!range.has_value()) {
        ESP_LOGE(tag, "%s sensor value range is not provided", type_name);
        return ESP_ERR_INVALID_ARG;
    }

    auto* cluster = esp_matter::data_model::provider::get_instance().registry().Get(
        ConcreteClusterPath(*endpoint_id, cluster_id));
    if (cluster == nullptr) {
        ESP_LOGE(tag,
                 "%s cluster not found for endpoint %u",
                 type_name,
                 static_cast<unsigned int>(*endpoint_id));
        return ESP_ERR_NOT_FOUND;
    }

    // An unavailable reading publishes null so clients never see a stale value.
    using Nullable = std::invoke_result_t<ConvFn, float, SensorValueRange>;
    const Nullable val = value.has_value() ? value_conv_func(*value, *range) : Nullable{};
    if (value.has_value()) {
        ESP_LOGD(tag, "Updating %s to %.2f", type_name, *value);
    } else {
        ESP_LOGW(tag, "%s value is not available, publishing null", type_name);
    }

    RETURN_ESP_ERR_ON_CHIP_FAIL(
        // Registry entry for this cluster id is registered as exactly `Cluster`.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
        static_cast<Cluster*>(cluster)->SetMeasuredValue(
            val.is_null() ? DataModel::NullNullable : DataModel::MakeNullable(val.value())),
        tag,
        "Failed to update %s sensor endpoint with new %s value",
        type_name,
        type_name);

    return ESP_OK;
}

template <typename T> bool isValidScaledRange(const nullable<T>& min, const nullable<T>& max) {
    return min.is_null() || max.is_null() || min.value() <= max.value();
}

// Reads the id of a created endpoint and adds the binding cluster.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::optional<uint16_t> finishEndpoint(esp_matter::endpoint_t* endpoint, const char* type_name) {
    if (endpoint == nullptr) {
        ESP_LOGE(tag, "Failed to create %s sensor endpoint.", type_name);
        return std::nullopt;
    }

    const uint16_t endpoint_id = esp_matter::endpoint::get_id(endpoint);
    ESP_LOGI(tag,
             "%s sensor endpoint created with id %u",
             type_name,
             static_cast<unsigned int>(endpoint_id));

    esp_matter::cluster::binding::config_t config;
    if (esp_matter::cluster::binding::create(endpoint, &config, esp_matter::CLUSTER_FLAG_SERVER) ==
        nullptr) {
        ESP_LOGW(tag, "Failed to add binding cluster to %s sensor endpoint", type_name);
    }
    return endpoint_id;
}

std::optional<uint16_t> createTemperatureSensorEndpoint(esp_matter::node_t* matter_node,
                                                        const nullable<int16_t> scaled_min,
                                                        const nullable<int16_t> scaled_max) {
    if (!isValidScaledRange(scaled_min, scaled_max)) {
        ESP_LOGE(tag, "Min temperature cannot be greater than max temperature.");
        return std::nullopt;
    }

    esp_matter::endpoint::temperature_sensor::config_t config;
    config.temperature_measurement.min_measured_value = scaled_min;
    config.temperature_measurement.max_measured_value = scaled_max;
    config.temperature_measurement.measured_value = nullable<int16_t>();

    return finishEndpoint(esp_matter::endpoint::temperature_sensor::create(
                              matter_node, &config, esp_matter::ENDPOINT_FLAG_NONE, nullptr),
                          "Temperature");
}

std::optional<uint16_t> createPressureSensorEndpoint(esp_matter::node_t* matter_node,
                                                     const nullable<int16_t> scaled_min,
                                                     const nullable<int16_t> scaled_max) {
    if (!isValidScaledRange(scaled_min, scaled_max)) {
        ESP_LOGE(tag, "Min pressure cannot be greater than max pressure.");
        return std::nullopt;
    }

    esp_matter::endpoint::pressure_sensor::config_t config;
    config.pressure_measurement.min_measured_value = scaled_min;
    config.pressure_measurement.max_measured_value = scaled_max;
    config.pressure_measurement.measured_value = nullable<int16_t>();

    return finishEndpoint(esp_matter::endpoint::pressure_sensor::create(
                              matter_node, &config, esp_matter::ENDPOINT_FLAG_NONE, nullptr),
                          "Pressure");
}

std::optional<uint16_t> createRelativeHumiditySensorEndpoint(esp_matter::node_t* matter_node,
                                                             const nullable<uint16_t> scaled_min,
                                                             const nullable<uint16_t> scaled_max) {
    if (!isValidScaledRange(scaled_min, scaled_max)) {
        ESP_LOGE(tag, "Min relative humidity cannot be greater than max relative humidity.");
        return std::nullopt;
    }

    esp_matter::endpoint::humidity_sensor::config_t config;
    config.relative_humidity_measurement.min_measured_value = scaled_min;
    config.relative_humidity_measurement.max_measured_value = scaled_max;
    config.relative_humidity_measurement.measured_value = nullable<uint16_t>();

    return finishEndpoint(esp_matter::endpoint::humidity_sensor::create(
                              matter_node, &config, esp_matter::ENDPOINT_FLAG_NONE, nullptr),
                          "Relative humidity");
}
} // namespace

esp_err_t
/// NOLINTNEXTLINE(readability-function-cognitive-complexity)
MatterManager::init(const SensorRanges& ranges) {
    s_listeners = &listeners_;

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

    // Create an endpoint for every type a detected sensor provides.
    if (const auto& range = ranges[toIndex(SensorType::TEMPERATURE)]) {
        auto& measurement = measurements_[toIndex(SensorType::TEMPERATURE)];
        measurement.range = range;
        measurement.endpoint_id = createTemperatureSensorEndpoint(
            matter_node, matterScaleTemp(range->min, *range), matterScaleTemp(range->max, *range));
        if (!measurement.endpoint_id) {
            ESP_LOGE(tag, "Failed to initialize temperature sensor.");
            return ESP_FAIL;
        }
    }

    if (const auto& range = ranges[toIndex(SensorType::PRESSURE)]) {
        auto& measurement = measurements_[toIndex(SensorType::PRESSURE)];
        measurement.range = range;
        measurement.endpoint_id =
            createPressureSensorEndpoint(matter_node,
                                         matterScalePressureToKpa(range->min, *range),
                                         matterScalePressureToKpa(range->max, *range));
        if (!measurement.endpoint_id) {
            ESP_LOGE(tag, "Failed to initialize pressure sensor.");
            return ESP_FAIL;
        }
    }

    if (const auto& range = ranges[toIndex(SensorType::RELATIVE_HUMIDITY)]) {
        auto& measurement = measurements_[toIndex(SensorType::RELATIVE_HUMIDITY)];
        measurement.range = range;
        measurement.endpoint_id =
            createRelativeHumiditySensorEndpoint(matter_node,
                                                 matterScaleRelativeHumidity(range->min, *range),
                                                 matterScaleRelativeHumidity(range->max, *range));
        if (!measurement.endpoint_id) {
            ESP_LOGE(tag, "Failed to initialize relative humidity sensor.");
            return ESP_FAIL;
        }
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

    // Console shell is helpful when developing/debugging the application.
    // Set CONFIG_ENABLE_CHIP_SHELL=n in sdkconfig.defaults in production.
    // https://docs.espressif.com/projects/esp-matter/en/latest/esp32/optimizations.html
#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
#    if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#    endif
    esp_matter::console::init();
#endif

    return ESP_OK;
}

esp_err_t MatterManager::updateValues(const SensorReadings& values) {
    esp_matter::lock::ScopedChipStackLock lock(portMAX_DELAY);

    const auto& temperature = measurements_[toIndex(SensorType::TEMPERATURE)];
    const auto& pressure = measurements_[toIndex(SensorType::PRESSURE)];
    const auto& humidity = measurements_[toIndex(SensorType::RELATIVE_HUMIDITY)];

    auto ret = ESP_OK;
    auto result =
        updateValue<TemperatureMeasurementCluster>(values[toIndex(SensorType::TEMPERATURE)],
                                                   temperature.endpoint_id,
                                                   temperature.range,
                                                   matterScaleTemp,
                                                   "temperature",
                                                   TemperatureMeasurement::Id);
    if (result != ESP_OK) { ret = ESP_ERR_NOT_FINISHED; }

    result = updateValue<PressureMeasurementCluster>(values[toIndex(SensorType::PRESSURE)],
                                                     pressure.endpoint_id,
                                                     pressure.range,
                                                     matterScalePressureToKpa,
                                                     "pressure",
                                                     PressureMeasurement::Id);
    if (result != ESP_OK) { ret = ESP_ERR_NOT_FINISHED; }

    result = updateValue<RelativeHumidityMeasurementCluster>(
        values[toIndex(SensorType::RELATIVE_HUMIDITY)],
        humidity.endpoint_id,
        humidity.range,
        matterScaleRelativeHumidity,
        "relative humidity",
        RelativeHumidityMeasurement::Id);
    if (result != ESP_OK) { ret = ESP_ERR_NOT_FINISHED; }

    return ret;
}
