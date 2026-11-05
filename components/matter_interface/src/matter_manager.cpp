#include "matter_interface/matter_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

#include <app/clusters/air-quality-server/AirQualityCluster.h>
#include <app/clusters/concentration-measurement-server/ConcentrationMeasurementCluster.h>
#include <app/clusters/illuminance-measurement-server/IlluminanceMeasurementCluster.h>
#include <app/clusters/pressure-measurement-server/PressureMeasurementCluster.h>
#include <app/clusters/relative-humidity-measurement-server/RelativeHumidityMeasurementCluster.h>
#include <app/clusters/temperature-measurement-server/TemperatureMeasurementCluster.h>
#include <app/server-cluster/ServerClusterInterfaceRegistry.h>
#include <data_model_provider/esp_matter_data_model_provider.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_data_model.h>
#include <esp_timer.h>
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
constexpr float matter_carbon_dioxide_concentration_scale = 1.0F;
constexpr float matter_tvoc_concentration_scale = 1.0F;
constexpr int64_t report_max_interval_us =
    static_cast<int64_t>(CONFIG_MATTER_REPORT_MAX_INTERVAL_S) * 1'000'000LL;

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

    if constexpr (std::is_floating_point_v<T>) {
        return nullable<T>(std::clamp(
            value * scale, sensor_value_range.min * scale, sensor_value_range.max * scale));
    } else {
        constexpr float low =
            std::is_signed_v<T> ? static_cast<float>(std::numeric_limits<T>::min()) + 1.0F : 0.0F;
        constexpr float high = std::is_signed_v<T>
                                   ? static_cast<float>(std::numeric_limits<T>::max())
                                   : static_cast<float>(std::numeric_limits<T>::max()) - 1.0F;

        return nullable<T>(static_cast<T>(
            std::lround(std::clamp(value * scale,
                                   std::max(low, sensor_value_range.min * scale),
                                   std::min(high, sensor_value_range.max * scale)))));
    }
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

nullable<float> matterScaleCarbonDioxideConcentration(float carbon_dioxide_concentration_ppm,
                                                      SensorValueRange sensor_value_range) {
    return scaleToMatter<float>(carbon_dioxide_concentration_ppm,
                                matter_carbon_dioxide_concentration_scale,
                                sensor_value_range);
}

nullable<float> matterScaleTvocConcentration(float tvoc_concentration_ppb,
                                             SensorValueRange sensor_value_range) {
    return scaleToMatter<float>(
        tvoc_concentration_ppb, matter_tvoc_concentration_scale, sensor_value_range);
}

nullable<uint16_t> matterScaleIlluminance(float illuminance_lux,
                                          SensorValueRange sensor_value_range) {
    if (!std::isfinite(illuminance_lux)) { return {}; }
    if (illuminance_lux < 1.0F) { return {0}; } // 0 = too low to measure (0 lx)

    const float clamped_lux =
        std::clamp(illuminance_lux, std::max(1.0F, sensor_value_range.min), sensor_value_range.max);
    const float scaled = (10000.0F * std::log10(clamped_lux)) + 1.0F;
    constexpr auto max_matter_val = static_cast<float>(std::numeric_limits<uint16_t>::max() - 1U);
    return {static_cast<uint16_t>(std::lround(std::clamp(scaled, 1.0F, max_matter_val)))};
}

bool shouldSkipReport(const std::optional<float>& value,
                      const SensorValueRange& range,
                      const std::optional<float>& last_reported_value,
                      const int64_t last_report_time_us,
                      const bool has_reported,
                      const int64_t now_us) {
    if (!has_reported || (now_us - last_report_time_us) >= report_max_interval_us) {
        return false; // First report or max reporting interval elapsed
    }
    if (!value.has_value() && !last_reported_value.has_value()) {
        return true; // Still unavailable, skip until max interval elapses
    }
    if (value.has_value() != last_reported_value.has_value()) {
        return false; // Availability changed, report immediately
    }

    const float delta = std::abs(*value - *last_reported_value);
    const float threshold =
        std::max(range.abs_threshold, range.rel_threshold * std::abs(*last_reported_value));
    return delta < threshold;
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

// esp-matter does not yet register ConcentrationMeasurementCluster in data_model::provider.
// One instantiation per concentration cluster id, since the init/shutdown callbacks carry no
// context.
template <uint32_t ClusterId, ConcentrationMeasurement::MeasurementUnitEnum Unit>
struct ConcentrationServerCluster {
    static constexpr auto unit = Unit;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static inline chip::app::LazyRegisteredServerCluster<
        ConcentrationMeasurement::ConcentrationMeasurementCluster>
        server_cluster;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static inline DataModel::Nullable<float> min_measured;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static inline DataModel::Nullable<float> max_measured;

    static void initCallback(chip::EndpointId endpoint_id) {
        if (!server_cluster.IsConstructed()) {
            const ConcentrationMeasurement::ConcentrationMeasurementCluster::Config config{
                .clusterId = ClusterId,
                .features = chip::BitFlags<ConcentrationMeasurement::Feature>(
                    ConcentrationMeasurement::Feature::kNumericMeasurement),
                .medium = ConcentrationMeasurement::MeasurementMediumEnum::kAir,
                .unit = Unit,
                .minMeasured = min_measured,
                .maxMeasured = max_measured,
            };
            server_cluster.Create(endpoint_id, config);
        }
        const CHIP_ERROR err = esp_matter::data_model::provider::get_instance().registry().Register(
            server_cluster.Registration());
        if (err != CHIP_NO_ERROR) {
            ESP_LOGE(tag,
                     "Concentration cluster 0x%04lx register failed ep %u: %s",
                     static_cast<unsigned long>(ClusterId),
                     endpoint_id,
                     err.Format());
        }
    }

    static void shutdownCallback(chip::EndpointId /*endpoint_id*/,
                                 ClusterShutdownType shutdown_type) {
        if (!server_cluster.IsConstructed()) { return; }
        const CHIP_ERROR err =
            esp_matter::data_model::provider::get_instance().registry().Unregister(
                &server_cluster.Cluster(), shutdown_type);
        if (err != CHIP_NO_ERROR) {
            ESP_LOGE(tag,
                     "Concentration cluster 0x%04lx unregister failed: %s",
                     static_cast<unsigned long>(ClusterId),
                     err.Format());
        }
        if (shutdown_type == ClusterShutdownType::kPermanentRemove) { server_cluster.Destroy(); }
    }
};

using Co2ServerCluster =
    ConcentrationServerCluster<CarbonDioxideConcentrationMeasurement::Id,
                               ConcentrationMeasurement::MeasurementUnitEnum::kPpm>;
using TvocServerCluster =
    ConcentrationServerCluster<TotalVolatileOrganicCompoundsConcentrationMeasurement::Id,
                               ConcentrationMeasurement::MeasurementUnitEnum::kPpb>;

using ConcentrationClusterCreateFn =
    esp_matter::cluster_t* (*)(esp_matter::endpoint_t*,
                               esp_matter::cluster::concentration_measurement::config_t*,
                               uint8_t);

// Attaches a numeric concentration measurement cluster to an Air Quality Sensor endpoint.
template <typename ServerCluster>
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
bool attachConcentrationCluster(esp_matter::endpoint_t* endpoint,
                                ConcentrationClusterCreateFn create_fn,
                                const nullable<float> scaled_min,
                                const nullable<float> scaled_max,
                                const char* type_name) {
    if (!isValidScaledRange(scaled_min, scaled_max)) {
        ESP_LOGE(tag, "Min %s cannot be greater than max %s.", type_name, type_name);
        return false;
    }

    esp_matter::cluster::concentration_measurement::config_t cluster_config;

    // IMPORTANT: Matter 1.3 requires explicitly setting Feature Flags for Concentration
    // Measurements. We explicitly enable Numeric Measurement (and optionally Average Measurement if
    // tracked).
    cluster_config.feature_flags =
        esp_matter::cluster::concentration_measurement::feature::numeric_measurement::get_id();

    cluster_config.measurement_medium =
        static_cast<uint8_t>(ConcentrationMeasurement::MeasurementMediumEnum::kAir);
    cluster_config.features.numeric_measurement.measurement_unit =
        static_cast<uint8_t>(ServerCluster::unit);

    // Set initial operational attribute bounds (Values are floating-point types in Matter 1.3+)
    cluster_config.features.numeric_measurement.measured_value = nullable<float>();
    cluster_config.features.numeric_measurement.min_measured_value = scaled_min;
    cluster_config.features.numeric_measurement.max_measured_value = scaled_max;

    // Widen bounds by 1 ULP because ConcentrationMeasurementCluster::IsInRange uses strict > and <.
    ServerCluster::min_measured =
        scaled_min.is_null() ? DataModel::NullNullable
                             : DataModel::MakeNullable(std::nextafter(
                                   scaled_min.value(), -std::numeric_limits<float>::infinity()));
    ServerCluster::max_measured =
        scaled_max.is_null() ? DataModel::NullNullable
                             : DataModel::MakeNullable(std::nextafter(
                                   scaled_max.value(), std::numeric_limits<float>::infinity()));

    esp_matter::cluster_t* cluster =
        create_fn(endpoint, &cluster_config, esp_matter::CLUSTER_FLAG_SERVER);
    if (cluster == nullptr) {
        ESP_LOGE(tag, "Failed to attach %s cluster to endpoint.", type_name);
        return false;
    }

    esp_matter::cluster::set_init_and_shutdown_callbacks(
        cluster, ServerCluster::initCallback, ServerCluster::shutdownCallback);
    ESP_LOGI(tag, "%s cluster attached to air quality endpoint", type_name);
    return true;
}

// Air Quality Sensor endpoint shared by CO2, TVOC and air quality; Matter requires the
// Air Quality cluster on this device type.
esp_matter::endpoint_t* createAirQualityEndpoint(esp_matter::node_t* matter_node) {
    esp_matter::endpoint::air_quality_sensor::config_t aq_endpoint_config;
    auto* endpoint = esp_matter::endpoint::air_quality_sensor::create(
        matter_node, &aq_endpoint_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    if (!finishEndpoint(endpoint, "Air quality")) { return nullptr; }

    if (auto* aq_cluster = esp_matter::cluster::get(endpoint, AirQuality::Id);
        aq_cluster != nullptr) {
        esp_matter::cluster::air_quality::feature::fair::add(aq_cluster);
        esp_matter::cluster::air_quality::feature::moderate::add(aq_cluster);
        esp_matter::cluster::air_quality::feature::very_poor::add(aq_cluster);
        esp_matter::cluster::air_quality::feature::extremely_poor::add(aq_cluster);
    }
    return endpoint;
}

AirQuality::AirQualityEnum toMatterAirQuality(const std::optional<float>& air_quality) {
    if (!air_quality.has_value() || !std::isfinite(*air_quality)) {
        return AirQuality::AirQualityEnum::kUnknown;
    }
    const long level = std::lround(*air_quality);
    if (level < static_cast<long>(AirQualityLevel::UNKNOWN) ||
        level > static_cast<long>(AirQualityLevel::EXTREMELY_POOR)) {
        return AirQuality::AirQualityEnum::kUnknown;
    }
    // AirQualityLevel values match AirQualityEnum.
    return static_cast<AirQuality::AirQualityEnum>(level);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity,readability-function-size)
esp_err_t updateAirQuality(const std::optional<uint16_t> endpoint_id,
                           const std::optional<float> air_quality) {
    if (!endpoint_id.has_value()) { return ESP_OK; }

    auto* cluster = esp_matter::data_model::provider::get_instance().registry().Get(
        ConcreteClusterPath(*endpoint_id, AirQuality::Id));
    if (cluster == nullptr) {
        ESP_LOGE(tag, "Air quality cluster not found for endpoint %u", *endpoint_id);
        return ESP_ERR_NOT_FOUND;
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
    auto* aq_cluster = static_cast<AirQualityCluster*>(cluster);
    const auto new_aq = toMatterAirQuality(air_quality);
    if (aq_cluster->GetAirQuality() == new_aq) { return ESP_OK; }

    if (aq_cluster->SetAirQuality(new_aq) != chip::Protocols::InteractionModel::Status::Success) {
        ESP_LOGE(tag, "Failed to update air quality to %u", static_cast<unsigned int>(new_aq));
        return ESP_ERR_NOT_FINISHED;
    }
    ESP_LOGI(tag, "Updating air quality to %u", static_cast<unsigned int>(new_aq));
    return ESP_OK;
}

std::optional<uint16_t> createIlluminanceSensorEndpoint(esp_matter::node_t* matter_node,
                                                        const nullable<uint16_t> scaled_min,
                                                        const nullable<uint16_t> scaled_max) {
    if (!isValidScaledRange(scaled_min, scaled_max)) {
        ESP_LOGE(tag, "Min illuminance cannot be greater than max illuminance.");
        return std::nullopt;
    }

    esp_matter::endpoint::light_sensor::config_t config;
    config.illuminance_measurement.min_measured_value = scaled_min;
    config.illuminance_measurement.max_measured_value = scaled_max;
    config.illuminance_measurement.measured_value = nullable<uint16_t>();

    return finishEndpoint(esp_matter::endpoint::light_sensor::create(
                              matter_node, &config, esp_matter::ENDPOINT_FLAG_NONE, nullptr),
                          "Illuminance");
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
    if (const auto& range = at<SensorType::TEMPERATURE>(ranges)) {
        auto& measurement = at<SensorType::TEMPERATURE>(measurements_);
        measurement.range = range;
        measurement.endpoint_id = createTemperatureSensorEndpoint(
            matter_node, matterScaleTemp(range->min, *range), matterScaleTemp(range->max, *range));
        if (!measurement.endpoint_id) {
            ESP_LOGE(tag, "Failed to initialize temperature sensor.");
            return ESP_FAIL;
        }
    }

    if (const auto& range = at<SensorType::PRESSURE>(ranges)) {
        auto& measurement = at<SensorType::PRESSURE>(measurements_);
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

    if (const auto& range = at<SensorType::RELATIVE_HUMIDITY>(ranges)) {
        auto& measurement = at<SensorType::RELATIVE_HUMIDITY>(measurements_);
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

    // CO2, TVOC and air quality are independent, but share one Air Quality Sensor endpoint.
    esp_matter::endpoint_t* aq_endpoint = nullptr;
    auto get_aq_endpoint = [&]() -> esp_matter::endpoint_t* {
        if (aq_endpoint == nullptr) { aq_endpoint = createAirQualityEndpoint(matter_node); }
        return aq_endpoint;
    };

    if (const auto& range = at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(ranges)) {
        auto& measurement = at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(measurements_);
        measurement.range = range;
        auto* endpoint = get_aq_endpoint();
        if (endpoint == nullptr ||
            !attachConcentrationCluster<Co2ServerCluster>(
                endpoint,
                esp_matter::cluster::carbon_dioxide_concentration_measurement::create,
                matterScaleCarbonDioxideConcentration(range->min, *range),
                matterScaleCarbonDioxideConcentration(range->max, *range),
                "Carbon dioxide concentration")) {
            ESP_LOGE(tag, "Failed to initialize carbon dioxide concentration sensor.");
            return ESP_FAIL;
        }
        measurement.endpoint_id = esp_matter::endpoint::get_id(endpoint);
    }

    if (const auto& range = at<SensorType::TVOC_CONCENTRATION>(ranges)) {
        auto& measurement = at<SensorType::TVOC_CONCENTRATION>(measurements_);
        measurement.range = range;
        auto* endpoint = get_aq_endpoint();
        if (endpoint == nullptr ||
            !attachConcentrationCluster<TvocServerCluster>(
                endpoint,
                esp_matter::cluster::total_volatile_organic_compounds_concentration_measurement::
                    create,
                matterScaleTvocConcentration(range->min, *range),
                matterScaleTvocConcentration(range->max, *range),
                "TVOC concentration")) {
            ESP_LOGE(tag, "Failed to initialize TVOC concentration sensor.");
            return ESP_FAIL;
        }
        measurement.endpoint_id = esp_matter::endpoint::get_id(endpoint);
    }

    if (const auto& range = at<SensorType::AIR_QUALITY>(ranges)) {
        auto& measurement = at<SensorType::AIR_QUALITY>(measurements_);
        measurement.range = range;
        auto* endpoint = get_aq_endpoint();
        if (endpoint == nullptr) {
            ESP_LOGE(tag, "Failed to initialize air quality sensor.");
            return ESP_FAIL;
        }
        measurement.endpoint_id = esp_matter::endpoint::get_id(endpoint);
    }

    if (const auto& range = at<SensorType::ILLUMINANCE>(ranges)) {
        auto& measurement = at<SensorType::ILLUMINANCE>(measurements_);
        measurement.range = range;
        measurement.endpoint_id =
            createIlluminanceSensorEndpoint(matter_node,
                                            matterScaleIlluminance(range->min, *range),
                                            matterScaleIlluminance(range->max, *range));
        if (!measurement.endpoint_id) {
            ESP_LOGE(tag, "Failed to initialize illuminance sensor.");
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

template <typename Cluster, typename ConvFn>
// NOLINTNEXTLINE(readability-function-cognitive-complexity,readability-function-size)
esp_err_t MatterManager::updateValue(const std::optional<float> value,
                                     Measurement& measurement,
                                     ConvFn value_conv_func,
                                     const char* type_name,
                                     const uint32_t cluster_id) {
    if (!measurement.endpoint_id.has_value()) {
        if (!value.has_value()) { return ESP_OK; } // type not exposed, nothing to publish
        ESP_LOGE(tag, "%s sensor endpoint ID is not set", type_name);
        return ESP_ERR_INVALID_ARG;
    }

    if (!measurement.range.has_value()) {
        ESP_LOGE(tag, "%s sensor value range is not provided", type_name);
        return ESP_ERR_INVALID_ARG;
    }

    auto* cluster = esp_matter::data_model::provider::get_instance().registry().Get(
        ConcreteClusterPath(*measurement.endpoint_id, cluster_id));
    if (cluster == nullptr) {
        ESP_LOGE(tag,
                 "%s cluster not found for endpoint %u",
                 type_name,
                 static_cast<unsigned int>(*measurement.endpoint_id));
        return ESP_ERR_NOT_FOUND;
    }

    const int64_t now_us = esp_timer_get_time();
    if (shouldSkipReport(value,
                         *measurement.range,
                         measurement.last_reported_value,
                         measurement.last_report_time_us,
                         measurement.has_reported,
                         now_us)) {
        ESP_LOGD(tag,
                 "Skipping reporting for %s, value has not changed noticeably since last report",
                 type_name);
        return ESP_OK;
    }

    // An unavailable reading publishes null so clients never see a stale value.
    auto to_matter_val = [&](const std::optional<float>& reading) {
        using ScaledNullable = std::invoke_result_t<ConvFn, float, SensorValueRange>;
        const ScaledNullable scaled =
            reading.has_value() ? value_conv_func(*reading, *measurement.range) : ScaledNullable{};
        return scaled.is_null() ? DataModel::NullNullable : DataModel::MakeNullable(scaled.value());
    };
    const auto matter_val = to_matter_val(value);

    // Keep last_reported_value unchanged if scaled value did not change (e.g. sub-LSB step).
    if (measurement.has_reported && matter_val == to_matter_val(measurement.last_reported_value)) {
        if ((now_us - measurement.last_report_time_us) >= report_max_interval_us) {
            measurement.last_report_time_us = now_us;
        }
        return ESP_OK;
    }

    // Registry entry for this cluster id is registered as exactly `Cluster`.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
    auto* typed_cluster = static_cast<Cluster*>(cluster);

    if (value.has_value()) {
        ESP_LOGI(tag, "Updating %s to %.2f", type_name, *value);
    } else {
        ESP_LOGW(tag, "%s value is not available, publishing null", type_name);
    }

    RETURN_ESP_ERR_ON_CHIP_FAIL(typed_cluster->SetMeasuredValue(matter_val),
                                tag,
                                "Failed to update %s sensor endpoint with new %s value",
                                type_name,
                                type_name);

    measurement.last_reported_value = value;
    measurement.last_report_time_us = now_us;
    measurement.has_reported = true;

    return ESP_OK;
}

esp_err_t MatterManager::updateValues(const SensorReadings& values) {
    esp_matter::lock::ScopedChipStackLock lock(portMAX_DELAY);

    auto ret = ESP_OK;
    auto result =
        updateValue<TemperatureMeasurementCluster>(at<SensorType::TEMPERATURE>(values),
                                                   at<SensorType::TEMPERATURE>(measurements_),
                                                   matterScaleTemp,
                                                   "temperature",
                                                   TemperatureMeasurement::Id);
    if (result != ESP_OK) { ret = ESP_ERR_NOT_FINISHED; }

    result = updateValue<PressureMeasurementCluster>(at<SensorType::PRESSURE>(values),
                                                     at<SensorType::PRESSURE>(measurements_),
                                                     matterScalePressureToKpa,
                                                     "pressure",
                                                     PressureMeasurement::Id);
    if (result != ESP_OK) { ret = ESP_ERR_NOT_FINISHED; }

    result = updateValue<RelativeHumidityMeasurementCluster>(
        at<SensorType::RELATIVE_HUMIDITY>(values),
        at<SensorType::RELATIVE_HUMIDITY>(measurements_),
        matterScaleRelativeHumidity,
        "relative humidity",
        RelativeHumidityMeasurement::Id);
    if (result != ESP_OK) { ret = ESP_ERR_NOT_FINISHED; }

    result = updateValue<ConcentrationMeasurement::ConcentrationMeasurementCluster>(
        at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(values),
        at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(measurements_),
        matterScaleCarbonDioxideConcentration,
        "carbon dioxide (CO2) concentration",
        CarbonDioxideConcentrationMeasurement::Id);
    if (result != ESP_OK) { ret = ESP_ERR_NOT_FINISHED; }

    result = updateValue<ConcentrationMeasurement::ConcentrationMeasurementCluster>(
        at<SensorType::TVOC_CONCENTRATION>(values),
        at<SensorType::TVOC_CONCENTRATION>(measurements_),
        matterScaleTvocConcentration,
        "TVOC concentration",
        TotalVolatileOrganicCompoundsConcentrationMeasurement::Id);
    if (result != ESP_OK) { ret = ESP_ERR_NOT_FINISHED; }

    result = updateAirQuality(at<SensorType::AIR_QUALITY>(measurements_).endpoint_id,
                              at<SensorType::AIR_QUALITY>(values));
    if (result != ESP_OK) { ret = ESP_ERR_NOT_FINISHED; }

    result = updateValue<IlluminanceMeasurementCluster>(at<SensorType::ILLUMINANCE>(values),
                                                        at<SensorType::ILLUMINANCE>(measurements_),
                                                        matterScaleIlluminance,
                                                        "illuminance",
                                                        IlluminanceMeasurement::Id);
    if (result != ESP_OK) { ret = ESP_ERR_NOT_FINISHED; }

    return ret;
}
