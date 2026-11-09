#include "matter_interface/matter_manager.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <type_traits>
#include <vector>

#include <app/clusters/air-quality-server/AirQualityCluster.h>
#include <app/clusters/concentration-measurement-server/ConcentrationMeasurementCluster.h>
#include <app/clusters/illuminance-measurement-server/IlluminanceMeasurementCluster.h>
#include <app/clusters/occupancy-sensor-server/OccupancySensingCluster.h>
#include <app/clusters/power-source-server/power-source-server.h>
#include <app/clusters/pressure-measurement-server/PressureMeasurementCluster.h>
#include <app/clusters/relative-humidity-measurement-server/RelativeHumidityMeasurementCluster.h>
#include <app/clusters/resource-monitoring-server/ResourceMonitoringCluster.h>
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
// Extended feature: ScaledValue = 10^Scale x pressure in Pa; Scale -1 gives 10 Pa (0.1 hPa) steps.
constexpr int8_t matter_pressure_scale_exponent = -1;
constexpr float matter_pressure_scaled_scale = 0.1F; // Pa -> ScaledValue
constexpr float matter_carbon_dioxide_concentration_scale = 1.0F;
constexpr float matter_tvoc_concentration_scale = 1.0F;
constexpr float matter_battery_percent_scale = 2.0F;    // BatPercentRemaining is in half percent
constexpr float matter_battery_voltage_scale = 1000.0F; // V -> mV
constexpr int64_t report_max_interval_us =
    static_cast<int64_t>(CONFIG_MATTER_REPORT_MAX_INTERVAL_S) * 1'000'000LL;

constexpr const char* tag = "matter_manager";

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

// Remaining level endpoint, for attaching the reset delegate once the server is up.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::optional<uint16_t> s_remaining_level_endpoint_id;

// Registry entry for HEPA Filter Monitoring is exactly a ResourceMonitoringCluster.
ResourceMonitoring::ResourceMonitoringCluster* getRemainingLevelCluster(uint16_t endpoint_id) {
    auto* cluster = esp_matter::data_model::provider::get_instance().registry().Get(
        ConcreteClusterPath(endpoint_id, HepaFilterMonitoring::Id));
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
    return static_cast<ResourceMonitoring::ResourceMonitoringCluster*>(cluster);
}

// The SDK resets Condition to 100; the listener re-teaches the "full" point.
class RemainingLevelResetDelegate : public ResourceMonitoring::Delegate {
  public:
    CHIP_ERROR Init() override {
        return CHIP_NO_ERROR;
    }
    chip::Protocols::InteractionModel::Status PostResetCondition() override {
        ESP_LOGI(tag, "Remaining level reset requested");
        if (s_listeners != nullptr && s_listeners->on_remaining_level_reset) {
            s_listeners->on_remaining_level_reset();
        }
        return chip::Protocols::InteractionModel::Status::Success;
    }
};

// Must outlive the cluster; the cluster dereferences it on every ResetCondition.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
RemainingLevelResetDelegate s_remaining_level_delegate;

// Clusters are constructed during server init, so this runs on kServerReady.
void attachRemainingLevelDelegate() {
    if (!s_remaining_level_endpoint_id.has_value()) { return; }
    auto* cluster = getRemainingLevelCluster(*s_remaining_level_endpoint_id);
    if (cluster == nullptr || cluster->SetDelegate(&s_remaining_level_delegate) != CHIP_NO_ERROR) {
        ESP_LOGE(tag, "Failed to attach the remaining level reset delegate");
    }
}

// Power Source endpoint and the endpoints it powers, set by init() for kServerReady.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::optional<uint16_t> s_power_source_endpoint_id;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::vector<chip::EndpointId> s_powered_endpoint_ids;

// Peak and average over a rolling window kept in fixed time buckets, so the window slides in
// bucket-sized steps.
class RollingStats {
  public:
    static constexpr uint32_t window_s = 3600;
    static constexpr size_t bucket_count = 12;
    static constexpr int64_t bucket_us = static_cast<int64_t>(window_s) * 1'000'000 / bucket_count;

    void add(float value, int64_t now_us) {
        auto& bucket = bucketAt(now_us);
        bucket.peak = bucket.count == 0 ? value : std::max(bucket.peak, value);
        bucket.sum += value;
        ++bucket.count;
    }

    // nullopt while the window holds no value.
    std::optional<float> peak(int64_t now_us) const {
        std::optional<float> result;
        for (const auto& bucket : buckets_) {
            if (isLive(bucket, now_us)) {
                result = std::max(result.value_or(bucket.peak), bucket.peak);
            }
        }
        return result;
    }

    std::optional<float> average(int64_t now_us) const {
        float sum = 0.0F;
        uint32_t count = 0;
        for (const auto& bucket : buckets_) {
            if (isLive(bucket, now_us)) {
                sum += bucket.sum;
                count += bucket.count;
            }
        }
        return count == 0 ? std::nullopt : std::optional(sum / static_cast<float>(count));
    }

  private:
    struct Bucket {
        int64_t index{-1}; // bucket number since boot
        float peak{};
        float sum{};
        uint32_t count{};
    };

    static int64_t indexAt(int64_t now_us) {
        return now_us / bucket_us;
    }

    static bool isLive(const Bucket& bucket, int64_t now_us) {
        return bucket.count > 0 &&
               bucket.index > indexAt(now_us) - static_cast<int64_t>(bucket_count);
    }

    Bucket& bucketAt(int64_t now_us) {
        const int64_t index = indexAt(now_us);
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        auto& bucket = buckets_[static_cast<size_t>(index) % bucket_count];
        if (bucket.index != index) { bucket = Bucket{.index = index}; }
        return bucket;
    }

    std::array<Bucket, bucket_count> buckets_{};
};

// CO2 peak/average; only touched under the CHIP lock (updateValues).
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
RollingStats s_co2_stats;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::optional<int64_t> s_co2_stats_published_us;

// The list is kept by the Power Source server, which needs the data model to be up.
void setPowerSourceEndpointList() {
    if (!s_power_source_endpoint_id.has_value()) { return; }
    const CHIP_ERROR err = PowerSourceServer::Instance().SetEndpointList(
        *s_power_source_endpoint_id,
        chip::Span<chip::EndpointId>(s_powered_endpoint_ids.data(), s_powered_endpoint_ids.size()));
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(tag, "Failed to set the power source endpoint list: %s", err.Format());
    }
}

// Matter device events: acts on a few and logs the notable ones; the rest are logged by number
// (see chip::DeviceLayer::DeviceEventType) to keep their names out of flash.
// NOLINTNEXTLINE(readability-function-cognitive-complexity): the log macros inflate it.
void matterEventCallback(const ChipDeviceEvent* event, intptr_t /*arg*/) {
    namespace DeviceEventType = chip::DeviceLayer::DeviceEventType;
    switch (event->Type) {
    case DeviceEventType::kWiFiConnectivityChange:
        ESP_LOGI(tag, "Wi-Fi connectivity change");
        break;
    case DeviceEventType::kThreadConnectivityChange:
        ESP_LOGI(tag, "Thread connectivity change");
        break;
    case DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(tag, "Interface IP address changed");
        break;
    case DeviceEventType::kServerReady:
        ESP_LOGI(tag, "Server is ready");
        setPowerSourceEndpointList();
        attachRemainingLevelDelegate();
        break;
    case DeviceEventType::kCommissioningComplete:
        ESP_LOGI(tag, "Commissioning complete");
        notifyCommissioningWindow(false);
        break;
    case DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGW(tag, "Commissioning failed, fail-safe timer expired");
        break;
    case DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(tag, "Commissioning window opened");
        notifyCommissioningWindow(true);
        break;
    case DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(tag, "Commissioning window closed");
        notifyCommissioningWindow(false);
        break;
    case DeviceEventType::kFabricRemoved:
        ESP_LOGI(tag, "Fabric removed");
        break;
    case DeviceEventType::kOtaStateChanged:
        ESP_LOGI(tag, "OTA state changed");
        break;
    default:
        ESP_LOGD(tag, "Matter event 0x%04x", static_cast<unsigned int>(event->Type));
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
        static_cast<unsigned int>(endpoint_id),
        static_cast<unsigned int>(effect_id),
        static_cast<unsigned int>(effect_variant));
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
             static_cast<unsigned int>(endpoint_id),
             static_cast<unsigned int>(cluster_id),
             static_cast<unsigned int>(attribute_id));
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

nullable<int16_t> matterScalePressureScaled(float pressure_pa,
                                            SensorValueRange sensor_value_range) {
    return scaleToMatter<int16_t>(pressure_pa, matter_pressure_scaled_scale, sensor_value_range);
}

// Matter Tolerance (uint16, at most 2048) from an accuracy already in Matter units; rounded up so
// it never understates the error.
uint16_t matterTolerance(float tolerance_in_matter_units) {
    constexpr float max_tolerance = 2048.0F;
    return static_cast<uint16_t>(
        std::clamp(std::ceil(tolerance_in_matter_units), 0.0F, max_tolerance));
}

// Illuminance is logarithmic (10000 x log10(lx) + 1), so a relative accuracy is a constant offset.
uint16_t matterIlluminanceTolerance(float relative_tolerance) {
    return relative_tolerance > 0.0F
               ? matterTolerance(10000.0F * std::log10(1.0F + relative_tolerance))
               : 0;
}

using CreateToleranceFn = esp_matter::attribute_t* (*)(esp_matter::cluster_t*, uint16_t);

// Adds the optional Tolerance attribute; 0 = none.
void addTolerance(esp_matter::endpoint_t* endpoint,
                  uint32_t cluster_id,
                  CreateToleranceFn create_tolerance,
                  uint16_t tolerance,
                  const char* type_name) {
    if (endpoint == nullptr || tolerance == 0) { return; }
    auto* cluster = esp_matter::cluster::get(endpoint, cluster_id);
    if (cluster == nullptr || create_tolerance(cluster, tolerance) == nullptr) {
        ESP_LOGW(tag, "Failed to add the %s tolerance", type_name);
    }
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
                                                        const nullable<int16_t> scaled_max,
                                                        const uint16_t tolerance) {
    if (!isValidScaledRange(scaled_min, scaled_max)) {
        ESP_LOGE(tag, "Min temperature cannot be greater than max temperature.");
        return std::nullopt;
    }

    esp_matter::endpoint::temperature_sensor::config_t config;
    config.temperature_measurement.min_measured_value = scaled_min;
    config.temperature_measurement.max_measured_value = scaled_max;
    config.temperature_measurement.measured_value = nullable<int16_t>();

    auto* endpoint = esp_matter::endpoint::temperature_sensor::create(
        matter_node, &config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    addTolerance(endpoint,
                 TemperatureMeasurement::Id,
                 esp_matter::cluster::temperature_measurement::attribute::create_tolerance,
                 tolerance,
                 "temperature");
    return finishEndpoint(endpoint, "Temperature");
}

// Pressure in 0.1 kPa (MeasuredValue) plus the Extended feature's finer ScaledValue.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::optional<uint16_t> createPressureSensorEndpoint(esp_matter::node_t* matter_node,
                                                     const SensorValueRange& range) {
    const auto scaled_min = matterScalePressureToKpa(range.min, range);
    const auto scaled_max = matterScalePressureToKpa(range.max, range);
    if (!isValidScaledRange(scaled_min, scaled_max)) {
        ESP_LOGE(tag, "Min pressure cannot be greater than max pressure.");
        return std::nullopt;
    }

    esp_matter::endpoint::pressure_sensor::config_t config;
    config.pressure_measurement.min_measured_value = scaled_min;
    config.pressure_measurement.max_measured_value = scaled_max;
    config.pressure_measurement.measured_value = nullable<int16_t>();

    auto* endpoint = esp_matter::endpoint::pressure_sensor::create(
        matter_node, &config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    auto* cluster =
        endpoint == nullptr ? nullptr : esp_matter::cluster::get(endpoint, PressureMeasurement::Id);
    esp_matter::cluster::pressure_measurement::feature::extended::config_t extended;
    extended.min_scaled_value = matterScalePressureScaled(range.min, range);
    extended.max_scaled_value = matterScalePressureScaled(range.max, range);
    extended.scale = matter_pressure_scale_exponent;
    if (cluster == nullptr || esp_matter::cluster::pressure_measurement::feature::extended::add(
                                  cluster, &extended) != ESP_OK) {
        ESP_LOGE(tag, "Failed to add the pressure Extended feature.");
        return std::nullopt;
    }
    addTolerance(endpoint,
                 PressureMeasurement::Id,
                 esp_matter::cluster::pressure_measurement::attribute::create_tolerance,
                 matterTolerance(range.tolerance * matter_pressure_scale),
                 "pressure");
    addTolerance(endpoint,
                 PressureMeasurement::Id,
                 esp_matter::cluster::pressure_measurement::attribute::create_scaled_tolerance,
                 matterTolerance(range.tolerance * matter_pressure_scaled_scale),
                 "pressure scaled");
    return finishEndpoint(endpoint, "Pressure");
}

std::optional<uint16_t> createRelativeHumiditySensorEndpoint(esp_matter::node_t* matter_node,
                                                             const nullable<uint16_t> scaled_min,
                                                             const nullable<uint16_t> scaled_max,
                                                             const uint16_t tolerance) {
    if (!isValidScaledRange(scaled_min, scaled_max)) {
        ESP_LOGE(tag, "Min relative humidity cannot be greater than max relative humidity.");
        return std::nullopt;
    }

    esp_matter::endpoint::humidity_sensor::config_t config;
    config.relative_humidity_measurement.min_measured_value = scaled_min;
    config.relative_humidity_measurement.max_measured_value = scaled_max;
    config.relative_humidity_measurement.measured_value = nullable<uint16_t>();

    auto* endpoint = esp_matter::endpoint::humidity_sensor::create(
        matter_node, &config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    addTolerance(endpoint,
                 RelativeHumidityMeasurement::Id,
                 esp_matter::cluster::relative_humidity_measurement::attribute::create_tolerance,
                 tolerance,
                 "relative humidity");
    return finishEndpoint(endpoint, "Relative humidity");
}

// esp-matter does not yet register ConcentrationMeasurementCluster in data_model::provider.
// One instantiation per concentration cluster id, since the init/shutdown callbacks carry no
// context. PeakAverage adds the Peak and Average Measurement features (RollingStats window).
template <uint32_t ClusterId, ConcentrationMeasurement::MeasurementUnitEnum Unit, bool PeakAverage>
struct ConcentrationServerCluster {
    static constexpr auto unit = Unit;
    static constexpr bool peak_average = PeakAverage;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static inline chip::app::LazyRegisteredServerCluster<
        ConcentrationMeasurement::ConcentrationMeasurementCluster>
        server_cluster;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static inline DataModel::Nullable<float> min_measured;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static inline DataModel::Nullable<float> max_measured;

    // NOLINTNEXTLINE(readability-function-cognitive-complexity): the log macros inflate it.
    static void initCallback(chip::EndpointId endpoint_id) {
        if (!server_cluster.IsConstructed()) {
            using ConcentrationMeasurement::Feature;
            using Flags = chip::BitFlags<Feature>;
            const Flags features = PeakAverage ? Flags(Feature::kNumericMeasurement)
                                                     .Set(Feature::kPeakMeasurement)
                                                     .Set(Feature::kAverageMeasurement)
                                               : Flags(Feature::kNumericMeasurement);
            const ConcentrationMeasurement::ConcentrationMeasurementCluster::Config config{
                .clusterId = ClusterId,
                .features = features,
                .medium = ConcentrationMeasurement::MeasurementMediumEnum::kAir,
                .unit = Unit,
                .minMeasured = min_measured,
                .maxMeasured = max_measured,
            };
            server_cluster.Create(endpoint_id, config);
            if constexpr (PeakAverage) {
                auto& cluster = server_cluster.Cluster();
                if (cluster.SetPeakMeasuredValueWindow(RollingStats::window_s) != CHIP_NO_ERROR ||
                    cluster.SetAverageMeasuredValueWindow(RollingStats::window_s) !=
                        CHIP_NO_ERROR) {
                    ESP_LOGE(tag, "Failed to set the concentration statistics windows");
                }
            }
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
                               ConcentrationMeasurement::MeasurementUnitEnum::kPpm,
                               true>;
using TvocServerCluster =
    ConcentrationServerCluster<TotalVolatileOrganicCompoundsConcentrationMeasurement::Id,
                               ConcentrationMeasurement::MeasurementUnitEnum::kPpb,
                               false>;

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
    // Measurements. Numeric Measurement always; Peak/Average Measurement where tracked.
    namespace concentration_feature = esp_matter::cluster::concentration_measurement::feature;
    cluster_config.feature_flags = concentration_feature::numeric_measurement::get_id();
    if constexpr (ServerCluster::peak_average) {
        cluster_config.feature_flags |= concentration_feature::peak_measurement::get_id() |
                                        concentration_feature::average_measurement::get_id();
        cluster_config.features.peak_measurement.peak_measured_value_window =
            RollingStats::window_s;
        cluster_config.features.average_measurement.average_measured_value_window =
            RollingStats::window_s;
    }

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
                                                        const nullable<uint16_t> scaled_max,
                                                        const uint16_t tolerance) {
    if (!isValidScaledRange(scaled_min, scaled_max)) {
        ESP_LOGE(tag, "Min illuminance cannot be greater than max illuminance.");
        return std::nullopt;
    }

    esp_matter::endpoint::light_sensor::config_t config;
    config.illuminance_measurement.min_measured_value = scaled_min;
    config.illuminance_measurement.max_measured_value = scaled_max;
    config.illuminance_measurement.measured_value = nullable<uint16_t>();

    auto* endpoint = esp_matter::endpoint::light_sensor::create(
        matter_node, &config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    addTolerance(endpoint,
                 IlluminanceMeasurement::Id,
                 esp_matter::cluster::illuminance_measurement::attribute::create_tolerance,
                 tolerance,
                 "illuminance");
    return finishEndpoint(endpoint, "Illuminance");
}

// Remaining level (e.g. salt block) as HEPA Filter Monitoring "Condition": the only HA-mapped
// "% of a consumable left" attribute. There is no fitting device type; Air Purifier is HEPA's home,
// used without its Fan Control cluster so controllers don't show a dummy fan.
// Deliberately non-conformant: Air Purifier requires Fan Control. Home Assistant accepts it; other
// controllers may hide the endpoint, and the device could not be certified as is.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::optional<uint16_t> createRemainingLevelEndpoint(esp_matter::node_t* matter_node) {
    static constexpr const char* type_name = "Remaining level";

    auto* endpoint =
        esp_matter::endpoint::create(matter_node, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    if (endpoint == nullptr) {
        ESP_LOGE(tag, "Failed to create %s sensor endpoint.", type_name);
        return std::nullopt;
    }

    esp_matter::cluster::descriptor::config_t descriptor_config;
    esp_matter::cluster::identify::config_t identify_config;
    if (esp_matter::cluster::descriptor::create(
            endpoint, &descriptor_config, esp_matter::CLUSTER_FLAG_SERVER) == nullptr ||
        esp_matter::endpoint::add_device_type(
            endpoint,
            esp_matter::endpoint::air_purifier::get_device_type_id(),
            esp_matter::endpoint::air_purifier::get_device_type_version()) != ESP_OK ||
        esp_matter::cluster::identify::create(
            endpoint, &identify_config, esp_matter::CLUSTER_FLAG_SERVER) == nullptr) {
        ESP_LOGE(tag, "Failed to set up %s endpoint.", type_name);
        return std::nullopt;
    }

    esp_matter::cluster::hepa_filter_monitoring::config_t hepa_config;
    auto* hepa_cluster = esp_matter::cluster::hepa_filter_monitoring::create(
        endpoint, &hepa_config, esp_matter::CLUSTER_FLAG_SERVER);
    esp_matter::cluster::hepa_filter_monitoring::feature::condition::config_t condition_config;
    condition_config.condition = 100;
    condition_config.degradation_direction =
        static_cast<uint8_t>(ResourceMonitoring::DegradationDirectionEnum::kDown);
    if (hepa_cluster == nullptr ||
        esp_matter::cluster::hepa_filter_monitoring::feature::condition::add(
            hepa_cluster, &condition_config) != ESP_OK ||
        esp_matter::cluster::hepa_filter_monitoring::command::create_reset_condition(
            hepa_cluster) == nullptr) {
        ESP_LOGE(tag, "Failed to attach HEPA Filter Monitoring cluster to endpoint.");
        return std::nullopt;
    }

    return finishEndpoint(endpoint, type_name);
}

// Condition is a whole, non-nullable percent: an unavailable reading keeps the last value.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t updateRemainingLevel(const std::optional<uint16_t> endpoint_id,
                               const std::optional<float> remaining_percent) {
    if (!endpoint_id.has_value()) { return ESP_OK; }
    if (!remaining_percent.has_value() || !std::isfinite(*remaining_percent)) {
        ESP_LOGW(tag, "Remaining level is not available, keeping the last value");
        return ESP_OK;
    }

    auto* resource_cluster = getRemainingLevelCluster(*endpoint_id);
    if (resource_cluster == nullptr) {
        ESP_LOGE(tag, "HEPA filter monitoring cluster not found for endpoint %u", *endpoint_id);
        return ESP_ERR_NOT_FOUND;
    }

    const auto condition =
        static_cast<uint8_t>(std::lround(std::clamp(*remaining_percent, 0.0F, 100.0F)));
    if (resource_cluster->GetCondition() == condition) { return ESP_OK; }

    if (resource_cluster->UpdateCondition(condition) !=
        chip::Protocols::InteractionModel::Status::Success) {
        ESP_LOGE(tag, "Failed to update remaining level to %u %%", condition);
        return ESP_ERR_NOT_FINISHED;
    }
    ESP_LOGI(tag, "Updating remaining level to %u %%", condition);
    return ESP_OK;
}

// Battery as a Power Source cluster (Battery feature; Rechargeable for the charge state) on its own
// endpoint. Its attributes are stored by esp-matter, so they are written with attribute::update.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::optional<uint16_t> createPowerSourceEndpoint(esp_matter::node_t* matter_node,
                                                  const bool has_percent,
                                                  const bool has_voltage,
                                                  const bool has_charge_state) {
    static constexpr const char* type_name = "Power source";
    static constexpr uint8_t max_battery_half_percent = 200;
    static constexpr uint32_t max_battery_millivolts = 5120; // MAX17048 VCELL full scale

    esp_matter::endpoint::power_source::config_t config;
    auto& power_source = config.power_source;
    power_source.status = chip::to_underlying(PowerSource::PowerSourceStatusEnum::kActive);
    std::snprintf(power_source.description, sizeof(power_source.description), "%s", "Battery");
    power_source.feature_flags = esp_matter::cluster::power_source::feature::battery::get_id();
    power_source.features.battery.bat_charge_level =
        chip::to_underlying(PowerSource::BatChargeLevelEnum::kOk);
    power_source.features.battery.bat_replaceability =
        chip::to_underlying(PowerSource::BatReplaceabilityEnum::kUnspecified);
    if (has_charge_state) {
        power_source.feature_flags |=
            esp_matter::cluster::power_source::feature::rechargeable::get_id();
        power_source.features.rechargeable.bat_charge_state =
            chip::to_underlying(PowerSource::BatChargeStateEnum::kUnknown);
        power_source.features.rechargeable.bat_functional_while_charging = true;
    }

    auto* endpoint = esp_matter::endpoint::power_source::create(
        matter_node, &config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    auto* cluster =
        endpoint == nullptr ? nullptr : esp_matter::cluster::get(endpoint, PowerSource::Id);
    if (cluster == nullptr ||
        (has_percent && esp_matter::cluster::power_source::attribute::create_bat_percent_remaining(
                            cluster,
                            nullable<uint8_t>(),
                            nullable<uint8_t>(0),
                            nullable<uint8_t>(max_battery_half_percent)) == nullptr) ||
        (has_voltage && esp_matter::cluster::power_source::attribute::create_bat_voltage(
                            cluster,
                            nullable<uint32_t>(),
                            nullable<uint32_t>(0),
                            nullable<uint32_t>(max_battery_millivolts)) == nullptr)) {
        ESP_LOGE(tag, "Failed to create %s endpoint.", type_name);
        return std::nullopt;
    }

    const uint16_t endpoint_id = esp_matter::endpoint::get_id(endpoint);
    ESP_LOGI(
        tag, "%s endpoint created with id %u", type_name, static_cast<unsigned int>(endpoint_id));
    return endpoint_id;
}

// CO2 PeakMeasuredValue / AverageMeasuredValue over RollingStats::window_s, published once per
// bucket (the window's step) rather than on every reading.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t updateCo2Statistics(const std::optional<float>& value,
                              const std::optional<SensorValueRange>& range) {
    if (!range.has_value() || !Co2ServerCluster::server_cluster.IsConstructed()) { return ESP_OK; }

    const int64_t now_us = esp_timer_get_time();
    if (value.has_value() && std::isfinite(*value)) { s_co2_stats.add(*value, now_us); }
    if (s_co2_stats_published_us.has_value() &&
        now_us - *s_co2_stats_published_us < RollingStats::bucket_us) {
        return ESP_OK;
    }

    auto to_matter_val = [&range](const std::optional<float>& stat) {
        const auto scaled = stat.has_value() ? matterScaleCarbonDioxideConcentration(*stat, *range)
                                             : nullable<float>();
        return scaled.is_null() ? DataModel::NullNullable : DataModel::MakeNullable(scaled.value());
    };
    auto& cluster = Co2ServerCluster::server_cluster.Cluster();
    RETURN_ESP_ERR_ON_CHIP_FAIL(
        cluster.SetPeakMeasuredValue(to_matter_val(s_co2_stats.peak(now_us))),
        tag,
        "Failed to update the CO2 peak");
    RETURN_ESP_ERR_ON_CHIP_FAIL(
        cluster.SetAverageMeasuredValue(to_matter_val(s_co2_stats.average(now_us))),
        tag,
        "Failed to update the CO2 average");
    s_co2_stats_published_us = now_us;
    return ESP_OK;
}

constexpr uint16_t root_endpoint_id = 0;

// Fixed endpoint IDs: a missing sensor must not renumber the others (controllers cache them).
constexpr uint16_t temperature_endpoint_id = 1;
constexpr uint16_t pressure_endpoint_id = 2;
constexpr uint16_t relative_humidity_endpoint_id = 3;
constexpr uint16_t air_quality_endpoint_id = 4;
constexpr uint16_t illuminance_endpoint_id = 5;
constexpr uint16_t remaining_level_endpoint_id = 6;
constexpr uint16_t occupancy_endpoint_id = 7;
constexpr uint16_t power_source_endpoint_id = 8;

// esp-matter numbers endpoints sequentially, so the IDs of absent sensors are taken by empty
// placeholders, destroyed once Matter has started.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
uint16_t s_next_endpoint_id = root_endpoint_id + 1;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::vector<esp_matter::endpoint_t*> s_placeholder_endpoints;

// NOLINTNEXTLINE(readability-function-cognitive-complexity): the log macros inflate it.
bool reserveEndpointsUpTo(esp_matter::node_t* matter_node, uint16_t endpoint_id) {
    while (s_next_endpoint_id < endpoint_id) {
        auto* placeholder = esp_matter::endpoint::create(
            matter_node, esp_matter::ENDPOINT_FLAG_DESTROYABLE, nullptr);
        if (placeholder == nullptr) { return false; }
        s_placeholder_endpoints.push_back(placeholder);
        if (esp_matter::endpoint::get_id(placeholder) != s_next_endpoint_id) {
            ESP_LOGE(tag, "Endpoint numbering out of sync");
            return false;
        }
        ++s_next_endpoint_id;
    }
    if (s_next_endpoint_id != endpoint_id) {
        ESP_LOGE(tag, "Endpoint %u is already taken", endpoint_id);
        return false;
    }
    ++s_next_endpoint_id; // taken by the endpoint created next
    return true;
}

void destroyPlaceholderEndpoints(esp_matter::node_t* matter_node) {
    for (auto* placeholder : s_placeholder_endpoints) {
        if (esp_matter::endpoint::destroy(matter_node, placeholder) != ESP_OK) {
            ESP_LOGW(tag, "Failed to remove a placeholder endpoint");
        }
    }
    s_placeholder_endpoints.clear();
}

// Wired builds (no battery gauge): a Power Source cluster (Wired feature, DC) on the root endpoint.
bool addWiredRootPowerSource(esp_matter::node_t* matter_node) {
    auto* root = esp_matter::endpoint::get(matter_node, root_endpoint_id);
    esp_matter::cluster::power_source::config_t config;
    config.status = chip::to_underlying(PowerSource::PowerSourceStatusEnum::kActive);
    std::snprintf(config.description, sizeof(config.description), "%s", "Wired");
    config.feature_flags = esp_matter::cluster::power_source::feature::wired::get_id();
    config.features.wired.wired_current_type =
        chip::to_underlying(PowerSource::WiredCurrentTypeEnum::kDc);
    if (root == nullptr ||
        esp_matter::cluster::power_source::create(root, &config, esp_matter::CLUSTER_FLAG_SERVER) ==
            nullptr ||
        esp_matter::endpoint::add_device_type(
            root,
            esp_matter::endpoint::power_source::get_device_type_id(),
            esp_matter::endpoint::power_source::get_device_type_version()) != ESP_OK) {
        return false;
    }
    ESP_LOGI(tag, "Wired power source added to the root endpoint");
    return true;
}

esp_matter_attr_val_t toBatteryPercentAttr(const std::optional<float>& percent,
                                           const SensorValueRange& range) {
    return esp_matter_nullable_uint8(
        percent.has_value() ? scaleToMatter<uint8_t>(*percent, matter_battery_percent_scale, range)
                            : nullable<uint8_t>());
}

esp_matter_attr_val_t toBatteryVoltageAttr(const std::optional<float>& voltage,
                                           const SensorValueRange& range) {
    return esp_matter_nullable_uint32(
        voltage.has_value() ? scaleToMatter<uint32_t>(*voltage, matter_battery_voltage_scale, range)
                            : nullable<uint32_t>());
}

esp_matter_attr_val_t toBatteryChargeStateAttr(const std::optional<float>& state,
                                               const SensorValueRange& /*range*/) {
    const long value = state.has_value() && std::isfinite(*state) ? std::lround(*state) : 0;
    const bool known = value > static_cast<long>(BatteryChargeState::UNKNOWN) &&
                       value <= static_cast<long>(BatteryChargeState::NOT_CHARGING);
    return esp_matter_enum8(known ? static_cast<uint8_t>(value)
                                  : chip::to_underlying(PowerSource::BatChargeStateEnum::kUnknown));
}

// Presence as Occupancy Sensing (Radar feature) with HoldTime: the cluster delays "unoccupied" by
// HoldTime, which controllers can write and the SDK persists.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::optional<uint16_t> createOccupancyEndpoint(esp_matter::node_t* matter_node) {
    static constexpr const char* type_name = "Occupancy";
    static constexpr uint16_t default_hold_time_s = 30; // SDK default; limits are 1..300 s

    esp_matter::endpoint::occupancy_sensor::config_t config;
    config.occupancy_sensing.feature_flags =
        esp_matter::cluster::occupancy_sensing::feature::radar::get_id();
    auto* endpoint = esp_matter::endpoint::occupancy_sensor::create(
        matter_node, &config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    auto* cluster =
        endpoint == nullptr ? nullptr : esp_matter::cluster::get(endpoint, OccupancySensing::Id);
    if (cluster == nullptr ||
        esp_matter::cluster::occupancy_sensing::attribute::create_hold_time(
            cluster, default_hold_time_s) == nullptr ||
        esp_matter::cluster::occupancy_sensing::event::create_occupancy_changed(cluster) ==
            nullptr) {
        ESP_LOGE(tag, "Failed to create %s sensor endpoint.", type_name);
        return std::nullopt;
    }
    return finishEndpoint(endpoint, type_name);
}

// Transitions only: SetOccupancy(false) restarts the hold timer on every call. Occupancy is not
// nullable, so an unavailable sensor reports clear (after the hold time) rather than a stuck state.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t updateOccupancy(const std::optional<uint16_t> endpoint_id,
                          const std::optional<float> value,
                          std::optional<float>& last_reported_value) {
    if (!endpoint_id.has_value()) { return ESP_OK; }
    const bool available = value.has_value() && std::isfinite(*value);
    const bool occupied = available && *value >= 0.5F;
    if (last_reported_value.has_value() && (*last_reported_value >= 0.5F) == occupied) {
        return ESP_OK;
    }

    auto* cluster = esp_matter::data_model::provider::get_instance().registry().Get(
        ConcreteClusterPath(*endpoint_id, OccupancySensing::Id));
    if (cluster == nullptr) {
        ESP_LOGE(tag, "Occupancy sensing cluster not found for endpoint %u", *endpoint_id);
        return ESP_ERR_NOT_FOUND;
    }
    // Registry entry for this cluster id is registered as exactly OccupancySensingCluster.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
    static_cast<OccupancySensingCluster*>(cluster)->SetOccupancy(occupied);
    if (!available) { ESP_LOGW(tag, "Occupancy sensor unavailable, reporting clear"); }
    ESP_LOGI(tag, "Updating occupancy to %s", occupied ? "occupied" : "clear (after hold time)");
    last_reported_value = occupied ? 1.0F : 0.0F;
    return ESP_OK;
}

// Mandatory BatChargeLevel, derived from the battery percentage; kept as is when unavailable and
// written only when the level changes.
esp_err_t updateBatteryChargeLevel(const std::optional<uint16_t> endpoint_id,
                                   const std::optional<float> percent) {
    // One Power Source endpoint, only updated under the CHIP lock.
    static std::optional<PowerSource::BatChargeLevelEnum> last_level;
    if (!endpoint_id.has_value() || !percent.has_value() || !std::isfinite(*percent)) {
        return ESP_OK;
    }
    auto level = PowerSource::BatChargeLevelEnum::kOk;
    if (*percent <= static_cast<float>(CONFIG_MATTER_BATTERY_CRITICAL_PERCENT)) {
        level = PowerSource::BatChargeLevelEnum::kCritical;
    } else if (*percent <= static_cast<float>(CONFIG_MATTER_BATTERY_WARNING_PERCENT)) {
        level = PowerSource::BatChargeLevelEnum::kWarning;
    }
    if (last_level == level) { return ESP_OK; }

    auto val = esp_matter_enum8(chip::to_underlying(level));
    const esp_err_t err = esp_matter::attribute::update(
        *endpoint_id, PowerSource::Id, PowerSource::Attributes::BatChargeLevel::Id, &val);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Failed to update battery charge level: %s", esp_err_to_name(err));
        return err;
    }
    last_level = level;
    return ESP_OK;
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
        if (!reserveEndpointsUpTo(matter_node, temperature_endpoint_id)) { return ESP_FAIL; }
        measurement.endpoint_id =
            createTemperatureSensorEndpoint(matter_node,
                                            matterScaleTemp(range->min, *range),
                                            matterScaleTemp(range->max, *range),
                                            matterTolerance(range->tolerance * matter_temp_scale));
        if (!measurement.endpoint_id) {
            ESP_LOGE(tag, "Failed to initialize temperature sensor.");
            return ESP_FAIL;
        }
    }

    if (const auto& range = at<SensorType::PRESSURE>(ranges)) {
        auto& measurement = at<SensorType::PRESSURE>(measurements_);
        measurement.range = range;
        if (!reserveEndpointsUpTo(matter_node, pressure_endpoint_id)) { return ESP_FAIL; }
        measurement.endpoint_id = createPressureSensorEndpoint(matter_node, *range);
        if (!measurement.endpoint_id) {
            ESP_LOGE(tag, "Failed to initialize pressure sensor.");
            return ESP_FAIL;
        }
    }

    if (const auto& range = at<SensorType::RELATIVE_HUMIDITY>(ranges)) {
        auto& measurement = at<SensorType::RELATIVE_HUMIDITY>(measurements_);
        measurement.range = range;
        if (!reserveEndpointsUpTo(matter_node, relative_humidity_endpoint_id)) { return ESP_FAIL; }
        measurement.endpoint_id = createRelativeHumiditySensorEndpoint(
            matter_node,
            matterScaleRelativeHumidity(range->min, *range),
            matterScaleRelativeHumidity(range->max, *range),
            matterTolerance(range->tolerance * matter_relative_humidity_scale));
        if (!measurement.endpoint_id) {
            ESP_LOGE(tag, "Failed to initialize relative humidity sensor.");
            return ESP_FAIL;
        }
    }

    // CO2, TVOC and air quality are independent, but share one Air Quality Sensor endpoint.
    esp_matter::endpoint_t* aq_endpoint = nullptr;
    auto get_aq_endpoint = [&]() -> esp_matter::endpoint_t* {
        if (aq_endpoint == nullptr && reserveEndpointsUpTo(matter_node, air_quality_endpoint_id)) {
            aq_endpoint = createAirQualityEndpoint(matter_node);
        }
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
        if (!reserveEndpointsUpTo(matter_node, illuminance_endpoint_id)) { return ESP_FAIL; }
        measurement.endpoint_id =
            createIlluminanceSensorEndpoint(matter_node,
                                            matterScaleIlluminance(range->min, *range),
                                            matterScaleIlluminance(range->max, *range),
                                            matterIlluminanceTolerance(range->tolerance));
        if (!measurement.endpoint_id) {
            ESP_LOGE(tag, "Failed to initialize illuminance sensor.");
            return ESP_FAIL;
        }
    }

    if (const auto& range = at<SensorType::REMAINING_LEVEL>(ranges)) {
        auto& measurement = at<SensorType::REMAINING_LEVEL>(measurements_);
        measurement.range = range;
        if (!reserveEndpointsUpTo(matter_node, remaining_level_endpoint_id)) { return ESP_FAIL; }
        measurement.endpoint_id = createRemainingLevelEndpoint(matter_node);
        if (!measurement.endpoint_id) {
            ESP_LOGE(tag, "Failed to initialize remaining level sensor.");
            return ESP_FAIL;
        }
        s_remaining_level_endpoint_id = measurement.endpoint_id;
    }

    if (const auto& range = at<SensorType::OCCUPANCY>(ranges)) {
        auto& measurement = at<SensorType::OCCUPANCY>(measurements_);
        measurement.range = range;
        if (!reserveEndpointsUpTo(matter_node, occupancy_endpoint_id)) { return ESP_FAIL; }
        measurement.endpoint_id = createOccupancyEndpoint(matter_node);
        if (!measurement.endpoint_id) {
            ESP_LOGE(tag, "Failed to initialize occupancy sensor.");
            return ESP_FAIL;
        }
    }

    {
        const auto& percent_range = at<SensorType::BATTERY_PERCENT>(ranges);
        const auto& voltage_range = at<SensorType::BATTERY_VOLTAGE>(ranges);
        const auto& charge_state_range = at<SensorType::BATTERY_CHARGE_STATE>(ranges);
        if (percent_range || voltage_range || charge_state_range) {
            if (!reserveEndpointsUpTo(matter_node, power_source_endpoint_id)) { return ESP_FAIL; }
            const auto endpoint_id = createPowerSourceEndpoint(matter_node,
                                                               percent_range.has_value(),
                                                               voltage_range.has_value(),
                                                               charge_state_range.has_value());
            if (!endpoint_id) {
                ESP_LOGE(tag, "Failed to initialize power source.");
                return ESP_FAIL;
            }
            auto attach = [&endpoint_id](Measurement& measurement,
                                         const std::optional<SensorValueRange>& range) {
                if (!range) { return; }
                measurement.range = range;
                measurement.endpoint_id = endpoint_id;
            };
            attach(at<SensorType::BATTERY_PERCENT>(measurements_), percent_range);
            attach(at<SensorType::BATTERY_VOLTAGE>(measurements_), voltage_range);
            attach(at<SensorType::BATTERY_CHARGE_STATE>(measurements_), charge_state_range);

            // The battery powers every other endpoint.
            s_power_source_endpoint_id = endpoint_id;
            s_powered_endpoint_ids.clear();
        } else {
            // Wired: the node's power source on the root endpoint, powering every endpoint.
            if (!addWiredRootPowerSource(matter_node)) {
                ESP_LOGE(tag, "Failed to initialize the wired power source.");
                return ESP_FAIL;
            }
            s_power_source_endpoint_id = root_endpoint_id;
            s_powered_endpoint_ids.assign(1, root_endpoint_id);
        }
        for (const auto& measurement : measurements_) { // shared endpoints listed once
            const auto& powered_id = measurement.endpoint_id;
            if (powered_id && powered_id != s_power_source_endpoint_id &&
                std::ranges::find(s_powered_endpoint_ids, *powered_id) ==
                    s_powered_endpoint_ids.end()) {
                s_powered_endpoint_ids.push_back(*powered_id);
            }
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
    destroyPlaceholderEndpoints(matter_node);
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

    // Pressure also publishes the Extended feature's finer ScaledValue.
    constexpr bool has_scaled_value = std::is_same_v<Cluster, PressureMeasurementCluster>;
    auto to_scaled_val = [&](const std::optional<float>& reading) {
        const auto scaled = reading.has_value()
                                ? matterScalePressureScaled(*reading, *measurement.range)
                                : nullable<int16_t>();
        return scaled.is_null() ? DataModel::NullNullable : DataModel::MakeNullable(scaled.value());
    };
    const bool unchanged = matter_val == to_matter_val(measurement.last_reported_value) &&
                           (!has_scaled_value ||
                            to_scaled_val(value) == to_scaled_val(measurement.last_reported_value));

    // Keep last_reported_value unchanged if scaled value did not change (e.g. sub-LSB step).
    if (measurement.has_reported && unchanged) {
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
    if constexpr (has_scaled_value) {
        RETURN_ESP_ERR_ON_CHIP_FAIL(typed_cluster->SetScaledValue(to_scaled_val(value)),
                                    tag,
                                    "Failed to update the %s scaled value",
                                    type_name);
    }

    measurement.last_reported_value = value;
    measurement.last_report_time_us = now_us;
    measurement.has_reported = true;

    return ESP_OK;
}

template <typename ToAttrVal>
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t MatterManager::updateAttribute(const std::optional<float> value,
                                         Measurement& measurement,
                                         ToAttrVal to_attr_val,
                                         const char* type_name,
                                         const uint32_t cluster_id,
                                         const uint32_t attribute_id) {
    if (!measurement.endpoint_id.has_value() || !measurement.range.has_value()) { return ESP_OK; }

    const int64_t now_us = esp_timer_get_time();
    if (shouldSkipReport(value,
                         *measurement.range,
                         measurement.last_reported_value,
                         measurement.last_report_time_us,
                         measurement.has_reported,
                         now_us)) {
        return ESP_OK;
    }

    if (value.has_value()) {
        ESP_LOGI(tag, "Updating %s to %.2f", type_name, *value);
    } else {
        ESP_LOGW(tag, "%s value is not available, publishing null", type_name);
    }
    auto val = to_attr_val(value, *measurement.range);
    const esp_err_t err =
        esp_matter::attribute::update(*measurement.endpoint_id, cluster_id, attribute_id, &val);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Failed to update %s: %s", type_name, esp_err_to_name(err));
        return err;
    }

    measurement.last_reported_value = value;
    measurement.last_report_time_us = now_us;
    measurement.has_reported = true;
    return ESP_OK;
}

esp_err_t MatterManager::updateValue(const SensorType type, const std::optional<float> value) {
    esp_matter::lock::ScopedChipStackLock lock(portMAX_DELAY);

    if (type == SensorType::OCCUPANCY) {
        auto& measurement = at<SensorType::OCCUPANCY>(measurements_);
        return updateOccupancy(measurement.endpoint_id, value, measurement.last_reported_value);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t MatterManager::updateValues(const SensorReadings& values) {
    esp_matter::lock::ScopedChipStackLock lock(portMAX_DELAY);

    auto ret = ESP_OK;
    // Any failed update makes the whole call report ESP_ERR_NOT_FINISHED.
    auto check = [&ret](esp_err_t result) {
        if (result != ESP_OK) { ret = ESP_ERR_NOT_FINISHED; }
    };

    check(updateValue<TemperatureMeasurementCluster>(at<SensorType::TEMPERATURE>(values),
                                                     at<SensorType::TEMPERATURE>(measurements_),
                                                     matterScaleTemp,
                                                     "temperature",
                                                     TemperatureMeasurement::Id));

    check(updateValue<PressureMeasurementCluster>(at<SensorType::PRESSURE>(values),
                                                  at<SensorType::PRESSURE>(measurements_),
                                                  matterScalePressureToKpa,
                                                  "pressure",
                                                  PressureMeasurement::Id));

    check(updateValue<RelativeHumidityMeasurementCluster>(
        at<SensorType::RELATIVE_HUMIDITY>(values),
        at<SensorType::RELATIVE_HUMIDITY>(measurements_),
        matterScaleRelativeHumidity,
        "relative humidity",
        RelativeHumidityMeasurement::Id));

    check(updateValue<ConcentrationMeasurement::ConcentrationMeasurementCluster>(
        at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(values),
        at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(measurements_),
        matterScaleCarbonDioxideConcentration,
        "carbon dioxide (CO2) concentration",
        CarbonDioxideConcentrationMeasurement::Id));
    check(updateCo2Statistics(at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(values),
                              at<SensorType::CARBON_DIOXIDE_CONCENTRATION>(measurements_).range));

    check(updateValue<ConcentrationMeasurement::ConcentrationMeasurementCluster>(
        at<SensorType::TVOC_CONCENTRATION>(values),
        at<SensorType::TVOC_CONCENTRATION>(measurements_),
        matterScaleTvocConcentration,
        "TVOC concentration",
        TotalVolatileOrganicCompoundsConcentrationMeasurement::Id));

    check(updateAirQuality(at<SensorType::AIR_QUALITY>(measurements_).endpoint_id,
                           at<SensorType::AIR_QUALITY>(values)));

    check(updateValue<IlluminanceMeasurementCluster>(at<SensorType::ILLUMINANCE>(values),
                                                     at<SensorType::ILLUMINANCE>(measurements_),
                                                     matterScaleIlluminance,
                                                     "illuminance",
                                                     IlluminanceMeasurement::Id));

    check(updateRemainingLevel(at<SensorType::REMAINING_LEVEL>(measurements_).endpoint_id,
                               at<SensorType::REMAINING_LEVEL>(values)));

    // OCCUPANCY is pushed (updateValue(type, value)) only: a polled value could be older than a
    // pushed transition.

    check(updateAttribute(at<SensorType::BATTERY_PERCENT>(values),
                          at<SensorType::BATTERY_PERCENT>(measurements_),
                          toBatteryPercentAttr,
                          "battery percent",
                          PowerSource::Id,
                          PowerSource::Attributes::BatPercentRemaining::Id));

    check(updateBatteryChargeLevel(at<SensorType::BATTERY_PERCENT>(measurements_).endpoint_id,
                                   at<SensorType::BATTERY_PERCENT>(values)));

    check(updateAttribute(at<SensorType::BATTERY_VOLTAGE>(values),
                          at<SensorType::BATTERY_VOLTAGE>(measurements_),
                          toBatteryVoltageAttr,
                          "battery voltage",
                          PowerSource::Id,
                          PowerSource::Attributes::BatVoltage::Id));

    check(updateAttribute(at<SensorType::BATTERY_CHARGE_STATE>(values),
                          at<SensorType::BATTERY_CHARGE_STATE>(measurements_),
                          toBatteryChargeStateAttr,
                          "battery charge state",
                          PowerSource::Id,
                          PowerSource::Attributes::BatChargeState::Id));

    return ret;
}
