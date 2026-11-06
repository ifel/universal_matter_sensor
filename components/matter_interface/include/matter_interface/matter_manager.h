#ifndef MATTER_MANAGER_H
#define MATTER_MANAGER_H

#include <cstdint>
#include <functional>
#include <optional>
#include <utility>

#include <esp_err.h>

#include "common_types/common_types.h"

class MatterManager {
  public:
    // Matter state notifications, invoked from the Matter task. Set before init().
    struct Listeners {
        std::function<void(bool)> on_commissioning_window; // true while the window is open
        std::function<void(bool)> on_identify;             // true while identifying
    };

    MatterManager() = default;
    ~MatterManager() = default;
    // Prevent copying
    MatterManager(const MatterManager&) = delete;
    MatterManager& operator=(const MatterManager&) = delete;

    // Prevent moving
    MatterManager(MatterManager&&) = delete;
    MatterManager& operator=(MatterManager&&) = delete;

    void setListeners(Listeners listeners) {
        listeners_ = std::move(listeners);
    }

    // Creates an endpoint for every type that has a range.
    esp_err_t init(const SensorRanges& ranges);
    // Values in °C, Pa, %RH; nullopt publishes a null measurement.
    esp_err_t updateValues(const SensorReadings& values);

  private:
    struct Measurement {
        std::optional<uint16_t> endpoint_id;
        std::optional<SensorValueRange> range;
        std::optional<float> last_reported_value;
        int64_t last_report_time_us{0};
        bool has_reported{false};
    };

    template <typename Cluster, typename ConvFn>
    static esp_err_t updateValue(std::optional<float> value,
                                 Measurement& measurement,
                                 ConvFn value_conv_func,
                                 const char* type_name,
                                 uint32_t cluster_id);

    // Same deadband/heartbeat for clusters whose attributes esp-matter stores (attribute::update).
    template <typename ToAttrVal>
    static esp_err_t updateAttribute(std::optional<float> value,
                                     Measurement& measurement,
                                     ToAttrVal to_attr_val,
                                     const char* type_name,
                                     uint32_t cluster_id,
                                     uint32_t attribute_id);

    PerSensorType<Measurement> measurements_{};
    Listeners listeners_;
};

#endif // MATTER_MANAGER_H
