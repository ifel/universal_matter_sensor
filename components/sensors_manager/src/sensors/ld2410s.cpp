#include <sdkconfig.h>

#if CONFIG_LD2410S_ENABLED // LD2410S Kconfig options exist only when enabled
#    include <esp_check.h>
#    include <esp_timer.h>

#    include "ld2410s.h"

constexpr const char* tag = "   ***Sensor LD2410S***   ";

// LD2410S Sensor Configuration (from Kconfig); ESP TX goes to the module's RX and vice versa.
constexpr auto uart_port = static_cast<uart_port_t>(CONFIG_LD2410S_UART_PORT);
constexpr int tx_gpio = CONFIG_LD2410S_TX_GPIO;
constexpr int rx_gpio = CONFIG_LD2410S_RX_GPIO;

namespace {
// The radar's own "no one" delay is kept at its minimum: Matter HoldTime adds the off-delay.
constexpr uint8_t unmanned_delay_s = LD2410S_UNMANNED_DELAY_MIN_S;
constexpr uint8_t report_rate_tenth_hz = 40; // 4 Hz status/distance reports
constexpr int configure_attempts = 3;        // the module may still be booting

constexpr TickType_t report_wait_ticks = pdMS_TO_TICKS(500);
constexpr int64_t stale_after_us = 5'000'000; // no report for this long = unavailable

constexpr uint32_t task_stack_size = 3072;
constexpr UBaseType_t task_priority = 5;
} // namespace

Ld2410s::~Ld2410s() {
    if (initialized_) {
        running_ = false;
        xSemaphoreTake(task_stopped_, portMAX_DELAY);
        vSemaphoreDelete(task_stopped_);
        ld2410s_deinit(&sensor_);
    }
}

std::optional<SensorValueRange> Ld2410s::getValueRangeForType(SensorType type) const {
    // Occupancy (0 = unoccupied, 1 = occupied)
    constexpr float occupancy_min = 0.0F;
    constexpr float occupancy_max = 1.0F;
    constexpr float occupancy_abs_threshold = 1.0F;

    switch (type) {
    case SensorType::TEMPERATURE:
    case SensorType::PRESSURE:
    case SensorType::RELATIVE_HUMIDITY:
    case SensorType::CARBON_DIOXIDE_CONCENTRATION:
    case SensorType::ILLUMINANCE:
    case SensorType::TVOC_CONCENTRATION:
    case SensorType::AIR_QUALITY:
    case SensorType::REMAINING_LEVEL:
    case SensorType::BATTERY_PERCENT:
    case SensorType::BATTERY_VOLTAGE:
    case SensorType::BATTERY_CHARGE_STATE:
        return std::nullopt;
    case SensorType::OCCUPANCY:
        return SensorValueRange{
            .min = occupancy_min, .max = occupancy_max, .abs_threshold = occupancy_abs_threshold};
    }
    return std::nullopt;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Ld2410s::init(int64_t /*read_interval_ms*/) {
    // idempotent, avoids leaking on a second call
    ESP_RETURN_ON_FALSE(!initialized_, ESP_OK, tag, "The sensor is already initialized");

    ESP_LOGI(tag, "Initializing sensor...");

    auto ret = ld2410s_init(&sensor_, uart_port, tx_gpio, rx_gpio);
    ESP_RETURN_ON_ERROR(ret, tag, "UART initialization failed: %s", esp_err_to_name(ret));

    for (int attempt = 0; attempt < configure_attempts; ++attempt) {
        ret = ld2410s_configure(&sensor_, unmanned_delay_s, report_rate_tenth_hz);
        if (ret == ESP_OK) { break; }
    }
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_ERROR(
        ret, cleanup, tag, "initialization failed on configure: %s", esp_err_to_name(ret));

    task_stopped_ = xSemaphoreCreateBinary();
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
    ESP_GOTO_ON_FALSE(task_stopped_ != nullptr, ESP_ERR_NO_MEM, cleanup, tag, "No semaphore");
    running_ = true;
    if (xTaskCreate(taskEntry, "ld2410s", task_stack_size, this, task_priority, &task_) != pdPASS) {
        running_ = false;
        vSemaphoreDelete(task_stopped_);
        ret = ESP_ERR_NO_MEM;
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-goto)
        ESP_GOTO_ON_ERROR(ret, cleanup, tag, "Failed to start the reader task");
    }

    initialized_ = true;
    ESP_LOGI(tag,
             "Sensor initialized: %s on UART%d (firmware %u.%u.%u).",
             getModelName(),
             static_cast<int>(uart_port),
             sensor_.fw_major,
             sensor_.fw_minor,
             sensor_.fw_patch);

    return ESP_OK;

cleanup:
    ld2410s_deinit(&sensor_); // UART was set up above, so release it on failure
    return ret;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
esp_err_t Ld2410s::read(SensorReadings& readings, SensorReadings& /*compensations*/) {
    // guard instead of a null deref
    ESP_RETURN_ON_FALSE(
        initialized_, ESP_ERR_INVALID_STATE, tag, "Cannot read values on non initialized sensor");

    at<SensorType::OCCUPANCY>(readings) = std::nullopt;

    const int occupied = occupied_;
    const bool fresh = occupied >= 0 && esp_timer_get_time() - last_report_us_ < stale_after_us;
    ESP_RETURN_ON_FALSE(fresh, ESP_ERR_TIMEOUT, tag, "No recent report from the radar");

    at<SensorType::OCCUPANCY>(readings) = static_cast<float>(occupied);
    return ESP_OK;
}

void Ld2410s::setReadingListener(ReadingListener listener) {
    {
        const std::scoped_lock lock(listener_mutex_);
        listener_ = std::move(listener);
    }
    push_current_ = true; // the reader task pushes the state known so far
}

void Ld2410s::notify(std::optional<float> value) {
    ReadingListener listener;
    {
        const std::scoped_lock lock(listener_mutex_);
        listener = listener_;
    }
    if (listener) { listener(SensorType::OCCUPANCY, value); }
}

void Ld2410s::taskEntry(void* arg) {
    auto* self = static_cast<Ld2410s*>(arg);
    self->readerLoop();
    xSemaphoreGive(self->task_stopped_);
    vTaskDelete(nullptr);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void Ld2410s::readerLoop() {
    while (running_) {
        // Every push comes from this task, so the listener gets them in order.
        if (push_current_.exchange(false)) {
            if (const int state = occupied_; state >= 0) { notify(static_cast<float>(state)); }
        }

        bool occupied = false;
        uint16_t distance_cm = 0;
        if (ld2410s_read_report(&sensor_, &occupied, &distance_cm, report_wait_ticks) != ESP_OK) {
            // A silent radar is pushed as unavailable once; its next report restores the state.
            if (occupied_ >= 0 && esp_timer_get_time() - last_report_us_ >= stale_after_us) {
                occupied_ = -1;
                ESP_LOGW(tag, "No report from the radar, occupancy unavailable");
                notify(std::nullopt);
            }
            continue;
        }
        last_report_us_ = esp_timer_get_time();
        ESP_LOGD(tag, "occupied %d, distance %u cm", occupied, distance_cm);

        if (occupied_.exchange(occupied ? 1 : 0) == (occupied ? 1 : 0)) { continue; }
        ESP_LOGI(
            tag, "Occupancy changed: %s (%u cm)", occupied ? "occupied" : "clear", distance_cm);

        notify(occupied ? 1.0F : 0.0F);
    }
}
#endif // CONFIG_LD2410S_ENABLED
