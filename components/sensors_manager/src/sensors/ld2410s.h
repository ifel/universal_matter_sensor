#ifndef SENSORS_LD2410S_H
#define SENSORS_LD2410S_H

#include <array>
#include <atomic>
#include <mutex>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <ld2410s.h>

#include "sensors_manager/sensor.h"

// Hi-Link HLK-LD2410S 24 GHz presence radar on UART. A reader task parses its reports and pushes
// occupancy changes (nullopt when the radar goes silent) through the reading listener; read()
// returns the latest state for logging.
class Ld2410s : public Sensor {
  public:
    Ld2410s() = default;
    ~Ld2410s() override;
    // Prevent copying
    Ld2410s(const Ld2410s&) = delete;
    Ld2410s& operator=(const Ld2410s&) = delete;

    // Prevent moving
    Ld2410s(Ld2410s&&) = delete;
    Ld2410s& operator=(Ld2410s&&) = delete;

    std::span<const SensorType> getTypes() const override {
        static constexpr std::array k_types = {
            SensorType::OCCUPANCY,
        };
        return std::span<const SensorType>(k_types);
    }
    const char* getModelName() const override {
        return "LD2410S";
    }
    std::optional<SensorValueRange> getValueRangeForType(SensorType type) const override;
    esp_err_t init(int64_t read_interval_ms) override;
    esp_err_t read(SensorReadings& readings, SensorReadings& compensations) override;
    void setReadingListener(ReadingListener listener) override;

  private:
    static void taskEntry(void* arg);
    void readerLoop();
    void notify(std::optional<float> value);

    ld2410s_t sensor_{};
    TaskHandle_t task_ = nullptr;
    SemaphoreHandle_t task_stopped_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<int> occupied_{-1};          // -1 until the first report or while silent
    std::atomic<bool> push_current_{false};  // set by setReadingListener
    std::atomic<int64_t> last_report_us_{0}; // esp_timer time of the last report
    std::mutex listener_mutex_;
    ReadingListener listener_;
    bool initialized_ = false; // true only after a successful init()
};

#endif // SENSORS_LD2410S_H
