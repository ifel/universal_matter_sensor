#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <cstdint>
#include <mutex>

#include <esp_timer.h>

// Status LED driven by three flags. Priority: identifying (fast blink) > commissioning (slow
// blink) > ready (solid on) > off. All methods are safe to call from any task.
class StatusLed {
  public:
    StatusLed() = default;
    ~StatusLed();

    StatusLed(const StatusLed&) = delete;
    StatusLed& operator=(const StatusLed&) = delete;
    StatusLed(StatusLed&&) = delete;
    StatusLed& operator=(StatusLed&&) = delete;

    void init();
    void setReady(bool ready);
    void setCommissioning(bool commissioning);
    void setIdentifying(bool identifying);

  private:
    enum class Mode : uint8_t { OFF, ON, SLOW_BLINK, FAST_BLINK };

    static void timerCallback(void* arg);
    void applyLocked();
    void setLevelLocked(bool is_on);

    std::mutex mutex_;
    esp_timer_handle_t timer_{nullptr};
    Mode mode_{Mode::OFF};
    bool led_on_{false};
    bool ready_{false};
    bool commissioning_{false};
    bool identifying_{false};
};

#endif // STATUS_LED_H
