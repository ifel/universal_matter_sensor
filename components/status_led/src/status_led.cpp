#include "status_led/status_led.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <sdkconfig.h>

namespace {
constexpr const char* tag = "status_led";
constexpr uint64_t k_slow_blink_us = 1000ULL * 1000ULL;
constexpr uint64_t k_fast_blink_us = 200ULL * 1000ULL;
constexpr uint64_t k_ready_on_us = 3ULL * 60ULL * 1000ULL * 1000ULL; // solid on after boot

#if CONFIG_STATUS_LED_GPIO >= 0
constexpr gpio_num_t k_status_led_gpio = static_cast<gpio_num_t>(CONFIG_STATUS_LED_GPIO);
#    ifdef CONFIG_STATUS_LED_ACTIVE_LOW
constexpr uint32_t k_led_on_level = 0;
constexpr uint32_t k_led_off_level = 1;
#    else
constexpr uint32_t k_led_on_level = 1;
constexpr uint32_t k_led_off_level = 0;
#    endif
#endif
} // namespace

StatusLed::~StatusLed() {
    if (timer_ != nullptr) {
        esp_timer_stop(timer_);
        esp_timer_delete(timer_);
    }
    if (ready_timer_ != nullptr) {
        esp_timer_stop(ready_timer_);
        esp_timer_delete(ready_timer_);
    }
}

void StatusLed::init() {
#if CONFIG_STATUS_LED_GPIO >= 0
    const std::lock_guard<std::mutex> lock(mutex_);
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << k_status_led_gpio);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    setLevelLocked(false);

    esp_timer_create_args_t timer_args = {};
    timer_args.callback = &StatusLed::timerCallback;
    timer_args.arg = this;
    timer_args.name = "status_led";
    if (esp_timer_create(&timer_args, &timer_) != ESP_OK) {
        ESP_LOGE(tag, "Failed to create blink timer");
        timer_ = nullptr;
    }

    timer_args.callback = &StatusLed::readyTimeoutCallback;
    timer_args.name = "status_led_ready";
    if (esp_timer_create(&timer_args, &ready_timer_) != ESP_OK) {
        ESP_LOGE(tag, "Failed to create ready timer");
        ready_timer_ = nullptr;
    }
#endif
}

void StatusLed::setReady(bool ready) {
    const std::lock_guard<std::mutex> lock(mutex_);
    ready_ = ready;
    if (ready_timer_ != nullptr) {
        esp_timer_stop(ready_timer_);
        if (ready) { esp_timer_start_once(ready_timer_, k_ready_on_us); }
    }
    applyLocked();
}

void StatusLed::setCommissioning(bool commissioning) {
    const std::lock_guard<std::mutex> lock(mutex_);
    commissioning_ = commissioning;
    applyLocked();
}

void StatusLed::setIdentifying(bool identifying) {
    const std::lock_guard<std::mutex> lock(mutex_);
    identifying_ = identifying;
    applyLocked();
}

void StatusLed::applyLocked() {
    if (timer_ == nullptr) { return; } // LED disabled or not initialized

    Mode mode = Mode::OFF;
    if (identifying_) {
        mode = Mode::FAST_BLINK;
    } else if (commissioning_) {
        mode = Mode::SLOW_BLINK;
    } else if (ready_) {
        mode = Mode::ON;
    }
    if (mode == mode_) { return; }
    mode_ = mode;

    esp_timer_stop(timer_);
    switch (mode) {
    case Mode::OFF:
        setLevelLocked(false);
        break;
    case Mode::ON:
        setLevelLocked(true);
        break;
    case Mode::SLOW_BLINK:
        esp_timer_start_periodic(timer_, k_slow_blink_us);
        break;
    case Mode::FAST_BLINK:
        esp_timer_start_periodic(timer_, k_fast_blink_us);
        break;
    }
}

void StatusLed::timerCallback(void* arg) {
    auto* self = static_cast<StatusLed*>(arg);
    const std::lock_guard<std::mutex> lock(self->mutex_);
    // A tick racing with a mode change must not touch a non-blinking LED.
    if (self->mode_ == Mode::SLOW_BLINK || self->mode_ == Mode::FAST_BLINK) {
        self->setLevelLocked(!self->led_on_);
    }
}

void StatusLed::readyTimeoutCallback(void* arg) {
    auto* self = static_cast<StatusLed*>(arg);
    const std::lock_guard<std::mutex> lock(self->mutex_);
    self->ready_ = false;
    self->applyLocked();
}

void StatusLed::setLevelLocked(bool is_on) {
    led_on_ = is_on;
#if CONFIG_STATUS_LED_GPIO >= 0
    gpio_set_level(k_status_led_gpio, is_on ? k_led_on_level : k_led_off_level);
#endif
}
