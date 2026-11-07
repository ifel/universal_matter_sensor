/**
 * @file ld2410s.h
 *
 * ESP-IDF driver for the Hi-Link HLK-LD2410S low-power 24 GHz human presence radar (UART,
 * 115200 8N1). Implements the "Serial Communication Protocol V1.00": minimal reports and the few
 * commands needed to detect and configure the module. Not compatible with the LD2410 / LD2410B/C.
 */
#ifndef LD2410S_H
#define LD2410S_H

#include <driver/uart.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LD2410S_UNMANNED_DELAY_MIN_S 10 //!< "no one" delay range (parameter 0x06)
#define LD2410S_UNMANNED_DELAY_MAX_S 120

typedef struct {
    uart_port_t port;
    uint16_t fw_major; //!< firmware version, set by ld2410s_configure()
    uint16_t fw_minor;
    uint16_t fw_patch;
} ld2410s_t;

/** Installs the UART driver (115200 8N1) on the given pins. */
esp_err_t ld2410s_init(ld2410s_t* dev, uart_port_t port, int tx_gpio, int rx_gpio);
esp_err_t ld2410s_deinit(ld2410s_t* dev);

/**
 * Enters configuration mode (ESP_ERR_TIMEOUT if no LD2410S answers), reads the firmware version,
 * selects minimal reports, sets the unmanned delay and the status/distance report rate (in 0.1 Hz,
 * 5..80), then resumes normal operation.
 */
esp_err_t ld2410s_configure(ld2410s_t* dev, uint8_t unmanned_delay_s, uint8_t report_rate_tenth_hz);

/**
 * Waits up to timeout for the next minimal report frame.
 * @param occupied     true when the radar sees someone (target state 2/3)
 * @param distance_cm  target distance
 */
esp_err_t
ld2410s_read_report(ld2410s_t* dev, bool* occupied, uint16_t* distance_cm, TickType_t timeout);

#ifdef __cplusplus
}
#endif

#endif // LD2410S_H
