#include "ld2410s.h"

#include <esp_check.h>
#include <freertos/task.h>
#include <string.h>

static const char* const TAG = "ld2410s";

#define BAUD_RATE      115200
#define RX_BUFFER_SIZE 256

// Command / ACK frame: header, 2-byte length (command word + value), command word, value, footer.
static const uint8_t CMD_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CMD_FOOTER[4] = {0x04, 0x03, 0x02, 0x01};
#define ACK_FLAG      0x0100 // ACK command word = command word | 0x0100
#define MAX_CMD_VALUE 36
#define MAX_ACK_VALUE 64

// Minimal report: head, target state (0/1 no one, 2/3 someone), 2-byte distance (cm), end.
#define REPORT_HEAD 0x6E
#define REPORT_END  0x62
#define REPORT_SIZE 5

#define CMD_ENABLE_CONFIG 0x00FF
#define CMD_END_CONFIG    0x00FE
#define CMD_READ_FIRMWARE 0x0000
#define CMD_OUTPUT_MODE   0x007A
#define CMD_WRITE_PARAMS  0x0070

#define PARAM_UNMANNED_DELAY     0x06
#define PARAM_STATUS_REPORT_HZ   0x02
#define PARAM_DISTANCE_REPORT_HZ 0x0C

#define ACK_TIMEOUT_MS 300

#define CHECK_ARG(VAL)                                                                             \
    do {                                                                                           \
        if (!(VAL)) return ESP_ERR_INVALID_ARG;                                                    \
    } while (0)

static inline void put_u16(uint8_t* buf, uint16_t value) {
    buf[0] = (uint8_t)(value & 0xff);
    buf[1] = (uint8_t)(value >> 8);
}

static inline void put_u32(uint8_t* buf, uint32_t value) {
    put_u16(buf, (uint16_t)(value & 0xffff));
    put_u16(buf + 2, (uint16_t)(value >> 16));
}

static inline uint16_t get_u16(const uint8_t* buf) {
    return (uint16_t)(buf[0] | (buf[1] << 8));
}

static esp_err_t read_exact(ld2410s_t* dev, uint8_t* buf, size_t size, TickType_t deadline) {
    size_t got = 0;
    while (got < size) {
        const TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0) return ESP_ERR_TIMEOUT;
        const int n = uart_read_bytes(dev->port, buf + got, size - got, deadline - now);
        if (n < 0) return ESP_FAIL;
        got += (size_t)n;
    }
    return ESP_OK;
}

// Skips report frames and noise until the ACK for cmd; value gets the bytes after the command word.
static esp_err_t read_ack(ld2410s_t* dev, uint16_t cmd, uint8_t* value, size_t* value_len) {
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ACK_TIMEOUT_MS) + 1;
    size_t matched = 0;
    while (matched < sizeof(CMD_HEADER)) {
        uint8_t byte = 0;
        ESP_RETURN_ON_ERROR(read_exact(dev, &byte, 1, deadline), TAG, "no ACK for 0x%04x", cmd);
        if (byte == CMD_HEADER[matched]) {
            matched++;
        } else {
            matched = (byte == CMD_HEADER[0]) ? 1 : 0;
        }
    }

    uint8_t len_buf[2];
    ESP_RETURN_ON_ERROR(read_exact(dev, len_buf, sizeof(len_buf), deadline), TAG, "short ACK");
    const uint16_t len = get_u16(len_buf);
    ESP_RETURN_ON_FALSE(len >= 2 && len <= 2 + MAX_ACK_VALUE,
                        ESP_ERR_INVALID_RESPONSE,
                        TAG,
                        "bad ACK length %u",
                        len);

    uint8_t body[2 + MAX_ACK_VALUE + sizeof(CMD_FOOTER)];
    ESP_RETURN_ON_ERROR(
        read_exact(dev, body, len + sizeof(CMD_FOOTER), deadline), TAG, "short ACK");
    ESP_RETURN_ON_FALSE(memcmp(body + len, CMD_FOOTER, sizeof(CMD_FOOTER)) == 0,
                        ESP_ERR_INVALID_RESPONSE,
                        TAG,
                        "bad ACK footer");
    ESP_RETURN_ON_FALSE(get_u16(body) == (cmd | ACK_FLAG),
                        ESP_ERR_INVALID_RESPONSE,
                        TAG,
                        "unexpected ACK 0x%04x for 0x%04x",
                        get_u16(body),
                        cmd);

    *value_len = len - 2;
    memcpy(value, body + 2, *value_len);
    return ESP_OK;
}

static esp_err_t send_command(ld2410s_t* dev,
                              uint16_t cmd,
                              const uint8_t* value,
                              size_t value_len,
                              uint8_t* ack_value,
                              size_t* ack_len) {
    uint8_t frame[sizeof(CMD_HEADER) + 4 + MAX_CMD_VALUE + sizeof(CMD_FOOTER)];
    if (value_len > MAX_CMD_VALUE) return ESP_ERR_INVALID_SIZE;

    size_t pos = 0;
    memcpy(frame, CMD_HEADER, sizeof(CMD_HEADER));
    pos += sizeof(CMD_HEADER);
    put_u16(frame + pos, (uint16_t)(2 + value_len));
    pos += 2;
    put_u16(frame + pos, cmd);
    pos += 2;
    if (value_len > 0) memcpy(frame + pos, value, value_len);
    pos += value_len;
    memcpy(frame + pos, CMD_FOOTER, sizeof(CMD_FOOTER));
    pos += sizeof(CMD_FOOTER);

    uart_flush_input(dev->port); // drop queued reports so the ACK is found quickly
    ESP_RETURN_ON_FALSE(
        uart_write_bytes(dev->port, frame, pos) == (int)pos, ESP_FAIL, TAG, "UART write failed");
    return read_ack(dev, cmd, ack_value, ack_len);
}

// Commands whose ACK value starts with a 2-byte status (0 = success).
static esp_err_t
send_checked(ld2410s_t* dev, uint16_t cmd, const uint8_t* value, size_t value_len) {
    uint8_t ack[MAX_ACK_VALUE];
    size_t ack_len = 0;
    ESP_RETURN_ON_ERROR(
        send_command(dev, cmd, value, value_len, ack, &ack_len), TAG, "command 0x%04x failed", cmd);
    ESP_RETURN_ON_FALSE(ack_len >= 2 && get_u16(ack) == 0,
                        ESP_ERR_INVALID_RESPONSE,
                        TAG,
                        "command 0x%04x rejected",
                        cmd);
    return ESP_OK;
}

esp_err_t ld2410s_init(ld2410s_t* dev, uart_port_t port, int tx_gpio, int rx_gpio) {
    CHECK_ARG(dev);

    memset(dev, 0, sizeof(*dev));
    dev->port = port;
    const uart_config_t config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(port, RX_BUFFER_SIZE, 0, 0, NULL, 0),
                        TAG,
                        "UART driver install failed");
    esp_err_t ret = uart_param_config(port, &config);
    if (ret == ESP_OK)
        ret = uart_set_pin(port, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        uart_driver_delete(port);
        ESP_LOGE(TAG, "UART setup failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t ld2410s_deinit(ld2410s_t* dev) {
    CHECK_ARG(dev);

    return uart_driver_delete(dev->port);
}

esp_err_t
ld2410s_configure(ld2410s_t* dev, uint8_t unmanned_delay_s, uint8_t report_rate_tenth_hz) {
    CHECK_ARG(dev);
    CHECK_ARG(unmanned_delay_s >= LD2410S_UNMANNED_DELAY_MIN_S &&
              unmanned_delay_s <= LD2410S_UNMANNED_DELAY_MAX_S);
    CHECK_ARG(report_rate_tenth_hz >= 5 && report_rate_tenth_hz <= 80 &&
              report_rate_tenth_hz % 5 == 0);

    uint8_t value[MAX_CMD_VALUE];
    put_u16(value, 0x0001);
    ESP_RETURN_ON_ERROR(
        send_checked(dev, CMD_ENABLE_CONFIG, value, 2), TAG, "enable configuration failed");

    uint8_t ack[MAX_ACK_VALUE];
    size_t ack_len = 0;
    esp_err_t ret = send_command(dev, CMD_READ_FIRMWARE, NULL, 0, ack, &ack_len);
    if (ret == ESP_OK && ack_len >= 6) {
        dev->fw_major = get_u16(ack);
        dev->fw_minor = get_u16(ack + 2);
        dev->fw_patch = get_u16(ack + 4);
    }

    if (ret == ESP_OK) {
        memset(value, 0, 6); // minimal output mode
        ret = send_checked(dev, CMD_OUTPUT_MODE, value, 6);
    }
    if (ret == ESP_OK) {
        const uint16_t words[3] = {
            PARAM_UNMANNED_DELAY, PARAM_STATUS_REPORT_HZ, PARAM_DISTANCE_REPORT_HZ};
        const uint32_t values[3] = {unmanned_delay_s, report_rate_tenth_hz, report_rate_tenth_hz};
        for (size_t i = 0; i < 3; i++) {
            put_u16(value + (i * 6), words[i]);
            put_u32(value + (i * 6) + 2, values[i]);
        }
        ret = send_checked(dev, CMD_WRITE_PARAMS, value, 18);
    }

    // Always try to resume normal operation, even after a failed step.
    const esp_err_t end_ret = send_checked(dev, CMD_END_CONFIG, NULL, 0);
    return ret != ESP_OK ? ret : end_ret;
}

esp_err_t
ld2410s_read_report(ld2410s_t* dev, bool* occupied, uint16_t* distance_cm, TickType_t timeout) {
    CHECK_ARG(dev && occupied && distance_cm);

    const TickType_t deadline = xTaskGetTickCount() + timeout;
    uint8_t frame[REPORT_SIZE];
    for (;;) {
        // Timeouts are normal between reports, so no logging here.
        esp_err_t ret = read_exact(dev, frame, 1, deadline);
        if (ret != ESP_OK) return ret;
        if (frame[0] != REPORT_HEAD) continue;
        ret = read_exact(dev, frame + 1, REPORT_SIZE - 1, deadline);
        if (ret != ESP_OK) return ret;
        if (frame[REPORT_SIZE - 1] != REPORT_END || frame[1] > 3) continue; // resync
        *occupied = frame[1] >= 2;
        *distance_cm = get_u16(frame + 2);
        return ESP_OK;
    }
}
