#include "serial_control.h"
#include "serial_protocol.h"
#include "command_control.h"
#include "led_control.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>

#define RESPONSE_CAPACITY 16
#define RX_BUDGET 256

typedef struct {
    protocol_message_t message;
    command_result_t result;
    robot_state_t state;
} response_t;

static serial_parser_t parser;
static response_t responses[RESPONSE_CAPACITY];
static unsigned head, count;
static uint32_t rx_messages, protocol_errors, dropped_responses;
static bool initialized;

static void increment(uint32_t *value) {
    if (*value != UINT32_MAX) ++*value;
}

esp_err_t serial_control_init(void) {
#if !CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG && !CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (initialized) return ESP_OK;
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t config = {
            .rx_buffer_size = 2048, .tx_buffer_size = 2048
        };
        esp_err_t err = usb_serial_jtag_driver_install(&config);
        if (err != ESP_OK) return err;
    }
    usb_serial_jtag_vfs_use_driver();
    serial_parser_init(&parser);
    initialized = true;
    return ESP_OK;
#endif
}

static void accept_message(const protocol_message_t *message, robot_state_t *state) {
    command_result_t result = COMMAND_INVALID;
    increment(&rx_messages);
    if (message->error == PROTOCOL_OK) {
        robot_state_t previous = *state;
        result = command_control_execute(state, message->command);
        if (*state != previous) led_control_update(*state);
    } else {
        increment(&protocol_errors);
    }
    /* Emergency execution never depends on free response queue space. */
    if (count == RESPONSE_CAPACITY) {
        increment(&dropped_responses);
        return;
    }
    responses[(head + count) % RESPONSE_CAPACITY] = (response_t){
        .message = *message, .result = result, .state = *state
    };
    ++count;
}

static void send_response(const response_t *response) {
    const protocol_message_t *message = &response->message;
    const char *code = message->error == PROTOCOL_OK
        ? command_control_result_name(response->result)
        : serial_protocol_error_name(message->error);
    bool ok = message->error == PROTOCOL_OK && response->result == COMMAND_ACCEPTED;
    char id[16] = "null";
    if (message->id_valid) snprintf(id, sizeof(id), "%" PRIu32, message->id);
    char diagnostic[320] = "";
    if (message->error == PROTOCOL_OK && message->command == COMMAND_DIAGNOSTIC) {
        snprintf(diagnostic, sizeof(diagnostic),
                 ",\"diagnostic\":{\"transport\":\"usb_serial_jtag\","
                 "\"max_line_bytes\":%d,\"frame_timeout_ms\":%d,"
                 "\"rx_messages\":%" PRIu32 ",\"protocol_errors\":%" PRIu32
                 ",\"dropped_responses\":%" PRIu32 ",\"estop_configured\":%s}",
                 SERIAL_LINE_MAX, SERIAL_FRAME_TIMEOUT_MS, rx_messages,
                 protocol_errors, dropped_responses,
                 CONFIG_MAGNIBOT_ESTOP_GPIO >= 0 ? "true" : "false");
    }
    printf("{\"type\":\"response\",\"version\":1,\"id\":%s,\"ok\":%s,"
           "\"code\":\"%s\",\"state\":\"%s\",\"state_code\":%d,"
           "\"mode\":\"bench\",\"motion_enabled\":false%s}\n",
           id, ok ? "true" : "false", code,
           state_machine_get_name(response->state), response->state, diagnostic);
}

void serial_control_poll(robot_state_t *state) {
    if (!initialized || !state) return;
    uint8_t data[RX_BUDGET];
    protocol_message_t message;
    int64_t now = esp_timer_get_time() / 1000;
    if (serial_parser_expire(&parser, now, &message)) accept_message(&message, state);
    int received = usb_serial_jtag_read_bytes(data, sizeof(data), 0);
    for (int i = 0; i < received; ++i) {
        if (serial_parser_feed(&parser, data[i], now, &message))
            accept_message(&message, state);
    }
    /* Process bounded RX, including emergency, before emitting one reply. */
    if (count) {
        send_response(&responses[head]);
        head = (head + 1) % RESPONSE_CAPACITY;
        --count;
    }
}
