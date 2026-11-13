#include "serial_control.h"
#include "serial_protocol.h"
#include "command_control.h"
#include "robot_runtime.h"
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
static bench_servo_t *bench_servos;

static void increment(uint32_t *value) {
    if (*value != UINT32_MAX) ++*value;
}

esp_err_t serial_control_init(bench_servo_t *servos) {
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
    bench_servos = servos;
    initialized = true;
    return ESP_OK;
#endif
}

static void accept_message(const protocol_message_t *message, robot_state_t *state) {
    command_result_t result = COMMAND_INVALID;
    increment(&rx_messages);
    if (message->error == PROTOCOL_OK) {
        result = message->command == COMMAND_SERVO
            ? command_control_servo(state, bench_servos, message->servo_gpio, message->servo_degrees)
            : command_control_execute(state, message->command);
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
    char diagnostic[768] = "";
    char servo[64] = "";
    if (message->error == PROTOCOL_OK && message->command == COMMAND_DIAGNOSTIC) {
        robot_runtime_status_t runtime = robot_runtime_status(esp_timer_get_time() / 1000);
        int written = snprintf(diagnostic, sizeof(diagnostic),
                 ",\"diagnostic\":{\"transport\":\"usb_serial_jtag\","
                 "\"max_line_bytes\":%d,\"frame_timeout_ms\":%d,"
                 "\"rx_messages\":%" PRIu32 ",\"protocol_errors\":%" PRIu32
                 ",\"dropped_responses\":%" PRIu32 ",\"estop_configured\":%s,"
                 "\"backend\":\"%s\",\"actuator_shutdown_confirmed\":%s,\"axes_configured\":%u,"
                 "\"sequence_phase\":\"%s\",\"sequence_fault\":\"%s\","
                 "\"distance_status\":\"%s\",\"current_status\":\"%s\","
                 "\"distance_io\":\"%s\",\"current_io\":\"%s\",\"actuator_io\":\"%s\","
                 "\"bench_servo\":{\"enabled\":%s,\"channels\":%u}}",
                 SERIAL_LINE_MAX, SERIAL_FRAME_TIMEOUT_MS, rx_messages,
                 protocol_errors, dropped_responses,
                 CONFIG_MAGNIBOT_ESTOP_GPIO >= 0 ? "true" : "false",
                 runtime.backend, runtime.actuator_shutdown_confirmed ? "true" : "false",
                 runtime.axes_configured, cycle_phase_name(runtime.phase), cycle_fault_name(runtime.fault),
                 sensor_status_name(runtime.distance_status), sensor_status_name(runtime.current_status),
                 hardware_result_name(runtime.distance_io), hardware_result_name(runtime.current_io),
                 hardware_result_name(runtime.actuator_io),
                 bench_servo_enabled(bench_servos) ? "true" : "false", bench_servo_channels(bench_servos));
        if (written < 0 || (size_t)written >= sizeof(diagnostic)) {
            diagnostic[0] = '\0';
            code = "internal_error";
            ok = false;
        }
    }
    if (ok && message->command == COMMAND_SERVO)
        snprintf(servo, sizeof(servo), ",\"servo\":{\"gpio\":%d,\"target_deg\":%d}",
                 message->servo_gpio, message->servo_degrees);
    printf("{\"type\":\"response\",\"version\":1,\"id\":%s,\"ok\":%s,"
           "\"code\":\"%s\",\"state\":\"%s\",\"state_code\":%d,"
           ROBOT_MODE_JSON "%s%s}\n",
           id, ok ? "true" : "false", code,
           state_machine_get_name(response->state), response->state, servo, diagnostic);
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
}

void serial_control_flush(void) {
    if (!initialized) return;
    if (count) {
        send_response(&responses[head]);
        head = (head + 1) % RESPONSE_CAPACITY;
        --count;
    }
}
