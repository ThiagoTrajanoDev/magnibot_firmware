#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SERIAL_LINE_MAX 96
/* Long enough for a person typing a line in a serial terminal. */
#define SERIAL_FRAME_TIMEOUT_MS 5000

typedef enum {
    COMMAND_STATUS, COMMAND_DIAGNOSTIC, COMMAND_START,
    COMMAND_CONFIRM, COMMAND_RESCAN, COMMAND_ESTOP,
    COMMAND_SERVO  /* bare "<gpio> <degrees>" line, no MB1 frame */
} serial_command_t;
typedef enum {
    PROTOCOL_OK, PROTOCOL_BAD_FRAME, PROTOCOL_BAD_VERSION, PROTOCOL_BAD_ID,
    PROTOCOL_UNKNOWN_COMMAND, PROTOCOL_TOO_LONG, PROTOCOL_TIMEOUT
} protocol_error_t;
typedef struct {
    uint32_t id;
    bool id_valid;
    serial_command_t command;
    protocol_error_t error;
    int servo_gpio, servo_degrees;  /* valid for COMMAND_SERVO */
} protocol_message_t;
typedef struct {
    char line[SERIAL_LINE_MAX + 1];
    size_t length;
    int64_t started_ms;
    bool active;
    bool discard;
    bool error_reported;
    protocol_error_t pending_error;
} serial_parser_t;

void serial_parser_init(serial_parser_t *parser);
bool serial_parser_feed(serial_parser_t *parser, uint8_t byte, int64_t now_ms,
                        protocol_message_t *message);
bool serial_parser_expire(serial_parser_t *parser, int64_t now_ms,
                          protocol_message_t *message);
const char *serial_protocol_error_name(protocol_error_t error);
#endif
