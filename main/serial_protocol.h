#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SERIAL_LINE_MAX 96
#define SERIAL_FRAME_TIMEOUT_MS 1000

typedef enum {
    COMMAND_STATUS, COMMAND_DIAGNOSTIC, COMMAND_START,
    COMMAND_CONFIRM, COMMAND_RESCAN, COMMAND_ESTOP
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
