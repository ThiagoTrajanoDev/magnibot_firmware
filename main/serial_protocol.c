#include "serial_protocol.h"
#include <string.h>

void serial_parser_init(serial_parser_t *parser) {
    if (parser) memset(parser, 0, sizeof(*parser));
}

static void parse_line(char *line, protocol_message_t *message) {
    *message = (protocol_message_t){.error = PROTOCOL_BAD_FRAME};
    char *first = strchr(line, ' ');
    if (!first) return;
    *first++ = '\0';
    if (strcmp(line, "MB1") != 0) {
        message->error = PROTOCOL_BAD_VERSION;
        return;
    }
    char *second = strchr(first, ' ');
    if (!second) return;
    *second++ = '\0';
    message->error = PROTOCOL_BAD_ID;
    if (*first == '\0') return;
    uint32_t id = 0;
    size_t digits = 0;
    for (const char *p = first; *p; ++p) {
        if (++digits > 10 || *p < '0' || *p > '9') return;
        unsigned digit = (unsigned)(*p - '0');
        if (id > (UINT32_MAX - digit) / 10) return;
        id = id * 10 + digit;
    }
    message->id = id;
    message->id_valid = true;
    message->error = PROTOCOL_BAD_FRAME;
    if (*second == '\0' || strchr(second, ' ')) return;
    static const char *const names[] = {
        "status", "diagnostic", "start", "confirm", "rescan", "estop"
    };
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); ++i) {
        if (strcmp(second, names[i]) == 0) {
            message->command = (serial_command_t)i;
            message->error = PROTOCOL_OK;
            return;
        }
    }
    message->error = PROTOCOL_UNKNOWN_COMMAND;
}

bool serial_parser_expire(serial_parser_t *parser, int64_t now_ms,
                          protocol_message_t *message) {
    if (!parser || !message || !parser->active || parser->error_reported ||
        now_ms - parser->started_ms < SERIAL_FRAME_TIMEOUT_MS) return false;
    *message = (protocol_message_t){.error = PROTOCOL_TIMEOUT};
    parser->discard = true;
    parser->error_reported = true;
    return true;
}

bool serial_parser_feed(serial_parser_t *parser, uint8_t byte, int64_t now_ms,
                        protocol_message_t *message) {
    if (!parser || !message) return false;
    if (byte == '\n') {
        bool emit = !parser->error_reported;
        *message = (protocol_message_t){.error = parser->pending_error};
        if (!parser->discard) {
            if (parser->length && parser->line[parser->length - 1] == '\r')
                --parser->length;
            parser->line[parser->length] = '\0';
            parse_line(parser->line, message);
        }
        serial_parser_init(parser);
        return emit;
    }
    if (!parser->active) {
        parser->active = true;
        parser->started_ms = now_ms;
    }
    if (parser->discard) return false;
    if (parser->length == SERIAL_LINE_MAX) {
        parser->discard = true;
        parser->pending_error = PROTOCOL_TOO_LONG;
        return false;
    }
    if ((byte < 32 && byte != '\r') || byte > 126) {
        parser->discard = true;
        parser->pending_error = PROTOCOL_BAD_FRAME;
        return false;
    }
    parser->line[parser->length++] = (char)byte;
    return false;
}

const char *serial_protocol_error_name(protocol_error_t error) {
    switch (error) {
        case PROTOCOL_OK: return "ok";
        case PROTOCOL_BAD_FRAME: return "bad_frame";
        case PROTOCOL_BAD_VERSION: return "bad_version";
        case PROTOCOL_BAD_ID: return "bad_id";
        case PROTOCOL_UNKNOWN_COMMAND: return "unknown_command";
        case PROTOCOL_TOO_LONG: return "too_long";
        case PROTOCOL_TIMEOUT: return "timeout";
        default: return "internal_error";
    }
}
