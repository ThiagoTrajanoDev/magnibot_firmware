#ifdef NDEBUG
#error "tests rely on assert() side effects; do not build with NDEBUG"
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "serial_protocol.h"
#include "command_control.h"

static protocol_message_t parse(const char *text) {
    serial_parser_t parser;
    serial_parser_init(&parser);
    protocol_message_t message = {0};
    unsigned events = 0;
    for (size_t i = 0; text[i]; ++i)
        if (serial_parser_feed(&parser, (uint8_t)text[i], (int64_t)i, &message)) ++events;
    assert(events == 1);
    return message;
}

int main(void) {
    const char *commands[] = {"status", "diagnostic", "start", "confirm", "rescan", "estop"};
    for (unsigned i = 0; i < 6; ++i) {
        char line[64];
        snprintf(line, sizeof(line), "MB1 %u %s\r\n", i, commands[i]);
        protocol_message_t m = parse(line);
        assert(m.error == PROTOCOL_OK && m.id_valid && m.id == i && m.command == (serial_command_t)i);
    }
    protocol_message_t m = parse("MB1 4294967295 status\n");
    assert(m.error == PROTOCOL_OK && m.id == UINT32_MAX);
    assert(parse("MB1 4294967296 status\n").error == PROTOCOL_BAD_ID);
    assert(parse("MB1 00000000000 status\n").error == PROTOCOL_BAD_ID);
    assert(parse("MB1 -1 status\n").error == PROTOCOL_BAD_ID);
    assert(parse("MB1 1.0 status\n").error == PROTOCOL_BAD_ID);
    assert(parse("MB1  status\n").error == PROTOCOL_BAD_ID);
    assert(parse("MB2 1 status\n").error == PROTOCOL_BAD_VERSION);
    assert(parse("MB1 1 restart\n").error == PROTOCOL_UNKNOWN_COMMAND);
    assert(parse("MB1 1 status extra\n").error == PROTOCOL_BAD_FRAME);
    assert(parse("MB1 1 \n").error == PROTOCOL_BAD_FRAME);
    assert(parse("\n").error == PROTOCOL_BAD_FRAME);
    assert(parse("MB1\t1 status\n").error == PROTOCOL_BAD_FRAME);
    m = parse("MB1 37 unknown\n");
    assert(m.id_valid && m.id == 37);

    serial_parser_t parser;
    serial_parser_init(&parser);
    const char *fragment = "MB1 81 estop";
    for (size_t i = 0; fragment[i]; ++i)
        assert(!serial_parser_feed(&parser, fragment[i], i, &m));
    assert(!serial_parser_expire(&parser, 999, &m));
    assert(serial_parser_expire(&parser, 1000, &m) && m.error == PROTOCOL_TIMEOUT);
    assert(!serial_parser_expire(&parser, 5000, &m));
    assert(!serial_parser_feed(&parser, '\n', 5001, &m));
    for (size_t i = 0; fragment[i]; ++i)
        assert(!serial_parser_feed(&parser, fragment[i], 5002 + i, &m));
    assert(serial_parser_feed(&parser, '\n', 5016, &m));
    assert(m.error == PROTOCOL_OK && m.command == COMMAND_ESTOP);

    serial_parser_init(&parser);
    for (unsigned i = 0; i < SERIAL_LINE_MAX + 100; ++i)
        assert(!serial_parser_feed(&parser, 'x', 0, &m));
    assert(serial_parser_feed(&parser, '\n', 0, &m) && m.error == PROTOCOL_TOO_LONG);
    assert(parser.length == 0 && !parser.discard);
    assert(!serial_parser_feed(&parser, 0, 0, &m));
    assert(serial_parser_feed(&parser, '\n', 0, &m) && m.error == PROTOCOL_BAD_FRAME);
    assert(!serial_parser_feed(NULL, 0, 0, &m));
    assert(!serial_parser_expire(NULL, 0, &m));

    /* Deterministic arbitrary byte stream exercises bounded storage and recovery. */
    unsigned value = 12345;
    for (unsigned i = 0; i < 100000; ++i) {
        value = value * 1664525u + 1013904223u;
        serial_parser_feed(&parser, (uint8_t)(value >> 24), i, &m);
        serial_parser_expire(&parser, i, &m);
        assert(parser.length <= SERIAL_LINE_MAX);
    }
    serial_parser_feed(&parser, '\n', 100001, &m);
    const char *status = "MB1 9 status\n";
    for (size_t i = 0; status[i]; ++i) {
        bool ready = serial_parser_feed(&parser, status[i], 100002 + i, &m);
        assert(ready == (status[i] == '\n'));
    }
    assert(m.error == PROTOCOL_OK && m.id == 9);

    assert(command_control_execute(NULL, COMMAND_ESTOP) == COMMAND_INVALID);
    for (int s = ROBOT_IDLE; s <= ROBOT_ERROR; ++s) {
        robot_state_t state = (robot_state_t)s;
        assert(command_control_execute(&state, COMMAND_STATUS) == COMMAND_ACCEPTED);
        assert(command_control_execute(&state, COMMAND_DIAGNOSTIC) == COMMAND_ACCEPTED);
        assert(state == (robot_state_t)s);
        for (int c = COMMAND_START; c <= COMMAND_RESCAN; ++c) {
            assert(command_control_execute(&state, (serial_command_t)c) ==
                   (s == ROBOT_ERROR ? COMMAND_EMERGENCY_LATCHED : COMMAND_MOTION_BLOCKED));
            assert(state == (robot_state_t)s);
        }
        assert(command_control_execute(&state, COMMAND_ESTOP) == COMMAND_ACCEPTED);
        assert(command_control_execute(&state, COMMAND_ESTOP) == COMMAND_ACCEPTED);
        assert(state == ROBOT_ERROR);
        assert(command_control_execute(&state, (serial_command_t)99) == COMMAND_INVALID);
        assert(state == ROBOT_ERROR);
    }
    puts("PASS: serial grammar, bounds, fragmentation, timeout, recovery, arbitrary bytes, command safety");
}
