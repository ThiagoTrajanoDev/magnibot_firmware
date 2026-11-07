#ifndef COMMAND_CONTROL_H
#define COMMAND_CONTROL_H
#include "serial_protocol.h"
#include "state_machine.h"
typedef enum {
    COMMAND_ACCEPTED, COMMAND_MOTION_BLOCKED, COMMAND_EMERGENCY_LATCHED,
    COMMAND_INVALID
} command_result_t;
command_result_t command_control_execute(robot_state_t *state, serial_command_t command);
const char *command_control_result_name(command_result_t result);
#endif
