#include "command_control.h"
#include "bench_control.h"
#include <stddef.h>
command_result_t command_control_execute(robot_state_t *state, serial_command_t command) {
    if (!state) return COMMAND_INVALID;
    switch (command) {
        case COMMAND_STATUS:
        case COMMAND_DIAGNOSTIC:
            return COMMAND_ACCEPTED;
        case COMMAND_ESTOP:
            bench_control_event(state, EVENT_EMERGENCY);
            return COMMAND_ACCEPTED;
        case COMMAND_START:
        case COMMAND_CONFIRM:
        case COMMAND_RESCAN:
            return *state == ROBOT_ERROR ? COMMAND_EMERGENCY_LATCHED : COMMAND_MOTION_BLOCKED;
        default:
            return COMMAND_INVALID;
    }
}
const char *command_control_result_name(command_result_t result) {
    switch (result) {
        case COMMAND_ACCEPTED: return "ok";
        case COMMAND_MOTION_BLOCKED: return "motion_blocked";
        case COMMAND_EMERGENCY_LATCHED: return "emergency_latched";
        default: return "invalid_command";
    }
}
