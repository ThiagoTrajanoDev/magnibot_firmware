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
        case COMMAND_IDLE_POSE:
        case COMMAND_CONFIRM:
        case COMMAND_RESCAN:
            return *state == ROBOT_ERROR ? COMMAND_EMERGENCY_LATCHED : COMMAND_MOTION_BLOCKED;
        default:
            return COMMAND_INVALID;
    }
}
command_result_t command_control_cycle(cycle_sequence_t *cycle, const cycle_plan_t *plan,
                                       serial_command_t command, int64_t now_ms) {
    robot_event_t event;
    switch (command) {
        case COMMAND_START: event = EVENT_SINGLE_CLICK; break;
        case COMMAND_CONFIRM: event = EVENT_DOUBLE_CLICK; break;
        case COMMAND_RESCAN: event = EVENT_LONG_PRESS; break;
        case COMMAND_ESTOP: event = EVENT_EMERGENCY; break;
        default: return COMMAND_INVALID;
    }
    if (!cycle || (command != COMMAND_ESTOP && !plan)) return COMMAND_MOTION_BLOCKED;
    if (cycle->phase == CYCLE_ERROR)
        return command == COMMAND_ESTOP ? COMMAND_ACCEPTED : COMMAND_EMERGENCY_LATCHED;
    return cycle_handle_event(cycle, plan, event, now_ms) ? COMMAND_ACCEPTED : COMMAND_MOTION_BLOCKED;
}

command_result_t command_control_servo(robot_state_t *state, bench_servo_t *servos,
                                       int gpio, float degrees) {
    if (!state) return COMMAND_INVALID;
    if (*state == ROBOT_ERROR) return COMMAND_EMERGENCY_LATCHED;
    if (*state != ROBOT_IDLE || (servos && (servos->scan_pose_active || servos->base_active)))
        return COMMAND_MOTION_BLOCKED;
    switch (bench_servo_set(servos, gpio, degrees)) {
        case BENCH_SERVO_OK: return COMMAND_ACCEPTED;
        case BENCH_SERVO_DISABLED: return COMMAND_SERVO_DISABLED;
        case BENCH_SERVO_BAD_GPIO: return COMMAND_SERVO_GPIO;
        case BENCH_SERVO_BAD_ANGLE: return COMMAND_SERVO_ANGLE;
        case BENCH_SERVO_NO_CHANNEL: return COMMAND_SERVO_NO_CHANNEL;
        default:
            bench_control_event(state, EVENT_EMERGENCY);
            return COMMAND_SERVO_FAULT;
    }
}
command_result_t command_control_prepare_scan(robot_state_t *state, bench_servo_t *servos) {
    if (!state) return COMMAND_INVALID;
    if (*state == ROBOT_ERROR) return COMMAND_EMERGENCY_LATCHED;
    if (*state != ROBOT_IDLE) return COMMAND_MOTION_BLOCKED;
    if (!bench_servo_enabled(servos)) return COMMAND_SERVO_DISABLED;
    return bench_servo_prepare_scan(servos) ? COMMAND_ACCEPTED : COMMAND_POSE_NOT_READY;
}

command_result_t command_control_scan_event(robot_state_t *state, bench_servo_t *servos,
    base_scan_t *scan, robot_event_t event, int64_t now_ms) {
    if (!state || !scan) return COMMAND_INVALID;
    if (*state == ROBOT_ERROR) return COMMAND_EMERGENCY_LATCHED;
    if (scan->latched || scan->fault || scan->moving || !scan->driver.write_us ||
        !scan->driver.shutdown || now_ms < 0 || now_ms < scan->last_ms)
        return COMMAND_MOTION_BLOCKED;
    if ((*state == ROBOT_IDLE && event == EVENT_SINGLE_CLICK) ||
        (*state == ROBOT_VARREDURA_CONCLUIDA && event == EVENT_LONG_PRESS)) {
        robot_state_t idle = ROBOT_IDLE;
        command_result_t result = command_control_prepare_scan(&idle, servos);
        if (result != COMMAND_ACCEPTED) return result;
        scan->found = scan->no_object = scan->home = scan->at_object = false;
        scan->found_distance_mm = 0;
        scan->found_offset_ms = 0;
        state_machine_process_event(state, event);
        return COMMAND_ACCEPTED;
    }
    if (*state == ROBOT_VARREDURA_CONCLUIDA && event == EVENT_DOUBLE_CLICK) {
        if (!base_scan_replay(scan, now_ms)) {
            if (scan->fault) state_machine_process_event(state, EVENT_EMERGENCY);
            return COMMAND_MOTION_BLOCKED;
        }
        state_machine_process_event(state, event);
        return COMMAND_ACCEPTED;
    }
    return COMMAND_MOTION_BLOCKED;
}

const char *command_control_result_name(command_result_t result) {
    switch (result) {
        case COMMAND_ACCEPTED: return "ok";
        case COMMAND_MOTION_BLOCKED: return "motion_blocked";
        case COMMAND_EMERGENCY_LATCHED: return "emergency_latched";
        case COMMAND_SERVO_DISABLED: return "servo_disabled";
        case COMMAND_SERVO_GPIO: return "gpio_not_allowed";
        case COMMAND_SERVO_ANGLE: return "angle_out_of_range";
        case COMMAND_SERVO_NO_CHANNEL: return "no_free_channel";
        case COMMAND_SERVO_FAULT: return "servo_fault";
        case COMMAND_POSE_NOT_READY: return "pose_not_ready";
        default: return "invalid_command";
    }
}
