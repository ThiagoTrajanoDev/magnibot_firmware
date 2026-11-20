#ifndef COMMAND_CONTROL_H
#define COMMAND_CONTROL_H
#include "serial_protocol.h"
#include "state_machine.h"
#include "bench_servo.h"
#include "cycle_sequence.h"
typedef enum {
    COMMAND_ACCEPTED, COMMAND_MOTION_BLOCKED, COMMAND_EMERGENCY_LATCHED,
    COMMAND_INVALID,
    COMMAND_SERVO_DISABLED, COMMAND_SERVO_GPIO, COMMAND_SERVO_ANGLE,
    COMMAND_SERVO_NO_CHANNEL, COMMAND_SERVO_FAULT, COMMAND_POSE_NOT_READY
} command_result_t;
command_result_t command_control_execute(robot_state_t *state, serial_command_t command);
command_result_t command_control_cycle(cycle_sequence_t *cycle, const cycle_plan_t *plan,
                                       serial_command_t command, int64_t now_ms);
/* Bench servo command; refused after an emergency. A driver fault also forces ERROR. */
command_result_t command_control_servo(robot_state_t *state, bench_servo_t *servos,
                                       int gpio, float degrees);
const char *command_control_result_name(command_result_t result);
#endif
