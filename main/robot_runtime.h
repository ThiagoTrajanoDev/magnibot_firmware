#ifndef ROBOT_RUNTIME_H
#define ROBOT_RUNTIME_H
#include "cycle_sequence.h"
#include "hardware_io.h"
typedef struct {
    unsigned axes_configured;
    motion_state_t motion_state;
    cycle_phase_t phase;
    cycle_fault_t fault;
    sensor_status_t distance_status, current_status;
    hardware_result_t distance_io, current_io, actuator_io;
} robot_runtime_status_t;
bool robot_runtime_init(void);
void robot_runtime_poll(robot_state_t *state, int64_t now_ms);
robot_runtime_status_t robot_runtime_status(int64_t now_ms);
#endif
