#ifndef ROBOT_RUNTIME_H
#define ROBOT_RUNTIME_H
#include "cycle_sequence.h"
#include <stdbool.h>
#include "hardware_io.h"
/* Fixed bench-mode fields shared by every JSON frame. motion_enabled covers the cycle/actuator
 * backend only; bench servos (opt-in, Kconfig) are reported separately as "bench_servo". */
#define ROBOT_MODE_JSON "\"mode\":\"bench\",\"motion_enabled\":false"
typedef struct {
    const char *backend;
    bool actuator_shutdown_confirmed;
    unsigned axes_configured;
    motion_state_t motion_state;
    cycle_phase_t phase;
    cycle_fault_t fault;
    sensor_status_t distance_status, current_status;
    hardware_result_t distance_io, current_io, actuator_io;
} robot_runtime_status_t;
/* One boot-time init: a failed or repeated call is final (never retried or reset). */
bool robot_runtime_init(void);
void robot_runtime_poll(robot_state_t *state, int64_t now_ms);
robot_runtime_status_t robot_runtime_status(int64_t now_ms);
#endif
