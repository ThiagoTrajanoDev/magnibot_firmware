#include "bench_control.h"
#include <stddef.h>
bool bench_control_event(robot_state_t *state, robot_event_t event) {
    if (state == NULL) return false;
    /* Never report an operation as running without hardware and an executor. */
    if (event != EVENT_EMERGENCY) return false;
    return state_machine_process_event(state, event);
}
