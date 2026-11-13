#ifdef NDEBUG
#error "tests rely on assert() side effects; do not build with NDEBUG"
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "robot_runtime.h"

int main(void) {
    /* Not initialised: fail closed, never IDLE. */
    robot_state_t state = ROBOT_IDLE;
    robot_runtime_poll(&state, 0);
    assert(state == ROBOT_ERROR);
    robot_runtime_status_t status = robot_runtime_status(0);
    assert(status.axes_configured == 0 && strcmp(status.backend, "disabled") == 0);

    /* The single boot-time init: poll never retries it. */
    state = ROBOT_IDLE;
    assert(robot_runtime_init());
    robot_runtime_poll(&state, 0);
    status = robot_runtime_status(0);
    assert(state == ROBOT_IDLE && status.axes_configured == 0 && status.distance_status == SENSOR_UNAVAILABLE);
    assert(status.distance_io == IO_DISABLED && status.current_io == IO_DISABLED && status.actuator_io == IO_BLOCKED);
    assert(!status.actuator_shutdown_confirmed && strcmp(status.backend, "disabled") == 0);

    state = ROBOT_ERROR;
    robot_runtime_poll(&state, 10);
    status = robot_runtime_status(10);
    assert(state == ROBOT_ERROR && status.phase == CYCLE_ERROR && status.fault == CYCLE_EMERGENCY);
    assert(status.distance_io == IO_LATCHED && status.current_io == IO_LATCHED && status.actuator_io == IO_LATCHED);
    assert(status.actuator_shutdown_confirmed);

    /* ERROR survives a later poll with a clean state and a repeated init. */
    state = ROBOT_IDLE;
    robot_runtime_poll(&state, 20);
    assert(state == ROBOT_ERROR);
    assert(!robot_runtime_init());
    state = ROBOT_IDLE;
    robot_runtime_poll(&state, 30);
    status = robot_runtime_status(30);
    assert(state == ROBOT_ERROR && status.phase == CYCLE_ERROR && status.actuator_io == IO_LATCHED);

    /* Clock regression after the latch does not release anything. */
    state = ROBOT_IDLE;
    robot_runtime_poll(&state, 5);
    assert(state == ROBOT_ERROR);
    puts("PASS: runtime fail-closed, single init, latch survives repoll/reinit, shutdown confirmed");
}
