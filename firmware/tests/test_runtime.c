#ifdef NDEBUG
#error "tests rely on assert() side effects; do not build with NDEBUG"
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "robot_runtime.h"

static int64_t sample_time;
static unsigned requests, cancels;
static float distance = 123;
static hardware_result_t request(void *context, int64_t now) {
    (void)context; sample_time = now; ++requests; return IO_OK;
}
static hardware_result_t poll(void *context, float *value, int64_t *timestamp) {
    (void)context; *value = distance; *timestamp = sample_time; return IO_OK;
}
static void cancel(void *context) { (void)context; ++cancels; }

int main(void) {
    /* Not initialised: fail closed, never IDLE. */
    robot_state_t state = ROBOT_IDLE;
    robot_runtime_poll(&state, 0);
    assert(state == ROBOT_ERROR);
    robot_runtime_status_t status = robot_runtime_status(0);
    assert(status.axes_configured == 0 && strcmp(status.backend, "disabled") == 0);

    /* The single boot-time init: poll never retries it. */
    state = ROBOT_IDLE;
    sensor_driver_t driver = {.request=request,.poll=poll,.cancel=cancel,.source=SENSOR_HARDWARE};
    assert(robot_runtime_init_with_distance(&driver));
    robot_runtime_poll(&state, 0);
    status = robot_runtime_status(0);
    assert(state == ROBOT_IDLE && status.axes_configured == 0 && status.distance_status == SENSOR_UNAVAILABLE);
    assert(status.distance_io == IO_PENDING && status.current_io == IO_DISABLED && status.actuator_io == IO_BLOCKED);
    assert(!status.actuator_shutdown_confirmed && strcmp(status.backend, "disabled") == 0);

    /* The real bench workflow owns these states, even with the generic cycle idle. */
    const robot_state_t physical_states[] = {ROBOT_RUN_VARREDURA, ROBOT_VARREDURA_CONCLUIDA,
                                            ROBOT_RUN_DESLOCAMENTO};
    for (unsigned i=0;i<sizeof(physical_states)/sizeof(physical_states[0]);++i) {
        state = physical_states[i];
        robot_runtime_poll_external(&state, 0);
        assert(state == physical_states[i]);
    }
    state = ROBOT_ERROR;
    robot_runtime_poll(&state, 10);
    status = robot_runtime_status(10);
    assert(state == ROBOT_ERROR && status.phase == CYCLE_ERROR && status.fault == CYCLE_EMERGENCY);
    assert(status.distance_io == IO_OK && status.current_io == IO_DISABLED && status.actuator_io == IO_LATCHED);
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

    float mm;
    assert(robot_runtime_distance(30, &mm) && mm == 123 && cancels == 0);
    distance = 150;
    robot_runtime_poll(&state, 60);
    robot_runtime_poll(&state, 70);
    assert(state == ROBOT_ERROR && requests == 2 && cancels == 0);
    assert(robot_runtime_distance(70, &mm) && mm == 150);
    assert(robot_runtime_status(70).actuator_io == IO_LATCHED);

    /* Sensor-only recovery cannot release actuator/cycle emergency latches. */
    assert(robot_runtime_set_distance_driver(NULL));
    assert(!robot_runtime_distance(70, &mm));
    assert(robot_runtime_set_distance_driver(&driver));
    distance = 160;
    robot_runtime_poll(&state, 80);
    robot_runtime_poll(&state, 90);
    assert(state == ROBOT_ERROR && requests == 3 && cancels == 0);
    assert(robot_runtime_distance(90, &mm) && mm == 160);
    assert(robot_runtime_status(90).actuator_io == IO_LATCHED);

    /* Clock regression after the latch does not release anything. */
    state = ROBOT_IDLE;
    robot_runtime_poll(&state, 5);
    assert(state == ROBOT_ERROR);
    assert(!robot_runtime_set_distance_driver(&driver));
    puts("PASS: runtime fail-closed, single init, latch survives repoll/reinit, shutdown confirmed");
}
