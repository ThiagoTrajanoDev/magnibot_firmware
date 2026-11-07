#include "robot_runtime.h"

static motion_model_t motion;
static sensor_data_t sensors;
static cycle_sequence_t cycle;
static bool initialized;

bool robot_runtime_init(void) {
    axis_limits_t limits[MOTION_AXES];
    motion_default_limits(limits);
    sensor_data_init(&sensors);
    initialized = motion_model_init(&motion, limits);
    cycle_sequence_init(&cycle, &motion, &sensors);
    return initialized;
}

void robot_runtime_poll(robot_state_t *state, int64_t now_ms) {
    if (!state) return;
    if (!initialized && !robot_runtime_init()) { *state = ROBOT_ERROR; return; }
    if (*state == ROBOT_ERROR) cycle_abort(&cycle, CYCLE_EMERGENCY);
    cycle_tick(&cycle, now_ms);
    *state = cycle_robot_state(&cycle);
}

robot_runtime_status_t robot_runtime_status(int64_t now_ms) {
    robot_runtime_status_t result = {
        .distance_status = SENSOR_UNAVAILABLE, .current_status = SENSOR_UNAVAILABLE
    };
    if (!initialized) return result;
    result.axes_configured = motion_configured_axes(&motion);
    result.motion_state = motion.state;
    result.phase = cycle.phase;
    result.fault = cycle.fault;
    result.distance_status = sensor_sample_check(&sensors.distance_mm, now_ms, 500, SENSOR_HARDWARE);
    result.current_status = sensor_sample_check(&sensors.current_ma, now_ms, 500, SENSOR_HARDWARE);
    return result;
}
