#include "robot_runtime.h"
#include "sensor_interface.h"
#include "actuator_interface.h"
#include <stddef.h>

static motion_model_t motion;
static sensor_data_t sensors;
static cycle_sequence_t cycle;
static sensor_interface_t acquisition;
static actuator_interface_t output;
static bool initialized;

bool robot_runtime_init(void) {
    axis_limits_t limits[MOTION_AXES];
    motion_default_limits(limits);
    sensor_data_init(&sensors);
    bool motion_ok = motion_model_init(&motion, limits);
    bool sensors_ok = sensor_interface_init(&acquisition, &sensors, NULL, NULL, 50, 200, 500);
    bool actuators_ok = actuator_interface_init(&output, NULL, limits);
    initialized = motion_ok && sensors_ok && actuators_ok;
    cycle_sequence_init(&cycle, &motion, &sensors);
    return initialized;
}

void robot_runtime_poll(robot_state_t *state, int64_t now_ms) {
    if (!state) return;
    if (!initialized && !robot_runtime_init()) { *state = ROBOT_ERROR; return; }
    if (*state == ROBOT_ERROR) {
        actuator_interface_stop(&output);
        sensor_interface_stop(&acquisition);
        cycle_abort(&cycle, CYCLE_EMERGENCY);
    }
    sensor_interface_poll(&acquisition, now_ms);
    cycle_tick(&cycle, now_ms);
    *state = cycle_robot_state(&cycle);
    if (*state == ROBOT_ERROR) {
        actuator_interface_stop(&output);
        sensor_interface_stop(&acquisition);
    }
}

robot_runtime_status_t robot_runtime_status(int64_t now_ms) {
    robot_runtime_status_t result = {
        .distance_status = SENSOR_UNAVAILABLE, .current_status = SENSOR_UNAVAILABLE,
        .distance_io = IO_DISABLED, .current_io = IO_DISABLED, .actuator_io = IO_BLOCKED
    };
    if (!initialized) return result;
    result.axes_configured = motion_configured_axes(&motion);
    result.motion_state = motion.state;
    result.distance_io = acquisition.distance.last_result;
    result.current_io = acquisition.current.last_result;
    result.actuator_io = output.last_result;
    result.phase = cycle.phase;
    result.fault = cycle.fault;
    result.distance_status = sensor_sample_check(&sensors.distance_mm, now_ms, 500, SENSOR_HARDWARE);
    result.current_status = sensor_sample_check(&sensors.current_ma, now_ms, 500, SENSOR_HARDWARE);
    return result;
}
