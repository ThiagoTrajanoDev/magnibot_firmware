#ifndef CYCLE_SEQUENCE_H
#define CYCLE_SEQUENCE_H
#include "motion_model.h"
#include "sensor_data.h"
#include "state_machine.h"
#define CYCLE_MAX_SCAN_POINTS 8
#define CYCLE_GRIPPER_AXIS 5

typedef enum {
    CYCLE_IDLE, CYCLE_SCAN_MOVE, CYCLE_SCAN_SAMPLE, CYCLE_SCAN_HOME,
    CYCLE_WAIT_CONFIRM, CYCLE_PICKUP, CYCLE_GRIP, CYCLE_LIFT,
    CYCLE_DEPOSIT, CYCLE_RELEASE, CYCLE_RETURN_HOME, CYCLE_ERROR
} cycle_phase_t;
typedef enum {
    CYCLE_NO_FAULT, CYCLE_BAD_PLAN, CYCLE_SENSOR_FAULT, CYCLE_OVERCURRENT,
    CYCLE_NO_OBJECT, CYCLE_NO_CONTACT, CYCLE_TIMEOUT, CYCLE_CLOCK,
    CYCLE_MAP_EXPIRED, CYCLE_EMERGENCY, CYCLE_MOTION_FAULT
} cycle_fault_t;
typedef struct {
    unsigned scan_count;
    float scan[CYCLE_MAX_SCAN_POINTS][MOTION_AXES];
    float pickup[CYCLE_MAX_SCAN_POINTS][MOTION_AXES];
    float home[MOTION_AXES], lift[MOTION_AXES], deposit[MOTION_AXES];
    float gripper_closed_deg, gripper_open_deg;
    float detect_min_mm, detect_max_mm, contact_ma, overload_ma;
    uint32_t settle_ms, step_timeout_ms, operation_timeout_ms, map_ttl_ms, sensor_max_age_ms;
    sensor_source_t sensor_source;
} cycle_plan_t;
typedef struct {
    motion_model_t *motion;
    sensor_data_t *sensors;
    cycle_plan_t plan;
    cycle_phase_t phase;
    cycle_fault_t fault;
    unsigned scan_index, selected_index;
    float selected_distance_mm;
    bool map_valid;
    int64_t started_ms, phase_ms, last_ms, mapped_ms;
} cycle_sequence_t;

void cycle_sequence_init(cycle_sequence_t *cycle, motion_model_t *motion, sensor_data_t *sensors);
bool cycle_start_scan(cycle_sequence_t *cycle, const cycle_plan_t *plan, int64_t now_ms);
bool cycle_confirm(cycle_sequence_t *cycle, int64_t now_ms);
void cycle_tick(cycle_sequence_t *cycle, int64_t now_ms);
void cycle_abort(cycle_sequence_t *cycle, cycle_fault_t reason);
robot_state_t cycle_robot_state(const cycle_sequence_t *cycle);
const char *cycle_phase_name(cycle_phase_t phase);
const char *cycle_fault_name(cycle_fault_t fault);
#endif
