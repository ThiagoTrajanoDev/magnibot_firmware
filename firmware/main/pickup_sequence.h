#ifndef PICKUP_SEQUENCE_H
#define PICKUP_SEQUENCE_H
#include "base_scan.h"
#include "bench_servo.h"

#define PICKUP_TARGET_MM 60.0f

typedef enum {
    PICKUP_DISABLED, PICKUP_READY, PICKUP_WAIT_SENSOR, PICKUP_APPROACH,
    PICKUP_VERIFY, PICKUP_CLOSE, PICKUP_LIFT, PICKUP_HOME, PICKUP_OPEN,
    PICKUP_DONE, PICKUP_FAILED
} pickup_phase_t;
typedef enum {
    PICKUP_NO_FAULT, PICKUP_SENSOR, PICKUP_LIMIT, PICKUP_TIMEOUT, PICKUP_CLOCK, PICKUP_DRIVER
} pickup_fault_t;
/* Limits must be checked on the assembled arm before providing a plan to init. */
typedef struct {
    float shoulder_floor_deg, elbow_limit_deg;
    float wrist_flex_deg, lift_delta_deg, approach_us_per_s, approach_us_per_s2;
    uint32_t phase_timeout_ms, sensor_max_age_ms;
} pickup_plan_t;
typedef struct {
    bench_servo_t *servos;
    base_scan_t *base;
    pickup_plan_t plan;
    pickup_phase_t phase;
    pickup_fault_t fault;
    unsigned close_samples;
    int64_t phase_ms, last_ms, last_sample_ms, settled_ms;
} pickup_sequence_t;
/* NULL plan keeps automatic pickup disabled; no hardware calls are made. */
bool pickup_sequence_init(pickup_sequence_t *pickup, bench_servo_t *servos,
                          base_scan_t *base, const pickup_plan_t *plan);
bool pickup_sequence_reset(pickup_sequence_t *pickup);
bool pickup_sequence_start(pickup_sequence_t *pickup, int64_t now_ms);
void pickup_sequence_poll(pickup_sequence_t *pickup, int64_t now_ms,
                          const sensor_sample_t *distance, hardware_result_t sensor_io);
const char *pickup_phase_name(pickup_phase_t phase);
const char *pickup_fault_name(pickup_fault_t fault);
#endif
