#include "pickup_sequence.h"
#include "config.h"
#include <math.h>
#include <stddef.h>

static bool active(const pickup_sequence_t *p) {
    return p->phase >= PICKUP_WAIT_SENSOR && p->phase <= PICKUP_OPEN;
}

static void abort_pickup(pickup_sequence_t *p, pickup_fault_t fault) {
    p->fault = fault;
    p->phase = PICKUP_FAILED;
    base_scan_stop(p->base);
    /* Keep the last joint PWM on a controlled abort; explicit estop still overrides hold. */
    bench_servo_hold(p->servos);
}

static void enter(pickup_sequence_t *p, pickup_phase_t phase, int64_t now) {
    p->phase = phase;
    p->phase_ms = now;
    p->settled_ms = -1;
}

static bool move_joint(pickup_sequence_t *p, int gpio, float angle, float speed, float accel) {
    bench_servo_pose_joint_t joint = {gpio, angle, speed, accel};
    if (bench_servo_move_pose(p->servos, &joint, 1)) return true;
    abort_pickup(p, PICKUP_DRIVER);
    return false;
}

bool pickup_sequence_init(pickup_sequence_t *p, bench_servo_t *servos,
                          base_scan_t *base, const pickup_plan_t *plan) {
    if (!p || !servos || !base) return false;
    *p = (pickup_sequence_t){.servos=servos, .base=base, .phase=PICKUP_DISABLED};
    if (!plan) return true;
    if (!isfinite(plan->shoulder_floor_deg) || plan->shoulder_floor_deg < 0 || plan->shoulder_floor_deg > 180 ||
        !isfinite(plan->elbow_limit_deg) || plan->elbow_limit_deg < 0 || plan->elbow_limit_deg > 180 ||
        !isfinite(plan->wrist_flex_deg) || plan->wrist_flex_deg < 0 || plan->wrist_flex_deg > 180 ||
        !isfinite(plan->lift_delta_deg) || plan->lift_delta_deg <= 0 || plan->lift_delta_deg > 20 ||
        !isfinite(plan->approach_us_per_s) || plan->approach_us_per_s <= 0 || plan->approach_us_per_s > 100 ||
        !isfinite(plan->approach_us_per_s2) || plan->approach_us_per_s2 <= 0 || plan->approach_us_per_s2 > 200 ||
        plan->phase_timeout_ms < 1000 || plan->phase_timeout_ms > 120000 ||
        plan->sensor_max_age_ms < 50 || plan->sensor_max_age_ms > 200) return false;
    p->plan = *plan;
    p->phase = PICKUP_READY;
    return true;
}

bool pickup_sequence_reset(pickup_sequence_t *p) {
    if (!p || active(p) || p->phase == PICKUP_FAILED || p->servos->latched || p->base->latched) return false;
    if (p->phase == PICKUP_DISABLED) return true;
    p->phase = PICKUP_READY;
    p->fault = PICKUP_NO_FAULT;
    p->close_samples = 0;
    return true;
}

bool pickup_sequence_start(pickup_sequence_t *p, int64_t now) {
    if (!p || p->phase != PICKUP_READY || !p->base->at_object || !p->base->found ||
        p->base->moving || p->base->fault || p->base->latched || p->servos->latched ||
        p->servos->scan_pose_active || now < 0 || now < p->base->last_ms ||
        now > INT64_MAX - 120000) return false;
    for (unsigned i=0;i<p->servos->used;++i)
        if (p->servos->channels[i].current != p->servos->channels[i].target) return false;
    float shoulder, elbow, commanded;
    if (!bench_servo_named_angle(p->servos, SERVO_OMBRO_GPIO, &shoulder) ||
        !bench_servo_named_angle(p->servos, SERVO_COTOVELO_GPIO, &elbow) ||
        !bench_servo_named_angle(p->servos, SERVO_GARRA_GPIO, &commanded) ||
        !bench_servo_named_angle(p->servos, SERVO_PUNHO_FLEX_GPIO, &commanded) ||
        shoulder < p->plan.shoulder_floor_deg || elbow < p->plan.elbow_limit_deg ||
        shoulder + p->plan.lift_delta_deg > 180) {
        abort_pickup(p, PICKUP_LIMIT);
        return false;
    }
    p->last_ms = now;
    p->last_sample_ms = now;
    p->close_samples = 0;
    enter(p, PICKUP_WAIT_SENSOR, now);
    return true;
}

static bool sensor_good(const pickup_sequence_t *p, const sensor_sample_t *s, hardware_result_t io, int64_t now) {
    return (io == IO_OK || io == IO_PENDING) && s &&
        sensor_sample_check(s, now, p->plan.sensor_max_age_ms, SENSOR_HARDWARE) == SENSOR_FRESH && s->value > 0;
}

void pickup_sequence_poll(pickup_sequence_t *p, int64_t now,
                          const sensor_sample_t *distance, hardware_result_t sensor_io) {
    if (!p || !active(p)) return;
    if (now < p->last_ms) { abort_pickup(p, PICKUP_CLOCK); return; }
    p->last_ms = now;
    if (bench_servo_faulted(p->servos) || p->servos->latched || p->base->fault || p->base->latched) {
        abort_pickup(p, PICKUP_DRIVER); return;
    }
    if (now - p->phase_ms > p->plan.phase_timeout_ms) { abort_pickup(p, PICKUP_TIMEOUT); return; }
    bool approaching = p->phase <= PICKUP_VERIFY;
    if (approaching && !sensor_good(p, distance, sensor_io, now)) { abort_pickup(p, PICKUP_SENSOR); return; }
    if (p->phase == PICKUP_WAIT_SENSOR) {
        if (distance->timestamp_ms <= p->last_sample_ms) {
            if (now - p->phase_ms > p->plan.sensor_max_age_ms) abort_pickup(p, PICKUP_SENSOR);
            return;
        }
        if (distance->value <= PICKUP_TARGET_MM) {
            p->close_samples = 1;
            p->last_sample_ms = distance->timestamp_ms;
            enter(p, PICKUP_VERIFY, now);
            return;
        }
        const pickup_plan_t *plan = &p->plan;
        const bench_servo_pose_joint_t joints[] = {
            {SERVO_OMBRO_GPIO, plan->shoulder_floor_deg, plan->approach_us_per_s, plan->approach_us_per_s2},
            {SERVO_COTOVELO_GPIO, plan->elbow_limit_deg, plan->approach_us_per_s, plan->approach_us_per_s2},
            {SERVO_PUNHO_FLEX_GPIO, plan->wrist_flex_deg, plan->approach_us_per_s, plan->approach_us_per_s2}
        };
        if (!bench_servo_move_pose(p->servos, joints, 3)) { abort_pickup(p, PICKUP_DRIVER); return; }
        enter(p, PICKUP_APPROACH, now);
    } else if (p->phase == PICKUP_APPROACH) {
        if (distance->timestamp_ms > p->last_sample_ms && distance->value <= PICKUP_TARGET_MM) {
            if (!bench_servo_pause(p->servos)) { abort_pickup(p, PICKUP_DRIVER); return; }
            p->close_samples = 1;
            p->last_sample_ms = distance->timestamp_ms;
            enter(p, PICKUP_VERIFY, now);
        } else if (!p->servos->scan_pose_active) abort_pickup(p, PICKUP_LIMIT);
    } else if (p->phase == PICKUP_VERIFY) {
        if (distance->timestamp_ms <= p->last_sample_ms) return;
        p->last_sample_ms = distance->timestamp_ms;
        p->close_samples = distance->value <= PICKUP_TARGET_MM ? p->close_samples + 1 : 0;
        if (p->close_samples >= 3 && move_joint(p, SERVO_GARRA_GPIO, 0, 250, 375)) enter(p, PICKUP_CLOSE, now);
    } else if (p->phase == PICKUP_HOME) {
        if (p->base->home && move_joint(p, SERVO_GARRA_GPIO, 70, 250, 375)) enter(p, PICKUP_OPEN, now);
    } else if (!p->servos->scan_pose_active) {
        if (p->settled_ms < 0) p->settled_ms = now;
        if (now - p->settled_ms < 250) return;
        if (p->phase == PICKUP_CLOSE) {
            float shoulder;
            if (!bench_servo_named_angle(p->servos, SERVO_OMBRO_GPIO, &shoulder) ||
                shoulder + p->plan.lift_delta_deg > 180) { abort_pickup(p, PICKUP_LIMIT); return; }
            if (move_joint(p, SERVO_OMBRO_GPIO, shoulder + p->plan.lift_delta_deg, 100, 150))
                enter(p, PICKUP_LIFT, now);
        } else if (p->phase == PICKUP_LIFT) {
            if (!base_scan_return_home(p->base, now)) { abort_pickup(p, PICKUP_DRIVER); return; }
            enter(p, PICKUP_HOME, now);
        } else if (p->phase == PICKUP_OPEN) {
            p->base->found = false;
            p->base->found_distance_mm = 0;
            p->base->found_offset_ms = 0;
            enter(p, PICKUP_DONE, now);
        }
    }
}

const char *pickup_phase_name(pickup_phase_t phase) {
    static const char *const names[] = {"disabled", "ready", "wait_sensor", "approach", "verify",
        "close", "lift", "home", "open", "done", "failed"};
    return phase >= PICKUP_DISABLED && phase <= PICKUP_FAILED ? names[phase] : "unknown";
}

const char *pickup_fault_name(pickup_fault_t fault) {
    static const char *const names[] = {"none", "sensor", "travel_limit", "timeout", "clock", "driver"};
    return fault >= PICKUP_NO_FAULT && fault <= PICKUP_DRIVER ? names[fault] : "unknown";
}
