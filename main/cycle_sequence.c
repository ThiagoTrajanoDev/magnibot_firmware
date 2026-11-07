#include "cycle_sequence.h"
#include <math.h>
#include <string.h>

void cycle_sequence_init(cycle_sequence_t *cycle, motion_model_t *motion, sensor_data_t *sensors) {
    if (!cycle) return;
    memset(cycle, 0, sizeof(*cycle));
    cycle->motion = motion;
    cycle->sensors = sensors;
    if (!motion || !sensors) cycle_abort(cycle, CYCLE_BAD_PLAN);
}

void cycle_abort(cycle_sequence_t *cycle, cycle_fault_t reason) {
    if (!cycle || cycle->phase == CYCLE_ERROR) return;
    cycle->phase = CYCLE_ERROR;
    cycle->fault = reason == CYCLE_NO_FAULT ? CYCLE_EMERGENCY : reason;
    cycle->map_valid = false;
    motion_cancel(cycle->motion);
}

static bool valid_plan(const cycle_sequence_t *cycle, const cycle_plan_t *plan) {
    if (!cycle->motion || !cycle->sensors || !plan ||
        motion_configured_axes(cycle->motion) != MOTION_AXES ||
        !plan->scan_count || plan->scan_count > CYCLE_MAX_SCAN_POINTS ||
        !plan->step_timeout_ms || plan->step_timeout_ms > MOTION_MAX_DURATION_MS ||
        plan->settle_ms >= plan->step_timeout_ms || !plan->operation_timeout_ms ||
        plan->operation_timeout_ms > 600000 || !plan->map_ttl_ms || plan->map_ttl_ms > 600000 ||
        !plan->sensor_max_age_ms || plan->sensor_max_age_ms > 10000 ||
        (plan->sensor_source != SENSOR_TEST && plan->sensor_source != SENSOR_HARDWARE) ||
        !isfinite(plan->detect_min_mm) || !isfinite(plan->detect_max_mm) ||
        plan->detect_min_mm <= 0 || plan->detect_min_mm > plan->detect_max_mm ||
        !isfinite(plan->contact_ma) || !isfinite(plan->overload_ma) ||
        plan->contact_ma <= 0 || plan->contact_ma >= plan->overload_ma ||
        !isfinite(plan->gripper_open_deg) || !isfinite(plan->gripper_closed_deg)) return false;
    if (motion_validate_pose(cycle->motion, plan->home) != MOTION_OK ||
        motion_validate_pose(cycle->motion, plan->lift) != MOTION_OK ||
        motion_validate_pose(cycle->motion, plan->deposit) != MOTION_OK) return false;
    for (unsigned i = 0; i < plan->scan_count; ++i) {
        if (motion_validate_pose(cycle->motion, plan->scan[i]) != MOTION_OK ||
            motion_validate_pose(cycle->motion, plan->pickup[i]) != MOTION_OK) return false;
    }
    const axis_limits_t *gripper = &cycle->motion->limits[CYCLE_GRIPPER_AXIS];
    return plan->gripper_open_deg >= gripper->min_deg && plan->gripper_open_deg <= gripper->max_deg &&
        plan->gripper_closed_deg >= gripper->min_deg && plan->gripper_closed_deg <= gripper->max_deg;
}

static bool healthy(cycle_sequence_t *cycle, int64_t now_ms) {
    if (sensor_sample_check(&cycle->sensors->current_ma, now_ms,
            cycle->plan.sensor_max_age_ms, cycle->plan.sensor_source) != SENSOR_FRESH) {
        cycle_abort(cycle, CYCLE_SENSOR_FAULT);
        return false;
    }
    if (cycle->sensors->current_ma.value >= cycle->plan.overload_ma) {
        cycle_abort(cycle, CYCLE_OVERCURRENT);
        return false;
    }
    if (sensor_sample_check(&cycle->sensors->distance_mm, now_ms,
            cycle->plan.sensor_max_age_ms, cycle->plan.sensor_source) != SENSOR_FRESH) {
        cycle_abort(cycle, CYCLE_SENSOR_FAULT);
        return false;
    }
    return true;
}

static bool move_to(cycle_sequence_t *cycle, cycle_phase_t phase,
                     const float pose[MOTION_AXES], int64_t now_ms, bool hold_grip) {
    float target[MOTION_AXES];
    memcpy(target, pose, sizeof(target));
    if (hold_grip) target[CYCLE_GRIPPER_AXIS] = cycle->motion->reference[CYCLE_GRIPPER_AXIS];
    motion_result_t result = motion_begin(cycle->motion, target, now_ms, cycle->plan.step_timeout_ms);
    if (result != MOTION_OK) {
        cycle_abort(cycle, result == MOTION_TIMEOUT ? CYCLE_TIMEOUT : CYCLE_MOTION_FAULT);
        return false;
    }
    cycle->phase = phase;
    cycle->phase_ms = now_ms;
    return true;
}

bool cycle_start_scan(cycle_sequence_t *cycle, const cycle_plan_t *plan, int64_t now_ms) {
    if (!cycle || (cycle->phase != CYCLE_IDLE && cycle->phase != CYCLE_WAIT_CONFIRM)) return false;
    if (now_ms < 0 || now_ms < cycle->last_ms || !valid_plan(cycle, plan)) {
        cycle_abort(cycle, CYCLE_BAD_PLAN);
        return false;
    }
    cycle->plan = *plan;
    cycle->map_valid = false;
    cycle->scan_index = cycle->selected_index = 0;
    cycle->selected_distance_mm = INFINITY;
    cycle->started_ms = cycle->last_ms = now_ms;
    if (!healthy(cycle, now_ms)) return false;
    return move_to(cycle, CYCLE_SCAN_MOVE, cycle->plan.scan[0], now_ms, false);
}

bool cycle_confirm(cycle_sequence_t *cycle, int64_t now_ms) {
    if (!cycle || cycle->phase != CYCLE_WAIT_CONFIRM || !cycle->map_valid) return false;
    if (now_ms < cycle->last_ms) { cycle_abort(cycle, CYCLE_CLOCK); return false; }
    if (now_ms - cycle->mapped_ms > cycle->plan.map_ttl_ms) {
        cycle_abort(cycle, CYCLE_MAP_EXPIRED);
        return false;
    }
    if (!healthy(cycle, now_ms)) return false;
    cycle->started_ms = cycle->last_ms = now_ms;
    return move_to(cycle, CYCLE_PICKUP, cycle->plan.pickup[cycle->selected_index], now_ms, false);
}

void cycle_tick(cycle_sequence_t *cycle, int64_t now_ms) {
    if (!cycle || cycle->phase == CYCLE_IDLE || cycle->phase == CYCLE_ERROR) return;
    if (now_ms < cycle->last_ms) { cycle_abort(cycle, CYCLE_CLOCK); return; }
    cycle->last_ms = now_ms;
    if (!healthy(cycle, now_ms)) return;
    if (cycle->phase == CYCLE_WAIT_CONFIRM) {
        if (now_ms - cycle->mapped_ms > cycle->plan.map_ttl_ms) cycle_abort(cycle, CYCLE_MAP_EXPIRED);
        return;
    }
    if (now_ms - cycle->started_ms > cycle->plan.operation_timeout_ms ||
        now_ms - cycle->phase_ms > cycle->plan.step_timeout_ms) {
        cycle_abort(cycle, CYCLE_TIMEOUT);
        return;
    }
    if (cycle->phase == CYCLE_SCAN_SAMPLE) {
        if (now_ms - cycle->phase_ms < cycle->plan.settle_ms ||
            cycle->sensors->distance_mm.timestamp_ms < cycle->phase_ms ||
            cycle->sensors->distance_mm.timestamp_ms - cycle->phase_ms < cycle->plan.settle_ms) return;
        float distance = cycle->sensors->distance_mm.value;
        if (distance >= cycle->plan.detect_min_mm && distance <= cycle->plan.detect_max_mm &&
            distance < cycle->selected_distance_mm) {
            cycle->selected_distance_mm = distance;
            cycle->selected_index = cycle->scan_index;
            cycle->map_valid = true;
            cycle->mapped_ms = cycle->sensors->distance_mm.timestamp_ms;
        }
        if (++cycle->scan_index < cycle->plan.scan_count)
            move_to(cycle, CYCLE_SCAN_MOVE, cycle->plan.scan[cycle->scan_index], now_ms, false);
        else move_to(cycle, CYCLE_SCAN_HOME, cycle->plan.home, now_ms, false);
        return;
    }
    if (cycle->phase == CYCLE_GRIP && cycle->sensors->current_ma.timestamp_ms > cycle->phase_ms &&
        cycle->sensors->current_ma.value >= cycle->plan.contact_ma) {
        motion_cancel(cycle->motion);
        move_to(cycle, CYCLE_LIFT, cycle->plan.lift, now_ms, true);
        return;
    }
    motion_state_t result = motion_tick(cycle->motion, now_ms);
    if (result == MOTION_FAULT || result == MOTION_CANCELLED) {
        cycle_abort(cycle, CYCLE_MOTION_FAULT);
        return;
    }
    if (result != MOTION_DONE) return;
    float pose[MOTION_AXES];
    switch (cycle->phase) {
        case CYCLE_SCAN_MOVE:
            cycle->phase = CYCLE_SCAN_SAMPLE; cycle->phase_ms = now_ms;
            break;
        case CYCLE_SCAN_HOME:
            if (!cycle->map_valid) cycle_abort(cycle, CYCLE_NO_OBJECT);
            else if (now_ms - cycle->mapped_ms > cycle->plan.map_ttl_ms)
                cycle_abort(cycle, CYCLE_MAP_EXPIRED);
            else cycle->phase = CYCLE_WAIT_CONFIRM;
            break;
        case CYCLE_PICKUP:
            memcpy(pose, cycle->motion->reference, sizeof(pose));
            pose[CYCLE_GRIPPER_AXIS] = cycle->plan.gripper_closed_deg;
            move_to(cycle, CYCLE_GRIP, pose, now_ms, false);
            break;
        case CYCLE_GRIP:
            cycle_abort(cycle, CYCLE_NO_CONTACT);
            break;
        case CYCLE_LIFT:
            move_to(cycle, CYCLE_DEPOSIT, cycle->plan.deposit, now_ms, true);
            break;
        case CYCLE_DEPOSIT:
            memcpy(pose, cycle->motion->reference, sizeof(pose));
            pose[CYCLE_GRIPPER_AXIS] = cycle->plan.gripper_open_deg;
            move_to(cycle, CYCLE_RELEASE, pose, now_ms, false);
            break;
        case CYCLE_RELEASE:
            move_to(cycle, CYCLE_RETURN_HOME, cycle->plan.home, now_ms, false);
            break;
        case CYCLE_RETURN_HOME:
            cycle->phase = CYCLE_IDLE; cycle->map_valid = false;
            break;
        default:
            cycle_abort(cycle, CYCLE_BAD_PLAN);
            break;
    }
}

robot_state_t cycle_robot_state(const cycle_sequence_t *cycle) {
    if (!cycle) return ROBOT_ERROR;
    if (cycle->phase == CYCLE_IDLE) return ROBOT_IDLE;
    if (cycle->phase == CYCLE_ERROR) return ROBOT_ERROR;
    if (cycle->phase == CYCLE_WAIT_CONFIRM) return ROBOT_VARREDURA_CONCLUIDA;
    return cycle->phase <= CYCLE_SCAN_HOME ? ROBOT_RUN_VARREDURA : ROBOT_RUN_DESLOCAMENTO;
}
const char *cycle_phase_name(cycle_phase_t phase) {
    static const char *const names[] = {"idle", "scan_move", "scan_sample", "scan_home", "wait_confirm",
        "pickup", "grip", "lift", "deposit", "release", "return_home", "error"};
    return (unsigned)phase < sizeof(names)/sizeof(names[0]) ? names[phase] : "unknown";
}
const char *cycle_fault_name(cycle_fault_t fault) {
    static const char *const names[] = {"none", "bad_plan", "sensor_fault", "overcurrent", "no_object",
        "no_contact", "timeout", "clock", "map_expired", "emergency", "motion_fault"};
    return (unsigned)fault < sizeof(names)/sizeof(names[0]) ? names[fault] : "unknown";
}
