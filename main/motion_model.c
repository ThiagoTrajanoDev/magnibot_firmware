#include "motion_model.h"
#include <math.h>
#include <string.h>

void motion_default_limits(axis_limits_t limits[MOTION_AXES]) {
    if (limits) memset(limits, 0, sizeof(axis_limits_t) * MOTION_AXES);
}

bool motion_model_init(motion_model_t *model, const axis_limits_t limits[MOTION_AXES]) {
    if (!model) return false;
    memset(model, 0, sizeof(*model));
    model->state = MOTION_FAULT;
    model->fault = MOTION_INVALID;
    if (!limits) return false;
    for (unsigned i = 0; i < MOTION_AXES; ++i) {
        const axis_limits_t *v = &limits[i];
        if (!isfinite(v->min_deg) || !isfinite(v->max_deg) || !isfinite(v->home_deg) ||
            !isfinite(v->max_speed_deg_s) || v->min_deg > v->max_deg ||
            v->home_deg < v->min_deg || v->home_deg > v->max_deg ||
            v->max_speed_deg_s < 0 || v->max_speed_deg_s > 180 ||
            (v->configured && v->max_speed_deg_s == 0)) return false;
    }
    memcpy(model->limits, limits, sizeof(model->limits));
    for (unsigned i = 0; i < MOTION_AXES; ++i) model->reference[i] = limits[i].home_deg;
    model->state = MOTION_IDLE;
    model->fault = MOTION_OK;
    return true;
}

unsigned motion_configured_axes(const motion_model_t *model) {
    unsigned count = 0;
    if (model) for (unsigned i = 0; i < MOTION_AXES; ++i) count += model->limits[i].configured;
    return count;
}

motion_result_t motion_validate_pose(const motion_model_t *model, const float target[MOTION_AXES]) {
    if (!model || !target || model->state == MOTION_FAULT) return MOTION_INVALID;
    for (unsigned i = 0; i < MOTION_AXES; ++i) {
        if (!isfinite(target[i])) return MOTION_INVALID;
        if (target[i] < model->limits[i].min_deg || target[i] > model->limits[i].max_deg)
            return MOTION_LIMIT;
        if (!model->limits[i].configured && target[i] != model->reference[i])
            return MOTION_UNCONFIGURED;
    }
    return MOTION_OK;
}

motion_result_t motion_begin(motion_model_t *model, const float target[MOTION_AXES],
                             int64_t now_ms, uint32_t timeout_ms) {
    motion_result_t result = motion_validate_pose(model, target);
    if (result != MOTION_OK) return result;
    if (model->state == MOTION_RUNNING) return MOTION_BUSY;
    if (now_ms < model->last_ms) return MOTION_CLOCK;
    if (now_ms < 0 || !timeout_ms || timeout_ms > MOTION_MAX_DURATION_MS) return MOTION_INVALID;
    double duration = 0;
    for (unsigned i = 0; i < MOTION_AXES; ++i) {
        double distance = fabs((double)target[i] - model->reference[i]);
        if (distance == 0) continue;
        double ms = ceil(distance * 1000 / model->limits[i].max_speed_deg_s);
        if (ms > duration) duration = ms;
    }
    if (duration > timeout_ms) return MOTION_TIMEOUT;
    memcpy(model->origin, model->reference, sizeof(model->origin));
    memcpy(model->target, target, sizeof(model->target));
    model->duration_ms = (uint32_t)duration;
    model->timeout_ms = timeout_ms;
    model->started_ms = model->last_ms = now_ms;
    model->state = MOTION_RUNNING;
    model->fault = MOTION_OK;
    return MOTION_OK;
}

motion_state_t motion_tick(motion_model_t *model, int64_t now_ms) {
    if (!model) return MOTION_FAULT;
    if (model->state != MOTION_RUNNING) return model->state;
    if (now_ms < model->last_ms) {
        model->fault = MOTION_CLOCK;
        model->state = MOTION_FAULT;
        return model->state;
    }
    model->last_ms = now_ms;
    int64_t elapsed = now_ms - model->started_ms;
    if (elapsed > model->timeout_ms) {
        model->fault = MOTION_TIMEOUT;
        model->state = MOTION_FAULT;
        return model->state;
    }
    double fraction = model->duration_ms ? (double)elapsed / model->duration_ms : 1;
    if (fraction >= 1) {
        memcpy(model->reference, model->target, sizeof(model->reference));
        model->state = MOTION_DONE;
    } else {
        for (unsigned i = 0; i < MOTION_AXES; ++i)
            model->reference[i] = (float)((double)model->origin[i] +
                ((double)model->target[i] - model->origin[i]) * fraction);
    }
    return model->state;
}

void motion_cancel(motion_model_t *model) {
    if (model && model->state != MOTION_FAULT) model->state = MOTION_CANCELLED;
}
