#include "base_motion.h"
#include <math.h>

static bool within_course(float angle) {
    return isfinite(angle) && angle >= -45 && angle <= 90;
}

bool base_motion_reference(base_motion_t *base, float degrees) {
    if (!base || base->moving || base->latched || !within_course(degrees)) return false;
    base->estimate_deg = degrees;
    base->referenced = true;
    return true;
}

bool base_motion_begin(base_motion_t *base, float target_deg, int64_t now_ms) {
    if (!base || !base->referenced || base->latched || base->moving ||
        !within_course(target_deg) || now_ms < base->last_ms || now_ms > INT64_MAX - 4010)
        return false;
    float delta = target_deg - base->estimate_deg;
    if (delta == 0) return true;
    float speed = delta > 0 ? 70.0f / 1.9f : -95.0f / 2.6f;
    int64_t duration = (int64_t)(1000.0f * delta / speed);
    if (duration == 0) return true;
    base->origin_deg = base->estimate_deg;
    base->velocity_deg_s = speed;
    base->start_ms = base->last_ms = now_ms;
    base->stop_ms = now_ms + duration;
    base->moving = true;
    return true;
}

unsigned base_motion_poll(base_motion_t *base, int64_t now_ms) {
    if (!base || !base->moving) return BASE_STOP_US;
    if (now_ms < base->last_ms || now_ms > base->stop_ms + 10) {
        base_motion_stop(base);
        return BASE_STOP_US;
    }
    base->last_ms = now_ms;
    base->estimate_deg = base->origin_deg +
        base->velocity_deg_s * (float)(now_ms - base->start_ms) / 1000.0f;
    if (!within_course(base->estimate_deg)) {
        base_motion_stop(base);
        return BASE_STOP_US;
    }
    if (now_ms >= base->stop_ms) {
        base->moving = false;
        return BASE_STOP_US;
    }
    return base->velocity_deg_s > 0 ? BASE_STOP_US + 85 : BASE_STOP_US - 85;
}

void base_motion_stop(base_motion_t *base) {
    if (!base) return;
    base->moving = false;
    base->referenced = false;
    base->latched = true;
}
