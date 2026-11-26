#include "base_scan.h"
#include "base_motion.h"

/* Spreadsheet values; physical polarity and return accuracy require calibration. */
static const unsigned pulse[] = {1573, 1403, 1573, BASE_STOP_US};
static const int duration_ms[] = {1000, 1750, 750};

void base_scan_init(base_scan_t *scan, const base_scan_driver_t *driver) {
    *scan = (base_scan_t){0};
    if (driver) scan->driver = *driver;
}
void base_scan_stop(base_scan_t *scan) {
    scan->moving = false;
    scan->latched = true;
    if (scan->shutdown_confirmed) return;
    if (scan->armed && scan->driver.write_us && scan->driver.write_us(scan->driver.context, BASE_STOP_US) != IO_OK)
        scan->fault = true;
    if (scan->driver.shutdown) {
        scan->shutdown_confirmed = scan->driver.shutdown(scan->driver.context) == IO_OK;
        if (!scan->shutdown_confirmed) scan->fault = true;
    } else scan->shutdown_confirmed = true;
}
static bool write_pulse(base_scan_t *scan, unsigned value) {
    if (scan->driver.write_us(scan->driver.context, value) == IO_OK) return true;
    scan->fault = true;
    base_scan_stop(scan);
    return false;
}
bool base_scan_begin(base_scan_t *scan, int64_t now_ms) {
    if (!scan || !scan->driver.write_us || !scan->driver.shutdown || scan->moving || scan->latched ||
        now_ms < 0 || now_ms < scan->last_ms || now_ms > INT64_MAX - 5000) return false;
    scan->found = scan->no_object = false;
    scan->paused = scan->home = scan->replaying = scan->at_object = scan->homing = false;
    scan->position_ms = scan->found_offset_ms = 0;
    scan->found_distance_mm = 0;
    scan->started_ms = now_ms;
    scan->armed = true;
    scan->phase = 0;
    scan->last_ms = now_ms;
    scan->deadline_ms = now_ms + duration_ms[0];
    scan->moving = write_pulse(scan, pulse[scan->phase]);
    return scan->moving;
}
void base_scan_poll(base_scan_t *scan, int64_t now_ms,
                    const sensor_sample_t *distance, hardware_result_t sensor_io) {
    if (!scan || !scan->moving) return;
    if (now_ms < scan->last_ms || now_ms > scan->deadline_ms + 50) {
        scan->fault = true;
        base_scan_stop(scan);
        return;
    }
    if (!scan->paused) {
        int direction = scan->phase == 1 ? -1 : 1;
        if (scan->replaying)
            direction = (scan->found_offset_ms >= 0 ? 1 : -1) * (scan->homing ? -1 : 1);
        scan->position_ms += (now_ms - scan->last_ms) * direction;
    }
    scan->last_ms = now_ms;
    if (!scan->found && distance && (sensor_io == IO_OK || sensor_io == IO_PENDING) &&
        sensor_sample_check(distance, now_ms, 200, SENSOR_HARDWARE) == SENSOR_FRESH &&
        distance->timestamp_ms > scan->started_ms &&
        distance->value > 0 && distance->value <= BASE_SCAN_REACH_MM) {
        if (!write_pulse(scan, BASE_STOP_US)) return;
        scan->found = true;
        scan->found_distance_mm = distance->value;
        scan->found_offset_ms = scan->position_ms;
        scan->resume_remaining_ms = scan->deadline_ms > now_ms ? scan->deadline_ms - now_ms : 0;
        scan->paused = true;
        scan->deadline_ms = now_ms + 200;
        return;
    }
    if (now_ms < scan->deadline_ms) return;
    if (scan->replaying) {
        if (!write_pulse(scan, BASE_STOP_US)) return;
        scan->position_ms = scan->homing ? 0 : scan->found_offset_ms;
        scan->home = scan->homing;
        scan->at_object = !scan->homing;
        scan->moving = scan->replaying = scan->homing = false;
        return;
    }
    if (scan->paused) {
        scan->paused = false;
        if (scan->resume_remaining_ms > 0) {
            if (!write_pulse(scan, pulse[scan->phase])) return;
            scan->deadline_ms = now_ms + scan->resume_remaining_ms;
            return;
        }
    }
    ++scan->phase;
    if (!write_pulse(scan, pulse[scan->phase])) return;
    scan->moving = scan->phase < 3;
    if (scan->moving) scan->deadline_ms = now_ms + duration_ms[scan->phase];
    else {
        scan->home = true;
        scan->no_object = !scan->found;
        scan->position_ms = 0; /* Estimated origin after the calibrated complete sweep. */
    }
}

static bool move_saved_position(base_scan_t *scan, int64_t now_ms, bool homing) {
    if (!scan || !scan->found || scan->moving || scan->latched || scan->fault ||
        (homing ? !scan->at_object : !scan->home) ||
        now_ms < scan->last_ms || now_ms > INT64_MAX - 5000) return false;
    scan->home = scan->at_object = false;
    scan->homing = homing;
    scan->last_ms = now_ms;
    int64_t duration = scan->found_offset_ms >= 0 ? scan->found_offset_ms : -scan->found_offset_ms;
    bool positive = (scan->found_offset_ms >= 0) != homing;
    if (!write_pulse(scan, duration == 0 ? BASE_STOP_US : pulse[positive ? 0 : 1])) return false;
    if (duration == 0) {
        scan->home = homing;
        scan->at_object = !homing;
        scan->homing = false;
    }
    scan->replaying = scan->moving = duration > 0;
    scan->deadline_ms = now_ms + duration;
    return true;
}

bool base_scan_replay(base_scan_t *scan, int64_t now_ms) {
    return move_saved_position(scan, now_ms, false);
}

bool base_scan_return_home(base_scan_t *scan, int64_t now_ms) {
    return move_saved_position(scan, now_ms, true);
}

const char *base_scan_status(const base_scan_t *scan) {
    if (scan->fault) return "fault";
    if (scan->latched) return "stopped";
    if (scan->no_object) return "no_object";
    if (scan->at_object) return "at_object";
    if (scan->replaying) return scan->homing ? "returning_origin" : "returning_object";
    if (scan->home && scan->found) return "awaiting_confirm";
    if (scan->found) return "returning_home";
    return scan->moving ? "searching" : "idle";
}
