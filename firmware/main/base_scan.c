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
static bool write_phase(base_scan_t *scan) {
    if (scan->driver.write_us(scan->driver.context, pulse[scan->phase]) == IO_OK) return true;
    scan->fault = true;
    base_scan_stop(scan);
    return false;
}
bool base_scan_begin(base_scan_t *scan, int64_t now_ms) {
    if (!scan || !scan->driver.write_us || !scan->driver.shutdown || scan->moving || scan->latched ||
        now_ms < 0 || now_ms < scan->last_ms || now_ms > INT64_MAX - 5000) return false;
    scan->armed = true;
    scan->phase = 0;
    scan->last_ms = now_ms;
    scan->deadline_ms = now_ms + duration_ms[0];
    scan->moving = write_phase(scan);
    return scan->moving;
}
void base_scan_poll(base_scan_t *scan, int64_t now_ms) {
    if (!scan || !scan->moving) return;
    if (now_ms < scan->last_ms || now_ms > scan->deadline_ms + 50) {
        scan->fault = true;
        base_scan_stop(scan);
        return;
    }
    scan->last_ms = now_ms;
    if (now_ms < scan->deadline_ms) return;
    ++scan->phase;
    if (!write_phase(scan)) return;
    scan->moving = scan->phase < 3;
    if (scan->moving) scan->deadline_ms = now_ms + duration_ms[scan->phase];
}
