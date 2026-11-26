#ifndef BASE_SCAN_H
#define BASE_SCAN_H
#include "hardware_io.h"
#include "sensor_data.h"
#define BASE_SCAN_REACH_MM 155.0f
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    hardware_result_t (*write_us)(void *context, unsigned pulse);
    hardware_result_t (*shutdown)(void *context);
    void *context;
} base_scan_driver_t;
typedef struct {
    base_scan_driver_t driver;
    int64_t deadline_ms, last_ms, started_ms;
    float found_distance_mm;
    /* Signed drive time at the calibrated pulses, not measured degrees. Positive = pulse 1573. */
    int64_t position_ms, found_offset_ms, resume_remaining_ms;
    bool paused, home, replaying, at_object, homing;
    unsigned phase;
    bool armed, moving, latched, fault, shutdown_confirmed;
    bool found, no_object;
} base_scan_t;
void base_scan_init(base_scan_t *scan, const base_scan_driver_t *driver);
bool base_scan_begin(base_scan_t *scan, int64_t now_ms);
void base_scan_poll(base_scan_t *scan, int64_t now_ms,
                    const sensor_sample_t *distance, hardware_result_t sensor_io);
bool base_scan_replay(base_scan_t *scan, int64_t now_ms);
/* Return from the saved target to the estimated sweep origin. */
bool base_scan_return_home(base_scan_t *scan, int64_t now_ms);
const char *base_scan_status(const base_scan_t *scan);
void base_scan_stop(base_scan_t *scan);
#endif
