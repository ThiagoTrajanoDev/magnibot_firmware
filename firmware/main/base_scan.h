#ifndef BASE_SCAN_H
#define BASE_SCAN_H
#include "hardware_io.h"
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    hardware_result_t (*write_us)(void *context, unsigned pulse);
    hardware_result_t (*shutdown)(void *context);
    void *context;
} base_scan_driver_t;
typedef struct {
    base_scan_driver_t driver;
    int64_t deadline_ms, last_ms;
    unsigned phase;
    bool armed, moving, latched, fault, shutdown_confirmed;
} base_scan_t;
void base_scan_init(base_scan_t *scan, const base_scan_driver_t *driver);
bool base_scan_begin(base_scan_t *scan, int64_t now_ms);
void base_scan_poll(base_scan_t *scan, int64_t now_ms);
void base_scan_stop(base_scan_t *scan);
#endif
