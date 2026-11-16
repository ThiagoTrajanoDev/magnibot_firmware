#ifndef BASE_MOTION_H
#define BASE_MOTION_H
#include <stdbool.h>
#include <stdint.h>

#define BASE_STOP_US 1488u

typedef struct {
    float estimate_deg;
    float origin_deg;
    float velocity_deg_s;
    int64_t start_ms, last_ms, stop_ms;
    bool referenced, moving, latched;
} base_motion_t;

/* Establish a known reference while stopped. Valid estimates span -45 to +90 degrees. */
bool base_motion_reference(base_motion_t *base, float degrees);
bool base_motion_begin(base_motion_t *base, float target_deg, int64_t now_ms);
unsigned base_motion_poll(base_motion_t *base, int64_t now_ms);
void base_motion_stop(base_motion_t *base);
#endif
