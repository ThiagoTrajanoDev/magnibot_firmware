#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H
#include <stdbool.h>
#include <stdint.h>
#define MOTION_AXES 6
#define MOTION_MAX_DURATION_MS 120000

typedef struct {
    float min_deg, max_deg, home_deg, max_speed_deg_s;
    bool configured;
} axis_limits_t;
typedef enum { MOTION_IDLE, MOTION_RUNNING, MOTION_DONE, MOTION_CANCELLED, MOTION_FAULT } motion_state_t;
typedef enum {
    MOTION_OK, MOTION_INVALID, MOTION_LIMIT, MOTION_UNCONFIGURED,
    MOTION_BUSY, MOTION_TIMEOUT, MOTION_CLOCK
} motion_result_t;
typedef struct {
    axis_limits_t limits[MOTION_AXES];
    float reference[MOTION_AXES], origin[MOTION_AXES], target[MOTION_AXES];
    int64_t started_ms, last_ms;
    uint32_t duration_ms, timeout_ms;
    motion_state_t state;
    motion_result_t fault;
} motion_model_t;

void motion_default_limits(axis_limits_t limits[MOTION_AXES]);
bool motion_model_init(motion_model_t *model, const axis_limits_t limits[MOTION_AXES]);
bool motion_limits_valid(const axis_limits_t limits[MOTION_AXES]);
motion_result_t motion_validate_limits(const axis_limits_t limits[MOTION_AXES],
                                       const float reference[MOTION_AXES],
                                       const float target[MOTION_AXES]);
motion_result_t motion_validate_pose(const motion_model_t *model, const float target[MOTION_AXES]);
motion_result_t motion_begin(motion_model_t *model, const float target[MOTION_AXES], int64_t now_ms, uint32_t timeout_ms);
motion_state_t motion_tick(motion_model_t *model, int64_t now_ms);
void motion_cancel(motion_model_t *model);
unsigned motion_configured_axes(const motion_model_t *model);
#endif
