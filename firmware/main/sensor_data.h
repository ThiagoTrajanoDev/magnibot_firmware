#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { SENSOR_NONE, SENSOR_TEST, SENSOR_HARDWARE } sensor_source_t;
typedef enum { SENSOR_FRESH, SENSOR_UNAVAILABLE, SENSOR_INVALID, SENSOR_STALE, SENSOR_FUTURE, SENSOR_WRONG_SOURCE } sensor_status_t;
typedef struct {
    float value;
    int64_t timestamp_ms, newest_ms;
    sensor_source_t source;
    bool received, valid, accepted;
} sensor_sample_t;
typedef struct { sensor_sample_t distance_mm, current_ma; } sensor_data_t;
void sensor_data_init(sensor_data_t *data);
bool sensor_sample_update(sensor_sample_t *sample, float value, int64_t timestamp_ms,
                          sensor_source_t source, bool driver_ok);
sensor_status_t sensor_sample_check(const sensor_sample_t *sample, int64_t now_ms,
                                    uint32_t max_age_ms, sensor_source_t expected);
const char *sensor_status_name(sensor_status_t status);
#endif
