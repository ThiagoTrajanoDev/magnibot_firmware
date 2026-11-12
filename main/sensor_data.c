#include "sensor_data.h"
#include <math.h>
#include <string.h>
void sensor_data_init(sensor_data_t *data) {
    if (data) memset(data, 0, sizeof(*data));
}
bool sensor_sample_update(sensor_sample_t *sample, float value, int64_t timestamp_ms,
                          sensor_source_t source, bool driver_ok) {
    if (!sample) return false;
    bool valid = driver_ok && isfinite(value) && value >= 0 && timestamp_ms >= 0 &&
        (source == SENSOR_TEST || source == SENSOR_HARDWARE) &&
        (!sample->accepted || timestamp_ms > sample->newest_ms);
    int64_t newest = sample->newest_ms;
    if (valid) newest = timestamp_ms;
    *sample = (sensor_sample_t){.value = value, .timestamp_ms = timestamp_ms,
        .newest_ms = newest, .source = source, .received = true, .valid = valid,
        .accepted = sample->accepted || valid};
    return valid;
}
sensor_status_t sensor_sample_check(const sensor_sample_t *sample, int64_t now_ms,
                                    uint32_t max_age_ms, sensor_source_t expected) {
    if (!sample || !sample->received) return SENSOR_UNAVAILABLE;
    if (!sample->valid || now_ms < 0 || !max_age_ms) return SENSOR_INVALID;
    if (sample->source != expected || expected == SENSOR_NONE) return SENSOR_WRONG_SOURCE;
    if (sample->timestamp_ms > now_ms) return SENSOR_FUTURE;
    if (now_ms - sample->timestamp_ms > max_age_ms) return SENSOR_STALE;
    return SENSOR_FRESH;
}
const char *sensor_status_name(sensor_status_t status) {
    switch (status) {
        case SENSOR_FRESH: return "fresh";
        case SENSOR_UNAVAILABLE: return "unavailable";
        case SENSOR_INVALID: return "invalid";
        case SENSOR_STALE: return "stale";
        case SENSOR_FUTURE: return "future";
        default: return "wrong_source";
    }
}
