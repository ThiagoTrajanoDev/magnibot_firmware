#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H
#include "hardware_io.h"
#include "sensor_data.h"

typedef struct {
    hardware_result_t (*request)(void *context, int64_t now_ms);
    hardware_result_t (*poll)(void *context, float *value, int64_t *timestamp_ms);
    void (*cancel)(void *context);
    void *context;
    sensor_source_t source;
} sensor_driver_t;
typedef struct {
    sensor_driver_t driver;
    sensor_sample_t *sample;
    hardware_result_t last_result;
    int64_t requested_ms;
    uint32_t failures;
    bool requested_once, pending;
} sensor_channel_t;
typedef struct {
    sensor_channel_t distance, current;
    uint32_t period_ms, timeout_ms, max_age_ms;
    int64_t last_ms;
    bool initialized, stopped;
} sensor_interface_t;

bool sensor_interface_init(sensor_interface_t *interface, sensor_data_t *data,
                           const sensor_driver_t *distance, const sensor_driver_t *current,
                           uint32_t period_ms, uint32_t timeout_ms, uint32_t max_age_ms);
void sensor_interface_poll(sensor_interface_t *interface, int64_t now_ms);
void sensor_interface_stop(sensor_interface_t *interface);
#endif
