#ifndef TOF_ACQUISITION_H
#define TOF_ACQUISITION_H
#include "sensor_interface.h"

typedef struct {
    hardware_result_t (*start)(void *context);
    /* IO_PENDING while ranging, IO_OK with the ST API RangeStatus when ready. */
    hardware_result_t (*read)(void *context, uint16_t *mm, uint8_t *range_status, int64_t *timestamp_ms);
    void (*cancel)(void *context);
    void *context;
} tof_backend_t;

bool tof_acquisition_driver(tof_backend_t *backend, sensor_driver_t *driver);
#endif
