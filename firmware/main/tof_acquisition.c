#include "tof_acquisition.h"

static hardware_result_t request(void *context, int64_t now_ms) {
    (void)now_ms;
    tof_backend_t *backend = context;
    return backend->start(backend->context);
}

static hardware_result_t poll(void *context, float *value, int64_t *timestamp_ms) {
    tof_backend_t *backend = context;
    uint16_t mm;
    uint8_t status;
    hardware_result_t result = backend->read(backend->context, &mm, &status, timestamp_ms);
    if (result != IO_OK) return result;
    if (status != 0 || mm == 0) return IO_INVALID;
    *value = mm;
    return IO_OK;
}

static void cancel(void *context) {
    tof_backend_t *backend = context;
    backend->cancel(backend->context);
}

bool tof_acquisition_driver(tof_backend_t *backend, sensor_driver_t *driver) {
    if (!backend || !driver || !backend->start || !backend->read || !backend->cancel) return false;
    *driver = (sensor_driver_t){
        .request = request, .poll = poll, .cancel = cancel,
        .context = backend, .source = SENSOR_HARDWARE
    };
    return true;
}
