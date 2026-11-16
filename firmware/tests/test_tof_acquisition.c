#include <assert.h>
#include <stdio.h>
#include "tof_acquisition.h"

typedef struct {
    hardware_result_t result;
    uint8_t status;
    unsigned cancelled;
} fixture_t;

static hardware_result_t start(void *context) {
    (void)context;
    return IO_OK;
}

static hardware_result_t read_range(void *context, uint16_t *mm, uint8_t *status, int64_t *time) {
    fixture_t *fixture = context;
    *mm = 123;
    *status = fixture->status;
    *time = 20;
    return fixture->result;
}

static void cancel(void *context) {
    ++((fixture_t *)context)->cancelled;
}

int main(void) {
    fixture_t fixture = {.result = IO_PENDING};
    tof_backend_t backend = {.start = start, .read = read_range, .cancel = cancel, .context = &fixture};
    sensor_driver_t driver;
    assert(tof_acquisition_driver(&backend, &driver));
    assert(driver.request(driver.context, 10) == IO_OK);
    float mm = -1;
    int64_t time;
    assert(driver.poll(driver.context, &mm, &time) == IO_PENDING && mm == -1);
    fixture.result = IO_FAULT;
    assert(driver.poll(driver.context, &mm, &time) == IO_FAULT && mm == -1);
    fixture.result = IO_OK;
    fixture.status = 2;
    assert(driver.poll(driver.context, &mm, &time) == IO_INVALID && mm == -1);
    fixture.status = 0;
    assert(driver.poll(driver.context, &mm, &time) == IO_OK && mm == 123 && time == 20);
    driver.cancel(driver.context);
    assert(fixture.cancelled == 1);
    puts("PASS: ToF pending, disconnect, invalid range, valid result and cancellation");
}
