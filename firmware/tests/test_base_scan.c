#include <assert.h>
#include <stdio.h>
#include "base_scan.h"
static unsigned pulse, writes, stops;
static bool fail;
static hardware_result_t write_us(void *context, unsigned value) {
    (void)context; pulse = value; ++writes; return fail ? IO_FAULT : IO_OK;
}
static hardware_result_t shutdown(void *context) { (void)context; ++stops; return IO_OK; }
int main(void) {
    base_scan_t scan;
    base_scan_driver_t driver = {.write_us = write_us, .shutdown = shutdown};
    base_scan_init(&scan, &driver);
    assert(writes == 0);
    base_scan_stop(&scan);
    assert(writes == 0);
    base_scan_init(&scan, &driver);
    assert(base_scan_begin(&scan, 100) && pulse == 1573);
    assert(!base_scan_begin(&scan, 200));
    base_scan_poll(&scan, 1099); assert(pulse == 1573);
    base_scan_poll(&scan, 1100); assert(pulse == 1403);
    base_scan_poll(&scan, 2849); assert(pulse == 1403);
    base_scan_poll(&scan, 2850); assert(pulse == 1573);
    base_scan_poll(&scan, 3599); assert(pulse == 1573);
    base_scan_poll(&scan, 3600); assert(pulse == 1488 && !scan.moving && !scan.fault);
    assert(base_scan_begin(&scan, 5000));
    base_scan_stop(&scan); assert(pulse == 1488 && stops == 2 && !scan.moving);
    assert(!base_scan_begin(&scan, 6000));
    base_scan_init(&scan, &driver);
    assert(base_scan_begin(&scan, 0));
    base_scan_poll(&scan, 1051); assert(scan.fault && scan.latched && pulse == 1488);
    base_scan_init(&scan, &driver);
    assert(base_scan_begin(&scan, 20));
    base_scan_poll(&scan, 19); assert(scan.fault && scan.latched);
    base_scan_init(&scan, &driver);
    fail = true;
    assert(!base_scan_begin(&scan, 0) && scan.fault && scan.latched);
    base_scan_init(&scan, NULL);
    assert(!base_scan_begin(&scan, 0));
    puts("PASS: timed base scan, neutral, emergency, timing overrun, clock and driver faults");
}
