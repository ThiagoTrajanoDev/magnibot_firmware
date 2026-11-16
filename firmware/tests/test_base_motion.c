#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "base_motion.h"

int main(void) {
    base_motion_t base = {0};
    assert(!base_motion_begin(&base, 70, 100));
    assert(base_motion_reference(&base, 0));
    assert(!base_motion_begin(&base, 91, 100));
    assert(base_motion_begin(&base, 70, 100));
    assert(!base_motion_reference(&base, 0));
    assert(base_motion_poll(&base, 100) == 1573);
    assert(base_motion_poll(&base, base.stop_ms) == BASE_STOP_US);
    assert(!base.moving && fabsf(base.estimate_deg - 70) < 0.05f);
    assert(base_motion_begin(&base, -25, base.last_ms));
    assert(base_motion_poll(&base, base.last_ms) == 1403);
    assert(base_motion_poll(&base, base.stop_ms + 11) == BASE_STOP_US);
    assert(base.latched && !base.referenced);
    assert(!base_motion_reference(&base, 0));
    assert(!base_motion_begin(&base, 0, 9000));
    puts("PASS: continuous base reference, direction, bounds, deadline and stop latch");
}
