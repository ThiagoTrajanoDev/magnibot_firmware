#include <assert.h>
#include <stdio.h>
#include "bench_servo.h"
#include "config.h"

int main(void) {
    float reference;
    assert(bench_servo_named_reference(SERVO_OMBRO_GPIO, 104, &reference));
    assert(reference > 0 && reference < 90);
    assert(bench_servo_named_reference(SERVO_OMBRO_GPIO, 150, &reference));
    assert(reference > 90 && reference < 180);
    assert(bench_servo_named_reference(SERVO_OMBRO_GPIO, 0, &reference) && reference == 0);
    assert(bench_servo_named_reference(SERVO_OMBRO_GPIO, 180, &reference) && reference == 180);
    assert(!bench_servo_named_reference(SERVO_OMBRO_GPIO, -1, &reference));
    assert(!bench_servo_named_reference(SERVO_OMBRO_GPIO, 181, &reference));
    puts("PASS: configurable lower and upper shoulder pulse limits");
}
