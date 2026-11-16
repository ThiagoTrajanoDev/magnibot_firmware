#ifdef NDEBUG
#error "tests rely on assert() side effects; do not build with NDEBUG"
#endif
#include <assert.h>
#include <stdio.h>
#include "gpio_policy.h"

int main(void) {
    const int start = 4, led = 48, estop = 6, base = 15;
    /* Free pins plus BTN_EMERG_GPIO; servo, input and Start pins are not valid emergency pins. */
    const int estop_ok[] = {1, 2, 6, 21, 38, 39, 40, 41, 42, 47};
    const int reserved[] = {-1, -5, 0, 3, 19, 20, 22, 23, 24, 25, 26, 30, 32, 33, 35, 37,
                            43, 44, 45, 46, 49, 100, start, led};
    const int assigned_inputs[] = {5, 7, 10, 11, 12, 13, 14};
    const int servo_outputs[] = {8, 9, 15, 16, 17, 18};
    for (unsigned i = 0; i < sizeof(estop_ok) / sizeof(estop_ok[0]); ++i)
        assert(gpio_policy_estop_allowed(estop_ok[i], start, led));
    for (unsigned i = 0; i < sizeof(reserved) / sizeof(reserved[0]); ++i)
        assert(!gpio_policy_estop_allowed(reserved[i], start, led));
    for (unsigned i = 0; i < sizeof(assigned_inputs) / sizeof(assigned_inputs[0]); ++i)
        assert(!gpio_policy_estop_allowed(assigned_inputs[i], start, led));
    for (unsigned i = 0; i < sizeof(servo_outputs) / sizeof(servo_outputs[0]); ++i)
        assert(!gpio_policy_estop_allowed(servo_outputs[i], start, led));
    assert(!gpio_policy_estop_allowed(21, 21, led) && !gpio_policy_estop_allowed(21, start, 21));
    /* Servo pins: same reserved rules, never the emergency pin, the continuous base or an input pin. */
    const int servo_ok[] = {1, 2, 8, 9, 16, 17, 18, 21, 38, 39, 40, 41, 42, 47};
    for (unsigned i = 0; i < sizeof(servo_ok) / sizeof(servo_ok[0]); ++i)
        assert(gpio_policy_servo_allowed(servo_ok[i], start, led, estop, base));
    for (unsigned i = 0; i < sizeof(reserved) / sizeof(reserved[0]); ++i)
        assert(!gpio_policy_servo_allowed(reserved[i], start, led, estop, base));
    for (unsigned i = 0; i < sizeof(assigned_inputs) / sizeof(assigned_inputs[0]); ++i)
        assert(!gpio_policy_servo_allowed(assigned_inputs[i], start, led, estop, base));
    assert(!gpio_policy_servo_allowed(estop, start, led, estop, base));   /* emergency pin */
    assert(!gpio_policy_servo_allowed(base, start, led, estop, base));    /* continuous base */
    assert(!gpio_policy_servo_allowed(6, start, led, -1, base));          /* emergency input, even unconfigured */
    assert(gpio_policy_servo_allowed(base, start, led, -1, -1));
    puts("PASS: emergency GPIO policy rejects reserved, absent, strapping and conflicting pins");
}
