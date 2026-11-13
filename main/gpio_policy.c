#include "gpio_policy.h"
#include <stdint.h>

#define PIN(n) (1ULL << (n))
#define PIN_RANGE(first, last) ((PIN((last) + 1) - 1) & ~(PIN(first) - 1))

static const uint64_t reserved =
    PIN(0) | PIN(3) | PIN(45) | PIN(46) | PIN(19) | PIN(20) | PIN(43) | PIN(44) |
    PIN_RANGE(22, 37);

bool gpio_policy_estop_allowed(int pin, int start_pin, int led_pin) {
    if (pin < 0 || pin > 48 || pin == start_pin || pin == led_pin) return false;
    return !(reserved & PIN(pin));
}

bool gpio_policy_servo_allowed(int pin, int start_pin, int led_pin, int estop_pin, int base_pin) {
    return pin != estop_pin && pin != base_pin && gpio_policy_estop_allowed(pin, start_pin, led_pin);
}
