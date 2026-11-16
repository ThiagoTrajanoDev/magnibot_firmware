#include "gpio_policy.h"
#include <stdint.h>
#include "config.h"

#define PIN(n) (1ULL << (n))
#define PIN_RANGE(first, last) ((PIN((last) + 1) - 1) & ~(PIN(first) - 1))

static const uint64_t reserved =
    PIN(0) | PIN(3) | PIN(45) | PIN(46) | PIN(19) | PIN(20) | PIN(43) | PIN(44) |
    PIN_RANGE(22, 37);

/* Pins the schematic (config.h) assigns to inputs: never a servo output. */
static const uint64_t input_pins =
    PIN(BTN_EMERG_GPIO) | PIN(BTN_FIM_GPIO) | PIN(POT_GPIO) | PIN(ENC_SW_GPIO) |
    PIN(ENC_CLK_GPIO) | PIN(ENC_DT_GPIO) | PIN(I2C_SDA_GPIO) | PIN(I2C_SCL_GPIO);
static const uint64_t servo_pins =
    PIN(SERVO_BASE_GPIO) | PIN(SERVO_OMBRO_GPIO) | PIN(SERVO_COTOVELO_GPIO) |
    PIN(SERVO_PUNHO_ROT_GPIO) | PIN(SERVO_PUNHO_FLEX_GPIO) | PIN(SERVO_GARRA_GPIO);

static bool usable(int pin, int start_pin, int led_pin) {
    if (pin < 0 || pin > 48 || pin == start_pin || pin == led_pin) return false;
    return !(reserved & PIN(pin));
}

bool gpio_policy_estop_allowed(int pin, int start_pin, int led_pin) {
    return usable(pin, start_pin, led_pin) &&
           !(((input_pins & ~PIN(BTN_EMERG_GPIO)) | servo_pins) & PIN(pin));
}

bool gpio_policy_servo_allowed(int pin, int start_pin, int led_pin, int estop_pin, int base_pin) {
    return pin != estop_pin && pin != base_pin && usable(pin, start_pin, led_pin) &&
           !(input_pins & PIN(pin));
}
