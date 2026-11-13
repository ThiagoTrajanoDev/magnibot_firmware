#ifndef GPIO_POLICY_H
#define GPIO_POLICY_H
#include <stdbool.h>
/* ESP32-S3 (N16R8 dev-kit): true only for a pin that can serve as the emergency input.
 * Rejects out-of-range/absent pins, strapping (0, 3, 45, 46), USB (19, 20), flash/PSRAM
 * (26-37), UART0 console (43, 44) and the pins already used by the Start button and LED. */
bool gpio_policy_estop_allowed(int pin, int start_pin, int led_pin);
/* Same pin rules as the emergency input, plus: not the emergency pin and not the base pin
 * (continuous-rotation servo: an angle command would spin it). */
bool gpio_policy_servo_allowed(int pin, int start_pin, int led_pin, int estop_pin, int base_pin);
#endif
