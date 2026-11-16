#ifndef GPIO_POLICY_H
#define GPIO_POLICY_H
#include <stdbool.h>
/* ESP32-S3 (N8R8/N16R8 dev-kit): true only for a pin that can serve as the emergency input.
 * Rejects out-of-range/absent pins, strapping (0, 3, 45, 46), USB (19, 20), flash/PSRAM
 * (26-37), UART0 console (43, 44), the Start/LED pins and the other pins that config.h
 * assigns to servos and inputs (the emergency pin BTN_EMERG_GPIO itself is allowed). */
bool gpio_policy_estop_allowed(int pin, int start_pin, int led_pin);
/* Same reserved/Start/LED rules, plus: not the emergency pin, not the base pin (continuous
 * rotation: an angle command would spin it) and not any input pin assigned in config.h
 * (emergency, end stop, potentiometer, encoder, I2C). Servo output pins are allowed. */
bool gpio_policy_servo_allowed(int pin, int start_pin, int led_pin, int estop_pin, int base_pin);
#endif
