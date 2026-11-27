#ifndef SERVO_PWM_H
#define SERVO_PWM_H
#include "esp_err.h"
#include "bench_servo.h"
#include "base_scan.h"
/* Positional MCPWM backend, 50 Hz and 0.3125 us compare steps. Outputs start on first write. */
esp_err_t servo_pwm_init(bench_servo_driver_t *driver);
void servo_pwm_base_driver(base_scan_driver_t *driver);
#endif
