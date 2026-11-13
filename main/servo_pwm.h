#ifndef SERVO_PWM_H
#define SERVO_PWM_H
#include "esp_err.h"
#include "bench_servo.h"
/* LEDC backend (50 Hz, 14 bit, one shared timer). Nothing is driven until a channel is attached. */
esp_err_t servo_pwm_init(bench_servo_driver_t *driver);
#endif
