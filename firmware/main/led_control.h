#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "state_machine.h"
#include "esp_err.h"

esp_err_t led_control_init(void);

void led_control_update(robot_state_t state);

void led_control_clear(void);

#endif
