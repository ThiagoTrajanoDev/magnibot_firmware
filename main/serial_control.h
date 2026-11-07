#ifndef SERIAL_CONTROL_H
#define SERIAL_CONTROL_H
#include "esp_err.h"
#include "state_machine.h"
esp_err_t serial_control_init(void);
void serial_control_poll(robot_state_t *state);
#endif
