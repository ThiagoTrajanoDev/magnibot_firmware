#ifndef SERIAL_CONTROL_H
#define SERIAL_CONTROL_H
#include "esp_err.h"
#include "state_machine.h"
esp_err_t serial_control_init(void);
/* Receives a bounded RX batch (emergency executes here) without emitting any output. */
void serial_control_poll(robot_state_t *state);
/* Emits at most one queued response; call after the runtime has reacted to the state. */
void serial_control_flush(void);
#endif
