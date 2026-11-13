#ifndef BENCH_CONTROL_H
#define BENCH_CONTROL_H
#include "state_machine.h"
/* No motion authorization exists in the bench firmware. */
bool bench_control_event(robot_state_t *state, robot_event_t event);
#endif
