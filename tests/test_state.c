#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include "bench_control.h"
int main(void) {
    robot_state_t state = ROBOT_IDLE;
    assert(!state_machine_process_event(NULL, EVENT_EMERGENCY));
    assert(state_machine_process_event(&state, EVENT_SINGLE_CLICK));
    assert(state == ROBOT_RUN_VARREDURA);
    assert(!state_machine_process_event(&state, EVENT_DOUBLE_CLICK));
    assert(state_machine_process_event(&state, EVENT_SCAN_DONE));
    assert(state_machine_process_event(&state, EVENT_LONG_PRESS));
    assert(state_machine_process_event(&state, EVENT_SCAN_DONE));
    assert(state_machine_process_event(&state, EVENT_DOUBLE_CLICK));
    assert(state_machine_process_event(&state, EVENT_MOVE_DONE));
    for (int s = ROBOT_IDLE; s <= ROBOT_ERROR; ++s) {
        state = s; bench_control_event(&state, EVENT_EMERGENCY);
        assert(state == ROBOT_ERROR);
        for (int e = EVENT_SINGLE_CLICK; e <= EVENT_EMERGENCY; ++e) {
            assert(!bench_control_event(&state, e));
            assert(!state_machine_process_event(&state, e));
            assert(state == ROBOT_ERROR);
        }
    }
    for (int e = EVENT_SINGLE_CLICK; e < EVENT_EMERGENCY; ++e) {
        state = ROBOT_IDLE; assert(!bench_control_event(&state, e)); assert(state == ROBOT_IDLE);
    }
    puts("PASS: state cycle, emergency latch, bench interlock");
}
