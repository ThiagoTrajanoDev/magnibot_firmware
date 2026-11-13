
#include "state_machine.h"
#include <stddef.h>

bool state_machine_process_event(
    robot_state_t *state,
    robot_event_t event
) {
    if (state == NULL) {
        return false;
    }

    robot_state_t previous_state = *state;

    // Emergencia pode ocorrer em qualquer estado
    if (event == EVENT_EMERGENCY) {
        *state = ROBOT_ERROR;
        return previous_state != *state;
    }

    // Nenhum evento normal e aceito em emergencia
    if (*state == ROBOT_ERROR) {
        return false;
    }

    switch (*state) {

        case ROBOT_IDLE:

            if (event == EVENT_SINGLE_CLICK) {
                *state = ROBOT_RUN_VARREDURA;
            }

            break;

        case ROBOT_RUN_VARREDURA:

            if (event == EVENT_SCAN_DONE) {
                *state = ROBOT_VARREDURA_CONCLUIDA;
            }

            break;

        case ROBOT_VARREDURA_CONCLUIDA:

            if (event == EVENT_DOUBLE_CLICK) {
                *state = ROBOT_RUN_DESLOCAMENTO;
            }
            else if (event == EVENT_LONG_PRESS) {
                *state = ROBOT_RUN_VARREDURA;
            }

            break;

        case ROBOT_RUN_DESLOCAMENTO:

            if (event == EVENT_MOVE_DONE) {
                *state = ROBOT_IDLE;
            }

            break;

        case ROBOT_ERROR:
            break;
    }

    return previous_state != *state;
}

const char *state_machine_get_name(robot_state_t state) {

    switch (state) {

        case ROBOT_IDLE:
            return "IDLE";

        case ROBOT_RUN_VARREDURA:
            return "RUN_VARREDURA";

        case ROBOT_VARREDURA_CONCLUIDA:
            return "VARREDURA_CONCLUIDA";

        case ROBOT_RUN_DESLOCAMENTO:
            return "RUN_DESLOCAMENTO";

        case ROBOT_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}
