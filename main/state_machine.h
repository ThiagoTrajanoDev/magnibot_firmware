#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>

// Estados operacionais
typedef enum {
    ROBOT_IDLE,
    ROBOT_RUN_VARREDURA,
    ROBOT_VARREDURA_CONCLUIDA,
    ROBOT_RUN_DESLOCAMENTO,
    ROBOT_ERROR
} robot_state_t;

// Eventos do sistema
typedef enum {
    EVENT_SINGLE_CLICK,
    EVENT_DOUBLE_CLICK,
    EVENT_LONG_PRESS,

    EVENT_SCAN_DONE,
    EVENT_MOVE_DONE,

    EVENT_EMERGENCY
} robot_event_t;

// Processa um evento e atualiza o estado
bool state_machine_process_event(
    robot_state_t *state,
    robot_event_t event
);

// Retorna o nome do estado
const char *state_machine_get_name(
    robot_state_t state
);

#endif
