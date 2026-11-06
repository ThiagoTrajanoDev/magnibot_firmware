
#ifndef BUTTON_CONTROL_H
#define BUTTON_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "state_machine.h"

// Tempos em milissegundos
#define BUTTON_DEBOUNCE_MS       50
#define BUTTON_DOUBLE_CLICK_MS 1000
#define BUTTON_LONG_PRESS_MS   2000

typedef struct {

    gpio_num_t gpio;

    // Debounce
    int estado_candidato;
    int estado_estavel;

    int64_t tempo_mudanca;

    // Controle de pressionamento
    int64_t tempo_pressionado;

    bool long_press_emitido;

    // Controle de duplo clique
    bool aguardando_segundo_clique;
    int64_t tempo_primeiro_clique;

} button_control_t;

// Inicializa o botao
void button_control_init(button_control_t *button, gpio_num_t gpio);

// Atualiza a leitura e retorna um evento quando identificado
bool button_control_update(
    button_control_t *button,
    robot_event_t *event
);

#endif
