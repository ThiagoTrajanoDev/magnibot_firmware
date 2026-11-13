
#include "button_control.h"

#include "esp_timer.h"
#include "esp_log.h"

#include <stddef.h>

static const char *TAG = "BUTTON";

// Retorna o tempo atual em milissegundos
static int64_t get_time_ms(void) {
    return esp_timer_get_time() / 1000;
}

esp_err_t button_control_init(button_control_t *button, gpio_num_t gpio) {

    if (button == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!GPIO_IS_VALID_GPIO(gpio)) return ESP_ERR_INVALID_ARG;
    button->gpio = gpio;

    button->estado_candidato = 1;
    button->estado_estavel = 1;

    button->tempo_mudanca = get_time_ms();
    button->tempo_pressionado = 0;

    button->long_press_emitido = false;

    button->aguardando_segundo_clique = false;
    button->tempo_primeiro_clique = 0;

    gpio_config_t config = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Botao inicializado no GPIO %d", gpio);
    return ESP_OK;
}

bool button_control_update(
    button_control_t *button,
    robot_event_t *event
) {

    if (button == NULL || event == NULL) {
        return false;
    }

    return button_control_sample(button, gpio_get_level(button->gpio), get_time_ms(), event);
}

bool button_control_sample(button_control_t *button, int leitura, int64_t agora, robot_event_t *event) {
    if (!button || !event || (leitura != 0 && leitura != 1)) return false;

    // Detecta mudanca no candidato
    if (leitura != button->estado_candidato) {

        button->estado_candidato = leitura;
        button->tempo_mudanca = agora;
    }

    // Verifica se a leitura permaneceu estavel
    if (
        button->estado_candidato != button->estado_estavel &&
        (agora - button->tempo_mudanca) >= BUTTON_DEBOUNCE_MS
    ) {

        button->estado_estavel = button->estado_candidato;

        // Botao pressionado (nivel baixo)
        if (button->estado_estavel == 0) {

            button->tempo_pressionado = agora;
            button->long_press_emitido = false;
        }

        // Botao solto (nivel alto)
        else {

            int64_t duracao =
                agora - button->tempo_pressionado;

            // Se nao foi clique longo
            if (!button->long_press_emitido) {

                if (duracao <= 1000) {

                    // Verifica se ja existia um primeiro clique
                    if (
                        button->aguardando_segundo_clique &&
                        (agora - button->tempo_primeiro_clique)
                            < BUTTON_DOUBLE_CLICK_MS
                    ) {

                        button->aguardando_segundo_clique = false;

                        *event = EVENT_DOUBLE_CLICK;

                        ESP_LOGI(TAG, "Duplo clique detectado");

                        return true;
                    }

                    // Registra o primeiro clique
                    else {

                        button->aguardando_segundo_clique = true;
                        button->tempo_primeiro_clique = agora;
                    }
                }
            }
        }
    }

    // Detecta pressao longa
    if (
        button->estado_estavel == 0 &&
        !button->long_press_emitido &&
        (agora - button->tempo_pressionado) > BUTTON_LONG_PRESS_MS
    ) {

        button->long_press_emitido = true;
        button->aguardando_segundo_clique = false;

        *event = EVENT_LONG_PRESS;

        ESP_LOGI(TAG, "Pressao longa detectada");

        return true;
    }

    // Expirou o tempo para um segundo clique
    if (
        button->aguardando_segundo_clique &&
        button->estado_estavel == 1 &&
        (agora - button->tempo_primeiro_clique)
            >= BUTTON_DOUBLE_CLICK_MS
    ) {

        button->aguardando_segundo_clique = false;

        *event = EVENT_SINGLE_CLICK;

        ESP_LOGI(TAG, "Clique simples detectado");

        return true;
    }

    return false;
}
