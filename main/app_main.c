
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "config.h"
#include "state_machine.h"
#include "button_control.h"

static const char *TAG = "MAGNIBOT";

void app_main(void) {

    robot_state_t estado_atual = ROBOT_IDLE;

    // Estrutura do botao Start
    button_control_t botao_start;

    // Inicializacao
    ESP_LOGI(TAG, "Inicializando MagniBot...");

    button_control_init(
        &botao_start,
        BTN_START_GPIO
    );

    ESP_LOGI(TAG, "Estado inicial: %s",
        state_machine_get_name(estado_atual)
    );

    while (1) {

        robot_event_t evento;

        // Verifica se algum evento foi identificado
        if (
            button_control_update(
                &botao_start,
                &evento
            )
        ) {

            // Envia evento para a maquina de estados
            bool mudou = state_machine_process_event(
                &estado_atual,
                evento
            );

            if (mudou) {

                ESP_LOGI(
                    TAG,
                    "Novo estado: %s",
                    state_machine_get_name(estado_atual)
                );
            }
        }

        // Periodo de monitoramento: 10 ms
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
