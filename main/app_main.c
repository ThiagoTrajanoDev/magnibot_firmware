#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "config.h"
#include "state_machine.h"


static const char *TAG = "MAGNIBOT";

void app_main(void) {

    // Estado inicial do robo
    robot_state_t estado_atual = ROBOT_IDLE;

    ESP_LOGI(TAG, "Inicializando MagniBot...");
    ESP_LOGI(TAG, "Eixos configurados: %d", TOTAL_EIXOS);
    ESP_LOGI(TAG, "UART: %d bps", UART_BAUDRATE);

    ESP_LOGI(
        TAG,
        "Estado inicial: %s",
        state_machine_get_name(estado_atual)
    );

    while (1) {

        // Futuramente:
        // 1. Leitura dos botoes
        // 2. Processamento dos eventos
        // 3. Controle dos motores
        // 4. Leitura dos sensores
        // 5. Atualizacao da telemetria

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
