
#include "led_control.h"
#include "config.h"

#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "LED_CONTROL";

static led_strip_handle_t led_strip = NULL;

// Define a cor do LED
static void set_color(uint8_t red, uint8_t green, uint8_t blue) {

    if (led_strip == NULL) {
        return;
    }

    ESP_ERROR_CHECK(
        led_strip_set_pixel(
            led_strip,
            0,
            red,
            green,
            blue
        )
    );

    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

// Inicializa o WS2812
void led_control_init(void) {

    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format =
            LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false
    };

    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(
            &strip_config,
            &rmt_config,
            &led_strip
        )
    );

    led_control_clear();

    ESP_LOGI(TAG, "WS2812 inicializado");
}

// Atualiza a cor conforme o estado do robo
void led_control_update(robot_state_t state) {

    switch (state) {

        case ROBOT_IDLE:
            // Verde
            set_color(0, 255, 0);
            break;

        case ROBOT_RUN_VARREDURA:
            // Azul
            set_color(0, 0, 255);
            break;

        case ROBOT_VARREDURA_CONCLUIDA:
            // Amarelo
            set_color(255, 255, 0);
            break;

        case ROBOT_RUN_DESLOCAMENTO:
            // Roxo
            set_color(128, 0, 255);
            break;

        case ROBOT_ERROR:
            // Vermelho
            set_color(255, 0, 0);
            break;

        default:
            led_control_clear();
            break;
    }
}

// Apaga o LED
void led_control_clear(void) {

    if (led_strip != NULL) {
        ESP_ERROR_CHECK(led_strip_clear(led_strip));
    }
}
