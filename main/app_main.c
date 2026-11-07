#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "config.h"
#include "button_control.h"
#include "led_control.h"
#include "bench_control.h"

static const char *TAG = "MAGNIBOT";
static portMUX_TYPE emergency_lock = portMUX_INITIALIZER_UNLOCKED;
static bool emergency_latched;

/* No PWM driver is present: ISR only latches the request. No 10 us claim. */
static void __attribute__((unused)) emergency_isr(void *arg) {
    (void)arg;
    portENTER_CRITICAL_ISR(&emergency_lock);
    emergency_latched = true;
    portEXIT_CRITICAL_ISR(&emergency_lock);
}

static esp_err_t emergency_init(void) {
#if CONFIG_MAGNIBOT_ESTOP_GPIO >= 0
    const int pin = CONFIG_MAGNIBOT_ESTOP_GPIO;
    if (!GPIO_IS_VALID_GPIO(pin) || pin == BTN_START_GPIO || pin == WS2812_GPIO)
        return ESP_ERR_INVALID_ARG;
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin, .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_NEGEDGE
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;
    err = gpio_install_isr_service(0);
    if (err != ESP_OK) return err;
    err = gpio_isr_handler_add(pin, emergency_isr, NULL);
    if (err != ESP_OK) return err;
    portENTER_CRITICAL(&emergency_lock);
    emergency_latched |= gpio_get_level(pin) == 0;
    portEXIT_CRITICAL(&emergency_lock);
#endif
    return ESP_OK;
}

void app_main(void) {
    robot_state_t state = ROBOT_IDLE;
    button_control_t start = {0};
    bool button_ok = button_control_init(&start, BTN_START_GPIO) == ESP_OK;
    esp_err_t led_err = led_control_init();
    esp_err_t emergency_err = emergency_init();
    if (!button_ok || led_err != ESP_OK || emergency_err != ESP_OK) {
        bench_control_event(&state, EVENT_EMERGENCY);
        ESP_LOGE(TAG, "Falha de inicializacao: button=%d led=%s estop=%s",
                 button_ok, esp_err_to_name(led_err), esp_err_to_name(emergency_err));
    }
    ESP_LOGW(TAG, "BANCADA: movimento bloqueado; sensores ausentes; estop_gpio=%d",
             CONFIG_MAGNIBOT_ESTOP_GPIO);
    led_control_update(state);
    int64_t next_telemetry = 0;
    for (;;) {
        portENTER_CRITICAL(&emergency_lock);
        bool emergency = emergency_latched;
        portEXIT_CRITICAL(&emergency_lock);
        if (emergency && bench_control_event(&state, EVENT_EMERGENCY))
            led_control_update(state);
        robot_event_t event;
        if (button_ok && button_control_update(&start, &event)) {
            bench_control_event(&state, event);
            ESP_LOGW(TAG, "Comando %d bloqueado: bancada sem atuadores validados", event);
        }
        int64_t now = esp_timer_get_time() / 1000;
        if (now >= next_telemetry) {
            printf("{\"type\":\"telemetry\",\"version\":1,\"uptime_ms\":%" PRId64
                   ",\"state\":\"%s\",\"mode\":\"bench\",\"motion_enabled\":false,"
                   "\"estop_configured\":%s,\"angles_deg\":null,\"distance_mm\":null,"
                   "\"current_ma\":null,\"gripper_percent\":null}\n",
                   now, state_machine_get_name(state),
                   CONFIG_MAGNIBOT_ESTOP_GPIO >= 0 ? "true" : "false");
            next_telemetry = now + 500;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
