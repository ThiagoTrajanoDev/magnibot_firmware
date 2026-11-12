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
#include "serial_control.h"
#include "robot_runtime.h"
#include "gpio_policy.h"
#include "esp_attr.h"

static const char *TAG = "MAGNIBOT";
static portMUX_TYPE emergency_lock = portMUX_INITIALIZER_UNLOCKED;
static bool emergency_latched;

/* No PWM driver is present: ISR only latches the request. No 10 us claim.
 * Kept in IRAM so it does not depend on the flash cache. */
static void IRAM_ATTR __attribute__((unused)) emergency_isr(void *arg) {
    (void)arg;
    portENTER_CRITICAL_ISR(&emergency_lock);
    emergency_latched = true;
    portEXIT_CRITICAL_ISR(&emergency_lock);
}

static esp_err_t emergency_init(void) {
#if CONFIG_MAGNIBOT_ESTOP_GPIO >= 0
    const int pin = CONFIG_MAGNIBOT_ESTOP_GPIO;
    if (!gpio_policy_estop_allowed(pin, BTN_START_GPIO, WS2812_GPIO)) return ESP_ERR_INVALID_ARG;
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin, .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_NEGEDGE
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
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
    bool runtime_ok = robot_runtime_init();
    button_control_t start = {0};
    bool button_ok = button_control_init(&start, BTN_START_GPIO) == ESP_OK;
    esp_err_t led_err = led_control_init();
    esp_err_t emergency_err = emergency_init();
    esp_err_t serial_err = serial_control_init();
    if (!runtime_ok || !button_ok || led_err != ESP_OK || emergency_err != ESP_OK || serial_err != ESP_OK) {
        bench_control_event(&state, EVENT_EMERGENCY);
        ESP_LOGE(TAG, "Falha de inicializacao: runtime=%d button=%d led=%s estop=%s serial=%s",
                 runtime_ok, button_ok, esp_err_to_name(led_err), esp_err_to_name(emergency_err), esp_err_to_name(serial_err));
    }
    ESP_LOGW(TAG, "BANCADA: movimento bloqueado; sensores ausentes; estop_gpio=%d",
             CONFIG_MAGNIBOT_ESTOP_GPIO);
    robot_state_t led_state = state;
    led_control_update(led_state);
    int64_t next_telemetry = 0;
    for (;;) {
        portENTER_CRITICAL(&emergency_lock);
        bool emergency = emergency_latched;
        portEXIT_CRITICAL(&emergency_lock);
        if (emergency) bench_control_event(&state, EVENT_EMERGENCY);
        serial_control_poll(&state);
        robot_event_t event;
        bool gesture = button_ok && button_control_update(&start, &event);
        if (gesture) bench_control_event(&state, event);
        int64_t now = esp_timer_get_time() / 1000;
        /* Runtime (actuator stop) and LED react before any console output. */
        robot_runtime_poll(&state, now);
        if (state != led_state) {
            led_state = state;
            led_control_update(state);
        }
        if (gesture) ESP_LOGW(TAG, "Comando %d bloqueado: bancada sem atuadores validados", event);
        serial_control_flush();
        if (now >= next_telemetry) {
            robot_runtime_status_t runtime = robot_runtime_status(now);
            printf("{\"type\":\"telemetry\",\"version\":1,\"uptime_ms\":%" PRId64
                   ",\"state\":\"%s\",\"state_code\":%d," ROBOT_MODE_JSON ","
                   "\"estop_configured\":%s,\"angles_deg\":null,\"distance_mm\":null,"
                   "\"current_ma\":null,\"gripper_percent\":null,\"execution\":{\"backend\":\"%s\","
                   "\"axes_configured\":%u,\"phase\":\"%s\",\"fault\":\"%s\"},"
                   "\"stack_min_free_bytes\":%u}\n",
                   now, state_machine_get_name(state), state,
                   CONFIG_MAGNIBOT_ESTOP_GPIO >= 0 ? "true" : "false",
                   runtime.backend, runtime.axes_configured, cycle_phase_name(runtime.phase),
                   cycle_fault_name(runtime.fault), (unsigned)uxTaskGetStackHighWaterMark(NULL));
            next_telemetry = now + 500;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
