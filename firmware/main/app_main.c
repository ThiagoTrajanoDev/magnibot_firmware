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
#include "tof_vl53l0x.h"
#include "gpio_policy.h"
#include "bench_servo.h"
#include "servo_pwm.h"
#include "esp_attr.h"

static const char *TAG = "MAGNIBOT";
static portMUX_TYPE emergency_lock = portMUX_INITIALIZER_UNLOCKED;
static bool emergency_latched;
static bench_servo_t servos;
static base_scan_t base_scan;

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

/* Servo commands stay refused unless CONFIG_MAGNIBOT_BENCH_SERVO is enabled. */
static esp_err_t servo_init(void) {
    bench_servo_config_t config = {
        .max_dps = 90,
        .start_pin = BTN_START_GPIO, .led_pin = WS2812_GPIO,
        .estop_pin = CONFIG_MAGNIBOT_ESTOP_GPIO, .base_pin = SERVO_BASE_GPIO
    };
    const bench_servo_driver_t *backend = NULL;
#if CONFIG_MAGNIBOT_BENCH_SERVO
    static bench_servo_driver_t driver;
    config.max_dps = CONFIG_MAGNIBOT_SERVO_MAX_DPS;
    esp_err_t err = servo_pwm_init(&driver);
    if (err != ESP_OK) return err;
    backend = &driver;
#endif
    return bench_servo_init(&servos, backend, &config) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

#if CONFIG_MAGNIBOT_TOF_DIAGNOSTIC
static void distance_diagnostic(void) {
    for (;;) {
        sensor_driver_t driver;
        esp_err_t result = tof_vl53l0x_init(&driver);
        if (result != ESP_OK) {
            printf("{\"type\":\"telemetry\",\"state\":\"DIAGNOSTIC\",\"distance_active\":true,"
                   "\"distance_mm\":null,\"distance_status\":\"init_failed\"}\n");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        sensor_data_t data;
        sensor_interface_t acquisition = {0};
        if (!sensor_interface_init(&acquisition, &data, &driver, NULL, 100, 200, 500)) return;
        int64_t next_output = 0;
        for (;;) {
            int64_t now = esp_timer_get_time() / 1000;
            sensor_interface_poll(&acquisition, now);
            hardware_result_t io = acquisition.distance.last_result;
            if (io == IO_FAULT || io == IO_TIMEOUT) break;
            if (now >= next_output) {
                sensor_status_t status = sensor_sample_check(&data.distance_mm, now, 500, SENSOR_HARDWARE);
                char mm[24] = "null";
                if (status == SENSOR_FRESH) snprintf(mm, sizeof(mm), "%.0f", (double)data.distance_mm.value);
                printf("{\"type\":\"telemetry\",\"state\":\"DIAGNOSTIC\",\"distance_active\":true,"
                       "\"distance_mm\":%s,\"distance_status\":\"%s\"}\n", mm, sensor_status_name(status));
                next_output = now + 500;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        sensor_interface_stop(&acquisition);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
#endif

void app_main(void) {
#if CONFIG_MAGNIBOT_TOF_DIAGNOSTIC
    distance_diagnostic();
    return;
#endif
    robot_state_t state = ROBOT_IDLE;
    bool runtime_ok;
#if CONFIG_MAGNIBOT_TOF
    sensor_driver_t distance_driver;
    esp_err_t tof_err = tof_vl53l0x_init(&distance_driver);
    runtime_ok = robot_runtime_init_with_distance(tof_err == ESP_OK ? &distance_driver : NULL);
    if (tof_err != ESP_OK) ESP_LOGE(TAG, "VL53L0X: %s", esp_err_to_name(tof_err));
#else
    runtime_ok = robot_runtime_init();
#endif
    button_control_t start = {0};
    bool button_ok = button_control_init(&start, BTN_START_GPIO) == ESP_OK;
    esp_err_t led_err = led_control_init();
    esp_err_t emergency_err = emergency_init();
    esp_err_t servo_err = servo_init();
    base_scan_driver_t base_driver = {0};
#if CONFIG_MAGNIBOT_BASE_SCAN
    servo_pwm_base_driver(&base_driver);
#endif
    base_scan_init(&base_scan, &base_driver);
#if CONFIG_MAGNIBOT_BASE_SCAN
    unsigned started_scan = 0;
#endif
    esp_err_t serial_err = serial_control_init(&servos);
    if (!runtime_ok || !button_ok || led_err != ESP_OK || emergency_err != ESP_OK ||
        servo_err != ESP_OK || serial_err != ESP_OK) {
        bench_control_event(&state, EVENT_EMERGENCY);
        ESP_LOGE(TAG, "Falha de inicializacao: runtime=%d button=%d led=%s estop=%s servo=%s serial=%s",
                 runtime_ok, button_ok, esp_err_to_name(led_err), esp_err_to_name(emergency_err),
                 esp_err_to_name(servo_err), esp_err_to_name(serial_err));
    }
    ESP_LOGW(TAG, "BANCADA: ciclo bloqueado; estop_gpio=%d; servo_bancada=%d",
             CONFIG_MAGNIBOT_ESTOP_GPIO, bench_servo_enabled(&servos));
    robot_state_t led_state = state;
    led_control_update(led_state);
    int64_t next_telemetry = 0;
    bool boot_idle_pending = true;
    TickType_t next_control_tick = xTaskGetTickCount();
    for (;;) {
        portENTER_CRITICAL(&emergency_lock);
        bool emergency = emergency_latched;
        portEXIT_CRITICAL(&emergency_lock);
        if (emergency) bench_control_event(&state, EVENT_EMERGENCY);
        if (boot_idle_pending) {
            boot_idle_pending = false;
            if (state == ROBOT_IDLE && bench_servo_enabled(&servos)) {
                if (!bench_servo_idle_pose(&servos))
                    bench_control_event(&state, EVENT_EMERGENCY);
                else
                    ESP_LOGI(TAG, "Movendo para pose IDLE apos reset");
            }
        }
        serial_control_poll(&state);
        robot_event_t event;
        bool gesture = button_ok && button_control_update(&start, &event);
        bool preparing = gesture && event == EVENT_SINGLE_CLICK && state == ROBOT_IDLE &&
                         bench_servo_prepare_scan(&servos);
        if (gesture && !preparing) bench_control_event(&state, event);
        int64_t now = esp_timer_get_time() / 1000;
        /* Servo slew, runtime (actuator stop) and LED react before any console output. */
        if (state != ROBOT_ERROR) {
            bench_servo_poll(&servos, now);
#if CONFIG_MAGNIBOT_BASE_SCAN
            if (servos.distance_ready &&
                servos.scan_request_id != started_scan) {
                started_scan = servos.scan_request_id;
                if (!base_scan_begin(&base_scan, now))
                    bench_control_event(&state, EVENT_EMERGENCY);
            }
#endif
            base_scan_poll(&base_scan, now);
            servos.base_active = base_scan.moving;
        }
        if (base_scan.fault) bench_control_event(&state, EVENT_EMERGENCY);
        if (bench_servo_faulted(&servos)) bench_control_event(&state, EVENT_EMERGENCY);
        if (state == ROBOT_ERROR) {
            base_scan_stop(&base_scan);
            bench_servo_stop(&servos);
        }
        robot_runtime_poll_with_acquisition(&state, now, !servos.scan_pose_active);
        if (state != led_state) {
            led_state = state;
            led_control_update(state);
        }
        if (preparing) ESP_LOGI(TAG, "Iniciando varredura");
        if (gesture && !preparing) ESP_LOGW(TAG, "Comando %d bloqueado: bancada sem atuadores validados", event);
        serial_control_flush();
        if (!servos.scan_pose_active && now >= next_telemetry) {
            robot_runtime_status_t runtime = robot_runtime_status(now);
            char distance[24] = "null";
            float mm;
            if (servos.distance_ready && robot_runtime_distance(now, &mm))
                snprintf(distance, sizeof(distance), "%.0f", (double)mm);
            if (base_scan.moving)
                printf("{\"type\":\"telemetry\",\"state\":\"VARREDURA\",\"distance_active\":true,"
                       "\"distance_mm\":%s,\"distance_status\":\"%s\",\"base_phase\":%u}\n",
                       distance, sensor_status_name(runtime.distance_status), base_scan.phase);
            else printf("{\"type\":\"telemetry\",\"version\":1,\"uptime_ms\":%" PRId64
                   ",\"state\":\"%s\",\"state_code\":%d," ROBOT_MODE_JSON ","
                   "\"estop_configured\":%s,\"angles_deg\":null,\"distance_mm\":%s,"
                   "\"distance_active\":%s,\"distance_status\":\"%s\","
                   "\"current_ma\":null,\"gripper_percent\":null,\"execution\":{\"backend\":\"%s\","
                   "\"axes_configured\":%u,\"phase\":\"%s\",\"fault\":\"%s\"},"
                   "\"bench_servo\":{\"enabled\":%s,\"channels\":%u},\"stack_min_free_bytes\":%u}\n",
                   now, state_machine_get_name(state), state,
                   CONFIG_MAGNIBOT_ESTOP_GPIO >= 0 ? "true" : "false",
                   distance, servos.distance_ready ? "true" : "false",
                   sensor_status_name(runtime.distance_status), runtime.backend, runtime.axes_configured, cycle_phase_name(runtime.phase),
                   cycle_fault_name(runtime.fault), bench_servo_enabled(&servos) ? "true" : "false",
                   bench_servo_channels(&servos), (unsigned)uxTaskGetStackHighWaterMark(NULL));
            next_telemetry = now + 500;
        }
        if (xTaskDelayUntil(&next_control_tick, pdMS_TO_TICKS(10)) != pdTRUE)
            next_control_tick = xTaskGetTickCount();
    }
}
