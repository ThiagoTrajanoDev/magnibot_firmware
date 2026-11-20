#include "servo_pwm.h"
#include "driver/ledc.h"
#include "config.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"

#define SERVO_FREQ_HZ 50
#define SERVO_PERIOD_US (1000000 / SERVO_FREQ_HZ)
#define POSITION_RESOLUTION_HZ 3200000u
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO_DUTY_STEPS (1u << SERVO_RESOLUTION)

static unsigned attached;
static mcpwm_timer_handle_t position_timer;
static mcpwm_oper_handle_t operators[3];
static mcpwm_cmpr_handle_t comparators[BENCH_SERVO_MAX_CHANNELS];
static mcpwm_gen_handle_t generators[BENCH_SERVO_MAX_CHANNELS];
static bool pulse_started[BENCH_SERVO_MAX_CHANNELS];
static int64_t release_after_us[BENCH_SERVO_MAX_CHANNELS];

static hardware_result_t shutdown(void *context) {
    (void)context;
    hardware_result_t result = IO_OK;
    for (unsigned i = 0; i < BENCH_SERVO_MAX_CHANNELS; ++i) {
        release_after_us[i] = 0;
        if (generators[i] && mcpwm_generator_set_force_level(generators[i], 0, true) != ESP_OK)
            result = IO_FAULT;
    }
    return result;
}

static hardware_result_t attach(void *context, int gpio, unsigned channel) {
    (void)context;
    if (channel >= BENCH_SERVO_MAX_CHANNELS || channel != attached) return IO_INVALID;
    mcpwm_comparator_config_t comparator = {.flags.update_cmp_on_tez = true};
    if (mcpwm_new_comparator(operators[channel / 2], &comparator, &comparators[channel]) != ESP_OK)
        return IO_FAULT;
    mcpwm_generator_config_t generator = {.gen_gpio_num = gpio};
    if (mcpwm_new_generator(operators[channel / 2], &generator, &generators[channel]) != ESP_OK)
        return IO_FAULT;
    if (mcpwm_generator_set_force_level(generators[channel], 0, true) != ESP_OK ||
        mcpwm_generator_set_action_on_timer_event(generators[channel],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)) != ESP_OK ||
        mcpwm_generator_set_action_on_compare_event(generators[channel],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[channel], MCPWM_GEN_ACTION_LOW)) != ESP_OK)
        return IO_FAULT;
    ++attached;
    return IO_OK;
}

static hardware_result_t write_precise_us(void *context, unsigned channel, float pulse_us) {
    (void)context;
    if (channel >= attached || !(pulse_us >= 500 && pulse_us <= 2500)) return IO_INVALID;
    uint32_t ticks = (uint32_t)(pulse_us * (POSITION_RESOLUTION_HZ / 1000000.0f) + 0.5f);
    if (mcpwm_comparator_set_compare_value(comparators[channel], ticks) != ESP_OK) return IO_FAULT;
    if (!pulse_started[channel] && !release_after_us[channel])
        release_after_us[channel] = esp_timer_get_time() + 20000;
    return IO_OK;
}
static hardware_result_t poll_outputs(void *context) {
    (void)context;
    int64_t now = esp_timer_get_time();
    for (unsigned i = 0; i < attached; ++i) {
        if (!release_after_us[i] || now < release_after_us[i]) continue;
        if (mcpwm_generator_set_force_level(generators[i], -1, true) != ESP_OK) return IO_FAULT;
        pulse_started[i] = true;
        release_after_us[i] = 0;
    }
    return IO_OK;
}
static hardware_result_t write_us(void *context, unsigned channel, unsigned pulse_us) {
    return write_precise_us(context, channel, pulse_us);
}

esp_err_t servo_pwm_init(bench_servo_driver_t *driver) {
    if (!driver || position_timer) return ESP_ERR_INVALID_ARG;
    /* LEDC is retained for the independent continuous-rotation base. */
    ledc_timer_config_t base_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = SERVO_RESOLUTION,
        .timer_num = LEDC_TIMER_0, .freq_hz = SERVO_FREQ_HZ, .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t result = ledc_timer_config(&base_timer);
    if (result != ESP_OK) return result;
    mcpwm_timer_config_t timer = {
        .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = POSITION_RESOLUTION_HZ,
        .period_ticks = POSITION_RESOLUTION_HZ / SERVO_FREQ_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    result = mcpwm_new_timer(&timer, &position_timer);
    if (result != ESP_OK) return result;
    for (unsigned i = 0; i < 3; ++i) {
        mcpwm_operator_config_t config = {.group_id = 0};
        result = mcpwm_new_operator(&config, &operators[i]);
        if (result != ESP_OK) return result;
        result = mcpwm_operator_connect_timer(operators[i], position_timer);
        if (result != ESP_OK) return result;
    }
    result = mcpwm_timer_enable(position_timer);
    if (result != ESP_OK) return result;
    result = mcpwm_timer_start_stop(position_timer, MCPWM_TIMER_START_NO_STOP);
    if (result != ESP_OK) return result;
    *driver = (bench_servo_driver_t){
        .attach = attach, .write_us = write_us, .shutdown = shutdown,
        .write_precise_us = write_precise_us, .poll = poll_outputs
    };
    return ESP_OK;
}

static bool base_attached;
static hardware_result_t base_write(void *context, unsigned pulse) {
    (void)context;
    if (pulse != 1488 && pulse != 1573 && pulse != 1403) return IO_INVALID;
    if (!base_attached) {
        ledc_channel_config_t config = {
            .gpio_num = SERVO_BASE_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_5, .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE, .duty = 0, .hpoint = 0
        };
        if (ledc_channel_config(&config) != ESP_OK) return IO_FAULT;
        base_attached = true;
    }
    uint32_t duty = (uint32_t)((uint64_t)pulse * SERVO_DUTY_STEPS / SERVO_PERIOD_US);
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, duty) != ESP_OK ||
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5) != ESP_OK) return IO_FAULT;
    return IO_OK;
}
static hardware_result_t base_shutdown(void *context) {
    (void)context;
    if (!base_attached) return IO_OK;
    return ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, 0) == ESP_OK ? IO_OK : IO_FAULT;
}
void servo_pwm_base_driver(base_scan_driver_t *driver) {
    *driver = (base_scan_driver_t){.write_us = base_write, .shutdown = base_shutdown};
}
