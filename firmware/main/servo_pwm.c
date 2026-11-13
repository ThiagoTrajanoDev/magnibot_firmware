#include "servo_pwm.h"
#include "driver/ledc.h"

#define SERVO_FREQ_HZ 50
#define SERVO_PERIOD_US (1000000 / SERVO_FREQ_HZ)
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO_DUTY_STEPS (1u << SERVO_RESOLUTION)

static unsigned attached;

static hardware_result_t attach(void *context, int gpio, unsigned channel) {
    (void)context;
    if (channel >= BENCH_SERVO_MAX_CHANNELS || channel != attached) return IO_INVALID;
    ledc_channel_config_t config = {
        .gpio_num = gpio, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)(LEDC_CHANNEL_0 + channel), .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE, .duty = 0, .hpoint = 0
    };
    if (ledc_channel_config(&config) != ESP_OK) return IO_FAULT;
    ++attached;
    return IO_OK;
}

static hardware_result_t write_us(void *context, unsigned channel, unsigned pulse_us) {
    (void)context;
    if (channel >= attached) return IO_INVALID;
    ledc_channel_t id = (ledc_channel_t)(LEDC_CHANNEL_0 + channel);
    uint32_t duty = (uint32_t)((uint64_t)pulse_us * SERVO_DUTY_STEPS / SERVO_PERIOD_US);
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, id, duty) != ESP_OK ||
        ledc_update_duty(LEDC_LOW_SPEED_MODE, id) != ESP_OK) return IO_FAULT;
    return IO_OK;
}

/* Stops the pulses and leaves each pin low: the servo loses its holding signal.
 * This is not a physical power cut. */
static hardware_result_t shutdown(void *context) {
    (void)context;
    hardware_result_t result = IO_OK;
    for (unsigned i = 0; i < attached; ++i)
        if (ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(LEDC_CHANNEL_0 + i), 0) != ESP_OK)
            result = IO_FAULT;
    return result;
}

esp_err_t servo_pwm_init(bench_servo_driver_t *driver) {
    if (!driver) return ESP_ERR_INVALID_ARG;
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = SERVO_RESOLUTION,
        .timer_num = LEDC_TIMER_0, .freq_hz = SERVO_FREQ_HZ, .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) return err;
    *driver = (bench_servo_driver_t){.attach = attach, .write_us = write_us, .shutdown = shutdown};
    return ESP_OK;
}
