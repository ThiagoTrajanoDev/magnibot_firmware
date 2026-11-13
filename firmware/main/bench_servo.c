#include "bench_servo.h"
#include "gpio_policy.h"
#include <stddef.h>

#define PULSE_MIN_LIMIT_US 400
#define PULSE_MAX_LIMIT_US 2600
#define MAX_DPS_LIMIT 360

static unsigned pulse_us(const bench_servo_config_t *config, float degrees) {
    return (unsigned)(config->min_us +
        (config->max_us - config->min_us) * degrees / BENCH_SERVO_MAX_DEGREES + 0.5f);
}

static hardware_result_t write_channel(bench_servo_t *servos, unsigned index) {
    return servos->driver.write_us(servos->driver.context, index,
                                   pulse_us(&servos->config, servos->channels[index].current));
}

static void fail(bench_servo_t *servos) {
    servos->fault = true;
    bench_servo_stop(servos);
}

bool bench_servo_init(bench_servo_t *servos, const bench_servo_driver_t *driver,
                      const bench_servo_config_t *config) {
    if (!servos || !config || servos->initialized) return false;
    if (config->min_us < PULSE_MIN_LIMIT_US || config->max_us > PULSE_MAX_LIMIT_US ||
        config->min_us >= config->max_us || config->max_dps == 0 || config->max_dps > MAX_DPS_LIMIT)
        return false;
    if (driver && (!driver->attach || !driver->write_us || !driver->shutdown)) return false;
    *servos = (bench_servo_t){.config = *config, .initialized = true};
    if (driver) servos->driver = *driver;
    return true;
}

bench_servo_result_t bench_servo_set(bench_servo_t *servos, int gpio, int degrees) {
    if (!servos || !servos->initialized || !servos->driver.write_us) return BENCH_SERVO_DISABLED;
    if (servos->latched) return BENCH_SERVO_STOPPED;
    if (degrees < 0 || degrees > BENCH_SERVO_MAX_DEGREES) return BENCH_SERVO_BAD_ANGLE;
    const bench_servo_config_t *config = &servos->config;
    if (!gpio_policy_servo_allowed(gpio, config->start_pin, config->led_pin,
                                   config->estop_pin, config->base_pin))
        return BENCH_SERVO_BAD_GPIO;
    for (unsigned i = 0; i < servos->used; ++i) {
        if (servos->channels[i].gpio == gpio) {
            servos->channels[i].target = (float)degrees;
            return BENCH_SERVO_OK;
        }
    }
    if (servos->used == BENCH_SERVO_MAX_CHANNELS) return BENCH_SERVO_NO_CHANNEL;
    unsigned index = servos->used;
    if (servos->driver.attach(servos->driver.context, gpio, index) != IO_OK) {
        fail(servos);
        return BENCH_SERVO_FAULT;
    }
    servos->channels[index] = (bench_servo_channel_t){
        .gpio = gpio, .current = (float)degrees, .target = (float)degrees
    };
    ++servos->used;
    if (write_channel(servos, index) != IO_OK) {
        fail(servos);
        return BENCH_SERVO_FAULT;
    }
    return BENCH_SERVO_OK;
}

void bench_servo_poll(bench_servo_t *servos, int64_t now_ms) {
    if (!servos || !servos->initialized) return;
    int64_t elapsed_ms = now_ms - servos->last_ms;
    servos->last_ms = now_ms;
    if (servos->latched || elapsed_ms <= 0) return;
    float step = servos->config.max_dps * (float)elapsed_ms / 1000.0f;
    for (unsigned i = 0; i < servos->used; ++i) {
        bench_servo_channel_t *channel = &servos->channels[i];
        float delta = channel->target - channel->current;
        if (delta == 0.0f) continue;
        channel->current = delta > step ? channel->current + step
                         : delta < -step ? channel->current - step : channel->target;
        if (write_channel(servos, i) != IO_OK) {
            fail(servos);
            return;
        }
    }
}

void bench_servo_stop(bench_servo_t *servos) {
    if (!servos || !servos->initialized) return;
    servos->latched = true;
    if (servos->shutdown_confirmed) return;
    servos->shutdown_confirmed = !servos->driver.shutdown ||
        servos->driver.shutdown(servos->driver.context) == IO_OK;
}

bool bench_servo_enabled(const bench_servo_t *servos) {
    return servos && servos->initialized && servos->driver.write_us;
}

bool bench_servo_faulted(const bench_servo_t *servos) {
    return servos && servos->fault;
}

unsigned bench_servo_channels(const bench_servo_t *servos) {
    return servos ? servos->used : 0;
}

const char *bench_servo_result_name(bench_servo_result_t result) {
    switch (result) {
        case BENCH_SERVO_OK: return "ok";
        case BENCH_SERVO_DISABLED: return "servo_disabled";
        case BENCH_SERVO_BAD_GPIO: return "gpio_not_allowed";
        case BENCH_SERVO_BAD_ANGLE: return "angle_out_of_range";
        case BENCH_SERVO_NO_CHANNEL: return "no_free_channel";
        case BENCH_SERVO_STOPPED: return "servo_stopped";
        default: return "servo_fault";
    }
}
