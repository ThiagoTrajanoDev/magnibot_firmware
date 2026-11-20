#include "bench_servo.h"
#include "gpio_policy.h"
#include "config.h"
#include <stddef.h>
#include <math.h>
#include <string.h>
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

#ifndef CONFIG_MAGNIBOT_SHOULDER_MIN_US
#define CONFIG_MAGNIBOT_SHOULDER_MIN_US 500
#endif
#ifndef CONFIG_MAGNIBOT_SHOULDER_MAX_US
#define CONFIG_MAGNIBOT_SHOULDER_MAX_US 2500
#endif

#define MAX_DPS_LIMIT 360

typedef struct {
    const char *name;
    int gpio;
    unsigned at_zero_us, neutral_us, at_full_us;
} joint_pwm_t;

/* Bench pulse ranges and neutral references per positional joint. */
static const joint_pwm_t joint_pwm[] = {
    {"ombro", SERVO_OMBRO_GPIO, CONFIG_MAGNIBOT_SHOULDER_MIN_US, 1916, CONFIG_MAGNIBOT_SHOULDER_MAX_US},
    {"cotovelo", SERVO_COTOVELO_GPIO,   2500,  939,  500},
    {"punho_rot", SERVO_PUNHO_ROT_GPIO,   500, 1205, 2500},
    {"punho_flex", SERVO_PUNHO_FLEX_GPIO,  500, 1400, 2500},
    {"garra", SERVO_GARRA_GPIO,       500, 1000, 2500},
};

static const joint_pwm_t *find_joint(int gpio) {
    for (unsigned i = 0; i < sizeof(joint_pwm) / sizeof(joint_pwm[0]); ++i)
        if (joint_pwm[i].gpio == gpio) return &joint_pwm[i];
    return NULL;
}

int bench_servo_named_gpio(const char *name) {
    for (unsigned i = 0; i < sizeof(joint_pwm) / sizeof(joint_pwm[0]); ++i)
        if (strcmp(name, joint_pwm[i].name) == 0) return joint_pwm[i].gpio;
    return -1;
}

static float reference_to_pulse(const joint_pwm_t *joint, float reference) {
    if (reference <= 90)
        return joint->at_zero_us + (joint->neutral_us - (float)joint->at_zero_us) * reference / 90;
    return joint->neutral_us + (joint->at_full_us - (float)joint->neutral_us) * (reference - 90) / 90;
}

static float pulse_to_reference(const joint_pwm_t *joint, float pulse) {
    const float zero = joint->at_zero_us, full = joint->at_full_us, middle = joint->neutral_us;
    if ((full > zero && pulse <= middle) || (full < zero && pulse >= middle))
        return 90 * (pulse - zero) / (middle - zero);
    return 90 + 90 * (pulse - middle) / (full - middle);
}

bool bench_servo_named_reference(int gpio, int degrees, float *reference) {
    const joint_pwm_t *joint = find_joint(gpio);
    if (!joint || !reference || degrees < 0 || degrees > 180) return false;
    const int zero = joint->at_zero_us, full = joint->at_full_us;
    const int pulse_scaled = 500 * 180 + degrees * 2000;
    if (pulse_scaled < (zero < full ? zero : full) * 180 ||
        pulse_scaled > (zero > full ? zero : full) * 180) return false;
    *reference = pulse_to_reference(joint, pulse_scaled / 180.0f);
    return true;
}

static hardware_result_t write_channel(bench_servo_t *servos, unsigned index) {
    const bench_servo_channel_t *channel = &servos->channels[index];
    const joint_pwm_t *joint = find_joint(channel->gpio);
    if (!joint) return IO_INVALID;
    const float pulse = reference_to_pulse(joint, channel->current);
    if (servos->driver.write_precise_us)
        return servos->driver.write_precise_us(servos->driver.context, index, pulse);
    return servos->driver.write_us(servos->driver.context, index, (unsigned)(pulse + 0.5f));
}

static void fail(bench_servo_t *servos) {
    servos->fault = true;
    bench_servo_stop(servos);
}

bool bench_servo_init(bench_servo_t *servos, const bench_servo_driver_t *driver,
                      const bench_servo_config_t *config) {
    if (!servos || !config || servos->initialized) return false;
    if (config->max_dps == 0 || config->max_dps > MAX_DPS_LIMIT)
        return false;
    if (driver && (!driver->attach || !driver->write_us || !driver->shutdown)) return false;
    *servos = (bench_servo_t){.config = *config, .initialized = true};
    if (driver) servos->driver = *driver;
    return true;
}

static bench_servo_result_t set_target(bench_servo_t *servos, int gpio, float degrees) {
    if (!servos || !servos->initialized || !servos->driver.write_us) return BENCH_SERVO_DISABLED;
    if (servos->latched) return BENCH_SERVO_STOPPED;
    if (!(degrees >= 0 && degrees <= BENCH_SERVO_MAX_DEGREES)) return BENCH_SERVO_BAD_ANGLE;
    const bench_servo_config_t *config = &servos->config;
    if (!find_joint(gpio) || !gpio_policy_servo_allowed(gpio, config->start_pin, config->led_pin,
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

bench_servo_result_t bench_servo_set(bench_servo_t *servos, int gpio, float degrees) {
    if (servos && (servos->scan_pose_active || servos->base_active)) return BENCH_SERVO_STOPPED;
    bench_servo_result_t result = set_target(servos, gpio, degrees);
    if (result == BENCH_SERVO_OK) servos->distance_ready = false;
    return result;
}

typedef struct {
    int gpio, degrees;
    float max_us_per_s, max_us_per_s2;
} pose_joint_t;

static const pose_joint_t scan_pose[] = {
    {SERVO_GARRA_GPIO, 180, 250, 375},
    {SERVO_PUNHO_FLEX_GPIO, 140, 262.5f, 406.25f},
    {SERVO_COTOVELO_GPIO, 50, 112.5f, 187.5f},
    {SERVO_OMBRO_GPIO, 100, 150, 200},
};

static const pose_joint_t idle_pose[] = {
    {SERVO_GARRA_GPIO, 45, 200, 240},
    {SERVO_PUNHO_FLEX_GPIO, 81, 210, 260},
    {SERVO_COTOVELO_GPIO, 40, 90, 120},
    {SERVO_OMBRO_GPIO, 127, 60, 80},
    {SERVO_PUNHO_ROT_GPIO, 63, 210, 260},
};

static bool reapply_pose_end(bench_servo_t *servos) {
    const int pins[] = {SERVO_PUNHO_FLEX_GPIO, SERVO_GARRA_GPIO};
    for (unsigned j = 0; j < sizeof(pins) / sizeof(pins[0]); ++j) {
        for (unsigned i = 0; i < servos->used; ++i) {
            if (servos->channels[i].gpio != pins[j]) continue;
            if (write_channel(servos, i) != IO_OK) {
                fail(servos);
                return false;
            }
        }
    }
    return true;
}

static bool prepare_pose(bench_servo_t *servos, const pose_joint_t *pose, unsigned count,
                         bool measure_after_pose) {
    if (!bench_servo_enabled(servos) || servos->latched || servos->scan_pose_active || servos->base_active) return false;
    float pose_targets[BENCH_SERVO_MAX_CHANNELS];
    for (unsigned i = 0; i < count; ++i) {
        if (!bench_servo_named_reference(pose[i].gpio, pose[i].degrees, &pose_targets[i]) ||
            !gpio_policy_servo_allowed(pose[i].gpio, servos->config.start_pin,
                servos->config.led_pin, servos->config.estop_pin, servos->config.base_pin))
            return false;
    }
    for (unsigned i = 0; i < servos->used; ++i)
        if (servos->channels[i].current != servos->channels[i].target) return false;
    servos->distance_ready = false;
    servos->measure_after_pose = measure_after_pose;
    /* A new channel starts at its configured neutral pulse, without position feedback. */
    for (unsigned i = 0; i < count; ++i) {
        unsigned channel = 0;
        while (channel < servos->used && servos->channels[channel].gpio != pose[i].gpio)
            ++channel;
        if (channel == servos->used && set_target(servos, pose[i].gpio, 90) != BENCH_SERVO_OK)
            return false;
    }
    float targets[BENCH_SERVO_MAX_CHANNELS];
    for (unsigned i = 0; i < servos->used; ++i) {
        if (servos->channels[i].current != servos->channels[i].target) return false;
        targets[i] = servos->channels[i].current;
    }
    for (unsigned i = 0; i < count; ++i) {
        unsigned channel = 0;
        while (channel < servos->used && servos->channels[channel].gpio != pose[i].gpio)
            ++channel;
        targets[channel] = pose_targets[i];
    }
    servos->scan_elapsed_ms = 0;
    servos->scan_pose_active = false;
    for (unsigned i = 0; i < servos->used; ++i) {
        const joint_pwm_t *joint = find_joint(servos->channels[i].gpio);
        servos->scan_origin[i] = reference_to_pulse(joint, servos->channels[i].current);
        servos->channels[i].target = targets[i];
        servos->scan_duration_ms[i] = 0;
        servos->scan_start_ms[i] = 0;
        for (unsigned j = 0; j < count; ++j) {
            if (servos->channels[i].gpio != pose[j].gpio) continue;
            const float distance = fabsf(reference_to_pulse(joint, targets[i]) - servos->scan_origin[i]);
            const float lower_span = fabsf((float)joint->neutral_us - joint->at_zero_us);
            const float upper_span = fabsf((float)joint->at_full_us - joint->neutral_us);
            const float span = lower_span < upper_span ? lower_span : upper_span;
            const float configured_rate = servos->config.max_dps * span / 90;
            const float rate = configured_rate < pose[j].max_us_per_s
                ? configured_rate : pose[j].max_us_per_s;
            /* Quintic velocity ramp: peak acceleration is 1.875 * speed / duration.
             * Acceleration and jerk reach zero at both ends of each ramp. */
            const float acceleration = pose[j].max_us_per_s2;
            const float triangular_rate = sqrtf(distance * acceleration / 1.875f);
            const float peak_rate = rate < triangular_rate ? rate : triangular_rate;
            const float ramp_seconds = 1.875f * peak_rate / acceleration;
            servos->scan_speed_us_per_s[i] = peak_rate;
            servos->scan_ramp_ms[i] = 1000 * ramp_seconds;
            servos->scan_duration_ms[i] = peak_rate > 0
                ? 1000 * (distance / peak_rate + ramp_seconds) : 0;
            if (distance > 0) servos->scan_pose_active = true;
        }
    }
    /* Gripper can settle alongside the wrist. Proximal joints wait their turn. */
    float start_ms = 0;
    for (unsigned j = 1; j < count; ++j) {
        for (unsigned i = 0; i < servos->used; ++i) {
            if (servos->channels[i].gpio != pose[j].gpio) continue;
            servos->scan_start_ms[i] = start_ms;
            if (servos->channels[i].current != servos->channels[i].target)
                start_ms += servos->scan_duration_ms[i];
        }
    }
    if (servos->scan_pose_active) return true;
    bool completed = reapply_pose_end(servos);
    servos->distance_ready = completed && measure_after_pose;
    return completed;
}

bool bench_servo_prepare_scan(bench_servo_t *servos) {
    bool accepted = prepare_pose(servos, scan_pose, sizeof(scan_pose) / sizeof(scan_pose[0]), true);
    if (accepted) ++servos->scan_request_id;
    return accepted;
}

bool bench_servo_idle_pose(bench_servo_t *servos) {
    return prepare_pose(servos, idle_pose, sizeof(idle_pose) / sizeof(idle_pose[0]), false);
}

static float ramp_distance(float speed, float duration, float time) {
    const float u = time / duration;
    const float u2 = u * u;
    return speed * duration * u2 * u2 * (2.5f + u * (u - 3));
}

void bench_servo_poll(bench_servo_t *servos, int64_t now_ms) {
    if (!servos || !servos->initialized) return;
    int64_t elapsed_ms = now_ms - servos->last_ms;
    servos->last_ms = now_ms;
    if (servos->latched) return;
    if (servos->driver.poll && servos->driver.poll(servos->driver.context) != IO_OK) {
        fail(servos);
        return;
    }
    if (elapsed_ms <= 0) return;
    float step = servos->config.max_dps * (float)elapsed_ms / 1000.0f;
    const bool synchronized = servos->scan_pose_active;
    bool pending = false;
    if (synchronized)
        servos->scan_elapsed_ms += elapsed_ms < 20 ? elapsed_ms : 20;
    for (unsigned i = 0; i < servos->used; ++i) {
        bench_servo_channel_t *channel = &servos->channels[i];
        float delta = channel->target - channel->current;
        if (delta == 0.0f) continue;
        if (synchronized) {
            const float elapsed = servos->scan_elapsed_ms - servos->scan_start_ms[i];
            if (elapsed <= 0) {
                pending = true;
                continue;
            }
            const float duration = servos->scan_duration_ms[i];
            const joint_pwm_t *joint = find_joint(channel->gpio);
            const float origin = servos->scan_origin[i];
            const float target_pulse = reference_to_pulse(joint, channel->target);
            if (elapsed >= duration) {
                channel->current = channel->target;
            } else {
                pending = true;
                const float time = elapsed / 1000;
                const float total = duration / 1000;
                const float ramp = servos->scan_ramp_ms[i] / 1000;
                const float speed = servos->scan_speed_us_per_s[i];
                float distance;
                if (time < ramp) {
                    distance = ramp_distance(speed, ramp, time);
                } else if (time > total - ramp) {
                    const float remaining = total - time;
                    distance = fabsf(target_pulse - origin) - ramp_distance(speed, ramp, remaining);
                } else {
                    distance = speed * (time - ramp / 2);
                }
                const float pulse = origin + (target_pulse > origin ? distance : -distance);
                channel->current = pulse_to_reference(joint, pulse);
            }
        } else {
            channel->current = delta > step ? channel->current + step
                         : delta < -step ? channel->current - step : channel->target;
        }
        if (write_channel(servos, i) != IO_OK) {
            fail(servos);
            return;
        }
    }
    if (synchronized) {
        servos->scan_pose_active = pending;
        if (!pending) {
            bool completed = reapply_pose_end(servos);
            servos->distance_ready = completed && servos->measure_after_pose;
        }
    }
}

void bench_servo_stop(bench_servo_t *servos) {
    if (!servos || !servos->initialized) return;
    servos->latched = true;
    servos->scan_pose_active = false;
    servos->distance_ready = false;
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
