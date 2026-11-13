#ifndef BENCH_SERVO_H
#define BENCH_SERVO_H
#include <stdbool.h>
#include <stdint.h>
#include "hardware_io.h"

#define BENCH_SERVO_MAX_CHANNELS 5
#define BENCH_SERVO_MAX_DEGREES 180

typedef enum {
    BENCH_SERVO_OK, BENCH_SERVO_DISABLED, BENCH_SERVO_BAD_GPIO, BENCH_SERVO_BAD_ANGLE,
    BENCH_SERVO_NO_CHANNEL, BENCH_SERVO_STOPPED, BENCH_SERVO_FAULT
} bench_servo_result_t;

/* PWM backend. A channel index is allocated by the module; shutdown stops every channel. */
typedef struct {
    hardware_result_t (*attach)(void *context, int gpio, unsigned channel);
    hardware_result_t (*write_us)(void *context, unsigned channel, unsigned pulse_us);
    hardware_result_t (*shutdown)(void *context);
    void *context;
} bench_servo_driver_t;

typedef struct {
    uint16_t min_us, max_us;  /* pulse width at 0 and at 180 degrees */
    uint16_t max_dps;         /* slew limit, degrees per second */
    int start_pin, led_pin, estop_pin, base_pin;  /* pins a servo may never use */
} bench_servo_config_t;

typedef struct {
    int gpio;
    float current, target;  /* commanded references, never a measured position */
} bench_servo_channel_t;

typedef struct {
    bench_servo_driver_t driver;
    bench_servo_config_t config;
    bench_servo_channel_t channels[BENCH_SERVO_MAX_CHANNELS];
    unsigned used;
    int64_t last_ms;
    bool initialized, latched, fault, shutdown_confirmed;
} bench_servo_t;

/* NULL driver = disabled: every command is refused. Refuses a second init and bad config. */
bool bench_servo_init(bench_servo_t *servos, const bench_servo_driver_t *driver,
                      const bench_servo_config_t *config);
/* Sets the target (0..180) of the servo on `gpio`, claiming a channel on first use.
 * The first command of a pin jumps to the target; later ones are slew limited by poll. */
bench_servo_result_t bench_servo_set(bench_servo_t *servos, int gpio, int degrees);
/* Non-blocking slew; call every loop. A driver failure latches and sets the fault flag. */
void bench_servo_poll(bench_servo_t *servos, int64_t now_ms);
/* Latches (no further commands) and stops the PWM; repeats shutdown until the driver confirms. */
void bench_servo_stop(bench_servo_t *servos);
bool bench_servo_enabled(const bench_servo_t *servos);
bool bench_servo_faulted(const bench_servo_t *servos);
unsigned bench_servo_channels(const bench_servo_t *servos);
const char *bench_servo_result_name(bench_servo_result_t result);
#endif
