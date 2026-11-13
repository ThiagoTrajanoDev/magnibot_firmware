#ifdef NDEBUG
#error "tests rely on assert() side effects; do not build with NDEBUG"
#endif
#include <assert.h>
#include <stdio.h>
#include "bench_servo.h"

/* Mock PWM backend: records the last pulse per channel; failures are injected by the test. */
typedef struct {
    int pins[BENCH_SERVO_MAX_CHANNELS];
    unsigned pulse[BENCH_SERVO_MAX_CHANNELS];
    unsigned attaches, writes, shutdowns;
    hardware_result_t attach_result, write_result, shutdown_result;
} mock_t;

static hardware_result_t attach(void *c, int gpio, unsigned channel) {
    mock_t *m = c;
    assert(channel == m->attaches);
    if (m->attach_result != IO_OK) return m->attach_result;
    m->pins[channel] = gpio;
    ++m->attaches;
    return IO_OK;
}
static hardware_result_t write_us(void *c, unsigned channel, unsigned pulse_us) {
    mock_t *m = c;
    assert(channel < m->attaches);
    ++m->writes;
    if (m->write_result != IO_OK) return m->write_result;
    m->pulse[channel] = pulse_us;
    return IO_OK;
}
static hardware_result_t shutdown(void *c) {
    mock_t *m = c;
    ++m->shutdowns;
    return m->shutdown_result;
}

static const bench_servo_config_t config = {
    .min_us = 600, .max_us = 2400, .max_dps = 90,
    .start_pin = 15, .led_pin = 48, .estop_pin = 17, .base_pin = 4
};

static void make(bench_servo_t *s, mock_t *m) {
    *s = (bench_servo_t){0};
    *m = (mock_t){0};
    bench_servo_driver_t driver = {.attach = attach, .write_us = write_us, .shutdown = shutdown, .context = m};
    assert(bench_servo_init(s, &driver, &config));
}

int main(void) {
    bench_servo_t s;
    mock_t m;

    /* Init validation and disabled backend. */
    s = (bench_servo_t){0};
    bench_servo_driver_t partial = {.attach = attach};
    assert(!bench_servo_init(NULL, NULL, &config) && !bench_servo_init(&s, NULL, NULL));
    assert(!bench_servo_init(&s, &partial, &config));
    bench_servo_config_t bad = config;
    bad.min_us = 300; assert(!bench_servo_init(&s, NULL, &bad));
    bad = config; bad.max_us = 2700; assert(!bench_servo_init(&s, NULL, &bad));
    bad = config; bad.min_us = 2400; assert(!bench_servo_init(&s, NULL, &bad));
    bad = config; bad.max_dps = 0; assert(!bench_servo_init(&s, NULL, &bad));
    bad = config; bad.max_dps = 361; assert(!bench_servo_init(&s, NULL, &bad));
    assert(bench_servo_init(&s, NULL, &config) && !bench_servo_init(&s, NULL, &config));
    assert(!bench_servo_enabled(&s));
    assert(bench_servo_set(&s, 5, 90) == BENCH_SERVO_DISABLED);
    bench_servo_stop(&s);
    assert(s.latched && s.shutdown_confirmed);
    assert(bench_servo_set(NULL, 5, 90) == BENCH_SERVO_DISABLED);
    bench_servo_poll(NULL, 0); bench_servo_stop(NULL);
    assert(!bench_servo_faulted(NULL) && bench_servo_channels(NULL) == 0);

    /* Range and pin policy are checked before the backend is touched. */
    make(&s, &m);
    assert(bench_servo_enabled(&s));
    assert(bench_servo_set(&s, 5, -1) == BENCH_SERVO_BAD_ANGLE);
    assert(bench_servo_set(&s, 5, 181) == BENCH_SERVO_BAD_ANGLE);
    const int refused[] = {-1, 0, 3, 4, 15, 17, 19, 20, 26, 30, 37, 43, 45, 46, 48, 49, 99};
    for (unsigned i = 0; i < sizeof(refused) / sizeof(refused[0]); ++i)
        assert(bench_servo_set(&s, refused[i], 90) == BENCH_SERVO_BAD_GPIO);
    assert(m.attaches == 0 && m.writes == 0 && bench_servo_channels(&s) == 0);

    /* First command jumps to the target: 0, 90 and 180 degrees map onto the pulse range. */
    assert(bench_servo_set(&s, 5, 0) == BENCH_SERVO_OK && m.pulse[0] == 600 && m.pins[0] == 5);
    assert(bench_servo_set(&s, 6, 90) == BENCH_SERVO_OK && m.pulse[1] == 1500);
    assert(bench_servo_set(&s, 7, 180) == BENCH_SERVO_OK && m.pulse[2] == 2400);
    assert(bench_servo_channels(&s) == 3 && m.writes == 3);

    /* Later commands on a known pin are slew limited and non-blocking. */
    assert(bench_servo_set(&s, 5, 180) == BENCH_SERVO_OK);
    assert(bench_servo_channels(&s) == 3 && m.writes == 3 && m.pulse[0] == 600);
    bench_servo_poll(&s, 0);
    for (int t = 10; t <= 1000; t += 10) bench_servo_poll(&s, t);
    assert(m.pulse[0] >= 1499 && m.pulse[0] <= 1501);      /* 90 deg after 1 s at 90 deg/s */
    for (int t = 1010; t <= 2000; t += 10) bench_servo_poll(&s, t);
    assert(m.pulse[0] == 2400);                              /* arrives, never overshoots */
    bench_servo_poll(&s, 2100);                              /* float accumulation settles on target */
    unsigned writes = m.writes;
    bench_servo_poll(&s, 2110);
    assert(m.writes == writes);                              /* idle servo is not rewritten */
    assert(bench_servo_set(&s, 5, 0) == BENCH_SERVO_OK);
    bench_servo_poll(&s, 2110);                              /* no time elapsed: no motion */
    bench_servo_poll(&s, 2105);                              /* clock going backwards: no motion */
    assert(m.pulse[0] == 2400 && !bench_servo_faulted(&s));
    bench_servo_poll(&s, 5000);
    assert(m.pulse[0] == 600);

    /* Channel table is bounded; reusing a pin does not consume a channel. */
    assert(bench_servo_set(&s, 10, 45) == BENCH_SERVO_OK && bench_servo_set(&s, 11, 45) == BENCH_SERVO_OK);
    assert(bench_servo_channels(&s) == BENCH_SERVO_MAX_CHANNELS);
    assert(bench_servo_set(&s, 12, 45) == BENCH_SERVO_NO_CHANNEL);
    assert(bench_servo_set(&s, 11, 90) == BENCH_SERVO_OK);
    assert(bench_servo_channels(&s) == BENCH_SERVO_MAX_CHANNELS);

    /* Stop latches, refuses commands and repeats shutdown until the backend confirms. */
    m.shutdown_result = IO_FAULT;
    bench_servo_stop(&s);
    bench_servo_stop(&s);
    assert(s.latched && !s.shutdown_confirmed && m.shutdowns == 2);
    assert(bench_servo_set(&s, 5, 10) == BENCH_SERVO_STOPPED);
    unsigned moved = m.writes;
    bench_servo_poll(&s, 9000);
    assert(m.writes == moved);
    m.shutdown_result = IO_OK;
    bench_servo_stop(&s);
    bench_servo_stop(&s);
    assert(s.shutdown_confirmed && m.shutdowns == 3);

    /* Backend failures latch and raise the fault flag (attach, first write, slew write). */
    make(&s, &m);
    m.attach_result = IO_FAULT;
    assert(bench_servo_set(&s, 5, 90) == BENCH_SERVO_FAULT);
    assert(bench_servo_faulted(&s) && s.latched && m.shutdowns == 1 && bench_servo_channels(&s) == 0);
    assert(bench_servo_set(&s, 5, 90) == BENCH_SERVO_STOPPED);
    make(&s, &m);
    m.write_result = IO_FAULT;
    assert(bench_servo_set(&s, 5, 90) == BENCH_SERVO_FAULT);
    assert(bench_servo_faulted(&s) && s.latched && m.shutdowns == 1);
    make(&s, &m);
    assert(bench_servo_set(&s, 5, 0) == BENCH_SERVO_OK && bench_servo_set(&s, 5, 180) == BENCH_SERVO_OK);
    m.write_result = IO_FAULT;
    bench_servo_poll(&s, 0);
    bench_servo_poll(&s, 100);
    assert(bench_servo_faulted(&s) && s.latched && m.shutdowns == 1);

    for (unsigned i = 0; i <= BENCH_SERVO_FAULT; ++i)
        assert(bench_servo_result_name((bench_servo_result_t)i)[0] != '\0');
    puts("PASS: bench servo limits, pin policy, slew, channel table, stop latch, driver faults");
}
