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
    unsigned channel_writes[BENCH_SERVO_MAX_CHANNELS];
    unsigned last_channel, previous_channel;
    float precise_pulse;
    unsigned precise_writes;
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
    ++m->channel_writes[channel];
    m->previous_channel = m->last_channel;
    m->last_channel = channel;
    if (m->write_result != IO_OK) return m->write_result;
    m->pulse[channel] = pulse_us;
    return IO_OK;
}
static hardware_result_t write_precise(void *context, unsigned channel, float pulse) {
    mock_t *mock = context;
    mock->precise_pulse = pulse;
    ++mock->precise_writes;
    return write_us(context, channel, (unsigned)(pulse + 0.5f));
}
static hardware_result_t driver_poll(void *context) {
    return ((mock_t *)context)->attach_result;
}
static hardware_result_t shutdown(void *c) {
    mock_t *m = c;
    ++m->shutdowns;
    return m->shutdown_result;
}

static const bench_servo_config_t config = {
    .max_dps = 90,
    .start_pin = 7, .led_pin = 48, .estop_pin = 6, .base_pin = 15
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
    bad.max_dps = 0; assert(!bench_servo_init(&s, NULL, &bad));
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
    assert(bench_servo_set(&s, 16, -1) == BENCH_SERVO_BAD_ANGLE);
    assert(bench_servo_set(&s, 16, 181) == BENCH_SERVO_BAD_ANGLE);
    const int refused[] = {-1, 0, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15, 19, 20, 26, 30, 37, 43, 45, 46, 48, 49, 99};
    for (unsigned i = 0; i < sizeof(refused) / sizeof(refused[0]); ++i)
        assert(bench_servo_set(&s, refused[i], 90) == BENCH_SERVO_BAD_GPIO);
    assert(m.attaches == 0 && m.writes == 0 && bench_servo_channels(&s) == 0);

    /* First command jumps to the target: 0, 90 and 180 degrees map onto the pulse range. */
    assert(bench_servo_set(&s, 16, 0) == BENCH_SERVO_OK && m.pulse[0] == 500 && m.pins[0] == 16);
    assert(bench_servo_set(&s, 17, 90) == BENCH_SERVO_OK && m.pulse[1] == 939);
    assert(bench_servo_set(&s, 18, 180) == BENCH_SERVO_OK && m.pulse[2] == 2500);
    assert(bench_servo_channels(&s) == 3 && m.writes == 3);

    /* Later commands on a known pin are slew limited and non-blocking. */
    assert(bench_servo_set(&s, 16, 180) == BENCH_SERVO_OK);
    assert(bench_servo_channels(&s) == 3 && m.writes == 3 && m.pulse[0] == 500);
    bench_servo_poll(&s, 0);
    for (int t = 10; t <= 1000; t += 10) bench_servo_poll(&s, t);
    assert(m.pulse[0] >= 1915 && m.pulse[0] <= 1917);      /* 90 deg after 1 s at 90 deg/s */
    for (int t = 1010; t <= 2000; t += 10) bench_servo_poll(&s, t);
    assert(m.pulse[0] == 2500);                              /* arrives, never overshoots */
    bench_servo_poll(&s, 2100);                              /* float accumulation settles on target */
    unsigned writes = m.writes;
    bench_servo_poll(&s, 2110);
    assert(m.writes == writes);                              /* idle servo is not rewritten */
    assert(bench_servo_set(&s, 16, 0) == BENCH_SERVO_OK);
    bench_servo_poll(&s, 2110);                              /* no time elapsed: no motion */
    bench_servo_poll(&s, 2105);                              /* clock going backwards: no motion */
    assert(m.pulse[0] == 2500 && !bench_servo_faulted(&s));
    bench_servo_poll(&s, 5000);
    assert(m.pulse[0] == 500);

    /* Channel table is bounded; reusing a pin does not consume a channel. */
    assert(bench_servo_set(&s, 8, 45) == BENCH_SERVO_OK && bench_servo_set(&s, 9, 45) == BENCH_SERVO_OK);
    assert(bench_servo_channels(&s) == BENCH_SERVO_MAX_CHANNELS);
    assert(bench_servo_set(&s, 21, 45) == BENCH_SERVO_BAD_GPIO);
    assert(bench_servo_set(&s, 9, 90) == BENCH_SERVO_OK);
    assert(bench_servo_channels(&s) == BENCH_SERVO_MAX_CHANNELS);

    /* Stop latches, refuses commands and repeats shutdown until the backend confirms. */
    m.shutdown_result = IO_FAULT;
    bench_servo_stop(&s);
    bench_servo_stop(&s);
    assert(s.latched && !s.shutdown_confirmed && m.shutdowns == 2);
    assert(bench_servo_set(&s, 16, 10) == BENCH_SERVO_STOPPED);
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
    assert(bench_servo_set(&s, 16, 90) == BENCH_SERVO_FAULT);
    assert(bench_servo_faulted(&s) && s.latched && m.shutdowns == 1 && bench_servo_channels(&s) == 0);
    assert(bench_servo_set(&s, 16, 90) == BENCH_SERVO_STOPPED);
    make(&s, &m);
    m.write_result = IO_FAULT;
    assert(bench_servo_set(&s, 16, 90) == BENCH_SERVO_FAULT);
    assert(bench_servo_faulted(&s) && s.latched && m.shutdowns == 1);
    make(&s, &m);
    assert(bench_servo_set(&s, 16, 0) == BENCH_SERVO_OK && bench_servo_set(&s, 16, 180) == BENCH_SERVO_OK);
    m.write_result = IO_FAULT;
    bench_servo_poll(&s, 0);
    bench_servo_poll(&s, 100);
    assert(bench_servo_faulted(&s) && s.latched && m.shutdowns == 1);

    make(&s, &m);
    assert(bench_servo_prepare_scan(&s) && m.attaches == 4 && s.scan_pose_active);
    assert(m.pulse[0] == 1000 && m.pulse[1] == 1400 &&
           m.pulse[2] == 939 && m.pulse[3] == 1916);
    bench_servo_stop(&s);
    assert(!s.scan_pose_active && !s.distance_ready && !bench_servo_prepare_scan(&s));
    make(&s, &m);
    m.attach_result = IO_FAULT;
    assert(!bench_servo_prepare_scan(&s) && s.latched && s.fault);
    make(&s, &m);
    float reference;
    assert(bench_servo_named_reference(8, 145, &reference));
    assert(bench_servo_named_reference(8, 146, &reference));
    assert(bench_servo_named_reference(8, 0, &reference) && reference == 0);
    assert(bench_servo_named_reference(8, 180, &reference) && reference == 180);
    assert(!bench_servo_named_reference(8, -1, &reference));
    assert(!bench_servo_named_reference(8, 181, &reference));
    static const int joints[] = {9, 8, 17, 16};
    static const unsigned expected[] = {2500, 2056, 1056, 1611};
    for (unsigned i = 0; i < 4; ++i)
        assert(bench_servo_set(&s, joints[i], 90) == BENCH_SERVO_OK);
    assert(bench_servo_prepare_scan(&s));
    assert(s.scan_start_ms[3] + s.scan_duration_ms[3] < 25000);
    assert(bench_servo_set(&s, 9, 0) == BENCH_SERVO_STOPPED);
    bench_servo_poll(&s, 10);
    assert(m.writes == 6);
    int sampled_pulse[4], sampled_delta[4] = {0};
    for (unsigned i = 0; i < 4; ++i) sampled_pulse[i] = m.pulse[i];
    for (int t = 20; s.scan_pose_active && t <= 60000; t += 10) {
        float previous[4];
        for (unsigned i = 0; i < 4; ++i) previous[i] = s.channels[i].current;
        bench_servo_poll(&s, t);
        if (t <= s.scan_duration_ms[1]) assert(s.channels[2].current == 90);
        if (t <= s.scan_duration_ms[1] + s.scan_duration_ms[2])
            assert(s.channels[3].current == 90);
        if (s.channels[2].current != 90)
            assert(s.channels[1].current == s.channels[1].target);
        if (s.channels[3].current != 90)
            assert(s.channels[2].current == s.channels[2].target);
        if (t <= 100) {
            const int initial_pulses[] = {1000, 1400, 939, 1916};
            for (unsigned i = 0; i < 4; ++i) {
                int displacement = (int)m.pulse[i] - initial_pulses[i];
                assert(displacement >= -2 && displacement <= 2);
            }
        }
        if (t % 100 == 0) {
            const float rates[] = {250, 262.5f, 112.5f, 150};
            const float accelerations[] = {375, 406.25f, 187.5f, 200};
            for (unsigned i = 0; i < 4; ++i) {
                int delta = (int)m.pulse[i] - sampled_pulse[i];
                assert(delta <= rates[i] * 0.1f + 1);
                assert(-delta <= rates[i] * 0.1f + 1);
                if (t >= 200) {
                    int change = delta - sampled_delta[i];
                    assert(change <= accelerations[i] * 0.01f + 2);
                    assert(-change <= accelerations[i] * 0.01f + 2);
                }
                sampled_delta[i] = delta;
                sampled_pulse[i] = m.pulse[i];
            }
        }
        for (unsigned i = 0; i < 4; ++i) {
            const float delta = s.channels[i].current - previous[i];
            assert(delta <= config.max_dps * 0.01f + 0.0001f);
            assert(-delta <= config.max_dps * 0.01f + 0.0001f);
        }

    }
    assert(!s.scan_pose_active && s.distance_ready && m.previous_channel == 1 && m.last_channel == 0);
    assert(m.channel_writes[0] > 150);
    for (unsigned i = 0; i < 4; ++i) assert(m.pulse[i] == expected[i]);
    unsigned finished_writes = m.writes;
    bench_servo_poll(&s, 60000);
    assert(m.writes == finished_writes);
    assert(bench_servo_prepare_scan(&s) && !s.scan_pose_active);
    assert(m.writes == finished_writes + 2 && m.last_channel == 0);
    assert(bench_servo_set(&s, 9, 90) == BENCH_SERVO_OK);
    bench_servo_poll(&s, 61000);
    assert(bench_servo_prepare_scan(&s) && s.scan_pose_active);
    bench_servo_stop(&s);
    assert(!s.scan_pose_active && !s.distance_ready && !bench_servo_prepare_scan(&s));

    make(&s, &m);
    for (unsigned i = 0; i < 4; ++i) {
        const int pose[] = {180, 140, 50, 100};
        assert(bench_servo_named_reference(joints[i], pose[i], &reference));
        assert(bench_servo_set(&s, joints[i], reference) == BENCH_SERVO_OK);
    }
    m.write_result = IO_FAULT;
    assert(!bench_servo_prepare_scan(&s) && s.latched && s.fault && m.shutdowns == 1);

    /* Short wrist motion exercises a profile with acceleration and deceleration only. */
    make(&s, &m);
    for (unsigned i = 0; i < 4; ++i) {
        const int pose[] = {180, 139, 50, 100};
        assert(bench_servo_named_reference(joints[i], pose[i], &reference));
        assert(bench_servo_set(&s, joints[i], reference) == BENCH_SERVO_OK);
    }
    assert(bench_servo_prepare_scan(&s));
    assert(s.scan_duration_ms[1] < 1000);
    for (int t = 10; t <= 1000; t += 10) bench_servo_poll(&s, t);
    assert(!s.scan_pose_active && !s.fault && m.pulse[1] == 2056);

    make(&s, &m);
    assert(bench_servo_idle_pose(&s) && s.used == 5);
    assert(!bench_servo_idle_pose(&s));
    for (int t = 10; t <= 60000; t += 10) bench_servo_poll(&s, t);
    assert(!s.scan_pose_active && !s.distance_ready && !s.fault);
    const unsigned idle_pulses[] = {1000, 1400, 944, 1911, 1200};
    for (unsigned i = 0; i < 5; ++i) assert(m.pulse[i] == idle_pulses[i]);
    bench_servo_stop(&s);
    assert(!bench_servo_idle_pose(&s));
    assert(!bench_servo_idle_pose(NULL));

    make(&s, &m);
    s.base_active = true;
    assert(bench_servo_set(&s, 16, 90) == BENCH_SERVO_STOPPED);
    assert(!bench_servo_prepare_scan(&s) && !bench_servo_idle_pose(&s));
    assert(m.writes == 0);

    const int full_range_pins[] = {16, 17, 18, 8, 9};
    for (unsigned i = 0; i < 5; ++i) {
        make(&s, &m);
        const int gpio = full_range_pins[i];
        assert(bench_servo_named_reference(gpio, 0, &reference));
        assert(bench_servo_set(&s, gpio, reference) == BENCH_SERVO_OK && m.pulse[0] == 500);
        assert(bench_servo_named_reference(gpio, 180, &reference));
        assert(bench_servo_set(&s, gpio, reference) == BENCH_SERVO_OK);
        for (int t = 10; t <= 2100; t += 10) bench_servo_poll(&s, t);
        assert(m.pulse[0] == 2500);
        assert(!bench_servo_named_reference(gpio, -1, &reference));
        assert(!bench_servo_named_reference(gpio, 181, &reference));
    }

    s = (bench_servo_t){0};
    m = (mock_t){0};
    bench_servo_driver_t precise_driver = {
        .attach = attach, .write_us = write_us, .shutdown = shutdown,
        .context = &m, .write_precise_us = write_precise, .poll = driver_poll
    };
    assert(bench_servo_init(&s, &precise_driver, &config));
    assert(bench_servo_named_reference(8, 140, &reference));
    assert(bench_servo_set(&s, 8, reference) == BENCH_SERVO_OK);
    assert(m.precise_writes == 1);
    assert(m.precise_pulse > 2055.5f && m.precise_pulse < 2055.6f);
    m.write_result = IO_FAULT;
    assert(bench_servo_set(&s, 8, 90) == BENCH_SERVO_OK);
    bench_servo_poll(&s, 20);
    assert(s.fault && s.latched && m.shutdowns == 1);

    s = (bench_servo_t){0};
    m = (mock_t){0};
    assert(bench_servo_init(&s, &precise_driver, &config));
    assert(bench_servo_set(&s, 8, 90) == BENCH_SERVO_OK);
    m.attach_result = IO_FAULT;
    bench_servo_poll(&s, 10);
    assert(s.fault && s.latched && m.shutdowns == 1);

    for (unsigned i = 0; i <= BENCH_SERVO_FAULT; ++i)
        assert(bench_servo_result_name((bench_servo_result_t)i)[0] != '\0');
    puts("PASS: bench servo limits, pin policy, slew, channel table, stop latch, driver faults");
}
