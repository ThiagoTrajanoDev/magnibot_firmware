#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "pickup_sequence.h"
#include "config.h"

static bench_servo_t servos;
static base_scan_t base;
static pickup_sequence_t pickup;
static int64_t now;
static unsigned writes, joint_stops, base_pulse;
static bool fail_joint;
static hardware_result_t attach(void *c, int gpio, unsigned index) {
    (void)c; (void)gpio; (void)index; return IO_OK;
}
static hardware_result_t write_joint(void *c, unsigned index, unsigned value) {
    (void)c; (void)index; (void)value; ++writes; return fail_joint ? IO_FAULT : IO_OK;
}
static hardware_result_t stop_joints(void *c) { (void)c; ++joint_stops; return IO_OK; }
static hardware_result_t write_base(void *c, unsigned value) { (void)c; base_pulse=value; return IO_OK; }
static hardware_result_t stop_base(void *c) { (void)c; return IO_OK; }
static const pickup_plan_t plan = {
    .shoulder_floor_deg=90, .elbow_limit_deg=0, .wrist_flex_deg=81, .lift_delta_deg=20,
    .approach_us_per_s=33, .approach_us_per_s2=60, .phase_timeout_ms=60000, .sensor_max_age_ms=200
};
static float angle(int gpio) {
    float value;
    assert(bench_servo_named_angle(&servos,gpio,&value));
    return value;
}
static sensor_sample_t sample(float mm) {
    return (sensor_sample_t){.value=mm,.timestamp_ms=now,.received=true,.valid=true,.source=SENSOR_HARDWARE};
}
static void tick(float mm, hardware_result_t io) {
    now+=10;
    bench_servo_poll(&servos,now);
    sensor_sample_t s=sample(mm);
    base_scan_poll(&base,now,&s,io);
    servos.base_active=base.moving;
    pickup_sequence_poll(&pickup,now,&s,io);
    servos.base_active=base.moving;
}
static void fixture(void) {
    servos=(bench_servo_t){0}; pickup=(pickup_sequence_t){0}; now=0;
    writes=joint_stops=0; fail_joint=false;
    bench_servo_config_t config={.max_dps=90,.start_pin=7,.led_pin=48,.estop_pin=6,.base_pin=15};
    bench_servo_driver_t joints={.attach=attach,.write_us=write_joint,.shutdown=stop_joints};
    base_scan_driver_t driver={.write_us=write_base,.shutdown=stop_base};
    assert(bench_servo_init(&servos,&joints,&config)); base_scan_init(&base,&driver);
    assert(bench_servo_prepare_scan(&servos));
    for (unsigned i=0;i<20000 && servos.scan_pose_active;++i) tick(156,IO_OK);
    assert(!servos.scan_pose_active);
    assert(fabsf(angle(SERVO_OMBRO_GPIO)-118)<.001f);
    assert(fabsf(angle(SERVO_COTOVELO_GPIO)-65)<.001f);
    assert(base_scan_begin(&base,now));
    for (unsigned i=0;i<30;++i) tick(156,IO_OK);
    tick(100,IO_OK); assert(base.found);
    while (base.moving) tick(156,IO_OK);
    assert(base_scan_replay(&base,now));
    while (base.moving) tick(156,IO_OK);
    assert(base.at_object && base_pulse==1488);
    assert(pickup_sequence_init(&pickup,&servos,&base,&plan));
}
static void approach(void) {
    assert(pickup_sequence_start(&pickup,now));
    for (unsigned i=0;i<10000 && pickup.phase!=PICKUP_APPROACH;++i) {
        tick(100,IO_OK);
        assert(pickup.phase!=PICKUP_FAILED);
    }
    assert(pickup.phase==PICKUP_APPROACH);
    for (unsigned i=0;i<200;++i) tick(100,IO_OK);
    assert(angle(SERVO_OMBRO_GPIO)<118 && angle(SERVO_OMBRO_GPIO)>90);
    assert(angle(SERVO_COTOVELO_GPIO)<65 && angle(SERVO_COTOVELO_GPIO)>0);
    assert(angle(SERVO_PUNHO_FLEX_GPIO)<140 && angle(SERVO_PUNHO_FLEX_GPIO)>81);
    float progress=(118-angle(SERVO_OMBRO_GPIO))/28;
    assert(fabsf(progress-(65-angle(SERVO_COTOVELO_GPIO))/65)<.0001f);
    assert(fabsf(progress-(140-angle(SERVO_PUNHO_FLEX_GPIO))/59)<.0001f);
}
int main(void) {
    fixture(); approach();
    tick(61,IO_OK); assert(pickup.phase==PICKUP_APPROACH);
    tick(60,IO_OK); assert(pickup.phase==PICKUP_VERIFY && !servos.scan_pose_active);
    float grasp_shoulder=angle(SERVO_OMBRO_GPIO), grasp_elbow=angle(SERVO_COTOVELO_GPIO);
    unsigned before=writes;
    sensor_sample_t same=sample(60);
    pickup_sequence_poll(&pickup,now,&same,IO_OK);
    pickup_sequence_poll(&pickup,now,&same,IO_OK);
    assert(pickup.close_samples==1 && writes==before);
    tick(61,IO_OK); assert(pickup.close_samples==0);
    tick(60,IO_OK); tick(60,IO_OK); assert(pickup.phase==PICKUP_VERIFY);
    tick(60,IO_OK); assert(pickup.phase==PICKUP_CLOSE);
    bool saw_lift=false, saw_home=false, saw_open=false;
    for (unsigned i=0;i<20000 && pickup.phase!=PICKUP_DONE;++i) {
        tick(NAN,IO_DISABLED); /* Occluded ToF after closure does not release the object. */
        assert(pickup.phase!=PICKUP_FAILED);
        if (pickup.phase==PICKUP_LIFT) {
            saw_lift=true; assert(fabsf(angle(SERVO_GARRA_GPIO))<.001f && !base.moving);
        }
        if (pickup.phase==PICKUP_HOME) {
            saw_home=true;
            assert(fabsf(angle(SERVO_OMBRO_GPIO)-(grasp_shoulder+20))<.001f);
            assert(fabsf(angle(SERVO_GARRA_GPIO))<.001f);
            if (base.moving) assert(base_pulse==1403);
        }
        if (pickup.phase==PICKUP_OPEN) { saw_open=true; assert(base.home && base_pulse==1488); }
    }
    assert(saw_lift && saw_home && saw_open && pickup.phase==PICKUP_DONE);
    assert(fabsf(angle(SERVO_GARRA_GPIO)-70)<.001f && fabsf(angle(SERVO_COTOVELO_GPIO)-grasp_elbow)<.001f);
    assert(base.home && base.position_ms==0 && !servos.latched && joint_stops==0);
    assert(pickup_sequence_reset(&pickup) && pickup.phase==PICKUP_READY);

    /* Bad sensor readings stop the approach, hold PWM and never auto-resume or close. */
    const hardware_result_t bad[]={IO_FAULT,IO_DISABLED,IO_TIMEOUT,IO_UNAVAILABLE,IO_INVALID,IO_LATCHED};
    for (unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) {
        fixture(); approach(); tick(20,bad[i]);
        assert(pickup.phase==PICKUP_FAILED && pickup.fault==PICKUP_SENSOR);
        assert(servos.latched && base.latched && !base.moving && joint_stops==0);
        assert(fabsf(angle(SERVO_GARRA_GPIO)-180)<.001f);
        before=writes; tick(20,IO_OK);
        assert(writes==before && !pickup_sequence_reset(&pickup));
        bench_servo_stop(&servos); assert(joint_stops==1); /* Explicit emergency overrides hold. */
    }
    for (unsigned kind=0;kind<4;++kind) {
        fixture(); approach(); sensor_sample_t s=sample(20);
        if (kind==0) s.timestamp_ms=now-201;
        if (kind==1) s.timestamp_ms=now+1;
        if (kind==2) s.source=SENSOR_TEST;
        if (kind==3) s.value=NAN;
        pickup_sequence_poll(&pickup,now,&s,IO_OK);
        assert(pickup.phase==PICKUP_FAILED && pickup.fault==PICKUP_SENSOR);
    }
    fixture(); assert(pickup_sequence_start(&pickup,now));
    before=writes;
    tick(20,IO_DISABLED); assert(pickup.phase==PICKUP_FAILED && writes==before);
    fixture(); approach(); tick(60,IO_OK); tick(60,IO_TIMEOUT);
    assert(pickup.phase==PICKUP_FAILED && pickup.fault==PICKUP_SENSOR);

    fixture(); approach();
    for (unsigned i=0;i<20000 && pickup.phase!=PICKUP_FAILED;++i) tick(100,IO_OK);
    assert(pickup.fault==PICKUP_LIMIT && fabsf(angle(SERVO_GARRA_GPIO)-180)<.001f);
    fixture(); approach(); pickup.plan.phase_timeout_ms=1000;
    tick(100,IO_OK); assert(pickup.fault==PICKUP_TIMEOUT);
    fixture(); approach(); sensor_sample_t s=sample(100);
    pickup_sequence_poll(&pickup,now-1,&s,IO_OK); assert(pickup.fault==PICKUP_CLOCK);
    fixture(); approach(); fail_joint=true; tick(100,IO_OK);
    assert(pickup.fault==PICKUP_DRIVER && joint_stops>0 && servos.hard_stop_requested);

    const pickup_phase_t stop_phases[]={PICKUP_WAIT_SENSOR,PICKUP_APPROACH,PICKUP_VERIFY,
        PICKUP_CLOSE,PICKUP_LIFT,PICKUP_HOME,PICKUP_OPEN};
    for (unsigned phase=0;phase<sizeof(stop_phases)/sizeof(stop_phases[0]);++phase) {
        fixture(); assert(pickup_sequence_start(&pickup,now));
        for (unsigned i=0;i<20000 && pickup.phase!=stop_phases[phase];++i)
            tick(pickup.phase==PICKUP_WAIT_SENSOR?100:60,IO_OK);
        assert(pickup.phase==stop_phases[phase]);
        base_scan_stop(&base); bench_servo_stop(&servos);
        before=writes; tick(60,IO_OK);
        assert(pickup.phase==PICKUP_FAILED && servos.hard_stop_requested && writes==before);
        assert(!pickup_sequence_reset(&pickup));
    }
    fixture(); before=writes;
    assert(pickup_sequence_init(&pickup,&servos,&base,NULL));
    assert(!pickup_sequence_start(&pickup,now) && pickup.phase==PICKUP_DISABLED && writes==before);
    pickup_plan_t invalid=plan; invalid.shoulder_floor_deg=NAN;
    assert(!pickup_sequence_init(&pickup,&servos,&base,&invalid) && writes==before);
    puts("PASS: mock feedback pickup 60 mm, smooth bounded approach, distinct samples, close/lift/home/open, sensor/limit/clock/driver aborts");
}
