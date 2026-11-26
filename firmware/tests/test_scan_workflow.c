#include <assert.h>
#include <stdio.h>
#include "command_control.h"
static unsigned positional_writes, base_pulse;
static hardware_result_t attach(void *c, int gpio, unsigned channel) {
    (void)c; (void)gpio; (void)channel; return IO_OK;
}
static hardware_result_t write_joint(void *c, unsigned channel, unsigned us) {
    (void)c; (void)channel; (void)us; ++positional_writes; return IO_OK;
}
static hardware_result_t write_base(void *c, unsigned us) { (void)c; base_pulse=us; return IO_OK; }
static hardware_result_t stop(void *c) { (void)c; return IO_OK; }
static bench_servo_t servos;
static base_scan_t scan;
static robot_state_t state;
static int64_t now;
static hardware_result_t distance_io;
static command_result_t gesture(robot_event_t event) {
    return command_control_scan_event(&state,&servos,&scan,event,now);
}
static void tick(float mm) {
    now+=10;
    bench_servo_poll(&servos,now);
    sensor_sample_t sample={.value=mm,.timestamp_ms=now,.source=SENSOR_HARDWARE,.received=true,.valid=true};
    base_scan_poll(&scan,now,&sample,distance_io);
    servos.base_active=scan.moving;
    if (state==ROBOT_RUN_VARREDURA && scan.home) state_machine_process_event(&state,EVENT_SCAN_DONE);
}
static void fixture(void) {
    servos=(bench_servo_t){0}; now=0; state=ROBOT_IDLE; distance_io=IO_OK;
    bench_servo_driver_t joints={.attach=attach,.write_us=write_joint,.shutdown=stop};
    bench_servo_config_t config={.max_dps=90,.start_pin=7,.led_pin=48,.estop_pin=6,.base_pin=15};
    base_scan_driver_t base={.write_us=write_base,.shutdown=stop};
    assert(bench_servo_init(&servos,&joints,&config)); base_scan_init(&scan,&base);
    assert(bench_servo_idle_pose(&servos));
    for (unsigned i=0;i<10000 && servos.scan_pose_active;++i) tick(156);
    assert(!servos.scan_pose_active);
}
static void scan_to_wait(void) {
    assert(gesture(EVENT_DOUBLE_CLICK)==COMMAND_MOTION_BLOCKED && state==ROBOT_IDLE);
    assert(gesture(EVENT_SINGLE_CLICK)==COMMAND_ACCEPTED && state==ROBOT_RUN_VARREDURA);
    assert(gesture(EVENT_SINGLE_CLICK)==COMMAND_MOTION_BLOCKED);
    for (unsigned i=0;i<10000 && servos.scan_pose_active;++i) tick(156);
    assert(servos.distance_ready && base_scan_begin(&scan,now));
    servos.base_active=true;
    for (unsigned i=0;i<50;++i) tick(156);
    tick(100); assert(scan.found && scan.found_offset_ms==510 && base_pulse==1488);
    assert(gesture(EVENT_DOUBLE_CLICK)==COMMAND_MOTION_BLOCKED);
    assert(command_control_servo(&state,&servos,16,90)==COMMAND_MOTION_BLOCKED);
    for (unsigned i=0;i<1000 && scan.moving;++i) tick(1);
    assert(state==ROBOT_VARREDURA_CONCLUIDA && scan.home && base_pulse==1488);
    assert(gesture(EVENT_SINGLE_CLICK)==COMMAND_MOTION_BLOCKED);
    assert(command_control_servo(&state,&servos,16,90)==COMMAND_MOTION_BLOCKED);
}
int main(void) {
    fixture(); scan_to_wait();
    unsigned before=positional_writes;
    assert(gesture(EVENT_DOUBLE_CLICK)==COMMAND_ACCEPTED && state==ROBOT_RUN_DESLOCAMENTO);
    assert(base_pulse==1573);
    for (unsigned i=0;i<1000 && scan.moving;++i) tick(1);
    assert(scan.at_object && base_pulse==1488 && state==ROBOT_RUN_DESLOCAMENTO);
    assert(positional_writes==before); /* No grip, transport or automatic IDLE at the target. */
    assert(gesture(EVENT_DOUBLE_CLICK)==COMMAND_MOTION_BLOCKED);
    state_machine_process_event(&state,EVENT_EMERGENCY); base_scan_stop(&scan); bench_servo_stop(&servos);
    assert(gesture(EVENT_SINGLE_CLICK)==COMMAND_EMERGENCY_LATCHED);
    tick(1); assert(state==ROBOT_ERROR && !scan.moving);

    fixture(); scan_to_wait();
    assert(gesture(EVENT_LONG_PRESS)==COMMAND_ACCEPTED && state==ROBOT_RUN_VARREDURA);
    assert(!scan.found && !scan.home && scan.found_offset_ms==0 && scan.found_distance_mm==0);
    assert(servos.distance_ready && base_scan_begin(&scan,now));
    for (unsigned i=0;i<400 && scan.moving;++i) tick(156);
    assert(scan.no_object && !scan.latched && !scan.fault && base_pulse==1488);
    assert(state==ROBOT_VARREDURA_CONCLUIDA);
    assert(gesture(EVENT_DOUBLE_CLICK)==COMMAND_MOTION_BLOCKED && state==ROBOT_VARREDURA_CONCLUIDA);
    assert(gesture(EVENT_LONG_PRESS)==COMMAND_ACCEPTED && state==ROBOT_RUN_VARREDURA);
    assert(!scan.no_object && !scan.found);
    /* Whole workflow without a driver: pose, timed sweep, yellow wait, no map/estop. */
    fixture(); distance_io=IO_DISABLED;
    assert(gesture(EVENT_SINGLE_CLICK)==COMMAND_ACCEPTED);
    for (unsigned i=0;i<10000 && servos.scan_pose_active;++i) tick(10);
    assert(servos.distance_ready && base_scan_begin(&scan,now));
    for (unsigned i=0;i<400 && scan.moving;++i) tick(10);
    assert(state==ROBOT_VARREDURA_CONCLUIDA && scan.no_object && scan.home && !scan.found);
    assert(!scan.fault && !scan.latched && !servos.latched && base_pulse==1488);
    assert(gesture(EVENT_DOUBLE_CLICK)==COMMAND_MOTION_BLOCKED);
    assert(gesture(EVENT_LONG_PRESS)==COMMAND_ACCEPTED);
    puts("PASS: mock bench workflow gestures, pose, map, home/wait, confirmation, target hold, rescan, no pickup");
}
