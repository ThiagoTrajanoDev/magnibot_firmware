#ifdef NDEBUG
#error "tests rely on assert() side effects; do not build with NDEBUG"
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "cycle_sequence.h"

static motion_model_t model;
static sensor_data_t sensors;
static cycle_sequence_t cycle;
static cycle_plan_t plan;
static int64_t now;
static bool contact, no_object;
static void refresh(void) {
    float distance=no_object ? 5000 : 300 - 100*(float)cycle.scan_index;
    float current=contact && cycle.phase==CYCLE_GRIP && now-cycle.phase_ms>=100 ? 200 : 50;
    assert(sensor_sample_update(&sensors.distance_mm,distance,now,SENSOR_TEST,true));
    assert(sensor_sample_update(&sensors.current_ma,current,now,SENSOR_TEST,true));
}
static void setup(void) {
    axis_limits_t limits[MOTION_AXES];
    for (unsigned i=0;i<MOTION_AXES;++i)
        limits[i]=(axis_limits_t){.min_deg=0,.max_deg=90,.home_deg=0,.max_speed_deg_s=90,.configured=true};
    assert(motion_model_init(&model,limits));
    sensor_data_init(&sensors);
    cycle_sequence_init(&cycle,&model,&sensors);
    plan=(cycle_plan_t){.scan_count=2,.gripper_closed_deg=30,.gripper_open_deg=0,
        .detect_min_mm=10,.detect_max_mm=800,.contact_ma=150,.overload_ma=1000,
        .settle_ms=20,.step_timeout_ms=5000,.operation_timeout_ms=30000,
        .map_ttl_ms=10000,.sensor_max_age_ms=200,.sensor_source=SENSOR_TEST};
    plan.scan[1][0]=20;
    plan.pickup[0][1]=10;
    plan.pickup[1][0]=20; plan.pickup[1][1]=10;
    plan.lift[1]=20; plan.deposit[0]=45; plan.deposit[1]=20;
    now=0; contact=true; no_object=false; refresh();
}
static void tick(void) { now+=10;refresh();cycle_tick(&cycle,now); }
static void reach(cycle_phase_t wanted) {
    for (unsigned i=0;i<4000 && cycle.phase!=wanted && cycle.phase!=CYCLE_ERROR;++i) {
        if (cycle.phase==CYCLE_WAIT_CONFIRM && wanted!=CYCLE_WAIT_CONFIRM) assert(cycle_confirm(&cycle,now));
        tick();
    }
    assert(cycle.phase==wanted);
}
int main(void) {
    setup(); assert(cycle_start_scan(&cycle,&plan,now));
    assert(cycle_robot_state(&cycle)==ROBOT_RUN_VARREDURA);
    reach(CYCLE_WAIT_CONFIRM);
    assert(cycle_robot_state(&cycle)==ROBOT_VARREDURA_CONCLUIDA);
    assert(cycle.map_valid && cycle.selected_index==1 && cycle.selected_distance_mm==200);
    assert(model.reference[0]==0);
    assert(cycle_confirm(&cycle,now));
    assert(cycle_robot_state(&cycle)==ROBOT_RUN_DESLOCAMENTO);
    reach(CYCLE_DEPOSIT);
    float grip=model.reference[CYCLE_GRIPPER_AXIS];
    assert(grip>0 && grip<30);
    while(cycle.phase==CYCLE_DEPOSIT) { tick(); assert(model.reference[CYCLE_GRIPPER_AXIS]==grip); }
    assert(cycle.phase==CYCLE_RELEASE && model.reference[0]==45);
    reach(CYCLE_IDLE);
    assert(cycle_robot_state(&cycle)==ROBOT_IDLE && !cycle.map_valid);
    for(unsigned i=0;i<MOTION_AXES;++i) assert(model.reference[i]==plan.home[i]);

    /* Every phase can be interrupted; no later tick or command restarts it. */
    for(int phase=CYCLE_IDLE;phase<CYCLE_ERROR;++phase) {
        setup();
        if(phase!=CYCLE_IDLE) { assert(cycle_start_scan(&cycle,&plan,now));reach((cycle_phase_t)phase); }
        float held[MOTION_AXES];memcpy(held,model.reference,sizeof(held));
        cycle_abort(&cycle,CYCLE_EMERGENCY);
        tick();
        assert(cycle.phase==CYCLE_ERROR && cycle.fault==CYCLE_EMERGENCY && !cycle.map_valid);
        assert(!cycle_start_scan(&cycle,&plan,now));assert(!cycle_confirm(&cycle,now));
        assert(memcmp(held,model.reference,sizeof(held))==0);
    }
    setup(); no_object=true;sensor_data_init(&sensors);refresh(); assert(cycle_start_scan(&cycle,&plan,now));reach(CYCLE_ERROR);
    assert(cycle.fault==CYCLE_NO_OBJECT && model.reference[0]==0);
    setup();contact=false;assert(cycle_start_scan(&cycle,&plan,now));reach(CYCLE_WAIT_CONFIRM);
    assert(cycle_confirm(&cycle,now));reach(CYCLE_ERROR);assert(cycle.fault==CYCLE_NO_CONTACT);
    setup();assert(cycle_start_scan(&cycle,&plan,now));now=10;refresh();
    now=11;sensor_sample_update(&sensors.current_ma,1000,now,SENSOR_TEST,true);cycle_tick(&cycle,now);
    assert(cycle.fault==CYCLE_OVERCURRENT);
    setup();assert(cycle_start_scan(&cycle,&plan,now));now=201;cycle_tick(&cycle,now);
    assert(cycle.fault==CYCLE_SENSOR_FAULT);
    setup();assert(cycle_start_scan(&cycle,&plan,now));now=10;
    sensor_sample_update(&sensors.distance_mm,100,now,SENSOR_TEST,false);cycle_tick(&cycle,now);
    assert(cycle.fault==CYCLE_SENSOR_FAULT);
    setup();sensor_sample_update(&sensors.current_ma,50,0,SENSOR_HARDWARE,true);
    assert(!cycle_start_scan(&cycle,&plan,now) && cycle.fault==CYCLE_SENSOR_FAULT);
    setup();sensor_data_init(&sensors);assert(!cycle_start_scan(&cycle,&plan,now));
    assert(cycle.fault==CYCLE_SENSOR_FAULT);
    setup();assert(cycle_start_scan(&cycle,&plan,now));reach(CYCLE_WAIT_CONFIRM);
    now+=plan.map_ttl_ms+1;refresh();assert(!cycle_confirm(&cycle,now));assert(cycle.fault==CYCLE_MAP_EXPIRED);
    setup();assert(cycle_start_scan(&cycle,&plan,now));reach(CYCLE_WAIT_CONFIRM);
    assert(cycle_start_scan(&cycle,&plan,now));assert(!cycle.map_valid && cycle.scan_index==0);
    setup();plan.map_ttl_ms=10;assert(cycle_start_scan(&cycle,&plan,now));reach(CYCLE_ERROR);
    assert(cycle.fault==CYCLE_MAP_EXPIRED);
    setup();plan.scan[1][1]=NAN;assert(!cycle_start_scan(&cycle,&plan,now));assert(cycle.fault==CYCLE_BAD_PLAN);
    setup();plan.pickup[1][CYCLE_GRIPPER_AXIS]=5;assert(!cycle_start_scan(&cycle,&plan,now));
    assert(cycle.fault==CYCLE_BAD_PLAN);
    setup();plan.scan[0][CYCLE_GRIPPER_AXIS]=5;assert(!cycle_start_scan(&cycle,&plan,now));
    setup();plan.gripper_closed_deg=100;assert(!cycle_start_scan(&cycle,&plan,now));
    setup();plan.step_timeout_ms=10;plan.settle_ms=0;plan.scan[0][0]=90;
    assert(!cycle_start_scan(&cycle,&plan,now));assert(cycle.fault==CYCLE_TIMEOUT);
    setup();assert(cycle_start_scan(&cycle,&plan,now));now=plan.operation_timeout_ms+1;refresh();cycle_tick(&cycle,now);
    assert(cycle.fault==CYCLE_TIMEOUT);
    setup();now=100;refresh();assert(cycle_start_scan(&cycle,&plan,now));cycle_tick(&cycle,99);
    assert(cycle.fault==CYCLE_CLOCK);
    setup();assert(cycle_start_scan(&cycle,&plan,now));motion_cancel(&model);tick();assert(cycle.fault==CYCLE_MOTION_FAULT);
    setup();assert(cycle_start_scan(&cycle,&plan,now));plan.overload_ma=1;tick();assert(cycle.phase!=CYCLE_ERROR);

    puts("PASS: synthetic cycle, mapped target, grip hold, all-phase emergency, sensors, no-object/contact, expiry, timeout, clock");
}
