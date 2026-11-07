#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motion_model.h"
#include "sensor_data.h"

int main(void) {
    axis_limits_t limits[MOTION_AXES];
    motion_model_t model;
    motion_default_limits(limits);
    assert(motion_model_init(&model, limits));
    assert(motion_configured_axes(&model) == 0);
    float target[MOTION_AXES] = {1};
    assert(motion_begin(&model, target, 0, 1000) == MOTION_LIMIT);
    limits[0].max_deg = 90;
    assert(motion_model_init(&model, limits));
    assert(motion_begin(&model, target, 0, 1000) == MOTION_UNCONFIGURED);
    for (unsigned i = 0; i < MOTION_AXES; ++i)
        limits[i] = (axis_limits_t){.min_deg=0,.max_deg=90,.home_deg=0,.max_speed_deg_s=30,.configured=true};
    assert(motion_model_init(&model, limits));
    assert(motion_configured_axes(&model) == 6);
    target[0]=60; target[1]=30;
    assert(motion_begin(&model, target, 0, 3000) == MOTION_OK);
    assert(model.duration_ms == 2000);
    assert(motion_begin(&model, target, 0, 3000) == MOTION_BUSY);
    assert(motion_tick(&model, 1000) == MOTION_RUNNING);
    assert(fabsf(model.reference[0]-30) < 0.001f && fabsf(model.reference[1]-15) < 0.001f);
    assert(motion_tick(&model, 2000) == MOTION_DONE);
    assert(model.reference[0]==60 && model.reference[1]==30);
    assert(motion_begin(&model, target, 1999, 3000) == MOTION_CLOCK);
    float previous[MOTION_AXES]; memcpy(previous, model.reference, sizeof(previous));
    target[5]=91;
    assert(motion_begin(&model,target,2000,3000)==MOTION_LIMIT);
    assert(memcmp(previous,model.reference,sizeof(previous))==0);
    target[5]=NAN;
    assert(motion_begin(&model,target,2000,3000)==MOTION_INVALID);
    target[5]=INFINITY;
    assert(motion_begin(&model,target,2000,3000)==MOTION_INVALID);
    target[5]=0; target[0]=0;
    assert(motion_begin(&model,target,2000,100)==MOTION_TIMEOUT);
    assert(motion_begin(&model,target,2000,3000)==MOTION_OK);
    motion_tick(&model,2500);
    float held=model.reference[0];
    motion_cancel(&model);
    assert(motion_tick(&model,3000)==MOTION_CANCELLED && model.reference[0]==held);
    assert(motion_begin(&model,target,3000,3000)==MOTION_OK);
    assert(motion_tick(&model,2999)==MOTION_FAULT && model.fault==MOTION_CLOCK);
    assert(motion_model_init(&model,limits));
    assert(motion_begin(&model,target,0,2000)==MOTION_OK);
    assert(motion_tick(&model,2001)==MOTION_FAULT && model.fault==MOTION_TIMEOUT);
    limits[0].max_speed_deg_s=0;
    assert(!motion_model_init(&model,limits));
    limits[0].max_speed_deg_s=NAN;
    assert(!motion_model_init(&model,limits));
    assert(!motion_model_init(NULL,limits));
    limits[0]=(axis_limits_t){.min_deg=-FLT_MAX,.max_deg=FLT_MAX,.home_deg=-FLT_MAX,.max_speed_deg_s=1,.configured=true};
    assert(motion_model_init(&model,limits));
    target[0]=FLT_MAX;
    assert(motion_begin(&model,target,0,MOTION_MAX_DURATION_MS)==MOTION_TIMEOUT);

    sensor_data_t data; sensor_data_init(&data);
    assert(sensor_sample_check(&data.current_ma,0,100,SENSOR_HARDWARE)==SENSOR_UNAVAILABLE);
    assert(sensor_sample_update(&data.current_ma,10,100,SENSOR_TEST,true));
    assert(sensor_sample_check(&data.current_ma,100,100,SENSOR_HARDWARE)==SENSOR_WRONG_SOURCE);
    assert(sensor_sample_check(&data.current_ma,99,100,SENSOR_TEST)==SENSOR_FUTURE);
    assert(sensor_sample_check(&data.current_ma,200,100,SENSOR_TEST)==SENSOR_FRESH);
    assert(sensor_sample_check(&data.current_ma,201,100,SENSOR_TEST)==SENSOR_STALE);
    assert(!sensor_sample_update(&data.current_ma,10,50,SENSOR_TEST,true));
    assert(sensor_sample_check(&data.current_ma,100,100,SENSOR_TEST)==SENSOR_INVALID);
    assert(!sensor_sample_update(&data.current_ma,10,70,SENSOR_TEST,true));
    assert(sensor_sample_update(&data.current_ma,10,200,SENSOR_TEST,true));
    assert(!sensor_sample_update(&data.current_ma,NAN,201,SENSOR_TEST,true));
    assert(!sensor_sample_update(&data.current_ma,10,202,SENSOR_TEST,false));
    assert(!sensor_sample_update(&data.current_ma,-1,203,SENSOR_TEST,true));
    assert(!sensor_sample_update(&data.current_ma,1,203,SENSOR_NONE,true));
    puts("PASS: locked defaults, atomic limits, interpolation, rate, cancel, timeout, clock, sensor validity/source/age");
}
