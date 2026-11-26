#ifdef NDEBUG
#error "tests rely on assert() side effects; do not build with NDEBUG"
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sensor_interface.h"
#include "actuator_interface.h"

typedef struct {
    unsigned requests, polls, cancels;
    hardware_result_t request_result, poll_result;
    float value;
    int64_t timestamp;
} sensor_mock_t;
static hardware_result_t request(void *context, int64_t now) {
    sensor_mock_t *mock=context; (void)now; ++mock->requests; return mock->request_result;
}
static hardware_result_t poll(void *context,float *value,int64_t *timestamp) {
    sensor_mock_t *mock=context;++mock->polls;*value=mock->value;*timestamp=mock->timestamp;return mock->poll_result;
}
static void cancel(void *context) { ++((sensor_mock_t *)context)->cancels; }
static sensor_driver_t sensor_driver(sensor_mock_t *mock) {
    return (sensor_driver_t){.request=request,.poll=poll,.cancel=cancel,.context=mock,.source=SENSOR_TEST};
}
typedef struct {unsigned writes,stops,mask;hardware_result_t result,stop_result;float pose[MOTION_AXES];} output_mock_t;
static hardware_result_t apply(void *context,unsigned mask,const float degrees[MOTION_AXES]) {
    output_mock_t *mock=context;++mock->writes;mock->mask=mask;memcpy(mock->pose,degrees,sizeof(mock->pose));return mock->result;
}
/* Init refuses an initialised object, so each scenario starts from a zeroed one. */
#define SENSOR_INIT(...) (memset(&acquisition,0,sizeof(acquisition)),sensor_interface_init(&acquisition,__VA_ARGS__))
#define ACTUATOR_INIT(...) (memset(&output,0,sizeof(output)),actuator_interface_init(&output,__VA_ARGS__))
static hardware_result_t shutdown(void *context) {
    output_mock_t *mock=context;++mock->stops;return mock->stop_result;
}
int main(void) {
    sensor_interface_t acquisition; sensor_data_t data;
    assert(SENSOR_INIT(&data,NULL,NULL,50,200,500));
    sensor_interface_poll(&acquisition,0);sensor_interface_poll(&acquisition,1000);
    assert(acquisition.distance.last_result==IO_DISABLED && !data.distance_mm.received);
    sensor_mock_t mock={.request_result=IO_OK,.poll_result=IO_PENDING};
    sensor_driver_t driver=sensor_driver(&mock);
    assert(SENSOR_INIT(&data,&driver,NULL,50,200,500));
    sensor_interface_poll(&acquisition,0);assert(mock.requests==1 && mock.polls==0);
    sensor_interface_poll(&acquisition,10);assert(mock.polls==1 && acquisition.distance.pending);
    sensor_interface_poll(&acquisition,200);
    assert(mock.cancels==1 && !acquisition.distance.pending && acquisition.distance.last_result==IO_TIMEOUT);
    sensor_interface_poll(&acquisition,201);assert(mock.requests==2);
    mock.poll_result=IO_OK;mock.value=123;mock.timestamp=205;
    sensor_interface_poll(&acquisition,205);
    assert(data.distance_mm.valid && data.distance_mm.source==SENSOR_TEST && data.distance_mm.value==123);
    assert(!data.current_ma.received && acquisition.distance.last_result==IO_OK);
    sensor_interface_poll(&acquisition,240);assert(mock.requests==2);
    mock.request_result=IO_FAULT;sensor_interface_poll(&acquisition,251);
    assert(!data.distance_mm.valid && mock.cancels==2 && acquisition.distance.last_result==IO_FAULT);
    mock.request_result=IO_OK;sensor_interface_poll(&acquisition,301);
    mock.value=NAN;mock.timestamp=310;sensor_interface_poll(&acquisition,310);
    assert(!data.distance_mm.valid && acquisition.distance.last_result==IO_INVALID);
    sensor_interface_poll(&acquisition,351);
    mock.value=10;mock.timestamp=1000;sensor_interface_poll(&acquisition,360);
    assert(!data.distance_mm.valid && acquisition.distance.last_result==IO_INVALID);
    sensor_interface_poll(&acquisition,401);
    mock.timestamp=410;sensor_interface_poll(&acquisition,410);
    assert(data.distance_mm.valid && acquisition.distance.last_result==IO_OK);
    mock=(sensor_mock_t){.request_result=IO_OK,.poll_result=IO_PENDING};driver=sensor_driver(&mock);
    assert(SENSOR_INIT(&data,&driver,&driver,50,200,500));
    sensor_interface_poll(&acquisition,100);assert(mock.requests==2);
    sensor_interface_poll(&acquisition,99);
    assert(acquisition.stopped && mock.cancels==2 && acquisition.current.last_result==IO_CLOCK);
    sensor_interface_poll(&acquisition,2000);assert(mock.requests==2);
    sensor_interface_stop(&acquisition);assert(mock.cancels==2);
    assert(SENSOR_INIT(&data,&driver,NULL,50,200,500));
    sensor_interface_poll(&acquisition,0);sensor_interface_stop(&acquisition);sensor_interface_stop(&acquisition);
    assert(mock.cancels==3 && acquisition.stopped);
    driver.poll=NULL;assert(!SENSOR_INIT(&data,&driver,NULL,50,200,500));
    assert(!SENSOR_INIT(&data,NULL,NULL,0,200,500));
    driver=sensor_driver(&mock);driver.source=SENSOR_NONE;
    assert(!SENSOR_INIT(&data,&driver,NULL,50,200,500));

    mock=(sensor_mock_t){.request_result=IO_OK,.poll_result=IO_OK,.value=123,.timestamp=20};
    driver=sensor_driver(&mock);
    assert(SENSOR_INIT(&data,&driver,NULL,50,200,500));
    sensor_interface_poll_enabled(&acquisition,0,false);
    assert(mock.requests==0);
    sensor_interface_poll_enabled(&acquisition,10,true);
    assert(mock.requests==1 && acquisition.distance.pending);
    sensor_interface_poll_enabled(&acquisition,20,false);
    assert(!acquisition.distance.pending && data.distance_mm.valid && mock.cancels==0);
    sensor_interface_poll_enabled(&acquisition,1000,false);
    assert(mock.requests==1 && mock.cancels==0);
    sensor_interface_poll_enabled(&acquisition,1010,true);
    assert(mock.requests==2 && acquisition.distance.pending);
    sensor_interface_stop(&acquisition);
    assert(mock.cancels==1 && acquisition.stopped);

    assert(!sensor_interface_set_distance_driver(&acquisition,&driver)); /* stopped */
    mock=(sensor_mock_t){.request_result=IO_OK,.poll_result=IO_PENDING};
    driver=sensor_driver(&mock);
    assert(SENSOR_INIT(&data,&driver,NULL,50,200,500));
    sensor_interface_poll(&acquisition,0);
    assert(sensor_interface_set_distance_driver(&acquisition,NULL));
    assert(mock.cancels==1 && !acquisition.distance.pending && !data.distance_mm.valid);
    sensor_interface_poll(&acquisition,10);
    assert(mock.requests==1 && acquisition.distance.last_result==IO_DISABLED);
    sensor_driver_t invalid_driver=driver;invalid_driver.poll=NULL;
    assert(!sensor_interface_set_distance_driver(&acquisition,&invalid_driver));
    assert(sensor_interface_set_distance_driver(&acquisition,&driver));
    assert(sensor_interface_set_distance_driver(&acquisition,&acquisition.distance.driver));
    sensor_interface_poll(&acquisition,20);
    assert(mock.requests==2);

    axis_limits_t limits[MOTION_AXES];motion_default_limits(limits);
    actuator_interface_t output;float degrees[MOTION_AXES]={0};
    assert(ACTUATOR_INIT(NULL,limits));
    assert(actuator_interface_apply(&output,ACTUATOR_ALL_AXES,degrees)==IO_BLOCKED);
    output_mock_t writes={.result=IO_OK};
    actuator_driver_t backend={.apply=apply,.shutdown=shutdown,.context=&writes,
        .supported_mask=ACTUATOR_ALL_AXES,.kind=ACTUATOR_HARDWARE};
    for(unsigned i=0;i<MOTION_AXES;++i)
        limits[i]=(axis_limits_t){.min_deg=0,.max_deg=90,.max_speed_deg_s=10,.configured=true};
    assert(ACTUATOR_INIT(&backend,limits));
    degrees[0]=10;
    assert(actuator_interface_apply(&output,1,degrees)==IO_BLOCKED && writes.writes==0);
    actuator_interface_stop(&output);actuator_interface_stop(&output);assert(writes.stops==1);
    assert(actuator_interface_apply(&output,1,degrees)==IO_LATCHED && writes.writes==0);
    backend.kind=ACTUATOR_TEST;
    assert(ACTUATOR_INIT(&backend,limits));
    assert(actuator_interface_apply(&output,1,degrees)==IO_OK && writes.writes==1);
    assert(writes.mask==1 && writes.pose[0]==10 && writes.pose[1]==0);
    degrees[0]=20;degrees[1]=NAN;
    assert(actuator_interface_apply(&output,3,degrees)==IO_INVALID && writes.writes==1);
    degrees[1]=100;
    assert(actuator_interface_apply(&output,3,degrees)==IO_INVALID && writes.writes==1);
    assert(actuator_interface_apply(&output,0,degrees)==IO_INVALID);
    assert(actuator_interface_apply(&output,1u<<20,degrees)==IO_INVALID);
    assert(output.reference[0]==10);
    writes.result=IO_FAULT;
    assert(actuator_interface_apply(&output,1,degrees)==IO_FAULT && output.latched && writes.stops==2);
    assert(actuator_interface_apply(&output,1,degrees)==IO_LATCHED && writes.writes==2);
    limits[0].configured=false;assert(ACTUATOR_INIT(&backend,limits));
    degrees[0]=0;
    assert(actuator_interface_apply(&output,1,degrees)==IO_INVALID && writes.writes==2);
    assert(ACTUATOR_INIT(&backend,limits));
    writes.stop_result=IO_FAULT;actuator_interface_stop(&output);
    assert(output.latched && output.last_result==IO_FAULT);
    assert(actuator_interface_apply(&output,1,degrees)==IO_LATCHED);
    /* A failing shutdown is retried on every stop until the driver confirms it. */
    unsigned before=writes.stops;
    for(int i=0;i<5;++i) actuator_interface_stop(&output);
    assert(writes.stops==before+5 && !output.shutdown_confirmed && output.last_result==IO_FAULT);
    writes.stop_result=IO_PENDING;actuator_interface_stop(&output);
    assert(!output.shutdown_confirmed && output.last_result==IO_FAULT);
    writes.stop_result=IO_OK;actuator_interface_stop(&output);
    assert(output.shutdown_confirmed && output.last_result==IO_LATCHED && writes.stops==before+7);
    actuator_interface_stop(&output);assert(writes.stops==before+7);
    /* Without a driver there is nothing to shut down: confirmed at once. */
    assert(ACTUATOR_INIT(NULL,limits));actuator_interface_stop(&output);
    assert(output.latched && output.shutdown_confirmed && output.last_result==IO_LATCHED);
    /* A write failure reports the write result once the shutdown itself succeeded. */
    writes.result=IO_FAULT;writes.stop_result=IO_OK;backend.kind=ACTUATOR_TEST;limits[0].configured=true;
    assert(ACTUATOR_INIT(&backend,limits));degrees[0]=10;
    assert(actuator_interface_apply(&output,1,degrees)==IO_FAULT && output.shutdown_confirmed);

    /* Init never resets a latched/stopped/active interface or drops a pending request. */
    assert(!actuator_interface_init(&output,&backend,limits) && output.latched && output.shutdown_confirmed);
    mock=(sensor_mock_t){.request_result=IO_OK,.poll_result=IO_PENDING};driver=sensor_driver(&mock);
    assert(SENSOR_INIT(&data,&driver,NULL,50,200,500));
    sensor_interface_poll(&acquisition,0);sensor_interface_stop(&acquisition);
    assert(!sensor_interface_init(&acquisition,&data,NULL,NULL,50,200,500) && acquisition.stopped);
    assert(SENSOR_INIT(&data,&driver,NULL,50,200,500));
    sensor_interface_poll(&acquisition,0);
    assert(!sensor_interface_init(&acquisition,&data,NULL,NULL,50,200,500) &&
           acquisition.distance.pending && mock.cancels==1);

    /* Replayed (equal) timestamps are not fresh data; a newer one after a fault is. */
    mock=(sensor_mock_t){.request_result=IO_OK,.poll_result=IO_OK,.value=7,.timestamp=100};driver=sensor_driver(&mock);
    assert(SENSOR_INIT(&data,&driver,NULL,10,200,500));
    sensor_interface_poll(&acquisition,100);sensor_interface_poll(&acquisition,100);
    assert(data.distance_mm.valid && data.distance_mm.timestamp_ms==100);
    sensor_interface_poll(&acquisition,110);sensor_interface_poll(&acquisition,110);
    assert(!data.distance_mm.valid && acquisition.distance.last_result==IO_INVALID);
    mock.timestamp=120;sensor_interface_poll(&acquisition,120);sensor_interface_poll(&acquisition,120);
    assert(data.distance_mm.valid && acquisition.distance.last_result==IO_OK);
    backend.shutdown=NULL;assert(!ACTUATOR_INIT(&backend,limits));
    assert(actuator_interface_apply(&output,1,degrees)==IO_INVALID && writes.writes==3);
    puts("PASS: disabled interfaces, async reads, timeout/cancel, invalidation, clock latch, hardware write blocked, atomic limits, output shutdown latch");
}
