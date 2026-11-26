#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "base_scan.h"
static unsigned pulse, writes, stops;
static bool fail;
static hardware_result_t write_us(void *context, unsigned value) {
    (void)context; pulse = value; ++writes; return fail ? IO_FAULT : IO_OK;
}
static hardware_result_t shutdown(void *context) { (void)context; ++stops; return IO_OK; }
static void make(base_scan_t *scan) {
    base_scan_driver_t driver = {.write_us = write_us, .shutdown = shutdown};
    pulse = writes = stops = 0; fail = false;
    base_scan_init(scan, &driver);
}
static void measure(base_scan_t *scan, int64_t now, float mm) {
    sensor_sample_t sample = {.value=mm,.timestamp_ms=now,.source=SENSOR_HARDWARE,.received=true,.valid=true};
    base_scan_poll(scan, now, &sample, IO_OK);
}
int main(void) {
    base_scan_t scan;
    make(&scan); base_scan_stop(&scan); assert(writes == 0);
    make(&scan); assert(base_scan_begin(&scan,100) && pulse==1573);
    assert(!base_scan_begin(&scan,200));
    measure(&scan,1100,156); assert(pulse==1403);
    measure(&scan,2850,156); assert(pulse==1573);
    measure(&scan,3600,156);
    assert(pulse==1488 && !scan.moving && !scan.fault && scan.no_object && !scan.latched && scan.home);
    assert(strcmp(base_scan_status(&scan),"no_object")==0);
    assert(!base_scan_replay(&scan,5000));
    assert(base_scan_begin(&scan,5000));

    const float values[] = {155,154,1,156,0,-1,NAN,INFINITY};
    for (unsigned i=0;i<sizeof(values)/sizeof(values[0]);++i) {
        make(&scan); assert(base_scan_begin(&scan,100));
        measure(&scan,110,values[i]);
        assert(scan.found==(i<3) && scan.moving && !scan.fault);
        if (scan.found) assert(pulse==1488 && scan.paused && scan.found_distance_mm==values[i]);
    }
    /* Save targets on either side of the origin, and at every phase boundary. */
    const int detections[] = {600,1100,1300,2100,2400,2850,3000,3600};
    const int offsets[] = {500,1000,800,0,-300,-750,-600,0};
    for (unsigned i=0;i<sizeof(detections)/sizeof(detections[0]);++i) {
        make(&scan); assert(base_scan_begin(&scan,100));
        int t;
        for (t=110;t<=detections[i];t+=10) measure(&scan,t,t==detections[i]?100:156);
        assert(scan.found && scan.found_offset_ms==offsets[i] && pulse==1488);
        assert(!base_scan_replay(&scan,t));
        for (;scan.moving;t+=10) {
            measure(&scan,t,1); /* Near readings during return never overwrite the target. */
            assert(!scan.fault);
        }
        assert(scan.home && pulse==1488 && scan.position_ms==0 && !scan.at_object);
        assert(scan.found_offset_ms==offsets[i] && scan.found_distance_mm==100);
        assert(strcmp(base_scan_status(&scan),"awaiting_confirm")==0);
        assert(base_scan_replay(&scan,t));
        if (offsets[i]) assert(pulse==(offsets[i]>0?1573:1403));
        for (;scan.moving;t+=10) measure(&scan,t,10);
        assert(!scan.fault && scan.at_object && pulse==1488 && scan.position_ms==offsets[i]);
        assert(!base_scan_replay(&scan,t));
        assert(strcmp(base_scan_status(&scan),"at_object")==0);
        assert(base_scan_return_home(&scan,t));
        if (offsets[i]) assert(pulse==(offsets[i]>0?1403:1573));
        for (;scan.moving;t+=10) base_scan_poll(&scan,t,NULL,IO_DISABLED);
        assert(scan.home && !scan.at_object && scan.position_ms==0 && pulse==1488 && !scan.fault);
        assert(!base_scan_return_home(&scan,t));
    }
    /* Sensor loss never creates a map, aborts movement or latches the base. */
    const hardware_result_t missing[] = {IO_DISABLED, IO_UNAVAILABLE, IO_TIMEOUT, IO_FAULT,
                                        IO_CLOCK, IO_LATCHED, IO_INVALID};
    for (unsigned i=0;i<sizeof(missing)/sizeof(missing[0]);++i) {
        make(&scan); assert(base_scan_begin(&scan,0));
        for (int t=10;t<=3500;t+=10) {
            sensor_sample_t cached={.value=100,.timestamp_ms=t,.source=SENSOR_HARDWARE,.received=true,.valid=true};
            base_scan_poll(&scan,t,&cached,missing[i]);
        }
        assert(scan.home && scan.no_object && !scan.found && !scan.fault && !scan.latched);
        assert(pulse==1488 && !scan.moving && stops==0 && !base_scan_replay(&scan,3600));
        assert(base_scan_begin(&scan,4000));
        measure(&scan,4100,100);
        assert(scan.found); /* A recovered, valid sensor can map a new target. */
        for (int t=4110;scan.moving;t+=10) base_scan_poll(&scan,t,NULL,missing[i]);
        assert(scan.home && scan.found && !scan.fault && !scan.latched);
        int64_t t=scan.last_ms;
        assert(base_scan_replay(&scan,t));
        while (scan.moving) { t+=10; base_scan_poll(&scan,t,NULL,missing[i]); }
        assert(scan.at_object && pulse==1488 && !scan.fault);
    }
    /* Reject old/future/stale/invalid/mock samples. */
    make(&scan); assert(base_scan_begin(&scan,100));
    sensor_sample_t sample={.value=100,.timestamp_ms=100,.source=SENSOR_HARDWARE,.received=true,.valid=true};
    base_scan_poll(&scan,110,&sample,IO_OK); assert(!scan.found);
    sample.timestamp_ms=401; base_scan_poll(&scan,400,&sample,IO_OK); assert(!scan.found);
    sample.timestamp_ms=150; base_scan_poll(&scan,400,&sample,IO_OK); assert(!scan.found);
    sample.timestamp_ms=410;sample.source=SENSOR_TEST;
    base_scan_poll(&scan,410,&sample,IO_OK); assert(!scan.found);
    sample.source=SENSOR_HARDWARE;sample.valid=false;
    base_scan_poll(&scan,420,&sample,IO_INVALID); assert(!scan.found);
    sample.valid=true;sample.timestamp_ms=430;
    base_scan_poll(&scan,430,&sample,IO_OK); assert(scan.found && pulse==1488);

    make(&scan); assert(base_scan_begin(&scan,0));
    base_scan_stop(&scan); assert(pulse==1488 && scan.latched && stops==1);
    assert(!base_scan_begin(&scan,100));
    make(&scan); assert(base_scan_begin(&scan,0));
    measure(&scan,1051,100); assert(scan.fault && scan.latched && !scan.found);
    make(&scan); assert(base_scan_begin(&scan,20));
    measure(&scan,19,100); assert(scan.fault && scan.latched);
    /* Explicit emergency during detection pause never resumes, even with good readings. */
    make(&scan); assert(base_scan_begin(&scan,0));measure(&scan,100,100);
    base_scan_stop(&scan); measure(&scan,300,100);
    assert(scan.latched && !scan.moving && pulse==1488);
    make(&scan); fail=true;
    assert(!base_scan_begin(&scan,0) && scan.fault && scan.latched);
    make(&scan); assert(base_scan_begin(&scan,0));fail=true;
    measure(&scan,100,100); assert(scan.fault && !scan.found && scan.latched);
    base_scan_init(&scan,NULL); assert(!base_scan_begin(&scan,0));
    puts("PASS: base detection, RAM map, full sweep return, signed replay, optional sensor, no-target home, boundaries, clock/driver faults");
}
