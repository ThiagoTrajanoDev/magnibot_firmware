#ifndef ACTUATOR_INTERFACE_H
#define ACTUATOR_INTERFACE_H
#include "hardware_io.h"
#include "motion_model.h"
#define ACTUATOR_ALL_AXES ((1u << MOTION_AXES) - 1u)
typedef enum { ACTUATOR_DISABLED, ACTUATOR_TEST, ACTUATOR_HARDWARE } actuator_kind_t;
typedef struct {
    hardware_result_t (*apply)(void *context, unsigned mask, const float degrees[MOTION_AXES]);
    hardware_result_t (*shutdown)(void *context);
    void *context;
    unsigned supported_mask;
    actuator_kind_t kind;
} actuator_driver_t;
typedef struct {
    actuator_driver_t driver;
    axis_limits_t limits[MOTION_AXES];
    float reference[MOTION_AXES];
    hardware_result_t last_result;
    bool initialized, latched, shutdown_confirmed;
} actuator_interface_t;

bool actuator_interface_init(actuator_interface_t *interface, const actuator_driver_t *driver,
                             const axis_limits_t limits[MOTION_AXES]);
hardware_result_t actuator_interface_apply(actuator_interface_t *interface, unsigned mask,
                                            const float degrees[MOTION_AXES]);
void actuator_interface_stop(actuator_interface_t *interface);
const char *actuator_backend_name(actuator_kind_t kind);
#endif
