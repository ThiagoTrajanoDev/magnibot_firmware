#include "actuator_interface.h"
#include <string.h>

bool actuator_interface_init(actuator_interface_t *interface, const actuator_driver_t *driver,
                             const axis_limits_t limits[MOTION_AXES]) {
    if (!interface) return false;
    memset(interface, 0, sizeof(*interface));
    interface->latched = true;
    interface->last_result = IO_INVALID;
    if (!motion_model_init(&interface->validation, limits)) return false;
    if (driver) {
        if (driver->kind != ACTUATOR_DISABLED && driver->kind != ACTUATOR_TEST &&
            driver->kind != ACTUATOR_HARDWARE) return false;
        if (driver->kind == ACTUATOR_DISABLED) {
            if (driver->apply || driver->shutdown || driver->supported_mask) return false;
        } else if (!driver->apply || !driver->shutdown || !driver->supported_mask ||
                   (driver->supported_mask & ~ACTUATOR_ALL_AXES)) return false;
        interface->driver = *driver;
    }
    interface->initialized = true;
    interface->latched = false;
    interface->last_result = IO_BLOCKED;
    return true;
}

void actuator_interface_stop(actuator_interface_t *interface) {
    if (!interface || !interface->initialized || interface->latched) return;
    interface->latched = true;
    interface->last_result = IO_LATCHED;
    if (interface->driver.shutdown) {
        hardware_result_t result = interface->driver.shutdown(interface->driver.context);
        if (result != IO_OK) interface->last_result = result == IO_PENDING ? IO_FAULT : result;
    }
}

hardware_result_t actuator_interface_apply(actuator_interface_t *interface, unsigned mask,
                                            const float degrees[MOTION_AXES]) {
    if (!interface || !interface->initialized) return IO_INVALID;
    if (interface->latched) return IO_LATCHED;
    /* No hardware authorization exists: only native test fixtures can write. */
    if (interface->driver.kind != ACTUATOR_TEST) return interface->last_result = IO_BLOCKED;
    if (!degrees || !mask || (mask & ~interface->driver.supported_mask))
        return interface->last_result = IO_INVALID;
    float pose[MOTION_AXES];
    memcpy(pose, interface->validation.reference, sizeof(pose));
    for (unsigned i = 0; i < MOTION_AXES; ++i) {
        if (!(mask & (1u << i))) continue;
        if (!interface->validation.limits[i].configured) return interface->last_result = IO_INVALID;
        pose[i] = degrees[i];
    }
    if (motion_validate_pose(&interface->validation, pose) != MOTION_OK)
        return interface->last_result = IO_INVALID;
    hardware_result_t result = interface->driver.apply(interface->driver.context, mask, pose);
    if (result != IO_OK) {
        actuator_interface_stop(interface);
        if (interface->last_result == IO_LATCHED)
            interface->last_result = result == IO_PENDING ? IO_FAULT : result;
        return interface->last_result;
    }
    memcpy(interface->validation.reference, pose, sizeof(pose));
    return interface->last_result = IO_OK;
}
