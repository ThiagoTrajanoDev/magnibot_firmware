#ifndef TOF_I2C_H
#define TOF_I2C_H
#include "driver/i2c_master.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t device;
} tof_i2c_t;

/* Call with a zero-initialized instance. Open only in a dedicated sensor profile. */
esp_err_t tof_i2c_open(tof_i2c_t *transport);
esp_err_t tof_i2c_read(tof_i2c_t *transport, uint8_t reg, uint8_t *data, size_t size);
esp_err_t tof_i2c_write(tof_i2c_t *transport, const uint8_t *frame, size_t size);
esp_err_t tof_i2c_close(tof_i2c_t *transport);
#endif
