#include "tof_vl53l0x.h"
#include "tof_acquisition.h"
#include "tof_i2c.h"
#include "vl53l0x_api.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static tof_i2c_t transport;
static struct vl53l0x_device device;
static int64_t deadline_us, started_ms;
static bool pending, fault;

static VL53L0X_Error read_register(void *context, uint8_t reg, uint8_t *data, uint32_t size) {
    return esp_timer_get_time() < deadline_us &&
           tof_i2c_read(context, reg, data, size) == ESP_OK
        ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

static VL53L0X_Error write_register(void *context, uint8_t reg, const uint8_t *data, uint32_t size) {
    uint8_t frame[257];
    if (!size || size >= sizeof(frame) || esp_timer_get_time() >= deadline_us)
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    frame[0] = reg;
    memcpy(frame + 1, data, size);
    return tof_i2c_write(context, frame, size + 1) == ESP_OK
        ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

static VL53L0X_Error delay_poll(void *context) {
    (void)context;
    vTaskDelay(1);
    return esp_timer_get_time() < deadline_us ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_TIME_OUT;
}

static hardware_result_t start(void *context) {
    if (fault || pending) return IO_FAULT;
    deadline_us = esp_timer_get_time() + 30000;
    started_ms = esp_timer_get_time() / 1000;
    pending = true;
    return VL53L0X_StartMeasurement(context) == VL53L0X_ERROR_NONE ? IO_OK : IO_FAULT;
}

static hardware_result_t read_range(void *context, uint16_t *mm, uint8_t *status, int64_t *timestamp) {
    if (fault || !pending) return IO_FAULT;
    deadline_us = esp_timer_get_time() + 30000;
    uint8_t ready;
    if (VL53L0X_GetMeasurementDataReady(context, &ready)) return IO_FAULT;
    if (!ready) return IO_PENDING;
    VL53L0X_RangingMeasurementData_t result;
    if (VL53L0X_GetRangingMeasurementData(context, &result) ||
        VL53L0X_ClearInterruptMask(context, 0)) return IO_FAULT;
    pending = false;
    *mm = result.RangeMilliMeter;
    *status = result.RangeStatus;
    *timestamp = started_ms;
    return IO_OK;
}

static void cancel(void *context) {
    (void)context;
    if (!pending) return;
    pending = false;
    /* A failed or timed-out conversion requires reinitializing the sensor on boot. */
    fault = true;
}

esp_err_t tof_vl53l0x_init(sensor_driver_t *driver) {
    if (!driver) return ESP_ERR_INVALID_ARG;
    esp_err_t result = tof_i2c_open(&transport);
    if (result != ESP_OK) return result;
    device = (struct vl53l0x_device){
        .context = &transport, .read = read_register, .write = write_register, .delay = delay_poll
    };
    deadline_us = esp_timer_get_time() + 1000000;
    uint8_t vhv, phase, aperture;
    uint32_t spad_count;
    if (VL53L0X_DataInit(&device) || VL53L0X_StaticInit(&device) ||
        VL53L0X_PerformRefCalibration(&device, &vhv, &phase) ||
        VL53L0X_PerformRefSpadManagement(&device, &spad_count, &aperture) ||
        VL53L0X_SetDeviceMode(&device, VL53L0X_DEVICEMODE_SINGLE_RANGING)) {
        esp_err_t cleanup = tof_i2c_close(&transport);
        return cleanup == ESP_OK ? ESP_FAIL : cleanup;
    }
    static tof_backend_t backend = {
        .start = start, .read = read_range, .cancel = cancel, .context = &device
    };
    return tof_acquisition_driver(&backend, driver) ? ESP_OK : ESP_FAIL;
}
