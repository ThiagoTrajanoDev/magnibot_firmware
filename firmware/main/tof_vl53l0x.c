#include "tof_vl53l0x.h"
#include "tof_acquisition.h"
#include "tof_i2c.h"
#include "vl53l0x_api.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static tof_i2c_t transport;
static struct vl53l0x_device device;
static int64_t deadline_us, started_ms;
static bool pending, fault;

static VL53L0X_Error read_register(void *context, uint8_t reg, uint8_t *data, uint32_t size) {
    esp_err_t result = esp_timer_get_time() < deadline_us
        ? tof_i2c_read(context, reg, data, size) : ESP_ERR_TIMEOUT;
    if (result != ESP_OK)
        ESP_LOGE("VL53L0X", "Read reg=0x%02x size=%lu: %s", reg, (unsigned long)size, esp_err_to_name(result));
    return result == ESP_OK ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

static VL53L0X_Error write_register(void *context, uint8_t reg, const uint8_t *data, uint32_t size) {
    uint8_t frame[257];
    if (!size || size >= sizeof(frame) || esp_timer_get_time() >= deadline_us)
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    frame[0] = reg;
    memcpy(frame + 1, data, size);
    esp_err_t result = tof_i2c_write(context, frame, size + 1);
    if (result != ESP_OK)
        ESP_LOGE("VL53L0X", "Write reg=0x%02x size=%lu: %s", reg, (unsigned long)size, esp_err_to_name(result));
    return result == ESP_OK ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
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
    esp_err_t result = tof_i2c_close(&transport);
    if (result != ESP_OK) return result;
    result = tof_i2c_open(&transport);
    if (result != ESP_OK) return result;
    /* ESP reset does not reset an independently powered sensor. Avoid the vendor
     * ResetDevice helper, whose polling loops have no deadline on I2C failures. */
    const uint8_t reset_low[] = {0xBF, 0x00};
    const uint8_t reset_high[] = {0xBF, 0x01};
    result = tof_i2c_write(&transport, reset_low, sizeof(reset_low));
    if (result == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
        result = tof_i2c_write(&transport, reset_high, sizeof(reset_high));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (result != ESP_OK) {
        ESP_LOGE("VL53L0X", "Soft reset failed: %s", esp_err_to_name(result));
        esp_err_t cleanup = tof_i2c_close(&transport);
        return cleanup == ESP_OK ? result : cleanup;
    }
    pending = fault = false;
    uint8_t model = 0;
    result = tof_i2c_read(&transport, 0xC0, &model, 1);
    if (result != ESP_OK || model != 0xEE) {
        ESP_LOGE("VL53L0X", "Model ID 0x%02x, expected 0xee; I2C: %s", model, esp_err_to_name(result));
        esp_err_t cleanup = tof_i2c_close(&transport);
        return cleanup == ESP_OK ? ESP_FAIL : cleanup;
    }
    ESP_LOGI("VL53L0X", "Model ID 0x%02x verified", model);
    device = (struct vl53l0x_device){
        .context = &transport, .read = read_register, .write = write_register, .delay = delay_poll
    };
    deadline_us = esp_timer_get_time() + 1000000;
    uint8_t vhv, phase, aperture;
    uint32_t spad_count;
    const char *stage = "DataInit";
    VL53L0X_Error error = VL53L0X_DataInit(&device);
    if (!error) {
        stage = "StaticInit";
        error = VL53L0X_StaticInit(&device);
    }
    if (!error) {
        stage = "RefCalibration";
        error = VL53L0X_PerformRefCalibration(&device, &vhv, &phase);
    }
    if (!error) {
        stage = "RefSpadManagement";
        error = VL53L0X_PerformRefSpadManagement(&device, &spad_count, &aperture);
    }
    if (!error) {
        stage = "SetDeviceMode";
        error = VL53L0X_SetDeviceMode(&device, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    }
    if (error) {
        ESP_LOGE("VL53L0X", "%s failed: %d", stage, error);
        esp_err_t cleanup = tof_i2c_close(&transport);
        return cleanup == ESP_OK ? ESP_FAIL : cleanup;
    }
    static tof_backend_t backend = {
        .start = start, .read = read_range, .cancel = cancel, .context = &device
    };
    return tof_acquisition_driver(&backend, driver) ? ESP_OK : ESP_FAIL;
}
