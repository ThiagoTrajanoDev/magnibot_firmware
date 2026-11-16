#include "tof_i2c.h"
#include "config.h"

#define TOF_I2C_TIMEOUT_MS 10
#define VL53L0X_ADDRESS 0x29

esp_err_t tof_i2c_open(tof_i2c_t *transport) {
    if (!transport) return ESP_ERR_INVALID_ARG;
    if (transport->bus || transport->device) return ESP_ERR_INVALID_STATE;
    const i2c_master_bus_config_t bus = {
        .i2c_port = -1, .sda_io_num = I2C_SDA_GPIO, .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7
    };
    esp_err_t result = i2c_new_master_bus(&bus, &transport->bus);
    if (result != ESP_OK) return result;
    result = i2c_master_probe(transport->bus, VL53L0X_ADDRESS, TOF_I2C_TIMEOUT_MS);
    if (result == ESP_OK) {
        const i2c_device_config_t device = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = VL53L0X_ADDRESS, .scl_speed_hz = 100000
        };
        result = i2c_master_bus_add_device(transport->bus, &device, &transport->device);
    }
    if (result != ESP_OK) {
        esp_err_t cleanup = tof_i2c_close(transport);
        if (cleanup != ESP_OK) return cleanup;
    }
    return result;
}

esp_err_t tof_i2c_read(tof_i2c_t *transport, uint8_t reg, uint8_t *data, size_t size) {
    if (!transport || !transport->device || !data || !size) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit_receive(transport->device, &reg, 1, data, size, TOF_I2C_TIMEOUT_MS);
}

esp_err_t tof_i2c_write(tof_i2c_t *transport, const uint8_t *frame, size_t size) {
    if (!transport || !transport->device || !frame || size < 2) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit(transport->device, frame, size, TOF_I2C_TIMEOUT_MS);
}

esp_err_t tof_i2c_close(tof_i2c_t *transport) {
    if (!transport) return ESP_ERR_INVALID_ARG;
    if (transport->device) {
        esp_err_t result = i2c_master_bus_rm_device(transport->device);
        if (result != ESP_OK) return result;
        transport->device = NULL;
    }
    if (transport->bus) {
        esp_err_t result = i2c_del_master_bus(transport->bus);
        if (result != ESP_OK) return result;
        transport->bus = NULL;
    }
    return ESP_OK;
}
