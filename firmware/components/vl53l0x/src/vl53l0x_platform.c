#include "vl53l0x_platform.h"

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t reg, uint8_t *data, uint32_t size) {
    return dev->read(dev->context, reg, data, size);
}
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t reg, uint8_t *data, uint32_t size) {
    return dev->write(dev->context, reg, data, size);
}
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t reg, uint8_t *value) {
    return VL53L0X_ReadMulti(dev, reg, value, 1);
}
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    VL53L0X_Error result = VL53L0X_ReadMulti(dev, reg, data, sizeof(data));
    if (!result) *value = (uint16_t)((data[0] << 8) | data[1]);
    return result;
}
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t reg, uint32_t *value) {
    uint8_t data[4];
    VL53L0X_Error result = VL53L0X_ReadMulti(dev, reg, data, sizeof(data));
    if (!result) *value = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                         ((uint32_t)data[2] << 8) | data[3];
    return result;
}
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t reg, uint8_t value) {
    return VL53L0X_WriteMulti(dev, reg, &value, 1);
}
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t reg, uint16_t value) {
    uint8_t data[] = {value >> 8, value};
    return VL53L0X_WriteMulti(dev, reg, data, sizeof(data));
}
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t reg, uint32_t value) {
    uint8_t data[] = {value >> 24, value >> 16, value >> 8, value};
    return VL53L0X_WriteMulti(dev, reg, data, sizeof(data));
}
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t reg, uint8_t and_mask, uint8_t or_mask) {
    uint8_t value;
    VL53L0X_Error result = VL53L0X_RdByte(dev, reg, &value);
    return result ? result : VL53L0X_WrByte(dev, reg, (value & and_mask) | or_mask);
}
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV dev) {
    return dev->delay(dev->context);
}
