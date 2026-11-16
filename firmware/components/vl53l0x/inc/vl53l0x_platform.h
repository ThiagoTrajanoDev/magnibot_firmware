#ifndef VL53L0X_PLATFORM_H
#define VL53L0X_PLATFORM_H
#include "vl53l0x_def.h"
#include <string.h>
#define VL53L0X_COPYSTRING(destination, source) strcpy(destination, source)

typedef struct vl53l0x_device {
    VL53L0X_DevData_t Data;
    void *context;
    VL53L0X_Error (*read)(void *, uint8_t, uint8_t *, uint32_t);
    VL53L0X_Error (*write)(void *, uint8_t, const uint8_t *, uint32_t);
    VL53L0X_Error (*delay)(void *);
} *VL53L0X_DEV;

#define PALDevDataGet(dev, field) ((dev)->Data.field)
#define PALDevDataSet(dev, field, value) ((dev)->Data.field = (value))
#define _LOG_FUNCTION_START(...) ((void)0)
#define _LOG_FUNCTION_END(...) ((void)0)
#define _LOG_FUNCTION_END_FMT(...) ((void)0)

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t reg, uint8_t *data, uint32_t size);
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t reg, uint8_t *data, uint32_t size);
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t reg, uint8_t *value);
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t reg, uint16_t *value);
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t reg, uint32_t *value);
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t reg, uint8_t value);
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t reg, uint16_t value);
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t reg, uint32_t value);
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t reg, uint8_t and_mask, uint8_t or_mask);
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV dev);
#endif
