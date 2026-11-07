#ifndef HARDWARE_IO_H
#define HARDWARE_IO_H
typedef enum {
    IO_OK, IO_PENDING, IO_DISABLED, IO_INVALID, IO_UNAVAILABLE,
    IO_TIMEOUT, IO_FAULT, IO_CLOCK, IO_BLOCKED, IO_LATCHED
} hardware_result_t;
const char *hardware_result_name(hardware_result_t result);
#endif
