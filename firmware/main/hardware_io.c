#include "hardware_io.h"
const char *hardware_result_name(hardware_result_t result) {
    static const char *const names[] = {"ok", "pending", "disabled", "invalid", "unavailable",
        "timeout", "fault", "clock", "blocked", "latched"};
    return (unsigned)result < sizeof(names)/sizeof(names[0]) ? names[result] : "invalid";
}
