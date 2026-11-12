#ifdef NDEBUG
#error "tests rely on assert() side effects; do not build with NDEBUG"
#endif
#include <assert.h>
#include <stdio.h>
#include "gpio_policy.h"

int main(void) {
    const int start = 15, led = 48;
    const int usable[] = {1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18, 21, 38, 39, 40, 41, 42, 47};
    for (unsigned i = 0; i < sizeof(usable) / sizeof(usable[0]); ++i)
        assert(gpio_policy_estop_allowed(usable[i], start, led));
    const int reserved[] = {-1, -5, 0, 3, 19, 20, 22, 23, 24, 25, 26, 30, 32, 33, 35, 37,
                            43, 44, 45, 46, 49, 100, start, led};
    for (unsigned i = 0; i < sizeof(reserved) / sizeof(reserved[0]); ++i)
        assert(!gpio_policy_estop_allowed(reserved[i], start, led));
    assert(!gpio_policy_estop_allowed(17, 17, led) && !gpio_policy_estop_allowed(17, start, 17));
    puts("PASS: emergency GPIO policy rejects reserved, absent, strapping and conflicting pins");
}
