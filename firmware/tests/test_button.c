#ifdef NDEBUG
#error "tests rely on assert() side effects; do not build with NDEBUG"
#endif
#include <assert.h>
#include <stdio.h>
#include "button_control.h"
static int64_t now;
static int level = 1;
static esp_err_t gpio_result;
int64_t esp_timer_get_time(void) { return now * 1000; }
esp_err_t gpio_config(const gpio_config_t *cfg) { (void)cfg; return gpio_result; }
int gpio_get_level(gpio_num_t pin) { (void)pin; return level; }
static button_control_t button;
static robot_event_t event;
static bool sample(int v, int64_t ms) {
    now = ms; level = v;
    return button_control_update(&button, &event);
}
static void reset(void) { now = 0; level = 1; gpio_result = ESP_OK; assert(button_control_init(&button, 15) == ESP_OK); }
static void press(int64_t t) { assert(!sample(0,t)); assert(!sample(0,t+50)); }
static void release(int64_t t) { assert(!sample(1,t)); }
int main(void) {
    assert(button_control_init(NULL, 15) == ESP_ERR_INVALID_ARG);
    assert(button_control_init(&button, -1) == ESP_ERR_INVALID_ARG);
    gpio_result = 42; assert(button_control_init(&button,15) == 42);
    assert(!button_control_update(NULL, &event));
    assert(!button_control_sample(&button, 2, 0, &event));
    assert(!button_control_sample(&button, 1, 0, NULL));
    reset(); assert(!sample(0,10)); assert(!sample(1,30)); assert(!sample(1,80));
    press(100); release(200); assert(!sample(1,250));
    assert(!sample(1,1249)); assert(sample(1,1250)); assert(event == EVENT_SINGLE_CLICK);
    assert(!sample(1,1300));
    reset(); press(100); release(200); assert(!sample(1,250));
    press(400); release(500); assert(sample(1,550)); assert(event == EVENT_DOUBLE_CLICK);
    assert(!sample(1,1600));
    reset(); press(100); assert(!sample(0,2150)); assert(sample(0,2151));
    assert(event == EVENT_LONG_PRESS); assert(!sample(0,3000)); release(3100); assert(!sample(1,3150));
    /* A held second press must not emit the pending single click. */
    reset(); press(100); release(200); assert(!sample(1,250)); press(400);
    assert(!sample(0,1250)); assert(sample(0,2451)); assert(event == EVENT_LONG_PRESS);
    reset(); press(100); release(1300); assert(!sample(1,1350)); assert(!sample(1,3000));
    puts("PASS: GPIO failure, debounce, click timing");
}
