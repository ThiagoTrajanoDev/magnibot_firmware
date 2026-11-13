#define _POSIX_C_SOURCE 200809L
#ifdef NDEBUG
#error "tests rely on assert() side effects; do not build with NDEBUG"
#endif
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "serial_control.h"
#include "robot_runtime.h"
#include "serial_protocol.h"
#include "driver/usb_serial_jtag.h"
#include "bench_servo.h"

static const char *input;
static size_t remaining;
static int64_t now;
static bool driver_installed;
static esp_err_t install_result = ESP_ERR_INVALID_ARG;
static robot_state_t state = ROBOT_IDLE;
static bench_servo_t servos;
static unsigned servo_pulse, servo_attaches;
static hardware_result_t servo_attach(void *c, int gpio, unsigned channel) { (void)c; (void)gpio; (void)channel; ++servo_attaches; return IO_OK; }
static hardware_result_t servo_write(void *c, unsigned channel, unsigned us) { (void)c; (void)channel; servo_pulse = us; return IO_OK; }
static hardware_result_t servo_shutdown(void *c) { (void)c; return IO_OK; }
int64_t esp_timer_get_time(void) { return now * 1000; }
bool usb_serial_jtag_is_driver_installed(void) { return driver_installed; }
esp_err_t usb_serial_jtag_driver_install(usb_serial_jtag_driver_config_t *config) {
    assert(config->rx_buffer_size >= 256 && config->tx_buffer_size >= 256);
    if (install_result == ESP_OK) driver_installed = true;
    return install_result;
}
void usb_serial_jtag_vfs_use_driver(void) { assert(driver_installed); }
int usb_serial_jtag_read_bytes(void *data, uint32_t size, uint32_t wait) {
    assert(wait == 0 && size <= 256);
    size_t n = remaining < size ? remaining : size;
    if (n) { memcpy(data, input, n); input += n; remaining -= n; }
    return (int)n;
}
static void feed(const char *text) {
    assert(remaining == 0);
    input = text; remaining = strlen(text);
    do { serial_control_poll(&state); serial_control_flush(); now += 10; } while (remaining);
}
int main(void) {
    assert(robot_runtime_init());
    serial_control_poll(NULL);
    bench_servo_config_t servo_config = {.min_us = 600, .max_us = 2400, .max_dps = 90,
        .start_pin = 15, .led_pin = 48, .estop_pin = -1, .base_pin = 4};
    bench_servo_driver_t servo_driver = {.attach = servo_attach, .write_us = servo_write, .shutdown = servo_shutdown};
    assert(bench_servo_init(&servos, &servo_driver, &servo_config));
    assert(serial_control_init(&servos) == ESP_ERR_INVALID_ARG);
    install_result = ESP_OK;
    assert(serial_control_init(&servos) == ESP_OK);
    assert(serial_control_init(&servos) == ESP_OK);
    FILE *capture = tmpfile();
    assert(capture);
    int original_stdout = dup(STDOUT_FILENO);
    assert(original_stdout >= 0 && dup2(fileno(capture), STDOUT_FILENO) >= 0);
    feed("MB1 1 sta");
    feed("tus\n");
    feed("MB1 2 start\n");
    assert(state == ROBOT_IDLE);
    feed("5 90\n");
    assert(servo_attaches == 1 && servo_pulse == 1500);
    feed("99 10\n5 181\n");
    feed("MB1 3 incomplete");
    now += SERIAL_FRAME_TIMEOUT_MS;
    serial_control_poll(&state);
    feed("\n");
    /* Saturate response queue; emergency still executes even with no reply slot. */
    feed("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    feed("\nMB1 4 estop\nMB1 5 start\n");
    assert(state == ROBOT_ERROR);
    feed("5 45\n");
    for (int i = 0; i < 20; ++i) { serial_control_poll(&state); serial_control_flush(); }
    feed("MB1 6 diagnostic\nMB1 7 estop\nMB1 8 start\n");
    for (int i = 0; i < 4; ++i) { serial_control_poll(&state); serial_control_flush(); }
    assert(state == ROBOT_ERROR);
    fflush(stdout);
    assert(dup2(original_stdout, STDOUT_FILENO) >= 0);
    close(original_stdout);
    rewind(capture);
    char output[16384];
    size_t n = fread(output, 1, sizeof(output) - 1, capture);
    output[n] = '\0'; fclose(capture);
    assert(strstr(output, "\"id\":1,\"ok\":true"));
    assert(strstr(output, "\"code\":\"motion_blocked\""));
    assert(strstr(output, "\"code\":\"timeout\""));
    assert(strstr(output, "\"diagnostic\":{"));
    assert(strstr(output, "\"backend\":\"disabled\",\"actuator_shutdown_confirmed\":false"));
    const char *dropped = strstr(output, "\"dropped_responses\":");
    assert(dropped && strtoul(dropped + strlen("\"dropped_responses\":"), NULL, 10) > 0);
    assert(strstr(output, "\"id\":null,\"ok\":true,\"code\":\"ok\""));
    assert(strstr(output, "\"servo\":{\"gpio\":5,\"target_deg\":90}"));
    assert(strstr(output, "\"code\":\"gpio_not_allowed\""));
    assert(strstr(output, "\"code\":\"angle_out_of_range\""));
    assert(strstr(output, "\"bench_servo\":{\"enabled\":true,\"channels\":1}"));
    assert(!strstr(output, "\"target_deg\":45"));
    assert(strstr(output, "\"id\":8,\"ok\":false,\"code\":\"emergency_latched\""));
    puts("PASS: serial integration, init failure, fragmented RX, timeout, saturated queue emergency, diagnostics");
}
