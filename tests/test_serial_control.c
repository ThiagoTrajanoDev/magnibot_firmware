#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "serial_control.h"
#include "robot_runtime.h"
#include "driver/usb_serial_jtag.h"

static const char *input;
static size_t remaining;
static int64_t now;
static unsigned led_updates;
static bool driver_installed;
static esp_err_t install_result = ESP_ERR_INVALID_ARG;
static robot_state_t state = ROBOT_IDLE;
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
void led_control_update(robot_state_t updated) {
    assert(updated == ROBOT_ERROR);
    ++led_updates;
}
static void feed(const char *text) {
    assert(remaining == 0);
    input = text; remaining = strlen(text);
    do { serial_control_poll(&state); now += 10; } while (remaining);
}
int main(void) {
    assert(robot_runtime_init());robot_state_t probe=ROBOT_IDLE;
    robot_runtime_poll(&probe,0);robot_runtime_status_t status=robot_runtime_status(0);
    assert(probe==ROBOT_IDLE && status.axes_configured==0 && status.distance_status==SENSOR_UNAVAILABLE);
    probe=ROBOT_ERROR;robot_runtime_poll(&probe,10);status=robot_runtime_status(10);
    assert(probe==ROBOT_ERROR && status.phase==CYCLE_ERROR && status.fault==CYCLE_EMERGENCY);
    probe=ROBOT_IDLE;robot_runtime_poll(&probe,20);assert(probe==ROBOT_ERROR);
    assert(robot_runtime_init());
    serial_control_poll(NULL);
    assert(serial_control_init() == ESP_ERR_INVALID_ARG);
    install_result = ESP_OK;
    assert(serial_control_init() == ESP_OK);
    assert(serial_control_init() == ESP_OK);
    FILE *capture = tmpfile();
    assert(capture);
    int original_stdout = dup(STDOUT_FILENO);
    assert(original_stdout >= 0 && dup2(fileno(capture), STDOUT_FILENO) >= 0);
    feed("MB1 1 sta");
    feed("tus\n");
    feed("MB1 2 start\n");
    assert(state == ROBOT_IDLE);
    feed("MB1 3 incomplete");
    now += 1000;
    serial_control_poll(&state);
    feed("\n");
    /* Saturate response queue; emergency still executes even with no reply slot. */
    feed("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    feed("\nMB1 4 estop\nMB1 5 start\n");
    assert(state == ROBOT_ERROR && led_updates == 1);
    for (int i = 0; i < 20; ++i) serial_control_poll(&state);
    feed("MB1 6 diagnostic\nMB1 7 estop\nMB1 8 start\n");
    for (int i = 0; i < 4; ++i) serial_control_poll(&state);
    assert(led_updates == 1 && state == ROBOT_ERROR);
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
    const char *dropped = strstr(output, "\"dropped_responses\":");
    assert(dropped && strtoul(dropped + strlen("\"dropped_responses\":"), NULL, 10) > 0);
    assert(strstr(output, "\"id\":8,\"ok\":false,\"code\":\"emergency_latched\""));
    puts("PASS: serial integration, init failure, fragmented RX, timeout, saturated queue emergency, diagnostics");
}
