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
#include "command_control.h"

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
static hardware_result_t distance_request(void *c, int64_t timestamp) {
    (void)c; (void)timestamp; return IO_PENDING;
}
static hardware_result_t distance_poll(void *c, float *value, int64_t *timestamp) {
    (void)c; *value=156; *timestamp=now; return IO_OK;
}
static void distance_cancel(void *c) { (void)c; }
static hardware_result_t base_write(void *c, unsigned value) { (void)c; (void)value; return IO_OK; }
int main(void) {
    assert(robot_runtime_init());
    serial_control_poll(NULL);
    bench_servo_config_t servo_config = {.max_dps = 90,
        .start_pin = 7, .led_pin = 48, .estop_pin = 6, .base_pin = 15};
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
    feed("MB1 2 confirm\n");
    assert(state == ROBOT_IDLE);
    fflush(stdout);
    const long before = ftell(capture);
    feed("ombro 181\nombro -1\nombro 999999999999999\n");
    fflush(stdout);
    assert(ftell(capture) == before && servo_attaches == 0);
    feed("ombro 105\n");
    assert(servo_attaches == 1 && servo_pulse == 1667);
    feed("ombro 0\n");
    assert(state == ROBOT_IDLE && servos.channels[0].target == 0);
    feed("16 90\n");
    assert(servo_attaches == 1 && servo_pulse == 1667);
    feed("varredura\n");
    assert(!servos.scan_pose_active && servo_attaches == 1);
    feed("99 10\n16 181\n");
    feed("MB1 3 incomplete");
    now += SERIAL_FRAME_TIMEOUT_MS;
    serial_control_poll(&state);
    feed("\n");
    /* Saturate response queue; emergency still executes even with no reply slot. */
    feed("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    feed("\nMB1 4 estop\nMB1 5 start\n");
    assert(state == ROBOT_ERROR);
    feed("16 45\n");
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
    assert(strstr(output, "\"code\":\"pose_not_ready\""));
    assert(strstr(output, "\"code\":\"timeout\""));
    assert(strstr(output, "\"diagnostic\":{"));
    assert(strstr(output, "\"backend\":\"disabled\",\"actuator_shutdown_confirmed\":false"));
    const char *dropped = strstr(output, "\"dropped_responses\":");
    assert(dropped && strtoul(dropped + strlen("\"dropped_responses\":"), NULL, 10) > 0);
    assert(strstr(output, "\"id\":null,\"ok\":true,\"code\":\"ok\""));
    assert(strstr(output, "\"servo\":{\"gpio\":16,\"target_deg\":90}"));
    assert(strstr(output, "\"servo\":{\"gpio\":16,\"target_deg\":0}"));
    assert(strstr(output, "\"code\":\"gpio_not_allowed\""));
    assert(strstr(output, "\"code\":\"angle_out_of_range\""));
    assert(strstr(output, "\"bench_servo\":{\"enabled\":true,\"channels\":1}"));
    assert(!strstr(output, "\"target_deg\":45"));
    assert(strstr(output, "\"id\":8,\"ok\":false,\"code\":\"emergency_latched\""));
    /* Fresh fixture: the serial shortcut starts the synchronized ramp only when ready. */
    state = ROBOT_IDLE;
    servos = (bench_servo_t){0};
    assert(bench_servo_init(&servos, &servo_driver, &servo_config));
    feed("preparar\n");
    assert(servos.used == 4 && servos.scan_pose_active && state == ROBOT_IDLE);
    feed("MB1 9 estop\npreparar\n");
    assert(state == ROBOT_ERROR);
    bench_servo_stop(&servos);
    assert(!servos.scan_pose_active && servos.latched);
    state = ROBOT_IDLE;
    servos = (bench_servo_t){0};
    assert(bench_servo_init(&servos, &servo_driver, &servo_config));
    feed("IDLE\n");
    assert(servos.used == 5 && servos.scan_pose_active && state == ROBOT_IDLE);
    feed("MB1 10 estop\nIDLE\n");
    assert(state == ROBOT_ERROR);
    bench_servo_stop(&servos);
    assert(!servos.scan_pose_active && servos.latched);
    /* Bind the physical executor: UART uses the same gesture validation as GPIO Start. */
    state = ROBOT_IDLE;
    servos = (bench_servo_t){0};
    assert(bench_servo_init(&servos, &servo_driver, &servo_config));
    base_scan_driver_t base_driver={.write_us=base_write,.shutdown=servo_shutdown};
    base_scan_t scan;
    base_scan_init(&scan,&base_driver);
    serial_control_bind_scan(&scan);
    feed("varredura\n");
    assert(state==ROBOT_RUN_VARREDURA && servos.scan_pose_active); /* No sensor required. */
    sensor_driver_t distance_driver={.request=distance_request,.poll=distance_poll,
        .cancel=distance_cancel,.source=SENSOR_HARDWARE};
    assert(robot_runtime_set_distance_driver(&distance_driver));
    robot_runtime_poll_external(&state,now);
    feed("varredura\n");
    assert(state==ROBOT_RUN_VARREDURA && servos.scan_pose_active);
    feed("MB1 11 confirm\n"); assert(state==ROBOT_RUN_VARREDURA);
    for (int i=0;i<10000 && servos.scan_pose_active;++i) {
        bench_servo_poll(&servos,now); now+=10;
    }
    assert(servos.distance_ready && base_scan_begin(&scan,now));
    sensor_sample_t object={.value=100,.timestamp_ms=now+10,.source=SENSOR_HARDWARE,.received=true,.valid=true};
    now+=10; base_scan_poll(&scan,now,&object,IO_OK);
    while (scan.moving) { now+=10; base_scan_poll(&scan,now,NULL,IO_OK); }
    assert(scan.home && state_machine_process_event(&state,EVENT_SCAN_DONE));
    /* Refresh sensor IO without overwriting the waiting state. */
    robot_runtime_poll_external(&state,now);
    feed("MB1 12 confirm\n");
    assert(state==ROBOT_RUN_DESLOCAMENTO && scan.replaying);
    feed("MB1 13 estop\n"); base_scan_stop(&scan);
    assert(state==ROBOT_ERROR && servos.latched && scan.latched);
    serial_control_bind_scan(NULL);
    puts("PASS: serial integration, init failure, fragmented RX, timeout, saturated queue emergency, diagnostics");
}
