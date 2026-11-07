idf_component_register(
    SRCS
        "app_main.c"
        "bench_control.c"
        "serial_protocol.c"
        "command_control.c"
        "serial_control.c"
        "state_machine.c"
        "button_control.c"
        "led_control.c"
    INCLUDE_DIRS "."
    REQUIRES
        led_strip
        esp_driver_gpio
        esp_timer
        esp_driver_usb_serial_jtag
)
