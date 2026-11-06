idf_component_register(
    SRCS
        "app_main.c"
        "state_machine.c"
        "button_control.c"
        "led_control.c"
    INCLUDE_DIRS "."
    REQUIRES
        led_strip
        esp_driver_gpio
        esp_timer
)