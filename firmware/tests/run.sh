#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d /tmp/magnibot-tests.XXXXXX)
trap 'rm -rf "$test_dir"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Itests/stubs -Imain tests/test_button.c main/button_control.c \
    -o "$test_dir/test_button"
"$test_dir/test_button"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Imain tests/test_state.c main/state_machine.c main/bench_control.c \
    -o "$test_dir/test_state"
"$test_dir/test_state"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Imain tests/test_protocol.c main/serial_protocol.c main/command_control.c \
    main/bench_control.c main/state_machine.c main/bench_servo.c main/gpio_policy.c main/cycle_sequence.c \
    main/motion_model.c main/sensor_data.c -lm -o "$test_dir/test_protocol"
"$test_dir/test_protocol"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Itests/stubs -Imain tests/test_serial_control.c main/serial_control.c \
    main/serial_protocol.c main/command_control.c main/bench_control.c \
    main/state_machine.c main/robot_runtime.c main/motion_model.c main/sensor_data.c \
    main/cycle_sequence.c main/hardware_io.c main/sensor_interface.c \
    main/actuator_interface.c main/bench_servo.c main/gpio_policy.c -lm -o "$test_dir/test_serial_control"
"$test_dir/test_serial_control"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Imain tests/test_runtime.c main/robot_runtime.c main/motion_model.c main/sensor_data.c \
    main/cycle_sequence.c main/hardware_io.c main/sensor_interface.c \
    main/actuator_interface.c main/state_machine.c -lm -o "$test_dir/test_runtime"
"$test_dir/test_runtime"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Imain tests/test_gpio_policy.c main/gpio_policy.c -o "$test_dir/test_gpio_policy"
"$test_dir/test_gpio_policy"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Imain tests/test_motion.c main/motion_model.c main/sensor_data.c \
    -lm -o "$test_dir/test_motion"
"$test_dir/test_motion"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Itests/stubs -Imain tests/test_cycle.c main/motion_model.c main/sensor_data.c \
    main/cycle_sequence.c main/command_control.c main/bench_control.c main/state_machine.c \
    main/bench_servo.c main/gpio_policy.c main/serial_protocol.c main/button_control.c -lm -o "$test_dir/test_cycle"
"$test_dir/test_cycle"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Imain tests/test_interfaces.c main/hardware_io.c main/sensor_interface.c \
    main/actuator_interface.c main/motion_model.c main/sensor_data.c \
    -lm -o "$test_dir/test_interfaces"
"$test_dir/test_interfaces"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Imain tests/test_bench_servo.c main/bench_servo.c main/gpio_policy.c \
    -lm -o "$test_dir/test_bench_servo"
"$test_dir/test_bench_servo"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g -Imain \
    tests/test_base_motion.c main/base_motion.c -lm -o "$test_dir/test_base_motion"
"$test_dir/test_base_motion"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g -Imain \
    tests/test_tof_acquisition.c main/tof_acquisition.c -o "$test_dir/test_tof"
"$test_dir/test_tof"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g -Imain \
    -DCONFIG_MAGNIBOT_SHOULDER_MIN_US=500 -DCONFIG_MAGNIBOT_SHOULDER_MAX_US=2500 \
    tests/test_shoulder_calibration.c main/bench_servo.c main/gpio_policy.c \
    -lm -o "$test_dir/test_shoulder_calibration"
"$test_dir/test_shoulder_calibration"
