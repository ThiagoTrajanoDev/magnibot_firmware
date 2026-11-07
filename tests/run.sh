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
    main/bench_control.c main/state_machine.c -o "$test_dir/test_protocol"
"$test_dir/test_protocol"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Itests/stubs -Imain tests/test_serial_control.c main/serial_control.c \
    main/serial_protocol.c main/command_control.c main/bench_control.c \
    main/state_machine.c main/robot_runtime.c main/motion_model.c main/sensor_data.c \
    main/cycle_sequence.c main/hardware_io.c main/sensor_interface.c \
    main/actuator_interface.c -lm -o "$test_dir/test_serial_control"
"$test_dir/test_serial_control"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Imain tests/test_motion.c main/motion_model.c main/sensor_data.c \
    -lm -o "$test_dir/test_motion"
"$test_dir/test_motion"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Imain tests/test_cycle.c main/motion_model.c main/sensor_data.c \
    main/cycle_sequence.c -lm -o "$test_dir/test_cycle"
"$test_dir/test_cycle"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g \
    -Imain tests/test_interfaces.c main/hardware_io.c main/sensor_interface.c \
    main/actuator_interface.c main/motion_model.c main/sensor_data.c \
    -lm -o "$test_dir/test_interfaces"
"$test_dir/test_interfaces"
