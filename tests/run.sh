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
