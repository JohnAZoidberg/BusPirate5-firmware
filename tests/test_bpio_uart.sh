#!/bin/sh
# Host test for the BPIO UART IRQ ring (src/binmode/bpio_uart.c) using mocks.
set -e
cd "$(dirname "$0")"
gcc -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all \
    -Ibpio_uart_stubs -I../src/binmode -o test_bpio_uart test_bpio_uart.c
ASAN_OPTIONS=detect_leaks=0 ./test_bpio_uart
