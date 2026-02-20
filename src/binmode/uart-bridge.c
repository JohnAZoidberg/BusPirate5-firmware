/**
 * @file uart-bridge.c
 * @brief Binmode transparent UART bridge on CDC 1
 *
 * Provides a transparent UART bridge on the binary mode CDC port (CDC 1),
 * leaving the terminal (CDC 0) free for commands (GPIO control, etc).
 *
 * Requires UART mode to be active (configures IO4=TX, IO5=RX).
 * Data received on CDC 1 is forwarded to the hardware UART TX.
 * Data received on the hardware UART RX is forwarded to CDC 1.
 */

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pirate.h"
#include "system_config.h"
#include "usb_rx.h"
#include "usb_tx.h"
#include "ui/ui_term.h"
#include "pirate/button.h"
#include "binmode/binmodes.h"
#include "modes.h"

const char uart_bridge_name[] = "Transparent UART bridge";

void uart_bridge_setup(void) {
}

void uart_bridge_setup_message(void) {
    printf("%sUART bridge active on CDC 1 (binmode port)%s\r\n",
           ui_term_color_info(), ui_term_color_reset());
    printf("%sPress button or use 'binmode' command to exit%s\r\n",
           ui_term_color_info(), ui_term_color_reset());
}

void uart_bridge_service(void) {
    if (button_get(0)) {
        binmode_cleanup();
        system_config.binmode_select = 0;
        binmode_setup();
        system_config.mode = 0;
        modes[system_config.mode].protocol_setup();
        return;
    }

    char c;

    /* CDC 1 (host) -> UART TX (target) */
    while (bin_rx_fifo_try_get(&c)) {
        uart_putc_raw(M_UART_PORT, c);
    }

    /* UART RX (target) -> CDC 1 (host) */
    while (uart_is_readable(M_UART_PORT)) {
        c = uart_getc(M_UART_PORT);
        bin_tx_fifo_put(c);
    }
}

void uart_bridge_cleanup(void) {
}
