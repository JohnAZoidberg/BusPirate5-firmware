// Copyright 2026 OpenAI. SPDX-License-Identifier: MIT
// Modified by Daniel Schaefer and Claude, 2026: independent UART channels.
// Build: cc -std=c11 -Wall -I tests/bpio_uart_stubs tests/test_bpio_uart.c -o /tmp/test_bpio_uart
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bpio_uart_stubs/mock.h"
struct mock_system_config system_config;
struct mock_uart mock_uarts[2];
uint32_t mock_time;
bool mock_tx_enabled = true;
uint32_t mock_ch0_applies;
bool mock_buf_out[8];
uint8_t mock_func[8];
const uint8_t bio2bufiopin[8] = { 8, 9, 10, 11, 12, 13, 14, 15 };
#include "../src/binmode/bpio_uart.c"

// hwuart.c side of the channel 0 interface.
bool hwuart_tx_enabled(void) { return mock_tx_enabled; }
void hwuart_bpio_channel0_apply(const bpio_uart_channel_config_t *cfg) {
    mock_ch0_applies++;
    mock_tx_enabled = cfg->tx_enable;
    mock_buf_out[M_UART_TX] = cfg->tx_enable;
}

static const bpio_uart_channel_config_t rx_only = { .enable = true, .speed = 115200, .data_bits = 8, .stop_bits = 1 };

static void legacy_channel0_tests(void) {
    uint8_t tx[512], rx[512];
    mock_uarts[0].loopback = true;
    for (unsigned i=0;i<sizeof(tx);i++) tx[i]=(uint8_t)(i*37);
    bpio_hwuart_rx_start();
    struct bpio_data_request_t request={.bytes_write=512, .bytes_read=512};
    // All RX arrives during TX, before foreground starts reading.
    assert(!bpio_hwuart_transaction(&request, tx, rx));
    assert(!memcmp(tx,rx,sizeof(tx)));
    request.bytes_read=0;
    // Exercise ring wrap repeatedly and async packet splitting.
    for (unsigned repeat=0;repeat<40;repeat++) {
        assert(!bpio_hwuart_transaction(&request,tx,rx));
        assert(bpio_hwuart_async_handler(rx)==512);
        assert(!memcmp(tx,rx,sizeof(tx)));
    }
    assert(!bpio_hwuart_rx_error());
    request.bytes_write=0; request.bytes_read=1;
    assert(bpio_hwuart_transaction(&request,tx,rx)); // bounded timeout
    assert(!bpio_hwuart_rx_error()); // timeout is not a UART error
    for (unsigned i=0;i<4096;i++) mock_inject(0, i & 255);
    assert(strstr(bpio_hwuart_rx_error(), "overflow=1"));
    uint8_t ch=9; const char *err=NULL;
    assert(bpio_hwuart_async_next(rx, &ch, &err)==0 && ch==0 && err); // one fault report
    assert(bpio_hwuart_async_next(rx, &ch, &err)==0 && !err); // no error-packet flood
    assert(bpio_hwuart_async_handler(rx)==0); // damaged stream quarantined
    assert(bpio_hwuart_transaction(&request,tx,rx)); // fault stays latched
    bpio_hwuart_rx_start(); // configuration clears stream and counters
    assert(!bpio_hwuart_rx_error());
    assert(bpio_hwuart_async_handler(rx)==0);
    mock_inject(0, 0xf00u | 42);
    const char *error=bpio_hwuart_rx_error();
    assert(error && strstr(error,"overrun=1 framing=1 parity=1 break=1"));
    assert(bpio_hwuart_async_handler(rx)==0);
    bpio_hwuart_rx_stop();
    bpio_hwuart_rx_stop(); // idempotent cleanup
    assert(!mock_uarts[0].irq);
    mock_uarts[0].loopback = false;
}

static void channel_tests(void) {
    uint8_t rx[512], tx[4] = { 'a', 'b', 'c', 'd' };
    uint8_t ch; const char *err;
    struct bpio_data_request_t request = { .bytes_write = 0, .bytes_read = 0 };

    // Mode entry with channel 0 receive only.
    mock_tx_enabled = false;
    bpio_hwuart_rx_start();
    assert(!bpio_hwuart_channel_configure(0, &rx_only));
    assert(mock_ch0_applies == 1 && !mock_buf_out[M_UART_TX]);

    // Invalid channels and inactive channel 1.
    assert(bpio_hwuart_channel_configure(2, &rx_only));
    assert(strstr(bpio_hwuart_channel_error(1), "not enabled"));
    assert(bpio_hwuart_channel_transaction(1, &request, tx, rx));
    assert(strstr(bpio_hwuart_transaction_error(1, &request), "not enabled"));
    assert(bpio_hwuart_channel_transaction(7, &request, tx, rx));
    bpio_uart_channel_config_t off = rx_only; off.enable = false;
    assert(strstr(bpio_hwuart_channel_configure(0, &off), "leaving UART mode"));

    // Bad framing is rejected before touching hardware.
    bpio_uart_channel_config_t bad = rx_only; bad.data_bits = 9;
    assert(bpio_hwuart_channel_configure(1, &bad) && !mock_uarts[1].initialized);

    // Terminal UART and pin conflicts.
    system_config.terminal_uart_enable = 1; system_config.terminal_uart_number = 1;
    assert(strstr(bpio_hwuart_channel_configure(1, &rx_only), "debug/terminal"));
    system_config.terminal_uart_enable = 0;
    system_config.pin_func[BP_DEBUG_UART_1_RX + 1] = BP_PIN_MODE;
    assert(strstr(bpio_hwuart_channel_configure(1, &rx_only), "in use"));
    system_config.pin_func[BP_DEBUG_UART_1_RX + 1] = BP_PIN_IO;

    // Channel 1 receive only: TX buffer stays an input and is not a UART pin.
    assert(!bpio_hwuart_channel_configure(1, &rx_only));
    assert(mock_uarts[1].initialized && mock_uarts[1].irq);
    assert(!mock_buf_out[BP_DEBUG_UART_1_TX] && mock_func[BP_DEBUG_UART_1_TX] == GPIO_FUNC_SIO);
    assert(mock_func[BP_DEBUG_UART_1_RX] == GPIO_FUNC_UART);
    assert(bpio_hwuart_channel_pins_claimed(1u << BP_DEBUG_UART_1_RX));
    assert(!bpio_hwuart_channel_pins_claimed(1u << 3));

    // TX refused on both receive-only channels.
    request.bytes_write = 4;
    assert(bpio_hwuart_channel_transaction(1, &request, tx, rx));
    assert(strstr(bpio_hwuart_transaction_error(1, &request), "TX disabled"));
    assert(bpio_hwuart_channel_transaction(0, &request, tx, rx));
    request.bytes_write = 0;

    // Routing: data stays on its channel.
    mock_inject(1, 'S'); mock_inject(0, 'E'); mock_inject(1, 'T');
    request.bytes_read = 2;
    assert(!bpio_hwuart_channel_transaction(1, &request, tx, rx) && rx[0] == 'S' && rx[1] == 'T');
    request.bytes_read = 1;
    assert(!bpio_hwuart_channel_transaction(0, &request, tx, rx) && rx[0] == 'E');
    request.bytes_read = 0;

    // Fairness: a channel that always has data cannot starve the other.
    unsigned served[2] = { 0, 0 };
    for (unsigned round = 0; round < 10; round++) {
        mock_inject(0, 'e'); mock_inject(1, 's');
        uint32_t n = bpio_hwuart_async_next(rx, &ch, &err);
        assert(n >= 1 && !err);
        served[ch]++;
        for (uint32_t i = 0; i < n; i++) assert(rx[i] == (ch ? 's' : 'e'));
    }
    assert(served[0] && served[1]);
    while (bpio_hwuart_async_next(rx, &ch, &err)) {}

    // Enabling TX on channel 1 drives it; channel 0 is untouched.
    uint32_t ch0_inits = mock_uarts[0].inits;
    bpio_uart_channel_config_t txon = rx_only; txon.tx_enable = true; txon.speed = 9600;
    assert(!bpio_hwuart_channel_configure(1, &txon));
    assert(mock_buf_out[BP_DEBUG_UART_1_TX] && mock_func[BP_DEBUG_UART_1_TX] == GPIO_FUNC_UART);
    assert(mock_uarts[1].baud == 9600 && mock_uarts[0].inits == ch0_inits && mock_uarts[0].irq);
    mock_uarts[1].loopback = true;
    request.bytes_write = 4; request.bytes_read = 4;
    assert(!bpio_hwuart_channel_transaction(1, &request, tx, rx) && !memcmp(tx, rx, 4));
    request.bytes_write = 0; request.bytes_read = 0;
    mock_uarts[1].loopback = false;

    // A fault on channel 1 is reported once, tagged, and does not affect channel 0.
    mock_inject(1, 0x400u); // break
    mock_inject(0, 'k');
    bool saw_fault = false, saw_data = false;
    for (unsigned i = 0; i < 4; i++) {
        uint32_t n = bpio_hwuart_async_next(rx, &ch, &err);
        if (err) { assert(ch == 1 && n == 0 && strstr(err, "channel=1") && strstr(err, "break=1")); saw_fault = true; }
        if (n) { assert(ch == 0 && rx[0] == 'k'); saw_data = true; }
    }
    assert(saw_fault && saw_data);
    assert(!bpio_hwuart_channel_error(0));
    // Reconfiguring channel 1 clears its fault only.
    assert(!bpio_hwuart_channel_configure(1, &rx_only));
    assert(!bpio_hwuart_channel_error(1) && !mock_buf_out[BP_DEBUG_UART_1_TX]);

    // Releasing channel 1 frees its pins and keeps channel 0 running.
    assert(!bpio_hwuart_channel_configure(1, &off));
    assert(!mock_uarts[1].initialized && !mock_uarts[1].irq && mock_uarts[0].irq);
    assert(system_config.pin_func[BP_DEBUG_UART_1_TX + 1] == BP_PIN_IO);
    assert(!bpio_hwuart_channel_pins_claimed(0xff));
    assert(!bpio_hwuart_channel_configure(1, &off)); // idempotent

    // Mode cleanup releases everything.
    assert(!bpio_hwuart_channel_configure(1, &rx_only));
    bpio_hwuart_channels_release();
    assert(!mock_uarts[0].irq && !mock_uarts[1].irq && !mock_uarts[1].initialized);
    assert(system_config.pin_func[BP_DEBUG_UART_1_RX + 1] == BP_PIN_IO);
}

// UART mode entered from the terminal (`m uart`) never ran BPIO mode entry.
static void rx_ensure_tests(void) {
    uint8_t rx[512];
    bpio_hwuart_channels_release();
    mock_tx_enabled = true;
    mock_uarts[0].loopback = false;

    system_config.terminal_uart_enable = 1;
    system_config.terminal_uart_number = 0;
    bpio_hwuart_rx_ensure();
    assert(!channels[0].rx_active && !mock_uarts[0].irq); // terminal owns UART0
    assert(!strcmp(bpio_hwuart_channel_error(0), "UART channel not enabled"));
    system_config.terminal_uart_enable = 0;

    bpio_hwuart_rx_ensure();
    assert(channels[0].rx_active && mock_uarts[0].irq && channels[0].tx_enabled);
    mock_inject(0, 'a');
    mock_inject(0, 'b');
    bpio_hwuart_rx_ensure(); // already running: must not reset the ring
    assert(bpio_hwuart_async_handler(rx) == 2 && rx[0] == 'a' && rx[1] == 'b');
    bpio_hwuart_channels_release();
}

int main(void) {
    legacy_channel0_tests();
    channel_tests();
    rx_ensure_tests();
    puts("BPIO UART channel tests passed");
    return 0;
}
