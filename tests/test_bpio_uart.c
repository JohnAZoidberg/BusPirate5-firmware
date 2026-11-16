// Copyright 2026 OpenAI. SPDX-License-Identifier: MIT
#include <assert.h>
#include <string.h>
#include "bpio_uart_stubs/mock.h"
struct mock_uart_hw mock_hw;
void (*mock_irq)(void);
bool mock_ready;
uint32_t mock_time;
#include "../src/binmode/bpio_uart.c"

static void inject(uint32_t dr) {
    mock_hw.dr=dr; mock_ready=true; mock_irq();
}
int main(void) {
    uint8_t tx[512], rx[512];
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
    for (unsigned i=0;i<4096;i++) inject(i & 255);
    assert(strstr(bpio_hwuart_rx_error(), "overflow=1"));
    assert(bpio_hwuart_rx_async_error());
    assert(!bpio_hwuart_rx_async_error()); // no error-packet flood
    assert(bpio_hwuart_async_handler(rx)==0); // damaged stream quarantined
    assert(bpio_hwuart_transaction(&request,tx,rx)); // fault stays latched
    bpio_hwuart_rx_start(); // configuration clears stream and counters
    assert(!bpio_hwuart_rx_error());
    assert(bpio_hwuart_async_handler(rx)==0);
    inject(0xf00u | 42);
    const char *error=bpio_hwuart_rx_error();
    assert(error && strstr(error,"overrun=1 framing=1 parity=1 break=1"));
    assert(bpio_hwuart_async_handler(rx)==0);
    bpio_hwuart_rx_stop();
    bpio_hwuart_rx_stop(); // idempotent cleanup
    assert(!mock_irq);
    puts("BPIO UART buffered RX tests passed");
    return 0;
}
