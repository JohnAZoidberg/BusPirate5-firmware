// Copyright 2026 OpenAI. SPDX-License-Identifier: MIT
// Modified 2026: on-demand start, bulk reads, RTS flow control, stop handoff.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bpio_uart_stubs/mock.h"
struct mock_uart_hw mock_hw;
void (*mock_irq)(void);
bool mock_ready;
uint32_t mock_time;
bool mock_flow;
int mock_rts = -1;
#include "../src/binmode/bpio_uart.c"

static void inject(uint32_t dr) {
    mock_hw.dr=dr; mock_ready=true; mock_irq();
}
int main(void) {
    uint8_t tx[512], rx[512];
    for (unsigned i=0;i<sizeof(tx);i++) tx[i]=(uint8_t)(i*37);
    struct bpio_data_request_t request={.bytes_write=512, .bytes_read=512};
    // No explicit start (UART mode chosen from the terminal): the first BPIO
    // request installs the IRQ itself. All RX arrives during TX.
    assert(!bpio_hwuart_rx_active());
    assert(!bpio_hwuart_transaction(&request, tx, rx));
    assert(bpio_hwuart_rx_active() && mock_irq);
    assert(!memcmp(tx,rx,sizeof(tx)));
    request.bytes_read=0;
    // Exercise ring wrap repeatedly and async packet splitting.
    for (unsigned repeat=0;repeat<40;repeat++) {
        assert(!bpio_hwuart_transaction(&request,tx,rx));
        assert(bpio_hwuart_async_handler(rx)==512);
        assert(!memcmp(tx,rx,sizeof(tx)));
    }
    // Terminal-side non-blocking reads share the ring in order.
    request.bytes_write=3;
    assert(!bpio_hwuart_transaction(&request,tx,rx));
    uint8_t b;
    assert(bpio_hwuart_rx_read(&b,1)==1 && b==tx[0]);
    assert(bpio_hwuart_async_handler(rx)==2 && rx[0]==tx[1] && rx[1]==tx[2]);
    assert(!bpio_hwuart_rx_error());
    request.bytes_write=0; request.bytes_read=1;
    assert(bpio_hwuart_transaction(&request,tx,rx)); // bounded timeout
    assert(!bpio_hwuart_rx_error()); // timeout is not a UART error
    // Bytes drained before a fault are returned; the fault blocks later reads.
    inject(7);
    assert(bpio_hwuart_async_handler(rx)==1 && rx[0]==7);
    for (unsigned i=0;i<4096;i++) inject(i & 255);
    assert(strstr(bpio_hwuart_rx_error(), "overflow=1"));
    assert(bpio_hwuart_rx_async_error());
    assert(!bpio_hwuart_rx_async_error()); // no error-packet flood
    assert(bpio_hwuart_async_handler(rx)==0); // damaged stream quarantined
    assert(bpio_hwuart_rx_read(&b,1)==0);
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
    assert(!mock_irq && !bpio_hwuart_rx_active());
    assert(mock_rts==-1); // RTS untouched without flow control
    assert(bpio_hwuart_rx_read(&b,1)==0); // inactive ring yields nothing

    // Flow control: RTS asserted (0) on start, deasserted at the high-water
    // mark, reasserted once drained below low water, released on stop.
    mock_flow=true;
    bpio_hwuart_rx_start();
    assert(mock_rts==0);
    for (unsigned i=0;i<RX_HIGH_WATER-1;i++) inject(i & 255);
    assert(mock_rts==0);
    inject(0);
    assert(mock_rts==1);
    uint8_t big[RX_HIGH_WATER];
    assert(bpio_hwuart_rx_read(big, RX_HIGH_WATER-RX_LOW_WATER-1)==RX_HIGH_WATER-RX_LOW_WATER-1);
    assert(mock_rts==1);
    assert(bpio_hwuart_rx_read(big,1)==1);
    assert(mock_rts==0);
    assert(!bpio_hwuart_rx_error());
    bpio_hwuart_rx_stop();
    assert(mock_rts==1);
    puts("BPIO UART buffered RX tests passed");
    return 0;
}
