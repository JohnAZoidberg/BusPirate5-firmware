// Modified by OpenAI, 2026: interrupt-buffered BPIO UART receive.
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pirate.h"
#include "pirate/bio.h"
#include "bpio_uart.h"
#include "bpio_reader.h"
#include "bpio_transactions.h"

#define BPIO_MAX_READ_SIZE 512
#define RX_RING_SIZE 4096u
#define RX_RING_MASK (RX_RING_SIZE - 1u)
// With flow control, deassert RTS well before the ring fills so a remote that
// is mid-FIFO can still finish, and reassert once half of the ring is free.
#define RX_HIGH_WATER (RX_RING_SIZE - 512u)
#define RX_LOW_WATER (RX_RING_SIZE / 2u)

// IRQ producer and foreground consumer run on the same core. Foreground
// accesses mask interrupts, so indices and fault snapshots cannot race.
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint32_t rx_head, rx_tail;
static volatile uint32_t rx_overflow, rx_overrun, rx_framing, rx_parity, rx_break;
static volatile bool rx_fault;
static bool rx_active, rx_fault_reported;
static bool rx_flow, rx_rts_ready;
static char rx_error_text[160];

// RTS is active low: 0 = ready to receive, 1 = not ready.
static inline void rx_set_rts(bool ready) {
    if (rx_flow && ready != rx_rts_ready) {
        rx_rts_ready = ready;
        bio_put(M_UART_RTS, !ready);
    }
}

static void __not_in_flash_func(bpio_uart_rx_irq)(void) {
    while (uart_is_readable(M_UART_PORT)) {
        uint32_t dr = uart_get_hw(M_UART_PORT)->dr;
        // PL011 DR[11:8] = overrun, break, parity, framing errors.
        if (dr & (1u << 11)) rx_overrun++;
        if (dr & (1u << 10)) rx_break++;
        if (dr & (1u << 9)) rx_parity++;
        if (dr & (1u << 8)) rx_framing++;
        if (dr & 0xf00u) rx_fault = true;
        uint32_t next = (rx_head + 1u) & RX_RING_MASK;
        if (next == rx_tail) {
            rx_overflow++;
            rx_fault = true;
        }
        if (!rx_fault) {
            rx_ring[rx_head] = (uint8_t)dr;
            rx_head = next;
            if (((next - rx_tail) & RX_RING_MASK) >= RX_HIGH_WATER) rx_set_rts(false);
        }
    }
    // Clear sticky UART error status after consuming the FIFO.
    uart_get_hw(M_UART_PORT)->rsr = 0;
}

void bpio_hwuart_rx_start(void) {
    bpio_hwuart_rx_stop();
    uint32_t saved = save_and_disable_interrupts();
    rx_head = rx_tail = 0;
    rx_overflow = rx_overrun = rx_framing = rx_parity = rx_break = 0;
    rx_fault = false;
    rx_fault_reported = false;
    rx_flow = hwuart_flow_control_enabled();
    rx_rts_ready = false;
    uart_get_hw(M_UART_PORT)->rsr = 0;
    // Install only on the BPIO core; mode selection rejects a debug terminal
    // using this peripheral on the other core.
    irq_add_shared_handler(UART_IRQ_NUM(M_UART_PORT), bpio_uart_rx_irq, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    rx_active = true;
    uart_set_irq_enables(M_UART_PORT, true, false);
    irq_set_enabled(UART_IRQ_NUM(M_UART_PORT), true);
    rx_set_rts(true);
    restore_interrupts(saved);
}

void bpio_hwuart_rx_stop(void) {
    if (!rx_active) return;
    uint32_t saved = save_and_disable_interrupts();
    uart_set_irq_enables(M_UART_PORT, false, false);
    irq_remove_handler(UART_IRQ_NUM(M_UART_PORT), bpio_uart_rx_irq);
    // Hand RTS back in the terminal's idle state (not ready).
    rx_set_rts(false);
    rx_active = false;
    restore_interrupts(saved);
}

bool bpio_hwuart_rx_active(void) {
    return rx_active;
}

// BPIO owns RX from its first UART request until UART cleanup, BPIO binmode
// cleanup, or another RX consumer (such as the bridge) takes the peripheral.
// Starting on demand covers UART mode selected from the terminal as well.
static inline void rx_ensure(void) {
    if (!rx_active) bpio_hwuart_rx_start();
}

const char *bpio_hwuart_rx_error(void) {
    uint32_t saved = save_and_disable_interrupts();
    bool fault = rx_fault;
    uint32_t overflow = rx_overflow, overrun = rx_overrun;
    uint32_t framing = rx_framing, parity = rx_parity, breaks = rx_break;
    restore_interrupts(saved);
    if (!fault) return NULL;
    snprintf(rx_error_text, sizeof(rx_error_text),
             "UART RX fault: overflow=%lu overrun=%lu framing=%lu parity=%lu break=%lu; reconfigure UART",
             (unsigned long)overflow, (unsigned long)overrun, (unsigned long)framing,
             (unsigned long)parity, (unsigned long)breaks);
    return rx_error_text;
}

const char *bpio_hwuart_rx_async_error(void) {
    const char *error = bpio_hwuart_rx_error();
    if (!error || rx_fault_reported) return NULL;
    rx_fault_reported = true;
    return error;
}

uint32_t bpio_hwuart_rx_read(uint8_t *dst, uint32_t max) {
    uint32_t n = 0;
    uint32_t saved = save_and_disable_interrupts();
    if (rx_active && !rx_fault) {
        uint32_t tail = rx_tail;
        uint32_t avail = (rx_head - tail) & RX_RING_MASK;
        n = avail < max ? avail : max;
        uint32_t first = RX_RING_SIZE - tail;
        if (first > n) first = n;
        memcpy(dst, &rx_ring[tail], first);
        memcpy(dst + first, rx_ring, n - first);
        rx_tail = (tail + n) & RX_RING_MASK;
        if (avail - n <= RX_LOW_WATER) rx_set_rts(true);
    }
    restore_interrupts(saved);
    return n;
}

uint32_t bpio_hwuart_transaction(struct bpio_data_request_t *request,
                                flatbuffers_uint8_vec_t data_write, uint8_t *data_read) {
    rx_ensure();
    if (rx_fault) return true;
    for (uint32_t i = 0; i < request->bytes_write; i++) {
        uart_putc_raw(M_UART_PORT, flatbuffers_uint8_vec_at(data_write, i));
        if (rx_fault) return true;
    }
    uint32_t got = 0;
    uint32_t started = time_us_32();
    while (got < request->bytes_read) {
        uint32_t n = bpio_hwuart_rx_read(&data_read[got], request->bytes_read - got);
        if (n) {
            got += n;
            started = time_us_32();
            continue;
        }
        // 1 s without a new byte; same per-byte timeout as before.
        if (rx_fault || time_us_32() - started >= 1000000u) return true;
        tight_loop_contents();
    }
    return rx_fault;
}

uint32_t bpio_hwuart_async_handler(uint8_t *data_read) {
    rx_ensure();
    return bpio_hwuart_rx_read(data_read, BPIO_MAX_READ_SIZE);
}
