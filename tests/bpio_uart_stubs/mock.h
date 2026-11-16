// Copyright 2026 OpenAI. SPDX-License-Identifier: MIT
// Modified by Daniel Schaefer and Claude, 2026: two UARTs, pin and buffer state.
#ifndef BPIO_UART_TEST_MOCK_H
#define BPIO_UART_TEST_MOCK_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define __not_in_flash_func(x) x
typedef void (*irq_handler_t)(void);
#define uart_inst_t void
#define M_UART_PORT ((uart_inst_t *)0)
#define BP_DEBUG_UART_1 ((uart_inst_t *)1)
#define M_UART_TX 4
#define M_UART_RX 5
#define BP_DEBUG_UART_1_TX 0
#define BP_DEBUG_UART_1_RX 1
#define UART_IRQ_NUM(x) ((int)(intptr_t)(x))
#define PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY 0
enum { UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD };
enum { GPIO_OVERRIDE_NORMAL, GPIO_OVERRIDE_INVERT };
enum { GPIO_FUNC_SIO = 5, GPIO_FUNC_UART = 2 };
enum bp_pin_func { BP_PIN_IO, BP_PIN_MODE, BP_PIN_DEBUG };
#define HW_PINS 10
struct mock_system_config {
    enum bp_pin_func pin_func[HW_PINS];
    uint32_t terminal_uart_enable, terminal_uart_number, debug_uart_enable, debug_uart_number;
};
extern struct mock_system_config system_config;

struct mock_uart_hw { uint32_t dr, rsr; };
struct mock_uart {
    struct mock_uart_hw hw;
    bool ready, initialized, loopback;
    uint32_t baud, inits;
    irq_handler_t irq;
};
extern struct mock_uart mock_uarts[2];
extern uint32_t mock_time;
extern bool mock_tx_enabled; // hwuart channel 0 TX state
extern uint32_t mock_ch0_applies;
// Per BIO pin: buffer drives outward, and GPIO function.
extern bool mock_buf_out[8];
extern uint8_t mock_func[8];
extern const uint8_t bio2bufiopin[8];

#define MU(p) (&mock_uarts[(intptr_t)(p)])
static inline struct mock_uart_hw *uart_get_hw(uart_inst_t *p) { return &MU(p)->hw; }
static inline uint32_t uart_get_index(uart_inst_t *p) { return (uint32_t)(intptr_t)p; }
static inline bool uart_is_readable(uart_inst_t *p) { bool r=MU(p)->ready; MU(p)->ready=false; return r; }
static inline void mock_inject(int port, uint32_t dr) {
    mock_uarts[port].hw.dr=dr; mock_uarts[port].ready=true;
    if (mock_uarts[port].irq) mock_uarts[port].irq();
}
static inline void uart_putc_raw(uart_inst_t *p, uint8_t b) { if (MU(p)->loopback) mock_inject((int)(intptr_t)p, b); }
static inline uint32_t uart_init(uart_inst_t *p, uint32_t baud) { MU(p)->initialized=true; MU(p)->baud=baud; MU(p)->inits++; return baud; }
static inline void uart_deinit(uart_inst_t *p) { MU(p)->initialized=false; }
static inline void uart_set_format(uart_inst_t *p, int d, int s, int par) { (void)p; (void)d; (void)s; (void)par; }
static inline void uart_set_hw_flow(uart_inst_t *p, bool a, bool b) { (void)p; (void)a; (void)b; }
static inline void uart_set_fifo_enabled(uart_inst_t *p, bool e) { (void)p; (void)e; }
static inline uint32_t save_and_disable_interrupts(void) { return 0; }
static inline void restore_interrupts(uint32_t s) { (void)s; }
static inline void irq_add_shared_handler(int n, irq_handler_t f, int priority) { (void)priority; mock_uarts[n].irq=f; }
static inline void irq_remove_handler(int n, irq_handler_t f) { (void)f; mock_uarts[n].irq=NULL; }
static inline void irq_set_enabled(int n, bool e) { (void)n; (void)e; }
static inline void uart_set_irq_enables(uart_inst_t *p, bool rx, bool tx) { (void)p; (void)rx; (void)tx; }
static inline uint32_t time_us_32(void) { mock_time+=1000000; return mock_time; }
static inline void tight_loop_contents(void) {}
static inline void gpio_set_outover(uint32_t g, int v) { (void)g; (void)v; }
static inline void gpio_set_inover(uint32_t g, int v) { (void)g; (void)v; }
static inline void bio_buf_output(uint8_t bio) { mock_buf_out[bio]=true; }
static inline void bio_buf_input(uint8_t bio) { mock_buf_out[bio]=false; }
static inline void bio_input(uint8_t bio) { mock_buf_out[bio]=false; }
static inline void bio_set_function(uint8_t bio, uint8_t f) { mock_func[bio]=f; }
static inline void system_bio_update_purpose_and_label(bool en, uint8_t bio, enum bp_pin_func f, const char *l) {
    (void)l; system_config.pin_func[bio+1] = en ? f : BP_PIN_IO;
}
typedef const uint8_t *flatbuffers_uint8_vec_t;
static inline uint8_t flatbuffers_uint8_vec_at(flatbuffers_uint8_vec_t p, unsigned i) { return p[i]; }
#endif
