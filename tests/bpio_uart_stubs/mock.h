// Copyright 2026 OpenAI. SPDX-License-Identifier: MIT
#ifndef BPIO_UART_TEST_MOCK_H
#define BPIO_UART_TEST_MOCK_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define __not_in_flash_func(x) x
#define M_UART_PORT 0
#define M_UART_RTS 3
#define UART_IRQ_NUM(x) 0
#define PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY 0
struct mock_uart_hw { uint32_t dr, rsr; };
extern struct mock_uart_hw mock_hw;
extern void (*mock_irq)(void);
extern bool mock_ready;
extern uint32_t mock_time;
extern bool mock_flow;
extern int mock_rts; // last value driven on M_UART_RTS, -1 = never
static inline void bio_put(uint8_t pin, bool v) { if (pin == M_UART_RTS) mock_rts = v; }
static inline bool hwuart_flow_control_enabled(void) { return mock_flow; }
static inline struct mock_uart_hw *uart_get_hw(int p) { (void)p; return &mock_hw; }
static inline bool uart_is_readable(int p) { (void)p; bool r=mock_ready; mock_ready=false; return r; }
static inline void uart_putc_raw(int p, uint8_t b) { (void)p; mock_hw.dr=b; mock_ready=true; mock_irq(); }
static inline uint32_t save_and_disable_interrupts(void) { return 0; }
static inline void restore_interrupts(uint32_t s) { (void)s; }
static inline void irq_add_shared_handler(int n, void (*f)(void), int priority) { (void)n; (void)priority; mock_irq=f; }
static inline void irq_remove_handler(int n, void (*f)(void)) { (void)n; (void)f; mock_irq=NULL; }
static inline void irq_set_enabled(int n, bool e) { (void)n; (void)e; }
static inline void uart_set_irq_enables(int p, bool rx, bool tx) { (void)p; (void)rx; (void)tx; }
static inline uint32_t time_us_32(void) { mock_time+=1000000; return mock_time; }
static inline void tight_loop_contents(void) {}
typedef const uint8_t *flatbuffers_uint8_vec_t;
static inline uint8_t flatbuffers_uint8_vec_at(flatbuffers_uint8_vec_t p, unsigned i) { return p[i]; }
#endif
