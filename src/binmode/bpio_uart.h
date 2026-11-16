/**
 * Modified by OpenAI, 2026: buffered RX lifecycle and errors.
 * @file bpio_uart.h
 * @brief BPIO UART protocol transaction handler.
 * @details Provides binary protocol UART transaction and async handling.
 */

#ifndef BPIO_UART_H
#define BPIO_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "bpio_reader.h"

// Forward declaration of the request structure (defined in bpio_transactions.h)
struct bpio_data_request_t;

/**
 * @brief UART transaction handler for BPIO.
 * @param request     Transaction request structure
 * @param data_write  Data to transmit
 * @param data_read   Buffer for received data
 * @return            Number of bytes received
 */
uint32_t bpio_hwuart_transaction(struct bpio_data_request_t *request, flatbuffers_uint8_vec_t data_write, uint8_t *data_read);

/**
 * @brief UART async handler for BPIO (unsolicited incoming data).
 * @param data_read  Buffer for received data
 * @return           Number of bytes received
 */
uint32_t bpio_hwuart_async_handler(uint8_t *data_read);

// BPIO starts IRQ RX on its first UART request and owns it until
// bpio_hwuart_rx_stop() (UART cleanup, BPIO binmode cleanup, or another RX
// consumer taking the peripheral). Faults latch until the next start.
void bpio_hwuart_rx_start(void);
void bpio_hwuart_rx_stop(void);
bool bpio_hwuart_rx_active(void);
const char *bpio_hwuart_rx_error(void);
const char *bpio_hwuart_rx_async_error(void);

/**
 * @brief Non-blocking copy of up to max buffered RX bytes.
 * @details Lets terminal UART reads share the ring while BPIO owns RX, since
 *          the IRQ empties the hardware FIFO. Returns 0 when inactive or faulted.
 */
uint32_t bpio_hwuart_rx_read(uint8_t *dst, uint32_t max);

/** @brief Whether UART mode uses RTS/CTS flow control (defined in mode/hwuart.c). */
bool hwuart_flow_control_enabled(void);

#endif // BPIO_UART_H
