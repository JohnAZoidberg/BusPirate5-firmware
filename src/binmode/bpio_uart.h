/**
 * Modified by OpenAI, 2026: buffered RX lifecycle and errors.
 * Modified by Daniel Schaefer and Claude, 2026: independent hardware UART channels.
 * @file bpio_uart.h
 * @brief BPIO UART protocol transaction handler.
 * @details Provides binary protocol UART transaction and async handling.
 *          Channel 0 is the UART mode peripheral; channel 1 is the second
 *          hardware UART, configured with ConfigurationRequest.uart_channels.
 */

#ifndef BPIO_UART_H
#define BPIO_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "bpio_reader.h"

#define BPIO_UART_CHANNEL_COUNT 2

// Forward declaration of the request structure (defined in bpio_transactions.h)
struct bpio_data_request_t;

typedef struct {
    bool enable;
    uint32_t speed;
    uint8_t data_bits;
    bool parity; // even parity when true
    uint8_t stop_bits;
    bool invert;
    bool tx_enable;
} bpio_uart_channel_config_t;

/**
 * @brief UART transaction handler for BPIO (channel 0).
 * @return true on failure
 */
uint32_t bpio_hwuart_transaction(struct bpio_data_request_t *request, flatbuffers_uint8_vec_t data_write, uint8_t *data_read);

/** @brief Transaction on a specific channel. Returns true on failure. */
uint32_t bpio_hwuart_channel_transaction(uint8_t channel, struct bpio_data_request_t *request,
                                         flatbuffers_uint8_vec_t data_write, uint8_t *data_read);

/** @brief Describe why a failed channel transaction failed (NULL if unknown). */
const char *bpio_hwuart_transaction_error(uint8_t channel, struct bpio_data_request_t *request);

/**
 * @brief UART async handler for BPIO (channel 0 only, legacy interface).
 * @param data_read  Buffer for received data
 * @return           Number of bytes received
 */
uint32_t bpio_hwuart_async_handler(uint8_t *data_read);

/**
 * @brief Fetch pending RX data or a newly latched fault from the next channel, round-robin.
 * @return Number of bytes; *error is set (and the return may be 0) for a fault report.
 */
uint32_t bpio_hwuart_async_next(uint8_t *data_read, uint8_t *channel, const char **error);

/** @brief Configure (or release, cfg->enable == false) a channel. Returns an error string or NULL. */
const char *bpio_hwuart_channel_configure(uint8_t channel, const bpio_uart_channel_config_t *cfg);

/** @brief Release every channel. Called from UART mode cleanup. */
void bpio_hwuart_channels_release(void);

/** @brief True if any IO in io_mask belongs to an active auxiliary channel. */
bool bpio_hwuart_channel_pins_claimed(uint8_t io_mask);

// Implemented by the UART mode (hwuart.c): channel 0 TX state and reconfiguration.
bool hwuart_tx_enabled(void);
void hwuart_bpio_channel0_apply(const bpio_uart_channel_config_t *cfg);

// BPIO mode owns channel 0 RX until hardware UART cleanup. Faults latch until start.
void bpio_hwuart_rx_start(void);
void bpio_hwuart_rx_stop(void);
// Start channel 0 RX if UART mode was entered without BPIO (terminal `m uart`).
// Caller must be in UART mode. No-op if already running or a terminal UART owns it.
void bpio_hwuart_rx_ensure(void);
const char *bpio_hwuart_rx_error(void);
const char *bpio_hwuart_channel_error(uint8_t channel);

#endif // BPIO_UART_H
