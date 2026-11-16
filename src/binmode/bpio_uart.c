// Modified by OpenAI, 2026: interrupt-buffered BPIO UART receive.
// Modified by Daniel Schaefer and Claude, 2026: independent hardware UART channels.
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pirate.h"
#include "system_config.h"
#include "pirate/bio.h"
#include "bpio_uart.h"
#include "bpio_reader.h"
#include "bpio_transactions.h"

#define BPIO_MAX_READ_SIZE 512
#define RX_RING_SIZE 4096u
#define RX_RING_MASK (RX_RING_SIZE - 1u)

// Channel 0 is the UART mode peripheral (M_UART_PORT on M_UART_TX/M_UART_RX).
// Channel 1 is the other hardware UART on the pins the platform routes it to.
#define AUX_UART_PORT BP_DEBUG_UART_1
#define AUX_UART_TX BP_DEBUG_UART_1_TX
#define AUX_UART_RX BP_DEBUG_UART_1_RX

static const char aux_pin_labels[][5] = { "TX1>", "RX1<" };

struct bpio_uart_channel {
    uart_inst_t *uart;
    uint8_t tx_pin, rx_pin; // BIO numbers
    irq_handler_t irq;
    // IRQ producer and foreground consumer run on the same core. Foreground
    // accesses mask interrupts, so indices and fault snapshots cannot race.
    uint8_t ring[RX_RING_SIZE];
    volatile uint32_t head, tail;
    volatile uint32_t overflow, overrun, framing, parity, brk;
    volatile bool fault;
    bool rx_active, fault_reported, tx_enabled;
    char error_text[176];
};

static void bpio_uart_ch0_irq(void);
static void bpio_uart_ch1_irq(void);

static struct bpio_uart_channel channels[BPIO_UART_CHANNEL_COUNT] = {
    { .uart = M_UART_PORT, .tx_pin = M_UART_TX, .rx_pin = M_UART_RX, .irq = bpio_uart_ch0_irq },
    { .uart = AUX_UART_PORT, .tx_pin = AUX_UART_TX, .rx_pin = AUX_UART_RX, .irq = bpio_uart_ch1_irq },
};
static uint8_t async_next_channel;

static inline void __not_in_flash_func(rx_drain)(struct bpio_uart_channel *c) {
    while (uart_is_readable(c->uart)) {
        uint32_t dr = uart_get_hw(c->uart)->dr;
        // PL011 DR[11:8] = overrun, break, parity, framing errors.
        if (dr & (1u << 11)) c->overrun++;
        if (dr & (1u << 10)) c->brk++;
        if (dr & (1u << 9)) c->parity++;
        if (dr & (1u << 8)) c->framing++;
        if (dr & 0xf00u) c->fault = true;
        uint32_t next = (c->head + 1u) & RX_RING_MASK;
        if (next == c->tail) {
            c->overflow++;
            c->fault = true;
        }
        if (!c->fault) {
            c->ring[c->head] = (uint8_t)dr;
            c->head = next;
        }
    }
    // Clear sticky UART error status after consuming the FIFO.
    uart_get_hw(c->uart)->rsr = 0;
}

static void __not_in_flash_func(bpio_uart_ch0_irq)(void) {
    rx_drain(&channels[0]);
}

static void __not_in_flash_func(bpio_uart_ch1_irq)(void) {
    rx_drain(&channels[1]);
}

static bool uart_in_use_by_terminal(uart_inst_t *uart) {
    uint32_t index = uart_get_index(uart);
    return (system_config.terminal_uart_enable && system_config.terminal_uart_number == index) ||
           (system_config.debug_uart_enable && system_config.debug_uart_number == index);
}

static void channel_rx_start(struct bpio_uart_channel *c) {
    uint32_t saved = save_and_disable_interrupts();
    c->head = c->tail = 0;
    c->overflow = c->overrun = c->framing = c->parity = c->brk = 0;
    c->fault = false;
    c->fault_reported = false;
    uart_get_hw(c->uart)->rsr = 0;
    // Install only on the BPIO core; channel configuration rejects a debug
    // terminal using this peripheral on the other core.
    irq_add_shared_handler(UART_IRQ_NUM(c->uart), c->irq, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    c->rx_active = true;
    uart_set_irq_enables(c->uart, true, false);
    irq_set_enabled(UART_IRQ_NUM(c->uart), true);
    restore_interrupts(saved);
}

static void channel_rx_stop(struct bpio_uart_channel *c) {
    if (!c->rx_active) return;
    uint32_t saved = save_and_disable_interrupts();
    uart_set_irq_enables(c->uart, false, false);
    irq_remove_handler(UART_IRQ_NUM(c->uart), c->irq);
    c->rx_active = false;
    restore_interrupts(saved);
}

void bpio_hwuart_rx_start(void) {
    struct bpio_uart_channel *c = &channels[0];
    channel_rx_stop(c);
    c->tx_enabled = hwuart_tx_enabled();
    channel_rx_start(c);
}

void bpio_hwuart_rx_stop(void) {
    channel_rx_stop(&channels[0]);
    channels[0].tx_enabled = false;
}

static void aux_pins_release(struct bpio_uart_channel *c) {
    bio_input(c->tx_pin);
    bio_input(c->rx_pin);
    bio_set_function(c->tx_pin, GPIO_FUNC_SIO);
    bio_set_function(c->rx_pin, GPIO_FUNC_SIO);
    gpio_set_outover(bio2bufiopin[c->tx_pin], GPIO_OVERRIDE_NORMAL);
    gpio_set_inover(bio2bufiopin[c->rx_pin], GPIO_OVERRIDE_NORMAL);
    system_bio_update_purpose_and_label(false, c->tx_pin, BP_PIN_MODE, 0);
    system_bio_update_purpose_and_label(false, c->rx_pin, BP_PIN_MODE, 0);
}

static void aux_release(struct bpio_uart_channel *c) {
    if (!c->rx_active) return;
    channel_rx_stop(c);
    // Release the pins before the peripheral so TX never floats while driven.
    aux_pins_release(c);
    uart_deinit(c->uart);
    c->tx_enabled = false;
}

void bpio_hwuart_channels_release(void) {
    for (uint8_t i = 1; i < BPIO_UART_CHANNEL_COUNT; i++) aux_release(&channels[i]);
    bpio_hwuart_rx_stop();
}

bool bpio_hwuart_channel_pins_claimed(uint8_t io_mask) {
    for (uint8_t i = 1; i < BPIO_UART_CHANNEL_COUNT; i++) {
        struct bpio_uart_channel *c = &channels[i];
        if (c->rx_active && (io_mask & ((1u << c->tx_pin) | (1u << c->rx_pin)))) return true;
    }
    return false;
}

static const char *validate_config(const bpio_uart_channel_config_t *cfg) {
    if (cfg->speed == 0 || cfg->speed > 7372800) return "UART channel speed out of range";
    if (cfg->data_bits < 5 || cfg->data_bits > 8) return "UART channel data bits out of range (5-8)";
    if (cfg->stop_bits < 1 || cfg->stop_bits > 2) return "UART channel stop bits out of range (1-2)";
    return NULL;
}

static const char *aux_configure(struct bpio_uart_channel *c, const bpio_uart_channel_config_t *cfg) {
    if (uart_in_use_by_terminal(c->uart)) return "UART channel peripheral is used by the debug/terminal UART";
    if (!c->rx_active) {
        // Pins must be free IO; another function (or a debug UART) owns them otherwise.
        if (system_config.pin_func[c->tx_pin + 1] != BP_PIN_IO ||
            system_config.pin_func[c->rx_pin + 1] != BP_PIN_IO) {
            return "UART channel pins are in use";
        }
    }
    channel_rx_stop(c);

    // Leave TX undriven while the peripheral is reset and reprogrammed.
    bio_input(c->tx_pin);
    bio_set_function(c->tx_pin, GPIO_FUNC_SIO);

    uart_init(c->uart, cfg->speed);
    uart_set_format(c->uart, cfg->data_bits, cfg->stop_bits, cfg->parity ? UART_PARITY_EVEN : UART_PARITY_NONE);
    uart_set_hw_flow(c->uart, false, false);
    uart_set_fifo_enabled(c->uart, true);

    gpio_set_outover(bio2bufiopin[c->tx_pin], cfg->invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
    gpio_set_inover(bio2bufiopin[c->rx_pin], cfg->invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

    bio_buf_input(c->rx_pin);
    bio_set_function(c->rx_pin, GPIO_FUNC_UART);
    if (cfg->tx_enable) {
        // The UART drives idle level on the MCU side before the buffer turns around.
        bio_set_function(c->tx_pin, GPIO_FUNC_UART);
        bio_buf_output(c->tx_pin);
    }
    system_bio_update_purpose_and_label(true, c->tx_pin, BP_PIN_MODE, aux_pin_labels[0]);
    system_bio_update_purpose_and_label(true, c->rx_pin, BP_PIN_MODE, aux_pin_labels[1]);

    while (uart_is_readable(c->uart)) (void)uart_get_hw(c->uart)->dr;
    c->tx_enabled = cfg->tx_enable;
    channel_rx_start(c);
    return NULL;
}

const char *bpio_hwuart_channel_configure(uint8_t channel, const bpio_uart_channel_config_t *cfg) {
    // Caller guarantees UART mode is active; channels live only inside it.
    if (channel >= BPIO_UART_CHANNEL_COUNT) return "Invalid UART channel";
    struct bpio_uart_channel *c = &channels[channel];

    if (!cfg->enable) {
        if (channel == 0) return "UART channel 0 is released by leaving UART mode";
        aux_release(c);
        return NULL;
    }
    const char *error = validate_config(cfg);
    if (error) return error;

    if (channel == 0) {
        channel_rx_stop(c);
        hwuart_bpio_channel0_apply(cfg);
        bpio_hwuart_rx_start();
        return NULL;
    }
    return aux_configure(c, cfg);
}

static const char *channel_error(struct bpio_uart_channel *c, uint8_t channel) {
    uint32_t saved = save_and_disable_interrupts();
    bool fault = c->fault;
    uint32_t overflow = c->overflow, overrun = c->overrun;
    uint32_t framing = c->framing, parity = c->parity, breaks = c->brk;
    restore_interrupts(saved);
    if (!fault) return NULL;
    snprintf(c->error_text, sizeof(c->error_text),
             "UART RX fault: channel=%u overflow=%lu overrun=%lu framing=%lu parity=%lu break=%lu; reconfigure UART",
             channel, (unsigned long)overflow, (unsigned long)overrun, (unsigned long)framing,
             (unsigned long)parity, (unsigned long)breaks);
    return c->error_text;
}

const char *bpio_hwuart_channel_error(uint8_t channel) {
    if (channel >= BPIO_UART_CHANNEL_COUNT) return "Invalid UART channel";
    struct bpio_uart_channel *c = &channels[channel];
    if (!c->rx_active) return "UART channel not enabled";
    return channel_error(c, channel);
}

const char *bpio_hwuart_rx_error(void) {
    return channel_error(&channels[0], 0);
}

static bool rx_get(struct bpio_uart_channel *c, uint8_t *byte) {
    uint32_t saved = save_and_disable_interrupts();
    bool available = c->rx_active && !c->fault && c->head != c->tail;
    if (available) {
        *byte = c->ring[c->tail];
        c->tail = (c->tail + 1u) & RX_RING_MASK;
    }
    restore_interrupts(saved);
    return available;
}

uint32_t bpio_hwuart_channel_transaction(uint8_t channel, struct bpio_data_request_t *request,
                                         flatbuffers_uint8_vec_t data_write, uint8_t *data_read) {
    if (channel >= BPIO_UART_CHANNEL_COUNT) return true;
    struct bpio_uart_channel *c = &channels[channel];
    if (!c->rx_active || c->fault) return true;
    if (request->bytes_write && !c->tx_enabled) return true;
    for (uint32_t i = 0; i < request->bytes_write; i++) {
        uart_putc_raw(c->uart, flatbuffers_uint8_vec_at(data_write, i));
        if (c->fault) return true;
    }
    for (uint32_t i = 0; i < request->bytes_read; i++) {
        uint32_t started = time_us_32();
        while (!rx_get(c, &data_read[i])) {
            if (c->fault || time_us_32() - started >= 1000000u) return true;
            tight_loop_contents();
        }
    }
    return c->fault;
}

uint32_t bpio_hwuart_transaction(struct bpio_data_request_t *request,
                                flatbuffers_uint8_vec_t data_write, uint8_t *data_read) {
    return bpio_hwuart_channel_transaction(0, request, data_write, data_read);
}

const char *bpio_hwuart_transaction_error(uint8_t channel, struct bpio_data_request_t *request) {
    if (channel >= BPIO_UART_CHANNEL_COUNT) return "Invalid UART channel";
    struct bpio_uart_channel *c = &channels[channel];
    if (!c->rx_active) return "UART channel not enabled";
    const char *error = channel_error(c, channel);
    if (error) return error;
    if (request->bytes_write && !c->tx_enabled) return "UART channel TX disabled";
    return NULL;
}

uint32_t bpio_hwuart_async_next(uint8_t *data_read, uint8_t *channel, const char **error) {
    // Round-robin: each call serves at most one channel, starting after the
    // channel served last, so a busy channel cannot starve the other.
    for (uint8_t n = 0; n < BPIO_UART_CHANNEL_COUNT; n++) {
        uint8_t ch = (uint8_t)((async_next_channel + n) % BPIO_UART_CHANNEL_COUNT);
        struct bpio_uart_channel *c = &channels[ch];
        if (!c->rx_active) continue;
        uint32_t count = 0;
        while (count < BPIO_MAX_READ_SIZE && rx_get(c, &data_read[count])) count++;
        const char *fault = NULL;
        if (!count && !c->fault_reported) {
            fault = channel_error(c, ch);
            if (fault) c->fault_reported = true;
        }
        if (count || fault) {
            async_next_channel = (uint8_t)((ch + 1u) % BPIO_UART_CHANNEL_COUNT);
            *channel = ch;
            *error = fault;
            return count;
        }
    }
    *error = NULL;
    return 0;
}

uint32_t bpio_hwuart_async_handler(uint8_t *data_read) {
    uint32_t count = 0;
    while (count < BPIO_MAX_READ_SIZE && rx_get(&channels[0], &data_read[count])) count++;
    return count;
}
