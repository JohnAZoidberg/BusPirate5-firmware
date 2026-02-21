/**
 * @file npcx-flasher.c
 * @brief Binmode: standalone NPCX EC flasher from BP5 local storage
 *
 * Implements the NPCX UART flash protocol as a binmode state machine.
 * Reads ec.bin and npcxmon.bin from BP5 storage, flashes EC on button press.
 * Progress output on CDC 0 (terminal). No host needed after file copy.
 *
 * Wiring:
 *   BP5 IO0      --> EC VCC1_RST    (reset, active low)
 *   BP5 IO1      --[4.7-10K]--> EC CR_SOUT1/FLPRG1  (flash mode strap)
 *   BP5 IO4 (TX) --> EC CR_SIN1     (UART RX)
 *   BP5 IO5 (RX) <-- EC CR_SOUT1   (UART TX)
 *   BP5 GND      --- EC GND
 */

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pirate.h"
#include "system_config.h"
#include "pirate/bio.h"
#include "pirate/button.h"
#include "fatfs/ff.h"
#include "pirate/file.h"
#include "ui/ui_term.h"
#include "binmode/binmodes.h"

/* -----------------------------------------------------------------------
 * NPCX protocol constants (from Chromium EC util/uut/)
 * ----------------------------------------------------------------------- */

#define UFPP_H2D_SYNC      0x55
#define UFPP_D2H_SYNC      0x5A
#define UFPP_WRITE          0x07
#define UFPP_READ           0x1C
#define UFPP_FCALL          0x70
#define UFPP_FCALL_RSLT     0x73

#define MONITOR_HDR_TAG     0xA5075001
#define MONITOR_HDR_ADDR    0x200C3000
#define MONITOR_ADDR        0x200C3020
#define FIRMWARE_START_ADDR 0x10090000
#define FIRMWARE_SEGMENT    0x1000  /* 4KB */
#define MAX_RW_DATA_SIZE    256

#define MAX_SYNC_RETRIES    10
#define UART_BYTE_TIMEOUT_MS 1000

#define BIO_RST   0  /* IO0 = reset */
#define BIO_FLPRG 1  /* IO1 = FLPRG1 strap */

/* -----------------------------------------------------------------------
 * CRC-16 (polynomial 0xA001, init 0x0000)
 * Ported from ec/util/uut/lib_crc.cc
 * ----------------------------------------------------------------------- */

static const uint16_t crc16_tab[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
};

static uint16_t update_crc(uint16_t crc, uint8_t c) {
    uint16_t tmp = crc ^ (0x00FF & (uint16_t)c);
    return (crc >> 8) ^ crc16_tab[tmp & 0xFF];
}

static uint16_t crc16_buf(const uint8_t *buf, uint32_t len) {
    uint16_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc = update_crc(crc, buf[i]);
    }
    return crc;
}

/* -----------------------------------------------------------------------
 * UART helpers (blocking, with timeout)
 * ----------------------------------------------------------------------- */

static bool uart_send(const uint8_t *data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        uart_putc_raw(M_UART_PORT, data[i]);
    }
    return true;
}

static bool uart_recv(uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        absolute_time_t deadline = make_timeout_time_ms(UART_BYTE_TIMEOUT_MS);
        while (!uart_is_readable(M_UART_PORT)) {
            if (absolute_time_diff_us(get_absolute_time(), deadline) < 0) {
                return false;
            }
        }
        buf[i] = uart_getc(M_UART_PORT);
    }
    return true;
}

/* -----------------------------------------------------------------------
 * NPCX command builders and transact
 * ----------------------------------------------------------------------- */

static bool npcx_sync(void) {
    uint8_t sync_byte = UFPP_H2D_SYNC;
    uint8_t resp;

    for (int i = 0; i < MAX_SYNC_RETRIES; i++) {
        /* Drain any stale data */
        while (uart_is_readable(M_UART_PORT)) {
            uart_getc(M_UART_PORT);
        }
        uart_send(&sync_byte, 1);
        if (uart_recv(&resp, 1) && resp == UFPP_D2H_SYNC) {
            return true;
        }
        sleep_ms(200);
    }
    return false;
}

static bool npcx_write_chunk(uint32_t addr, const uint8_t *data, uint32_t len) {
    uint8_t pkt[6 + MAX_RW_DATA_SIZE + 2];
    uint32_t pkt_len = 0;

    pkt[pkt_len++] = UFPP_WRITE;
    pkt[pkt_len++] = (uint8_t)(len - 1);
    /* Address big-endian */
    pkt[pkt_len++] = (uint8_t)(addr >> 24);
    pkt[pkt_len++] = (uint8_t)(addr >> 16);
    pkt[pkt_len++] = (uint8_t)(addr >> 8);
    pkt[pkt_len++] = (uint8_t)(addr);
    memcpy(&pkt[pkt_len], data, len);
    pkt_len += len;
    /* CRC over everything so far, appended MSB first */
    uint16_t crc = crc16_buf(pkt, pkt_len);
    pkt[pkt_len++] = (uint8_t)(crc >> 8);
    pkt[pkt_len++] = (uint8_t)(crc);

    uart_send(pkt, pkt_len);

    uint8_t resp;
    if (!uart_recv(&resp, 1) || resp != UFPP_WRITE) {
        return false;
    }
    return true;
}

static bool npcx_read_chunk(uint32_t addr, uint8_t *buf, uint32_t len) {
    uint8_t pkt[8];
    uint32_t pkt_len = 0;

    pkt[pkt_len++] = UFPP_READ;
    pkt[pkt_len++] = (uint8_t)(len - 1);
    pkt[pkt_len++] = (uint8_t)(addr >> 24);
    pkt[pkt_len++] = (uint8_t)(addr >> 16);
    pkt[pkt_len++] = (uint8_t)(addr >> 8);
    pkt[pkt_len++] = (uint8_t)(addr);
    uint16_t crc = crc16_buf(pkt, pkt_len);
    pkt[pkt_len++] = (uint8_t)(crc >> 8);
    pkt[pkt_len++] = (uint8_t)(crc);

    uart_send(pkt, pkt_len);

    /* Response: [0x1C, data(len), CRC_MSB, CRC_LSB] */
    uint8_t resp[1 + MAX_RW_DATA_SIZE + 2];
    uint32_t resp_len = 1 + len + 2;
    if (!uart_recv(resp, resp_len)) {
        return false;
    }
    if (resp[0] != UFPP_READ) {
        return false;
    }
    memcpy(buf, &resp[1], len);
    return true;
}

static bool npcx_exec_return(uint32_t addr) {
    uint8_t pkt[8];
    uint32_t pkt_len = 0;

    pkt[pkt_len++] = UFPP_FCALL;
    pkt[pkt_len++] = 0x00;
    pkt[pkt_len++] = (uint8_t)(addr >> 24);
    pkt[pkt_len++] = (uint8_t)(addr >> 16);
    pkt[pkt_len++] = (uint8_t)(addr >> 8);
    pkt[pkt_len++] = (uint8_t)(addr);
    uint16_t crc = crc16_buf(pkt, pkt_len);
    pkt[pkt_len++] = (uint8_t)(crc >> 8);
    pkt[pkt_len++] = (uint8_t)(crc);

    uart_send(pkt, pkt_len);

    /* Response: [status, 0x73, result] */
    uint8_t resp[3];
    if (!uart_recv(resp, 3)) {
        return false;
    }
    if (resp[1] != UFPP_FCALL_RSLT || resp[2] != 0x03) {
        return false;
    }
    return true;
}

/* -----------------------------------------------------------------------
 * State machine
 * ----------------------------------------------------------------------- */

enum npcx_state {
    STATE_IDLE,
    STATE_OPEN_FILES,
    STATE_ENTER_FLASH,
    STATE_SYNC,
    STATE_UPLOAD_MONITOR,
    STATE_FLASH_SEGMENT,
    STATE_REBOOT,
    STATE_DONE,
    STATE_ERROR,
};

static enum npcx_state state;
static FIL fil_ec;
static FIL fil_mon;
static uint8_t monitor_buf[2048];
static uint32_t monitor_size;
static uint32_t ec_size;
static uint32_t flash_offset;
static uint32_t ec_bytes_remaining;
static uint8_t seg_buf[FIRMWARE_SEGMENT];
static const char *error_msg;

const char npcx_flasher_name[] = "NPCX EC Flasher";

void npcx_flasher_setup(void) {
    /* Configure UART: 115200 8N1 on IO4/IO5 */
    uart_init(M_UART_PORT, 115200);
    uart_set_format(M_UART_PORT, 8, 1, UART_PARITY_NONE);
    bio_buf_output(M_UART_TX);
    bio_buf_input(M_UART_RX);
    bio_set_function(M_UART_TX, GPIO_FUNC_UART);
    bio_set_function(M_UART_RX, GPIO_FUNC_UART);
    uart_set_hw_flow(M_UART_PORT, false, false);

    /* Drain any stale UART data */
    while (uart_is_readable(M_UART_PORT)) {
        uart_getc(M_UART_PORT);
    }

    /* IO0/IO1 start as inputs (floating) */
    bio_input(BIO_RST);
    bio_input(BIO_FLPRG);

    state = STATE_IDLE;
}

void npcx_flasher_setup_message(void) {
    printf("%sNPCX EC Flasher ready%s\r\n",
           ui_term_color_info(), ui_term_color_reset());
    printf("Copy ec.bin and npcxmon.bin to BP5 storage, then press button to flash\r\n");
}

static void enter_error(const char *msg) {
    error_msg = msg;
    state = STATE_ERROR;
}

void npcx_flasher_service(void) {
    uint32_t bytes_read;

    switch (state) {

    case STATE_IDLE:
        if (button_check_press(0) != BP_BUTT_NO_PRESS) {
            state = STATE_OPEN_FILES;
        }
        break;

    case STATE_OPEN_FILES:
        printf("Opening files from storage...\r\n");

        if (!file_open(&fil_mon, "npcxmon.bin", FA_READ | FA_OPEN_EXISTING)) {
            enter_error("Cannot open npcxmon.bin");
            break;
        }
        monitor_size = file_size(&fil_mon);
        if (monitor_size == 0 || monitor_size > sizeof(monitor_buf)) {
            file_close(&fil_mon);
            enter_error("npcxmon.bin invalid size");
            break;
        }
        if (!file_read(&fil_mon, monitor_buf, monitor_size, &bytes_read) ||
            bytes_read != monitor_size) {
            file_close(&fil_mon);
            enter_error("Failed to read npcxmon.bin");
            break;
        }
        file_close(&fil_mon);

        if (!file_open(&fil_ec, "ec.bin", FA_READ | FA_OPEN_EXISTING)) {
            enter_error("Cannot open ec.bin");
            break;
        }
        ec_size = file_size(&fil_ec);
        if (ec_size == 0) {
            file_close(&fil_ec);
            enter_error("ec.bin is empty");
            break;
        }

        printf("  npcxmon.bin: %lu bytes\r\n", (unsigned long)monitor_size);
        printf("  ec.bin:      %lu bytes\r\n", (unsigned long)ec_size);

        flash_offset = 0;
        ec_bytes_remaining = ec_size;
        state = STATE_ENTER_FLASH;
        break;

    case STATE_ENTER_FLASH:
        printf("Entering EC flash mode...\r\n");

        /* IO0 + IO1 output low (assert RST + FLPRG1) */
        bio_output(BIO_RST);
        bio_put(BIO_RST, false);
        bio_output(BIO_FLPRG);
        bio_put(BIO_FLPRG, false);
        sleep_ms(100);

        /* Release RST (IO0 -> input), keep FLPRG1 low */
        bio_input(BIO_RST);
        sleep_ms(1000);

        /* Release FLPRG1 */
        bio_input(BIO_FLPRG);
        sleep_ms(100);

        state = STATE_SYNC;
        break;

    case STATE_SYNC:
        printf("Syncing with EC...\r\n");
        if (!npcx_sync()) {
            file_close(&fil_ec);
            enter_error("Sync failed - no EC response");
            break;
        }
        printf("Sync OK\r\n");
        state = STATE_UPLOAD_MONITOR;
        break;

    case STATE_UPLOAD_MONITOR: {
        printf("Uploading flash monitor (%lu bytes)...\r\n",
               (unsigned long)monitor_size);
        uint32_t addr = MONITOR_ADDR;
        uint32_t remaining = monitor_size;
        uint32_t offset = 0;
        while (remaining > 0) {
            uint32_t chunk = (remaining > MAX_RW_DATA_SIZE) ?
                             MAX_RW_DATA_SIZE : remaining;
            if (!npcx_write_chunk(addr, &monitor_buf[offset], chunk)) {
                file_close(&fil_ec);
                enter_error("Monitor upload failed");
                return;
            }
            addr += chunk;
            offset += chunk;
            remaining -= chunk;
        }
        printf("Monitor uploaded\r\n");
        state = STATE_FLASH_SEGMENT;
        break;
    }

    case STATE_FLASH_SEGMENT: {
        if (ec_bytes_remaining == 0) {
            /* All segments done - clear header tag */
            uint32_t zero = 0;
            npcx_write_chunk(MONITOR_HDR_ADDR, (const uint8_t *)&zero, 4);
            file_close(&fil_ec);
            state = STATE_REBOOT;
            break;
        }

        uint32_t seg_size = (ec_bytes_remaining > FIRMWARE_SEGMENT) ?
                            FIRMWARE_SEGMENT : ec_bytes_remaining;

        if (!file_read(&fil_ec, seg_buf, seg_size, &bytes_read) ||
            bytes_read != seg_size) {
            file_close(&fil_ec);
            enter_error("Failed to read ec.bin segment");
            break;
        }

        /* Check if segment is all 0xFF */
        bool all_ff = true;
        for (uint32_t i = 0; i < seg_size; i++) {
            if (seg_buf[i] != 0xFF) {
                all_ff = false;
                break;
            }
        }

        /* Build monitor header: [tag, seg_size, src_addr, flash_dest] */
        uint32_t hdr[4];
        hdr[0] = MONITOR_HDR_TAG;
        hdr[1] = seg_size;
        hdr[3] = flash_offset;

        if (all_ff) {
            /* Erase-only: src_addr=0 */
            hdr[2] = 0;
            if (!npcx_write_chunk(MONITOR_HDR_ADDR, (const uint8_t *)hdr, 16)) {
                file_close(&fil_ec);
                enter_error("Header write failed (erase)");
                break;
            }
        } else {
            /* Write data to RAM, then program */
            hdr[2] = FIRMWARE_START_ADDR;
            if (!npcx_write_chunk(MONITOR_HDR_ADDR, (const uint8_t *)hdr, 16)) {
                file_close(&fil_ec);
                enter_error("Header write failed");
                break;
            }
            uint32_t addr = FIRMWARE_START_ADDR;
            uint32_t chunk_remaining = seg_size;
            uint32_t chunk_offset = 0;
            while (chunk_remaining > 0) {
                uint32_t chunk = (chunk_remaining > MAX_RW_DATA_SIZE) ?
                                 MAX_RW_DATA_SIZE : chunk_remaining;
                if (!npcx_write_chunk(addr, &seg_buf[chunk_offset], chunk)) {
                    file_close(&fil_ec);
                    enter_error("Segment data write failed");
                    return;
                }
                addr += chunk;
                chunk_offset += chunk;
                chunk_remaining -= chunk;
            }
        }

        if (!npcx_exec_return(MONITOR_ADDR)) {
            file_close(&fil_ec);
            enter_error("Monitor execution failed");
            break;
        }

        flash_offset += seg_size;
        ec_bytes_remaining -= seg_size;

        uint32_t pct = (ec_size - ec_bytes_remaining) * 100 / ec_size;
        printf("\r[%lu%%] %lu/%lu",
               (unsigned long)pct,
               (unsigned long)(ec_size - ec_bytes_remaining),
               (unsigned long)ec_size);
        if (ec_bytes_remaining == 0) {
            printf("\r\n");
        }
        break;
    }

    case STATE_REBOOT:
        printf("Flash complete. Rebooting EC...\r\n");
        /* Assert RST */
        bio_output(BIO_RST);
        bio_put(BIO_RST, false);
        sleep_ms(500);
        /* Release RST */
        bio_input(BIO_RST);

        state = STATE_DONE;
        break;

    case STATE_DONE:
        printf("%sFlash successful!%s\r\n",
               ui_term_color_info(), ui_term_color_reset());
        state = STATE_IDLE;
        break;

    case STATE_ERROR:
        printf("%sError: %s%s\r\n",
               ui_term_color_error(), error_msg, ui_term_color_reset());
        /* Clean up GPIO */
        bio_input(BIO_RST);
        bio_input(BIO_FLPRG);
        state = STATE_IDLE;
        break;
    }
}

void npcx_flasher_cleanup(void) {
    uart_deinit(M_UART_PORT);
    bio_input(BIO_RST);
    bio_input(BIO_FLPRG);
    bio_input(M_UART_TX);
    bio_input(M_UART_RX);
}
