#!/usr/bin/env python3
#
# Copyright (c) 2026 Daniel Schaefer
#
# SPDX-License-Identifier: MIT

import argparse
import os
import sys
import time

try:
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")


EPILOG = """
Flash the EC of a Framework Laptop 13 with Intel 12th/13th Gen (hx30, ADL/RPL)
using a Bus Pirate, like FrameworkDebugger's flash_adl_ec.sh does with a Teensy.

The hx30 EC is a Microchip MEC1521. It has no internal flash, its boot ROM loads
ec.bin from a W25Q80 SPI flash. This tool holds the EC in reset through JECDB,
reprograms the W25Q80 over SPI, then lets the EC boot again.

It uses the BPIO2 binary mode instead of flashrom: flashrom's legacy binary
mode switches the Bus Pirate to HiZ every time it connects, which would release
EC reset in the middle of the flash.

WIRING
  Bus Pirate    hx30
  IO3           JECDB pin 9  EC_RST#  (fit RE536, it is not populated)
  VOUT/VREF     JECDB pin 1  3VL_EC   (reference only, the PSU stays off)
  GND           JECDB pin 10 GND
  IO5 CS        W25Q80 pin 1 /CS
  IO4 MISO      W25Q80 pin 2 DO
  IO7 MOSI      W25Q80 pin 5 DI
  IO6 CLK       W25Q80 pin 6 CLK
  Leave W25Q80 pin 8 (VCC) alone, the laptop powers it.

  The laptop stays off with battery or AC connected so 3VL_EC is up. Never
  inject power into it. EC_RST# is only ever pulled low or released.

BEFORE RUNNING
  In the Bus Pirate terminal run "binmode 2" (BPIO2 flatbuffer interface).
  This needs the pybpio client from BusPirate-BPIO2-flatbuffer-interface, plus
  pyserial, cobs and numpy. FrameworkDebugger has it as a submodule.

WHAT IT DOES
  1. Pulls EC_RST# low, then switches the Bus Pirate to SPI.
  2. Checks the JEDEC ID and saves the whole chip to a backup file.
  3. Writes ec.bin over the start of the chip. Everything after it is kept
     from the backup, including the EC settings at 0x80000. --pad fills it
     with zeros instead, like flash_adl_ec.sh.
  4. Erases and programs only the 4K sectors that changed, then reads the
     whole chip back to verify.
  5. Switches the Bus Pirate to HiZ, which releases EC_RST# and the SPI pins,
     and the EC boots.

EXAMPLES
  python flash_adl_ec.py ~/ec/build/hx30/ec.bin
  python flash_adl_ec.py ec.bin --pybpio ~/FrameworkDebugger/scripts/BusPirate-BPIO2-flatbuffer-interface/python
  python flash_adl_ec.py ec.bin --port /dev/ttyACM1 --backup hx30-before.bin
"""


BP_USB_VID = 0x1209
BP_USB_PID = 0x7331

SECTOR_SIZE = 4096
PAGE_SIZE = 256
READ_CHUNK = 512  # BPIO_MAX_READ_SIZE

CMD_WRITE_ENABLE = 0x06
CMD_READ_STATUS = 0x05
CMD_READ_DATA = 0x03
CMD_PAGE_PROGRAM = 0x02
CMD_SECTOR_ERASE = 0x20
CMD_JEDEC_ID = 0x9F

STATUS_BUSY = 0x01
STATUS_BP_MASK = 0x1C


def import_pybpio(path):
    if path:
        sys.path.insert(0, os.path.expanduser(path))
    try:
        from pybpio.bpio_client import BPIOClient
        from pybpio.bpio_spi import BPIOSPI
    except ImportError as e:
        sys.exit(f"Cannot import pybpio ({e}).\n"
                 "Pass --pybpio <BusPirate-BPIO2-flatbuffer-interface>/python or set PYBPIO_PATH.")
    return BPIOClient, BPIOSPI


def find_bin_port():
    ports = [p for p in list_ports.comports() if p.vid == BP_USB_VID and p.pid == BP_USB_PID]
    for p in ports:
        if "bin" in (p.interface or "").lower():
            return p.device
    # interface strings are not always available, the binary port is the second CDC
    if len(ports) >= 2:
        return sorted(p.device for p in ports)[1]
    return None


def addr24(addr):
    return [(addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF]


class SpiFlash:
    def __init__(self, spi):
        self.spi = spi

    def xfer(self, data, read=0):
        result = self.spi.transfer(write_data=data, read_bytes=read)
        if result is False:
            raise RuntimeError(f"SPI transfer failed (command 0x{data[0]:02X})")
        if read and (result is None or len(result) != read):
            raise RuntimeError(f"SPI short read (command 0x{data[0]:02X})")
        return result

    def jedec_id(self):
        return self.xfer([CMD_JEDEC_ID], 3)

    def status(self):
        return self.xfer([CMD_READ_STATUS], 1)[0]

    def wait_ready(self, timeout):
        deadline = time.time() + timeout
        while self.status() & STATUS_BUSY:
            if time.time() > deadline:
                raise RuntimeError("Flash stayed busy, timed out")

    def read(self, size, label):
        data = bytearray()
        start = time.time()
        while len(data) < size:
            n = min(READ_CHUNK, size - len(data))
            data += self.xfer([CMD_READ_DATA] + addr24(len(data)), n)
            if len(data) % 0x10000 == 0 or len(data) == size:
                progress(label, len(data), size, start)
        return bytes(data)

    def erase_sector(self, addr):
        self.xfer([CMD_WRITE_ENABLE])
        self.xfer([CMD_SECTOR_ERASE] + addr24(addr))
        self.wait_ready(timeout=1.0)

    def program_page(self, addr, data):
        self.xfer([CMD_WRITE_ENABLE])
        self.xfer([CMD_PAGE_PROGRAM] + addr24(addr) + list(data))
        self.wait_ready(timeout=0.1)


def progress(label, done, total, start):
    end = "\n" if done == total else ""
    print(f"\r{label}: {done // 1024}/{total // 1024} KiB ({time.time() - start:.1f}s)", end=end, flush=True)


def program(flash, current, target):
    sectors = [a for a in range(0, len(target), SECTOR_SIZE)
               if current[a:a + SECTOR_SIZE] != target[a:a + SECTOR_SIZE]]
    if not sectors:
        print("Flash already contains the image, nothing to write")
        return
    total = len(sectors) * SECTOR_SIZE
    start = time.time()
    for i, sector in enumerate(sectors):
        flash.erase_sector(sector)
        for page in range(sector, sector + SECTOR_SIZE, PAGE_SIZE):
            data = target[page:page + PAGE_SIZE]
            if data != b"\xff" * PAGE_SIZE:
                flash.program_page(page, data)
        progress("Write", (i + 1) * SECTOR_SIZE, total, start)


def hold_reset(client, io):
    # latch low before enabling the output so EC_RST# is never driven high
    if not client.configuration_request(io_value_mask=1 << io, io_value=0):
        raise RuntimeError("Could not set the reset IO low")
    if not client.configuration_request(io_direction_mask=1 << io, io_direction=1 << io):
        raise RuntimeError("Could not set the reset IO to output")


def check_vref(client):
    status = client.status_request(psu=True)
    if not status:
        raise RuntimeError("No answer from BPIO, run \"binmode 2\" in the Bus Pirate terminal")
    mv = status.get("psu_measured_mv") or 0
    if not 3000 <= mv <= 3600:
        raise RuntimeError(f"VOUT/VREF reads {mv} mV, expected 3VL_EC (3.3 V) from JECDB pin 1.\n"
                           "Connect it and keep battery or AC connected to the laptop.")
    print(f"VREF: {mv} mV")


def main():
    parser = argparse.ArgumentParser(description="Flash a Framework hx30 EC with a Bus Pirate",
                                     epilog=EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", help="EC image, e.g. build/hx30/ec.bin")
    parser.add_argument("--port", help="Bus Pirate binary CDC port (default: autodetect)")
    parser.add_argument("--pybpio", default=os.environ.get("PYBPIO_PATH"),
                        help="path to BusPirate-BPIO2-flatbuffer-interface/python (or PYBPIO_PATH)")
    parser.add_argument("--reset-io", type=int, default=3, choices=range(0, 4),
                        help="Bus Pirate IO wired to EC_RST# (default: 3)")
    parser.add_argument("--speed", type=int, default=2000000, help="SPI clock in Hz (default: 2000000)")
    parser.add_argument("--backup", help="backup file (default: ec-backup-<time>.bin)")
    parser.add_argument("--pad", action="store_true",
                        help="zero the rest of the chip instead of keeping it, like flash_adl_ec.sh")
    args = parser.parse_args()

    BPIOClient, BPIOSPI = import_pybpio(args.pybpio)

    with open(args.image, "rb") as f:
        image = f.read()

    port = args.port or find_bin_port()
    if not port:
        sys.exit("No Bus Pirate binary port found, pass --port")
    backup_path = args.backup or time.strftime("ec-backup-%Y%m%d-%H%M%S.bin")

    client = BPIOClient(port)
    try:
        if not client.configuration_request(psu_disable=True, pullup_disable=True):
            raise RuntimeError("No answer from BPIO, run \"binmode 2\" in the Bus Pirate terminal")
        check_vref(client)

        hold_reset(client, args.reset_io)
        print(f"EC held in reset (IO{args.reset_io})")
        time.sleep(0.1)

        spi = BPIOSPI(client)
        if not spi.configure(speed=args.speed, clock_polarity=False, clock_phase=False, chip_select_idle=True):
            raise RuntimeError("Could not switch the Bus Pirate to SPI")
        flash = SpiFlash(spi)

        jedec = flash.jedec_id()
        print("JEDEC ID: " + " ".join(f"{b:02X}" for b in jedec))
        if jedec in (b"\x00\x00\x00", b"\xff\xff\xff") or not 16 <= jedec[2] <= 24:
            raise RuntimeError("No SPI flash found, check the clip and that the EC is held in reset")
        size = 1 << jedec[2]
        if len(image) > size:
            raise RuntimeError(f"Image is {len(image)} bytes, the flash only {size}")
        status = flash.status()
        if status & STATUS_BP_MASK:
            raise RuntimeError(f"Flash block protection is set (status 0x{status:02X}), writes would fail")

        current = flash.read(size, "Backup")
        with open(backup_path, "wb") as f:
            f.write(current)
        print(f"Saved {backup_path}")

        rest = bytes(size - len(image)) if args.pad else current[len(image):]
        target = image + rest

        program(flash, current, target)
        if flash.read(size, "Verify") != target:
            raise RuntimeError(f"Verify failed, restore with: {sys.argv[0]} {backup_path}")
        print("Verify OK")
    except RuntimeError as e:
        print(f"\nERROR: {e}", file=sys.stderr)
        return 1
    finally:
        # HiZ releases every IO: the SPI pins and EC_RST#, the EC boots
        if client.configuration_request(mode="HiZ", mode_configuration={}):
            print("EC released from reset")
        else:
            print(f"Could not switch to HiZ, the EC may still be held in reset by IO{args.reset_io}",
                  file=sys.stderr)
        client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
