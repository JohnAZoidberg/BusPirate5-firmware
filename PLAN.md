# Simultaneous EC and SoC UART consoles

Recorded 2026-11-16. Implemented on branch `dual-uart` (from `main`) and
validated on hardware the same day. The `pio-uart` branch (arbitrary-pin PIO
UART) is not needed for this wiring and is kept separately.

## Goal

Expose two independent, simultaneous UART consoles through one Bus Pirate BPIO
connection, readable and writable from Python and as two host PTYs. Opening
consoles must not enable the Bus Pirate PSU or pull-ups. Preserve existing
single-UART clients.

## Hardware and wiring

Bus Pirate 5 rev10 (RP2040). Directions are from the Bus Pirate's perspective.
Target IO voltage is 3.3 V for both consoles (user confirmed; idle TX lines
measured ~3.33 V against a ~3.36 V external reference).

| Console | BP TX → target RX | BP RX ← target TX | MCU GPIOs | Peripheral | BPIO channel |
| --- | --- | --- | --- | --- | --- |
| EC | IO4 | IO5 | GPIO12 / GPIO13 | UART0 | 0 |
| CPU / SoC | IO0 | IO1 | GPIO8 / GPIO9 | UART1 | 1 |

Evidence for the mapping: `BUFIO0..7 = GPIO8..15` (`src/platform/bpi5-rev10.h`),
RP2040 `IO_BANK0_GPIO{8,9,12,13}_CTRL_FUNCSEL_VALUE_UART{1_TX,1_RX,0_TX,0_RX}`,
`M_UART_PORT uart0` on IO4/IO5 (`src/pirate.h`) and `BP_DEBUG_UART_1 uart1` on
IO0/IO1. The target is powered independently; PSU and pull-ups stay off.

## Protocol (BPIO 2.3, appended fields only)

- `StatusResponse.uart_channel_count` (2 on this firmware; absent/0 = legacy).
- `ConfigurationRequest.uart_channels: [UartChannelConfiguration]` with
  `channel, enable=true, speed=115200, data_bits=8, parity, stop_bits=1,
  signal_inversion, tx_enable=false`. Applied after `mode`, in order; requires
  UART mode. Configuring a channel resets only that channel's RX buffer and
  latched faults. `enable=false` releases channel 1; channel 0 is released by
  leaving UART mode.
- A channel 0 entry in the same request as `mode="UART"` decides whether mode
  entry drives IO4 at all, so receive-only start never drives TX.
- `DataRequest.channel` / `DataResponse.channel` (default 0). Unsolicited RX
  and fault packets carry the channel; channels are served round-robin.
- Errors: invalid channel, channel not enabled, TX disabled, pins in use,
  debug/terminal UART owns the peripheral, out-of-range framing.
  `io_direction`/`io_value` touching an active channel 1 pin is rejected.
- Requests using channels set `minimum_version_minor = 3`, so older firmware
  refuses them instead of silently using UART0. Clients check
  `uart_channel_count` first.
- Legacy clients (no channel fields) behave as before: UART mode entry drives
  IO4 TX, channel 0 data is untagged.

Known limitation: any `mode="UART"` request runs mode cleanup and releases
channel 1. Reconfigure channel 0 through `uart_channels` instead.
`EcTarget.enter_flash_mode()` still re-enters the mode, so EC flashing tears
down the SoC console until it is changed.

## Host software

- `frameworkdebugger-library/scripts/bpconsoles.py`: `BusPirateConsoles`
  owns the BPIO port; `bp.ec` / `bp.soc` are `UartChannel` objects with
  `write`, `send`, `read`, `expect`, `output`, `configure`, `start_pty`.
  CLI: `python3 bpconsoles.py --ec-tx --soc-tx --log-dir /tmp/bp-logs`
  creates `/tmp/bp-ec-console` and `/tmp/bp-soc-console`. No PSU or pull-up
  fields are ever sent. RX faults are recovered per channel at most once a
  second.
- Python bindings regenerated with nixpkgs `flatc` 25.12.19 (identical output
  for the old schema); C bindings with nixpkgs `flatcc` 0.6.4 (only the version
  comment differs from the committed 0.6.2 output).

## Validation

- Unit test (two mocked UARTs, routing, fairness, faults, TX-disabled pins,
  release): `cc -std=c11 -Wall -I tests/bpio_uart_stubs tests/test_bpio_uart.c`.
- Nix build OK. RAM 89.5% → 91.3% (second 4 KB RX ring).
- Hardware, PSU and pull-ups off throughout:
  - receive-only start: IO0, IO1, IO4, IO5 buffers all inputs;
  - enabling one channel's TX turns around only that pin;
  - EC answered `ec:~>` and `version` (npcx9m3f, `dahlia-0.0.0-247ab50`);
  - SoC answered the shell prompt and `uname`; `seq 1 5000` (~24 KB) intact at
    near line rate;
  - EC command answered during the SoC burst, burst still intact, no
    cross-channel data, no faults;
  - closing/reopening the EC PTY and disabling EC TX left the SoC working;
  - repeated on the committed build: device reports `3783b18`.
  - power cycle: EC reset via IO3 (active low, 0.5 s; IO7 untouched), then
    `powerbtn` on the EC console. Separate logs captured the EC from
    `*** Booting Zephyr OS` (reset cause: reset-pin) through G3→S5→S0 and the
    SoC from the coreboot bootblock banner to `nixos login:` (~56 s). Each
    channel latched one break/framing fault as its line dropped and recovered
    automatically; PSU and pull-ups stayed off. Logs:
    `/tmp/bp-powercycle-20261116-150718/{ec,soc}.raw`.

## Open items

- Address the four implementation review findings below. RX-fix integration
  is planned separately in `PLAN-uart-integration.md`; it does not resolve all
  four findings by itself.
- Legacy single-UART path is covered by unit tests only on this build.
- `ectarget._PtyBridge` leaves its PTY slave in cooked/echo mode, so target
  output is echoed back to the target until a terminal sets raw mode (fixed in
  `bpconsoles.py` with `tty.setraw(slave, TCSANOW)`; `EcTarget.pty_bridge()`
  still has it).
- Move `EcTarget` mode re-entry to `uart_channels` channel 0 reconfiguration.
- Upstreaming the BPIO submodule change needs a fork of
  `DangerousPrototypes/BusPirate-BPIO2-flatbuffer-interface`.

## Implementation review findings (2026-11-16)

These remain open. Review validation: the Nix build and existing C UART tests
passed; the Python configuration race was reproduced with a mocked bus. No
hardware configuration was changed during the review.

1. **P1: blocking requests can overflow the other channel.**
   `src/binmode/bpio_uart.c`, `bpio_hwuart_channel_transaction()` still uses
   blocking writes and a one-second idle read timeout. A 512-byte write at
   9600 baud takes roughly half a second; an idle read can block for a second.
   The other channel's 4095-byte ring fills in about 356 ms at 115200 8N1.
   Interrupt buffering alone cannot prevent this loss. Make transactions
   cooperative so both channels continue draining to the host; bound service
   latency and preserve legacy request/response semantics. Add tests with a
   silent read or slow TX on one channel and sustained RX on the other, rather
   than testing only round-robin async packet selection.

2. **P2: recovery can undo a TX-disable request.**
   `frameworkdebugger-library/scripts/bpconsoles.py`, `UartChannel.configure()`
   reads and updates `self.config` outside the BPIO transport lock. Recovery
   can capture the previous `tx_enable=True`, reapply it after disable, and
   leave firmware TX enabled while the cached state says disabled. Serialize
   the complete read/configure/cache-update operation, including recovery.
   Add a deterministic concurrent disable/recovery test and assert that both
   the applied and cached configurations agree.

3. **P2: channel 0 misses debug-UART ownership rejection.**
   `src/binmode/bpio.c`, `mode_change_new()` checks terminal UART ownership
   but omits debug UART ownership for UART0. Channel 1 checks both. Reject
   conflicts before changing hardware and return the specific reason. The
   `29a771b` RX-fix branch contains this guard; preserve it during integration
   and test both terminal/debug ownership on both peripherals.

4. **P2: initialization failure leaves partial configuration active.**
   `frameworkdebugger-library/scripts/bpconsoles.py`,
   `BusPirateConsoles.__init__()` closes only the host port on failure.
   Firmware applies channel entries sequentially: if EC configuration succeeds
   with TX enabled and SoC configuration then fails, EC TX can remain driven
   after the constructor raises. Once hardware configuration has begun,
   attempt HiZ cleanup before closing the transport, preserving PSU/pull-up
   state and the original exception. Do not alter hardware for a capability
   rejection before configuration starts. Test a failure on the second channel
   and a cleanup failure, checking resource release and error reporting.
