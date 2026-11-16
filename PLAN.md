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

## Open items

- Capture separately identified EC and SoC boot logs across a target power
  cycle (exercises fault recovery while the reference voltage is absent).
- Legacy single-UART path is covered by unit tests only on this build.
- `ectarget._PtyBridge` leaves its PTY slave in cooked/echo mode, so target
  output is echoed back to the target until a terminal sets raw mode (fixed in
  `bpconsoles.py` with `tty.setraw(slave, TCSANOW)`; `EcTarget.pty_bridge()`
  still has it).
- Move `EcTarget` mode re-entry to `uart_channels` channel 0 reconfiguration.
- Upstreaming the BPIO submodule change needs a fork of
  `DangerousPrototypes/BusPirate-BPIO2-flatbuffer-interface`.
