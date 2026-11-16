# Integrate BPIO UART RX fixes with dual UART channels

Recorded 2026-11-16. Plan only; nothing below is implemented yet.

## Starting point

| Branch | Base | Contents | State |
| --- | --- | --- | --- |
| `bpio-uart-rx-fixes` (also on remote `zoid`) | `main` (`2696a09`) | `29a771b` RX ownership, async fault race, RTS flow control; `6dbe299` breaks do not latch | Hardware-validated for EC ROM ISP and IO4→IO5 loopback |
| `dual-uart` | `main` (`2696a09`) | `b6bba7a` BPIO 2.3 UART channels; `3783b18`, `f6364b8` PLAN.md | Flashed (`3783b18`); EC + SoC consoles validated incl. power cycle |
| frameworkdebugger-library `buspirate-library` | | `bpconsoles.py` (`a28845e`, `2058d3c`, + recovery/`set_pin` commit) | Running bridge on the attached target |
| BPIO submodule `uart-channels` | upstream `36a5302` | `52f5528` schema + Python client | Local only; upstream is DangerousPrototypes |

A trial merge of the two firmware branches conflicts in `src/binmode/bpio.c`
and overlaps heavily in `bpio_uart.c`, `bpio_uart.h`, `hwuart.c` and the UART
tests: `dual-uart` replaced the single ring with per-channel state.

## Decisions

1. **Order:** finish and land `bpio-uart-rx-fixes` first (fixes on `main`,
   independently useful upstream), then rebase `dual-uart` onto it.
   Reason: the fixes are smaller, already validated, and the channel code is
   easier to port onto them than the reverse.
2. **Break semantics:** breaks never latch, on every channel. For consoles this
   removes the fault/recovery gap seen at EC reset and target power-off.
3. **Ownership:** channel 0 keeps the on-demand start from `29a771b` (works
   when UART mode was chosen from the terminal). Channel 1 starts only from an
   explicit `uart_channels` request. RTS flow control applies to channel 0
   only (RTS/CTS are IO7/IO6 and belong to UART mode).
4. **Hardware safety:** do not test RTS/CTS on the attached target. IO7 is
   wired to the EC flash strap (`ectarget.py`: IO7 via 4.7-10K to
   CR_SOUT1/FLPRG1) and IO6 is unknown. Flow control needs a bare loopback
   setup.
5. **Bridge policy:** initially reject terminal `bridge` entry while channel 1
   is enabled, before changing either channel. The existing blocking bridge
   loop cannot service BPIO; leaving UART1's IRQ enabled alone would overflow
   its ring. Concurrent terminal bridge support is deferred until that loop
   can service BPIO cooperatively with explicit RX ownership.
6. **Shared host transport is required:** concurrent EC flashing and SoC
   monitoring are acceptance requirements, so C3 is part of this plan.

## Phase A: harden `bpio-uart-rx-fixes`

A1. **Deliver bytes received before a fault.** Today `bpio_hwuart_rx_read()`
    returns 0 as soon as `rx_fault` is set, discarding everything already
    buffered. Record the head index when a fault first latches; let reads
    drain up to that point, then report the fault. For explicit-length
    transactions, stage received bytes without advancing the ring tail until
    the complete read succeeds. On timeout or fault, fail the transaction
    without consuming those staged bytes, allowing subsequent async delivery.
    Ensure the foreground request owns this reservation, IRQ overflow remains
    bounded, and no async/terminal reader can consume the same bytes meanwhile.
    Successful reads consume once; failed reads must not return partial data
    and also enqueue it asynchronously. Document the failed-read delivery
    semantics for clients, including recovery ordering.
    Tests: fault with N bytes buffered → N bytes delivered, then exactly one
    fault report, then nothing until restart. Also inject a fault or timeout
    after an explicit-length read has staged some bytes: those bytes arrive
    asynchronously exactly once; a successful read has no async duplicates.

A2. **Make a latched fault visible to the terminal.** When BPIO owns a faulted
    ring, `hwuart_read` (`r`) should report the fault text, not "no data", and
    async print should say once that RX is faulted. Clearing remains a mode
    change or BPIO reconfigure; document that.

A3. **Do not call flash code from the RX IRQ.** `rx_set_rts()` calls
    `bio_put()`, an ordinary flash function, from the `__not_in_flash_func`
    handler. Use `gpio_put(bio2bufiopin[M_UART_RTS], ...)` (inline) instead.

A4. **Hardware checks still missing from `6dbe299`'s notes:**
    - Leaving BPIO (`binmode` → other) stops IRQ RX; the terminal `r` then
      reads the hardware FIFO again.
    - Peripheral guard: with the terminal or debug UART on UART0, a BPIO
      `mode=UART` request fails with the new specific error text.
    - RTS flow control on a bare loopback (IO4→IO5, IO7(RTS)→IO6(CTS)), host
      not draining: RTS deasserts at high water, no overflow, data intact.
      **Not on the EC target** (decision 4).
    - Re-run the IO4→IO5 regression sweep and an EC ROM ISP backup/flash/
      readback with A1–A3 in, compare with the `6dbe299` table.

A5. Commit, then decide whether to open the upstream PR from `zoid` now or
    after Phase B (the user's call).

## Phase B: rebase `dual-uart` onto hardened `bpio-uart-rx-fixes`

B1. Rebase `b6bba7a` (keep the PLAN.md commits). Resolve by porting the
    single-ring behavior into the per-channel struct:
    - break handling (count, drop, no latch), fault-head drain (A1) and bulk
      `rx_read` for both channels;
    - `rx_flow`/RTS fields on channel 0 only, gated by
      `hwuart_flow_control_enabled()`;
    - `rx_ensure()` for channel 0 in transaction/async paths; channel 1 never
      starts implicitly;
    - async: keep `dual-uart`'s round-robin `bpio_hwuart_async_next()`; it
      already reports a fault only on an empty poll, matching `29a771b`.

B2. Ownership and lifecycle:
    - `mode_change_new()`: drop the explicit RX start (on-demand now) but keep
      the `uart_tx_disable` prescan so receive-only entry never drives IO4.
    - `dirtyproto_mode_cleanup()`: release channel 1 (pins and UART1) and stop
      channel 0 RX.
    - `bridge`: reject entry while channel 1 is active, before stopping channel
      0 RX or modifying pins. When channel 1 is disabled, preserve the legacy
      handoff of channel 0 RX to the bridge.
    - Terminal `hwuart_rx_try_get()` shares channel 0's ring only.
    - Keep `mode_change_error` reporting and the terminal/debug ownership guards
      from `29a771b`; check both UART peripherals before modifying hardware.
      Add channel errors in the same style.

B3. **Fix blocking cross-channel starvation (original review P1).** Replace
    blocking UART transactions with cooperative request progress: retain one
    pending synchronous request while servicing both RX queues and UART TX
    readiness. Async delivery must exclude bytes reserved for that request
    (A1). Preserve response ordering and legacy completion/timeout semantics;
    do not dispatch another synchronous request until the pending one finishes.
    Bound channel service latency below the RX ring fill time at supported
    baud rates. At 115200 8N1, 4095 bytes provide only about 356 ms: a one-second
    idle read or 512-byte write at 9600 baud must not prevent servicing the
    other channel. Define backpressure/overflow behavior when USB cannot drain.

B4. Tests: merge both suites into one `tests/test_bpio_uart.sh` run (two-UART
    mock, pin state, RTS mock). Must cover: breaks don't latch on either
    channel; fault-head drain and failed-read preservation per channel;
    round-robin fairness; silent synchronous read and slow TX on one channel
    during sustained RX on the other; terminal/debug conflicts on both UARTs;
    bridge rejection leaves both channels unchanged; legacy bridge handoff
    works with channel 1 disabled; leaving BPIO releases channel 1 and stops
    channel 0 IRQ RX; receive-only TX pins stay inputs; legacy single-UART
    sequence unchanged. Exercise the BPIO dispatch and lifecycle paths as well
    as the UART helper functions.

B5. Nix build, check RAM (currently 91.3% on `dual-uart`), flash, verify the
    reported hash.

## Phase C: host software

C1. `bpconsoles.py`: with breaks no longer latching, recovery should be rare;
    keep the backoff. Show break counts when a fault text includes them.
    Standalone break-counter queries require an appended protocol field or
    diagnostics request; defer that extension. Break-only tests can inspect
    counters in the firmware mock; hardware tests verify uninterrupted output
    and absence of break-induced fault packets.
    - **Atomic configuration/recovery (original review P2):** lock the complete
      read-current-config, send-config, update-cache operation. Recovery must
      acquire the same lock before reading configuration so it cannot restore
      stale TX-enable state after `disable_tx()`. Add a deterministic concurrent
      disable/recovery test that checks applied and cached state.
    - **Initialization rollback (original review P2):** track when hardware
      configuration begins. On a later constructor failure, attempt HiZ before
      closing the port, leaving PSU/pull-ups unchanged. Preserve the original
      exception and report cleanup failure separately. Capability rejection
      before configuration must not modify hardware. Test second-channel
      failure after EC TX enable and failed cleanup.

C2. `ectarget.py`:
    - `_PtyBridge`: put the slave in raw mode with `TCSANOW` (same echo-loop
      bug fixed in `bpconsoles.py`).
    - `enter_flash_mode()`: the post-strap `mode="UART"` re-entry existed only
      to clear the break fault. With `6dbe299` it is unnecessary (ISP entry
      verified without it); remove it. Audit all mode re-entry paths, including
      initialization and teardown, so the shared-transport path never releases
      channel 1. Reconfigure channel 0 through `uart_channels` when needed;
      retain a compatible mode-based path for legacy firmware. Keep IO3/IO7
      restore and exclusive EC binary-reader ownership during ROM ISP.
    - Add an explicit externally-powered option (no PSU enable), default
      unchanged.

C3. Required: share one BPIO connection between `EcTarget` (channel 0, flash/
    ROM ISP) and `BusPirateConsoles` (channel 1) so EC flashing can run while
    the SoC console stays open. Inject the existing transport/channel into
    `EcTarget` instead of opening the physical port again. Use one request lock
    and one receive dispatcher; route EC bytes exclusively to the ROM/flash
    reader while it owns channel 0, and continue feeding the SoC PTY. Closing
    the EC adapter must not close the shared port, enter HiZ, disable the PSU,
    or release channel 1. The transport owner performs final global teardown.
    Test attach, flashing success/failure, and adapter close with SoC traffic.

C4. BPIO submodule: decide on a fork for `uart-channels`, update the library's
    `.gitmodules` URL accordingly.

## Phase D: hardware validation on the attached target

Precondition: user ends any interactive `tio` sessions; bridge stopped by PID.
PSU and pull-ups stay off throughout; IO6/IO7 untouched except by `EcTarget`'s
documented strap handling.

1. Both consoles receive-only, then TX enabled; buffer directions as before.
2. Concurrent traffic: SoC `seq 1 5000` while EC answers commands; intact.
   Also test a silent read and slow TX on one channel while the other receives
   sustained data; record drops, faults, and maximum service latency.
3. Power cycle (IO3 reset + `powerbtn`): a break must not latch a fault. Capture
   EC from the Zephyr banner and SoC from the coreboot banner. Distinguish
   genuine framing/overrun faults from break-associated error flags; power
   transitions can cause errors other than breaks. Standalone break counts
   are not exposed by the initial protocol. Verify no unexplained output gaps
   against expected boot markers; do not equate no fault packets with no loss.
4. EC ROM ISP backup + flash + readback via `EcTarget` with the SoC console
   open on channel 1: SoC console survives; timings comparable to `6dbe299`.
5. Legacy client: plain `EcTarget` (no channels) still works unchanged.
6. Record firmware hash, configuration, logs and results in PLAN.md.

## Acceptance criteria

- Pre-fault buffered bytes, including staged bytes from failed explicit reads,
  are delivered exactly once before the async fault report. Faults are
  observable from both BPIO and terminal paths, without repeated-report floods
  or one consumer hiding the fault from the other. Overflow remains an explicit
  loss condition; reconfiguration intentionally clears remaining buffers.
- Breaks never latch on either channel; real framing/parity/overrun faults
  remain observable. Reset/power-transition captures contain expected boot
  markers without unexplained gaps.
- `bpio-uart-rx-fixes` behavior (terminal-selected UART mode, bridge handoff,
  binmode cleanup, flow control) holds with the explicit lifecycle policy:
  bridge entry is rejected while channel 1 is active; leaving BPIO releases
  channel 1 and stops channel 0 IRQ RX.
- Silent reads and slow TX do not starve the other channel while host USB
  reception is available; buffer exhaustion is reported explicitly.
- Concurrent recovery cannot undo a completed TX-disable request, and failed
  initialization attempts cleanup without changing PSU/pull-up state.
- EC flashing does not disturb the SoC console.
- Unit tests pass under ASan/UBSan; Nix build OK; legacy clients unchanged.

## Open questions for the user

- Upstream PR for `bpio-uart-rx-fixes` before or after Phase B?
- Is a bare Bus Pirate loopback available for the RTS/CTS test (A4)?
