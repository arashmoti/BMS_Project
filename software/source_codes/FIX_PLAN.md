# Fix Plan (Detailed)

## Scope
This plan targets the issues found in the first‑party firmware/app code (not vendor sources or build artifacts):
- `bms_viewer.py`
- `SerialHandler`, `InterfaceCommHandler`, `SdCardHandler`, `ErrorHandler`, `PowerHandler`, `Operation`, `Measurement`, `Logging`, `Tasking`, `Configuration`, `StateOfChargeHandler`, `main.cpp`

## Priority Order (Why)
1. **Safety/Security**: remote UART control exposure and unsafe printf usage can directly affect hardware or crash the system.
2. **Data integrity**: EEPROM error log overflow and SD eject races risk data loss/corruption.
3. **State correctness**: contactor dwell logic, charger guard, current scaling, logging correctness.
4. **Reliability/perf**: threading/locking consistency, queue growth, misleading UI.

---

## Phase 1 — Safety/Security Hardening
### 1) Lock down network control in `bms_viewer.py`
**Goal:** Prevent unauthorized command/control over UART.
- **Default bind**: change default `--host` to `127.0.0.1` or require explicit `--host 0.0.0.0`.
- **Auth/token**: introduce a simple bearer token (query param or header) or a local‑only toggle to gate `/api/serial/select` and WS send.
- **Read‑only default**: optionally default to `--read-only` and require flag to enable sending.
- **Explicit warning**: log a prominent warning when binding non‑loopback or when `read_only` is false.
- **Test/validation:** manual run with/without token and confirm unauthorized clients cannot send UART commands.

### 2) Fix unsafe `SerialHandler::vsend`
**Goal:** Eliminate UB and overflow risk.
- Replace `vsprintf` with `vsnprintf`.
- Call formatting once; don’t reuse the `va_list`.
- Ensure written length matches buffer size.
- **Test/validation:** send long format strings; verify no crash and correct truncation.

### 3) Initialize SD active filename pointer
**Goal:** Remove potential crash on first use.
- Initialize `m_cPtractiveFileName` to `nullptr` in constructor.
- Guard `free()` in `setActiveFile` (already checks for NULL, but it must start NULL).
- **Test/validation:** exercise SD logging from cold boot; no crash on first write.

---

## Phase 2 — Data Integrity & Safe Shutdown
### 4) Bound/rotate EEPROM error log index
**Goal:** Prevent overwrite of unrelated EEPROM regions.
- Define a max number of error records based on EEPROM layout.
- Wrap/rotate `m_u8ErrorIndex` and `m_u16ErrorStartAddressToWrite` when max reached.
- Optionally track “overwrite oldest” behavior.
- **Test/validation:** write >max errors; ensure address never crosses reserved region.

### 5) Coordinate SD `safeEject()` with logging
**Goal:** Avoid unmount while logging writes.
- Add a shared SD access lock or a “shutdown in progress” flag.
- Logging should stop writes once shutdown/eject begins.
- Ensure `safeEject()` is invoked after logging thread acknowledges shutdown.
- **Test/validation:** trigger shutdown during heavy logging; verify SD is unmounted safely and files remain readable.

---

## Phase 3 — State/Logic Correctness
### 6) Contactor dwell consistency in `PowerHandler`
**Goal:** Ensure contactors can re‑enable after forced disable.
- When calling `switchesDisableAll()`, also reset or update the static `*_LastState`/timestamps used by `setCharge/setDisCharge/setPreCharge`.
- Alternatively route all contactor changes through unified APIs that update state.
- **Test/validation:** force disable then re‑enable; verify contactors change as expected.

### 7) Re‑arm charger disconnect guard correctly
**Goal:** Prevent false disconnect on later CHARGING sessions.
- Reset `guard_until_ms` when leaving CHARGING or entering CHARGING regardless of prior guard state.
- Consider tying guard to a state transition rather than static value.
- **Test/validation:** enter CHARGING → exit → re‑enter; ensure 5s guard applies both times.

### 8) Fix current scaling heuristic in UI
**Goal:** Ensure plotted current is not mis‑scaled.
- Remove ambiguous integer‑vs‑mA heuristic, or use explicit unit tags from logs.
- If logs always in mA, always divide by 1000; if in A, never divide.
- **Test/validation:** feed known current values and verify UI plots match expected amps.

### 9) CSV export formatting
**Goal:** Produce valid CSV in dashboard export.
- Join rows with `\n` instead of a space.
- **Test/validation:** export and open in spreadsheet; rows parse correctly.

---

## Phase 4 — Concurrency & Reliability
### 10) Protect shared pack info with mutex
**Goal:** avoid race conditions / torn reads.
- Wrap Measurement updates to `m_ptrPowerElecPackInfoConfig` in `g_packInfoMutex` where it is read by other threads.
- Alternatively create a snapshot struct and swap atomically.
- **Test/validation:** stress concurrent logging + measurement; ensure no inconsistent values.

### 11) Make `txEnqueue` thread‑safe for human console
**Goal:** avoid buffer corruption and lost bytes.
- Add a small mutex around `m_txQueue` or use a thread‑safe queue.
- Avoid calling `txEnqueue` concurrently without a lock.
- **Test/validation:** simultaneous high‑rate `printToInterface` calls; confirm output remains coherent.

### 12) Bound `out_q` growth in `bms_viewer.py`
**Goal:** prevent unbounded memory usage.
- Replace `queue.Queue()` with a bounded queue or add backpressure/overflow behavior (drop oldest/newest).
- **Test/validation:** run with no clients for extended time; confirm memory stays stable.

---

## Phase 5 — UX/Clarity
### 13) Dashboard “All” window messaging
**Goal:** avoid misleading UI.
- If samples are capped at 1000, rename “All” to “Last 1000” or increase sample retention.
- **Test/validation:** verify UI label matches actual behavior.

### 14) ISO parser for Ω
**Goal:** parse isolation lines consistently.
- Update regex to accept `MΩ` and `MOhm` variants (and keep sanitize compatibility).
- **Test/validation:** use sample log line with `MΩ` and confirm it parses.

---

## Recommended Validation Checklist (After Fixes)
- Smoke run: boot firmware, observe serial logs, no crashes.
- SD logging: create logs, rotate, and safe‑eject during activity; verify file integrity.
- State machine: cycle through INIT → PRE_CHARGE → LOAD_ENABLED → CHARGING → BALANCING → POWER_DOWN.
- UI: verify logs page, dashboard charts, CSV export, and SD analyzer open correctly.
- Security: attempt WS/HTTP actions from another machine without token; ensure blocked.

---

## Deliverables
- Updated source files with changes above.
- Short changelog of fixes.
- Optional minimal tests or scripts (if desired).
