# EP-MAIN-FW v3.0.0 — Full Feature Release

## Overview

Firmware for NUCLEO_G474RE (Mbed OS, ARMC6) providing a complete BMS control stack:

- Robust state machine for power/contactor control
- SOC estimation (OCV initialization + coulomb counting with guarded recalibration)
- Auto-balancing with hardware control and visual indication
- Protections for voltage, temperature, and current
- SD card logging (event stream + periodic/triggered snapshots)
- Human-friendly UART telemetry
- RGB LED policies
- Buzzer effect engine for alerts

Units and sign conventions:

- Voltage: mV
- Current: mA (negative = charge, positive = discharge)
- Temperature: milli-Celsius (m°C); when printed, shown in °C
- SOC: 0..1000 (0.1% increments)

---

## Build & Run

- Target: `NUCLEO_G474RE`
- Toolchain: ARMC6 (Mbed CLI)
- Build: `mbed compile -m NUCLEO_G474RE -t ARMC6`
- Threads started: BQ + Tasking + Logging + Operation + Interface + Measurement (see `main.cpp`)

Quick checks after flashing:

1. Insert SD card (FAT). CSV files are created on first write.
2. Observe UART (250000 baud) for human-friendly lines (voltages, SOC, current, capacity, slope).
3. Exercise states (charge, discharge, pre-charge, power down) and verify LED/buzzer policies.

---

## Operation State Machine

States:

- INIT, PRE_CHARGE, LOAD_ENABLED, CHARGING, BALANCING, CHARGED, BATTERY_DEAD, ERROR, ERROR_PRECHARGE, POWER_DOWN, FORCEON

Highlights:

- Pre-charge sequencing with retries and debounced contactor control
- Guarded CHARGING disconnect (5 s entry guard; deferred disconnect timer)
- Clean POWER_DOWN (switches off, LEDs off, power enable deassert)
- Transitions trigger one-shot snapshots to SD

Key files:

- `Operation/Inc/operation_params.h` (enums)
- `Operation/Src/operation.cpp` (logic)

---

## SOC Calculation

Source of truth:

- `u16ModuleSoc` (0..1000, x0.1%)
- Remaining capacity in Ah
- Pack current `i32ModuleCurrent` (mA; negative = charging)

Algorithm:

- Initialization (OCV):
  - IR-compensated cell OCV estimate via `StateOfChargeHandler`
  - Saturated to [0..1000], used to seed remaining Ah
  - Bootstrap only if no valid SOC is loaded (EEPROM/journal) after stable idle voltage dwell
  - If SOC is known, boot does not accept OCV; runtime trims are step-limited
  - Recalibration (idle+stable+confidence+throughput+cooldown) applies OCV with a capped step
- Coulomb counting (runtime):
  - Integrates current with a bounded dt; remaining Ah clamped to [0..capacity]; SOC recomputed each cycle
- Persistence (EEPROM):
  - When SOC changes by ≥ 1.0% (10 in x0.1% units), store remaining capacity to EEPROM (mAh, split across two words)
  - On boot, restore last remaining capacity if valid

Interfaces & refs:

- `StateOfChargeHandler/Inc/state_of_charge_handler.h`
- `StateOfChargeHandler/Src/state_of_charge_handler.cpp`
- `Measurement/Src/measurement.cpp`

---

## Balancing

Policy:

- Start at dv ≥ 45 mV; stop at dv ≤ 30 mV (with current-state gating)
- Selection maps per cell (top group during charge; above vmin when idle)
- Concurrency caps (tunable in `Measurement/Inc/balancing_tuning.h`):
  - Per-device max `MAX_ACTIVE_CELLS_PER_DEV` (default 8)
  - Global max `MAX_ACTIVE_CELLS_TOTAL` (default 24)
- Optional parity selection to avoid adjacent cells when `NO_ADJACENT_CELLS` is enabled
- Balancing has clean-measure windows and periodic pauses for stable reads
- Exit balancing if no eligible cells persist for `NO_BLEED_EXIT_MS`

Hardware Control:

- BQ79616 handler programs timers/duty and drives bleeders; firmware provides the active cell mask.

Indicators:

- RGB: BLUE flash while auto-balancing is active

Files:

- `Measurement/Src/measurement.cpp`
- `BQ79616Handler/Inc/IMbq79616.h`
- `BQ79616Handler/Inc/BQ79616_handler.h`
- `Operation/Src/operation.cpp`

---

## Protections

Voltage:

- Soft/hard UV/OV thresholds; hysteresis configurable and applied per mode

Temperature:

- Charge/discharge min/max with hysteresis (evaluated on min/max pack temps)

Current:

- Soft/hard charge/discharge limits derived from capacity; used in SOA and gating decisions

Overvoltage State:

- NORMAL / TRIPPED / RECOVERY; used to clamp behaviors and throttle

SOA Flags:

- `bPackInSOADischarge` / `bPackInSOACharge` drive behavior and event logging

Files:

- `Settings.h` (thresholds/hysteresis)
- `Measurement/Src/measurement.cpp` (protection loops)

---

## LEDs

### RGB LED (state color)

- LOAD_ENABLED: solid GREEN (healthy/online)
- CHARGING: GREEN flash
- BALANCING: BLUE flash (priority over green to indicate activity)
- FORCEON (dev): BLUE flash
- ERROR: solid RED
- ERROR_PRECHARGE: RED fast flash
- POWER_DOWN: all off
- INIT/PRE_CHARGE: GREEN on while pre-charging

Files:

- `Operation/Src/operation.cpp`
- `EffectHandler/Src/effect_handler.cpp`

Engine:

- Exclusive color channels; optional PWM effects (`ENABLE_RGB_PWM_EFFECTS`)
- Active-low wiring handled via `RGB_LED_ACTIVE_LOW`

---

## Buzzer

Behavior:

- Controlled centrally in operation thread
- Patterns: ON, OFF/RESET, FLASH (~100 ms), FLASH_FAST (~50 ms)
- Enabled via `bBuzzerEnabledState`, pattern via `buzzerSignalType`
- Output is digital gating (beeps), not tonal PWM

Files:

- `Operation/Src/operation.cpp` (callback)
- `EffectHandler/Src/effect_handler.cpp` (effect dispatch)

Optional — Auto-map States to Patterns:

- Recommended mapping (kept quiet in normal operation):
  - `ERROR`: `FLASH_FAST` (continuous fast beeps until cleared)
  - `ERROR_PRECHARGE`: `FLASH_FAST`
  - `PRE_CHARGE`: `FLASH` (slow beep during pre-charge)
  - `BATTERY_DEAD`: periodic chirp (e.g., `FLASH` for 100 ms every few seconds)
  - `POWER_DOWN`: single short chirp on entry, then OFF
  - `FORCEON`: single short chirp on entry (debug), then OFF
  - `CHARGING` / `LOAD_ENABLED` / `BALANCING`: `OFF`

---

## SD Card Logging

Event Log (human-readable):

- File: `EVENTS_LOG.csv` (columns: `timeLogging,event`)
- Change-gated events only:
  - Operation state: `OpState->INIT/.../FORCEON`
  - BMS state: `BMS->IDLE/DISCHARGE/CHARGE`
  - OV state: `OV->NORMAL/TRIPPED/RECOVERY`
  - Inverter response: `INV->OK/FAULT`
  - Charger presence: `ChargerDetected->ON/OFF`
  - Charge balance window: `ChargeBalanceActive->ON/OFF`
  - SOA: `SOA_D->IN/OUT`, `SOA_C->IN/OUT`
  - Powerdown flag: `PowerdownFlag->SET/CLR`
  - SOC save request: `SOC_SaveReq->SET/CLR`
  - SD write errors: `SD_ERR_CELL:x`, `SD_ERR_TEMP:x`, `SD_ERR_SYS:x`

Snapshots:

- Periodic: every 30 s (system info + pack temperatures)
- One-shot: on op-state change and balancing start/stop

Performance:

- Throttled to ~1 write/s
- Per-event open/write/close; quickly returns if SD not present

Files:

- `Logging/Src/logging.cpp`
- `SdCardHandler/Inc/ILSdCard.h`
- `SdCardHandler/Src/sdcard_handler.cpp`

---

## UART Telemetry (Human-Friendly)

- Status line prints:
  - Vmin/Vmax (x.xxx V), SOC (y.y%), Temp (°C), Current (±a.bb A), Remaining capacity (Ah), Capacity (Ah), Slope (mAh/s and ~A)
- SOC and current sign conventions applied (neg for charge)

File:

- `Measurement/Src/measurement.cpp`

---

## BMS Web Viewer (logs, dashboard, SD analyzer)

Local, no-internet viewer that renders live UART logs, charts, and SD-card CSVs.

Current UI: `bms_viewer_v2/` (Tauri app). Use the scripts in `bms_viewer_v2/package.json`
to run or build the viewer. The legacy Python viewer is archived as `bms_viewer_old.bak`.

Notes:

- The Dashboard shows a balancing status chip (bal: ON/OFF) based on parsed terminal lines:
  - `[BAL-SEL] dv=... Vbat=... Target=... Above=...`
  - Per-pack selections and `C#=####` cell entries
- The Dashboard charts sample locally at ~1 Hz from parsed values; CSV export and PNG save are available.

Protections interplay visible in logs:

- dV trend lock clamps charging when dv rises while charging or is very large; unlocks after stability.
  Timing/thresholds are tunable in `Measurement/Inc/balancing_tuning.h`
  (`DV_UNLOCK_MV`, `DV_STOP_HOLD_MS`, `DV_RECOVERY_HOLD_MS`, `DV_UNLOCK_RAMP_HOLD_MS`).
  Logs include `[DV-LOCK] ...` messages for traceability.
