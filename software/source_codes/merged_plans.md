# Merged Plan (Updated, 54Ah Cells)

## Scope
First‑party firmware + tooling only:
- Firmware: `Operation`, `Measurement`, `Logging`, `PowerHandler`, `PowerStateHandler`, `InterfaceCommHandler`, `SerialHandler`, `SdCardHandler`, `ErrorHandler`, `Configuration`, `StateOfChargeHandler`, `main.cpp`
- Tooling/UI: `bms_viewer.py`

Assumptions confirmed:
- **Battery capacity: 54Ah** (align all SOC, CAN limits, and testing with this value).

Cell spec constraints (L194F54):
- Rated capacity 54.0 Ah (standard discharge shows 55.4 Ah), rated 3.20-3.22 V.
- Voltage limits (cell): operation charge 3.65 V; operation discharge 2.3 V (>10C) / 2.0 V (<=10C). Safety charge 3.8 V; safety discharge 1.9 V (>0C) / 1.6 V (<=0C).
- Temperature limits: charge -30..55C (operation), -35..60C (safety); discharge -35..55C (operation), -40..60C (safety); storage -40..60C.
- Internal resistance: ACR <= 0.45 mOhm, DCR <= 1.3 mOhm.
- OCV at 25C (table): discharge 100% 3.520 V, 50% 3.307 V, 20% 3.223 V, 10% 2.921 V; charge 100% 3.429 V, 50% 3.294 V, 20% 3.256 V, 10% 3.210 V, 0% 2.704 V.
- Current limits are SOC/temperature dependent; implement as a lookup or policy (example at 25C, 50% SOC: discharge peak 861 A for 10s, continuous 344 A for 180s; charge peak 336 A for 10s, continuous 135 A for 180s).

---

## Phase 0 — Critical Safety & Stability (must‑do before refactors)
### 0.1 Lock down remote UART control (bms_viewer)
- Default bind to localhost or require explicit opt‑in for LAN exposure.
- Add auth/token for WS and `/api/serial/select` or default to read‑only.
- Emit warnings when bound to non‑loopback or when write‑enabled.

### 0.2 Fix unsafe `SerialHandler::vsend`
- Replace `vsprintf` usage; format once with `vsnprintf` into bounded buffer.
- Avoid double use of `va_list`.

### 0.3 Initialize SD filename pointer
- Initialize `m_cPtractiveFileName` to `nullptr` in constructor to avoid `free()` on garbage.

### 0.4 Contactor dwell logic consistency
- Ensure `switchesDisableAll()` updates/clears the same dwell/last‑state tracking used by `setCharge/setDisCharge/setPreCharge` so contactors can re‑enable after forced disable.

### 0.5 EEPROM error index bounds
- Cap or rotate error records to prevent overwriting settings or SOC journals.
- Define maximum record count from EEPROM layout.

### 0.6 Remove heap allocation in ISR (BQ handler)
- Replace `std::vector` allocations inside ISR with raw buffer CRC checks (use existing `verifyCRC_raw()`).
- Rationale: heap use in ISR is non-deterministic and risks hard faults.

---

## Phase 1 — SOC Accuracy & Persistence (54Ah aware)
### 1.1 OCV confidence scoring (LiFePO4 plateau)
- Add confidence metric to OCV‑based SOC and only apply when confidence is high.
- Use the 25C OCV table; avoid recalibration in the plateau (approx 3.32-3.34 V). Prefer recal near the ends (e.g., >=3.40 V high SOC, <=3.23 V low SOC discharge).

### 1.2 Drift compensation and idle offset
- Auto‑calibrate current offset only after **verified idle** (noise floor + stable period).
- Add charge/discharge efficiency factor (documented).

### 1.3 SOC persistence strategy (align with existing journal)
- Reconcile SOC journal in `EepromHandler` with any “rate limiting” logic.
- Decide on a single source of truth and explicitly document it.
- Force save on power‑down but avoid excessive writes during normal operation.

### 1.4 Centralize SOC tunables
- Create a `soc_tuning.h` (or equivalent) with all SOC constants.
- Ensure values reflect 54Ah capacity and LiFePO4 characteristics.

---

## Phase 2 — Balancing & Charge Control
### 2.1 Smooth cell selection
- Replace abrupt tiered windows with smooth dv‑based windowing.

### 2.2 Balance progress tracking
- Add progress logging for dV decay rate and ETA.

### 2.3 dV trend lock refactor
- Extract to a small class with explicit states, hysteresis, and logs.

### 2.4 Centralize balancing tunables
- Create `balancing_tuning.h` (or equivalent).
- Enforce BQ79612 manual balancing constraints: max 8 channels. Adjacency rules depend on DEV_CONF[NO_ADJ_CB]; if set, disallow adjacent channels; otherwise allow adjacent pairs but never >2 consecutive. Integrate balancing thermal pause (TCB_WARN ~105C, 10C hysteresis). Base expected balancing current on spec (~240 mA at 75C, lower when hotter or with higher series R).

---

## Phase 3 — Protection System Improvements
### 3.1 Consolidate protection evaluation
- Build a single protection engine to remove duplicated state handling.

### 3.2 Severity levels and behavior mapping
- Define severity → behavior (throttle/soft trip/hard trip) with explicit integration into `Operation` state machine.
- Align UV/OV and temperature thresholds with the L194F54 operation/safety limits above; document any intentional deviations.

Proposed protection thresholds (cell-level, L194F54):

| Protection | Threshold | Severity | Behavior |
|-----------|-----------|----------|----------|
| Soft UV | 2.3 V | THROTTLE | Limit discharge current |
| Hard UV | 2.0 V | SOFT_TRIP | Open contactors, auto-retry |
| Safety UV | 1.9 V | HARD_TRIP | Open contactors, manual reset |
| Soft OV | 3.65 V | THROTTLE | Limit charge current |
| Hard OV | 3.75 V | SOFT_TRIP | Open contactors |
| Safety OV | 3.8 V | HARD_TRIP | Open contactors, manual reset |
| Balancing overtemp | >105 C | PAUSE_BAL | Pause balancing only |

### 3.3 Enhanced protection logging
- Include thresholds, current value, and time‑in‑state in logs.

---

## Phase 4 — Logging, Concurrency, and Data Integrity
### 4.1 SD safe‑eject coordination
- Add logging shutdown coordination or lock to prevent unmount while writing.

### 4.2 Pack info thread safety
- Use a mutex or snapshot model consistently for pack data used across threads.

### 4.3 Interface TX queue safety
- Protect `txEnqueue` with a mutex or use a thread‑safe queue to avoid corruption under concurrent prints.

### 4.4 bms_viewer data correctness
- Fix CSV export line breaks.
- Fix current scaling heuristics (use explicit units or known source format).
- Accept ISO values reported as `MOhm` (and normalize other variants).
- Clarify “All” window if sample cap remains.
- Bound `out_q` to prevent memory growth with no clients.

---

## Phase 5 — Structural Refactor (bms_viewer)
### 5.1 Module split
- Entry point + HTTP/WS handlers + parsers + HTML templates.

### 5.2 New features (optional)
- Per‑cell charting, SOC confidence indicators, protection panel, config editor.

---

## Implementation Order (Suggested)
1. Phase 0 (all items)
2. Phase 1.3 (SOC persistence reconciliation)
3. Phase 1.1–1.4
4. Phase 2.1–2.4
5. Phase 3.1–3.3
6. Phase 4.1–4.4
7. Phase 5.1–5.2

---

## Validation Checklist (updated)
### SOC
- Full charge → SOC 100% (54Ah)
- Discharge at ~1C equivalent for 54Ah → linear tracking
- Idle drift test overnight (offset + efficiency)
- Power cycle at 50% → restore from journal correctly
- OCV recal only at knees (≈3150mV/3350mV)
- OCV recal only at ends per spec table (avoid plateau around 3.32-3.34 V)

### Balancing
- Inject 100mV imbalance → start/stop thresholds correct
- Verify progress logging and ETA behavior
- dV‑lock transitions with hysteresis

### Protections
- Trigger each protection condition and confirm correct severity behavior
- Verify enhanced logs include thresholds + context

### SD/Logging
- Logging during shutdown → safe eject without corruption
- Log rotation and error records remain within EEPROM bounds

### bms_viewer
- CSV export opens correctly in spreadsheet
- Current units displayed correctly
- ISO lines with Ω parse correctly
- ISO lines with MOhm parse correctly
- Memory stable with no clients connected
