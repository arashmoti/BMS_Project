# BMS Firmware Complete Refactoring Plan

*Final unified plan: Gemini Deep Dive + ChatGPT Electrical Specs*

## 1. Project Specifications (L194F54 LiFePO4)

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Capacity** | **54.0 Ah** | Standard discharge: 55.4 Ah |
| **Op Voltage** | 2.0V - 3.65V | Discharge limit depends on C-rate (>10C: 2.3V) |
| **Safety Voltage** | 1.6V - 3.8V | Absolute hardware trip limits |
| **Temp Charge** | -30°C to 55°C | Safety: -35°C to 60°C |
| **Temp Discharge**| -35°C to 55°C | Safety: -40°C to 60°C |
| **Cell Monitor** | BQ79616 | Max 8 concurrent channels, **NO ADJACENT CELL BALANCING** |
| **Chemistry** | LiFePO4 | Flat plateau 3.22V - 3.32V (20-80% SOC) |

---

## Verification Summary

| # | Issue | Status | Location |
|---|-------|--------|----------|
| 1 | ISR heap allocation | ✅ CONFIRMED | `BQ79616_handler.cpp:489` |
| 2 | Unsafe vsprintf | ✅ CONFIRMED | `serial_handler.cpp:105-111` |
| 3 | SD filename uninitialized | ✅ CONFIRMED | `sdcard_handler.h:75` |
| 4 | Contactor dwell reset | ✅ CONFIRMED | `power_handler.cpp:92-222` |
| 5 | TX queue multi-producer | ✅ CONFIRMED | `interface_comm_handler.cpp:624` |
| 6 | Pack info mutex gaps | ✅ CONFIRMED | Multiple files |
| 7 | UART remote control | ⚠️ RISK | No auth on WS/API |

---

## PHASE 0: Critical Safety ⚠️ DO FIRST

### 0.1 Fix ISR Heap Allocation 🔴 CRITICAL

**File:** `BQ79616Handler/Src/BQ79616_handler.cpp:489`

**Current Code (DANGEROUS):**

```cpp
// Inside BQ_ISR() interrupt handler
vector<uint8_t> message(rxData, rxData + rxIndex);  // HEAP ALLOCATION IN ISR!
if (verifyCRC(message)) { ... }
```

**Why This Is Dangerous:**

- `std::vector` allocates memory from heap at runtime
- In ISR, this is non-deterministic and can fragment heap
- Can cause hard faults or watchdog timeouts

**Fix:** Use existing `verifyCRC_raw()` static function:

```cpp
if (verifyCRC_raw(rxData, rxIndex)) { ... }
```

**Effort:** 30 min | **Risk:** LOW

---

### 0.2 Fix vsprintf Buffer Overflow 🔴 CRITICAL

**File:** `SerialHandler/Src/serial_handler.cpp:102-115`

**Current Code (DANGEROUS):**

```cpp
if (vsprintf(m_sendBuffer, format, args) < 0) { return false; }
// DOUBLE USE of va_list without reset!
m_serial.write(m_sendBuffer, vsprintf(m_sendBuffer, format, args));
```

**Fix:**

```cpp
bool SerialHandler::vsend(const char *format, va_list args) {
    char m_sendBuffer[512];
    int len = vsnprintf(m_sendBuffer, sizeof(m_sendBuffer), format, args);
    if (len < 0 || len >= (int)sizeof(m_sendBuffer)) return false;
    lock();
    m_serial.write(m_sendBuffer, len);
    unlock();
    return true;
}
```

**Effort:** 30 min | **Risk:** LOW

---

### 0.3 Lock Down Remote UART (bms_viewer) 🟡 HIGH

**File:** `bms_viewer.py`

**Fix Strategy:**

1. **Bind:** Default to `127.0.0.1`. require `--host 0.0.0.0` for LAN.
2. **Auth:** Add bearer token for WebSocket and `/api/serial/select`.
3. **Mode:** Default to `--read-only`. Warn loudly if write-enabled.

**Effort:** 1 hr | **Risk:** LOW

---

### 0.4 Initialize SD Filename Pointer

**File:** `SdCardHandler/Inc/sdcard_handler.h:75`
**Problem:** `char* m_cPtractiveFileName` uninitialized → crash on `free()`.
**Fix:** Initialize to `nullptr` in constructor.

---

### 0.5 Contactor Dwell State Reset 🟡 CONFIRMED BUG

**File:** `PowerHandler/Src/power_handler.cpp`

**Root Cause:**
`switchesDisableAll()` turns off pins but **DOES NOT** reset the static `chargeLastState` or timestamps used in `setCharge()`.

**Bug Scenario:**

1. `setCharge(true)` → `chargeLastState = true`
2. Error → `switchesDisableAll()`
3. Recovery → `setCharge(true)`
4. `chargeLastState` is still `true`, so logic thinks "no change needed"
5. **Contactor stays OFF.**

**Fix:**

```cpp
void PowerHandler::switchesDisableAll(void) {
    m_preDischargeEnPin.write(0);
    // ...
    resetDwellState(); // New method to clear static/member tracking vars
}
```

**Effort:** 2 hr | **Risk:** MEDIUM

---

### 0.6 Bound EEPROM Error Index

**File:** `ErrorHandler/Src/error_handler.cpp:88`
**Fix:** Wrap `m_u8ErrorIndex` at `MAX_ERROR_RECORDS` (e.g. 30) to prevent overwriting settings.

---

## PHASE 1: SOC Accuracy (54Ah LiFePO4)

### 1.1 OCV Confidence Strategy

**L194F54 Characteristics:**

- **Plateau:** 3.22V - 3.32V (approx 20-80% SOC). **Action:** Disable OCV recalibration here.
- **Recalibration Zones:**
  - **High:** >3.40V (Charge knee)
  - **Low:**  <3.23V (Discharge knee)
- **Table:** Use specified 25°C OCV points:
  - 100%: 3.520V / 3.429V (D/C)
  - 50%: 3.307V / 3.294V
  - 10%: 2.921V / 3.210V

### 1.2 Drift Compensation

- **Zeroing:** Auto-calibrate current sensor offset during verified idle (noise floor check).
- **Efficiency:** Apply 99.5% factor for charge, 100% for discharge.

### 1.3 Persistence

- Reconcile `EepromHandler` journal with Measurement logic.
- **Rate Limit:** Max 1 write per minute.
- **Force Save:** On power-down request.

---

## PHASE 2: Balancing (BQ79616)

### 2.1 Constraints Enforcement (New)

1. **Max Channels:** 8 concurrent cells per module.
2. **Adjacency:** **NO ADJACENT CELLS** (Hardware constraint).
    - *Algo change:* If cell `i` needs balance, skip `i-1` and `i+1`.
3. **Thermal:** Pause if T_CB > 105°C (resume < 95°C).

### 2.2 Smooth Algorithm

**Current:** Abrupt tiered windows (10/20/30mV).
**Proposed:**

```cpp
// Target top 25% of deviation
uint16_t target = (vmax - avg) * 0.5f + avg;
if (v > target && v > avg + 10mV) bleed = true;
```

### 2.3 Progress Tracking

- Log `dV` at start vs current `dV`.
- Calculate decay rate (mV/min) and estimate time to target (30mV).

---

## PHASE 3: Thread Safety ⚠️ DEEP DIVE

### 3.1 TX Queue Multi-Producer Fix 🟡 CONFIRMED

**File:** `InterfaceCommHandler/Src/interface_comm_handler.cpp:624`

**Problem:** Multiple threads (`Logging`, `Operation`) call `txEnqueue` concurrently. Mbed `CircularBuffer` is NOT thread-safe for multiple producers.

**Fix:**

```cpp
void InterfaceCommHandler::txEnqueue(const uint8_t *data, size_t len) {
    m_txMutex.lock();  // ← Add mutex
    // ... push to queue ...
    m_txMutex.unlock();
    m_txSem.release();
}
```

### 3.2 Pack Info Mutex Consistency

**Problem:** `g_packInfoMutex` used in `logging.cpp` line 211, but ignored in line 269 (Live Rack) and 563 (Interface).

**Fix Strategy:**

1. **Measurement (Writer):** Lock during aggregate updates.
2. **Logging (Reader):** Take snapshot under lock:

    ```cpp
    g_packInfoMutex.lock();
    auto snapshot = *m_ptrPowerElecPackInfoConfig;
    g_packInfoMutex.unlock();
    // use snapshot...
    ```

**Effort:** 4 hr | **Risk:** MEDIUM

---

## PHASE 4: Protection System

### 4.1 Consolidated Engine

Single `ProtectionEngine` class to replace scattered `subTask*` logic.

### 4.2 Severity & Thresholds (L194F54)

| Protection | Threshold | Severity | logic |
|------------|-----------|----------|-------|
| **Soft UV** | 2.3V | THROTTLE | Limit discharge current |
| **Hard UV** | 2.0V | SOFT_TRIP | Open contactors, auto-retry |
| **Safety UV**| 1.9V | HARD_TRIP | Open contactors, manual reset |
| **Soft OV** | 3.65V | THROTTLE | Limit charge current |
| **Hard OV** | 3.75V | SOFT_TRIP | Open contactors |
| **Safety OV**| 3.8V | HARD_TRIP | Open contactors |
| **Pcb Temp** | >105°C | PAUSE_BAL | Stop balancing only |

---

## PHASE 5: Implementation Order

| # | Task | Effort | Notes |
|---|------|--------|-------|
| 1 | **ISR Heap Fix** | 30m | 🔴 **Prevents random crashes** |
| 2 | **vsprintf Fix** | 30m | 🔴 **Prevents memory corruption** |
| 3 | **UART Security** | 1h | 🟡 **Prevents unauthorized access** |
| 4 | **Contactor Dwell** | 2h | 🟡 **Fixes recovery logic** |
| 5 | Thread Safety | 4h | Fixes race conditions |
| 6 | SOC (54Ah) | 12h | Tuned for L194F54 |
| 7 | Balancing | 10h | Enforce adjacency rules |
| 8 | Protections | 12h | Unified engine |
| 9 | Viewer Refactor | 8h | Module split |

**Total:** ~8-10 Days

---

## Testing Verification

### Critical

- [ ] **ISR:** Run system for 1h under heavy CAN load → no hard faults.
- [ ] **UART:** curl from another IP → Rejected.

### Logic

- [ ] **Contactor:** Trip UV → Wait → Recover voltage → Verify enabled.
- [ ] **Balancing:** Verify Cell 1 and Cell 2 **NEVER** balance simultaneously.
- [ ] **SOC:** Discharge 54Ah → SOC 0%.

### Data

- [ ] **Mutex:** Concurrent logs from 3 threads → Clean output.
- [ ] **CSV:** Open in Excel → rows align correctly.
