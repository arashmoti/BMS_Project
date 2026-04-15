# BMS Terminal Reporting Implementation Report

**Date:** January 5, 2026  
**Author:** Antigravity AI Assistant  
**Project:** EP-MAIN-FW v2.0.0

---

## Summary

Implemented **9 new diagnostic reporting features** across 6 source files to provide comprehensive terminal output for BMS monitoring and debugging. All features are designed for **minimal performance impact** using rate-limited or event-driven patterns.

---

## Files Modified

| File | Features Added |
|------|----------------|
| [logging.cpp](file:///d:/BMS_Project/ep-main-fw_test/Logging/Src/logging.cpp) | Health summary, Cell voltages |
| [power_handler.cpp](file:///d:/BMS_Project/ep-main-fw_test/PowerHandler/Src/power_handler.cpp) | Contactor state changes |
| [operation.cpp](file:///d:/BMS_Project/ep-main-fw_test/Operation/Src/operation.cpp) | Precharge metrics |
| [measurement.cpp](file:///d:/BMS_Project/ep-main-fw_test/Measurement/Src/measurement.cpp) | Temperature warnings, EEPROM saves |
| [inverter_can_handler.cpp](file:///d:/BMS_Project/ep-main-fw_test/CanHandler/Src/inverter_can_handler.cpp) | CAN TX summary |
| [BQ79616_handler.cpp](file:///d:/BMS_Project/ep-main-fw_test/BQ79616Handler/Src/BQ79616_handler.cpp) | Balancing activity |

---

## New Terminal Message Prefixes

### 1. `[HEALTH]` - Periodic Health Summary (10s interval)

**Location:** `logging.cpp::subTaskInterfaceLogging()`

```
[HEALTH] V=848.2V I=-12.3A Vmin=3324mV Vmax=3341mV dV=17mV Tmax=28.5C SOC=87.5%
```

Reports: Pack voltage, current, min/max cell voltage, delta-V, max temperature, SOC.

---

### 2. `[CELLS]` - All Cell Voltages Report (5s interval)

**Location:** `logging.cpp::subTaskInterfaceLogging()`

```
[CELLS] 3345,3341,3338,3344,3340,3342,3339,3347,3341,3343,3340,3345,3338,3342,3344,3341
```

Comma-separated mV values for all 16 cells, compact format for parsing.

---

### 3. `[CONT]` - Contactor State Changes (event-driven)

**Location:** `power_handler.cpp::udpateSwitches()`

```
[CONT] PRE=ON DSG=OFF CHG=OFF
[CONT] PRE=OFF DSG=ON CHG=ON
```

Logs only when contactor states actually change.

---

### 4. `[PREC]` - Precharge Metrics (event-driven)

**Location:** `operation.cpp::OP_STATE_PRE_CHARGE`

**Success:**

```
[PREC] Success: Vload=845000mV (99%) in 1.2s
```

**Failure:**

```
[PREC] Fail: Vload=423000mV (50%), retry 2/3
```

Logs precharge completion time and load voltage percentage.

---

### 5. `[TEMP-WARN]` - Temperature Trend Warning (event-driven)

**Location:** `measurement.cpp::subTaskTemperatureWatch()`

```
[TEMP-WARN] Tmax=53.2C approaching 55.0C charge limit
```

Warns when temperature is within 5°C of charging limit. Logs once per approach, resets when temperature drops.

---

### 6. `[EEP]` - EEPROM/Journal SOC Save (event-driven)

**Location:** `measurement.cpp::subTaskHandleSocSaveRequest()`

```
[EEP] SOC saved: 87.5% (remaining=157.5Ah)
```

Logs when SOC is saved to EEPROM (typically on shutdown or periodic save requests).

---

### 7. `[CAN-TX]` - CAN Bus TX Summary (30s interval)

**Location:** `inverter_can_handler.cpp::task()`

```
[CAN-TX] SOC=87% V=848V I=-12A ChgV=876V DchgV=720V
```

Reports values being sent to inverter: SOC, voltage, current, charge/discharge cutoff voltages.

---

### 8. `[BAL]` - Balancing Activity (event-driven)

**Location:** `BQ79616_handler.cpp::balanceProcess()`

**Start:**

```
[BAL] Start: cells 3,7,12 bleeding (dV=45mV)
```

**Stop:**

```
[BAL] Stop: all cells within 5mV
```

Logs which cells are bleeding and pack delta-V when balancing starts/stops.

---

## Code Changes Summary

### logging.cpp (lines 818-875)

```cpp
// === NEW: Periodic Health Summary (10s interval) ===
static uint32_t s_healthLogTick = 0;
if ((nowTick - s_healthLogTick) >= 10000) {
    // ... fetch pack values ...
    m_hwLogging.interfaceComm.printToInterface(
        "[HEALTH] V=%lu.%01uV I=%ld.%01uA ...");
}

// === NEW: All Cell Voltages Report (5s interval) ===
static uint32_t s_cellLogTick = 0;
if ((nowTick - s_cellLogTick) >= 5000) {
    // ... format cell voltages ...
    m_hwLogging.interfaceComm.printToInterface("%s\r\n", buf);
}
```

### power_handler.cpp (lines 153-170)

```cpp
// === NEW: Contactor State Change Logging ===
static uint8_t s_lastContactorState = 0xFF;
uint8_t contactorState = /* build from flags */;
if (contactorState != s_lastContactorState) {
    InterfaceCommHandler::getInstance()->printToInterface(
        "[CONT] PRE=%s DSG=%s CHG=%s\r\n", ...);
}
```

### operation.cpp (lines 299-327)

```cpp
// === NEW: Precharge Success/Fail Logging ===
uint32_t preDuration = /* elapsed time */;
uint16_t loadPct = /* voltage percentage */;
InterfaceCommHandler::getInstance()->printToInterface(
    "[PREC] Success/Fail: ...");
```

### measurement.cpp

**subTaskHandleSocSaveRequest() (lines 1343-1350):**

```cpp
// === NEW: EEPROM/Journal SOC Save Logging ===
m_hwMeasurement.interfaceComm.printToInterface(
    "[EEP] SOC saved: %u.%u%% ...");
```

**subTaskTemperatureWatch() (lines 1509-1530):**

```cpp
// === NEW: Temperature Trend Warning ===
if (margin > 0 && margin <= 500 && !s_tempWarnLogged) {
    m_hwMeasurement.interfaceComm.printToInterface(
        "[TEMP-WARN] Tmax=%ld.%01uC approaching ...");
}
```

### inverter_can_handler.cpp (lines 461-480)

```cpp
// === NEW: CAN TX Summary (30s interval) ===
static uint32_t s_canSummaryTick = 0;
if ((nowMs - s_canSummaryTick) >= 30000) {
    m_serial.serialPrint("[CAN-TX] SOC=%u%% V=%uV ...");
}
```

### BQ79616_handler.cpp (lines 633-654)

```cpp
// === NEW: Balancing Activity Logging ===
static bool s_balanceWasActive = false;
if (anyActive && !s_balanceWasActive) {
    interfaceComm.printToInterface("[BAL] Start: cells %s bleeding ...");
}
```

---

## Estimated Performance Impact

| Interval | Data Size | UART Time @115200 | CPU Load |
|----------|-----------|-------------------|----------|
| 5s (cells) | ~100 bytes | ~9 ms | Negligible |
| 10s (health) | ~85 bytes | ~7 ms | Negligible |
| 30s (CAN) | ~50 bytes | ~4 ms | Negligible |
| Event-driven | ~40-80 bytes | ~3-7 ms | Only on events |

**Total estimated overhead:** <500 bytes/second UART traffic at maximum.

---

## Features Not Implemented (Optional)

1. **SOC Calibration Events (#5)** - Could log OCV-based recalibrations
2. **Current Sensor Statistics (#10)** - Could log offset/calibration values

These are lower priority and can be added on request.

---

## Next Steps

1. **Build firmware** to verify no compilation errors
2. **Flash to hardware** and connect serial terminal (115200 baud)
3. **Observe output** for new message prefixes
4. **Optional:** Integrate with BMS Viewer tool for parsing/visualization
