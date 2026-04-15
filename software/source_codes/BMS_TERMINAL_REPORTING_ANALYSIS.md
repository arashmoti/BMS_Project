# BMS Terminal Reporting Analysis

## Current Reporting Summary

After scanning your BMS codebase, here's what is **currently being reported** to the terminal:

### Boot Phase (`main.cpp`)

- `[BOOT] RCC_CSR` - Reset cause register value
- `[HARDFAULT]` - Previous HardFault context (PC, LR, xPSR, SP, EXC_RETURN, CONTROL)
- Watchdog status and timeout
- `[CFG] Battery capacity`
- `[COMMISSION] done/active` status (on boot + every 60s)
- `[EEPROM]` reset request and confirmation

### Operation States (`operation.cpp`)

- Every 1s: Operation state + BMS state (e.g., `OP_STATE_LOAD_ENABLED: bmsState: CHARGE`)
- Powerdown request with source (button/allowed)
- Charger disconnect with elapsed time
- Inverter ACK timeout warning

### Measurement & Protections (`measurement.cpp`)

- Protection state transitions with detailed context:
  - Cell UV/OV: threshold values, hysteresis
  - Diff OV: delta voltage info
  - Module OV: pack voltage vs threshold
  - OC Charge/Discharge: current vs limits
  - OT/UT: temperature vs limits
- `[DV-LOCK]` delta-voltage lock events
- Inverter command retries

### Event Logging (`logging.cpp`) - to SD EVENTS_LOG

- `OpState->` transitions
- `ChargeBalanceActive->ON/OFF`
- `ChargerDetected->ON/OFF`
- `BMS->IDLE/DISCHARGE/CHARGE`
- `OV->NORMAL/TRIPPED/RECOVERY`
- `INV->OK/FAULT`
- `SOA_D/SOA_C->IN/OUT`
- `SocSaveReq`
- SD card errors

---

## Suggested Additional Reporting (Low Performance Impact)

### 1. **Periodic Health Summary** (~10s interval)

Print a one-liner with key metrics:

```
[HEALTH] V=848.2V I=-12.3A Vmin=3324mV Vmax=3341mV ΔV=17mV Tmax=28°C SOC=87.5%
```

- **Impact**: ~100 bytes/10s = negligible
- **Location**: `logging.cpp::subTaskInterfaceLogging()` with rate limit

### 2. **Balancing Activity Report**

When balancing starts/stops, log which cells are bleeding:

```
[BAL] Start: cells 3,7,12 bleeding (target=3320mV)
[BAL] Stop: all cells within 5mV
```

- **Impact**: Event-driven, very occasional
- **Location**: `BQ79616_handler.cpp::balanceProcess()`

### 3. **Contactor State Changes**

Log when contactors physically toggle:

```
[CONT] PRE=ON, DSG=OFF, CHG=OFF
[CONT] PRE=OFF, DSG=ON, CHG=ON
```

- **Impact**: Only on actual state change
- **Location**: `power_handler.cpp::udpateSwitches()`

### 4. **CAN TX Summary** (~30s interval)

Log what's being sent to inverter:

```
[CAN-TX] SOC=87% V=848V I=-12A Tlim: chg=100A dchg=120A
```

- **Impact**: Minimal, periodic
- **Location**: `inverter_can_handler.cpp::sendBatteryPileInfoFrame()`

### 5. **SOC Calibration Events**

Log when SOC is recalibrated via OCV or thresholds:

```
[SOC] OCV-calib: Vmin=3340mV->92.3% (conf=0.85)
[SOC] Full-charge clamp: 100%
[SOC] Empty clamp: 0%
```

- **Impact**: Rare events only
- **Location**: `measurement.cpp::subTaskSetInitialSoc()` and `subTaskRecalibrateSoc()`

### 6. **Precharge Metrics**

On precharge completion/failure:

```
[PREC] Success: Vload=845V (99%) in 1.2s
[PREC] Fail: Vload=423V (50%), retry 2/3
```

- **Impact**: Only during state transitions
- **Location**: `operation.cpp::OP_STATE_PRE_CHARGE` case

### 7. **Communication Statistics** (~60s interval)

BQ79616 communication health:

```
[BQ-STAT] TX=1234 RX=1230 CRC-err=2 timeout=1 uptime=3600s
```

- **Impact**: Single line/minute
- **Location**: `BQ79616_handler.cpp` using existing counters

### 8. **Temperature Trend Warning**

When temperature approaches limits (e.g., within 5°C of limit):

```
[TEMP-WARN] Tmax=53°C approaching 55°C charge limit
```

- **Impact**: Only when approaching limits
- **Location**: `measurement.cpp::subTaskAllModulePackTemperatureWatch()`

### 9. **EEPROM/Journal Activity**

Log SOC saves and journal writes:

```
[EEP] SOC saved: 87.5% (remaining=157.5Ah)
[JOURNAL] slot=3 written: 157500mAh, 875mpermil
```

- **Impact**: Very occasional (shutdown, periodic saves)
- **Location**: `measurement.cpp::subTaskHandleSocSaveRequest()`

### 10. **Current Sensor Statistics**

Periodic offset/calibration status:

```
[ISENSE] offset=-23mA, range OK
```

- **Impact**: Single line/minute
- **Location**: Current measurement path

### 11. **All Cell Voltages Report** (~5-10s interval)

Compact comma-separated format for all cells:

```
[CELLS] 3345,3341,3338,3344,3340,3342,3339,3347,3341,3343,3340,3345,3338,3342,3344,3341
```

Or grouped by module for multi-module systems:

```
[CELLS-M1] 3345,3341,3338,3344,3340,3342,3339,3347,3341,3343,3340,3345,3338,3342,3344,3341
[CELLS-M2] 3312,3308,3315,3310,3307,3311,3309,3314,3308,3312,3306,3313,3310,3308,3311,3307
```

**Bandwidth Analysis:**

| Modules | Cells | Formatted Size | TX Time @115200 | Avg Load @5s |
|---------|-------|----------------|-----------------|--------------|
| 1       | 16    | ~100 bytes     | ~9 ms           | 20 B/s       |
| 2       | 32    | ~180 bytes     | ~16 ms          | 36 B/s       |
| 4       | 64    | ~350 bytes     | ~30 ms          | 70 B/s       |

- **Impact**: <1% of UART bandwidth, negligible CPU (one snprintf/interval)
- **Location**: `logging.cpp::subTaskInterfaceLogging()` with rate limit

**Example Implementation:**

```cpp
static uint32_t s_cellLogTick = 0;
if ((LOGGING_GET_TICK(m_loggingManagerTim) - s_cellLogTick) >= 5000) {
    s_cellLogTick = LOGGING_GET_TICK(m_loggingManagerTim);
    char buf[200];
    int pos = snprintf(buf, sizeof(buf), "[CELLS] ");
    for (uint8_t c = 0; c < nCells && pos < 180; ++c) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%u%s",
                        cellVoltages[c], (c < nCells-1) ? "," : "");
    }
    m_hwLogging.interfaceComm.printToInterface("%s\r\n", buf);
}
```

---

## Implementation Notes - NO CHANGES PER REQUEST

The suggestions above are designed to be:

- **Non-blocking**: Use existing rate-limited patterns (`timerDelay1ms`)
- **Event-driven where possible**: Only log on state changes
- **Compact**: Single-line formats to minimize UART bandwidth
- **Parseable**: Prefixed tags for easy filtering in viewer

All suggestions use the existing `InterfaceCommHandler::getInstance()->printToInterface()` pattern which is already buffered and thread-safe.

**Estimated overhead**: <500 bytes/second additional UART traffic at maximum, typically much less during stable operation.
