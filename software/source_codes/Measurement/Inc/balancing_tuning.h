#ifndef __BALANCING_TUNING__H_
#define __BALANCING_TUNING__H_

#include <cstdint>

namespace BalancingArgs
{
constexpr int32_t MAX_CHARGE_CURRENT_MA = 1000; // do not balance if |charge current| > 1A (reduced from 2A)
constexpr int32_t MAX_DISCHARGE_CURRENT_MA = 1000; // stop balancing if load/discharge current exceeds 1A
constexpr int32_t THROTTLE_CURRENT_MA = 500;    // throttle charge to 0.5A during balancing
constexpr int32_t IDLE_ABS_CURRENT_MA = 100;    // max |current| to consider pack truly idle
constexpr uint32_t IDLE_HOLD_MS = 30u * 1000u;  // minimum idle dwell before allowing balance //5u * 60u * 1000u;
constexpr int32_t CHARGE_THRESH_MA = -50;       // current threshold (neg) to treat as charging
constexpr int16_t THROTTLE_X10MA = 0; // 0 -> stop charging
constexpr uint16_t DV_START_MV = 45;            // enter balancing when dv exceeds this
constexpr uint16_t DV_STOP_MV = 5;             // exit balancing when dv falls below this
constexpr uint16_t PACK_MISMATCH_ENTRY_MV = 200; // dv to trigger charge clamp/lock-in  //320
constexpr uint16_t PACK_MISMATCH_EXIT_MV = 50;   // was 180 - keep throttle until dV is low

constexpr uint16_t WINDOW_MIN_MV = 10;          // lowest window to qualify top cells
constexpr uint16_t WINDOW_MAX_MV = 50;          // widest window when dv is high
constexpr uint16_t WINDOW_HIGH_DV_MV = 200;     // dv that drives window to max

constexpr uint16_t MAX_ACTIVE_CELLS_PER_DEV = 8; // per-device simultaneous bleeders (reduced from 8 for noise)
constexpr uint16_t MAX_ACTIVE_CELLS_TOTAL = 24;  // global limit across all modules (comms budget)
constexpr bool NO_ADJACENT_CELLS = true; // align with DEV_CONF[NO_ADJ_CB]

constexpr uint32_t DV_TREND_OBS_MS = 3u * 60u * 1000u; // window to watch dv growth while charging
constexpr uint32_t DV_RECOVERY_HOLD_MS = 60u * 1000u; // hold time before unlocking dv clamp
constexpr uint32_t DV_STOP_HOLD_MS = 30u * 1000u; // time dv must stay below stop threshold
constexpr uint16_t DV_UNLOCK_MV = 100;           // dv below which discharge can clear clamp
constexpr uint16_t DV_UNLOCK_RAMP_START_CA = 100; // start charge limit at 1.00 A after dv lock clears
constexpr uint16_t DV_UNLOCK_RAMP_STEP_CA = 100;  // ramp step (1.00 A)
constexpr uint32_t DV_UNLOCK_RAMP_PERIOD_MS = 1000u; // ramp step interval
constexpr uint32_t DV_UNLOCK_RAMP_HOLD_MS = 60u * 1000u; // dv must stay below unlock for this long before ramp
constexpr uint16_t DV_TREND_INC_MIN_MV = 25;     // dv increase over obs window to trigger lock
constexpr uint16_t DV_TREND_MIN_BASE_MV = 60;    // minimum dv to start trend watching
constexpr int32_t CHG_ACTIVE_MA = -200;          // current (neg) considered "active charge" for dv lock
constexpr uint32_t REENFORCE_MS = 2000u;         // how often to re-send clamp while locked
constexpr uint16_t MODULE_PACK_IMBALANCE_MV = 1000; // module pack voltage delta vs avg others to clamp charge
constexpr uint16_t MODULE_PACK_IMBALANCE_CLEAR_MV = 300; // clear clamp when delta falls below this
constexpr uint32_t MODULE_PACK_IMBALANCE_CLEAR_HOLD_MS = 30u * 1000u; // hold time below clear threshold

// Periodic balance pause for clean measurement windows
constexpr uint32_t PERIODIC_BALANCE_PAUSE_MS = 5u * 60u * 1000u; // run time before pausing
constexpr uint32_t PERIODIC_BALANCE_HOLD_MS = 30u * 1000u;       // pause duration for clean reads
constexpr uint32_t BALANCE_MEASURE_SETTLE_MS = 1000u;            // settle time before accepting clean samples
constexpr uint32_t NO_BLEED_EXIT_MS = 10u * 60u * 1000u;         // exit balancing after no eligible cells for this long
constexpr bool FILTER_MIN_ADJACENT_TO_BLEED = true;              // ignore neighbors of active bleeders for Vmin
constexpr uint8_t FILTER_MAX_MODE = 2;                           // 0=off, 1=mask bleed cells only, 2=mask bleed+neighbors

constexpr int32_t THERMAL_PAUSE_MC = 105000;  // 105C
constexpr int32_t THERMAL_RESUME_MC = 95000;  // 95C

constexpr uint16_t BLEED_MA_75C = 240;          // target bleed current per cell at 75C (mA)
constexpr int32_t BLEED_DERATE_MA_PER_C = 2;    // mA reduction per °C above 75C

// Commissioning mode constants (moved from measurement.cpp for centralization)
constexpr uint16_t COMM_DV_FULL_MV = 40;        // dV below which commissioning uses full current
constexpr uint16_t COMM_DV_LIMIT_MV = 80;       // dV above which current limiting kicks in
constexpr uint16_t COMM_STOP_ENTER_MV = 90;     // dV to enter stop-charge state
constexpr uint16_t COMM_STOP_EXIT_MV = 70;      // dV to exit stop-charge state
constexpr uint16_t COMM_DONE_DV_MV = 30;        // dV target for commissioning completion
constexpr uint32_t COMM_DONE_HOLD_MS = 30u * 60u * 1000u;   // hold time at target dV
constexpr uint32_t COMM_TREND_MS = 10u * 60u * 1000u;       // observation window for dV trend
constexpr uint16_t COMM_TREND_DROP_MV = 5;      // minimum dV drop to consider low-current mode
constexpr uint16_t COMM_CURRENT_1A = 100;       // 1.00 A in centi-amps (commissioning current)
constexpr uint16_t COMM_CURRENT_0_5A = 50;      // 0.50 A in centi-amps (reduced current)

// Testing/debug flags
constexpr bool DISABLE_CHARGE_DURING_BALANCE = true; // Set to true to disable charging while balancing

// Pre-balance communication health check
constexpr uint8_t MAX_COMM_FAULT_RETRIES = 3;        // Max retries before proceeding despite faults

// Bleeder soft-start (staggered activation to reduce inductive noise)
constexpr uint32_t BLEEDER_DEVICE_STAGGER_MS = 50;   // Delay between activating each device's bleeders
constexpr uint32_t BLEEDER_INTRA_DEVICE_MS = 30;     // Delay between high-bank and low-bank within device
} // namespace BalancingArgs

#endif // __BALANCING_TUNING__H_
