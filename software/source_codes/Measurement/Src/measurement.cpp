#include <cstdint>
#include <stdint.h>
#include <cstring>
#include <memory>
#include "measurement.h"
#include "inverter_can_handler_params.h"
#include "eeprom_handler_params.h"
#include "balancing_tuning.h"
#include <cmath>
#include "Callback.h"
#include "Timer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdlib>
#include "interface_comm_handler.h" // <-- Add this include for printing
#include "sdcard_handler.h"
#include <algorithm>
#include "operation.h" // for hb_set / HB_MEAS
#include "inverter_error_simulation.h"
#include <chrono>
using namespace std::chrono;
rtos::Mutex g_packInfoMutex;
extern SdCardHandler sdCard;

// FORCE_COMMISSIONING_DONE is defined in Settings.h
// #define SUPPRESS_COMM_FAULT 1 // [DEBUG] Set to 1 to downgrade COMM faults to warnings (no system reset)

// Forward declaration for clamp debug logger
static void logChargeClampStatic(const char *reason);

namespace {
struct PackInfoGuard
{
  PackInfoGuard() { g_packInfoMutex.lock(); }
  ~PackInfoGuard() { g_packInfoMutex.unlock(); }
};

static InverterCanArgs::st_INVERTER_PROTECT_WARNING inverterProtectWarning{};
} // namespace


// Human-readable mapping for protections reporting
static const char *prot_name_from_index(uint8_t idx)
{
  switch (idx)
  {
  case 0:
    return "Cell UnderVoltage";
  case 1:
    return "Cell OverVoltage";
  case 2:
    return "Cell Diff OverVoltage";
  case 3:
    return "Module OverVoltage";
  case 4:
    return "Discharge OverCurrent";
  case 5:
    return "Charge OverCurrent";
  case 6:
    return "OverTemp Charge";
  case 7:
    return "OverTemp Discharge";
  case 8:
    return "UnderTemp Charge";
  case 9:
    return "UnderTemp Discharge";
  default:
    return "Unknown";
  }
}

static const char *prot_state_to_str(enProtectionState s)
{
  switch (s)
  {
  case PROT_STATE_OK:
    return "OK";
  case PROT_STATE_DETECT_TIMEOUT_START:
    return "Detect-Delay-Start";
  case PROT_STATE_COUNT_TIMEOUT:
    return "Detect-Counting";
  case PROT_STATE_DETECT:
    return "Detected";
  case PROT_STATE_RELEASE_TIMEOUT_START:
    return "Release-Delay-Start";
  case PROT_STATE_RELEASE:
    return "Released";
  default:
    return "Unknown";
  }
}

// Inverter stop command retry policy while waiting for response
#ifndef INVERTER_STOP_CMD_RESEND_MS
#define INVERTER_STOP_CMD_RESEND_MS 200 // Re-send stop/limit command every 200ms while waiting
#endif
#ifndef INVERTER_STOP_CMD_MAX_RESENDS
#define INVERTER_STOP_CMD_MAX_RESENDS 8 // 5 // Re-send up to 8 times (1.6s) before giving up
#endif

// #############################################################################################//

// Monotonic ms since boot (doesn’t depend on your member Timer)
static uint32_t uptime_ms()
{
  static Timer boot_timer;
  static bool started = false;
  if (!started)
  {
    boot_timer.start();
    started = true;
  }
  return duration_cast<milliseconds>(boot_timer.elapsed_time()).count();
}
// #############################################################################################//

static float ocv_confidence_from_voltage(uint16_t vmin_mV, uint16_t vmax_mV)
{
  const uint16_t V_LOW_KNEE_MV = MeasurementArgs::SOC_OCV_V_LOW_KNEE_MV;
  const uint16_t V_PLATEAU_LOW_MV = MeasurementArgs::SOC_OCV_V_PLATEAU_LOW_MV;
  const uint16_t V_PLATEAU_HIGH_MV = MeasurementArgs::SOC_OCV_V_PLATEAU_HIGH_MV;
  const uint16_t V_HIGH_KNEE_MV = MeasurementArgs::SOC_OCV_V_HIGH_KNEE_MV;

  float low_conf = 0.0f;
  if (vmin_mV <= V_LOW_KNEE_MV)
  {
    low_conf = 1.0f;
  }
  else if (vmin_mV < V_PLATEAU_LOW_MV)
  {
    low_conf = static_cast<float>(V_PLATEAU_LOW_MV - vmin_mV) /
               static_cast<float>(V_PLATEAU_LOW_MV - V_LOW_KNEE_MV);
  }

  float high_conf = 0.0f;
  if (vmax_mV >= V_HIGH_KNEE_MV)
  {
    high_conf = 1.0f;
  }
  else if (vmax_mV > V_PLATEAU_HIGH_MV)
  {
    high_conf = static_cast<float>(vmax_mV - V_PLATEAU_HIGH_MV) /
                static_cast<float>(V_HIGH_KNEE_MV - V_PLATEAU_HIGH_MV);
  }

  return std::max(low_conf, high_conf);
}

static void apply_soft_uv_throttle(void);

namespace {
// Plausibility check for voltage readings before caching
// Rejects readings that are out of physical range or show sudden large jumps
bool isSamplePlausible(uint16_t vmin, uint16_t vmax, uint16_t cachedVmin, uint16_t cachedVmax)
{
  // Reject if out of physical range
  if (vmin < 2500 || vmax > 4300 || vmax < vmin)
    return false;
  const uint16_t dv = (uint16_t)(vmax - vmin);

  // Reject if jump versus last clean snapshot is extreme (>150 mV per edge), when available
  if (cachedVmax != 0)
  {
    const uint16_t lastMin = cachedVmin;
    const uint16_t lastMax = cachedVmax;
    const uint16_t lastDv = (uint16_t)(lastMax - lastMin);
    const uint16_t jumpMin = (vmin > lastMin) ? (uint16_t)(vmin - lastMin) : (uint16_t)(lastMin - vmin);
    const uint16_t jumpMax = (vmax > lastMax) ? (uint16_t)(vmax - lastMax) : (uint16_t)(lastMax - vmax);
    const bool dvDown = (dv < lastDv);
    if (!dvDown)
    {
      if (jumpMin > 150 || jumpMax > 150)
        return false;
      const uint16_t dvJump = (dv > lastDv) ? (uint16_t)(dv - lastDv) : (uint16_t)(lastDv - dv);
      if (dvJump > 120) // reject large dv swings in one sample
        return false;
    }
    else
    {
      // Allow larger downward dV jumps so cache can recover after balancing,
      // but still reject extreme edge jumps.
      if (jumpMin > 300 || jumpMax > 300)
        return false;
    }
  }
  return true;
}

// Tracks consecutive passes/fails for the filter
struct PlausibilityTracker
{
  uint8_t consecutivePasses = 0;
  uint8_t consecutiveFails = 0;
  uint16_t lastAcceptedMin = 0;
  uint16_t lastAcceptedMax = 0;
};

static float cRateFromTemp(float tC)
{
  if (tC >= 0.0f && tC < 5.0f)
    return 0.1f;
  if (tC >= 5.0f && tC < 15.0f)
    return 0.3f;
  if (tC >= 15.0f && tC < 35.0f)
  {
    const float frac = (tC - 15.0f) / 20.0f; // 15C->35C maps 0->1
    return 0.5f + (0.5f * frac);            // 0.5C -> 1.0C
  }
  if (tC >= 35.0f && tC < 45.0f)
    return 0.3f;
  return 0.05f; // outside 0-45C: 0.05C trickle
}

static float cRateFromTempDischarge(float tC)
{
  if (tC < -20.0f || tC >= 55.0f)
    return 0.0f;
  if (tC < 0.0f)
    return 0.5f;
  if (tC < 10.0f)
    return 0.8f;
  if (tC < 25.0f)
    return 0.9f;
  if (tC < 45.0f)
    return 1.0f;
  return 0.5f; // 45C to 55C
}

// Charge derating based on cell temperature (LFP table).
// Returns a limit in centi-amps (0.01A), clamped to the provided base limit.
uint16_t derateChargeCurrent_cA(uint16_t base_cA, const st_generalConfig *gen, const st_powerElecPackInfoConfig *pack)
{
  if (base_cA == 0 || !gen || !pack)
    return base_cA;
  if (pack->u32LastBqDataTimestamp == 0)
    return base_cA;

  // Use hottest and coldest individual sensor values (not average)
  int32_t tMax_mC = pack->i32ModuleTemperatureMax;
  int32_t tMin_mC = pack->i32ModuleTemperatureMin;
  {
    uint8_t nModules = gen->u8NumberOfModules;
    if (nModules > NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
      nModules = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
    uint8_t nTemps = gen->u8NumberOfTemperature;
    if (nTemps > NoOfTEMP_POSSIBLE_ON_CHIP)
      nTemps = NoOfTEMP_POSSIBLE_ON_CHIP;

    int32_t maxSeen_mC = -1000000;
    int32_t minSeen_mC = 1000000;
    bool any = false;
    for (uint8_t m = 0; m < nModules; ++m)
    {
      for (uint8_t t = 0; t < nTemps; ++t)
      {
        const int32_t temp_mC = pack->i32arrCellModuleBMSEXTTemperature[m][t];
        // Reject obviously invalid readings
        if (temp_mC < -50000 || temp_mC > 150000)
          continue;
        if (!any || temp_mC > maxSeen_mC)
        {
          maxSeen_mC = temp_mC;
        }
        if (!any || temp_mC < minSeen_mC)
          minSeen_mC = temp_mC;
        any = true;
      }
    }
    if (any)
    {
      tMax_mC = maxSeen_mC;
      tMin_mC = minSeen_mC;
    }
  }

  float tMaxC = static_cast<float>(tMax_mC) / 1000.0f;
  float tMinC = static_cast<float>(tMin_mC) / 1000.0f;

  // Simulation profile start: start at 30s, step every 10s (absolute from boot).
 /* {
    const uint32_t nowMs = uptime_ms();
    if (nowMs >= 30000U && nowMs < 40000U)
    {
      tMaxC = 4.0f;
      tMinC = 4.0f;
    }
    else if (nowMs >= 40000U && nowMs < 50000U)
    {
      tMaxC = 8.0f;
      tMinC = 8.0f;
    }
    else if (nowMs >= 50000U && nowMs < 60000U)
    {
      tMaxC = 17.0f;
      tMinC = 17.0f;
    }
    else if (nowMs >= 60000U && nowMs < 70000U)
    {
      tMaxC = 37.4f;
      tMinC = 37.4f;
    }
    else if (nowMs >= 70000U)
    {
      tMaxC = 37.4f;
      tMinC = 37.4f;
    }
  }
      */
  // Simulation profile end: start at 30s, step every 10s (absolute from boot).
  const float cRateMax = cRateFromTemp(tMaxC);
  const float cRateMin = cRateFromTemp(tMinC);
  float cRate = std::min(cRateMax, cRateMin);
  static uint8_t cRateDerateFlags = 0;
  if (pack->u16ModuleCellVoltageAverage > 3380u && cRateDerateFlags == 0)
  {
    cRateDerateFlags = 1;
  }
  if ((pack->u16ModuleCellVoltageAverage < 3350u))
  {
    cRateDerateFlags = 0;
  }


  const float cap_Ah = static_cast<float>(gen->u16BatteryCapacity);
  uint32_t allowed_cA = 0;
  if (cap_Ah <= 0.0f)
    return base_cA;

  if (cRateDerateFlags == 1)
  {
    allowed_cA = static_cast<uint32_t>(std::lround(cap_Ah * 0.25 * 100.0f));
  }
  else
  {
    allowed_cA = static_cast<uint32_t>(std::lround(cap_Ah * cRate * 100.0f));
    if (allowed_cA > 0xFFFFu)
    allowed_cA = 0xFFFFu;
  }
  const uint16_t derated_cA = static_cast<uint16_t>(allowed_cA);
  return (derated_cA < base_cA) ? derated_cA : base_cA;
}

// Discharge derating based on cell temperature (LFP table).
// Returns a limit in centi-amps (0.01A), clamped to the provided base limit.
uint16_t derateDischargeCurrent_cA(uint16_t base_cA, const st_generalConfig *gen, const st_powerElecPackInfoConfig *pack)
{
  if (base_cA == 0 || !gen || !pack)
    return base_cA;
  if (pack->u32LastBqDataTimestamp == 0)
    return base_cA;

  // Use hottest and coldest individual sensor values (not average)
  int32_t tMax_mC = pack->i32ModuleTemperatureMax;
  int32_t tMin_mC = pack->i32ModuleTemperatureMin;
  {
    uint8_t nModules = gen->u8NumberOfModules;
    if (nModules > NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
      nModules = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
    uint8_t nTemps = gen->u8NumberOfTemperature;
    if (nTemps > NoOfTEMP_POSSIBLE_ON_CHIP)
      nTemps = NoOfTEMP_POSSIBLE_ON_CHIP;

    int32_t maxSeen_mC = -1000000;
    int32_t minSeen_mC = 1000000;
    bool any = false;
    for (uint8_t m = 0; m < nModules; ++m)
    {
      for (uint8_t t = 0; t < nTemps; ++t)
      {
        const int32_t temp_mC = pack->i32arrCellModuleBMSEXTTemperature[m][t];
        // Reject obviously invalid readings
        if (temp_mC < -50000 || temp_mC > 150000)
          continue;
        if (!any || temp_mC > maxSeen_mC)
        {
          maxSeen_mC = temp_mC;
        }
        if (!any || temp_mC < minSeen_mC)
        {
          minSeen_mC = temp_mC;
        }
        any = true;
      }
    }
    if (any)
    {
      tMax_mC = maxSeen_mC;
      tMin_mC = minSeen_mC;
    }
  }

  const float tMaxC = static_cast<float>(tMax_mC) / 1000.0f;
  const float tMinC = static_cast<float>(tMin_mC) / 1000.0f;

  const float cRateMax = cRateFromTempDischarge(tMaxC);
  const float cRateMin = cRateFromTempDischarge(tMinC);
  const float cRate = std::min(cRateMax, cRateMin);

  const float cap_Ah = static_cast<float>(gen->u16BatteryCapacity);
  if (cap_Ah <= 0.0f)
    return base_cA;

  uint32_t allowed_cA = static_cast<uint32_t>(std::lround(cap_Ah * cRate * 100.0f));
  if (allowed_cA > 0xFFFFu)
    allowed_cA = 0xFFFFu;

  const uint16_t derated_cA = static_cast<uint16_t>(allowed_cA);
  return (derated_cA < base_cA) ? derated_cA : base_cA;
}
} // namespace

class DvLockController
{
public:
  void update(uint32_t nowMs,
              uint16_t dv_mV,
              int32_t current_mA,
              enBmsState bmsState,
              uint16_t packMismatchEntry_mV,
              uint16_t dvStop_mV,
              HwMeasurementBase &hw,
              st_generalConfig *gen,
              st_powerElecPackInfoConfig *pack)
  {
    static constexpr uint32_t kDV_TREND_OBS_MS = BalancingArgs::DV_TREND_OBS_MS;
    static constexpr uint32_t kDV_RECOVERY_HOLD_MS = BalancingArgs::DV_RECOVERY_HOLD_MS;
    static constexpr uint32_t kDV_STOP_HOLD_MS = BalancingArgs::DV_STOP_HOLD_MS;
    static constexpr uint16_t kDV_UNLOCK_mV = BalancingArgs::DV_UNLOCK_MV;
    static constexpr uint16_t kDV_TREND_INC_MIN_mV = BalancingArgs::DV_TREND_INC_MIN_MV;
    static constexpr uint16_t kDV_TREND_MIN_BASE_mV = BalancingArgs::DV_TREND_MIN_BASE_MV;
    static constexpr int32_t kCHG_ACTIVE_mA = BalancingArgs::CHG_ACTIVE_MA;
    static constexpr uint32_t kREENFORCE_MS = BalancingArgs::REENFORCE_MS;
    static constexpr uint16_t kDV_UNLOCK_RAMP_START_cA = BalancingArgs::DV_UNLOCK_RAMP_START_CA;
    static constexpr uint16_t kDV_UNLOCK_RAMP_STEP_cA = BalancingArgs::DV_UNLOCK_RAMP_STEP_CA;
    static constexpr uint32_t kDV_UNLOCK_RAMP_PERIOD_MS = BalancingArgs::DV_UNLOCK_RAMP_PERIOD_MS;
    static constexpr uint32_t kDV_UNLOCK_RAMP_HOLD_MS = BalancingArgs::DV_UNLOCK_RAMP_HOLD_MS;
    static constexpr uint32_t kDV_ENFORCE_LOG_MS = 10000U;
    static constexpr uint32_t kDV_STATUS_LOG_MS = 30000U;
    if (!m_active)
    {
      if (dv_mV >= packMismatchEntry_mV)
      {
        activate(nowMs, hw, gen, pack);
        hw.interfaceComm.printToInterface(
            "[DV-LOCK] dv=%u mV >= %u -> clamp 0A\r\n",
            (unsigned)dv_mV, (unsigned)packMismatchEntry_mV);
      }
      else
      {
        const bool chgState = (bmsState == BMS_CHARGE);
        const bool chgCurr = (current_mA <= kCHG_ACTIVE_mA);
        if (chgState && chgCurr)
        {
          if (dv_mV < kDV_TREND_MIN_BASE_mV)
          {
            // dv too small to consider; reset trend accumulator
            m_trendStartMs = nowMs;
            m_dvAtTrendStart_mV = dv_mV;
          }
          else if (m_trendStartMs == 0)
          {
            m_trendStartMs = nowMs;
            m_dvAtTrendStart_mV = dv_mV;
          }
          else if ((uint32_t)(nowMs - m_trendStartMs) >= kDV_TREND_OBS_MS)
          {
            const uint16_t netInc = (dv_mV >= m_dvAtTrendStart_mV)
                                        ? (uint16_t)(dv_mV - m_dvAtTrendStart_mV)
                                        : 0u;
            if (netInc >= kDV_TREND_INC_MIN_mV)
            {
              activate(nowMs, hw, gen, pack);
              hw.interfaceComm.printToInterface(
                  "[DV-LOCK] dv +%u mV over 3m -> clamp 0A\r\n",
                  (unsigned)netInc);
            }
            m_trendStartMs = nowMs;
            m_dvAtTrendStart_mV = dv_mV;
          }
        }
        else
        {
          m_trendStartMs = 0;
        }
      }

      if (m_rampPending || m_rampActive)
      {
        if (!m_rampActive)
        {
          if (dv_mV <= kDV_UNLOCK_mV)
          {
            if (m_rampHoldStartMs == 0)
              m_rampHoldStartMs = nowMs;
          }
          else
          {
            m_rampHoldStartMs = 0;
            return;
          }
          if (m_rampHoldStartMs != 0 &&
              (uint32_t)(nowMs - m_rampHoldStartMs) < kDV_UNLOCK_RAMP_HOLD_MS)
          {
            return;
          }
        }

        const bool chargingState = (bmsState == BMS_CHARGE);
        const bool allowStart = !pack->bBalancingActive;
        if (allowStart)
        {
          const uint16_t target_cA = derateChargeCurrent_cA(gen->u16MaxSoftChgAllowedCurrent, gen, pack);
          if (!m_rampActive)
          {
            m_rampCurrent_cA = std::min<uint16_t>(kDV_UNLOCK_RAMP_START_cA, target_cA);
            m_rampLastMs = nowMs;
            m_rampActive = (m_rampCurrent_cA < target_cA);
            m_rampPending = false;
            hw.inverterCan.updateMaxChargeLimit(gen->u16MaxChargePackVoltage, m_rampCurrent_cA);
            hw.interfaceComm.printToInterface(
                "[DV-LOCK] ramp start -> %.2fA\r\n",
                (double)m_rampCurrent_cA / 100.0);
            logChargeClampStatic("DV_LOCK_RAMP_START");
            if (!m_rampActive)
              logChargeClampStatic("DV_LOCK_RAMP_DONE");
          }
          else if (chargingState &&
                   (uint32_t)(nowMs - m_rampLastMs) >= kDV_UNLOCK_RAMP_PERIOD_MS)
          {
            const uint32_t next_cA = (uint32_t)m_rampCurrent_cA + (uint32_t)kDV_UNLOCK_RAMP_STEP_cA;
            m_rampCurrent_cA = (next_cA >= target_cA) ? target_cA : (uint16_t)next_cA;
            hw.inverterCan.updateMaxChargeLimit(gen->u16MaxChargePackVoltage, m_rampCurrent_cA);
            m_rampLastMs = nowMs;
            if (m_rampCurrent_cA >= target_cA)
            {
              m_rampActive = false;
              logChargeClampStatic("DV_LOCK_RAMP_DONE");
            }
          }
        }
      }
    }
    else
    {
      // Early clear if discharging significantly and imbalance is already below unlock threshold.
      static constexpr int32_t kDISCH_CLEAR_mA = 1000; // ~1A discharge
      if (current_mA >= kDISCH_CLEAR_mA && dv_mV <= kDV_UNLOCK_mV)
      {
        deactivate(hw, gen, pack);
        hw.interfaceComm.printToInterface(
            "[DV-LOCK] cleared by discharge dv=%u mV I=%ld mA\r\n",
            (unsigned)dv_mV, (long)current_mA);
        return;
      }

      if (dv_mV <= dvStop_mV)
      {
        if (m_unlockStartMs == 0)
          m_unlockStartMs = nowMs;
        if ((uint32_t)(nowMs - m_unlockStartMs) >= kDV_STOP_HOLD_MS)
        {
          deactivate(hw, gen, pack);
          hw.interfaceComm.printToInterface(
              "[DV-LOCK] cleared after 60s <= stop %u mV -> restore\r\n",
              (unsigned)dvStop_mV);
        }
      }
      else if (dv_mV <= kDV_UNLOCK_mV)
      {
        if (m_unlockStartMs == 0)
          m_unlockStartMs = nowMs;
        if ((uint32_t)(nowMs - m_unlockStartMs) >= kDV_RECOVERY_HOLD_MS)
        {
          deactivate(hw, gen, pack);
          hw.interfaceComm.printToInterface(
              "[DV-LOCK] cleared after hold <= %u mV -> restore\r\n",
              (unsigned)kDV_UNLOCK_mV);
        }
      }
      else
      {
        m_unlockStartMs = 0;
      }

      if ((uint32_t)(nowMs - m_lastEnforceMs) >= kREENFORCE_MS)
      {
        const bool uvpActive = (pack && gen && (pack->u16ModuleCellVoltageMin <= gen->u16SoftLowCurrentUnderVoltage));
        if (!uvpActive)
        {
          if ((m_lastEnforceLogMs == 0) || ((uint32_t)(nowMs - m_lastEnforceLogMs) >= kDV_ENFORCE_LOG_MS))
          {
            hw.interfaceComm.printToInterface(
                "[DV-LOCK] enforce dv=%u mV I=%ld mA\r\n",
                (unsigned)dv_mV, (long)current_mA);
            m_lastEnforceLogMs = nowMs;
          }
          if(pack->u16ModuleCellVoltageMin > 3100)
          {
             hw.inverterCan.updateMaxChargeLimit(gen->u16MaxChargePackVoltage, 0);
            logChargeClampStatic("DV_LOCK_ENFORCE");
          }
         
        }
        m_lastEnforceMs = nowMs;
      }

      // Periodic status log while active (every 10s)
      if ((uint32_t)(nowMs - m_lastStatusLogMs) >= kDV_STATUS_LOG_MS)
      {
        uint32_t held_ms = (m_unlockStartMs == 0) ? 0 : (nowMs - m_unlockStartMs);
        uint32_t target_ms = (dv_mV <= dvStop_mV) ? kDV_STOP_HOLD_MS : kDV_RECOVERY_HOLD_MS;
        hw.interfaceComm.printToInterface(
            "[DV-LOCK] active dv=%u mV I=%ld mA hold=%lu/%lu ms\r\n",
            (unsigned)dv_mV, (long)current_mA,
            (unsigned long)held_ms, (unsigned long)target_ms);
        m_lastStatusLogMs = nowMs;
      }
    }
  }

  bool active() const { return m_active; }

private:
  void activate(uint32_t nowMs, HwMeasurementBase &hw, st_generalConfig *gen, st_powerElecPackInfoConfig *pack)
  {
    m_active = true;
    m_unlockStartMs = 0;
    m_lastStatusLogMs = nowMs;
    m_lastEnforceMs = nowMs;
    m_lastEnforceLogMs = 0;
    m_rampPending = false;
    m_rampActive = false;
    m_rampCurrent_cA = 0;
    m_rampHoldStartMs = 0;
    hw.inverterCan.updateMaxChargeLimit(gen->u16MaxChargePackVoltage, 0);
    logChargeClampStatic("DV_LOCK_ACTIVATE");
    pack->inverterResponseState = WAITING_FOR_RESPONSE;
    pack->u32InverterCommandTimestamp = nowMs;
  }

  void deactivate(HwMeasurementBase &hw, st_generalConfig *gen, st_powerElecPackInfoConfig *pack)
  {
    (void)hw;
    (void)gen;
    (void)pack;
    m_active = false;
    m_trendStartMs = 0;
    m_unlockStartMs = 0;
    m_rampPending = true;
    m_rampActive = false;
    m_rampCurrent_cA = 0;
    m_rampHoldStartMs = 0;
  }

  bool m_active = false;
  uint32_t m_trendStartMs = 0;
  uint16_t m_dvAtTrendStart_mV = 0;
  uint32_t m_unlockStartMs = 0;
  uint32_t m_lastEnforceMs = 0;
  uint32_t m_lastStatusLogMs = 0;
  uint32_t m_lastEnforceLogMs = 0;
  bool m_rampPending = false;
  bool m_rampActive = false;
  uint32_t m_rampLastMs = 0;
  uint16_t m_rampCurrent_cA = 0;
  uint32_t m_rampHoldStartMs = 0;
};

static protections_params *st_protections_params;
static st_generalConfig *g_ptrGenConfig = nullptr;
static st_powerElecPackInfoConfig *g_ptrPowerElecPackInfoConfig = nullptr;
static HwMeasurementBase *g_hwMeasurement = nullptr;   // Declare the static pointer
static mbed::Timer *g_measurementManagerTim = nullptr; // Declare the static timer pointer
static uint8_t s_bqCommConsecutiveTimeouts = 0;       // BQ comm loss attempts
static bool s_bqCommFaultLatched = false;             // Tracks active comm fault
static bool s_bqCommManualResetRequired = false;      // Require manual intervention after 3 failed attempts
static uint32_t s_bqCommResetCount = 0;               // Auto-reset attempts (monotonic until recovery)
static uint32_t s_bqCommLastResetMs = 0;              // Timestamp of last auto-reset attempt
static uint32_t s_bqCommLastFaultMs = 0;
static uint32_t s_bqCommLastRecoveryMs = 0;
static uint32_t s_bqCommFaultStartMs = 0;             // Timestamp when current comm fault episode started
static bool s_bqCommSystemResetAttempted = false;     // One-shot flag: true after we've tried full system reset
extern volatile uint32_t g_opLoopTimestampMs;         // exported from Operation thread

bool Measurement::isBalanceMeasureWindow() const
{
  return m_bBalanceMeasurePhase || m_balanceState.periodicPauseActive;
}

uint16_t Measurement::getDvMvCleanAware() const
{
  const uint16_t liveVmin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
  const uint16_t liveVmax = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax;

  // While balancing is active and we are in the ACTIVE phase (bleeders on), prefer
  // the cached clean readings captured during the MEASURE phase to avoid IR-drop noise.
  const bool measureWindow = isBalanceMeasureWindow();
  if (m_ptrPowerElecPackInfoConfig->bBalancingActive && !measureWindow)
  {
    const uint16_t cachedVmin = m_balanceState.cachedVmin;
    const uint16_t cachedVmax = m_balanceState.cachedVmax;
    if (cachedVmax >= cachedVmin && cachedVmax != 0)
      return static_cast<uint16_t>(cachedVmax - cachedVmin);
  }

  if (liveVmax >= liveVmin)
    return static_cast<uint16_t>(liveVmax - liveVmin);
  return 0;
}

static uint16_t enforceGlobalBleedLimit(st_cellMonitorCells *map, size_t N, uint16_t maxTotal)
{
  if (!map || maxTotal == 0)
    return 0;

  struct BleedCandidate
  {
    uint16_t idx;
    uint16_t mv;
  };

  BleedCandidate candidates[NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS];
  uint16_t count = 0;
  for (size_t i = 0; i < N; ++i)
  {
    if (!map[i].bCellBleedActive)
      continue;
    candidates[count++] = {static_cast<uint16_t>(i), map[i].u16CellVoltage};
  }

  if (count <= maxTotal)
    return count;

  for (uint16_t i = 0; i < count; ++i)
  {
    uint16_t maxIdx = i;
    for (uint16_t j = (uint16_t)(i + 1); j < count; ++j)
    {
      if (candidates[j].mv > candidates[maxIdx].mv)
        maxIdx = j;
    }
    if (maxIdx != i)
    {
      const BleedCandidate tmp = candidates[i];
      candidates[i] = candidates[maxIdx];
      candidates[maxIdx] = tmp;
    }
  }

  for (uint16_t i = maxTotal; i < count; ++i)
  {
    map[candidates[i].idx].bCellBleedActive = false;
  }

  return maxTotal;
}

static void clearBleedFlags(st_powerElecPackInfoConfig *pack)
{
  if (!pack)
    return;
  const size_t count = NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
  for (size_t i = 0; i < count; ++i)
  {
    pack->cellVoltagesIndividual[i].bCellBleedActive = false;
  }
}

static void applyBleedFlagsFromMap(st_powerElecPackInfoConfig *pack, const st_cellMonitorCells *map, size_t count)
{
  if (!pack || !map)
    return;
  for (size_t i = 0; i < count; ++i)
  {
    pack->cellVoltagesIndividual[i].bCellBleedActive = map[i].bCellBleedActive;
  }
}

static bool isCellMaskedByBleed(const st_powerElecPackInfoConfig *pack,
                                uint8_t module,
                                uint8_t cell,
                                uint8_t cellsPerModule,
                                bool includeNeighbors)
{
  if (!pack)
    return false;
  const size_t base = static_cast<size_t>(module) * NoOfCELL_POSSIBLE_ON_CHIP;
  const size_t idx = base + cell;
  if (pack->cellVoltagesIndividual[idx].bCellBleedActive)
    return true;
  if (!includeNeighbors)
    return false;
  if (cell > 0 && pack->cellVoltagesIndividual[base + cell - 1].bCellBleedActive)
    return true;
  if ((uint8_t)(cell + 1) < cellsPerModule && pack->cellVoltagesIndividual[base + cell + 1].bCellBleedActive)
    return true;
  return false;
}

static bool isMinAdjacentToBleed(const st_powerElecPackInfoConfig *pack,
                                 uint8_t module,
                                 uint8_t cell,
                                 uint8_t cellsPerModule)
{
  return isCellMaskedByBleed(pack, module, cell, cellsPerModule, true);
}

static void updateBleedMaskFromMap(BalanceState &state, const st_cellMonitorCells *map, size_t count)
{
  if (!map)
  {
    state.bleedMaskValid = false;
    return;
  }
  const size_t maxCount = NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
  const size_t n = (count > maxCount) ? maxCount : count;
  for (size_t i = 0; i < n; ++i)
  {
    state.bleedMask[i] = map[i].bCellBleedActive;
  }
  for (size_t i = n; i < maxCount; ++i)
  {
    state.bleedMask[i] = false;
  }
  state.bleedMaskValid = true;
}

static void clearBleedMask(BalanceState &state)
{
  const size_t count = NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
  for (size_t i = 0; i < count; ++i)
  {
    state.bleedMask[i] = false;
  }
  state.bleedMaskValid = false;
}

static bool isBleedMasked(const BalanceState &state,
                          uint8_t module,
                          uint8_t cell,
                          uint8_t cellsPerModule,
                          bool includeNeighbors)
{
  if (!state.bleedMaskValid)
    return false;
  const size_t base = static_cast<size_t>(module) * NoOfCELL_POSSIBLE_ON_CHIP;
  const size_t idx = base + cell;
  if (state.bleedMask[idx])
    return true;
  if (!includeNeighbors)
    return false;
  if (cell > 0 && state.bleedMask[base + cell - 1])
    return true;
  if ((uint8_t)(cell + 1) < cellsPerModule && state.bleedMask[base + cell + 1])
    return true;
  return false;
}

static bool computeBalancingExtremes(const st_powerElecPackInfoConfig *pack,
                                     const st_generalConfig *gen,
                                     const BalanceState &state,
                                     uint16_t &outMin,
                                     uint16_t &outMax)
{
  if (!pack || !gen)
    return false;
  uint8_t nModules = gen->u8NoOfCellsPerModule;
  uint8_t nCells = gen->u8NumberOfCells;
  if (nModules > NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
    nModules = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
  if (nCells > NoOfCELL_POSSIBLE_ON_CHIP)
    nCells = NoOfCELL_POSSIBLE_ON_CHIP;

  if (nModules == 0 || nCells == 0)
    return false;

  uint16_t minAll = 10000;
  uint16_t maxAll = 0;
  uint16_t minMasked = 10000;
  uint16_t maxMasked = 0;
  const bool maskMin = BalancingArgs::FILTER_MIN_ADJACENT_TO_BLEED && state.bleedMaskValid;
  const uint8_t maxMode = BalancingArgs::FILTER_MAX_MODE;
  const bool maskMax = state.bleedMaskValid && (maxMode != 0u);
  const bool maskMaxNeighbors = (maxMode >= 2u);

  for (uint8_t m = 0; m < nModules; ++m)
  {
    for (uint8_t c = 0; c < nCells; ++c)
    {
      const size_t idx = (static_cast<size_t>(m) * NoOfCELL_POSSIBLE_ON_CHIP) + c;
      const uint16_t v = pack->cellVoltagesIndividual[idx].u16CellVoltage;
      if (v == 0)
        continue;
      if (v < minAll)
        minAll = v;
      if (v > maxAll)
        maxAll = v;
      if (!maskMin || !isBleedMasked(state, m, c, nCells, true))
      {
        if (v < minMasked)
          minMasked = v;
      }
      if (!maskMax || !isBleedMasked(state, m, c, nCells, maskMaxNeighbors))
      {
        if (v > maxMasked)
          maxMasked = v;
      }
    }
  }

  if (minAll == 10000 || maxAll == 0)
    return false;

  outMin = (maskMin && minMasked != 10000) ? minMasked : minAll;
  outMax = (maskMax && maxMasked != 0) ? maxMasked : maxAll;
  if (outMax < outMin)
  {
    outMin = minAll;
    outMax = maxAll;
  }
  return true;
}

// Helper: log when we clamp charge to 0, with a short rate limit to avoid spam.
static void logChargeClampStatic(const char *reason)
{
  if (!g_hwMeasurement || !g_ptrPowerElecPackInfoConfig || !g_measurementManagerTim)
    return;
  static uint32_t s_lastLogMs = 0;
  const uint32_t nowMs = MEASUREMENT_GET_TICK((*g_measurementManagerTim));
  if ((uint32_t)(nowMs - s_lastLogMs) < 500)
    return;
  s_lastLogMs = nowMs;
  g_hwMeasurement->interfaceComm.printToInterface(
      "[CLAMP] %s ov=%u inv=%u chgAllowed=%d op=%u bms=%u dv=%u I=%ld bal=%d\r\n",
      reason,
      (unsigned)g_ptrPowerElecPackInfoConfig->overvoltageState,
      (unsigned)g_ptrPowerElecPackInfoConfig->inverterResponseState,
      (int)g_ptrPowerElecPackInfoConfig->bChargeAllowed,
      (unsigned)g_ptrPowerElecPackInfoConfig->u8operationState,
      (unsigned)g_ptrPowerElecPackInfoConfig->bmsState,
      (unsigned)g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch,
      (long)g_ptrPowerElecPackInfoConfig->i32ModuleCurrent,
      (int)g_ptrPowerElecPackInfoConfig->bBalancingActive);
}

static bool s_dischargeClampActive = false;
static uint16_t s_lastDischargeLimit_cA = 0xFFFFu;

static void clampInverterDischargeZero(const char *reason)
{
  if (!g_hwMeasurement || !g_ptrGenConfig)
    return;
  g_hwMeasurement->inverterCan.updateMaxDischargeLimit(g_ptrGenConfig->u16MaxDischargePackVoltage, 0);
  s_dischargeClampActive = true;
  s_lastDischargeLimit_cA = 0xFFFFu;
  logChargeClampStatic(reason);
}

static void restoreInverterDischargeMax(const char *reason)
{
  if (!g_hwMeasurement || !g_ptrGenConfig)
    return;
  const uint16_t target_cA =
      derateDischargeCurrent_cA(g_ptrGenConfig->u16MaxHardDchgAllowedCurrent, g_ptrGenConfig, g_ptrPowerElecPackInfoConfig);
  g_hwMeasurement->inverterCan.updateMaxDischargeLimit(
      g_ptrGenConfig->u16MaxDischargePackVoltage,
      target_cA);
  s_dischargeClampActive = false;
  s_lastDischargeLimit_cA = target_cA;
  logChargeClampStatic(reason);
}

static void apply_soft_uv_throttle(void)
{
  static bool s_uvThrottle = false;

  if (!g_hwMeasurement || !g_ptrGenConfig || !g_ptrPowerElecPackInfoConfig)
    return;

  const uint16_t vmin = g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
  const uint16_t soft = g_ptrGenConfig->u16SoftLowCurrentUnderVoltage;
  const uint16_t hard = g_ptrGenConfig->u16HardUnderVoltage;
  const uint16_t hyst = g_ptrGenConfig->u16HysteresisDischarge;

  if (s_dischargeClampActive)
    return;

  if (vmin <= hard)
  {
    s_uvThrottle = false;
    s_lastDischargeLimit_cA = 0xFFFFu;
    return;
  }

  if (vmin <= soft)
  {
    s_uvThrottle = true;
  }
  else if (s_uvThrottle && vmin >= (uint16_t)(soft + hyst))
  {
    s_uvThrottle = false;
  }

  const uint16_t base_cA = s_uvThrottle ? g_ptrGenConfig->u16MaxSoftDchgAllowedCurrent
                                        : g_ptrGenConfig->u16MaxHardDchgAllowedCurrent;
  const uint16_t target_cA = derateDischargeCurrent_cA(base_cA, g_ptrGenConfig, g_ptrPowerElecPackInfoConfig);
  if (target_cA != s_lastDischargeLimit_cA)
  {
    g_hwMeasurement->inverterCan.updateMaxDischargeLimit(
        g_ptrGenConfig->u16MaxDischargePackVoltage,
        target_cA);
    s_lastDischargeLimit_cA = target_cA;
  }
}

// ##################################################################################################################

static bool sysUVDetected(void);
static void sysUVDetectedCallback(void);
static bool sysUVReleased(void);
static void sysUVReleasedCallback(void);

static bool sysOVDetected(void);
static void sysOVDetectedCallback(void);
static bool sysOVReleased(void);
static void sysOVReleasedCallback(void);
static uint16_t getOvVmaxEffective(void);

static bool sysDOVDetected(void);
static void sysDOVDetectedCallback(void);
static bool sysDOVReleased(void);
static void sysDOVReleasedCallback(void);

static bool sysOTCDetected(void);
static void sysOTCDetectedCallback(void);
static bool sysOTCReleased(void);
static void sysOTCReleasedCallback(void);

static bool sysOTDDetected(void);
static void sysOTDDetectedCallback(void);
static bool sysOTDReleased(void);
static void sysOTDReleasedCallback(void);

static bool sysUTCDetected(void);
static void sysUTCDetectedCallback(void);
static bool sysUTCReleased(void);
static void sysUTCReleasedCallback(void);

static bool sysUTDDetected(void);
static void sysUTDDetectedCallback(void);
static bool sysUTDReleased(void);
static void sysUTDReleasedCallback(void);

static bool sysModuleOVDetected(void);
static void sysModuleOVDetectedCallback(void);
static bool sysModuleOVReleased(void);
static void sysModuleOVReleasedCallback(void);

static bool sysDchgOCDetected(void);
static void sysDchgOCDetectedCallback(void);
static bool sysDchgOCReleased(void);
static void sysDchgOCReleasedCallback(void);

static bool sysChgOCDetected(void);
static void sysChgOCDetectedCallback(void);
static bool sysChgOCReleased(void);
static void sysChgOCReleasedCallback(void);

static void log_protection_transition(uint8_t protIndex,
                                      enProtectionState newState,
                                      uint32_t nowMs,
                                      uint32_t lastChangeMs)
{
  uint32_t dt = (nowMs >= lastChangeMs) ? (nowMs - lastChangeMs) : 0;
  if (lastChangeMs == 0)
    dt = 0;
  uint16_t vmin = g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
  uint16_t vmax = g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax;
  uint16_t dv = g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch;
  uint32_t vpack = g_ptrPowerElecPackInfoConfig->u32ModuleVoltage;
  int32_t imA = g_ptrPowerElecPackInfoConfig->i32ModuleCurrent;
  int32_t tmax = g_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax;
  int32_t tmin = g_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin;

  InterfaceCommHandler::getInstance()->printToInterface(
      "--- Protection: %s -> %s (dt=%lu ms) ---\r\n",
      prot_name_from_index(protIndex),
      prot_state_to_str(newState),
      (unsigned long)dt);

  switch (protIndex)
  {
  case MODULE_UNDER_CELL_VOLTAGE:
    InterfaceCommHandler::getInstance()->printToInterface(
        "  vmin=%u mV soft=%u hard=%u hyst=%u\r\n",
        (unsigned)vmin,
        (unsigned)g_ptrGenConfig->u16SoftLowCurrentUnderVoltage,
        (unsigned)g_ptrGenConfig->u16HardUnderVoltage,
        (unsigned)g_ptrGenConfig->u16HysteresisDischarge);
    break;
  case MODULE_OVER_CELL_VOLTAGE:
    InterfaceCommHandler::getInstance()->printToInterface(
        "  vmax=%u mV soft=%u hard=%u hyst=%u\r\n",
        (unsigned)vmax,
        (unsigned)g_ptrGenConfig->u16SoftOverVoltage,
        (unsigned)g_ptrGenConfig->u16HardOverVoltage,
        (unsigned)g_ptrGenConfig->u16HysteresisCharge);
    break;
  case MODULE_OVER_DIFF_CELL_VOLTAGE:
    InterfaceCommHandler::getInstance()->printToInterface(
        "  dv=%u mV thresh=%u hyst=%u\r\n",
        (unsigned)dv,
        (unsigned)g_ptrGenConfig->u16OverMismatchVoltage,
        (unsigned)g_ptrGenConfig->u16HysteresisOverMismatchVoltage);
    break;
  case MODULE_OVER_VOLTAGE:
  {
    uint32_t modThresh =
        (g_ptrGenConfig->u8NoOfCellsPerModule * g_ptrGenConfig->u8NumberOfCells *
         g_ptrGenConfig->u16HardOverVoltage) + ProtectionConstants::MODULE_VOLTAGE_MARGIN_MV;
    InterfaceCommHandler::getInstance()->printToInterface(
        "  vpack=%lu mV thresh=%lu mV\r\n",
        (unsigned long)vpack,
        (unsigned long)modThresh);
  }
  break;
  case MODULE_OVER_CURRENT_DISCHARGE:
    InterfaceCommHandler::getInstance()->printToInterface(
        "  I=%ld mA soft=%u hyst=%u\r\n",
        (long)imA,
        (unsigned)(g_ptrGenConfig->u16MaxSoftDchgAllowedCurrent * 10U),
        (unsigned)g_ptrGenConfig->u16HysteresisCurrent);
    break;
  case MODULE_OVER_CURRENT_CHARGE:
    InterfaceCommHandler::getInstance()->printToInterface(
        "  I=%ld mA soft=%u hyst=%u\r\n",
        (long)imA,
        (unsigned)(g_ptrGenConfig->u16MaxSoftChgAllowedCurrent * 10U),
        (unsigned)g_ptrGenConfig->u16HysteresisCurrent);
    break;
  case MODULE_OVER_TEMPERATURE:
    InterfaceCommHandler::getInstance()->printToInterface(
        "  tmax=%ld mC limit=%u mC hyst=%u\r\n",
        (long)tmax,
        (unsigned)(g_ptrGenConfig->u16AllowedTempBattDischargingMax * 100U),
        (unsigned)(g_ptrGenConfig->u16HysteresisOverTemperature * 100U));
    break;
  case MODULE_UNDER_TEMPERATURE:
    InterfaceCommHandler::getInstance()->printToInterface(
        "  tmin=%ld mC limit=%u mC hyst=%u\r\n",
        (long)tmin,
        (unsigned)(g_ptrGenConfig->u16AllowedTempBattDischargingMin * 100U),
        (unsigned)(g_ptrGenConfig->u16HysteresisOverTemperature * 100U));
    break;
  default:
    InterfaceCommHandler::getInstance()->printToInterface(
        "  vmin=%u vmax=%u dv=%u I=%ld tmin=%ld tmax=%ld\r\n",
        (unsigned)vmin, (unsigned)vmax, (unsigned)dv, (long)imA, (long)tmin, (long)tmax);
    break;
  }
}

// #############################################################################################//

const uint8_t PROTECTIONS_TYPE_NUMBER = 10;
protections_params all_protections_list[] =
    {
        {//  Modele Cell UnderVoltage
         .bProtectionsMeasurementDetectionFlag = true,
         .bProtectionsTimeoutFlag = false,
         .bProtectionsReleaseFlag = false,
         .u8ProtectionsCount = 0,
         .ku8ConsecutiveReadingsThreshold = 6,
         .u32ProtectionsDetectionTimeout = 0,
         .u32ProtectionsDetectionCountingTimeout = 0,
         .u32ProtectionsProtectionReleaseTimeout = 0,
         .u32ProtectionsClearTimeout = 0,
         .ku32DetectionCountingTimeoutMs = 500,
         .ku32DetectionTimeoutMs = 3000,
         .ku32ReleaseTimeoutMs = 3000,
         .ku32ProtectionStateClearTimeoutMs = 1000,
         .protectionsDetected = sysUVDetected,
         .protectionsDetectedCallback = sysUVDetectedCallback,
         .protectionsReleased = sysUVReleased,
         .protectionsReleasedCallback = sysUVReleasedCallback},
        {//  Modele Cell OverVoltage
         .bProtectionsMeasurementDetectionFlag = true,
         .bProtectionsTimeoutFlag = false,
         .bProtectionsReleaseFlag = false,
         .u8ProtectionsCount = 0,
         .ku8ConsecutiveReadingsThreshold = 6,
         .u32ProtectionsDetectionTimeout = 0,
         .u32ProtectionsDetectionCountingTimeout = 0,
         .u32ProtectionsProtectionReleaseTimeout = 0,
         .u32ProtectionsClearTimeout = 0,
         .ku32DetectionCountingTimeoutMs = 500,
         .ku32DetectionTimeoutMs = 3000,
         .ku32ReleaseTimeoutMs = 3000,
         .ku32ProtectionStateClearTimeoutMs = 1000,
         .protectionsDetected = sysOVDetected,
         .protectionsDetectedCallback = sysOVDetectedCallback,
         .protectionsReleased = sysOVReleased,
         .protectionsReleasedCallback = sysOVReleasedCallback

        },
        {//  Modele Cell Difference OverVoltage
         .bProtectionsMeasurementDetectionFlag = true,
         .bProtectionsTimeoutFlag = false,
         .bProtectionsReleaseFlag = false,
         .u8ProtectionsCount = 0,
         .ku8ConsecutiveReadingsThreshold = 6,
         .u32ProtectionsDetectionTimeout = 0,
         .u32ProtectionsDetectionCountingTimeout = 0,
         .u32ProtectionsProtectionReleaseTimeout = 0,
         .u32ProtectionsClearTimeout = 0,
         .ku32DetectionCountingTimeoutMs = 500,
         .ku32DetectionTimeoutMs = 3000,
         .ku32ReleaseTimeoutMs = 3000,
         .ku32ProtectionStateClearTimeoutMs = 1000,
         .protectionsDetected = sysDOVDetected,
         .protectionsDetectedCallback = sysDOVDetectedCallback,
         .protectionsReleased = sysDOVReleased,
         .protectionsReleasedCallback = sysDOVReleasedCallback},
        {//  Module OverVoltage
         .bProtectionsMeasurementDetectionFlag = true,
         .bProtectionsTimeoutFlag = false,
         .bProtectionsReleaseFlag = false,
         .u8ProtectionsCount = 0,
         .ku8ConsecutiveReadingsThreshold = 6,
         .u32ProtectionsDetectionTimeout = 0,
         .u32ProtectionsDetectionCountingTimeout = 0,
         .u32ProtectionsProtectionReleaseTimeout = 0,
         .u32ProtectionsClearTimeout = 0,
         .ku32DetectionCountingTimeoutMs = 500,
         .ku32DetectionTimeoutMs = 3000,
         .ku32ReleaseTimeoutMs = 3000,
         .ku32ProtectionStateClearTimeoutMs = 1000,
         .protectionsDetected = sysModuleOVDetected,
         .protectionsDetectedCallback = sysModuleOVDetectedCallback,
         .protectionsReleased = sysModuleOVReleased,
         .protectionsReleasedCallback = sysModuleOVReleasedCallback},
        {//  Discharge Overcurrent
         .bProtectionsMeasurementDetectionFlag = true,
         .bProtectionsTimeoutFlag = false,
         .bProtectionsReleaseFlag = false,
         .u8ProtectionsCount = 0,
         .ku8ConsecutiveReadingsThreshold = 6,
         .u32ProtectionsDetectionTimeout = 0,
         .u32ProtectionsDetectionCountingTimeout = 0,
         .u32ProtectionsProtectionReleaseTimeout = 0,
         .u32ProtectionsClearTimeout = 0,
         .ku32DetectionCountingTimeoutMs = 500,
         .ku32DetectionTimeoutMs = 50, // 0
         .ku32ReleaseTimeoutMs = 2000,
         .ku32ProtectionStateClearTimeoutMs = 100,
         .protectionsDetected = sysDchgOCDetected,
         .protectionsDetectedCallback = sysDchgOCDetectedCallback,
         .protectionsReleased = sysDchgOCReleased,
         .protectionsReleasedCallback = sysDchgOCReleasedCallback},
        {//  Charge Overcurrent
         .bProtectionsMeasurementDetectionFlag = true,
         .bProtectionsTimeoutFlag = false,
         .bProtectionsReleaseFlag = false,
         .u8ProtectionsCount = 0,
         .ku8ConsecutiveReadingsThreshold = 6,
         .u32ProtectionsDetectionTimeout = 0,
         .u32ProtectionsDetectionCountingTimeout = 0,
         .u32ProtectionsProtectionReleaseTimeout = 0,
         .u32ProtectionsClearTimeout = 0,
         .ku32DetectionCountingTimeoutMs = 500,
         .ku32DetectionTimeoutMs = 50, // 0
         .ku32ReleaseTimeoutMs = 2000,
         .ku32ProtectionStateClearTimeoutMs = 100,
         .protectionsDetected = sysChgOCDetected,
         .protectionsDetectedCallback = sysChgOCDetectedCallback,
         .protectionsReleased = sysChgOCReleased,
         .protectionsReleasedCallback = sysChgOCReleasedCallback},
        {// Over-Temperature during Charge (OTC)
         .ku32DetectionTimeoutMs = 3000,
         .ku32ReleaseTimeoutMs = 1000,
         .protectionsDetected = sysOTCDetected,
         .protectionsDetectedCallback = sysOTCDetectedCallback,
         .protectionsReleased = sysOTCReleased,
         .protectionsReleasedCallback = sysOTCReleasedCallback},

        {// Over-Temperature during Discharge (OTD)
         .ku32DetectionTimeoutMs = 3000,
         .ku32ReleaseTimeoutMs = 1000,
         .protectionsDetected = sysOTDDetected,
         .protectionsDetectedCallback = sysOTDDetectedCallback,
         .protectionsReleased = sysOTDReleased,
         .protectionsReleasedCallback = sysOTDReleasedCallback},

        {// Under-Temperature during Charge (UTC)
         .ku32DetectionTimeoutMs = 3000,
         .ku32ReleaseTimeoutMs = 1000,
         .protectionsDetected = sysUTCDetected,
         .protectionsDetectedCallback = sysUTCDetectedCallback,
         .protectionsReleased = sysUTCReleased,
         .protectionsReleasedCallback = sysUTCReleasedCallback},

        {// Under-Temperature during Discharge (UTD)
         .ku32DetectionTimeoutMs = 3000,
         .ku32ReleaseTimeoutMs = 1000,
         .protectionsDetected = sysUTDDetected,
         .protectionsDetectedCallback = sysUTDDetectedCallback,
         .protectionsReleased = sysUTDReleased,
         .protectionsReleasedCallback = sysUTDReleasedCallback}

};

// #############################################################################################################################################################################
Measurement::Measurement(Mail<st_mailPowerElecPackInfoConfig, BQ_MEASUREMENT_COMMS_MAIL_SIZE> &_mailPowerElecBox,
                         Mail<st_mail_cell_voltages_t, LOGGING_CELL_VOLTAGE_MAIL_SIZE> &_mailCellVoltageBox,
                         HwMeasurementBase &_hwMeasurement)
    : m_hwMeasurement(_hwMeasurement),
      m_mailPowerElecBox(_mailPowerElecBox),
      m_mailCellVoltageBox(_mailCellVoltageBox),
      m_u8SoftOverCurrentErrorCount(0),
      m_u16HardOverTemperatureFlags(0),
      m_bPowerElecAllowForcedOnState(false),
      m_measurementThreadVar(MEASUREMENT_THREAD_PRIORITY, MEASUREMENT_THREAD_STACK_SIZE, nullptr, "Measurement")
{
  m_measurementManagerTim.start();
}

// #############################################################################################################################################################################

Measurement::~Measurement()
{
}

// #############################################################################################################################################################################

void Measurement::startThread(void)
{
  m_measurementThreadVar.start(mbed::callback(this, &Measurement::measurementThread));
}

// #############################################################################################################################################################################

void Measurement::init(st_generalConfig *genConfig, st_powerElecPackInfoConfig *powerElecPackInfoConfig)
{
  m_u16SOH_mpermil = (uint16_t)(DEFAULT_SOH * 10);
  m_u32CycleCount = 0;

  m_fRemainingCapacityAh = 0.0f;

  m_bInitialSocEstimationComplete = false;
  m_u32VoltageStableTimestamp = 0;
  m_u16LastCellVoltageMin = 0;

  // Highlight: Initialize the new flag to false.
  m_bInitialSocSet = false;

  // Highlight: Initialize the new timestamp.
  m_u32IdleStateEntryTimestamp = 0;

  m_ptrGenConfig = genConfig;
  m_ptrPowerElecPackInfoConfig = powerElecPackInfoConfig;
  // Highlight: Initialize the new overvoltage state to normal.
  m_ptrPowerElecPackInfoConfig->overvoltageState = OV_STATE_NORMAL;

  m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxBal = 0;
  m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinBal = 0;
  m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatchBal = 0;

  m_ptrPowerElecPackInfoConfig->inverterResponseState = INVERTER_OK;
  m_ptrPowerElecPackInfoConfig->u32InverterCommandTimestamp = 0;

  g_hwMeasurement = &m_hwMeasurement;                 // Initialize the pointer
  g_measurementManagerTim = &m_measurementManagerTim; // Initialize the pointer

  // Highlight: Initialize the new save request flag.
  m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest = false;
  {
    const uint16_t commissioning_raw = m_hwMeasurement.eeprom.getSettingValue(CMD_EE_ADDR_CommissioningDone);
#if FORCE_COMMISSIONING_DONE
    // Temporary override: force commissioning to be done
    m_commissioningActive = false;
    (void)commissioning_raw;  // Suppress unused warning
#else
    m_commissioningActive = (commissioning_raw != 1);
#endif
    m_commissioningStopActive = false;
    m_commissioningUseLowCurrent = false;
    m_commissioningLastLimit = 0xFFFF;
    m_commissioningDvAtTrendStart_mV = 0;
    m_commissioningTrendStartMs = 0;
    m_commissioningDvStableStartMs = 0;
    m_commissioningFullChargeStartMs = 0;
  }
  // Prefer legacy stored SOC; fall back to journal if legacy is missing/invalid.
  bool soc_loaded = false;
  uint16_t mappedVal = m_hwMeasurement.eeprom.getSettingValue(CMD_EE_ADDR_MappedEepromVal);
  m_hwMeasurement.interfaceComm.printToInterface("[SOC-DBG] MappedVal=0x%04X (expect 0x%04X)\r\n",
      (unsigned)mappedVal, (unsigned)EEPROM_MAPPED_VALUE);
      
  if (mappedVal == EEPROM_MAPPED_VALUE)
  {
    // If the mapping is valid, THEN we can try to load the legacy stored SOC.
    uint16_t capacity_l = m_hwMeasurement.eeprom.getSettingValue(CMD_EE_ADDR_LastRemainingCapacity_L);
    uint16_t capacity_h = m_hwMeasurement.eeprom.getSettingValue(CMD_EE_ADDR_LastRemainingCapacity_H);
    uint32_t remaining_capacity_mah = ((uint32_t)capacity_h << 16) | capacity_l;

    m_hwMeasurement.interfaceComm.printToInterface("[SOC-DBG] EEPROM: L=0x%04X H=0x%04X mAh=%lu cap=%u\r\n",
        (unsigned)capacity_l, (unsigned)capacity_h, 
        (unsigned long)remaining_capacity_mah, (unsigned)genConfig->u16BatteryCapacity);

    // Validate the loaded SOC value.
    if (remaining_capacity_mah != 0xFFFFFFFF && remaining_capacity_mah <= (uint32_t)genConfig->u16BatteryCapacity * 1000)
    {
      // If the value is valid, use it and mark the initial SOC as set.
      m_fRemainingCapacityAh = (float)remaining_capacity_mah / 1000.0f;
      // Also compute and set u16ModuleSoc so that soc_known check in subTaskSetInitialSoc
      // will see a valid SOC and not override with OCV
      if (genConfig->u16BatteryCapacity > 0)
      {
        uint16_t soc_x10 = (uint16_t)((m_fRemainingCapacityAh / (float)genConfig->u16BatteryCapacity) * 1000.0f);
        if (soc_x10 > 1000) soc_x10 = 1000;
        m_ptrPowerElecPackInfoConfig->u16ModuleSoc = soc_x10;
      }
      m_bInitialSocSet = true;
      soc_loaded = true;
      m_hwMeasurement.interfaceComm.printToInterface("[SOC-BOOT] EEPROM loaded: %u.%02u Ah, SOC=%u.%u%%\r\n",
          (unsigned)(int)m_fRemainingCapacityAh,
          (unsigned)((int)(m_fRemainingCapacityAh * 100.0f) % 100),
          (unsigned)(m_ptrPowerElecPackInfoConfig->u16ModuleSoc / 10),
          (unsigned)(m_ptrPowerElecPackInfoConfig->u16ModuleSoc % 10));
    }
    else
    {
      m_hwMeasurement.interfaceComm.printToInterface("[SOC-BOOT] EEPROM invalid: mAh=%lu, cap=%u (max=%lu)\r\n",
          (unsigned long)remaining_capacity_mah, (unsigned)genConfig->u16BatteryCapacity,
          (unsigned long)(genConfig->u16BatteryCapacity * 1000UL));
    }
  }
  else
  {
    m_hwMeasurement.interfaceComm.printToInterface("[SOC-DBG] EEPROM mapping FAILED, skipping legacy load\r\n");
  }
  if (!soc_loaded)
  {
    EepromM24C24Args::SocJournalRecord rec = {};
    if (m_hwMeasurement.eeprom.loadLatestSocJournal(rec))
    {
      const int32_t max_mah = static_cast<int32_t>(genConfig->u16BatteryCapacity) * 1000;
      if (rec.remaining_mAh >= 0 && rec.remaining_mAh <= max_mah && rec.soc_mpermil <= 1000)
      {
        m_fRemainingCapacityAh = static_cast<float>(rec.remaining_mAh) / 1000.0f;
        m_ptrPowerElecPackInfoConfig->u16ModuleSoc = rec.soc_mpermil;
        m_bInitialSocSet = true;
        soc_loaded = true;
      }
    }
  }
  // If the EEPROM mapping was invalid OR the stored SOC was invalid,
  // m_bInitialSocSet will remain false, forcing a new OCV estimation.

#if FORCE_SOC_90_PERCENT
  // ONE-TIME: Force SOC to 93.8% and save to EEPROM
  {
    const float forced_soc = 0.938f;  // 93.8%
    m_fRemainingCapacityAh = forced_soc * (float)genConfig->u16BatteryCapacity;
    m_ptrPowerElecPackInfoConfig->u16ModuleSoc = 938;  // 93.8% in x0.1% units
    m_bInitialSocSet = true;
    
    // Save to legacy EEPROM immediately
    uint32_t remaining_capacity_mah = (uint32_t)(m_fRemainingCapacityAh * 1000.0f);
    m_hwMeasurement.eeprom.setSettingValue(CMD_EE_ADDR_LastRemainingCapacity_L, (uint16_t)(remaining_capacity_mah & 0xFFFF));
    ThisThread::sleep_for(10ms);
    m_hwMeasurement.eeprom.setSettingValue(CMD_EE_ADDR_LastRemainingCapacity_H, (uint16_t)((remaining_capacity_mah >> 16) & 0xFFFF));
    ThisThread::sleep_for(10ms);
    
    // Also save to journal immediately (not just request flag)
    EepromM24C24Args::SocJournalRecord rec = {};
    rec.soc_mpermil = 938;  // 93.8%
    rec.remaining_mAh = (int32_t)remaining_capacity_mah;
    rec.vmin_mV = 0;
    rec.current_mA = 0;
    rec.temp_mC = 0;
    rec.ocv_mV = 0;
    rec.timestamp_ms = 0;
    m_hwMeasurement.eeprom.appendSocJournal(rec);
    
    m_hwMeasurement.interfaceComm.printToInterface("[SOC] FORCED to 90%% (%.1f Ah) saved to EEPROM + Journal\r\n", m_fRemainingCapacityAh);
  }
#endif
  m_u32IdleStateEntryTimestamp = 0;
  st_protections_params = all_protections_list;

  // Assign the global pointers
  g_ptrGenConfig = genConfig;
  g_ptrPowerElecPackInfoConfig = powerElecPackInfoConfig;

  m_ptrPowerElecPackInfoConfig->i32ModuleCurrent = 0;
  m_ptrPowerElecPackInfoConfig->i32ModulePower = 0;
  m_ptrPowerElecPackInfoConfig->u16SupplyVoltage = 0;
  m_ptrPowerElecPackInfoConfig->u16ModuleBusbarVal = 0;
  m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax = 0;
  m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin = 0;
  m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureAverage = 0;
  m_ptrPowerElecPackInfoConfig->u8allowedShutDown = 0;
  m_ptrPowerElecPackInfoConfig->u8ActiveModuleNumber = 1;
  m_ptrPowerElecPackInfoConfig->bDischargeDesiredFlag = false;
  m_ptrPowerElecPackInfoConfig->bChargeDesiredFlag = false;
  m_ptrPowerElecPackInfoConfig->bPredischargeDesiredFlag = false;
  m_ptrPowerElecPackInfoConfig->bChargeAllowed = true;
  m_ptrPowerElecPackInfoConfig->bDischargeHighCurrentAllowed = true;
  m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed = true;
  m_ptrPowerElecPackInfoConfig->bChargeBalanceActive = false;
  m_ptrPowerElecPackInfoConfig->bBalancingActive = false;
  m_ptrPowerElecPackInfoConfig->bChargeCurrentDetected = false;
  m_ptrPowerElecPackInfoConfig->bPackInSOADischarge = true;
  m_ptrPowerElecPackInfoConfig->bPackInSOACharge = true;
  m_ptrPowerElecPackInfoConfig->bAllowedErrorFlags = false;
  m_ptrPowerElecPackInfoConfig->bPushedShutdownChipSet = false;
  m_ptrPowerElecPackInfoConfig->bTempEnableBatteryDisCharge = false;
  m_ptrPowerElecPackInfoConfig->bTempEnableBatteryCharge = false;
  m_ptrPowerElecPackInfoConfig->bUpdatedSettingFlags = false;

  m_ptrPowerElecPackInfoConfig->u8BuzzerSignalPersistant = true;
  m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = false;
  m_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_OFF;
  m_ptrPowerElecPackInfoConfig->buzzerSignalType = BUZZER_SIGNAL_TYPE_OFF;

  m_ptrPowerElecPackInfoConfig->packOperationCellState = PACK_STATE_NORMAL;
  m_ptrPowerElecPackInfoConfig->packBatteryWarErrState = SYS_OK;
  m_ptrPowerElecPackInfoConfig->packLastErrorState = SYS_OK;

  m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysSecond = 0;
  m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysMinute = 0;
  m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysHour = 0;
  m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysDay = 0;
  m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysMonth = 0;
  m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysYear = 0;

  m_ptrPowerElecPackInfoConfig->rackBalancingState = RACK_BALANCING_INACTIVE;
  m_ptrPowerElecPackInfoConfig->u8InterfaceActiveButtonState = (enInterfaceButtonState)0;
  m_ptrPowerElecPackInfoConfig->u8SoftDischargeOvercurrentReleasingCount = 0;
  m_ptrPowerElecPackInfoConfig->u8SoftChargeOvercurrentReleasingCount = 0;

  m_ptrPowerElecPackInfoConfig->bmsState = BMS_IDLE;
  // REMOVED: m_ptrPowerElecPackInfoConfig->bForceBalanceActive = false; // Dead code removed
  m_ptrPowerElecPackInfoConfig->u8SocConfidence = 0;         // Initialize SOC confidence to 0%

  for (uint8_t modulePointer = 0; modulePointer < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; modulePointer++)
  {
    for (uint8_t cellPointer = 0; cellPointer < NoOfCELL_POSSIBLE_ON_CHIP; cellPointer++)
    {
      m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[(modulePointer * NoOfCELL_POSSIBLE_ON_CHIP) + cellPointer].u16CellVoltage = 0;
      m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[(modulePointer * NoOfCELL_POSSIBLE_ON_CHIP) + cellPointer].bCellBleedActive = false;
    }
  }

  for (uint8_t modulePointer = 0; modulePointer < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; modulePointer++)
  {
    for (uint8_t cellPointer = 0; cellPointer < NoOfCELL_POSSIBLE_ON_CHIP; cellPointer++)
    {
      m_ptrPowerElecPackInfoConfig->cellModuleVoltages[modulePointer][cellPointer] = 0;
    }
  }

  for (uint8_t modulePointer = 0; modulePointer < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; modulePointer++)
  {
    for (uint8_t tempPointer = 0; tempPointer < NoOfTEMP_POSSIBLE_ON_CHIP; tempPointer++)
    {
      m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature[modulePointer][tempPointer] = 0;
    }
  }

  for (uint8_t i = 0; i < NoOfPROTS_POSSIBLE_ON_SYS; i++)
  {
    m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].protectionState = PROT_STATE_OK;
    m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].lastProtectionState = PROT_STATE_OK;
    m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].bOperationProtectionActive = false;
    m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].bLoggingProtectionActive = false;
  }

  m_u32ChargeCurrentDetectionLastTick = MEASUREMENT_GET_TICK(m_measurementManagerTim);
  m_u32BalanceModeActiveLastTick = MEASUREMENT_GET_TICK(m_measurementManagerTim);

  m_u32StateOfChargeLargeCoulombTick = MEASUREMENT_GET_TICK(m_measurementManagerTim);
  m_u32StateOfChargeStoreSoCTick = MEASUREMENT_GET_TICK(m_measurementManagerTim);

  m_u32SOAChargeChangeLastTick = MEASUREMENT_GET_TICK(m_measurementManagerTim);
  m_u32SOADisChargeChangeLastTick = MEASUREMENT_GET_TICK(m_measurementManagerTim);

  m_ptrPowerElecPackInfoConfig->u32LastBqDataTimestamp = 0; // Initialize the timestamp
}
// #############################################################################################################################################################################
static inline uint32_t timerNowMs()
{
  using namespace std::chrono;
  return (uint32_t)duration_cast<milliseconds>(
             Kernel::Clock::now().time_since_epoch())
      .count();
}
// #############################################################################################################################################################################

uint8_t Measurement::timerDelay1ms(uint32_t *last, uint32_t ticks)
{
  if ((uint32_t)(MEASUREMENT_GET_TICK(m_measurementManagerTim) - *last) >= ticks)
  {
    *last = MEASUREMENT_GET_TICK(m_measurementManagerTim);
    return true;
  }
  return false;
}

// #############################################################################################################################################################################
void Measurement::subTaskInitialSocEstimation(void)
{
  const uint32_t VOLTAGE_STABLE_TIME = 10 * 60 * 1000; // 10 minutes
  const uint16_t VOLTAGE_STABLE_THRESHOLD = 10;        // 10mV

  // if (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent == 0)
  if (abs(m_ptrPowerElecPackInfoConfig->i32ModuleCurrent) <= 100)
  {
    if (m_u32VoltageStableTimestamp == 0)
    {
      m_u32VoltageStableTimestamp = MEASUREMENT_GET_TICK(m_measurementManagerTim);
      m_u16LastCellVoltageMin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
    }

    if (abs(m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin - m_u16LastCellVoltageMin) > VOLTAGE_STABLE_THRESHOLD)
    {
      m_u32VoltageStableTimestamp = MEASUREMENT_GET_TICK(m_measurementManagerTim);
      m_u16LastCellVoltageMin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
    }

    if (timerDelay1ms(&m_u32VoltageStableTimestamp, VOLTAGE_STABLE_TIME))
    {
      subTaskSetInitialSoc(false);
      m_bInitialSocEstimationComplete = true;
    }
  }
  else
  {
    m_u32VoltageStableTimestamp = 0;
  }
}
// #############################################################################################################################################################################
void Measurement::measurementThread(void)
{
  uint32_t _u32SocCalculationLastTick = 0;
  uint32_t _u32MeasurementServiceMeasureIntervalLastTick = 0;
  float _fInternalResistanceAsmOhm = 0.0f;
  static bool s_wdt_crosscheck_enable = true; // Set false to disable op-thread stall cross-check
  uint32_t wdt_crosscheck_last_log = 0;

  uint32_t _u32MeasurementServiceStartupDelay =
      (uint32_t)(MEASUREMENT_GET_TICK(m_measurementManagerTim));
  while (!timerDelay1ms(&_u32MeasurementServiceStartupDelay, 100))
    ;

  while (true)
  {
    // 1) Always run SOC via Coulomb counting (every 100 ms)
    if (timerDelay1ms(&_u32SocCalculationLastTick, 100))
    {
      PackInfoGuard guard;
      subTaskSocCalculation();
    }

    // 2) Run the initial OCV estimator NON-BLOCKING in the background
    if (!m_bInitialSocEstimationComplete)
    {
      PackInfoGuard guard;
      subTaskInitialSocEstimation(); // advances its own state machine
    }

    // 3) Periodic (10 ms) service work
    if (timerDelay1ms(&_u32MeasurementServiceMeasureIntervalLastTick, 10))
    {
      static enBmsState lastBmsState = BMS_IDLE;
      if (m_ptrPowerElecPackInfoConfig->bmsState == BMS_IDLE && lastBmsState != BMS_IDLE)
      {
        m_u32IdleStateEntryTimestamp = MEASUREMENT_GET_TICK(m_measurementManagerTim);
        
        // SOC Auto-Complete: If charging just finished and SOC >= 99.1%, round to 100%
        if (lastBmsState == BMS_CHARGE && m_ptrPowerElecPackInfoConfig->u16ModuleSoc >= 991)
        {
            m_ptrPowerElecPackInfoConfig->u16ModuleSoc = 1000;
            m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest = true;
            m_hwMeasurement.interfaceComm.printToInterface("[SOC] Auto-complete: Charged to 100%%\r\n");
        }
      }
      lastBmsState = m_ptrPowerElecPackInfoConfig->bmsState;

      // m_hwMeasurement.serial.serialPrint("Measurement Service\r\n");
    }

    // 4) Main measurement / protection / reporting tasks
    {
        updatePeriodicBalancePause();
        {
          PackInfoGuard guard;
          subTaskMeasurementModulePower();
        }

      {
        PackInfoGuard guard;
        subTaskCheckBqCommunication();
      }
      {
        PackInfoGuard guard;
        subTaskChipsetErrorWatch();
      }
      {
        PackInfoGuard guard;
        subTaskHandleSocSaveRequest();
      }
      if (m_ptrPowerElecPackInfoConfig->packBatteryWarErrState != SYS_ERROR_CHIPSET_COMM)
      {
        if (!m_bInitialSocSet)
        {
          // Keep your existing call; if you adopted the bypass flag, pass false here
          PackInfoGuard guard;
          subTaskSetInitialSoc(); // or subTaskSetInitialSoc(false);
        }

#if !USE_LEGACY_BALANCING
        // Refresh balance phase early so min/max freeze logic uses the correct window for this loop.
        updateBalancePhase(true);
#endif

        {
          PackInfoGuard guard;
          subTaskCellsVoltageWatch();
        }
        {
          PackInfoGuard guard;
          subTaskAllModulePackTemperatureWatch();
        }
        {
          PackInfoGuard guard;
          subTaskSysPowerWatch();
        }
        {
          PackInfoGuard guard;
          subTaskDatetimeWatch();
        }
        {
          PackInfoGuard guard;
          subTaskPackGeneralWatch();
        }
        {
          PackInfoGuard guard;
          subTaskModuleVoltageWatch();
        }
        {
          PackInfoGuard guard;
          subTaskModuleTemperatureWatch();
        }
        {
          PackInfoGuard guard;
          subTaskTemperatureWatch();
        }
        {
          PackInfoGuard guard;
          subTaskCalculateAllCellsStat();
        }
        {
          PackInfoGuard guard;
          // One-shot SOC force when average cell voltage is high enough
          static bool s_avgSocForced = false;
          const enBmsState bmsState_for_soc_force = m_ptrPowerElecPackInfoConfig->bmsState;
          const uint8_t opState_for_soc_force = m_ptrPowerElecPackInfoConfig->u8operationState;

          if (bmsState_for_soc_force == BMS_DISCHARGE || bmsState_for_soc_force == BMS_IDLE || opState_for_soc_force == OP_STATE_BALANCING)
          {
            s_avgSocForced = false;
          }
          if (!s_avgSocForced &&
              m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageAverage >= 3450)
          {
            s_avgSocForced = true;
            m_ptrPowerElecPackInfoConfig->u16ModuleSoc = 1000; // 100.0%
            m_ptrPowerElecPackInfoConfig->u8SocConfidence = 100;
            m_fRemainingCapacityAh = static_cast<float>(m_ptrGenConfig->u16BatteryCapacity);
            m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest = true;
            m_hwMeasurement.interfaceComm.printToInterface(
                "[SOC] Avg>=3.42V (%u mV): force SOC=100%% and save\r\n",
                (unsigned)m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageAverage);
          }
        }
        {
          PackInfoGuard guard;
          subTaskVoltageProtectionWatch();
        }
        {
          PackInfoGuard guard;
          subTaskBuzzer();
        }
        {
          PackInfoGuard guard;
          subTaskCheckPackSOA();
        }
        {
          PackInfoGuard guard;
          subTaskCheckWarning();
        }
        {
          PackInfoGuard guard;
          subTaskPackStateDetermination();
        }
        {
          PackInfoGuard guard;
          subTaskRecalibrateSoc();        // recalibration path (may call subTaskSetInitialSoc(true))
        }
        {
          PackInfoGuard guard;
          subTaskCheckInverterResponse(); // inverter watchdog
        }
        {
          PackInfoGuard guard;
          subTaskOvRecoverySupervisor();  // OV recovery clamp: stop/resume trickle based on Vmax
        }
        {
          PackInfoGuard guard;
          subTaskPackBalancing();
        }
        {
          PackInfoGuard guard;
          subTaskProtectionsControllerMechanism();
        }
        {
          PackInfoGuard guard;
          subTaskInverterValuesReporting();
        }

        // Non-blocking cell-voltage logging
        static uint32_t _u32CellVoltageLogLastTick = 0;
        if (timerDelay1ms(&_u32CellVoltageLogLastTick, 5000))
        {
          st_mail_cell_voltages_t *mail = m_mailCellVoltageBox.try_alloc();
          if (mail)
          {
            memcpy(mail->cell_voltages,
                   m_ptrPowerElecPackInfoConfig->cellModuleVoltages,
                   sizeof(mail->cell_voltages));
            m_mailCellVoltageBox.put(mail);
          }
        }
      }
    }

    // 6) Watchdog cross-check: ensure op thread loop timestamp is fresh
    if (s_wdt_crosscheck_enable)
    {
      const uint32_t now_ms = (uint32_t)chrono::duration_cast<chrono::milliseconds>(
                                  Kernel::Clock::now().time_since_epoch())
                                  .count();
      if (g_opLoopTimestampMs != 0)
      {
        const uint32_t age_ms = now_ms - g_opLoopTimestampMs;
        if (age_ms > 1500U && timerDelay1ms(&wdt_crosscheck_last_log, 1000))
        {
          m_hwMeasurement.interfaceComm.printToInterface(
              "[WDT-XCHK] op loop stale: age=%lu ms\r\n", (unsigned long)age_ms);
          sdCard.logEvent("WDT-XCHK op loop stale age=%lums", (unsigned long)age_ms);
        }
      }
    }

    // 5) Charge detection / balance mode window
    int32_t charge_threshold = (int32_t)m_ptrGenConfig->u16ChargerEnabledThreshold;
    if (m_commissioningActive && charge_threshold > 500)
    {
      charge_threshold = 500;
    }
    bool is_charging =
        (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent <= -charge_threshold);

    if (is_charging && !m_ptrPowerElecPackInfoConfig->bChargeCurrentDetected)
    {
      if (timerDelay1ms(&m_u32ChargeCurrentDetectionLastTick, 5000))
      {
        m_ptrPowerElecPackInfoConfig->bChargeCurrentDetected = true;
        m_ptrPowerElecPackInfoConfig->bChargeBalanceActive = true;
        m_u32BalanceModeActiveLastTick = MEASUREMENT_GET_TICK(m_measurementManagerTim);
      }
    }
    else if (!is_charging)
    {
      m_ptrPowerElecPackInfoConfig->bChargeCurrentDetected = false;
      m_ptrPowerElecPackInfoConfig->bChargeBalanceActive = false;
      m_u32ChargeCurrentDetectionLastTick = MEASUREMENT_GET_TICK(m_measurementManagerTim);
    }

    if (m_ptrPowerElecPackInfoConfig->bChargeCurrentDetected &&
        timerDelay1ms(&m_u32BalanceModeActiveLastTick, 3 * 60 * 1000))
    {
      m_ptrPowerElecPackInfoConfig->bChargeBalanceActive = false;
    }

    // Heartbeat + pacing
    hb_set(HB_MEAS);
    ThisThread::sleep_for(10ms);
  }
}

// NOTE: Legacy measurementThread() implementation removed during cleanup (2026-01-14)
// The old commented-out code (~115 lines) was replaced by the active implementation above.

// #############################################################################################################################################################################
void Measurement::subTaskRecalibrateSoc(void)
{
  // -------------------- Tunables --------------------
  const uint32_t REST_STABLE_MS = MeasurementArgs::SOC_RECAL_REST_STABLE_MS;
  const float OCV_CONFIDENCE_MIN = MeasurementArgs::SOC_RECAL_OCV_CONFIDENCE_MIN;
  const float THROUGHPUT_FRACTION = MeasurementArgs::SOC_RECAL_THROUGHPUT_FRACTION;
  const uint32_t RECAL_COOLDOWN_MS = MeasurementArgs::SOC_RECAL_COOLDOWN_MS;

  // -------------------- Time base -------------------
  const uint32_t now = MEASUREMENT_GET_TICK(m_measurementManagerTim);
  static uint32_t last_tick = now;
  const uint32_t dt_ms = now - last_tick;
  last_tick = now;

  // -------------------- Accumulate throughput (|Ah|) since last recal -------------------
  static float ah_since_recal = 0.0f;
  {
    const float I_A = m_ptrPowerElecPackInfoConfig->i32ModuleCurrent / 1000.0f;
    ah_since_recal += fabsf(I_A) * (dt_ms / 3600000.0f); // A * h
  }

  // -------------------- Cooldown since last recal -------------------
  static uint32_t last_recal_ms = 0;
  const bool cooldown_ok = (now - last_recal_ms) >= RECAL_COOLDOWN_MS;

  // -------------------- Idle & voltage stability -------------------
  // Use state machine to detect IDLE (centralized policy)
  const bool idle = (m_ptrPowerElecPackInfoConfig->bmsState == BMS_IDLE);
  // Voltage stability: Vmin hasn’t moved more than a small threshold for REST_STABLE_MS
  static uint16_t last_vmin = 0;
  static uint32_t v_stable_ms = 0;
  {
    const uint16_t vnow = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
    const uint16_t dv = (vnow > last_vmin) ? (vnow - last_vmin) : (last_vmin - vnow);
    const uint16_t STABLE_THRESH_mV = MeasurementArgs::SOC_VOLTAGE_STABLE_MV;
    if (dv <= STABLE_THRESH_mV)
    {
      if (v_stable_ms < REST_STABLE_MS)
        v_stable_ms += dt_ms;
    }
    else
    {
      last_vmin = vnow;
      v_stable_ms = 0;
    }
  }
  const bool voltage_stable = (v_stable_ms >= REST_STABLE_MS);

  // -------------------- OCV confidence (avoid mid-plateau) -------------------
  const float ocv_confidence = ocv_confidence_from_voltage(
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin,
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax);
  // Continuously update the SOC confidence in pack info for display/logging
  m_ptrPowerElecPackInfoConfig->u8SocConfidence = static_cast<uint8_t>(ocv_confidence * 100.0f);
  const bool ocv_confident = (ocv_confidence >= OCV_CONFIDENCE_MIN);

  // -------------------- Throughput gating -------------------
  const float cap_Ah = static_cast<float>(m_ptrGenConfig->u16BatteryCapacity);
  const bool throughput_ok = (ah_since_recal >= (THROUGHPUT_FRACTION * cap_Ah));

  // -------------------- Decide if we can recal -------------------
  if (idle && voltage_stable && ocv_confident && throughput_ok && cooldown_ok)
  {
    // We’re allowed to recalibrate: apply OCV immediately and reset gates.
    m_bInitialSocSet = false;   // allow the setter to run
    subTaskSetInitialSoc(true); // bypass sanity gate by policy here
    {
      const uint16_t vmin_now = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
      const uint16_t vmax_now = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax;
      const uint16_t dv_now = getDvMvCleanAware();
      const uint16_t soc_x10 = m_ptrPowerElecPackInfoConfig->u16ModuleSoc;
      m_hwMeasurement.interfaceComm.printToInterface(
          "[SOC] Recal OCV applied: %u.%u%% (Vmin=%u mV, Vmax=%u mV, dv=%u mV)",
          (unsigned)(soc_x10 / 10), (unsigned)(soc_x10 % 10),
          (unsigned)vmin_now, (unsigned)vmax_now, (unsigned)dv_now);
    }
  last_recal_ms = now;
  ah_since_recal = 0.0f;
  v_stable_ms = 0; // require new stability before next attempt
}
  // Top-off latch — set SOC to 100% when you truly finish a charge.
  // Signature: CHARGE state, Vmax near soft-OVP, small dV, and charge current in taper for a dwell.
  {
    const enBmsState bms = m_ptrPowerElecPackInfoConfig->bmsState;
    const uint16_t vmax = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax; // mV
    const uint16_t dv_mV = getDvMvCleanAware();
    const uint16_t soft_mV = m_ptrGenConfig->u16SoftOverVoltage;
    const uint16_t HYST_mV = 30; // reuse common hysteresis notion
    const uint16_t top_thresh_mV = (soft_mV > HYST_mV) ? (uint16_t)(soft_mV - HYST_mV) : soft_mV;
    // Taper current threshold (negative while charging): default C/20; clamp to at least 200 mA magnitude
    const float cap_Ah = static_cast<float>(m_ptrGenConfig->u16BatteryCapacity);
    const int TAPER_mA = -(int)std::max(200.0f, 0.05f * cap_Ah * 1000.0f);
    static uint32_t eoc_dwell_ms = 0;
    const bool cond = (bms == BMS_CHARGE) &&
                      (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent >= TAPER_mA) &&
                      (vmax >= top_thresh_mV) &&
                      (dv_mV <= 40u);
    if (cond)
    {
      eoc_dwell_ms += dt_ms;
      // Require 10 minutes in taper at the top with small dV
      if (eoc_dwell_ms >= 10u * 60u * 1000u)
      {
        // Snap to 100% SOC and persist; reset throughput accumulator
        m_fRemainingCapacityAh = cap_Ah;
        m_ptrPowerElecPackInfoConfig->u16ModuleSoc = 1000; // x0.1% units
        m_bInitialSocSet = true;
        m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest = true; // handled elsewhere
        last_recal_ms = now;
        ah_since_recal = 0.0f;
        eoc_dwell_ms = 0;
        m_hwMeasurement.interfaceComm.printToInterface(
            "[SOC] Full-charge detected: set SOC=100%% (Vmax=%u mV, dV=%u mV)\r\n",
            (unsigned)vmax, (unsigned)dv_mV);
      }
    }
    else
    {
      eoc_dwell_ms = 0;
    }
  }

  // SOC Dwell Rounding (for Inverter ONLY): If SOC stays >= 99.4% for 10 minutes,
  // report 100% to inverter but keep internal SOC unchanged
  {
    static uint32_t soc_high_dwell_ms = 0;
    const uint16_t socNow = m_ptrPowerElecPackInfoConfig->u16ModuleSoc;
    
    if (socNow >= 994 && socNow < 1000) // 99.4% to 99.9%
    {
      soc_high_dwell_ms += dt_ms;
      if (soc_high_dwell_ms >= 10u * 60u * 1000u) // 10 minutes
      {
        if (!m_bSocDwellReportAs100)
        {
          m_bSocDwellReportAs100 = true;
          m_hwMeasurement.interfaceComm.printToInterface(
              "[SOC] Dwell: Held >= 99.4%% for 10min, reporting 100%% to inverter\r\n");
        }
      }
    }
    else if (socNow >= 1000)
    {
      // Already 100%, flag can be true (doesn't matter)
      m_bSocDwellReportAs100 = true;
      soc_high_dwell_ms = 0;
    }
    else
    {
      // SOC dropped below 99.4%, reset flag and timer
      m_bSocDwellReportAs100 = false;
      soc_high_dwell_ms = 0;
    }
  }

  // #########################

  // const uint32_t IDLE_TIME_FOR_RECALIBRATION_MS = 10 * 60 * 1000; // 10 minutes

  // // Check if the BMS is in an idle state
  // if (m_ptrPowerElecPackInfoConfig->bmsState == BMS_IDLE)
  // {
  //   // If we have been idle for long enough, recalibrate the SOC.
  //   if (timerDelay1ms(&m_u32IdleStateEntryTimestamp, IDLE_TIME_FOR_RECALIBRATION_MS))
  //   {
  //     m_hwMeasurement.serial.serialPrint("--- Recalibrating SOC based on OCV ---\r\n");
  //     // By setting m_bInitialSocSet to false, we trigger a new OCV-based estimation.
  //     m_bInitialSocSet = false;
  //     subTaskSetInitialSoc(true); // Force recalibration using OCV

  //     // Reset the timestamp to prevent continuous recalibration.
  //     m_u32IdleStateEntryTimestamp = MEASUREMENT_GET_TICK(m_measurementManagerTim);
  //   }
  // }
  // else
  // {
  //   // If we are not idle, reset the idle timer.
  //   m_u32IdleStateEntryTimestamp = MEASUREMENT_GET_TICK(m_measurementManagerTim);
  // }
}
// #############################################################################################################################################################################
void Measurement::subTaskHandleSocSaveRequest(void)
{
  if (m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest)
  {
    static uint32_t s_lastAttemptTick = 0;
    if (!timerDelay1ms(&s_lastAttemptTick, 50))
    {
      return; // throttle retries to avoid I2C hammering
    }
    static uint8_t s_failCount = 0; // track consecutive failures to avoid log spam

    EepromM24C24Args::SocJournalRecord rec = {};
    rec.soc_mpermil = m_ptrPowerElecPackInfoConfig->u16ModuleSoc;
    rec.remaining_mAh = static_cast<int32_t>(m_fRemainingCapacityAh * 1000.0f);
    rec.vmin_mV = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
    rec.current_mA = m_ptrPowerElecPackInfoConfig->i32ModuleCurrent;
    rec.temp_mC = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureAverage;
    rec.ocv_mV = 0;
    rec.timestamp_ms = MEASUREMENT_GET_TICK(m_measurementManagerTim);

    // === NEW: EEPROM/Journal SOC Save Logging ===
    uint16_t soc = rec.soc_mpermil;
    int32_t remaining_mAh = rec.remaining_mAh;
    m_hwMeasurement.interfaceComm.printToInterface(
        "[EEP] SOC saved: %u.%u%% (remaining=%ld.%01uAh)\r\n",
        (unsigned)(soc / 10), (unsigned)(soc % 10),
        (long)(remaining_mAh / 1000), (unsigned)((abs(remaining_mAh) % 1000) / 100));

    const bool ok_journal = m_hwMeasurement.eeprom.appendSocJournal(rec);

    // Allow EEPROM write cycle to complete (typically 5-10ms)
    ThisThread::sleep_for(10ms);

    // ALWAYS update the legacy slot for redundancy/boot-loading
    // (This ensures that even if journal is full/fails/ignored, we have the last value)
    uint32_t remaining_capacity_mah = (uint32_t)(m_fRemainingCapacityAh * 1000.0f);
    uint16_t capacity_l = (uint16_t)(remaining_capacity_mah & 0xFFFF);
    uint16_t capacity_h = (uint16_t)(remaining_capacity_mah >> 16);
    m_hwMeasurement.eeprom.setSettingValue(CMD_EE_ADDR_LastRemainingCapacity_L, capacity_l);
    ThisThread::sleep_for(10ms); // Wait for next write
    m_hwMeasurement.eeprom.setSettingValue(CMD_EE_ADDR_LastRemainingCapacity_H, capacity_h);
    ThisThread::sleep_for(10ms); // Wait for final write

    // Verify legacy write by reading back
    const uint16_t rd_l = m_hwMeasurement.eeprom.getSettingValue(CMD_EE_ADDR_LastRemainingCapacity_L);
    const uint16_t rd_h = m_hwMeasurement.eeprom.getSettingValue(CMD_EE_ADDR_LastRemainingCapacity_H);
    const uint32_t rd_mah = ((uint32_t)rd_h << 16) | rd_l;
    const bool ok_legacy = (rd_mah == remaining_capacity_mah);

    if (ok_journal && ok_legacy)
    {
      s_failCount = 0;
      m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest = false;
    }
    else
    {
      ++s_failCount;
      // Keep request flagged so we retry next loop; log once per failure streak
      if (s_failCount == 1 || (s_failCount % 10) == 0)
      {
        m_hwMeasurement.interfaceComm.printToInterface(
            "[EEP-ERR] SOC persist failed (journal=%d legacy=%d rd=%lu mAh exp=%lu mAh)\r\n",
            (int)ok_journal, (int)ok_legacy,
            (unsigned long)rd_mah, (unsigned long)remaining_capacity_mah);
      }
    }
  }
}
// #############################################################################################################################################################################

void Measurement::subTaskVoltageProtectionWatch(void)
{

  static uint32_t _u32CellProtectionIntervalLastTick = 0;
  static constexpr uint32_t PROTECTION_CHECK_INTERVAL_MS = 100;
  static bool lastDischargeAllowed = false;
  static bool lastChargeAllowed = false;

  // Guard: skip protection state updates until we have valid cell data
  if (m_ptrPowerElecPackInfoConfig->u32LastBqDataTimestamp == 0 ||
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin == 0)
  {
    return;
  }

  if (timerDelay1ms(&_u32CellProtectionIntervalLastTick, PROTECTION_CHECK_INTERVAL_MS))
  {
    apply_soft_uv_throttle();
    m_hwMeasurement.bq.getHardOverTemperatureFlags(&m_u16HardOverTemperatureFlags);

    // Guard: Do NOT overwrite critical faults with lower-priority states
    if (m_ptrPowerElecPackInfoConfig->packOperationCellState != PACK_STATE_ERROR_HARD_CELLVOLTAGE &&
        m_ptrPowerElecPackInfoConfig->packOperationCellState != PACK_STATE_ERROR_OVER_TEMPERATURE &&
        m_ptrPowerElecPackInfoConfig->packOperationCellState != PACK_STATE_ERROR_UNDER_TEMPERATURE &&
        m_ptrPowerElecPackInfoConfig->packOperationCellState != PACK_STATE_CRITICAL_TEMPERATURE)
    {
      enPowerElecPackOperationCellState newState = PACK_STATE_NORMAL;

      const uint16_t vmin_mV = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;

      if (vmin_mV <= m_ptrGenConfig->u16HardUnderVoltage)
      {
        newState = PACK_STATE_ERROR_HARD_CELLVOLTAGE;
      }
      // High-priority short-circuit check.
      else if (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent >= (m_ptrGenConfig->u16MaxHardDchgAllowedCurrent * 20))  // 100 amp error generation
      {
        newState = PACK_STATE_ERROR_SHORT_CURRENT;
        m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed = false;
        m_ptrPowerElecPackInfoConfig->bDischargeHighCurrentAllowed = false;
      }
      else if (m_ptrPowerElecPackInfoConfig->moduleProtectionParams[MODULE_OVER_CURRENT_DISCHARGE]
                   .u8ProtectionDetectionCounter >= ProtectionConstants::MODULE_DCHG_OC_DETECTION_COUNTER)
      {
        newState = PACK_STATE_ERROR_OVER_CURRENT;
      }
      else if (m_ptrPowerElecPackInfoConfig->moduleProtectionParams[MODULE_OVER_CURRENT_CHARGE]
                   .u8ProtectionDetectionCounter >= ProtectionConstants::MODULE_CHG_OC_DETECTION_COUNTER)
      {
        newState = PACK_STATE_ERROR_CHARGE_OVER_CURRENT;
      }
      else if (m_u16HardOverTemperatureFlags)
      {
        // Over-temp: do not open contactors; only clamp discharge to zero.
        newState = PACK_STATE_NORMAL;
        clampInverterDischargeZero("OTP_HW_FLAG");
      }
      else if (!(m_ptrPowerElecPackInfoConfig->bChargeAllowed &&
                 m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed))
      {
        newState = PACK_STATE_ERROR_SOFT_CELLVOLTAGE;
      }

      m_ptrPowerElecPackInfoConfig->packOperationCellState = newState;
    }

    if ((lastChargeAllowed != m_ptrPowerElecPackInfoConfig->bChargeAllowed) ||
        (lastDischargeAllowed != m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed))
    {
      lastChargeAllowed = m_ptrPowerElecPackInfoConfig->bChargeAllowed;
      lastDischargeAllowed = m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed;
      m_hwMeasurement.pwr.udpateSwitches();
    }
  }
}

// #############################################################################################################################################################################

void Measurement::subTaskCellOperationStateWatch(void)
{

  // Guard Clause: Do NOT overwrite Temperature Faults with "Normal" or "Soft Voltage Error"
  // This allows the Hard Temp protection logic (running earlier) to persist its state.
  if (m_ptrPowerElecPackInfoConfig->packOperationCellState == PACK_STATE_ERROR_OVER_TEMPERATURE || 
      m_ptrPowerElecPackInfoConfig->packOperationCellState == PACK_STATE_ERROR_UNDER_TEMPERATURE ||
      m_ptrPowerElecPackInfoConfig->packOperationCellState == PACK_STATE_CRITICAL_TEMPERATURE)
  {
      return; 
  }

  if (m_ptrPowerElecPackInfoConfig->bChargeAllowed && m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed)
    m_ptrPowerElecPackInfoConfig->packOperationCellState = PACK_STATE_NORMAL;
  else
    m_ptrPowerElecPackInfoConfig->packOperationCellState = PACK_STATE_ERROR_SOFT_CELLVOLTAGE;
}

// #############################################################################################################################################################################

void Measurement::subTaskCellVoltagesProtectionController(void)
{

  static bool lastdisChargeLCAllowed = false;
  static bool lastChargeAllowed = false;

  // update outputs directly if needed
  if ((lastChargeAllowed != m_ptrPowerElecPackInfoConfig->bChargeAllowed) || (lastdisChargeLCAllowed != m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed))
  {
    lastChargeAllowed = m_ptrPowerElecPackInfoConfig->bChargeAllowed;
    lastdisChargeLCAllowed = m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed;
    m_hwMeasurement.pwr.udpateSwitches();
  }
}

// #############################################################################################################################################################################
// I am gonna modify this section to incorparete the current management mechanism,
// making it simpler, easier and more understandable
void Measurement::subTaskCurrentWatch(void)
{
  // Add this new block at the beginning of the function
  // High-priority check for short-circuit condition.
  if (g_ptrPowerElecPackInfoConfig->i32ModuleCurrent >= (g_ptrGenConfig->u16MaxHardDchgAllowedCurrent * 20))
  {
    g_ptrPowerElecPackInfoConfig->packOperationCellState = PACK_STATE_ERROR_SHORT_CURRENT;
    g_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed = false;
    g_ptrPowerElecPackInfoConfig->bDischargeHighCurrentAllowed = false;
    return; // Exit immediately to ensure fastest response
  }
  // End of new block
  if (m_ptrPowerElecPackInfoConfig->packOperationCellState != PACK_STATE_ERROR_OVER_CURRENT)
  {
  }

  if (m_ptrPowerElecPackInfoConfig->moduleProtectionParams[MODULE_OVER_CURRENT_DISCHARGE].u8ProtectionDetectionCounter >= ProtectionConstants::MODULE_DCHG_OC_DETECTION_COUNTER)
  {
    m_ptrPowerElecPackInfoConfig->packOperationCellState = PACK_STATE_ERROR_OVER_CURRENT;
    // Implementation Discharging OverCurrent
  }

  if (m_ptrPowerElecPackInfoConfig->moduleProtectionParams[MODULE_OVER_CURRENT_CHARGE].u8ProtectionDetectionCounter >= ProtectionConstants::MODULE_CHG_OC_DETECTION_COUNTER)
  {
    m_ptrPowerElecPackInfoConfig->packOperationCellState = PACK_STATE_ERROR_CHARGE_OVER_CURRENT;
    // Implementation Charging OverCurrent
  }
}

// #############################################################################################################################################################################

void Measurement::subTaskTemperatureWatch(void)
{
  static bool lastdisChargeLCAllowed = false;
  static bool lastChargeAllowed = false;
  static bool s_hwOtClamped = false;
  static uint32_t s_hardOTDetectStart = 0;
  static bool s_hardOTActive = false;
  static uint32_t s_hardUTDetectStart = 0;
  static bool s_hardUTActive = false;

  m_hwMeasurement.bq.getHardOverTemperatureFlags(&m_u16HardOverTemperatureFlags);
  const bool hwOtp = (m_u16HardOverTemperatureFlags != 0);
  if (hwOtp)
  {
    // Over-temp: do not open contactors; only clamp discharge to zero.
    clampInverterDischargeZero("OTP_HW_FLAG");
    s_hwOtClamped = true;
  }
  else if (s_hwOtClamped && !s_hardOTActive)
  {
    restoreInverterDischargeMax("OTP_HW_RELEASE");
    s_hwOtClamped = false;
  }

  // Hard Over or Under Voltage Error Handling - Handle hard cell voltage limits
  if (m_u16HardOverTemperatureFlags)
  {
    // Over-temp: do not open contactors; only clamp discharge to zero.
    clampInverterDischargeZero("OTP_HW_FLAG");
  }

  // === NEW: Temperature Trend Warning ===
  {
    static bool s_tempWarnLogged = false;
    int32_t tMax = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax;
    int32_t limitChg = (int32_t)(m_ptrGenConfig->u16AllowedTempBattChargingMax * 100); // mC
    int32_t margin = limitChg - tMax;
    if (margin > 0 && margin <= 500 && !s_tempWarnLogged) // within 5°C of limit
    {
      m_hwMeasurement.interfaceComm.printToInterface(
          "[TEMP-WARN] Tmax=%ld.%01uC approaching %u.0C charge limit\r\n",
          (long)(tMax / 100), (unsigned)((abs(tMax) % 100) / 10),
          (unsigned)m_ptrGenConfig->u16AllowedTempBattChargingMax);
      s_tempWarnLogged = true;
    }
    else if (margin > 500)
    {
      s_tempWarnLogged = false;
    }
  }

  // === CRITICAL: Absolute Hard Over-Temperature and Under-Temperature Protection ===
  // This protection triggers regardless of BMS state and DISCONNECTS CONTACTORS
  {
    const int32_t tMax = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax;
    const int32_t tMin = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin;
    
    // Limits
    const int32_t limitMax_mC = (int32_t)m_ptrGenConfig->u16AllowedTempBattChargingMax * 100;
    const int32_t limitCritical_mC = 200000; // 200°C Absolute Max
    const int32_t limitMin_mC = -20000; // -20°C
    const int32_t hystOT_mC = (int32_t)m_ptrGenConfig->u16HysteresisOverTemperature * 100;
    const int32_t hystUT_mC = hystOT_mC;

    // Debug Logging (rate-limited to reduce spam)
    static uint32_t s_lastTempDbgMs = 0;
    uint32_t nowDbg = duration_cast<milliseconds>(m_measurementManagerTim.elapsed_time()).count();
    const uint32_t tempDbgIntervalMs = (s_hardOTActive || s_hardUTActive) ? 5000u : 30000u;
    if ((nowDbg - s_lastTempDbgMs) >= tempDbgIntervalMs)
    {
      s_lastTempDbgMs = nowDbg;
      m_hwMeasurement.interfaceComm.printToInterface(
          "[TEMP-DBG] Tmax=%ld mC, Tmin=%ld mC, OT_Active=%d, UT_Active=%d\r\n",
          (long)tMax, (long)tMin, (int)s_hardOTActive, (int)s_hardUTActive);
    }

    // Over-Temperature Logic
    if (tMax >= limitMax_mC)
    {
      if (s_hardOTDetectStart == 0) 
      {
          s_hardOTDetectStart = duration_cast<milliseconds>(m_measurementManagerTim.elapsed_time()).count();
      }
      else if (!s_hardOTActive)
      {
        if ((duration_cast<milliseconds>(m_measurementManagerTim.elapsed_time()).count() - s_hardOTDetectStart) >= 3000)
        {
          s_hardOTActive = true;
          // Differential Action based on Temperature Severity
          if (tMax >= limitCritical_mC)
          {
             m_hwMeasurement.interfaceComm.printToInterface("[TEMP-CRITICAL] >100C! Tmax=%ld mC\r\n", (long)tMax);
          }
          else
          {
             m_hwMeasurement.interfaceComm.printToInterface("[TEMP-CRITICAL] >60C! Hard Over-Temp Triggered! Tmax=%ld mC\r\n", (long)tMax);
          }

          // Over-temp: do not open contactors; only clamp discharge to zero.
          clampInverterDischargeZero("OTP_HARD");
        }
      }
      else if (s_hardOTActive)
      {
           // Update log if temp rises from 60C to 100C while already active
           if (tMax >= limitCritical_mC)
           {
               m_hwMeasurement.interfaceComm.printToInterface("[TEMP-CRITICAL] Escalated to >100C! Tmax=%ld mC\r\n", (long)tMax);
           }
      }
    }
    else
    {
      s_hardOTDetectStart = 0;
    }

    // Under-Temperature Logic
    if (tMin <= limitMin_mC)
    {
      if (s_hardUTDetectStart == 0) 
      {
          s_hardUTDetectStart = duration_cast<milliseconds>(m_measurementManagerTim.elapsed_time()).count();
      }
      else if (!s_hardUTActive)
      {
        if ((duration_cast<milliseconds>(m_measurementManagerTim.elapsed_time()).count() - s_hardUTDetectStart) >= 3000)
        {
          s_hardUTActive = true;
          m_hwMeasurement.interfaceComm.printToInterface("[TEMP-CRITICAL] Hard Under-Temp Triggered! Tmin=%ld mC\r\n", (long)tMin);
        }
      }
    }
    else
    {
      s_hardUTDetectStart = 0;
    }

    // Release Condition
    if (s_hardOTActive || s_hardUTActive)
    {
      bool otReleased = !s_hardOTActive || (tMax <= (limitMax_mC - hystOT_mC));
      bool utReleased = !s_hardUTActive || (tMin > (limitMin_mC + hystUT_mC));

      if (otReleased && s_hardOTActive)
      { 
          s_hardOTActive = false; 
          m_hwMeasurement.interfaceComm.printToInterface("[TEMP-INFO] OT Released\r\n"); 
          restoreInverterDischargeMax("OTP_RELEASE");
          s_hwOtClamped = false;
      }
      if (utReleased && s_hardUTActive) 
      { 
          s_hardUTActive = false; 
          m_hwMeasurement.interfaceComm.printToInterface("[TEMP-INFO] UT Released\r\n"); 
      }

      if (!s_hardOTActive && !s_hardUTActive)
      {
          m_hwMeasurement.interfaceComm.printToInterface("[TEMP-INFO] All Temp Protections Released.\r\n");
      }
    }
  }

  // update outputs directly if needed
  if ((lastChargeAllowed != m_ptrPowerElecPackInfoConfig->bChargeAllowed) || (lastdisChargeLCAllowed != m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed))
  {
    lastChargeAllowed = m_ptrPowerElecPackInfoConfig->bChargeAllowed;
    lastdisChargeLCAllowed = m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed;
    m_hwMeasurement.pwr.udpateSwitches();
  }
}

// #############################################################################################################################################################################

void Measurement::subTaskCheckPackSOA(void)
{

  static int16_t hysteresysDischarge = -2000;
  static int16_t hysteresysCharge = -2000;

  static bool lastPackInSOACharge = true;
  static bool lastPackInSOADisCharge = true;

  bool _bOutsideLimitsDischarge = false;
  bool _bOutsideLimitsCharge = false;

  // Check Battery Limits Discharge
  if (m_ptrPowerElecPackInfoConfig->bTempEnableBatteryDisCharge)
  {
    _bOutsideLimitsDischarge |= (m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax > ((m_ptrGenConfig->u16AllowedTempBattDischargingMax * 100) + hysteresysDischarge)) ? true : false;
    _bOutsideLimitsDischarge |= (m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin < -((m_ptrGenConfig->u16AllowedTempBattDischargingMin * 100) - hysteresysDischarge)) ? true : false;

    if (_bOutsideLimitsDischarge)
    {
      hysteresysDischarge = -2000;
    }
    else
      hysteresysDischarge = 2000;
  }

  // Check Battery Limits Charge
  if (m_ptrPowerElecPackInfoConfig->bTempEnableBatteryCharge)
  {
    _bOutsideLimitsCharge |= (m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax > ((m_ptrGenConfig->u16AllowedTempBattChargingMax * 100) + hysteresysCharge)) ? true : false;
    _bOutsideLimitsCharge |= (m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin < -((m_ptrGenConfig->u16AllowedTempBattChargingMin * 100) - hysteresysCharge)) ? true : false;

    if (_bOutsideLimitsCharge)
    {
      hysteresysCharge = -2000;
    }
    else
      hysteresysCharge = 2000;
  }

  // DisCharge delayed response
  if (lastPackInSOADisCharge != !(_bOutsideLimitsDischarge))
  {
    if (timerDelay1ms(&m_u32SOADisChargeChangeLastTick, 1000))
    {
      lastPackInSOADisCharge = m_ptrPowerElecPackInfoConfig->bPackInSOADischarge = !(_bOutsideLimitsDischarge);
    }
  }
  else
  {
    m_u32SOADisChargeChangeLastTick = MEASUREMENT_GET_TICK(m_measurementManagerTim);
  }

  // Charge delayed response
  if (lastPackInSOACharge != !(_bOutsideLimitsCharge))
  {
    if (timerDelay1ms(&m_u32SOAChargeChangeLastTick, 1000))
    {
      lastPackInSOACharge = m_ptrPowerElecPackInfoConfig->bPackInSOACharge = !(_bOutsideLimitsCharge);
    }
  }
  else
  {
    m_u32SOAChargeChangeLastTick = MEASUREMENT_GET_TICK(m_measurementManagerTim);
  }
}

// #############################################################################################################################################################################

void Measurement::subTaskCheckWarning(void)
{

  static uint32_t lastErrorReset;
  static bool bFirstErrorCleanFlags = true;

  int32_t _i32MaxChargeCurrent = 0;
  int32_t _i32MaxDisChargeCurrent = 0;
  enBatteryErrors newerror = SYS_OK;

  if ((m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin > 24000) && (m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax < 50000))
  {
    _i32MaxChargeCurrent = (int32_t)(0.7f * (float)(m_ptrGenConfig->u16BatteryCapacity * 1000));

  }

  else if ((m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin > 0) && (m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax < 24000))
  {
    _i32MaxChargeCurrent = (int32_t)(0.3f * (float)(m_ptrGenConfig->u16BatteryCapacity * 1000));
  }

  if ((m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin > -30000) && (m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax < -20000))
  {
    _i32MaxDisChargeCurrent = (int32_t)(0.2f * (float)m_ptrGenConfig->u16BatteryCapacity * 1000);
  }

  else if ((m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin > -20000) && (m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax < 5000))
  {
    _i32MaxDisChargeCurrent = (int32_t)(0.3f * (float)m_ptrGenConfig->u16BatteryCapacity * 1000);
  }

  else if ((m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin > 5000) && (m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax < 60000))
  {
    _i32MaxDisChargeCurrent = (int32_t)(1.5f * (float)m_ptrGenConfig->u16BatteryCapacity * 1000);
  }

  else if ((m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax > 60000))
  {
    _i32MaxDisChargeCurrent = (int32_t)(0.5f * (float)m_ptrGenConfig->u16BatteryCapacity * 1000);
  }

  if (m_ptrPowerElecPackInfoConfig->bAllowedErrorFlags)
  {
    inverterProtectWarning.sysProtect.bytes = 0;

    switch (m_ptrPowerElecPackInfoConfig->packOperationCellState)
    {
    case PACK_STATE_ERROR_OVER_CURRENT:
    {
      newerror = SYS_ERROR_DCHG_OVER_CURRENT;
      m_hwMeasurement.inverterCan.updateWarningAndProtect(SYS_ERROR_CHG_OVER_CURRENT);
      inverterProtectWarning.sysProtect.u8bitDisChargeHighCurrentArrive = 1;
    }
    break;
    case PACK_STATE_ERROR_CHARGE_OVER_CURRENT:
    {
      newerror = SYS_ERROR_CHG_OVER_CURRENT;
      m_hwMeasurement.inverterCan.updateWarningAndProtect(SYS_ERROR_CHG_OVER_CURRENT);
      inverterProtectWarning.sysProtect.u8bitChargeHighCurrentArrive = 1;
    }
    break;
    case PACK_STATE_ERROR_SHORT_CURRENT:
    {
      m_hwMeasurement.inverterCan.updateWarningAndProtect(SYS_ERROR_DCHG_OVER_CURRENT);
      newerror = SYS_ERROR_SHORT_CIRCUIT;
      inverterProtectWarning.sysProtect.u8bitShortCircuitArrive = 1;
    }
    break;

    case PACK_STATE_ERROR_SOFT_CELLVOLTAGE:
    {
      //newerror = SYS_ERROR_CELL_UNDER_VOLTAGE;
      inverterProtectWarning.sysProtect.u8bitLowVoltageArrive = 1;
      m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 2500);
      m_hwMeasurement.inverterCan.updateWarningAndProtect(SYS_ERROR_CELL_UNDER_VOLTAGE);  
    }
    break;

    case PACK_STATE_ERROR_HARD_CELLVOLTAGE:
    {
      newerror = SYS_ERROR_CELL_OVER_VOLTAGE;
      inverterProtectWarning.sysProtect.u8bitHighVoltageArrive = 1;
    }
    break;

    default:
      break;
    }

    if (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent >= 50000)
    {
      newerror = SYS_ERROR_DCHG_OVER_CURRENT;
      m_hwMeasurement.inverterCan.updateWarningAndProtect(SYS_ERROR_DCHG_OVER_CURRENT);
      //inverterProtectWarning.sysProtect.u8bitDisChargeHighCurrentArrive = 1;
    }
    if (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent <= -35000)
    {
      newerror = SYS_ERROR_DCHG_OVER_CURRENT;
      m_hwMeasurement.inverterCan.updateWarningAndProtect(SYS_ERROR_CHG_OVER_CURRENT);
      //inverterProtectWarning.sysProtect.u8bitDisChargeHighCurrentArrive = 1;
    }
    // Check temperature (fixed limits + hysteresis)
    {
      const int32_t tMax_mC = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax;
      const int32_t tMin_mC = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin;

      const int32_t otpLimit_mC = 60000;
      const int32_t otpRelease_mC = 55000;

      int32_t utpLimit_mC = -20000;
      int32_t utpRelease_mC = -15000;

      uint8_t mode = 2; // 0=charge,1=discharge,2=idle
      if (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent <= -100)
      {
        mode = 0;
        utpLimit_mC = 0;
        utpRelease_mC = 5000;
      }
      else if (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent >= 100)
      {
        mode = 1;
      }

      static bool otActive[3] = {};
      static bool utActive[3] = {};

      if (!otActive[mode])
      {
        if (tMax_mC >= otpLimit_mC)
          otActive[mode] = true;
      }
      else
      {
        if (tMax_mC <= otpRelease_mC)
          otActive[mode] = false;
      }

      if (!utActive[mode])
      {
        if (tMin_mC <= utpLimit_mC)
          utActive[mode] = true;
      }
      else
      {
        if (tMin_mC > utpRelease_mC)
          utActive[mode] = false;
      }

      if (otActive[mode])
      {
        newerror = (mode == 0) ? SYS_ERROR_CELL_OVER_TEMPERATURE_CHARGE
                               : (mode == 1) ? SYS_ERROR_CELL_OVER_TEMPERATURE_DISCHARGE
                                             : SYS_ERROR_CELL_OVER_TEMPERATURE_IDLE;
      }
      else if (utActive[mode])
      {
        newerror = (mode == 0) ? SYS_ERROR_CELL_UNDER_TEMPERATURE_CHARGE
                               : (mode == 1) ? SYS_ERROR_CELL_UNDER_TEMPERATURE_DISCHARGE
                                             : SYS_ERROR_CELL_UNDER_TEMPERATURE_IDLE;
      }
    }

    // Check cell voltage (guard against invalid/stale min)
    const bool vmin_valid = (m_ptrPowerElecPackInfoConfig->u32LastBqDataTimestamp != 0) &&
                            (m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin != 0);
    const uint16_t vmin_uv = (m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinClean != 0)
                                 ? m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinClean
                                 : m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
    if (m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax > m_ptrGenConfig->u16SoftOverVoltage)
    {
      //newerror = SYS_ERROR_CELL_OVER_VOLTAGE;//if its active inverter shows over voltage error at the and of the
      inverterProtectWarning.sysProtect.u8bitHighVoltageArrive = 1;
      //m_hwMeasurement.inverterCan.updateWarningAndProtect(SYS_ERROR_CELL_OVER_VOLTAGE);  
    }
    if (vmin_valid && (vmin_uv < m_ptrGenConfig->u16SoftLowCurrentUnderVoltage  || vmin_uv < m_ptrGenConfig->u16SoftHighCurrentUnderVoltage || vmin_uv < m_ptrGenConfig->u16HardUnderVoltage   ))
    {
      newerror = SYS_ERROR_CELL_UNDER_VOLTAGE;
      inverterProtectWarning.sysProtect.u8bitLowVoltageArrive = 1;
      if (m_ptrPowerElecPackInfoConfig->u16ModuleSoc <= 200)  
      {
        m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 2500);
      }

     // m_hwMeasurement.inverterCan.updateWarningAndProtect(SYS_ERROR_CELL_UNDER_VOLTAGE);  
    }

    if (m_hwMeasurement.bq.getInternalError() != NO_ERROR)
    {

      newerror = SYS_ERROR_CHIPSET;
      m_ptrPowerElecPackInfoConfig->chipsetErrorState = m_hwMeasurement.bq.getInternalError();
      inverterProtectWarning.sysProtect.u8bitInternalArrive = 1;
    }
    // NOTE: Legacy balance state imbalance reporting removed during cleanup (2026-01-14)
  }

  // reset error after 5 secondss
  if (newerror == SYS_OK)
  {
    const bool commFaultActive = (m_ptrPowerElecPackInfoConfig->packBatteryWarErrState == SYS_ERROR_CHIPSET_COMM);
    const bool commHold = commFaultActive && (s_bqCommFaultLatched || s_bqCommManualResetRequired);

    if (commHold)
    {
      lastErrorReset = MEASUREMENT_GET_TICK(m_measurementManagerTim); // keep latched until comm is healthy or manual reset
    }
    else if (timerDelay1ms(&lastErrorReset, 5000))
    {
      m_ptrPowerElecPackInfoConfig->packBatteryWarErrState = SYS_OK;
      m_ptrPowerElecPackInfoConfig->packLastErrorState = SYS_OK;
      inverterProtectWarning.sysProtect.bytes = 0;
      if (bFirstErrorCleanFlags)
      {
        bFirstErrorCleanFlags = false;
        m_ptrPowerElecPackInfoConfig->bAllowedErrorFlags = true;
      }
    }
  }
  else
  {
    lastErrorReset = MEASUREMENT_GET_TICK(m_measurementManagerTim);
    m_ptrPowerElecPackInfoConfig->packBatteryWarErrState = newerror;
  }

#if FEATURE_INVERTER_PROTECT_STATUS
  static enBatteryErrors s_lastInverterErr = SYS_OK;
  const enBatteryErrors currentErr = m_ptrPowerElecPackInfoConfig->packBatteryWarErrState;
  if (currentErr != s_lastInverterErr)
  {
    m_hwMeasurement.inverterCan.updateWarningAndProtect(currentErr);
    s_lastInverterErr = currentErr;
  }
#endif
}

// #############################################################################################################################################################################

void Measurement::subTaskPackStateDetermination(void)
{

  // Highlight: This entire function is replaced to use the calculated current in milliamps,
  // which is more reliable than using the raw ADC value.

  // A positive current means discharging
  if (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent > 500) // Discharging with a 500mA deadband //100
  {
    m_ptrPowerElecPackInfoConfig->bmsState = BMS_DISCHARGE;
    m_ptrPowerElecPackInfoConfig->bTempEnableBatteryDisCharge = true;
    m_ptrPowerElecPackInfoConfig->bTempEnableBatteryCharge = false;
  }
  // A negative current means charging
  else if (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent < -100) // Charging with a 100mA deadband
  {
    m_ptrPowerElecPackInfoConfig->bmsState = BMS_CHARGE;
    m_ptrPowerElecPackInfoConfig->bTempEnableBatteryCharge = true;
    m_ptrPowerElecPackInfoConfig->bTempEnableBatteryDisCharge = false;
  }
  // Otherwise, the battery is idle
  else
  {
    m_ptrPowerElecPackInfoConfig->bmsState = BMS_IDLE;
    m_ptrPowerElecPackInfoConfig->bTempEnableBatteryDisCharge = false;
    m_ptrPowerElecPackInfoConfig->bTempEnableBatteryCharge = false;
  }
}

// #############################################################################################################################################################################

void Measurement::subTaskSocCalculation(void)
{
  // Use the Measurement manager timer to get real elapsed ms
  static uint32_t _last_ms = 0;
  uint32_t now_ms = duration_cast<milliseconds>(m_measurementManagerTim.elapsed_time()).count(); // or your existing GET_TICK macro
  uint32_t dt_ms_u32 = (_last_ms == 0) ? 100u : (now_ms - _last_ms);
  _last_ms = now_ms;

  // Clamp dt to avoid huge jumps after long pauses (e.g., resume from stop)
  if (dt_ms_u32 < 10u)
    dt_ms_u32 = 10u;
  if (dt_ms_u32 > 1000u)
    dt_ms_u32 = 1000u;

  const float dt_ms = static_cast<float>(dt_ms_u32);
  // This function is now more accurate and uses a fixed time step.
  float stateOfCharge = 0.0f;
  // const float dt_ms = 100.0f; // This function is called every 100ms
  int32_t current_for_soc = m_ptrPowerElecPackInfoConfig->i32ModuleCurrent;

  // Auto-zero current offset after verified idle (noise floor + stable dwell).
  {
    static int32_t s_offset_mA = 0;
    static uint32_t s_idle_start_ms = 0;
    static int64_t s_idle_sum_mA = 0;
    static uint32_t s_idle_samples = 0;
    const int32_t kIdleCurrent_mA = MeasurementArgs::SOC_OFFSET_IDLE_CURRENT_MA;
    const uint32_t kIdleStable_ms = MeasurementArgs::SOC_OFFSET_IDLE_STABLE_MS;
    const int32_t kMaxOffset_mA = MeasurementArgs::SOC_OFFSET_MAX_MA;
    const bool idle_state = (m_ptrPowerElecPackInfoConfig->bmsState == BMS_IDLE);
    const bool quiet = (std::abs(current_for_soc) <= kIdleCurrent_mA);

    if (idle_state && quiet)
    {
      if (s_idle_start_ms == 0)
      {
        s_idle_start_ms = now_ms;
        s_idle_sum_mA = 0;
        s_idle_samples = 0;
      }
      s_idle_sum_mA += current_for_soc;
      s_idle_samples++;
      if ((now_ms - s_idle_start_ms >= kIdleStable_ms) && (s_idle_samples > 0))
      {
        int32_t avg = static_cast<int32_t>(s_idle_sum_mA / static_cast<int64_t>(s_idle_samples));
        if (avg > kMaxOffset_mA)
          avg = kMaxOffset_mA;
        if (avg < -kMaxOffset_mA)
          avg = -kMaxOffset_mA;
        s_offset_mA = avg;
        s_idle_start_ms = now_ms;
        s_idle_sum_mA = 0;
        s_idle_samples = 0;
      }
    }
    else
    {
      s_idle_start_ms = 0;
      s_idle_sum_mA = 0;
      s_idle_samples = 0;
    }

    current_for_soc -= s_offset_mA;
  }

  const float CHARGE_EFF = MeasurementArgs::SOC_CHARGE_EFF;
  const float DISCHARGE_EFF = MeasurementArgs::SOC_DISCHARGE_EFF;
  const float eff = (current_for_soc < 0) ? CHARGE_EFF : DISCHARGE_EFF;

  // Coulomb counting formula
  m_fRemainingCapacityAh -= (float)current_for_soc / 1000.0f * eff * (dt_ms / 3600000.0f);
  // Clamp the value to prevent it from going out of bounds.
  m_fRemainingCapacityAh = clamp(m_fRemainingCapacityAh, 0.0f, (float)m_ptrGenConfig->u16BatteryCapacity);

  if (m_ptrGenConfig->u16BatteryCapacity > 0)
  {
    stateOfCharge = (m_fRemainingCapacityAh / (float)m_ptrGenConfig->u16BatteryCapacity) * 1000.0f;
  }

  m_ptrPowerElecPackInfoConfig->u16ModuleSoc = (uint16_t)stateOfCharge;
  // m_ptrPowerElecPackInfoConfig->u16ModuleSoc = 500;

  // Persist SOC to EEPROM on every ±1% change (10 = 1.0% in x0.1% units)
  {
    static uint16_t s_lastSocSaved = 0;
    static bool s_socInit = false;

    const uint16_t socNow = m_ptrPowerElecPackInfoConfig->u16ModuleSoc;

    if (!s_socInit)
    {
      s_lastSocSaved = socNow; // seed with current SOC on first run
      s_socInit = true;
    }

    // Only raise a new request if one isn't already in flight
    if (!m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest)
    {
      const uint16_t delta =
          (socNow > s_lastSocSaved) ? (socNow - s_lastSocSaved) : (s_lastSocSaved - socNow);

      if (delta >= 10)
      {                                                               // 10 = 1.0%
        m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest = true; // handled by subTaskHandleSocSaveRequest()
        s_lastSocSaved = socNow;                                      // remember the last SOC we asked to persist
      }
    }
  }
}

// #############################################################################################################################################################################
void Measurement::updateBalancePhase(bool logTransitions)
{
  static constexpr uint32_t BALANCE_CYCLE_MEASURE_MS = 5000;   // 0-5s clean sample window
  static constexpr uint32_t BALANCE_CYCLE_ACTIVE_MS = 30000;   // 5-35s bleeders allowed
  static constexpr uint32_t BALANCE_CYCLE_QUIET_MS = 5000;     // 28-33s enforced off/stabilize
  static constexpr uint32_t BALANCE_FORCE_OFF_MS = BALANCE_CYCLE_MEASURE_MS + BALANCE_CYCLE_ACTIVE_MS;
  static constexpr uint32_t BALANCE_FORCE_OFF_GUARD_MS = 250;  // stop a hair early to avoid wrap jitter
  static constexpr uint32_t CYCLE_PERIOD = BALANCE_FORCE_OFF_MS + BALANCE_CYCLE_QUIET_MS;

  const uint32_t nowMs = timerNowMs();

  if (!m_balanceState.cycleInitialized)
  {
    m_balanceState.cycleStartMs = nowMs;
    m_balanceState.cycleInitialized = true;
  }

  uint32_t cycleTime = nowMs - m_balanceState.cycleStartMs;
  bool wrapped = false;
  const bool shouldLog = logTransitions && m_ptrPowerElecPackInfoConfig->bBalancingActive;

  if (cycleTime >= CYCLE_PERIOD)
  {
    m_balanceState.cycleStartMs = nowMs;
    cycleTime = 0;
    wrapped = true;
    m_balanceState.oddCycle = !m_balanceState.oddCycle;
    if (shouldLog)
    {
      m_hwMeasurement.interfaceComm.printToInterface("[BAL] cycle parity=%s\r\n",
                                                     m_balanceState.oddCycle ? "ODD" : "EVEN");
    }
  }

  const bool isMeasurePhase = (cycleTime < BALANCE_CYCLE_MEASURE_MS);
  const bool isActivePhase = (cycleTime >= BALANCE_CYCLE_MEASURE_MS) && (cycleTime < (BALANCE_FORCE_OFF_MS - BALANCE_FORCE_OFF_GUARD_MS));
  const bool isQuietStopPhase = (!isMeasurePhase && !isActivePhase);

  m_balanceState.cycleTimeMs = cycleTime;
  m_balanceState.isMeasurePhase = isMeasurePhase;
  m_balanceState.isActivePhase = isActivePhase;
  m_balanceState.isQuietStopPhase = isQuietStopPhase;

  // Treat quiet/measure as "clean" so other tasks can refresh min/max; active blocks reads.
  m_bBalanceMeasurePhase = !isActivePhase;

  if (shouldLog)
  {
    static int s_lastPhase = -1; // 0=measure, 1=active, 2=quiet
    const int phase = isMeasurePhase ? 0 : (isActivePhase ? 1 : 2);
    if (phase != s_lastPhase)
    {
      const char *label = isMeasurePhase ? "MEASURE phase (bleeders OFF)"
                                         : (isActivePhase ? "ACTIVE phase (bleeders ON)"
                                                          : "QUIET phase (bleeders OFF)");
      m_hwMeasurement.interfaceComm.printToInterface("[BAL] %s\r\n", label);
      s_lastPhase = phase;
    }
  }
}

void Measurement::updatePeriodicBalancePause(void)
{
#if DISABLE_BALANCING
  m_balanceState.periodicPauseActive = false;
  return;
#else
  const uint32_t nowMs = timerNowMs();
  const bool baseBalanceAllowed = m_balanceState.allowBalance && !m_balanceState.thermalPaused;
  bool periodicPauseActive = false;

  static uint32_t s_periodicStartMs = 0;
  static uint32_t s_pauseStartMs = 0;
  static bool s_inPause = false;

  if (baseBalanceAllowed)
  {
    if (s_periodicStartMs == 0)
      s_periodicStartMs = nowMs;
  }
  else
  {
    s_periodicStartMs = 0;
    s_pauseStartMs = 0;
    s_inPause = false;
  }

  if (s_periodicStartMs != 0 && !s_inPause &&
      (uint32_t)(nowMs - s_periodicStartMs) >= BalancingArgs::PERIODIC_BALANCE_PAUSE_MS)
  {
    s_inPause = true;
    s_pauseStartMs = nowMs;
    periodicPauseActive = true;
    m_hwMeasurement.interfaceComm.printToInterface(
        "[BAL] periodic pause start (%lu ms)\r\n",
        (unsigned long)BalancingArgs::PERIODIC_BALANCE_HOLD_MS);
  }
  else if (s_inPause)
  {
    periodicPauseActive = true;
    if ((uint32_t)(nowMs - s_pauseStartMs) >= BalancingArgs::PERIODIC_BALANCE_HOLD_MS)
    {
      s_inPause = false;
      s_pauseStartMs = 0;
      s_periodicStartMs = nowMs;
      periodicPauseActive = false;
      m_hwMeasurement.interfaceComm.printToInterface(
          "[BAL] periodic pause end, resume check\r\n");
    }
  }

  m_balanceState.periodicPauseActive = periodicPauseActive;
#endif
}

// ---------- Pre-balance + charge-throttle + per-cell map ----------
void Measurement::subTaskPackBalancing(void)
{
#if USE_LEGACY_BALANCING
  // Legacy balancing logic from commit 7c8688e3 (for regression comparison)
  // Disable new freeze behavior to match old flow.
  m_balanceState.freezeMeasurements = false;

  // Default to measure phase (allow min/max updates) in case we return early
  m_bBalanceMeasurePhase = true;
  
  // ---- Tunables (safe defaults) -----------------------------------------
  static constexpr uint32_t kIdleHold_ms = BalancingArgs::IDLE_HOLD_MS;
  static constexpr int32_t kChargeThresh_mA = BalancingArgs::CHARGE_THRESH_MA;
  static constexpr uint16_t kDV_START_mV = BalancingArgs::DV_START_MV;
  static constexpr uint16_t kDV_STOP_mV = BalancingArgs::DV_STOP_MV;
  static constexpr uint16_t kPACK_MISMATCH_ENTRY_mV = BalancingArgs::PACK_MISMATCH_ENTRY_MV;
  static constexpr uint16_t kPACK_MISMATCH_EXIT_mV = BalancingArgs::PACK_MISMATCH_EXIT_MV;
  static constexpr int16_t THROTTLE_X10MA = BalancingArgs::THROTTLE_X10MA;
  static constexpr uint16_t kCommDvFull_mV = 40;
  static constexpr uint16_t kCommDvLimit_mV = 80;
  static constexpr uint16_t kCommStopEnter_mV = 90;
  static constexpr uint16_t kCommStopExit_mV = 70;
  static constexpr uint16_t kCommDoneDv_mV = 30;
  static constexpr uint32_t kCommDoneHoldMs = 30u * 60u * 1000u;
  static constexpr uint32_t kCommTrendMs = 10u * 60u * 1000u;
  static constexpr uint16_t kCommTrendDrop_mV = 5;
  static constexpr uint16_t kCommCurrent1A = 100;  // 1.00 A in centi-amps
  static constexpr uint16_t kCommCurrent0_5A = 50; // 0.50 A in centi-amps

  // ---- Inputs -----------------------------------------------------------
  const uint32_t nowMs = timerNowMs();
  const int32_t I_mA = m_ptrPowerElecPackInfoConfig->i32ModuleCurrent;         // signed (neg = charge)
  
  // Cache clean readings: During MEASURE phase (bleeders OFF), store the readings.
  // During ACTIVE phase (bleeders ON), use the cached values to avoid IR drop corruption.
  static uint16_t s_cachedVmin = 0;
  static uint16_t s_cachedVmax = 0;
  
  uint16_t vmin, vmax;
  if (isBalanceMeasureWindow())
  {
    // MEASURE phase: Use live values and cache them
    vmin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
    vmax = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax;
    s_cachedVmin = vmin;
    s_cachedVmax = vmax;
    
    // Log clean readings periodically
    static uint32_t _u32CleanReadingLogTick = 0;
    if (timerDelay1ms(&_u32CleanReadingLogTick, 5000)) {
      m_hwMeasurement.interfaceComm.printToInterface(
        "[BAL-CLEAN] Vmin=%u mV, Vmax=%u mV, dV=%u mV\r\n", 
        (unsigned)vmin, (unsigned)vmax, (unsigned)(vmax - vmin));
    }
  }
  else
  {
    // ACTIVE phase: Use cached clean values
    vmin = s_cachedVmin;
    vmax = s_cachedVmax;
  }
  
  const uint16_t dv_mV = (vmax >= vmin) ? (uint16_t)(vmax - vmin) : 0;
  const uint16_t vmax_raw =
      (m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxRaw != 0)
          ? m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxRaw
          : vmax;

  // ---- dV Trend Watchdog ----------------------------------------------------
  static DvLockController s_dvLock;
  s_dvLock.update(nowMs,
                  dv_mV,
                  I_mA,
                  m_ptrPowerElecPackInfoConfig->bmsState,
                  kPACK_MISMATCH_ENTRY_mV,
                  kDV_STOP_mV,
                  m_hwMeasurement,
                  m_ptrGenConfig,
                  m_ptrPowerElecPackInfoConfig);

  // ---- Derived state ----------------------------------------------------
  const bool chargingNow = (I_mA < kChargeThresh_mA);
  const bool currentSmall = (std::abs(I_mA) <= (int32_t)BalancingArgs::MAX_CHARGE_CURRENT_MA);
  const enBmsState bms = m_ptrPowerElecPackInfoConfig->bmsState;
  const bool movingState = (bms == BMS_CHARGE) || (bms == BMS_DISCHARGE);
  (void)currentSmall;
  (void)movingState;
  // Cooldown after discharge to avoid immediate rebalance from rebound dv
  static uint32_t lastDischargeMs = 0;
  if (bms == BMS_DISCHARGE)
    lastDischargeMs = nowMs;
  const bool recentDischarge = (lastDischargeMs != 0) && ((uint32_t)(nowMs - lastDischargeMs) < kIdleHold_ms);

  // Idle detector: align with BMS_IDLE state (not just |I| <= 100 mA)
  // This avoids mismatch (your BMS_IDLE uses +500 mA / -100 mA deadbands)
  static uint32_t idleStartMs = 0;
  static bool idleArmed = false;
  const bool isIdleState = (m_ptrPowerElecPackInfoConfig->bmsState == BMS_IDLE);
  if (isIdleState)
  {
    if (!idleArmed)
    {
      idleArmed = true;
      idleStartMs = nowMs;
    }
  }
  else
  {
    idleArmed = false;
  }
  const bool idleLong = idleArmed && (nowMs - idleStartMs >= kIdleHold_ms);
  // One-line log when the idle dwell condition is first met
  {
    static bool s_idleLongLogged = false;
    if (idleLong && !s_idleLongLogged)
    {
      m_hwMeasurement.interfaceComm.printToInterface(" [BAL] Idle dwell met (%u ms), dv=%u mV", (unsigned)kIdleHold_ms, (unsigned)dv_mV);
      s_idleLongLogged = true;
    }
    else if (!idleLong)
    {
      s_idleLongLogged = false;
    }
  }

  // Balance permission with hysteresis and current guard:
  // allow when dv is high AND (we are in long idle OR OV recovery OR |I| is small)
  static bool allowBalance = false;
  if (!allowBalance)
  {
    const bool recovery = (m_ptrPowerElecPackInfoConfig->overvoltageState == OV_STATE_RECOVERY);
    const bool chargingCurrentSmall = (I_mA >= BalancingArgs::MAX_CHARGE_CURRENT_MA * -1); // negative while charging
    const bool gatingOk = (bms == BMS_DISCHARGE) ? false // Never balance during discharge
                          : (bms == BMS_CHARGE)  ? (chargingCurrentSmall && !recentDischarge)
                                                 : ((idleLong || recovery) && !recentDischarge);
    if (dv_mV >= kDV_START_mV && gatingOk)
      allowBalance = true;
  }
  else
  {
    const bool recovery = (m_ptrPowerElecPackInfoConfig->overvoltageState == OV_STATE_RECOVERY);
    const bool chargingCurrentSmall = (I_mA >= BalancingArgs::MAX_CHARGE_CURRENT_MA * -1); // negative while charging
    const bool gatingOk = (bms == BMS_DISCHARGE) ? false // Never balance during discharge
                          : (bms == BMS_CHARGE)  ? (chargingCurrentSmall && !recentDischarge)
                                                 : ((idleLong || recovery) && !recentDischarge);
    if (dv_mV <= kDV_STOP_mV || !gatingOk)
    {
      allowBalance = false;
    }
  }

#if !DISABLE_BALANCING
  // --------------------------------------------------------------------------
  // TIME-SLICED BALANCING LOGIC
  // Cycle: 5s MEASURE (Balancing OFF) -> 30s ACTIVE (Balancing ON) -> 5s QUIET (Balancing OFF)
  // --------------------------------------------------------------------------

  // Thermal pause for balancing (TCB_WARN ~105C with hysteresis).
  static bool s_balanceThermalPaused = false;
  
  static uint32_t s_balanceCycleStartMs = 0;
  static bool s_cycleInitialized = false;
  if (!s_cycleInitialized) {
      s_balanceCycleStartMs = nowMs;
      s_cycleInitialized = true;
  }

  static bool s_balanceOddCycle = true; // Odd Cycle (1, 3, 5...)

  const uint32_t BALANCE_CYCLE_MEASURE_MS = 1000;  // 5 seconds
  const uint32_t BALANCE_CYCLE_ACTIVE_MS = 5000;  // 30 seconds
  const uint32_t BALANCE_CYCLE_QUIET_MS = 1000;    // 5 seconds
  const uint32_t CYCLE_PERIOD =
      BALANCE_CYCLE_MEASURE_MS + BALANCE_CYCLE_ACTIVE_MS + BALANCE_CYCLE_QUIET_MS;

  uint32_t cycleTime = nowMs - s_balanceCycleStartMs;
  if (cycleTime >= CYCLE_PERIOD) {
      s_balanceCycleStartMs = nowMs;
      cycleTime = 0;
      
      // Toggle Parity (Odd <-> Even)
      s_balanceOddCycle = !s_balanceOddCycle;
      m_hwMeasurement.interfaceComm.printToInterface("[BAL-CYCLE] New Cycle: Parity=%s\r\n", s_balanceOddCycle ? "ODD" : "EVEN");
  }

  const bool isMeasurePhase = (cycleTime < BALANCE_CYCLE_MEASURE_MS);
  const bool isActivePhase =
      (cycleTime >= BALANCE_CYCLE_MEASURE_MS) &&
      (cycleTime < (BALANCE_CYCLE_MEASURE_MS + BALANCE_CYCLE_ACTIVE_MS));
  const bool isQuietPhase = (!isMeasurePhase && !isActivePhase);

  // Update the member flag so other functions (e.g., min/max calc) know the phase
  m_bBalanceMeasurePhase = !isActivePhase;
  
  if (isMeasurePhase || isQuietPhase)
  {
      // --- MEASURE/QUIET PHASE (Balancing FORCE OFF) ---
      // This ensures we get clean voltage readings without IR drop from bleeders.
      s_balanceThermalPaused = false; // Reset thermal flag during measurement
      
      // Force balancing OFF at the hardware level
      m_hwMeasurement.bq.setBalanceActive(false);
      clearBleedFlags(m_ptrPowerElecPackInfoConfig);
      
      // Update UI/System state but indicate we are measuring
      if (allowBalance) {
          // Log occasionally that we are in the measurement pause
          static uint32_t _u32LogMeasure = 0;
          if (timerDelay1ms(&_u32LogMeasure, 10000)) {
               m_hwMeasurement.interfaceComm.printToInterface("[BAL-CYCLE] MEASURING (Balancing Paused) dv=%u\r\n", (unsigned)dv_mV);
          }
      }
      
      return; // SKIP the rest of the balancing logic
  }
  // --- Else: ACTIVE PHASE (Normal Balancing Logic proceeds below) ---
#endif

  // Thermal check continues (re-using the now visible s_balanceThermalPaused)
  const int32_t temp_max_mC = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax;
  if (!s_balanceThermalPaused && temp_max_mC >= BalancingArgs::THERMAL_PAUSE_MC)
  {
    s_balanceThermalPaused = true;
    m_hwMeasurement.interfaceComm.printToInterface(
        "[BAL] thermal pause at %ld mC\r\n", (long)temp_max_mC);
  }
  else if (s_balanceThermalPaused && temp_max_mC <= BalancingArgs::THERMAL_RESUME_MC)
  {
    s_balanceThermalPaused = false;
    m_hwMeasurement.interfaceComm.printToInterface(
        "[BAL] thermal resume at %ld mC\r\n", (long)temp_max_mC);
  }
#if DISABLE_BALANCING
  const bool balanceAllowed = false;  // Balancing completely disabled via Settings.h
#else
  const bool balanceAllowed = allowBalance && !s_balanceThermalPaused;
#endif

  // Balance progress tracking: dv decay rate + ETA (log every ~60s while active)
  {
    static bool s_active = false;
    static uint16_t s_last_dv_mV = 0;
    static uint32_t s_last_ms = 0;
    static uint32_t s_last_log_ms = 0;
    const uint32_t kLogInterval_ms = 60u * 1000u;
    if (balanceAllowed)
    {
      if (!s_active)
      {
        s_active = true;
        s_last_dv_mV = dv_mV;
        s_last_ms = nowMs;
        s_last_log_ms = nowMs;
      }
      else if ((nowMs - s_last_log_ms) >= kLogInterval_ms)
      {
        const uint32_t dt_ms = nowMs - s_last_ms;
        const int32_t dv_drop = static_cast<int32_t>(s_last_dv_mV) - static_cast<int32_t>(dv_mV);
        float rate_mV_per_min = 0.0f;
        if (dt_ms > 0)
        {
          rate_mV_per_min = (dv_drop * 60000.0f) / static_cast<float>(dt_ms);
        }
        if (rate_mV_per_min > 0.1f)
        {
          const float eta_min = (dv_mV > kDV_STOP_mV)
                                    ? (static_cast<float>(dv_mV - kDV_STOP_mV) / rate_mV_per_min)
                                    : 0.0f;
          m_hwMeasurement.interfaceComm.printToInterface(
              "[BAL] dv=%u mV, rate=%.2f mV/min, ETA=%.1f min\r\n",
              (unsigned)dv_mV, rate_mV_per_min, eta_min);
        }
        else
        {
          m_hwMeasurement.interfaceComm.printToInterface(
              "[BAL] dv=%u mV, rate=%.2f mV/min, ETA=--\r\n",
              (unsigned)dv_mV, rate_mV_per_min);
        }
        s_last_dv_mV = dv_mV;
        s_last_ms = nowMs;
        s_last_log_ms = nowMs;
      }
    }
    else
    {
      s_active = false;
      s_last_ms = 0;
      s_last_log_ms = 0;
    }
  }

  // Charge-stop hysteresis for large pack mismatch (keeps inverter chatter-free)
  // Charge-stop hysteresis for large pack mismatch (keeps inverter chatter-free)
  static bool throttleActive = false;
  // Soft-start state for releasing throttle
  static bool s_rampingActive = false;
  static uint32_t s_currentLimit_centiA = 0; // Working limit in 0.01A units
  static uint32_t s_lastRampTick = 0;
  auto setBalancingFlag = [&](bool active) {
    m_ptrPowerElecPackInfoConfig->bBalancingActive = active;
  };

  auto startChargeRamp = [&](uint32_t start_cA) {
    const uint32_t maxCurrent = derateChargeCurrent_cA(m_ptrGenConfig->u16MaxHardChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig);
    s_currentLimit_centiA = std::min(start_cA, maxCurrent);
    throttleActive = false;
    s_rampingActive = true;
    s_lastRampTick = nowMs;
    m_hwMeasurement.inverterCan.updateMaxChargeLimit(
        m_ptrGenConfig->u16MaxChargePackVoltage, (uint16_t)s_currentLimit_centiA);
  };

  // --- End-of-Charge (EOC) Cycle Limiter ---
  static uint8_t s_eocCycleCount = 0;
  static bool s_eocLatched = false;

  // Reset EOC count if SOC drops (< 90%) or if we are discharging (> 1A)
  if (m_ptrPowerElecPackInfoConfig->u16ModuleSoc < 900 || m_ptrPowerElecPackInfoConfig->i32ModuleCurrent > 1000)
  {
      if (s_eocCycleCount > 0) s_eocCycleCount = 0;
      if (s_eocLatched) {
          s_eocLatched = false;
          m_hwMeasurement.interfaceComm.printToInterface("[BAL] EOC Reset (SOC/Identified Discharge)\r\n");
      }
  }

  // Check Limit
  if (s_eocLatched)
  {
      // Enforce Stop Charge
      m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 0);
      logChargeClampStatic("EOC_LATCH");
      
      // Force states if needed to persist 100%
      if (m_ptrPowerElecPackInfoConfig->u16ModuleSoc < 1000) {
          m_ptrPowerElecPackInfoConfig->u16ModuleSoc = 1000;
          m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest = true; // ensure it saves
      }
      
      // Skip the rest of the throttle logic to prevent overriding the 0A limit
      // We still allow the outer function to proceed (which might handle balancing), 
      // but we shouldn't execute the throttle state machine below.
  }
  else if (s_dvLock.active())
  {
    if (throttleActive) throttleActive = false; 
    if (s_rampingActive) s_rampingActive = false;
  }
  else if (true /* dv clamp irrespective of charging */)
  {
    // Don't exit throttle while in OP_STATE_BALANCING (10)
    const bool inBalancingState = (m_ptrPowerElecPackInfoConfig->u8operationState == 10);
    const uint32_t RAMP_STEP_CENTIA = 50; // 0.50 A per step
    const uint32_t RAMP_PERIOD_MS = 1000;  // 1 second per step

    // 1. Throttle Entry
    // If not throttled and imbalance is high -> Clamp current immediately
    if (!throttleActive && !s_rampingActive && dv_mV >= kPACK_MISMATCH_ENTRY_mV)
    {
      throttleActive = true;
      
      // Increment EOC counter if we are near full (SOC >= 90%)
      if (m_ptrPowerElecPackInfoConfig->u16ModuleSoc >= 900) {
          s_eocCycleCount++;
          m_hwMeasurement.interfaceComm.printToInterface("[BAL] EOC Cycle %d/5 detected\r\n", s_eocCycleCount);
          
      if (s_eocCycleCount >= 5) {
          s_eocLatched = true;
          m_ptrPowerElecPackInfoConfig->u16ModuleSoc = 1000; // Force 100%
          m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest = true;
          m_hwMeasurement.interfaceComm.printToInterface("[BAL] EOC LIMIT REACHED: Forced SOC 100%, Charge Stopped.\r\n");
          
          m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 0);
          logChargeClampStatic("EOC_LATCH_LIMIT");
          return; // Exit checking for this tick
      }
  }

      // THROTTLE_X10MA is "0 (stop)" in tuning header
      m_hwMeasurement.inverterCan.updateMaxChargeLimit(
          m_ptrGenConfig->u16MaxChargePackVoltage, (uint16_t)THROTTLE_X10MA);
      logChargeClampStatic("THROTTLE_ENTRY");
      
      m_hwMeasurement.interfaceComm.printToInterface(
          "[BAL] THROTTLE ON: dv=%u mV I=%ld mA\r\n", (unsigned)dv_mV, (long)I_mA);
    }
    // 2. Throttle Exit -> Start Ramp
    // If throttled, mismatch is low, and not actively balancing -> Start ramping up
    else if (throttleActive && dv_mV <= kPACK_MISMATCH_EXIT_mV && !inBalancingState)
    {
      // Start ramp from low base (e.g. 5A = 500 centiA) to avoid jumps if throttle was 0
      startChargeRamp(500);
      const float limit_A = static_cast<float>(s_currentLimit_centiA) / 100.0f;
      m_hwMeasurement.interfaceComm.printToInterface(
          "[BAL] RAMP START: dv=%u mV I=%ld mA set=%.2fA\r\n",
          (unsigned)dv_mV, (long)I_mA, (double)limit_A);
    }
    // 3. Ramping Logic
    else if (s_rampingActive)
    {
        // Safety: If dv rises near entry threshold again, abort ramp and throttle immediately
        if (dv_mV >= (kPACK_MISMATCH_ENTRY_mV - 20)) 
        {
            s_rampingActive = false;
            throttleActive = true;
            // Note: We could increment EOC here too, but let's stick to the main entry point to avoid double counting fast flutters
            
            m_hwMeasurement.inverterCan.updateMaxChargeLimit(
                m_ptrGenConfig->u16MaxChargePackVoltage, (uint16_t)THROTTLE_X10MA);
            logChargeClampStatic("THROTTLE_RAMP_ABORT");
            m_hwMeasurement.interfaceComm.printToInterface(
                "[BAL] RAMP ABORT: dv=%u too high! Throttle back.\r\n", (unsigned)dv_mV);
        }
        else if ((nowMs - s_lastRampTick) >= RAMP_PERIOD_MS)
        {
            s_currentLimit_centiA += RAMP_STEP_CENTIA;
            const uint32_t maxCurrent = derateChargeCurrent_cA(m_ptrGenConfig->u16MaxHardChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig); // already centi-amps

            if (s_currentLimit_centiA >= maxCurrent) {
                s_currentLimit_centiA = maxCurrent;
                s_rampingActive = false; // Ramping complete
                m_hwMeasurement.interfaceComm.printToInterface("[BAL] RAMP DONE: Full Current Restored (%u cA)\r\n", (unsigned)maxCurrent);
            } else {
                m_hwMeasurement.interfaceComm.printToInterface("[BAL] RAMP UP: Set %u cA (dv=%u)\r\n", (unsigned)s_currentLimit_centiA, (unsigned)dv_mV);
            }

            m_hwMeasurement.inverterCan.updateMaxChargeLimit(
                m_ptrGenConfig->u16MaxChargePackVoltage, (uint16_t)s_currentLimit_centiA);
            
            s_lastRampTick = nowMs;
        }
    }
  }
  else
  {
      if (throttleActive || s_rampingActive) {
          throttleActive = false;
          s_rampingActive = false;
      }
  }

  if (m_commissioningActive)
  {
    const uint16_t softOv_mV = m_ptrGenConfig->u16SoftOverVoltage;
    const uint16_t top_thresh_mV = (softOv_mV > 30U) ? (uint16_t)(softOv_mV - 30U) : softOv_mV;

    if (dv_mV <= kCommDoneDv_mV && vmax_raw >= top_thresh_mV)
    {
      if (m_commissioningDvStableStartMs == 0)
        m_commissioningDvStableStartMs = nowMs;
      if ((nowMs - m_commissioningDvStableStartMs) >= kCommDoneHoldMs)
      {
        m_commissioningActive = false;
      }
    }
    else
    {
      m_commissioningDvStableStartMs = 0;
    }

    if (m_commissioningActive)
    {
      const float cap_Ah = static_cast<float>(m_ptrGenConfig->u16BatteryCapacity);
      const int32_t taper_mA = -(int32_t)std::max(200.0f, 0.05f * cap_Ah * 1000.0f);
      if (bms == BMS_CHARGE && I_mA >= taper_mA && vmax >= top_thresh_mV && dv_mV <= 40u)
      {
        if (m_commissioningFullChargeStartMs == 0)
          m_commissioningFullChargeStartMs = nowMs;
        if ((nowMs - m_commissioningFullChargeStartMs) >= 10u * 60u * 1000u)
        {
          m_commissioningActive = false;
        }
      }
      else
      {
        m_commissioningFullChargeStartMs = 0;
      }
    }

    if (!m_commissioningActive)
    {
      m_hwMeasurement.eeprom.setSettingValue(CMD_EE_ADDR_CommissioningDone, 1);
      m_hwMeasurement.interfaceComm.printToInterface(
          "[COMMISSION] complete dv=%u mV vmax=%u mV\r\n",
          (unsigned)dv_mV, (unsigned)vmax);
      m_hwMeasurement.inverterCan.updateMaxChargeLimit(
          m_ptrGenConfig->u16MaxChargePackVoltage,
          derateChargeCurrent_cA(m_ptrGenConfig->u16MaxHardChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig));
      m_commissioningLastLimit = 0xFFFF;
      m_commissioningStopActive = false;
      m_commissioningUseLowCurrent = false;
      m_commissioningTrendStartMs = 0;
    }
  }

  if (m_commissioningActive)
  {
    if (m_commissioningStopActive)
    {
      if (dv_mV <= kCommStopExit_mV)
        m_commissioningStopActive = false;
    }
    else if (dv_mV >= kCommStopEnter_mV)
    {
      m_commissioningStopActive = true;
    }

    if (!m_commissioningStopActive && dv_mV > kCommDvFull_mV && dv_mV <= kCommDvLimit_mV)
    {
      if (m_commissioningTrendStartMs == 0)
      {
        m_commissioningTrendStartMs = nowMs;
        m_commissioningDvAtTrendStart_mV = dv_mV;
      }
      else if ((nowMs - m_commissioningTrendStartMs) >= kCommTrendMs)
      {
        const uint16_t drop_mV = (m_commissioningDvAtTrendStart_mV > dv_mV)
                                     ? (uint16_t)(m_commissioningDvAtTrendStart_mV - dv_mV)
                                     : 0u;
        m_commissioningUseLowCurrent = (drop_mV < kCommTrendDrop_mV);
        m_commissioningTrendStartMs = nowMs;
        m_commissioningDvAtTrendStart_mV = dv_mV;
      }
      if (m_commissioningUseLowCurrent && dv_mV <= 50u)
      {
        m_commissioningUseLowCurrent = false;
      }
    }
    else
    {
      m_commissioningTrendStartMs = 0;
      m_commissioningDvAtTrendStart_mV = dv_mV;
      if (dv_mV <= kCommDvFull_mV)
        m_commissioningUseLowCurrent = false;
    }

    const bool ov_override = (m_ptrPowerElecPackInfoConfig->overvoltageState != OV_STATE_NORMAL) || (vmax_raw >= 3650u);
    if (!s_dvLock.active() && !ov_override)
    {
      uint16_t target_current = derateChargeCurrent_cA(m_ptrGenConfig->u16MaxHardChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig);
      if (m_commissioningStopActive)
      {
        target_current = 0;
      }
      else if (dv_mV > kCommDvFull_mV)
      {
        target_current = m_commissioningUseLowCurrent ? kCommCurrent0_5A : kCommCurrent1A;
      }
      if (target_current != m_commissioningLastLimit)
      {
        const float limit_A = static_cast<float>(target_current) / 100.0f;
        m_hwMeasurement.inverterCan.updateMaxChargeLimit(
            m_ptrGenConfig->u16MaxChargePackVoltage, target_current);
        if (target_current == 0)
          logChargeClampStatic("COMMISSION_STOP");
        m_hwMeasurement.interfaceComm.printToInterface(
            "[COMMISSION] dv=%u mV limit=%.2fA\r\n",
            (unsigned)dv_mV, (double)limit_A);
        m_commissioningLastLimit = target_current;
      }
    }
  }

  uint16_t activeBleeders = 0;
  int32_t estBleedPerCell_mA = 0;
  int32_t estTotalBleed_mA = 0;

  // ---- Per-cell map: “top-of-pack” targeting ----------------------------
    if (balanceAllowed)
    {
    st_cellMonitorCells balanceMap[NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {};
    const size_t N = NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
    const uint16_t kWindowMin_mV = BalancingArgs::WINDOW_MIN_MV;
    const uint16_t kWindowMax_mV = BalancingArgs::WINDOW_MAX_MV;
    const uint16_t kWindowHighDV_mV = BalancingArgs::WINDOW_HIGH_DV_MV;
    uint16_t window_mV = kWindowMin_mV;
    if (dv_mV >= kWindowHighDV_mV)
    {
      window_mV = kWindowMax_mV;
    }
    else if (dv_mV > kDV_STOP_mV)
    {
      window_mV = static_cast<uint16_t>(
          kWindowMin_mV +
          ((dv_mV - kDV_STOP_mV) * (kWindowMax_mV - kWindowMin_mV)) /
              (kWindowHighDV_mV - kDV_STOP_mV));
    }

    for (size_t i = 0; i < N; ++i)
    {
      const uint16_t v = m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[i].u16CellVoltage;
      bool bleed = false;

      // Smooth dv-based window so selection changes gradually with imbalance.
      // Condition 1: Must be effectively the highest cell (within window)
      bleed = (uint16_t)(v + window_mV) >= vmax;
      
      // Condition 2: Must be above the minimum balancing voltage (Start Voltage)
      if (v < m_ptrGenConfig->u16BalanceStartVoltage)
      {
          bleed = false;
      }

      balanceMap[i].u16CellVoltage = v;
      balanceMap[i].u8CellNumber = (uint8_t)i;
      balanceMap[i].bCellBleedActive = bleed;
    }

    // Enforce per-device constraints: max channels + adjacency rules.
    for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
    {
      bool selected[NoOfCELL_POSSIBLE_ON_CHIP] = {};
      uint8_t order[NoOfCELL_POSSIBLE_ON_CHIP];
      for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
      {
        order[cell] = (uint8_t)cell;
      }
      for (size_t i = 0; i < NoOfCELL_POSSIBLE_ON_CHIP; ++i)
      {
        for (size_t j = i + 1; j < NoOfCELL_POSSIBLE_ON_CHIP; ++j)
        {
          const size_t idx_i = pack * NoOfCELL_POSSIBLE_ON_CHIP + order[i];
          const size_t idx_j = pack * NoOfCELL_POSSIBLE_ON_CHIP + order[j];
          if (balanceMap[idx_j].u16CellVoltage > balanceMap[idx_i].u16CellVoltage)
          {
            const uint8_t tmp = order[i];
            order[i] = order[j];
            order[j] = tmp;
          }
        }
      }

      const uint16_t maxSel = BalancingArgs::MAX_ACTIVE_CELLS_PER_DEV;
      uint16_t countSel = 0;
      for (size_t oi = 0; oi < NoOfCELL_POSSIBLE_ON_CHIP; ++oi)
      {
        const uint8_t cell = order[oi];
        const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
        if (!balanceMap[idx].bCellBleedActive)
          continue;
        if (countSel >= maxSel)
          break;

        bool ok = true;
        if (BalancingArgs::NO_ADJACENT_CELLS)
        {
          // STRICT PARITY INTERLEAVING
          // Logic: 
          //   - Cycle A: Balance Odd Cells (1, 3, 5...) -> Index 0, 2, 4...
          //   - Cycle B: Balance Even Cells (2, 4, 6...) -> Index 1, 3, 5...
          // This guarantees no adjacent cells are ever selected together, 
          // AND guarantees every cell gets 50% duty cycle (Solving Shadowing).
          
          const uint16_t cellV = balanceMap[idx].u16CellVoltage;
          const uint16_t ovpThreshold = (m_ptrGenConfig->u16SoftOverVoltage > 30) 
                                          ? (m_ptrGenConfig->u16SoftOverVoltage - 30) 
                                          : m_ptrGenConfig->u16SoftOverVoltage;
          
          // OVP EMERGENCY OVERRIDE: If cell is near OVP, bypass parity and balance immediately
          const bool nearOvp = (cellV >= ovpThreshold);
          
          if (!nearOvp)
          {
              // Normal Parity Logic
              const bool isEvenIndex = ((cell % 2) == 0); // Cell 1 is Index 0 (Even Index)
              
              // s_balanceOddCycle=true -> Target Odd Cells (1,3,5) -> Target Even Indices (0,2,4)
              // s_balanceOddCycle=false -> Target Even Cells (2,4,6) -> Target Odd Indices (1,3,5)
              
              if (s_balanceOddCycle != isEvenIndex) 
              {
                  // Skip because it doesn't match the current cycle's parity
                  continue;
              }
          }
          // else: Cell is near OVP, bypass parity check and allow immediate balancing
          
          // Adjacency is implicitly handled by parity select (dist >= 2).
          // We effectively skip the expensive explicit check.
        }
        else
        {
          const bool l1 = (cell > 0 && selected[cell - 1]);
          const bool l2 = (cell > 1 && selected[cell - 2]);
          const bool r1 = (cell + 1 < NoOfCELL_POSSIBLE_ON_CHIP && selected[cell + 1]);
          const bool r2 = (cell + 2 < NoOfCELL_POSSIBLE_ON_CHIP && selected[cell + 2]);
          if ((l1 && l2) || (r1 && r2) || (l1 && r1))
            ok = false;
        }

        if (!ok)
          continue;
        selected[cell] = true;
        countSel++;
      }

      for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
      {
        const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
        balanceMap[idx].bCellBleedActive = selected[cell];
      }
    }

    enforceGlobalBleedLimit(balanceMap, N, BalancingArgs::MAX_ACTIVE_CELLS_TOTAL);
    applyBleedFlagsFromMap(m_ptrPowerElecPackInfoConfig, balanceMap, N);
    updateBleedMaskFromMap(m_balanceState, balanceMap, N);

    for (size_t i = 0; i < N; ++i)
    {
      if (balanceMap[i].bCellBleedActive)
        ++activeBleeders;
    }
    {
      const int32_t tempC = temp_max_mC / 1000;
      int32_t bleed_mA = (int32_t)BalancingArgs::BLEED_MA_75C;
      if (tempC > 75)
      {
        bleed_mA -= (tempC - 75) * BalancingArgs::BLEED_DERATE_MA_PER_C;
      }
      if (bleed_mA < 0)
        bleed_mA = 0;
      estBleedPerCell_mA = bleed_mA;
      estTotalBleed_mA = bleed_mA * (int32_t)activeBleeders;
    }

    // Hand per-cell map + dv to BQ (BQ picks timers/duty internally via your handler)
    m_hwMeasurement.bq.sendBalancingValues(balanceMap, dv_mV); // restores original feature
    m_hwMeasurement.bq.setBalanceActive(true);                 // enable bleeders now that we’re allowed
    // Request a system snapshot to SD to mark balancing start
    m_ptrPowerElecPackInfoConfig->u8SystemInformationLoggingState = INTTERFACE_BOOL_TRUE;
    // Indicate auto-balancing via Blue LED blinking
    m_hwMeasurement.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_BLUE, ModEffectArgs::EffectStateType::STATE_FLASH);

    // === UI-COMPATIBLE LOG: [BAL] Start: cells X,Y,Z bleeding (dV=XmV) ===
    // This format is parsed by the BMS Viewer UI to show balancing status
    {
      static uint32_t s_lastBalUiLog = 0;
      if (timerDelay1ms(&s_lastBalUiLog, 5000)) // Every 5 seconds
      {
        // Build CSV list of all active cell indices (M<pack>C<cell>)
        char cellList[256] = {0};
        size_t pos = 0;
        bool first = true;
        
        for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
        {
          for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
          {
            const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
            if (balanceMap[idx].bCellBleedActive && pos < sizeof(cellList) - 10)
            {
              if (!first) {
                cellList[pos++] = ',';
              }
              pos += snprintf(&cellList[pos], sizeof(cellList) - pos, "M%uC%u", (unsigned)(pack + 1), (unsigned)(cell + 1));
              first = false;
            }
          }
        }
        
        if (activeBleeders > 0) {
          m_hwMeasurement.interfaceComm.printToInterface("[BAL] Start: cells %s bleeding (dV=%umV)\r\n", 
            cellList, (unsigned)dv_mV);
        }
      }
    }

    // Periodic log of selected cells per pack (device) with voltage, target, and counts
    {
      static uint32_t s_lastSelPrint = 0;
      if (timerDelay1ms(&s_lastSelPrint, 5000)) // Reduced to 5s
      {
        // Determine the selection target used
        uint16_t target_mV = (vmax > window_mV) ? (uint16_t)(vmax - window_mV) : 0U;

        // First pass: count total and per-pack cells above target (selected)
        uint16_t totalAbove = 0;
        uint16_t packCount[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {0};
        for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
        {
          for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
          {
            const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
            if (balanceMap[idx].bCellBleedActive)
            {
              ++packCount[pack];
              ++totalAbove;
            }
          }
        }

        // Header summary line with dv, Vbat (total), target and total count
        const uint32_t vbat_mV = m_ptrPowerElecPackInfoConfig->u32ModuleVoltage;
        m_hwMeasurement.interfaceComm.printToInterface("[BAL-SEL] dv=%u mV Vbat=%u mV Target=%u mV Floor=%u mV Above=%u\r\n",
                                                       (unsigned)dv_mV,
                                                       (unsigned)vbat_mV,
                                                       (unsigned)target_mV,
                                                       (unsigned)m_ptrGenConfig->u16BalanceStartVoltage,
                                                       (unsigned)totalAbove);

        // Always report all per-pack voltages (even if no cells selected)
        m_hwMeasurement.interfaceComm.printToInterface("[BAL-VPACK]");
        for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
        {
          const uint16_t pack_mV = m_ptrPowerElecPackInfoConfig->moduleMonitorParams[pack].u16PackVoltage;
          m_hwMeasurement.interfaceComm.printToInterface(" P%u=%u", (unsigned)pack, (unsigned)pack_mV);
        }
        m_hwMeasurement.interfaceComm.printToInterface(" \r\n");

        // Second pass: per-pack details (only packs with any selected cells)
        for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
        {
          if (packCount[pack] == 0)
            continue;

          const uint16_t pack_mV = m_ptrPowerElecPackInfoConfig->moduleMonitorParams[pack].u16PackVoltage;
          m_hwMeasurement.interfaceComm.printToInterface("[BAL-SEL] Pack %u Vpack=%u mV (%u):",
                                                         (unsigned)pack,
                                                         (unsigned)pack_mV,
                                                         (unsigned)packCount[pack]);

          for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
          {
            const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
            if (balanceMap[idx].bCellBleedActive)
            {
              m_hwMeasurement.interfaceComm.printToInterface(" C%u=%u",
                                                             (unsigned)(cell + 1),
                                                             (unsigned)balanceMap[idx].u16CellVoltage);
            }
          }
          m_hwMeasurement.interfaceComm.printToInterface(" \r\n");
        }
      }
    }
  }
  else
  {
    m_hwMeasurement.bq.setBalanceActive(false);
    clearBleedFlags(m_ptrPowerElecPackInfoConfig);
    // Request a system snapshot to SD to mark balancing stop
    m_ptrPowerElecPackInfoConfig->u8SystemInformationLoggingState = INTTERFACE_BOOL_TRUE;
    // Turn off Blue if we previously indicated balancing
    m_hwMeasurement.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_BLUE, ModEffectArgs::EffectStateType::STATE_OFF);
  }
  
  // === Track balancing state for UI stop log + exit when no selections ===
  static bool s_wasBalancingForUi = false;
  const bool isBalancingNow = (balanceAllowed && activeBleeders > 0);
  bool balanceStoppedThisCycle = false;
  
  if (isBalancingNow) {
    s_wasBalancingForUi = true;
  } else if (s_wasBalancingForUi) {
    // Was balancing, now stopped
    m_hwMeasurement.interfaceComm.printToInterface("[BAL] Stop: cells balanced\r\n");
    s_wasBalancingForUi = false;
    balanceStoppedThisCycle = true;
  }

  // Unified balancing flag (true if any bleeder active while allowed)
  setBalancingFlag(isBalancingNow);

  if (balanceStoppedThisCycle)
  {
    const bool canRamp =
        !throttleActive &&
        !s_rampingActive &&
        !s_eocLatched &&
        !s_dvLock.active() &&
        m_ptrPowerElecPackInfoConfig->inverterResponseState != WAITING_FOR_RESPONSE &&
        (m_ptrPowerElecPackInfoConfig->overvoltageState == OV_STATE_NORMAL);

    if (canRamp)
    {
      // Resume gently after balancing to avoid hammering freshly equalized cells
      const uint32_t start_cA = std::max<uint32_t>(
          static_cast<uint32_t>(BalancingArgs::THROTTLE_CURRENT_MA / 10U), 50u);
      startChargeRamp(start_cA);
      m_hwMeasurement.interfaceComm.printToInterface(
          "[BAL] RAMP AFTER STOP: start=%u cA dv=%u\r\n",
          (unsigned)s_currentLimit_centiA, (unsigned)dv_mV);
    }
  }

  // If balancing is allowed but no cells are above window, unwind to NORMAL
  if (balanceAllowed && activeBleeders == 0)
  {
    // Clear any balancing state/flags
    allowBalance = false;
    m_ptrPowerElecPackInfoConfig->u8operationState = OP_STATE_LOAD_ENABLED; // back to normal op state
    m_hwMeasurement.interfaceComm.printToInterface("[BAL] No cells selected -> exit balancing, restore normal\r\n");
  }

  // Optional steady log while active (matches your log style)
  if (balanceAllowed || throttleActive)
  {
    static uint32_t _u32DebugPrintLastTick = 0;
    const uint32_t dbg_period_ms = chargingNow ? 3000u : 1000u;

    if (timerDelay1ms(&_u32DebugPrintLastTick, dbg_period_ms))
    {
      m_hwMeasurement.interfaceComm.printToInterface(
          " [BAL] dv=%u mV I=%ld mA allow=%d bleeders=%u est=%ld mA ",
          (unsigned)dv_mV, (long)I_mA, (int)balanceAllowed,
          (unsigned)activeBleeders, (long)estTotalBleed_mA);
      m_hwMeasurement.interfaceComm.printToInterface(" throttle=%d  mode=%s \r\n", (int)throttleActive, chargingNow ? "CHG" : (idleLong ? "IDLE" : "RUN"));
    }
  }
  return;
#else
  // ---- Tunables (safe defaults) -----------------------------------------
  static constexpr uint32_t kIdleHold_ms = BalancingArgs::IDLE_HOLD_MS;
  static constexpr int32_t kChargeThresh_mA = BalancingArgs::CHARGE_THRESH_MA;
  static constexpr uint16_t kDV_START_mV = BalancingArgs::DV_START_MV;
  static constexpr uint16_t kDV_STOP_mV = BalancingArgs::DV_STOP_MV;
  static constexpr uint16_t kPACK_MISMATCH_ENTRY_mV = BalancingArgs::PACK_MISMATCH_ENTRY_MV;
  static constexpr uint16_t kPACK_MISMATCH_EXIT_mV = BalancingArgs::PACK_MISMATCH_EXIT_MV;
  static constexpr int16_t THROTTLE_X10MA = BalancingArgs::THROTTLE_X10MA; // Commissioning mode tunables (now centralized in balancing_tuning.h)
  static constexpr uint16_t kCommDvFull_mV = BalancingArgs::COMM_DV_FULL_MV;
  static constexpr uint16_t kCommDvLimit_mV = BalancingArgs::COMM_DV_LIMIT_MV;
  static constexpr uint16_t kCommStopEnter_mV = BalancingArgs::COMM_STOP_ENTER_MV;
  static constexpr uint16_t kCommStopExit_mV = BalancingArgs::COMM_STOP_EXIT_MV;
  static constexpr uint16_t kCommDoneDv_mV = BalancingArgs::COMM_DONE_DV_MV;
  static constexpr uint32_t kCommDoneHoldMs = BalancingArgs::COMM_DONE_HOLD_MS;
  static constexpr uint32_t kCommTrendMs = BalancingArgs::COMM_TREND_MS;
  static constexpr uint16_t kCommTrendDrop_mV = BalancingArgs::COMM_TREND_DROP_MV;
  static constexpr uint16_t kCommCurrent1A = BalancingArgs::COMM_CURRENT_1A;
  static constexpr uint16_t kCommCurrent0_5A = BalancingArgs::COMM_CURRENT_0_5A;

  // ---- Inputs -----------------------------------------------------------
  const uint32_t nowMs = timerNowMs();
  const int32_t I_mA = m_ptrPowerElecPackInfoConfig->i32ModuleCurrent;         // signed (neg = charge)
  
  // Cache clean readings: During MEASURE phase (bleeders OFF), store the readings.
  // During ACTIVE phase (bleeders ON), use the cached values to avoid IR drop corruption.
  uint16_t vmin, vmax;
  if (isBalanceMeasureWindow())
  {
    // MEASURE phase: Use live values and validate before caching
    const uint16_t rawVmin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinRaw;
    const uint16_t rawVmax = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxRaw;
    uint16_t liveVmin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
    uint16_t liveVmax = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax;

    // Debug: raw min/max snapshot (rate-limited to avoid spam)
    {
      static uint32_t s_rawLogTick = 0;
      if (timerDelay1ms(&s_rawLogTick, 5000))
      {
        const uint16_t dbgVmin = (rawVmin != 0) ? rawVmin : liveVmin;
        const uint16_t dbgVmax = (rawVmax != 0) ? rawVmax : liveVmax;
        m_hwMeasurement.interfaceComm.printToInterface(
            "[BAL-RAW] Vmin=%u mV Vmax=%u mV dV=%u mV\r\n",
            (unsigned)dbgVmin, (unsigned)dbgVmax,
            (unsigned)((dbgVmax >= dbgVmin) ? (dbgVmax - dbgVmin) : 0));
      }
    }

#if BQ_FEATURE_MEASURE_FILTER_MEDIAN3
    {
      auto median3 = [](uint16_t a, uint16_t b, uint16_t c) -> uint16_t {
        return std::max(std::min(a, b), std::min(std::max(a, b), c));
      };

      static uint16_t vmin_hist[3] = {0};
      static uint16_t vmax_hist[3] = {0};
      static uint8_t hist_count = 0;
      static uint8_t hist_idx = 0;

      vmin_hist[hist_idx] = liveVmin;
      vmax_hist[hist_idx] = liveVmax;
      if (hist_count < 3)
      {
        hist_count++;
      }
      hist_idx = (uint8_t)((hist_idx + 1) % 3);

      if (hist_count == 3)
      {
        liveVmin = median3(vmin_hist[0], vmin_hist[1], vmin_hist[2]);
        liveVmax = median3(vmax_hist[0], vmax_hist[1], vmax_hist[2]);
      }
    }
#endif

#if BQ_FEATURE_MEASURE_FILTER_IIR
    {
      static bool s_iir_init = false;
      static uint16_t s_iir_vmin = 0;
      static uint16_t s_iir_vmax = 0;
      const uint32_t alpha = (uint32_t)BQ_MEASURE_FILTER_IIR_ALPHA_PCT;

      if (!s_iir_init)
      {
        s_iir_vmin = liveVmin;
        s_iir_vmax = liveVmax;
        s_iir_init = true;
      }
      else
      {
        const uint32_t inv = 100U - alpha;
        s_iir_vmin = (uint16_t)((s_iir_vmin * inv + (uint32_t)liveVmin * alpha + 50U) / 100U);
        s_iir_vmax = (uint16_t)((s_iir_vmax * inv + (uint32_t)liveVmax * alpha + 50U) / 100U);
      }

      liveVmin = s_iir_vmin;
      liveVmax = s_iir_vmax;
    }
#endif
    
    uint16_t balVmin = liveVmin;
    uint16_t balVmax = liveVmax;
    const bool balOk = computeBalancingExtremes(
        m_ptrPowerElecPackInfoConfig,
        m_ptrGenConfig,
        m_balanceState,
        balVmin,
        balVmax);

    // Plausibility check and consecutive-pass filter
    static PlausibilityTracker s_plaus;
    const bool pass = isSamplePlausible(liveVmin, liveVmax, 
                                         m_balanceState.cachedVmin, 
                                         m_balanceState.cachedVmax);
    if (pass)
    {
      s_plaus.consecutivePasses++;
      s_plaus.consecutiveFails = 0;
    }
    else
    {
      s_plaus.consecutiveFails++;
      s_plaus.consecutivePasses = 0;
    }

    // Accept if plausible AND either we already have a cached baseline or we've seen N consecutive passes.
    constexpr uint8_t kAcceptAfterPasses = 3;
    const bool haveBaseline = (m_balanceState.cachedVmax != 0);
    static bool s_wasRejected = false; // Track rejection state for logging
    if (pass && (haveBaseline || s_plaus.consecutivePasses >= kAcceptAfterPasses))
    {
      // Log immediately if this is first acceptance after rejection(s)
      if (s_wasRejected)
      {
        m_hwMeasurement.interfaceComm.printToInterface(
          "[BAL-CLEAN] recovered after %u rejects: Vmin=%u Vmax=%u dV=%u\r\n",
          (unsigned)s_plaus.consecutiveFails,
          (unsigned)liveVmin, (unsigned)liveVmax, 
          (unsigned)(liveVmax - liveVmin));
        s_wasRejected = false;
      }
      
      m_balanceState.cachedVmin = liveVmin;
      m_balanceState.cachedVmax = liveVmax;
      m_balanceState.cachedBalVmin = balOk ? balVmin : liveVmin;
      m_balanceState.cachedBalVmax = balOk ? balVmax : liveVmax;
      m_balanceState.cachedBalValid = true;
      s_plaus.lastAcceptedMin = liveVmin;
      s_plaus.lastAcceptedMax = liveVmax;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinClean = liveVmin;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxClean = liveVmax;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatchClean =
          (liveVmax >= liveVmin) ? (uint16_t)(liveVmax - liveVmin) : 0;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinBal =
          balOk ? balVmin : liveVmin;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxBal =
          balOk ? balVmax : liveVmax;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatchBal =
          (m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxBal >=
           m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinBal)
              ? (uint16_t)(m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxBal -
                           m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinBal)
              : 0;

      // Cache per-module clean extremes for DV reporting during active balance phase.
      {
        uint8_t nModules = m_ptrGenConfig->u8NumberOfModules;
        if (nModules > NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
          nModules = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
        uint8_t nCells = m_ptrGenConfig->u8NumberOfCells;
        if (nCells > NoOfCELL_POSSIBLE_ON_CHIP)
          nCells = NoOfCELL_POSSIBLE_ON_CHIP;

        if (nModules > 0 && nCells > 0)
        {
          for (uint8_t m = 0; m < nModules; ++m)
          {
            uint16_t minV = 10000;
            uint16_t maxV = 0;
            for (uint8_t c = 0; c < nCells; ++c)
            {
              const uint16_t v = m_ptrPowerElecPackInfoConfig->cellModuleVoltages[m][c];
              if (v > maxV)
                maxV = v;
              if (v < minV)
                minV = v;
            }
            if (minV == 10000)
              minV = 0;
            m_balanceState.cachedModuleVmin[m] = minV;
            m_balanceState.cachedModuleVmax[m] = maxV;
          }
          m_balanceState.cachedModulesValid = true;
        }
      }
      vmin = balOk ? balVmin : liveVmin;
      vmax = balOk ? balVmax : liveVmax;
      
      // Log clean readings periodically
      static uint32_t _u32CleanReadingLogTick = 0;
      if (timerDelay1ms(&_u32CleanReadingLogTick, 5000)) {
        m_hwMeasurement.interfaceComm.printToInterface(
          "[BAL-CLEAN] Vmin=%u mV, Vmax=%u mV, dV=%u mV\r\n",
          (unsigned)vmin, (unsigned)vmax, (unsigned)(vmax - vmin));
      }
    }
    else
    {
      // Keep prior clean snapshot if new sample looks corrupted or not yet vetted
      if (m_balanceState.cachedBalValid)
      {
        vmin = m_balanceState.cachedBalVmin;
        vmax = m_balanceState.cachedBalVmax;
      }
      else
      {
        vmin = m_balanceState.cachedVmin;
        vmax = m_balanceState.cachedVmax;
      }
      if (vmax != 0)
      {
        m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinClean = vmin;
        m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxClean = vmax;
        m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatchClean =
            (vmax >= vmin) ? (uint16_t)(vmax - vmin) : 0;
      }
      s_wasRejected = true; // Mark for recovery logging
      
      // Log rejected samples periodically for debugging
      static uint32_t s_lastRejectLog = 0;
      if (timerDelay1ms(&s_lastRejectLog, 2000))
      {
        const uint16_t lastDv = (m_balanceState.cachedVmax >= m_balanceState.cachedVmin) 
            ? (uint16_t)(m_balanceState.cachedVmax - m_balanceState.cachedVmin) : 0;
        m_hwMeasurement.interfaceComm.printToInterface(
            "[BAL-CLEAN] rejected Vmin=%u Vmax=%u dv=%u cached=[%u,%u] dv=%u pass=%d fails=%u\r\n",
            (unsigned)liveVmin, (unsigned)liveVmax, 
            (unsigned)((liveVmax >= liveVmin) ? (liveVmax - liveVmin) : 0),
            (unsigned)m_balanceState.cachedVmin, (unsigned)m_balanceState.cachedVmax,
            (unsigned)lastDv,
            (int)pass, (unsigned)s_plaus.consecutiveFails);
      }
    }
  }
  else
  {
    // ACTIVE phase: Use cached clean values and update pack info for correct UI display
    const uint16_t cachedVmin = m_balanceState.cachedVmin;
    const uint16_t cachedVmax = m_balanceState.cachedVmax;
    if (m_balanceState.cachedBalValid)
    {
      vmin = m_balanceState.cachedBalVmin;
      vmax = m_balanceState.cachedBalVmax;
    }
    else
    {
      vmin = cachedVmin;
      vmax = cachedVmax;
    }
    
    // Update pack info with clean values so UI doesn't show IR-drop-corrupted readings
    if (cachedVmin != 0 && cachedVmax != 0)
    {
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin = cachedVmin;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax = cachedVmax;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch =
          (uint16_t)(cachedVmax - cachedVmin);
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinClean = cachedVmin;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxClean = cachedVmax;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatchClean =
          (uint16_t)(cachedVmax - cachedVmin);
    }
  }
  
  // Use the local cached/live values we already calculated above
  const uint16_t dv_mV = (vmax >= vmin) ? (uint16_t)(vmax - vmin) : 0;

  // ---- Module pack voltage imbalance clamp --------------------------------
  static bool s_moduleClampActive = false;
  static uint32_t s_moduleClampClearStartMs = 0;
  {
    uint8_t nModules = m_ptrGenConfig->u8NumberOfModules;
    if (nModules > NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
      nModules = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;

    if (nModules >= 2)
    {
      uint32_t sum_mV = 0;
      uint16_t max_mV = 0;
      uint8_t max_idx = 0;
      for (uint8_t m = 0; m < nModules; ++m)
      {
        const uint16_t pack_mV = m_ptrPowerElecPackInfoConfig->moduleMonitorParams[m].u16PackVoltage;
        sum_mV += pack_mV;
        if (pack_mV >= max_mV)
        {
          max_mV = pack_mV;
          max_idx = m;
        }
      }
      const uint32_t avg_others_mV = (sum_mV - max_mV) / (uint32_t)(nModules - 1);
      const uint16_t delta_mV = (max_mV > avg_others_mV) ? (uint16_t)(max_mV - avg_others_mV) : 0;
      if (!s_moduleClampActive && delta_mV >= BalancingArgs::MODULE_PACK_IMBALANCE_MV)
      {
        s_moduleClampActive = true;
        s_moduleClampClearStartMs = 0;
        m_hwMeasurement.interfaceComm.printToInterface(
            "[MOD-CLAMP] M%u delta=%u mV >= %u -> clamp 0A, start balance\r\n",
            (unsigned)(max_idx + 1),
            (unsigned)delta_mV,
            (unsigned)BalancingArgs::MODULE_PACK_IMBALANCE_MV);
      }
      else if (s_moduleClampActive)
      {
        if (delta_mV <= BalancingArgs::MODULE_PACK_IMBALANCE_CLEAR_MV)
        {
          if (s_moduleClampClearStartMs == 0)
            s_moduleClampClearStartMs = nowMs;
          if ((uint32_t)(nowMs - s_moduleClampClearStartMs) >=
              BalancingArgs::MODULE_PACK_IMBALANCE_CLEAR_HOLD_MS)
          {
            s_moduleClampActive = false;
            s_moduleClampClearStartMs = 0;
            m_hwMeasurement.interfaceComm.printToInterface(
                "[MOD-CLAMP] clear M%u delta=%u mV <= %u (hold %lu ms)\r\n",
                (unsigned)(max_idx + 1),
                (unsigned)delta_mV,
                (unsigned)BalancingArgs::MODULE_PACK_IMBALANCE_CLEAR_MV,
                (unsigned long)BalancingArgs::MODULE_PACK_IMBALANCE_CLEAR_HOLD_MS);
          }
        }
        else
        {
          s_moduleClampClearStartMs = 0;
        }
      }
    }
    else
    {
      s_moduleClampActive = false;
      s_moduleClampClearStartMs = 0;
    }
  }

  // ---- dV Trend Watchdog ----------------------------------------------------
  static DvLockController s_dvLock;
  s_dvLock.update(nowMs,
                  dv_mV,
                  I_mA,
                  m_ptrPowerElecPackInfoConfig->bmsState,
                  kPACK_MISMATCH_ENTRY_mV,
                  kDV_STOP_mV,
                  m_hwMeasurement,
                  m_ptrGenConfig,
                  m_ptrPowerElecPackInfoConfig);

  // ---- Derived state ----------------------------------------------------
  const bool chargingNow = (I_mA < kChargeThresh_mA);
  const enBmsState bms = m_ptrPowerElecPackInfoConfig->bmsState;
  const bool loadCurrentHigh = (I_mA > BalancingArgs::MAX_DISCHARGE_CURRENT_MA);
  const bool movingState = (bms == BMS_CHARGE) || (bms == BMS_DISCHARGE);
  const bool moduleClampActive = s_moduleClampActive && (bms == BMS_CHARGE);
  // Cooldown after discharge to avoid immediate rebalance from rebound dv
  if (bms == BMS_DISCHARGE)
    m_balanceState.lastDischargeMs = nowMs;
  const bool recentDischarge = (m_balanceState.lastDischargeMs != 0) && ((uint32_t)(nowMs - m_balanceState.lastDischargeMs) < kIdleHold_ms);

  // Idle detector: align with BMS_IDLE state (not just |I| <= 100 mA)
  // This avoids mismatch (your BMS_IDLE uses +500 mA / -100 mA deadbands)
  const bool isIdleState = (m_ptrPowerElecPackInfoConfig->bmsState == BMS_IDLE);
  if (isIdleState)
  {
    if (!m_balanceState.idleArmed)
    {
      m_balanceState.idleArmed = true;
      m_balanceState.idleStartMs = nowMs;
    }
  }
  else
  {
    m_balanceState.idleArmed = false;
  }
  const bool idleLong = m_balanceState.idleArmed && (nowMs - m_balanceState.idleStartMs >= kIdleHold_ms);
  // One-line log when the idle dwell condition is first met
  {
    static bool s_idleLongLogged = false;
    if (idleLong && !s_idleLongLogged)
    {
      m_hwMeasurement.interfaceComm.printToInterface(" [BAL] Idle dwell met (%u ms), dv=%u mV", (unsigned)kIdleHold_ms, (unsigned)dv_mV);
      s_idleLongLogged = true;
    }
    else if (!idleLong)
    {
      s_idleLongLogged = false;
    }
  }

  // Balance permission with hysteresis and current guard:
  // allow when dv is high AND (we are in long idle OR OV recovery OR |I| is small)
  // CRITICAL: Also require the operation state machine to be in a valid operating state
  // (not INIT, ERROR, POWER_DOWN, etc.) to prevent balancing while contactors are off.
  const uint8_t opState = m_ptrPowerElecPackInfoConfig->u8operationState;
  const bool opStateValid = (opState == OP_STATE_LOAD_ENABLED ||
                             opState == OP_STATE_CHARGING ||
                             opState == OP_STATE_BALANCING ||
                             opState == OP_STATE_FORCEON);

  if (!m_balanceState.allowBalance)
  {
    const bool recovery = (m_ptrPowerElecPackInfoConfig->overvoltageState == OV_STATE_RECOVERY);
    const bool chargingCurrentSmall = (I_mA >= BalancingArgs::MAX_CHARGE_CURRENT_MA * -1); // negative while charging
    const bool gatingOk = (bms == BMS_DISCHARGE) ? false // Never balance during discharge
                          : (bms == BMS_CHARGE)  ? (chargingCurrentSmall && !recentDischarge && !loadCurrentHigh)
                                                 : ((idleLong || recovery) && !recentDischarge && !loadCurrentHigh);
    // GUARD: only allow balancing if in valid operation state
    if ((dv_mV >= kDV_START_mV || moduleClampActive) && (gatingOk || moduleClampActive) && opStateValid)
      m_balanceState.allowBalance = true;
  }
  else
  {
    const bool recovery = (m_ptrPowerElecPackInfoConfig->overvoltageState == OV_STATE_RECOVERY);
    const bool chargingCurrentSmall = (I_mA >= BalancingArgs::MAX_CHARGE_CURRENT_MA * -1); // negative while charging
    const bool gatingOk = (bms == BMS_DISCHARGE) ? false // Never balance during discharge
                          : (bms == BMS_CHARGE)  ? (chargingCurrentSmall && !recentDischarge && !loadCurrentHigh)
                                                 : ((idleLong || recovery) && !recentDischarge && !loadCurrentHigh);
    // GUARD: also stop balancing if operation state is no longer valid (e.g., INIT, ERROR, POWER_DOWN)
    if ((!moduleClampActive && dv_mV <= kDV_STOP_mV) || (!moduleClampActive && !gatingOk) || !opStateValid)
    {
      if (!gatingOk && loadCurrentHigh)
      {
        m_hwMeasurement.interfaceComm.printToInterface("[BAL] stop: load current %ld mA\r\n", (long)I_mA);
      }
      if (!opStateValid)
      {
        m_hwMeasurement.interfaceComm.printToInterface("[BAL] stop: invalid opState=%u (not in valid operating state)\r\n", (unsigned)opState);
      }
      m_balanceState.allowBalance = false;
    }
  }

#if !DISABLE_BALANCING
  const bool isMeasurePhase = m_balanceState.isMeasurePhase;
  const bool isActivePhase = m_balanceState.isActivePhase;
  const bool isQuietStopPhase = m_balanceState.isQuietStopPhase;

  auto forceBleedersOff = [&]() {
    if (m_hwMeasurement.bq.isBalanceActive())
    {
      m_hwMeasurement.bq.forceClearBleeders(); // clear HW timers proactively
    }
    m_hwMeasurement.bq.setBalanceActive(false);
    clearBleedFlags(m_ptrPowerElecPackInfoConfig);
  };

  if (isMeasurePhase || isQuietStopPhase)
  {
    // --- MEASUREMENT/QUIET PHASE (Balancing FORCE OFF) ---
    // Ensures clean voltage readings and guarantees bleeders are off before wrap.
    m_balanceState.thermalPaused = false; // Reset thermal flag during clean windows

    forceBleedersOff();

    // Optional breadcrumb while measuring (rate limited)
    if (m_balanceState.allowBalance && isMeasurePhase)
    {
      static uint32_t _u32LogMeasure = 0;
      if (timerDelay1ms(&_u32LogMeasure, 10000))
      {
        m_hwMeasurement.interfaceComm.printToInterface("[BAL-CYCLE] MEASURING (Balancing Paused) dv=%u\r\n", (unsigned)dv_mV);
      }
    }

    return; // Skip active balancing logic
  }
  // --- Else: ACTIVE PHASE (Normal Balancing Logic proceeds below) ---
#endif

  // Thermal check continues (uses m_balanceState.thermalPaused)
  const int32_t temp_max_mC = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax;
  if (!m_balanceState.thermalPaused && temp_max_mC >= BalancingArgs::THERMAL_PAUSE_MC)
  {
    m_balanceState.thermalPaused = true;
    m_hwMeasurement.interfaceComm.printToInterface(
        "[BAL] thermal pause at %ld mC\r\n", (long)temp_max_mC);
  }
  else if (m_balanceState.thermalPaused && temp_max_mC <= BalancingArgs::THERMAL_RESUME_MC)
  {
    m_balanceState.thermalPaused = false;
    m_hwMeasurement.interfaceComm.printToInterface(
        "[BAL] thermal resume at %ld mC\r\n", (long)temp_max_mC);
  }
#if DISABLE_BALANCING
  const bool baseBalanceAllowed = false;  // Balancing completely disabled via Settings.h
#else
  const bool baseBalanceAllowed = m_balanceState.allowBalance && !m_balanceState.thermalPaused;
#endif

  bool periodicPauseActive = m_balanceState.periodicPauseActive;
  if (!baseBalanceAllowed && periodicPauseActive)
  {
    periodicPauseActive = false;
    m_balanceState.periodicPauseActive = false;
  }
  const bool balanceAllowed = baseBalanceAllowed && !periodicPauseActive;

  {
    static bool s_periodicClampActive = false;
    static uint32_t s_periodicClampResendMs = 0;
    if (periodicPauseActive)
    {
      if (!s_periodicClampActive)
      {
        m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 0);
        m_ptrPowerElecPackInfoConfig->inverterResponseState = WAITING_FOR_RESPONSE;
        m_ptrPowerElecPackInfoConfig->u32InverterCommandTimestamp = nowMs;
        logChargeClampStatic("BAL_PAUSE");
        s_periodicClampActive = true;
        s_periodicClampResendMs = nowMs;
      }
      else if (timerDelay1ms(&s_periodicClampResendMs, 1000))
      {
        m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 0);
        logChargeClampStatic("BAL_PAUSE_REASSERT");
      }

      if (m_hwMeasurement.bq.isBalanceActive())
      {
        m_hwMeasurement.bq.forceClearBleeders();
        m_hwMeasurement.bq.setBalanceActive(false);
        clearBleedFlags(m_ptrPowerElecPackInfoConfig);
      }
      return;
    }
    s_periodicClampActive = false;
  }

  // Balance progress tracking: dv decay rate + ETA (log every ~60s while active)
  {
    static bool s_active = false;
    static uint16_t s_last_dv_mV = 0;
    static uint32_t s_last_ms = 0;
    static uint32_t s_last_log_ms = 0;
    const uint32_t kLogInterval_ms = 60u * 1000u;
    if (balanceAllowed)
    {
      if (!s_active)
      {
        s_active = true;
        s_last_dv_mV = dv_mV;
        s_last_ms = nowMs;
        s_last_log_ms = nowMs;
      }
      else if ((nowMs - s_last_log_ms) >= kLogInterval_ms)
      {
        const uint32_t dt_ms = nowMs - s_last_ms;
        const int32_t dv_drop = static_cast<int32_t>(s_last_dv_mV) - static_cast<int32_t>(dv_mV);
        float rate_mV_per_min = 0.0f;
        if (dt_ms > 0)
        {
          rate_mV_per_min = (dv_drop * 60000.0f) / static_cast<float>(dt_ms);
        }
        if (rate_mV_per_min > 0.1f)
        {
          const float eta_min = (dv_mV > kDV_STOP_mV)
                                    ? (static_cast<float>(dv_mV - kDV_STOP_mV) / rate_mV_per_min)
                                    : 0.0f;
          m_hwMeasurement.interfaceComm.printToInterface(
              "[BAL] dv=%u mV, rate=%.2f mV/min, ETA=%.1f min\r\n",
              (unsigned)dv_mV, rate_mV_per_min, eta_min);
        }
        else
        {
          m_hwMeasurement.interfaceComm.printToInterface(
              "[BAL] dv=%u mV, rate=%.2f mV/min, ETA=--\r\n",
              (unsigned)dv_mV, rate_mV_per_min);
        }
        s_last_dv_mV = dv_mV;
        s_last_ms = nowMs;
        s_last_log_ms = nowMs;
      }
    }
    else
    {
      s_active = false;
      s_last_ms = 0;
      s_last_log_ms = 0;
    }
  }

  // Charge-stop hysteresis for large pack mismatch (keeps inverter chatter-free)
  // Charge-stop hysteresis for large pack mismatch (keeps inverter chatter-free)
  static bool throttleActive = false;
  // Soft-start state for releasing throttle
  static bool s_rampingActive = false;
  static uint32_t s_currentLimit_centiA = 0; // Working limit in 0.01A units
  static uint32_t s_lastRampTick = 0;
  auto setBalancingFlag = [&](bool active) {
    m_ptrPowerElecPackInfoConfig->bBalancingActive = active;
  };

  auto startChargeRamp = [&](uint32_t start_cA) {
    const uint32_t maxCurrent = derateChargeCurrent_cA(m_ptrGenConfig->u16MaxSoftChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig);
    s_currentLimit_centiA = std::min(start_cA, maxCurrent);
    throttleActive = false;
    s_rampingActive = true;
    s_lastRampTick = nowMs;
    m_hwMeasurement.inverterCan.updateMaxChargeLimit(
        m_ptrGenConfig->u16MaxChargePackVoltage, (uint16_t)s_currentLimit_centiA);
  };

  // --- End-of-Charge (EOC) Cycle Limiter ---
  static uint8_t s_eocCycleCount = 0;
  static bool s_eocLatched = false;

  // Reset EOC count if SOC drops (< 90%) or if we are discharging (> 1A)
  if (m_ptrPowerElecPackInfoConfig->u16ModuleSoc < 900 || m_ptrPowerElecPackInfoConfig->i32ModuleCurrent > 1000)
  {
      if (s_eocCycleCount > 0) s_eocCycleCount = 0;
      if (s_eocLatched) {
          s_eocLatched = false;
          m_hwMeasurement.interfaceComm.printToInterface("[BAL] EOC Reset (SOC/Identified Discharge)\r\n");
      }
  }

  // Check Limit
  if (s_eocLatched)
  {
      // Enforce Stop Charge
      m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 0);
      logChargeClampStatic("EOC_LATCH");
      
      // Force states if needed to persist 100%
      if (m_ptrPowerElecPackInfoConfig->u16ModuleSoc < 1000) {
          m_ptrPowerElecPackInfoConfig->u16ModuleSoc = 1000;
          m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest = true; // ensure it saves
      }
      
      // Skip the rest of the throttle logic to prevent overriding the 0A limit
      // We still allow the outer function to proceed (which might handle balancing), 
      // but we shouldn't execute the throttle state machine below.
  }
  else if (s_dvLock.active())
  {
    if (throttleActive) throttleActive = false; 
    if (s_rampingActive) s_rampingActive = false;
  }
  else if (true /* dv clamp irrespective of charging */)
  {
    // Don't exit throttle while in OP_STATE_BALANCING (10)
    const bool inBalancingState = (m_ptrPowerElecPackInfoConfig->u8operationState == 10);
    const uint32_t RAMP_STEP_CENTIA = 50; // 0.50 A per step
    const uint32_t RAMP_PERIOD_MS = 1000;  // 1 second per step
    const bool chargingNow = (I_mA < -100); // only ramp when actually charging

    // 1. Throttle Entry
    // If not throttled and imbalance is high -> Clamp current immediately
    if (!throttleActive && !s_rampingActive && dv_mV >= kPACK_MISMATCH_ENTRY_mV)
    {
      throttleActive = true;
      
      // Increment EOC counter if we are near full (SOC >= 90%)
      if (m_ptrPowerElecPackInfoConfig->u16ModuleSoc >= 900) {
          s_eocCycleCount++;
          m_hwMeasurement.interfaceComm.printToInterface("[BAL] EOC Cycle %d/5 detected\r\n", s_eocCycleCount);
          
      if (s_eocCycleCount >= 5) {
          s_eocLatched = true;
          m_ptrPowerElecPackInfoConfig->u16ModuleSoc = 1000; // Force 100%
          m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest = true;
          m_hwMeasurement.interfaceComm.printToInterface("[BAL] EOC LIMIT REACHED: Forced SOC 100%, Charge Stopped.\r\n");
          
          m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 0);
          logChargeClampStatic("EOC_LATCH_LIMIT");
          return; // Exit checking for this tick
      }
  }

      // THROTTLE_X10MA is "0 (stop)" in tuning header
      m_hwMeasurement.inverterCan.updateMaxChargeLimit(
          m_ptrGenConfig->u16MaxChargePackVoltage, (uint16_t)THROTTLE_X10MA);
      logChargeClampStatic("THROTTLE_ENTRY");
      
      m_hwMeasurement.interfaceComm.printToInterface(
          "[BAL] THROTTLE ON: dv=%u mV I=%ld mA\r\n", (unsigned)dv_mV, (long)I_mA);
    }
    // 2. Throttle Exit -> Start Ramp
    // If throttled, mismatch is low, and not actively balancing -> Start ramping up
    else if (throttleActive && dv_mV <= kPACK_MISMATCH_EXIT_mV && !inBalancingState && chargingNow)
    {
      // Start ramp from low base (e.g. 5A = 500 centiA) to avoid jumps if throttle was 0
      startChargeRamp(1000); // start at 1A after balancing/off throttle
      const float limit_A = static_cast<float>(s_currentLimit_centiA) / 100.0f;
      m_hwMeasurement.interfaceComm.printToInterface(
          "[BAL] RAMP START: dv=%u mV I=%ld mA set=%.2fA\r\n",
          (unsigned)dv_mV, (long)I_mA, (double)limit_A);
    }
    // 3. Ramping Logic
    else if (s_rampingActive)
    {
        // Safety: If dv rises near entry threshold again, abort ramp and throttle immediately
        if (dv_mV >= (kPACK_MISMATCH_ENTRY_mV - 20)) 
        {
            s_rampingActive = false;
            throttleActive = true;
            // Note: We could increment EOC here too, but let's stick to the main entry point to avoid double counting fast flutters
            
            m_hwMeasurement.inverterCan.updateMaxChargeLimit(
                m_ptrGenConfig->u16MaxChargePackVoltage, (uint16_t)THROTTLE_X10MA);
            logChargeClampStatic("THROTTLE_RAMP_ABORT");
            m_hwMeasurement.interfaceComm.printToInterface(
                "[BAL] RAMP ABORT: dv=%u too high! Throttle back.\r\n", (unsigned)dv_mV);
        }
        else if ((nowMs - s_lastRampTick) >= RAMP_PERIOD_MS)
        {
            s_currentLimit_centiA += RAMP_STEP_CENTIA;
            const uint32_t maxCurrent = derateChargeCurrent_cA(m_ptrGenConfig->u16MaxSoftChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig); // already centi-amps

            if (s_currentLimit_centiA >= maxCurrent) {
                s_currentLimit_centiA = maxCurrent;
                s_rampingActive = false; // Ramping complete
                m_hwMeasurement.interfaceComm.printToInterface("[BAL] RAMP DONE: Full Current Restored (%u cA)\r\n", (unsigned)maxCurrent);
            } else {
                m_hwMeasurement.interfaceComm.printToInterface("[BAL] RAMP UP: Set %u cA (dv=%u)\r\n", (unsigned)s_currentLimit_centiA, (unsigned)dv_mV);
            }

            m_hwMeasurement.inverterCan.updateMaxChargeLimit(
                m_ptrGenConfig->u16MaxChargePackVoltage, (uint16_t)s_currentLimit_centiA);
            
            s_lastRampTick = nowMs;
        }
    }
  }
  else
  {
      if (throttleActive || s_rampingActive) {
          throttleActive = false;
          s_rampingActive = false;
      }
  }

  if (m_commissioningActive)
  {
    const uint16_t softOv_mV = m_ptrGenConfig->u16SoftOverVoltage;
    const uint16_t top_thresh_mV = (softOv_mV > 30U) ? (uint16_t)(softOv_mV - 30U) : softOv_mV;

    if (dv_mV <= kCommDoneDv_mV && vmax >= top_thresh_mV)
    {
      if (m_commissioningDvStableStartMs == 0)
        m_commissioningDvStableStartMs = nowMs;
      if ((nowMs - m_commissioningDvStableStartMs) >= kCommDoneHoldMs)
      {
        m_commissioningActive = false;
      }
    }
    else
    {
      m_commissioningDvStableStartMs = 0;
    }

    if (m_commissioningActive)
    {
      const float cap_Ah = static_cast<float>(m_ptrGenConfig->u16BatteryCapacity);
      const int32_t taper_mA = -(int32_t)std::max(200.0f, 0.05f * cap_Ah * 1000.0f);
      if (bms == BMS_CHARGE && I_mA >= taper_mA && vmax >= top_thresh_mV && dv_mV <= 40u)
      {
        if (m_commissioningFullChargeStartMs == 0)
          m_commissioningFullChargeStartMs = nowMs;
        if ((nowMs - m_commissioningFullChargeStartMs) >= 10u * 60u * 1000u)
        {
          m_commissioningActive = false;
        }
      }
      else
      {
        m_commissioningFullChargeStartMs = 0;
      }
    }

    if (!m_commissioningActive)
    {
      m_hwMeasurement.eeprom.setSettingValue(CMD_EE_ADDR_CommissioningDone, 1);
      m_hwMeasurement.interfaceComm.printToInterface(
          "[COMMISSION] complete dv=%u mV vmax=%u mV\r\n",
          (unsigned)dv_mV, (unsigned)vmax);
      m_hwMeasurement.inverterCan.updateMaxChargeLimit(
          m_ptrGenConfig->u16MaxChargePackVoltage,
          derateChargeCurrent_cA(m_ptrGenConfig->u16MaxSoftChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig));
      m_commissioningLastLimit = 0xFFFF;
      m_commissioningStopActive = false;
      m_commissioningUseLowCurrent = false;
      m_commissioningTrendStartMs = 0;
    }
  }

  if (m_commissioningActive)
  {
    if (m_commissioningStopActive)
    {
      if (dv_mV <= kCommStopExit_mV)
        m_commissioningStopActive = false;
    }
    else if (dv_mV >= kCommStopEnter_mV)
    {
      m_commissioningStopActive = true;
    }

    if (!m_commissioningStopActive && dv_mV > kCommDvFull_mV && dv_mV <= kCommDvLimit_mV)
    {
      if (m_commissioningTrendStartMs == 0)
      {
        m_commissioningTrendStartMs = nowMs;
        m_commissioningDvAtTrendStart_mV = dv_mV;
      }
      else if ((nowMs - m_commissioningTrendStartMs) >= kCommTrendMs)
      {
        const uint16_t drop_mV = (m_commissioningDvAtTrendStart_mV > dv_mV)
                                     ? (uint16_t)(m_commissioningDvAtTrendStart_mV - dv_mV)
                                     : 0u;
        m_commissioningUseLowCurrent = (drop_mV < kCommTrendDrop_mV);
        m_commissioningTrendStartMs = nowMs;
        m_commissioningDvAtTrendStart_mV = dv_mV;
      }
      if (m_commissioningUseLowCurrent && dv_mV <= 50u)
      {
        m_commissioningUseLowCurrent = false;
      }
    }
    else
    {
      m_commissioningTrendStartMs = 0;
      m_commissioningDvAtTrendStart_mV = dv_mV;
      if (dv_mV <= kCommDvFull_mV)
        m_commissioningUseLowCurrent = false;
    }

    const bool ov_override = (m_ptrPowerElecPackInfoConfig->overvoltageState != OV_STATE_NORMAL) || (vmax >= 3650u);
    if (!s_dvLock.active() && !ov_override)
    {
      uint16_t target_current = derateChargeCurrent_cA(m_ptrGenConfig->u16MaxSoftChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig);
      if (m_commissioningStopActive)
      {
        target_current = 0;
      }
      else if (dv_mV > kCommDvFull_mV)
      {
        target_current = m_commissioningUseLowCurrent ? kCommCurrent0_5A : kCommCurrent1A;
      }
      if (target_current != m_commissioningLastLimit)
      {
        const float limit_A = static_cast<float>(target_current) / 100.0f;
        m_hwMeasurement.inverterCan.updateMaxChargeLimit(
            m_ptrGenConfig->u16MaxChargePackVoltage, target_current);
        if (target_current == 0)
          logChargeClampStatic("COMMISSION_STOP");
        m_hwMeasurement.interfaceComm.printToInterface(
            "[COMMISSION] dv=%u mV limit=%.2fA\r\n",
            (unsigned)dv_mV, (double)limit_A);
        m_commissioningLastLimit = target_current;
      }
    }
  }

  uint16_t activeBleeders = 0;
  int32_t estBleedPerCell_mA = 0;
  int32_t estTotalBleed_mA = 0;

  // ---- Per-cell map: “top-of-pack” targeting ----------------------------
  // === GATE 1: Only allow cell selection during ACTIVE phase of balance cycle ===
  // (m_balanceState.isActivePhase is set by updateBalanceCycleState)
  if (!m_balanceState.isActivePhase)
  {
    // During MEASURE or QUIET phase, skip cell selection entirely
    // Bleeders should be OFF, and we're collecting clean voltage readings
    return;
  }

  // === GATE 2: Wait for complete measurement cycle before cell selection ===
  // Ensure we have fresh voltage readings from all 4 devices captured during MEASURE phase
#if BQ_FEATURE_MEASURE_CYCLE_GATE
  static bool s_waitingForMeasure = false;
  static uint32_t s_waitStartMs = 0;
  static uint32_t s_waitLogMs = 0;
  static uint32_t s_lastGateLogMs = 0;
  const uint32_t kGateLogIntervalMs = 5000;
  if (!m_hwMeasurement.bq.isMeasureCycleComplete())
  {
    if (balanceAllowed)
    {
      if (!s_waitingForMeasure)
      {
        s_waitingForMeasure = true;
        s_waitStartMs = nowMs;
        s_waitLogMs = nowMs;
        if ((uint32_t)(nowMs - s_lastGateLogMs) >= kGateLogIntervalMs)
        {
          m_hwMeasurement.interfaceComm.printToInterface(
              "[BAL] Not balancing yet: waiting for complete measurement cycle\r\n");
          s_lastGateLogMs = nowMs;
        }
      }
      else if ((uint32_t)(nowMs - s_waitLogMs) >= 30000U)
      {
        const uint32_t wait_s = (uint32_t)((nowMs - s_waitStartMs) / 1000U);
        m_hwMeasurement.interfaceComm.printToInterface(
            "[BAL] Still waiting for measurement cycle (%lus)\r\n",
            (unsigned long)wait_s);
        s_waitLogMs = nowMs;
        s_lastGateLogMs = nowMs;
      }
    }
    return; // Skip this iteration until all devices are read
  }
  {
    if (s_waitingForMeasure)
    {
      const uint32_t waitMs = nowMs - s_waitStartMs;
      if (waitMs >= 1000U && (uint32_t)(nowMs - s_lastGateLogMs) >= kGateLogIntervalMs)
      {
        m_hwMeasurement.interfaceComm.printToInterface(
            "[BAL] Measurement cycle complete -> selecting cells\r\n");
        s_lastGateLogMs = nowMs;
      }
    }
    s_waitingForMeasure = false;
  }
  // Reset the flag for next cycle
  m_hwMeasurement.bq.resetMeasureCycleComplete();
#endif

  // === GATE 3: Pre-balance communication health check ===
  // If COMM faults are detected on any device, delay balancing until the bus is clean
  // THROTTLE: Only check health every 2000ms to reduce bus traffic
  static uint32_t s_lastHealthCheck = 0;
  static bool s_lastHealthStatus = true; // Assume healthy initially or retain last status
  static uint8_t s_healthCheckRetries = 0;

  if (!balanceAllowed)
  {
    // Avoid stale "not healthy" latches when balancing is not allowed.
    s_lastHealthStatus = true;
    s_healthCheckRetries = 0;
  }
  else
  {
    // Only perform the expensive bus transaction if enough time has passed
    if (timerDelay1ms(&s_lastHealthCheck, 2000))
    {
      s_lastHealthStatus = m_hwMeasurement.bq.isCommHealthy();
    }

    if (!s_lastHealthStatus)
    {
      if (s_healthCheckRetries < BalancingArgs::MAX_COMM_FAULT_RETRIES)
      {
        s_healthCheckRetries++;
        static uint32_t s_lastCommFaultLog = 0;
#if ENABLE_COMM_DEBUG_LOGS
      if (timerDelay1ms(&s_lastCommFaultLog, 1000)) {
          m_hwMeasurement.interfaceComm.printToInterface(
              "[BAL] COMM fault detected, delaying balance start (retry %u/%u)\r\n",
              (unsigned)s_healthCheckRetries, (unsigned)BalancingArgs::MAX_COMM_FAULT_RETRIES);
      }
#else
      (void)s_lastCommFaultLog;
#endif
        
        // CRITICAL FIX: Ensure we report OFF state to UI when skipping
        // Otherwise UI latches the last "Active" message and shows stuck balancing
        m_hwMeasurement.bq.forceClearBleeders(); 
        clearBleedFlags(m_ptrPowerElecPackInfoConfig);
        
        return; // Skip this balancing opportunity
      }
      else
      {
#if ENABLE_COMM_DEBUG_LOGS
      m_hwMeasurement.interfaceComm.printToInterface(
          "[BAL] WARN: Proceeding despite COMM faults after %u retries\r\n",
          (unsigned)s_healthCheckRetries);
#endif
      s_healthCheckRetries = 0;
      }
    }
    else
    {
      s_healthCheckRetries = 0;
    }
  }

    if (balanceAllowed)
  {
    st_cellMonitorCells balanceMap[NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {};
    const size_t N = NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
    uint16_t eligibleCount[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {};
    uint16_t selectedCount[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {};
    char eligibleList[NoOfCELL_MONITORS_POSSIBLE_ON_BMS][128] = {};
    char selectedList[NoOfCELL_MONITORS_POSSIBLE_ON_BMS][128] = {};
    size_t eligiblePos[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {};
    size_t selectedPos[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {};

    const uint16_t kWindowMin_mV = BalancingArgs::WINDOW_MIN_MV;
    const uint16_t kWindowMax_mV = BalancingArgs::WINDOW_MAX_MV;
    const uint16_t kWindowHighDV_mV = BalancingArgs::WINDOW_HIGH_DV_MV;
    uint16_t window_mV = kWindowMin_mV;
    if (dv_mV >= kWindowHighDV_mV)
    {
      window_mV = kWindowMax_mV;
    }
    else if (dv_mV > kDV_STOP_mV)
    {
      window_mV = static_cast<uint16_t>(
          kWindowMin_mV +
          ((dv_mV - kDV_STOP_mV) * (kWindowMax_mV - kWindowMin_mV)) /
              (kWindowHighDV_mV - kDV_STOP_mV));
    }

    for (size_t i = 0; i < N; ++i)
    {
      const size_t packIdx = i / NoOfCELL_POSSIBLE_ON_CHIP;
      const uint16_t v = m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[i].u16CellVoltage;
      bool bleed = false;

      // Smooth dv-based window so selection changes gradually with imbalance.
      // Condition 1: Must be effectively the highest cell (within window)
      bleed = (uint16_t)(v + window_mV) >= vmax;
      
      // Condition 2: Must be above the minimum balancing voltage (Start Voltage)
      if (v < m_ptrGenConfig->u16BalanceStartVoltage)
      {
          bleed = false;
      }

      if (bleed)
      {
        eligibleCount[packIdx]++;
        if (eligiblePos[packIdx] < sizeof(eligibleList[packIdx]) - 8)
        {
          if (eligiblePos[packIdx] > 0)
            eligibleList[packIdx][eligiblePos[packIdx]++] = ',';
          eligiblePos[packIdx] += snprintf(&eligibleList[packIdx][eligiblePos[packIdx]],
                                           sizeof(eligibleList[packIdx]) - eligiblePos[packIdx],
                                           "M%uC%u",
                                           (unsigned)(packIdx + 1),
                                           (unsigned)((i % NoOfCELL_POSSIBLE_ON_CHIP) + 1));
        }
      }

      balanceMap[i].u16CellVoltage = v;
      balanceMap[i].u8CellNumber = (uint8_t)i;
      balanceMap[i].bCellBleedActive = bleed;
    }

    // Enforce per-device constraints: max channels + adjacency rules.
    for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
    {
      bool selected[NoOfCELL_POSSIBLE_ON_CHIP] = {};
      uint8_t order[NoOfCELL_POSSIBLE_ON_CHIP];
      for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
      {
        order[cell] = (uint8_t)cell;
      }
      for (size_t i = 0; i < NoOfCELL_POSSIBLE_ON_CHIP; ++i)
      {
        for (size_t j = i + 1; j < NoOfCELL_POSSIBLE_ON_CHIP; ++j)
        {
          const size_t idx_i = pack * NoOfCELL_POSSIBLE_ON_CHIP + order[i];
          const size_t idx_j = pack * NoOfCELL_POSSIBLE_ON_CHIP + order[j];
          if (balanceMap[idx_j].u16CellVoltage > balanceMap[idx_i].u16CellVoltage)
          {
            const uint8_t tmp = order[i];
            order[i] = order[j];
            order[j] = tmp;
          }
        }
      }

      const uint16_t maxSel = BalancingArgs::MAX_ACTIVE_CELLS_PER_DEV;
      uint16_t countSel = 0;
      for (size_t oi = 0; oi < NoOfCELL_POSSIBLE_ON_CHIP; ++oi)
      {
        const uint8_t cell = order[oi];
        const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
        if (!balanceMap[idx].bCellBleedActive)
          continue;
        if (countSel >= maxSel)
          break;

        bool ok = true;
        if (BalancingArgs::NO_ADJACENT_CELLS)
        {
          // STRICT PARITY INTERLEAVING
          // Logic: 
          //   - Cycle A: Balance Odd Cells (1, 3, 5...) -> Index 0, 2, 4...
          //   - Cycle B: Balance Even Cells (2, 4, 6...) -> Index 1, 3, 5...
          // This guarantees no adjacent cells are ever selected together, 
          // AND guarantees every cell gets 50% duty cycle (Solving Shadowing).
          
          const uint16_t cellV = balanceMap[idx].u16CellVoltage;
          const uint16_t ovpThreshold = (m_ptrGenConfig->u16SoftOverVoltage > 30) 
                                          ? (m_ptrGenConfig->u16SoftOverVoltage - 30) 
                                          : m_ptrGenConfig->u16SoftOverVoltage;
          
          // OVP EMERGENCY OVERRIDE: If cell is near OVP, bypass parity and balance immediately
          const bool nearOvp = (cellV >= ovpThreshold);
          
          if (!nearOvp)
          {
              // Normal Parity Logic
              const bool isEvenIndex = ((cell % 2) == 0); // Cell 1 is Index 0 (Even Index)
              
              // s_balanceOddCycle=true -> Target Odd Cells (1,3,5) -> Target Even Indices (0,2,4)
              // s_balanceOddCycle=false -> Target Even Cells (2,4,6) -> Target Odd Indices (1,3,5)
              
              if (m_balanceState.oddCycle != isEvenIndex) 
              {
                  // Skip because it doesn't match the current cycle's parity
                  continue;
              }
          }
          // else: Cell is near OVP, bypass parity check and allow immediate balancing
          
          // Adjacency is implicitly handled by parity select (dist >= 2).
          // We effectively skip the expensive explicit check.
        }
        else
        {
          const bool l1 = (cell > 0 && selected[cell - 1]);
          const bool l2 = (cell > 1 && selected[cell - 2]);
          const bool r1 = (cell + 1 < NoOfCELL_POSSIBLE_ON_CHIP && selected[cell + 1]);
          const bool r2 = (cell + 2 < NoOfCELL_POSSIBLE_ON_CHIP && selected[cell + 2]);
          if ((l1 && l2) || (r1 && r2) || (l1 && r1))
            ok = false;
        }

        if (!ok)
          continue;
        selected[cell] = true;
        countSel++;
      }

      for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
      {
        const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
        balanceMap[idx].bCellBleedActive = selected[cell];
      }

      selectedCount[pack] = countSel;
      if (countSel > 0)
      {
        for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
        {
          if (!selected[cell])
            continue;
          if (selectedPos[pack] < sizeof(selectedList[pack]) - 8)
          {
            if (selectedPos[pack] > 0)
              selectedList[pack][selectedPos[pack]++] = ',';
            selectedPos[pack] += snprintf(&selectedList[pack][selectedPos[pack]],
                                          sizeof(selectedList[pack]) - selectedPos[pack],
                                          "M%uC%u",
                                          (unsigned)(pack + 1),
                                          (unsigned)(cell + 1));
          }
        }
      }
    }

    enforceGlobalBleedLimit(balanceMap, N, BalancingArgs::MAX_ACTIVE_CELLS_TOTAL);
    applyBleedFlagsFromMap(m_ptrPowerElecPackInfoConfig, balanceMap, N);
    updateBleedMaskFromMap(m_balanceState, balanceMap, N);

    for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
    {
      selectedCount[pack] = 0;
      selectedPos[pack] = 0;
      selectedList[pack][0] = '\0';
    }

    for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
    {
      for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
      {
        const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
        if (!balanceMap[idx].bCellBleedActive)
          continue;
        selectedCount[pack]++;
        if (selectedPos[pack] < sizeof(selectedList[pack]) - 8)
        {
          if (selectedPos[pack] > 0)
            selectedList[pack][selectedPos[pack]++] = ',';
          selectedPos[pack] += snprintf(&selectedList[pack][selectedPos[pack]],
                                        sizeof(selectedList[pack]) - selectedPos[pack],
                                        "M%uC%u",
                                        (unsigned)(pack + 1),
                                        (unsigned)(cell + 1));
        }
      }
    }

    for (size_t i = 0; i < N; ++i)
    {
      if (balanceMap[i].bCellBleedActive)
        ++activeBleeders;
    }
    {
      const int32_t tempC = temp_max_mC / 1000;
      int32_t bleed_mA = (int32_t)BalancingArgs::BLEED_MA_75C;
      if (tempC > 75)
      {
        bleed_mA -= (tempC - 75) * BalancingArgs::BLEED_DERATE_MA_PER_C;
      }
      if (bleed_mA < 0)
        bleed_mA = 0;
      estBleedPerCell_mA = bleed_mA;
      estTotalBleed_mA = bleed_mA * (int32_t)activeBleeders;
    }

    // Hand per-cell map + dv to BQ (BQ picks timers/duty internally via your handler)
    m_hwMeasurement.bq.sendBalancingValues(balanceMap, dv_mV); // restores original feature
    m_hwMeasurement.bq.setBalanceActive(true);                 // enable bleeders now that we’re allowed
    // Request a system snapshot to SD to mark balancing start
    m_ptrPowerElecPackInfoConfig->u8SystemInformationLoggingState = INTTERFACE_BOOL_TRUE;
    // Indicate auto-balancing via Blue LED blinking
    m_hwMeasurement.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_BLUE, ModEffectArgs::EffectStateType::STATE_FLASH);

    // === UI-COMPATIBLE LOG: [BAL] Start: cells X,Y,Z bleeding (dV=XmV) ===
    // This format is parsed by the BMS Viewer UI to show balancing status
    {
      static uint32_t s_lastBalUiLog = 0;
      if (timerDelay1ms(&s_lastBalUiLog, 5000)) // Every 5 seconds
      {
        // Build CSV list of all active cell indices (M<pack>C<cell>)
        char cellList[256] = {0};
        size_t pos = 0;
        bool first = true;
        
        for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
        {
          for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
          {
            const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
            if (balanceMap[idx].bCellBleedActive && pos < sizeof(cellList) - 10)
            {
              if (!first) {
                cellList[pos++] = ',';
              }
              pos += snprintf(&cellList[pos], sizeof(cellList) - pos, "M%uC%u", (unsigned)(pack + 1), (unsigned)(cell + 1));
              first = false;
            }
          }
        }
        
        if (activeBleeders > 0) {
          m_hwMeasurement.interfaceComm.printToInterface("[BAL] Start: cells %s bleeding (dV=%umV)\r\n", 
            cellList, (unsigned)dv_mV);
        }
      }
    }

    // Periodic log of selected cells per pack (device) with voltage, target, and counts
    {
      static uint32_t s_lastSelPrint = 0;
      if (timerDelay1ms(&s_lastSelPrint, 5000)) // Reduced to 5s
      {
        // Determine the selection target used
        uint16_t target_mV = (vmax > window_mV) ? (uint16_t)(vmax - window_mV) : 0U;

        // First pass: count total and per-pack cells above target (selected)
        uint16_t totalAbove = 0;
        uint16_t packCount[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {0};
        for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
        {
          for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
          {
            const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
            if (balanceMap[idx].bCellBleedActive)
            {
              ++packCount[pack];
              ++totalAbove;
            }
          }
        }

        // Header summary line with dv, Vbat (total), target and total count
        const uint32_t vbat_mV = m_ptrPowerElecPackInfoConfig->u32ModuleVoltage;
        m_hwMeasurement.interfaceComm.printToInterface("[BAL-SEL] dv=%u mV Vbat=%u mV Target=%u mV Floor=%u mV Above=%u\r\n",
                                                       (unsigned)dv_mV,
                                                       (unsigned)vbat_mV,
                                                       (unsigned)target_mV,
                                                       (unsigned)m_ptrGenConfig->u16BalanceStartVoltage,
                                                       (unsigned)totalAbove);

        // Eligible vs active snapshot (per pack, 5s cadence)
        for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
        {
          const char *eligStr = (eligiblePos[pack] > 0) ? eligibleList[pack] : "-";
          const char *selStr = (selectedPos[pack] > 0) ? selectedList[pack] : "-";
          m_hwMeasurement.interfaceComm.printToInterface("[BAL-MAP] Pack %u elig=%u sel=%u | %s | %s\r\n",
                                                         (unsigned)(pack + 1),
                                                         (unsigned)eligibleCount[pack],
                                                         (unsigned)selectedCount[pack],
                                                         eligStr,
                                                         selStr);
        }

        // Always report all per-pack voltages (even if no cells selected)
        m_hwMeasurement.interfaceComm.printToInterface("[BAL-VPACK]");
        for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
        {
          const uint16_t pack_mV = m_ptrPowerElecPackInfoConfig->moduleMonitorParams[pack].u16PackVoltage;
          m_hwMeasurement.interfaceComm.printToInterface(" P%u=%u", (unsigned)pack, (unsigned)pack_mV);
        }
        m_hwMeasurement.interfaceComm.printToInterface(" \r\n");

        // Second pass: per-pack details (only packs with any selected cells)
        for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
        {
          if (packCount[pack] == 0)
            continue;

          const uint16_t pack_mV = m_ptrPowerElecPackInfoConfig->moduleMonitorParams[pack].u16PackVoltage;
          
          // Build cell list string first to print all on one line (for viewer parsing)
          char cellBuf[128] = {0};
          int pos = 0;
          for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP && pos < (int)sizeof(cellBuf) - 16; ++cell)
          {
            const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
            if (balanceMap[idx].bCellBleedActive)
            {
              pos += snprintf(cellBuf + pos, sizeof(cellBuf) - pos, " C%u=%u",
                              (unsigned)(cell + 1),
                              (unsigned)balanceMap[idx].u16CellVoltage);
            }
          }
          
          // Print complete line (header + cells) for viewer regex parsing
          m_hwMeasurement.interfaceComm.printToInterface("[BAL-SEL] Pack %u Vpack=%u mV (%u):%s\r\n",
                                                         (unsigned)pack,
                                                         (unsigned)pack_mV,
                                                         (unsigned)packCount[pack],
                                                         cellBuf);
        }
      }
    }
  }
  else
  {
    m_hwMeasurement.bq.setBalanceActive(false);
    clearBleedFlags(m_ptrPowerElecPackInfoConfig);
    // Request a system snapshot to SD to mark balancing stop
    m_ptrPowerElecPackInfoConfig->u8SystemInformationLoggingState = INTTERFACE_BOOL_TRUE;
    // Turn off Blue if we previously indicated balancing
    m_hwMeasurement.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_BLUE, ModEffectArgs::EffectStateType::STATE_OFF);
  }
  
  // === Track balancing state for UI stop log + exit when no selections ===
  static bool s_wasBalancingForUi = false;
  static bool s_balanceLatched = false;
  static uint32_t s_noBleedStartMs = 0;
  static bool s_noBleedExitLogged = false;
  static constexpr uint32_t kNoBleedExitMs = BalancingArgs::NO_BLEED_EXIT_MS;

  bool noBleedTimeout = false;
  if (balanceAllowed && activeBleeders == 0 && !moduleClampActive)
  {
    if (s_noBleedStartMs == 0)
      s_noBleedStartMs = nowMs;
    if ((uint32_t)(nowMs - s_noBleedStartMs) >= kNoBleedExitMs)
      noBleedTimeout = true;
  }
  else
  {
    s_noBleedStartMs = 0;
    s_noBleedExitLogged = false;
  }

  const bool latchHold = ((dv_mV > kDV_STOP_mV) &&
                         (vmax >= m_ptrGenConfig->u16BalanceStartVoltage)) ||
                         moduleClampActive;

  // Latch balancing while dv/vmax conditions persist, even if bleeders are momentarily off
  if (noBleedTimeout)
  {
    s_balanceLatched = false;
    if (!s_noBleedExitLogged)
    {
      m_hwMeasurement.interfaceComm.printToInterface(
          "[BAL] exit: no eligible cells for %lu ms\r\n",
          (unsigned long)kNoBleedExitMs);
      s_noBleedExitLogged = true;
    }
  }
  else if (balanceAllowed && latchHold)
    s_balanceLatched = true;
  else if (!latchHold)
    s_balanceLatched = false;

  const uint8_t lastOpState = m_ptrPowerElecPackInfoConfig->u8operationState;
  if (s_balanceLatched)
    m_ptrPowerElecPackInfoConfig->u8operationState = OP_STATE_BALANCING;
  else
    m_ptrPowerElecPackInfoConfig->u8operationState = lastOpState;

  // Optional policy: block charging completely while balancing is latched
  {
    static bool s_blockCharge = false;
    static bool s_clampedCharge = false;
    static uint16_t s_restoreChargeLimit = 0;
    static uint32_t s_clampResendMs = 0;
    // Use centralized constant instead of removed struct field
    const bool disableDuringBalance = BalancingArgs::DISABLE_CHARGE_DURING_BALANCE;

    // Clamp if policy is enabled and either latched or any bleeders are active this cycle,
    // or if module pack imbalance clamp is active.
    const bool clampNow =
        (disableDuringBalance && (s_balanceLatched || activeBleeders > 0)) ||
        moduleClampActive;

    if (clampNow)
    {
      if (!s_blockCharge)
      {
        // Save current limit to restore later (soft max by default)
        s_restoreChargeLimit = derateChargeCurrent_cA(m_ptrGenConfig->u16MaxSoftChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig);
      }
      // Command inverter to 0 A instead of opening contactor
      if (!s_clampedCharge)
      {
        m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 0);
        m_ptrPowerElecPackInfoConfig->inverterResponseState = WAITING_FOR_RESPONSE;
        m_ptrPowerElecPackInfoConfig->u32InverterCommandTimestamp = nowMs;
        m_hwMeasurement.interfaceComm.printToInterface("[BAL-CLAMP] charge limit set to 0A while balancing\r\n");
        s_clampedCharge = true;
        s_clampResendMs = nowMs;
      }

      const bool chargingNow = (I_mA <= BalancingArgs::CHG_ACTIVE_MA);
      if (s_clampedCharge && chargingNow && timerDelay1ms(&s_clampResendMs, 1000))
      {
        m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 0);
        logChargeClampStatic(moduleClampActive ? "MOD_CLAMP_REASSERT" : "BAL_CLAMP_REASSERT");
      }
      s_blockCharge = true;
    }
    else if (s_blockCharge && !clampNow)
    {
      // Restore previous state after balancing ends
      if (s_clampedCharge)
      {
        const uint32_t start_cA = std::max<uint32_t>(
            static_cast<uint32_t>(BalancingArgs::DV_UNLOCK_RAMP_START_CA), 50u);
        startChargeRamp(start_cA);
        m_hwMeasurement.interfaceComm.printToInterface(
            "[BAL-CLAMP] ramp restore after balancing: start=%u cA\r\n",
            (unsigned)s_currentLimit_centiA);
        s_clampedCharge = false;
      }
      s_blockCharge = false;
      s_clampResendMs = 0;
    }
  }

  auto clearBalanceCaches = [&]() {
    m_balanceState.cachedVmin = 0;
    m_balanceState.cachedVmax = 0;
    m_balanceState.cachedBalVmin = 0;
    m_balanceState.cachedBalVmax = 0;
    m_balanceState.cachedBalValid = false;
    m_balanceState.cachedModulesValid = false;
    m_balanceState.bleedMaskValid = false;
    memset(m_balanceState.cachedModuleVmin, 0, sizeof(m_balanceState.cachedModuleVmin));
    memset(m_balanceState.cachedModuleVmax, 0, sizeof(m_balanceState.cachedModuleVmax));
    memset(m_balanceState.bleedMask, 0, sizeof(m_balanceState.bleedMask));
  };

  const bool isBalancingNow = ((balanceAllowed && activeBleeders > 0) || s_balanceLatched);
  {
    static bool s_prevBalancingNow = false;
    if (isBalancingNow && !s_prevBalancingNow)
    {
      if (s_rampingActive || s_currentLimit_centiA != 0)
      {
        s_rampingActive = false;
        s_currentLimit_centiA = 0;
        s_lastRampTick = 0;
        m_hwMeasurement.interfaceComm.printToInterface("[BAL] ramp reset: balancing enabled\r\n");
      }
    }
    else if (!isBalancingNow && s_prevBalancingNow)
    {
      clearBalanceCaches();
    }
    s_prevBalancingNow = isBalancingNow;
  }
  bool balanceStoppedThisCycle = false;
  const bool exitAllowed = (!s_balanceLatched) &&
                           ((dv_mV <= kDV_STOP_mV) ||
                            (vmax < m_ptrGenConfig->u16BalanceStartVoltage) ||
                            !balanceAllowed ||
                            noBleedTimeout);
  
  if (isBalancingNow) {
    s_wasBalancingForUi = true;
  } else if (s_wasBalancingForUi && exitAllowed) {
    // Was balancing, now stopped, and safe to leave balancing state
    m_hwMeasurement.interfaceComm.printToInterface("[BAL] Stop: cells balanced\r\n");
    s_wasBalancingForUi = false;
    balanceStoppedThisCycle = true;
  }

  // Unified balancing flag (true if any bleeder active while allowed)
  setBalancingFlag(isBalancingNow);

  if (balanceStoppedThisCycle)
  {
    const bool canRamp =
        !throttleActive &&
        !s_rampingActive &&
        !s_eocLatched &&
        !s_dvLock.active() &&
        m_ptrPowerElecPackInfoConfig->inverterResponseState != WAITING_FOR_RESPONSE &&
        (m_ptrPowerElecPackInfoConfig->overvoltageState == OV_STATE_NORMAL);

    if (canRamp)
    {
      // Resume gently after balancing to avoid hammering freshly equalized cells
      const uint32_t start_cA = std::max<uint32_t>(
          static_cast<uint32_t>(BalancingArgs::THROTTLE_CURRENT_MA / 10U), 50u);
      startChargeRamp(start_cA);
      m_hwMeasurement.interfaceComm.printToInterface(
          "[BAL] RAMP AFTER STOP: start=%u cA dv=%u\r\n",
          (unsigned)s_currentLimit_centiA, (unsigned)dv_mV);
    }
  }

  // If balancing is allowed but no cells are above window, unwind to NORMAL
  if (balanceAllowed && activeBleeders == 0 && exitAllowed)
  {
    // Clear any balancing state/flags
    m_balanceState.allowBalance = false;
    m_ptrPowerElecPackInfoConfig->u8operationState = OP_STATE_LOAD_ENABLED; // back to normal op state
    m_hwMeasurement.interfaceComm.printToInterface("[BAL] No cells selected -> exit balancing, restore normal\r\n");
  }

  // Optional steady log while active (matches your log style)
  if (balanceAllowed || throttleActive)
  {
    static uint32_t _u32DebugPrintLastTick = 0;
    const uint32_t dbg_period_ms = chargingNow ? 3000u : 1000u;

    if (timerDelay1ms(&_u32DebugPrintLastTick, dbg_period_ms))
    {
      m_hwMeasurement.interfaceComm.printToInterface(
          " [BAL] dv=%u mV I=%ld mA allow=%d bleeders=%u est=%ld mA ",
          (unsigned)dv_mV, (long)I_mA, (int)balanceAllowed,
          (unsigned)activeBleeders, (long)estTotalBleed_mA);
      m_hwMeasurement.interfaceComm.printToInterface(" throttle=%d  mode=%s \r\n", (int)throttleActive, chargingNow ? "CHG" : (idleLong ? "IDLE" : "RUN"));
    }
  }

  // Only freeze min/max while bleeders are actually active in the active phase; if nothing is bleeding,
  // allow fresh readings so we don't pin to stale "clean" values.
  m_balanceState.freezeMeasurements = (isActivePhase && (activeBleeders > 0));
#endif
}
// #############################################################################################################################################################################

// NOTE: Legacy balancing logic is now selectable via USE_LEGACY_BALANCING in Settings.h.


// #############################################################################################################################################################################

void Measurement::subTaskInverterValuesReporting(void)
{
  // Adaptive, delta-gated prints to reduce load while charging
  static uint32_t _u32DebugPrintLastTick = 0;
  const bool charging = (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent < -(int32_t)m_ptrGenConfig->u16ChargerEnabledThreshold);
  const uint32_t period_ms = charging ? 5000u : 10000u;

  // Change thresholds
  constexpr uint16_t DV_THRESH_mV = 10; // vmin/vmax change gate
  constexpr uint16_t DSOC_x10 = 2;      // 0.2% in x0.1%
  constexpr int32_t DI_mA = 100;        // 0.1 A

  static uint16_t s_lastVmin = 0xFFFF;
  static uint16_t s_lastVmax = 0xFFFF;
  static uint16_t s_lastSocx10 = 0xFFFF;
  static int32_t s_lastImA = INT32_MIN;

  if (timerDelay1ms(&_u32DebugPrintLastTick, period_ms))
  {
    const uint16_t vmin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
    const uint16_t vmax = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax;
    const uint16_t socx10 = m_ptrPowerElecPackInfoConfig->u16ModuleSoc;
    const int32_t imA = m_ptrPowerElecPackInfoConfig->i32ModuleCurrent;

    const bool changedEnough =
        (s_lastVmin == 0xFFFF || (vmin > s_lastVmin ? (vmin - s_lastVmin) : (s_lastVmin - vmin)) >= DV_THRESH_mV) ||
        (s_lastVmax == 0xFFFF || (vmax > s_lastVmax ? (vmax - s_lastVmax) : (s_lastVmax - vmax)) >= DV_THRESH_mV) ||
        (s_lastSocx10 == 0xFFFF || (socx10 > s_lastSocx10 ? (socx10 - s_lastSocx10) : (s_lastSocx10 - socx10)) >= DSOC_x10) ||
        (s_lastImA == INT32_MIN || (imA > s_lastImA ? (imA - s_lastImA) : (s_lastImA - imA)) >= DI_mA);

    if (!charging || changedEnough)
    {
      const char *stateStr = (m_ptrPowerElecPackInfoConfig->bmsState == BMS_IDLE) ? "IDLE" : (m_ptrPowerElecPackInfoConfig->bmsState == BMS_DISCHARGE) ? "DISCHARGE"
                                                                                         : (m_ptrPowerElecPackInfoConfig->bmsState == BMS_CHARGE)      ? "CHARGE"
                                                                                                                                                       : "UNKNOWN";
      // Pretty-print values with human-readable units without using floats
      const uint16_t vmin_V_int = vmin / 1000U;
      const uint16_t vmin_V_frac = vmin % 1000U;
      const uint16_t vmax_V_int = vmax / 1000U;
      const uint16_t vmax_V_frac = vmax % 1000U;

      const uint16_t soc_pct_int = socx10 / 10U;  // percent integer part
      const uint16_t soc_pct_frac = socx10 % 10U; // one decimal place

      const int32_t imA_val = imA; // signed mA (neg = charge)
      const int32_t abs_mA = (imA_val < 0) ? -imA_val : imA_val;
      const uint32_t hundredths_A = (uint32_t)((abs_mA + 5) / 10); // round to 0.01 A
      const uint32_t cur_A_int = hundredths_A / 100U;
      const uint32_t cur_A_frac2 = hundredths_A % 100U;
      const char *sign = (imA_val < 0) ? "-" : "";

      // Remaining capacity in Ah with 2 decimals (centi-Ah)
      uint32_t rem_centi_Ah = 0;
      {
        float rem_Ah = m_fRemainingCapacityAh; // already maintained elsewhere
        if (rem_Ah < 0.0f)
          rem_Ah = 0.0f;
        const float scaled = (rem_Ah * 100.0f) + 0.5f; // round to nearest 0.01 Ah
        rem_centi_Ah = (scaled < 0.0f) ? 0u : (uint32_t)scaled;
      }
      const uint32_t rem_Ah_int = rem_centi_Ah / 100U;
      const uint32_t rem_Ah_frac2 = rem_centi_Ah % 100U;

      // Capacity slope (delta per second) in mAh/s with 2 decimals; also estimate equivalent current
      int32_t slope_centi_mAh_per_s = 0; // signed 0.01 mAh/s
      int32_t i_est_hundredths_A = 0;    // signed 0.01 A (derived from slope)
      {
        static uint32_t s_last_ms = 0;
        static int32_t s_last_mAh = -1; // -1 means uninitialized

        // Get current time and remaining capacity in mAh (rounded)
        uint32_t now_ms = duration_cast<milliseconds>(m_measurementManagerTim.elapsed_time()).count();
        int32_t rem_mAh = (int32_t)(m_fRemainingCapacityAh * 1000.0f + 0.5f);

        if (s_last_mAh >= 0 && now_ms > s_last_ms)
        {
          uint32_t dt_ms = now_ms - s_last_ms;
          int32_t d_mAh = rem_mAh - s_last_mAh; // negative while discharging
          // 0.01 mAh/s = (d_mAh * 100000) / dt_ms
          int64_t num = (int64_t)d_mAh * 100000LL;
          int64_t den = (int64_t)dt_ms;
          // rounding toward nearest
          if (num >= 0)
            slope_centi_mAh_per_s = (int32_t)((num + den / 2) / den);
          else
            slope_centi_mAh_per_s = (int32_t)((num - den / 2) / den);

          // Estimate current from slope: I[A] ≈ -(mAh/s) * 3.6
          // We want 0.01 A units: A*100 = -(centi_mAh_s/100)*3.6*100 = -(centi_mAh_s * 360) / 100
          int64_t numA = (int64_t)slope_centi_mAh_per_s * 360LL;
          if (numA >= 0)
            i_est_hundredths_A = (int32_t)(-((numA + 50) / 100));
          else
            i_est_hundredths_A = (int32_t)(-((numA - 50) / 100));
        }

        s_last_ms = now_ms;
        s_last_mAh = rem_mAh;
      }

      // Build sign and integer/frac for slope and estimated current
      const char *slope_sign = (slope_centi_mAh_per_s < 0) ? "-" : (slope_centi_mAh_per_s > 0 ? "+" : "");
      uint32_t slope_abs = (slope_centi_mAh_per_s < 0) ? (uint32_t)(-slope_centi_mAh_per_s) : (uint32_t)slope_centi_mAh_per_s;
      uint32_t slope_int = slope_abs / 100U;
      uint32_t slope_frac2 = slope_abs % 100U;

      const char *iest_sign = (i_est_hundredths_A < 0) ? "-" : (i_est_hundredths_A > 0 ? "+" : "");
      uint32_t iest_abs = (i_est_hundredths_A < 0) ? (uint32_t)(-i_est_hundredths_A) : (uint32_t)i_est_hundredths_A;
      uint32_t iest_int = iest_abs / 100U;
      uint32_t iest_frac2 = iest_abs % 100U;

      m_hwMeasurement.interfaceComm.printToInterface(
          " Vmin:%u.%03u V, Vmax:%u.%03u V, SOC:%u.%u%%, Temp:%d C, Current:%s%u.%02u A, Rem:%u.%02u Ah, Cap:%u Ah, Slope:%s%u.%02u mAh/s (~%s%u.%02u A)  ",//, BMS State:%s
          (unsigned)vmin_V_int,
          (unsigned)vmin_V_frac,
          (unsigned)vmax_V_int,
          (unsigned)vmax_V_frac,
          (unsigned)soc_pct_int,
          (unsigned)soc_pct_frac,
          (int)(m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureAverage / 1000),
          sign,
          (unsigned)cur_A_int,
          (unsigned)cur_A_frac2,
          (unsigned)rem_Ah_int,
          (unsigned)rem_Ah_frac2,
          (unsigned)m_ptrGenConfig->u16BatteryCapacity,
          slope_sign,
          (unsigned)slope_int,
          (unsigned)slope_frac2,
          iest_sign,
          (unsigned)iest_int,
          (unsigned)iest_frac2
          );//,stateStr
      s_lastVmin = vmin;
      s_lastVmax = vmax;
      s_lastSocx10 = socx10;
      s_lastImA = imA;
    }
  }

  // Always publish to inverter; charger is physically connected, it will
  // only pull current when inverter enables charge AND our current limit allows it.
  const uint16_t deratedMax_cA =
      derateChargeCurrent_cA(m_ptrGenConfig->u16MaxHardChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig);
  m_hwMeasurement.inverterCan.setChargeLimitCeiling(deratedMax_cA);
  m_hwMeasurement.inverterCan.updateBatteryTemperature(
      (uint16_t)((m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureAverage / 100) + 1000));
  m_hwMeasurement.inverterCan.updateBatteryVoltage(
      (uint16_t)(m_ptrPowerElecPackInfoConfig->u32ModuleVoltage / 100));
  
  // SOC to inverter: If dwell condition met (99.4%+ for 10min), report 100%
  const uint16_t socForInverter = m_bSocDwellReportAs100 ? 100u : (m_ptrPowerElecPackInfoConfig->u16ModuleSoc / 10);
  m_hwMeasurement.inverterCan.updateSOC(socForInverter);

#if FEATURE_INVERTER_PROTECT_STATUS
  sendInverterProtectionStatus(m_hwMeasurement.inverterCan,
                               inverterProtectWarning,
                               m_ptrPowerElecPackInfoConfig->bAllowedErrorFlags);
#endif

  // // Timer to control the debug message frequency
  // static uint32_t _u32DebugPrintLastTick = 0;

  // // This 'if' block ensures the print statement only runs once per second
  // if (timerDelay1ms(&_u32DebugPrintLastTick, 1000))
  // {
  //   m_hwMeasurement.interfaceComm.printToInterface(" Vmin:%d, Vmax:%d, SOC:%d, Temp:%d, Current:%d , BMS State:%d  ",
  //                                                  m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin,
  //                                                  m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax,
  //                                                  m_ptrPowerElecPackInfoConfig->u16ModuleSoc,
  //                                                  (int)(m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureAverage / 1000),
  //                                                  (int)(m_ptrPowerElecPackInfoConfig->i32ModuleCurrent),
  //                                                  m_ptrPowerElecPackInfoConfig->bmsState);
  // }

  // // The formula converts the internal temperature (scaled by 1000) to the format required by the CAN protocol
  // m_hwMeasurement.inverterCan.updateBatteryTemperature((uint16_t)((m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureAverage / 100) + 1000));

  // m_hwMeasurement.inverterCan.updateBatteryVoltage((uint16_t)(m_ptrPowerElecPackInfoConfig->u32ModuleVoltage / 100));
  // m_hwMeasurement.inverterCan.updateSOC(m_ptrPowerElecPackInfoConfig->u16ModuleSoc / 10);

  // /* ADDR -> 0x35A */
  // /* System Protect and Warning frame sent by BMS */
  // if (m_ptrPowerElecPackInfoConfig->bAllowedErrorFlags)
  // {
  //   // m_hwMeasurement.inverterCan.updateWarningAndProtect(&inverterProtectWarning);
  // }
}

// #############################################################################################################################################################################

void Measurement::subTaskCalculateAllCellsStat(void)
{
  // While balancing, only freeze during ACTIVE phase; MEASURE/QUIET should refresh.
  const bool freezeForBalancing = m_balanceState.freezeMeasurements && !isBalanceMeasureWindow();
  if (freezeForBalancing)
    return;

  uint32_t cellVoltagesSummed = 0;
  m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax = 0;
  m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin = 10000;

  const bool maskMin = BalancingArgs::FILTER_MIN_ADJACENT_TO_BLEED &&
                       m_ptrPowerElecPackInfoConfig->bBalancingActive;
  const uint8_t maxMode = BalancingArgs::FILTER_MAX_MODE;
  const bool maskMax = (maxMode != 0u) && m_ptrPowerElecPackInfoConfig->bBalancingActive;
  const bool maskMaxNeighbors = (maxMode >= 2u);

  uint8_t nModules = m_ptrGenConfig->u8NoOfCellsPerModule;
  uint8_t nCells = m_ptrGenConfig->u8NumberOfCells;
  if (nModules > NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
    nModules = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
  if (nCells > NoOfCELL_POSSIBLE_ON_CHIP)
    nCells = NoOfCELL_POSSIBLE_ON_CHIP;
  const uint16_t totalCells = static_cast<uint16_t>(nModules) * nCells;

  if (totalCells > 0)
  {
    uint16_t minAll = 10000;
    uint16_t maxAll = 0;
    uint16_t minMasked = 10000;
    uint16_t maxMasked = 0;

    for (uint8_t modulePointer = 0; modulePointer < nModules; ++modulePointer)
    {
      for (uint8_t cellPointer = 0; cellPointer < nCells; ++cellPointer)
      {
        const size_t idx =
            (static_cast<size_t>(modulePointer) * NoOfCELL_POSSIBLE_ON_CHIP) + cellPointer;
        const uint16_t v = m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[idx].u16CellVoltage;
        cellVoltagesSummed += v;
        if (v > maxAll)
          maxAll = v;
        if (v < minAll)
          minAll = v;
        if (!maskMin || !isCellMaskedByBleed(m_ptrPowerElecPackInfoConfig, modulePointer, cellPointer, nCells, true))
        {
          if (v < minMasked)
            minMasked = v;
        }
        if (!maskMax || !isCellMaskedByBleed(m_ptrPowerElecPackInfoConfig, modulePointer, cellPointer, nCells, maskMaxNeighbors))
        {
          if (v > maxMasked)
            maxMasked = v;
        }
      }
    }

    const uint16_t finalMin = (maskMin && minMasked != 10000) ? minMasked : minAll;
    const uint16_t finalMax = (maskMax && maxMasked != 0) ? maxMasked : maxAll;
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin = finalMin;
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax = finalMax;
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageAverage =
        static_cast<uint16_t>(cellVoltagesSummed / totalCells);
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch =
        finalMax - finalMin;
  }

  if (!m_ptrPowerElecPackInfoConfig->bBalancingActive)
  {
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxClean = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax;
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinClean = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatchClean = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch;
  }
}

// #############################################################################################################################################################################

void Measurement::subTaskModuleVoltageWatch(void)
{
  uint32_t _u32arrCellVoltagesSummed[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {0};
  uint32_t _u32moduleVoltage = 0;

  for (uint8_t modulePointer = 0; modulePointer < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; modulePointer++)
  {
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u16CellVoltageHigh = 0;
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u16CellVoltageLow = 10000;
  }

  const bool maskMin = BalancingArgs::FILTER_MIN_ADJACENT_TO_BLEED &&
                       m_ptrPowerElecPackInfoConfig->bBalancingActive;
  const uint8_t maxMode = BalancingArgs::FILTER_MAX_MODE;
  const bool maskMax = (maxMode != 0u) && m_ptrPowerElecPackInfoConfig->bBalancingActive;
  const bool maskMaxNeighbors = (maxMode >= 2u);
  uint8_t nCells = m_ptrGenConfig->u8NumberOfCells;
  if (nCells > NoOfCELL_POSSIBLE_ON_CHIP)
    nCells = NoOfCELL_POSSIBLE_ON_CHIP;

  for (uint8_t modulePointer = 0; modulePointer < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; modulePointer++)
  {
    uint16_t minAll = 10000;
    uint16_t minMasked = 10000;
    uint16_t maxAll = 0;
    uint16_t maxMasked = 0;
    for (uint8_t cellPointer = 0; cellPointer < nCells; cellPointer++)
    {
      const uint16_t v = m_ptrPowerElecPackInfoConfig->cellModuleVoltages[modulePointer][cellPointer];
      _u32arrCellVoltagesSummed[modulePointer] += v;

      if (v > maxAll)
        maxAll = v;
      if (v < minAll)
        minAll = v;
      if (!maskMin || !isMinAdjacentToBleed(m_ptrPowerElecPackInfoConfig, modulePointer, cellPointer, nCells))
      {
        if (v < minMasked)
          minMasked = v;
      }
      if (!maskMax || !isCellMaskedByBleed(m_ptrPowerElecPackInfoConfig, modulePointer, cellPointer, nCells, maskMaxNeighbors))
      {
        if (v > maxMasked)
          maxMasked = v;
      }
    }
    const uint16_t finalMax = (maskMax && maxMasked != 0) ? maxMasked : maxAll;
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u16CellVoltageHigh = finalMax;
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u16CellVoltageLow =
        (maskMin && minMasked != 10000) ? minMasked : minAll;
  }

  for (uint8_t modulePointer = 0; modulePointer < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; modulePointer++)
  {
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u16PackVoltage = _u32arrCellVoltagesSummed[modulePointer];
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u16CellVoltageAverage = _u32arrCellVoltagesSummed[modulePointer] / m_ptrGenConfig->u8NumberOfCells;
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u16CellVoltageMisMatch = m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u16CellVoltageHigh - m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u16CellVoltageLow;
    _u32moduleVoltage += _u32arrCellVoltagesSummed[modulePointer];
  }

  // During balance ACTIVE phase, override per-module min/max with cached clean values.
  const bool useCachedModules = (!isBalanceMeasureWindow()) && m_balanceState.cachedModulesValid;
  if (useCachedModules)
  {
    uint8_t nModules = m_ptrGenConfig->u8NumberOfModules;
    if (nModules > NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
      nModules = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
    for (uint8_t m = 0; m < nModules; ++m)
    {
      const uint16_t cachedMin = m_balanceState.cachedModuleVmin[m];
      const uint16_t cachedMax = m_balanceState.cachedModuleVmax[m];
      if (cachedMax != 0 && cachedMax >= cachedMin)
      {
        m_ptrPowerElecPackInfoConfig->moduleMonitorParams[m].u16CellVoltageLow = cachedMin;
        m_ptrPowerElecPackInfoConfig->moduleMonitorParams[m].u16CellVoltageHigh = cachedMax;
        m_ptrPowerElecPackInfoConfig->moduleMonitorParams[m].u16CellVoltageMisMatch =
            (uint16_t)(cachedMax - cachedMin);
      }
    }
  }

  m_ptrPowerElecPackInfoConfig->u32ModuleVoltage = _u32moduleVoltage;
}

// #############################################################################################################################################################################

void Measurement::subTaskModuleTemperatureWatch(void)
{
  uint32_t packTemperaturesSummed[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {0};
  uint8_t _u8PackTemperaturesValidSensorCounter = 0;
  for (uint8_t modulePointer = 0; modulePointer < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; modulePointer++)
  {
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].i32TemperatureMax = -40000;
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].i32TemperatureMin = 100000;
  }

  for (uint8_t modulePointer = 0; modulePointer < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; modulePointer++)
  {
    for (uint8_t tempPointer = 0; tempPointer < m_ptrGenConfig->u8NumberOfTemperature; tempPointer++)
    {
      if (m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature[modulePointer][tempPointer] != 0)
      {
        packTemperaturesSummed[modulePointer] += m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature[modulePointer][tempPointer];
        _u8PackTemperaturesValidSensorCounter++;

        if (m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature[modulePointer][tempPointer] > m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].i32TemperatureMax)
          m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].i32TemperatureMax = m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature[modulePointer][tempPointer];

        if (m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature[modulePointer][tempPointer] < m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].i32TemperatureMin)
          m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].i32TemperatureMin = m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature[modulePointer][tempPointer];
      }
    }
  }

  for (uint8_t modulePointer = 0; modulePointer < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; modulePointer++)
  {
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].i32TemperatureAverage = packTemperaturesSummed[modulePointer] / NoOfTEMP_POSSIBLE_ON_CHIP;
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].i32TemperatureMismatch = m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].i32TemperatureMax - m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].i32TemperatureMin;
  }
}

// #############################################################################################################################################################################
// This is the new watchdog function.
void Measurement::subTaskCheckBqCommunication(void)
{
  const uint32_t BQ_COMMUNICATION_TIMEOUT_MS = 3000; // 3 seconds

  // Check if we have received valid data from the BQ chip within the timeout period.
  // Use manual check to avoid resetting the timestamp (which timerDelay1ms does).
  const uint32_t nowMs = MEASUREMENT_GET_TICK(m_measurementManagerTim);
  static uint32_t s_commBootGraceStartMs = 0;
  if (s_commBootGraceStartMs == 0)
  {
    s_commBootGraceStartMs = nowMs;
  }

  if ((BQ_COMM_BOOT_GRACE_MS > 0) &&
      (m_ptrPowerElecPackInfoConfig->u32LastBqDataTimestamp == 0) &&
      ((nowMs - s_commBootGraceStartMs) < BQ_COMM_BOOT_GRACE_MS))
  {
    // Allow BQ chain to come up without flagging comm faults on first boot.
    return;
  }

  if ((nowMs - m_ptrPowerElecPackInfoConfig->u32LastBqDataTimestamp) >= BQ_COMMUNICATION_TIMEOUT_MS)
  {
    // If the timer expires, it means we have lost communication.
    // const uint32_t nowMs = ... (hoisted above)
    s_bqCommConsecutiveTimeouts++;
    s_bqCommFaultLatched = true;
    s_bqCommLastFaultMs = nowMs;

#if ENABLE_COMM_DEBUG_LOGS
    // Rate-limit timeout spam: only log every 5 seconds to prevent terminal flood
    static uint32_t s_lastTimeoutLogMs = 0;
    const bool shouldLog = (s_bqCommConsecutiveTimeouts == 1) || 
                           (nowMs - s_lastTimeoutLogMs >= 5000);
    
    if (shouldLog)
    {
      s_lastTimeoutLogMs = nowMs;
      m_hwMeasurement.interfaceComm.printToInterface(
          "[COMM] BQ data timeout %u/3 (no frame for %lu ms)%s\r\n",
          (unsigned)s_bqCommConsecutiveTimeouts,
          (unsigned long)(s_bqCommConsecutiveTimeouts * BQ_COMMUNICATION_TIMEOUT_MS),
          s_bqCommConsecutiveTimeouts > 1 ? " [rate-limited]" : "");
    }
#endif
    
#if ENABLE_COMM_SD_LOGS
    if (s_bqCommConsecutiveTimeouts == 1)
    {
      sdCard.logEvent("COMM BQ TIMEOUT start (>%ums)", BQ_COMMUNICATION_TIMEOUT_MS);
    }
#endif

    // Auto-attempt a chipset re-init + re-address every 15s until comm recovers
    // (optimized for 30s reset window: allows 1-2 attempts before system reset)
    static constexpr uint32_t kBQ_RESET_RETRY_MS = 15000;
    const uint32_t since_last_reset = nowMs - s_bqCommLastResetMs;

    if (s_bqCommLastResetMs == 0 || since_last_reset >= kBQ_RESET_RETRY_MS)
    {
      s_bqCommResetCount++;
      const auto err = m_hwMeasurement.bq.reinitAndReaddress();
      s_bqCommLastResetMs = nowMs;
#if ENABLE_COMM_DEBUG_LOGS
      m_hwMeasurement.interfaceComm.printToInterface(
          "[COMM] Auto-reinit/readdr attempt %lu (err=%d)\r\n",
          (unsigned long)s_bqCommResetCount, (int)err);
#endif
#if ENABLE_COMM_SD_LOGS
      sdCard.logEvent("COMM BQ auto-reinit %lu err=%d",
                      (unsigned long)s_bqCommResetCount, (int)err);
#endif
    }

    if (s_bqCommConsecutiveTimeouts >= 3)
    {
      if (!s_bqCommManualResetRequired)
      {
#if ENABLE_COMM_DEBUG_LOGS
        m_hwMeasurement.interfaceComm.printToInterface("[COMM] BQ comm failed 3 times -> manual reset required\r\n");
#endif
#if ENABLE_COMM_SD_LOGS
        sdCard.logEvent("COMM BQ TIMEOUT 3x -> manual reset required");
#endif
      }
      s_bqCommManualResetRequired = true;
    }

    // One-shot full system reset after 30s of continuous comm failure.
    // This attempts to re-initialize everything from scratch, but ONLY ONCE
    // per power cycle to prevent infinite reset loops.
    // (Reduced from 60s: faster recovery now that auto-transition is in place)
    constexpr uint32_t kSYSTEM_RESET_GRACE_MS = 30000; // 30 seconds

    if (s_bqCommFaultStartMs == 0)
    {
      s_bqCommFaultStartMs = nowMs; // Mark when comm fault started
    }

    const uint32_t faultDurationMs = nowMs - s_bqCommFaultStartMs;
    if (!s_bqCommSystemResetAttempted && faultDurationMs >= kSYSTEM_RESET_GRACE_MS)
    {
#if SUPPRESS_COMM_FAULT
      // Downgraded to warning for testing
      static uint32_t s_lastResetWarn = 0;
#if ENABLE_COMM_DEBUG_LOGS
      if (timerDelay1ms(&s_lastResetWarn, 5000)) {
           m_hwMeasurement.interfaceComm.printToInterface("[WARN] BQ comm lost > 30s - Reset suppressed by debug flag\r\n");
      }
#else
      (void)s_lastResetWarn;
#endif
#else
      s_bqCommSystemResetAttempted = true; // One-shot: never reset again this power cycle
#if ENABLE_COMM_DEBUG_LOGS
      m_hwMeasurement.interfaceComm.printToInterface(
          "[COMM] BQ comm lost for %lu s -> system reset (one-shot)\r\n",
          (unsigned long)(faultDurationMs / 1000));
#endif
#if ENABLE_COMM_SD_LOGS
      sdCard.logEvent("COMM BQ TIMEOUT %lus -> system reset", (unsigned long)(faultDurationMs / 1000));
#endif
      ThisThread::sleep_for(100ms); // Allow log to flush
      NVIC_SystemReset();
#endif
    }

    if (m_ptrPowerElecPackInfoConfig->packBatteryWarErrState != SYS_ERROR_CHIPSET_COMM)
    {
#if SUPPRESS_COMM_FAULT
      m_hwMeasurement.interfaceComm.printToInterface("--- WARNING: BQ Communication LOST! (Fault suppressed) ---\r\n");
#else
      m_hwMeasurement.interfaceComm.printToInterface("--- FATAL: BQ Communication LOST! ---\r\n");
#endif
      m_hwMeasurement.bq.printLastCommFaultReason();
    }
#if !SUPPRESS_COMM_FAULT
    m_ptrPowerElecPackInfoConfig->packBatteryWarErrState = SYS_ERROR_CHIPSET_COMM;
#endif

  }
  else
  {
    // Comm is healthy — reset the fault start timestamp for next fault episode
    s_bqCommFaultStartMs = 0;
  }
}
// #############################################################################################################################################################################

void Measurement::subTaskMeasurementModulePower(void)
{

  // Correctly declare the variable as a pointer to the mail struct
  st_mailPowerElecPackInfoConfig *mailPowerElecPackInfoConfig = m_mailPowerElecBox.try_get();

  // Check if the pointer is not null (meaning we got a message)
  if (mailPowerElecPackInfoConfig)
  {
    // Capture raw pack min/max from the incoming BQ snapshot (even if UI is frozen).
    {
      const size_t maxCells = NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
      size_t numCells = static_cast<size_t>(m_ptrGenConfig->u8NumberOfCells) *
                        static_cast<size_t>(m_ptrGenConfig->u8NoOfCellsPerModule);
      if (numCells > maxCells)
        numCells = maxCells;

      uint16_t rawMin = 10000;
      uint16_t rawMax = 0;
      for (size_t i = 0; i < numCells; ++i)
      {
        const uint16_t v = mailPowerElecPackInfoConfig->cellVoltagesIndividual[i];
        if (v > rawMax)
          rawMax = v;
        if (v < rawMin)
          rawMin = v;
      }
      if (rawMin == 10000)
        rawMin = 0;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxRaw = rawMax;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinRaw = rawMin;
      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatchRaw =
          (rawMax >= rawMin) ? (uint16_t)(rawMax - rawMin) : 0;
    }

    // Only update cell voltages during MEASURE phase (bleeders OFF) to avoid IR-drop corrupted readings.
    // During ACTIVE phase, we freeze the displayed values to show the last clean snapshot.
    const uint32_t nowMs = MEASUREMENT_GET_TICK(m_measurementManagerTim);
    const bool measureWindow = isBalanceMeasureWindow();
    static bool s_lastMeasureWindow = false;
    static uint32_t s_measureWindowStartMs = 0;
    if (measureWindow && !s_lastMeasureWindow)
    {
      s_measureWindowStartMs = nowMs;
    }
    s_lastMeasureWindow = measureWindow;

    const bool settling =
        measureWindow &&
        m_ptrPowerElecPackInfoConfig->bBalancingActive &&
        (uint32_t)(nowMs - s_measureWindowStartMs) < BalancingArgs::BALANCE_MEASURE_SETTLE_MS;

    const bool shouldUpdateVoltages =
        (!m_ptrPowerElecPackInfoConfig->bBalancingActive) ||
        (measureWindow && !settling);
    
    if (shouldUpdateVoltages)
    {
      // MEASURE phase or not balancing: update cell voltages with fresh readings
      memcpy(&cellVoltagesMeasIndividual, &mailPowerElecPackInfoConfig->cellVoltagesIndividual, sizeof(mailPowerElecPackInfoConfig->cellVoltagesIndividual));
      memcpy(&tempValuesMeasIndividual, &mailPowerElecPackInfoConfig->tempValuesIndividual, sizeof(mailPowerElecPackInfoConfig->tempValuesIndividual));
    }
    
    // Propagate the clean/frozen readings to the main pack info struct
    // This ensures balancing logic (which reads m_ptrPowerElecPackInfoConfig) uses clean data
    
    // 1. Copy Voltages: Source is uint16_t[], Dest is struct st_cellMonitorCells[].
    // mismatched types require a loop, not memcpy!
    const size_t numCells = NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
    for (size_t i = 0; i < numCells; i++)
    {
       m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[i].u16CellVoltage = cellVoltagesMeasIndividual[i];
    }

    // 2. Copy Temperatures: Source is int32_t[], Dest is int32_t[][]. 
    // Types are compatible (contiguous memory), so memcpy is safe.
    memcpy(&m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature, &tempValuesMeasIndividual, sizeof(tempValuesMeasIndividual));

    // Always update balance state and current (not affected by IR drop)
    memcpy(&balanceStateIndividual, &mailPowerElecPackInfoConfig->balanceStateIndividual, sizeof(balanceStateIndividual));
    m_ptrPowerElecPackInfoConfig->u16ModuleBusbarVal = mailPowerElecPackInfoConfig->u16ModuleBusbarVal;
    // Update latest filtered pack current (mA) from BQ mail
    m_ptrPowerElecPackInfoConfig->i32ModuleCurrent = mailPowerElecPackInfoConfig->i32ModuleCurrent;

    // Free the mail memory
    m_mailPowerElecBox.free(mailPowerElecPackInfoConfig);

  // Update timestamp when latest BQ data is received
    m_ptrPowerElecPackInfoConfig->u32LastBqDataTimestamp = nowMs;

    // Clear comm fault counters on successful traffic
    if (s_bqCommConsecutiveTimeouts > 0 || s_bqCommFaultLatched || s_bqCommManualResetRequired)
    {
#if ENABLE_COMM_DEBUG_LOGS || ENABLE_COMM_SD_LOGS
      uint32_t down_ms = 0;
      if (s_bqCommLastFaultMs != 0)
      {
        down_ms = nowMs - s_bqCommLastFaultMs;
      }
#endif
#if ENABLE_COMM_DEBUG_LOGS
      m_hwMeasurement.interfaceComm.printToInterface(
          "[COMM] BQ data restored after %u consecutive timeouts (manual=%d, down=%lu ms)\r\n",
          (unsigned)s_bqCommConsecutiveTimeouts, (int)s_bqCommManualResetRequired, (unsigned long)down_ms);
#endif
#if ENABLE_COMM_SD_LOGS
      sdCard.logEvent("COMM BQ RESTORED after %u timeouts, down=%lums",
                      (unsigned)s_bqCommConsecutiveTimeouts, (unsigned long)down_ms);
#endif
    }
    s_bqCommConsecutiveTimeouts = 0;
    s_bqCommFaultLatched = false;
    s_bqCommManualResetRequired = false;
    s_bqCommResetCount = 0;
    s_bqCommLastResetMs = nowMs;
    s_bqCommLastRecoveryMs = nowMs;
  }
  // If no message is available, the function will simply continue without blocking.
}

// #############################################################################################################################################################################

// void Measurement::subTaskMeasurementFirstSocPerPack(void)
// {
//   static uint16_t _u16GetVoltageToSoc = 0;
//   _u16GetVoltageToSoc = m_hwMeasurement.soc.getVoltageToSoc(static_cast<float>(m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin) / 1000.0f);

//   // Correctly initialize the remaining capacity in Amp-hours
//   // The value from getVoltageToSoc is scaled by 1000 (e.g., 500 = 50.0%)
//   m_fRemainingCapacityAh = (_u16GetVoltageToSoc / 1000.0f) * (float)m_ptrGenConfig->u16BatteryCapacity;

//   for (uint8_t modulePointer = 0; modulePointer < m_ptrGenConfig->u8NoOfCellsPerModule; modulePointer++)
//   {
//     m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u16Soc = m_hwMeasurement.soc.getVoltageToSoc(static_cast<float>(m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u16CellVoltageLow) / (1000.0));
//   }
// }

// #############################################################################################################################################################################
// Highlight: This new function replaces the old 'subTaskMeasurementFirstSocPerPack'
void Measurement::subTaskSetInitialSoc(bool from_recalibration)
{
#if DISABLE_OCV_CORRECTION
  // OCV correction is disabled via Settings.h
  if (!m_bInitialSocSet)
  {
    m_bInitialSocSet = true;  // Mark as set so we don't keep trying
  }
  return;
#endif

  // Only run if we still haven't committed an initial SOC and we have a valid reading
  if (m_bInitialSocSet || m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin == 0)
  {
    // DEBUG: Show why we're returning early
    static bool s_debugOnce = false;
    if (!s_debugOnce && m_bInitialSocSet)
    {
      m_hwMeasurement.interfaceComm.printToInterface("[SOC-DBG] subTaskSetInitialSoc skipped - already set\r\n");
      s_debugOnce = true;
    }
    return;
  }
  
  // DEBUG: OCV is being applied
  m_hwMeasurement.interfaceComm.printToInterface("[SOC-DBG] Applying OCV! from_recal=%d\r\n", (int)from_recalibration);

  // 1) Derive OCV-based SOC (×10 units; 490 = 49.0%)
  uint16_t ocv_x10 = m_hwMeasurement.soc.getVoltageToSoc(
      static_cast<float>(m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin) / 1000.0f,
      m_ptrPowerElecPackInfoConfig->i32ModuleCurrent,
      m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureAverage);

  // Saturate
  if (ocv_x10 > 1000)
    ocv_x10 = 1000;

  // Current SOC (×10)
  uint16_t now_x10 = m_ptrPowerElecPackInfoConfig->u16ModuleSoc;
  const uint16_t vmin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
  const uint16_t vmax = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax;
  const float ocv_confidence = ocv_confidence_from_voltage(vmin, vmax);

  // 2) Decide whether to bootstrap (accept OCV) or trim (step-limit)
  const bool soc_known = (now_x10 > 0 && now_x10 <= 1000);
  const bool just_booted = (uptime_ms() < MeasurementArgs::SOC_BOOTSTRAP_UPTIME_MS);
  const bool pack_idle = (std::abs(m_ptrPowerElecPackInfoConfig->i32ModuleCurrent) <=
                          MeasurementArgs::SOC_BOOTSTRAP_IDLE_MA);
  const bool dv_ok = (m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax -
                      m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin) <=
                     MeasurementArgs::SOC_BOOTSTRAP_DV_MV;
  const bool rest_ok = pack_idle && dv_ok && just_booted;
  
  // STRICT: If we know the SOC (from EEPROM), DO NOT accept OCV at boot.
  // We only accept OCV if:
  // 1. Explicit recalibration requested (runtime) -> from_recalibration
  // 2. We have NO valid SOC (first boot ever) -> !soc_known
  //
  // We removed the (rest_ok && ocv_confidence...) check when soc_known is true
  // to ensure EEPROM is never overridden at boot.
  const bool ocv_trusted =
      from_recalibration || !soc_known;

  uint16_t commit_x10 = 0;

  if (from_recalibration && soc_known)
  {
    // Recalibration with known SOC: apply OCV, but cap the step.
    const int MAX_STEP_X10 = MeasurementArgs::SOC_RECAL_MAX_STEP_X10;
    int delta = static_cast<int>(ocv_x10) - static_cast<int>(now_x10);
    if (delta > MAX_STEP_X10)
      delta = MAX_STEP_X10;
    if (delta < -MAX_STEP_X10)
      delta = -MAX_STEP_X10;
    commit_x10 = static_cast<uint16_t>(now_x10 + delta);
  }
  else if (ocv_trusted)
  {
    // Bootstrap / trusted recal — accept OCV fully
    commit_x10 = ocv_x10;
  }
  else
  {
    // Runtime trim: small, bounded correction
    const int MAX_STEP_X10 = MeasurementArgs::SOC_TRIM_MAX_STEP_X10;
    const float trim_scale = std::min(1.0f, std::max(0.0f, ocv_confidence));
    const int max_step_x10 = static_cast<int>(MAX_STEP_X10 * trim_scale);
    int delta = static_cast<int>(ocv_x10) - static_cast<int>(now_x10);
    if (delta > max_step_x10)
      delta = max_step_x10;
    if (delta < -max_step_x10)
      delta = -max_step_x10;
    commit_x10 = static_cast<uint16_t>(now_x10 + delta);
  }

  // 3) Apply (×10 → Ah)
  float soc_frac = commit_x10 / 1000.0f;
  m_fRemainingCapacityAh = soc_frac * static_cast<float>(m_ptrGenConfig->u16BatteryCapacity);

  // Keep your single source of truth in config and for inverter I/O
  m_ptrPowerElecPackInfoConfig->u16ModuleSoc = commit_x10; // stays ×10 internally
  m_ptrPowerElecPackInfoConfig->u8SocConfidence = static_cast<uint8_t>(ocv_confidence * 100.0f); // 0-100%
  m_bInitialSocSet = true;                                 // only now that we’ve committed

  // Log SOC recalibration with confidence info
  const char *soc_mode = nullptr;
  if (from_recalibration)
    soc_mode = soc_known ? "RecalClamp" : "Recal";
  else
    soc_mode = ocv_trusted ? "Bootstrap" : "Trim";
  m_hwMeasurement.interfaceComm.printToInterface(
      "[SOC] %s SOC=%u.%u%% conf=%u%% Vmin=%u Vmax=%u mV\r\n",
      soc_mode,
      (unsigned)(commit_x10 / 10), (unsigned)(commit_x10 % 10),
      (unsigned)(m_ptrPowerElecPackInfoConfig->u8SocConfidence),
      (unsigned)vmin, (unsigned)vmax);

  // // This function is now the single point of truth for setting the initial SOC from OCV.
  // if (!m_bInitialSocSet && m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin > 0)
  // {
  //   // Estimate the SOC from the Open Circuit Voltage (OCV) of the lowest cell.
  //   uint16_t _u16GetVoltageToSoc = m_hwMeasurement.soc.getVoltageToSoc(
  //       static_cast<float>(m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin) / 1000.0f,
  //       m_ptrPowerElecPackInfoConfig->i32ModuleCurrent,
  //       m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureAverage);

  //   m_fRemainingCapacityAh = (_u16GetVoltageToSoc / 1000.0f) * (float)m_ptrGenConfig->u16BatteryCapacity;

  //   // Mark the initial SOC as set so this logic doesn't run again until the next power cycle.
  //   m_bInitialSocSet = true;
  // }
}
// #############################################################################################################################################################################

void Measurement::subTaskCellsVoltageWatch(void)
{
  uint32_t _u32CellVoltagesSummed = 0;
  const bool freeze = m_balanceState.freezeMeasurements && !isBalanceMeasureWindow();

  for (uint8_t modulePointer = 0; modulePointer < m_ptrGenConfig->u8NoOfCellsPerModule; modulePointer++)
  {
    for (uint8_t cellPointer = 0; cellPointer < m_ptrGenConfig->u8NumberOfCells; cellPointer++)
    {
      m_ptrPowerElecPackInfoConfig->cellModuleVoltages[modulePointer][cellPointer] = cellVoltagesMeasIndividual[(modulePointer * NoOfCELL_POSSIBLE_ON_CHIP) + cellPointer];
      m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[(modulePointer * NoOfCELL_POSSIBLE_ON_CHIP) + cellPointer].u16CellVoltage = cellVoltagesMeasIndividual[(modulePointer * NoOfCELL_POSSIBLE_ON_CHIP) + cellPointer]; // + m_ptrGenConfig->i16CellVoltagesOffsetVoltage
    }
  }

  // Preserve last clean min/max/dv while frozen; cells still refreshed above.
  if (freeze)
    return;

  // Use cached clean values during balancing ACTIVE phase (bleeders on) to keep UI clean,
  // even before the external bBalancingActive flag is latched.
  const bool useCached = (!isBalanceMeasureWindow());
  if (useCached && m_balanceState.cachedVmax >= m_balanceState.cachedVmin && m_balanceState.cachedVmax != 0)
  {
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax = m_balanceState.cachedVmax;
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin = m_balanceState.cachedVmin;
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch =
        m_balanceState.cachedVmax - m_balanceState.cachedVmin;
    // Average from cached extremes is lossy; keep prior average when using clean snapshot.
    return;
  }

  m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax = 0;
  m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin = 10000;

  // Compute fresh min/max/mismatch (and average) immediately after the copy so
  // external readers never see the reset 10000/0 placeholders.
  const bool maskMin = BalancingArgs::FILTER_MIN_ADJACENT_TO_BLEED &&
                       m_ptrPowerElecPackInfoConfig->bBalancingActive;
  const uint8_t maxMode = BalancingArgs::FILTER_MAX_MODE;
  const bool maskMax = (maxMode != 0u) && m_ptrPowerElecPackInfoConfig->bBalancingActive;
  const bool maskMaxNeighbors = (maxMode >= 2u);

  uint8_t nModules = m_ptrGenConfig->u8NoOfCellsPerModule;
  uint8_t nCells = m_ptrGenConfig->u8NumberOfCells;
  if (nModules > NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
    nModules = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
  if (nCells > NoOfCELL_POSSIBLE_ON_CHIP)
    nCells = NoOfCELL_POSSIBLE_ON_CHIP;
  const uint16_t totalCells = static_cast<uint16_t>(nModules) * nCells;

  if (totalCells > 0)
  {
    uint16_t minAll = 10000;
    uint16_t maxAll = 0;
    uint16_t minMasked = 10000;
    uint16_t maxMasked = 0;

    for (uint8_t modulePointer = 0; modulePointer < nModules; ++modulePointer)
    {
      for (uint8_t cellPointer = 0; cellPointer < nCells; ++cellPointer)
      {
        const size_t idx =
            (static_cast<size_t>(modulePointer) * NoOfCELL_POSSIBLE_ON_CHIP) + cellPointer;
        const uint16_t v = m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[idx].u16CellVoltage;
        _u32CellVoltagesSummed += v;
        if (v > maxAll)
          maxAll = v;
        if (v < minAll)
          minAll = v;
        if (!maskMin || !isCellMaskedByBleed(m_ptrPowerElecPackInfoConfig, modulePointer, cellPointer, nCells, true))
        {
          if (v < minMasked)
            minMasked = v;
        }
        if (!maskMax || !isCellMaskedByBleed(m_ptrPowerElecPackInfoConfig, modulePointer, cellPointer, nCells, maskMaxNeighbors))
        {
          if (v > maxMasked)
            maxMasked = v;
        }
      }
    }

    const uint16_t finalMin = (maskMin && minMasked != 10000) ? minMasked : minAll;
    const uint16_t finalMax = (maskMax && maxMasked != 0) ? maxMasked : maxAll;
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin = finalMin;
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax = finalMax;
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageAverage =
        (uint16_t)(_u32CellVoltagesSummed / totalCells);
    m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch =
        finalMax - finalMin;
  }

  // if(m_ptrPowerElecPackInfoConfig->bmsState == BMS_DISCHARGE || m_ptrPowerElecPackInfoConfig->bmsState == BMS_CHARGE) {
  //   _fActualCurrent = ((float)(m_ptrPowerElecPackInfoConfig->i32ModuleCurrent) / 1000.0f) * ((float)(m_ptrGenConfig->u16InternalResistance) / 1000.0f);
  //   m_ptrPowerElecPackInfoConfig->u32ModuleVoltage = (uint16_t)((float)(m_ptrPowerElecPackInfoConfig->u32ModuleVoltage) + _fActualCurrent);
  //   for (uint8_t cellPointer = 0; cellPointer < m_ptrGenConfig->u8NumberOfCells; cellPointer++) {
  //     m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[cellPointer].u16CellVoltage = (uint16_t)((float)(m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[cellPointer].u16CellVoltage) + _fActualCurrent);
  //   }
  // }
}

// #############################################################################################################################################################################

void Measurement::subTaskAllModulePackTemperatureWatch(void)
{

  int32_t _i32TempBatteryMax;
  int32_t _i32TempBatteryMin;
  int32_t _i32TempBatterySum = 0;
  uint8_t _u8TempBatterySumCount = 0;

  _i32TempBatteryMax = -100000;
  _i32TempBatteryMin = 100000;

  for (uint8_t modulePointer = 0; modulePointer < m_ptrGenConfig->u8NoOfCellsPerModule; modulePointer++)
  {
    for (uint8_t tempPointer = 0; tempPointer < m_ptrGenConfig->u8NumberOfTemperature; tempPointer++)
    {
      m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature[modulePointer][tempPointer] = tempValuesMeasIndividual[(modulePointer * NoOfTEMP_POSSIBLE_ON_CHIP) + tempPointer];

      int32_t _u32TemperatureTempVal = m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature[modulePointer][tempPointer];

      _i32TempBatterySum += _u32TemperatureTempVal;
      _u8TempBatterySumCount++;

      if (_u32TemperatureTempVal > _i32TempBatteryMax)
        _i32TempBatteryMax = _u32TemperatureTempVal;
      if (_u32TemperatureTempVal < _i32TempBatteryMin)
        _i32TempBatteryMin = _u32TemperatureTempVal;
    }
  }

  m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax = _i32TempBatteryMax;
  m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin = _i32TempBatteryMin;
  m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMismatch = _i32TempBatteryMax - _i32TempBatteryMin;
  if (_u8TempBatterySumCount)
    m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureAverage = (int32_t)((float)_i32TempBatterySum / (float)_u8TempBatterySumCount);
  else
    m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureAverage = 0;
}

// #############################################################################################################################################################################

void Measurement::subTaskCheckInverterResponse(void)
{
  const uint32_t INVERTER_RESPONSE_TIMEOUT_MS = 3000; // allow longer ramp-down before faulting
  static uint32_t _u32LastResendTick = 0;
  static uint8_t _u8ResendCount = 0;

  // This logic is only active when we are waiting for a response.
  if (m_ptrPowerElecPackInfoConfig->inverterResponseState == WAITING_FOR_RESPONSE)
  {
    // Proactively re-send the "stop/limit" command while waiting, to avoid a missed frame.
    if (timerDelay1ms(&_u32LastResendTick, INVERTER_STOP_CMD_RESEND_MS) && _u8ResendCount < INVERTER_STOP_CMD_MAX_RESENDS)
    {
      m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 0);
      logChargeClampStatic("INV_WAIT_RESEND");
      _u8ResendCount++;
    }

    // Check if the inverter has successfully reduced the current.
    if (m_ptrPowerElecPackInfoConfig->i32ModuleCurrent > -2000) // treat <2A charge as success
    {
      // Success. The inverter responded. Return to normal state.
      m_ptrPowerElecPackInfoConfig->inverterResponseState = INVERTER_OK;
      _u8ResendCount = 0;
      InterfaceCommHandler::getInstance()->printToInterface("--- Inverter responded correctly to charge stop command ---\r\n");
    }
    // Check if the timeout has elapsed.
    else if (timerDelay1ms(&m_ptrPowerElecPackInfoConfig->u32InverterCommandTimestamp, INVERTER_RESPONSE_TIMEOUT_MS))
    {
      // Failure. The inverter did not respond in time.
      m_ptrPowerElecPackInfoConfig->inverterResponseState = INVERTER_FAULT;
      _u8ResendCount = 0;
      InterfaceCommHandler::getInstance()->printToInterface("--- FATAL: Inverter FAILED to respond to charge stop command! ---\r\n");
    }
  }
  else
  {
    _u8ResendCount = 0; // not waiting
  }
}
// #############################################################################################################################################################################
void Measurement::subTaskOvRecoverySupervisor(void)
{
  // Sticky latches for clamp/dwell
  static bool s_stopped = false;    // we sent 0 A limit (soft clamp)
  static uint32_t s_dwellStart = 0; // timer to re-enable 2 A trickle
  static uint32_t s_hardStart = 0;  // dwell timer for hard OVP disconnect
  static bool s_inverterCut = false; // latched inverter disconnect due to severe OV
  static uint8_t s_softOvCount = 0;
  static uint8_t s_hardOvCount = 0;
  static uint32_t s_softOvSampleTick = 0;
  static uint32_t s_hardOvSampleTick = 0;

  const uint16_t vmax = getOvVmaxEffective(); // mV
  const uint16_t soft = m_ptrGenConfig->u16SoftOverVoltage;                    // mV (e.g., 3550)
  const uint16_t hard = m_ptrGenConfig->u16HardOverVoltage;                    // mV (e.g., 3600)
  const uint16_t HYST_mV = 30;                                                 // mV to avoid chatter
  // Requested explicit thresholds
  const uint16_t SOFT_STOP_mV = 3650;                                          // stop charging at 3.65 V
  const uint16_t HARD_ESCALATE_mV = 3700;                                      // cut inverter at 3.70 V if current not near 0
  const uint8_t OVP_CONSECUTIVE_SAMPLES = 3;                                   // require consecutive samples before clamp
  const uint32_t OVP_SAMPLE_GAP_MS = 200;                                      // minimum time between samples
  const int32_t NEAR_ZERO_mA = 100;                                            // "+/-100 mA" is near 0
  const int32_t I_mA = m_ptrPowerElecPackInfoConfig->i32ModuleCurrent;         // signed (neg = charge)
  const uint32_t DWELL_MS = 5000;                                              // 5 s below (soft - HYST) to resume 2 A
  const bool chargingNow = (I_mA <= BalancingArgs::CHG_ACTIVE_MA);
  const bool ovpChargeGuard = !m_ptrPowerElecPackInfoConfig->bBalancingActive || chargingNow;

  // 0) Immediate soft-stop at 3.65 V regardless of OV state
  {
    static uint32_t s_lastSoftCmd = 0;
    if (vmax >= SOFT_STOP_mV && ovpChargeGuard)
    {
      if (timerDelay1ms(&s_softOvSampleTick, OVP_SAMPLE_GAP_MS))
      {
        if (s_softOvCount < OVP_CONSECUTIVE_SAMPLES)
          s_softOvCount++;
      }
      if (timerDelay1ms(&s_lastSoftCmd, 1000))
      {
        if (s_softOvCount >= OVP_CONSECUTIVE_SAMPLES)
        {
          m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 0);
          m_ptrPowerElecPackInfoConfig->inverterResponseState = WAITING_FOR_RESPONSE;
          m_ptrPowerElecPackInfoConfig->u32InverterCommandTimestamp = MEASUREMENT_GET_TICK(m_measurementManagerTim);
          m_hwMeasurement.interfaceComm.printToInterface("[OVP] vmax=%u mV >= 3650 -> command 0A\r\n", (unsigned)vmax);
          // Track that we sent a stop; s_stopped is also used in RECOVERY path
          s_stopped = true;
          logChargeClampStatic("OVP_SOFTSTOP");
        }
      }
    }
    else
    {
      s_softOvCount = 0;
      s_softOvSampleTick = 0;
    }
  }

  // 0.5) Escalate to inverter cut at 3.70 V if current hasn't dropped near 0
  if (!s_inverterCut && vmax >= HARD_ESCALATE_mV)
  {
    if (timerDelay1ms(&s_hardOvSampleTick, OVP_SAMPLE_GAP_MS))
    {
      if (s_hardOvCount < OVP_CONSECUTIVE_SAMPLES)
        s_hardOvCount++;
    }
    const bool stillCharging = (I_mA <= -NEAR_ZERO_mA); // negative = charging
    if (stillCharging && s_hardOvCount >= OVP_CONSECUTIVE_SAMPLES)
    {
      s_inverterCut = true;
      m_ptrPowerElecPackInfoConfig->bChargeAllowed = false; // policy flag -> switches logic will open charge path
      m_ptrPowerElecPackInfoConfig->packOperationCellState = PACK_STATE_ERROR_HARD_CELLVOLTAGE;
      m_hwMeasurement.interfaceComm.printToInterface(
          "[OVP] vmax=%u mV >= 3700 and I=%ld mA (not ~0) -> disconnect inverter\r\n",
          (unsigned)vmax, (long)I_mA);
    }
  }
  else if (s_inverterCut && vmax <= (uint16_t)(SOFT_STOP_mV - HYST_mV))
  {
    // Clear the latch when we have backed off below soft-HYST; policy will re-allow elsewhere
    s_inverterCut = false;
    s_hardOvCount = 0;
    s_hardOvSampleTick = 0;
  }
  else if (vmax < HARD_ESCALATE_mV)
  {
    s_hardOvCount = 0;
    s_hardOvSampleTick = 0;
  }

  // While charging, report cells crossing soft/hard OVP once per second
  if (m_ptrPowerElecPackInfoConfig->bmsState == BMS_CHARGE)
  {
    static uint32_t s_lastOvPrint = 0;
    if (timerDelay1ms(&s_lastOvPrint, 1000))
    {
      // List hard OVP cells first
      bool anyHard = false;
      for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
      {
        bool anyInPack = false;
        for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
        {
          const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
          const uint16_t v = m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[idx].u16CellVoltage;
          if (v >= hard)
          {
            if (!anyHard)
            {
              anyHard = true;
              m_hwMeasurement.interfaceComm.printToInterface("[OVP] HARD:");
            }
            if (!anyInPack)
            {
              anyInPack = true;
              m_hwMeasurement.interfaceComm.printToInterface(" Pack %u:", (unsigned)pack);
            }
            m_hwMeasurement.interfaceComm.printToInterface(" C%u=%u", (unsigned)(cell + 1), (unsigned)v);
          }
        }
      }
      if (anyHard)
        m_hwMeasurement.interfaceComm.printToInterface(" \r\n");

      // Then list soft OVP (exclude those already at hard)
      bool anySoft = false;
      for (size_t pack = 0; pack < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++pack)
      {
        bool anyInPack = false;
        for (size_t cell = 0; cell < NoOfCELL_POSSIBLE_ON_CHIP; ++cell)
        {
          const size_t idx = pack * NoOfCELL_POSSIBLE_ON_CHIP + cell;
          const uint16_t v = m_ptrPowerElecPackInfoConfig->cellVoltagesIndividual[idx].u16CellVoltage;
          if (v >= soft && v < hard)
          {
            if (!anySoft)
            {
              anySoft = true;
              m_hwMeasurement.interfaceComm.printToInterface("[OVP] SOFT:");
            }
            if (!anyInPack)
            {
              anyInPack = true;
              m_hwMeasurement.interfaceComm.printToInterface(" Pack %u:", (unsigned)pack);
            }
            m_hwMeasurement.interfaceComm.printToInterface(" C%u=%u", (unsigned)(cell + 1), (unsigned)v);
          }
        }
      }
      if (anySoft)
        m_hwMeasurement.interfaceComm.printToInterface(" \r\n");
    }
  }

  // 1) Hard OVP dwell → physically disconnect charge after 5 s
  if (vmax >= hard)
  {
    if (s_hardStart == 0)
      s_hardStart = MEASUREMENT_GET_TICK(m_measurementManagerTim);
    if (timerDelay1ms(&s_hardStart, 5000))
    {
      // Open charge path via policy flag; Operation / switches reflect this
      m_ptrPowerElecPackInfoConfig->bChargeAllowed = false;
      m_ptrPowerElecPackInfoConfig->packOperationCellState = PACK_STATE_ERROR_HARD_CELLVOLTAGE;
      // keep OV state latched
    }
  }
  else
  {
    s_hardStart = 0; // no hard OVP now
  }

  // 2) Soft OVP clamp (only when in RECOVERY): command 0 A but keep contactors closed
  if (m_ptrPowerElecPackInfoConfig->overvoltageState == OV_STATE_RECOVERY)
  {
    if (!ovpChargeGuard)
    {
      s_stopped = false;
      s_dwellStart = 0;
    }
    else if (vmax >= soft)
    {
      if (!s_stopped)
      {
        m_hwMeasurement.inverterCan.updateMaxChargeLimit(m_ptrGenConfig->u16MaxChargePackVoltage, 0);
        logChargeClampStatic("OVP_RECOVERY_STOP");
        m_ptrPowerElecPackInfoConfig->inverterResponseState = WAITING_FOR_RESPONSE;
        m_ptrPowerElecPackInfoConfig->u32InverterCommandTimestamp = MEASUREMENT_GET_TICK(m_measurementManagerTim);
        s_stopped = true;
      }
      s_dwellStart = 0; // reset dwell while above soft
    }
    else if (s_stopped)
    {
      // Below soft: after dwell at (soft - HYST), restore 2 A trickle
      if (vmax <= (uint16_t)(soft - HYST_mV))
      {
        if (s_dwellStart == 0)
          s_dwellStart = MEASUREMENT_GET_TICK(m_measurementManagerTim);
        if (timerDelay1ms(&s_dwellStart, DWELL_MS))
        {
          m_hwMeasurement.inverterCan.updateMaxChargeLimit(
              m_ptrGenConfig->u16MaxChargePackVoltage, BalancingArgs::THROTTLE_CURRENT_MA);
          logChargeClampStatic("OVP_TRICKLE_RESTORE");
          s_stopped = false;
          s_dwellStart = 0;
        }
      }
      else
      {
        s_dwellStart = 0; // not yet below threshold; keep waiting
      }
    }

    // While in recovery and not in soft-stop, keep reasserting the trickle limit
    // so other limit writers cannot raise current back to hard max.
    if (!s_stopped &&
        m_ptrPowerElecPackInfoConfig->inverterResponseState != WAITING_FOR_RESPONSE &&
        ovpChargeGuard &&
        I_mA <= BalancingArgs::CHG_ACTIVE_MA)
    {
      static uint32_t s_recoveryReassertMs = 0;
      if (timerDelay1ms(&s_recoveryReassertMs, 1000))
      {
        m_hwMeasurement.inverterCan.updateMaxChargeLimit(
            m_ptrGenConfig->u16MaxChargePackVoltage, BalancingArgs::THROTTLE_CURRENT_MA);
        logChargeClampStatic("OVP_RECOVERY_REASSERT");
      }
    }

    // RECOVERY->NORMAL handoff: when below soft-HYST and dv is small for a dwell, restore full charge
    {
      static uint32_t s_recovNormStart = 0;
      const uint16_t vmin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin; // mV
      const uint16_t dv_mV = getDvMvCleanAware();
      const uint16_t DV_EXIT_mV = 180; // must align with pack mismatch exit
      if (vmax <= (uint16_t)(soft - HYST_mV) && dv_mV <= DV_EXIT_mV)
      {
        if (s_recovNormStart == 0)
          s_recovNormStart = MEASUREMENT_GET_TICK(m_measurementManagerTim);
        if (timerDelay1ms(&s_recovNormStart, DWELL_MS))
        {
          m_ptrPowerElecPackInfoConfig->overvoltageState = OV_STATE_NORMAL;
          // Restore full charging current limit now that cells are balanced enough
          m_hwMeasurement.inverterCan.updateMaxChargeLimit(
              m_ptrGenConfig->u16MaxChargePackVoltage,
              derateChargeCurrent_cA(m_ptrGenConfig->u16MaxHardChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig));
          const float restoredLimitA =
              derateChargeCurrent_cA(m_ptrGenConfig->u16MaxHardChgAllowedCurrent, m_ptrGenConfig, m_ptrPowerElecPackInfoConfig) / 100.0f;
          m_hwMeasurement.interfaceComm.printToInterface(
              "[OVP] Charge limit cleared -> %.2fA\r\n",
              (double)restoredLimitA);
          // Clear soft/hard OV warning once recovered to stop lingering LED/buzzer indications
          if (m_ptrPowerElecPackInfoConfig->packBatteryWarErrState == SYS_ERROR_CELL_OVER_VOLTAGE)
          {
            m_ptrPowerElecPackInfoConfig->packBatteryWarErrState = SYS_OK;
            m_ptrPowerElecPackInfoConfig->packLastErrorState = SYS_OK;
          }
          s_recovNormStart = 0;
        }
      }
      else
      {
        s_recovNormStart = 0;
      }
    }
  }
}
// #############################################################################################################################################################################
void Measurement::subTaskSysPowerWatch(void)
{
  m_ptrPowerElecPackInfoConfig->u16SupplyVoltage = m_hwMeasurement.pwr.getSysVoltage();
  m_ptrPowerElecPackInfoConfig->u32ModuleLoadVoltage = m_hwMeasurement.pwr.getLoadVoltage();

  m_bPowerElecAllowForcedOnState = m_hwMeasurement.pwr.getForceOnState();
}

// #############################################################################################################################################################################

void Measurement::subTaskDatetimeWatch(void)
{
  static uint32_t _u32AllowedDateAndTimeTick = 0;

  if (timerDelay1ms(&_u32AllowedDateAndTimeTick, 1000))
  {
    m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysSecond = m_hwMeasurement.datetime.getSecond();
    m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysMinute = m_hwMeasurement.datetime.getMinute();
    m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysHour = m_hwMeasurement.datetime.getHour();
    m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysDay = m_hwMeasurement.datetime.getDay();
    m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysMonth = m_hwMeasurement.datetime.getMonth();
    m_ptrPowerElecPackInfoConfig->datetimeInfo.u8SysYear = m_hwMeasurement.datetime.getYear();
  }
}

// #############################################################################################################################################################################

void Measurement::subTaskChipsetErrorWatch(void)
{
  // m_ptrGenConfig->u8ErrorDetected     = m_hwMeasurement.bq.getInternalError();

  // Operation sets bPushedShutdownChipSet in OP_STATE_POWER_DOWN. Issue the shutdown command once per
  // power-down request (Operation may keep the flag asserted every loop).
  static bool s_shutdownIssued = false;
  if (m_ptrPowerElecPackInfoConfig->bPushedShutdownChipSet)
  {
    if (!s_shutdownIssued)
    {
      s_shutdownIssued = true;
      m_hwMeasurement.bq.shutdown();
    }
  }
  else
  {
    s_shutdownIssued = false;
  }
}

// #############################################################################################################################################################################

void Measurement::subTaskPackGeneralWatch(void)
{
  for (uint8_t modulePointer = 0; modulePointer < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; modulePointer++)
  {
    // m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u8arrPackSerialId        = m_u8arrMeasPartId[modulePointer];
    m_ptrPowerElecPackInfoConfig->moduleMonitorParams[modulePointer].u8arrPackBalanceState = m_u8arrMeasBalanceState[modulePointer];
  }
}

// #############################################################################################################################################################################

void Measurement::subTaskBuzzer(void)
{
  // determin whether buzzer should sound
  switch (m_ptrPowerElecPackInfoConfig->buzzerSignalSource)
  {
  case BUZZER_SOURCE_OFF:
    m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = false;
    break;
  case BUZZER_SOURCE_ON:
    m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = true;
    m_ptrPowerElecPackInfoConfig->buzzerSignalType = BUZZER_SIGNAL_TYPE_ON;

    break;
  case BUZZER_SOURCE_CELL_UNDERVOLTAGE:
    m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = true;
    m_ptrPowerElecPackInfoConfig->buzzerSignalType = BUZZER_SIGNAL_TYPE_TOGGLE_FAST;
    break;
  case BUZZER_SOURCE_CELL_OVERVOLTAGE:
    m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = true;
    m_ptrPowerElecPackInfoConfig->buzzerSignalType = BUZZER_SIGNAL_TYPE_TOGGLE;
    break;
  case BUZZER_SOURCE_CELL_DIFF_OVERVOLTAGE:
    m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = true;
    m_ptrPowerElecPackInfoConfig->buzzerSignalType = BUZZER_SIGNAL_TYPE_ON;
    break;
  default:
    m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = false;
    break;
  }
}

// ##################################################################################################################

void Measurement::subTaskProtectionsControllerMechanism(void)
{
  // --- THIS IS THE CENTRALIZED DEBUGGING FIX ---
  // We loop through every protection and check if its state has changed.
  static uint32_t s_lastStateChangeMs[NoOfPROTS_POSSIBLE_ON_SYS] = {};
  for (uint8_t i = 0; i < NoOfPROTS_POSSIBLE_ON_SYS; i++)
  {
    if (m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].bLoggingProtectionActive &&
        (m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].lastProtectionState != m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].protectionState))
    {
      const uint32_t now = MEASUREMENT_GET_TICK(m_measurementManagerTim);
      log_protection_transition(
          i,
          m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].protectionState,
          now,
          s_lastStateChangeMs[i]);
      s_lastStateChangeMs[i] = now;

      // Update the last state so we don't print the same message repeatedly.
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].lastProtectionState = m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].protectionState;
    }
  }

  for (uint8_t protPointer = 0; protPointer < PROTECTIONS_TYPE_NUMBER; protPointer++)
  {
    if ((st_protections_params[protPointer].protectionsDetected()) &&
        (st_protections_params[protPointer].bProtectionsMeasurementDetectionFlag))
    {

      st_protections_params[protPointer].u8ProtectionsCount++;

      if (st_protections_params[protPointer].u8ProtectionsCount > st_protections_params[protPointer].ku8ConsecutiveReadingsThreshold)
      {
        st_protections_params[protPointer].bProtectionsMeasurementDetectionFlag = false;
        st_protections_params[protPointer].bProtectionsTimeoutFlag = true;
        st_protections_params[protPointer].u8ProtectionsCount = 0;
        // CORRECTED: Use the Mbed timer, not the HAL timer.
        st_protections_params[protPointer].u32ProtectionsDetectionTimeout = MEASUREMENT_GET_TICK(m_measurementManagerTim);
        m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].protectionState = PROT_STATE_DETECT_TIMEOUT_START;
        m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].bLoggingProtectionActive = true;
      }
      else
      {
        // CORRECTED: Use the Mbed timer, not the HAL timer.
        st_protections_params[protPointer].u32ProtectionsDetectionCountingTimeout = MEASUREMENT_GET_TICK(m_measurementManagerTim);
      }
    }

    if ((timerDelay1ms(&st_protections_params[protPointer].u32ProtectionsDetectionCountingTimeout, st_protections_params[protPointer].ku32DetectionCountingTimeoutMs)) &&
        (st_protections_params[protPointer].bProtectionsMeasurementDetectionFlag) &&
        (st_protections_params[protPointer].u8ProtectionsCount > 0))
    {
      st_protections_params[protPointer].u8ProtectionsCount = 0;
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].protectionState = PROT_STATE_COUNT_TIMEOUT;
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].bLoggingProtectionActive = true;
    }

    if ((timerDelay1ms(&st_protections_params[protPointer].u32ProtectionsDetectionTimeout, st_protections_params[protPointer].ku32DetectionTimeoutMs)) &&
        (st_protections_params[protPointer].bProtectionsTimeoutFlag))
    {
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].protectionState = PROT_STATE_DETECT;
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].bLoggingProtectionActive = true;
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].bMeasurementProtectionActive = true;
      st_protections_params[protPointer].protectionsDetectedCallback();
    }

    if ((st_protections_params[protPointer].protectionsReleased()) &&
        (st_protections_params[protPointer].bProtectionsTimeoutFlag))
    {
      st_protections_params[protPointer].bProtectionsTimeoutFlag = false;
      st_protections_params[protPointer].bProtectionsMeasurementDetectionFlag = true;
      st_protections_params[protPointer].bProtectionsReleaseFlag = true;
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].u8ProtectionDetectionCounter++;
      // CORRECTED: Use the Mbed timer, not the HAL timer.
      st_protections_params[protPointer].u32ProtectionsProtectionReleaseTimeout = MEASUREMENT_GET_TICK(m_measurementManagerTim);
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].protectionState = PROT_STATE_RELEASE_TIMEOUT_START;
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].bOperationProtectionActive = true;
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].bLoggingProtectionActive = true;
    }

    if ((timerDelay1ms(&st_protections_params[protPointer].u32ProtectionsProtectionReleaseTimeout, st_protections_params[protPointer].ku32ReleaseTimeoutMs)) &&
        (st_protections_params[protPointer].protectionsReleased()) &&
        (st_protections_params[protPointer].bProtectionsReleaseFlag))
    {
      st_protections_params[protPointer].protectionsReleasedCallback();
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].protectionState = PROT_STATE_RELEASE;
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].bLoggingProtectionActive = true;
      // CORRECTED: Use the Mbed timer, not the HAL timer.
      st_protections_params[protPointer].u32ProtectionsClearTimeout = MEASUREMENT_GET_TICK(m_measurementManagerTim);
      st_protections_params[protPointer].bProtectionsReleaseFlag = false;
    }

    if ((m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].protectionState == PROT_STATE_RELEASE) &&
        (timerDelay1ms(&st_protections_params[protPointer].u32ProtectionsClearTimeout, st_protections_params[protPointer].ku32ProtectionStateClearTimeoutMs)))
    {
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].protectionState = PROT_STATE_OK;
      if (protPointer == MODULE_OVER_CURRENT_DISCHARGE || protPointer == MODULE_OVER_CURRENT_CHARGE)
      {
        m_ptrPowerElecPackInfoConfig->moduleProtectionParams[protPointer].u8ProtectionDetectionCounter = 0;
      }
    }
  }
}

// ##################################################################################################################

// --- Cell Undervoltage ---
static bool bqCellVoltageValid(void)
{
  if (!g_ptrPowerElecPackInfoConfig)
    return false;
  const bool in_grace = (BQ_COMM_BOOT_GRACE_MS > 0) &&
                        (uptime_ms() < BQ_COMM_BOOT_GRACE_MS);
  if (in_grace)
  {
    if (g_ptrPowerElecPackInfoConfig->u32LastBqDataTimestamp == 0)
      return false;
    if (g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin == 0)
      return false;
  }
  return true;
}

static bool sysUVDetected(void)
{
  if (!bqCellVoltageValid())
    return false;
  return (g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin <= g_ptrGenConfig->u16HardUnderVoltage);
}
static void sysUVDetectedCallback(void)
{

  g_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed = false; // Stop discharging
  g_ptrPowerElecPackInfoConfig->bDischargeHighCurrentAllowed = false;
  g_hwMeasurement->inverterCan.updateMaxDischargeLimit(
      g_ptrGenConfig->u16MaxDischargePackVoltage, 0);
  g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_CELL_UNDERVOLTAGE;
}
static bool sysUVReleased(void)
{
  if (!bqCellVoltageValid())
    return false;
  return (g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin >= (g_ptrGenConfig->u16HardUnderVoltage + g_ptrGenConfig->u16HysteresisDischarge));
}
// This is the key fix for the buzzer issue.
static void sysUVReleasedCallback(void)
{
  g_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed = true; // Allow discharging
  g_ptrPowerElecPackInfoConfig->bDischargeHighCurrentAllowed = true;
  g_hwMeasurement->inverterCan.updateMaxDischargeLimit(
      g_ptrGenConfig->u16MaxDischargePackVoltage,
      g_ptrGenConfig->u16MaxHardDchgAllowedCurrent);
  // --- THIS IS THE FIX ---
  // Explicitly turn the buzzer off when the undervoltage condition clears.
  g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_OFF;
}

// ##################################################################################################################

// --- Cell Overvoltage ---
static uint16_t getOvVmaxEffective(void)
{
  if (!g_ptrPowerElecPackInfoConfig)
    return 0;
  const int32_t I_mA = g_ptrPowerElecPackInfoConfig->i32ModuleCurrent;
  const bool chargingNow = (I_mA <= BalancingArgs::CHG_ACTIVE_MA);
  if (chargingNow)
  {
    const uint16_t rawVmax = g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxRaw;
    if (rawVmax != 0)
      return rawVmax;
  }
  const uint16_t cleanVmax = g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxClean;
  if (cleanVmax != 0)
    return cleanVmax;
  return g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax;
}

static bool sysOVDetected(void)
{
  // Trigger on hard OVP regardless of state, or on soft OVP only from NORMAL state.
  const uint16_t vmax = getOvVmaxEffective();
  const uint16_t soft = g_ptrGenConfig->u16SoftOverVoltage;
  const uint16_t hard = g_ptrGenConfig->u16HardOverVoltage;
  if (vmax >= hard)
    return true; // immediate safety re-trip allowed in any state
  const int32_t I_mA = g_ptrPowerElecPackInfoConfig->i32ModuleCurrent;
  const bool chargingNow = (I_mA <= BalancingArgs::CHG_ACTIVE_MA);
  if (g_ptrPowerElecPackInfoConfig->bBalancingActive && !chargingNow)
    return false;
  return (vmax >= soft && g_ptrPowerElecPackInfoConfig->overvoltageState == OV_STATE_NORMAL);
}

static void sysOVDetectedCallback(void)
{
  // ACTION: Overvoltage has been detected.
  // Do not open contactors at the soft threshold; we only command 0 A here.

  // 1. Send the command to the inverter to stop charging.
  g_hwMeasurement->inverterCan.updateMaxChargeLimit(g_ptrGenConfig->u16MaxChargePackVoltage, 0);
  logChargeClampStatic("SYS_OV_DETECTED");

  // 2. Start the timer and set the state to "waiting".
  g_ptrPowerElecPackInfoConfig->inverterResponseState = WAITING_FOR_RESPONSE;
  // Use the new, valid global pointer to the timer.
  g_ptrPowerElecPackInfoConfig->u32InverterCommandTimestamp = MEASUREMENT_GET_TICK((*g_measurementManagerTim));

  // 3. Transition to the "tripped" state for balancing.
  g_ptrPowerElecPackInfoConfig->overvoltageState = OV_STATE_TRIPPED;
  g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_CELL_OVERVOLTAGE;
  g_hwMeasurement->serial.serialPrint("--- PROTECTION: Cell Overvoltage DETECTED ---\r\n");
}

static bool sysOVReleased(void)
{
  // Recovery condition: Highest cell voltage has dropped 50mV below the HARD limit AND we are in the "tripped" state.
  return (getOvVmaxEffective() <= (g_ptrGenConfig->u16HardOverVoltage - 50) &&
          g_ptrPowerElecPackInfoConfig->overvoltageState == OV_STATE_TRIPPED);
}

static void sysOVReleasedCallback(void)
{
  // ACTION: Voltage has recovered. Time to start trickle charging and balancing.
  // 1. Allow charging again.
  g_ptrPowerElecPackInfoConfig->bChargeAllowed = true;

  // 2. Set the CAN charge current limit to the low "throttling" value.
  g_hwMeasurement->inverterCan.updateMaxChargeLimit(
      g_ptrGenConfig->u16MaxChargePackVoltage,
      BalancingArgs::THROTTLE_CURRENT_MA);
  const float throttleLimitA = BalancingArgs::THROTTLE_CURRENT_MA / 100.0f;
  g_hwMeasurement->interfaceComm.printToInterface(
      "[OVP] Charge limit applied: %.2fA (recovery)\r\n",
      (double)throttleLimitA);

  // 3. Transition to the "recovery" state.
  g_ptrPowerElecPackInfoConfig->overvoltageState = OV_STATE_RECOVERY;
  g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_OFF;
}

// ##################################################################################################################

// --- Cell Differential Overvoltage ---
static bool sysDOVDetected(void)
{
  return (g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch >= g_ptrGenConfig->u16OverMismatchVoltage);
}
static void sysDOVDetectedCallback(void)
{
  // InterfaceCommHandler::getInstance()->printToInterface("--- PROTECTION TRIGGERED: Cell Mismatch ---\r\n");

  // CORRECTED: This protection should only signal that balancing is needed.
  // It should NOT disable charging. The overvoltage protection will handle that.
  // g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_CELL_DIFF_OVERVOLTAGE;
}
static bool sysDOVReleased(void)
{
  return (g_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch <= (g_ptrGenConfig->u16OverMismatchVoltage - g_ptrGenConfig->u16HysteresisOverMismatchVoltage));
}
static void sysDOVReleasedCallback(void)
{
  // When the mismatch is resolved, allow normal operation and turn off the buzzer.
  g_ptrPowerElecPackInfoConfig->bChargeAllowed = true;
  g_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed = true;
  g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_OFF;
}

// ##################################################################################################################

// --- Module Overvoltage ---
static bool sysModuleOVDetected(void)
{
  static uint32_t _u32ModuleOVThreshold = (g_ptrGenConfig->u8NoOfCellsPerModule * g_ptrGenConfig->u8NumberOfCells * g_ptrGenConfig->u16HardOverVoltage) + ProtectionConstants::MODULE_VOLTAGE_MARGIN_MV;
  return (g_ptrPowerElecPackInfoConfig->u32ModuleVoltage >= _u32ModuleOVThreshold);
}
static void sysModuleOVDetectedCallback(void)
{
  g_ptrPowerElecPackInfoConfig->bChargeAllowed = false;
  g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_MODULE_OVERVOLTAGE;
}
static bool sysModuleOVReleased(void)
{
  static uint32_t _u32ModuleOVThreshold = (g_ptrGenConfig->u8NoOfCellsPerModule * g_ptrGenConfig->u8NumberOfCells * g_ptrGenConfig->u16HardOverVoltage) + ProtectionConstants::MODULE_VOLTAGE_MARGIN_MV;
  return (g_ptrPowerElecPackInfoConfig->u32ModuleVoltage <= (_u32ModuleOVThreshold - ProtectionConstants::MODULE_VOLTAGE_MARGIN_MV));
}
static void sysModuleOVReleasedCallback(void)
{
  g_ptrPowerElecPackInfoConfig->bChargeAllowed = true;
  g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_OFF;
}
// ##################################################################################################################

// --- Discharge Overcurrent ---
static bool sysDchgOCDetected(void)
{
  return (g_ptrPowerElecPackInfoConfig->i32ModuleCurrent >= (g_ptrGenConfig->u16MaxHardDchgAllowedCurrent * 12.0));//%20 marge for prevent false trigger action
  // return (false); // Temporarily disable discharge overcurrent protection for testing
}
static void sysDchgOCDetectedCallback(void)
{
  InterfaceCommHandler::getInstance()->printToInterface("--- i32ModuleCurrent = %d  ---\r\n", g_ptrPowerElecPackInfoConfig->i32ModuleCurrent);
  g_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed = false;
  g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_OVERCCURRENT;
}
static bool sysDchgOCReleased(void)
{
  return (g_ptrPowerElecPackInfoConfig->i32ModuleCurrent <= ((g_ptrGenConfig->u16MaxSoftDchgAllowedCurrent * 12.0) - g_ptrGenConfig->u16HysteresisCurrent));
}
static void sysDchgOCReleasedCallback(void)
{
  g_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed = true;
  g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_OFF;
}

// --- Charge Overcurrent ---
static bool sysChgOCDetected(void)
{
  return (g_ptrPowerElecPackInfoConfig->i32ModuleCurrent <= -(g_ptrGenConfig->u16MaxSoftChgAllowedCurrent * 12.0)); // Note: The current is negative for charging
}
static void sysChgOCDetectedCallback(void)
{
  g_ptrPowerElecPackInfoConfig->bChargeAllowed = false;
  g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_OVERCCURRENT;
}
static bool sysChgOCReleased(void)
{
  return (g_ptrPowerElecPackInfoConfig->i32ModuleCurrent >= (-(g_ptrGenConfig->u16MaxSoftChgAllowedCurrent * 12.0) + g_ptrGenConfig->u16HysteresisCurrent));
}
static void sysChgOCReleasedCallback(void)
{
  g_ptrPowerElecPackInfoConfig->bChargeAllowed = true;
  g_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_OFF;
}

// --- Over-Temperature Charge ---
// Note: EEPROM stores temps in deci-Celsius (550 = 55.0°C), i32ModuleTemperature is milli-Celsius (21000 = 21°C)
// Multiply limits by 100 to convert deci-C to milli-C for comparison
static bool sysOTCDetected(void)
{
  return (g_ptrPowerElecPackInfoConfig->bmsState == BMS_CHARGE && 
          g_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax >= (int32_t)g_ptrGenConfig->u16AllowedTempBattChargingMax * 100);
}
static void sysOTCDetectedCallback(void)
{
  // Over-temp: do not open contactors; only clamp discharge to zero.
  clampInverterDischargeZero("OTP_CHG");
}
static bool sysOTCReleased(void)
{
  const int32_t limit_mC = (int32_t)g_ptrGenConfig->u16AllowedTempBattChargingMax * 100;
  const int32_t hyst_mC = (int32_t)g_ptrGenConfig->u16HysteresisOverTemperature * 100;
  return (g_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax <= (limit_mC - hyst_mC));
}
static void sysOTCReleasedCallback(void) {}

// --- Over-Temperature Discharge ---
static bool sysOTDDetected(void)
{
  return (g_ptrPowerElecPackInfoConfig->bmsState == BMS_DISCHARGE && 
          g_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax >= (int32_t)g_ptrGenConfig->u16AllowedTempBattDischargingMax * 100);
}
static void sysOTDDetectedCallback(void)
{
  // Over-temp: do not open contactors; only clamp discharge to zero.
  clampInverterDischargeZero("OTP_DCHG");
}
static bool sysOTDReleased(void)
{
  const int32_t limit_mC = (int32_t)g_ptrGenConfig->u16AllowedTempBattDischargingMax * 100;
  const int32_t hyst_mC = (int32_t)g_ptrGenConfig->u16HysteresisOverTemperature * 100;
  return (g_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax <= (limit_mC - hyst_mC));
}
static void sysOTDReleasedCallback(void) {}

// --- Under-Temperature Charge ---
static bool sysUTCDetected(void)
{
  return (g_ptrPowerElecPackInfoConfig->bmsState == BMS_CHARGE && 
          g_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin <= (int32_t)g_ptrGenConfig->u16AllowedTempBattChargingMin * 100);
}
static void sysUTCDetectedCallback(void)
{
  g_ptrPowerElecPackInfoConfig->bChargeAllowed = false;
}
static bool sysUTCReleased(void)
{
  const int32_t limit_mC = (int32_t)g_ptrGenConfig->u16AllowedTempBattChargingMin * 100;
  const int32_t hyst_mC = (int32_t)g_ptrGenConfig->u16HysteresisOverTemperature * 100;
  return (g_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin > (limit_mC + hyst_mC));
}
static void sysUTCReleasedCallback(void)
{
  g_ptrPowerElecPackInfoConfig->bChargeAllowed = true;
}

// --- Under-Temperature Discharge ---
static bool sysUTDDetected(void)
{
  return (g_ptrPowerElecPackInfoConfig->bmsState == BMS_DISCHARGE && 
          g_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin <= (int32_t)g_ptrGenConfig->u16AllowedTempBattDischargingMin * 100);
}
static void sysUTDDetectedCallback(void)
{
  g_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed = false;
}
static bool sysUTDReleased(void)
{
  const int32_t limit_mC = (int32_t)g_ptrGenConfig->u16AllowedTempBattDischargingMin * 100;
  const int32_t hyst_mC = (int32_t)g_ptrGenConfig->u16HysteresisOverTemperature * 100;
  return (g_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin > (limit_mC + hyst_mC));
}
static void sysUTDReleasedCallback(void)
{
  g_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed = true;
}

