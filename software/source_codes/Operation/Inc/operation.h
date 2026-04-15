#ifndef __OPERATION__H_
#define __OPERATION__H_

#include "operation_params.h"
#include "io_periph_defs.h"
#include "Settings.h"
#include "hw_operation_base.h"

#include "PinNames.h"
#include "Thread.h"
#include "ThisThread.h"
#include "Callback.h"
#include "Mutex.h"
#include "Timer.h"
#include "Ticker.h"

#include <cstdarg>
#include <cstdint>
#include <vector>

// --- Balancing/charge management constants ---
static constexpr uint16_t kDV_START_mV = 45;        // start balancing (and throttle charge)
static constexpr uint16_t kDV_STOP_mV = 30;         // stop condition (hysteresis)
static constexpr uint32_t kDV_STOP_HOLD_ms = 10000; // must stay <= kDV_STOP_mV this long
static constexpr uint16_t kTHROTTLE_1A_CMD = 100;   // your units: 100 == 1 A (10 mA per unit)

extern volatile uint32_t g_heartbeat_mask;
// Watchdog kick period (must be << WATCHDOG_TIMEOUT_MS)
static constexpr uint32_t WDT_KICK_PERIOD_MS = 500;
static_assert(WATCHDOG_TIMEOUT_MS >= (WDT_KICK_PERIOD_MS * 3),
              "WATCHDOG_TIMEOUT_MS too small for the chosen kick period");

enum : uint32_t
{
  HB_MEAS = 1u << 0, // Measurement thread heartbeat
  HB_BQ = 1u << 1,   // BQ thread heartbeat
  HB_LOG = 1u << 2,  // Logging thread heartbeat
};
void hb_set(uint32_t bit);
class Operation
{

public:
  Operation(HwOperationBase &_hwOperation);
  virtual ~Operation();

  // void hb_set(uint32_t bit);

  void startThread(void);
  void init(st_generalConfig *genConfig, st_powerElecPackInfoConfig *powerElecPackInfoConfig);

private:
  void opThread(void);
  uint8_t timerDelay1ms(uint32_t *last, uint32_t ticks);

  void operationStateTask(void);
  void operationStateUpdateStates(void);
  void operationStateSetAllStates(enOpStateType newState);
  enOpStateType operationStateGetState(void);
  void operationStateSetNewState(enOpStateType newState);
  void operationStateTerminateOperation(void);
  void handleChargerDisconnect(enOpStateType newState);

  void operationBuzzerCallback(void);
  void operationModuleProtectionCallback(void);

private:
  rtos::Thread m_opThreadVar;
  mbed::Timer m_opManagerTim;

  HwOperationBase &m_hwOperation;
  st_generalConfig *m_ptrGenConfig;
  st_powerElecPackInfoConfig *m_ptrPowerElecPackInfoConfig;
  st_OledDisplayData displayData;
  st_errorEepromData errorEepromData;
  st_errorSdCardData errorSdCardData;

  enOpStateType m_operationalStateLastState;
  enOpStateType m_operationalStateCurrentState;
  enOpStateType m_operationalStateNewState;
  enPowerElecPackOperationCellState packOperationCellStateLastErrorState;
  enBatteryErrors packStateLastErrorState;
  bool m_bOpStateForceOn;
  bool m_bOpStatePressedState;
  bool m_bEepromEmptyErrorFlag;

  uint8_t m_u8PrechargeRetryCount; // retry counter

  uint32_t m_u32StateChargerTimeout;
  uint32_t m_u32StateChargedTimeout;
  uint32_t m_u32StatePreChargeTimeout;
  uint32_t m_u32StateStartupDelay;
  uint32_t m_u32StateChargerDisconnectDetectDelay;
  uint32_t m_u32StateBatteryDeadDisplayTime;
  uint32_t m_u32StateErrorDisplayTime;
  uint32_t m_u32StateNotUsedResetDelay;
  uint32_t m_u32StateNotUsedTime;
  uint32_t m_u32StatePSPDisableDelay;
  uint32_t m_u32StateWatchDogCountdownLastTick;
};

#endif // __OPERATION__H_
