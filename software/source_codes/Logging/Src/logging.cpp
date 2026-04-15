#include <cstdint>
#include <cstring>
#include <memory>
#include "logging.h"

#include "Callback.h"
#include "Timer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdlib>
#include <cstdarg>
#include "measurement.h" // add this so logging sees g_packInfoMutex
#include "operation.h"   // for hb_set / HB_LOG

// Forward declaration for early use
class HwLoggingBase; // fwd
static void log_event_if_changed(HwLoggingBase &hw, const char *fmt, ...);

// ---- Small name helpers for readable EVENTS_LOG ----
static const char *op_state_to_str(uint8_t s)
{
  switch ((enOpStateType)s)
  {
  case OP_STATE_INIT:           return "INIT";
  case OP_STATE_CHARGING:       return "CHARGING";
  case OP_STATE_PRE_CHARGE:     return "PRE_CHARGE";
  case OP_STATE_LOAD_ENABLED:   return "LOAD_ENABLED";
  case OP_STATE_BATTERY_DEAD:   return "BATTERY_DEAD";
  case OP_STATE_POWER_DOWN:     return "POWER_DOWN";
  case OP_STATE_EXTERNAL:       return "EXTERNAL";
  case OP_STATE_ERROR:          return "ERROR";
  case OP_STATE_ERROR_PRECHARGE:return "ERROR_PRECHARGE";
  case OP_STATE_BALANCING:      return "BALANCING";
  case OP_STATE_CHARGED:        return "CHARGED";
  case OP_STATE_FORCEON:        return "FORCEON";
  default:                      return "UNKNOWN";
  }
}

static const char *bms_to_str(enBmsState s)
{
  switch (s)
  {
  case BMS_IDLE:       return "IDLE";
  case BMS_DISCHARGE:  return "DISCHARGE";
  case BMS_CHARGE:     return "CHARGE";
  default:             return "UNKNOWN";
  }
}

static const char *ov_to_str(enOvervoltageState s)
{
  switch (s)
  {
  case OV_STATE_NORMAL:   return "NORMAL";
  case OV_STATE_TRIPPED:  return "TRIPPED";
  case OV_STATE_RECOVERY: return "RECOVERY";
  default:                return "UNKNOWN";
  }
}

static const char *inv_to_str(enInverterResponseState s)
{
  switch (s)
  {
  case INVERTER_OK:    return "OK";
  case INVERTER_FAULT: return "FAULT";
  default:             return "UNKNOWN";
  }
}

// removed: legacy u8CellVoltageState (unused)

// ##############################################################################################################################

Logging::Logging(HwLoggingBase &_hwLogging, Mail<st_mail_cell_voltages_t, LOGGING_CELL_VOLTAGE_MAIL_SIZE> &mailCellVoltageBox)
    : m_hwLogging(_hwLogging),
      m_mailCellVoltageBox(mailCellVoltageBox),
      m_u32OledProcessTick(0),
      m_u8InterfaceButtonLastState(0),
      m_u8ErrorIndexCount(0),
      m_u32LoggingGeneralIntervalLastTick(0),
      m_u32LoggingSdCardIntervalLastTick(0),
      m_bErrorLoadingEnableFlag(false),
      m_bErrorResetEnableFlag(false),
      m_loggingThreadVar(LOGGING_THREAD_PRIORITY, LOGGING_THREAD_STACK_SIZE, nullptr, "Logging")
{
  m_loggingManagerTim.start();
}

// ##############################################################################################################################

Logging::~Logging()
{
}

// ##############################################################################################################################

void Logging::startThread()
{
  m_loggingThreadVar.start(mbed::callback(this, &Logging::loggingThread));
}

// ##############################################################################################################################

void Logging::init(st_generalConfig *genConfig, st_powerElecPackInfoConfig *powerElecPackInfoConfig)
{
  m_ptrGenConfig = genConfig;
  m_ptrPowerElecPackInfoConfig = powerElecPackInfoConfig;
  // m_hwLogging.serial.printToInterface("Logging Init\r\n");
}

// ##############################################################################################################################

void Logging::loggingThread()
{

  static bool _bShutdownSystemLastState = true;

  while (true)
  {

    // SD logging: handled below via non-blocking mail + periodic snapshots

    // Periodic snapshots: schedule pack temperatures and system info every 30s
    {
      static uint32_t s_lastSnapMs = 0;
      const uint32_t now = LOGGING_GET_TICK(m_loggingManagerTim);
      if ((now - s_lastSnapMs) >= 30000u) // 30 s
      {
        s_lastSnapMs = now;
        g_packInfoMutex.lock();
        m_ptrPowerElecPackInfoConfig->u8PackTemperatureLoggingState = INTTERFACE_BOOL_TRUE;
        m_ptrPowerElecPackInfoConfig->u8SystemInformationLoggingState = INTTERFACE_BOOL_TRUE;
        g_packInfoMutex.unlock();
      }
    }

    // Non-blocking SD card cell-voltage logging via mail
    st_mail_cell_voltages_t *mail = m_mailCellVoltageBox.try_get();
    if (mail)
    {
      m_u32LoggingSdCardIntervalLastTick = LOGGING_GET_TICK(m_loggingManagerTim);

      // We loop through each pack and log its voltages to the SD card.
      for (uint8_t module_num = 0; module_num < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++module_num)
      {
        if (m_hwLogging.sdCard.cellVoltageLogging(m_u32LoggingSdCardIntervalLastTick, module_num, mail->cell_voltages[module_num]) != 0xFF)
        {
          m_hwLogging.interfaceComm.send1Byte(COMM_FW_SDCARD_LOG_CELL_VOLTAGES_ERROR, m_hwLogging.sdCard.getSdCardStatus());
          m_hwLogging.sdCard.resetSdCardStatus();
          log_event_if_changed(m_hwLogging, "SD_ERR_CELL:%u", (unsigned)m_hwLogging.sdCard.getSdCardStatus());
        }
      }
      m_mailCellVoltageBox.free(mail);
    }

    subTaskInterfaceLogging();

    // Honor one-shot snapshot flags (cleared after success inside the subtasks)
    subTaskTemperatureLoggingToSdCard();
    subTaskSystemInformationLoggingToSdCard();

    subTaskInterfaceButtonStateWatch();

    bool pushedShutdown = false;
    g_packInfoMutex.lock();
    pushedShutdown = m_ptrPowerElecPackInfoConfig->bPushedShutdownChipSet;
    g_packInfoMutex.unlock();

    if (pushedShutdown && _bShutdownSystemLastState)
    {
      _bShutdownSystemLastState = false;
      m_hwLogging.interfaceComm.send1Byte(COMM_FW_POWEROFF_SYSTEM, 1);
    }

    if (m_bErrorResetEnableFlag)
    {
      static uint8_t resetErrorCount = 0;
      m_hwLogging.error.resetErrorFromEeprom(resetErrorCount);
      resetErrorCount++;
      if (resetErrorCount == m_u8ErrorIndex)
      {
        m_hwLogging.error.resetErrorIndex();
        m_bErrorResetEnableFlag = false;
        m_hwLogging.interfaceComm.send1Byte(COMM_FW_DIAGNOSTIC_LOAD_ERROR, 3);
      }
    }
    hb_set(HB_LOG);
    ThisThread::sleep_for(1ms);
  }
}

// ##############################################################################################################################

void Logging::subTaskInterfacePageLiveModuleUpdate(void)
{
  int32_t ind = 0;
  static uint8_t _counter = 1;

  // sanitize active module number (your existing code)
  const uint8_t maxMods = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
  uint8_t mod = 1;
  st_moduleMonitorParams paramsSnap;
  uint16_t cellsSnap[NoOfCELL_POSSIBLE_ON_CHIP];
  uint8_t nCells = 0;

  g_packInfoMutex.lock();
  mod = m_ptrPowerElecPackInfoConfig->u8ActiveModuleNumber;
  if (mod == 0)
    mod = 1;
  if (mod > maxMods)
    mod = maxMods;
  const uint8_t idx = mod - 1;

  paramsSnap = m_ptrPowerElecPackInfoConfig->moduleMonitorParams[idx];
  {
    nCells =
        (m_ptrGenConfig->u8NumberOfCells < NoOfCELL_POSSIBLE_ON_CHIP)
            ? m_ptrGenConfig->u8NumberOfCells
            : NoOfCELL_POSSIBLE_ON_CHIP;
    for (uint8_t c = 0; c < nCells; ++c)
    {
      cellsSnap[c] = m_ptrPowerElecPackInfoConfig->cellModuleVoltages[idx][c];
    }
    // If you also read other fields in this function, snapshot them here too.
  }
  g_packInfoMutex.unlock();
  // -------------------------------------------------------------------

  switch (_counter)
  {
  case 1:
  {
    m_u8arrSendBuffer[ind++] = INTERFACE_COMM_CMD_LIVE_MODULE_GENERAL;
    bufferAppend_uint16(m_u8arrSendBuffer, paramsSnap.u16PackVoltage, &ind);
    bufferAppend_uint16(m_u8arrSendBuffer, paramsSnap.u16CellVoltageHigh, &ind);
    bufferAppend_uint16(m_u8arrSendBuffer, paramsSnap.u16CellVoltageLow, &ind);
    bufferAppend_uint16(m_u8arrSendBuffer, paramsSnap.u16CellVoltageAverage, &ind);
    bufferAppend_uint16(m_u8arrSendBuffer, paramsSnap.u16CellVoltageMisMatch, &ind);
    bufferAppend_uint16(m_u8arrSendBuffer, paramsSnap.u16Soc, &ind);
    bufferAppend_uint16(m_u8arrSendBuffer, paramsSnap.i32TemperatureMax, &ind);
    bufferAppend_uint16(m_u8arrSendBuffer, paramsSnap.i32TemperatureMin, &ind);
    bufferAppend_uint16(m_u8arrSendBuffer, paramsSnap.i32TemperatureAverage, &ind);
    bufferAppend_uint8(m_u8arrSendBuffer, 7, &ind);
    m_hwLogging.interfaceComm.processPacket(m_u8arrSendBuffer, ind);
    _counter++;
  }
  break;

  case 2:
  {
    m_u8arrSendBuffer[ind++] = INTERFACE_COMM_CMD_LIVE_MODULE_CELL_VOLTAGES;
    for (uint8_t c = 0; c < nCells; ++c)
    {
      bufferAppend_uint16(m_u8arrSendBuffer, cellsSnap[c], &ind);
    }
    m_hwLogging.interfaceComm.processPacket(m_u8arrSendBuffer, ind);
    _counter++;
  }
  break;

  default:
    _counter = 1;
    break;
  }

  hb_set(HB_LOG);
}

// ##############################################################################################################################

void Logging::subTaskInterfacePageLiveRackUpdate(void)
{
  int32_t ind = 0;
  uint32_t moduleVoltage = 0;
  uint32_t moduleLoadVoltage = 0;
  int32_t moduleCurrent = 0;
  int32_t modulePower = 0;
  uint16_t cellVoltageMax = 0;
  uint16_t cellVoltageMin = 0;
  uint16_t cellVoltageAvg = 0;
  uint16_t cellVoltageMismatch = 0;
  uint16_t moduleSoc = 0;
  int32_t tempMax = 0;
  int32_t tempMin = 0;
  int32_t tempAvg = 0;
  uint8_t bmsState = 0;
  uint8_t opState = 0;
  uint8_t warErrState = 0;
  uint8_t rackBalancingState = 0;

  g_packInfoMutex.lock();
  moduleVoltage = m_ptrPowerElecPackInfoConfig->u32ModuleVoltage;
  moduleLoadVoltage = m_ptrPowerElecPackInfoConfig->u32ModuleLoadVoltage;
  moduleCurrent = m_ptrPowerElecPackInfoConfig->i32ModuleCurrent;
  modulePower = m_ptrPowerElecPackInfoConfig->i32ModulePower;
  cellVoltageMax = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax;
  cellVoltageMin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
  cellVoltageAvg = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageAverage;
  cellVoltageMismatch = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch;
  moduleSoc = m_ptrPowerElecPackInfoConfig->u16ModuleSoc;
  tempMax = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax;
  tempMin = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMin;
  tempAvg = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureAverage;
  bmsState = m_ptrPowerElecPackInfoConfig->bmsState;
  opState = m_ptrPowerElecPackInfoConfig->u8operationState;
  warErrState = m_ptrPowerElecPackInfoConfig->packBatteryWarErrState;
  rackBalancingState = m_ptrPowerElecPackInfoConfig->rackBalancingState;
  g_packInfoMutex.unlock();

  m_u8arrSendBuffer[ind++] = INTERFACE_COMM_CMD_LIVE_RACK_PARAMS;
  bufferAppend_uint16(m_u8arrSendBuffer, (uint16_t)((float)moduleVoltage / 10.0), &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, (uint16_t)((float)moduleLoadVoltage / 10.0), &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, (uint16_t)((float)moduleCurrent / 10.0), &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, (uint16_t)((float)modulePower / 10.0), &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, cellVoltageMax, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, cellVoltageMin, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, cellVoltageAvg, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, cellVoltageMismatch, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, moduleSoc, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, tempMax, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, tempMin, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, tempAvg, &ind);
  bufferAppend_uint8(m_u8arrSendBuffer, bmsState, &ind);
  bufferAppend_uint8(m_u8arrSendBuffer, opState, &ind);
  bufferAppend_uint8(m_u8arrSendBuffer, warErrState, &ind);
  bufferAppend_uint8(m_u8arrSendBuffer, rackBalancingState, &ind);
  m_hwLogging.interfaceComm.processPacket(m_u8arrSendBuffer, ind);
}

// #################################################################################################################

void Logging::subTaskLoadErrorToInterface(void)
{

  if (m_u8ErrorIndexCount < m_u8ErrorIndex)
  {
    m_hwLogging.error.getErrorFromEeprom(&errorEepromData, m_u8ErrorIndexCount);
    m_u8ErrorIndexCount++;
    int32_t ind = 0;
    m_u8arrSendBuffer[ind++] = COMM_FW_DIAGNOSTIC_LOAD;
    bufferAppend_uint8(m_u8arrSendBuffer, errorEepromData.errorType, &ind);
    bufferAppend_uint8(m_u8arrSendBuffer, errorEepromData.operationState, &ind);
    bufferAppend_uint8(m_u8arrSendBuffer, errorEepromData.datetimeInfo.u8SysSecond, &ind);
    bufferAppend_uint8(m_u8arrSendBuffer, errorEepromData.datetimeInfo.u8SysMinute, &ind);
    bufferAppend_uint8(m_u8arrSendBuffer, errorEepromData.datetimeInfo.u8SysHour, &ind);
    bufferAppend_uint8(m_u8arrSendBuffer, errorEepromData.datetimeInfo.u8SysDay, &ind);
    bufferAppend_uint8(m_u8arrSendBuffer, errorEepromData.datetimeInfo.u8SysMonth, &ind);
    bufferAppend_uint8(m_u8arrSendBuffer, errorEepromData.datetimeInfo.u8SysYear, &ind);
    m_hwLogging.interfaceComm.processPacket(m_u8arrSendBuffer, ind);
  }
  else
  {
    m_bErrorLoadingEnableFlag = false;
  }
  hb_set(HB_LOG);
}

// #################################################################################################################

void Logging::subTaskSendingSettingsParameter(void)
{
  int32_t ind = 0;
  m_u8arrSendBuffer[ind++] = COMM_FW_SETTINGS_RESET_PARAMS_GROUP1;
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16AllowedTempBattChargingMin, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16AllowedTempBattChargingMax, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16AllowedTempBattDischargingMin, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16AllowedTempBattDischargingMax, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, 20, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, 60, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16SoftOverVoltage, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16SoftLowCurrentUnderVoltage, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16MaxHardDchgAllowedCurrent, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16MaxSoftDchgAllowedCurrent, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16MaxHardChgAllowedCurrent, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16MaxSoftChgAllowedCurrent, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16ShuntResistance, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16BalanceStartVoltage, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u32ArrNtc25DegResistance, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u32ArrNtcBetaFactor, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16MinimalPrechargePercentage, &ind);
  // Encode signed current offset into uint16 for transport: negative -> 32768 + magnitude
  int16_t cur_off_signed = (m_ptrGenConfig->u16CurrentOffsetValue > NEGATIVE_SENSITIVE_VALUE)
                               ? -(int16_t)(m_ptrGenConfig->u16CurrentOffsetValue - NEGATIVE_SENSITIVE_VALUE)
                               : (int16_t)m_ptrGenConfig->u16CurrentOffsetValue;
  uint16_t cur_off_encoded = (cur_off_signed < 0)
                                 ? (uint16_t)(32768u + (uint16_t)(-cur_off_signed))
                                 : (uint16_t)cur_off_signed;
  bufferAppend_uint16(m_u8arrSendBuffer, cur_off_encoded, &ind);
  m_hwLogging.interfaceComm.processPacket(m_u8arrSendBuffer, ind);

  ind = 0;
  m_u8arrSendBuffer[ind++] = COMM_FW_SETTINGS_RESET_PARAMS_GROUP2;
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16MaxMismatchThreshold, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16NotUsedCurrentThreshold, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16ChargerEnabledThreshold, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16TimeoutChargingCompletedMinimalMismatch, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16TimeoutChargeCompleted, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16TimeoutChargerDisconnected, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16TimeoutNotUsed, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16TimeoutHardOverCurrentRetry, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16TimeoutSoftOverCurrentRetry, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16TimeoutLowCurrentPreChargeRetry, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16TimeoutOverTemperatureRetry, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16TimeoutChargeRetry, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16TimeoutDischargeRetry, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u32ArrNtc25DegResistance, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u32ArrNtcBetaFactor, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16BatteryCapacity, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, m_ptrGenConfig->u16InternalResistance, &ind);
  bufferAppend_uint16(m_u8arrSendBuffer, (m_ptrGenConfig->i16CellVoltagesOffsetVoltage < 0) ? (32768 + abs(m_ptrGenConfig->i16CellVoltagesOffsetVoltage)) : (m_ptrGenConfig->i16CellVoltagesOffsetVoltage), &ind);

  m_hwLogging.interfaceComm.processPacket(m_u8arrSendBuffer, ind);
}

// #################################################################################################################

void Logging::subTaskloggingModuleProtectionCallback(void)
{

  // update outputs directly if needed
  for (uint8_t i = 0; i < NoOfPROTS_POSSIBLE_ON_SYS; i++)
  {
    bool doLog = false;
    uint8_t state = 0;

    g_packInfoMutex.lock();
    if (m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].bLoggingProtectionActive &&
        (m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].lastProtectionState !=
         m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].protectionState))
    {
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].bLoggingProtectionActive = false;
      m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].lastProtectionState =
          m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].protectionState;
      state = m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].protectionState;
      doLog = true;
    }
    g_packInfoMutex.unlock();

    if (doLog)
    {
      m_hwLogging.interfaceComm.printToInterface("ModuleError[%d] : %d\r\n", i, state);
    }
  }
}

// #################################################################################################################

// removed: legacy cell voltage SD logger (mail-driven path is used)

// #################################################################################################################

void Logging::subTaskTemperatureLoggingToSdCard(void)
{
  // Throttle SD writes
  const uint32_t nowTick = LOGGING_GET_TICK(m_loggingManagerTim);
  if ((nowTick - m_u32LoggingSdCardIntervalLastTick) < 1000u)
    return;

  static uint8_t _u8TotalModuleInd = 0;
  int32_t temperatureInfo[NoOfTEMP_POSSIBLE_ON_CHIP] = {0};
  uint8_t moduleIndex = 0;
  bool doLog = false;

  g_packInfoMutex.lock();
  if (m_ptrPowerElecPackInfoConfig->u8PackTemperatureLoggingState == INTTERFACE_BOOL_TRUE)
  {
    moduleIndex = _u8TotalModuleInd;
    memcpy(&temperatureInfo, &m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature[moduleIndex], sizeof(temperatureInfo));
    doLog = true;
  }
  g_packInfoMutex.unlock();

  if (!doLog)
    return;

  m_u32LoggingSdCardIntervalLastTick = nowTick;

  if (m_hwLogging.sdCard.packTemperatureLogging(m_u32LoggingSdCardIntervalLastTick, moduleIndex, temperatureInfo) != 0xFF)
  {
    m_hwLogging.interfaceComm.send1Byte(COMM_FW_SDCARD_LOG_CELL_VOLTAGES_ERROR, m_hwLogging.sdCard.getSdCardStatus());
    m_hwLogging.sdCard.resetSdCardStatus();
    log_event_if_changed(m_hwLogging, "SD_ERR_TEMP:%u", (unsigned)m_hwLogging.sdCard.getSdCardStatus());
    g_packInfoMutex.lock();
    m_ptrPowerElecPackInfoConfig->u8PackTemperatureLoggingState = 0;
    g_packInfoMutex.unlock();
  }
  else
  {
    if (++_u8TotalModuleInd == NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
    {
      _u8TotalModuleInd = 0;
      // After one full cycle, turn off the snapshot flag
      g_packInfoMutex.lock();
      m_ptrPowerElecPackInfoConfig->u8PackTemperatureLoggingState = 0;
      g_packInfoMutex.unlock();
    }
  }
}

// #################################################################################################################

void Logging::subTaskSystemInformationLoggingToSdCard(void)
{
  // Throttle SD writes
  const uint32_t nowTick = LOGGING_GET_TICK(m_loggingManagerTim);
  if ((nowTick - m_u32LoggingSdCardIntervalLastTick) < 1000u)
    return;
  bool didLog = false;
  uint8_t logStatus = 0xFF;

  g_packInfoMutex.lock();
  if (m_ptrPowerElecPackInfoConfig->u8SystemInformationLoggingState == INTTERFACE_BOOL_TRUE)
  {
    m_u32LoggingSdCardIntervalLastTick = nowTick;
    logStatus = m_hwLogging.sdCard.systemInformationLogging(m_u32LoggingSdCardIntervalLastTick, m_ptrPowerElecPackInfoConfig);
    m_ptrPowerElecPackInfoConfig->u8SystemInformationLoggingState = 0;
    didLog = true;
  }
  g_packInfoMutex.unlock();

  if (!didLog)
    return;

  if (logStatus != 0xFF)
  {
    m_hwLogging.interfaceComm.send1Byte(COMM_FW_SDCARD_LOG_CELL_VOLTAGES_ERROR, m_hwLogging.sdCard.getSdCardStatus());
    m_hwLogging.sdCard.resetSdCardStatus();
    log_event_if_changed(m_hwLogging, "SD_ERR_SYS:%u", (unsigned)m_hwLogging.sdCard.getSdCardStatus());
  }
}

// #################################################################################################################

void Logging::subTaskDebuggingInInterface(void)
{
  uint8_t debugState = 0;
  uint16_t busbarVal = 0;

  g_packInfoMutex.lock();
  debugState = m_ptrPowerElecPackInfoConfig->u8DebuggingState;
  busbarVal = m_ptrPowerElecPackInfoConfig->u16ModuleBusbarVal;
  g_packInfoMutex.unlock();

  if (debugState == INTTERFACE_BOOL_TRUE)
  {
    m_hwLogging.interfaceComm.printToInterface("BusBar : %d\r\n", busbarVal);
  }
  else if (debugState == INTTERFACE_BOOL_FALSE)
  {
    m_hwLogging.interfaceComm.printToInterface("Debugging paused\r\n");
    g_packInfoMutex.lock();
    m_ptrPowerElecPackInfoConfig->u8DebuggingState = 0;
    g_packInfoMutex.unlock();
  }
}

// #################################################################################################################

void Logging::subTaskErrorLoggingWatch(void)
{

  static uint8_t _u8ErrorCount = 0;
  uint8_t errorCount = 0;
  enBatteryErrors warErrState = SYS_OK;
  enBqErrorType chipsetErrState = NO_ERROR;

  g_packInfoMutex.lock();
  errorCount = m_ptrPowerElecPackInfoConfig->u8ErrorCount;
  warErrState = m_ptrPowerElecPackInfoConfig->packBatteryWarErrState;
  chipsetErrState = m_ptrPowerElecPackInfoConfig->chipsetErrorState;
  g_packInfoMutex.unlock();

  if (_u8ErrorCount < errorCount)
  {
    if (warErrState == SYS_ERROR_CHIPSET)
    {
      m_hwLogging.interfaceComm.send2Bytes(COMM_FW_DIAGNOSTIC_ERROR_GETTING, warErrState, chipsetErrState);
      g_packInfoMutex.lock();
      m_ptrPowerElecPackInfoConfig->chipsetErrorState = NO_ERROR;
      g_packInfoMutex.unlock();
    }
    else
    {
      m_hwLogging.interfaceComm.send1Byte(COMM_FW_DIAGNOSTIC_ERROR_GETTING, warErrState);
    }
    _u8ErrorCount++;
  }
  else
  {
    g_packInfoMutex.lock();
    m_ptrPowerElecPackInfoConfig->u8ErrorCount = 0;
    g_packInfoMutex.unlock();
    _u8ErrorCount = 0;
  }
}

// #################################################################################################################

void Logging::subTaskInterfaceButtonStateWatch(void)
{

  // In case of extreme cellvoltages or current goto error state
  uint8_t buttonState = 0;
  g_packInfoMutex.lock();
  buttonState = m_ptrPowerElecPackInfoConfig->u8InterfaceActiveButtonState;
  g_packInfoMutex.unlock();

  if ((buttonState != m_u8InterfaceButtonLastState))
  {
    m_u8InterfaceButtonLastState = buttonState;
    // m_hwLogging.serial.printToInterface("Button State : %d\n\r", m_u8InterfaceButtonLastState);

    if (buttonState == 0x7)
    {
      m_u8ErrorIndex = m_hwLogging.error.getErrorIndexVal();
      if (m_u8ErrorIndex == 0)
      {
        m_hwLogging.interfaceComm.send1Byte(COMM_FW_DIAGNOSTIC_LOAD_ERROR, 1);
      }
      else
      {
        m_bErrorLoadingEnableFlag = true;
      }
    }

    if (buttonState == 0x8)
    {
      m_u8ErrorIndex = m_hwLogging.error.getErrorIndexVal();

      if (m_u8ErrorIndex == 0)
      {
        m_hwLogging.interfaceComm.send1Byte(COMM_FW_DIAGNOSTIC_LOAD_ERROR, 2);
      }
      else
      {
        m_bErrorResetEnableFlag = true;
      }
    }

    if (buttonState == 0x5)
    {
      subTaskSendingSettingsParameter();
    }
  }
}

// #################################################################################################################
// Emit high-level events to EVENTS_LOG.csv
static void log_event_if_changed(HwLoggingBase &hw, const char *fmt, ...)
{
  char buf[128];
  va_list vl;
  va_start(vl, fmt);
  vsnprintf(buf, sizeof(buf), fmt, vl);
  va_end(vl);
  hw.sdCard.logEvent("%s", buf);
}

// removed: legacy SD logging aggregator
// #################################################################################################################
void Logging::subTaskInterfaceLogging(void)
{

  if ((LOGGING_GET_TICK(m_loggingManagerTim) - m_u32LoggingGeneralIntervalLastTick) >= 100)
  {
    m_u32LoggingGeneralIntervalLastTick = LOGGING_GET_TICK(m_loggingManagerTim);

    subTaskDebuggingInInterface();

    subTaskErrorLoggingWatch();

    bool updatedSettingFlags = false;
    uint8_t buttonState = 0;
    uint8_t opState = 0;
    bool chargeBalanceActive = false;
    bool chargeCurrentDetected = false;
    enBmsState bmsState = BMS_IDLE;
    enOvervoltageState ovState = OV_STATE_NORMAL;
    enInverterResponseState invState = INVERTER_OK;
    bool soaDischarge = false;
    bool soaCharge = false;
    bool socSaveReq = false;
    bool pushedShutdown = false;

    g_packInfoMutex.lock();
    updatedSettingFlags = m_ptrPowerElecPackInfoConfig->bUpdatedSettingFlags;
    buttonState = m_ptrPowerElecPackInfoConfig->u8InterfaceActiveButtonState;
    opState = m_ptrPowerElecPackInfoConfig->u8operationState;
    chargeBalanceActive = m_ptrPowerElecPackInfoConfig->bChargeBalanceActive;
    chargeCurrentDetected = m_ptrPowerElecPackInfoConfig->bChargeCurrentDetected;
    bmsState = m_ptrPowerElecPackInfoConfig->bmsState;
    ovState = m_ptrPowerElecPackInfoConfig->overvoltageState;
    invState = m_ptrPowerElecPackInfoConfig->inverterResponseState;
    soaDischarge = m_ptrPowerElecPackInfoConfig->bPackInSOADischarge;
    soaCharge = m_ptrPowerElecPackInfoConfig->bPackInSOACharge;
    socSaveReq = m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest;
    pushedShutdown = m_ptrPowerElecPackInfoConfig->bPushedShutdownChipSet;
    g_packInfoMutex.unlock();

    if (updatedSettingFlags)
    {
      subTaskSendingSettingsParameter();
      g_packInfoMutex.lock();
      m_ptrPowerElecPackInfoConfig->bUpdatedSettingFlags = false;
      g_packInfoMutex.unlock();
    }

    if ((enInterfaceButtonState)buttonState == INTERFACE_BUTTON_PAGE_LIVE_RACK)
    {
      subTaskInterfacePageLiveRackUpdate();
    }

    if ((enInterfaceButtonState)buttonState == INTERFACE_BUTTON_PAGE_LIVE_MODULE)
    {
      subTaskInterfacePageLiveModuleUpdate();
    }

    if (m_bErrorLoadingEnableFlag)
    {
      subTaskLoadErrorToInterface();
    }

    // Event logging: operation state changes and charge-balance window
    {
      static uint8_t s_lastOpState = 0xFF;
      if (s_lastOpState != opState)
      {
        log_event_if_changed(m_hwLogging, "OpState->%s", op_state_to_str(opState));
        s_lastOpState = opState;
      }
    }
    {
      static int s_lastBal = -1;
      const int bal = chargeBalanceActive ? 1 : 0;
      if (s_lastBal != bal)
      {
        log_event_if_changed(m_hwLogging, "ChargeBalanceActive->%s", bal ? "ON" : "OFF");
        s_lastBal = bal;
      }
    }
    // Charger present (debounced by Measurement)
    {
      static int s_lastChg = -1;
      const int chg = chargeCurrentDetected ? 1 : 0;
      if (s_lastChg != chg)
      {
        log_event_if_changed(m_hwLogging, "ChargerDetected->%s", chg ? "ON" : "OFF");
        s_lastChg = chg;
      }
    }
    // BMS mode changes (IDLE/DISCHARGE/CHARGE)
    {
      static uint8_t s_lastBms = 0xFF;
      if (s_lastBms != (uint8_t)bmsState)
      {
        log_event_if_changed(m_hwLogging, "BMS->%s", bms_to_str(bmsState));
        s_lastBms = (uint8_t)bmsState;
      }
    }
    // Overvoltage state transitions (NORMAL/TRIPPED/RECOVERY)
    {
      static uint8_t s_lastOv = 0xFF;
      if (s_lastOv != (uint8_t)ovState)
      {
        log_event_if_changed(m_hwLogging, "OV->%s", ov_to_str(ovState));
        s_lastOv = (uint8_t)ovState;
      }
    }
    // Inverter response state
    {
      static uint8_t s_lastInv = 0xFF;
      if (s_lastInv != (uint8_t)invState)
      {
        log_event_if_changed(m_hwLogging, "INV->%s", inv_to_str(invState));
        s_lastInv = (uint8_t)invState;
      }
    }
    // SOA windows
    {
      static int s_lastSOAD = -1, s_lastSOAC = -1;
      const int soad = soaDischarge ? 1 : 0;
      const int soac = soaCharge ? 1 : 0;
      if (s_lastSOAD != soad)
      {
        log_event_if_changed(m_hwLogging, "SOA_D->%s", soad ? "IN" : "OUT");
        s_lastSOAD = soad;
      }
      if (s_lastSOAC != soac)
      {
        log_event_if_changed(m_hwLogging, "SOA_C->%s", soac ? "IN" : "OUT");
        s_lastSOAC = soac;
      }
    }
    // Power-down pushed
    {
      static int s_lastPdn = -1;
      const int pdn = pushedShutdown ? 1 : 0;
      if (s_lastPdn != pdn)
      {
        log_event_if_changed(m_hwLogging, "PowerdownFlag->%s", pdn ? "SET" : "CLR");
        s_lastPdn = pdn;
      }
    }
    // SOC save request lifecycle (set/cleared)
    {
      static int s_lastSocReq = -1;
      const int socReq = socSaveReq ? 1 : 0;
      if (s_lastSocReq != socReq)
      {
        log_event_if_changed(m_hwLogging, "SOC_SaveReq->%s", socReq ? "SET" : "CLR");
        s_lastSocReq = socReq;
      }
    }

    // === Periodic OP_STATE/BMS heartbeat for UI (every 5s) ===
    {
      static uint32_t s_stateHeartbeatTick = 0;
      const uint32_t nowTick = LOGGING_GET_TICK(m_loggingManagerTim);
      if ((nowTick - s_stateHeartbeatTick) >= 5000)
      {
        s_stateHeartbeatTick = nowTick;
        m_hwLogging.interfaceComm.printToInterface(
            "----- OP_STATE_%s: bmsState: %s \r\n",
            op_state_to_str(opState),
            bms_to_str(bmsState));
      }
    }

    // === NEW: Periodic Health Summary (1s interval) ===
    {
      static uint32_t s_healthLogTick = 0;
      const uint32_t nowTick = LOGGING_GET_TICK(m_loggingManagerTim);
      if ((nowTick - s_healthLogTick) >= 1000)
      {
        s_healthLogTick = nowTick;
        uint32_t vPack = 0;
        int32_t iMod = 0;
        uint16_t vMin = 0, vMax = 0, dV = 0, soc = 0;
        uint16_t vMinBal = 0, vMaxBal = 0, dVBal = 0;
        bool balActive = false;
        int32_t tMax = 0;

        g_packInfoMutex.lock();
        vPack = m_ptrPowerElecPackInfoConfig->u32ModuleVoltage;
        iMod = m_ptrPowerElecPackInfoConfig->i32ModuleCurrent;
        vMin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
        vMax = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMax;
        dV = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch;
        vMinBal = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMinBal;
        vMaxBal = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMaxBal;
        dVBal = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatchBal;
        balActive = m_ptrPowerElecPackInfoConfig->bBalancingActive;
        tMax = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax;
        soc = m_ptrPowerElecPackInfoConfig->u16ModuleSoc;
        uint8_t conf = m_ptrPowerElecPackInfoConfig->u8SocConfidence;
        g_packInfoMutex.unlock();

        if (balActive && vMaxBal >= vMinBal && vMaxBal != 0 && vMinBal != 0)
        {
          vMin = vMinBal;
          vMax = vMaxBal;
          dV = dVBal;
        }

        m_hwLogging.interfaceComm.printToInterface(
            "[HEALTH] V=%lu.%01uV I=%ld.%01uA Vmin=%umV Vmax=%umV dV=%umV Tmax=%ld.%01uC SOC=%u.%01u%% conf=%u%%\r\n",
            (unsigned long)(vPack / 1000), (unsigned)((vPack % 1000) / 100),
            (long)(iMod / 1000), (unsigned)((abs(iMod) % 1000) / 100),
            (unsigned)vMin, (unsigned)vMax, (unsigned)dV,
            (long)(tMax / 1000), (unsigned)((abs(tMax) % 1000) / 100),
            (unsigned)(soc / 10), (unsigned)(soc % 10),
            (unsigned)conf);
      }
    }

    const uint32_t kDetailFastMs = 10000;
    const uint32_t kDetailSlowMs = 20000;
    bool verboseDetailLogs = false;
    {
      const uint16_t dvThreshold = m_ptrGenConfig->u16BalanceDifferenceThreshold;
      g_packInfoMutex.lock();
      const bool balActive = m_ptrPowerElecPackInfoConfig->bBalancingActive;
      const uint16_t dv_mV = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch;
      g_packInfoMutex.unlock();
      verboseDetailLogs = balActive || (dvThreshold > 0 && dv_mV >= dvThreshold);
    }

#if ENABLE_MODULE_DV_REPORT
    // === NEW: Per-module dV summary (interval depends on balancing/dV) ===
    {
      static uint32_t s_dvLogTick = 0;
      const uint32_t nowTick = LOGGING_GET_TICK(m_loggingManagerTim);
      const uint32_t dvInterval = verboseDetailLogs ? kDetailFastMs : kDetailSlowMs;
      if ((nowTick - s_dvLogTick) >= dvInterval)
      {
        s_dvLogTick = nowTick;
        uint8_t nModules = m_ptrGenConfig->u8NumberOfModules;
        if (nModules > NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
          nModules = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;

        uint16_t vminSnap[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {0};
        uint16_t vmaxSnap[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {0};
        uint16_t dvSnap[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {0};

        g_packInfoMutex.lock();
        for (uint8_t m = 0; m < nModules; ++m)
        {
          const st_moduleMonitorParams &mod = m_ptrPowerElecPackInfoConfig->moduleMonitorParams[m];
          vminSnap[m] = mod.u16CellVoltageLow;
          vmaxSnap[m] = mod.u16CellVoltageHigh;
          dvSnap[m] = mod.u16CellVoltageMisMatch;
        }
        g_packInfoMutex.unlock();

        for (uint8_t m = 0; m < nModules; ++m)
        {
          m_hwLogging.interfaceComm.printToInterface(
              "[DV-M%u] Vmin=%umV Vmax=%umV dV=%umV\r\n",
              (unsigned)(m + 1),
              (unsigned)vminSnap[m],
              (unsigned)vmaxSnap[m],
              (unsigned)dvSnap[m]);
        }
      }
    }
#endif

    // === NEW: All Cell Voltages Report (interval depends on balancing/dV) - ALL MODULES ===
    {
      static uint32_t s_cellLogTick = 0;
      const uint32_t nowTick = LOGGING_GET_TICK(m_loggingManagerTim);
      const uint32_t cellInterval = verboseDetailLogs ? kDetailFastMs : kDetailSlowMs;
      if ((nowTick - s_cellLogTick) >= cellInterval)
      {
        s_cellLogTick = nowTick;
        uint8_t nCells = m_ptrGenConfig->u8NumberOfCells;
        if (nCells > NoOfCELL_POSSIBLE_ON_CHIP)
          nCells = NoOfCELL_POSSIBLE_ON_CHIP;
        uint8_t nModules = m_ptrGenConfig->u8NumberOfModules;
        if (nModules > NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
          nModules = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;

        g_packInfoMutex.lock();
        for (uint8_t m = 0; m < nModules; ++m)
        {
          char buf[200];
          int pos = snprintf(buf, sizeof(buf), "[CELLS-M%u] ", (unsigned)(m + 1));
          for (uint8_t c = 0; c < nCells && pos < 180; ++c)
          {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%u%s",
                            (unsigned)m_ptrPowerElecPackInfoConfig->cellModuleVoltages[m][c],
                            (c < nCells - 1) ? "," : "");
          }
          m_hwLogging.interfaceComm.printToInterface("%s\r\n", buf);
        }
        g_packInfoMutex.unlock();
      }
    }

    // === NEW: All Cell Temps Report (interval depends on balancing/dV) - ALL MODULES ===
    {
      static uint32_t s_tempLogTick = 0;
      const uint32_t nowTick = LOGGING_GET_TICK(m_loggingManagerTim);
      const uint32_t tempInterval = verboseDetailLogs ? kDetailFastMs : kDetailSlowMs;
      if ((nowTick - s_tempLogTick) >= tempInterval)
      {
        s_tempLogTick = nowTick;
        uint8_t nTemps = m_ptrGenConfig->u8NumberOfTemperature;
        if (nTemps > NoOfTEMP_POSSIBLE_ON_CHIP)
          nTemps = NoOfTEMP_POSSIBLE_ON_CHIP;
          
        uint8_t nModules = m_ptrGenConfig->u8NumberOfModules;
        if (nModules > NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
          nModules = NoOfCELL_MONITORS_POSSIBLE_ON_BMS;

        g_packInfoMutex.lock();
        for (uint8_t m = 0; m < nModules; ++m)
        {
          char buf[200];
          int pos = snprintf(buf, sizeof(buf), "[TEMPS-M%u] ", (unsigned)(m + 1));
          for (uint8_t t = 0; t < nTemps && pos < 180; ++t)
          {
            int32_t temp_mC = m_ptrPowerElecPackInfoConfig->i32arrCellModuleBMSEXTTemperature[m][t];
            bool isNeg = (temp_mC < 0);
            if (isNeg) temp_mC = -temp_mC;
            
            // Round to nearest 0.1C (100mC). 
            // Add 50mC before dividing by 100.
            int32_t val_decic = (temp_mC + 50) / 100;
            
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%s%ld.%u%s",
                            isNeg ? "-" : "",
                            (long)(val_decic / 10), 
                            (unsigned)(val_decic % 10),
                            (t < nTemps - 1) ? "," : "");
          }
          m_hwLogging.interfaceComm.printToInterface("%s\r\n", buf);
        }
        g_packInfoMutex.unlock();
      }
    }
  }
}
// #################################################################################################################

/*
void Logging::subTaskLivePageSendingData(void) {

  int32_t ind = 0;


  m_u8arrSendBuffer[ind++] = COMM_FW_VERSION;

  bufferAppend_float32(m_u8arrSendBuffer, 60.354, 1e3, &ind);

  bufferAppend_uint8(m_u8arrSendBuffer, (uint8_t)round(78.25), &ind);

  bufferAppend_float16(m_u8arrSendBuffer, 12.33, 1e2, &ind);

  bufferAppend_uint8(m_u8arrSendBuffer, (uint8_t)3, &ind);

  m_u8arrSendBuffer[ind++] = 170;
  strcpy((char*)(m_u8arrSendBuffer + ind), HW_VERSION);
  ind += strlen(HW_VERSION) + 1;

  m_hwLogging.serial.processPacket(m_u8arrSendBuffer, ind);
}
*/
