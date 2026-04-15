#include "power_handler.h"
#include "mbed.h"
#include <cstdint>
#include "interface_comm_handler.h"

PowerHandler::PowerHandler()
    : m_chargeEnPin(PWR_CONT_CHARGE_PIN),
      m_dischargeEnPin(PWR_CONT_DISCHARGE_PIN),
      m_preDischargeEnPin(PWR_CONT_PRECHARGE_PIN),

      m_packVoltagePin(PWR_PACK_VOL_PIN, ADC_VREF_VALUE),
      m_loadVoltagePin(PWR_LOAD_VOL_PIN, ADC_VREF_VALUE),

      m_bAllowedForceOnFlag(false),
      m_preStamp(0),
      m_dchgStamp(0),
      m_chgStamp(0),
      m_preLastState(false),
      m_dischargeLastState(false),
      m_chargeLastState(false)
{
}

// #############################################################################################//

PowerHandler::~PowerHandler() {}

// #############################################################################################//

void PowerHandler::init(st_generalConfig *genConfig, st_powerElecPackInfoConfig *powerElecPackInfoConfig)
{
  m_ptrGenConfig = genConfig;
  m_ptrPowerElecPackInfoConfig = powerElecPackInfoConfig;
}

// #############################################################################################//

void PowerHandler::task()
{

  m_fSysVoltage = m_packVoltagePin.read_voltage();
  m_fLoadVoltage = m_loadVoltagePin.read_voltage();

  UTILS_LP_FAST(m_fFilteredSysVoltage, m_fSysVoltage, I_IN_FILTER_CONST);
  UTILS_LP_FAST(m_fFilteredLoadVoltage, m_fLoadVoltage, I_IN_FILTER_CONST);
}

// Simple global dwell for contactor toggling to avoid chatter
#ifndef PWR_CONTACTOR_DWELL_MS
#define PWR_CONTACTOR_DWELL_MS 3000u
#endif

// #############################################################################################//

uint16_t PowerHandler::getSysVoltage(void) const
{
  return (uint16_t)(((SYS_VOLT_CALC_COEFF1 * (m_fFilteredSysVoltage)) + SYS_VOLT_CALC_COEFF2) * 1000.0f);
}

// #############################################################################################//

uint32_t PowerHandler::getLoadVoltage(void) const
{
  // return (uint16_t)((m_fLoadVoltage *  / (PACK_DIV_RB_VALUE))) * 1000.0f);
  return (uint32_t)(((m_fLoadVoltage * (PACK_DIV_RA_VALUE + PACK_DIV_RB_VALUE)) / PACK_DIV_RB_VALUE) * 1000.0f);
}

// #############################################################################################//

void PowerHandler::switchesSetSwitchState(enSwitchesID switchID, enSwitchesState newState)
{
  switch (switchID)
  {
  case SWITCH_PRECHARGE:
  {
    m_preDischargeEnPin.write(newState);
  }
  break;

  case SWITCH_CHARGE:
  {
    m_chargeEnPin.write(newState);
  }
  break;

  case SWITCH_DISCHARGE:
  {
    m_dischargeEnPin.write(newState);
  }
  break;

  default:
    break;
  }
}

// #############################################################################################//

void PowerHandler::switchesDisableAll(void)
{
  m_preDischargeEnPin.write(0);
  m_dischargeEnPin.write(0);
  m_chargeEnPin.write(0);
  resetDwellState();
}

// #############################################################################################//

void PowerHandler::allowedForceOn(bool newState)
{
  m_bAllowedForceOnFlag = newState;
}

// #############################################################################################//

bool PowerHandler::getForceOnState(void) const
{
  return m_bAllowedForceOnFlag;
}

// #############################################################################################//

void PowerHandler::udpateSwitches(void)
{
  // Handle pre charge output
  if (m_ptrPowerElecPackInfoConfig->bPredischargeDesiredFlag && (m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed || m_bAllowedForceOnFlag))
  {
    switchesSetSwitchState(SWITCH_PRECHARGE, SWITCH_SET);
  }
  else
  {
    switchesSetSwitchState(SWITCH_PRECHARGE, SWITCH_RESET);
  }

  // Handle discharge output
  if (m_ptrPowerElecPackInfoConfig->bDischargeDesiredFlag && (m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed || m_bAllowedForceOnFlag))
  {
    switchesSetSwitchState(SWITCH_DISCHARGE, SWITCH_SET);
  }
  else
  {
    switchesSetSwitchState(SWITCH_DISCHARGE, SWITCH_RESET);
  }

  // Handle charge input
  if (m_ptrPowerElecPackInfoConfig->bChargeDesiredFlag && m_ptrPowerElecPackInfoConfig->bChargeAllowed)
  {
    switchesSetSwitchState(SWITCH_CHARGE, SWITCH_SET);
  }
  else
  {
    switchesSetSwitchState(SWITCH_CHARGE, SWITCH_RESET);
  }

  // === NEW: Contactor State Change Logging ===
  static uint8_t s_lastContactorState = 0xFF;
  uint8_t contactorState =
      ((m_ptrPowerElecPackInfoConfig->bPredischargeDesiredFlag && (m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed || m_bAllowedForceOnFlag)) ? 0x01 : 0) |
      ((m_ptrPowerElecPackInfoConfig->bDischargeDesiredFlag && (m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed || m_bAllowedForceOnFlag)) ? 0x02 : 0) |
      ((m_ptrPowerElecPackInfoConfig->bChargeDesiredFlag && m_ptrPowerElecPackInfoConfig->bChargeAllowed) ? 0x04 : 0);

  if (contactorState != s_lastContactorState)
  {
    s_lastContactorState = contactorState;
    InterfaceCommHandler::getInstance()->printToInterface(
        "[CONT] PRE=%s DSG=%s CHG=%s\r\n",
        (contactorState & 0x01) ? "ON" : "OFF",
        (contactorState & 0x02) ? "ON" : "OFF",
        (contactorState & 0x04) ? "ON" : "OFF");
  }
}

// #############################################################################################//

void PowerHandler::setPreCharge(bool newState)
{
  const uint32_t _now = rtos::Kernel::Clock::now().time_since_epoch().count();

  if (_now - m_preStamp < PWR_CONTACTOR_DWELL_MS)
  {
    return;
  }
  if (m_preLastState != newState)
  {
    m_preLastState = newState;
    m_ptrPowerElecPackInfoConfig->bPredischargeDesiredFlag = newState;
    udpateSwitches();
    m_preStamp = _now;
  }
}

// #############################################################################################//

bool PowerHandler::setDisCharge(bool newState)
{
  const uint32_t _now = rtos::Kernel::Clock::now().time_since_epoch().count();

  if (_now - m_dchgStamp >= PWR_CONTACTOR_DWELL_MS && m_dischargeLastState != newState)
  {

    m_ptrPowerElecPackInfoConfig->bDischargeDesiredFlag = newState;
    udpateSwitches();
    m_dchgStamp = _now;
    m_dischargeLastState = newState;
  }

#if USE_PRECHARGE
  if ((m_ptrPowerElecPackInfoConfig->u32ModuleLoadVoltage < m_ptrPowerElecPackInfoConfig->u32ModuleVoltage * (PRECHARGE_PERCENTAGE / 100)) && (m_ptrGenConfig->bUsePrechargeFlag))
  {
    return false;
  }
  else
  {
    return true;
  }

#else
  return true;

#endif
}

// #############################################################################################//

void PowerHandler::setCharge(bool newState)
{
  const uint32_t _now = rtos::Kernel::Clock::now().time_since_epoch().count();

  if (_now - m_chgStamp < PWR_CONTACTOR_DWELL_MS)
  {
    return;
  }
  if (m_chargeLastState != newState)
  {
    m_chargeLastState = newState;
    m_ptrPowerElecPackInfoConfig->bChargeDesiredFlag = newState;
    udpateSwitches();
    m_chgStamp = _now;
  }
}

void PowerHandler::resetDwellState(void)
{
  m_preLastState = false;
  m_dischargeLastState = false;
  m_chargeLastState = false;
}

// #############################################################################################//
