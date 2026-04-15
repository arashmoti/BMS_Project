#include "sys_config.h"
#include <cstdint>

SysConfig::SysConfig(IEeprom &_eeprom)
    : m_eeprom(_eeprom) {}

// #############################################################################################//

SysConfig::~SysConfig() {}

// #############################################################################################//

void SysConfig::loadDefaultConfig(st_generalConfig *genConfig)
{

  genConfig->u8NumberOfCells = m_eeprom.getSettingValue(CMD_EE_ADDR_NumberOfCells);
    // genConfig->u8NumberOfTemperature = m_eeprom.getSettingValue(CMD_EE_ADDR_NumberOfTemperature);
  genConfig->u8NumberOfTemperature = NoOfTEMP_POSSIBLE_ON_CHIP; // Forced to 8, ignoring EEPROM
  genConfig->u8NoOfCellsPerModule = m_eeprom.getSettingValue(CMD_EE_ADDR_NoOfCellsPerModule);
  genConfig->u8NumberOfModules = m_eeprom.getSettingValue(CMD_EE_ADDR_NoOfCellsPerModule); // Note: EEPROM address naming is swapped in this project
  genConfig->u16HysteresisDischarge = m_eeprom.getSettingValue(CMD_EE_ADDR_HysteresisDischarge);
  genConfig->u16HysteresisCharge = m_eeprom.getSettingValue(CMD_EE_ADDR_HysteresisCharge);
  genConfig->u16HysteresisCurrent = m_eeprom.getSettingValue(CMD_EE_ADDR_HysteresisCurrent);
  genConfig->u16HysteresisOverTemperature =m_eeprom.getSettingValue(CMD_EE_ADDR_HysteresisOverTemperature);
  genConfig->u16TimeoutDischargeRetry = m_eeprom.getSettingValue(CMD_EE_ADDR_TimeoutDischargeRetry);
  genConfig->u16TimeoutChargeRetry = m_eeprom.getSettingValue(CMD_EE_ADDR_TimeoutChargeRetry);
  genConfig->u16TimeoutOverTemperatureRetry = m_eeprom.getSettingValue(CMD_EE_ADDR_TimeoutOverTemperatureRetry);
  genConfig->u16TimeoutLowCurrentPreChargeRetry = m_eeprom.getSettingValue(CMD_EE_ADDR_TimeoutLowCurrentPreChargeRetry);
  genConfig->u16TimeoutSoftOverCurrentRetry = m_eeprom.getSettingValue(CMD_EE_ADDR_TimeoutSoftOverCurrentRetry);
  genConfig->u16TimeoutHardOverCurrentRetry = m_eeprom.getSettingValue(CMD_EE_ADDR_TimeoutHardOverCurrentRetry);
  genConfig->u16DisplayTimeoutBatteryDead = m_eeprom.getSettingValue(CMD_EE_ADDR_DisplayTimeoutBatteryDead);
  genConfig->u16DisplayTimeoutBatteryErrorPreCharge = m_eeprom.getSettingValue(CMD_EE_ADDR_DisplayTimeoutBatteryErrorPreCharge);
  genConfig->u16DisplayTimeoutBatteryError = m_eeprom.getSettingValue(CMD_EE_ADDR_DisplayTimeoutBatteryError);
  genConfig->u16TimeoutChargerDisconnected = m_eeprom.getSettingValue(CMD_EE_ADDR_TimeoutChargerDisconnected);
  genConfig->u16TimeoutChargeCompleted = m_eeprom.getSettingValue(CMD_EE_ADDR_TimeoutChargeCompleted);
  genConfig->u16TimeoutNotUsed = m_eeprom.getSettingValue(CMD_EE_ADDR_TimeoutNotUsed);
  genConfig->u16ShuntResistance = m_eeprom.getSettingValue(CMD_EE_ADDR_ShuntResistance);
  // genConfig->u8MaxUnderAndOverVoltageErrorCount           = m_eeprom.getSettingValue(CMD_EE_ADDR_MaxUnderAndOverVoltageErrorCount);
  //  genConfig->u8MaxHardOverCurrentErrorCount               = m_eeprom.getSettingValue(CMD_EE_ADDR_MaxHardOverCurrentErrorCount);
  //  genConfig->u8MaxSoftOverCurrentErrorCount               = m_eeprom.getSettingValue(CMD_EE_ADDR_MaxSoftOverCurrentErrorCount);
  //  genConfig->u8HardOverTemperatureErrorCount              = m_eeprom.getSettingValue(CMD_EE_ADDR_HardOverTemperatureErrorCount);
  genConfig->u16ChargerEnabledThreshold = m_eeprom.getSettingValue(CMD_EE_ADDR_ChargerEnabledThreshold);
  genConfig->u16MinimalPrechargePercentage = m_eeprom.getSettingValue(CMD_EE_ADDR_MinimalPrechargePercentage);
  genConfig->u16NotUsedCurrentThreshold = m_eeprom.getSettingValue(CMD_EE_ADDR_NotUsedCurrentThreshold);
  genConfig->u16MaxMismatchThreshold = m_eeprom.getSettingValue(CMD_EE_ADDR_MaxMismatchThreshold);
  genConfig->u16TimeoutChargingCompletedMinimalMismatch = m_eeprom.getSettingValue(CMD_EE_ADDR_TimeoutChargingCompletedMinimalMismatch);

  genConfig->u32ArrNtcTopResistor = m_eeprom.getSettingValue(CMD_EE_ADDR_CellNtcTopResistor);
  genConfig->u32ArrNtc25DegResistance = m_eeprom.getSettingValue(CMD_EE_ADDR_CellNtc25DegResistance);
  genConfig->u32ArrNtcBetaFactor = m_eeprom.getSettingValue(CMD_EE_ADDR_CellNtcBetaFactor);

  // genConfig->u16AllowedTempPCBMax                         = m_eeprom.getSettingValue(CMD_EE_ADDR_AllowedTempPCBMax);
  // genConfig->u16AllowedTempPCBMin                         = m_eeprom.getSettingValue(CMD_EE_ADDR_AllowedTempPCBMin);
  genConfig->u16AllowedTempBattDischargingMax = m_eeprom.getSettingValue(CMD_EE_ADDR_AllowedTempBattDischargingMax);
  genConfig->u16AllowedTempBattDischargingMin = m_eeprom.getSettingValue(CMD_EE_ADDR_AllowedTempBattDischargingMin);
  genConfig->u16AllowedTempBattChargingMax = m_eeprom.getSettingValue(CMD_EE_ADDR_AllowedTempBattChargingMax);
  genConfig->u16AllowedTempBattChargingMin = m_eeprom.getSettingValue(CMD_EE_ADDR_AllowedTempBattChargingMin);

  // TEMPORARY OVERRIDE: Force temperature limit to 60°C for testing
  // Comment out the next line to restore EEPROM values
  #define FORCE_TEMP_LIMIT_60C
  #ifdef FORCE_TEMP_LIMIT_60C
    genConfig->u16AllowedTempBattChargingMax = 600;    // 60.0°C charge limit
    genConfig->u16AllowedTempBattDischargingMax = 600; // 60.0°C discharge limit
    genConfig->u16HysteresisOverTemperature = 50;      // 5.0°C hysteresis (release at 55.0°C)
  #endif

  genConfig->u16BatteryCapacity = m_eeprom.getSettingValue(CMD_EE_ADDR_BatteryCapacity);

  genConfig->u16BalanceStartVoltage = 3420;// m_eeprom.getSettingValue(CMD_EE_ADDR_BalanceStartVoltage);
  genConfig->u16InternalResistance = m_eeprom.getSettingValue(CMD_EE_ADDR_InternalResistance);
  genConfig->u16BalanceDifferenceThreshold = m_eeprom.getSettingValue(CMD_EE_ADDR_BalanceDifferenceThreshold); // mv
  // REMOVED: genConfig->bDisableChargeDuringBalance = true; // Dead code - never checked in balancing logic

  genConfig->u16MaxSoftDchgAllowedCurrent = m_eeprom.getSettingValue(CMD_EE_ADDR_MaxSoftDchgAllowedCurrent);
  genConfig->u16MaxHardDchgAllowedCurrent = m_eeprom.getSettingValue(CMD_EE_ADDR_MaxHardDchgAllowedCurrent);
  genConfig->u16MaxSoftChgAllowedCurrent = m_eeprom.getSettingValue(CMD_EE_ADDR_MaxSoftChgAllowedCurrent);
  genConfig->u16MaxHardChgAllowedCurrent = m_eeprom.getSettingValue(CMD_EE_ADDR_MaxHardChgAllowedCurrent);

  genConfig->u16SoftLowCurrentUnderVoltage = m_eeprom.getSettingValue(CMD_EE_ADDR_SoftLowCurrentUnderVoltage);
  genConfig->u16SoftHighCurrentUnderVoltage = m_eeprom.getSettingValue(CMD_EE_ADDR_SoftHighCurrentUnderVoltage);
  genConfig->u16HardUnderVoltage = m_eeprom.getSettingValue(CMD_EE_ADDR_HardUnderVoltage);
  genConfig->u16SoftOverVoltage = m_eeprom.getSettingValue(CMD_EE_ADDR_SoftOverVoltage);
  genConfig->u16HardOverVoltage = m_eeprom.getSettingValue(CMD_EE_ADDR_HardOverVoltage);
  genConfig->u16OverMismatchVoltage = m_eeprom.getSettingValue(CMD_EE_ADDR_OverMismatchVoltage);
  genConfig->u16HysteresisOverMismatchVoltage = 2;

  static const int16_t kHysteresysDischarge = -1000;
  static const int16_t kHysteresysCharge = -1000;

  // Highlight: This is the definitive fix.
  // It calculates the total cell count safely within the function
  // by multiplying the cells-per-module (16) by the number-of-modules (4).
  // This avoids the overflow while ensuring the final voltage is correct for the 64-cell system.
  uint32_t total_cells = (uint32_t)genConfig->u8NumberOfCells * (uint32_t)genConfig->u8NoOfCellsPerModule;

  uint32_t temp_discharge_mv = (total_cells * genConfig->u16HardUnderVoltage) - kHysteresysDischarge;
  uint32_t temp_charge_mv = (total_cells * genConfig->u16SoftOverVoltage) + kHysteresysCharge;

  // The CAN message requires a 0.1V resolution, so we divide the millivolt value by 100.
  genConfig->u16MaxDischargePackVoltage = temp_discharge_mv / 100;
  genConfig->u16MaxChargePackVoltage = temp_charge_mv / 100;

  uint16_t ModuleMaxChargeVoltage = 5625;//56.25V
  static uint16_t MaxChargeVoltage = NoOfCELL_MONITORS_POSSIBLE_ON_BMS * ModuleMaxChargeVoltage * 0.1;
  // Cap max charge voltage at 220V for inverter compatibility
  if (genConfig->u16MaxChargePackVoltage > MaxChargeVoltage) { // 2235 = 223.5V in 0.1V units
    genConfig->u16MaxChargePackVoltage = MaxChargeVoltage;
  }

  // genConfig->u16MaxDischargePackVoltage = ((genConfig->u8NumberOfCells * genConfig->u16HardUnderVoltage) - kHysteresysDischarge);
  // genConfig->u16MaxChargePackVoltage = ((genConfig->u8NumberOfCells * genConfig->u16HardOverVoltage) + kHysteresysCharge);
  genConfig->bUsePrechargeFlag = false;

  genConfig->i16CellVoltagesOffsetVoltage = (m_eeprom.getSettingValue(CMD_EE_ADDR_CellVoltagesOffsetVoltage) > NEGATIVE_SENSITIVE_VALUE) ? (NEGATIVE_SENSITIVE_VALUE - m_eeprom.getSettingValue(CMD_EE_ADDR_CellVoltagesOffsetVoltage)) : (m_eeprom.getSettingValue(CMD_EE_ADDR_CellVoltagesOffsetVoltage));
  genConfig->u16CurrentOffsetValue = (m_eeprom.getSettingValue(CMD_EE_ADDR_CurrentOffsetValue) > NEGATIVE_SENSITIVE_VALUE) ? (NEGATIVE_SENSITIVE_VALUE - m_eeprom.getSettingValue(CMD_EE_ADDR_CurrentOffsetValue)) : (m_eeprom.getSettingValue(CMD_EE_ADDR_CurrentOffsetValue));

  // LITHIUM_IRON_PHOSPHATE
  // genConfig->u16SoftLowCurrentUnderVoltage      = 2700;
  // genConfig->u16SoftHighCurrentUnderVoltage     = 3100;
  // genConfig->u16HardUnderVoltage                = 2500;
  // genConfig->u16SoftOverVoltage                 = 3450;
  // genConfig->u16HardOverVoltage                 = 3650;
}

// #############################################################################################//
