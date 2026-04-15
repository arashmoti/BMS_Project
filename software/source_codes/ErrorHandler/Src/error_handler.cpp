
#include "error_handler.h"
#include <cstdint>
#include <stdint.h>

namespace {
constexpr uint16_t kErrorRecordSize = 8;
constexpr uint16_t kMaxErrorRecords =
    (EEPROM_ERROR_INDEX_ADDRESS - ERROR_START_ADDRESS) / kErrorRecordSize;
} // namespace



ErrorHandler::ErrorHandler(HwErrorBase& _hwErrorBase)
    : m_hwErrorBase(_hwErrorBase),
      m_u8ErrorIndex(0),
      m_u8ErrorCurrentIndex(0),
      m_u16ErrorStartAddressToWrite(0)
{
}

// #############################################################################################//

ErrorHandler::~ErrorHandler() {}

// #############################################################################################//

void ErrorHandler::init(void)
{
  m_u8ErrorIndex = m_hwErrorBase.eeprom.getSettingValue(enEepromValuesIndex::CMD_EE_ADDR_ErrorIndexCount);
  if (kMaxErrorRecords == 0) {
    m_u8ErrorIndex = 0;
    m_u16ErrorStartAddressToWrite = ERROR_START_ADDRESS;
    return;
  }
  if (m_u8ErrorIndex >= kMaxErrorRecords) {
    m_u8ErrorIndex = 0;
  }
  // resetErrorIndex();
  // m_u8ErrorIndex = 0;
  m_u16ErrorStartAddressToWrite = (m_u8ErrorIndex * kErrorRecordSize) + ERROR_START_ADDRESS;
  // m_serial.serialPrint("Error Index : %d\r\n", m_u8ErrorIndex);
  // m_serial.serialPrint("Error Addrs : %d\r\n", m_u16ErrorStartAddressToWrite);
}

// #############################################################################################//

void ErrorHandler::saveToEeprom(enBatteryErrors newState, st_errorEepromData errorData) 
{
    if (kMaxErrorRecords == 0) {
        return;
    }
    if (m_u8ErrorIndex >= kMaxErrorRecords) {
        m_u8ErrorIndex = 0;
    }
    m_u16ErrorStartAddressToWrite =
        (m_u8ErrorIndex * kErrorRecordSize) + ERROR_START_ADDRESS;
    if ((m_errorCurrentState != newState))
    {
        switch (newState)
        {
        case SYS_ERROR_CELL_OVER_VOLTAGE:
        {
        }
        break;

        case SYS_ERROR_CELL_UNDER_VOLTAGE:
        {
        }
        break;

        case SYS_ERROR_CELL_OVER_TEMPERATURE_CHARGE:
        {
        }
        break;

        case SYS_ERROR_CELL_OVER_TEMPERATURE_DISCHARGE:
        {

        }
        break;

        case SYS_ERROR_CELL_UNDER_TEMPERATURE_CHARGE:
        {

        }
        break;

        case SYS_ERROR_CELL_UNDER_TEMPERATURE_DISCHARGE:
        {

        }
        break;

        case SYS_WARNING_NO_CHARGE:
        {

        }
        break;

        case SYS_WARNING_NO_DISCHARGE:
        {
        }
        break;

        default:
            break;
        }
  }
  m_u8ErrorIndex++;
  m_hwErrorBase.eeprom.setErrorValue(m_u16ErrorStartAddressToWrite++, errorData.errorType);
  m_hwErrorBase.eeprom.setErrorValue(m_u16ErrorStartAddressToWrite++, errorData.operationState);
  m_hwErrorBase.eeprom.setErrorValue(m_u16ErrorStartAddressToWrite++, errorData.datetimeInfo.u8SysSecond);
  m_hwErrorBase.eeprom.setErrorValue(m_u16ErrorStartAddressToWrite++, errorData.datetimeInfo.u8SysMinute);
  m_hwErrorBase.eeprom.setErrorValue(m_u16ErrorStartAddressToWrite++, errorData.datetimeInfo.u8SysHour);
  m_hwErrorBase.eeprom.setErrorValue(m_u16ErrorStartAddressToWrite++, errorData.datetimeInfo.u8SysDay);
  m_hwErrorBase.eeprom.setErrorValue(m_u16ErrorStartAddressToWrite++, errorData.datetimeInfo.u8SysMonth);
  m_hwErrorBase.eeprom.setErrorValue(m_u16ErrorStartAddressToWrite++, errorData.datetimeInfo.u8SysYear);
  m_hwErrorBase.eeprom.setSettingValue(enEepromValuesIndex::CMD_EE_ADDR_ErrorIndexCount, m_u8ErrorIndex);
  m_errorCurrentState = newState;

  // m_serial.serialPrint("Error was Saved\n\r");

}

// #############################################################################################//

void ErrorHandler::saveToModuleErrorCounterToEeprom(enModuleErrors curState, uint8_t counter) {
  
}

// #############################################################################################//

void ErrorHandler::getErrorFromEeprom(st_errorEepromData* errorEepromData, uint8_t errorEepromIndex) {
  errorEepromData->errorType          = (m_hwErrorBase.eeprom.getErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 0));
  errorEepromData->operationState     = (m_hwErrorBase.eeprom.getErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 1));
  errorEepromData->datetimeInfo.u8SysSecond        = (m_hwErrorBase.eeprom.getErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 2));
  errorEepromData->datetimeInfo.u8SysMinute        = (m_hwErrorBase.eeprom.getErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 3));
  errorEepromData->datetimeInfo.u8SysHour          = (m_hwErrorBase.eeprom.getErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 4));
  errorEepromData->datetimeInfo.u8SysDay           = (m_hwErrorBase.eeprom.getErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 5));
  errorEepromData->datetimeInfo.u8SysMonth         = (m_hwErrorBase.eeprom.getErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 6));
  errorEepromData->datetimeInfo.u8SysYear          = (m_hwErrorBase.eeprom.getErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 7)); 
}

// #############################################################################################//

void ErrorHandler::resetErrorFromEeprom(uint8_t errorEepromIndex) {
  m_hwErrorBase.eeprom.setErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 0, 0xFF);
  m_hwErrorBase.eeprom.setErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 1, 0xFF);
  m_hwErrorBase.eeprom.setErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 2, 0xFF);
  m_hwErrorBase.eeprom.setErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 3, 0xFF);
  m_hwErrorBase.eeprom.setErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 4, 0xFF);
  m_hwErrorBase.eeprom.setErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 5, 0xFF);
  m_hwErrorBase.eeprom.setErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 6, 0xFF);
  m_hwErrorBase.eeprom.setErrorValue((errorEepromIndex * 8) + ERROR_START_ADDRESS + 7, 0xFF); 
}

// #############################################################################################//

uint8_t ErrorHandler::getErrorIndexVal(void) const {
  return m_u8ErrorIndex;
}

// #############################################################################################//

void ErrorHandler::resetErrorIndex(void) {
  m_hwErrorBase.eeprom.setSettingValue(enEepromValuesIndex::CMD_EE_ADDR_ErrorIndexCount, 0);
  m_u8ErrorIndex = 0;
}

// #############################################################################################//

bool ErrorHandler::getEepromErrorCount(void) {
  static bool _bEepromEmptyFlag = false;
  if(m_hwErrorBase.eeprom.getSettingValue(enEepromValuesIndex::CMD_EE_ADDR_MappedEepromVal) != EEPROM_ERROR_INDEX_ADDRESS)
    _bEepromEmptyFlag = true;
  return _bEepromEmptyFlag;
}

// #############################################################################################//

void ErrorHandler::saveToSDCard(enBatteryErrors newState, st_errorSdCardData errorSdCardData) {
  // m_serial.printToInterface("Error Type  : %d\r\n", errorSdCardData.errorType);
  // m_serial.printToInterface("Op State    : %d\r\n", errorSdCardData.operationState);
  // m_serial.printToInterface("Second      : %d\r\n", errorSdCardData.datetimeInfo.u8SysSecond);
  // m_serial.printToInterface("Minute      : %d\r\n", errorSdCardData.datetimeInfo.u8SysMinute);
  // m_serial.printToInterface("Hour        : %d\r\n", errorSdCardData.datetimeInfo.u8SysHour);
  // m_serial.printToInterface("Day         : %d\r\n", errorSdCardData.datetimeInfo.u8SysDay);
  // m_serial.printToInterface("Month       : %d\r\n", errorSdCardData.datetimeInfo.u8SysMonth);
  // m_serial.printToInterface("Year        : %d\r\n", errorSdCardData.datetimeInfo.u8SysYear);

  // m_serial.printToInterface("Voltage     : %d\r\n", errorSdCardData.u16SysVoltage);
  // m_serial.printToInterface("Current     : %d\r\n", errorSdCardData.i32SysCurrent);
  // m_serial.printToInterface("T PCB       : %d\r\n", errorSdCardData.i32PcbTemperature);
  // m_serial.printToInterface("T Cell      : %d\r\n", errorSdCardData.i32CellTemperatureAverage);
  // m_serial.printToInterface("Cell        : %d\r\n", errorSdCardData.u16CellVoltageAverage);

  // m_serial.printToInterface("C Min       : %d\r\n", errorSdCardData.u16CellVoltageMin);
  // m_serial.printToInterface("C Max       : %d\r\n", errorSdCardData.u16CellVoltageMax);
  // m_serial.printToInterface("T Min       : %d\r\n", errorSdCardData.i32TemperatureMin);
  // m_serial.printToInterface("T Max       : %d\r\n", errorSdCardData.i32TemperatureMax);
  if(m_hwErrorBase.sdCard.sdCardErrorCallback(&errorSdCardData) == 255)
  {
    //m_serial.printToInterface("All data have been successfully recorded\r\n", errorSdCardData.i32TemperatureMax);
  }

  else {
    //m_serial.printToInterface("SD Card Error : %d\r\n", m_hwErrorBase.sdCard.sdCardErrorCallback(&errorSdCardData));
  }
  

}
