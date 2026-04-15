#ifndef __IEEEPROM__H_
#define __IEEEPROM__H_

#include "stdint.h"
#include "Settings.h"
#include <cstdint>


class IEEeprom
{
public:
  IEEeprom() {};
  virtual ~IEEeprom() = default;

  virtual uint16_t getSettingValue(enEepromValuesIndex ee_index) = 0;
  virtual void setSettingValue(enEepromValuesIndex eeprom_index, uint16_t value) = 0;
  virtual void setErrorValue(uint16_t eeprom_index, uint8_t value) = 0;
  virtual uint8_t getErrorValue(uint16_t eepromIndex) = 0;

};


#endif // __IEEPROM_INFO__H_
