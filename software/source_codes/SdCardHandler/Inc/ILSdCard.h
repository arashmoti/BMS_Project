#ifndef __ILSDCARD__H_
#define __ILSDCARD__H_

#include "stddef.h"
#include "Settings.h"
#include <cstdint>


class ILSdcard
{
public:
  ILSdcard() {};
  virtual ~ILSdcard() = default;
    
  virtual uint8_t cellVoltageLogging(uint32_t time, uint8_t module_num, uint16_t* cellInfo) = 0;
  virtual uint8_t packTemperatureLogging(uint32_t time, uint8_t module_num, int32_t* tempInfo) = 0;
  virtual uint8_t systemInformationLogging(uint32_t time, st_powerElecPackInfoConfig* powerElecPackInfoConfig) = 0;
  virtual uint8_t getSdCardStatus(void) = 0;
  virtual void resetSdCardStatus(void) = 0;

  // Human-readable event line (CSV): timeLogging,event
  virtual void logEvent(const char *fmt, ...) = 0;

};


#endif // __ILSDCARD__H_
