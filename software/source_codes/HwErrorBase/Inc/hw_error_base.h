#ifndef __HW_ERROR_BASE__H_
#define __HW_ERROR_BASE__H_


#include "IESdCard.h"
#include "IEEeprom.h"

class HwErrorBase {
public:
  HwErrorBase(IESdcard& _sdCard, IEEeprom& _eeprom);
  virtual ~HwErrorBase();

public:
  IESdcard& sdCard;
  IEEeprom& eeprom;
};

#endif // __HW_ERROR_BASE__H_
