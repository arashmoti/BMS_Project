#include "hw_error_base.h"
#include <cstdint>


HwErrorBase::HwErrorBase(IESdcard& _sdCard, IEEeprom& _eeprom)
: 
  sdCard(_sdCard),
  eeprom(_eeprom)
 {
 }

// #############################################################################################//

HwErrorBase::~HwErrorBase() {}

// #############################################################################################//
