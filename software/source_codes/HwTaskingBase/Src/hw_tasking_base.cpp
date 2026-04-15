#include "hw_tasking_base.h"
#include <cstdint>


HwTaskingBase::HwTaskingBase(ITEffect& _effect, ITDatetime& _datetime, ITPower& _pwr, ITPowerState& _pwrState, ITInverterCan& _inverterCan)
: 
  effect(_effect),
  datetime(_datetime),
  pwr(_pwr),
  pwrState(_pwrState),
  inverterCan(_inverterCan)
 {}

// #############################################################################################//

HwTaskingBase::~HwTaskingBase() {}

// #############################################################################################//
