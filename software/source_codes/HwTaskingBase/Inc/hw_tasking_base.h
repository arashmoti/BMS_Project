#ifndef __HW_TASKING_BASE__H_
#define __HW_TASKING_BASE__H_

#include "ITPower.h"
#include "ITDatetime.h"
#include "ITEffect.h"
#include "ITPowerState.h"
#include "ITInverterCan.h"

class HwTaskingBase {
public:
  HwTaskingBase(ITEffect& _effect, ITDatetime& _datetime, ITPower& _pwr, ITPowerState& _pwrState, ITInverterCan& _inverterCan);
  virtual ~HwTaskingBase();

public:
  ITEffect& effect;
  ITDatetime& datetime;
  ITPower& pwr;
  ITPowerState& pwrState;
  ITInverterCan& inverterCan;

};

#endif // __HW_TASKING_BASE__H_
