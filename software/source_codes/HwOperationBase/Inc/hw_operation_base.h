#ifndef __HW_OPERATION_BASE__H_
#define __HW_OPERATION_BASE__H_

#include "IOPower.h"
#include "IOPowerState.h"
#include "IOEffect.h"
#include "IOError.h"
#include "IMbq79616.h"
class HwOperationBase
{
public:
  HwOperationBase(IOEffect &_effect, IOPowerState &_pwrState, IOPower &_pwr, IOError &_error, IMbq79616 &_bq);
  virtual ~HwOperationBase();

public:
  IOPower &pwr;
  IOEffect &effect;
  IOPowerState &pwrState;
  IOError &error;
  IMbq79616 &bq;
};

#endif // __HW_OPERATIONAL_BASE__H_
