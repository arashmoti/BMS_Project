#include "hw_operation_base.h"
#include <cstdint>

HwOperationBase::HwOperationBase(IOEffect &_effect, IOPowerState &_pwrState, IOPower &_pwr, IOError &_error, IMbq79616 &_bq)
    : pwr(_pwr),
      effect(_effect),
      pwrState(_pwrState),
      error(_error),
      bq(_bq)
{
}

// #############################################################################################//

HwOperationBase::~HwOperationBase() {}

// #############################################################################################//
