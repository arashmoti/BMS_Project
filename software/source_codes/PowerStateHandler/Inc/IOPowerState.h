#ifndef __IOPOWERSTATE__H_
#define __IOPOWERSTATE__H_

#include "stddef.h"
#include "power_state_handler_params.h"


class IOPowerState
{
public:
  IOPowerState() {};
  virtual ~IOPowerState() = default;

  virtual bool buttonForceOnRequest(void) = 0;
  virtual bool getButtonPressedState(void) const = 0;
  virtual bool getButtonPressedOnTurnon(void) const = 0;
  virtual bool getPowerdownRequest(void) const = 0;
  virtual void setPwrBtnLedPin(int value) = 0;
  virtual void setPwrBtnEnPin(int value) = 0;
  virtual void task(void) = 0;
  virtual bool chargeDetected(void) = 0;

};


#endif // __IOPOWERSTATE__H_