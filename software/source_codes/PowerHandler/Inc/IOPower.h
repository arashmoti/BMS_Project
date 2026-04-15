#ifndef __IOPOWER__H_
#define __IOPOWER__H_

#include "Settings.h" // Required for enSwitchesID and enSwitchesState

class IOPower
{
public:
  IOPower() {};
  virtual ~IOPower() = default;
  
  virtual void switchesSetSwitchState(enSwitchesID switchID, enSwitchesState newState) = 0;

  virtual void allowedForceOn(bool newState) = 0;

  virtual void setPreCharge(bool newState);
  virtual bool setDisCharge(bool newState);
  virtual void setCharge(bool newState);

  virtual void switchesDisableAll(void) = 0;
};

#endif // __IOPOWER__H_
