#ifndef __IMPOWER__H_
#define __IMPOWER__H_

#include "Settings.h"


class IMPower
{
public:
  IMPower() {};
  virtual ~IMPower() = default;

  virtual uint16_t getSysVoltage(void) const = 0;
  virtual uint32_t getLoadVoltage(void) const = 0;
  virtual void switchesSetSwitchState(enSwitchesID switchID, enSwitchesState newState) = 0;
  virtual bool getForceOnState(void) const = 0;
  virtual void udpateSwitches(void) = 0;
  
};


#endif // __IMPOWER__H_


    
    
    
    