#ifndef __IMSTATEOFCHARGE__H_
#define __IMSTATEOFCHARGE__H_

#include "Settings.h"

class IMStateOfCharge
{
public:
  IMStateOfCharge() {};
  virtual ~IMStateOfCharge() = default;

  // The function is updated to take all necessary parameters for dynamic IR compensation.
  virtual uint16_t getVoltageToSoc(float cell_voltage, int32_t pack_current, int32_t temperature_mc) = 0;
};

#endif