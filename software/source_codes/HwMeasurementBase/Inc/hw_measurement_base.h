#ifndef __HW_MEASUREMENT_BASE__H_
#define __HW_MEASUREMENT_BASE__H_

#include "IMDatetime.h"
#include "IMPower.h"
#include "IMbq79616.h"
#include "IMInverterCan.h" // <-- Re-add this
#include "IMStateOfCharge.h"
#include "ISerial.h"
#include "ILInterfaceComm.h"
#include "IOEffect.h"
#include "IOPowerState.h"
// Highlight: Added include for the EEPROM interface.
#include "IEeprom.h"
class HwMeasurementBase
{
public:
  HwMeasurementBase(IMbq79616 &_bq,
                    IMDatetime &_datetime,
                    IMPower &_pwr,
                    ILInterfaceComm &_interfaceComm,
                    IMInverterCan &_inverterCan,
                    IMStateOfCharge &_soc,
                    ISerial &_serial,
                    IOEffect &_effect,
                    IOPowerState &_pwrState,
                    IEeprom &_eeprom);

  virtual ~HwMeasurementBase();

public:
  IMDatetime &datetime;
  IMPower &pwr;
  IMbq79616 &bq;
  ILInterfaceComm &interfaceComm;
  IMInverterCan &inverterCan; // <-- Re-add this
  IMStateOfCharge &soc;
  ISerial &serial;
  IOEffect &effect;
  IOPowerState &pwrState;
  // Highlight: Added the EEPROM member reference.
  IEeprom &eeprom;
};

#endif // __HW_MEASUREMENT_BASE__H_