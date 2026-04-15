#include "hw_measurement_base.h"
#include <cstdint>

HwMeasurementBase::HwMeasurementBase(IMbq79616 &_bq, IMDatetime &_datetime, IMPower &_pwr, ILInterfaceComm &_interfaceComm, IMInverterCan &_inverterCan, IMStateOfCharge &_soc, ISerial &_serial, IOEffect &_effect, IOPowerState &_pwrState, IEeprom &_eeprom)
    : datetime(_datetime),
      pwr(_pwr),
      bq(_bq),
      interfaceComm(_interfaceComm),
      inverterCan(_inverterCan), // <-- Re-add this
      soc(_soc),
      serial(_serial),
      effect(_effect),
      pwrState(_pwrState),
      eeprom(_eeprom)
{
}

// #############################################################################################//

HwMeasurementBase::~HwMeasurementBase() {}

// #############################################################################################//
