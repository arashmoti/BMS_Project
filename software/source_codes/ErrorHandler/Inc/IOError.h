#ifndef __IOERROR__H_
#define __IOERROR__H_

#include "Settings.h"

class IOError
{
public:
    IOError() {};
    virtual ~IOError() = default;

    virtual void saveToEeprom(enBatteryErrors newState, st_errorEepromData errorEepromData) = 0;
    virtual void saveToSDCard(enBatteryErrors newState, st_errorSdCardData errorSdCardData) = 0;
    virtual void saveToModuleErrorCounterToEeprom(enModuleErrors curState, uint8_t counter) = 0;
    virtual bool getEepromErrorCount(void) = 0;
};


#endif // __IOERROR__H_
