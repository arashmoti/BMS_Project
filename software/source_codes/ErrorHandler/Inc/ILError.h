#ifndef __ILERROR__H_
#define __ILERROR__H_

#include "Settings.h"
#include <cstdint>

class ILError
{
public:
    ILError() {};
    virtual ~ILError() = default;

    virtual void getErrorFromEeprom(st_errorEepromData* errorEepromData, uint8_t errorEepromIndex) = 0;
    virtual uint8_t getErrorIndexVal(void) const = 0;
    virtual void resetErrorIndex(void) = 0;
    virtual void resetErrorFromEeprom(uint8_t errorIndex) = 0;
};

#endif // __ILERROR__H_
