#ifndef __ERROR_HANDLER__H_
#define __ERROR_HANDLER__H_


// Includes
#include <cstdint>
#include <stdint.h>
#include <string> 
#include "mbed.h"


#include "IOError.h"
#include "ILError.h"


#include "hw_error_base.h"
#include "ISerial.h"

class ErrorHandler : public IOError, public ILError
{
public:
    ErrorHandler(HwErrorBase& _hwErrorBase);
    virtual ~ErrorHandler();

    virtual void saveToEeprom(enBatteryErrors newState, st_errorEepromData errorData) override;
    virtual void saveToSDCard(enBatteryErrors newState, st_errorSdCardData errorSdCardData) override;
    virtual void saveToModuleErrorCounterToEeprom(enModuleErrors curState, uint8_t counter) override;

    virtual void getErrorFromEeprom(st_errorEepromData* errorEepromData, uint8_t errorEepromIndex) override;
    virtual uint8_t getErrorIndexVal(void) const override;
    virtual void resetErrorIndex(void) override;
    virtual void resetErrorFromEeprom(uint8_t errorEepromIndex) override;
    virtual bool getEepromErrorCount(void) override;

    void init(void);

private:
    HwErrorBase& m_hwErrorBase;
    enBatteryErrors m_errorCurrentState;

    uint8_t m_u8ErrorIndex;
    uint8_t m_u8ErrorCurrentIndex;
    uint16_t m_u16ErrorStartAddressToWrite;
          
};


#endif // __ERROR_HANDLER__H_
