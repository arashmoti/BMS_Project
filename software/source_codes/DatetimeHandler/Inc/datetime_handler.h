#ifndef __DATETIME_HANDLER__H_
#define __DATETIME_HANDLER__H_


// Includes
#include <cstdint>
#include <string> 
#include "mbed.h"
#include "datetime_handler_params.h"
#include "io_periph_defs.h"

#include "IDatetime.h"
#include "IMDatetime.h"
#include "ITDatetime.h"

class DatetimeHandler : public IDatetime, public IMDatetime, public ITDatetime
{
public:
    DatetimeHandler();
    virtual ~DatetimeHandler();

    virtual void task(void) override;
    
    virtual void setDate(RtcArgs::st_dateInfo_t* date) override;

    virtual uint8_t getSecond(void) const override;
    virtual uint8_t getMinute(void) const override;
    virtual uint8_t getHour(void) const override;
    virtual uint8_t getDay(void) const override;
    virtual uint8_t getMonth(void) const override;
    virtual uint8_t getYear(void) const override;

public:

  void init(void);

private:
    void setRegisterVal(uint8_t u8RegisterAddr, uint8_t u8RegisterVal);

private:
    I2C _i2c;
    RtcArgs::st_dateInfo_t m_dateInfo;             
};


#endif // __DATETIME_HANDLER__H_
