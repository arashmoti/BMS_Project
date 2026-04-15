
#include "datetime_handler.h"
#include <cstdint>

// #############################################################################################//

DatetimeHandler::DatetimeHandler()
    : _i2c(EEPROM_RTC_SDA_PIN, EEPROM_RTC_SCL_PIN),
      m_dateInfo{0}
{}

// #############################################################################################// 

DatetimeHandler::~DatetimeHandler() {}

// #############################################################################################// 

void DatetimeHandler::init(void)
{
  RtcArgs::st_dateInfo_t dttime;
  dttime.u8Second   = 0;
  dttime.u8Minute   = 2;
  dttime.u8Hour     = 13;

  dttime.u8Day      = 24;
  dttime.u8Month    = 7;
  dttime.u8Year     = 25;
  setDate(&dttime);
}

// #############################################################################################//

void DatetimeHandler::setRegisterVal(uint8_t u8RegisterAddr, uint8_t u8RegisterVal)
{
    static uint8_t c_arru8tempVar[4U];
    static uint8_t c_u8TempReg = 0;

    if (u8RegisterAddr != CONTROL_REGISTER1_ADDRESS || u8RegisterAddr != CONTROL_REGISTER2_ADDRESS)
    {
        c_u8TempReg = (uint8_t)(u8RegisterVal % 10);
        c_u8TempReg += (uint8_t)(u8RegisterVal * 0.1F) << 4U;
        c_arru8tempVar[0U] = u8RegisterAddr;
        c_arru8tempVar[1U] = c_u8TempReg;
    }
    else
    {
        c_arru8tempVar[0U] = u8RegisterAddr;
        c_arru8tempVar[1U] = c_u8TempReg;
    }

    _i2c.write((int)PCF85063A_SLAVE_WRITE_ADDRESS, (char *)c_arru8tempVar, 2U);
}

// #############################################################################################//

void DatetimeHandler::task(void)
{
    static uint8_t c_arru8tempVar[11U];
    static uint8_t c_u8TempVal = 0;

    _i2c.write((int)PCF85063A_SLAVE_READ_ADDRESS, (char *)CONTROL_REGISTER1_ADDRESS, 1U);
    _i2c.read((int)PCF85063A_SLAVE_READ_ADDRESS, (char *)c_arru8tempVar, 11U);

    c_u8TempVal = c_arru8tempVar[4U] & 0x7FU;
    m_dateInfo.u8Second = c_u8TempVal & 0x0F;
    c_u8TempVal = c_u8TempVal >> 4U;
    m_dateInfo.u8Second += c_u8TempVal * 10U;

    c_u8TempVal = c_arru8tempVar[5U] & 0x7FU;
    m_dateInfo.u8Minute = c_u8TempVal & 0x0F;
    c_u8TempVal = c_u8TempVal >> 4U;
    m_dateInfo.u8Minute += c_u8TempVal * 10U;

    c_u8TempVal = c_arru8tempVar[6U] & 0x3FU;
    m_dateInfo.u8Hour = c_u8TempVal & 0x0F;
    c_u8TempVal = c_u8TempVal >> 4U;
    m_dateInfo.u8Hour += c_u8TempVal * 10U;

    c_u8TempVal = c_arru8tempVar[7U] & 0x3FU;
    m_dateInfo.u8Day = c_u8TempVal & 0x0F;
    c_u8TempVal = c_u8TempVal >> 4U;
    m_dateInfo.u8Day += c_u8TempVal * 10U;

    c_u8TempVal = c_arru8tempVar[9U] & 0x1FU;
    m_dateInfo.u8Month = c_u8TempVal & 0x0F;
    c_u8TempVal = c_u8TempVal >> 4U;
    m_dateInfo.u8Month += c_u8TempVal * 10U;

    c_u8TempVal = c_arru8tempVar[10U] & 0xFFU;
    m_dateInfo.u8Year = c_u8TempVal & 0x0F;
    c_u8TempVal = c_u8TempVal >> 4U;
    m_dateInfo.u8Year += c_u8TempVal * 10U;

}

// #############################################################################################//

void DatetimeHandler::setDate(RtcArgs::st_dateInfo_t* date)
{
    setRegisterVal(SECONDS_ADDRESS, date->u8Second);
    setRegisterVal(MINUTES_ADDRESS, date->u8Minute);
    setRegisterVal(HOURS_ADDRESS, date->u8Hour);
    setRegisterVal(DAYS_ADDRESS, date->u8Day);
    setRegisterVal(MONTHS_ADDRESS, date->u8Month);
    setRegisterVal(YEARS_ADDRESS, date->u8Year);
}


// #############################################################################################//

uint8_t DatetimeHandler::getSecond(void) const
{
  return m_dateInfo.u8Second;
}


// #############################################################################################//

uint8_t DatetimeHandler::getMinute(void) const
{
  return m_dateInfo.u8Minute;
}

// #############################################################################################//

uint8_t DatetimeHandler::getHour(void) const
{
  return m_dateInfo.u8Hour;
}

// #############################################################################################//

uint8_t DatetimeHandler::getDay(void) const
{
  return m_dateInfo.u8Day;
}

// #############################################################################################//

uint8_t DatetimeHandler::getMonth(void) const
{
  return m_dateInfo.u8Month;
}

// #############################################################################################//

uint8_t DatetimeHandler::getYear(void) const
{
  return m_dateInfo.u8Year;
}


