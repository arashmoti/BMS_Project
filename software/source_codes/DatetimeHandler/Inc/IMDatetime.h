#ifndef __IMDATETIME__H_
#define __IMDATETIME__H_

#include "stdint.h"

class IMDatetime
{
public:

  IMDatetime() {};
  virtual ~IMDatetime() = default;

  virtual uint8_t getSecond(void) const = 0;
  virtual uint8_t getMinute(void) const = 0;
  virtual uint8_t getHour(void) const = 0;
  virtual uint8_t getDay(void) const = 0;
  virtual uint8_t getMonth(void) const = 0;
  virtual uint8_t getYear(void) const = 0;

};


#endif // __IMDATETIME__H_