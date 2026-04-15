#ifndef __IESDCARD__H_
#define __IESDCARD__H_

#include "stdint.h"
#include "Settings.h"


class IESdcard
{
public:
  IESdcard() {};
  virtual ~IESdcard() = default;

  virtual uint8_t sdCardErrorCallback(st_errorSdCardData* errorSDcardData) = 0;
  virtual uint8_t getSdCardStatus(void) = 0;
  virtual void resetSdCardStatus(void) = 0;

};


#endif // __ISDCARD_INFO__H_