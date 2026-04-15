#ifndef __HW_LOGGING_BASE__H_
#define __HW_LOGGING_BASE__H_

#include "ILSdCard.h"
#include "ILInterfaceComm.h"
#include "ILError.h"

class HwLoggingBase {
public:
  HwLoggingBase(ILSdcard& _sdCard, ILInterfaceComm& _interfaceComm, ILError& _error);
  virtual ~HwLoggingBase();

public:
  ILInterfaceComm& interfaceComm;
  ILSdcard& sdCard;
  ILError& error;
};

#endif // __HW_LOGGING_BASE__H_
