#include "hw_logging_base.h"
#include <cstdint>


HwLoggingBase::HwLoggingBase(ILSdcard& _sdCard, ILInterfaceComm& _interfaceComm, ILError& _error)
: 
  interfaceComm(_interfaceComm),
  sdCard(_sdCard),
  error(_error)
 {}

// #############################################################################################//

HwLoggingBase::~HwLoggingBase() {}

// #############################################################################################//
