#ifndef __POWER_HANDLER_PARAMS__H_
#define __POWER_HANDLER_PARAMS__H_

#include <cstdint>
#include "Settings.h"

#define ADC_VREF_VALUE            3.277f
#define PRECHARGE_PERCENTAGE      70
#define PRECHARGE_CURRENT_LIM     130
#define I_IN_FILTER_CONST         0.006
#define V_IN_FILTER_CONST         0.006

namespace PowerArgs
{

#pragma pack(push)
#pragma pack(1)

#pragma pack(pop)

}

#endif // __POWER_HANDLER_PARAMS__H_