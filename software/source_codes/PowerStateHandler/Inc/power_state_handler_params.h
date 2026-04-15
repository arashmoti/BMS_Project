#ifndef __POWER_STATE_HANDLER_PARAMS__H_
#define __POWER_STATE_HANDLER_PARAMS__H_

#include <cstdint>

#define PWR_STATE_GET_TICK(x)                 std::chrono::duration_cast<std::chrono::milliseconds>(x.elapsed_time()).count() 

namespace PowerStateArgs
  {

#pragma pack(push)
#pragma pack(1)
    typedef struct _stPowerBtnVarsConfig
    {
      uint32_t u32ButtonPressedTimeStamp;
      uint32_t u32ButtonPressedDuration;
      uint32_t u32PowerDownTimeout;
      bool bLastButtonFirstPress;
      bool bLastButtonPressedVar;
      bool bPulsePowerDownDesired;
      bool bForceOnDesired;
      bool bPressedVar;
      bool bBootButtonPressed;  // Latched flag: true if button was pressed at boot, stays true until consumed
    }st_powerBtnVarsConfig;
#pragma pack(pop)
}


#endif // __POWER_STATE_HANDLER_PARAMS__H_
