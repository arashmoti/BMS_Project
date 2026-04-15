#ifndef __TASKING__H_
#define __TASKING__H_

#include "tasking_params.h"
#include "io_periph_defs.h"
#include "Settings.h"
#include "hw_tasking_base.h"


#include "PinNames.h"
#include "Thread.h"
#include "ThisThread.h"
#include "Callback.h"
#include "Mutex.h"
#include "Timer.h"

#include <cstdarg>
#include <cstdint>
#include <vector>

class Tasking
{

public:
  Tasking(HwTaskingBase& _hwTasking);
  virtual ~Tasking();

  void startThread(void);
  void init(void);

private:
  void taskingThread(void);

private:
    
  rtos::Thread m_taskingThreadVar;
  mbed::Timer m_taskingManagerTim;


  HwTaskingBase& m_hwTasking;

  uint32_t m_u32SysTask1s;
  uint32_t m_u32SysTask20ms;
};

#endif // __LOGGING__H_