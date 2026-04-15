#include <cstdint>
#include <memory>
#include "tasking.h"

#include "Callback.h"
#include "Timer.h"
#include "Watchdog.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdlib>

// #############################################################################################//

Tasking::Tasking(HwTaskingBase &_hwTasking)
    : m_hwTasking(_hwTasking),
      m_u32SysTask1s(0),
      m_u32SysTask20ms(0),
      m_taskingThreadVar(TASKING_THREAD_PRIORITY, TASKING_THREAD_STACK_SIZE, nullptr, "Tasking")
{
  m_taskingManagerTim.start();
}

// #############################################################################################//

Tasking::~Tasking()
{
}

// #############################################################################################//

void Tasking::startThread()
{
  m_taskingThreadVar.start(mbed::callback(this, &Tasking::taskingThread));
}

// #############################################################################################//

void Tasking::init(void)
{
}

// #############################################################################################//

void Tasking::taskingThread()
{
  while (true)
  {

    if ((TASKING_GET_TICK(m_taskingManagerTim) - m_u32SysTask20ms) >= 20)
    {
      m_u32SysTask20ms = TASKING_GET_TICK(m_taskingManagerTim);
      m_hwTasking.inverterCan.task();
    }

    if ((TASKING_GET_TICK(m_taskingManagerTim) - m_u32SysTask1s) >= 1000)
    {
      m_u32SysTask1s = TASKING_GET_TICK(m_taskingManagerTim);
      m_hwTasking.datetime.task();
    }

    m_hwTasking.pwr.task();
    m_hwTasking.effect.task();
    m_hwTasking.pwrState.task();

#if defined(IS_DEVELOPMENT)
    m_hwTasking.pwrState.task();
#endif

    ThisThread::sleep_for(1ms);
  }
}
