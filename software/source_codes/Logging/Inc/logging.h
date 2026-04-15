#ifndef __LOGGING__H_
#define __LOGGING__H_

#include "logging_params.h"
#include "io_periph_defs.h"
#include "Settings.h"
#include "hw_logging_base.h"
#include "lib_buffer.h"

#include "PinNames.h"
#include "Thread.h"
#include "ThisThread.h"
#include "Callback.h"
#include "Mutex.h"
#include "Timer.h"
#include "Ticker.h"

#include <cstdarg>
#include <cstdint>
#include <vector>

class Logging
{

public:
  // Logging(HwLoggingBase& _hwLogging);
  Logging(HwLoggingBase &_hwLogging, Mail<st_mail_cell_voltages_t, LOGGING_CELL_VOLTAGE_MAIL_SIZE> &mailCellVoltageBox);

  virtual ~Logging();

  void startThread(void);
  void init(st_generalConfig *genConfig, st_powerElecPackInfoConfig *powerElecPackInfoConfig);

private:
  void loggingThread(void);
  void subTaskInterfacePageLiveRackUpdate(void);
  void subTaskInterfacePageLiveModuleUpdate(void);
  void subTaskLoadErrorToInterface(void);
  void subTaskSendingSettingsParameter(void);
  void subTaskloggingModuleProtectionCallback(void);
  void subTaskDebuggingInInterface(void);
  void subTaskErrorLoggingWatch(void);
  void subTaskInterfaceButtonStateWatch(void);

  void subTaskTemperatureLoggingToSdCard(void);
  void subTaskSystemInformationLoggingToSdCard(void);

  void subTaskInterfaceLogging(void);

private:
  rtos::Thread m_loggingThreadVar;
  rtos::Mutex m_mutex;
  mbed::Timer m_loggingManagerTim;
  
  Mail<st_mail_cell_voltages_t, LOGGING_CELL_VOLTAGE_MAIL_SIZE> &m_mailCellVoltageBox;

  HwLoggingBase &m_hwLogging;
  st_errorEepromData errorEepromData;

  uint8_t m_u8ErrorIndex;
  uint8_t m_u8ErrorIndexCount;
  uint8_t m_u8InterfaceButtonLastState;
  bool m_bErrorLoadingEnableFlag;
  bool m_bErrorResetEnableFlag;

  st_generalConfig *m_ptrGenConfig;
  st_powerElecPackInfoConfig *m_ptrPowerElecPackInfoConfig;

  uint8_t m_u8arrSendBuffer[64];

  uint32_t m_u32LoggingGeneralIntervalLastTick;
  uint32_t m_u32LoggingSdCardIntervalLastTick;
  uint32_t m_u32OledProcessTick;
};

#endif // __LOGGING__H_
