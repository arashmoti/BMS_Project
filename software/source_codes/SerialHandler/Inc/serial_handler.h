#ifndef __SERIAL_HANDLER__H_
#define __SERIAL_HANDLER__H_

#include "serial_handler_params.h"
#include "io_periph_defs.h"
#include "Settings.h"

#include "PinNames.h"
#include "BufferedSerial.h"
#include "ThisThread.h"
#include "Callback.h"
#include "Mutex.h"

#include "ISerial.h"

#include <cstdarg>
#include <cstdint>
#include <vector>

class SerialHandler : public ISerial
{

public:
  SerialHandler();
  virtual ~SerialHandler();
  virtual bool serialPrint(const char *command, ...) override;
  virtual void LOGI(const char *message) override;
  virtual void LOGW(const char *message) override;
  virtual void LOGE(const char *message) override;

  void init(st_generalConfig* genConfig, st_powerElecPackInfoConfig* powerElecPackInfoConfig);

private:
  bool vsend(const char *format, va_list args);
  void puts(const char *data, size_t size);
  void lock(void);
  void unlock(void);


private:
  rtos::Mutex m_mutex;
  mbed::BufferedSerial m_serial;

  st_generalConfig* m_ptrGenConfig;
  st_powerElecPackInfoConfig* m_ptrPowerElecPackInfoConfig;

};

#endif // __SERIAL_HANDLER__H_