#ifndef __INTERFACE_COMM_HANDLER__H_
#define __INTERFACE_COMM_HANDLER__H_

#include "interface_comm_handler_params.h"
#include "io_periph_defs.h"
#include "Settings.h"

#include "PinNames.h"
#include "BufferedSerial.h"
#include "ThisThread.h"
#include "Callback.h"
#include "Mutex.h"

#include "ILInterfaceComm.h"

#include "IEeprom.h"
#include "IDatetime.h"
#include "ISerial.h"

#include <cstdarg>
#include <cstdint>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdlib>
#include "CircularBuffer.h"

#define IFCOMM_HUMAN_CONSOLE 1

class InterfaceCommHandler : public ILInterfaceComm
{

public:
  static InterfaceCommHandler *getInstance();

  InterfaceCommHandler(IEeprom &_eeprom, IDatetime &_datetime, ISerial &_serial);
  virtual ~InterfaceCommHandler();

  virtual bool interfaceCommPrint(const char *command, ...) override;
  virtual void printToInterface(const char *format, ...) override;
  virtual void send1Byte(uint8_t msg_type, uint8_t data) override;
  virtual void send2Bytes(uint8_t msg_type, uint8_t dataH, uint8_t dataL) override;
  virtual void processPacket(unsigned char *data, unsigned char len) override;

  // mbed::BufferedSerial m_interfaceCommBufferedSerial;

  void startThread(void);
  void init(st_generalConfig *genConfig, st_powerElecPackInfoConfig *powerElecPackInfoConfig);

private:
  bool vsend(const char *format, va_list args);
  void puts(const char *data, size_t size);
  void lock(void);
  void unlock(void);

  void txThread(void);                             // <— NEW: drains TX queue
  void txEnqueue(const uint8_t *data, size_t len); // <— NEW: enqueue bytes (non-blocking)

  void packageReset(void);
  bool isCheckedPackage(void);
  void txByte(uint8_t data);
  void txPackage(void);
  void txAppend(uint8_t data);
  void interfaceCommRxISR(void);
  void interfaceCommThread(void);

  void handleLoggingNotification(uint8_t *pDestination, uint8_t value, const char *description);
  void interfaceProcessMessage(SerialArgs::stInterfaceCommParams *interfaceCommParams);
  void handleAsciiCommand(const char *line);

private:
  rtos::Mutex m_mutex;
  mbed::BufferedSerial m_interfaceCommBufferedSerial;
  rtos::Thread m_serialThreadVar;
  rtos::Thread m_txThreadVar;           // <— NEW: TX thread
  rtos::Semaphore m_txSem{0};           // <— NEW: wake TX thread
  rtos::Mutex m_txMutex;               // <— NEW: protects m_txQueue
  CircularBuffer<char, 4096> m_txQueue; // <— NEW: ~7–9ms at 460800 bps; tune freely
  SerialArgs::stInterfaceCommParams m_interfaceCommParams;

  st_generalConfig *m_ptrGenConfig;
  st_powerElecPackInfoConfig *m_ptrPowerElecPackInfoConfig;

  IEeprom &m_eeprom;
  IDatetime &m_datetime;
  ISerial &m_serial;
  static InterfaceCommHandler *instance;
};

#endif // __INTERFACE_COMM_HANDLER__H_
