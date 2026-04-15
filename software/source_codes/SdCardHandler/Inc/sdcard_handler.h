#ifndef __SDCARD_HANDLER__H_
#define __SDCARD_HANDLER__H_

#include "sdcard_handler_params.h"
#include "io_periph_defs.h"

#include "ISerial.h"
#include "IESdCard.h"
#include "ILSdCard.h"

#include "PinNames.h"
#include "DigitalIn.h"
#include "FATFileSystem.h"
#include "SDBlockDevice.h"
#include <cstdint>
#include "rtos/Mutex.h"


class SdCardHandler : public IESdcard, public ILSdcard 
{
  using SdCardStatus = SdCardArgs::e_statusType;

public:
  SdCardHandler(ISerial& _serial);
  virtual ~SdCardHandler();


  virtual uint8_t sdCardErrorCallback(st_errorSdCardData* errorSDcardData) override;

  virtual uint8_t cellVoltageLogging(uint32_t time, uint8_t module_num, uint16_t* cellInfo) override;
  virtual uint8_t packTemperatureLogging(uint32_t time, uint8_t module_num, int32_t* tempInfo) override;
  virtual uint8_t systemInformationLogging(uint32_t time, st_powerElecPackInfoConfig* powerElecPackInfoConfig) override;

  virtual uint8_t getSdCardStatus(void) override;
  virtual void resetSdCardStatus(void) override;

  // Lightweight event logging (CSV: timeLogging,event)
  virtual void logEvent(const char *fmt, ...) override;

  // Expose active filename for internal helpers (rotation) and diagnostics
  const char *getActiveFileName();

  // Safely unmount and de-initialize the SD card so it can be removed
  // or power can be cut with minimal corruption risk.
  // This does not persist any application state beyond ensuring the
  // filesystem metadata is written and the block device is closed.
  void safeEject();

  

private:

  bool beginAccess();
  void endAccess();

  void init(void);
  void deinit();
  bool sdCardPluggedDetected(void);
  void readActiveFile(); 
  void writeErrorLogToActiveFile(st_errorSdCardData* errorSDcardData);
  void setActiveFile(const char *path);
  void createFilenameToSdcard();
  void isFilenameExist(void);
  void addErrorHeaderName(void); 
  void addPackTemperatureHeaderName(void);
  void addCellVoltageHeaderName(void);
  void addSystemInformationHeaderName(void);
  void addEventHeaderName(void);
  void writeEventLine(uint32_t time, const char *text);
  void writeCellVoltageLogToActiveFile(uint32_t time, uint8_t module_num, uint16_t* cellInfo);
  void writePackTemperatureLogToActiveFile(uint32_t time, uint8_t module_num, int32_t* tempInfo);
  void writeSystemInformationLogToActiveFile(uint32_t time, st_powerElecPackInfoConfig* powerElecPackInfoConfig);
  

private:
  mbed::DigitalIn m_sdCardDetPin;
  SDBlockDevice m_sd;
  FATFileSystem m_fs;
  char* m_cPtractiveFileName;
  SdCardStatus statusType;
  bool m_allowWrites;
  rtos::Mutex m_sdMutex;

  ISerial& m_serial;

};

#endif // __SDCARD_HANDLER__H_
