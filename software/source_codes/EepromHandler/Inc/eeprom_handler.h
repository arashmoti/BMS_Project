#ifndef __EEPROM_HANDLER__H_
#define __EEPROM_HANDLER__H_

#include <string>
#include <cstddef>

#include "eeprom_handler_params.h"
#include "eeprom.h"
#include "mbed.h"
#include "io_periph_defs.h"
#include "Settings.h"

#include "IEEeprom.h"
#include "IEeprom.h"

#include "DigitalOut.h"
#include "PinNames.h"

class EepromHandler : public IEEeprom, public IEeprom
{
public:
  EepromHandler();
  virtual ~EepromHandler();

  virtual uint16_t getSettingValue(enEepromValuesIndex ee_index) override;
  virtual void setSettingValue(enEepromValuesIndex eeprom_index, uint16_t value) override;
  virtual void setErrorValue(uint16_t eeprom_index, uint8_t value) override;
  virtual uint8_t getErrorValue(uint16_t eepromIndex) override;

public:
  void defaultLoading() override;
  // SOC persistence (journaled)
  bool loadLatestSocJournal(EepromM24C24Args::SocJournalRecord &out) override;
  bool loadLatestSohCycle(EepromM24C24Args::SohCycleRecord &out);
  bool appendSohCycle(const EepromM24C24Args::SohCycleRecord &in);
  bool appendSocJournal(const EepromM24C24Args::SocJournalRecord &in) override;

  void invalidate_eeprom_mappings() override; // invalidate mapping to force defaults
  void clearSocJournal() override;

private:
  // Helpers for SOC journal
  uint32_t crc32_calc(const uint8_t *data, size_t len);
  bool readRaw(uint32_t addr, void *buf, uint32_t len);
  bool writeRaw(uint32_t addr, const void *buf, uint32_t len);

  mbed::DigitalOut m_wcPin;
  EEPROM m_ep;

  EepromM24C24Args::stEeprom32bit m_ee32bitVal;

  //-------------------------------------
};
#endif // __EEPROM_HANDLER__H_
