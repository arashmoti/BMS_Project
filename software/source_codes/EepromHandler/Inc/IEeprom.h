#ifndef __IEEPROM__H_
#define __IEEPROM__H_

#include "stdint.h"
#include "Settings.h"
#include "eeprom_handler_params.h"


class IEeprom
{
public:
  IEeprom() {};
  virtual ~IEeprom() = default;

  virtual void setSettingValue(enEepromValuesIndex eeprom_index, uint16_t value) = 0;
  virtual uint16_t getSettingValue(enEepromValuesIndex ee_index) = 0;
  virtual bool loadLatestSocJournal(EepromM24C24Args::SocJournalRecord &out) = 0;
  virtual bool appendSocJournal(const EepromM24C24Args::SocJournalRecord &in) = 0;
  virtual void defaultLoading() = 0;
  virtual void invalidate_eeprom_mappings() = 0;
  virtual void clearSocJournal() = 0;

};


#endif // __IEEPROM__H_
