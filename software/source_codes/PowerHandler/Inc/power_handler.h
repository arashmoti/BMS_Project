#ifndef __HIGHPOWER_HANDLER__H_
#define __HIGHPOWER_HANDLER__H_

#include "power_handler_params.h"

#include "io_periph_defs.h"
#include "mbed.h"
#include "Settings.h"

#include "PinNames.h"
#include "DigitalOut.h"
#include "DigitalIn.h"
#include "Timer.h"
#include "Thread.h"
#include <cstdint>

#include "IOPower.h"
#include "IMPower.h"
#include "ITPower.h"


class PowerHandler : public IOPower, public IMPower, public ITPower
{

  using UnsignedShortHandle = dShort;
  using SignedShortHandle = dIntShort;

public:
    PowerHandler();
    virtual ~PowerHandler();

  virtual void task(void) override;
    

  virtual void udpateSwitches(void) override;

  virtual uint16_t getSysVoltage(void) const override;
  virtual uint32_t getLoadVoltage(void) const override;
  virtual bool getForceOnState(void) const override;
  virtual void switchesSetSwitchState(enSwitchesID switchID, enSwitchesState newState) override;

  virtual void allowedForceOn(bool newState) override;
  virtual void switchesDisableAll(void) override;
  virtual void setPreCharge(bool newState) override;
  virtual bool setDisCharge(bool newState) override;
  virtual void setCharge(bool newState) override;
  
  void init(st_generalConfig* genConfig, st_powerElecPackInfoConfig* powerElecPackInfoConfig);

private:

  void resetDwellState(void);

private:

  mbed::AnalogIn m_packVoltagePin;
  mbed::AnalogIn m_loadVoltagePin;

  mbed::DigitalOut m_chargeEnPin;
  mbed::DigitalOut m_dischargeEnPin;
  mbed::DigitalOut m_preDischargeEnPin;

  st_generalConfig* m_ptrGenConfig;
  st_powerElecPackInfoConfig* m_ptrPowerElecPackInfoConfig;

  float m_fSysVoltage;
  float m_fFilteredSysVoltage;

  float m_fLoadVoltage;
  float m_fFilteredLoadVoltage;

  bool m_bAllowedForceOnFlag;

  uint32_t m_preStamp;
  uint32_t m_dchgStamp;
  uint32_t m_chgStamp;
  bool m_preLastState;
  bool m_dischargeLastState;
  bool m_chargeLastState;

};

#endif // __HIGHPOWER_HANDLER__H_
