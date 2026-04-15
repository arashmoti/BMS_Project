#ifndef __POWER_BUTTON_HANDLER__H_
#define __POWER_BUTTON_HANDLER__H_

#include "effect_handler_params.h"
#include "io_periph_defs.h"
#include "Settings.h"

#include "mbed.h"
#include "PinNames.h"
#include "DigitalOut.h"
#include "InterruptIn.h"
#include <cstdint>

#include "IOPowerState.h"
#include "ITPowerState.h"

class PowerStateHandler : public IOPowerState, public ITPowerState
{

public:
    PowerStateHandler();
    virtual ~PowerStateHandler();

  virtual void setPwrBtnLedPin(int value) override;
  virtual void setPwrBtnEnPin(int value) override;
  virtual bool buttonForceOnRequest(void) override;
  virtual bool getButtonPressedState(void) const override;
  virtual bool getButtonPressedOnTurnon(void) const override;
  virtual bool getPowerdownRequest(void) const override;
  virtual void task(void) override;
  virtual bool chargeDetected(void) override;

  void init();

private:
  uint8_t pwrButtonDelay1ms(uint32_t *last, uint32_t ticks);

private:
  mbed::Timer m_powerButtonProcessTimer;

  mbed::DigitalOut m_powerEnablePin;
  mbed::DigitalOut m_powerButtonLedPin;
  mbed::DigitalIn m_powerButtonDetectionPin;
  mbed::DigitalIn m_chgDetectPin;

  PowerStateArgs::st_powerBtnVarsConfig m_stPowerBtnVarsConfig;



};

#endif // __POWER_BUTTON_HANDLER__H_
