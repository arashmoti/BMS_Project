#include "power_state_handler.h"
#include <cstdint>

PowerStateHandler::PowerStateHandler()
    : m_powerEnablePin(POWER_EN_PIN),
      m_powerButtonLedPin(POWER_BTN_LED_PIN),
      m_powerButtonDetectionPin(POWER_BTN_DET_PIN),
      m_chgDetectPin(CHG_DETECT_PIN)
{
  m_powerButtonProcessTimer.start();
}

// #############################################################################################//

PowerStateHandler::~PowerStateHandler() {}

// #############################################################################################//

void PowerStateHandler::init()
{

  setPwrBtnEnPin(PIN_STATE_ENABLE);
  // Configure pull-up/pull-down based on active-low setting
  m_powerButtonDetectionPin.mode(POWER_BTN_ACTIVE_LOW ? PullUp : PullDown);
  m_stPowerBtnVarsConfig.u32PowerDownTimeout = PWR_STATE_GET_TICK(m_powerButtonProcessTimer);
  // Handle active-low button: invert the read value if active-low
  const bool pressed = POWER_BTN_ACTIVE_LOW ? !m_powerButtonDetectionPin.read() : m_powerButtonDetectionPin.read();
  m_stPowerBtnVarsConfig.bLastButtonFirstPress = m_stPowerBtnVarsConfig.bLastButtonPressedVar = pressed;
  // Latch the boot button state - this stays true until consumed, unlike bLastButtonFirstPress
  // which gets cleared on button release (needed for power-off detection)
  m_stPowerBtnVarsConfig.bBootButtonPressed = pressed;
}

// #############################################################################################//

uint8_t PowerStateHandler::pwrButtonDelay1ms(uint32_t *last, uint32_t ticks)
{
  if ((uint32_t)(PWR_STATE_GET_TICK(m_powerButtonProcessTimer) - *last) >= ticks)
  {
    *last = PWR_STATE_GET_TICK(m_powerButtonProcessTimer);
    return true;
  }

  return false;
}

// #############################################################################################//

void PowerStateHandler::task()
{
  // Handle active-low button: invert the read value if active-low
  const bool raw = m_powerButtonDetectionPin.read();
  const bool tempButtonPressed = POWER_BTN_ACTIVE_LOW ? !raw : raw;

  if (m_stPowerBtnVarsConfig.bLastButtonPressedVar != tempButtonPressed)
  {
    if (m_stPowerBtnVarsConfig.bLastButtonPressedVar)
    { // If was high and now low (button released)
      m_stPowerBtnVarsConfig.bLastButtonFirstPress = false;
    }
    else
    { // If was low and now high (button pressed)
      m_stPowerBtnVarsConfig.u32ButtonPressedTimeStamp = PWR_STATE_GET_TICK(m_powerButtonProcessTimer);
    }
    m_stPowerBtnVarsConfig.bLastButtonPressedVar = tempButtonPressed;
  }

  if (tempButtonPressed)
  {

    m_stPowerBtnVarsConfig.u32ButtonPressedDuration =
        PWR_STATE_GET_TICK(m_powerButtonProcessTimer) - m_stPowerBtnVarsConfig.u32ButtonPressedTimeStamp;

    if ((m_stPowerBtnVarsConfig.u32ButtonPressedDuration >= 3000) &&
        (m_stPowerBtnVarsConfig.bLastButtonFirstPress == false))
    {
      m_stPowerBtnVarsConfig.bPulsePowerDownDesired = true;
      m_stPowerBtnVarsConfig.u32ButtonPressedDuration = 0;
    }

    if ((m_stPowerBtnVarsConfig.u32ButtonPressedDuration >= 5000) &&
        (m_stPowerBtnVarsConfig.bLastButtonFirstPress == true))
    {
      m_stPowerBtnVarsConfig.bForceOnDesired = true;
      m_stPowerBtnVarsConfig.u32ButtonPressedDuration = 0;
    }

    if ((m_stPowerBtnVarsConfig.u32ButtonPressedDuration > 1000) &&
        (m_stPowerBtnVarsConfig.bLastButtonFirstPress == false))
      m_stPowerBtnVarsConfig.bPressedVar = true;

    m_stPowerBtnVarsConfig.u32PowerDownTimeout = PWR_STATE_GET_TICK(m_powerButtonProcessTimer);
  }

  else
  {
    if (pwrButtonDelay1ms(&m_stPowerBtnVarsConfig.u32PowerDownTimeout, 500))
      m_stPowerBtnVarsConfig.bPressedVar = false;
  }
}

// #############################################################################################//

bool PowerStateHandler::getButtonPressedState(void) const { return m_stPowerBtnVarsConfig.bPressedVar; }

// #############################################################################################//

bool PowerStateHandler::getPowerdownRequest(void) const
{
  return m_stPowerBtnVarsConfig.bPulsePowerDownDesired;
}

// #############################################################################################//

bool PowerStateHandler::getButtonPressedOnTurnon(void) const
{
  // Return the latched boot button state, not bLastButtonFirstPress
  // (which is cleared when button is released, breaking INIT detection)
  return m_stPowerBtnVarsConfig.bBootButtonPressed;
}

// #############################################################################################//

bool PowerStateHandler::buttonForceOnRequest(void)
{
  static bool firstTrigger = true;

  if (m_stPowerBtnVarsConfig.bForceOnDesired && firstTrigger)
  {
    m_stPowerBtnVarsConfig.bForceOnDesired = false;
    firstTrigger = false;
    return true;
  }

  else
  {
    return false;
  }
}

// #############################################################################################//

void PowerStateHandler::setPwrBtnLedPin(int value)
{
  m_powerButtonLedPin.write(value);
}

// #############################################################################################//

void PowerStateHandler::setPwrBtnEnPin(int value)
{
  m_powerEnablePin.write(value);
}

// #############################################################################################//

bool PowerStateHandler::chargeDetected(void)
{
  static bool _bChgDetected = false;
  _bChgDetected = m_chgDetectPin.read();
  return _bChgDetected;
  // return false;
}

// #############################################################################################//
