#include "effect_handler.h"
#include <cstdint>

mbed::DigitalOut m_buzzerPin(BUZZER_PIN);
mbed::DigitalOut m_rgbRedPin(RGB_RED_PIN);
mbed::DigitalOut m_rgbGreenPin(RGB_GREEN_PIN);
mbed::DigitalOut m_rgbBluePin(RGB_BLUE_PIN);

static st_effect_status effect_status[NoOfSTATs] = // Hold all STAT StateIndicator data
    {
        {ModEffectArgs::STATE_OFF, 1}, // STAT_RGB_RED_LED
        {ModEffectArgs::STATE_OFF, 1}, // STAT_RGB_GREEN_LED
        {ModEffectArgs::STATE_OFF, 1}, // STAT_RGB_BLUE_LED
        {ModEffectArgs::STATE_OFF, 0}  // STAT_BUZZER
};

static st_effect_ports status_ports[NoOfSTATs] = // Hold all status configuration data
    {
        {m_rgbRedPin, RGB_RED_PIN},
        {m_rgbGreenPin, RGB_GREEN_PIN},
        {m_rgbBluePin, RGB_BLUE_PIN},
        {m_buzzerPin, BUZZER_PIN}};

EffectHandler::EffectHandler()
{
  m_buzzerPin.write(PIN_STATE_DISABLE);
  // Set DigitalOuts to OFF respecting active-low RGB wiring
  m_rgbRedPin.write(RGB_LED_ACTIVE_LOW ? 1 : PIN_STATE_DISABLE);
  m_rgbGreenPin.write(RGB_LED_ACTIVE_LOW ? 1 : PIN_STATE_DISABLE);
  m_rgbBluePin.write(RGB_LED_ACTIVE_LOW ? 1 : PIN_STATE_DISABLE);
  m_effectProcessTimer.start();
#if ENABLE_RGB_PWM_EFFECTS
  // Configure PWM frequency for LEDs (reduce visible flicker)
  m_pwmRedPin.period_ms(1);   // 1 kHz
  m_pwmGreenPin.period_ms(1); // 1 kHz
  m_pwmBluePin.period_ms(1);  // 1 kHz
  // Initialize PWM to OFF (respect active-low)
  const float effOff = (RGB_LED_ACTIVE_LOW ? 1.0f : 0.0f);
  m_pwmRedPin.write(effOff);
  m_pwmGreenPin.write(effOff);
  m_pwmBluePin.write(effOff);
  m_redBrightness = 0.0f;
  m_greenBrightness = 0.0f;
  m_blueBrightness = 0.0f;
  m_lastFadeTick = EFFECT_GET_TICK(m_effectProcessTimer);
#endif
}

// #############################################################################################//

EffectHandler::~EffectHandler() {}

// #############################################################################################//

void EffectHandler::setGpioPin(ModEffectArgs::EffectIdType _type, int value)
{
  switch (_type)
  {
  case ModEffectArgs::EffectIdType::RGB_GREEN:
  case ModEffectArgs::EffectIdType::RGB_RED:
  case ModEffectArgs::EffectIdType::RGB_BLUE:
  {
#if ENABLE_RGB_PWM_EFFECTS
    setLedDuty(_type, (value ? 1.0f : 0.0f));
#else
    const int on_level = (RGB_LED_ACTIVE_LOW ? 0 : 1);
    const int off_level = (RGB_LED_ACTIVE_LOW ? 1 : 0);
    if (_type == ModEffectArgs::EffectIdType::RGB_GREEN)
    {
      if (value)
      {
        m_rgbRedPin.write(off_level);
        m_rgbBluePin.write(off_level);
        m_rgbGreenPin.write(on_level);
      }
      else
      {
        m_rgbGreenPin.write(off_level);
      }
    }
    else if (_type == ModEffectArgs::EffectIdType::RGB_RED)
    {
      if (value)
      {
        m_rgbGreenPin.write(off_level);
        m_rgbBluePin.write(off_level);
        m_rgbRedPin.write(on_level);
      }
      else
      {
        m_rgbRedPin.write(off_level);
      }
    }
    else // RGB_BLUE
    {
      if (value)
      {
        m_rgbGreenPin.write(off_level);
        m_rgbRedPin.write(off_level);
        m_rgbBluePin.write(on_level);
      }
      else
      {
        m_rgbBluePin.write(off_level);
      }
    }
#endif
  }
  break;

  case ModEffectArgs::EffectIdType::BUZZER:
  {
    m_buzzerPin.write(value);
  }
  break;

  default:
    break;
  }
}

void EffectHandler::ledsDisableAll(void)
{
  effectChangeState(ModEffectArgs::EffectIdType::RGB_RED, ModEffectArgs::EffectStateType::STATE_RESET);
  effectChangeState(ModEffectArgs::EffectIdType::RGB_GREEN, ModEffectArgs::EffectStateType::STATE_RESET);
  effectChangeState(ModEffectArgs::EffectIdType::RGB_BLUE, ModEffectArgs::EffectStateType::STATE_RESET);
  // Force RGB pins low now so non-PWM mode doesn't leave a channel on.
  setGpioPin(ModEffectArgs::EffectIdType::RGB_RED, PIN_STATE_DISABLE);
  setGpioPin(ModEffectArgs::EffectIdType::RGB_GREEN, PIN_STATE_DISABLE);
  setGpioPin(ModEffectArgs::EffectIdType::RGB_BLUE, PIN_STATE_DISABLE);
}

// #############################################################################################//

void EffectHandler::effectChangeState(ModEffectArgs::EffectIdType _type, ModEffectArgs::EffectStateType _state)
{
  // Maintain legacy fields for API compatibility
  m_state = _state;
  m_type = _type;
#if ENABLE_RGB_PWM_EFFECTS
  // Per-channel desired state
  m_channelState[_type] = _state;
#endif
}

// #############################################################################################//

void EffectHandler::task(void)
{
#if ENABLE_RGB_PWM_EFFECTS
  // Resolve single active color to drive the shared RGB LED
  auto active = resolveActiveColor();

  // Drive only the chosen color channel, force others off to avoid mixing
  if (active == ModEffectArgs::EffectIdType::RGB_RED)
  {
    applyChannel(ModEffectArgs::EffectIdType::RGB_RED);
    setGpioPin(ModEffectArgs::EffectIdType::RGB_GREEN, PIN_STATE_DISABLE);
    setGpioPin(ModEffectArgs::EffectIdType::RGB_BLUE, PIN_STATE_DISABLE);
  }
  else if (active == ModEffectArgs::EffectIdType::RGB_GREEN)
  {
    applyChannel(ModEffectArgs::EffectIdType::RGB_GREEN);
    setGpioPin(ModEffectArgs::EffectIdType::RGB_RED, PIN_STATE_DISABLE);
    setGpioPin(ModEffectArgs::EffectIdType::RGB_BLUE, PIN_STATE_DISABLE);
  }
  else if (active == ModEffectArgs::EffectIdType::RGB_BLUE)
  {
    applyChannel(ModEffectArgs::EffectIdType::RGB_BLUE);
    setGpioPin(ModEffectArgs::EffectIdType::RGB_RED, PIN_STATE_DISABLE);
    setGpioPin(ModEffectArgs::EffectIdType::RGB_GREEN, PIN_STATE_DISABLE);
  }
  else
  {
    // No active color effect -> all off
    setGpioPin(ModEffectArgs::EffectIdType::RGB_RED, PIN_STATE_DISABLE);
    setGpioPin(ModEffectArgs::EffectIdType::RGB_GREEN, PIN_STATE_DISABLE);
    setGpioPin(ModEffectArgs::EffectIdType::RGB_BLUE, PIN_STATE_DISABLE);
  }

  // Buzzer is independent
  applyChannel(ModEffectArgs::EffectIdType::BUZZER);
#else
  switch (m_state)
  {
  case ModEffectArgs::EffectStateType::STATE_OFF:
    setGpioPin(m_type, PIN_STATE_DISABLE);
    break;
  case ModEffectArgs::EffectStateType::STATE_RESET:
    setGpioPin(m_type, PIN_STATE_DISABLE);
    break;

  case ModEffectArgs::EffectStateType::STATE_ON:
    setGpioPin(m_type, PIN_STATE_ENABLE);
    break;
  case ModEffectArgs::EffectStateType::STATE_SET:
    setGpioPin(m_type, PIN_STATE_ENABLE);
    break;

  case ModEffectArgs::EffectStateType::STATE_FLASH:
    setGpioPin(m_type, effectTaskFlash(100));
    break;

  case ModEffectArgs::EffectStateType::STATE_FLASH_FAST:
    setGpioPin(m_type, effectTaskFlash(50));
    break;

  default:
    break;
  }
#endif
}

// Only provide these helpers when PWM effects are enabled and declared in the header
#if ENABLE_RGB_PWM_EFFECTS
ModEffectArgs::EffectIdType EffectHandler::resolveActiveColor()
{
  auto is_active = [&](ModEffectArgs::EffectIdType c)
  {
    auto st = m_channelState[c];
    return (st != ModEffectArgs::EffectStateType::STATE_OFF &&
            st != ModEffectArgs::EffectStateType::STATE_RESET);
  };

  if (is_active(ModEffectArgs::EffectIdType::RGB_RED))
    return ModEffectArgs::EffectIdType::RGB_RED;
  if (is_active(ModEffectArgs::EffectIdType::RGB_BLUE))
    return ModEffectArgs::EffectIdType::RGB_BLUE;
  if (is_active(ModEffectArgs::EffectIdType::RGB_GREEN))
    return ModEffectArgs::EffectIdType::RGB_GREEN;

  return (ModEffectArgs::EffectIdType)255; // sentinel
}

void EffectHandler::applyChannel(ModEffectArgs::EffectIdType id)
{
  const auto state = m_channelState[id];
  switch (state)
  {
  case ModEffectArgs::EffectStateType::STATE_OFF:
  case ModEffectArgs::EffectStateType::STATE_RESET:
    if (id != ModEffectArgs::EffectIdType::BUZZER)
      setLedDuty(id, 0.0f);
    setGpioPin(id, PIN_STATE_DISABLE);
    break;

  case ModEffectArgs::EffectStateType::STATE_ON:
  case ModEffectArgs::EffectStateType::STATE_SET:
    if (id != ModEffectArgs::EffectIdType::BUZZER)
      setLedDuty(id, 1.0f);
    setGpioPin(id, PIN_STATE_ENABLE);
    break;

  case ModEffectArgs::EffectStateType::STATE_FLASH:
  {
    const uint32_t period = 100;
    if (effectDelay1ms(&m_lastBlinkTick[id], period))
      m_blinkPhase[id] ^= 1;
    if (id != ModEffectArgs::EffectIdType::BUZZER)
      setLedDuty(id, m_blinkPhase[id] ? 1.0f : 0.0f);
    else
      setGpioPin(id, m_blinkPhase[id] ? PIN_STATE_ENABLE : PIN_STATE_DISABLE);
  }
  break;

  case ModEffectArgs::EffectStateType::STATE_FLASH_FAST:
  {
    const uint32_t period = 50;
    if (effectDelay1ms(&m_lastBlinkTick[id], period))
      m_blinkPhase[id] ^= 1;
    if (id != ModEffectArgs::EffectIdType::BUZZER)
      setLedDuty(id, m_blinkPhase[id] ? 1.0f : 0.0f);
    else
      setGpioPin(id, m_blinkPhase[id] ? PIN_STATE_ENABLE : PIN_STATE_DISABLE);
  }
  break;

  case ModEffectArgs::EffectStateType::STATE_FADE_IN:
    if (id != ModEffectArgs::EffectIdType::BUZZER)
      taskFade(id, true, 1000, 100);
    break;

  case ModEffectArgs::EffectStateType::STATE_FADE_OUT:
    if (id != ModEffectArgs::EffectIdType::BUZZER)
      taskFade(id, false, 1000, 100);
    break;

  case ModEffectArgs::EffectStateType::STATE_BREATH:
    if (id != ModEffectArgs::EffectIdType::BUZZER)
      taskBreath(id, 2000);
    break;

  default:
    break;
  }
}
#endif // ENABLE_RGB_PWM_EFFECTS

// #############################################################################################//

uint8_t EffectHandler::effectDelay1ms(uint32_t *last, uint32_t ticks)
{
  if ((uint32_t)(EFFECT_GET_TICK(m_effectProcessTimer) - *last) >= ticks)
  {
    *last = EFFECT_GET_TICK(m_effectProcessTimer);
    return true;
  }

  return false;
}

// #############################################################################################//

ModEffectArgs::EffectStateType EffectHandler::effectTaskFlash(uint32_t blinkTime)
{
  static uint32_t lastTick;
  static ModEffectArgs::EffectStateType _state = ModEffectArgs::EffectStateType::STATE_RESET;

  if (effectDelay1ms(&lastTick, blinkTime))
  {
    if (_state)
    {
      _state = ModEffectArgs::EffectStateType::STATE_RESET;
    }
    else
    {
      _state = ModEffectArgs::EffectStateType::STATE_SET;
    }
  }

  return _state;
}

// #############################################################################################//

void EffectHandler::temporary_effect_leds(void)
{

  static uint32_t _u32EffectGeneralIntervalLastTick = 0;
  static uint8_t _u8Counter = 0;
  if ((EFFECT_GET_TICK(m_effectProcessTimer) - _u32EffectGeneralIntervalLastTick) >= 1000)
  {
    _u8Counter++;
    _u32EffectGeneralIntervalLastTick = EFFECT_GET_TICK(m_effectProcessTimer);
    switch (_u8Counter)
    {
    case 1:
    {
      effectChangeState(ModEffectArgs::EffectIdType::RGB_RED, ModEffectArgs::EffectStateType::STATE_FLASH_FAST);
    }
    break;

    case 2:
    {
      effectChangeState(ModEffectArgs::EffectIdType::RGB_GREEN, ModEffectArgs::EffectStateType::STATE_BREATH);
    }
    break;

    case 3:
    {
      effectChangeState(ModEffectArgs::EffectIdType::RGB_BLUE, ModEffectArgs::EffectStateType::STATE_FADE_IN);
    }
    break;

    case 4:
    {
      effectChangeState(ModEffectArgs::EffectIdType::RGB_BLUE, ModEffectArgs::EffectStateType::STATE_FADE_OUT);
      _u8Counter = 0;
    }
    break;

    default:

      break;
    }
  }
}

void EffectHandler::setEffectOutput(ModEffectArgs::EffectIdType LEDId, uint8_t newState)
{
  status_ports[LEDId].Port.write(newState);
}

#if ENABLE_RGB_PWM_EFFECTS
void EffectHandler::setLedDuty(ModEffectArgs::EffectIdType type, float duty)
{
  // Clamp duty
  if (duty < 0.0f)
    duty = 0.0f;
  if (duty > 1.0f)
    duty = 1.0f;

  // Invert for active-low hardware if configured
  const float eff = (RGB_LED_ACTIVE_LOW ? (1.0f - duty) : duty);

  switch (type)
  {
  case ModEffectArgs::EffectIdType::RGB_RED:
    m_redBrightness = duty;
    m_pwmRedPin.write(eff);
    break;
  case ModEffectArgs::EffectIdType::RGB_GREEN:
    m_greenBrightness = duty;
    m_pwmGreenPin.write(eff);
    break;
  case ModEffectArgs::EffectIdType::RGB_BLUE:
    m_blueBrightness = duty;
    m_pwmBluePin.write(eff);
    break;
  case ModEffectArgs::EffectIdType::BUZZER:
    // Keep buzzer purely digital; no PWM duty change here
    break;
  default:
    break;
  }
}

void EffectHandler::taskFade(ModEffectArgs::EffectIdType type, bool fadeIn, uint32_t totalMs, uint8_t steps)
{
  if (steps == 0)
    steps = 1;
  const uint32_t stepMs = totalMs / steps;
  if (stepMs == 0)
    return;

  if (!effectDelay1ms(&m_lastFadeTickCh[type], stepMs))
    return;

  float current = 0.0f;
  switch (type)
  {
  case ModEffectArgs::EffectIdType::RGB_RED:
    current = m_redBrightness;
    break;
  case ModEffectArgs::EffectIdType::RGB_GREEN:
    current = m_greenBrightness;
    break;
  case ModEffectArgs::EffectIdType::RGB_BLUE:
    current = m_blueBrightness;
    break;
  default:
    return;
  }

  const float delta = 1.0f / steps;
  if (fadeIn)
    current += delta;
  else
    current -= delta;

  setLedDuty(type, current);

  // When finished, snap to ON/OFF state
  if (current <= 0.0f && !fadeIn)
  {
    setLedDuty(type, 0.0f);
  }
  else if (current >= 1.0f && fadeIn)
  {
    setLedDuty(type, 1.0f);
  }
}

void EffectHandler::taskBreath(ModEffectArgs::EffectIdType type, uint32_t cycleMs)
{
  // One full breath cycle is fade-in then fade-out

  const uint8_t steps = 100; // smooth enough
  const uint32_t halfCycle = cycleMs / 2;
  const uint32_t stepMs = halfCycle / steps;

  if (stepMs == 0)
    return;

  if (!effectDelay1ms(&m_lastFadeTickCh[type], stepMs))
    return;

  float current = 0.0f;
  switch (type)
  {
  case ModEffectArgs::EffectIdType::RGB_RED:
    current = m_redBrightness;
    break;
  case ModEffectArgs::EffectIdType::RGB_GREEN:
    current = m_greenBrightness;
    break;
  case ModEffectArgs::EffectIdType::RGB_BLUE:
    current = m_blueBrightness;
    break;
  default:
    return;
  }

  const float delta = 1.0f / steps;
  current += (m_breathUp[type] ? delta : -delta);
  // Keep the breathing visibly above a minimum floor
  const float floor = 0.15f; // 15% minimum brightness
  if (current < 0.0f)
    current = 0.0f;
  if (current > 1.0f)
    current = 1.0f;
  const float scaled = floor + (1.0f - floor) * current;
  setLedDuty(type, scaled);

  if (current >= 1.0f)
  {
    m_breathUp[type] = 0;
  }
  else if (current <= 0.0f)
  {
    m_breathUp[type] = 1;
  }
}
#endif
