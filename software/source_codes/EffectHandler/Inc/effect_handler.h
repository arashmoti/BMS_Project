#ifndef __EFFECT_HANDLER__H_
#define __EFFECT_HANDLER__H_

#include "effect_handler_params.h"
#include "io_periph_defs.h"
#include "Settings.h"

#include "PinNames.h"
#include "Timer.h"
#include "DigitalOut.h"
#if ENABLE_RGB_PWM_EFFECTS
#include "PwmOut.h"
#endif
#include <cstdint>

#include "IOEffect.h"
#include "ITEffect.h"


class EffectHandler : public IOEffect, public ITEffect
{

public:
    EffectHandler();
    virtual ~EffectHandler();

    virtual void effectChangeState(ModEffectArgs::EffectIdType _type, ModEffectArgs::EffectStateType _state) override;
    virtual void ledsDisableAll(void) override;

    virtual void task(void) override;
    virtual void temporary_effect_leds(void) override;

    

private:
  uint8_t effectDelay1ms(uint32_t *last, uint32_t ticks);
  void setGpioPin(ModEffectArgs::EffectIdType _type, int value);

  ModEffectArgs::EffectStateType effectTaskFlash(uint32_t blinkTime);
  ModEffectArgs::EffectStateType effectTaskFlashFast(void);
  ModEffectArgs::EffectStateType modEffectTaskBlinkShort(uint32_t LEDPointer, uint32_t blinkTime);
  ModEffectArgs::EffectStateType modEffectTaskBlinkLong(uint32_t LEDPointer, uint32_t blinkTime);
  ModEffectArgs::EffectStateType modEffectTaskBlinkShortLong(uint32_t blinkTimeShort, uint32_t blinkRatio);

  void setEffectOutput(ModEffectArgs::EffectIdType LEDId, uint8_t newState);

private:
  mbed::Timer m_effectProcessTimer;

  // PWM support for smooth LED effects
#if ENABLE_RGB_PWM_EFFECTS
  mbed::PwmOut m_pwmRedPin{RGB_RED_PIN};
  mbed::PwmOut m_pwmGreenPin{RGB_GREEN_PIN};
  mbed::PwmOut m_pwmBluePin{RGB_BLUE_PIN};

  float m_redBrightness{0.0f};
  float m_greenBrightness{0.0f};
  float m_blueBrightness{0.0f};
  uint32_t m_lastFadeTick{0};

  void setLedDuty(ModEffectArgs::EffectIdType type, float duty);
  void taskFade(ModEffectArgs::EffectIdType type, bool fadeIn, uint32_t totalMs, uint8_t steps);
  void taskBreath(ModEffectArgs::EffectIdType type, uint32_t cycleMs);
#endif

#if ENABLE_RGB_PWM_EFFECTS
  // Per-channel effect engine state
  ModEffectArgs::EffectStateType m_channelState[NoOfSTATs] = {
      ModEffectArgs::EffectStateType::STATE_OFF,
      ModEffectArgs::EffectStateType::STATE_OFF,
      ModEffectArgs::EffectStateType::STATE_OFF,
      ModEffectArgs::EffectStateType::STATE_OFF};

  // Per-channel blink phase and timers
  uint32_t m_lastBlinkTick[NoOfSTATs] = {0, 0, 0, 0};
  uint8_t m_blinkPhase[NoOfSTATs] = {0, 0, 0, 0}; // 0=off, 1=on

  uint32_t m_lastFadeTickCh[NoOfSTATs] = {0, 0, 0, 0};
  uint8_t m_breathUp[NoOfSTATs] = {1, 1, 1, 1};

  void applyChannel(ModEffectArgs::EffectIdType id);
  ModEffectArgs::EffectIdType resolveActiveColor();
#endif



  ModEffectArgs::EffectIdType m_type;
  ModEffectArgs::EffectStateType m_state;

};

#endif // __EFFECT_HANDLER__H_
