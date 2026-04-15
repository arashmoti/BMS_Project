#ifndef __ITEFFECT__H_
#define __ITEFFECT__H_

class ITEffect
{
public:
    ITEffect() {};
    virtual ~ITEffect() = default;

    virtual void task(void) = 0;
    virtual void temporary_effect_leds(void) = 0;
};


#endif // __ITEFFECT__H_