#ifndef __IOEFFECT__H_
#define __IOEFFECT__H_

#include "stddef.h"
#include "Settings.h"


class IOEffect
{
public:
    IOEffect() {};
    virtual ~IOEffect() = default;

    virtual void effectChangeState(ModEffectArgs::EffectIdType _type, ModEffectArgs::EffectStateType _state) = 0;
    virtual void ledsDisableAll(void) = 0;
};


#endif // __IOEFFECT__H_