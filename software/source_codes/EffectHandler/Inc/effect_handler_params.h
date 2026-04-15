#ifndef __EFFECT_HANDLER_PARAMS__H_
#define __EFFECT_HANDLER_PARAMS__H_

#include <cstdint>
#include "Settings.h"

#define EFFECT_GET_TICK(x)                 std::chrono::duration_cast<std::chrono::milliseconds>(x.elapsed_time()).count() 

#define NoOfSTATs				            4
#define MAX_BLINKSHORT_CYCLES	      20
#define MAX_BLINKLONG_CYCLES	      5

typedef struct _effect_status{
	ModEffectArgs::EffectStateType 	state;
	uint32_t                        u32Count;
} st_effect_status;

typedef struct _effect_ports{
	mbed::DigitalOut 	Port;
	PinName 		u32Pin;
} st_effect_ports;





#endif // __EFFECT_HANDLER_PARAMS__H_
