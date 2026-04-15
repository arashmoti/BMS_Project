#ifndef __OPERATION_PARAMS__H_
#define __OPERATION_PARAMS__H_

#include <cstdint>

#define OPERATION_GET_TICK(x)                 std::chrono::duration_cast<std::chrono::milliseconds>(x.elapsed_time()).count()


#define SHORT_CURRENT_VAL                     32.0f

#pragma pack(push) 
#pragma pack(1) 

#pragma pack(pop)

enum enOpStateType : uint8_t {
	OP_STATE_INIT = 0,											// 0
	OP_STATE_CHARGING,											// 1
	OP_STATE_PRE_CHARGE,										// 2
	OP_STATE_LOAD_ENABLED,									// 3
	OP_STATE_BATTERY_DEAD,									// 4
	OP_STATE_POWER_DOWN,										// 5
	OP_STATE_EXTERNAL,											// 6
	OP_STATE_ERROR,													// 7
	OP_STATE_ERROR_PRECHARGE,								// 8
	OP_STATE_BALANCING,											// 9
	OP_STATE_CHARGED,												// 10
	OP_STATE_FORCEON,												// 11
};


#endif // __OPERATION_PARAMS__H_