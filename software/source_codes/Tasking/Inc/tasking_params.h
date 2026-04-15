#ifndef __TASKING_PARAMS__H_
#define __TASKING_PARAMS__H_

#include <cstdint>


#define TASKING_TIMER_PERIOD                10 /* ms*/
#define TASKING_GET_TICK(x)                 std::chrono::duration_cast<std::chrono::milliseconds>(x.elapsed_time()).count() 

namespace TaskingArgs {

#pragma pack(push) 
#pragma pack(1) 

#pragma pack(pop)

}

#endif // __TASKING_PARAMS__H_