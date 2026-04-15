#ifndef __LOGGING_PARAMS__H_
#define __LOGGING_PARAMS__H_

#include <cstdint>


#define LOGGING_TIMER_PERIOD                10 /* ms*/
#define LOGGING_GET_TICK(x)                 std::chrono::duration_cast<std::chrono::milliseconds>(x.elapsed_time()).count() 

namespace LoggingArgs {

#pragma pack(push) 
#pragma pack(1) 

#pragma pack(pop)

}

#endif // __LOGGING_PARAMS__H_