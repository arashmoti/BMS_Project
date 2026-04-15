#ifndef __DATETIME_HANDLER_PARAMS__H_
#define __DATETIME_HANDLER_PARAMS__H_

#include <cstdint>


// RTC Configuration
#define PCF85063A_TIMEOUT                   100U
#define PCF85063A_SLAVE_WRITE_ADDRESS       0xA2U
#define PCF85063A_SLAVE_READ_ADDRESS        0xA3U

/** Real Time Clock defines BEGIN**/
#define CONTROL_REGISTER1_ADDRESS           0x00U
#define CONTROL_REGISTER2_ADDRESS           0x01U

#define CONTROL_REGISTER1_DEFAULT           0x00U
#define CONTROL_REGISTER2_DEFAULT           0x07U

#define SECONDS_ADDRESS                     0x04U
#define MINUTES_ADDRESS                     0x05U
#define HOURS_ADDRESS                       0x06U
#define DAYS_ADDRESS                        0x07U
#define WEEKDAYS_ADDRESS                    0x08U
#define MONTHS_ADDRESS                      0x09U
#define YEARS_ADDRESS                       0x0AU

namespace RtcArgs {
#pragma pack(push) // push current alignment rules to internal stack
#pragma pack(1)    // force struct to have 1 bytes of packing, unless size of
                   // downlink becomes 4 bytes (2x2 bytes).

#pragma pack(pop)

typedef struct _st_dateInfo_t {
  uint8_t u8Second;
  uint8_t u8Minute;
  uint8_t u8Hour;
  uint8_t u8Day;
  uint8_t u8Month;
  uint8_t u8Year;
} st_dateInfo_t;

} 

#endif // __DATETIME_HANDLER_PARAMS__H_