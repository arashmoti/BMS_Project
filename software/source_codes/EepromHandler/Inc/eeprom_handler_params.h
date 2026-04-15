#ifndef __EEPROM_HANDLER_PARAMS__H_
#define __EEPROM_HANDLER_PARAMS__H_

#include <cstdint>
#include <stdint.h>

namespace EepromM24C24Args
{

#pragma pack(push)
#pragma pack(1)

  typedef union _stEeprom32bit
  {
    uint32_t u32EepromBytes;
    uint8_t arru8Eeprom[4];
    struct
    {
      uint16_t u16EepromBytesL;
      uint16_t u16EepromBytesH;
    };
    struct
    {
      uint8_t u8EepromBytesLL;
      uint8_t u8EepromBytesLH;
      uint8_t u8EepromBytesHL;
      uint8_t u8EepromBytesHH;
    };
  } stEeprom32bit;

  // ---- SOC journal record (power-loss safe) ----
  typedef struct __attribute__((packed)) SocJournalRecord
  {
    uint32_t magic;        // 'SOCR' = 0x524F4353
    uint16_t version;      // 1
    uint16_t soc_mpermil;  // 0..1000
    int32_t remaining_mAh; // milliamp-hours
    uint16_t vmin_mV;      // cell minimum mV (for reference)
    int32_t current_mA;    // pack current
    int32_t temp_mC;       // average temp
    uint32_t ocv_mV;       // estimated OCV reference (optional)
    uint32_t timestamp_ms; // system tick ms from caller
    uint32_t seq;          // monotonically increasing sequence
    uint32_t crc32;        // over bytes [magic .. seq]
  } SocJournalRecord;
  // ----------------------------------------------

  // ---- SOH & cycles journal record ----
  typedef struct __attribute__((packed)) SohCycleRecord
  {
    uint32_t magic;          // 'SOHX' = 0x58484F53
    uint16_t version;        // 1
    uint16_t soh_mpermil;    // 0..1000
    uint32_t cycle_count;    // equivalent full cycles (integer)
    int32_t cap_est_mAh;     // estimated usable capacity (mAh)
    uint64_t discharged_mAs; // total discharged milliamp*seconds (lifetime)
    uint32_t timestamp_ms;   // system tick
    uint32_t seq;            // sequence
    uint32_t crc32;          // CRC32 over [magic..seq]
  } SohCycleRecord;
// -------------------------------------
#pragma pack(pop)
}

#endif // __EEPROM_HANDLER_PARAMS__H_
