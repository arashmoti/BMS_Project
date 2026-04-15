
#include "eeprom_handler.h"
#include <cstdint>
#include <cstddef>


EepromHandler::EepromHandler()
    : m_wcPin(EEPROM_WC_PIN),
      m_ep(EEPROM_RTC_SDA_PIN, EEPROM_RTC_SCL_PIN, 0x00, EEPROM::T24C64)
{
  m_ee32bitVal.u32EepromBytes = 0;
  m_wcPin.write(PIN_STATE_ENABLE);
}

// #############################################################################################//

EepromHandler::~EepromHandler() {}

// #############################################################################################//

uint16_t EepromHandler::getSettingValue(enEepromValuesIndex eeprom_index)
{
  m_ep.resetError();
  m_ep.read(eeprom_index, (uint8_t *)m_ee32bitVal.arru8Eeprom, 2);
  return m_ee32bitVal.u16EepromBytesL;
}

// #############################################################################################//

uint8_t EepromHandler::getErrorValue(uint16_t eepromIndex) {
  m_ep.resetError();
  m_ep.read(eepromIndex, (uint8_t *)m_ee32bitVal.arru8Eeprom, 1);
  return m_ee32bitVal.u8EepromBytesLL;
}

// #############################################################################################//

void EepromHandler::setSettingValue(enEepromValuesIndex eeprom_index, uint16_t value)
{
  m_ep.resetError();
  m_ee32bitVal.u16EepromBytesL = value;
  m_wcPin.write(PIN_STATE_DISABLE);
  m_ep.write(eeprom_index, (uint8_t *)m_ee32bitVal.arru8Eeprom, 2);
  m_wcPin.write(PIN_STATE_ENABLE);
}

// #############################################################################################//

void EepromHandler::setErrorValue(uint16_t eeprom_index, uint8_t value)
{
  m_ep.resetError();
  m_ee32bitVal.u8EepromBytesLL = value;
  m_wcPin.write(PIN_STATE_DISABLE);
  m_ep.write(eeprom_index, (uint8_t *)m_ee32bitVal.arru8Eeprom, 1);
  m_wcPin.write(PIN_STATE_ENABLE);
}
// #############################################################################################//
void EepromHandler::invalidate_eeprom_mappings()
{
  // Write a value that is different from EEPROM_MAPPED_VALUE to invalidate the settings.
  // 0x0000 is a safe choice.
  setSettingValue(CMD_EE_ADDR_MappedEepromVal, 0x0000);
}
// #############################################################################################//

void EepromHandler::defaultLoading()
{
  setSettingValue(CMD_EE_ADDR_NumberOfCells                             , 16);//16
  setSettingValue(CMD_EE_ADDR_NoOfCellsPerModule                        , 4);
  setSettingValue(CMD_EE_ADDR_NumberOfTemperature                       , 4);
  setSettingValue(CMD_EE_ADDR_HysteresisDischarge                       , 20);
  setSettingValue(CMD_EE_ADDR_HysteresisCharge                          , 10);
  setSettingValue(CMD_EE_ADDR_HysteresisCurrent                         , 100);
  setSettingValue(CMD_EE_ADDR_HysteresisOverTemperature                 , 50); // 5.0C hysteresis (release at 55C when limit=60C)
  setSettingValue(CMD_EE_ADDR_TimeoutDischargeRetry                     , 4 * 1000);
  setSettingValue(CMD_EE_ADDR_TimeoutChargeRetry                        , 30 * 1000);
  setSettingValue(CMD_EE_ADDR_TimeoutOverTemperatureRetry               , 4 * 1000);
  setSettingValue(CMD_EE_ADDR_TimeoutLowCurrentPreChargeRetry           , 400);
  setSettingValue(CMD_EE_ADDR_TimeoutSoftOverCurrentRetry               , 2000);
  setSettingValue(CMD_EE_ADDR_TimeoutHardOverCurrentRetry               , 4);
  setSettingValue(CMD_EE_ADDR_DisplayTimeoutBatteryDead                 , 5 * 1000);
  setSettingValue(CMD_EE_ADDR_DisplayTimeoutBatteryErrorPreCharge       , 2 * 1000);
  setSettingValue(CMD_EE_ADDR_DisplayTimeoutBatteryError                , 2 * 1000);
  setSettingValue(CMD_EE_ADDR_TimeoutChargerDisconnected                , 2 * 1000);
  setSettingValue(CMD_EE_ADDR_TimeoutChargeCompleted                    , 1);
  setSettingValue(CMD_EE_ADDR_TimeoutNotUsed                            , 1);
  setSettingValue(CMD_EE_ADDR_ShuntResistance                           , 400);
  setSettingValue(CMD_EE_ADDR_BalanceStartVoltage                       , 3500);//3550//3500
  setSettingValue(CMD_EE_ADDR_MaxUnderAndOverVoltageErrorCount          , 5);
  setSettingValue(CMD_EE_ADDR_MaxHardOverCurrentErrorCount              , 2);
  setSettingValue(CMD_EE_ADDR_MaxSoftOverCurrentErrorCount              , 5);
  setSettingValue(CMD_EE_ADDR_HardOverTemperatureErrorCount             , 5);
  setSettingValue(CMD_EE_ADDR_ChargerEnabledThreshold                   , 2000);
  setSettingValue(CMD_EE_ADDR_MinimalPrechargePercentage                , 8);
  setSettingValue(CMD_EE_ADDR_NotUsedCurrentThreshold                   , 5000);
  setSettingValue(CMD_EE_ADDR_MaxMismatchThreshold                      , 15);
  setSettingValue(CMD_EE_ADDR_TimeoutChargingCompletedMinimalMismatch   , 6 * 1000);
  setSettingValue(CMD_EE_ADDR_MaxSoftDchgAllowedCurrent                 , 3500);//15000
  setSettingValue(CMD_EE_ADDR_MaxHardDchgAllowedCurrent                 , 5000);//20000
  setSettingValue(CMD_EE_ADDR_MaxSoftChgAllowedCurrent                  , 3500);//12000
  setSettingValue(CMD_EE_ADDR_MaxHardChgAllowedCurrent                  , 5000);//15000

  // Highlight: Updated Voltage Limits based on L194F54 Datasheet
  setSettingValue(CMD_EE_ADDR_SoftLowCurrentUnderVoltage                , 2900); // Was 2900
  setSettingValue(CMD_EE_ADDR_SoftHighCurrentUnderVoltage               , 2600); // Was 2700
  setSettingValue(CMD_EE_ADDR_HardUnderVoltage                          , 2500); // Was 2500
  setSettingValue(CMD_EE_ADDR_SoftOverVoltage                           , 3550); // Was 3500//3650//3550
  setSettingValue(CMD_EE_ADDR_HardOverVoltage                           , 3650); // Was 3650//3800

  setSettingValue(CMD_EE_ADDR_PcbNtcTopResistor                         , 10);
  setSettingValue(CMD_EE_ADDR_PcbNtc25DegResistance                     , 10);
  setSettingValue(CMD_EE_ADDR_PcbNtcBetaFactor                          , 3399);
  setSettingValue(CMD_EE_ADDR_CellNtcTopResistor                        , 10);
  setSettingValue(CMD_EE_ADDR_CellNtc25DegResistance                    , 10);
  setSettingValue(CMD_EE_ADDR_CellNtcBetaFactor                         , 3399);
  setSettingValue(CMD_EE_ADDR_AllowedTempPCBMax                         , 550);
  setSettingValue(CMD_EE_ADDR_AllowedTempPCBMin                         , 100);

  // Highlight: Updated Temperature Limits based on L194F54 Datasheet
  setSettingValue(CMD_EE_ADDR_AllowedTempBattDischargingMax             , 600);  // 60.0C over-temp trip
  setSettingValue(CMD_EE_ADDR_AllowedTempBattDischargingMin             , -20); // Was 100
  setSettingValue(CMD_EE_ADDR_AllowedTempBattChargingMax                , 600);
  setSettingValue(CMD_EE_ADDR_AllowedTempBattChargingMin                , -0); // Was 0

  // Highlight: Updated Capacity and Internal Resistance based on L194F54 Datasheet
  setSettingValue(CMD_EE_ADDR_BatteryCapacity                           , 54);   // Was 100
  setSettingValue(CMD_EE_ADDR_InternalResistance                        , 1300); // Was 2250, assuming micro-ohms

  setSettingValue(CMD_EE_ADDR_CurrentOffsetValue                        , 0);//0
  setSettingValue(CMD_EE_ADDR_BalanceDifferenceThreshold                , 2);
  setSettingValue(CMD_EE_ADDR_CellVoltagesOffsetVoltage                 , 5);
  setSettingValue(CMD_EE_ADDR_OverMismatchVoltage                       , 45);//45

  // Explicitly invalidate the stored SOC when loading default settings.
  setSettingValue(CMD_EE_ADDR_LastRemainingCapacity_L                   , 0xFFFF);
  setSettingValue(CMD_EE_ADDR_LastRemainingCapacity_H                   , 0xFFFF);
  setSettingValue(CMD_EE_ADDR_CommissioningDone                         , 0);

  setSettingValue(CMD_EE_ADDR_MappedEepromVal                           , EEPROM_MAPPED_VALUE);
  setSettingValue(CMD_EE_ADDR_ErrorIndexCount                           , 0);
}

// legacy defaultLoading variant removed (commented block)


// ===== SOC journal implementation =====
static const uint32_t SOC_MAGIC = 0x524F4353u; // 'SOCR'
static const uint16_t SOC_VERSION = 1;
static const uint32_t SOC_AREA_SIZE = 1024; // last 1KB of EEPROM
static const uint32_t SOC_SLOT_SIZE = 64;   // per record
static const uint32_t SOC_SLOTS = SOC_AREA_SIZE / SOC_SLOT_SIZE;

static inline uint32_t soc_base_addr_bytes(EEPROM &ep)
{
  // Use reported size (bytes) to place the SOC journal at the last 1KB of the device
  return ep.getSize() - SOC_AREA_SIZE;
}

void EepromHandler::clearSocJournal()
{
  const uint32_t base = soc_base_addr_bytes(m_ep);
  uint8_t zeros[SOC_SLOT_SIZE] = {};
  for (uint32_t i = 0; i < SOC_SLOTS; ++i)
  {
    const uint32_t addr = base + i * SOC_SLOT_SIZE;
    (void)writeRaw(addr, zeros, sizeof(zeros));
  }
}

uint32_t EepromHandler::crc32_calc(const uint8_t* data, size_t len) {
    // Standard CRC-32 (Ethernet)
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

bool EepromHandler::readRaw(uint32_t addr, void* buf, uint32_t len) {
    m_ep.resetError();
    m_ep.read(addr, buf, len);
    return (m_ep.getError() == 0);
}

bool EepromHandler::writeRaw(uint32_t addr, const void* buf, uint32_t len) {
    m_ep.resetError();
    m_wcPin.write(PIN_STATE_DISABLE); // Enable Write
    m_ep.write(addr, (int8_t*)buf, len);
    m_wcPin.write(PIN_STATE_ENABLE); // Disable Write (Protect)
    return (m_ep.getError() == 0);
}

bool EepromHandler::loadLatestSocJournal(EepromM24C24Args::SocJournalRecord &out) {
    using namespace EepromM24C24Args;
    const uint32_t base = soc_base_addr_bytes(m_ep);
    SocJournalRecord rec;
    bool found = false;
    uint32_t best_seq = 0;
    for (uint32_t i = 0; i < SOC_SLOTS; ++i) {
        uint32_t addr = base + i * SOC_SLOT_SIZE;
        if (!readRaw(addr, &rec, sizeof(SocJournalRecord))) continue;
        if (rec.magic != SOC_MAGIC || rec.version != SOC_VERSION) continue;
        // Validate CRC
        uint32_t calc = crc32_calc((const uint8_t*)&rec, offsetof(SocJournalRecord, crc32));
        if (calc != rec.crc32) continue;
        // Plausibility checks
        if (rec.soc_mpermil > 1000) continue;
        // Choose highest seq
        if (!found || rec.seq > best_seq) {
            best_seq = rec.seq;
            out = rec;
            found = true;
        }
    }
    return found;
}

bool EepromHandler::appendSocJournal(const EepromM24C24Args::SocJournalRecord &in_rec) {
    using namespace EepromM24C24Args;
    const uint32_t base = soc_base_addr_bytes(m_ep);
    SocJournalRecord rec = in_rec;
    // Determine next slot from current max seq
    SocJournalRecord tmp;
    uint32_t best_seq = 0;
    bool have = false;
    for (uint32_t i = 0; i < SOC_SLOTS; ++i) {
        uint32_t addr = base + i * SOC_SLOT_SIZE;
        if (!readRaw(addr, &tmp, sizeof(SocJournalRecord))) continue;
        if (tmp.magic == SOC_MAGIC && tmp.version == SOC_VERSION) {
            uint32_t calc = crc32_calc((const uint8_t*)&tmp, offsetof(SocJournalRecord, crc32));
            if (calc == tmp.crc32) {
                if (!have || tmp.seq > best_seq) {
                    best_seq = tmp.seq; have = true;
                }
            }
        }
    }
    rec.magic = SOC_MAGIC;
    rec.version = SOC_VERSION;
    rec.seq = have ? (best_seq + 1) : 1u;
    rec.crc32 = crc32_calc((const uint8_t*)&rec, offsetof(SocJournalRecord, crc32));

    // Compute slot index
    uint32_t next_index = have ? ((best_seq + 1) % SOC_SLOTS) : 0u;
    uint32_t addr = base + next_index * SOC_SLOT_SIZE;
    return writeRaw(addr, &rec, sizeof(SocJournalRecord));
}
// =====================================

// ===== SOH/Cycles journal implementation =====
static const uint32_t SOHX_MAGIC = 0x58484F53u; // 'SOHX'
static const uint16_t SOHX_VERSION = 1;
static const uint32_t SOHX_AREA_SIZE = 1024; // another 1KB block above SOC or below; use the 2nd-to-last KB
static const uint32_t SOHX_SLOT_SIZE = 32 + 8; // struct ~36 bytes, reserve 40 per slot
static const uint32_t SOHX_SLOTS = SOHX_AREA_SIZE / 40;

static inline uint32_t sohx_base_addr_bytes(EEPROM &ep) {
    // Place SOHX just before SOC area
    return (uint32_t)ep.Type - (1024 + 1024);
}

bool EepromHandler::loadLatestSohCycle(EepromM24C24Args::SohCycleRecord &out) {
    using namespace EepromM24C24Args;
    const uint32_t base = sohx_base_addr_bytes(m_ep);
    SohCycleRecord rec;
    bool found = false;
    uint32_t best_seq = 0;
    for (uint32_t i = 0; i < SOHX_SLOTS; ++i) {
        uint32_t addr = base + i * 40;
        if (!readRaw(addr, &rec, sizeof(SohCycleRecord))) continue;
        if (rec.magic != SOHX_MAGIC || rec.version != SOHX_VERSION) continue;
        uint32_t calc = crc32_calc((const uint8_t*)&rec, offsetof(SohCycleRecord, crc32));
        if (calc != rec.crc32) continue;
        if (rec.soh_mpermil > 1000) continue;
        if (!found || rec.seq > best_seq) {
            best_seq = rec.seq; out = rec; found = true;
        }
    }
    return found;
}

bool EepromHandler::appendSohCycle(const EepromM24C24Args::SohCycleRecord &in_rec) {
    using namespace EepromM24C24Args;
    const uint32_t base = sohx_base_addr_bytes(m_ep);
    SohCycleRecord rec = in_rec;

    // find current max seq
    SohCycleRecord tmp;
    uint32_t best_seq = 0; bool have=false;
    for (uint32_t i = 0; i < SOHX_SLOTS; ++i) {
        uint32_t addr = base + i * 40;
        if (!readRaw(addr, &tmp, sizeof(SohCycleRecord))) continue;
        if (tmp.magic == SOHX_MAGIC && tmp.version == SOHX_VERSION) {
            uint32_t calc = crc32_calc((const uint8_t*)&tmp, offsetof(SohCycleRecord, crc32));
            if (calc == tmp.crc32) { if (!have || tmp.seq > best_seq) { best_seq = tmp.seq; have = true; } }
        }
    }

    rec.magic = SOHX_MAGIC;
    rec.version = SOHX_VERSION;
    rec.seq = have ? (best_seq + 1) : 1u;
    rec.crc32 = crc32_calc((const uint8_t*)&rec, offsetof(SohCycleRecord, crc32));

    uint32_t next_index = have ? ((best_seq + 1) % SOHX_SLOTS) : 0u;
    uint32_t addr = base + next_index * 40;
    return writeRaw(addr, &rec, sizeof(SohCycleRecord));
}
// ================================================
