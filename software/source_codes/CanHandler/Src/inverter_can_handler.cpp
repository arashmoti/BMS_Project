#include "inverter_can_handler.h"
#include <cstdint>

InverterCanHandler::InverterCanHandler(ISerial &_serial)
    : m_can(CAN_RX_PIN, CAN_TX_PIN, CANBUS_FREQUENCY),
      m_packID(0x00),
      m_serial(_serial)
{
}

// #############################################################################################//

InverterCanHandler::~InverterCanHandler() {}

// #############################################################################################//

void InverterCanHandler::updateBatteryVoltage(uint16_t voltage)
{
    SysVoltageCurrentTempSocSoh_Package.u16SysVoltage = voltage;
}

void InverterCanHandler::updateBatteryCurrent(uint16_t current)
{
    SysVoltageCurrentTempSocSoh_Package.u16SysCurrent = current;
}

void InverterCanHandler::updateBatteryTemperature(uint16_t temp)
{
    SysVoltageCurrentTempSocSoh_Package.u16Temp = temp;
}

void InverterCanHandler::updateBatteryCycles(uint16_t Cycles)
{
    m_can_mutex.lock();
    SYSTEM_STATUS_Package.st_bytes.Cycle_Period = Cycles;
    m_can_mutex.unlock();
}

void InverterCanHandler::updateSOC(uint8_t soc)
{
    // SysVoltageCurrentTempSocSoh_Package.u8Soc = soc;
    m_can_mutex.lock(); // Lock the data
    SysVoltageCurrentTempSocSoh_Package.u8Soc = soc;
    m_can_mutex.unlock(); // Release the lock
}

void InverterCanHandler::updateSOH(uint8_t soh)
{
    static uint8_t last = 255;
    m_can_mutex.lock();
    SysVoltageCurrentTempSocSoh_Package.u8Soh = soh;
    m_can_mutex.unlock();
    if (soh != last)
    {
        last = soh;
        printf("[INV] updateSOH -> %u ", (unsigned)soh);
    }
}

uint8_t InverterCanHandler::sendBatteryPileInfoFrame(void)
{
    uint8_t result = 0;
    static uint8_t last_soh_sent = 255;
    m_can_mutex.lock();
    uint8_t soh_now = SysVoltageCurrentTempSocSoh_Package.u8Soh;
    if (soh_now != last_soh_sent)
    {
        last_soh_sent = soh_now;
        printf("[INV] 0x4211 send SOH=%u ", (unsigned)soh_now);
    }
    result = canBusWrite(InverterCanArgs::BATTERY_PILE_INFO_FRAME_ID, CANBUS_DATA_SIZE, SysVoltageCurrentTempSocSoh_Package.arrData);
    m_can_mutex.unlock();
    return result;
}

// #############################################################################################//

// Highlight: Implementation of the new function to update CAN values.
void InverterCanHandler::update_can_parameters(st_generalConfig *genConfig)
{
    updateMaxChargeLimit(genConfig->u16MaxChargePackVoltage, genConfig->u16MaxSoftChgAllowedCurrent); // u16MaxHardChgAllowedCurrent
    updateMaxDischargeLimit(genConfig->u16MaxDischargePackVoltage, genConfig->u16MaxHardDchgAllowedCurrent);
    updateAH_Number(genConfig->u16BatteryCapacity);
}

// #############################################################################################//

void InverterCanHandler::updateMaxChargeLimit(uint16_t voltage, uint16_t current)
{
    m_can_mutex.lock();
    m_lastChargeVoltage = voltage;
    m_lastChargeLimitRequest_cA = current;
    ChargeDischargeCutoffVoltageCurrent_Package.u16Charge_Cutoff_Voltage = voltage;
    if (current > m_chargeLimitCeiling_cA)
        current = m_chargeLimitCeiling_cA;
    // Protocol: resolution 0.1A, offset -3000A → Raw = (A + 3000)*10
    float actual_amps = (float)current / 100.0f; // Convert from centi-amps to amps
    uint16_t raw_value = (uint16_t)((actual_amps + 3000.0f) * 10.0f);
    ChargeDischargeCutoffVoltageCurrent_Package.u16Max_Charge_Current = raw_value;
    m_can_mutex.unlock();
}

void InverterCanHandler::setChargeLimitCeiling(uint16_t current)
{
    m_can_mutex.lock();
    m_chargeLimitCeiling_cA = current;
    // Re-apply last requested charge limit through the new ceiling so a change is sent.
    uint16_t effective_cA = m_lastChargeLimitRequest_cA;
    if (effective_cA == 0xFFFF)
        effective_cA = m_chargeLimitCeiling_cA;
    if (effective_cA > m_chargeLimitCeiling_cA)
        effective_cA = m_chargeLimitCeiling_cA;

    float actual_amps = (float)effective_cA / 100.0f;
    uint16_t raw_value = (uint16_t)((actual_amps + 3000.0f) * 10.0f);
    ChargeDischargeCutoffVoltageCurrent_Package.u16Max_Charge_Current = raw_value;
    m_can_mutex.unlock();
}

// #############################################################################################//
void InverterCanHandler::updateMaxDischargeLimit(uint16_t voltage, uint16_t current)
{
    m_can_mutex.lock();
    ChargeDischargeCutoffVoltageCurrent_Package.u16Discharge_Cutoff_Voltage = voltage;
    float actual_amps = (float)current / 100.0f; // Convert from centi-amps to amps
    uint16_t raw_value = (uint16_t)((actual_amps + 3000.0f) * 10.0f);
    ChargeDischargeCutoffVoltageCurrent_Package.u16Max_Discharge_Current = raw_value;
    m_can_mutex.unlock();
}

uint8_t InverterCanHandler::sendSystemLimitFrame(void)
{
    uint8_t result = 0;
    m_can_mutex.lock();
    result = canBusWrite(InverterCanArgs::SYSTEM_LIMITS_FRAME_ID, CANBUS_DATA_SIZE, ChargeDischargeCutoffVoltageCurrent_Package.arrData);
    m_can_mutex.unlock();
    return result;
}

// #############################################################################################//

void InverterCanHandler::updateMaxCellVoltage(float maxCellVoltage)
{
    MIN_MAX_SINGLE_BATTERY_CELL_VOLTAGE_Package.u16Max_Single_Battery_Cell_Voltage = (uint16_t)(maxCellVoltage /* CELL_VOLTAGE_SENS*/);
}

void InverterCanHandler::updateMinCellVoltage(float minCellVoltage)
{
    MIN_MAX_SINGLE_BATTERY_CELL_VOLTAGE_Package.u16Min_Single_Battery_Cell_Voltage = (uint16_t)(minCellVoltage /* CELL_VOLTAGE_SENS*/);
}

void InverterCanHandler::updateMaxCellVoltageNumber(float maxCellVoltageNumber)
{
    MIN_MAX_SINGLE_BATTERY_CELL_VOLTAGE_Package.u16Max_Single_Battery_Cell_Voltage_Number = (uint16_t)(maxCellVoltageNumber /* CELL_VOLTAGE_SENS*/);
}

void InverterCanHandler::updateMinCellVoltageNumber(float minCellVoltageNumber)
{
    MIN_MAX_SINGLE_BATTERY_CELL_VOLTAGE_Package.u16Min_Single_Battery_Cell_Voltage_Number = (uint16_t)(minCellVoltageNumber /* CELL_VOLTAGE_SENS*/);
}

uint8_t InverterCanHandler::sendMinMaxSingleBatteryCellVoltageFrame(void)
{
    uint8_t result = 0;
    result = canBusWrite(InverterCanArgs::MIN_MAX_SINGLE_BATTERY_CELL_VOLTAGE_FRAME_ID, CANBUS_DATA_SIZE, MIN_MAX_SINGLE_BATTERY_CELL_VOLTAGE_Package.arrData);
    return result;
}

// #############################################################################################//

void InverterCanHandler::updateMaxCellTemperature(float maxCellTemperature)
{
    MIN_MAX_SINGLE_BATTERY_CELL_Temperature_Package.u16Max_Single_Battery_Cell_Temperature = (uint16_t)(maxCellTemperature /* CELL_VOLTAGE_SENS*/);
}

void InverterCanHandler::updateMinCellTemperature(float minCellTemperature)
{
    MIN_MAX_SINGLE_BATTERY_CELL_Temperature_Package.u16Min_Single_Battery_Cell_Temperature = (uint16_t)(minCellTemperature /* CELL_VOLTAGE_SENS*/);
}

void InverterCanHandler::updateMaxCellTemperatureNumber(float maxCellTemperatureNumber)
{
    MIN_MAX_SINGLE_BATTERY_CELL_Temperature_Package.u16Max_Single_Battery_Cell_Temperature_Number = (uint16_t)(maxCellTemperatureNumber /* CELL_VOLTAGE_SENS*/);
}

void InverterCanHandler::updateMinCellTemperatureNumber(float minCellTemperatureNumber)
{
    MIN_MAX_SINGLE_BATTERY_CELL_Temperature_Package.u16Min_Single_Battery_Cell_Temperature_Number = (uint16_t)(minCellTemperatureNumber /* CELL_VOLTAGE_SENS*/);
}

uint8_t InverterCanHandler::sendMinMaxSingleBatteryCellTemperatureFrame(void)
{
    uint8_t result = 0;
    result = canBusWrite(InverterCanArgs::MIN_MAX_SINGLE_BATTERY_CELL_TEMPERATURE_FRAME_ID, CANBUS_DATA_SIZE, MIN_MAX_SINGLE_BATTERY_CELL_Temperature_Package.arrData);
    return result;
}

// #############################################################################################//

uint8_t InverterCanHandler::sendSystemStatusFrame(void)
{
    // uint8_t result = 0;
    // result = canBusWrite(InverterCanArgs::SYSTEM_STATUS_FRAME_ID, CANBUS_DATA_SIZE, SYSTEM_STATUS_Package.arrData);
    // return result;
/*
    SYSTEM_CONDITION_Package.byte.alarm                         = 0x0;
    SYSTEM_CONDITION_Package.byte.protections.proBits.BUV       = 0;
    SYSTEM_CONDITION_Package.byte.protections.proBits.BOV       = 0;

    SYSTEM_CONDITION_Package.byte.protections.proBits.PUV       = 1;         // Undervoltage
    SYSTEM_CONDITION_Package.byte.protections.proBits.POV       = 0;         // Overvoltage

    SYSTEM_CONDITION_Package.byte.protections.proBits.CUT       = 0;        // Charge UnderTemperature
    SYSTEM_CONDITION_Package.byte.protections.proBits.COT       = 0;        // Charge OverTemperature

    SYSTEM_CONDITION_Package.byte.protections.proBits.DUT       = 1;        // Discharge UnderTemperature
    SYSTEM_CONDITION_Package.byte.protections.proBits.DOT       = 0;        // Discharge OverTemperature

    SYSTEM_CONDITION_Package.byte.protections.proBits.COC       = 0;        // Charge Overcurrent
    SYSTEM_CONDITION_Package.byte.protections.proBits.DOC       = 0;        // Discharge Overcurrent

    SYSTEM_CONDITION_Package.byte.protections.proBits.MUV       = 0;
    SYSTEM_CONDITION_Package.byte.protections.proBits.MOV       = 0;
*/


   /* if( SYS_ERROR_CELL_OVER_TEMPERATURE_CHARGE || SYS_ERROR_CELL_OVER_TEMPERATURE_DISCHARGE || SYS_ERROR_CELL_UNDER_TEMPERATURE_CHARGE)
        SYSTEM_CONDITION_Package.byte.protections.proBits.DOT = 1; // Discharge OverTemperature*/
    uint8_t result = 0;
    result = canBusWrite(InverterCanArgs::SYSTEM_STATUS_FRAME_ID, CANBUS_DATA_SIZE, SYSTEM_CONDITION_Package.arrData);
    return result; 

     
}

// #############################################################################################//

void InverterCanHandler::updateMaxModuleVoltage(float maxModuleVoltage)
{
    MIN_MAX_MODULE_VOLATGE_Package.u16Module_Max_Voltage = (uint16_t)maxModuleVoltage;
}

void InverterCanHandler::updateMinModuleVoltage(float minModuleVoltage)
{
    MIN_MAX_MODULE_VOLATGE_Package.u16Module_Min_Voltage = (uint16_t)minModuleVoltage;
}

void InverterCanHandler::updateMaxModuleVoltageNumber(float maxModuleVoltageNumber)
{
    MIN_MAX_MODULE_VOLATGE_Package.u16Module_Max_Voltage_Number = (uint16_t)maxModuleVoltageNumber;
}

void InverterCanHandler::updateMinModuleVoltageNumber(float minModuleVoltageNumber)
{
    MIN_MAX_MODULE_VOLATGE_Package.u16Module_Min_Voltage_Number = (uint16_t)minModuleVoltageNumber;
}

uint8_t InverterCanHandler::sendMinMaxModuleVoltageFrame(void)
{
    uint8_t result = 0;
    result = canBusWrite(InverterCanArgs::MIN_MAX_MODULE_VOLTAGE_FRAME_ID, CANBUS_DATA_SIZE, MIN_MAX_MODULE_VOLATGE_Package.arrData);
    return result;
}

// #############################################################################################//

void InverterCanHandler::updateMaxModuleTemperature(float maxModuleTemperature)
{
    MIN_MAX_MODULE_TEMPERATURE_Package.u16Module_Max_Temperature = (uint16_t)maxModuleTemperature;
}

void InverterCanHandler::updateMinModuleTemperature(float minModuleTemperature)
{
    MIN_MAX_MODULE_TEMPERATURE_Package.u16Module_Min_Temperature = (uint16_t)minModuleTemperature;
}

void InverterCanHandler::updateMaxModuleTemperatureNumber(float maxModuleTemperatureNumber)
{
    MIN_MAX_MODULE_TEMPERATURE_Package.u16Module_Max_Temperature_Number = (uint16_t)maxModuleTemperatureNumber;
}

void InverterCanHandler::updateMinModuleTemperatureNumber(float minModuleTemperatureNumber)
{
    MIN_MAX_MODULE_TEMPERATURE_Package.u16Module_Min_Temperature_Number = (uint16_t)minModuleTemperatureNumber;
}

uint8_t InverterCanHandler::sendMinMaxModuleTemperatureFrame(void)
{
    uint8_t result = 0;
    result = canBusWrite(InverterCanArgs::MIN_MAX_MODULE_TEMPERATURE_FRAME_ID, CANBUS_DATA_SIZE, MIN_MAX_MODULE_TEMPERATURE_Package.arrData);
    return result;
}

// #############################################################################################//

void InverterCanHandler::updateChgDchgForbiddenMark(uint8_t chargeState, uint8_t dischargeState)
{
    CHAREGE_DISCHARGE_FORBIDDEN_MARK_Package.Charge_forbidden_mark = chargeState;
    CHAREGE_DISCHARGE_FORBIDDEN_MARK_Package.Discharge_forbidden_mark = dischargeState;
}

uint8_t InverterCanHandler::sendChargeDischargeForbiddenMarkFrame(void)
{
    uint8_t result = 0;
    result = canBusWrite(InverterCanArgs::CHARGE_DISCHARGE_FORBIDDEN_MARK_FRAME_ID, CANBUS_DATA_SIZE, CHAREGE_DISCHARGE_FORBIDDEN_MARK_Package.arrData);
    return result;
}

// #############################################################################################//

uint8_t InverterCanHandler::sendErrorFrame(void)
{
    uint8_t result = 0;
    result = canBusWrite(InverterCanArgs::ERROR_FRAME_ID, CANBUS_DATA_SIZE, ABNORMALITY_Package.arrData);
    return result;
}

// #############################################################################################//

void InverterCanHandler::updateHardwareVersion(uint8_t hardwareVersion)
{
    HARDWARE_SOFTWARE_VERSION_Package.Hardware_version = hardwareVersion;
}

void InverterCanHandler::updateHardwareVersionV(uint8_t hardwareVersionV)
{
    HARDWARE_SOFTWARE_VERSION_Package.Hardware_Version_V = hardwareVersionV;
}

void InverterCanHandler::updateHardwareVersionR(uint8_t hardwareVersionR)
{
    HARDWARE_SOFTWARE_VERSION_Package.Hardware_Version_R = hardwareVersionR;
}

void InverterCanHandler::updateSoftwareVersionVMajor(uint8_t softwareVersionVMajor)
{
    HARDWARE_SOFTWARE_VERSION_Package.Software_Version_V_Major = softwareVersionVMajor;
}

void InverterCanHandler::updateSoftwareVersionVMinor(uint8_t softwareVersionVMinor)
{
    HARDWARE_SOFTWARE_VERSION_Package.Software_Version_V_Minor = softwareVersionVMinor;
}

void InverterCanHandler::updateDevelopmentMasterVersion(uint8_t developmentMasterVersion)
{
    HARDWARE_SOFTWARE_VERSION_Package.Development_Master_Version = developmentMasterVersion;
}

void InverterCanHandler::updateDevelopmentSubVersion(uint8_t developmentSubVersion)
{
    HARDWARE_SOFTWARE_VERSION_Package.Development_Sub_Version = developmentSubVersion;
}

uint8_t InverterCanHandler::sendSoftwareHardwareVersionFrame(void)
{
    uint8_t result = 0;
    result = canBusWrite(InverterCanArgs::SOFTWARE_HARDWARE_VERSION_FRAME_ID, CANBUS_DATA_SIZE, HARDWARE_SOFTWARE_VERSION_Package.arrData);
    return result;
}

// #############################################################################################//

void InverterCanHandler::updateModuleQty(uint16_t moduleQty)
{
    BATTERY_MODULE_INFO_Package.Battery_Module_Qty = moduleQty;
}

void InverterCanHandler::updateModule_In_Series_Qty(uint8_t moduleInSeriesQty)
{
    BATTERY_MODULE_INFO_Package.Battery_Module_In_Series_Qty = moduleInSeriesQty;
}

void InverterCanHandler::updateCellQtyInBatteryModule(uint8_t cellQtyInBatteryModule)
{
    BATTERY_MODULE_INFO_Package.Cell_Qty_In_Battery_Module = cellQtyInBatteryModule;
}

void InverterCanHandler::updateVoltageLevel(uint16_t voltageLevel)
{
    BATTERY_MODULE_INFO_Package.Voltage_Level = voltageLevel;
}

void InverterCanHandler::updateAH_Number(uint16_t AH_Number)
{
    BATTERY_MODULE_INFO_Package.AH_number = AH_Number;
}

uint8_t InverterCanHandler::sendBatteryModuleInfoFrame(void)
{
    uint8_t result = 0;
    result = canBusWrite(InverterCanArgs::BATTERY_MODULE_INFO_FRAME_ID, CANBUS_DATA_SIZE, BATTERY_MODULE_INFO_Package.arrData);
    return result;
}

// #############################################################################################//

uint8_t InverterCanHandler::sendNameFrame(char *ManufacturerName)
{

    if (strlen(ManufacturerName) > 16)
    {
        return 1;
    }
    else
    {
        uint8_t counter = 0;
        for (counter = 0; counter < 8; counter++)
        {
            MANUFACTURER_NAME_HIGH_Package.arrData[counter] = ManufacturerName[counter];
        }
        canBusWrite(InverterCanArgs::NAME_FRAME_ID, CANBUS_DATA_SIZE, MANUFACTURER_NAME_HIGH_Package.arrData);
        ThisThread::sleep_for(1ms);

        for (counter = 8; counter < 16; counter++)
        {
            MANUFACTURER_NAME_LOW_Package.arrData[counter - 6] = ManufacturerName[counter];
        }
        canBusWrite(InverterCanArgs::NAME_2_FRAME_ID, CANBUS_DATA_SIZE, MANUFACTURER_NAME_LOW_Package.arrData);
        ThisThread::sleep_for(1ms);
    }

    return 0;
}

// #############################################################################################//

void InverterCanHandler::updateSystemCondition(uint8_t sysCondition)
{
    // SYSTEM_CONDITION_Package.st_bytes.System_condition_able_to_act_this_command_or_not = sysCondition;
}

uint8_t InverterCanHandler::sendSystemConditionFrame(void)
{
    uint8_t result = 0;
    result = canBusWrite(InverterCanArgs::SYSTEM_CONDITION, CANBUS_DATA_SIZE, SYSTEM_CONDITION_Package.arrData);
    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* LOAD INVERTER DEFAULT PARAMETER */
void InverterCanHandler::init(void)
{
    // CAN ID：0x4210 + Addr (Default)
    updateBatteryVoltage(DEFAULT_BATTERY_PILE_TOTAL_VOLTAGE);
    updateBatteryCurrent(DEFAULT_BATTERY_PILE_CURRENT);
    updateBatteryTemperature(DEFAULT_BMS_TEMPERATURE);
    updateSOC(DEFAULT_SOC);
    updateSOH(DEFAULT_SOH);

    updateMaxChargeLimit(DEFAULT_CHARGE_CUTOFF_VOLTAGE, DEFAULT_MAX_CHARGE_CURRENT);
    updateMaxDischargeLimit(DEFAULT_DISCHARGE_CUTOFF_VOLTAGE, DEFAULT_MAX_DISCHARGE_CURRENT);

    updateAH_Number(DEFAULT_BATTERY_CAPACITY_AH);

    // SYSTEM_STATUS_Package.st_bytes.ut_Alarm.st_Alarm_Bits.BHV = 1;
    SYSTEM_STATUS_Package.st_bytes.ut_Fault.st_Fault_bits.VOLT_ERR = 0;
    SYSTEM_STATUS_Package.st_bytes.ut_Fault.st_Fault_bits.Battery_damage = 1;
}

// #############################################################################################//

void InverterCanHandler::task(void)
{
    if (sendBatteryPileInfoFrame()) /* ADDR -> 0x4211 */
    {
    }
    ThisThread::sleep_for(1ms);

    if (sendSystemLimitFrame()) /* ADDR -> 0x4221 */
    {
    }
    ThisThread::sleep_for(1ms);

    // if (sendMinMaxSingleBatteryCellVoltageFrame())      /* ADDR -> 0x4231 */ {}
    // ThisThread::sleep_for(1ms);

    // if (sendMinMaxSingleBatteryCellTemperatureFrame())  /* ADDR -> 0x4241 */ {}
    // ThisThread::sleep_for(1ms);

    if (sendSystemStatusFrame()) /* ADDR -> 0x4251 */
    {
    }
    ThisThread::sleep_for(1ms);

    // if (sendMinMaxModuleVoltageFrame())                 /* ADDR -> 0x4261 */ {}
    // ThisThread::sleep_for(1ms);

    // if (sendMinMaxModuleTemperatureFrame())             /* ADDR -> 0x4271 */ {}
    // ThisThread::sleep_for(1ms);

    // if (sendChargeDischargeForbiddenMarkFrame())        /* ADDR -> 0x4281 */ {}
    // ThisThread::sleep_for(1ms);

    // if (sendErrorFrame())                               /* ADDR -> 0x4291 */ {}
    // ThisThread::sleep_for(1ms);

    // if (sendSoftwareHardwareVersionFrame())             /* ADDR -> 0x7311 */ {}
    // ThisThread::sleep_for(1ms);

    if (sendBatteryModuleInfoFrame()) /* ADDR -> 0x7321 */
    {
    }
    ThisThread::sleep_for(1ms);

    // if (sendNameFrame((char *)DEFAULT_MANUFACTURER_NAME_HIGH))      /* ADDR -> 0x7331 & 0x7341 */ {}
    // ThisThread::sleep_for(1ms);

    // if (sendSystemConditionFrame())      /* ADDR -> 0x8251 */ {}
    // ThisThread::sleep_for(1ms);

    // === NEW: CAN TX Summary (30s interval) ===
    {
        static uint32_t s_canSummaryTick = 0;
        uint32_t nowMs = rtos::Kernel::Clock::now().time_since_epoch().count();
        if ((nowMs - s_canSummaryTick) >= 30000)
        {
            s_canSummaryTick = nowMs;
            m_can_mutex.lock();
            uint8_t soc = SysVoltageCurrentTempSocSoh_Package.u8Soc;
            uint16_t voltage = SysVoltageCurrentTempSocSoh_Package.u16SysVoltage;
            uint16_t current = SysVoltageCurrentTempSocSoh_Package.u16SysCurrent;
            uint16_t chgVolt = ChargeDischargeCutoffVoltageCurrent_Package.u16Charge_Cutoff_Voltage;
            uint16_t dchgVolt = ChargeDischargeCutoffVoltageCurrent_Package.u16Discharge_Cutoff_Voltage;
            m_can_mutex.unlock();
            m_serial.serialPrint(
                "[CAN-TX] SOC=%u%% V=%uV I=%dA ChgV=%uV DchgV=%uV\r\n",
                (unsigned)soc, (unsigned)(voltage / 10), (int)((int16_t)current / 10),
                (unsigned)(chgVolt / 10), (unsigned)(dchgVolt / 10));
        }
    }
}

// #############################################################################################//
/* WRITE DATA TO CAN BUS HARDWARE */
uint8_t InverterCanHandler::canBusWrite(int32_t id, uint8_t dlc, char data[])
{
    txMessage.id = id;
    // txMessage.format  = CANStandard;
    txMessage.format = CANExtended;
    txMessage.type = CANData;
    txMessage.len = dlc;
    memcpy(txMessage.data, data, 8);

    uint8_t cnt = 0;
    while (!m_can.write(txMessage) && cnt < 3)
    {
        cnt++;
    }
    return cnt;
}
// #############################################################################################//
// Map BMS error to System Status protection/fault bits

void InverterCanHandler::updateWarningAndProtect(enBatteryErrors err)
{
#if FEATURE_INVERTER_PROTECT_STATUS
    m_can_mutex.lock();
    SYSTEM_STATUS_Package.st_bytes.ut_Protection.u16_Protection_Bytes = 0;
    SYSTEM_STATUS_Package.st_bytes.ut_Fault.st_Fault_bits.IN_COMM_ERR = (err == SYS_ERROR_CHIPSET) ? 1 : 0;

    switch (err)
    {
    case SYS_ERROR_CELL_OVER_VOLTAGE:
        SYSTEM_CONDITION_Package.byte.protections.proBits.POV = 1; // Overvoltage
        break;
    case SYS_ERROR_CELL_UNDER_VOLTAGE:
        SYSTEM_CONDITION_Package.byte.protections.proBits.PUV = 1; // Undervoltage

        break;
    case SYS_ERROR_CHG_OVER_CURRENT:
        SYSTEM_CONDITION_Package.byte.protections.proBits.DOC = 1; // Charge Overcurrent
        SYSTEM_CONDITION_Package.byte.protections.proBits.COC = 1; // Discharge Overcurrent
        break;
    case SYS_ERROR_DCHG_OVER_CURRENT:
        SYSTEM_CONDITION_Package.byte.protections.proBits.DOC = 1; // Discharge Overcurrent
        SYSTEM_CONDITION_Package.byte.protections.proBits.COC = 1; // Charge Overcurrent
        break;
    case SYS_ERROR_CELL_OVER_TEMPERATURE_CHARGE:
        SYSTEM_CONDITION_Package.byte.protections.proBits.DOT = 1; // Discharge OverTemperature
        break;
    case SYS_ERROR_CELL_OVER_TEMPERATURE_DISCHARGE:
        SYSTEM_CONDITION_Package.byte.protections.proBits.DOT = 1; // Discharge OverTemperature
        break;
    case SYS_ERROR_CELL_OVER_TEMPERATURE_IDLE:
        SYSTEM_CONDITION_Package.byte.protections.proBits.DOT = 1; // Discharge OverTemperature
        break;
    case SYS_ERROR_CELL_UNDER_TEMPERATURE_CHARGE:
        SYSTEM_CONDITION_Package.byte.protections.proBits.CUT = 1; // Charge UnderTemperature
        SYSTEM_CONDITION_Package.byte.protections.proBits.DUT = 1; // Discharge UnderTemperature

        break;
    case SYS_ERROR_CELL_UNDER_TEMPERATURE_DISCHARGE:
        SYSTEM_CONDITION_Package.byte.protections.proBits.DUT = 1; // Discharge UnderTemperature
        SYSTEM_CONDITION_Package.byte.protections.proBits.CUT = 1; // Charge UnderTemperature
        break;
    case SYS_ERROR_SHORT_CIRCUIT:
        // Represent as discharge overcurrent protect for inverter side
        SYSTEM_CONDITION_Package.byte.protections.proBits.DOC = 1; // Discharge Overcurrent
        break;
    case SYS_OK:
        SYSTEM_CONDITION_Package.byte.protections.proBits.POV = 0;
        SYSTEM_CONDITION_Package.byte.protections.proBits.PUV = 0;
        SYSTEM_CONDITION_Package.byte.protections.proBits.COC = 0;
        SYSTEM_CONDITION_Package.byte.protections.proBits.DOC = 0;
        SYSTEM_CONDITION_Package.byte.protections.proBits.CUT = 0;
        SYSTEM_CONDITION_Package.byte.protections.proBits.DUT = 0;
        SYSTEM_CONDITION_Package.byte.protections.proBits.DOT = 0;
        break;
    default:
        break;
    }
    m_can_mutex.unlock();
#else
    (void)err;
#endif
}
void InverterCanHandler::updateWarningAndProtect(const InverterCanArgs::st_INVERTER_PROTECT_WARNING *warn)
{
    /*
#if FEATURE_INVERTER_PROTECT_STATUS
    if (!warn)
        return;
    m_can_mutex.lock();
    SYSTEM_STATUS_Package.st_bytes.ut_Protection.u16_Protection_Bytes = 0;

    // Protection bit positions per inverter table (Bit0..Bit11)
    constexpr uint16_t PROT_BUV = (1u << 0);
    constexpr uint16_t PROT_BOV = (1u << 1);
    constexpr uint16_t PROT_PUV = (1u << 2);
    constexpr uint16_t PROT_POV = (1u << 3);
    constexpr uint16_t PROT_CUT = (1u << 4);
    constexpr uint16_t PROT_COT = (1u << 5);
    constexpr uint16_t PROT_DUT = (1u << 6);
    constexpr uint16_t PROT_DOT = (1u << 7);
    constexpr uint16_t PROT_COC = (1u << 8);
    constexpr uint16_t PROT_DOC = (1u << 9);
    constexpr uint16_t PROT_MUV = (1u << 10);
    constexpr uint16_t PROT_MOV = (1u << 11);

    SYSTEM_STATUS_Package.st_bytes.ut_Fault.st_Fault_bits.IN_COMM_ERR =
        warn->sysProtect.u8bitInternalArrive ? 1 : 0;

    if (warn->sysProtect.u8bitHighVoltageArrive)
        SYSTEM_STATUS_Package.st_bytes.ut_Protection.u16_Protection_Bytes |= PROT_BOV;
    if (warn->sysProtect.u8bitLowVoltageArrive)
        SYSTEM_STATUS_Package.st_bytes.ut_Protection.u16_Protection_Bytes |= PROT_BUV;
    if (warn->sysProtect.u8bitChargeHighCurrentArrive)
        SYSTEM_STATUS_Package.st_bytes.ut_Protection.u16_Protection_Bytes |= PROT_COC;
    if (warn->sysProtect.u8bitDisChargeHighCurrentArrive || warn->sysProtect.u8bitShortCircuitArrive)
        SYSTEM_STATUS_Package.st_bytes.ut_Protection.u16_Protection_Bytes |= PROT_DOC;
    if (warn->sysProtect.u8bitChargeHighTempArrive)
        SYSTEM_STATUS_Package.st_bytes.ut_Protection.u16_Protection_Bytes |= PROT_COT;
    if (warn->sysProtect.u8bitDisChargeHighTempArrive)
        SYSTEM_STATUS_Package.st_bytes.ut_Protection.u16_Protection_Bytes |= PROT_DOT;
    if (warn->sysProtect.u8bitChargeLowTempArrive)
        SYSTEM_STATUS_Package.st_bytes.ut_Protection.u16_Protection_Bytes |= PROT_CUT;
    if (warn->sysProtect.u8bitDisChargeLowTempArrive)
        SYSTEM_STATUS_Package.st_bytes.ut_Protection.u16_Protection_Bytes |= PROT_DUT;

    m_can_mutex.unlock();
#else
    (void)warn;
#endif
*/
}

void InverterCanHandler::updateSystemConditionProtection(uint16_t alarmBits, uint16_t protectionBits)
{
    m_can_mutex.lock();
    SYSTEM_CONDITION_Package.byte.alarm = alarmBits;
    SYSTEM_CONDITION_Package.byte.protections.proWord = protectionBits;
    m_can_mutex.unlock();
}

