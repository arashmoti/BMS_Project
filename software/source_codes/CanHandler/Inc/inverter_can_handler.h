#ifndef __INVERTER_CAN__H_
#define __INVERTER_CAN__H_

#include "mbed.h"
#include <cstdint>
#include <cstdio>

#include "Settings.h"

#include "ISerial.h"

#include "inverter_can_handler_params.h"
#include "io_periph_defs.h"

#include "IMInverterCan.h"
#include "ITInverterCan.h"

class InverterCanHandler : public IMInverterCan, public ITInverterCan
{
public:
    InverterCanHandler(ISerial &_serial);
    virtual ~InverterCanHandler();

    virtual void updateMaxDischargeLimit(uint16_t voltage, uint16_t current) override;
    virtual void updateMaxChargeLimit(uint16_t voltage, uint16_t current) override;
    virtual void setChargeLimitCeiling(uint16_t current) override;
    virtual void updateMinCellVoltage(float minCellVoltage) override;
    virtual void updateMaxCellVoltage(float maxCellVoltage) override;
    virtual void updateMaxCellVoltageNumber(float maxCellVoltageNumber) override;
    virtual void updateMinCellVoltageNumber(float minCellVoltageNumber) override;
    virtual void updateMaxCellTemperature(float maxCellTemperature) override;
    virtual void updateMinCellTemperature(float minCellTemperature) override;
    virtual void updateMaxCellTemperatureNumber(float maxCellTemperatureNumber) override;
    virtual void updateMinCellTemperatureNumber(float minCellTemperatureNumber) override;
    virtual void updateMaxModuleVoltage(float maxModuleVoltage) override;
    virtual void updateMinModuleVoltage(float minModuleVoltage) override;
    virtual void updateMaxModuleVoltageNumber(float maxModuleVoltageNumber) override;
    virtual void updateMinModuleVoltageNumber(float minModuleVoltageNumber) override;
    virtual void updateMaxModuleTemperature(float maxModuleTemperature) override;
    virtual void updateMinModuleTemperature(float minModuleTemperature) override;
    virtual void updateMaxModuleTemperatureNumber(float maxModuleTemperatureNumber) override;
    virtual void updateMinModuleTemperatureNumber(float minModuleTemperatureNumber) override;
    virtual void updateChgDchgForbiddenMark(uint8_t chargeState, uint8_t dischargeState) override;
    virtual void updateHardwareVersion(uint8_t hardwareVersion) override;
    virtual void updateHardwareVersionV(uint8_t hardwareVersionV) override;
    virtual void updateHardwareVersionR(uint8_t hardwareVersionR) override;
    virtual void updateSoftwareVersionVMajor(uint8_t softwareVersionVMajor) override;
    virtual void updateSoftwareVersionVMinor(uint8_t softwareVersionVMinor) override;
    virtual void updateDevelopmentMasterVersion(uint8_t developmentMasterVersion) override;
    virtual void updateDevelopmentSubVersion(uint8_t developmentSubVersion) override;
    virtual void updateModuleQty(uint16_t moduleQty) override;
    virtual void updateModule_In_Series_Qty(uint8_t moduleInSeriesQty) override;
    virtual void updateCellQtyInBatteryModule(uint8_t cellQtyInBatteryModule) override;
    virtual void updateVoltageLevel(uint16_t voltageLevel) override;
    virtual void updateAH_Number(uint16_t AH_Number) override;
    virtual void updateSystemCondition(uint8_t sysCondition) override;
    virtual void updateSOH(uint8_t soh) override;
    virtual void updateSOC(uint8_t soc) override;
    virtual void updateBatteryVoltage(uint16_t voltage) override;
    virtual void updateBatteryCurrent(uint16_t current) override;
    virtual void updateBatteryTemperature(uint16_t temp) override;
    // virtual void updateBatteryCycles              (uint16_t Cycles)                              override;
    virtual void task(void) override;

    void init(void);
    // Publish protect/warning summary into 0x4251 frame (guarded by FEATURE_INVERTER_PROTECT_STATUS)
    void updateWarningAndProtect(enBatteryErrors err) override;
    void updateWarningAndProtect(const InverterCanArgs::st_INVERTER_PROTECT_WARNING *warn) override;
    void updateSystemConditionProtection(uint16_t alarmBits, uint16_t protectionBits) override;
    // Highlight: Added a new function declaration to update the handler with system configuration.
    void update_can_parameters(st_generalConfig *genConfig);

private:
    uint8_t canBusWrite(int32_t id, uint8_t dlc, char data[]);
    virtual void updateBatteryCycles(uint16_t Cycles) override;

    uint8_t sendBatteryPileInfoFrame(void);
    uint8_t sendSystemLimitFrame(void);
    uint8_t sendMinMaxSingleBatteryCellVoltageFrame(void);
    uint8_t sendMinMaxSingleBatteryCellTemperatureFrame(void);
    uint8_t sendSystemStatusFrame(void);
    uint8_t sendMinMaxModuleVoltageFrame(void);
    uint8_t sendMinMaxModuleTemperatureFrame(void);
    uint8_t sendChargeDischargeForbiddenMarkFrame(void);
    uint8_t sendErrorFrame(void);
    uint8_t sendSoftwareHardwareVersionFrame(void);
    uint8_t sendBatteryModuleInfoFrame(void);
    uint8_t sendNameFrame(char *ManufacturerName);
    uint8_t sendSystemConditionFrame(void);

private:
    rtos::Mutex m_can_mutex;

    InverterCanArgs::ut_bmsSysVoltageCurrentTempSocSoh_FRAME_DATA SysVoltageCurrentTempSocSoh_Package;
    InverterCanArgs::ut_bmsChargeDischargeCutoffVoltageCurrent_FRAME_DATA ChargeDischargeCutoffVoltageCurrent_Package;
    InverterCanArgs::ut_MIN_MAX_SINGLE_BATTERY_CELL_VOLTAGE_FRAME_DATA MIN_MAX_SINGLE_BATTERY_CELL_VOLTAGE_Package;
    InverterCanArgs::ut_MIN_MAX_SINGLE_BATTERY_CELL_Temperature_FRAME_DATA MIN_MAX_SINGLE_BATTERY_CELL_Temperature_Package;
    InverterCanArgs::ut_SYSTEM_STATUS_FRAME_DATA SYSTEM_STATUS_Package;
    InverterCanArgs::ut_MIN_MAX_MODULE_VOLATGE_FRAME_DATA MIN_MAX_MODULE_VOLATGE_Package;
    InverterCanArgs::ut_MIN_MAX_MODULE_TEMPERATURE_FRAME_DATA MIN_MAX_MODULE_TEMPERATURE_Package;
    InverterCanArgs::ut_CHAREGE_DISCHARGE_FORBIDDEN_MARK_FRAME_DATA CHAREGE_DISCHARGE_FORBIDDEN_MARK_Package;
    InverterCanArgs::ut_ABNORMALITY_FRAME_DATA ABNORMALITY_Package;
    InverterCanArgs::ut_HARDWARE_SOFTWARE_VERSION_FRAME_DATA HARDWARE_SOFTWARE_VERSION_Package;
    InverterCanArgs::ut_BATTERY_MODULE_INFO_FRAME_DATA BATTERY_MODULE_INFO_Package;
    InverterCanArgs::ut_MANUFACTURER_NAME_HIGH_FRAME_DATA MANUFACTURER_NAME_HIGH_Package;
    InverterCanArgs::ut_MANUFACTURER_NAME_LOW_FRAME_DATA MANUFACTURER_NAME_LOW_Package;
    InverterCanArgs::ut_SLEEP_AWAKE_COMMAND_FRAME_DATA SLEEP_AWAKE_COMMAND_Package;
    InverterCanArgs::ut_CHARGE_DISCHARGE_COMMAND_FRAME_DATA CHARGE_DISCHARGE_COMMAND_Package;
    InverterCanArgs::ut_COMMUNICATION_ERROR_COMMAND_FRAME_DATA COMMUNICATION_ERROR_COMMAND_Package;
    InverterCanArgs::ut_SYSTEM_CONDITION_FRAME_DATA SYSTEM_CONDITION_Package;

    CAN m_can;
    CANMessage txMessage;
    uint8_t m_packID;
    uint16_t m_chargeLimitCeiling_cA = 0xFFFF;
    uint16_t m_lastChargeLimitRequest_cA = 0xFFFF;
    uint16_t m_lastChargeVoltage = 0;

    ISerial &m_serial;
};

#endif // __INVERTER_CAN_HANDLER__H_
