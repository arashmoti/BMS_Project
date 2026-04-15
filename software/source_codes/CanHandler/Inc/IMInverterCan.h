#ifndef __IMINVERTERCAN__H_
#define __IMINVERTERCAN__H_

#include "stdint.h"
#include "inverter_can_handler_params.h"
#include <cstdint>
#include "Settings.h"


class IMInverterCan
{
public:
    IMInverterCan() {};
    virtual ~IMInverterCan() = default;

    virtual void updateMaxDischargeLimit          (uint16_t voltage, uint16_t current)          = 0;
    virtual void updateMaxChargeLimit             (uint16_t voltage, uint16_t current)          = 0;
    // Ceiling for charge current (centi-amps). Applied inside updateMaxChargeLimit.
    virtual void setChargeLimitCeiling            (uint16_t current)                             = 0;
    virtual void updateMinCellVoltage             (float minCellVoltage)                        = 0;
    virtual void updateMaxCellVoltage             (float maxCellVoltage)                        = 0;
    virtual void updateMaxCellVoltageNumber       (float maxCellVoltageNumber)                  = 0;
    virtual void updateMinCellVoltageNumber       (float minCellVoltageNumber)                  = 0;
    virtual void updateMaxCellTemperature         (float maxCellTemperature)                    = 0;
    virtual void updateMinCellTemperature         (float minCellTemperature)                    = 0;
    virtual void updateMaxCellTemperatureNumber   (float maxCellTemperatureNumber)              = 0;
    virtual void updateMinCellTemperatureNumber   (float minCellTemperatureNumber)              = 0;
    virtual void updateMaxModuleVoltage           (float maxModuleVoltage)                      = 0;
    virtual void updateMinModuleVoltage           (float minModuleVoltage)                      = 0;
    virtual void updateMaxModuleVoltageNumber     (float maxModuleVoltageNumber)                = 0;
    virtual void updateMinModuleVoltageNumber     (float minModuleVoltageNumber)                = 0;
    virtual void updateMaxModuleTemperature       (float maxModuleTemperature)                  = 0;
    virtual void updateMinModuleTemperature       (float minModuleTemperature)                  = 0;
    virtual void updateMaxModuleTemperatureNumber (float maxModuleTemperatureNumber)            = 0;
    virtual void updateMinModuleTemperatureNumber (float minModuleTemperatureNumber)            = 0;
    virtual void updateChgDchgForbiddenMark       (uint8_t chargeState, uint8_t dischargeState) = 0;
    virtual void updateHardwareVersion            (uint8_t hardwareVersion)                     = 0;
    virtual void updateHardwareVersionV           (uint8_t hardwareVersionV)                    = 0;
    virtual void updateHardwareVersionR           (uint8_t hardwareVersionR)                    = 0;
    virtual void updateSoftwareVersionVMajor      (uint8_t softwareVersionVMajor)               = 0;
    virtual void updateSoftwareVersionVMinor      (uint8_t softwareVersionVMinor)               = 0;
    virtual void updateDevelopmentMasterVersion   (uint8_t developmentMasterVersion)            = 0;
    virtual void updateDevelopmentSubVersion      (uint8_t developmentSubVersion)               = 0;
    virtual void updateModuleQty                  (uint16_t moduleQty)                          = 0;
    virtual void updateModule_In_Series_Qty       (uint8_t moduleInSeriesQty)                   = 0;
    virtual void updateCellQtyInBatteryModule     (uint8_t cellQtyInBatteryModule)              = 0;
    virtual void updateVoltageLevel               (uint16_t voltageLevel)                       = 0;
    virtual void updateAH_Number                  (uint16_t AH_Number)                          = 0;
    virtual void updateSystemCondition            (uint8_t sysCondition)                        = 0;
    virtual void updateSOH                        (uint8_t soh)                                 = 0; 
    virtual void updateSOC                        (uint8_t soc)                                 = 0; 
    virtual void updateBatteryVoltage             (uint16_t voltage)                               = 0;
    virtual void updateBatteryCurrent             (uint16_t current)                               = 0;
    virtual void updateBatteryTemperature         (uint16_t temp)                                  = 0;
    virtual void updateBatteryCycles              (uint16_t Cycles)                             = 0;
    virtual void updateWarningAndProtect(enBatteryErrors err) = 0;
    virtual void updateWarningAndProtect(const InverterCanArgs::st_INVERTER_PROTECT_WARNING *warn) = 0;
    virtual void updateSystemConditionProtection(uint16_t alarmBits, uint16_t protectionBits) = 0;
                           
};


#endif // __IMINVERTERCAN__H_
