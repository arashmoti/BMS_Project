#ifndef __BQ79616_HANDLER__H_
#define __BQ79616_HANDLER__H_

#include "mbed.h"

#include <stdint.h>
#include <string>
#include <vector>
#include <cstdio>
#include <chrono>
#include <cstdint>

#include "ISerial.h"
#include "IMbq79616.h"
#include "BQ79616_handler_params.h"
#include "Settings.h"
#include "io_periph_defs.h"

#include "BufferedSerial.h"
#include "ThisThread.h"
#include "Mail.h"
#include "ILInterfaceComm.h"

#include "sdcard_handler.h"
#include "Mutex.h"

// ########################################################################################################################################

class BQ79616Handler : public IMbq79616
{
public:
    BQ79616Handler(Mail<st_mailPowerElecPackInfoConfig, BQ_MEASUREMENT_COMMS_MAIL_SIZE> &mailPowerElecBox, ISerial &_serial, ILInterfaceComm &_interfaceComm, SdCardHandler &_sdCard);
    virtual ~BQ79616Handler();

    virtual void setBalanceActive(bool _value) override;
    virtual bool isBalanceActive() const override { return m_bIsActiveBalance; }
    virtual void setAllowedForResetInternalError() override;

    virtual vector<enCellBalanceTimeIsUp> getBalanceTimeUp(void) const override;
    virtual enBqErrorType getInternalError(void) const override;
    virtual vector<vector<uint8_t>> getAllInternalErrors(void) const override;
    virtual void getHardOverandUnderVoltageFlags(uint16_t *underVoltageFlags, uint16_t *overVoltageFlags) override;
    virtual void getHardOverTemperatureFlags(uint16_t *overTemperatureFlags) override;

    virtual bool faultDetected(void) override;
    virtual measComm::enErrorCodes reset(void) override;
    virtual measComm::enErrorCodes reinitAndReaddress(void) override;
    virtual void shutdown(void) override;
    virtual void sendBalancingValues(st_cellMonitorCells *cellVoltagesIndividual, uint16_t cellMisMatch) override;
    virtual void forceClearBleeders(void) override;

    // Synchronized balance writes - call from Measurement during MEASURE phase
    void executeBalanceWrites(void);
    bool isBalanceWritePending(void) const { return m_bBalanceWritePending; }

    void init(st_bqConfigStructTypedef *bqConfig);
    void startThread(void);

    // Debug: Print communication statistics summary (call periodically, not on every request)
    void printCommStats(void);
    
    // Check communication fault registers
    void validateCommunication(void);

    // Pre-balance health check - returns true if no COMM faults on any device
    bool isCommHealthy(void) override;
    void printLastCommFaultReason(void) override;

    // Track if we've completed a full measurement cycle for all devices
    bool isMeasureCycleComplete(void) const override { return m_bMeasureCycleComplete; }
    void resetMeasureCycleComplete(void) override { m_bMeasureCycleComplete = false; }

private:
    SdCardHandler &m_sdCard;
    void balancing_task(void);

    void request_task(void);
    void process_task(void);
    uint8_t timerDelay1ms(uint32_t *last, uint32_t ticks);
    void BQ_Thread(void);
    void timeout(void);
    void BQ_ISR(void);

    void balanceBattery(void);
    void balanceProcess(void);

    uint8_t getPartID(void);

    bool autoAddress(bool reverseDirection);
#if USE_LEGACY_BQ_DRIVER
    void bqWriteSysConfig(st_bqConfigStructTypedef *bqConfig);
#else
    bool bqWriteSysConfig(st_bqConfigStructTypedef *bqConfig);
#endif
    void setCurrentDeviceAddress(uint8_t _currentDeviceAddress);
    void resetInternalError(void);

    measComm::enErrorCodes setTwoStopBits(void);
    measComm::enErrorCodes setMainADCMode(enMainAdcMode value);
    measComm::enErrorCodes setCellsPerDevice(enNoOFCells noOfCells);
    measComm::enErrorCodes setOverVoltageThreshold(enOverVoltageThreshold overVoltageThreshold);
    measComm::enErrorCodes setUnderVoltageThreshold(enUnderVoltageThreshold underVoltageThreshold);
    measComm::enErrorCodes setOverTemperatureThreshold(enOverTemperatureThreshold overTemperatureThreshold);
    measComm::enErrorCodes setUnderTemperatureThreshold(enUnderTemperatureThreshold underTemperatureThreshold);
    measComm::enErrorCodes setEnableTSREF(void);
    measComm::enErrorCodes GPIO_SetAll(measSettings::enGpioConfig gpio_config);
    measComm::enErrorCodes setVoltageProtectorMode(enProtectionMode mode);
    measComm::enErrorCodes setTemperatureProtectorMode(enProtectionMode value);
    measComm::enErrorCodes maskAllOfFaults(void);
    measComm::enErrorCodes resetAllFaults(void);
    measComm::enErrorCodes resetFaults(void);
    measComm::enErrorCodes setBalanceTime(measSettings::enBalancingTimer time);
    measComm::enErrorCodes resetAllBalanceTime(void);
    measComm::enErrorCodes setBalanceDutyCycle(enBalanceDutyCycle balanceDutyCycle);
    measComm::enErrorCodes pause_CB(void);
    measComm::enErrorCodes start_CB(bool stopOnFault);
    measComm::enErrorCodes set_normalCommDirection();
    measComm::enErrorCodes set_reverseCommDirection();

    measComm::enErrorCodes sleep(void);
    measComm::enErrorCodes sleepToActive(void);
    // measComm::enErrorCodes reset(void);

    measComm::enErrorCodes UpdatePartID(void);
    measComm::enErrorCodes UpdateDeviceAddress(void);
    measComm::enErrorCodes updateVoltages(void);
    measComm::enErrorCodes updateBusbar(void);
    measComm::enErrorCodes updateTSRef(void);
    measComm::enErrorCodes updateTemperatures(void);
    measComm::enErrorCodes updateBalanceState();
    measComm::enErrorCodes updateCellBalanceTimeUp();
    measComm::enErrorCodes updateErrorSummary(void);
    measComm::enErrorCodes updatePWRFaults(void);
    measComm::enErrorCodes updateSystemFaults(void);
    measComm::enErrorCodes updateOVUVFaults(void);
    measComm::enErrorCodes updateOTUTFaults(void);
    measComm::enErrorCodes updateCommFaults(void);
    measComm::enErrorCodes updateComm1SubFaults(void);
    measComm::enErrorCodes updateComm2SubFaults(void);
    void recordCommFault(uint8_t dev, uint8_t comm1, uint8_t comm2, bool hasComm3, uint8_t comm3);

    void processAllDataSaved(void);

    void processPartID(void);
    void processDeviceAddress(void);
    void processVoltages(void);
    void processBusbar(void);
    void processTSRef(void);
    void processTemperatures(void);
    void processBalanceState();
    void processCellBalanceTimeUp();
    void processErrorSummary(void);
    void proccessPWRFaults(void);
    void processSystemFaults(void);
    void processOVUVFaults(void);
    void processOTUTFaults(void);
    void processCommFaults(void);
    void processComm1SubFaults(void);
    void processComm2SubFaults(void);

    vector<uint8_t> createFrame(measComm::enReqType reqType, uint8_t deviceAddr, uint16_t regAddr, const vector<uint8_t> &data);
    uint16_t calculateCRC(const vector<uint8_t> &data);
    bool verifyCRC(const vector<uint8_t> &responseFrame);
    measComm::enErrorCodes sendFrame(const vector<uint8_t> &CommandFrame);
    measComm::enErrorCodes request(measComm::enReqType reqType, uint8_t deviceAddr, uint16_t firstRegAddr, const vector<uint8_t> &dataToWrite);

private:
    st_generalConfig *m_ptrGenConfig;

    ILInterfaceComm &interfaceComm;
    uint32_t m_u32LastSuccessfulCommTimestamp;
    Mail<st_mailPowerElecPackInfoConfig, BQ_MEASUREMENT_COMMS_MAIL_SIZE> &m_mailPowerElecBox;
    mbed::UnbufferedSerial *m_bq79616Serial;
    PinName m_tx;
    PinName m_rx;
    mbed::Timer m_bqManagerTim;

    volatile bool m_bAllDataSavedEnableFlag;
    volatile bool dataReady = false;
    volatile bool TSREF_UPDATED_FLAG = false;
    uint8_t req = 1;

    uint8_t rxSnap[256];
    uint16_t rxSnapLen = 0;

    Ticker ticker;
    Semaphore readSuccess;
    uint8_t MAX_ATTEMPTS = 5;
    uint8_t rxTimer = 0;
    uint8_t firstByte = 0x00;
    uint8_t NumberOfBytes = 0;
    char c_chNewChar;
    uint8_t rxData[256];
    uint8_t rxBuffer[256];
    uint16_t rxIndex = 0;

    ISerial &m_serial;

    mbed::DigitalIn m_faultPin;
    rtos::Thread BQ79616ThreadVar;

    uint16_t m_u16tsRef;
    uint8_t m_u8PartId;
    uint8_t m_u8deviceAddress;
    uint8_t m_u8CurrentDeviceAddress;

    vector<enCellBalanceTimeIsUp> m_enVecCellTimeup;

    int32_t m_i32ModuleCurrent; // filtered signed current in mA
    int32_t m_i32ModuleCurrent_temp;
    uint16_t m_u16ModuleBusbarVal; // last magnitude in mA (non-negative)
    uint16_t m_u16ModuleMaxCellVoltage;
    uint16_t m_u16ModuleMinCellVoltage;
    uint16_t m_u16ModuleCellAverageVoltage;
    uint16_t m_u16CellVoltageMismatch;
    int32_t m_i32FastCurrentVal;
    int32_t m_i32ModuleMaxTemperature;
    int32_t m_i32ModuleMinTemperature;
    int32_t m_i32ModuleAverageTemperature;

    chrono::milliseconds tryTime = 5ms;

    uint8_t m_u8underVoltageThreshold;
    uint8_t m_u8overVoltageThreshold;

    uint8_t m_u8underTemperatureThreshold;
    uint8_t m_u8overTemperatureThreshold;

    enBqErrorType m_u8ErrorType;
    enCellBalanceProcessState m_enBalanceState;

    // Balancing State
    bool m_bIsActiveBalance = false;
    uint32_t m_u32BalancingStopTick = 0;  // Timestamp when balancing last stopped
    uint32_t m_u32BalanceStateUpdateTick = 0;  // Timestamp of last BAL_STAT update

    uint8_t m_u8balanceDutyCycle;

    bool m_bIsAllowedResetInternalError;
    st_bqConfigStructTypedef *m_bqConfig;

    uint8_t m_lastCommFaultDev = 0xFF;
    uint8_t m_lastComm1 = 0;
    uint8_t m_lastComm2 = 0;
    uint8_t m_lastComm3 = 0xFF; // 0xFF means unknown
    uint32_t m_lastCommFaultMs = 0;

    uint8_t m_u8noOfDevices;
    bool m_bShutdownChipFlag;
    bool m_bSleepChipFlag;

    vector<vector<uint8_t>> errorCodes;

    measInternalErrors::unFaultOv1_Register overVoltageGroup1Register;
    measInternalErrors::unFaultOv2_Register overVoltageGroup2Register;
    measInternalErrors::unFaultUv1_Register underVoltageGroup1Register;
    measInternalErrors::unFaultUv2_Register underVoltageGroup2Register;

    measInternalErrors::unFaultOt_Register overTemperatureRegister;

    st_cellMonitorCells cellBalancingValues[NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS];
    uint16_t cellVoltagesBQIndividual[NoOfCELL_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS];
    int32_t tempValuesBQIndividual[NoOfTEMP_POSSIBLE_ON_CHIP * NoOfCELL_MONITORS_POSSIBLE_ON_BMS];
    uint8_t balanceStateIndividual[NoOfCELL_MONITORS_POSSIBLE_ON_BMS];
    measComm::enErrorCodes m_commGeneralError;

    // Debug statistics for communication troubleshooting
    struct {
        uint32_t totalRequests = 0;
        uint32_t successCount = 0;
        uint32_t failCount = 0;
        uint32_t crcErrors = 0;
        uint32_t timeoutErrors = 0;
        uint16_t lastFailedReg = 0;
        uint8_t lastFailedDevice = 0;
        uint8_t consecutiveFails = 0;
        uint32_t lastStatsPrintTime = 0;
    } m_debugStats;

    // Mutex for serializing all BQ communication (prevents read/write collisions)
    rtos::Mutex m_bqCommMutex;
    
    // Flag to pause request_task() polling during balance writes
    volatile bool m_bBalanceWritePending = false;
    
    // Tracks if we've read all devices since last balance cycle (set in request_task case 7)
    volatile bool m_bMeasureCycleComplete = false;
    
    // Queued balance data for batched writes during MEASURE phase
    struct {
        uint8_t timers[NoOfCELL_MONITORS_POSSIBLE_ON_BMS][NoOfCELL_POSSIBLE_ON_CHIP];
        bool pending = false;
    } m_pendingBalanceWrites;
};
#endif // __BQ79616_HANDLER__H_
