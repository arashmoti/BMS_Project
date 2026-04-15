#include "BQ79616_handler.h"
#include <cstdint>
#include <cstring>
#include <numeric>
#include <stdint.h>
#include <vector>
#include "interface_comm_handler.h"
#include "platform/CriticalSectionLock.h"
#include "mbed_critical.h" // for core_util_critical_section_enter/exit
#include "operation.h"     // for hb_set / HB_BQ

// ########################################################################################################################################

// NEW: defensive check for external addresses
static inline bool validDeviceAddr(uint8_t addr)
{
    return addr < NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
}

// ########################################################################################################################################

BQ79616Handler::BQ79616Handler(Mail<st_mailPowerElecPackInfoConfig, BQ_MEASUREMENT_COMMS_MAIL_SIZE> &_mailPowerElecBox, ISerial &_serial, ILInterfaceComm &_interfaceComm, SdCardHandler &_sdCard) : interfaceComm(_interfaceComm), m_sdCard(_sdCard),
                                                                                                                                                                             m_tx(BQ79616_TX_PIN),
                                                                                                                                                                             m_rx(BQ79616_RX_PIN),
                                                                                                                                                                             m_serial(_serial),
                                                                                                                                                                             m_mailPowerElecBox(_mailPowerElecBox),
                                                                                                                                                                             m_faultPin(BQ79616_FAULT_PIN),
                                                                                                                                                                             m_u8overVoltageThreshold(0),
                                                                                                                                                                             m_u8underVoltageThreshold(0),
                                                                                                                                                                             m_u8underTemperatureThreshold(0),
                                                                                                                                                                             m_u8overTemperatureThreshold(0),
                                                                                                                                                                             m_i32ModuleCurrent(0),
                                                                                                                                                                             m_i32FastCurrentVal(0),
                                                                                                                                                                             m_i32ModuleMaxTemperature(0),
                                                                                                                                                                             m_i32ModuleMinTemperature(0),
                                                                                                                                                                             m_u16ModuleBusbarVal(0),
                                                                                                                                                                             m_u16ModuleMaxCellVoltage(0),
                                                                                                                                                                             m_u16ModuleMinCellVoltage(0),
                                                                                                                                                                             m_u16ModuleCellAverageVoltage(0),
                                                                                                                                                                             m_u16CellVoltageMismatch(0),
                                                                                                                                                                             m_i32ModuleAverageTemperature(0),
                                                                                                                                                                             m_bIsActiveBalance(false),
                                                                                                                                                                             m_bIsAllowedResetInternalError(false),
                                                                                                                                                                             m_bShutdownChipFlag(false),
                                                                                                                                                                             m_bSleepChipFlag(false),
                                                                                                                                                                             m_bAllDataSavedEnableFlag(false),
                                                                                                                                                                             m_enBalanceState(INACTIVE),
                                                                                                                                                                             BQ79616ThreadVar(BQ_THREAD_PRIORITY, BQ_THREAD_STACK_SIZE, nullptr, "BQ"),
                                                                                                                                                                             errorCodes(NoOfCELL_MONITORS_POSSIBLE_ON_BMS, vector<uint8_t>(0))
{
    m_bqManagerTim.start();
}

// ########################################################################################################################################

BQ79616Handler::~BQ79616Handler() {}

// ########################################################################################################################################

void BQ79616Handler::startThread()
{
    BQ79616ThreadVar.start(mbed::callback(this, &BQ79616Handler::BQ_Thread));
}

// ########################################################################################################################################

void BQ79616Handler::init(st_bqConfigStructTypedef *bqConfig)
{
    m_bqConfig = bqConfig;

    interfaceComm.printToInterface("[BQ_INIT] Starting BQ79616 initialization...\r\n");

    // STEP 1: Send a strong hardware wake-up pulse.
    // We toggle the actual TX pin (if accessible) or just wait.
    DigitalOut wake(m_tx);
    wake = 0;
    wait_us(5000); // 5ms pulse
    wake = 1;

    // STEP 2: Wait for the BQ chip to stabilize (LDOs, OSCs).
    // TI recommends ~10-20ms, but we use 250ms to be safe.
    ThisThread::sleep_for(250ms);

    // STEP 3: Now that the chips are awake and stable, initialize the serial port.
    m_bq79616Serial = new UnbufferedSerial(m_tx, m_rx, BQ79616_BAUDRATE);
    m_bq79616Serial->format(8, SerialBase::None, 1);
    ThisThread::sleep_for(20ms);  // Slightly longer delay

    interfaceComm.printToInterface("[BQ_INIT] Serial port configured, starting auto-address...\r\n");

    // STEP 4: Send the software reset command. This is now done from main.cpp.

    // STEP 5: Proceed with the normal configuration.
    bqWriteSysConfig(m_bqConfig);
    ThisThread::sleep_for(150ms);  // Increased from 100ms

    // setTwoStopBits(); // Disabled to ensure standard 1-stop bit alignment with MCU
    ThisThread::sleep_for(20ms);
    m_bq79616Serial->format(8, SerialBase::None, 1); // Standard: 1 Stop Bit
    ThisThread::sleep_for(150ms);  // Increased from 100ms

    m_bq79616Serial->attach(mbed::callback(this, &BQ79616Handler::BQ_ISR));
    ThisThread::sleep_for(100ms);

    ticker.attach(mbed::callback(this, &BQ79616Handler::timeout), 1ms);

    m_u32LastSuccessfulCommTimestamp = BQ_GET_TICK(m_bqManagerTim);
    
    interfaceComm.printToInterface("[BQ_INIT] Initialization complete\r\n");
}
// ########################################################################################################################################
void BQ79616Handler::BQ_Thread(void)
{

    // local snapshot buffer to parse from safely (use fixed size, bound copies)
    // Note: avoid VLA; use the member rxSnap[256] defined in the class

    uint32_t _u32bqServiceMeasureIntervalLastTick = 0;
    uint32_t _u32balancingTaskLastTick = 0;
    uint32_t _u32statsLastTick = 0;

    // Print initial stats after 5 seconds to confirm BQ communication started
    bool _bInitialStatsPrinted = false;

    while (1)
    {
        if (dataReady)
        {
            // ---- snapshot the ISR-produced frame atomically ----
            core_util_critical_section_enter();
            // We assume ISR only sets dataReady after memcpy-ing a full frame into rxBuffer
            uint16_t copyLen = NumberOfBytes;
            if (copyLen > sizeof(rxSnap))
                copyLen = sizeof(rxSnap);
            memcpy(rxSnap, rxBuffer, copyLen);
            dataReady = false; // consume the frame
            core_util_critical_section_exit();
            // ----------------------------------------------------

            // Keep your existing parsers unchanged by copying the stable
            // snapshot back into rxBuffer (short and bounded copy).
            memcpy(rxBuffer, rxSnap, copyLen);

            if (m_commGeneralError == measComm::enErrorCodes::SUCCESS)
            {
                process_task(); // uses rxBuffer as before
            }
        }

        if (!m_bShutdownChipFlag && !m_bAllDataSavedEnableFlag)
        {
            request_task();

            if (timerDelay1ms(&_u32balancingTaskLastTick, 3000))
            {
                balancing_task();
            }
        }

        if (m_bAllDataSavedEnableFlag)
        {
            processAllDataSaved();
        }

        // Print initial stats after 5 seconds, then every 30 seconds
        if (!_bInitialStatsPrinted && timerDelay1ms(&_u32statsLastTick, 5000))
        {
            _bInitialStatsPrinted = true;
            printCommStats();
            validateCommunication(); // Check for COMM faults
        }
        else if (_bInitialStatsPrinted && timerDelay1ms(&_u32statsLastTick, 30000))
        {
            printCommStats();
            validateCommunication(); // Check for COMM faults
        }

        hb_set(HB_BQ); // <— add this

        ThisThread::sleep_for(1ms);
    }

    // uint32_t _u32bqServiceMeasureIntervalLastTick = 0;
    // uint32_t _u32balancingTaskLastTick = 0;

    // while (1)
    // {
    //     if (dataReady)
    //     {
    //         dataReady = false;
    //         if (m_commGeneralError == measComm::enErrorCodes::SUCCESS)
    //         {
    //             process_task();
    //         }
    //     }

    //     if (!m_bShutdownChipFlag && !m_bAllDataSavedEnableFlag)
    //     {
    //         request_task();

    //         if (timerDelay1ms(&_u32balancingTaskLastTick, 3000))
    //         {
    //             balancing_task();
    //         }
    //     }

    //     if (m_bAllDataSavedEnableFlag)
    //     {
    //         processAllDataSaved();
    //     }

    //     ThisThread::sleep_for(1ms);
    // }
}

// ########################################################################################################################################

void BQ79616Handler::process_task()
{
    uint16_t regAddress;
    regAddress = (rxBuffer[2] << 8 | rxBuffer[3]);

    // if(mailPowerElecPackInfoConfig)
    {
        switch (regAddress)
        {
        case PARTID:
            processPartID();
            break;

        case DIR0_ADDR:
            processDeviceAddress();
            break;

        case VCELL16_HI:
        {
            processVoltages();
        }
        break;

        case BUSBAR_HI:
            processBusbar();
            break;

        case TSREF_HI:
            processTSRef();
            break;

        case GPIO1_HI:
            processTemperatures();
            break;

        case BAL_STAT:
            processBalanceState();
            break;

        case CB_COMPLETE1:
            processCellBalanceTimeUp();
            break;

        case FAULT_SUMMARY:
            processErrorSummary();
            break;

        case FAULT_PWR1:
            proccessPWRFaults();
            break;

        case FAULT_SYS:
            processSystemFaults();
            break;

        case FAULT_OV1:
            processOVUVFaults();
            break;

        case FAULT_OT:
            processOTUTFaults();
            break;

        case FAULT_COMM1:
            processCommFaults();
            break;

        case DEBUG_UART_RC:
            processComm1SubFaults();
            break;

        case DEBUG_COMH_BIT:
            processComm2SubFaults();
            break;
        }
    }
}

// ########################################################################################################################################

void BQ79616Handler::request_task(void)
{
    static uint8_t deviceIndex = 0;
    static uint8_t taskState = 0;
    static measComm::enErrorCodes error = measComm::enErrorCodes::SUCCESS;

    switch (taskState)
    {
    case 0:
        setCurrentDeviceAddress(deviceIndex);
        taskState = 1;
        break;

    case 1:
        // UpdatePartID();
        taskState = 2;
        break;

    case 2:
        error = updateTSRef();
        taskState = 3;
        break;

    case 3:
        error = updateTemperatures();
        taskState = 4;
        break;

    case 4:
        error = updateErrorSummary();
        taskState = 5;
        break;

    case 5:
        error = updateBusbar();
        taskState = 6;
        break;

    case 6:

        // This case is now only responsible for getting new voltage data.
        error = updateVoltages();
        taskState = 7;
        break;

        // if (m_bIsActiveBalance)
        //     balanceBattery();
        // else
        //     error = updateVoltages();
        // taskState = 7;
        // break;
        // Step 1: ALWAYS request new voltage data to keep the system live.

        // error = updateVoltages();

        // // Step 2: As a separate action, if balancing is needed, send those commands.
        // if (m_bIsActiveBalance)
        // {
        //     balanceBattery();
        // }

        // taskState = 7;
        // break;

    case 7:
        deviceIndex++;
        if (deviceIndex >= NoOfCELL_MONITORS_POSSIBLE_ON_BMS)
        {
            deviceIndex = 0;
        }
        taskState = 0;
        break;
    }
}

void BQ79616Handler::processAllDataSaved(void)
{
    // st_mailPowerElecPackInfoConfig *mailPowerElecPackInfoConfig = m_mailPowerElecBox.alloc();

    // if (mailPowerElecPackInfoConfig)
    // {
    //     memcpy(&mailPowerElecPackInfoConfig->cellVoltagesIndividual, &cellVoltagesBQIndividual, sizeof(cellVoltagesBQIndividual));
    //     memcpy(&mailPowerElecPackInfoConfig->tempValuesIndividual, &tempValuesBQIndividual, sizeof(tempValuesBQIndividual));
    //     memcpy(&mailPowerElecPackInfoConfig->balanceStateIndividual, &balanceStateBQIndividual, sizeof(balanceStateBQIndividual));

    //     mailPowerElecPackInfoConfig->u16ModuleBusbarVal = m_u16ModuleBusbarVal;
    // (removed legacy commented assignment of module current)

    //     m_mailPowerElecBox.put(mailPowerElecPackInfoConfig);
    //     m_bAllDataSavedEnableFlag = false;
    // }
    st_mailPowerElecPackInfoConfig *mailPowerElecPackInfoConfig = m_mailPowerElecBox.try_alloc();

    if (mailPowerElecPackInfoConfig)
    {
        memcpy(&mailPowerElecPackInfoConfig->cellVoltagesIndividual, &cellVoltagesBQIndividual, sizeof(cellVoltagesBQIndividual));
        memcpy(&mailPowerElecPackInfoConfig->tempValuesIndividual, &tempValuesBQIndividual, sizeof(tempValuesBQIndividual));
        memcpy(&mailPowerElecPackInfoConfig->balanceStateIndividual, &balanceStateIndividual, sizeof(balanceStateIndividual));

        mailPowerElecPackInfoConfig->u16ModuleBusbarVal = m_u16ModuleBusbarVal;
        mailPowerElecPackInfoConfig->i32ModuleCurrent = m_i32ModuleCurrent;

        m_mailPowerElecBox.put(mailPowerElecPackInfoConfig);
        m_bAllDataSavedEnableFlag = false;
    }
}

// ########################################################################################################################################

void BQ79616Handler::timeout()
{
    if (rxTimer != 0)
    {
        rxTimer--;
    }
    if (rxTimer == 1)
    {
        rxTimer = 0;
        rxIndex = 0;
    }
}

// ########################################################################################################################################
static inline uint16_t calculateCRC_raw(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF; // same start value as your vector version
    for (uint16_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]);
        for (uint8_t b = 0; b < 8; ++b)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ POLY; // uses the same POLY constant already used in your file
            else
                crc >>= 1;
        }
    }
    return crc;
}
// ########################################################################################################################################
static inline bool verifyCRC_raw(const uint8_t *data, uint16_t len)
{
    // Either implement CRC directly here,
    // or call your existing verifyCRC by creating a *local* std::array if len is fixed,
    // but DO NOT allocate std::vector in ISR.
    return calculateCRC_raw(data, len) == 0x0000;
}
// ########################################################################################################################################
void BQ79616Handler::BQ_ISR()
{
    // uint8_t byte;
    // int processed = 0;
    // const int kMaxPerIrq = 32; // cap to avoid ISR hogging if line is noisy

    // while (processed < kMaxPerIrq && m_bq79616Serial->read(&byte, 1) == 1)
    // {
    //     // resync to frame start
    //     if (rxIndex == 0 && byte != firstByte)
    //     {
    //         rxTimer = 10;
    //         ++processed;
    //         continue;
    //     }

    //     // bounds
    //     const uint16_t cap = static_cast<uint16_t>(sizeof(rxData));
    //     if (rxIndex < cap)
    //     {
    //         rxData[rxIndex++] = byte;
    //     }
    //     else
    //     {
    //         rxIndex = 0; // overflow recovery
    //         ++processed;
    //         continue;
    //     }

    //     rxTimer = 10;

    //     if (rxIndex == NumberOfBytes)
    //     {
    //         if (rxData[0] == firstByte && verifyCRC_raw(rxData, NumberOfBytes))
    //         {
    //             memcpy(rxBuffer, rxData, NumberOfBytes);
    //             rxIndex = 0;
    //             dataReady = true;

    //             // ONLY signal the RTOS if the kernel is running
    //             if (osKernelGetState() == osKernelRunning)
    //             {
    //                 readSuccess.release();
    //             }

    //             m_u32LastSuccessfulCommTimestamp = BQ_GET_TICK(m_bqManagerTim);
    //         }
    //         else
    //         {
    //             rxIndex = 0;
    //         }
    //     }

    //     ++processed;
    // }

    if (m_bq79616Serial->read(&c_chNewChar, 1))
    {
        rxData[rxIndex] = c_chNewChar;
        rxIndex++;
        rxTimer = 10;

        if (rxIndex == NumberOfBytes)
        {
            if (rxData[0] == firstByte)
            {
                if (verifyCRC_raw(rxData, rxIndex))
                {
                    memcpy(rxBuffer, rxData, rxIndex);

                    rxIndex = 0;
                    dataReady = true;

                    readSuccess.release();
                    m_u32LastSuccessfulCommTimestamp = BQ_GET_TICK(m_bqManagerTim);
                }
            }
        }
    }
}

// #############################################################################################################################################################################

uint8_t BQ79616Handler::timerDelay1ms(uint32_t *last, uint32_t ticks)
{
    if ((uint32_t)(BQ_GET_TICK(m_bqManagerTim) - *last) >= ticks)
    {
        *last = BQ_GET_TICK(m_bqManagerTim);
        return true;
    }
    return false;
}

// ########################################################################################################################################

void BQ79616Handler::balanceBattery(void)
{
    updateBalanceState();

    if (m_enBalanceState != RUNNING)
    {
        updateCellBalanceTimeUp();
        // NOTE: Removed updateVoltages() call here - it was causing conflicting reads with the main
        // polling loop, resulting in wild voltage swings during balancing. The main polling loop
        // (request_task) already updates voltages continuously. Balancing decisions use the
        // already-populated cellBalancingValues[] from sendBalancingValues().
        balanceProcess();
    }
    else if (m_enBalanceState == DONE)
    {
        m_bIsActiveBalance = false;
    }
}

// ########################################################################################################################################

void BQ79616Handler::balanceProcess()
{

      // Decide per-cell timer from pack mismatch (more aggressive for larger dv)
      auto decideTimer = [&](uint16_t dv_mV) -> uint8_t
      {
          if (dv_mV >= 100)
              return (uint8_t)measSettings::enBalancingTimer::T_10M;   // max duration
          if (dv_mV >= 60)
              return (uint8_t)measSettings::enBalancingTimer::T_300S;  // ~50% duty
          if (dv_mV >= 30)
              return (uint8_t)measSettings::enBalancingTimer::T_60S;
          return (uint8_t)measSettings::enBalancingTimer::T_30S;
      };

      // Keep existing timers running unless we change selection elsewhere
      // resetAllBalanceTime();

    bool anyActiveGlobal = false;

    // Track last mask per device to avoid log spam
    static uint16_t s_lastMask[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {};
    static bool s_lastActive[NoOfCELL_MONITORS_POSSIBLE_ON_BMS] = {};

    for (uint8_t dev = 0; dev < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++dev)
    {
        // Slice into the flattened cell array for this device
        const size_t base = (size_t)dev * NoOfCELL_POSSIBLE_ON_CHIP;
        uint8_t t[NoOfCELL_POSSIBLE_ON_CHIP] = {0};
        bool anyActive = false;
        uint16_t mask = 0;

        for (int c = 0; c < NoOfCELL_POSSIBLE_ON_CHIP; ++c)
        {
            const size_t idx = base + (size_t)c;
            if (idx < (sizeof(cellBalancingValues) / sizeof(cellBalancingValues[0])) &&
                cellBalancingValues[idx].bCellBleedActive)
            {
                t[c] = decideTimer(m_u16CellVoltageMismatch);
                anyActive = true;
                mask |= (uint16_t)(1u << c);
            }
            else
            {
                t[c] = (uint8_t)measSettings::enBalancingTimer::T_NONE;
            }
        }

        // --- Explicit register order ---
        // Many BQ7961x maps CB_CELL16_CTRL to C16..C9 (8 bytes) and CB_CELL8_CTRL to C8..C1 (8 bytes).
        // We avoid reverse() and fill in exact order to remove ambiguity.

        std::vector<uint8_t> data_cell16;
        data_cell16.reserve(8);       // C16..C9
        data_cell16.push_back(t[15]); // C16
        data_cell16.push_back(t[14]); // C15
        data_cell16.push_back(t[13]); // C14
        data_cell16.push_back(t[12]); // C13
        data_cell16.push_back(t[11]); // C12
        data_cell16.push_back(t[10]); // C11
        data_cell16.push_back(t[9]);  // C10
        data_cell16.push_back(t[8]);  // C9

        std::vector<uint8_t> data_cell8;
        data_cell8.reserve(8);      // C8..C1
        data_cell8.push_back(t[7]); // C8
        data_cell8.push_back(t[6]); // C7
        data_cell8.push_back(t[5]); // C6
        data_cell8.push_back(t[4]); // C5
        data_cell8.push_back(t[3]); // C4
        data_cell8.push_back(t[2]); // C3
        data_cell8.push_back(t[1]); // C2
        data_cell8.push_back(t[0]); // C1

        // Program this device's mask/timers
        setCurrentDeviceAddress(dev);
        (void)request(measComm::enReqType::SINGLE_WRITE, m_u8CurrentDeviceAddress, CB_CELL16_CTRL, data_cell16);
        (void)request(measComm::enReqType::SINGLE_WRITE, m_u8CurrentDeviceAddress, CB_CELL8_CTRL, data_cell8);

        // Start CB engine only if at least one cell has a non-zero timer on this device
        if (anyActive)
        {
            start_CB(true);
            anyActiveGlobal = true;
        }

        // Log mask changes per device (compact to avoid spam)
        if (anyActive != s_lastActive[dev] || mask != s_lastMask[dev])
        {
            s_lastActive[dev] = anyActive;
            s_lastMask[dev] = mask;

            char cells[80] = {0};
            int pos = 0;
            for (int c = 0; c < NoOfCELL_POSSIBLE_ON_CHIP && pos < (int)sizeof(cells) - 6; ++c)
            {
                if (mask & (1u << c))
                {
                    pos += snprintf(cells + pos, sizeof(cells) - pos, "%sC%d", (pos > 0) ? "," : "", c + 1);
                }
            }

            interfaceComm.printToInterface("[BAL-DEV] Dev=%u mask=0x%04X cells=%s active=%u dV=%umV\r\n",
                                           (unsigned)dev,
                                           (unsigned)mask,
                                           (pos > 0) ? cells : "-",
                                           (unsigned)anyActive,
                                           (unsigned)m_u16CellVoltageMismatch);
        }
    }

    // === NEW: Balancing Activity Logging ===
    static bool s_balanceWasActive = false;
    if (anyActiveGlobal && !s_balanceWasActive)
    {
        interfaceComm.printToInterface("[BAL] Start: cells bleeding (dV=%umV)\r\n",
                                       (unsigned)m_u16CellVoltageMismatch);
        s_balanceWasActive = true;
    }
    else if (!anyActiveGlobal && s_balanceWasActive)
    {
        interfaceComm.printToInterface("[BAL] Stop: all cells within %umV\r\n",
                                       (unsigned)m_u16CellVoltageMismatch);
        s_balanceWasActive = false;
    }

    // bool _packBalanced = true;

    // vector<uint8_t> dataToWrite;
    // measComm::enErrorCodes error = measComm::enErrorCodes::SUCCESS;

    // // reset all timers
    // resetAllBalanceTime();

    // // check for imbalance
    // for (int i = 0; i < NoOfCELL_POSSIBLE_ON_CHIP; ++i)
    // {
    //     if (cellBalancingValues[i].bCellBleedActive)
    //     {
    //         //   InterfaceCommHandler::getInstance()->printToInterface("--- Inverter responded correctly to charge stop command ---\r\n");

    //         InterfaceCommHandler::getInstance()->printToInterface(" //#|S| Device %d ; cell%d needs balance: %d \r\n", currentDeviceAddress, i + 1, VoltageDifference);

    //         // m_log.serialPrint("//#|S| Device %d ; cell%d needs balance: %d\n", currentDeviceAddress, i+1, VoltageDifference);
    //         _packBalanced = false;

    //         if (m_u16CellVoltageMismatch >= 40)
    //         {
    //             dataToWrite.push_back((uint8_t)(measSettings::enBalancingTimer::T_10M));
    //         }
    //         else if (m_u16CellVoltageMismatch >= 30 && m_u16CellVoltageMismatch < 40)
    //         {
    //             dataToWrite.push_back((uint8_t)(measSettings::enBalancingTimer::T_300S)); // consumes 10mv with 50% duty
    //         }
    //         else if (m_u16CellVoltageMismatch >= 20 && m_u16CellVoltageMismatch < 30)
    //         {
    //             dataToWrite.push_back((uint8_t)(measSettings::enBalancingTimer::T_60S));
    //         }
    //         else if (m_u16CellVoltageMismatch >= 10 && m_u16CellVoltageMismatch < 20)
    //         {
    //             dataToWrite.push_back((uint8_t)(measSettings::enBalancingTimer::T_30S));
    //         }
    //         else if (m_u16CellVoltageMismatch < 10)
    //         {
    //             dataToWrite.push_back((uint8_t)(measSettings::enBalancingTimer::T_10S));
    //         }
    //     }
    //     else
    //     {
    //         dataToWrite.push_back((uint8_t)(measSettings::enBalancingTimer::T_NONE));
    //     }
    // }

    // // Reverse the vector to align with register map, this way we can send all 8 values using only one command
    // reverse(dataToWrite.begin(), dataToWrite.end());

    // vector<uint8_t> dataToWrite1(dataToWrite.begin(), dataToWrite.begin() + 8);
    // vector<uint8_t> dataToWrite2(dataToWrite.begin() + 8, dataToWrite.end());

    // error = request(measComm::enReqType::SINGLE_WRITE, m_u8CurrentDeviceAddress, CB_CELL16_CTRL, dataToWrite1);

    // error = request(measComm::enReqType::SINGLE_WRITE, m_u8CurrentDeviceAddress, CB_CELL8_CTRL, dataToWrite2);

    // _packBalanced = (m_enBalanceState) ? (BALANCED) : (IMBALANCED);

    // start_CB(true);
}

// ########################################################################################################################################

void BQ79616Handler::sendBalancingValues(st_cellMonitorCells *_cellVoltagesIndividual, uint16_t _cellMisMatch)
{
    memcpy(cellBalancingValues, _cellVoltagesIndividual, sizeof(cellBalancingValues));
    m_u16CellVoltageMismatch = _cellMisMatch;
}

// ########################################################################################################################################

uint8_t BQ79616Handler::getPartID()
{
    return m_u8PartId;
}

// ########################################################################################################################################

void BQ79616Handler::getHardOverandUnderVoltageFlags(uint16_t *underVoltageFlags, uint16_t *overVoltageFlags)
{
    if (getInternalError() == OVUV_FAULT)
    {
        *overVoltageFlags = (overVoltageGroup1Register.all << 8) | overVoltageGroup2Register.all;
        *underVoltageFlags = (underVoltageGroup1Register.all << 8) | underVoltageGroup2Register.all;
    }
}

// ########################################################################################################################################

void BQ79616Handler::getHardOverTemperatureFlags(uint16_t *overTemperatureFlags)
{
    if (getInternalError() == OTUT_FAULT)
    {
        if (overTemperatureRegister.all)
            *overTemperatureFlags = overTemperatureRegister.all;
    }
}

// ########################################################################################################################################

vector<enCellBalanceTimeIsUp> BQ79616Handler::getBalanceTimeUp(void) const
{
    return m_enVecCellTimeup;
}

// ########################################################################################################################################

enBqErrorType BQ79616Handler::getInternalError(void) const
{
    return m_u8ErrorType;
}

// ########################################################################################################################################

vector<vector<uint8_t>> BQ79616Handler::getAllInternalErrors(void) const
{
    return errorCodes;
}

// ########################################################################################################################################

bool BQ79616Handler::faultDetected(void)
{
    static bool _bFaultDetected = false;
    _bFaultDetected = m_faultPin.read();
    return !(_bFaultDetected); // Active Low
}

// #######################################################// System configuration and Settings // #######################################################//

/*
Gives addresses to all measurement devices
- reverseDirection : false = address in straight direction
-                  : true = address in reverse direction for ring architecture
*/
bool BQ79616Handler::autoAddress(bool reverseDirection)
{
    vector<uint8_t> dataToWrite;
    vector<uint8_t> numberOfBytes;
    vector<uint8_t> commandFrame;

    uint16_t AddressRegister = DIR0_ADDR;   // Register for normal direction
    vector<uint8_t> CONTROL1_Data = {0x01}; // Enable auto-address in normal direction

    measComm::enErrorCodes error = measComm::enErrorCodes::SUCCESS;

    if (reverseDirection == true)
    {
        AddressRegister = DIR1_ADDR; // Register for reverse direction
        CONTROL1_Data = {0x81};      // Enable auto-address in reverse direction
    }

    // Synchronize daisy chain DLL
    interfaceComm.printToInterface("[BQ_ADDR] Sync DLL...\r\n");
    dataToWrite = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, OTP_ECC_TEST, dataToWrite);

    // Enable auto-addressing
    interfaceComm.printToInterface("[BQ_ADDR] Enable auto-addr (dir=%d)...\r\n", reverseDirection);
    error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, CONTROL1, CONTROL1_Data);

    // Loop through all devices and set consecutive addresses
    for (uint8_t i = 0; i < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; i++)
    {
        // Data for setting device address
        dataToWrite = {i};
        error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, AddressRegister, dataToWrite);
        interfaceComm.printToInterface("[BQ_ADDR] Set Addr %d -> Reg 0x%04X (Res=%d)\r\n", i, AddressRegister, error);
    }

    if (NoOfCELL_MONITORS_POSSIBLE_ON_BMS == 1)
    {
        // Base device and top of stack
        dataToWrite = {0x01};
        error = request(measComm::enReqType::SINGLE_WRITE, 0x00, COMM_CTRL, dataToWrite);
    }
    else
    {
        // Set all devices as a stack
        dataToWrite = {0x02};
        error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, COMM_CTRL, dataToWrite);

        // Set device with address 0 as a base device
        dataToWrite = {0x00};
        error = request(measComm::enReqType::SINGLE_WRITE, 0x00, COMM_CTRL, dataToWrite);

        // Set device with highest address as Top of Stack
        dataToWrite = {0x03};
        error = request(measComm::enReqType::SINGLE_WRITE, (NoOfCELL_MONITORS_POSSIBLE_ON_BMS - 1), COMM_CTRL, dataToWrite);

        if (!reverseDirection)
        {
            // Synchronize daisy chain DLL again
            numberOfBytes = {1}; // read one byte
            error = request(measComm::enReqType::BROADCAST_READ, 0x00, OTP_ECC_TEST, numberOfBytes);
            ThisThread::sleep_for(1ms);
        }
    }
    
    interfaceComm.printToInterface("[BQ_ADDR] Address seq complete. Verifying...\r\n");
    
    // DELAY: Give BQ chips time to process the stack configuration before we start hammering them with reads
    ThisThread::sleep_for(50ms);
    
    // Verify addresses by reading Part ID from each expected device
    bool verificationSuccess = true;
    for(uint8_t i = 0; i < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; i++) {
        numberOfBytes = {1};
        measComm::enErrorCodes verifyErr = measComm::enErrorCodes::READ_ERROR;
        
        // Retry verification up to 3 times per device
        for(int retry=0; retry<3; retry++) {
            verifyErr = request(measComm::enReqType::SINGLE_READ, i, PARTID, numberOfBytes);
            if(verifyErr == measComm::enErrorCodes::SUCCESS) break;
            ThisThread::sleep_for(5ms);
        }

        if (verifyErr == measComm::enErrorCodes::SUCCESS) {
             interfaceComm.printToInterface("[BQ_ADDR] Verify Dev %d: OK\r\n", i);
        } else {
            interfaceComm.printToInterface("[BQ_ADDR] WARNING: Verify Dev %d FAIL (ignoring)\r\n", i);
            // verificationSuccess = false; // Soft-disable strict check
        }
    }

    if (!verificationSuccess) {
        // return false; // Soft-disable strict check
        interfaceComm.printToInterface("[BQ_ADDR] Proceeding despite verification warning.\r\n");
    }

    // mask all faults
    // mask all faults
    maskAllOfFaults();

    // clear all faults
    resetAllFaults();

    // convert communication direction back to normal after addressing in reverse direction
    if (reverseDirection)
    {
        set_normalCommDirection();
    }

    return true;
}

// ########################################################################################################################################

void BQ79616Handler::bqWriteSysConfig(st_bqConfigStructTypedef *bqConfig)
{
    // Retry auto-addressing up to 3 times
    uint8_t addressAttempts = 0;
    bool addressSuccess = false;
    
    while (addressAttempts < 3 && !addressSuccess)
    {
        addressAttempts++;
        if (addressAttempts > 1) {
            interfaceComm.printToInterface("[BQ_INIT] Auto-address retry %d/3...\r\n", addressAttempts);
            ThisThread::sleep_for(100ms);
        }
        
        addressSuccess = autoAddress(false);
        
        if (!addressSuccess) {
            interfaceComm.printToInterface("[BQ_INIT] Auto-address failed attempt %d\r\n", addressAttempts);
        }
    }
    
    if (!addressSuccess) {
        interfaceComm.printToInterface("[BQ_INIT] CRITICAL: Auto-address failed after 3 attempts.\r\n");
        m_sdCard.logEvent("BQ_INIT_FAIL");
    } else {
        interfaceComm.printToInterface("[BQ_INIT] Auto-address success on attempt %d\r\n", addressAttempts);
    }
    
    ThisThread::sleep_for(5ms);

    // for(uint8_t i = 0 ; i < NUMBER_OF_DEVICES ; i ++)
    // {
    //     setCurrentDeviceAddress(i);
    //     ThisThread::sleep_for(1ms);
    //     UpdateDeviceAddress();
    // }

    setEnableTSREF();

    setCellsPerDevice(bqConfig->numberOfCells);
    setOverVoltageThreshold(bqConfig->OVThreshold);
    setUnderVoltageThreshold(bqConfig->UVThreshold);
    setOverTemperatureThreshold(bqConfig->OTThreshold);
    setUnderTemperatureThreshold(bqConfig->UTThreshold);
    for (uint8_t i = 1; i < (NoOfTEMP_ACTIVE + 1); i++)
    {
        GPIO_SetAll(measSettings::enGpioConfig::ADC_OTUT_Input);
    }
    setMainADCMode(bqConfig->mainAdcMode);
    setVoltageProtectorMode(bqConfig->protectionOVUVMode);
    setTemperatureProtectorMode(bqConfig->protectionOTUTMode);
    setBalanceDutyCycle(bqConfig->balanceDutyCycle);
    maskAllOfFaults();
    resetAllFaults();
}

// ########################################################################################################################################

void BQ79616Handler::
    setCurrentDeviceAddress(uint8_t _currentDeviceAddress)
{
    m_u8CurrentDeviceAddress = _currentDeviceAddress;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setTwoStopBits()
{
    vector<uint8_t> dataToWrite = {0x5C};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_WRITE, 0x00, DEV_CONF, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setMainADCMode(enMainAdcMode value)
{
    measControl::unADC_CTRL1 myADC_CTRL1;
    myADC_CTRL1.bits.MAIN_MODE = static_cast<uint8_t>(value); // Set main ADC mode
    myADC_CTRL1.bits.MAIN_GO = 1;                             // Start main ADC
    myADC_CTRL1.bits.LPF_VCELL_EN = 1;
    myADC_CTRL1.bits.LPF_BB_EN = 1;

    vector<uint8_t> dataToWrite = {myADC_CTRL1.all};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, ADC_CTRL1, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setCellsPerDevice(enNoOFCells noOfCells)
{
    static uint8_t _cellsPerDevice;
    _cellsPerDevice = static_cast<uint8_t>(noOfCells);

    vector<uint8_t> dataToWrite = {static_cast<uint8_t>(noOfCells)};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, ACTIVE_CELL, dataToWrite);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setOverVoltageThreshold(enOverVoltageThreshold overVoltageThreshold)
{
    m_u8overVoltageThreshold = static_cast<uint8_t>(overVoltageThreshold);

    vector<uint8_t> dataToWrite = {static_cast<uint8_t>(overVoltageThreshold)};

    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, OV_THRESH, dataToWrite);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setUnderVoltageThreshold(enUnderVoltageThreshold underVoltageThreshold)
{
    m_u8underVoltageThreshold = static_cast<uint8_t>(underVoltageThreshold);

    vector<uint8_t> dataToWrite = {static_cast<uint8_t>(underVoltageThreshold)};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, UV_THRESH, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setOverTemperatureThreshold(enOverTemperatureThreshold overTemperatureThreshold)
{
    m_u8overTemperatureThreshold = static_cast<uint8_t>(overTemperatureThreshold);
    // combined threshold value of UT and OT
    uint8_t Threshold = m_u8underTemperatureThreshold | m_u8overTemperatureThreshold;

    // Set Over-Temperature Threshold values
    vector<uint8_t> dataToWrite = {Threshold};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, OTUT_THRESH, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setUnderTemperatureThreshold(enUnderTemperatureThreshold underTemperatureThreshold)
{
    m_u8underTemperatureThreshold = static_cast<uint8_t>(underTemperatureThreshold);
    // combined threshold value of UT and OT
    uint8_t Threshold = m_u8underTemperatureThreshold | m_u8overTemperatureThreshold;

    // Set Over-Temperature Threshold values
    vector<uint8_t> dataToWrite = {Threshold};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, OTUT_THRESH, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setEnableTSREF(void)
{
    // Enable TSREF
    vector<uint8_t> dataToWrite = {0x01};

    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, CONTROL2, dataToWrite);
    ThisThread::sleep_for(std::chrono::milliseconds(6)); // Wait for 6ms for TSREF to fully enable

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::GPIO_SetAll(measSettings::enGpioConfig gpio_config)
{
    uint8_t GPIO_All = static_cast<uint8_t>(gpio_config) | (static_cast<uint8_t>(gpio_config) << 3);

    vector<uint8_t> dataToWrite = {GPIO_All, GPIO_All, GPIO_All, GPIO_All};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, GPIO_CONF1, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setVoltageProtectorMode(enProtectionMode mode)
{
    measControl::unOVUV_CTRL _OVUV_CTRL;
    _OVUV_CTRL.bits.OVUV_MODE = static_cast<uint8_t>(mode); // Set OVUV mode
    _OVUV_CTRL.bits.OVUV_GO = 1;                            // Start OVUV protector

    vector<uint8_t> dataToWrite = {_OVUV_CTRL.all};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, OVUV_CTRL, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setTemperatureProtectorMode(enProtectionMode mode)
{
    measControl::unOTUT_CTRL _OTUT_CTRL;
    _OTUT_CTRL.bits.OTUT_MODE = static_cast<uint8_t>(mode); // Set OTUT mode
    _OTUT_CTRL.bits.OTUT_GO = 1;                            // Start OTUT

    vector<uint8_t> dataToWrite = {_OTUT_CTRL.all};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, OTUT_CTRL, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::maskAllOfFaults()
{
    vector<uint8_t> dataToWrite = {kFaultMask1, kFaultMask2};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, FAULT_MSK1, dataToWrite);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::resetAllFaults()
{
    vector<uint8_t> datatowrite = {0xFF, 0xFF};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, FAULT_RST1, datatowrite);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::resetFaults()
{
    vector<uint8_t> datatowrite = {0xFF, 0xFF};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_WRITE, m_u8CurrentDeviceAddress, FAULT_RST1, datatowrite);
    return error;
}

// ########################################################################################################################################

void BQ79616Handler::setBalanceActive(bool _value)
{
#if DISABLE_BALANCING
    _value = false;
#endif
    m_bIsActiveBalance = _value;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setBalanceTime(measSettings::enBalancingTimer time)
{
    // uint16_t timerRegister;

    // vector<uint8_t> dataToWrite = {static_cast<uint8_t>(time)};
    // measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_WRITE, 0x00, timerRegister, dataToWrite);

    // return error;

    // Configure the global balance timer control.
    // Single-byte write, value is measSettings::enBalancingTimer.
    constexpr uint16_t timerRegister = MB_TIMER_CTRL; // 0x329

    std::vector<uint8_t> dataToWrite = {
        static_cast<uint8_t>(time)};
    return request(
        measComm::enReqType::SINGLE_WRITE,
        0x00,
        timerRegister,
        dataToWrite);
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::resetAllBalanceTime()
{
    measComm::enErrorCodes error = measComm::enErrorCodes::SUCCESS;

    // reset balance timers for all cells
    vector<uint8_t> dataToWrite = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 2; i++)
    {
        measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, m_u8CurrentDeviceAddress, CB_CELL16_CTRL + (i * 8), dataToWrite);
        if (error != measComm::enErrorCodes::SUCCESS)
            return error;
    }

    return error;
}

// ########################################################################################################################################

void BQ79616Handler::setAllowedForResetInternalError(void)
{
    m_bIsAllowedResetInternalError = true;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::setBalanceDutyCycle(enBalanceDutyCycle value)
{
    m_u8balanceDutyCycle = static_cast<uint8_t>(value);

    measControl::unBAL_CTRL1 myBAL_CTRL1;
    myBAL_CTRL1.bits.DUTY = static_cast<uint8_t>(value);

    // Set duty cycle
    vector<uint8_t> dataToWrite = {myBAL_CTRL1.all};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, BAL_CTRL1, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::start_CB(bool stopOnFault)
{
    measControl::unBAL_CTRL2 myBAL_CTRL2;
    // initilize register value
    myBAL_CTRL2.all = 0;
    if (stopOnFault)
    {
        myBAL_CTRL2.bits.FLTSTOP_EN = 1; // Enable stop on fault
    }
    myBAL_CTRL2.bits.AUTO_BAL = 1; // Set to auto-balancing
    myBAL_CTRL2.bits.BAL_GO = 1;   // Start balancing
    myBAL_CTRL2.bits.OTCB_EN = 1;

    vector<uint8_t> dataToWrite = {myBAL_CTRL2.all};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_WRITE, m_u8CurrentDeviceAddress, BAL_CTRL2, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::set_normalCommDirection()
{
    vector<uint8_t> dataToWrite = {0x00};

    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_WRITE, 0x00, CONTROL1, dataToWrite);
    if (error != measComm::enErrorCodes::SUCCESS)
        return error;
    ThisThread::sleep_for(5ms);

    error = request(measComm::enReqType::R_BROADCAST_WRITE, 0x00, CONTROL1, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::set_reverseCommDirection()
{
    vector<uint8_t> dataToWrite = {0x80};

    // set base device to reverse direction
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_WRITE, 0x00, CONTROL1, dataToWrite);
    if (error != measComm::enErrorCodes::SUCCESS)
        return error;
    // HAL_Delay(5);
    ThisThread::sleep_for(5ms);

    // set stack devices to reverse direction
    error = request(measComm::enReqType::R_BROADCAST_WRITE, 0x00, CONTROL1, dataToWrite);

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::pause_CB()
{
    // initilize register value
    measControl::unBAL_CTRL2 myBAL_CTRL2;
    myBAL_CTRL2.all = 0;
    myBAL_CTRL2.bits.CB_PAUSE = 1;

    vector<uint8_t> dataToWrite = {myBAL_CTRL2.all};
    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, BAL_CTRL2, dataToWrite);

    return error;
}

// ########################################################################################################################################

void BQ79616Handler::resetInternalError(void)
{
    m_u8ErrorType = NO_ERROR;
}

// #######################################################// Power Controls // #######################################################//

measComm::enErrorCodes BQ79616Handler::sleep(void)
{
    measControl::unControl1 myControl1;
    myControl1.bits.GOTO_SLEEP = 1;
    vector<uint8_t> dataToWrite = {myControl1.all};

    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, CONTROL1, dataToWrite);

    if (error == measComm::enErrorCodes::SUCCESS)
    {
        m_bSleepChipFlag = true;
    }

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::sleepToActive(void)
{
    measControl::unControl1 myControl1;
    myControl1.bits.SEND_SLPTOACT = 1;
    vector<uint8_t> dataToWrite = {myControl1.all};

    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, CONTROL1, dataToWrite);
    ThisThread::sleep_for(10ms);

    if (error == measComm::enErrorCodes::SUCCESS)
    {
        m_bSleepChipFlag = false;
        m_bShutdownChipFlag = false;
    }

    return error;
}

// ########################################################################################################################################

void BQ79616Handler::shutdown(void)
{

    measComm::enErrorCodes error = measComm::enErrorCodes::SUCCESS;

    measControl::unControl1 myControl1;
    myControl1.bits.GOTO_SHUTDOWN = 1;

    vector<uint8_t> dataToWrite = {myControl1.all};
    error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, CONTROL1, dataToWrite);

    if (error == measComm::enErrorCodes::SUCCESS)
    {
        m_bShutdownChipFlag = true;
    }
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::reset(void)
{
    measControl::unControl2 myControl2;
    myControl2.bits.SEND_HW_RESET = 1;
    vector<uint8_t> dataToWrite = {myControl2.all};

    measComm::enErrorCodes error = request(measComm::enReqType::BROADCAST_WRITE, 0x00, CONTROL2, dataToWrite);

    if (error == measComm::enErrorCodes::SUCCESS)
    {
        m_bShutdownChipFlag = false;
        m_bSleepChipFlag = false;
    }

    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::UpdatePartID(void)
{
    vector<uint8_t> NumberOfRegisters = {1};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, PARTID, NumberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::UpdateDeviceAddress(void)
{
    vector<uint8_t> NumberOfRegisters = {1};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, DIR0_ADDR, NumberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateVoltages(void)
{
    vector<uint8_t> NumberOfRegisters = {32};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, VCELL16_HI, NumberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateBusbar(void)
{
    vector<uint8_t> NumberOfRegisters = {2};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, 0, BUSBAR_HI, NumberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateTSRef(void)
{
    vector<uint8_t> NumberOfRegisters = {2};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, TSREF_HI, NumberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateTemperatures(void)
{
    measComm::enErrorCodes error = measComm::enErrorCodes::SUCCESS;

    if (TSREF_UPDATED_FLAG)
    {
        TSREF_UPDATED_FLAG = false;

        vector<uint8_t> NumberOfRegisters = {16};
        error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, GPIO1_HI, NumberOfRegisters);
    }
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateBalanceState()
{
    vector<uint8_t> numberOfRegisters = {1};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, BAL_STAT, numberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateCellBalanceTimeUp()
{
    vector<uint8_t> numberOfRegisters = {2};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, CB_COMPLETE1, numberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateErrorSummary(void)
{
    vector<uint8_t> numberOfRegisters = {1};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, FAULT_SUMMARY, numberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updatePWRFaults(void)
{
    vector<uint8_t> numberOfRegisters{3};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, FAULT_PWR1, numberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateSystemFaults(void)
{
    vector<uint8_t> numberOfRegisters = {1};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, FAULT_SYS, numberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateOVUVFaults(void)
{
    vector<uint8_t> numberOfRegisters = {4};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, FAULT_OV1, numberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateOTUTFaults(void)
{
    vector<uint8_t> numberOfRegisters = {2};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, FAULT_OT, numberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateCommFaults(void)
{
    vector<uint8_t> numberOfRegisters = {3};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, FAULT_COMM1, numberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateComm1SubFaults(void)
{
    vector<uint8_t> numberOfRegisters = {2};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, DEBUG_UART_RC, numberOfRegisters);
    return error;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::updateComm2SubFaults(void)
{
    vector<uint8_t> numberOfRegisters = {6};
    measComm::enErrorCodes error = request(measComm::enReqType::SINGLE_READ, m_u8CurrentDeviceAddress, DEBUG_COMH_BIT, numberOfRegisters);
    return error;
}

// ########################################################################################################################################

void BQ79616Handler::processPartID(void)
{
    // parse data
    meas::_register parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    // fill data in global variable
    m_u8PartId = parseObject.fields.data;

    // m_serial.serialPrint("Device %d ID = %d\n\r", parseObject.fields.deviceAddress, m_u8PartId);
}

// ########################################################################################################################################

void BQ79616Handler::processDeviceAddress(void)
{
    // parse data
    meas::_register parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    // fill data in global variable
    m_u8deviceAddress = parseObject.fields.data;

    // if (m_u8CurrentDeviceAddress == m_u8deviceAddress)
    // {
    //     m_serial.serialPrint("Device %d Addressed successfully = %d\n\r", parseObject.fields.deviceAddress);
    // }

    // m_serial.serialPrint("Device %d Address = %d\n\r", parseObject.fields.deviceAddress, m_u8deviceAddress);
}

// ########################################################################################################################################

double meas::voltages::calculate(uint16_t cellValue)
{
    // Swap bytes
    uint16_t swappedCellValue = (cellValue >> 8) | (cellValue << 8);
    return (swappedCellValue * mainADCResolution) / 1000000.0; // Convert to volts
}

// ########################################################################################################################################

double meas::temperatures::calculate(uint16_t gpioValue, uint16_t TSREF_VALUE)
{
    // Swap temperature bytes
    uint16_t swappedCellValue = (gpioValue >> 8) | (gpioValue << 8);

    double gpioVoltage = swappedCellValue * gpioResolution / 1000000.0;
    double TSREFVoltage = TSREF_VALUE * TSREFResolution / 1000000.0;

    constexpr float kEps = 1e-6f;
    const float denom = TSREFVoltage - gpioVoltage;
    if (fabsf(denom) < kEps)
    {
        return NAN;
    } // or mark sample invalid

    // convert to resistance
    const float rntc = Rref_VALUE * (gpioVoltage / denom);
    if (rntc <= 0.0f)
    {
        return NAN;
    }

    // double rntc = Rref_VALUE * (gpioVoltage / (TSREFVoltage - gpioVoltage));

    // Convert resistor value to temperature
    double temperatureK = 1 / (1 / To + (1 / Beta) * logf(rntc / Ro));

    // Convert Kelvin to Celsius
    double temperatureC = temperatureK - 273.15;

    return temperatureC;
}

// ########################################################################################################################################

void BQ79616Handler::processVoltages(void)
{
    // parse data
    meas::voltages parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    // NEW: validate device address from the frame header
    const uint8_t dev = parseObject.fields.deviceAddress;
    if (!validDeviceAddr(dev))
    {
        return;
    }

    // Safe base offset for this device
    const size_t base = static_cast<size_t>(dev) * NoOfCELL_POSSIBLE_ON_CHIP;

    // (modulePointer * NoOfCELL_POSSIBLE_ON_CHIP) + cellPointer
    for (uint8_t cellPointer = 0; cellPointer < NoOfCELL_POSSIBLE_ON_CHIP; cellPointer++)
    {
        const uint16_t mV = (uint16_t)(parseObject.calculate(parseObject.fields.cellVoltages[(NoOfCELL_POSSIBLE_ON_CHIP - 1) - cellPointer]) * 1000.0f);

        cellVoltagesBQIndividual[base + cellPointer] = mV;
    }

    if (dev == (NoOfCELL_MONITORS_POSSIBLE_ON_BMS - 1))
    {
        m_bAllDataSavedEnableFlag = true;
    }

    // // parse data
    // meas::voltages parseObject;
    // memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    // // (modulePointer * NoOfCELL_POSSIBLE_ON_CHIP) + cellPointer
    // for (uint8_t cellPointer = 0; cellPointer < NoOfCELL_POSSIBLE_ON_CHIP; cellPointer++)
    // {
    //     cellVoltagesBQIndividual[(parseObject.fields.deviceAddress * NoOfCELL_POSSIBLE_ON_CHIP) + cellPointer] = (uint16_t)(parseObject.calculate(parseObject.fields.cellVoltages[(NoOfCELL_POSSIBLE_ON_CHIP - 1) - cellPointer]) * 1000.0);
    // }

    // if (parseObject.fields.deviceAddress == (NoOfCELL_MONITORS_POSSIBLE_ON_BMS - 1))
    // {
    //     m_bAllDataSavedEnableFlag = true;
    // }
}

// ########################################################################################################################################

void BQ79616Handler::processBusbar(void)
{
    // ... (parsing logic remains the same) ...
    meas::busBar parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    uint16_t swappedValue = (parseObject.fields.busBar >> 8) | (parseObject.fields.busBar << 8);
    m_u16ModuleBusbarVal = swappedValue;

    float calculated_current_A = 0.0f;

    if (swappedValue > 32768)
    {
        // CHARGING: Value is high, indicating negative current flow
        calculated_current_A = -1.0f * (((65535 - swappedValue) * 0.0204761905f) + 1.93095238f);
    }
    else
    {
        // DISCHARGING or IDLE: Value is low, indicating positive current flow
        calculated_current_A = ((swappedValue * 0.0204761905f) + 1.93095238f);
    }

    // Apply the 2A (2000mA) offset correction
    calculated_current_A -= 2.0f;

    // Store the final, correctly signed and offset current in milliamps
    m_i32ModuleCurrent = (int32_t)(calculated_current_A * 1000.0f);
    if (m_i32ModuleCurrent < 0 && m_i32ModuleCurrent >= -2000)
    {
        m_i32ModuleCurrent = 0; // If the value is between -2A and 0A, set it to 0
    }
    else if (m_i32ModuleCurrent < 0 && m_i32ModuleCurrent < -2000)
    {
        m_i32ModuleCurrent += 4000; // If the value is less than -2A, add 4A to it
    }
    const int DEAD_BAND_mA = 100; // tune if needed
    if (abs(m_i32ModuleCurrent) < DEAD_BAND_mA)
    {
        m_i32ModuleCurrent = 0;
    }

    // if (m_i32ModuleCurrent < 0 && m_i32ModuleCurrent >= -2000)
    // {
    //     m_i32ModuleCurrent = 0; // If the value is between -2A and 0A, set it to 0
    // }
    // else if (m_i32ModuleCurrent < 0 && m_i32ModuleCurrent < -2000)
    // {
    //     m_i32ModuleCurrent += 4000; // If the value is less than -2A, add 4A to it
    // }
}

// ########################################################################################################################################

void BQ79616Handler::processTSRef(void)
{
    // parse data
    meas::_TSREF parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    // calculate value
    uint16_t swappedValue = (parseObject.fields.data >> 8) | (parseObject.fields.data << 8);
    m_u16tsRef = swappedValue;

    TSREF_UPDATED_FLAG = true;
    // m_serial.serialPrint("Device %d TSRef : %d\n\r", parseObject.fields.deviceAddress, m_u16tsRef);
}

// ########################################################################################################################################

void BQ79616Handler::processTemperatures(void)
{
    meas::temperatures parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    // NEW: validate device address from the frame header
    const uint8_t dev = parseObject.fields.deviceAddress;
    if (!validDeviceAddr(dev))
    {
        return;
    }

    for (uint8_t tempPointer = 0; tempPointer < NoOfTEMP_POSSIBLE_ON_CHIP; tempPointer++)
    {
        tempValuesBQIndividual[(parseObject.fields.deviceAddress * NoOfTEMP_POSSIBLE_ON_CHIP) + tempPointer] = (int32_t)(parseObject.calculate(parseObject.fields.tempValues[tempPointer], m_u16tsRef) * 1000.0);
    }

    // if(parseObject.fields.deviceAddress == (NoOfCELL_MONITORS_POSSIBLE_ON_BMS - 1))
    // {
    //     for (uint8_t i = 0; i < 4; i++)
    //     {
    //         m_serial.serialPrint("C[%d] : %d\n\r", i, tempValuesBQIndividual[i]);
    //     }
    //     m_serial.serialPrint("\n\r");
    // }
}

// ########################################################################################################################################

void BQ79616Handler::processBalanceState()
{
    meas::_register parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    // NEW: validate device address from the frame header
    const uint8_t dev = parseObject.fields.deviceAddress;
    if (!validDeviceAddr(dev))
    {
        return;
    }

    measState::unBalStat_Register dataParseObject;
    dataParseObject.all = parseObject.fields.data;

    if (dataParseObject.all != 0)
    {
        if (dataParseObject.bits.CB_DONE)
            m_enBalanceState = DONE;
        if (dataParseObject.bits.CB_RUN)
            m_enBalanceState = RUNNING;
        if (dataParseObject.bits.CB_INPAUSE)
            m_enBalanceState = PAUSED;
        if (dataParseObject.bits.OT_PAUSE_DET)
            m_enBalanceState = OT_PAUSED;
        if (dataParseObject.bits.ABORTFLT)
            m_enBalanceState = ABORTFLT;
    }
    else
        m_enBalanceState = INACTIVE;

    balanceStateIndividual[parseObject.fields.deviceAddress] = static_cast<uint8_t>(m_enBalanceState);
}

// ########################################################################################################################################

void BQ79616Handler::processCellBalanceTimeUp()
{
    // parse data
    meas::twoRegisters parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    measState::unCBComplete1_Register status;
    status.all = parseObject.fields.reg1;

    measState::unCBComplete2_Register status2;
    status2.all = parseObject.fields.reg2;

    bool CBstatus = true;

    if (status2.bits.CELL1_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL1);
    if (status2.bits.CELL2_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL2);
    if (status2.bits.CELL3_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL3);
    if (status2.bits.CELL4_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL4);
    if (status2.bits.CELL5_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL5);
    if (status2.bits.CELL6_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL6);
    if (status2.bits.CELL7_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL7);
    if (status2.bits.CELL8_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL8);

    if (status.bits.CELL9_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL9);
    if (status.bits.CELL10_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL10);
    if (status.bits.CELL11_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL11);
    if (status.bits.CELL12_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL12);
    if (status.bits.CELL13_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL13);
    if (status.bits.CELL14_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL14);
    if (status.bits.CELL15_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL15);
    if (status.bits.CELL16_DONE == 1)
        m_enVecCellTimeup.push_back(enCellBalanceTimeIsUp::TIME_UP_CELL16);
}

// ########################################################################################################################################

void BQ79616Handler::processErrorSummary(void)
{
    // parse data
    meas::_register parseObject;
    measInternalErrors::unFaultSummary_Register summary;

    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    if (!validDeviceAddr(parseObject.fields.deviceAddress))
    {
        return;
    }

    summary.all = parseObject.fields.data;

    if (summary.bits.FAULT_PWR)
        updatePWRFaults();
    if (summary.bits.FAULT_SYS_bit)
        updateSystemFaults();
    if (summary.bits.FAULT_OVUV)
        updateOVUVFaults();
    if (summary.bits.FAULT_OTUT)
        updateOTUTFaults();
    if (summary.bits.FAULT_COMM)
        updateCommFaults();

    // m_serial.serialPrint("device %d errors summary : 0x%02x\n\r", parseObject.fields.deviceAddress, parseObject.fields.data);
    // for(int i = 0 ; i< sizeof(errorCodes[parseObject.fields.deviceAddress]) ; i++)
    // {
    //     m_serial.serialPrint("%d - ", errorCodes[parseObject.fields.deviceAddress][i]);
    // }
    // m_serial.serialPrint("\n\n\r");

    resetFaults();
}

// ########################################################################################################################################

void BQ79616Handler::proccessPWRFaults(void)
{
    // parse data
    meas::threeRegisters parseObject;
    measInternalErrors::unFaultPwr1_Register data1ParseObject;
    measInternalErrors::unFaultPwr2_Register data2ParseObject;
    measInternalErrors::unFaultPwr3_Register data3ParseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    if (!validDeviceAddr(parseObject.fields.deviceAddress))
    {
        return;
    }

    data1ParseObject.all = parseObject.fields.reg1;
    data2ParseObject.all = parseObject.fields.reg2;
    data3ParseObject.all = parseObject.fields.reg3;

    if (data1ParseObject.bits.AVDD_OV)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::AVDD_OV));
    if (data1ParseObject.bits.AVDD_OSC)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::AVDD_OSC));
    if (data1ParseObject.bits.DVDD_OV)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::DVDD_OV));
    if (data1ParseObject.bits.CVDD_OV)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::CVDD_OV));
    if (data1ParseObject.bits.CVDD_UV)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::CVDD_UV));
    if (data1ParseObject.bits.REFHM_OPEN)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::REFHM_OPEN));
    if (data1ParseObject.bits.DVSS_OPEN)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::DVSS_OPEN));
    if (data1ParseObject.bits.CVSS_OPEN)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::CVSS_OPEN));

    if (data2ParseObject.bits.TSREF_OV)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::TSREF_OV));
    if (data2ParseObject.bits.TSREF_UV)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::TSREF_UV));
    if (data2ParseObject.bits.TSREF_OSC)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::TSREF_OSC));
    if (data2ParseObject.bits.NEG5V_UV)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::NEG5V_UV));
    if (data2ParseObject.bits.REFH_OSC)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::REFH_OSC));
    if (data2ParseObject.bits.PWRBIST_FAIL)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::PWRBIST_FAIL));

    if (data3ParseObject.bits.AVDDUV_DRST)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::AVDDUV_DRST));
}

// ########################################################################################################################################

void BQ79616Handler::processSystemFaults(void)
{
    meas::_register parseObject;
    measInternalErrors::unFaultSys_Register dataParseObject;

    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    if (!validDeviceAddr(parseObject.fields.deviceAddress))
    {
        return;
    }

    dataParseObject.all = parseObject.fields.data;
    if (dataParseObject.bits.TWARN)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::TWARN));
    if (dataParseObject.bits.TSHUT)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::TSHUT));
    if (dataParseObject.bits.CTS)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::CTS));
    if (dataParseObject.bits.CTL)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::CTL));
    if (dataParseObject.bits.DRST)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::DRST));
    if (dataParseObject.bits.GPIO)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::GPIO));
    if (dataParseObject.bits.LFO)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::LFO));
}

// ########################################################################################################################################

void BQ79616Handler::processOVUVFaults(void)
{
    // parse data
    meas::fourRegisters parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    if (!validDeviceAddr(parseObject.fields.deviceAddress))
    {
        return;
    }

    measInternalErrors::unFaultOv1_Register data1ParseObject;
    data1ParseObject.all = parseObject.fields.reg1;

    measInternalErrors::unFaultOv2_Register data2ParseObject;
    data2ParseObject.all = parseObject.fields.reg2;

    measInternalErrors::unFaultUv1_Register data3ParseObject;
    data3ParseObject.all = parseObject.fields.reg3;

    measInternalErrors::unFaultUv2_Register data4ParseObject;
    data4ParseObject.all = parseObject.fields.reg4;

    if (data1ParseObject.bits.OV9_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV9_DET));
    if (data1ParseObject.bits.OV10_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV10_DET));
    if (data1ParseObject.bits.OV11_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV11_DET));
    if (data1ParseObject.bits.OV12_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV12_DET));
    if (data1ParseObject.bits.OV13_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV13_DET));
    if (data1ParseObject.bits.OV14_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV14_DET));
    if (data1ParseObject.bits.OV15_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV15_DET));
    if (data1ParseObject.bits.OV16_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV16_DET));

    if (data2ParseObject.bits.OV1_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV1_DET));
    if (data2ParseObject.bits.OV2_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV2_DET));
    if (data2ParseObject.bits.OV3_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV3_DET));
    if (data2ParseObject.bits.OV4_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV4_DET));
    if (data2ParseObject.bits.OV5_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV5_DET));
    if (data2ParseObject.bits.OV6_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV6_DET));
    if (data2ParseObject.bits.OV7_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV7_DET));
    if (data2ParseObject.bits.OV8_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OV8_DET));

    if (data3ParseObject.bits.UV9_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV9_DET));
    if (data3ParseObject.bits.UV10_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV10_DET));
    if (data3ParseObject.bits.UV11_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV11_DET));
    if (data3ParseObject.bits.UV12_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV12_DET));
    if (data3ParseObject.bits.UV13_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV13_DET));
    if (data3ParseObject.bits.UV14_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV14_DET));
    if (data3ParseObject.bits.UV15_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV15_DET));
    if (data3ParseObject.bits.UV16_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV16_DET));

    if (data4ParseObject.bits.UV1_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV1_DET));
    if (data4ParseObject.bits.UV2_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV2_DET));
    if (data4ParseObject.bits.UV3_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV3_DET));
    if (data4ParseObject.bits.UV4_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV4_DET));
    if (data4ParseObject.bits.UV5_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV5_DET));
    if (data4ParseObject.bits.UV6_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV6_DET));
    if (data4ParseObject.bits.UV7_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV7_DET));
    if (data4ParseObject.bits.UV8_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UV8_DET));

    // old code was filling those objects
    // if (error == measComm::enErrorCodes::SUCCESS) {
    //     overVoltageGroup1Register.all = registerValue;
    // }
    // if (error == measComm::enErrorCodes::SUCCESS) {
    //     overVoltageGroup2Register.all = registerValue;
    // }
    // if (error == measComm::enErrorCodes::SUCCESS) {
    //     underVoltageGroup1Register.all = registerValue;
    // }
    // if (error == measComm::enErrorCodes::SUCCESS) {
    //     underVoltageGroup2Register.all = registerValue;
    // }
}

// ########################################################################################################################################

void BQ79616Handler::processOTUTFaults(void)
{
    // parse data
    meas::twoRegisters parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    if (!validDeviceAddr(parseObject.fields.deviceAddress))
    {
        return;
    }

    measInternalErrors::unFaultOt_Register data1ParseObject;
    data1ParseObject.all = parseObject.fields.reg1;

    measInternalErrors::unFaultUt_Register data2ParseObject;
    data2ParseObject.all = parseObject.fields.reg2;

    if (data1ParseObject.bits.OT1_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OT1_DET));
    if (data1ParseObject.bits.OT2_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OT2_DET));
    if (data1ParseObject.bits.OT3_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OT3_DET));
    if (data1ParseObject.bits.OT4_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OT4_DET));
    if (data1ParseObject.bits.OT5_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OT5_DET));
    if (data1ParseObject.bits.OT6_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OT6_DET));
    if (data1ParseObject.bits.OT7_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OT7_DET));
    if (data1ParseObject.bits.OT8_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::OT8_DET));

    if (data2ParseObject.bits.UT1_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UT1_DET));
    if (data2ParseObject.bits.UT2_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UT2_DET));
    if (data2ParseObject.bits.UT3_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UT3_DET));
    if (data2ParseObject.bits.UT4_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UT4_DET));
    if (data2ParseObject.bits.UT5_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UT5_DET));
    if (data2ParseObject.bits.UT6_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UT6_DET));
    if (data2ParseObject.bits.UT7_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UT7_DET));
    if (data2ParseObject.bits.UT8_DET)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UT8_DET));

    // old code was filling those objects
    // if (error == measComm::enErrorCodes::SUCCESS) {
    //     overTemperatureRegister.all = registerValue;
    // }
}

// ########################################################################################################################################

void BQ79616Handler::processCommFaults(void)
{
    // parse data
    meas::threeRegisters parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    if (!validDeviceAddr(parseObject.fields.deviceAddress))
    {
        return;
    }

    measInternalErrors::unFaultComm3_Register data1ParseObject;
    data1ParseObject.all = parseObject.fields.reg1;

    if (parseObject.fields.reg1 != 0)
        updateComm1SubFaults();
    if (parseObject.fields.reg2 != 0)
        updateComm2SubFaults();

    if (parseObject.fields.reg3 != 0)
    {
        if (data1ParseObject.bits.HB_FAST)
            errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::HB_FAST));
        if (data1ParseObject.bits.HB_FAIL)
            errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::HB_FAIL));
        if (data1ParseObject.bits.FTONE_DET)
            errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::FTONE_DET));
        if (data1ParseObject.bits.FCOMM_DET)
            errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::FCOMM_DET));
    }
}

// ########################################################################################################################################

void BQ79616Handler::processComm1SubFaults(void)
{
    meas::twoRegisters parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    if (!validDeviceAddr(parseObject.fields.deviceAddress))
    {
        return;
    }

    measInternalErrors::unDebugUartRc_Register data1ParseObject;
    data1ParseObject.all = parseObject.fields.reg1;

    measInternalErrors::unDebugUartRrTr_Register data2ParseObject;
    data2ParseObject.all = parseObject.fields.reg2;

    if (data1ParseObject.bits.RC_CRC)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UART_RC_CRC));
    if (data1ParseObject.bits.RC_UNEXP)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UART_RC_UNEXP));
    if (data1ParseObject.bits.RC_BYTE_ERR)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UART_RC_BYTE_ERR));
    if (data1ParseObject.bits.RC_SOF)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UART_RC_SOF));
    if (data1ParseObject.bits.RC_TXDIS)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UART_RC_TXDIS));
    if (data1ParseObject.bits.RC_IERR)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UART_RC_IERR));

    if (data2ParseObject.bits.RR_CRC)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UART_RR_CRC));
    if (data2ParseObject.bits.RR_BYTE_ERR)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UART_RR_BYTE_ERR));
    if (data2ParseObject.bits.RR_SOF)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UART_RR_SOF));
    if (data2ParseObject.bits.TR_WAIT)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UART_TR_WAIT));
    if (data2ParseObject.bits.TR_SOF)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::UART_RR_TR_SOF));
}

// ########################################################################################################################################

void BQ79616Handler::processComm2SubFaults(void)
{
    // parse data
    meas::sixRegisters parseObject;
    memcpy(parseObject.raw, rxBuffer, sizeof(parseObject.raw));

    if (!validDeviceAddr(parseObject.fields.deviceAddress))
    {
        return;
    }

    measInternalErrors::unDebugComh_Register data1ParseObject;
    data1ParseObject.all = parseObject.fields.reg1;

    measInternalErrors::unDebugComhRc_Register data2ParseObject;
    data2ParseObject.all = parseObject.fields.reg2;

    measInternalErrors::unDebugComhRrTr_Register data3ParseObject;
    data3ParseObject.all = parseObject.fields.reg3;

    measInternalErrors::unDebugComl_Register data4ParseObject;
    data4ParseObject.all = parseObject.fields.reg4;

    measInternalErrors::unDebugComlRc_Register data5ParseObject;
    data5ParseObject.all = parseObject.fields.reg5;

    measInternalErrors::unDebugComlRrTr_Register data6ParseObject;
    data6ParseObject.all = parseObject.fields.reg6;

    if (data1ParseObject.bits.BIT)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_BIT_ERROR));
    if (data1ParseObject.bits.SYNC1)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_SYNC1));
    if (data1ParseObject.bits.SYNC2)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_SYNC2));
    if (data1ParseObject.bits.BERR_TAG)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_BERR_TAG));
    if (data1ParseObject.bits.PERR)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_PERR));

    if (data2ParseObject.bits.RC_CRC)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RC_CRC));
    if (data2ParseObject.bits.RC_UNEXP)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RC_UNEXP));
    if (data2ParseObject.bits.RC_BYTE_ERR)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RC_BYTE_ERR));
    if (data2ParseObject.bits.RC_SOF)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RC_SOF));
    if (data2ParseObject.bits.RC_TXDIS)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RC_TXDIS));
    if (data2ParseObject.bits.RC_IERR)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RC_IERR));

    if (data3ParseObject.bits.RR_CRC)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RR_CRC));
    if (data3ParseObject.bits.RR_UNEXP)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RR_UNEXP));
    if (data3ParseObject.bits.RR_BYTE_ERR)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RR_BYTE_ERR));
    if (data3ParseObject.bits.RR_SOF)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RR_SOF));
    if (data3ParseObject.bits.RR_TXDIS)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RR_TXDIS));
    if (data3ParseObject.bits.TR_WAIT)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COMH_RR_WAIT));

    if (data4ParseObject.bits.BIT)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_BIT_ERROR));
    if (data4ParseObject.bits.SYNC1)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_SYNC1));
    if (data4ParseObject.bits.SYNC2)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_SYNC2));
    if (data4ParseObject.bits.BERR_TAG)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_BERR_TAG));
    if (data4ParseObject.bits.PERR)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_PERR));

    if (data5ParseObject.bits.RC_CRC)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RC_CRC));
    if (data5ParseObject.bits.RC_UNEXP)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RC_UNEXP));
    if (data5ParseObject.bits.RC_BYTE_ERR)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RC_BYTE_ERR));
    if (data5ParseObject.bits.RC_SOF)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RC_SOF));
    if (data5ParseObject.bits.RC_TXDIS)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RC_TXDIS));
    if (data5ParseObject.bits.RC_IERR)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RC_IERR));

    if (data6ParseObject.bits.RR_CRC)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RR_CRC));
    if (data6ParseObject.bits.RR_UNEXP)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RR_UNEXP));
    if (data6ParseObject.bits.RR_BYTE_ERR)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RR_BYTE_ERR));
    if (data6ParseObject.bits.RR_SOF)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RR_SOF));
    if (data6ParseObject.bits.RR_TXDIS)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RR_TXDIS));
    if (data6ParseObject.bits.TR_WAIT)
        errorCodes[parseObject.fields.deviceAddress].push_back(static_cast<uint8_t>(measInternalErrors::enErrorCodes::COML_RR_WAIT));
}

// #######################################################// Communication functions // #######################################################//

vector<uint8_t> BQ79616Handler::createFrame(measComm::enReqType reqType, uint8_t deviceAddr, uint16_t regAddr, const vector<uint8_t> &data)
{

    std::vector<uint8_t> commandFrame;

    // INIT byte
    uint8_t initByte = 0x80; // 1000 0000 - Frame Type = Command

    initByte |= static_cast<uint8_t>(reqType) << 4; // Set REQ_TYPE
    initByte |= ((data.size() - 1) & 0x07);         // Set DATA_SIZE
    commandFrame.push_back(initByte);

    // Device Address byte - include if required for the command type
    if (reqType == measComm::enReqType::SINGLE_READ || reqType == measComm::enReqType::SINGLE_WRITE)
    {
        commandFrame.push_back(deviceAddr & 0x3F); // Ensure only the device address bits are set
    }

    // Register Address bytes
    commandFrame.push_back(static_cast<uint8_t>(regAddr >> 8));   // MSB
    commandFrame.push_back(static_cast<uint8_t>(regAddr & 0xFF)); // LSB

    if (reqType == measComm::enReqType::SINGLE_READ || reqType == measComm::enReqType::BROADCAST_READ)
    {
        // Data bytes
        commandFrame.push_back(data[0] - 1);
    }

    else
    {
        // Data bytes
        commandFrame.insert(commandFrame.end(), data.begin(), data.end());
    }

    // Calculate the CRC on the frame
    uint16_t crc = calculateCRC(commandFrame);

    // Append the CRC in normal byte order (LSB first)
    commandFrame.push_back(static_cast<uint8_t>(crc & 0xFF)); // CRC LSB
    commandFrame.push_back(static_cast<uint8_t>(crc >> 8));   // CRC MSB

    return commandFrame;
}

// ########################################################################################################################################

uint16_t BQ79616Handler::calculateCRC(const vector<uint8_t> &data)
{
    // Start with 0xFFFF
    uint16_t crc = 0xFFFF;

    for (uint8_t byte : data)
    {
        crc ^= static_cast<uint16_t>(byte);

        for (uint8_t i = 0; i < 8; ++i)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ POLY;
            else
                crc >>= 1;
        }
    }

    return crc;
}

// ########################################################################################################################################

bool BQ79616Handler::verifyCRC(const vector<uint8_t> &responseFrame)
{
    // Calculate the CRC over the entire response, including the CRC bytes.
    uint16_t calculatedCRC = calculateCRC(responseFrame);

    // If the CRC is correct, the result should be 0x0000.
    if (calculatedCRC != 0x0000)
        return false;

    return true;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::sendFrame(const vector<uint8_t> &CommandFrame)
{
    if (m_bq79616Serial->writable())
    {
        ssize_t bytesWritten = m_bq79616Serial->write(CommandFrame.data(), CommandFrame.size());
        if (bytesWritten < 0 || static_cast<size_t>(bytesWritten) != CommandFrame.size())
        {
            return measComm::enErrorCodes::WRITE_ERROR;
        }
    }
    else
    {
        return measComm::enErrorCodes::DEVICE_NOT_WRITABLE;
    }

    return measComm::enErrorCodes::SUCCESS;
}

// ########################################################################################################################################

measComm::enErrorCodes BQ79616Handler::request(measComm::enReqType reqType, uint8_t deviceAddr, uint16_t firstRegAddr, const vector<uint8_t> &dataToWrite)
{
    // uint8_t attempts = 0;
    // measComm::enErrorCodes error;

    // // in case reqType is reading, determine number of bytes to read, first 4 bytes and write them to global variables
    // if (reqType == measComm::enReqType::SINGLE_READ)
    // {
    //     NumberOfBytes = dataToWrite[0] + 6;
    //     firstByte = static_cast<uint8_t>(dataToWrite[0] - 1);
    // }
    // else if (reqType == measComm::enReqType::BROADCAST_READ)
    // {
    //     NumberOfBytes = (dataToWrite[0] + 6) * NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
    //     firstByte = static_cast<uint8_t>(dataToWrite[0] - 1);
    // }

    // while (attempts < MAX_ATTEMPTS)
    // {
    //     measComm::enErrorCodes error = sendFrame(createFrame(reqType, deviceAddr, firstRegAddr, dataToWrite));
    //     if (reqType == measComm::enReqType::SINGLE_READ || reqType == measComm::enReqType::BROADCAST_READ)
    //     {
    //         if (error == measComm::enErrorCodes::SUCCESS && readSuccess.try_acquire_for(tryTime)) // wait until semaphore released
    //         {
    //             // read successful
    //             // m_serial.serialPrint("read successful\r\n");
    //             return error;
    //         }
    //         else
    //         {
    //             // retry
    //             attempts++;
    //             rxIndex = 0;
    //             // m_serial.serialPrint("try %d\r\n", attempts);
    //         }
    //     }
    //     else
    //         break;
    // }
    // if (reqType == measComm::enReqType::SINGLE_READ || reqType == measComm::enReqType::BROADCAST_READ)
    // {
    //     // max attempts failed
    //     error = measComm::enErrorCodes::READ_ERROR;
    //     // m_serial.serialPrint("chipset comms read error \r\n");
    // }

    // return error;
    uint8_t attempts = 0;
    measComm::enErrorCodes error = measComm::enErrorCodes::SUCCESS;

    // Track total requests (for read operations only, writes don't have responses)
    bool isReadOp = (reqType == measComm::enReqType::SINGLE_READ || reqType == measComm::enReqType::BROADCAST_READ);
    if (isReadOp)
    {
        m_debugStats.totalRequests++;
    }

    if (reqType == measComm::enReqType::SINGLE_READ)
    {
        NumberOfBytes = dataToWrite[0] + 6;
        firstByte = static_cast<uint8_t>(dataToWrite[0] - 1);
    }
    else if (reqType == measComm::enReqType::BROADCAST_READ)
    {
        NumberOfBytes = (dataToWrite[0] + 6) * NoOfCELL_MONITORS_POSSIBLE_ON_BMS;
        firstByte = static_cast<uint8_t>(dataToWrite[0] - 1);
    }

    while (attempts < MAX_ATTEMPTS)
    {
        error = sendFrame(createFrame(reqType, deviceAddr, firstRegAddr, dataToWrite));
        if (isReadOp)
        {
            if (error == measComm::enErrorCodes::SUCCESS && readSuccess.try_acquire_for(tryTime))
            {
                // Success - update stats silently
                m_debugStats.successCount++;
                m_debugStats.consecutiveFails = 0;
                return measComm::enErrorCodes::SUCCESS;
            }
            else
            {
                attempts++;
                rxIndex = 0;
            }
        }
        else
        {
            return error;
        }
    }

    // Failed after all retries - update stats and log this failure
    if (isReadOp)
    {
        m_debugStats.failCount++;
        m_debugStats.timeoutErrors++;
        m_debugStats.lastFailedReg = firstRegAddr;
        m_debugStats.lastFailedDevice = deviceAddr;
        m_debugStats.consecutiveFails++;

        // Only print if we have significant consecutive failures (e.g. wire cut/chip dead)
        // Intermittent failures (consecutive=1) are hidden to avoid spam, but tracked in [BQ_STATS]
        // if (m_debugStats.consecutiveFails >= 5 && (m_debugStats.consecutiveFails % 5) == 0)
        // {
        //     interfaceComm.printToInterface("[BQ_ERR] Dev=%d Reg=0x%04X fail#%lu (total: %lu/%lu)\r\n",
        //         deviceAddr, firstRegAddr,
        //         (unsigned long)m_debugStats.consecutiveFails,
        //         (unsigned long)m_debugStats.failCount,
        //         (unsigned long)m_debugStats.totalRequests);
        // }
    }

    return measComm::enErrorCodes::READ_ERROR;
}

// ########################################################################################################################################
void BQ79616Handler::balancing_task(void)
{
    // This task is now separate and will not block data acquisition.
    if (m_bIsActiveBalance)
    {
        balanceBattery();
    }
}
// ########################################################################################################################################
void BQ79616Handler::validateCommunication(void)
{
    // Check FAULT_COMM registers for each device to detect silent communication errors
    for(uint8_t i = 0; i < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; i++) {
        uint8_t comm1 = 0;
        uint8_t comm2 = 0;
        vector<uint8_t> data = {1}; // request 1 byte

        // Read FAULT_COMM1
        if(request(measComm::enReqType::SINGLE_READ, i, FAULT_COMM1, data) == measComm::enErrorCodes::SUCCESS) {
             comm1 = data[0];
        }

        // Read FAULT_COMM2
        data = {1}; // Reset size/request
        if(request(measComm::enReqType::SINGLE_READ, i, FAULT_COMM2, data) == measComm::enErrorCodes::SUCCESS) {
             comm2 = data[0];
        }

        if(comm1 != 0 || comm2 != 0) {
            interfaceComm.printToInterface("[BQ_FAULT] Dev %d COMM1=0x%02X COMM2=0x%02X\r\n", i, comm1, comm2);
            m_sdCard.logEvent("BQ_COMM_FAULT D%d C1:%02X C2:%02X", i, comm1, comm2);
            
            // Clear faults by writing 1s to the set bits
            if (comm1 != 0) {
                 vector<uint8_t> clearData = {comm1};
                 request(measComm::enReqType::SINGLE_WRITE, i, FAULT_COMM1, clearData);
            }
            if (comm2 != 0) {
                 vector<uint8_t> clearData = {comm2};
                 request(measComm::enReqType::SINGLE_WRITE, i, FAULT_COMM2, clearData);
            }
        }
    }
}

// ########################################################################################################################################
void BQ79616Handler::printCommStats(void)
{
    uint32_t successRate = 0;
    if (m_debugStats.totalRequests > 0)
    {
        successRate = (m_debugStats.successCount * 100) / m_debugStats.totalRequests;
    }

    interfaceComm.printToInterface("[BQ_STATS] Total:%lu OK:%lu Fail:%lu (%lu%%) Timeouts:%lu\r\n",
        (unsigned long)m_debugStats.totalRequests,
        (unsigned long)m_debugStats.successCount,
        (unsigned long)m_debugStats.failCount,
        (unsigned long)successRate,
        (unsigned long)m_debugStats.timeoutErrors);

    if (m_debugStats.failCount > 0)
    {
        interfaceComm.printToInterface("[BQ_STATS] LastFail: Dev=%d Reg=0x%04X ConsecFails=%d FAULT_PIN=%d\r\n",
            m_debugStats.lastFailedDevice,
            m_debugStats.lastFailedReg,
            m_debugStats.consecutiveFails,
            faultDetected() ? 1 : 0);
    }
}
// ########################################################################################################################################
