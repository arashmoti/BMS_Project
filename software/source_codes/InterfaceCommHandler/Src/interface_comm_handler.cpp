#include <cstdint>
#include <cstring>
#include <memory>
#include <cctype>
#include <cstdio>
#include "interface_comm_handler.h"

#include "Callback.h"
#include "Timer.h"
#include "rtos/Mutex.h"

extern rtos::Mutex g_packInfoMutex;

// #############################################################################################//

InterfaceCommHandler::InterfaceCommHandler(IEeprom &_eeprom, IDatetime &_datetime, ISerial &_serial)
    : m_eeprom(_eeprom),
      m_datetime(_datetime),
      m_serial(_serial),
      m_interfaceCommBufferedSerial(TERMINAL_TX_PIN, TERMINAL_RX_PIN, SERIAL_BAUDRATE),
      m_interfaceCommParams{0},
      m_serialThreadVar(SERIAL_THREAD_PRIORITY, SERIAL_STACK_SIZE, nullptr, "InterfaceComm")
{
    // Initialize the global instance pointer and configure UART
    instance = this;
    // Make UART fast and non-blocking from the start
    m_interfaceCommBufferedSerial.set_baud(250000);
    m_interfaceCommBufferedSerial.set_blocking(false);
}

// #############################################################################################//

InterfaceCommHandler::~InterfaceCommHandler()
{
}

// #############################################################################################//

// VV-- ADD THESE TWO BLOCKS OF CODE --VV

// Initialize the static instance pointer to null
InterfaceCommHandler *InterfaceCommHandler::instance = nullptr;

// Implementation of the getInstance function
InterfaceCommHandler *InterfaceCommHandler::getInstance()
{
    return instance;
}
// ^^------------------------------------^^

void InterfaceCommHandler::init(st_generalConfig *genConfig, st_powerElecPackInfoConfig *powerElecPackInfoConfig)
{
    m_ptrGenConfig = genConfig;
    m_ptrPowerElecPackInfoConfig = powerElecPackInfoConfig;

    packageReset();
}

// #############################################################################################//

void InterfaceCommHandler::startThread()
{
    m_interfaceCommBufferedSerial.sigio(mbed::callback(this, &InterfaceCommHandler::interfaceCommRxISR));
    m_serialThreadVar.start(mbed::callback(this, &InterfaceCommHandler::interfaceCommThread));
    // Start TX draining thread at low priority so it never blocks control loops
    m_txThreadVar.start(mbed::callback(this, &InterfaceCommHandler::txThread));
    m_txThreadVar.set_priority(osPriorityLow);
}

// #############################################################################################//

void InterfaceCommHandler::lock()
{
    m_mutex.lock();
}

// #############################################################################################//

void InterfaceCommHandler::unlock()
{
    m_mutex.unlock();
}

// #############################################################################################//

bool InterfaceCommHandler::interfaceCommPrint(const char *command, ...)
{
    va_list args;
    va_start(args, command);
    bool res = vsend(command, args);
    va_end(args);
    return res;
}

// #############################################################################################//

void InterfaceCommHandler::puts(const char *data, size_t size)
{
    // lock();
    // m_interfaceCommBufferedSerial.write(data, size);
    // unlock();
    // Non-blocking: enqueue and return
    txEnqueue(reinterpret_cast<const uint8_t *>(data), size);
}

// #############################################################################################//

bool InterfaceCommHandler::vsend(const char *format, va_list args)
{
    // char m_sendBuffer[512];
    // if (vsprintf(m_sendBuffer, format, args) < 0)
    // {
    //     return false;
    // }

    // lock();
    // m_interfaceCommBufferedSerial.write(m_sendBuffer, vsprintf(m_sendBuffer, format, args));
    // unlock();

    // return true;

    char m_sendBuffer[192];
    int n = vsnprintf(m_sendBuffer, sizeof(m_sendBuffer), format, args);
    if (n <= 0)
        return false;
    if (n >= (int)sizeof(m_sendBuffer))
    {
        // Truncate with ellipsis
        m_sendBuffer[sizeof(m_sendBuffer) - 3] = '.';
        m_sendBuffer[sizeof(m_sendBuffer) - 2] = '.';
        m_sendBuffer[sizeof(m_sendBuffer) - 1] = '\0';
        n = sizeof(m_sendBuffer) - 1;
    }
    txEnqueue(reinterpret_cast<const uint8_t *>(m_sendBuffer), (size_t)n);
    return true;
}

// ######################################################################################################################

void InterfaceCommHandler::packageReset()
{
    m_interfaceCommParams.u8Index = 0;
    m_interfaceCommParams.u16HasError = 0;
    m_interfaceCommParams.u8PackageReady = 0;
    m_interfaceCommParams.u8AckReceived = 0;
    m_interfaceCommParams.u8NackReceived = 0;
    m_interfaceCommParams.u8StatusRespReceived = 0;
    m_interfaceCommParams.u8CurByte = 0;
    m_interfaceCommParams.curState = SerialArgs::INTERFACE_COMM_STATE_IDLE;

    memset((void *)m_interfaceCommParams.unInterfaceCommRX.arru8Bytes, 0, sizeof(m_interfaceCommParams.unInterfaceCommRX.arru8Bytes));
}

// ########################################################################################################################

bool InterfaceCommHandler::isCheckedPackage()
{
    static uint8_t CRC2 = 0;
    static uint8_t CRC1 = 0;
    static uint8_t CRC0 = 0;
    uint32_t u32_crc_get;
    uint32_t u32_crc_calculated;

    if (m_interfaceCommParams.unInterfaceCommRX.u8StartByte != SerialArgs::INTERFACE_COMM_BYTE_START)
    {
        m_interfaceCommParams.stInterfaceCommErrors.u8FrameStartError = 1;
    }

    u32_crc_calculated = (uint32_t)(0xFFFFFF);

    CRC2 = m_interfaceCommParams.unInterfaceCommRX.arru8Bytes[m_interfaceCommParams.u8Index - 1];
    CRC1 = m_interfaceCommParams.unInterfaceCommRX.arru8Bytes[m_interfaceCommParams.u8Index - 2];
    CRC0 = m_interfaceCommParams.unInterfaceCommRX.arru8Bytes[m_interfaceCommParams.u8Index - 3];

    u32_crc_get = (((uint32_t)CRC0 << 16) + ((uint32_t)CRC1 << 8) + ((uint32_t)CRC2));

    if (u32_crc_calculated != u32_crc_get)
    {
        m_interfaceCommParams.stInterfaceCommErrors.u8CrcError = 1;
    }

    if (m_interfaceCommParams.unInterfaceCommRX.u8PayloadLength != (m_interfaceCommParams.u8Index - 5)) // 5--> 3CRC+ 1START + 1PAYLOADLEN
    {
        m_interfaceCommParams.stInterfaceCommErrors.u8PayloadSizeError = 1;
    }

    return !(m_interfaceCommParams.u16HasError > 0);
}

// ######################################################################################################################

void InterfaceCommHandler::txAppend(uint8_t data)
{
    m_interfaceCommParams.unInterfaceCommTX.arru8Bytes[m_interfaceCommParams.u8TxIndex] = data;
    m_interfaceCommParams.u8TxIndex++;
}

// ######################################################################################################################

void InterfaceCommHandler::txByte(uint8_t data)
{
    // m_interfaceCommBufferedSerial.write((uint8_t *)&data, 1);
    // Enqueue single byte, do not block
    txEnqueue(&data, 1);
}

// ######################################################################################################################

void InterfaceCommHandler::txPackage()
{
    // for (uint8_t i = 0; i < m_interfaceCommParams.u8TxIndex; i++)
    // {

    //     switch (m_interfaceCommParams.unInterfaceCommTX.arru8Bytes[i])
    //     {
    //     // case INTERFACE_CHR_ESC:
    //     // case INTERFACE_CHR_STATUS:
    //     //  case INTERFACE_CHR_ACK:
    //     //  case INTERFACE_CHR_NACK:
    //     case SerialArgs::INTERFACE_COMM_BYTE_START:
    //         if ((m_interfaceCommParams.unInterfaceCommTX.arru8Bytes[i] == SerialArgs::INTERFACE_COMM_BYTE_START) &&
    //             (i == 0 || i == m_interfaceCommParams.u8TxIndex - 1)) // Start and end chars
    //         {
    //             txByte(m_interfaceCommParams.unInterfaceCommTX.arru8Bytes[i]); // Send without escaping
    //         }
    //         else
    //         {
    //             // transmitByte(INTERFACE_CHR_ESC); // Escape
    //             // transmitByte(m_serialMsg.un_Tx.arru8Bytes[i] ^ 0x20);
    //         }
    //         break;
    //     default:
    //         txByte(m_interfaceCommParams.unInterfaceCommTX.arru8Bytes[i]);
    //         break;
    //     }
    // }

    // m_interfaceCommParams.u8TxIndex = 0;

    // Enqueue whole frame at once (no escaping currently used)
    txEnqueue(const_cast<const uint8_t *>(m_interfaceCommParams.unInterfaceCommTX.arru8Bytes), m_interfaceCommParams.u8TxIndex);
    m_interfaceCommParams.u8TxIndex = 0; // reset builder index
}

// ######################################################################################################################

void InterfaceCommHandler::send1Byte(uint8_t msg_type, uint8_t data)
{
#if IFCOMM_HUMAN_CONSOLE
    (void)msg_type;
    (void)data;
    return; // suppress framed traffic in human-console mode
#endif
    lock(); // protect the shared TX builder
    for (uint8_t i = 0; i < SerialArgs::INTERFACE_COMM_BUFFER_SIZE; i++)
    {
        m_interfaceCommParams.unInterfaceCommTX.arru8Bytes[i] = 0;
    }

    m_interfaceCommParams.u8TxIndex = 0;

    txAppend(SerialArgs::INTERFACE_COMM_BYTE_START);
    txAppend(0);        // Payload Len, will be calculated below
    txAppend(msg_type); // Message Type (by user)

    txAppend(data);

    m_interfaceCommParams.unInterfaceCommTX.u8PayloadLength = m_interfaceCommParams.u8TxIndex - 2;
    txAppend(0xFF);
    txAppend(0xFF);
    txAppend(0xFF);
    txAppend(SerialArgs::INTERFACE_COMM_BYTE_START);

    txPackage();
    unlock();
}

// ######################################################################################################################

void InterfaceCommHandler::send2Bytes(uint8_t msg_type, uint8_t dataH, uint8_t dataL)
{
#if IFCOMM_HUMAN_CONSOLE
    (void)msg_type;
    (void)dataH;
    (void)dataL;
    return; // suppress framed traffic in human-console mode
#endif
    lock(); // protect the shared TX builder
    for (uint8_t i = 0; i < SerialArgs::INTERFACE_COMM_BUFFER_SIZE; i++)
    {
        m_interfaceCommParams.unInterfaceCommTX.arru8Bytes[i] = 0;
    }

    m_interfaceCommParams.u8TxIndex = 0;

    txAppend(SerialArgs::INTERFACE_COMM_BYTE_START);
    txAppend(0);        // Payload Len, will be calculated below
    txAppend(msg_type); // Message Type (by user)

    txAppend(dataH);
    txAppend(dataL);

    m_interfaceCommParams.unInterfaceCommTX.u8PayloadLength = m_interfaceCommParams.u8TxIndex - 2;
    txAppend(0xFF);
    txAppend(0xFF);
    txAppend(0xFF);
    txAppend(SerialArgs::INTERFACE_COMM_BYTE_START);

    txPackage();
    unlock();
}

// ######################################################################################################

void InterfaceCommHandler::processPacket(unsigned char *data, unsigned char len)
{
#if IFCOMM_HUMAN_CONSOLE
    (void)data;
    (void)len;
    return; // suppress framed traffic in human-console mode
#endif
    lock(); // protect the shared TX builder
    for (uint8_t i = 0; i < SerialArgs::INTERFACE_COMM_BUFFER_SIZE; i++)
    {
        m_interfaceCommParams.unInterfaceCommTX.arru8Bytes[i] = 0;
    }

    m_interfaceCommParams.u8TxIndex = 0;

    enInterfaceCommCmdId cmdId;

    cmdId = (enInterfaceCommCmdId)data[0];
    data++;
    len--;

    txAppend(SerialArgs::INTERFACE_COMM_BYTE_START);
    txAppend(0);     // Payload Len, will be calculated below
    txAppend(cmdId); // Message Type (by user)

    for (uint8_t i = 0; i < len; i++)
    {
        txAppend(data[i]);
    }

    m_interfaceCommParams.unInterfaceCommTX.u8PayloadLength = m_interfaceCommParams.u8TxIndex - 2;
    txAppend(0xFF);
    txAppend(0xFF);
    txAppend(0xFF);
    txAppend(SerialArgs::INTERFACE_COMM_BYTE_START);

    txPackage();
    unlock();
}

// #############################################################################################//

void InterfaceCommHandler::printToInterface(const char *format, ...)
{
#if IFCOMM_HUMAN_CONSOLE
    // Plain ASCII path for human console
    va_list vl_ascii;
    va_start(vl_ascii, format);
    vsend(format, vl_ascii); // non-blocking enqueue of formatted text
    va_end(vl_ascii);
    const char crlf_ascii[] = "\r\n";
    txEnqueue(reinterpret_cast<const uint8_t *>(crlf_ascii), sizeof(crlf_ascii) - 1);
    return;
#endif
    // Protocol: 7 bytes overhead → payload ≤ 57 when buffer size is 64
    constexpr uint8_t kOverhead = 7;
    constexpr uint8_t kMaxPayload = SerialArgs::INTERFACE_COMM_BUFFER_SIZE - kOverhead;

    // Remember last payload to suppress identical spam
    static uint8_t s_lastLen = 0;
    static char s_last[kMaxPayload] = {0};

    lock(); // protect the shared TX builder

    // Clear the TX frame builder
    for (uint8_t i = 0; i < SerialArgs::INTERFACE_COMM_BUFFER_SIZE; i++)
    {
        m_interfaceCommParams.unInterfaceCommTX.arru8Bytes[i] = 0;
    }
    m_interfaceCommParams.u8TxIndex = 0;

    // Format into a local buffer (then trim to protocol payload size)
    char buf[192];
    va_list vl;
    va_start(vl, format);
    int n = vsnprintf(buf, sizeof(buf), format, vl);
    va_end(vl);
    if (n <= 0)
    {
        unlock();
        return;
    }

    uint8_t copy_len = (uint8_t)((n > (int)kMaxPayload) ? kMaxPayload : n);

    // --- DUPLICATE SUPPRESSION ---
    if (s_lastLen == copy_len && memcmp(buf, s_last, copy_len) == 0)
    {
        // exact same line as last time → drop it
        unlock();
        return;
    }
    // store as new "last" payload
    memcpy(s_last, buf, copy_len);
    s_lastLen = copy_len;
    // -----------------------------

    // Build and enqueue one framed packet
    txAppend(SerialArgs::INTERFACE_COMM_BYTE_START);
    txAppend(0);                               // payload length (filled after payload)
    txAppend(INTERFACE_COMM_CMD_PRINT_TO_LOG); // message type

    for (uint8_t i = 0; i < copy_len; i++)
    {
        txAppend((uint8_t)buf[i]);
    }

    // payload length = all bytes after start+len
    m_interfaceCommParams.unInterfaceCommTX.u8PayloadLength = m_interfaceCommParams.u8TxIndex - 2;

    // dummy CRCs (unchanged from your code) + end
    txAppend(0xFF);
    txAppend(0xFF);
    txAppend(0xFF);
    txAppend(SerialArgs::INTERFACE_COMM_BYTE_START);

    txPackage();
    unlock();

    // // Build a safe, truncated frame and enqueue it (non-blocking).
    // // Max payload that fits in our 64-byte frame:
    // // start + len + type + payload + 3*CRC + end = 7 overhead  → payload ≤ 57
    // constexpr uint8_t kOverhead = 7;
    // constexpr uint8_t kMaxPayload = SerialArgs::INTERFACE_COMM_BUFFER_SIZE - kOverhead; // 57

    // lock(); // protect the shared TX builder
    // for (uint8_t i = 0; i < SerialArgs::INTERFACE_COMM_BUFFER_SIZE; i++)
    // {
    //     m_interfaceCommParams.unInterfaceCommTX.arru8Bytes[i] = 0;
    // }

    // m_interfaceCommParams.u8TxIndex = 0;

    // // char buf[100];
    // char buf[192];

    // va_list vl;

    // va_start(vl, format);
    // // vsprintf(buf, format, vl);
    // int n = vsnprintf(buf, sizeof(buf), format, vl);

    // va_end(vl);
    // if (n < 0)
    // {
    //     unlock();
    //     return;
    // }
    // if (n > (int)sizeof(buf))
    //     n = sizeof(buf); // defensive
    // // Trim to fit protocol frame
    // uint8_t copy_len = (uint8_t)((n > (int)kMaxPayload) ? kMaxPayload : n);

    // txAppend(SerialArgs::INTERFACE_COMM_BYTE_START);
    // txAppend(0);                               // Payload Len, will be calculated below
    // txAppend(INTERFACE_COMM_CMD_PRINT_TO_LOG); // Message Type (by user)

    // // for (uint8_t i = 0; i < strlen(buf); i++)
    // for (uint8_t i = 0; i < copy_len; i++)
    // {
    //     txAppend(buf[i]);
    // }

    // m_interfaceCommParams.unInterfaceCommTX.u8PayloadLength = m_interfaceCommParams.u8TxIndex - 2;
    // txAppend(0xFF);
    // txAppend(0xFF);
    // txAppend(0xFF);
    // txAppend(SerialArgs::INTERFACE_COMM_BYTE_START);

    // txPackage();
    // unlock();
}

// #############################################################################################//

// Function to process common message types
void InterfaceCommHandler::handleLoggingNotification(uint8_t *pDestination, uint8_t value, const char *description)
{
    if (pDestination != NULL)
    {
        g_packInfoMutex.lock();
        *pDestination = value;
        g_packInfoMutex.unlock();

        // Optional: Add logging for debugging

        m_serial.serialPrint("Updated %s to %u\r\n", description, value);
    }
}

// #############################################################################################//

static bool starts_with_ci(const char *str, const char *prefix)
{
    if (!str || !prefix)
        return false;
    while (*prefix && *str)
    {
        if (std::tolower(static_cast<unsigned char>(*str)) !=
            std::tolower(static_cast<unsigned char>(*prefix)))
        {
            return false;
        }
        ++str;
        ++prefix;
    }
    return *prefix == '\0';
}

void InterfaceCommHandler::handleAsciiCommand(const char *line)
{
    if (!line)
        return;

    while (*line == ' ' || *line == '\t')
        ++line;
    if (*line == '\0')
        return;

    if (starts_with_ci(line, "RTC"))
    {
        int year = 0;
        int month = 0;
        int day = 0;
        int hour = 0;
        int minute = 0;
        int second = 0;

        int matched = std::sscanf(line, "RTC,%d,%d,%d,%d,%d,%d",
                                  &year, &month, &day, &hour, &minute, &second);
        if (matched != 6)
        {
            matched = std::sscanf(line, "RTC %d-%d-%d %d:%d:%d",
                                  &year, &month, &day, &hour, &minute, &second);
        }

        if (matched == 6)
        {
            if (year >= 2000)
                year -= 2000;

            if (year >= 0 && year <= 99 &&
                month >= 1 && month <= 12 &&
                day >= 1 && day <= 31 &&
                hour >= 0 && hour <= 23 &&
                minute >= 0 && minute <= 59 &&
                second >= 0 && second <= 59)
            {
                RtcArgs::st_dateInfo_t dt = {};
                dt.u8Second = static_cast<uint8_t>(second);
                dt.u8Minute = static_cast<uint8_t>(minute);
                dt.u8Hour = static_cast<uint8_t>(hour);
                dt.u8Day = static_cast<uint8_t>(day);
                dt.u8Month = static_cast<uint8_t>(month);
                dt.u8Year = static_cast<uint8_t>(year);
                m_datetime.setDate(&dt);
                printToInterface("[RTC] synced to %02u-%02u-%02u %02u:%02u:%02u",
                                 dt.u8Year, dt.u8Month, dt.u8Day,
                                 dt.u8Hour, dt.u8Minute, dt.u8Second);
                return;
            }
        }

        printToInterface("[RTC] invalid format; use RTC,YYYY,MM,DD,HH,MM,SS");
        return;
    }

    if (starts_with_ci(line, "EEPROM_CLEAR"))
    {
        if (starts_with_ci(line, "EEPROM_CLEAR,CONFIRM"))
        {
            printToInterface("[EEPROM] UI reset confirmed; reloading defaults");
            m_eeprom.invalidate_eeprom_mappings();
            m_eeprom.defaultLoading();
            m_eeprom.clearSocJournal();
            ThisThread::sleep_for(250ms);
            NVIC_SystemReset();
            return;
        }

        printToInterface("[EEPROM] confirm with EEPROM_CLEAR,CONFIRM");
        return;
    }

    if (starts_with_ci(line, "HELP"))
    {
        printToInterface("[HELP] RTC,YYYY,MM,DD,HH,MM,SS | EEPROM_CLEAR,CONFIRM");
        return;
    }
}

// #############################################################################################//

void InterfaceCommHandler::interfaceCommRxISR()
{
    m_serialThreadVar.flags_set(SerialArgs::kRxSignal);
}

// #############################################################################################//

void InterfaceCommHandler::interfaceCommThread()
{
    while (true)
    {

        rtos::ThisThread::flags_wait_any(SerialArgs::kRxSignal);

#if IFCOMM_HUMAN_CONSOLE
        while (m_interfaceCommBufferedSerial.readable())
        {
            char c_chNewChar;
            m_interfaceCommBufferedSerial.read(&c_chNewChar, 1);

            static char s_lineBuf[96];
            static uint8_t s_lineLen = 0;

            if (c_chNewChar == '\r' || c_chNewChar == '\n')
            {
                if (s_lineLen > 0)
                {
                    s_lineBuf[s_lineLen] = '\0';
                    handleAsciiCommand(s_lineBuf);
                    s_lineLen = 0;
                }
                continue;
            }

            if (s_lineLen < (sizeof(s_lineBuf) - 1))
            {
                s_lineBuf[s_lineLen++] = c_chNewChar;
            }
        }
#else
        while (m_interfaceCommBufferedSerial.readable())
        {
            // 7E 01 C3 AC 31 83 7E
            char c_chNewChar;
            m_interfaceCommBufferedSerial.read(&c_chNewChar, 1);
            m_interfaceCommParams.u8CurByte = static_cast<uint8_t>(c_chNewChar);
            uint8_t m_u8AppendNewByte = 0;

            switch (m_interfaceCommParams.u8CurByte)
            {
            case SerialArgs::INTERFACE_COMM_BYTE_START:
                // Terminate/Start
                if (m_interfaceCommParams.curState == SerialArgs::INTERFACE_COMM_STATE_IDLE) // Start
                {
                    m_interfaceCommParams.curState = SerialArgs::INTERFACE_COMM_STATE_RECEIVING;
                    m_u8AppendNewByte = 1;
                }
                else if (m_interfaceCommParams.curState == SerialArgs::INTERFACE_COMM_STATE_ESCAPE)
                {
                    m_interfaceCommParams.stInterfaceCommErrors.u8FinishedWithEsc = 1;
                    m_interfaceCommParams.u8PackageReady = 1;
                }
                else // (DSL_STATE_RECEIVING?) ,Terminate the package
                {

                    m_interfaceCommParams.u8PackageReady = 1;
                }
                break;

            case SerialArgs::INTERFACE_COMM_BYTE_STATUS:
                m_interfaceCommParams.u8StatusRespReceived = 1;
                break;

            case SerialArgs::INTERFACE_COMM_BYTE_ESC:
                m_interfaceCommParams.curState = SerialArgs::INTERFACE_COMM_STATE_ESCAPE;
                break;

            default:
                if (m_interfaceCommParams.curState == SerialArgs::INTERFACE_COMM_STATE_ESCAPE)
                {
                    m_interfaceCommParams.u8CurByte ^= 0x20;
                    switch (m_interfaceCommParams.u8CurByte)
                    {
                    case SerialArgs::INTERFACE_COMM_BYTE_START:
                    case SerialArgs::INTERFACE_COMM_BYTE_STATUS:
                    case SerialArgs::INTERFACE_COMM_BYTE_ESC:
                        m_u8AppendNewByte = 1;
                        break;

                    default:
                        // Why used escape for this byte?
                        // It doesn't make any sense.
                        m_interfaceCommParams.stInterfaceCommErrors.u8EscapeError = 1;
                        break;
                    }
                }
                else
                {
                    m_u8AppendNewByte = 1;
                }

                m_interfaceCommParams.curState = (m_interfaceCommParams.u8Index > 0) ? (SerialArgs::INTERFACE_COMM_STATE_RECEIVING) : (SerialArgs::INTERFACE_COMM_STATE_IDLE);
                break;
            }

            if (m_u8AppendNewByte)
            {
                if (m_interfaceCommParams.u8Index == 0 && m_interfaceCommParams.u8CurByte != SerialArgs::INTERFACE_COMM_BYTE_START)
                {
                    // Do nothing...
                    // Wait for preamble (Start Char)
                }
                else if (m_interfaceCommParams.u8Index >= SerialArgs::INTERFACE_COMM_BUFFER_SIZE)
                {
                    m_interfaceCommParams.stInterfaceCommErrors.u8MaxSizeOverflow = 1;
                }
                else
                {
                    m_interfaceCommParams.unInterfaceCommRX.arru8Bytes[m_interfaceCommParams.u8Index] = m_interfaceCommParams.u8CurByte;
                    m_interfaceCommParams.u8Index++;
                }
            }
        }

        if (m_interfaceCommParams.u8StatusRespReceived)
        {
            m_interfaceCommParams.u8StatusRespReceived = 0;
            packageReset();
        }

        if (m_interfaceCommParams.u8PackageReady)
        {
            if (isCheckedPackage())
            {
                interfaceProcessMessage(&m_interfaceCommParams);
            }
            else
            {
            }

            packageReset();
        }
#endif
        ThisThread::sleep_for(1ms);
    }
}

// #############################################################################################//
// ---------------------------------------------------------------------------------------------
// TX draining thread (non-blocking logger)
// ---------------------------------------------------------------------------------------------
void InterfaceCommHandler::txEnqueue(const uint8_t *data, size_t len)
{
    m_txMutex.lock();
    for (size_t i = 0; i < len; ++i)
    {
        if (!m_txQueue.full())
        {
            m_txQueue.push((char)data[i]); // returns false if full, but we just checked
        }
        else
        {
            // Queue full → drop remaining bytes (never block control threads)
            break;
        }
    }
    m_txMutex.unlock();
    // core_util_critical_section_exit();
    m_txSem.release();
}

// ##############################################################################################//
void InterfaceCommHandler::txThread(void)
{
    while (true)
    {
        m_txSem.acquire(); // sleep until there is something to send
                           // Drain as much as possible without blocking higher-priority threads
        while (true)
        {
            char c;
            bool hasData = false;
            m_txMutex.lock();
            if (!m_txQueue.empty())
            {
                hasData = m_txQueue.pop(c);
            }
            m_txMutex.unlock();
            if (!hasData)
            {
                break; // nothing to send
            }
            // Non-blocking write of 1 byte; if UART not ready, retry a bit without hogging CPU
            for (int tries = 0; tries < 5; ++tries)
            {
                ssize_t w = m_interfaceCommBufferedSerial.write(&c, 1);
                if (w == 1)
                    break;                  // sent OK
                ThisThread::sleep_for(1ms); // give ISR time to drain HW FIFO
            }
        }
        // If still not empty (UART was busy), poke ourselves again
        bool hasMore = false;
        m_txMutex.lock();
        hasMore = !m_txQueue.empty();
        m_txMutex.unlock();
        if (hasMore)
        {
            m_txSem.release();
        }
    }
}

// #############################################################################################//

void InterfaceCommHandler::interfaceProcessMessage(SerialArgs::stInterfaceCommParams *interfaceCommParams)
{

    int i;
    bool messageHandled = false; // Flag to track if message was handled
    uint8_t parameterValue;      // Store the parameter value

    // Define all mappings in one place for easy maintenance
    static const stInterafaceMessageTypeMapping messageMappings[] = {
        {INTERFACE_MSG_TYPE_BUTTON_TYPE,
         &m_ptrPowerElecPackInfoConfig->u8InterfaceActiveButtonState,
         "Interface Comm Button Type"},

        {INTERFACE_MSG_TYPE_MODULE_NUMBER,
         &m_ptrPowerElecPackInfoConfig->u8ActiveModuleNumber,
         "Interface Comm Module Number"},

        {INTERFACE_MSG_TYPE_DEV_DEBUGGING,
         &m_ptrPowerElecPackInfoConfig->u8DebuggingState,
         "Interface Comm Dev Debugging"},
    };

    // Get number of elements in the array
    size_t numMappings = sizeof(messageMappings) / sizeof(messageMappings[0]);
    // Extract the parameter value once
    parameterValue = interfaceCommParams->unInterfaceCommRX.arru8Parameters[0];

    // First check if it's a standard logging notification message
    for (i = 0; i < numMappings; i++)
    {
        if ((enInterfaceMessageType)interfaceCommParams->unInterfaceCommRX.u8MessageType == messageMappings[i].interfaceMessageType)
        {
            handleLoggingNotification(messageMappings[i].pDestination,
                                      interfaceCommParams->unInterfaceCommRX.arru8Parameters[0],
                                      messageMappings[i].description);
            messageHandled = true;
            break; // Exit the loop but continue processing
        }
    }
}

// #############################################################################################//
