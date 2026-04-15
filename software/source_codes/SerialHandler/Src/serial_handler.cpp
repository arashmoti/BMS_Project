#include <cstdint>
#include <memory>
#include "serial_handler.h"

#include "Callback.h"
#include "Timer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdlib>

// #############################################################################################//

SerialHandler::SerialHandler()
    // : m_serial(TERMINAL_TX_PIN, TERMINAL_RX_PIN, SERIAL_BAUDRATE)
    : m_serial(RS485_TX_PIN, RS485_RX_PIN, SERIAL_BAUDRATE)

{
}

// #############################################################################################//

SerialHandler::~SerialHandler()
{
}

// #############################################################################################//

void SerialHandler::init(st_generalConfig *genConfig, st_powerElecPackInfoConfig *powerElecPackInfoConfig)
{
    m_ptrGenConfig = genConfig;
    m_ptrPowerElecPackInfoConfig = powerElecPackInfoConfig;
}

// #############################################################################################//

void SerialHandler::lock()
{
    m_mutex.lock();
}

// #############################################################################################//

void SerialHandler::unlock()
{
    m_mutex.unlock();
}

// #############################################################################################//

bool SerialHandler::serialPrint(const char *command, ...)
{
    va_list args;
    va_start(args, command);
    bool res = vsend(command, args);
    va_end(args);
    return res;
}

// #############################################################################################//

void SerialHandler::puts(const char *data, size_t size)
{
    lock();
    m_serial.write(data, size);
    unlock();
}

// #############################################################################################//

void SerialHandler::LOGI(const char *message)
{
    lock();
    m_serial.write(TERMINAL_BLUEBOLD, 8);
    unlock();
    puts(message, strlen(message));
}

// #############################################################################################//

void SerialHandler::LOGW(const char *message)
{
    lock();
    m_serial.write(TERMINAL_YELLOW, 8);
    unlock();
    puts(message, strlen(message));
}

// #############################################################################################//

void SerialHandler::LOGE(const char *message)
{
    lock();
    m_serial.write(TERMINAL_REDBOLD, 8);
    unlock();
    puts(message, strlen(message));
}

// #############################################################################################//

bool SerialHandler::vsend(const char *format, va_list args)
{
    char m_sendBuffer[512];
    int len = vsnprintf(m_sendBuffer, sizeof(m_sendBuffer), format, args);
    if (len < 0)
    {
        return false;
    }
    if (len >= (int)sizeof(m_sendBuffer))
    {
        len = (int)sizeof(m_sendBuffer) - 1;
    }

    lock();
    m_serial.write(m_sendBuffer, len);
    unlock();

    return true;
}
