#ifndef __ISERIAL__H_
#define __ISERIAL__H_

#include "stddef.h"

#define TERMINAL_BLACK               "\033[0;30m"
#define TERMINAL_RED                 "\033[0;31m"
#define TERMINAL_GREEN               "\033[0;32m"
#define TERMINAL_YELLOW              "\033[0;33m"
#define TERMINAL_BLUE                "\033[0;34m"
#define TERMINAL_PURPLE              "\033[0;35m"
#define TERMINAL_CYAN                "\033[0;36m"
#define TERMINAL_WHITE               "\033[0;37m"
#define TERMINAL_BLACKBOLD           "\033[1;30m"
#define TERMINAL_REDBOLD             "\033[1;31m"
#define TERMINAL_GREENBOLD           "\033[1;32m"
#define TERMINAL_YELLOWBOLD          "\033[1;33m"
#define TERMINAL_BLUEBOLD            "\033[1;34m"
#define TERMINAL_PURPLEBOLD          "\033[1;35m"
#define TERMINAL_CYANBOLD            "\033[1;36m"
#define TERMINAL_WHITEBOLD           "\033[1;37m"

class ISerial
{
public:
    ISerial() {};
    virtual ~ISerial() = default;

    virtual bool serialPrint(const char *format, ...) = 0;
    virtual void LOGI(const char *message) = 0;
    virtual void LOGW(const char *message) = 0;
    virtual void LOGE(const char *message) = 0;

};


#endif // __ISERIAL__H_