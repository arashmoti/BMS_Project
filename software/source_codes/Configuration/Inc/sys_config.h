#ifndef __SYS_CONFIG__H_
#define __SYS_CONFIG__H_

#include "io_periph_defs.h"
#include "Settings.h"
#include "IEeprom.h"

#include <cstdint>





class SysConfig
{

public:
    SysConfig(IEeprom& _eeprom);
    virtual ~SysConfig();
    void loadDefaultConfig(st_generalConfig* genConfig);

private:


private:
  st_generalConfig* m_ptrGenConfig;
  IEeprom& m_eeprom;

};

#endif // __SYS_CONFIG__H_
