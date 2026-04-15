#include "inverter_error_simulation.h"
#include "Settings.h"
#include "Timer.h"
#include <cstddef>
#include <chrono>
#include "inverter_can_handler.h"

using namespace std::chrono;

void sendInverterProtectionStatus(IMInverterCan &inverterCan,
                                  const InverterCanArgs::st_INVERTER_PROTECT_WARNING &warn,
                                  bool allowed)
{
  (void)warn;
  if (!allowed)
    return;

  extern st_powerElecPackInfoConfig powerElecPackInfoConfig;
  const enBatteryErrors err = powerElecPackInfoConfig.packBatteryWarErrState;
  //inverterCan.updateWarningAndProtect(err);
}
