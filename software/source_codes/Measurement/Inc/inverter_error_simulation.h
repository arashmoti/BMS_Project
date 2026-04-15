#ifndef INVERTER_ERROR_SIMULATION_H
#define INVERTER_ERROR_SIMULATION_H

#include "IMInverterCan.h"

void sendInverterProtectionStatus(IMInverterCan &inverterCan,
                                  const InverterCanArgs::st_INVERTER_PROTECT_WARNING &warn,
                                  bool allowed);

#endif // INVERTER_ERROR_SIMULATION_H
