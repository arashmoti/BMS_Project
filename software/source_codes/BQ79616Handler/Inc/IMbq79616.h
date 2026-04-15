#ifndef __IMBQ79616__H_
#define __IMBQ79616__H_

#include "Settings.h"

#include "stddef.h"
#include <cstdint>
#include <vector>
#include "BQ79616_handler_params.h"

class IMbq79616
{
public:
    IMbq79616() {};
    virtual ~IMbq79616() = default;

    virtual std::vector<enCellBalanceTimeIsUp> getBalanceTimeUp(void) const = 0;
    virtual enBqErrorType getInternalError(void) const = 0;
    virtual std::vector<std::vector<uint8_t>> getAllInternalErrors(void) const = 0;
    virtual void getHardOverandUnderVoltageFlags(uint16_t *underVoltageFlags, uint16_t *overVoltageFlags) = 0;
    virtual void getHardOverTemperatureFlags(uint16_t *overTemperatureFlags) = 0;
    virtual bool faultDetected(void) = 0;

    virtual measComm::enErrorCodes reset(void) = 0;
    virtual measComm::enErrorCodes reinitAndReaddress(void) = 0;

    virtual void setAllowedForResetInternalError() = 0;
    virtual void shutdown(void) = 0;

    // virtual void balanceProcess(st_cellMonitorCells cellVoltagesIndividual[64], uint16_t cellMisMatch) = 0;
    virtual void sendBalancingValues(st_cellMonitorCells *cellVoltagesIndividual, uint16_t cellMisMatch) = 0;
    virtual void setBalanceActive(bool _value) = 0;
    virtual bool isBalanceActive() const = 0;
    virtual void forceClearBleeders(void) = 0;

    virtual bool isCommHealthy(void) = 0;
    virtual bool isMeasureCycleComplete(void) const = 0;
    virtual void resetMeasureCycleComplete(void) = 0;
    virtual void printLastCommFaultReason(void) = 0;
};

#endif // __IMBQ79616__H_
