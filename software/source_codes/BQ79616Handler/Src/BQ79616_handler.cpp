#include "Settings.h"

// Wrapper: select legacy or new driver implementation at compile time.
// Legacy source stays untouched in BQ79616_handler_legacy.inc for easy rollback.
#if USE_LEGACY_BQ_DRIVER
#include "BQ79616_handler_legacy.inc"

// Legacy compatibility shims for newer IMbq79616 API.
measComm::enErrorCodes BQ79616Handler::reinitAndReaddress(void)
{
    return reset();
}

void BQ79616Handler::forceClearBleeders(void)
{
    std::vector<uint8_t> zeros(8, (uint8_t)measSettings::enBalancingTimer::T_NONE);
    for (uint8_t dev = 0; dev < NoOfCELL_MONITORS_POSSIBLE_ON_BMS; ++dev)
    {
        setCurrentDeviceAddress(dev);
        (void)request(measComm::enReqType::SINGLE_WRITE, m_u8CurrentDeviceAddress, CB_CELL16_CTRL, zeros);
        (void)request(measComm::enReqType::SINGLE_WRITE, m_u8CurrentDeviceAddress, CB_CELL8_CTRL, zeros);
    }

    // Datasheet forced-stop mechanism: after zeroing timers, issue BAL_CTRL2[BAL_GO]=1.
    {
        measControl::unBAL_CTRL2 bal = {};
        bal.all = 0;
        bal.bits.AUTO_BAL = 1;
        bal.bits.BAL_GO = 1;
        bal.bits.OTCB_EN = 1;
        std::vector<uint8_t> dataToWrite = {bal.all};
        (void)request(measComm::enReqType::BROADCAST_WRITE, 0x00, BAL_CTRL2, dataToWrite);
    }
}

bool BQ79616Handler::isCommHealthy(void)
{
    // Legacy driver did not gate balancing on comm-health.
    return true;
}

void BQ79616Handler::printLastCommFaultReason(void)
{
    interfaceComm.printToInterface("[COMM] Legacy driver: fault detail unavailable\r\n");
}
#else
#include "BQ79616_handler_new.inc"
#endif
