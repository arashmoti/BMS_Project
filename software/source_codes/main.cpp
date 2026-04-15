#include "mbed.h"
#include "Settings.h"

#include "datetime_handler.h"
#include "eeprom_handler.h"
#include "effect_handler.h"
#include "power_handler.h"
#include "serial_handler.h"
#include "sdcard_handler.h"
#include "interface_comm_handler.h"
#include "power_state_handler.h"
#include "BQ79616_handler.h"
#include "error_handler.h"
#include "inverter_can_handler.h"
#include "state_of_charge_handler.h"

#include "hw_measurement_base.h"
#include "hw_logging_base.h"
#include "hw_operation_base.h"
#include "hw_tasking_base.h"
#include "hw_error_base.h"

#include "measurement.h"
#include "logging.h"
#include "operation.h"
#include "tasking.h"

#include "sys_config.h"

#include <cstdint>
#include <stdint.h>
#include <string>
#include "drivers/Watchdog.h"
#include "mbed_fault_handler.h" // provides mbed_fault_context_t and API

using mbed::Watchdog;

/* USER CODE BEGIN PS */
st_generalConfig genConfig;
st_powerElecPackInfoConfig powerElecPackInfoConfig;
st_bqConfigStructTypedef bqConfig;
/* USER CODE END PS */

Mail<st_mailPowerElecPackInfoConfig, BQ_MEASUREMENT_COMMS_MAIL_SIZE> mailPowerElecBox;

Mail<st_mail_cell_voltages_t, LOGGING_CELL_VOLTAGE_MAIL_SIZE> mailCellVoltageBox;

Watchdog &wd = Watchdog::get_instance();

EffectHandler effect;
DatetimeHandler datetime;
EepromHandler eeprom;
PowerHandler pwr;
PowerStateHandler pwrState;
SerialHandler serialPort;
StateOfChargeHandler soc;
SdCardHandler sdCard(serialPort);
InverterCanHandler inverterCan(serialPort);
InterfaceCommHandler interfaceComm(eeprom, datetime, serialPort);

SysConfig sysConfig(eeprom);

BQ79616Handler bq(mailPowerElecBox, serialPort, interfaceComm, sdCard);

HwErrorBase hwError(sdCard, eeprom);
ErrorHandler err(hwError);

HwLoggingBase hwLogging(sdCard, interfaceComm, err);
Logging logging(hwLogging, mailCellVoltageBox);

HwMeasurementBase hwMeasurement(bq, datetime, pwr, interfaceComm, inverterCan, soc, serialPort, effect, pwrState, eeprom);
Measurement msment(mailPowerElecBox, mailCellVoltageBox, hwMeasurement);

HwOperationBase hwOperation(effect, pwrState, pwr, err, bq);
Operation op(hwOperation);

HwTaskingBase hwTasking(effect, datetime, pwr, pwrState, inverterCan);
Tasking task(hwTasking);

/* USER CODE BEGIN PV */

int main()
{

// --- Boot diagnostics: print reset cause (guarded) ---
#ifdef RCC
  interfaceComm.printToInterface("[BOOT] RCC_CSR=0x%08lX\r\n", (unsigned long)RCC->CSR);
  RCC->CSR |= RCC_CSR_RMVF; // clear flags
#endif
  // --- HardFault breadcrumbs from last run (if any) ---
  // if (g_fault_info.magic == 0xDEADBEEFu)
  // {
  //   interfaceComm.printToInterface(
  //       "[HARDFAULT] CFSR=0x%08lX HFSR=0x%08lX MMFAR=0x%08lX BFAR=0x%08lX LR=0x%08lX PC=0x%08lX xPSR=0x%08lX\r\n",
  //       (unsigned long)g_fault_info.cfsr, (unsigned long)g_fault_info.hfsr,
  //       (unsigned long)g_fault_info.mmfar, (unsigned long)g_fault_info.bfar,
  //       (unsigned long)g_fault_info.lr, (unsigned long)g_fault_info.pc,
  //       (unsigned long)g_fault_info.psr);
  //   g_fault_info.magic = 0; // clear after reporting
  // }
  mbed_fault_context_t fc;
  if (mbed_get_reboot_fault_context(&fc) == MBED_SUCCESS)
  {
    interfaceComm.printToInterface(
        "[HARDFAULT] PC=0x%08lX LR=0x%08lX xPSR=0x%08lX SP=0x%08lX EXC_RETURN=0x%08lX CONTROL=0x%08lX\r\n",
        (unsigned long)fc.PC_reg, (unsigned long)fc.LR_reg,
        (unsigned long)fc.xPSR, (unsigned long)fc.SP_reg,
        (unsigned long)fc.EXC_RETURN, (unsigned long)fc.CONTROL);
    // Optionally: zero 'fc' by calling mbed_get_reboot_fault_context again into a dummy,
    // or just leave it—Mbed clears it after you retrieve it next reboot.
  }

  interfaceComm.printToInterface(" - starting hardware watchdog (%ums)\r\n", (unsigned)WATCHDOG_TIMEOUT_MS);
  wd.start(WATCHDOG_TIMEOUT_MS);
  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;
  SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

  interfaceComm.printToInterface(" - watchdog running: %d\r\n", wd.is_running());
  // Assert power-hold as early as possible so PSU stays latched
  pwrState.init(); // sets POWER_EN (PB4) and power-button LED pins
  // BQ wake pulse (ensure BQ chips are listening)

  // STEP 1: Force a hardware wake-up pulse on the BQ communication line.
  // This is a direct, low-level way to ensure the BQ chips are listening.
  DigitalOut bq_wake(BQ79616_TX_PIN);
  bq_wake = 0;
  wait_us(5000); // 5ms pulse
  bq_wake = 1;
  ThisThread::sleep_for(250ms); // Give the BQ chips a generous time to power up.
  interfaceComm.printToInterface("\r\n--- MCU Booting Up. Initializing BQ Hardware... ---\r\n");

  // End of BQ wake sequence
  // datetime.init();
  // Developer note: to force a one-time EEPROM wipe at next boot,
  // you can call eeprom.invalidate_eeprom_mappings() here (keep commented in production).
  // Early config sanity: load defaults if mapping invalid
  if (eeprom.getSettingValue(enEepromValuesIndex::CMD_EE_ADDR_MappedEepromVal) != EEPROM_MAPPED_VALUE)
  {
    // First boot or invalid mapping: seed with defaults
    eeprom.defaultLoading();
  }
  // Always load current settings from EEPROM into genConfig
  sysConfig.loadDefaultConfig(&genConfig);

  // // --- Temporary Force Update ---
  // // Forces the precharge timeout to 400ms without requiring a full EEPROM reset.
  // // Remove this line after the first successful boot with the new firmware.
  // eeprom.setSettingValue(CMD_EE_ADDR_TimeoutLowCurrentPreChargeRetry, 400);

  // Announce configured battery capacity (tweaked value from EEPROM/defaults)
  interfaceComm.printToInterface("[CFG] Battery capacity: %u Ah\r\n", (unsigned)genConfig.u16BatteryCapacity);

  // Now that configuration is loaded, initialize subsystems that depend on it
  pwr.init(&genConfig, &powerElecPackInfoConfig);
  serialPort.init(&genConfig, &powerElecPackInfoConfig);
  interfaceComm.init(&genConfig, &powerElecPackInfoConfig);
  {
    const uint16_t commissioning_raw = eeprom.getSettingValue(CMD_EE_ADDR_CommissioningDone);
    // FORCE_COMMISSIONING_DONE is defined in Settings.h
#if FORCE_COMMISSIONING_DONE
    // Temporary override: force commissioning to appear done
    const uint8_t commissioning_done = 1U;
    const uint8_t commissioning_active = 0U;
    (void)commissioning_raw;  // Suppress unused warning
#else
    const uint8_t commissioning_done = (commissioning_raw == 1) ? 1U : 0U;
    const uint8_t commissioning_active = commissioning_done ? 0U : 1U;
#endif
    interfaceComm.printToInterface("[COMMISSION] done=%u active=%u\r\n",
                                   (unsigned)commissioning_done,
                                   (unsigned)commissioning_active);
    interfaceComm.printToInterface("[FLAGS] DISABLE_BALANCING=%u DISABLE_OCV_CORRECTION=%u\r\n",
                                   (unsigned)DISABLE_BALANCING,
                                   (unsigned)DISABLE_OCV_CORRECTION);
    sdCard.logEvent("COMMISSION done=%u active=%u",
                    (unsigned)commissioning_done,
                    (unsigned)commissioning_active);
  }
  err.init();

  logging.init(&genConfig, &powerElecPackInfoConfig);
  op.init(&genConfig, &powerElecPackInfoConfig);
  msment.init(&genConfig, &powerElecPackInfoConfig);
  task.init();
  inverterCan.init();

  bqConfig.numberOfCells = CELL_NUMBER_16S;
#if defined(LITHIUM_ION)
  bqConfig.UVThreshold = UV_THR_2500mV;
  bqConfig.OVThreshold = OV_THR_4250mV;
#endif

#if defined(LITHIUM_IRON_PHOSPHATE)
  bqConfig.UVThreshold = UV_THR_2500mV;
  bqConfig.OVThreshold = OV_THR_3650mV;
#endif
  bqConfig.UTThreshold = PERCENT_80;
  bqConfig.OTThreshold = PERCENT_25;
  bqConfig.mainAdcMode = CONTINUOUS_RUN;
  bqConfig.protectionOVUVMode = ROUND_ROBIN;
  bqConfig.protectionOTUTMode = ROUND_ROBIN;
  // Use long HW timer so firmware explicitly stops bleeders before wrap (quiet window handles cut-off)
  bqConfig.balanceDutyCycle = SEC_30; // MIN_1; SEC_10;
  // ThisThread::sleep_for(5ms); // Add a small delay after init
  // Optional: force a BQ hardware reset (enable only for debugging)
  // bq.reset();
  // ThisThread::sleep_for(10ms);

  // sysConfig.loadDefaultConfig(&genConfig);
  // Highlight: Add a call to the new function to update the CAN handler.
  inverterCan.update_can_parameters(&genConfig);
  bq.init(&bqConfig);

  interfaceComm.printToInterface(" Hello World from main! \r\n");
  // serialPort.serialPrint("\r\n\r\n--- MCU Booting Up ---\r\n");

  // interfaceComm.printToInterface("System started at %s\r\n", datetime.getCurrentDateTime().c_str());
  // serialPort.LOGI("SystemReady...\r\n");
  interfaceComm.printToInterface("--- All Systems Initialized. Starting Threads. ---\r\n");

  bq.startThread();
  task.startThread();
  logging.startThread();
  op.startThread();
  interfaceComm.startThread();
  msment.startThread();
  Watchdog::get_instance().kick(); // optional one-shot

  // --- Runtime EEPROM reset window (first 10 s after boot) ---
  using namespace std::chrono;
  DigitalIn eep_reset_pin(PB_10);
  eep_reset_pin.mode(PullUp);
  const Kernel::Clock::time_point reset_window_end = Kernel::Clock::now() + 10s;
  uint32_t hold_low_ms = 0;
  bool s_pb10Announced = false;
  Kernel::Clock::time_point last_sample = Kernel::Clock::now();
  Kernel::Clock::time_point last_commission_log = Kernel::Clock::now();

  while (true)
  {
    // Non-blocking sample every ~20 ms within the first 10 s window
    const Kernel::Clock::time_point now_tp = Kernel::Clock::now();
    if (now_tp < reset_window_end)
    {
      if (duration_cast<milliseconds>(now_tp - last_sample).count() >= 20)
      {
        last_sample = now_tp;
        // Keep PSU latched during window
        pwrState.setPwrBtnEnPin(PIN_STATE_ENABLE);
        // If PB10 is low (active), accumulate hold time; else reset accumulator
        if (eep_reset_pin.read() == 0)
        {
          hold_low_ms += 20;
          // Show RED while holding
          effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_RED, ModEffectArgs::EffectStateType::STATE_ON);
          if (!s_pb10Announced)
          {
            interfaceComm.printToInterface("[EEPROM] Reset request detected; holding...\r\n");
            s_pb10Announced = true;
          }
        }
        else
        {
          hold_low_ms = 0;
          effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_RED, ModEffectArgs::EffectStateType::STATE_OFF);
          s_pb10Announced = false;
        }

        if (hold_low_ms >= 2000) // 2 s continuous
        {
          interfaceComm.printToInterface("[EEPROM] Reset confirmed; reloading defaults\r\n");
          sdCard.logEvent("EEPROM reset defaults at boot");
          eeprom.invalidate_eeprom_mappings();
          eeprom.defaultLoading();
          ThisThread::sleep_for(250ms);
          NVIC_SystemReset();
        }
      }
    }
    if (duration_cast<milliseconds>(now_tp - last_commission_log).count() >= 60000)
    {
      last_commission_log = now_tp;
      const uint16_t commissioning_raw = eeprom.getSettingValue(CMD_EE_ADDR_CommissioningDone);
#if FORCE_COMMISSIONING_DONE
      // Temporary override: force commissioning to appear done
      const uint8_t commissioning_done = 1U;
      const uint8_t commissioning_active = 0U;
      (void)commissioning_raw;  // Suppress unused warning
#else
      const uint8_t commissioning_done = (commissioning_raw == 1) ? 1U : 0U;
      const uint8_t commissioning_active = commissioning_done ? 0U : 1U;
#endif
      interfaceComm.printToInterface("[COMMISSION] done=%u active=%u\r\n",
                                     (unsigned)commissioning_done,
                                     (unsigned)commissioning_active);
      sdCard.logEvent("COMMISSION done=%u active=%u",
                      (unsigned)commissioning_done,
                      (unsigned)commissioning_active);
    }
    
    // Periodically run effect task (LEDs, Buzzer)
    effect.task();
    
    ThisThread::sleep_for(1ms);
  }
}

// ##############################################################################################################################
