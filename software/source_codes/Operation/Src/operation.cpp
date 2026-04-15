#include <cstdint>
#include <memory>
#include "operation.h"

#include "Callback.h"
#include "Timer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdlib>
#include <chrono>
#include "interface_comm_handler.h"
#include "sdcard_handler.h"
#include "balancing_tuning.h"
#include "mbed.h" // pulls Kernel, CriticalSection, etc.
#include "drivers/Watchdog.h"
#include "rtos/Mutex.h"
using mbed::Watchdog;

// Access global SD handler defined in main.cpp
extern SdCardHandler sdCard;
extern rtos::Mutex g_packInfoMutex;

volatile uint32_t g_heartbeat_mask = 0;
volatile uint32_t g_hb_last_ms[3] = {0, 0, 0}; // MEAS, BQ, LOG last-seen timestamps (Kernel::Clock ms)
volatile uint32_t g_opLoopTimestampMs = 0;     // updated each opThread loop for external watchdog diagnostics

static inline void hb_or_u32(volatile uint32_t &v, uint32_t m)
{
	core_util_critical_section_enter();
	v |= m;
	core_util_critical_section_exit();
}

void hb_set(uint32_t bit)
{
	// Record both the bit and when we last saw it, so we can report ages if it goes missing
	const uint32_t now_ms =
	    (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(Kernel::Clock::now().time_since_epoch()).count();

	hb_or_u32(g_heartbeat_mask, bit);

	if (bit & HB_MEAS)
		g_hb_last_ms[0] = now_ms;
	if (bit & HB_BQ)
		g_hb_last_ms[1] = now_ms;
	if (bit & HB_LOG)
		g_hb_last_ms[2] = now_ms;
}

static constexpr int32_t kChargeAbsCurrentFloor_mA = 100;
static bool s_wdt_diag_enable = true; // Set false to disable WDT diagnostics

// #############################################################################################//
// Human-readable BMS state string
static const char *bms_state_to_str(enBmsState s)
{
	switch (s)
	{
	case BMS_IDLE:
		return "IDLE";
	case BMS_DISCHARGE:
		return "DISCHARGE";
	case BMS_CHARGE:
		return "CHARGE";
	default:
		return "UNKNOWN";
	}
}

// #############################################################################################//
// #############################################################################################//

Operation::Operation(HwOperationBase &_hwOperation)
	: m_hwOperation(_hwOperation),
	  m_bOpStateForceOn(false),
	  m_bOpStatePressedState(false),
	  m_opThreadVar(OPERATION_THREAD_PRIORITY, OPERATION_THREAD_STACK_SIZE, nullptr, "Operation")
{
	m_opManagerTim.start();
}

// #############################################################################################//

Operation::~Operation()
{
}

// #############################################################################################//

void Operation::startThread()
{
	m_opThreadVar.start(mbed::callback(this, &Operation::opThread));
}

// #############################################################################################//

void Operation::init(st_generalConfig *genConfig, st_powerElecPackInfoConfig *powerElecPackInfoConfig)
{
	m_ptrGenConfig = genConfig;
	m_ptrPowerElecPackInfoConfig = powerElecPackInfoConfig;
	operationStateSetAllStates(OP_STATE_INIT);

	m_u32StateStartupDelay = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
	m_u32StateChargerDisconnectDetectDelay = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
	packOperationCellStateLastErrorState = PACK_STATE_NORMAL;
	m_bOpStateForceOn = false;
	m_u32StateNotUsedTime = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
	m_u8PrechargeRetryCount = 0; // Initialize
	m_u32StateNotUsedResetDelay = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
}

// #############################################################################################//

uint8_t Operation::timerDelay1ms(uint32_t *last, uint32_t ticks)
{
	if ((uint32_t)(OPERATION_GET_TICK(m_opManagerTim) - *last) >= ticks)
	{
		*last = OPERATION_GET_TICK(m_opManagerTim);
		return true;
	}
	return false;
}

// #############################################################################################//

void Operation::opThread()
{
	using mbed::Watchdog;

	using namespace std::chrono;
	auto lastKick_tp = Kernel::Clock::now();

	// Kick unconditionally during a short boot grace so init + first heartbeats can arrive.
	static constexpr uint32_t WDT_BOOT_GRACE_MS = 10000; // 10 s
	auto boot_grace_end_tp = lastKick_tp + milliseconds(WDT_BOOT_GRACE_MS);

	bool saw_log = false; // becomes true once HB_LOG is ever observed

	// Watchdog diagnostics
	uint32_t last_iter_ms = 0;
	uint32_t last_kick_ms = 0;
	uint32_t last_kick_log_ms = 0;

	while (true)
	{
		// --- existing fault handling & state machine ---
		enBatteryErrors packErrState = SYS_OK;
		enInverterResponseState inverterState = INVERTER_OK;
	g_packInfoMutex.lock();
	packErrState = m_ptrPowerElecPackInfoConfig->packBatteryWarErrState;
	inverterState = m_ptrPowerElecPackInfoConfig->inverterResponseState;
	// Check for cell operation faults (Temperature, Voltage, etc.)
	enPowerElecPackOperationCellState cellOpState = m_ptrPowerElecPackInfoConfig->packOperationCellState;
	g_packInfoMutex.unlock();

		if (packErrState == SYS_ERROR_CHIPSET_COMM || 
			cellOpState == PACK_STATE_ERROR_OVER_TEMPERATURE || 
			cellOpState == PACK_STATE_ERROR_UNDER_TEMPERATURE ||
			cellOpState == PACK_STATE_CRITICAL_TEMPERATURE)
		{
			operationStateSetNewState(OP_STATE_ERROR);
		}
		else if (inverterState == INVERTER_FAULT)
		{
			InterfaceCommHandler::getInstance()->printToInterface(
				"!!! Inverter ACK timeout → opening CHARGE only, staying online.\r\n");
			// Keep contactors unchanged; rely on hard-OV dwell for physical disconnect
			operationStateSetNewState(OP_STATE_BALANCING);
			g_packInfoMutex.lock();
			m_ptrPowerElecPackInfoConfig->inverterResponseState = INVERTER_OK;
			g_packInfoMutex.unlock();
		}

		operationStateTask();

		// --- watchdog kick gate ---
		const auto now_tp = Kernel::Clock::now();
		const uint32_t now_ms = (uint32_t)duration_cast<milliseconds>(now_tp.time_since_epoch()).count();

		// export loop timestamp for external cross-check (measurement thread)
		g_opLoopTimestampMs = now_ms;

		// Detect long loop gaps (helps catch stalls)
		if (s_wdt_diag_enable && last_iter_ms != 0)
		{
			const uint32_t loop_gap = now_ms - last_iter_ms;
			if (loop_gap > 1000U)
			{
				InterfaceCommHandler::getInstance()->printToInterface(
					"[WDT] Loop gap %lu ms\n", (unsigned long)loop_gap);
				sdCard.logEvent("WDT loop gap %lums", (unsigned long)loop_gap);
			}
		}
		last_iter_ms = now_ms;

		if (duration_cast<milliseconds>(now_tp - lastKick_tp).count() >= WDT_KICK_PERIOD_MS) // ~500 ms (from your header)
		{
			const bool in_grace = (now_tp < boot_grace_end_tp);

			// Latch that logging is alive once HB_LOG shows up at least once
			if (g_heartbeat_mask & HB_LOG)
			{
				saw_log = true;
			}

			// Require LOG only after it’s alive (or after grace expires)
			const uint32_t required =
				HB_MEAS | HB_BQ | ((saw_log || !in_grace) ? HB_LOG : 0u);

			if (in_grace || ((g_heartbeat_mask & required) == required))
			{
				Watchdog::get_instance().kick();
				lastKick_tp = now_tp;
				last_kick_ms = now_ms;

				if (s_wdt_diag_enable)
				{
					static int s_lastGraceState = -1;
					const uint32_t since_log = now_ms - last_kick_log_ms;
					const int graceState = in_grace ? 1 : 0;
					if (since_log >= 10000U || s_lastGraceState != graceState)
					{
						InterfaceCommHandler::getInstance()->printToInterface(
							"[WDT] Kick ok (dt=%lu ms, grace=%d)\n",
							(unsigned long)since_log, (int)in_grace);
						sdCard.logEvent("WDT kick ok dt=%lums grace=%d", (unsigned long)since_log, (int)in_grace);
						last_kick_log_ms = now_ms;
						s_lastGraceState = graceState;
					}
				}

				// Only reset the window when we’re enforcing heartbeats
				if (!in_grace)
				{
					g_heartbeat_mask = 0;
				}
			}
			else
			{
				// Log once while we skip a kick so we know which heartbeats were missing
				static uint32_t last_skip_log_ms = 0;
				static uint32_t last_missing = 0;
				const uint32_t missing = required & (~g_heartbeat_mask);
				const uint32_t age_meas = g_hb_last_ms[0] ? (now_ms - g_hb_last_ms[0]) : 0;
				const uint32_t age_bq = g_hb_last_ms[1] ? (now_ms - g_hb_last_ms[1]) : 0;
				const uint32_t age_log = g_hb_last_ms[2] ? (now_ms - g_hb_last_ms[2]) : 0;
				if (s_wdt_diag_enable && ((missing != last_missing) || ((uint32_t)(now_ms - last_skip_log_ms) >= 2000U)))
				{
					InterfaceCommHandler::getInstance()->printToInterface(
						"[WDT] Skip kick: missing HB=0x%02lX req=0x%02lX ages(ms) MEAS=%lu BQ=%lu LOG=%lu\n",
						(unsigned long)missing, (unsigned long)required,
						(unsigned long)age_meas, (unsigned long)age_bq, (unsigned long)age_log);
					sdCard.logEvent("WDT skip: miss HB=0x%02lX req=0x%02lX ages M:%lums BQ:%lums L:%lums",
					                (unsigned long)missing, (unsigned long)required,
					                (unsigned long)age_meas, (unsigned long)age_bq, (unsigned long)age_log);
					last_skip_log_ms = now_ms;
					last_missing = missing;
				}
				// (optional) add a half-timeout failsafe here to force safe outputs
			}
		}

		ThisThread::sleep_for(1ms);
	}
}

// #############################################################################################//

void Operation::operationStateTask(void)
{

	enBmsState bmsState = BMS_IDLE;
	uint32_t lastBqTimestamp = 0;
	uint16_t cellVoltageMin = 0;
	bool dischargeLowAllowed = false;
	uint32_t moduleLoadVoltage = 0;
	uint32_t moduleVoltage = 0;
	bool chargeAllowed = false;
	enOvervoltageState ovState = OV_STATE_NORMAL;
	int32_t moduleCurrent = 0;
	uint16_t cellVoltageMismatch = 0;
	int32_t moduleTempMax = 0;
	uint8_t allowedShutdown = 0;

	g_packInfoMutex.lock();
	bmsState = m_ptrPowerElecPackInfoConfig->bmsState;
	lastBqTimestamp = m_ptrPowerElecPackInfoConfig->u32LastBqDataTimestamp;
	cellVoltageMin = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMin;
	dischargeLowAllowed = m_ptrPowerElecPackInfoConfig->bDischargeLowCurrentAllowed;
	moduleLoadVoltage = m_ptrPowerElecPackInfoConfig->u32ModuleLoadVoltage;
	moduleVoltage = m_ptrPowerElecPackInfoConfig->u32ModuleVoltage;
	chargeAllowed = m_ptrPowerElecPackInfoConfig->bChargeAllowed;
	ovState = m_ptrPowerElecPackInfoConfig->overvoltageState;
	moduleCurrent = m_ptrPowerElecPackInfoConfig->i32ModuleCurrent;
	cellVoltageMismatch = m_ptrPowerElecPackInfoConfig->u16ModuleCellVoltageMisMatch;
	moduleTempMax = m_ptrPowerElecPackInfoConfig->i32ModuleTemperatureMax;
	allowedShutdown = m_ptrPowerElecPackInfoConfig->u8allowedShutDown;
	g_packInfoMutex.unlock();

static constexpr uint32_t kOpStateLogMs = 1000;

	static bool s_powerdown_req_logged = false;
	const bool button_powerdown = m_hwOperation.pwrState.getPowerdownRequest();
	const bool force_powerdown = (button_powerdown || allowedShutdown);
	if (force_powerdown && m_operationalStateCurrentState != OP_STATE_POWER_DOWN)
	{
		if (!s_powerdown_req_logged)
		{
			InterfaceCommHandler::getInstance()->printToInterface(
				"--- Powerdown request detected (button=%u, allowed=%u) ---\r\n",
				(unsigned)button_powerdown,
				(unsigned)allowedShutdown);
			s_powerdown_req_logged = true;
		}
		operationStateSetNewState(OP_STATE_POWER_DOWN);
		operationStateUpdateStates();
		sdCard.safeEject();
	}

	switch (m_operationalStateCurrentState)
	{
	case OP_STATE_INIT:
	{
		// Reset BQ when we first (re)enter INIT
		if (m_operationalStateLastState != OP_STATE_INIT)
		{
			m_hwOperation.bq.reset();
			ThisThread::sleep_for(10ms);
		}

		static uint32_t _u32DebugPrintLastTick0 = 0;
		if (timerDelay1ms(&_u32DebugPrintLastTick0, kOpStateLogMs))
			InterfaceCommHandler::getInstance()->printToInterface("----- OP_STATE_INIT: bmsState: %s \r\n", bms_state_to_str(bmsState));

		if (m_hwOperation.pwrState.chargeDetected() || m_hwOperation.pwrState.getButtonPressedOnTurnon())
		{
			operationStateSetNewState(OP_STATE_PRE_CHARGE);
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_GREEN, ModEffectArgs::EffectStateType::STATE_ON);
		}

		if (timerDelay1ms(&m_u32StateStartupDelay, 5000))
		{
			// Only declare dead battery if we have valid BQ data and no charger
			if (lastBqTimestamp > 0 &&
				!m_hwOperation.pwrState.chargeDetected())
			{
				if (cellVoltageMin < m_ptrGenConfig->u16HardUnderVoltage)
				{
					operationStateSetNewState(OP_STATE_BATTERY_DEAD);
					m_u32StateBatteryDeadDisplayTime = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
				}
				else
				{
					// AUTO-TRANSITION after soft reset: If BQ data is valid and battery is healthy,
					// auto-transition to PRE_CHARGE. This handles autonomous resets (e.g., 60-second
					// comm timeout reset) where no button press or charger is present.
					InterfaceCommHandler::getInstance()->printToInterface(
						"[OP] Auto-transition INIT->PRE_CHARGE (soft reset recovery, Vmin=%u mV)\r\n",
						(unsigned)cellVoltageMin);
					operationStateSetNewState(OP_STATE_PRE_CHARGE);
					m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_GREEN, ModEffectArgs::EffectStateType::STATE_ON);
				}
			}
			operationStateUpdateStates();
		}
	}
	break;

	case OP_STATE_PRE_CHARGE:
	{
		static uint32_t _u32DebugPrintLastTick01 = 0;
		if (timerDelay1ms(&_u32DebugPrintLastTick01, kOpStateLogMs))
			InterfaceCommHandler::getInstance()->printToInterface("----- OP_STATE_PRE_CHARGE: bmsState: %s \r\n", bms_state_to_str(bmsState));

		if (m_operationalStateLastState != m_operationalStateCurrentState)
		{
			m_u8PrechargeRetryCount = 0;
			m_u32StatePreChargeTimeout = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
			m_hwOperation.pwr.setDisCharge(false);
			m_hwOperation.pwr.setCharge(false);
		}

		if (dischargeLowAllowed || m_bOpStateForceOn)
		{
			m_hwOperation.pwr.setPreCharge(true);
		}
		else
		{
			m_hwOperation.pwr.setPreCharge(false);
			m_u32StatePreChargeTimeout = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
		}

		if ((moduleLoadVoltage > (uint16_t)((float)moduleVoltage * ((float)m_ptrGenConfig->u16MinimalPrechargePercentage / 100.0))) && (dischargeLowAllowed || m_bOpStateForceOn))
		{
			// === NEW: Precharge Success Logging ===
			uint32_t preDuration = OPERATION_GET_TICK(m_opManagerTim) - m_u32StatePreChargeTimeout;
			uint16_t loadPct = (moduleVoltage > 0) ? (uint16_t)((moduleLoadVoltage * 100) / moduleVoltage) : 0;
			InterfaceCommHandler::getInstance()->printToInterface(
				"[PREC] Success: Vload=%lumV (%u%%) in %lu.%01us\r\n",
				(unsigned long)moduleLoadVoltage, (unsigned)loadPct,
				(unsigned long)(preDuration / 1000), (unsigned)((preDuration % 1000) / 100));

			if (m_bOpStateForceOn)
			{
				operationStateSetNewState(OP_STATE_FORCEON); // Goto force on
			}
			else
			{
				operationStateSetNewState(OP_STATE_LOAD_ENABLED); // Goto normal load enabled operation
			}
		}
		else if (timerDelay1ms(&m_u32StatePreChargeTimeout, m_ptrGenConfig->u16TimeoutLowCurrentPreChargeRetry))
		{
			// === NEW: Precharge Failure Logging ===
			uint32_t preDuration = OPERATION_GET_TICK(m_opManagerTim) - m_u32StatePreChargeTimeout;
			uint16_t loadPct = (moduleVoltage > 0) ? (uint16_t)((moduleLoadVoltage * 100) / moduleVoltage) : 0;
			InterfaceCommHandler::getInstance()->printToInterface(
				"[PREC] Fail: Vload=%lumV (%u%%) in %lu.%01us, retry %u/3\r\n",
				(unsigned long)moduleLoadVoltage, (unsigned)loadPct,
				(unsigned long)(preDuration / 1000), (unsigned)((preDuration % 1000) / 100),
				(unsigned)m_u8PrechargeRetryCount + 1);

			m_u8PrechargeRetryCount++;
			if (m_u8PrechargeRetryCount >= 3) // exceeded retry limit → hard error
			{
				operationStateSetNewState(OP_STATE_ERROR);
			}
			else
			{
				if (m_ptrGenConfig->bUsePrechargeFlag)
					operationStateSetNewState(OP_STATE_ERROR_PRECHARGE);
				else
					operationStateSetNewState(OP_STATE_LOAD_ENABLED);
			}
		}
		operationStateUpdateStates();
	}
	break;

	case OP_STATE_CHARGING:
	{
		static uint32_t _u32DebugPrintLastTick1 = 0;
		if (timerDelay1ms(&_u32DebugPrintLastTick1, kOpStateLogMs))
			InterfaceCommHandler::getInstance()->printToInterface("----- OP_STATE_CHARGING: bmsState: %s \r\n", bms_state_to_str(bmsState));

			// If balancing starts during charge, hand off to the balancing state to keep UI and LEDs in sync.
			if (m_ptrPowerElecPackInfoConfig->bBalancingActive)
			{
				operationStateSetNewState(OP_STATE_BALANCING);
				operationStateUpdateStates();
				break;
			}

		// Stop charging if not allowed or OV handling takes over
		if (!chargeAllowed || ovState != OV_STATE_NORMAL)
		{
			operationStateSetNewState(OP_STATE_BALANCING);
		}
		else
		{
			// Charger present & allowed -> keep contactors closed
			handleChargerDisconnect(OP_STATE_LOAD_ENABLED);
			// LED policy while charging: if charge-balancing mode is active, show BLUE; otherwise GREEN
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_GREEN, ModEffectArgs::EffectStateType::STATE_FLASH);

			// Debounced main contactor close to prevent ticking/oscillation
			static uint32_t disch_close_attempt_ms_ch = 0;
			const uint32_t now_ms_ch = OPERATION_GET_TICK(m_opManagerTim);

			m_hwOperation.pwr.setCharge(true);
			bool disch_closed_ch = m_hwOperation.pwr.setDisCharge(true);
			if (disch_closed_ch)
			{
				disch_close_attempt_ms_ch = 0;
				m_hwOperation.pwr.setPreCharge(false);
			}
			else
			{
				if (disch_close_attempt_ms_ch == 0)
					disch_close_attempt_ms_ch = now_ms_ch;
				if ((uint32_t)(now_ms_ch - disch_close_attempt_ms_ch) > 1500U)
				{
					// could not keep main closed → re-run precharge
					operationStateSetNewState(OP_STATE_PRE_CHARGE);
					m_hwOperation.pwr.setDisCharge(false);
					m_hwOperation.pwr.setCharge(false);
					disch_close_attempt_ms_ch = 0;
				}
			}
		}

		operationStateUpdateStates();
	}
	break;

	case OP_STATE_LOAD_ENABLED:
	{
		static uint32_t _u32DebugPrintLastTick010 = 0;
		if (timerDelay1ms(&_u32DebugPrintLastTick010, kOpStateLogMs))
			InterfaceCommHandler::getInstance()->printToInterface("----- OP_STATE_LOAD_ENABLED: bmsState: %s \r\n", bms_state_to_str(bmsState));

			// If balancing is active (unified flag), reflect that in the op-state for UI/policy clarity.
			if (m_ptrPowerElecPackInfoConfig->bBalancingActive)
			{
				operationStateSetNewState(OP_STATE_BALANCING);
				operationStateUpdateStates();
				break;
			}

		// Solid green in normal operation; make it exclusive
		m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_GREEN, ModEffectArgs::EffectStateType::STATE_ON);

		{
			// Debounced main contactor close
			static uint32_t disch_close_attempt_ms = 0;
			const uint32_t now_ms = OPERATION_GET_TICK(m_opManagerTim);

			bool disch_closed = m_hwOperation.pwr.setDisCharge(true);
			if (disch_closed)
			{
				disch_close_attempt_ms = 0;
				m_hwOperation.pwr.setPreCharge(false);
				m_hwOperation.pwr.setCharge(true);
			}
			else
			{
				if (disch_close_attempt_ms == 0)
					disch_close_attempt_ms = now_ms; // start debounce window
				if ((uint32_t)(now_ms - disch_close_attempt_ms) > 1500U)
				{
					operationStateSetNewState(OP_STATE_PRE_CHARGE);
					m_hwOperation.pwr.setDisCharge(false);
					m_hwOperation.pwr.setCharge(false);
					disch_close_attempt_ms = 0; // reset
				}
			}
		}
		// Enter charging if BMS requests it — do NOT open contactors here (prevents the “blip”)
		if (bmsState == BMS_CHARGE)
		{
			operationStateSetNewState(OP_STATE_CHARGING);
		}

		if (!dischargeLowAllowed)
		{ // battery empty
			operationStateSetNewState(OP_STATE_PRE_CHARGE);
			m_hwOperation.pwr.setDisCharge(false);
			m_hwOperation.pwr.setCharge(false);
		}

		if (std::abs(moduleCurrent) >= m_ptrGenConfig->u16NotUsedCurrentThreshold)
		{
			if (timerDelay1ms(&m_u32StateNotUsedResetDelay, 1000))
				m_u32StateNotUsedTime = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
		}
		else
		{
			m_u32StateNotUsedResetDelay = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
		}

		operationStateUpdateStates();

		displayData.u16PackVoltage = moduleVoltage;
		displayData.i32PackCurrent = moduleCurrent;
		displayData.i32PackTemperature = moduleTempMax;
	}
	break;

	case OP_STATE_BATTERY_DEAD:
	{
		static uint32_t _u32DebugPrintLastTick110 = 0;
		if (timerDelay1ms(&_u32DebugPrintLastTick110, kOpStateLogMs))
			InterfaceCommHandler::getInstance()->printToInterface("----- OP_STATE_BATTERY_DEAD: bmsState: %s \r\n", bms_state_to_str(bmsState));

		if (m_operationalStateLastState != m_operationalStateCurrentState)
		{
			m_u32StateBatteryDeadDisplayTime = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
		}

		m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_RED, ModEffectArgs::EffectStateType::STATE_ON);
		m_hwOperation.pwr.switchesDisableAll();
		if (timerDelay1ms(&m_u32StateBatteryDeadDisplayTime, m_ptrGenConfig->u16DisplayTimeoutBatteryDead))
			operationStateSetNewState(OP_STATE_POWER_DOWN);
		operationStateUpdateStates();
	}
	break;

	case OP_STATE_ERROR:
	{
		static uint32_t _u32DebugPrintLastTick110 = 0;
		if (timerDelay1ms(&_u32DebugPrintLastTick110, kOpStateLogMs))
			InterfaceCommHandler::getInstance()->printToInterface("----- OP_STATE_ERROR: bmsState: %s \r\n", bms_state_to_str(bmsState));

		if (m_operationalStateLastState != m_operationalStateCurrentState)
			m_u32StateErrorDisplayTime = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));

		// Shutdown Policy:
		// - Standard OT (60C) and UT (-20C): Stay ON to alert (Infinite Wait/No Shutdown).
		// - Critical OT (>100C) or System Faults: Power Down after timeout.
		// - Recovery: If state returns to NORMAL, go back to LOAD_ENABLED.
		
		g_packInfoMutex.lock();
		enPowerElecPackOperationCellState cellOpState = m_ptrPowerElecPackInfoConfig->packOperationCellState;
		enBatteryErrors errState = m_ptrPowerElecPackInfoConfig->packBatteryWarErrState;
		g_packInfoMutex.unlock();
		
		// DEBUG: Log the decision every second
		static uint32_t _u32ShutdownDbgTick = 0;
		if (timerDelay1ms(&_u32ShutdownDbgTick, 1000))
		{
			InterfaceCommHandler::getInstance()->printToInterface(
				"[OP-DBG] cellOpState=%u autoShutdown=%d\r\n", (unsigned)cellOpState, 
				(int)(cellOpState != PACK_STATE_ERROR_OVER_TEMPERATURE && cellOpState != PACK_STATE_ERROR_UNDER_TEMPERATURE && cellOpState != PACK_STATE_NORMAL));
		}

		// FIRST: Check for Recovery (state back to normal)
		if (cellOpState == PACK_STATE_NORMAL && errState == SYS_OK)
		{
			operationStateSetNewState(OP_STATE_LOAD_ENABLED);
			// Reset LED/Buzzer immediately
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_RED, ModEffectArgs::EffectStateType::STATE_OFF);
			
			g_packInfoMutex.lock();
			m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = false;
			m_ptrPowerElecPackInfoConfig->buzzerSignalType = BUZZER_SIGNAL_TYPE_OFF;
			g_packInfoMutex.unlock();
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::BUZZER, ModEffectArgs::EffectStateType::STATE_RESET);
			
			InterfaceCommHandler::getInstance()->printToInterface("[OP-RECOVER] Temp fault cleared, returning to LOAD_ENABLED\r\n");
		}
		// SECOND: Standard Temp Faults (60C OT, -20C UT) - Stay ON indefinitely
		else if (cellOpState == PACK_STATE_ERROR_OVER_TEMPERATURE || 
		         cellOpState == PACK_STATE_ERROR_UNDER_TEMPERATURE)
		{
			// Stay in ERROR state with alerts, but DO NOT use shutdown timer
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_RED, ModEffectArgs::EffectStateType::STATE_ON);
			g_packInfoMutex.lock();
			m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = true;
			m_ptrPowerElecPackInfoConfig->buzzerSignalType = BUZZER_SIGNAL_TYPE_TOGGLE_FAST;
			g_packInfoMutex.unlock();
		}
		// THIRD: Chipset communication loss - stay latched in ERROR until comm recovers or user intervenes
		else if (errState == SYS_ERROR_CHIPSET_COMM)
		{
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_RED, ModEffectArgs::EffectStateType::STATE_ON);
			g_packInfoMutex.lock();
			m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = true;
			m_ptrPowerElecPackInfoConfig->buzzerSignalType = BUZZER_SIGNAL_TYPE_ON;
			g_packInfoMutex.unlock();
			// No auto power-down; await comm recovery or manual reset
		}
		// THIRD: Critical Faults (CRITICAL_TEMP, etc.) - Use shutdown timer
		else
		{
			if (timerDelay1ms(&m_u32StateErrorDisplayTime, m_ptrGenConfig->u16DisplayTimeoutBatteryError))
				operationStateSetNewState(OP_STATE_POWER_DOWN);
			
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_RED, ModEffectArgs::EffectStateType::STATE_ON);
			g_packInfoMutex.lock();
			m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = true;
			m_ptrPowerElecPackInfoConfig->buzzerSignalType = BUZZER_SIGNAL_TYPE_ON; // Continuous for critical
			g_packInfoMutex.unlock();
		}

		m_hwOperation.pwr.switchesDisableAll();
		operationStateUpdateStates();
	}
	break;

	case OP_STATE_ERROR_PRECHARGE:
	{
		static uint32_t _u32DebugPrintLastTick0111 = 0;
		if (timerDelay1ms(&_u32DebugPrintLastTick0111, kOpStateLogMs))
			InterfaceCommHandler::getInstance()->printToInterface("----- OP_STATE_ERROR_PRECHARGE: bmsState: %s \r\n", bms_state_to_str(bmsState));

		if (m_operationalStateLastState != m_operationalStateCurrentState)
			m_u32StateErrorDisplayTime = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));

		if (timerDelay1ms(&m_u32StateErrorDisplayTime, m_ptrGenConfig->u16DisplayTimeoutBatteryErrorPreCharge))
		{
			// After the error display timeout, shut down
			operationStateSetNewState(OP_STATE_POWER_DOWN);
		}
		else
		{
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_RED, ModEffectArgs::EffectStateType::STATE_FLASH_FAST);
		}
		m_hwOperation.pwr.switchesDisableAll();
		operationStateUpdateStates();
	}
	break;

	case OP_STATE_BALANCING:
	{
			// Keep switches/limits under Measurement control; only handle user-facing LED/state transitions here.
			const bool balActive = m_ptrPowerElecPackInfoConfig->bBalancingActive;
		if (balActive)
		{
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_BLUE, ModEffectArgs::EffectStateType::STATE_FLASH);
		}
		else
		{
			// Balancing has stopped -> return to normal LED policy via the main states.
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_BLUE, ModEffectArgs::EffectStateType::STATE_OFF);
			if (bmsState == BMS_CHARGE)
				operationStateSetNewState(OP_STATE_CHARGING);
			else
				operationStateSetNewState(OP_STATE_LOAD_ENABLED);
		}
		operationStateUpdateStates();
	}
	break;

	case OP_STATE_CHARGED:
	{
		static uint32_t _u32DebugPrintLastTick3 = 0;
		if (timerDelay1ms(&_u32DebugPrintLastTick3, kOpStateLogMs))
			InterfaceCommHandler::getInstance()->printToInterface("----- OP_STATE_CHARGED: bmsState: %s \r\n", bms_state_to_str(bmsState));

		handleChargerDisconnect(OP_STATE_LOAD_ENABLED);

		if (!m_hwOperation.pwrState.chargeDetected())
		{
			operationStateSetNewState(OP_STATE_INIT);
		}
		else
		{
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_GREEN, ModEffectArgs::EffectStateType::STATE_ON);
		}
	}
	break;

	case OP_STATE_FORCEON:
	{
		static uint32_t _u32DebugPrintLastTick20 = 0;
		if (timerDelay1ms(&_u32DebugPrintLastTick20, kOpStateLogMs))
			InterfaceCommHandler::getInstance()->printToInterface("----- OP_STATE_FORCEON: bmsState: %s \r\n", bms_state_to_str(bmsState));

		if (m_hwOperation.pwr.setDisCharge(true))
		{
			m_hwOperation.pwr.setPreCharge(false);
		}
		else
		{
			operationStateSetNewState(OP_STATE_PRE_CHARGE);
			m_hwOperation.pwr.setDisCharge(false);
		}

		if (std::abs(moduleCurrent) >= m_ptrGenConfig->u16NotUsedCurrentThreshold)
		{
			if (timerDelay1ms(&m_u32StateNotUsedResetDelay, 1000))
				m_u32StateNotUsedTime = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
		}
		else
		{
			m_u32StateNotUsedResetDelay = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
		}

		if (timerDelay1ms(&m_u32StateNotUsedTime, m_ptrGenConfig->u16TimeoutNotUsed * 60 * 1000))
			operationStateSetNewState(OP_STATE_POWER_DOWN);
		m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_BLUE, ModEffectArgs::EffectStateType::STATE_FLASH);
		operationStateUpdateStates();
	}
	break;

	case OP_STATE_POWER_DOWN:
	{
		static uint32_t _u32DebugPrintLastTick40 = 0;
		if (timerDelay1ms(&_u32DebugPrintLastTick40, kOpStateLogMs))
			InterfaceCommHandler::getInstance()->printToInterface("----- OP_STATE_POWER_DOWN: bmsState: %s, timer=%lu ms -----\r\n", 
				bms_state_to_str(bmsState), 
				(unsigned long)(OPERATION_GET_TICK(m_opManagerTim) - m_u32StatePSPDisableDelay));

		if (m_operationalStateLastState != m_operationalStateCurrentState)
		{
			// InterfaceCommHandler::getInstance()->printToInterface("*** ENTERING POWER_DOWN from state %u ***\r\n", 
			// 	(unsigned)m_operationalStateLastState);
			g_packInfoMutex.lock();
			m_ptrPowerElecPackInfoConfig->bSaveSocToEepromRequest = true;
			g_packInfoMutex.unlock();
			m_u32StatePSPDisableDelay = (uint32_t)(OPERATION_GET_TICK(m_opManagerTim));
			
			// Update states ONLY on entry to sync lastState = currentState
			operationStateUpdateStates();
		}

		// Only keep alerting during power-down dwell for chipset comm loss; otherwise retain legacy silence policy
		const bool comm_fault_active = (m_ptrPowerElecPackInfoConfig->packBatteryWarErrState == SYS_ERROR_CHIPSET_COMM);
		if (!comm_fault_active)
		{
			m_hwOperation.effect.ledsDisableAll();
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::BUZZER, ModEffectArgs::EffectStateType::STATE_RESET);
		}
		else
		{
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::RGB_RED, ModEffectArgs::EffectStateType::STATE_ON);
			g_packInfoMutex.lock();
			m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = true;
			m_ptrPowerElecPackInfoConfig->buzzerSignalType = BUZZER_SIGNAL_TYPE_ON;
			g_packInfoMutex.unlock();
		}
		
		// Disable everything
		m_hwOperation.pwr.switchesDisableAll();
		g_packInfoMutex.lock();
		m_ptrPowerElecPackInfoConfig->bPushedShutdownChipSet = true;
		m_ptrPowerElecPackInfoConfig->bPredischargeDesiredFlag = false;
		m_ptrPowerElecPackInfoConfig->bDischargeDesiredFlag = false;
		m_ptrPowerElecPackInfoConfig->bChargeDesiredFlag = false;
		g_packInfoMutex.unlock();

		if (timerDelay1ms(&m_u32StatePSPDisableDelay, 2000))
		{
			// Now silence indicators just before cutting power
			g_packInfoMutex.lock();
			m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState = false;
			m_ptrPowerElecPackInfoConfig->buzzerSignalType = BUZZER_SIGNAL_TYPE_OFF;
			m_ptrPowerElecPackInfoConfig->buzzerSignalSource = BUZZER_SOURCE_OFF;
			g_packInfoMutex.unlock();
			m_hwOperation.effect.ledsDisableAll();
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::BUZZER, ModEffectArgs::EffectStateType::STATE_RESET);
			// InterfaceCommHandler::getInstance()->printToInterface("*** TERMINATING - releasing PSU latch ***\r\n");
			operationStateTerminateOperation();
		}
	}
	break;

	default:
		operationStateSetAllStates(OP_STATE_ERROR);
		break;
	}

	// Power button LED policy:
	// - Blink at 1 Hz on selected BMS fault flags (UV/OV/OC + chipset/comm)
	// - Solid ON when there is no active fault
	{
		static uint32_t s_btnLedBlinkTick = 0;
		static uint8_t s_btnLedBlinkPhase = 0;
		static bool s_faultActivePrev = false;
		enBatteryErrors errState = SYS_OK;
		enBqErrorType chipsetErrState = NO_ERROR;
		bool balancingActive = false;

		g_packInfoMutex.lock();
		errState = m_ptrPowerElecPackInfoConfig->packBatteryWarErrState;
		chipsetErrState = m_ptrPowerElecPackInfoConfig->chipsetErrorState;
		balancingActive = m_ptrPowerElecPackInfoConfig->bBalancingActive;
		g_packInfoMutex.unlock();

		const bool faultByErrorState =
			((errState == SYS_ERROR_CELL_UNDER_VOLTAGE) && !balancingActive) ||
			(errState == SYS_ERROR_CELL_OVER_VOLTAGE) ||
			(errState == SYS_ERROR_CHG_OVER_CURRENT) ||
			(errState == SYS_ERROR_DCHG_OVER_CURRENT) ||
			(errState == SYS_ERROR_SHORT_CIRCUIT) ||
			(errState == SYS_ERROR_CHIPSET) ||
			(errState == SYS_ERROR_CHIPSET_COMM);
		const bool faultActive = faultByErrorState || (chipsetErrState != NO_ERROR);

		if (!faultActive)
		{
			s_faultActivePrev = false;
			s_btnLedBlinkPhase = 0;
			m_hwOperation.pwrState.setPwrBtnLedPin(PIN_STATE_ENABLE);
		}
		else
		{
			if (!s_faultActivePrev)
			{
				s_btnLedBlinkTick = OPERATION_GET_TICK(m_opManagerTim);
				s_btnLedBlinkPhase = 1u;
				m_hwOperation.pwrState.setPwrBtnLedPin(PIN_STATE_ENABLE);
			}
			else if (timerDelay1ms(&s_btnLedBlinkTick, 500))
			{
				// Blink at 1 Hz (toggle every 500 ms)
				s_btnLedBlinkPhase ^= 1u;
				m_hwOperation.pwrState.setPwrBtnLedPin(s_btnLedBlinkPhase ? PIN_STATE_ENABLE : PIN_STATE_DISABLE);
			}
			s_faultActivePrev = true;
		}
	}

	// Power button → shutdown
#if defined(IS_DEVELOPMENT)
	if (m_hwOperation.pwrState.buttonForceOnRequest())
	{
		m_bOpStateForceOn = true;
		m_hwOperation.pwr.allowedForceOn(true);
		operationStateSetNewState(OP_STATE_PRE_CHARGE);
	}
#endif

	operationBuzzerCallback();
	operationModuleProtectionCallback();
}

// #############################################################################################//

void Operation::operationStateUpdateStates(void)
{
	m_operationalStateLastState = m_operationalStateCurrentState;
	m_operationalStateCurrentState = m_operationalStateNewState;
	g_packInfoMutex.lock();
	m_ptrPowerElecPackInfoConfig->u8operationState = m_operationalStateNewState;
	// Snapshot to SD on state change (logged by Logging thread)
	m_ptrPowerElecPackInfoConfig->u8SystemInformationLoggingState = INTTERFACE_BOOL_TRUE;
	g_packInfoMutex.unlock();
}

// #############################################################################################//

void Operation::operationStateSetAllStates(enOpStateType newState)
{
	m_operationalStateLastState = m_operationalStateCurrentState = m_operationalStateNewState = newState;
}

// #############################################################################################//

void Operation::operationStateSetNewState(enOpStateType newState)
{
	m_operationalStateNewState = newState;
}

// #############################################################################################//

void Operation::operationStateTerminateOperation(void)
{
	m_hwOperation.pwrState.setPwrBtnEnPin(PIN_STATE_DISABLE);
}

// #############################################################################################//

enOpStateType Operation::operationStateGetState(void)
{
	return m_operationalStateCurrentState;
}

// #####################################################################################################################################################################################################################

void Operation::handleChargerDisconnect(enOpStateType newState)
{
	const uint32_t now_ms = OPERATION_GET_TICK(m_opManagerTim);

	// Arm a 5 s guard once per CHARGING session to ignore spurious "disconnect" right after switching to CHARGING.
	static uint32_t guard_until_ms = 0;

	// When we're in CHARGING and no guard is armed, arm it and prime the timer.
	if (guard_until_ms == 0 && m_operationalStateCurrentState == OP_STATE_CHARGING)
	{
		guard_until_ms = now_ms + 5000U;				 // 5 s guard on entry
		m_u32StateChargerDisconnectDetectDelay = now_ms; // prime once on entry
	}

	// During guard window, ignore disconnect decisions.
	if (now_ms < guard_until_ms)
	{
		return;
	}

	// While the BMS reports CHARGE we keep deferring the disconnect timer.
	enBmsState bmsState = BMS_IDLE;
	g_packInfoMutex.lock();
	bmsState = m_ptrPowerElecPackInfoConfig->bmsState;
	g_packInfoMutex.unlock();
	if (bmsState == BMS_CHARGE)
	{
		m_u32StateChargerDisconnectDetectDelay = now_ms;
		return;
	}

	// Not in CHARGE: once enough time has passed since we last saw charge, declare disconnected.
	const uint32_t elapsed = now_ms - m_u32StateChargerDisconnectDetectDelay;
	if (elapsed > m_ptrGenConfig->u16TimeoutChargerDisconnected)
	{
		InterfaceCommHandler::getInstance()->printToInterface(
			"----- Charger disconnected (elapsed=%lu ms, timeout=%u ms) → state: %d\r\n",
			(unsigned long)elapsed, m_ptrGenConfig->u16TimeoutChargerDisconnected, newState);
		// Reset guard so next time we enter CHARGING we get a fresh guard window
		guard_until_ms = 0;
		operationStateSetNewState(newState);
	}
}

// #####################################################################################################################################################################################################################

void Operation::operationBuzzerCallback(void)
{
	if (m_operationalStateCurrentState == OP_STATE_POWER_DOWN)
	{
		m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::BUZZER, ModEffectArgs::EffectStateType::STATE_RESET);
		return;
	}
	static uint32_t _u32BuzzerUpdateIntervalLastTick = 0;
	// update buzzer state every second
	if (timerDelay1ms(&_u32BuzzerUpdateIntervalLastTick, 1000))
	{
		bool buzzerEnabled = false;
		enBuzzerSignalType buzzerType = BUZZER_SIGNAL_TYPE_OFF;
		g_packInfoMutex.lock();
		buzzerEnabled = m_ptrPowerElecPackInfoConfig->bBuzzerEnabledState;
		buzzerType = m_ptrPowerElecPackInfoConfig->buzzerSignalType;
		g_packInfoMutex.unlock();

		if (buzzerEnabled)
		{
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::BUZZER, (ModEffectArgs::EffectStateType)buzzerType);
		}
		else
		{
			// Always turn it off when disabled
			m_hwOperation.effect.effectChangeState(ModEffectArgs::EffectIdType::BUZZER, ModEffectArgs::EffectStateType::STATE_RESET);
		}
	}
}

// #####################################################################################################################################################################################################################

void Operation::operationModuleProtectionCallback(void)
{
	for (uint8_t i = 0; i < NoOfPROTS_POSSIBLE_ON_SYS; i++)
	{
		bool doSave = false;
		uint8_t counter = 0;
		g_packInfoMutex.lock();
		if (m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].bOperationProtectionActive)
		{
			m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].bOperationProtectionActive = false;
			counter = m_ptrPowerElecPackInfoConfig->moduleProtectionParams[i].u8ProtectionDetectionCounter;
			doSave = true;
		}
		g_packInfoMutex.unlock();
		if (doSave)
		{
			m_hwOperation.error.saveToModuleErrorCounterToEeprom((enModuleErrors)i, counter);
		}
	}
}
// #####################################################################################################################################################################################################################
