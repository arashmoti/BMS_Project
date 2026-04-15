#ifndef __MEASUREMENT_PARAMS__H_
#define __MEASUREMENT_PARAMS__H_

#include <cstdint>


#define MEASUREMENT_GET_TICK(x)                     std::chrono::duration_cast<std::chrono::milliseconds>(x.elapsed_time()).count() 
#define MEASUREMENT_GET_TICK_US(x)                  std::chrono::duration_cast<std::chrono::microseconds>(x.elapsed_time()).count() 

namespace MeasurementArgs {

// SOC tuning parameters
constexpr int32_t  SOC_REST_CURRENT_THRESHOLD_MA = 1080;        // ~0.02C for 54Ah
constexpr uint32_t SOC_REST_SETTLE_MS            = 5 * 60 * 1000; // 5 minutes voltage settle for OCV
constexpr uint16_t SOC_VOLTAGE_STABLE_MV         = 10;            // consider Vmin stable if drift <= 10 mV
constexpr int32_t  SOC_RECAL_IDLE_DEADBAND_MA    = 500;
constexpr uint32_t SOC_RECAL_REST_STABLE_MS      = 10u * 60u * 1000u;
constexpr float    SOC_RECAL_OCV_CONFIDENCE_MIN  = 0.95f;
constexpr float    SOC_RECAL_THROUGHPUT_FRACTION = 0.05f;
constexpr uint32_t SOC_RECAL_COOLDOWN_MS         = 6u * 60u * 60u * 1000u;

constexpr uint32_t SOC_BOOTSTRAP_UPTIME_MS       = 10u * 1000u;
constexpr int32_t  SOC_BOOTSTRAP_IDLE_MA         = 500;
constexpr uint16_t SOC_BOOTSTRAP_DV_MV           = 30;

constexpr int32_t  SOC_OFFSET_IDLE_CURRENT_MA    = 100;
constexpr uint32_t SOC_OFFSET_IDLE_STABLE_MS     = 5u * 60u * 1000u;
constexpr int32_t  SOC_OFFSET_MAX_MA             = 200;

constexpr float    SOC_CHARGE_EFF                = 0.98f;
constexpr float    SOC_DISCHARGE_EFF             = 1.0f;

constexpr uint16_t SOC_OCV_V_LOW_KNEE_MV         = 3230;
constexpr uint16_t SOC_OCV_V_PLATEAU_LOW_MV      = 3320;
constexpr uint16_t SOC_OCV_V_PLATEAU_HIGH_MV     = 3340;
constexpr uint16_t SOC_OCV_V_HIGH_KNEE_MV        = 3400;
constexpr int32_t  SOC_TRIM_MAX_STEP_X10         = 20; // ±2% in x0.1% units (was 50)
constexpr int32_t  SOC_RECAL_MAX_STEP_X10        = 50; // max OCV recal step when SOC is known (±5.0%)


#pragma pack(push) 
#pragma pack(1) 

#pragma pack(pop)

enum e_measurementStateType : uint8_t
{
    IDLE_MAIN = 1,
    INITIALIZE_PARAMETERS,
    INTEFACE_COMMS_ACTIVE,
    INTERFACE_HANDLER_MESSAGES
    
};

}

#endif // __MEASUREMENT_PARAMS__H_
