#ifndef __INVERTER_CAN_HANDLER_PARAMS__H_
#define __INVERTER_CAN_HANDLER_PARAMS__H_

#include "stdint.h"
#include <cstdint>
#include <stdint.h>

#define CANBUS_FREQUENCY                                    500000
#define CANBUS_DATA_SIZE                                    8

/* 0x4211: BMS sends live battery information frame */

#define DEFAULT_BATTERY_PILE_TOTAL_VOLTAGE                  2500      /* v */
#define DEFAULT_BATTERY_PILE_CURRENT                        30202     /* A  Offset : 3000, Res : 0.1 ->  (30201 = 20.1A) */ 
#define DEFAULT_BMS_TEMPERATURE                             1050       /* ᵒC */
#define DEFAULT_SOC                                         50        /* % */
#define DEFAULT_SOH                                         90        /* % */

/* 0x4221: BMS sends charge and discharge management frame */

#define DEFAULT_CHARGE_CUTOFF_VOLTAGE                       2920      /* v */
#define DEFAULT_DISCHARGE_CUTOFF_VOLTAGE                    1820      /* v */
#define DEFAULT_MAX_CHARGE_CURRENT                          30250      /* A */
#define DEFAULT_MAX_DISCHARGE_CURRENT                       30350      /* A */

/* 0x4231: BMS sends battery cell voltage information frame */

#define DEFAULT_MAX_SINGLE_BATTERY_CELL_VOLTAGE             4.2     /* v */
#define DEFAULT_MIN_SINGLE_BATTERY_CELL_VOLTAGE             3.5     /* v */
#define DEFAULT_MAX_SINGLE_BATTERY_CELL_VOLTAGE_NUMBER      15      /* A */
#define DEFAULT_MIN_SINGLE_BATTERY_CELL_VOLTAGE_NUMBER      28      /* A */

/* 0x4241: BMS sends battery cell temperature information frame */

#define DEFAULT_MAX_SINGLE_BATTERY_CELL_TEMPERATURE         25
#define DEFAULT_MIN_SINGLE_BATTERY_CELL_TEMPERATURE         15
#define DEFAULT_MAX_SINGLE_BATTERY_CELL_TEMPERATURE_NUMBER  00
#define DEFAULT_MIN_SINGLE_BATTERY_CELL_TEMPERATURE_NUMBER  00

/* 0x4261: BMS sends module voltage information frame */

#define DEFAULT_MODULE_MAX_VOLTAGE                          67.2
#define DEFAULT_MODULE_MIN_VOLTAGE                          56
#define DEFAULT_MODULE_MAX_VOLTAGE_NUMBER                   00
#define DEFAULT_MODULE_MIN_VOLTAGE_NUMBER                   00

/* 0x4271: BMS sends module temperature information frame */

#define DEFAULT_MODULE_Module_MAX_TEMPERATURE               28
#define DEFAULT_MODULE_Module_MIN_TEMPERATURE               15
#define DEFAULT_MODULE_Module_MAX_TEMPERATURE_NUMBER        00
#define DEFAULT_MODULE_Module_MIN_TEMPERATURE_NUMBER        00

/* 0x4281: BMS sends charge and discharge forbidden mark information frame */

#define DEFAULT_CHARGE_FORBIDDEN_MARK                       00
#define DEFAULT_DISCHARGE_FORBIDDEN_MARK                    00

/* 0x7311: BMS sends hardware and software version information frame */

#define DEFAULT_HARDWARE_VERSION                            00
#define DEFAULT_HARDWARE_VERSION_V                          00
#define DEFAULT_HARDWARE_VERSION_R                          00
#define DEFAULT_SOFTWARE_VERSION_V_MAJOR                    00
#define DEFAULT_SOFTWARE_VERSION_V_MINOR                    00
#define DEFAULT_DEVELOPMENT_MASTER_VERSION                  00
#define DEFAULT_DEVELOPMENT_SUB_VERSION                     00

/* 0x7321: BMS sends module information frame */

#define DEFAULT_BATTERY_MODULE_QTY                          5
#define DEFAULT_BATTERY_MODULE_IN_SERIES_QTY                5
#define DEFAULT_CELL_QTY_IN_BATTERY_MODULE                  16
#define DEFAULT_VOLTAGE_LEVEL                               250
#define DEFAULT_BATTERY_CAPACITY_AH                         54

/* 0x7331 & 0x7341 : BMS sends manufacturer name high frame */

#define DEFAULT_MANUFACTURER_NAME_HIGH                    "ORBITENERJI"

/* 0x8200: host sends sleep/awake command frame */

#define SLEEP_COMMAND                                     0X55
#define AWAKE_COMMAND                                     0XAA

/* 0x8210: host sends charge/discharge command frame */

#define CHARGE_COMMAND                                    0xAA
#define DISCHARGE_COMMAND                                 0xAA

/* 0x8240: host sends communication error frame */

#define EXTERNAL_COMMUNICATION_ERROR_MASK                 0xAA

/* 0x8251: BMS sends communication error condition frame */

#define System_condition                                  0xAA


#define CHARGE_FORBIDDEN_ENABLE                           0xAA
#define CHARGE_FORBIDDEN_DISABLE                          0x00


namespace InverterCanArgs
{
    typedef enum _inverterCanRegisterAddr : uint16_t
    {
        // Ensemble information
        ENSEMBLE_SYSTEM_EQUIPMENT_INFORMATION_COMMAND_FRAME_ID = 0x4200, // DATA 0 
        BATTERY_PILE_INFO_FRAME_ID                             = 0x4211,
        SYSTEM_LIMITS_FRAME_ID                                 = 0x4221,
        MIN_MAX_SINGLE_BATTERY_CELL_VOLTAGE_FRAME_ID           = 0x4231,
        MIN_MAX_SINGLE_BATTERY_CELL_TEMPERATURE_FRAME_ID       = 0x4241,
        SYSTEM_STATUS_FRAME_ID                                 = 0x4251,
        MIN_MAX_MODULE_VOLTAGE_FRAME_ID                        = 0x4261,
        MIN_MAX_MODULE_TEMPERATURE_FRAME_ID                    = 0x4271,
        CHARGE_DISCHARGE_FORBIDDEN_MARK_FRAME_ID               = 0x4281,
        ERROR_FRAME_ID                                         = 0x4291,
     
        // system equipment information  
        SOFTWARE_HARDWARE_VERSION_FRAME_ID                     = 0x7311,
        BATTERY_MODULE_INFO_FRAME_ID                           = 0x7321,
        NAME_FRAME_ID                                          = 0x7331,
        NAME_2_FRAME_ID                                        = 0x7341,
     
        // Control Command
        SLEEP_AWAKE_COMMAND_FRAME_ID                           = 0x8200, // No battery response
        CHARGE_DISCHARGE_COMMAND_FRAME_ID                      = 0x8210, // No battery response
        COMMUNICATION_ERROR_COMMAND_FRAME_ID                   = 0x8240, 
        SYSTEM_CONDITION                                       = 0x8251,
    
    }e_inverterCanRegisterAddr;

    typedef enum _ForbiddenMarkStatus : uint8_t
    {
        FORBIDDEN_ENABLE                       = ((uint8_t) 0xAA),
        FORBIDDEN_DISABLE                      = ((uint8_t) 0x00)
    }e_ForbiddenMarkStatus;

    #pragma pack(push)
    #pragma pack(1)

    // ####################################################################
    // #######################    0x4211    ###############################
    // ####################################################################

    typedef union _bmsSysVoltageCurrentTempSocSoh_FRAME_DATA
    {
        char arrData[8];
        struct
        {
            uint16_t u16SysVoltage;         // Battery Pile Total Voltage // Resolution: 0.1V // Offset: 0
            uint16_t u16SysCurrent;         // Battery Pile Current // Resolution: 0.1A // Offset: -3000A
            uint16_t u16Temp;               // second level BMS Temperature // Resolution: 0.1 ᵒC // Offset: -100 ᵒC
            uint8_t  u8Soc;                 // Resolution: 1% // Offset: 0
            uint8_t  u8Soh;                 // Resolution: 1% // Offset: 0
        };
    }ut_bmsSysVoltageCurrentTempSocSoh_FRAME_DATA;

    // ####################################################################
    // ########################   0x4221    ###############################
    // ####################################################################

    typedef union _bmsChargeDischargeCutoffVoltageCurrent_FRAME_DATA
    {
        char arrData[8];
        struct
        {
            uint16_t u16Charge_Cutoff_Voltage;      // Charge Cutoff Voltage    // Resolution: 0.1V // Offset: 0
            uint16_t u16Discharge_Cutoff_Voltage;   // Discharge Cutoff Voltage // Resolution: 0.1V // Offset: 0
            int16_t  u16Max_Charge_Current;         // Resolution: 0.1A // Offset: -3000A
            int16_t  u16Max_Discharge_Current;      // Resolution: 0.1A // Offset: -3000A
        };
    }ut_bmsChargeDischargeCutoffVoltageCurrent_FRAME_DATA;

    // ####################################################################
    // ########################   0x4231    ###############################
    // ####################################################################

    typedef union _MIN_MAX_SINGLE_BATTERY_CELL_VOLTAGE_FRAME_DATA
    {
        char arrData[8];
        struct
        {
            uint16_t u16Max_Single_Battery_Cell_Voltage;            // Resolution: 0.001V // Offset: 0
            uint16_t u16Min_Single_Battery_Cell_Voltage;            // Resolution: 0.001V // Offset: 0
            uint16_t u16Max_Single_Battery_Cell_Voltage_Number;     // Resolution: 1 // Offset: 0
            uint16_t u16Min_Single_Battery_Cell_Voltage_Number;     // Resolution: 1 // Offset: 0
        };
    }ut_MIN_MAX_SINGLE_BATTERY_CELL_VOLTAGE_FRAME_DATA;

    // ####################################################################
    // ########################   0x4241    ###############################
    // ####################################################################

    typedef union _MIN_MAX_SINGLE_BATTERY_CELL_Temperature_FRAME_DATA
    {
        char arrData[8];
        struct
        {
            uint16_t u16Max_Single_Battery_Cell_Temperature;            // Resolution: 0.1ᵒC // Offset: -100
            uint16_t u16Min_Single_Battery_Cell_Temperature;            // Resolution: 0.1 ᵒC // Offset: -100
            uint16_t u16Max_Single_Battery_Cell_Temperature_Number;     // Resolution: 1 // Offset: 0
            uint16_t u16Min_Single_Battery_Cell_Temperature_Number;     // Resolution: 1 // Offset: 0
        };
    }ut_MIN_MAX_SINGLE_BATTERY_CELL_Temperature_FRAME_DATA;

    // ####################################################################
    // ########################   0x4251    ###############################
    // ####################################################################

typedef union _SYSTEM_STATUS_FRAME_DATA
{
        char arrData[8];
        
        struct _bytes
        {
            union _Basic_Status
            {
                uint8_t u8Basic_Status_Byte;
                struct _Basic_Status_Bits
                {
                    uint8_t Reversed                    : 3; 
                    uint8_t Balance_charge_request      : 1; // 0：Null // 1 : Balance charge request
                    uint8_t Forced_charge_request       : 1; // 0：Null // 1 : Forced charge request
                    uint8_t Sleep_Charge_Discharge_Idle : 3; // 0: Sleep // 1: Charge // 2: Discharge // 3: Idle // 4-7: Reserve
                }st_Basic_Status_Bits;
            }ut_Basic_Status;

            uint16_t Cycle_Period;

            union _Fault_Status
            {
                uint8_t u8Fault_Byte;
                struct
                {
                    uint8_t Other_error    : 1;
                    uint8_t Battery_damage : 1; // caused by battery over-discharge, etc.
                    uint8_t RELAY_ERR      : 1; // Relay Check Error
                    uint8_t RV_ERR         : 1; // Input transposition Error
                    uint8_t DCOV_ERR       : 1; // Input Over Voltage Error
                    uint8_t IN_COMM_ERR    : 1; // Internal Communication Error
                    uint8_t TMPR_ERR       : 1; // Temperature Sensor Error
                    uint8_t VOLT_ERR       : 1; // Voltage Sensor Error
                }st_Fault_bits;
            }ut_Fault;

            union _Alarm
            {
                uint16_t u16_Alarm_Bytes;
                struct _Alarm_Bits
                {
                    uint8_t BLV     : 1; // Single Cell Low Voltage Alarm
                    uint8_t BHV     : 1; // Single Cell High Voltage Alarm
                    uint8_t PLV     : 1; // Discharge system Low Voltage Alarm
                    uint8_t PHV     : 1; // Charge system High Voltage Alarm
                    uint8_t CLT     : 1; // Charge Cell Low Temperature Alarm
                    uint8_t CHT     : 1; // Charge Cell High Temperature Alarm
                    uint8_t DLT     : 1; // Discharge Cell Low Temperature Alarm
                    uint8_t DHT     : 1; // Discharge Cell High Temperature Alarm
                    uint8_t COCA    : 1; // Charge Over Current Alarm
                    uint8_t DOCA    : 1; // Discharge Over Current Alarm
                    uint8_t MLV     : 1; // Module Low Voltage Alarm
                    uint8_t MHV     : 1; // Module High Voltage Alarm
                    uint8_t Reserve : 4; // Reserved
                }st_Alarm_Bits;
            }ut_Alarm;

            union _Protection
            {
                uint16_t u16_Protection_Bytes;
                struct _Protection_Bits
                {
                    uint8_t BUV     : 1; // Single Cell Under Voltage Protect (Bit0)
                    uint8_t BOV     : 1; // Single Cell Over Voltage Protect (Bit1)
                    uint8_t PUV     : 1; // Discharge system Under Voltage Protect (Bit2)
                    uint8_t POV     : 1; // Charge system Over Voltage Protect (Bit3)
                    uint8_t CUT     : 1; // Charge Cell Under Temperature Protect (Bit4)
                    uint8_t COT     : 1; // Charge Cell Over Temperature Protect (Bit5)
                    uint8_t DUT     : 1; // Discharge Cell Under Temperature Protect (Bit6)
                    uint8_t DOT     : 1; // Discharge Cell Over Temperature Protect (Bit7)
                    uint8_t COC     : 1; // Charge Over Current Protect (Bit8)
                    uint8_t DOC     : 1; // Discharge Over Current Protect (Bit9)
                    uint8_t MUV     : 1; // Module Under Voltage Protect (Bit10)
                    uint8_t MOV     : 1; // Module Over Voltage Protect (Bit11)
                    uint8_t Reserve : 4; // Reserved (Bit12-15)
                }st_Protection_Bits;
            }ut_Protection;
        }st_bytes;
    }ut_SYSTEM_STATUS_FRAME_DATA;

    typedef union _INVERTER_PROTECT_SYS
    {
        uint16_t bytes;
        struct
        {
            uint16_t u8bitDisChargeHighCurrentArrive : 1;
            uint16_t u8bitChargeHighCurrentArrive : 1;
            uint16_t u8bitShortCircuitArrive : 1;
            uint16_t u8bitLowVoltageArrive : 1;
            uint16_t u8bitHighVoltageArrive : 1;
            uint16_t u8bitChargeLowTempArrive : 1;
            uint16_t u8bitChargeHighTempArrive : 1;
            uint16_t u8bitDisChargeLowTempArrive : 1;
            uint16_t u8bitDisChargeHighTempArrive : 1;
            uint16_t u8bitInternalArrive : 1;
            uint16_t u8bitImbalanceArrive : 1;
            uint16_t u8bitImbalanceLeave : 1;
            uint16_t reserved : 4;
        };
    }ut_INVERTER_PROTECT_SYS;

    typedef struct _INVERTER_PROTECT_WARNING
    {
        ut_INVERTER_PROTECT_SYS sysProtect;
    }st_INVERTER_PROTECT_WARNING;

    // ####################################################################
    // ########################   0x4261    ###############################
    // ####################################################################

    typedef union _MIN_MAX_MODULE_VOLATGE_FRAME_DATA
    {
        char arrData[8];
        struct
        {
            uint16_t u16Module_Max_Voltage;            // Resolution: 0.001V // Offset: 0
            uint16_t u16Module_Min_Voltage;            // Resolution: 0.001V // Offset: 0
            uint16_t u16Module_Max_Voltage_Number;     // Resolution: 1 // Offset: 0
            uint16_t u16Module_Min_Voltage_Number;     // Resolution: 1 // Offset: 0
        };
    }ut_MIN_MAX_MODULE_VOLATGE_FRAME_DATA;

    // ####################################################################
    // ########################   0x4271    ###############################
    // ####################################################################

    typedef union _MIN_MAX_MODULE_TEMPERATURE_FRAME_DATA
    {
        char arrData[8];
        struct
        {
            uint16_t u16Module_Max_Temperature;            // Resolution: 0.1ᵒC // Offset: -100
            uint16_t u16Module_Min_Temperature;            // Resolution: 0.1ᵒCe // Offset: -100
            uint16_t u16Module_Max_Temperature_Number;     // Resolution: 1 // Offset: 0
            uint16_t u16Module_Min_Temperature_Number;     // Resolution: 1 // Offset: 0
        };
    }ut_MIN_MAX_MODULE_TEMPERATURE_FRAME_DATA;


    // ####################################################################
    // ########################   0x4281    ###############################
    // ####################################################################

    typedef union _CHAREGE_DISCHARGE_FORBIDDEN_MARK_FRAME_DATA
    {
        char arrData[8];
        struct
        {
            uint8_t Charge_forbidden_mark;    // 0xAA：effect; other value: NULL
            uint8_t Discharge_forbidden_mark; // 0xAA：effect；other value: NULL
            uint8_t Reserved1;
            uint8_t Reserved2;
            uint8_t Reserved3;
            uint8_t Reserved4;
            uint8_t Reserved5;
            uint8_t Reserved6;
        };
    }ut_CHAREGE_DISCHARGE_FORBIDDEN_MARK_FRAME_DATA;

    // ####################################################################
    // ########################   0x4291    ###############################
    // ####################################################################

    typedef union _ABNORMALITY_FRAME_DATA
    {
        char arrData[8];

        struct _bytes
        {
            typedef union _Fault_Extension
            {
                uint8_t u8Fault_Extension_Byte;
                struct _Fault_Extension_Bits
                {
                    uint8_t Reversed                        : 3;
                    uint8_t Security_Function_Abnormality   : 1;
                    uint8_t POST_Abnormality                : 1;
                    uint8_t Internal_Bus_Abnormality        : 1;
                    uint8_t BMIC_abnormal                   : 1;
                    uint8_t Shutdown_Circuit_abnormality    : 1;
                }st_Fault_Extension_Bits;

            }ut_Basic_Status;

            uint8_t Reserved2;
            uint8_t Reserved3;
            uint8_t Reserved4;
            uint8_t Reserved5;
            uint8_t Reserved6;
            uint8_t Reserved7;
            uint8_t Reserved8;

        }st_bytes;
    }ut_ABNORMALITY_FRAME_DATA;

    // ####################################################################
    // ########################   0x7311    ###############################
    // ####################################################################

    typedef union _HARDWARE_SOFTWARE_VERSION_FRAME_DATA
    {
        char arrData[8];
        struct 
        {
            uint8_t Hardware_version;           // 0: Null; 1: ver. A; 2: ver. B; Others: Reserve.
            uint8_t Reserved;
            uint8_t Hardware_Version_V;         // 0x02
            uint8_t Hardware_Version_R;         // 0x01
            uint8_t Software_Version_V_Major;   // 0x01
            uint8_t Software_Version_V_Minor;   // 0x02
            uint8_t Development_Master_Version;
            uint8_t Development_Sub_Version;
        };

    }ut_HARDWARE_SOFTWARE_VERSION_FRAME_DATA;

    // ####################################################################
    // ########################   0x7321    ###############################
    // ####################################################################

    typedef union _BATTERY_MODULE_INFO_FRAME_DATA
    {
        char arrData[8];
        struct
        {
            uint16_t Battery_Module_Qty;
            uint8_t  Battery_Module_In_Series_Qty;
            uint8_t  Cell_Qty_In_Battery_Module;
            uint16_t Voltage_Level; // Resolution: 1V // Offset: 0
            uint16_t AH_number;  // Resolution: 1AH // Offset: 0
        };

    }ut_BATTERY_MODULE_INFO_FRAME_DATA;

    // ####################################################################
    // ########################   0x7331    ###############################
    // ####################################################################

    typedef union _MANUFACTURER_NAME_HIGH_FRAME_DATA
    {
        char arrData[8];
        struct _bytes
        {
            uint8_t Letter1;
            uint8_t Letter2;
            uint8_t Letter3;
            uint8_t Letter4;
            uint8_t Letter5;
            uint8_t Letter6;
            uint8_t Letter7;
            uint8_t Letter8;
        }st_bytes;

    }ut_MANUFACTURER_NAME_HIGH_FRAME_DATA;

    // ####################################################################
    // ########################   0x7341    ###############################
    // ####################################################################

    typedef union _MANUFACTURER_NAME_LOW_FRAME_DATA
    {
        char arrData[8];
        struct _bytes
        {
            uint8_t Letter1;
            uint8_t Letter2;
            uint8_t Letter3;
            uint8_t Letter4;
            uint8_t Letter5;
            uint8_t Letter6;
            uint8_t Letter7;
            uint8_t Letter8;
        }st_bytes;

    }ut_MANUFACTURER_NAME_LOW_FRAME_DATA;

    // ####################################################################
    // ########################   0x8200    ###############################
    // ####################################################################

    typedef union _SLEEP_AWAKE_COMMAND_FRAME_DATA
    {
        char arrData[8];
        struct _bytes
        {
            uint8_t Sleep_Awake_Control; // 0x55 : Control device enter sleep status // 0xAA: Control device quit sleep status // Others: Null
            uint8_t Reserved2;
            uint8_t Reserved3;
            uint8_t Reserved4;
            uint8_t Reserved5;
            uint8_t Reserved6;
            uint8_t Reserved7;
            uint8_t Reserved8;
        }st_bytes;

    }ut_SLEEP_AWAKE_COMMAND_FRAME_DATA;


    // ####################################################################
    // ########################   0x8210    ###############################
    // ####################################################################

    typedef union _CHARGE_DISCHARGE_COMMAND_FRAME_DATA
    {
        char arrData[8];
        struct _bytes
        {
            uint8_t Charge_Command;     // 0xAA: effect; Others: Null
            uint8_t Discharge_Command;  //0xAA: effect; Others: Null
            uint8_t Reserved3;
            uint8_t Reserved4;
            uint8_t Reserved5;
            uint8_t Reserved6;
            uint8_t Reserved7;
            uint8_t Reserved8;
        }st_bytes;

    }ut_CHARGE_DISCHARGE_COMMAND_FRAME_DATA;

    // ####################################################################
    // ########################   0x8240    ###############################
    // ####################################################################

    typedef union _COMMUNICATION_ERROR_COMMAND_FRAME_DATA
    {
        char arrData[8];
        struct _bytes
        {
            uint8_t BMS_masking_external_communication_error;   // 0xAA: effect; Others: Null
            uint8_t Reserved2;
            uint8_t Reserved3;
            uint8_t Reserved4;
            uint8_t Reserved5;
            uint8_t Reserved6;
            uint8_t Reserved7;
            uint8_t Reserved8;
        }st_bytes;

    }ut_COMMUNICATION_ERROR_COMMAND_FRAME_DATA;

    // ####################################################################
    // ########################   0x8251    ###############################
    // ####################################################################

    // typedef union _SYSTEM_CONDITION_FRAME_DATA
    // {
    //     char arrData[8];
    //     struct _bytes
    //     {
    //         uint8_t System_condition_able_to_act_this_command_or_not; // 0xAA ; OK, will act this commend immediately Others：; won`t act this command
    //         uint8_t Reserved2;
    //         uint8_t Reserved3;
    //         uint8_t Reserved4;
    //         uint8_t Reserved5;
    //         uint8_t Reserved6;
    //         uint8_t Reserved7;
    //         uint8_t Reserved8;
    //     }st_bytes;

    // }ut_SYSTEM_CONDITION_FRAME_DATA;

typedef union
{
    char arrData[8];

    struct PACKED
    {
        uint8_t  basicStatus;     // Byte0
        uint16_t cyclePeriod;     // Byte1-2 (endianness matters)
        uint8_t  error;           // Byte3
        uint16_t alarm;           // Byte4-5 (endianness matters)

        union
        {
            uint16_t proWord;     // Byte6-7
            struct
            {
                uint8_t BUV     : 1; // Bit0
                uint8_t BOV     : 1; // Bit1
                uint8_t PUV     : 1; // Bit2
                uint8_t POV     : 1; // Bit3
                uint8_t CUT     : 1; // Bit4
                uint8_t COT     : 1; // Bit5
                uint8_t DUT     : 1; // Bit6
                uint8_t DOT     : 1; // Bit7
                uint8_t COC     : 1; // Bit8
                uint8_t DOC     : 1; // Bit9
                uint8_t MUV     : 1; // Bit10
                uint8_t MOV     : 1; // Bit11
                uint8_t Reserve : 4; // Bit12-15
            } proBits;
        } protections;

    } byte;

} ut_SYSTEM_CONDITION_FRAME_DATA;
    

    #pragma pack(pop)
}

#endif
