#ifndef __BQ79616_HANDLER_PARAMS__H_
#define __BQ79616_HANDLER_PARAMS__H_

#include <cstdint> // For uint8_t
#include <vector>  // For std::vector

/********************************************************************/

#define BQ_GET_TICK(x)                     std::chrono::duration_cast<std::chrono::milliseconds>(x.elapsed_time()).count() 
#define BQ_GET_TICK_US(x)                  std::chrono::duration_cast<std::chrono::microseconds>(x.elapsed_time()).count()

// register addresses of battery IC bQ79616
#define DIR0_ADDR_OTP                   0x0
#define DIR1_ADDR_OTP                   0x1
#define DEV_CONF                        0x2
#define ACTIVE_CELL                     0x3
#define BBVC_POSN1                      0x5
#define BBVC_POSN2                      0x6
#define ADC_CONF1                       0x7
#define ADC_CONF2                       0x8
#define OV_THRESH                       0x9
#define UV_THRESH                       0xA
#define OTUT_THRESH                     0xB
#define UV_DISABLE1                     0xC
#define UV_DISABLE2                     0xD
#define GPIO_CONF1                      0xE
#define GPIO_CONF2                      0xF
#define GPIO_CONF3                      0x10
#define GPIO_CONF4                      0x11
#define OTP_SPARE14                     0x12
#define OTP_SPARE13                     0x13
#define OTP_SPARE12                     0x14
#define OTP_SPARE11                     0x15
#define FAULT_MSK1                      0x16
#define FAULT_MSK2                      0x17
#define PWR_TRANSIT_CONF                0x18
#define COMM_TIMEOUT_CONF               0x19
#define TX_HOLD_OFF                     0x1A
#define MAIN_ADC_CAL1                   0x1B
#define MAIN_ADC_CAL2                   0x1C
#define AUX_ADC_CAL1                    0x1D
#define AUX_ADC_CAL2                    0x1E
#define OTP_RSVD1F                      0x1F
#define OTP_RSVD20                      0x20
#define CUST_MISC1                      0x21
#define CUST_MISC2                      0x22
#define CUST_MISC3                      0x23
#define CUST_MISC4                      0x24
#define CUST_MISC5                      0x25
#define CUST_MISC6                      0x26
#define CUST_MISC7                      0x27
#define CUST_MISC8                      0x28
#define STACK_RESPONSE                  0x29
#define BBP_LOC                         0x2A
#define OTP_RSVD_2B                     0x2B
#define OTP_SPARE_10                    0x2C
#define OTP_SPARE_9                     0x2D
#define OTP_SPARE_8                     0x2E
#define OTP_SPARE_7                     0x2F
#define OTP_SPARE_6                     0x30
#define OTP_SPARE_5                     0x31
#define OTP_SPARE_4                     0x32
#define OTP_SPARE_3                     0x33
#define OTP_SPARE_2                     0x34
#define OTP_SPARE_1                     0x35
#define CUST_CRC_HI                     0x36
#define CUST_CRC_LO                     0x37
#define OTP_PROG_UNLOCK1A               0x300
#define OTP_PROG_UNLOCK1B               0x301
#define OTP_PROG_UNLOCK1C               0x302
#define OTP_PROG_UNLOCK1D               0x303
#define DIR0_ADDR                       0x306
#define DIR1_ADDR                       0x307
#define COMM_CTRL                       0x308
#define CONTROL1                        0x309
#define CONTROL2                        0x30A
#define OTP_PROG_CTRL                   0x30B
#define ADC_CTRL1                       0x30D
#define ADC_CTRL2                       0x30E
#define ADC_CTRL3                       0x30F
#define REG_INT_RSVD                    0x310
#define CB_CELL16_CTRL                  0x318
#define CB_CELL15_CTRL                  0x319
#define CB_CELL14_CTRL                  0x31A
#define CB_CELL13_CTRL                  0x31B
#define CB_CELL12_CTRL                  0x31C
#define CB_CELL11_CTRL                  0x31D
#define CB_CELL10_CTRL                  0x31E
#define CB_CELL9_CTRL                   0x31F
#define CB_CELL8_CTRL                   0x320
#define CB_CELL7_CTRL                   0x321
#define CB_CELL6_CTRL                   0x322
#define CB_CELL5_CTRL                   0x323
#define CB_CELL4_CTRL                   0x324
#define CB_CELL3_CTRL                   0x325
#define CB_CELL2_CTRL                   0x326
#define CB_CELL1_CTRL                   0x327
#define VMB_DONE_THRESH                 0x328
#define MB_TIMER_CTRL                   0x329
#define VCB_DONE_THRESH                 0x32A
#define OTCB_THRESH                     0x32B
#define OVUV_CTRL                       0x32C
#define OTUT_CTRL                       0x32D
#define BAL_CTRL1                       0x32E
#define BAL_CTRL2                       0x32F
#define BAL_CTRL3                       0x330
#define FAULT_RST1                      0x331
#define FAULT_RST2                      0x332
#define DIAG_OTP_CTRL                   0x335
#define DIAG_COMM_CTRL                  0x336
#define DIAG_PWR_CTRL                   0x337
#define DIAG_CBFET_CTRL1                0x338
#define DIAG_CBFET_CTRL2                0x339
#define DIAG_COMP_CTRL1                 0x33A
#define DIAG_COMP_CTRL2                 0x33B
#define DIAG_COMP_CTRL3                 0x33C
#define DIAG_COMP_CTRL4                 0x33D
#define DIAG_PROT_CTRL                  0x33E
#define OTP_ECC_DATAIN1                 0x343
#define OTP_ECC_DATAIN2                 0x344
#define OTP_ECC_DATAIN3                 0x345
#define OTP_ECC_DATAIN4                 0x346
#define OTP_ECC_DATAIN5                 0x347
#define OTP_ECC_DATAIN6                 0x348
#define OTP_ECC_DATAIN7                 0x349
#define OTP_ECC_DATAIN8                 0x34A
#define OTP_ECC_DATAIN9                 0x34B
#define OTP_ECC_TEST                    0x34C
#define SPI_CONF                        0x34D
#define SPI_TX3                         0x34E
#define SPI_TX2                         0x34F
#define SPI_TX1                         0x350
#define SPI_EXE                         0x351
#define OTP_PROG_UNLOCK2A               0x352
#define OTP_PROG_UNLOCK2B               0x353
#define OTP_PROG_UNLOCK2C               0x354
#define OTP_PROG_UNLOCK2D               0x355
#define DEBUG_CTRL_UNLOCK               0x700
#define DEBUG_COMM_CTRL1                0x701
#define DEBUG_COMM_CTRL2                0x702
#define PARTID                          0x500
#define DEV_REVID                       0xE00
#define DIE_ID1                         0x501
#define DIE_ID2                         0x502
#define DIE_ID3                         0x503
#define DIE_ID4                         0x504
#define DIE_ID5                         0x505
#define CURRENT_HI                      0x506   // DIE_ID6
#define CURRENT_MID                     0x507   // DIE_ID7
#define CURRENT_LO                      0x508   // DIE_ID8
#define DIE_ID9                         0x509
#define CUST_CRC_RSLT_HI                0x50C
#define CUST_CRC_RSLT_LO                0x50D
#define OTP_ECC_DATAOUT1                0x510
#define OTP_ECC_DATAOUT2                0x511
#define OTP_ECC_DATAOUT3                0x512
#define OTP_ECC_DATAOUT4                0x513
#define OTP_ECC_DATAOUT5                0x514
#define OTP_ECC_DATAOUT6                0x515
#define OTP_ECC_DATAOUT7                0x516
#define OTP_ECC_DATAOUT8                0x517
#define OTP_ECC_DATAOUT9                0x518
#define OTP_PROG_STAT                   0x519
#define OTP_CUST1_STAT                  0x51A
#define OTP_CUST2_STAT                  0x51B
#define SPI_RX3                         0x520
#define SPI_RX2                         0x521
#define SPI_RX1                         0x522
#define DIAG_STAT                       0x526
#define ADC_STAT1                       0x527
#define ADC_STAT2                       0x528
#define GPIO_STAT                       0x52A
#define BAL_STAT                        0x52B
#define DEV_STAT                        0x52C
#define FAULT_SUMMARY                   0x52D
#define FAULT_COMM1                     0x530
#define FAULT_COMM2                     0x531
#define FAULT_COMM3                     0x532
#define FAULT_OTP                       0x535
#define FAULT_SYS                       0x536
#define FAULT_PROT1                     0x53A
#define FAULT_PROT2                     0x53B
#define FAULT_OV1                       0x53C
#define FAULT_OV2                       0x53D
#define FAULT_UV1                       0x53E
#define FAULT_UV2                       0x53F
#define FAULT_OT                        0x540
#define FAULT_UT                        0x541
#define FAULT_COMP_GPIO                 0x543
#define FAULT_COMP_VCCB1                0x545
#define FAULT_COMP_VCCB2                0x546
#define FAULT_COMP_VCOW1                0x548
#define FAULT_COMP_VCOW2                0x549
#define FAULT_COMP_CBOW1                0x54B
#define FAULT_COMP_CBOW2                0x54C
#define FAULT_COMP_CBFET1               0x54E
#define FAULT_COMP_CBFET2               0x54F
#define FAULT_COMP_MISC                 0x550
#define FAULT_PWR1                      0x552
#define FAULT_PWR2                      0x553
#define FAULT_PWR3                      0x554
#define CB_COMPLETE1                    0x556
#define CB_COMPLETE2                    0x557
#define BAL_TIME                        0x558
#define VCELL16_HI                      0x568
#define VCELL16_LO                      0x569
#define VCELL15_HI                      0x56A
#define VCELL15_LO                      0x56B
#define VCELL14_HI                      0x56C
#define VCELL14_LO                      0x56D
#define VCELL13_HI                      0x56E
#define VCELL13_LO                      0x56F
#define VCELL12_HI                      0x570
#define VCELL12_LO                      0x571
#define VCELL11_HI                      0x572
#define VCELL11_LO                      0x573
#define VCELL10_HI                      0x574
#define VCELL10_LO                      0x575
#define VCELL9_HI                       0x576
#define VCELL9_LO                       0x577
#define VCELL8_HI                       0x578
#define VCELL8_LO                       0x579
#define VCELL7_HI                       0x57A
#define VCELL7_LO                       0x57B
#define VCELL6_HI                       0x57C
#define VCELL6_LO                       0x57D
#define VCELL5_HI                       0x57E
#define VCELL5_LO                       0x57F
#define VCELL4_HI                       0x580
#define VCELL4_LO                       0x581
#define VCELL3_HI                       0x582
#define VCELL3_LO                       0x583
#define VCELL2_HI                       0x584
#define VCELL2_LO                       0x585
#define VCELL1_HI                       0x586
#define VCELL1_LO                       0x587
#define BUSBAR_HI                       0x588
#define BUSBAR_LO                       0x589
#define TSREF_HI                        0x58C
#define TSREF_LO                        0x58D
#define GPIO1_HI                        0x58E
#define GPIO1_LO                        0x58F
#define GPIO2_HI                        0x590
#define GPIO2_LO                        0x591
#define GPIO3_HI                        0x592
#define GPIO3_LO                        0x593
#define GPIO4_HI                        0x594
#define GPIO4_LO                        0x595
#define GPIO5_HI                        0x596
#define GPIO5_LO                        0x597
#define GPIO6_HI                        0x598
#define GPIO6_LO                        0x599
#define GPIO7_HI                        0x59A
#define GPIO7_LO                        0x59B
#define GPIO8_HI                        0x59C
#define GPIO8_LO                        0x59D
#define DIETEMP1_HI                     0x5AE
#define DIETEMP1_LO                     0x5AF
#define DIETEMP2_HI                     0x5B0
#define DIETEMP2_LO                     0x5B1
#define AUX_CELL_HI                     0x5B2
#define AUX_CELL_LO                     0x5B3
#define AUX_GPIO_HI                     0x5B4
#define AUX_GPIO_LO                     0x5B5
#define AUX_BAT_HI                      0x5B6
#define AUX_BAT_LO                      0x5B7
#define AUX_REFL_HI                     0x5B8
#define AUX_REFL_LO                     0x5B9
#define AUX_VBG2_HI                     0x5BA
#define AUX_VBG2_LO                     0x5BB
#define AUX_AVAO_REF_HI                 0x5BE
#define AUX_AVAO_REF_LO                 0x5BF
#define AUX_AVDD_REF_HI                 0x5C0
#define AUX_AVDD_REF_LO                 0x5C1
#define AUX_OV_DAC_HI                   0x5C2
#define AUX_OV_DAC_LO                   0x5C3
#define AUX_UV_DAC_HI                   0x5C4
#define AUX_UV_DAC_LO                   0x5C5
#define AUX_OT_OTCB_DAC_HI              0x5C6
#define AUX_OT_OTCB_DAC_LO              0x5C7
#define AUX_UT_DAC_HI                   0x5C8
#define AUX_UT_DAC_LO                   0x5C9
#define AUX_VCBDONE_DAC_HI              0x5CA
#define AUX_VCBDONE_DAC_LO              0x5CB
#define AUX_VCM_HI                      0x5CC
#define AUX_VCM_LO                      0x5CD
#define REFOVDAC_HI                     0x5D0
#define REFOVDAC_LO                     0x5D1
#define DIAG_MAIN_HI                    0x5D2
#define DIAG_MAIN_LO                    0x5D3
#define DIAG_AUX_HI                     0x5D4
#define DIAG_AUX_LO                     0x5D5
#define DEBUG_COMM_STAT                 0x780
#define DEBUG_UART_RC                   0x781
#define DEBUG_UART_RR_TR                0x782
#define DEBUG_COMH_BIT                  0x783
#define DEBUG_COMH_RC                   0x784
#define DEBUG_COMH_RR_TR                0x785
#define DEBUG_COML_BIT                  0x786
#define DEBUG_COML_RC                   0x787
#define DEBUG_COML_RR_TR                0x788
#define DEBUG_UART_DISCARD              0x789
#define DEBUG_COMH_DISCARD              0x78A
#define DEBUG_COML_DISCARD              0x78B
#define DEBUG_UART_VALID_HI             0x78C
#define DEBUG_UART_VALID_LO             0x78D
#define DEBUG_COMH_VALID_HI             0x78E
#define DEBUG_COMH_VALID_LO             0x78F
#define DEBUG_COML_VALID_HI             0x790
#define DEBUG_COML_VALID_LO             0x791
#define DEBUG_OTP_SEC_BLK               0x7A0
#define DEBUG_OTP_DED_BLK               0x7A1

/********************************************************************/

// Part ID of battery IC BQ79616
#define PartID              0x21

// CRC-16-IBM Polynomial (reversed 0x8005)
#define POLY                0xA001

// ADC resolution of battery IC BQ79616
#define mainADCResolution   190.73 // ADC resolution in microvolts per LSB
#define gpioResolution      152.59 // GPIO resolution in microvolts per LSB
#define TSREFResolution     169.54 // TSREF resolution in microvolts per LSB
#define BUSBAR_RESOLATION    30.52  // ADC resolution in microvolts per LSB

/********************************************************************/

// voltage values of battery
#define DEFAULT_FULLY_CHARGED_VOLTAGE       4200
#define DEFAULT_BALANCE_VOLTAGE_OFFSET      50
#define DEFAULT_BALANCE_THRESHOLD           3

/********************************************************************/

// thermistor constants
#define Rref_VALUE          10000   // 10KOHM reference resistor value for temperature measurements
#define Beta                3380.0  // B-constant of the thermistor
#define To                  298.15  // Ambient temperature in Kelvin (25°C + 273.15)
#define Ro                  10000.0 // Resistance in ambient temperature in kOhms (10kΩ)

/********************************************************************/

// sum of paralel resistors for current sensing
// #define SHUNT_RESISTOR_VAL              0.002/5.0
#define SHUNT_RESISTOR_VAL              0.0015
/********************************************************************/


const int kRxBQSignal = 0x1;

namespace measState {

    // Union for parsing DEV_STAT register data
    union unDevStat_Register {
        uint8_t all;
        struct {
            uint8_t MAIN_RUN      : 1;
            uint8_t AUX_RUN       : 1;
            uint8_t RSVD          : 1;
            uint8_t OVUV_RUN      : 1;
            uint8_t OTUT_RUN      : 1;
            uint8_t CUST_CRC_DONE : 1;
            uint8_t FACT_CRC_DONE : 1;
            uint8_t RSVD1         : 1;
        } bits;
    };

    // Union for parsing BAL_STAT register data
    union unBalStat_Register {
        uint8_t all;
        struct {
            uint8_t CB_DONE        : 1;
            uint8_t MB_DONE        : 1;
            uint8_t ABORTFLT       : 1;
            uint8_t CB_RUN         : 1;
            uint8_t MB_RUN         : 1;
            uint8_t CB_INPAUSE     : 1;
            uint8_t OT_PAUSE_DET   : 1;
            uint8_t INVALID_CBCONF : 1;
        } bits;
    };

    // Union for parsing CB_COMPLETE1 register data
    union unCBComplete1_Register {
        uint8_t all;
        struct {
            uint8_t CELL9_DONE  : 1;
            uint8_t CELL10_DONE : 1;
            uint8_t CELL11_DONE : 1;
            uint8_t CELL12_DONE : 1;
            uint8_t CELL13_DONE : 1;
            uint8_t CELL14_DONE : 1;
            uint8_t CELL15_DONE : 1;
            uint8_t CELL16_DONE : 1;
        } bits;
    };

    // Union for parsing CB_COMPLETE2 register data
    union unCBComplete2_Register {
        uint8_t all;
        struct {
            uint8_t CELL1_DONE : 1;
            uint8_t CELL2_DONE : 1;
            uint8_t CELL3_DONE : 1;
            uint8_t CELL4_DONE : 1;
            uint8_t CELL5_DONE : 1;
            uint8_t CELL6_DONE : 1;
            uint8_t CELL7_DONE : 1;
            uint8_t CELL8_DONE : 1;
        } bits;
    };

}

namespace measSettings {

    // Enum class for balance timer values
    enum class enBalancingTimer : uint8_t {
        T_NONE = 0x00, // 0 seconds, stop balancing
        T_10S = 0x01,         // 10 seconds
        T_30S = 0x02,         // 30 seconds
        T_60S = 0x03,         // 60 seconds
        T_300S = 0x04,        // 300 seconds (5 minutes)
        T_10M = 0x05,       // 10 minutes
        T_20M = 0x06,       // 20 minutes
        T_30M = 0x07,       // 30 minutes
        T_40M = 0x08,       // 40 minutes
        T_50M = 0x09,       // 50 minutes
        T_60M = 0x0A,       // 60 minutes
        T_70M = 0x0B,       // 70 minutes
        T_80M = 0x0C,       // 80 minutes
        T_90M = 0x0D,       // 90 minutes
        T_100M = 0x0E,      // 100 minutes
        T_110M = 0x0F,      // 110 minutes
        T_120M = 0x10,      // 120 minutes
        T_150M = 0x11,      // 150 minutes
        T_180M = 0x12,      // 180 minutes
        T_210M = 0x13,      // 210 minutes
        T_240M = 0x14,      // 240 minutes
        T_270M = 0x15,      // 270 minutes
        T_300M = 0x16,      // 300 minutes
        T_330M = 0x17,      // 330 minutes
        T_360M = 0x18,      // 360 minutes
        T_390M = 0x19,      // 390 minutes
        T_420M = 0x1A,      // 420 minutes
        T_450M = 0x1B,      // 450 minutes
        T_480M = 0x1C,      // 480 minutes
        T_510M = 0x1D,      // 510 minutes
        T_540M = 0x1E,      // 540 minutes
        T_600M = 0x1F       // 600 minutes
    };

    // Enum class for main ADC modes
    enum class enCellBalancingThreshold : uint8_t {
        DISABLED = 0x00, // Disables voltage based on CB_DONE comparison
        V_2_450 = 0x01,  // Threshold of 2.450 V
        V_2_475 = 0x02,  // Threshold of 2.475 V
        V_2_500 = 0x03,  // Threshold of 2.500 V
        V_2_525 = 0x04,  // Threshold of 2.525 V
        V_2_550 = 0x05,  // Threshold of 2.550 V
        V_2_575 = 0x06,  // Threshold of 2.575 V
        V_2_600 = 0x07,  // Threshold of 2.600 V
        V_2_625 = 0x08,  // Threshold of 2.625 V
        V_2_650 = 0x09,  // Threshold of 2.650 V
        V_2_675 = 0x0A,  // Threshold of 2.675 V
        V_2_700 = 0x0B,  // Threshold of 2.700 V
        V_2_725 = 0x0C,  // Threshold of 2.725 V
        V_2_750 = 0x0D,  // Threshold of 2.750 V
        V_2_775 = 0x0E,  // Threshold of 2.775 V
        V_2_800 = 0x0F,  // Threshold of 2.800 V
        V_2_825 = 0x10,  // Threshold of 2.825 V
        V_2_850 = 0x11,  // Threshold of 2.850 V
        V_2_875 = 0x12,  // Threshold of 2.875 V
        V_2_900 = 0x13,  // Threshold of 2.900 V
        V_2_925 = 0x14,  // Threshold of 2.925 V
        V_2_950 = 0x15,  // Threshold of 2.950 V
        V_2_975 = 0x16,  // Threshold of 2.975 V
        V_3_000 = 0x17,  // Threshold of 3.000 V
        V_3_025 = 0x18,  // Threshold of 3.025 V
        V_3_050 = 0x19,  // Threshold of 3.050 V
        V_3_075 = 0x1A,  // Threshold of 3.075 V
        V_3_100 = 0x1B,  // Threshold of 3.100 V
        V_3_125 = 0x1C,  // Threshold of 3.125 V
        V_3_150 = 0x1D,  // Threshold of 3.150 V
        V_3_175 = 0x1E,  // Threshold of 3.175 V
        V_3_200 = 0x1F,  // Threshold of 3.200 V
        V_3_225 = 0x20,  // Threshold of 3.225 V
        V_3_250 = 0x21,  // Threshold of 3.250 V
        V_3_275 = 0x22,  // Threshold of 3.275 V
        V_3_300 = 0x23,  // Threshold of 3.300 V
        V_3_325 = 0x24,  // Threshold of 3.325 V
        V_3_350 = 0x25,  // Threshold of 3.350 V
        V_3_375 = 0x26,  // Threshold of 3.375 V
        V_3_400 = 0x27,  // Threshold of 3.400 V
        V_3_425 = 0x28,  // Threshold of 3.425 V
        V_3_450 = 0x29,  // Threshold of 3.450 V
        V_3_475 = 0x2A,  // Threshold of 3.475 V
        V_3_500 = 0x2B,  // Threshold of 3.500 V
        V_3_525 = 0x2C,  // Threshold of 3.525 V
        V_3_550 = 0x2D,  // Threshold of 3.550 V
        V_3_575 = 0x2E,  // Threshold of 3.575 V
        V_3_600 = 0x2F,  // Threshold of 3.600 V
        V_3_625 = 0x30,  // Threshold of 3.625 V
        V_3_650 = 0x31,  // Threshold of 3.650 V
        V_3_675 = 0x32,  // Threshold of 3.675 V
        V_3_700 = 0x33,  // Threshold of 3.700 V
        V_3_725 = 0x34,  // Threshold of 3.725 V
        V_3_750 = 0x35,  // Threshold of 3.750 V
        V_3_775 = 0x36,  // Threshold of 3.775 V
        V_3_800 = 0x37,  // Threshold of 3.800 V
        V_3_825 = 0x38,  // Threshold of 3.825 V
        V_3_850 = 0x39,  // Threshold of 3.850 V
        V_3_875 = 0x3A,  // Threshold of 3.875 V
        V_3_900 = 0x3B,  // Threshold of 3.900 V
        V_3_925 = 0x3C,  // Threshold of 3.925 V
        V_4_00 = 0x3F    // Threshold of 4.00 V
    };


    // Enum class for GPIO values
    enum class enGpioConfig : uint8_t {
        Disabled = 0x00,
        ADC_OTUT_Input = 0x01,
        ADC_only_Input = 0x02,
        Digital_Input = 0x03,
        Output_High = 0x04,
        Output_Low = 0x05,
        ADC_Input_Weak_Pullup = 0x06,
        ADC_Input_Weak_Pulldown = 0x07
    };

}

namespace measControl {

    #pragma pack(push)
    #pragma pack(1)

    union unControl1 {
        uint8_t all;
        struct {
            uint8_t ADDR_WR : 1;
            uint8_t SOFT_RESET : 1;
            uint8_t GOTO_SLEEP: 1;
            uint8_t GOTO_SHUTDOWN : 1;
            uint8_t SEND_SLPTOACT : 1;
            uint8_t SEND_WAKE : 1;
            uint8_t SEND_SHUTDOWN: 1;
            uint8_t DIR_SEL : 1;
        } bits;
    };

    // Union for parsing CONTROL2 register data
    union unControl2 {
        uint8_t all;
        struct {
            uint8_t TSREF_EN : 1;
            uint8_t SEND_HW_RESET : 1;
            uint8_t RSVD  : 6;
        } bits;
    };

    union unADC_CTRL1 {
        uint8_t all;
        struct {
            uint8_t MAIN_MODE    : 2;
            uint8_t MAIN_GO      : 1;
            uint8_t LPF_VCELL_EN : 1;
            uint8_t LPF_BB_EN    : 1;
            uint8_t RSVD         : 3;
        } bits;
    };
    
    union unOTUT_CTRL {
        uint8_t all;
        struct {
            uint8_t OTUT_MODE     : 2;
            uint8_t OTUT_GO       : 1;
            uint8_t OTUT_LOCK     : 3;
            uint8_t OTCB_THR_LOCK : 1;
            uint8_t RSVD          : 1;
        } bits;
    };

    union unOVUV_CTRL {
        uint8_t all;
        struct {
            uint8_t OVUV_MODE        : 2;
            uint8_t OVUV_GO          : 1;
            uint8_t OVUV_LOCK        : 4;
            uint8_t VCBDONE_THR_LOCK : 1;
        } bits;
    };

    union unBAL_CTRL1 {
        uint8_t all;
        struct{
            uint8_t DUTY : 3;
            uint8_t RSVD : 5;
        } bits;
    };

    union unBAL_CTRL2 {
        uint8_t all;
        struct {
            uint8_t AUTO_BAL   : 1 ;
            uint8_t BAL_GO     : 1 ;
            uint8_t BAL_ACT    : 2 ;
            uint8_t OTCB_EN    : 1 ;
            uint8_t FLTSTOP_EN : 1 ;
            uint8_t CB_PAUSE   : 1 ;
            uint8_t RSVD       : 1 ;
        } bits;
    };

    #pragma pack(pop)

}

namespace measInternalErrors {

    // System error codes
    enum class enErrorCodes : uint8_t {
        // FAULT_PWR1_UNION errors
        AVDD_OV = 1,
        AVDD_OSC,
        DVDD_OV,
        CVDD_OV,
        CVDD_UV,
        REFHM_OPEN,
        DVSS_OPEN,
        CVSS_OPEN,

        // FAULT_PWR2_UNION errors
        TSREF_OV,
        TSREF_UV,
        TSREF_OSC,
        NEG5V_UV,
        REFH_OSC,
        PWRBIST_FAIL,

        // FAULT_PWR3_UNION error
        AVDDUV_DRST,

        // FAULT_SYS_UNION errors
        TWARN,
        TSHUT,
        CTS,
        CTL,
        DRST,
        GPIO,
        LFO,

        // FAULT_OV1_UNION errors
        OV9_DET,
        OV10_DET,
        OV11_DET,
        OV12_DET,
        OV13_DET,
        OV14_DET,
        OV15_DET,
        OV16_DET,

        // FAULT_OV2_UNION errors
        OV1_DET,
        OV2_DET,
        OV3_DET,
        OV4_DET,
        OV5_DET,
        OV6_DET,
        OV7_DET,
        OV8_DET,

        // FAULT_UV1_UNION errors
        UV9_DET,
        UV10_DET,
        UV11_DET,
        UV12_DET,
        UV13_DET,
        UV14_DET,
        UV15_DET,
        UV16_DET,

        // FAULT_UV2_UNION errors
        UV1_DET,
        UV2_DET,
        UV3_DET,
        UV4_DET,
        UV5_DET,
        UV6_DET,
        UV7_DET,
        UV8_DET,

        // FAULT_OT_UNION errors
        OT1_DET,
        OT2_DET,
        OT3_DET,
        OT4_DET,
        OT5_DET,
        OT6_DET,
        OT7_DET,
        OT8_DET,

        // FAULT_UT_UNION errors
        UT1_DET,
        UT2_DET,
        UT3_DET,
        UT4_DET,
        UT5_DET,
        UT6_DET,
        UT7_DET,
        UT8_DET,

        // FAULT_COMM3_UNION errors
        HB_FAST,
        HB_FAIL,
        FTONE_DET,
        FCOMM_DET,

        // FAULT_COMM2_UNION errors
        COMH_BIT,
        COMH_RC,
        COMH_RR,
        COMH_TR,
        COML_BIT,
        COML_RC,
        COML_RR,
        COML_TR,

        // FAULT_COMM1_UNION errors
        STOP_DET,
        COMMCLR_DET,
        UART_RC,
        UART_RR,
        UART_TR,

        // FAULT_OTP_UNION errors
        GBLOVERR,
        FACTLDERR,
        CUSTLDERR,
        FACT_CRC,
        CUST_CRC,
        SEC_DET,
        DED_DET,

        // FAULT_COMP_MISC_UNION errors
        LPF_FAIL,
        COMP_ADC_ABORT,

        // FAULT_COMP_CBFET2_UNION errors
        CBFET1_FAIL,
        CBFET2_FAIL,
        CBFET3_FAIL,
        CBFET4_FAIL,
        CBFET5_FAIL,
        CBFET6_FAIL,
        CBFET7_FAIL,
        CBFET8_FAIL,

        // FAULT_COMP_CBFET1_UNION errors
        CBFET9_FAIL,
        CBFET10_FAIL,
        CBFET11_FAIL,
        CBFET12_FAIL,
        CBFET13_FAIL,
        CBFET14_FAIL,
        CBFET15_FAIL,
        CBFET16_FAIL,

        // FAULT_COMP_CBOW2_UNION errors
        CBOW1_FAIL,
        CBOW2_FAIL,
        CBOW3_FAIL,
        CBOW4_FAIL,
        CBOW5_FAIL,
        CBOW6_FAIL,
        CBOW7_FAIL,
        CBOW8_FAIL,

        // FAULT_COMP_CBOW1_UNION errors
        CBOW9_FAIL,
        CBOW10_FAIL,
        CBOW11_FAIL,
        CBOW12_FAIL,
        CBOW13_FAIL,
        CBOW14_FAIL,
        CBOW15_FAIL,
        CBOW16_FAIL,

        // FAULT_COMP_VCOW2_UNION errors
        VCOW1_FAIL,
        VCOW2_FAIL,
        VCOW3_FAIL,
        VCOW4_FAIL,
        VCOW5_FAIL,
        VCOW6_FAIL,
        VCOW7_FAIL,
        VCOW8_FAIL,

        // FAULT_COMP_VCOW1_UNION errors
        VCOW9_FAIL,
        VCOW10_FAIL,
        VCOW11_FAIL,
        VCOW12_FAIL,
        VCOW13_FAIL,
        VCOW14_FAIL,
        VCOW15_FAIL,
        VCOW16_FAIL,

        // FAULT_COMP_VCCB2_UNION errors
        CELL1_FAIL,
        CELL2_FAIL,
        CELL3_FAIL,
        CELL4_FAIL,
        CELL5_FAIL,
        CELL6_FAIL,
        CELL7_FAIL,
        CELL8_FAIL,

        // FAULT_COMP_VCCB1_UNION errors
        CELL9_FAIL,
        CELL10_FAIL,
        CELL11_FAIL,
        CELL12_FAIL,
        CELL13_FAIL,
        CELL14_FAIL,
        CELL15_FAIL,
        CELL16_FAIL,

        // FAULT_COMP_GPIO_UNION errors
        GPIO1_FAIL,
        GPIO2_FAIL,
        GPIO3_FAIL,
        GPIO4_FAIL,
        GPIO5_FAIL,
        GPIO6_FAIL,
        GPIO7_FAIL,
        GPIO8_FAIL,

        // FAULT_PROT2_UNION errors
        UVCOMP_FAIL,
        OVCOMP_FAIL,
        OTCOMP_FAIL,
        UTCOMP_FAIL,
        VPATH_FAIL,
        TPATH_FAIL,
        BIST_ABORT,

        // FAULT_PROT1_UNION errors
        VPARITY_FAIL,
        TPARITY_FAIL,

        // DEBUG_COMH_BIT_UNION errors
        COMH_BIT_ERROR,
        COMH_SYNC1,
        COMH_SYNC2,
        COMH_BERR_TAG,
        COMH_PERR,

        // DEBUG_COMH_RC_UNION errors
        COMH_RC_CRC,
        COMH_RC_UNEXP,
        COMH_RC_BYTE_ERR,
        COMH_RC_SOF,
        COMH_RC_TXDIS,
        COMH_RC_IERR,

        // DEBUG_COMH_RR_TR_UNION errors
        COMH_RR_CRC,
        COMH_RR_UNEXP,
        COMH_RR_BYTE_ERR,
        COMH_RR_SOF,
        COMH_RR_TXDIS,
        COMH_RR_WAIT,

        // DEBUG_COML_BIT_UNION errors
        COML_BIT_ERROR,
        COML_SYNC1,
        COML_SYNC2,
        COML_BERR_TAG,
        COML_PERR,

        // DEBUG_COML_RC_UNION errors
        COML_RC_CRC,
        COML_RC_UNEXP,
        COML_RC_BYTE_ERR,
        COML_RC_SOF,
        COML_RC_TXDIS,
        COML_RC_IERR,

        // DEBUG_COML_RR_TR_UNION errors
        COML_RR_CRC,
        COML_RR_UNEXP,
        COML_RR_BYTE_ERR,
        COML_RR_SOF,
        COML_RR_TXDIS,
        COML_RR_WAIT,

        // DEBUG_UART_RC_UNION errors
        UART_RC_CRC,
        UART_RC_UNEXP,
        UART_RC_BYTE_ERR,
        UART_RC_SOF,
        UART_RC_TXDIS,
        UART_RC_IERR,

        // DEBUG_UART_RR_TR_UNION errors
        UART_RR_CRC,
        UART_RR_BYTE_ERR,
        UART_RR_SOF,
        UART_TR_WAIT,
        UART_RR_TR_SOF
    };

    #pragma pack(push)
    #pragma pack(1)

    // Level 1
    union unFaultSummary_Register {
        uint8_t all;
        struct {
            uint8_t FAULT_PWR       : 1;
            uint8_t FAULT_SYS_bit   : 1;
            uint8_t FAULT_OVUV      : 1;
            uint8_t FAULT_OTUT      : 1;
            uint8_t FAULT_COMM      : 1;
            uint8_t FAULT_OTP_bit   : 1;
            uint8_t FAULT_COMP_ADC  : 1;
            uint8_t FAULT_PROT      : 1;
        } bits;
    };

    // Level 2
    union unFaultPwr1_Register {
        uint8_t all;
        struct {
            uint8_t AVDD_OV     : 1;
            uint8_t AVDD_OSC    : 1;
            uint8_t DVDD_OV     : 1;
            uint8_t CVDD_OV     : 1;
            uint8_t CVDD_UV     : 1;
            uint8_t REFHM_OPEN  : 1;
            uint8_t DVSS_OPEN   : 1;
            uint8_t CVSS_OPEN   : 1;
        } bits;
    };

    union unFaultPwr2_Register {
        uint8_t all;
        struct {
            uint8_t TSREF_OV        : 1;
            uint8_t TSREF_UV        : 1;
            uint8_t TSREF_OSC       : 1;
            uint8_t NEG5V_UV        : 1;
            uint8_t REFH_OSC        : 1;
            uint8_t RSVD            : 1;
            uint8_t PWRBIST_FAIL    : 1;
            uint8_t RSVD1           : 1;
        } bits;
    };

    union unFaultPwr3_Register {
        uint8_t all;
        struct {
            uint8_t AVDDUV_DRST : 1;
            uint8_t RSVD    : 1;
            uint8_t RSVD1   : 1;
            uint8_t RSVD2   : 1;
            uint8_t RSVD3   : 1;
            uint8_t RSVD4   : 1;
            uint8_t RSVD5   : 1;
            uint8_t RSVD6   : 1;
        } bits;
    };

    union unFaultSys_Register {
        uint8_t all;
        struct {
            uint8_t TWARN   : 1;
            uint8_t TSHUT   : 1;
            uint8_t CTS     : 1;
            uint8_t CTL     : 1;
            uint8_t DRST    : 1;
            uint8_t GPIO    : 1;
            uint8_t RSVD    : 1;
            uint8_t LFO     : 1;
        } bits;
    };

    union unFaultOv1_Register {
        uint8_t all;
        struct {
            uint8_t OV9_DET : 1;
            uint8_t OV10_DET : 1;
            uint8_t OV11_DET : 1;
            uint8_t OV12_DET : 1;
            uint8_t OV13_DET : 1;
            uint8_t OV14_DET : 1;
            uint8_t OV15_DET : 1;
            uint8_t OV16_DET : 1;
        } bits;
    };

    union unFaultOv2_Register {
        uint8_t all;
        struct {
            uint8_t OV1_DET : 1;
            uint8_t OV2_DET : 1;
            uint8_t OV3_DET : 1;
            uint8_t OV4_DET : 1;
            uint8_t OV5_DET : 1;
            uint8_t OV6_DET : 1;
            uint8_t OV7_DET : 1;
            uint8_t OV8_DET : 1;
        } bits;
    };

    union unFaultUv1_Register {
        uint8_t all;
        struct {
            uint8_t UV9_DET : 1;
            uint8_t UV10_DET : 1;
            uint8_t UV11_DET : 1;
            uint8_t UV12_DET : 1;
            uint8_t UV13_DET : 1;
            uint8_t UV14_DET : 1;
            uint8_t UV15_DET : 1;
            uint8_t UV16_DET : 1;
        } bits;
    };

    union unFaultUv2_Register {
        uint8_t all;
        struct {
            uint8_t UV1_DET : 1;
            uint8_t UV2_DET : 1;
            uint8_t UV3_DET : 1;
            uint8_t UV4_DET : 1;
            uint8_t UV5_DET : 1;
            uint8_t UV6_DET : 1;
            uint8_t UV7_DET : 1;
            uint8_t UV8_DET : 1;
        } bits;
    };

    union unFaultOt_Register {
        uint8_t all;
        struct {
            uint8_t OT1_DET : 1;
            uint8_t OT2_DET : 1;
            uint8_t OT3_DET : 1;
            uint8_t OT4_DET : 1;
            uint8_t OT5_DET : 1;
            uint8_t OT6_DET : 1;
            uint8_t OT7_DET : 1;
            uint8_t OT8_DET : 1;
        } bits;
    };

    union unFaultUt_Register {
        uint8_t all;
        struct {
            uint8_t UT1_DET : 1;
            uint8_t UT2_DET : 1;
            uint8_t UT3_DET : 1;
            uint8_t UT4_DET : 1;
            uint8_t UT5_DET : 1;
            uint8_t UT6_DET : 1;
            uint8_t UT7_DET : 1;
            uint8_t UT8_DET : 1;
        } bits;
    };

    union unFaultComm3_Register {
        uint8_t all;
        struct {
            uint8_t HB_FAST : 1;
            uint8_t HB_FAIL : 1;
            uint8_t FTONE_DET : 1;
            uint8_t FCOMM_DET : 1;
            uint8_t RSVD1 : 1;
            uint8_t RSVD2 : 1;
            uint8_t RSVD3 : 1;
            uint8_t RSVD4 : 1;
        } bits;
    };

    union unFaultComm2_Register {
        uint8_t all;
        struct {
            uint8_t COMH_BIT : 1;
            uint8_t COMH_RC : 1;
            uint8_t COMH_RR : 1;
            uint8_t COMH_TR : 1;
            uint8_t COML_BIT : 1;
            uint8_t COML_RC : 1;
            uint8_t COML_RR : 1;
            uint8_t COML_TR : 1;
        } bits;
    };

    union unFaultComm1_Register {
        uint8_t all;
        struct {
            uint8_t STOP_DET : 1;
            uint8_t COMMCLR_DET : 1;
            uint8_t UART_RC : 1;
            uint8_t UART_RR : 1;
            uint8_t UART_TR : 1;
        } bits;
    };

    union unFaultOtp_Register {
        uint8_t all;
        struct {
            uint8_t GBLOVERR : 1;
            uint8_t FACTLDERR : 1;
            uint8_t CUSTLDERR : 1;
            uint8_t FACT_CRC : 1;
            uint8_t CUST_CRC : 1;
            uint8_t SEC_DET : 1;
            uint8_t DED_DET : 1;
            uint8_t RSVD : 1;
        } bits;
    };

    union unFaultCompMisc_Register {
        uint8_t all;
        struct {
            uint8_t LPF_FAIL : 1;
            uint8_t COMP_ADC_ABORT : 1;
            uint8_t RSRVD1 : 1;
            uint8_t RSRVD2 : 1;
            uint8_t RSRVD3 : 1;
            uint8_t RSRVD4 : 1;
            uint8_t RSRVD5 : 1;
            uint8_t RSRVD6 : 1;
        } bits;
    };

    union unFaultCompCbfet2_Register {
        uint8_t all;
        struct {
            uint8_t CBFET1_FAIL : 1;
            uint8_t CBFET2_FAIL : 1;
            uint8_t CBFET3_FAIL : 1;
            uint8_t CBFET4_FAIL : 1;
            uint8_t CBFET5_FAIL : 1;
            uint8_t CBFET6_FAIL : 1;
            uint8_t CBFET7_FAIL : 1;
            uint8_t CBFET8_FAIL : 1;
        } bits;
    };

    union unFaultCompCbfet1_Register {
        uint8_t all;
        struct {
            uint8_t CBFET9_FAIL : 1;
            uint8_t CBFET10_FAIL : 1;
            uint8_t CBFET11_FAIL : 1;
            uint8_t CBFET12_FAIL : 1;
            uint8_t CBFET13_FAIL : 1;
            uint8_t CBFET14_FAIL : 1;
            uint8_t CBFET15_FAIL : 1;
            uint8_t CBFET16_FAIL : 1;
        } bits;
    };

    union unFaultCompCbow2_Register {
        uint8_t all;
        struct {
            uint8_t CBOW1_FAIL : 1;
            uint8_t CBOW2_FAIL : 1;
            uint8_t CBOW3_FAIL : 1;
            uint8_t CBOW4_FAIL : 1;
            uint8_t CBOW5_FAIL : 1;
            uint8_t CBOW6_FAIL : 1;
            uint8_t CBOW7_FAIL : 1;
            uint8_t CBOW8_FAIL : 1;
        } bits;
    };

    union unFaultCompCbow1_Register {
        uint8_t all;
        struct {
            uint8_t CBOW9_FAIL : 1;
            uint8_t CBOW10_FAIL : 1;
            uint8_t CBOW11_FAIL : 1;
            uint8_t CBOW12_FAIL : 1;
            uint8_t CBOW13_FAIL : 1;
            uint8_t CBOW14_FAIL : 1;
            uint8_t CBOW15_FAIL : 1;
            uint8_t CBOW16_FAIL : 1;
        } bits;
    };

    union unFaultCompVcow2_Register {
        uint8_t all;
        struct {
            uint8_t VCOW1_FAIL : 1;
            uint8_t VCOW2_FAIL : 1;
            uint8_t VCOW3_FAIL : 1;
            uint8_t VCOW4_FAIL : 1;
            uint8_t VCOW5_FAIL : 1;
            uint8_t VCOW6_FAIL : 1;
            uint8_t VCOW7_FAIL : 1;
            uint8_t VCOW8_FAIL : 1;
        } bits;
    };

    union unFaultCompVcow1_Register {
        uint8_t all;
        struct {
            uint8_t VCOW9_FAIL : 1;
            uint8_t VCOW10_FAIL : 1;
            uint8_t VCOW11_FAIL : 1;
            uint8_t VCOW12_FAIL : 1;
            uint8_t VCOW13_FAIL : 1;
            uint8_t VCOW14_FAIL : 1;
            uint8_t VCOW15_FAIL : 1;
            uint8_t VCOW16_FAIL : 1;
        } bits;
    };

    union unFaultCompVccb2_Register {
        uint8_t all;
        struct {
            uint8_t CELL1_FAIL : 1;
            uint8_t CELL2_FAIL : 1;
            uint8_t CELL3_FAIL : 1;
            uint8_t CELL4_FAIL : 1;
            uint8_t CELL5_FAIL : 1;
            uint8_t CELL6_FAIL : 1;
            uint8_t CELL7_FAIL : 1;
            uint8_t CELL8_FAIL : 1;
        } bits;
    };

    union unFaultCompVccb1_Register {
        uint8_t all;
        struct {
            uint8_t CELL9_FAIL : 1;
            uint8_t CELL10_FAIL : 1;
            uint8_t CELL11_FAIL : 1;
            uint8_t CELL12_FAIL : 1;
            uint8_t CELL13_FAIL : 1;
            uint8_t CELL14_FAIL : 1;
            uint8_t CELL15_FAIL : 1;
            uint8_t CELL16_FAIL : 1;
        } bits;
    };

    union unFaultCompGpio_Register {
        uint8_t all;
        struct {
            uint8_t GPIO1_FAIL : 1;
            uint8_t GPIO2_FAIL : 1;
            uint8_t GPIO3_FAIL : 1;
            uint8_t GPIO4_FAIL : 1;
            uint8_t GPIO5_FAIL : 1;
            uint8_t GPIO6_FAIL : 1;
            uint8_t GPIO7_FAIL : 1;
            uint8_t GPIO8_FAIL : 1;
        } bits;
    };

    union unFaultProt2_Register {
        uint8_t all;
        struct {
            uint8_t UVCOMP_FAIL : 1;
            uint8_t OVCOMP_FAIL : 1;
            uint8_t OTCOMP_FAIL : 1;
            uint8_t UTCOMP_FAIL : 1;
            uint8_t VPATH_FAIL : 1;
            uint8_t TPATH_FAIL : 1;
            uint8_t BIST_ABORT : 1;
            uint8_t RSRVD : 1;
        } bits;
    };

    union unFaultProt1_Register {
        uint8_t all;
        struct {
            uint8_t VPARITY_FAIL : 1;
            uint8_t TPARITY_FAIL : 1;
            uint8_t RSRVD1 : 1;
            uint8_t RSRVD2 : 1;
            uint8_t RSRVD3 : 1;
            uint8_t RSRVD4 : 1;
            uint8_t RSRVD5 : 1;
            uint8_t RSRVD6 : 1;
        } bits;
    };

    // Level 3
    union unDebugComh_Register {
        uint8_t all;
        struct {
            uint8_t BIT : 1;
            uint8_t SYNC1 : 1;
            uint8_t SYNC2 : 1;
            uint8_t BERR_TAG : 1;
            uint8_t PERR : 1;
            uint8_t RSRVD1 : 1;
            uint8_t RSRVD2 : 1;
            uint8_t RSRVD3 : 1;
        } bits;
    };

    union unDebugComhRc_Register {
        uint8_t all;
        struct {
            uint8_t RC_CRC : 1;
            uint8_t RC_UNEXP : 1;
            uint8_t RC_BYTE_ERR : 1;
            uint8_t RC_SOF : 1;
            uint8_t RC_TXDIS : 1;
            uint8_t RC_IERR : 1;
            uint8_t RSRVD1 : 1;
            uint8_t RSRVD2 : 1;
        } bits;
    };

    union unDebugComhRrTr_Register {
        uint8_t all;
        struct {
            uint8_t RR_CRC : 1;
            uint8_t RR_UNEXP : 1;
            uint8_t RR_BYTE_ERR : 1;
            uint8_t RR_SOF : 1;
            uint8_t RR_TXDIS : 1;
            uint8_t TR_WAIT : 1;
            uint8_t RSRVD1 : 1;
            uint8_t RSRVD2 : 1;
        } bits;
    };

    union unDebugComl_Register {
        uint8_t all;
        struct {
            uint8_t BIT : 1;
            uint8_t SYNC1 : 1;
            uint8_t SYNC2 : 1;
            uint8_t BERR_TAG : 1;
            uint8_t PERR : 1;
            uint8_t RSRVD1 : 1;
            uint8_t RSRVD2 : 1;
            uint8_t RSRVD3 : 1;
        } bits;
    };

    union unDebugComlRc_Register {
        uint8_t all;
        struct {
            uint8_t RC_CRC : 1;
            uint8_t RC_UNEXP : 1;
            uint8_t RC_BYTE_ERR : 1;
            uint8_t RC_SOF : 1;
            uint8_t RC_TXDIS : 1;
            uint8_t RC_IERR : 1;
            uint8_t RSRVD1 : 1;
            uint8_t RSRVD2 : 1;
        } bits;
    };

    union unDebugComlRrTr_Register {
        uint8_t all;
        struct {
            uint8_t RR_CRC : 1;
            uint8_t RR_UNEXP : 1;
            uint8_t RR_BYTE_ERR : 1;
            uint8_t RR_SOF : 1;
            uint8_t RR_TXDIS : 1;
            uint8_t TR_WAIT : 1;
            uint8_t RSRVD1 : 1;
            uint8_t RSRVD2 : 1;
        } bits;
    };

    union unDebugUartRc_Register {
        uint8_t all;
        struct {
            uint8_t RC_CRC : 1;
            uint8_t RC_UNEXP : 1;
            uint8_t RC_BYTE_ERR : 1;
            uint8_t RC_SOF : 1;
            uint8_t RC_TXDIS : 1;
            uint8_t RC_IERR : 1;
            uint8_t RSRVD1 : 1;
            uint8_t RSRVD2 : 1;
        } bits;
    };

    union unDebugUartRrTr_Register {
        uint8_t all;
        struct {
            uint8_t RR_CRC : 1;
            uint8_t RR_BYTE_ERR : 1;
            uint8_t RR_SOF : 1;
            uint8_t TR_WAIT : 1;
            uint8_t TR_SOF : 1;
            uint8_t RSRVD1 : 1;
            uint8_t RSRVD2 : 1;
            uint8_t RSRVD3 : 1;
        } bits;
    };

    struct stdebugOtpSecBlk_Register {
        uint8_t BLOCK;
    };

    struct stdebugOtpDedBlk_Register {
        uint8_t BLOCK;
    };
    #pragma pack(pop)
}

namespace meas {

    #pragma pack(push) // push current alignment rules to internal stack
    #pragma pack(1)
    
    // Structure for parsing any register response frames
    struct _register {
        union {
            struct {
                uint8_t length;
                uint8_t deviceAddress;
                uint16_t registerAddress;
                uint8_t data;
                uint16_t crc;
            } fields;
            uint8_t raw[7]; // Raw byte array for the entire frame
        };

        uint8_t parse(const std::vector<uint8_t>& bytes);
    };

    // Structure for parsing voltage response frames
    struct voltages {
        union {
            struct 
            {
                uint8_t length;
                uint8_t deviceAddress;
                uint16_t registerAddress;
                union
                {
                  struct 
                  {
                    uint16_t cell16;
                    uint16_t cell15;
                    uint16_t cell14;
                    uint16_t cell13;
                    uint16_t cell12;
                    uint16_t cell11;
                    uint16_t cell10;
                    uint16_t cell9;
                    uint16_t cell8;
                    uint16_t cell7;
                    uint16_t cell6;
                    uint16_t cell5;
                    uint16_t cell4;
                    uint16_t cell3;
                    uint16_t cell2;
                    uint16_t cell1;
                  };
                  uint16_t cellVoltages[16];
                };
                uint16_t crc; 
            } fields;
            uint8_t raw[38]; // Raw byte array for the entire frame
        };

        void parse(const std::vector<uint8_t> &bytes);
        double calculate(uint16_t cellValue);
    };

    // Structure for parsing busbar response frames
    struct busBar {
        union {
            struct {
                uint8_t length;
                uint8_t deviceAddress;
                uint16_t registerAddress;
                uint16_t busBar;
                uint16_t crc; 
            } fields;
            uint8_t raw[8]; // Raw byte array for the entire frame
        };
    };

    // Structure for parsing temperature response frames
    struct temperatures {
        union {
            struct {
                uint8_t length;
                uint8_t deviceAddress;
                uint16_t registerAddress;
                union
                {
                  struct 
                  {
                    uint16_t gpio1;
                    uint16_t gpio2;
                    uint16_t gpio3;
                    uint16_t gpio4;
                    uint16_t gpio5;
                    uint16_t gpio6;
                    uint16_t gpio7;
                    uint16_t gpio8;
                  };
                  uint16_t tempValues[8];
                };
                uint16_t crc;
            } fields;
            uint8_t raw[22]; // Raw byte array for the entire frame
        };

        // void parse(const std::vector<uint8_t> &bytes);
        // void parse(const uint8_t *bytes);
        double calculate(uint16_t gpioValue, uint16_t TSREF_VALUE);
    };

    // Structure for parsing TSREF register response frames
    struct _TSREF {
        union {
            struct {
                uint8_t length;
                uint8_t deviceAddress;
                uint16_t registerAddress;
                uint16_t data;
                uint16_t crc;
            } fields;
            uint8_t raw[8]; // Raw byte array for the entire frame
        };
    };

    struct twoRegisters {
        union {
            struct {
                uint8_t length;
                uint8_t deviceAddress;
                uint16_t registerAddress;
                uint8_t reg1;
                uint8_t reg2;
                uint16_t crc;
            } fields;
            uint8_t raw[8]; // Raw byte array for the entire frame
        };
    };

    struct threeRegisters {
        union {
            struct {
                uint8_t length;
                uint8_t deviceAddress;
                uint16_t registerAddress;
                uint8_t reg1;
                uint8_t reg2;
                uint8_t reg3;
                uint16_t crc;
            } fields;
            uint8_t raw[9]; // Raw byte array for the entire frame
        };
    };

    struct fourRegisters {
        union {
            struct {
                uint8_t length;
                uint8_t deviceAddress;
                uint16_t registerAddress;
                uint8_t reg1;
                uint8_t reg2;
                uint8_t reg3;
                uint8_t reg4;
                uint16_t crc;
            } fields;
            uint8_t raw[10]; // Raw byte array for the entire frame
        };
    };

    struct sixRegisters {
        union {
            struct {
                uint8_t length;
                uint8_t deviceAddress;
                uint16_t registerAddress;
                uint8_t reg1;
                uint8_t reg2;
                uint8_t reg3;
                uint8_t reg4;
                uint8_t reg5;
                uint8_t reg6;
                uint16_t crc;
            } fields;
            uint8_t raw[12]; // Raw byte array for the entire frame
        };
    };

    #pragma pack(pop)
}

namespace measComm {

    // Enumeration to define REQ_TYPE
    enum class enReqType : uint8_t {
        SINGLE_READ       = 0b000,
        SINGLE_WRITE      = 0b001,
        STACK_READ        = 0b010,
        STACK_WRITE       = 0b011,
        BROADCAST_READ    = 0b100,
        BROADCAST_WRITE   = 0b101,
        R_BROADCAST_WRITE = 0b110
    };

    // Communication error codes 
    enum class enErrorCodes {
        SUCCESS = 0,
        WRITE_ERROR,
        DEVICE_NOT_WRITABLE,
        No_RESPONSE,
        READ_ERROR,
        READ_TIMEOUT,
        PARSE_ERROR,
        CRC_ERROR,
        INVALID_PARAMETER,
        UNKNOWN_ERROR
    };

}


#endif // __BQ79616_HANDLER_PARAMS__H_