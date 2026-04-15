#ifndef __IO_DEFS_H
#define __IO_DEFS_H

#define BUZZER_PORT                 GPIOC
#define BUZZER_PIN                  PC_6

#define RGB_RED_PORT                GPIOB
#define RGB_RED_PIN                 PB_0

#define RGB_GREEN_PORT              GPIOB
#define RGB_GREEN_PIN               PB_1

#define RGB_BLUE_PORT               GPIOB
#define RGB_BLUE_PIN                PB_2

#define POWER_EN_PORT               GPIOB
#define POWER_EN_PIN                PB_4

#define POWER_BTN_DET_PORT          GPIOB
#define POWER_BTN_DET_PIN           PB_3

#define POWER_BTN_LED_PORT          GPIOA
#define POWER_BTN_LED_PIN           PB_6

#define TERMINAL_TX_PORT            GPIOC
#define TERMINAL_TX_PIN             PC_10

#define TERMINAL_RX_PORT            GPIOC
#define TERMINAL_RX_PIN             PC_11

#define RS485_TX_PORT               GPIOA
#define RS485_TX_PIN                PA_2

#define RS485_RX_PORT               GPIOA
#define RS485_RX_PIN                PA_3

#define CAN_TX_PORT                 GPIOA
#define CAN_TX_PIN                  PA_12

#define CAN_RX_PORT                 GPIOA
#define CAN_RX_PIN                  PA_11

#define BQ79616_TX_PORT             GPIOC
#define BQ79616_TX_PIN              PC_4

#define BQ79616_RX_PORT             GPIOC
#define BQ79616_RX_PIN              PC_5

#define BQ79616_FAULT_PORT          GPIOA
#define BQ79616_FAULT_PIN           PA_7

#define OLED_SCL_PORT               GPIOC
#define OLED_SCL_PIN                PC_8

#define OLED_SDA_PORT               GPIOC
#define OLED_SDA_PIN                PC_9

#define EEPROM_RTC_SCL_PORT         GPIOA
#define EEPROM_RTC_SCL_PIN          PA_9

#define EEPROM_RTC_SDA_PORT         GPIOA
#define EEPROM_RTC_SDA_PIN          PA_8

#define PWR_DISCHARGE_PORT          GPIOC
#define PWR_DISCHARGE_PIN           PC_15

#define PWR_CHARGE_PORT             GPIOC
#define PWR_CHARGE_PIN              PC_0

#define PWR_PREDISCHARGE_PORT       GPIOC
#define PWR_PREDISCHARGE_PIN        PC_1

#define PWR_CUR_LIM_PORT            GPIOA
#define PWR_CUR_LIM_PIN             PA_4

#define PWR_OCP_RST_PORT            GPIOB
#define PWR_OCP_RST_PIN             PB_5

#define PWR_PACK_CUR_PORT           GPIOC
#define PWR_PACK_CUR_PIN            PC_3

#define PWR_LOAD_VOL_PORT           GPIOA
#define PWR_LOAD_VOL_PIN            PC_3

#define PWR_PCB_TEMP_PORT           GPIOC
#define PWR_PCB_TEMP_PIN            PC_2

#define PWR_PACK_VOL_PORT           GPIOA
#define PWR_PACK_VOL_PIN            PA_0

#define PWR_CONT_DISCHARGE_PORT     GPIOA
#define PWR_CONT_DISCHARGE_PIN      PA_6

#define PWR_CONT_CHARGE_PORT        GPIOA
#define PWR_CONT_CHARGE_PIN         PA_5

#define PWR_CONT_PRECHARGE_PORT     GPIOC
#define PWR_CONT_PRECHARGE_PIN      PC_14

#define SD_DET_PORT                 GPIOC
#define SD_DET_PIN                  PC_13

#define EEPROM_WC_PORT              GPIOA
#define EEPROM_WC_PIN               PA_15

#define CHG_DETECT_PORT              GPIOC
#define CHG_DETECT_PIN               PC_7

#define DCHG_OCP_DETECT_PORT         GPIOB
#define DCHG_OCP_DETECT_PIN          PB_9

#define SD_SCLK_PORT                GPIOB
#define SD_SCLK_PIN                 PB_13

#define SD_MOSI_PORT                GPIOB
#define SD_MOSI_PIN                 PB_15

#define SD_MISO_PORT                GPIOB
#define SD_MISO_PIN                 PB_14

#define SD_CS_PORT                  GPIOB
#define SD_CS_PIN                   PB_12



#endif /* IO_DEFS */