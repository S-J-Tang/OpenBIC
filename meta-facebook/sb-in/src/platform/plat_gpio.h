/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PLAT_GPIO_H
#define PLAT_GPIO_H

#include "hal_gpio.h"

void gpio_int_default();

// clang-format off

// gpio_cfg(chip, number, is_init, direction, status, int_type, int_callback)
// dedicate gpio A0~A7, B0~B7, C0~C7, D0~D7, E0~E7, total 40 gpios
// Default name: Reserve_GPIOH0
#define name_gpio0	\
	gpio_name_to_num(FM_ASIC_0_THERMTRIP_R_N) \
	gpio_name_to_num(RST_ASTRID_PWR_ON_PLD_R1_N) \
	gpio_name_to_num(BUFF3_100M_LOSB_MMC) \
	gpio_name_to_num(Reserve_GPIO03) \
	gpio_name_to_num(ZORAVDD_I2C_LV_EN) \
	gpio_name_to_num(ALL_VR_PM_ALERT_R_N) \
	gpio_name_to_num(SMB_HAMSA_MMC_LVC33_ALERT_N) \
	gpio_name_to_num(FM_PLD_UBC_EN_R)
#define name_gpio1	\
	gpio_name_to_num(ZORA01_HBM_CATTRIP_MMC_LVC33_ALARM) \
	gpio_name_to_num(Reserve_GPIO11) \
	gpio_name_to_num(Reserve_GPIO12) \
	gpio_name_to_num(Reserve_GPIO13) \
	gpio_name_to_num(Reserve_GPIO14) \
	gpio_name_to_num(Reserve_GPIO15) \
	gpio_name_to_num(NUWA1_CHIP_STRAP1_MMC) \
	gpio_name_to_num(Reserve_GPIO17)
#define name_gpio2	\
	gpio_name_to_num(HAMSA_UART_MUX_SEL_R) \
	gpio_name_to_num(Reserve_GPIO21) \
	gpio_name_to_num(ZORA10_CHIP_STRAP0_MMC) \
	gpio_name_to_num(Reserve_GPIO23) \
	gpio_name_to_num(Reserve_GPIO24) \
	gpio_name_to_num(MMC_U684_OE) \
	gpio_name_to_num(MMC_U626_OE) \
	gpio_name_to_num(Reserve_GPIO27)
#define name_gpio3	\
	gpio_name_to_num(Reserve_GPIO30) \
	gpio_name_to_num(Reserve_GPIO31) \
	gpio_name_to_num(Reserve_GPIO32) \
	gpio_name_to_num(Reserve_GPIO33) \
	gpio_name_to_num(Reserve_GPIO34) \
	gpio_name_to_num(Reserve_GPIO35) \
	gpio_name_to_num(Reserve_GPIO36) \
	gpio_name_to_num(Reserve_GPIO37)
#define name_gpio4	\
	gpio_name_to_num(FM_OWL_W_JTAG_MUX_SEL_00) \
	gpio_name_to_num(FM_OWL_W_JTAG_MUX_SEL_01) \
	gpio_name_to_num(FM_OWL_W_JTAG_MUX_SEL_02) \
	gpio_name_to_num(FM_OWL_W_JTAG_MUX_SEL_03) \
	gpio_name_to_num(Reserve_GPIO44) \
	gpio_name_to_num(Reserve_GPIO45) \
	gpio_name_to_num(Reserve_GPIO46) \
	gpio_name_to_num(Reserve_GPIO47)
#define name_gpio5	\
	gpio_name_to_num(SPI_HAMSA_MUX_IN1) \
	gpio_name_to_num(SMB_ZORA00_CRM_ALERT_N_LS_LVC33_MMC) \
	gpio_name_to_num(SMB_ZORA01_CRM_ALERT_N_LS_LVC33_MMC) \
	gpio_name_to_num(SMB_ZORA10_CRM_ALERT_N_LS_LVC33_MMC) \
	gpio_name_to_num(FM_OWL_W_UART_MUX_SEL_00) \
	gpio_name_to_num(Reserve_GPIO55) \
	gpio_name_to_num(Reserve_GPIO56) \
	gpio_name_to_num(ZORA11_HBM_CATTRIP_MMC_LVC33_ALARM)
#define name_gpio6	\
	gpio_name_to_num(Reserve_GPIO60) \
	gpio_name_to_num(Reserve_GPIO61) \
	gpio_name_to_num(SMB_ZORA11_CRM_ALERT_N_LS_LVC33_MMC) \
	gpio_name_to_num(FM_OWL_E_JTAG_MUX_SEL_00) \
	gpio_name_to_num(FM_OWL_E_JTAG_MUX_SEL_01) \
	gpio_name_to_num(FM_OWL_W_UART_MUX_SEL_01) \
	gpio_name_to_num(Reserve_GPIO66) \
	gpio_name_to_num(Reserve_GPIO67)
#define name_gpio7	\
	gpio_name_to_num(Reserve_GPIO70) \
	gpio_name_to_num(FM_OWL_E_JTAG_MUX_SEL_02) \
	gpio_name_to_num(FM_OWL_E_JTAG_MUX_SEL_03) \
	gpio_name_to_num(SPI_CRD_CS0_N) \
	gpio_name_to_num(Reserve_GPIO74) \
	gpio_name_to_num(Reserve_GPIO75) \
	gpio_name_to_num(Reserve_GPIO76) \
	gpio_name_to_num(Reserve_GPIO77)
#define name_gpio8	\
	gpio_name_to_num(FM_OWL_W_UART_MUX_SEL_02) \
	gpio_name_to_num(NC_FAN_TACH1) \
	gpio_name_to_num(LED_MMC_HEARTBEAT_R) \
	gpio_name_to_num(ZORA10_HBM_CATTRIP_MMC_ALARM) \
	gpio_name_to_num(Reserve_GPIO84) \
	gpio_name_to_num(Reserve_GPIO85) \
	gpio_name_to_num(ZORA01_CHIP_STRAP1_MMC) \
	gpio_name_to_num(Reserve_GPIO87)
#define name_gpio9	\
	gpio_name_to_num(ZORA00_CHIP_STRAP0_MMC) \
	gpio_name_to_num(ZORA00_CHIP_STRAP1_MMC) \
	gpio_name_to_num(ZORA01_CHIP_STRAP0_MMC) \
	gpio_name_to_num(ZORA11_CHIP_STRAP1_MMC) \
	gpio_name_to_num(ZORA11_CHIP_STRAP0_MMC) \
	gpio_name_to_num(ZORA10_CHIP_STRAP1_MMC) \
	gpio_name_to_num(Reserve_GPIO96) \
	gpio_name_to_num(Reserve_GPIO97)
#define name_gpioA	\
	gpio_name_to_num(Reserve_GPIOA0) \
	gpio_name_to_num(Reserve_GPIOA1) \
	gpio_name_to_num(Reserve_GPIOA2) \
	gpio_name_to_num(Reserve_GPIOA3) \
	gpio_name_to_num(Reserve_GPIOA4) \
	gpio_name_to_num(Reserve_GPIOA5) \
	gpio_name_to_num(Reserve_GPIOA6) \
	gpio_name_to_num(Reserve_GPIOA7)
#define name_gpioB	\
	gpio_name_to_num(ZORA00_HBM_CATTRIP_MMC_LVC33_ALARM) \
	gpio_name_to_num(FM_OWL_E_UART_MUX_SEL_02) \
	gpio_name_to_num(FM_OWL_E_UART_MUX_SEL_00) \
	gpio_name_to_num(FM_OWL_E_UART_MUX_SEL_01) \
	gpio_name_to_num(Reserve_GPIOB4) \
	gpio_name_to_num(Reserve_GPIOB5) \
	gpio_name_to_num(FM_PDB_INA238_I2C_BUF_EN) \
	gpio_name_to_num(Reserve_GPIOB7)
#define name_gpioC	\
	gpio_name_to_num(Reserve_GPIOC0) \
	gpio_name_to_num(Reserve_GPIOC1) \
	gpio_name_to_num(Reserve_GPIOC2) \
	gpio_name_to_num(Reserve_GPIOC3) \
	gpio_name_to_num(Reserve_GPIOC4) \
	gpio_name_to_num(Reserve_GPIOC5) \
	gpio_name_to_num(Reserve_GPIOC6) \
	gpio_name_to_num(Reserve_GPIOC7)
#define name_gpioD	\
	gpio_name_to_num(Reserve_GPIOD0) \
	gpio_name_to_num(Reserve_GPIOD1) \
	gpio_name_to_num(Reserve_GPIOD2) \
	gpio_name_to_num(Reserve_GPIOD3) \
	gpio_name_to_num(Reserve_GPIOD4) \
	gpio_name_to_num(Reserve_GPIOD5) \
	gpio_name_to_num(Reserve_GPIOD6) \
	gpio_name_to_num(Reserve_GPIOD7)
#define name_gpioE	\
	gpio_name_to_num(PRSNT_PDB_SENSOR_N_R) \
	gpio_name_to_num(I3C_INGRID_ALERT_N) \
	gpio_name_to_num(Reserve_GPIOE2) \
	gpio_name_to_num(Reserve_GPIOE3) \
	gpio_name_to_num(Reserve_GPIOE4) \
	gpio_name_to_num(EN_MMC_CLKGEN_SW_R) \
	gpio_name_to_num(Reserve_GPIOE6) \
	gpio_name_to_num(Reserve_GPIOE7)
#define name_gpioF	\
	gpio_name_to_num(Reserve_GPIOF0) \
	gpio_name_to_num(Reserve_GPIOF1) \
	gpio_name_to_num(Reserve_GPIOF2) \
	gpio_name_to_num(Reserve_GPIOF3)

// clang-format on

#define gpio_name_to_num(x) x,
enum _GPIO_NUMS_ {
	name_gpio0 name_gpio1 name_gpio2 name_gpio3 name_gpio4 name_gpio5 name_gpio6 name_gpio7
		name_gpio8 name_gpio9 name_gpioA name_gpioB name_gpioC name_gpioD name_gpioE
			name_gpioF
};

extern enum _GPIO_NUMS_ GPIO_NUMS;
#undef gpio_name_to_num

extern char *gpio_name[];

// clang-format off
/*
 * SGPIO0/SGPIO1 real pin mapping, from the sb-in "CPLD SGPIO Slave" table.
 *
 * The table's columns are named from the CPLD's point of view: its "SGPIO0
 * Input" (MMC --> CPLD) is data the BIC drives OUT, i.e. this board's
 * SGPIO0_OUT(); its "SGPIO0 Output" (CPLD --> MMC) is data the BIC reads IN,
 * i.e. this board's SGPIO0_IN(). Confirmed against the schematic - do not
 * flip this again without re-checking.
 */

// SGPIO0_OUT (IOX1, BIC --> CPLD; table column "SGPIO0 Input")
#define wHAMSA_MFIO19                SGPIO0_OUT(0)
#define wZORA00_VQPS_TOP_EN          SGPIO0_OUT(1)
#define wZORA01_VQPS_TOP_EN          SGPIO0_OUT(2)
#define wZORA10_VQPS_TOP_EN          SGPIO0_OUT(3)
#define wZORA11_VQPS_TOP_EN          SGPIO0_OUT(4)
#define wZORA00_VQPS_U_EN            SGPIO0_OUT(5)
#define wZORA01_VQPS_U_EN            SGPIO0_OUT(6)
#define wZORA10_VQPS_U_EN            SGPIO0_OUT(7)
#define wZORA11_VQPS_U_EN            SGPIO0_OUT(8)
#define wHAMSA_VQPS_EFUSE_USER_EN    SGPIO0_OUT(9)
#define wHAMSA_TEST_STRAP_R          SGPIO0_OUT(10)
#define wHAMSA_LS_STRAP0             SGPIO0_OUT(11)
#define wHAMSA_LS_STRAP1             SGPIO0_OUT(12)
#define wHAMSA_CRM_STRAP0            SGPIO0_OUT(13)
#define wHAMSA_CRM_STRAP1            SGPIO0_OUT(14)
#define wHAMSA_MFIO7                 SGPIO0_OUT(15)
#define wHAMSA_MFIO9                 SGPIO0_OUT(16)
#define wHAMSA_MFIO11                SGPIO0_OUT(17)
#define wHAMSA_MFIO14                SGPIO0_OUT(18)
#define wHAMSA_MFIO16                SGPIO0_OUT(19)
#define wHAMSA_MFIO17                SGPIO0_OUT(20)
#define wHAMSA_MFIO18                SGPIO0_OUT(21)
#define wHAMSA_MFIO22                SGPIO0_OUT(22)
#define wHAMSA_MFIO23                SGPIO0_OUT(23)
#define wHAMSA_MFIO25                SGPIO0_OUT(24)
#define wHAMSA_CORE_TAP_CTRL_L       SGPIO0_OUT(25)
#define wHAMSA_TRI_L                 SGPIO0_OUT(26)
#define wHAMSA_ATPG_MODE_L           SGPIO0_OUT(27)
#define wHAMSA_DFT_TAP_EN_L          SGPIO0_OUT(28)
#define wFM_JTAG_HAMSA_JTCE0         SGPIO0_OUT(29)
#define wFM_JTAG_HAMSA_JTCE1         SGPIO0_OUT(30)
#define wFM_JTAG_HAMSA_JTCE2         SGPIO0_OUT(31)
#define wFM_JTAG_HAMSA_JTCE3         SGPIO0_OUT(32)
#define wZORA00_TEST_STRAP           SGPIO0_OUT(33)
#define wZORA01_TEST_STRAP           SGPIO0_OUT(34)
#define wZORA10_TEST_STRAP           SGPIO0_OUT(35)
#define wZORA11_TEST_STRAP           SGPIO0_OUT(36)
#define wZORA00_CRM_STRAP0           SGPIO0_OUT(37)
#define wZORA01_CRM_STRAP0           SGPIO0_OUT(38)
#define wZORA10_CRM_STRAP0           SGPIO0_OUT(39)
#define wZORA11_CRM_STRAP0           SGPIO0_OUT(40)
#define wZORA00_CRM_STRAP1           SGPIO0_OUT(41)
#define wZORA01_CRM_STRAP1           SGPIO0_OUT(42)
#define wZORA10_CRM_STRAP1           SGPIO0_OUT(43)
#define wZORA11_CRM_STRAP1           SGPIO0_OUT(44)
#define wZORA00_CORE_TAP_CTRL_PLD_L  SGPIO0_OUT(45)
#define wZORA01_CORE_TAP_CTRL_PLD_L  SGPIO0_OUT(46)
#define wZORA10_CORE_TAP_CTRL_PLD_L  SGPIO0_OUT(47)
#define wZORA11_CORE_TAP_CTRL_PLD_L  SGPIO0_OUT(48)
#define wZORA00_TRI_L                SGPIO0_OUT(49)
#define wZORA01_TRI_L                SGPIO0_OUT(50)
#define wZORA10_TRI_L                SGPIO0_OUT(51)
#define wZORA11_TRI_L                SGPIO0_OUT(52)
#define wZORA00_ATPG_MODE_L          SGPIO0_OUT(53)
#define wZORA01_ATPG_MODE_L          SGPIO0_OUT(54)
#define wZORA10_ATPG_MODE_L          SGPIO0_OUT(55)
#define wZORA11_ATPG_MODE_L          SGPIO0_OUT(56)
#define wZORA00_DFT_TAP_EN_PLD_L     SGPIO0_OUT(57)
#define wZORA01_DFT_TAP_EN_PLD_L     SGPIO0_OUT(58)
#define wZORA10_DFT_TAP_EN_PLD_L     SGPIO0_OUT(59)
#define wZORA11_DFT_TAP_EN_PLD_L     SGPIO0_OUT(60)
#define wFM_JTAG_ZORA00_JTCE0        SGPIO0_OUT(61)
#define wFM_JTAG_ZORA00_JTCE1        SGPIO0_OUT(62)
#define wFM_JTAG_ZORA00_JTCE2        SGPIO0_OUT(63)

// SGPIO0_IN (IOX1, CPLD --> BIC; table column "SGPIO0 Output")
#define SMB_ZORA00_CRM_LS_LVC33_ALERT_N  SGPIO0_IN(0)
#define SMB_ZORA01_CRM_LS_LVC33_ALERT_N  SGPIO0_IN(1)
#define SMB_ZORA10_CRM_LS_LVC33_ALERT_N  SGPIO0_IN(2)
#define SMB_ZORA11_CRM_LS_LVC33_ALERT_N  SGPIO0_IN(3)
#define FM_MODULE_PWRBRK_R_N             SGPIO0_IN(4)
#define IRQ_TMP75_1_ALERT_R_N            SGPIO0_IN(5)
#define IRQ_TMP75_2_ALERT_R_N            SGPIO0_IN(6)
#define IRQ_TMP75_3_ALERT_R_N            SGPIO0_IN(7)
#define TEMP_MON2_OVERT_N                SGPIO0_IN(8)
#define TEMP_MON3_OVERT_N                SGPIO0_IN(9)
#define TEMP_MON4_OVERT_N                SGPIO0_IN(10)
#define TEMP_MON5_OVERT_N                SGPIO0_IN(11)
#define ZORA00_VDD_SMBALERT_N_R          SGPIO0_IN(12)
#define ZORA01_VDD_SMBALERT_N_R          SGPIO0_IN(13)
#define ZORA10_VDD_SMBALERT_N_R          SGPIO0_IN(14)
#define ZORA11_VDD_SMBALERT_N_R          SGPIO0_IN(15)
#define MAX_EW2_VDD_SMBALERT_R_N         SGPIO0_IN(16)
#define MAX_EW1_VDD_SMBALERT_R_N         SGPIO0_IN(17)
#define OWL_W_VDD_SMBALERT_N             SGPIO0_IN(18)
#define OWL_E_VDD_SMBALERT_N             SGPIO0_IN(19)
#define VDDPHY_HBM2367_SMBALERT_N        SGPIO0_IN(20)
#define VDDPHY_HBM0145_SMBALERT_N        SGPIO0_IN(21)
#define VDDC_HBM2367_SMBALERT_N          SGPIO0_IN(22)
#define SMB_HAMSA_LS_LVC33_ALERT_N       SGPIO0_IN(23)
#define RADSOK_CONN_PRSNT                SGPIO0_IN(24)
#define CHASSIS1_LEAK_Q_N                SGPIO0_IN(25)
#define LEAK1_DETECT                     SGPIO0_IN(26)
#define PRSNT_CHASSIS1_LEAK_CABLE_R_N    SGPIO0_IN(27)
#define VDDC_HBM0145_SMBALERT_N          SGPIO0_IN(28)
#define SMB_VPP_HBM_P1V8_ALERT_N         SGPIO0_IN(29)
#define VRHOT_ZORA00_VDD_N               SGPIO0_IN(30)
#define VRHOT_ZORA01_VDD_N               SGPIO0_IN(31)
#define VRHOT_ZORA10_VDD_N               SGPIO0_IN(32)
#define VRHOT_ZORA11_VDD_N               SGPIO0_IN(33)
#define ZORA00_STANDALONE_MODE_R         SGPIO0_IN(34)
#define ZORA01_STANDALONE_MODE_R         SGPIO0_IN(35)
#define ZORA10_STANDALONE_MODE_R         SGPIO0_IN(36)
#define ZORA11_STANDALONE_MODE_R         SGPIO0_IN(37)
#define ZORA00_HBM_CATTRIP_ALARM_PLD     SGPIO0_IN(38)
#define ZORA01_HBM_CATTRIP_ALARM_PLD     SGPIO0_IN(39)
#define ZORA10_HBM_CATTRIP_ALARM_PLD     SGPIO0_IN(40)
#define ZORA11_HBM_CATTRIP_ALARM_PLD     SGPIO0_IN(41)
#define HAMSA_CATTRIP_R                  SGPIO0_IN(42)
#define OWL_E_SOC_CATTRIP_R              SGPIO0_IN(43)
#define OWL_W_SOC_CATTRIP_R              SGPIO0_IN(44)
#define OWL_W_SOC_CATTRIP_R_BIT45        SGPIO0_IN(45)
#define CLK_RDY_RC38208_PLD              SGPIO0_IN(46)
#define HOST_PWRGD_R                     SGPIO0_IN(47)

// SGPIO1_OUT (IOX2, BIC --> CPLD; table column "SGPIO1 Input")
#define wMMC_HAMSA_POWER_ON_RESET_PLD_L   SGPIO1_OUT(0)
#define wMMC_ZORA00_POWER_ON_RESET_PLD_L  SGPIO1_OUT(1)
#define wMMC_ZORA01_POWER_ON_RESET_PLD_L  SGPIO1_OUT(2)
#define wMMC_ZORA10_POWER_ON_RESET_PLD_L  SGPIO1_OUT(3)
#define wMMC_ZORA11_POWER_ON_RESET_PLD_L  SGPIO1_OUT(4)
#define wMMC_HAMSA_SYS_RST_PLD_L          SGPIO1_OUT(5)
#define wMMC_ZORA00_SYS_RST_PLD_L         SGPIO1_OUT(6)
#define wMMC_ZORA01_SYS_RST_PLD_L         SGPIO1_OUT(7)
#define wMMC_ZORA10_SYS_RST_PLD_L         SGPIO1_OUT(8)
#define wMMC_ZORA11_SYS_RST_PLD_L         SGPIO1_OUT(9)
#define ZORA00_PWR_CAP_LV1_PLD            SGPIO1_OUT(41)
#define ZORA00_PWR_CAP_LV2_PLD            SGPIO1_OUT(42)
#define ZORA00_PWR_CAP_LV3_PLD            SGPIO1_OUT(43)
#define ZORA_PWR_CAP_LV1_PLD              SGPIO1_OUT(44)
#define ZORA_PWR_CAP_LV2_PLD              SGPIO1_OUT(45)
#define ZORA_PWR_CAP_LV3_PLD              SGPIO1_OUT(46)
#define wPLD_OWL_E_DFT_TAP_EN_L           SGPIO1_OUT(47)
#define wPLD_OWL_E_CORE_TAP_CTRL_L        SGPIO1_OUT(48)
#define wPLD_OWL_E_PAD_TRI_L              SGPIO1_OUT(49)
#define wPLD_OWL_E_ATPG_MODE_L            SGPIO1_OUT(50)
#define wPLD_OWL_W_DFT_TAP_EN_L           SGPIO1_OUT(51)
#define wPLD_OWL_W_CORE_TAP_CTRL_L        SGPIO1_OUT(52)
#define wPLD_OWL_W_PAD_TRI_L              SGPIO1_OUT(53)
#define wPLD_OWL_W_ATPG_MODE_L            SGPIO1_OUT(54)
#define wFM_JTAG_ZORA01_JTCE0             SGPIO1_OUT(55)
#define wFM_JTAG_ZORA01_JTCE1             SGPIO1_OUT(56)
#define wFM_JTAG_ZORA01_JTCE2             SGPIO1_OUT(57)
#define wFM_JTAG_ZORA10_JTCE0             SGPIO1_OUT(58)
#define wFM_JTAG_ZORA10_JTCE1             SGPIO1_OUT(59)
#define wFM_JTAG_ZORA10_JTCE2             SGPIO1_OUT(60)
#define wFM_JTAG_ZORA11_JTCE0             SGPIO1_OUT(61)
#define wFM_JTAG_ZORA11_JTCE1             SGPIO1_OUT(62)
#define wFM_JTAG_ZORA11_JTCE2             SGPIO1_OUT(63)

// SGPIO1_IN (IOX2, CPLD --> BIC; table column "SGPIO1 Output")
#define FM_52V_PWROFF_SENSE_PLD     SGPIO1_IN(0)
#define FM_3V3_PWROFF_SENSE_PLD     SGPIO1_IN(1)
#define PWRGD_P1V8_AUX              SGPIO1_IN(2)
#define PWRGD_P1V2_AUX              SGPIO1_IN(3)
#define PWR_EN                      SGPIO1_IN(4)
#define P12V_UBC1_PWRGD             SGPIO1_IN(5)
#define P12V_UBC2_PWRGD             SGPIO1_IN(6)
#define PWRGD_P3V3_R                SGPIO1_IN(7)
#define PWRGD_P4V2_R                SGPIO1_IN(8)
#define PWRGD_U677                  SGPIO1_IN(9)
#define PWRGD_U678                  SGPIO1_IN(10)
#define PWRGD_U682                  SGPIO1_IN(11)
#define PWRGD_P5V_R                 SGPIO1_IN(12)
#define PWRGD_LDO_IN_1V2_R          SGPIO1_IN(13)
#define PWRGD_P1V8_BF_R2            SGPIO1_IN(14)
#define PWRGD_P0V75_AVDD_HCSL_R     SGPIO1_IN(15)
#define PWRGD_HAMSA_VDD_R           SGPIO1_IN(16)
#define PWRGD_OWL_W_VDD_PLD         SGPIO1_IN(17)
#define PWRGD_OWL_E_VDD_PLD         SGPIO1_IN(18)
#define PWRGD_MAX_EW2_VDD_PLD       SGPIO1_IN(19)
#define PWRGD_MAX_EW1_VDD_PLD       SGPIO1_IN(20)
#define PWRGD_MAX_N_VDD_PLD         SGPIO1_IN(21)
#define PWRGD_MAX_M_VDD_PLD         SGPIO1_IN(22)
#define PWRGD_MAX_S_VDD_PLD         SGPIO1_IN(23)
#define PWRGD_OWL_E_TRVDD0P75_PLD   SGPIO1_IN(24)
#define PWRGD_OWL_W_TRVDD0P75_PLD   SGPIO1_IN(25)
#define PWRGD_VDDPHY_HBM0145_PLD    SGPIO1_IN(26)
#define PWRGD_VDDPHY_HBM2367_PLD    SGPIO1_IN(27)
#define PWRGD_ZORA00_VDDL_PLD       SGPIO1_IN(28)
#define PWRGD_ZORA01_VDDL_PLD       SGPIO1_IN(29)
#define PWRGD_ZORA10_VDDL_PLD       SGPIO1_IN(30)
#define PWRGD_ZORA11_VDDL_PLD       SGPIO1_IN(31)
#define PWRGD_ZORA00_VDD_PLD        SGPIO1_IN(32)
#define PWRGD_ZORA01_VDD_PLD        SGPIO1_IN(33)
#define PWRGD_ZORA10_VDD_PLD        SGPIO1_IN(34)
#define PWRGD_ZORA11_VDD_PLD        SGPIO1_IN(35)
#define PWRGD_P1V5_PLL_VDDA_OWL_E   SGPIO1_IN(36)
#define PWRGD_P1V5_PLL_VDDA_OWL_W   SGPIO1_IN(37)
#define PWRGD_P1V5_PLL_VDDA_SOC     SGPIO1_IN(38)
#define PWRGD_P1V2_PLL_VDDA_OWL_W   SGPIO1_IN(39)
#define PWRGD_VPP_HBM2367_R         SGPIO1_IN(40)
#define PWRGD_VPP_HBM0145_R         SGPIO1_IN(41)
#define PWRGD_VDDC_HBM0145          SGPIO1_IN(42)
#define PWRGD_VDDC_HBM2367_PLD      SGPIO1_IN(43)
#define PWRGD_VDDQ_HBM0145_PLD      SGPIO1_IN(44)
#define PWRGD_VDDQ_HBM2367_PLD      SGPIO1_IN(45)
#define PWRGD_VDDQL_HBM0145_PLD     SGPIO1_IN(46)
#define PWRGD_VDDQL_HBM2367_PLD     SGPIO1_IN(47)
#define PWRGD_HAMSA_AVDD_PCIE_PLD   SGPIO1_IN(48)
#define PWRGD_OWL_E_TRVDD0P9_PLD    SGPIO1_IN(49)
#define PWRGD_OWL_W_TRVDD0P9_PLD    SGPIO1_IN(50)
#define PWRGD_PVDD1P5               SGPIO1_IN(51)
#define PWRGD_HAMSA_VDDHRXTX_PCIE3  SGPIO1_IN(52)
#define PWRGD_HAMSA_VDDHRXTX_PCIE2  SGPIO1_IN(53)
#define PWRGD_HAMSA_VDDHRXTX_PCIE1  SGPIO1_IN(54)
#define PWRGD_HAMSA_VDDHRXTX_PCIE0  SGPIO1_IN(55)
// clang-format on

#endif
