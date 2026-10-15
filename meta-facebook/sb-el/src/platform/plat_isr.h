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

#ifndef PLAT_ISR_H
#define PLAT_ISR_H

#include "plat_cpld.h"

#define ASIC_SPI_FLASH_DEV_NAME "spi_fiu0_cs1"
#define SPI_FIU0_DEV_NAME "SPI_FIU0"
#define SPI_FIU0_CS1_SLAVE 1
#define ASIC_ODS_MASK 0x07
#define HAMSA_ODS_CODE 0x3
#define NUWA_ODS_CODE 0x6

#define SPI_NOR_CMD_WREN 0x06
#define SPI_NOR_CMD_RDSR 0x05
#define SPI_NOR_CMD_WRSR 0x01
#define SPI_NOR_CMD_RDCR 0x15
#define SPI_NOR_CMD_RDHCR 0x71
#define SPI_NOR_CMD_WRHCR 0x72

#define ASIC_HAMSA_POR_BIT 5
#define ASIC_NUWA0_POR_BIT 4
#define ASIC_NUWA1_POR_BIT 3
#define ASIC_HAMSA_SYS_RST_BIT 2
#define ASIC_NUWA0_SYS_RST_BIT 1
#define ASIC_NUWA1_SYS_RST_BIT 0

enum asic_spi_chip { ASIC_SPI_HAMSA, ASIC_SPI_NUWA0, ASIC_SPI_NUWA1 };

void ISR_GPIO_ALL_VR_PM_ALERT_R_N();
void ISR_GPIO_FM_PLD_UBC_EN_R();
bool ubc_en_changed_callback(cpld_info *info, uint8_t *data);
void plat_switch_pin_a12(bool use_gpio73);
void ISR_GPIO_RST_ARKE_PWR_ON_PLD_R1_N();
void ISR_GPIO_SMB_HAMSA_MMC_LVC33_ALERT_N();
void ISR_ASIC_THERMTRIP_TRIGGER(void);
void set_pwr_steps_on_flag(uint8_t flag_value);
uint8_t get_pwr_steps_on_flag(void);
bool get_sensor_polling_delay_elapsed(void);
void set_sensor_polling_delay_elapsed(bool elapsed);

#endif
