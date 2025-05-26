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

void ISR_GPIO_FM_ASIC_0_THERMTRIP_R_N();
void ISR_GPIO_RST_HAMSA_PWR_ON_R_PLD_N();
void ISR_GPIO_ALL_VR_PM_ALERT_R_N();
void ISR_GPIO_SMBUS_HAMSA_PSOC_LVC33_ALERT_N();
void ISR_GPIO_FM_PLD_UBC_EN_R();
void ISR_GPIO_MEDHA0_HBM_CATTRIP_PSOC_LVC33_ALARM();
void ISR_GPIO_MEDHA1_HBM_CATTRIP_PSOC_LVC33_ALARM();
void ISR_GPIO_SMB_RAINBOW_ALERT_N();
void ISR_GPIO_MEDHA0_CURRENT_SENSE_0_LS_LVC33();
void ISR_GPIO_MEDHA0_CURRENT_SENSE_1_LS_LVC33();
void ISR_GPIO_MEDHA1_CURRENT_SENSE_0_LS_LVC33();
void ISR_GPIO_MEDHA1_CURRENT_SENSE_1_LS_LVC33();
void ISR_GPIO_SMBUS_MEDHA0_CRM_LS_PSOC_LVC33_ALERT_N();
void ISR_GPIO_SMBUS_MEDHA1_CRM_LS_PSOC_LVC33_ALERT_N();
bool plat_i2c_read(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len);
bool plat_i2c_write(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len);

void plat_clock_init();
void plat_eusb_init();

#endif
