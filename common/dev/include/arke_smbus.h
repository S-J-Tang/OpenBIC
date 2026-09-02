/* Copyright (c) Meta Platforms, Inc. and affiliates. */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ARKE_SMBUS_H
#define ARKE_SMBUS_H

#define ASIC_MONITOR_HBM_TEMP_REG 0x8F
#define ASIC_MONITOR_TEMP_REG 0x70
#define ASIC_MONITOR_TEMP_REG_LEN 10
#define ASIC_MONITOR_HBM_TEMP_REG_LEN 10

typedef struct {
	uint8_t temp[ASIC_MONITOR_TEMP_REG_LEN];
	uint8_t hbm_temp[ASIC_MONITOR_HBM_TEMP_REG_LEN];
} arke_smbus_priv_data_t;

#endif
