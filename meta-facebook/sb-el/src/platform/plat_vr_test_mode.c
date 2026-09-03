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
#include <logging/log.h>
#include "plat_util.h"
#include "plat_pldm_sensor.h"
#include "plat_vr_test_mode.h"
#include "plat_hook.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_vr_test_mode);

struct k_thread vr_test_mode_thread;
K_KERNEL_STACK_MEMBER(vr_test_mode_thread_stack, 512);
k_tid_t vr_test_mode_tid;
bool vr_test_mode_flag = false;

bool get_vr_test_mode_flag(void)
{
	return vr_test_mode_flag;
}
//clang-format off
const vr_test_mode_setting_t vr_test_mode_table[] = {
	// vr_rail, fast ocp: x/10(A), slow ocp: x/10(A), uvp: 1(mV), ovp: 1(mV), v max: 1(mV), lcr(mV), ucr(mV), vout_default(mV)
	{ VR_RAIL_E_ASIC_P0V75_NUWA0_VDD, 14300, 14300, 200, 940, 930, 595, 930, 0 },
	{ VR_RAIL_E_ASIC_P0V75_NUWA1_VDD, 14300, 14300, 200, 940, 930, 595, 930, 0 },
	{ VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD, 250, 250, 200, 1130, 1120, 630, 1120, 0 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD, 140, 140, 200, 1130, 975, 525, 975, 0 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_M_VDD, 500, 500, 200, 1130, 975, 525, 975, 0 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357, 1200, 1200, 200, 940, 975, 525, 975, 0 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_VDD, 1650, 1650, 200, 940, 930, 525, 930, 0 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357, 650, 650, 200, 800, 520, 280, 520, 0 },
	{ VR_RAIL_E_ASIC_P1V05_VDDC_HBM1357, 2200, 2200, 200, 1210, 1200, 780, 1200, 0 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM1357, 250, 250, 200, 1980, 1970, 1260, 1970, 0 },
	{ VR_RAIL_E_ASIC_P0V9_VDDQ_HBM1357, 1440, 1440, 200, 1100, 1090, 700, 1090, 0 },
	{ VR_RAIL_E_ASIC_P0V85_HAMSA_VDD, 850, 850, 200, 1063, 1053, 595, 1053, 0 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_N_VDD, 500, 500, 200, 1130, 975, 525, 975, 0 },
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, 450, 450, 200, 1130, 1040, 560, 1040, 0 },
	{ VR_RAIL_E_ASIC_P0V9_VDDQ_HBM0246, 1440, 1440, 200, 1100, 1090, 700, 1090, 0 },
	{ VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE, 150, 150, 200, 1320, 1310, 840, 1310, 0 },
	{ VR_RAIL_E_ASIC_P1V05_VDDC_HBM0246, 2200, 2200, 200, 1210, 1200, 780, 1200, 0 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM0246, 250, 250, 200, 1980, 1970, 1260, 1970, 0 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246, 650, 650, 200, 800, 520, 280, 520, 0 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246, 1200, 1200, 200, 940, 975, 525, 975, 0 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_VDD, 1650, 1650, 200, 940, 930, 525, 930, 0 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_S_VDD, 500, 500, 200, 1130, 975, 525, 975, 0 },
	{ VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD, 250, 250, 200, 1130, 1120, 630, 1120, 0 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD, 140, 140, 200, 1130, 975, 525, 975, 0 },
};
const vr_test_mode_setting_t vr_test_mode_table_default[] = {
	// vr_rail, fast ocp: x/10(A), slow ocp: x/10(A), uvp: 1(mV), ovp: 1(mV), v max: 1(mV), lcr(mV), ucr(mV), vout_default(mV)
	{ VR_RAIL_E_ASIC_P0V75_NUWA0_VDD, 14300, 14300, 577, 940, 940, 674, 940, 775 },
	{ VR_RAIL_E_ASIC_P0V75_NUWA1_VDD, 14300, 14300, 577, 940, 940, 674, 940, 775 },
	{ VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD, 250, 230, 500, 954, 954, 846, 954, 900 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD, 140, 120, 350, 795, 795, 728, 795, 750 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_M_VDD, 500, 430, 350, 848, 848, 690, 848, 750 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357, 1200, 1130, 350, 795, 795, 705, 795, 750 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_VDD, 1650, 1500, 350, 810, 810, 690, 810, 750 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357, 650, 620, 325, 800, 565, 460, 565, 512 },
	{ VR_RAIL_E_ASIC_P1V05_VDDC_HBM1357, 2200, 2110, 715, 1210, 1210, 1034, 1210, 1115 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM1357, 250, 220, 1400, 1950, 1950, 1746, 1950, 1800 },
	{ VR_RAIL_E_ASIC_P0V9_VDDQ_HBM1357, 1440, 1310, 620, 1100, 1100, 940, 1100, 1020 },
	{ VR_RAIL_E_ASIC_P0V85_HAMSA_VDD, 850, 810, 480, 961, 961, 739, 961, 880 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_N_VDD, 500, 430, 350, 848, 848, 690, 848, 750 },
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, 450, 390, 400, 880, 880, 780, 880, 800 },
	{ VR_RAIL_E_ASIC_P0V9_VDDQ_HBM0246, 1440, 1310, 620, 1100, 1100, 940, 1100, 1020 },
	{ VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE, 150, 100, 800, 1272, 1272, 1164, 1272, 1200 },
	{ VR_RAIL_E_ASIC_P1V05_VDDC_HBM0246, 2200, 2110, 715, 1210, 1210, 1034, 1210, 1115 },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM0246, 250, 220, 1400, 1950, 1950, 1746, 1950, 1800 },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246, 650, 620, 325, 800, 565, 460, 565, 512 },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246, 1200, 1130, 350, 795, 795, 705, 795, 750 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_VDD, 1650, 1500, 345, 810, 810, 685, 810, 745 },
	{ VR_RAIL_E_ASIC_P0V75_MAX_S_VDD, 500, 430, 343, 848, 848, 684, 848, 743 },
	{ VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD, 250, 230, 500, 954, 954, 846, 954, 900 },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD, 140, 120, 350, 795, 795, 705, 795, 750 },
};
const mps_vr_test_mode_setting_t vr_mps_test_mode_table[] = {
	/*
	uint8_t vr_rail;
	uint16_t total_ocp;
	uint16_t mp2971_uvp_threshold_gain;
	uint16_t uvp_threshold;
	uint16_t lcr;
	uint16_t ucr;
	uint16_t ovp2_threshold;
	uint16_t ovp1_threshold;
	uint16_t ovp2;
	uint16_t ovp1;
	uint16_t vout_default;
	uint16_t vout_max;
	uint16_t uvp;
	*/
	{ VR_RAIL_E_ASIC_P0V75_NUWA0_VDD, 1424, NO_USE, 500, 595, 930, NO_USE, NO_USE, NO_ACTION,
	  940, NO_USE, 930, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_NUWA1_VDD, 1424, NO_USE, 500, 595, 930, NO_USE, NO_USE, NO_ACTION,
	  940, NO_USE, 930, NO_USE },
	{ VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD, 17, 0, 400, 630, 1120, NO_USE, NO_USE, NO_ACTION, 1170,
	  NO_USE, 1120, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD, 11, 0, 400, 525, 975, NO_USE, NO_USE, NO_ACTION, 1175,
	  NO_USE, 975, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_MAX_M_VDD, 120, 0, 400, 525, 975, NO_USE, NO_USE, NO_ACTION, 1175,
	  NO_USE, 975, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357, 138, 0, 400, 525, 930, NO_USE, NO_USE, NO_ACTION,
	  980, NO_USE, 930, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_VDD, 159, 0, 400, 525, 930, NO_USE, NO_USE, NO_ACTION, 980,
	  NO_USE, 930, NO_USE },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357, 53, 0, 400, 280, 520, NO_USE, NO_USE, NO_ACTION, 820,
	  NO_USE, 520, NO_USE },
	{ VR_RAIL_E_ASIC_P1V05_VDDC_HBM1357, 318, 0, 400, 525, 975, NO_USE, NO_USE, NO_ACTION, 1025,
	  NO_USE, 975, NO_USE },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM1357, 14, 0, 400, 1260, 1970, NO_USE, NO_USE, NO_ACTION, 2020,
	  NO_USE, 1970, NO_USE },
	{ VR_RAIL_E_ASIC_P0V9_VDDQ_HBM1357, 318, 0, 400, 770, 1310, NO_USE, NO_USE, NO_ACTION, 1360,
	  NO_USE, 1310, NO_USE },
	{ VR_RAIL_E_ASIC_P0V85_HAMSA_VDD, 100, 0, 400, 595, 1055, NO_USE, NO_USE, NO_ACTION, 1105,
	  NO_USE, 1055, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_MAX_N_VDD, 67, 0, 400, 525, 975, NO_USE, NO_USE, NO_ACTION, 1175,
	  NO_USE, 975, NO_USE },
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, 50, 0, 400, 560, 1040, NO_USE, NO_USE, NO_ACTION,
	  1140, NO_USE, 1040, NO_USE },
	{ VR_RAIL_E_ASIC_P0V9_VDDQ_HBM0246, 318, 0, 400, 770, 1310, NO_USE, NO_USE, NO_ACTION, 1360,
	  NO_USE, 1310, NO_USE },
	{ VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE, 10, 0, 400, 840, 1310, NO_USE, NO_USE, NO_ACTION,
	  1360, NO_USE, 1310, NO_USE },
	{ VR_RAIL_E_ASIC_P1V05_VDDC_HBM0246, 318, 0, 400, 525, 975, NO_USE, NO_USE, NO_ACTION, 1025,
	  NO_USE, 975, NO_USE },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM0246, 14, 0, 400, 1260, 1970, NO_USE, NO_USE, NO_ACTION, 2020,
	  NO_USE, 1970, NO_USE },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246, 53, 0, 400, 280, 520, NO_USE, NO_USE, NO_ACTION, 820,
	  NO_USE, 520, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246, 138, 0, 400, 525, 930, NO_USE, NO_USE, NO_ACTION,
	  980, NO_USE, 930, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_VDD, 159, 0, 400, 525, 930, NO_USE, NO_USE, NO_ACTION, 980,
	  NO_USE, 930, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_MAX_S_VDD, 66, 0, 400, 525, 975, NO_USE, NO_USE, NO_ACTION, 1175,
	  NO_USE, 975, NO_USE },
	{ VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD, 17, 0, 400, 630, 1120, NO_USE, NO_USE, NO_ACTION, 1170,
	  NO_USE, 1120, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD, 11, 0, 400, 525, 975, NO_USE, NO_USE, NO_ACTION, 1175,
	  NO_USE, 975, NO_USE },
};
const mps_vr_test_mode_setting_t vr_mps_normal_mode_table[] = {
	/*
	uint8_t vr_rail;
	uint16_t total_ocp;
	uint16_t mp2971_uvp_threshold_gain;
	uint16_t uvp_threshold;
	uint16_t lcr;
	uint16_t ucr;
	uint16_t ovp2_threshold;
	uint16_t ovp1_threshold;
	uint16_t ovp2;
	uint16_t ovp1;
	uint16_t vout_default;
	uint16_t vout_max;
	uint16_t uvp;
	*/
	{ VR_RAIL_E_ASIC_P0V75_NUWA0_VDD, 1424, NO_USE, 200, 652, 848, NO_USE, NO_USE, NO_ACTION,
	  940, 750, 848, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_NUWA1_VDD, 1424, NO_USE, 200, 782, 918, NO_USE, NO_USE, 850, 940,
	  750, 918, NO_USE },
	{ VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD, 23, 0, 200, 846, 960, NO_USE, NO_USE, 1300, 1010, 900,
	  960, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_TRVDD, 12, 0, 175, 705, 810, NO_USE, NO_USE, 865, 910, 750,
	  810, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_MAX_M_VDD, 44, 0, 200, 690, 810, NO_USE, NO_USE, 980, 1210, 750, 810,
	  NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM1357, 112, 0, 175, 705, 810, NO_USE, NO_USE, 1000, 1210,
	  750, 810, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_OWL_E_VDD, 152, 0, 175, 690, 810, NO_USE, NO_USE, 990, 1210, 750,
	  810, NO_USE },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM1357, 62, 0, 75, 380, 440, NO_USE, NO_USE, 550, 840, 400,
	  440, NO_USE },
	{ VR_RAIL_E_ASIC_P1V05_VDDC_HBM1357, 212, 0, 200, 987, 1120, NO_USE, NO_USE, 1290, 1520,
	  1050, 1120, NO_USE },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM1357, 22, 0, 400, 1746, 1910, NO_USE, NO_USE, 1940, 2310, 1800,
	  1910, NO_USE },
	{ VR_RAIL_E_ASIC_P0V9_VDDQ_HBM1357, 132, 0, 200, 940, 1200, NO_USE, NO_USE, 1410, 1600,
	  1000, 1200, NO_USE },
	{ VR_RAIL_E_ASIC_P0V85_HAMSA_VDD, 82, 0, 200, 739, 1050, NO_USE, NO_USE, 1280, 1450, 850,
	  1050, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_MAX_N_VDD, 44, 0, 175, 690, 810, NO_USE, NO_USE, 990, 1210, 760, 810,
	  NO_USE },
	{ VR_RAIL_E_ASIC_P0V8_HAMSA_AVDD_PCIE, 39, 0, 175, 752, 850, NO_USE, NO_USE, 950, 950, 830,
	  850, NO_USE },
	{ VR_RAIL_E_ASIC_P0V9_VDDQ_HBM0246, 132, 0, 200, 940, 1200, NO_USE, NO_USE, 1150, 1600,
	  1000, 1200, NO_USE },
	{ VR_RAIL_E_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE, 10, 0, 200, 1128, 1400, NO_USE, NO_USE, 1340,
	  1800, 1200, 1400, NO_USE },
	{ VR_RAIL_E_ASIC_P1V05_VDDC_HBM0246, 212, 0, 200, 987, 1120, NO_USE, NO_USE, 1290, 1520,
	  1050, 1120, NO_USE },
	{ VR_RAIL_E_ASIC_P1V8_VPP_HBM0246, 22, 0, 400, 1746, 1910, NO_USE, NO_USE, 1940, 2310, 1800,
	  1910, NO_USE },
	{ VR_RAIL_E_ASIC_P0V4_VDDQL_HBM0246, 62, 0, 75, 380, 440, NO_USE, NO_USE, 550, 840, 400,
	  440, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_VDDPHY_HBM0246, 112, 0, 175, 705, 810, NO_USE, NO_USE, 1000, 1210,
	  750, 810, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_VDD, 152, 0, 175, 690, 810, NO_USE, NO_USE, 910, 1210, 750,
	  810, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_MAX_S_VDD, 43, 0, 200, 690, 810, NO_USE, NO_USE, 910, 1210, 760, 810,
	  NO_USE },
	{ VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD, 23, 0, 200, 846, 960, NO_USE, NO_USE, 1040, 1010, 900,
	  960, NO_USE },
	{ VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD, 12, 0, 175, 705, 810, NO_USE, NO_USE, 970, 910, 750,
	  810, NO_USE },
};
//clang-format on
const uint8_t vr_test_mode_table_size = ARRAY_SIZE(vr_test_mode_table);
const uint8_t vr_test_mode_table_dafault_size = ARRAY_SIZE(vr_test_mode_table_default);

const uint8_t vr_mps_test_mode_table_size = ARRAY_SIZE(vr_mps_test_mode_table);
const uint8_t vr_mps_normal_mode_table_size = ARRAY_SIZE(vr_mps_normal_mode_table);

bool update_vr_reg(uint8_t rail, uint8_t reg, uint16_t val)
{
	uint8_t data[2];
	data[0] = val & 0xFF;
	data[1] = (val >> 8) & 0xFF;

	if (!plat_set_vr_reg(rail, reg, data, 2)) {
		LOG_ERR("fail to set vr rail %x, reg 0x%02x, val: %x", rail, reg, val);
		return false;
	}
	return true;
}

static bool is_raa229140a_rail(uint8_t rail)
{
	uint8_t sensor_id = vr_rail_table[rail].sensor_id;
	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);

	return cfg && cfg->type == sensor_dev_raa229140a;
}

static bool encode_vr_test_mode_reg(uint8_t rail, uint8_t reg, uint16_t val, uint16_t *raw)
{
	CHECK_NULL_ARG_WITH_RETURN(raw, false);

	if (!is_raa229140a_rail(rail)) {
		*raw = val;
		return true;
	}

	switch (reg) {
	case VR_FAST_OCP_REG:
	case VR_SLOW_OCP_REG:
		return linear11_from_scaled(val, 10, raw); // table unit: 0.1 A
	case VR_UVP_REG:
	case VR_OVP_REG:
	case VR_VOUT_MAX_REG:
		*raw = linear16u_from_scaled(val, 1000, 9); // table unit: mV, exponent -9
		return true;
	default:
		return false;
	}
}

bool get_vr_test_mode_reg_value(uint8_t rail, uint8_t reg, uint16_t *val)
{
	CHECK_NULL_ARG_WITH_RETURN(val, false);

	bool is_raa229140a = is_raa229140a_rail(rail);
	uint8_t data[2] = { 0 };
	if (!get_raw_data_from_sensor_id(vr_rail_table[rail].sensor_id, reg, data, sizeof(data)))
		return false;

	uint16_t raw = data[0] | (data[1] << 8);
	if (!is_raa229140a) {
		*val = raw;
		return true;
	}

	switch (reg) {
	case VR_FAST_OCP_REG:
	case VR_SLOW_OCP_REG:
		*val = linear11_to_scaled(raw, 10); // return unit: 0.1 A
		return true;
	case VR_UVP_REG:
	case VR_OVP_REG:
	case VR_VOUT_MAX_REG:
		*val = linear16u_to_scaled(raw, 1000, 9); // return unit: mV, exponent -9
		return true;
	default:
		return false;
	}
}

bool dma_write_vr(uint8_t rail, uint16_t reg, uint8_t *data, uint8_t len)
{
	if (!update_vr_reg(rail, 0xC7, reg)) {
		LOG_ERR("VR %x dma write 0x%04x to 0xC7 fail", rail, reg);
		return false;
	}

	if (!plat_set_vr_reg(rail, 0xC5, data, len)) {
		LOG_ERR("VR %x dma write 0xC5 fail", rail);
		return false;
	}

	return true;
}

bool dma_read_vr(uint8_t rail, uint16_t reg, uint8_t *data, uint8_t len)
{
	if (!update_vr_reg(rail, 0xC7, reg)) {
		LOG_ERR("VR %x dma write 0x%04x to 0xC7 fail", rail, reg);
		return false;
	}
	uint8_t sensor_num = vr_rail_table[rail].sensor_id;
	if (!get_raw_data_from_sensor_id(sensor_num, 0xC5, data, len)) {
		LOG_ERR("VR %x dma write 0x%04x to 0xC5 fail", rail, reg);
		return false;
	}
	return true;
}

// uvp = vout - offset uvp, uvp = vout + offset ovp,
bool get_vr_offset_uvp_ovp(uint8_t rail, uint16_t *uvp, uint16_t *ovp)
{
	CHECK_NULL_ARG_WITH_RETURN(uvp, false);
	CHECK_NULL_ARG_WITH_RETURN(ovp, false);

	uint8_t data[2];
	uint8_t sensor_num = vr_rail_table[rail].sensor_id;
	if (!get_raw_data_from_sensor_id(sensor_num, VR_VOUT_REG, data, sizeof(data))) {
		LOG_ERR("VR %x get Vout fail", rail);
		return false;
	}

	uint16_t vout_raw = (data[1] << 8) | data[0];
	uint16_t vout =
		is_raa229140a_rail(rail) ? linear16u_to_scaled(vout_raw, 1000, 9) : vout_raw;
	uint8_t page = get_vr_page(rail);

	uint16_t reg = page ? VR_UVP_1_DMA_ADDR : VR_UVP_0_DMA_ADDR;
	if (!dma_read_vr(rail, reg, data, 2)) {
		LOG_ERR("VR %x get dma uvp fail", rail);
		return false;
	}
	uint16_t uvp_offset = (data[1] << 8) | data[0];

	reg = page ? VR_OVP_1_DMA_ADDR : VR_OVP_0_DMA_ADDR;
	if (!dma_read_vr(rail, reg, data, 2)) {
		LOG_ERR("VR %x get dma ovp fail", rail);
		return false;
	}
	uint16_t ovp_offset = (data[1] << 8) | data[0];

	*uvp = (vout > uvp_offset) ? (vout - uvp_offset) : 0;
	*ovp = vout + ovp_offset;

	return true;
}

// check if offset uvp/ovp is used.
bool use_offset_uvp_ovp(uint8_t rail)
{
	switch (rail) {
	case VR_RAIL_E_ASIC_P0V75_NUWA0_VDD:
	case VR_RAIL_E_ASIC_P0V75_NUWA1_VDD:
		return false;
	default:
		return true;
	}
}
bool get_vr_fixed_uvp_ovp_enable(uint8_t rail)
{
	// no offset UVP/OVP setting
	if (!use_offset_uvp_ovp(rail))
		return true;

	uint8_t data[2];
	uint8_t sensor_num = vr_rail_table[rail].sensor_id;
	if (!get_raw_data_from_sensor_id(sensor_num, VR_LOOPCFG_REG, data, sizeof(data))) {
		LOG_ERR("VR %x get loopcfg fail", rail);
		return false;
	}

	if (data[1] & BIT(6))
		return true;
	return false;
}
bool set_vr_fixed_uvp_ovp_enable(uint8_t rail, uint8_t enable)
{
	// no offset UVP/OVP setting
	if (!use_offset_uvp_ovp(rail))
		return true;

	uint8_t data[2];
	uint8_t sensor_num = vr_rail_table[rail].sensor_id;
	if (!get_raw_data_from_sensor_id(sensor_num, VR_LOOPCFG_REG, data, sizeof(data))) {
		LOG_ERR("VR %x get loopcfg fail", rail);
		return false;
	}

	WRITE_BIT(data[1], 6, enable);
	if (!plat_set_vr_reg(rail, VR_LOOPCFG_REG, data, 2)) {
		LOG_ERR("VR %x set loopcfg fail: %x %x", rail, data[0], data[1]);
		return false;
	}

	return true;
}

static bool set_vr_test_mode_reg(bool is_default)
{
	bool ret = true;
	const vr_test_mode_setting_t *table =
		is_default ? vr_test_mode_table_default : vr_test_mode_table;
	uint8_t table_size = is_default ? ARRAY_SIZE(vr_test_mode_table_default) :
					  ARRAY_SIZE(vr_test_mode_table);

	for (uint8_t i = 0; i < table_size; i++) {
		const vr_test_mode_setting_t *cfg = &table[i];
		const struct {
			uint8_t offset;
			uint16_t val;
			const char *name;
		} regs[] = {
			{ VR_FAST_OCP_REG, cfg->fast_ocp, "FAST OCP" },
			{ VR_SLOW_OCP_REG, cfg->slow_ocp, "SLOW OCP" },
			{ VR_UVP_REG, cfg->uvp, "UVP" },
			{ VR_OVP_REG, cfg->ovp, "OVP" },
			{ VR_VOUT_MAX_REG, cfg->vout_max, "VOUT MAX" },
		};

		if (is_default) {
			uint16_t vout_default = cfg->vout_default;
			if (!plat_set_vout_command(cfg->vr_rail, &vout_default, false)) {
				LOG_ERR("VR rail %x set vout to default: %d failed", cfg->vr_rail,
					cfg->vout_default);
				ret = false;
			}
		}

		for (size_t j = 0; j < ARRAY_SIZE(regs); j++) {
			uint16_t raw = 0;
			if (!encode_vr_test_mode_reg(cfg->vr_rail, regs[j].offset, regs[j].val,
						     &raw) ||
			    !update_vr_reg(cfg->vr_rail, regs[j].offset, raw)) {
				ret = false;
				LOG_ERR("VR rail %x set %s to %d failed", cfg->vr_rail,
					regs[j].name, regs[j].val);
			}
		}

		vout_range_user_settings.change_vout_min[i] = cfg->lcr;
		vout_range_user_settings.change_vout_max[i] = cfg->ucr;
	}
	return ret;
}

static bool set_mps_vr_test_mode_reg(bool is_default)
{
	bool ret = true;
	const mps_vr_test_mode_setting_t *table =
		is_default ? vr_mps_normal_mode_table : vr_mps_test_mode_table;
	uint8_t table_size = is_default ? ARRAY_SIZE(vr_mps_normal_mode_table) :
					  ARRAY_SIZE(vr_mps_test_mode_table);

	for (uint8_t i = 0; i < table_size; i++) {
		const mps_vr_test_mode_setting_t *cfg = &table[i];
		const struct {
			uint8_t function;
			uint16_t val;
			const char *name;
		} regs[] = {
			{ TOTAL_OCP, cfg->total_ocp, "TOTAL OCP" },
			{ UVP_THRESHOLD, cfg->uvp_threshold, "UVP_THRESHOLD" },
			{ VOUT_MAX, cfg->vout_max, "VOUT MAX" },
			{ OVP_1, cfg->ovp1, "OVP1" },
		};

		if (is_default) {
			uint16_t vout_default = cfg->vout_default;
			if (!plat_set_vout_command(cfg->vr_rail, &vout_default, false)) {
				LOG_ERR("VR rail %x set vout to default: %d failed", cfg->vr_rail,
					cfg->vout_default);
				ret = false;
			}
		}

		uint16_t set_val = 0;
		for (size_t j = 0; j < ARRAY_SIZE(regs); j++) {
			set_val = regs[j].val;
			if (i < VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD) {
				if (!fab2_mps_ic_second_source) {
					if (set_vr_mp29816a_reg(cfg->vr_rail, &set_val,
								regs[j].function)) {
						LOG_ERR("MPS29816a VR rail %x set %s to %d failed",
							cfg->vr_rail, regs[j].name, regs[j].val);
					}
				} else {
					if (set_vr_mp29526_reg(cfg->vr_rail, &set_val,
							       regs[j].function)) {
						LOG_ERR("MPS29526 VR rail %x set %s to %d failed",
							cfg->vr_rail, regs[j].name, regs[j].val);
					}
				}
			} else {
				if (set_vr_mp2971_reg(cfg->vr_rail, &set_val, regs[j].function)) {
					LOG_ERR("MPS2971 VR rail %d set %s to %d failed",
						cfg->vr_rail, regs[j].name, regs[j].val);
				}
			}
		}

		vout_range_user_settings.change_vout_min[i] = cfg->lcr;
		vout_range_user_settings.change_vout_max[i] = cfg->ucr;
	}
	return ret;
}

static bool check_vr_slow_ocp_match_test_mode(void)
{
	for (uint8_t i = 0; i < VR_RAIL_E_P3V3_OSFP_VOLT_V; i++) {
		uint16_t val = 0;
		if (!get_vr_test_mode_reg_value(i, VR_SLOW_OCP_REG, &val))
			return false;

		if (vr_test_mode_table[i].slow_ocp != val)
			return false;
	}

	return true;
}

void vr_test_mode_enable(bool onoff)
{
	LOG_WRN("vr_test_mode_enable: %d", onoff);
	vr_test_mode_flag = onoff;
	uint8_t vr = get_vr_module();
	if (vr == VR_MODULE_RNS) {
		set_vr_test_mode_reg((onoff ? false : true));
		// not include P3V3
		for (uint8_t i = 0; i < VR_RAIL_E_P3V3_OSFP_VOLT_V; i++) {
			if (!set_vr_fixed_uvp_ovp_enable(i, (onoff ? 1 : 0)))
				LOG_ERR("set vr %d fix uvp/ovp enable fail!", i);
		}
	} else if (vr == VR_MODULE_MPS) {
		// mp29816C
		// if set to test mode, set ovp2 action to no action
		// mp2971
		// if set to test mode, set divider enable
		uint16_t action = 0;
		uint16_t div_en = 0;
		if (vr_test_mode_flag == true) {
			action = NO_ACTION;
			div_en = DISABLE;
		} else if (vr_test_mode_flag == false) {
			action = LATCH_OFF;
			div_en = ENABLE;
		} else {
			LOG_ERR("MPS test mode setting flag error! flag: %d", vr_test_mode_flag);
			return;
		}
		for (uint8_t i = 0; i <= VR_RAIL_E_ASIC_P0V75_NUWA1_VDD; i++) {
			// set ovp2 action to no action
			if (!fab2_mps_ic_second_source) {
				if (set_vr_mp29816a_reg(i, &action, OVP_2_ACTION))
					LOG_ERR("set vr %d ovp2 action fail!", i);
			} else {
				if (set_vr_mp29526_reg(i, &action, OVP_2_ACTION))
					LOG_ERR("set vr %d ovp2 action fail!", i);
			}
		}
		for (uint8_t i = VR_RAIL_E_ASIC_P0V9_OWL_E_TRVDD; i < VR_RAIL_E_P3V3_OSFP_VOLT_V;
		     i++) {
			// set ovp2 action to no action
			if (set_vr_mp2971_reg(i, &action, OVP_2_ACTION))
				LOG_ERR("set vr %d ovp2 action fail!", i);
			// set divider enable/disable
			if (i == VR_RAIL_E_ASIC_P1V8_VPP_HBM0246 ||
			    i == VR_RAIL_E_ASIC_P1V8_VPP_HBM1357)
				// divider always disable for HBM VPP rails
				continue;
			if (set_vr_mp2971_reg(i, &div_en, DIV_EN))
				LOG_ERR("set vr %d divider enable/disable fail!", i);
		}

		set_mps_vr_test_mode_reg((onoff ? false : true));
	} else {
		LOG_ERR("VR module %d is not supported!", vr);
	}
}

void vr_test_mode_handler(void *arg1, void *arg2, void *arg3)
{
	k_sleep(K_MSEC(5000)); // wait sensor thread ready

	while (1) {
		k_sleep(K_MINUTES(1));
		if (vr_test_mode_flag)
			LOG_INF("VR TEST MODE is running! vr test mode flag: %d",
				vr_test_mode_flag);
	}
}

void init_vr_test_mode_polling(void)
{
	if (is_mb_dc_on()) {
		if (check_vr_slow_ocp_match_test_mode()) {
			vr_test_mode_enable(true);
			LOG_INF("Slow OCP value is the same, start VR TEST MODE");
		}
	}

	vr_test_mode_tid = k_thread_create(&vr_test_mode_thread, vr_test_mode_thread_stack,
					   K_THREAD_STACK_SIZEOF(vr_test_mode_thread_stack),
					   vr_test_mode_handler, NULL, NULL, NULL,
					   CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&vr_test_mode_thread, "vr_test_mode_mode_thread");
}
