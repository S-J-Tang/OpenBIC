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

#include <stdlib.h>
#include <shell/shell.h>

#include "plat_pldm_sensor.h"
#include "plat_cpld.h"
#include "plat_class.h"
#include "plat_adc.h"
#include "plat_hook.h"

// test command
void cmd_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "Hello world!");
	// uvp
	uint16_t medha0_uvp_threshold = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD, &medha0_uvp_threshold, UVP_THRESHOLD);
	shell_print(shell, "medha0_uvp_threshold = %d", medha0_uvp_threshold);
	uint16_t medha1_uvp_threshold = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD, &medha1_uvp_threshold, UVP_THRESHOLD);
	shell_print(shell, "medha1_uvp_threshold = %d", medha1_uvp_threshold);
	// vout max
	uint16_t medha0_vout_max = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD, &medha0_vout_max, VOUT_MAX);
	shell_print(shell, "medha0_vout_max = %d", medha0_vout_max);
	uint16_t medha1_vout_max = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD, &medha1_vout_max, VOUT_MAX);
	shell_print(shell, "medha1_vout_max = %d", medha1_vout_max);
	// vout command
	uint16_t medha0_vout_command = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD, &medha0_vout_command, VOUT_COMMAND);
	shell_print(shell, "medha0_vout_command = %d", medha0_vout_command);
	uint16_t medha1_vout_command = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD, &medha1_vout_command, VOUT_COMMAND);
	shell_print(shell, "medha1_vout_command = %d", medha1_vout_command);
	// Vout offset
	uint16_t medha0_vout_offset = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD, &medha0_vout_offset, VOUT_OFFSET);
	shell_print(shell, "medha0_vout_offset = %d", medha0_vout_offset);
	uint16_t medha1_vout_offset = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD, &medha1_vout_offset, VOUT_OFFSET);
	shell_print(shell, "medha1_vout_offset = %d", medha1_vout_offset);
	// total OCP
	uint16_t medha0_total_ocp = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD, &medha0_total_ocp, TOTAL_OCP);
	shell_print(shell, "medha0_total_ocp = %d", medha0_total_ocp);
	uint16_t medha1_total_ocp = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD, &medha1_total_ocp, TOTAL_OCP);
	shell_print(shell, "medha1_total_ocp = %d", medha1_total_ocp);
	// ovp1
	uint16_t medha0_ovp_1 = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA0_VDD, &medha0_ovp_1, OVP_1);
	shell_print(shell, "medha0_ovp_1 = %d", medha0_ovp_1);
	uint16_t medha1_ovp_1 = 0;
	get_vr_mp29816a_reg(VR_RAIL_E_ASIC_P0V85_MEDHA1_VDD, &medha1_ovp_1, OVP_1);
	shell_print(shell, "medha1_ovp_1 = %d", medha1_ovp_1);
}

void cmd_read_raw(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t sensor_id = strtoul(argv[1], NULL, 16);
	uint8_t offset = strtoul(argv[2], NULL, 16);
	uint8_t len = strtoul(argv[3], NULL, 10);

	if (!len)
		len = 1;
	uint8_t data[len];
	memset(data, 0, len);

	if ((sensor_id == 0) || (sensor_id >= SENSOR_NUM_NUMBERS)) {
		if (!plat_read_cpld(offset, data, 1)) {
			shell_warn(shell, "cpld read 0x%02x fail", offset);
			return;
		}
	} else {
		if (!get_raw_data_from_sensor_id(sensor_id, offset, data, len)) {
			shell_warn(shell, "sensor_id 0x%02x read 0x%02x fail", sensor_id, offset);
			return;
		}
	}

	shell_hexdump(shell, data, len);
	shell_print(shell, "");
}

void cmd_read_info(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t sensor_id = strtoul(argv[1], NULL, 16);

	sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
	if (cfg == NULL)
		return;

	shell_print(shell, "sensor_id 0x%02x bus: %d, addr: 0x%x(0x%x)", sensor_id, cfg->port,
		    cfg->target_addr, (cfg->target_addr >> 1));
}

void cmd_cpld_dump(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test cpld dump <offset> <length>");
		return;
	}

	uint8_t offset = strtoul(argv[1], NULL, 16);
	uint8_t len = strtoul(argv[2], NULL, 10);

	if (!len)
		len = 1;
	uint8_t data[len];
	memset(data, 0, len);

	if (!plat_read_cpld(offset, data, len)) {
		shell_warn(shell, "cpld read 0x%02x fail", offset);
		return;
	}

	shell_hexdump(shell, data, len);
	shell_print(shell, "");
}
void cmd_cpld_write(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_warn(shell, "Help: test cpld write <offset> <data>");
		return;
	}

	uint8_t offset = strtoul(argv[1], NULL, 16);
	uint8_t data = strtoul(argv[2], NULL, 16);

	if (!plat_write_cpld(offset, &data)) {
		shell_warn(shell, "cpld write 0x%02x fail", offset);
		return;
	}

	shell_warn(shell, "cpld write %02x to offset %02x", data, offset);
}

void cmd_info(const struct shell *shell, size_t argc, char **argv)
{
	static const char *const vr_module_str[] = {
		[VR_MODULE_MPS] = "MPS",
		[VR_MODULE_RNS] = "RNS",
	};

	static const char *const ubc_module_str[] = {
		[UBC_MODULE_DELTA] = "DELTA",
		[UBC_MODULE_MPS] = "MPS",
		[UBC_MODULE_FLEX] = "FLEX",
		[UBC_MODULE_LUXSHARE] = "LUXSHARE",
	};

	static const char *const asic_board_id_str[] = {
		[ASIC_BOARD_ID_RSVD1] = "RSVD1",
		[ASIC_BOARD_ID_RSVD2] = "RSVD2",
		[ASIC_BOARD_ID_RAINBOW] = "RAINBOW",
		[ASIC_BOARD_ID_EVB] = "EVB",
	};

	uint8_t vr = get_vr_module();
	uint8_t ubc = get_ubc_module();
	uint8_t board_id = get_asic_board_id();
	uint8_t board_rev = get_board_rev_id();
	uint8_t adc_idx = get_adc_type();
	uint8_t tray_loc = get_tray_location();

	shell_warn(shell, "vr module: %s",
		   (vr < VR_MODULE_UNKNOWN) ? vr_module_str[vr] : "UNKNOWN");
	shell_warn(shell, "ubc module: %s",
		   (ubc < UBC_MODULE_UNKNOWN) ? ubc_module_str[ubc] : "UNKNOWN");
	shell_warn(shell, "mmc slot: %d", get_mmc_slot() + 1);
	shell_warn(shell, "asic board id: %s",
		   (board_id < ASIC_BOARD_ID_UNKNOWN) ? asic_board_id_str[board_id] : "UNKNOWN");
	shell_warn(shell, "asic board rev id: %d", board_rev);
	shell_warn(shell, "adc idx: %d (0:ADI, 1:TI)", adc_idx);
	shell_warn(shell, "tray location: %d", tray_loc);
}

void cmd_mp2971_set_ocp(const struct shell *shell, size_t argc, char **argv)
{
	/* Usage: test mp2971_set_ocp <total_ocp> */
	if (argc != 2) {
		shell_warn(shell, "Usage: test mp2971_set_ocp <total_ocp>");
		return;
	}

	uint16_t ocp_set = (uint16_t)strtoul(argv[1], NULL, 0);

	/* Target rail for test */
	uint8_t rail = VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD;

	shell_print(shell, "MP2971 set TOTAL_OCP: rail=%u (%s), ocp_set=%u",
		    rail, "VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD", ocp_set);

	/* Write */
	int ret = set_vr_mp2971_reg(rail, &ocp_set, TOTAL_OCP);
	if (ret != 0) {
		shell_warn(shell, "set_vr_mp2971_reg(TOTAL_OCP) failed, ret=%d", ret);
		return;
	}

	/* Read back */
	uint16_t ocp_rb = 0;
	ret = get_vr_mp2971_reg(rail, &ocp_rb, TOTAL_OCP);
	if (ret != 0) {
		shell_warn(shell, "get_vr_mp2971_reg(TOTAL_OCP) failed, ret=%d", ret);
		return;
	}

	shell_print(shell, "MP2971 TOTAL_OCP readback: %u", ocp_rb);
}

void cmd_mp2971_set_ovp2_mode(const struct shell *shell, size_t argc, char **argv)
{
	/* Usage: test mp2971_set_ovp2_mode <mode>
	 * mode: 0=No action, 1=Latch-off
	 */
	if (argc != 2) {
		shell_warn(shell, "Usage: test mp2971_set_ovp2_mode <mode(0:NoAction 1:LatchOff)>");
		return;
	}

	uint16_t mode_u16 = (uint16_t)strtoul(argv[1], NULL, 0);

	uint8_t rail = VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD;

	int ret = set_vr_mp2971_reg(rail, &mode_u16, OVP_2_MODE);
	if (ret != 0) {
		shell_warn(shell, "set_vr_mp2971_reg(OVP_2_MODE) failed, ret=%d", ret);
		return;
	}

	shell_print(shell, "MP2971 set OVP2 mode done: rail=%u mode=%u", rail, mode_u16);
}

void cmd_mp2971_get_ovp2_mode(const struct shell *shell, size_t argc, char **argv)
{
	/* Usage: test mp2971_get_ovp2_mode */
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint8_t rail = VR_RAIL_E_ASIC_P0V9_OWL_W_TRVDD;

	uint16_t mode_u16 = 0xFFFF;
	int ret = get_vr_mp2971_reg(rail, &mode_u16, OVP_2_MODE);
	if (ret != 0) {
		shell_warn(shell, "get_vr_mp2971_reg(OVP_2_MODE) failed, ret=%d", ret);
		return;
	}

	const char *mode_str = "UNKNOWN";
	if (mode_u16 == 0) {
		mode_str = "NO_ACTION";
	} else if (mode_u16 == 1) {
		mode_str = "LATCH_OFF";
	}

	shell_print(shell, "MP2971 get OVP2 mode: rail=%u mode=%u (%s)", rail, mode_u16, mode_str);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cpld_cmds, SHELL_CMD(dump, NULL, "cpld dump", cmd_cpld_dump),
			       SHELL_CMD(write, NULL, "write cpld register", cmd_cpld_write),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_test_cmds, SHELL_CMD(test, NULL, "test command", cmd_test),
	SHELL_CMD(mp2971_set_ocp, NULL, "set MP2971 TOTAL_OCP on OWL_W_TRVDD", cmd_mp2971_set_ocp),
	SHELL_CMD(mp2971_get_ovp2_mode, NULL, "get MP2971 OVP2 action mode (0:NoAction 1:LatchOff)",
	  cmd_mp2971_get_ovp2_mode),
	SHELL_CMD(mp2971_set_ovp2_mode, NULL, "set MP2971 OVP2 action mode (0:NoAction 1:LatchOff)",
		cmd_mp2971_set_ovp2_mode),
	SHELL_CMD(read_raw, NULL, "read raw data test command", cmd_read_raw),
	SHELL_CMD(read_info, NULL, "read sensor info test command", cmd_read_info),
	SHELL_CMD(cpld, &sub_cpld_cmds, "cpld commands", NULL),
	SHELL_CMD(info, NULL, "info commands", cmd_info), SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(test, &sub_test_cmds, "Test commands", NULL);
