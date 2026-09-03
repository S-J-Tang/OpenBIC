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

#include <shell/shell.h>
#include <stdlib.h>
#include <logging/log.h>
#include "sensor.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_event.h"
#include "plat_cpld.h"
#include "plat_vr_test_mode.h"
#include "plat_user_setting.h"

LOG_MODULE_REGISTER(plat_voltage_shell, LOG_LEVEL_DBG);

static int cmd_voltage_get_all(const struct shell *shell, size_t argc, char **argv)
{
	/* is_ubc_enabled_delayed_enabled() is to wait for all VR to be enabled  */
	/* (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) is to shut down polling immediately when UBC is disabled */
	// (get_is_ubc_enabled() && is_ubc_enabled_delayed_enabled())
	if (!is_mb_dc_on()) {
		shell_error(shell, "Can't get voltage command because VR has no power yet.");
		return -1;
	}

	shell_print(shell, "  id|              sensor_name               |vout(mV) ");
	/* list all vr sensor value */
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (((get_asic_board_id() != ASIC_BOARD_ID_EVB)) &&
		    (i == VR_RAIL_E_P3V3_OSFP_VOLT_V))
			continue; // skip osfp p3v3 on BD

		uint16_t vout = 0;
		uint8_t *rail_name = NULL;
		if (!vr_rail_name_get((uint8_t)i, &rail_name)) {
			shell_print(shell, "Can't find vr_rail_name by rail index: %d", i);
			continue;
		}

		if (!plat_get_vout_command(i, &vout)) {
			shell_print(shell, "Can't find vout by rail index: %d", i);
			continue;
		}

		shell_print(shell, "%4d|%-40s|%4d", i, rail_name, vout);
	}

	return 0;
}

static int cmd_voltage_set(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;

	/* is_ubc_enabled_delayed_enabled() is to wait for all VR to be enabled  */
	/* (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) is to shut down polling immediately when UBC is disabled */
	// (get_is_ubc_enabled() && is_ubc_enabled_delayed_enabled())
	if (!is_mb_dc_on()) {
		shell_error(shell, "Can't set voltage command because VR has no power yet.");
		return -1;
	}

	if (argc >= 4) {
		if (!strcmp(argv[3], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}

	/* covert rail string to enum */
	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return -1;
	}

	uint16_t millivolt = strtol(argv[2], NULL, 0);

	/* Apply SVS offset if enabled */
	uint8_t svs_flag = get_svs_flag();
	if (svs_flag) {
		uint16_t vout_offset = 0;
		if (voltage_offset_get(rail, &vout_offset)) {
			millivolt += vout_offset;
		} else {
			shell_warn(shell, "SVS enabled but failed to get voltage offset");
		}
	}

	uint16_t vout_max_millivolt = vout_range_user_settings.change_vout_max[rail];
	uint16_t vout_min_millivolt = vout_range_user_settings.change_vout_min[rail];
	if (millivolt < vout_min_millivolt || millivolt > vout_max_millivolt) {
		shell_error(shell, "vout[%d] cannot be less than %dmV or greater than %dmV", rail,
			    vout_min_millivolt, vout_max_millivolt);
		return -1;
	}
	// can't set voltage for osfp p3v3
	if (rail == VR_RAIL_E_P3V3_OSFP_VOLT_V) {
		shell_warn(shell, "OSFP P3V3 can't set voltage");
		return -1;
	}
	shell_info(shell, "Set %s(%d) to %d mV, %svolatile\n", argv[1], rail, millivolt,
		   (argc == 4) ? "non-" : "");

	/* set the vout */
	if ((get_asic_board_id() != ASIC_BOARD_ID_EVB) && (rail == VR_RAIL_E_P3V3_OSFP_VOLT_V)) {
		shell_print(shell, "There is no osfp p3v3");
		return 0;
	}

	if (!plat_set_vout_command(rail, &millivolt, is_perm)) {
		shell_error(shell, "Can't set vout by rail index: %d", rail);
		return -1;
	}

	return 0;
}

static void voltage_rname_get(size_t idx, struct shell_static_entry *entry)
{
	if (((get_asic_board_id() != ASIC_BOARD_ID_EVB)) && (idx == VR_RAIL_E_P3V3_OSFP_VOLT_V))
		idx++;
	uint8_t *name = NULL;
	vr_rail_name_get((uint8_t)idx, &name);

	if (idx == VR_RAIL_E_P3V3_OSFP_VOLT_V) {
		return;
	}

	entry->syntax = (name) ? (const char *)name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

static void cmd_svs_flag_get(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t svs_flag = get_svs_flag();
	shell_print(shell, "voltage offset(1:enable, 0:disable) : %d", svs_flag);
}

static void cmd_svs_flag_set(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;

	if (argc >= 3) {
		if (!strcmp(argv[2], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return;
		}
	}

	if (argc < 2) {
		shell_error(shell, "Usage: set voltage offset <0/1> [perm]");
		return;
	}

	uint8_t svs_flag = strtol(argv[1], NULL, 0);
	//flag only support 0 or 1
	if (svs_flag > 1) {
		shell_error(shell, "Invalid voltage offset value: %d. Only 0 or 1 is allowed.",
			    svs_flag);
		return;
	}
	if (!set_svs_flag(svs_flag, is_perm)) {
		shell_error(shell, "Can't set voltage offset=%d", svs_flag);
		return;
	}
	shell_print(shell, "voltage offset(1:enable, 0:disable): %d, %svolatile\n", svs_flag,
		    (argc == 3) ? "non-" : "");
}

static void cmd_get_vout_offset(const struct shell *shell, size_t argc, char **argv)
{
	uint16_t vout_offset_value = 0;
	uint8_t *rail_name = NULL;

	shell_print(shell, "  id|rail_name                               |vout_offset(mV)");

	/* Print the cached VOUT offset for every supported ASIC rail. */
	for (int i = VR_RAIL_E_ASIC_P0V75_NUWA0_VDD; i <= VR_RAIL_E_ASIC_P0V75_OWL_W_TRVDD; i++) {
		vout_offset_value = 0;
		if (!voltage_offset_get((uint8_t)i, &vout_offset_value)) {
			shell_warn(shell, "Failed to get vout offset for rail index: %d", i);
			continue;
		}

		rail_name = NULL;
		if (!vr_rail_name_get((uint8_t)i, &rail_name)) {
			shell_warn(shell, "Failed to get rail name for index: %d", i);
			continue;
		}

		shell_print(shell, "%4d|%-40s|%4d", i, rail_name, vout_offset_value);
	}
}

void cmd_svs_asic_voltage_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(shell, "Usage: set svs_asic_voltage <0/1>");
		return;
	}

	uint8_t svs_asic_voltage_flag = strtol(argv[1], NULL, 0);
	//flag only support 0 or 1
	if (svs_asic_voltage_flag > 1) {
		shell_error(shell, "Only 0 or 1 is allowed.");
		return;
	}
	set_svs_asic_voltage_flag(svs_asic_voltage_flag);
	shell_print(shell, "svs asic voltage setting(1:apply, 0:block): %d", svs_asic_voltage_flag);
}

void cmd_svs_asic_voltage_get(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t svs_asic_voltage_flag = get_svs_asic_voltage_flag();
	shell_print(shell, "svs asic voltage setting(1:apply, 0:block): %d", svs_asic_voltage_flag);
}

static int cmd_voffset_mmc_get(const struct shell *shell, size_t argc, char **argv)
{
	/* is_ubc_enabled_delayed_enabled() is to wait for all VR to be enabled  */
	/* (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) is to shut down polling immediately when UBC is disabled */
	// (get_is_ubc_enabled() && is_ubc_enabled_delayed_enabled())
	if (!is_mb_dc_on()) {
		shell_error(shell, "Can't get Voffset_mmc command because VR has no power yet.");
		return -1;
	}

	shell_print(shell, "  id|              sensor_name               |Voffset_mmc(mV) ");
	/* list all vr sensor value */
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (((get_asic_board_id() != ASIC_BOARD_ID_EVB)) &&
		    (i == VR_RAIL_E_P3V3_OSFP_VOLT_V))
			continue; // skip osfp p3v3 on BD

		uint8_t *rail_name = NULL;
		if (!vr_rail_name_get((uint8_t)i, &rail_name)) {
			shell_print(shell, "Can't find vr_rail_name by rail index: %d", i);
			continue;
		}

		shell_print(shell, "%4d|%-40s|%4d", i, rail_name,
			    vr_voffset_mmc_command_get.voffset_mmc[i]);
	}

	return 0;
}

static bool ovp_uvp_check(const struct shell *shell, const char *rail_str, enum VR_RAIL_E *rail)
{
	// (get_is_ubc_enabled() && is_ubc_enabled_delayed_enabled())
	if (!is_mb_dc_on()) {
		shell_error(shell, "VR no power");
		return false;
	}
	if (get_vr_module() != VR_MODULE_MPS) {
		shell_error(shell, "MPS support only");
		return false;
	}
	if (!vr_rail_enum_get((uint8_t *)rail_str, (uint8_t *)rail)) {
		shell_error(shell, "Invalid rail: %s", rail_str);
		return false;
	}
	if (*rail != VR_RAIL_E_ASIC_P0V75_NUWA0_VDD && *rail != VR_RAIL_E_ASIC_P0V75_NUWA1_VDD) {
		shell_error(shell, "Please input NUWA0/1 voltage rail");
		return false;
	}
	return true;
}

static int cmd_ovp_get(const struct shell *shell, size_t argc, char **argv)
{
	enum VR_RAIL_E rail;
	if (!ovp_uvp_check(shell, argv[1], &rail))
		return -1;
	uint16_t val = 0;
	if (get_vr_mp29816a_reg(rail, &val, OVP_1) != 0) {
		shell_error(shell, "OVP get fail");
		return -1;
	}
	shell_print(shell, "OVP %s: %d mV", argv[1], val);
	return 0;
}

static int cmd_ovp_set(const struct shell *shell, size_t argc, char **argv)
{
	enum VR_RAIL_E rail;
	if (!ovp_uvp_check(shell, argv[1], &rail))
		return -1;
	uint16_t val = (uint16_t)strtol(argv[2], NULL, 0);
	if (set_vr_mp29816a_reg(rail, &val, OVP_1) != 0) {
		shell_error(shell, "OVP set fail");
		return -1;
	}
	shell_print(shell, "OVP %s: %d mV", argv[1], val);
	return 0;
}

static int cmd_uvp_get(const struct shell *shell, size_t argc, char **argv)
{
	enum VR_RAIL_E rail;
	if (!ovp_uvp_check(shell, argv[1], &rail))
		return -1;
	uint16_t val = 0;
	if (get_vr_mp29816a_reg(rail, &val, UVP) != 0) {
		shell_error(shell, "UVP get fail");
		return -1;
	}
	shell_print(shell, "UVP %s: %d mV", argv[1], val);
	return 0;
}

static int cmd_uvp_set(const struct shell *shell, size_t argc, char **argv)
{
	enum VR_RAIL_E rail;
	if (!ovp_uvp_check(shell, argv[1], &rail))
		return -1;
	uint16_t target = (uint16_t)strtol(argv[2], NULL, 0);
	uint16_t vout_cmd = 0;
	if (get_vr_mp29816a_reg(rail, &vout_cmd, VOUT_COMMAND) != 0) {
		shell_error(shell, "UVP set fail");
		return -1;
	}
	uint16_t max_uvp = vout_cmd;
	uint16_t min_uvp = vout_cmd - 500;
	if (target >= vout_cmd) {
		shell_error(shell, "UVP target out of range");
		shell_error(shell, "Valid UVP range: %d to %d mV", min_uvp, max_uvp);
		shell_error(shell, "Supported points: %d, %d, %d, ... , %d in 50mV steps", max_uvp,
			    max_uvp - 100, max_uvp - 150, min_uvp);
		return -1;
	}
	uint16_t offset = vout_cmd - target;
	if (offset < 100 || offset > 500) {
		shell_error(shell, "UVP target out of range");
		shell_error(shell, "Valid UVP range: %d to %d mV", min_uvp, max_uvp);
		shell_error(shell, "Supported points: %d, %d, %d, ... , %d in 50mV steps", max_uvp,
			    max_uvp - 100, max_uvp - 150, min_uvp);
		return -1;
	}
	if (((offset - 100) % 50) != 0) {
		shell_error(shell, "UVP target out of range");
		shell_error(shell, "Valid UVP range: %d to %d mV", min_uvp, max_uvp);
		shell_error(shell, "Supported points: %d, %d, %d, ... , %d in 50mV steps", max_uvp,
			    max_uvp - 100, max_uvp - 150, min_uvp);
		return -1;
	}
	if (set_vr_mp29816a_reg(rail, &offset, UVP_THRESHOLD) != 0) {
		shell_error(shell, "UVP set fail");
		return -1;
	}
	shell_print(shell, "UVP %s: %d mV", argv[1], target);
	return 0;
}

static int cmd_voffset_mmc_set(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;

	/* is_ubc_enabled_delayed_enabled() is to wait for all VR to be enabled  */
	/* (gpio_get(FM_PLD_UBC_EN_R) == GPIO_HIGH) is to shut down polling immediately when UBC is disabled */
	// (get_is_ubc_enabled() && is_ubc_enabled_delayed_enabled()
	if (!is_mb_dc_on()) {
		shell_error(shell, "Can't set Voffset_mmc command because VR has no power yet.");
		return -1;
	}

	if (argc >= 4) {
		if (!strcmp(argv[3], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}

	/* covert rail string to enum */
	enum VR_RAIL_E rail;
	if (vr_rail_enum_get(argv[1], &rail) == false) {
		shell_error(shell, "Invalid rail name: %s", argv[1]);
		return -1;
	}

	int16_t millivolt = strtol(argv[2], NULL, 0);

	// can't set voltage for osfp p3v3
	if (rail == VR_RAIL_E_P3V3_OSFP_VOLT_V) {
		shell_warn(shell, "OSFP P3V3 can't set voltage");
		return -1;
	}
	shell_info(shell, "Set %s(%d) to %d mV, %svolatile\n", argv[1], rail, millivolt,
		   (argc == 4) ? "non-" : "");

	/* set the vout */
	if ((get_asic_board_id() != ASIC_BOARD_ID_EVB) && (rail == VR_RAIL_E_P3V3_OSFP_VOLT_V)) {
		shell_print(shell, "There is no osfp p3v3");
		return 0;
	}

	if (!plat_set_voffset_mmc_command(rail, &millivolt, is_perm)) {
		shell_error(shell, "Can't set Voffset_mmc by rail index: %d", rail);
		return -1;
	}

	return 0;
}

static bool svs_voltage_range_check(const struct shell *shell, const char *rail_str,
				    enum VR_RAIL_E *rail)
{
	if (!is_mb_dc_on()) {
		shell_error(shell, "VR no power");
		return false;
	}

	if (!vr_rail_enum_get((uint8_t *)rail_str, (uint8_t *)rail)) {
		shell_error(shell, "Invalid rail: %s", rail_str);
		return false;
	}

	if (*rail != VR_RAIL_E_ASIC_P0V75_NUWA0_VDD && *rail != VR_RAIL_E_ASIC_P0V75_NUWA1_VDD) {
		shell_error(shell, "Please input NUWA0/1 voltage rail");
		return false;
	}

	return true;
}

static int cmd_svs_voltage_range_get(const struct shell *shell, size_t argc, char **argv)
{
	for (int i = VR_RAIL_E_ASIC_P0V75_NUWA0_VDD; i <= VR_RAIL_E_ASIC_P0V75_NUWA1_VDD; i++) {
		uint8_t *rail_name = NULL;
		if (!vr_rail_name_get((uint8_t)i, &rail_name)) {
			shell_print(shell, "Can't find vr_rail_name by rail index: %d", i);
			continue;
		}

		uint16_t vmin = svs_voltage_range_command_get.vout_min[i];
		uint16_t vmax = svs_voltage_range_command_get.vout_max[i];
		shell_print(shell, "%4d|%-40s|min:%4dmV max:%4dmV", i, rail_name, vmin, vmax);
	}

	return 0;
}

static int cmd_svs_voltage_range_set(const struct shell *shell, size_t argc, char **argv)
{
	bool is_perm = false;

	if (argc >= 5) {
		if (!strcmp(argv[4], "perm")) {
			is_perm = true;
		} else {
			shell_error(shell, "The last argument must be <perm>");
			return -1;
		}
	}

	if (argc < 4) {
		shell_error(shell, "Usage: svs_voltage_range set <rail> <min-mV> <max-mV> [perm]");
		return -1;
	}

	enum VR_RAIL_E rail;
	if (!svs_voltage_range_check(shell, argv[1], &rail))
		return -1;

	uint16_t set_value_min = strtol(argv[2], NULL, 10);
	uint16_t set_value_max = strtol(argv[3], NULL, 10);
	if (set_value_min > set_value_max) {
		shell_error(shell, "min value should be less than max value");
		return -1;
	}

	svs_voltage_range_command_get.vout_min[rail] = set_value_min;
	svs_voltage_range_command_get.vout_max[rail] = set_value_max;

	if (is_perm) {
		svs_voltage_range_user_settings.vout_min[rail] = set_value_min;
		svs_voltage_range_user_settings.vout_max[rail] = set_value_max;
		if (!set_user_settings_svs_voltage_range_to_eeprom(
			    &svs_voltage_range_user_settings,
			    sizeof(svs_voltage_range_user_settings))) {
			shell_error(shell, "Can't set svs_voltage_range user settings");
			return -1;
		}
	}

	shell_info(shell, "Set %s min:%4dmV max:%4dmV, %svolatile", argv[1], set_value_min,
		   set_value_max, is_perm ? "non-" : "");

	return 0;
}

SHELL_DYNAMIC_CMD_CREATE(voltage_rname, voltage_rname_get);

/* OVP/UVP level 2 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_ovp_cmds, SHELL_CMD_ARG(get, &voltage_rname, "get OVP <rail>", cmd_ovp_get, 2, 0),
	SHELL_CMD_ARG(set, &voltage_rname, "set OVP <rail> <mV>", cmd_ovp_set, 3, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_uvp_cmds, SHELL_CMD_ARG(get, &voltage_rname, "get UVP <rail>", cmd_uvp_get, 2, 0),
	SHELL_CMD_ARG(set, &voltage_rname, "set UVP <rail> <mV>", cmd_uvp_set, 3, 0),
	SHELL_SUBCMD_SET_END);

/* level 2 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_voltage_get_cmds,
			       SHELL_CMD(all, NULL, "get voltage all vout command",
					 cmd_voltage_get_all),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_svs_cmds, SHELL_CMD(get, NULL, "get svs flag", cmd_svs_flag_get),
			       SHELL_CMD_ARG(set, NULL, "set svs flag <0/1> [perm]",
					     cmd_svs_flag_set, 2, 1),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_svs_asic_voltage,
			       SHELL_CMD(set, NULL, "set svs asic voltage: apply or block",
					 cmd_svs_asic_voltage_set),
			       SHELL_CMD(get, NULL, "get svs asic voltage: apply or block",
					 cmd_svs_asic_voltage_get),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_voffset_mmc_cmds,
			       SHELL_CMD_ARG(set, &voltage_rname,
					     "voffset_mmc set  <voltage-rail> <new-voltage> [perm]",
					     cmd_voffset_mmc_set, 3, 1),
			       SHELL_CMD(get, NULL, "voffset_mmc get", cmd_voffset_mmc_get),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_svs_voltage_range_cmds,
	SHELL_CMD(get, NULL, "get NUWA0/1 SVS voltage ranges", cmd_svs_voltage_range_get),
	SHELL_CMD_ARG(set, &voltage_rname, "svs_voltage_range set <rail> <min-mV> <max-mV> [perm]",
		      cmd_svs_voltage_range_set, 4, 1),
	SHELL_SUBCMD_SET_END);

/* level 1 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_voltage_cmds, SHELL_CMD(get, &sub_voltage_get_cmds, "get voltage all", NULL),
	SHELL_CMD_ARG(set, &voltage_rname, "set <voltage-rail> <new-voltage> [perm]",
		      cmd_voltage_set, 3, 1),
	SHELL_CMD(svs_apply_offset, &sub_svs_cmds, "svs apply commands", NULL),
	SHELL_CMD(svs_asic_voltage, &sub_svs_asic_voltage, "svs asic voltage setting commands",
		  NULL),
	SHELL_CMD(get_vout_offset, NULL, "get ASIC rail VOUT offsets", cmd_get_vout_offset),
	SHELL_CMD(voffset_mmc, &sub_voffset_mmc_cmds, "Voffset_mmc set/get commands", NULL),
	SHELL_CMD(ovp, &sub_ovp_cmds, "OVP get/set commands (MPS NUWA0/1 only)", NULL),
	SHELL_CMD(uvp, &sub_uvp_cmds, "UVP get/set commands (MPS NUWA0/1 only)", NULL),
	SHELL_CMD(svs_voltage_range, &sub_svs_voltage_range_cmds,
		  "svs_voltage_range get/set commands (NUWA0/1 only)", NULL),
	SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(voltage, &sub_voltage_cmds, "voltage set/get commands", NULL);
