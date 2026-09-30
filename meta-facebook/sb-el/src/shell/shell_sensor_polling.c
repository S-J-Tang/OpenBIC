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

#include <zephyr.h>
#include <stdlib.h>
#include <shell/shell.h>
#include "plat_pldm_sensor.h"
#include "plat_user_setting.h"
#include "plat_cpld.h"

static void print_plat_sensor_polling_status(const struct shell *shell)
{
	shell_print(
		shell,
		"get_sensor_polling all -> %d , ubc -> %d, ina238 -> %d, vr -> %d, temp -> %d, cpld -> %d ",
		get_plat_sensor_polling_enable_flag(), get_plat_sensor_ubc_polling_enable_flag(),
		get_plat_sensor_ina238_polling_enable_flag(),
		get_plat_sensor_vr_polling_enable_flag(),
		get_plat_sensor_temp_polling_enable_flag(), get_cpld_polling_enable_flag());
}

void cmd_set_plat_sensor_polling_all(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: set_sensor_polling all <value>");
		return;
	}
	int value = strtol(argv[1], NULL, 10);
	if (value != 0 && value != 1) {
		shell_warn(shell, "Help: set_sensor_polling all value should only accept 0 or 1");
		return;
	}

	set_plat_sensor_polling_enable_flag(value);
	shell_print(shell, "set_sensor_polling all -> %d ,success!", value);
	print_plat_sensor_polling_status(shell);
	shell_print(shell, "Note: all does not include CPLD polling");
	return;
}

void cmd_set_plat_sensor_polling_ubc(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: set_sensor_polling ubc <value>");
		return;
	}
	int value = strtol(argv[1], NULL, 10);
	if (value != 0 && value != 1) {
		shell_warn(shell, "Help: set_sensor_polling ubc value should only accept 0 or 1");
		return;
	}

	set_plat_sensor_ubc_polling_enable_flag(value);
	shell_print(shell, "set_sensor_polling ubc -> %d ,success!", value);
	print_plat_sensor_polling_status(shell);
	return;
}

void cmd_set_plat_sensor_polling_vr(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: set_sensor_polling vr <value>");
		return;
	}
	int value = strtol(argv[1], NULL, 10);
	if (value != 0 && value != 1) {
		shell_warn(shell, "Help: set_sensor_polling vr value should only accept 0 or 1");
		return;
	}

	set_plat_sensor_vr_polling_enable_flag(value);
	shell_print(shell, "set_sensor_polling vr -> %d ,success!", value);
	print_plat_sensor_polling_status(shell);
	return;
}

void cmd_set_plat_sensor_polling_ina238(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: set_sensor_polling ina238 <value>");
		return;
	}
	int value = strtol(argv[1], NULL, 10);
	if (value != 0 && value != 1) {
		shell_warn(shell,
			   "Help: set_sensor_polling ina238 value should only accept 0 or 1");
		return;
	}

	set_plat_sensor_ina238_polling_enable_flag(value);
	shell_print(shell, "set_sensor_polling ina238 -> %d ,success!", value);
	print_plat_sensor_polling_status(shell);
}

void cmd_set_plat_sensor_polling_temp(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: set_sensor_polling temp <value>");
		return;
	}
	int value = strtol(argv[1], NULL, 10);
	if (value != 0 && value != 1) {
		shell_warn(shell, "Help: set_sensor_polling temp value should only accept 0 or 1");
		return;
	}

	set_plat_sensor_temp_polling_enable_flag(value);
	shell_print(shell, "set_sensor_polling temp -> %d ,success!", value);
	print_plat_sensor_polling_status(shell);
	return;
}

void cmd_set_plat_cpld_polling(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_warn(shell, "Help: set_sensor_polling temp <value>");
		return;
	}
	int value = strtol(argv[1], NULL, 10);
	if (value != 0 && value != 1) {
		shell_warn(shell, "Help: set_sensor_polling cpld value should only accept 0 or 1");
		return;
	}

	set_cpld_polling_enable_flag(value);
	shell_print(shell, "set_cpld_polling -> %d ,success!", value);
	print_plat_sensor_polling_status(shell);
	return;
}

void cmd_get_plat_sensor_polling_all(const struct shell *shell, size_t argc, char **argv)
{
	print_plat_sensor_polling_status(shell);
	return;
}

/* Sub-command Level 3 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(
	cmd_set_plat_sensor_polling,
	SHELL_CMD(all, NULL, "set platform sensor polling all", cmd_set_plat_sensor_polling_all),
	SHELL_CMD(ubc, NULL, "set platform sensor polling ubc", cmd_set_plat_sensor_polling_ubc),
	SHELL_CMD(vr, NULL, "set platform sensor polling vr", cmd_set_plat_sensor_polling_vr),
	SHELL_CMD(temp, NULL, "set platform sensor polling temp", cmd_set_plat_sensor_polling_temp),
	SHELL_CMD(ina238, NULL, "set platform sensor polling ina238",
		  cmd_set_plat_sensor_polling_ina238),
	SHELL_CMD(cpld, NULL, "set platform cpld polling", cmd_set_plat_cpld_polling),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(cmd_get_plat_sensor_polling,
			       SHELL_CMD(all, NULL, "get platform sensor polling all",
					 cmd_get_plat_sensor_polling_all),
			       SHELL_SUBCMD_SET_END);

/* Sub-command Level 2 of command test */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_plat_sensor_polling_cmd,
			       SHELL_CMD(set, &cmd_set_plat_sensor_polling,
					 "set platform sensor polling", NULL),
			       SHELL_CMD(get, &cmd_get_plat_sensor_polling,
					 "get platform sensor polling", NULL),
			       SHELL_SUBCMD_SET_END);

/* Root of command test */
SHELL_CMD_REGISTER(set_sensor_polling, &sub_plat_sensor_polling_cmd,
		   "Disable/Enable sensor polling for group of sensors", NULL);

static int cmd_sensor_poll_rate_get(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	uint16_t poll_ms;
	if (!plat_pldm_sensor_get_load_test_poll_interval(&poll_ms)) return -1;
	shell_print(shell, "Adjustable sensor polling rate: %u ms", poll_ms);
	return 0;
}

static int cmd_sensor_poll_rate_set(const struct shell *shell, size_t argc, char **argv)
{
	char *end = NULL;
	unsigned long poll_ms = strtoul(argv[1], &end, 0);
	if (!argv[1][0] || *end || poll_ms < 1 || poll_ms > 1000) { shell_error(shell, "poll_ms must be between 1 and 1000"); return -1; }
	bool is_perm = argc == 3;
	if (is_perm && strcmp(argv[2], "perm")) { shell_error(shell, "The last argument must be <perm>"); return -1; }
	if (!set_sensor_poll_rate_user_settings((uint16_t)poll_ms, is_perm)) { shell_error(shell, "Failed to set sensor polling rate"); return -1; }
	shell_print(shell, "Set adjustable sensor polling rate=%lu ms%s", poll_ms, is_perm ? " permanently" : "");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sensor_poll_rate_cmds,
	SHELL_CMD(get, NULL, "get adjustable sensor polling rate", cmd_sensor_poll_rate_get),
	SHELL_CMD_ARG(set, NULL, "set <poll_ms> [perm]", cmd_sensor_poll_rate_set, 2, 1),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(sensor_poll_rate, &sub_sensor_poll_rate_cmds, "Get/set adjustable sensor polling rate", NULL);
