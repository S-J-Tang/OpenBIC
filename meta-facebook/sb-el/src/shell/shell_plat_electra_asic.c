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
#include <string.h>

#include "hal_i2c.h"
#include "plat_i2c.h"

#define ASIC_I2C_BUS I2C_BUS12
#define ASIC_I2C_ADDR 0x32
#define I2C_MAX_RETRY 3

#define ASIC_REG_STATUS_REG 0x00
#define ASIC_MODULE_STATUS_REG 0x60
#define ASIC_THERMAL_THRESHOLD_REG 0x62
#define ASIC_VERSION_REG 0x68
#define ASIC_MONITOR_TEMP_REG 0x70
#define ASIC_MONITOR_HBM_TEMP_REG 0x8F
#define ASIC_SVS_CORE_VOLT_REG 0x9B

#define ASIC_STATUS_REG_LEN 8
#define ASIC_MODULE_STATUS_REG_LEN 7
#define ASIC_THERMAL_THRESHOLD_LEN 2
#define ASIC_VERSION_REG_LEN 10
#define ASIC_MONITOR_TEMP_REG_LEN 10
#define ASIC_MONITOR_HBM_TEMP_REG_LEN 10
#define ASIC_SVS_CORE_VOLT_REG_LEN 6

static int asic_read_cmd(const struct shell *shell, uint8_t reg, uint8_t *data, uint8_t len)
{
	I2C_MSG i2c_msg = {
		.bus = ASIC_I2C_BUS,
		.target_addr = ASIC_I2C_ADDR,
		.tx_len = 1,
		.rx_len = len,
	};

	i2c_msg.data[0] = reg;
	if (i2c_master_read(&i2c_msg, I2C_MAX_RETRY)) {
		shell_error(shell, "Can't get data from ASIC, reg: 0x%02x", reg);
		return -1;
	}

	memcpy(data, i2c_msg.data, len);
	return 0;
}

static int asic_write_cmd(const struct shell *shell, uint8_t reg, const uint8_t *data, uint8_t len)
{
	I2C_MSG i2c_msg = {
		.bus = ASIC_I2C_BUS,
		.target_addr = ASIC_I2C_ADDR,
		.tx_len = len + 1,
	};

	i2c_msg.data[0] = reg;
	if (len > 0)
		memcpy(&i2c_msg.data[1], data, len);

	if (i2c_master_write(&i2c_msg, I2C_MAX_RETRY)) {
		shell_error(shell, "Can't set data to ASIC, reg: 0x%02x", reg);
		return -1;
	}

	return 0;
}

static void asic_boot_status_cmd(const struct shell *shell)
{
	uint8_t status_data[ASIC_STATUS_REG_LEN] = { 0 };

	if (asic_read_cmd(shell, ASIC_REG_STATUS_REG, status_data, sizeof(status_data))) {
		shell_warn(shell, "Can't get status data from ASIC");
		return;
	}

	shell_print(shell, "Boot status from ASIC: 0x%02x", status_data[1]);
	if (status_data[1] & BIT(6)) {
		shell_print(shell, "ASIC is not ready");
		return;
	}

	shell_print(shell, "ASIC is ready");
}

static void asic_version_cmd(const struct shell *shell)
{
	uint8_t version_data[ASIC_VERSION_REG_LEN] = { 0 };

	if (asic_read_cmd(shell, ASIC_VERSION_REG, version_data, sizeof(version_data))) {
		shell_warn(shell, "Can't get version data from ASIC");
		return;
	}

	shell_print(shell, "boot1 VER from ASIC: %02d.%02d.%02d", version_data[2], version_data[3],
		    version_data[4]);
	shell_print(shell, "boot0 VER from ASIC: %02d.%02d.%02d", version_data[9], version_data[8],
		    version_data[7]);
}

static void max_asic_temp_history_cmd(const struct shell *shell)
{
	uint8_t temp_data[ASIC_MONITOR_TEMP_REG_LEN] = { 0 };

	if (asic_read_cmd(shell, ASIC_MONITOR_TEMP_REG, temp_data, sizeof(temp_data))) {
		shell_warn(shell, "Can't get max ASIC temp data, reg: 0x%02x",
			   ASIC_MONITOR_TEMP_REG);
		return;
	}

	shell_print(shell, "  %-22s raw: 0x%02X -> %d degC", "hamsa_remote_temp", temp_data[1],
		    temp_data[1]);
	shell_print(shell, "  %-22s raw: 0x%02X -> %d degC", "nuwa0_remote_temp", temp_data[2],
		    temp_data[2]);
	shell_print(shell, "  %-22s raw: 0x%02X -> %d degC", "nuwa1_remote_temp", temp_data[3],
		    temp_data[3]);
	shell_print(shell, "  %-22s raw: 0x%02X -> %d degC", "owl_e_remote_temp", temp_data[4],
		    temp_data[4]);
	shell_print(shell, "  %-22s raw: 0x%02X -> %d degC", "owl_w_remote_temp", temp_data[5],
		    temp_data[5]);
	shell_print(shell, "  %-22s raw: 0x%02X -> %d degC", "max_asic_temp", temp_data[6],
		    temp_data[6]);

	uint8_t hbm_temp_data[ASIC_MONITOR_HBM_TEMP_REG_LEN] = { 0 };

	if (asic_read_cmd(shell, ASIC_MONITOR_HBM_TEMP_REG, hbm_temp_data, sizeof(hbm_temp_data))) {
		shell_warn(shell, "Can't get max ASIC HBM temp data, reg: 0x%02x",
			   ASIC_MONITOR_HBM_TEMP_REG);
		return;
	}

	static const char *const hbm_names[] = {
		"nuwa0_hbm0_remote_temp", "nuwa0_hbm1_remote_temp", "nuwa0_hbm2_remote_temp",
		"nuwa0_hbm3_remote_temp", "nuwa1_hbm0_remote_temp", "nuwa1_hbm1_remote_temp",
		"nuwa1_hbm2_remote_temp", "nuwa1_hbm3_remote_temp",
	};

	for (uint8_t i = 0; i < ARRAY_SIZE(hbm_names); i++) {
		shell_print(shell, "  %-22s raw: 0x%02X -> %d degC", hbm_names[i],
			    hbm_temp_data[i + 1], hbm_temp_data[i + 1]);
	}
}

static void asic_read_all_cmd(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc != 1) {
		shell_error(shell, "Usage: el_asic read_all");
		return;
	}

	shell_print(shell, "=== Electra ASIC Dump ===");
	shell_print(shell, "\n[1/3] System Boot Status");
	shell_print(shell, "============================================");
	asic_boot_status_cmd(shell);
	shell_print(shell, "\n[2/3] MAX ASIC Temperature History");
	shell_print(shell, "============================================");
	max_asic_temp_history_cmd(shell);
	shell_print(shell, "\n[3/3] Firmware Versions");
	shell_print(shell, "============================================");
	asic_version_cmd(shell);
	shell_print(shell, "\n=== Electra ASIC Dump Complete ===");
}

static void asic_read_svs_core_voltage_cmd(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc != 1) {
		shell_error(shell, "Usage: el_asic svs_core_voltage_get");
		return;
	}

	uint8_t data[ASIC_SVS_CORE_VOLT_REG_LEN] = { 0 };
	if (asic_read_cmd(shell, ASIC_SVS_CORE_VOLT_REG, data, sizeof(data))) {
		shell_warn(shell, "Can't get SVS core voltage data, reg: 0x%02x",
			   ASIC_SVS_CORE_VOLT_REG);
		return;
	}

	uint16_t nuwa0_mv = ((uint16_t)data[2] << 8) | data[1];
	uint16_t nuwa1_mv = ((uint16_t)data[4] << 8) | data[3];

	shell_print(shell, "SVS Core Voltage:");
	shell_print(shell, "  NUWA0: raw: 0x%02X%02X -> %d mV", data[2], data[1], nuwa0_mv);
	shell_print(shell, "  NUWA1: raw: 0x%02X%02X -> %d mV", data[4], data[3], nuwa1_mv);
}

static void asic_thermal_threshold_get_cmd(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc != 1) {
		shell_error(shell, "Usage: el_asic thermal_threshold get");
		return;
	}

	uint8_t data[ASIC_MODULE_STATUS_REG_LEN] = { 0 };
	if (asic_read_cmd(shell, ASIC_MODULE_STATUS_REG, data, sizeof(data))) {
		shell_warn(shell, "Can't get module status from ASIC, reg: 0x%02x",
			   ASIC_MODULE_STATUS_REG);
		return;
	}

	shell_print(shell,
		    "lower_thermal_threshold: 0x%02x, upper_thermal_threshold: 0x%02x, reg: 0x%02x",
		    data[2], data[3], ASIC_MODULE_STATUS_REG);
}

static void asic_thermal_threshold_set_cmd(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(
			shell,
			"Usage: el_asic thermal_threshold set <lower_threshold> <upper_threshold>");
		return;
	}

	char *end = NULL;
	unsigned long lower = strtoul(argv[1], &end, 0);
	if (*argv[1] == '\0' || *end != '\0' || lower > UINT8_MAX) {
		shell_error(shell, "Invalid lower threshold: %s", argv[1]);
		return;
	}

	unsigned long upper = strtoul(argv[2], &end, 0);
	if (*argv[2] == '\0' || *end != '\0' || upper > UINT8_MAX) {
		shell_error(shell, "Invalid upper threshold: %s", argv[2]);
		return;
	}

	uint8_t data[ASIC_THERMAL_THRESHOLD_LEN] = { (uint8_t)lower, (uint8_t)upper };
	if (asic_write_cmd(shell, ASIC_THERMAL_THRESHOLD_REG, data, sizeof(data))) {
		shell_warn(shell, "Can't set thermal threshold, reg: 0x%02x",
			   ASIC_THERMAL_THRESHOLD_REG);
		return;
	}

	shell_print(
		shell,
		"set lower_thermal_threshold: 0x%02x, upper_thermal_threshold: 0x%02x on reg 0x%02x success",
		data[0], data[1], ASIC_THERMAL_THRESHOLD_REG);
}

static void asic_help_cmd(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_info(shell, "Usage: el_asic read_all");
	shell_info(shell, "       el_asic svs_core_voltage_get");
	shell_info(shell, "       el_asic thermal_threshold get");
	shell_info(shell,
		   "       el_asic thermal_threshold set <lower_threshold> <upper_threshold>");
}

SHELL_STATIC_SUBCMD_SET_CREATE(asic_thermal_threshold_subcmds,
			       SHELL_CMD(get, NULL, "Get ASIC thermal thresholds",
					 asic_thermal_threshold_get_cmd),
			       SHELL_CMD(set, NULL, "Set ASIC lower and upper thermal thresholds",
					 asic_thermal_threshold_set_cmd),
			       SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_asic_cmds,
	SHELL_CMD(read_all, NULL, "Read all Electra ASIC system data", asic_read_all_cmd),
	SHELL_CMD(svs_core_voltage_get, NULL, "Get SVS core voltage from ASIC",
		  asic_read_svs_core_voltage_cmd),
	SHELL_CMD(thermal_threshold, &asic_thermal_threshold_subcmds,
		  "Get or set ASIC thermal thresholds", NULL),
	SHELL_CMD(help, NULL, "Display Electra ASIC command help", asic_help_cmd),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(el_asic, &sub_asic_cmds, "Electra ASIC low-level commands", NULL);
