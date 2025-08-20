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
#include <stdio.h>
#include "hal_i2c.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(dev_pex90144);

void get_pcie_tmp(struct shell *shell, size_t argc, char **argv)
{
	for (size_t i = 1; i < argc; i++) {
		printf("%s ", argv[i]);
	}
	printf("\n");

	I2C_MSG msg;
	msg.bus = 3;
	msg.target_addr = 0x6a;
	uint8_t retry = 5;
	
	uint8_t cmd1[8] = {0x03, 0xFC, 0x3C, 0x40, 0xFF, 0xE0, 0x00, 0x04};
	uint8_t cmd2[8] = {0x03, 0xFC, 0x3C, 0x41, 0x00, 0x21, 0x17, 0x0B};
	uint8_t cmd3[8] = {0x03, 0xFC, 0x3C, 0x42, 0x00, 0x00, 0x00, 0x01};

	uint8_t cmd4[8] = {0x03, 0xFC, 0x3C, 0x40, 0xFF, 0xE0, 0x00, 0x08};
	uint8_t cmd5[8] = {0x03, 0xFC, 0x3C, 0x41, 0x00, 0x01, 0x00, 0x07};
	uint8_t cmd6[8] = {0x03, 0xFC, 0x3C, 0x42, 0x00, 0x00, 0x00, 0x01};

	uint8_t cmd7[8] = {0x03, 0xFC, 0x3C, 0x40, 0xFF, 0xE0, 0x00, 0x0C};
	uint8_t cmd8[8] = {0x03, 0xFC, 0x3C, 0x41, 0x00, 0x22, 0x17, 0x14};
	uint8_t cmd9[8] = {0x03, 0xFC, 0x3C, 0x42, 0x00, 0x00, 0x00, 0x01};

	uint8_t cmd10[8] = {0x03, 0xFC, 0x3C, 0x40, 0xFF, 0xE0, 0x00, 0x10};
	uint8_t cmd11[8] = {0x03, 0xFC, 0x3C, 0x42, 0x00, 0x00, 0x00, 0x02};
	// pex89000_i2c_encode(oft, 0xF, BRCM_I2C5_CMD_WRITE, &cmd);

	uint8_t *cmds[] = {
		cmd1, cmd2, cmd3,
		cmd4, cmd5, cmd6,
		cmd7, cmd8, cmd9,
		cmd10, cmd11,
	};
	int num_cmds = sizeof(cmds) / sizeof(cmds[0]);

	for (int i = 0; i < num_cmds; i++) {
		msg.tx_len = 8;
		memcpy(msg.data, cmds[i], msg.tx_len);

		if (i2c_master_write(&msg, retry)) {
			LOG_ERR("Chime write failed at cmd%d", i + 1);
		}
	}
	uint8_t cmd12[4] = {0x07, 0xfc, 0x3c, 0x41};
	msg.tx_len = 4;
	msg.rx_len = 4;
	memcpy(msg.data, cmd12, msg.tx_len);
	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("ADS7830 read failed");
	}
	// uint8_t raw_adc = msg.data[0];
	LOG_INF("%x, %x, %x, %x", msg.data[0], msg.data[1], msg.data[2], msg.data[3]);


	// uint8_t retry = 5;
	// I2C_MSG msg;

	// msg.bus = 3;
	// msg.target_addr = 0x6a << 1;
	// // msg.tx_len = sizeof(cmd) + data_len;
	// msg.tx_len = sizeof(cmd);
	// memcpy(&msg.data[0], cmd, msg.tx_len);
	// // memcpy(&msg.data[0], &cmd, sizeof(cmd));
	// // memcpy(&msg.data[4], data, data_len);

	// if (i2c_master_write(&msg, retry)) {
	// 	LOG_ERR("Chime write failed");
	// }
	
}

SHELL_CMD_REGISTER(pcie_test, NULL, "echo command", get_pcie_tmp);
