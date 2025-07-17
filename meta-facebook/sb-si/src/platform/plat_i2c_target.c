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

/*
  NAME: I2C TARGET INIT
  FILE: plat_i2c_target.c
  DESCRIPTION: Provide i2c target EN/CFG table "I2C_TARGET_EN_TABLE[]/I2C_TARGET_CFG_TABLE[]" for init target config.
  AUTHOR: MouchenHung
  DATE/VERSION: 2021.11.26 - v1.1
  Note: 
    (1) "plat_i2c_target.h" is included by "hal_i2c_target.h"
*/

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <logging/log.h>
#include "plat_i2c_target.h"
#include "plat_mctp.h"
#include <drivers/flash.h>

LOG_MODULE_REGISTER(plat_i2c_target);

#define MAX_SLOT 4

static const uint8_t eid_table[MAX_SLOT] = {
	10, // SLOT 0
	20, // SLOT 1
	30, // SLOT 2
	40  // SLOT 3
};

static bool command_reply_data_handle(void *arg);

void set_eid_handler();
void plat_set_eid_init(int slot_id);
K_WORK_DELAYABLE_DEFINE(set_eid_work, set_eid_handler);

/* I2C target init-enable table */
const bool I2C_TARGET_ENABLE_TABLE[MAX_TARGET_NUM] = {
	TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE, TARGET_DISABLE,
	TARGET_DISABLE, TARGET_ENABLE, TARGET_DISABLE, TARGET_DISABLE,
	TARGET_DISABLE, TARGET_DISABLE, TARGET_ENABLE, TARGET_DISABLE,
};

/* I2C target init-config table */
const struct _i2c_target_config I2C_TARGET_CONFIG_TABLE[MAX_TARGET_NUM] = {
	{ 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA }, { 0x40, 0xA }, { 0xFF, 0xA }, { 0x40, 0xA, command_reply_data_handle},
	{ 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA }, { 0xFF, 0xA }, { 0x40, 0xA , command_reply_data_handle}, { 0xFF, 0xA },
};

static bool command_reply_data_handle(void *arg)
{
	struct i2c_target_data *data = (struct i2c_target_data *)arg;
	LOG_INF("command_reply_data_handle");
	/*TODO: put board telemetry here*/

	/* Only check fisrt byte from received data */
	if (data->wr_buffer_idx >= 1) {
		if (data->wr_buffer_idx == 1) {
			uint8_t reg_offset = data->target_wr_msg.msg[0];
			size_t struct_size = 0;

			LOG_INF("data->wr_buffer_idx == 1");

			// Make sure the target buffer is not exceeded when reading
			if (struct_size > sizeof(data->target_rd_msg.msg)) {
				struct_size = sizeof(data->target_rd_msg.msg);
			}
			switch (reg_offset) {
			case SLOT_0_I2C_SET_EID_REG: {
				plat_set_eid_init(0);
			} break;
			case SLOT_1_I2C_SET_EID_REG: {
				plat_set_eid_init(1);
			} break;
			case SLOT_2_I2C_SET_EID_REG: {
				plat_set_eid_init(2);
			} break;
			case SLOT_3_I2C_SET_EID_REG: {
				plat_set_eid_init(3);
			} break;
			default:
				LOG_ERR("Unknown reg offset: 0x%02x", reg_offset);
				data->target_rd_msg.msg_length = 1;
				data->target_rd_msg.msg[0] = 0xFF;
				break;
			}
		} else {
			LOG_ERR("Received data length: 0x%02x", data->wr_buffer_idx);
			data->target_rd_msg.msg_length = 1;
			data->target_rd_msg.msg[0] = 0xFF;
		}
	}
	LOG_INF("return false");
	return false;
}

void set_eid_handler(struct k_work *work)
{
	struct mmc_info *info = CONTAINER_OF(work, struct mmc_info, set_eid_work);
	uint8_t slot = info->slot;

	if (slot >= MAX_SLOT) {
		LOG_ERR("Invalid slot_id: %d", slot);
		free(info);
		return;
	}

	uint8_t eid = eid_table[slot];
	LOG_INF("Setting EID %d for slot %d", eid, slot);

	plat_set_eid(eid);
	LOG_INF("EID:%d", plat_get_eid());

	const struct device *flash_dev;
	uint8_t write_buf = eid;
	uint8_t read_back_buf = 255;
    uint32_t op_addr = 0x0FF000;
	uint32_t erase_sz = 0x1000;
	uint32_t ret = 0;

    flash_dev = device_get_binding("spi_spim0_cs0");
    if (flash_dev == NULL) {
        LOG_INF("Failed to get device.\n");
    }

	ret = flash_erase(flash_dev, op_addr, erase_sz);
    if (ret != 0) {
        LOG_INF("Failed to erase %u.\n", op_addr);
    }

	ret = flash_write(flash_dev, op_addr, &write_buf, 1);
    if (ret != 0) {
        LOG_INF("Failed to write %u.\n", op_addr);
    }

	ret = flash_read(flash_dev, op_addr, &read_back_buf, 1);
    if (ret != 0) {
        LOG_ERR("Failed to read %u.\n", op_addr);
    }

    LOG_INF("EID:%d get from flash", read_back_buf);

	if (memcmp(&write_buf, &read_back_buf, 1) != 0) {
        LOG_ERR("Failed to write flash at 0x%x.", op_addr);
        LOG_ERR("to be written:%d",write_buf);
        LOG_ERR("readback:%d",read_back_buf);
    }


	free(info);
}


void plat_set_eid_init(int slot_id)
{
	struct mmc_info *info = malloc(sizeof(struct mmc_info));
	if (!info) {
		LOG_ERR("Failed to allocate memory for mmc_info");
		return;
	}
	info->slot = slot_id;
	k_work_init(&info->set_eid_work, set_eid_handler);
	LOG_DBG("plat_set_eid_init started");
	k_work_submit(&info->set_eid_work);
}
