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
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <stdlib.h>
#include "mctp.h"
#include "mctp_ctrl.h"
#include "pldm.h"
#include "ipmi.h"
#include "sensor.h"
#include "plat_i2c.h"
#include "plat_i3c.h"
#include "plat_hook.h"
#include "plat_mctp.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "plat_i2c_target.h"
#include <drivers/flash.h>

LOG_MODULE_REGISTER(plat_mctp);

/* i2c 8 bit address */
#define I2C_ADDR_BIC 0x40
#define I2C_ADDR_BMC 0x20

/* i2c dev bus */
#define I2C_BUS_BMC I3C_BUS6

/* mctp endpoint */
#define MCTP_EID_BMC 0x08

void plat_init_set_eid();

uint8_t plat_eid = MCTP_DEFAULT_ENDPOINT;

static mctp_port i3c_port[] = { {
	.conf.i3c_conf.addr = 0x21,
	.conf.i3c_conf.bus = I3C_BUS6,
} };

mctp_route_entry mctp_route_tbl[] = {
	{ MCTP_EID_BMC, I2C_BUS_BMC, I2C_ADDR_BMC },
};

uint8_t MCTP_SUPPORTED_MESSAGES_TYPES[] = {
	TYPE_MCTP_CONTROL,
	TYPE_PLDM,
};

static mctp *find_mctp_by_i3cbus(uint8_t bus)
{
	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(i3c_port); i++) {
		mctp_port *p = i3c_port + i;

		if (bus == p->conf.i3c_conf.bus)
			return p->mctp_inst;
	}

	return NULL;
}

uint8_t get_mctp_info(uint8_t dest_endpoint, mctp **mctp_inst, mctp_ext_params *ext_params)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(ext_params, MCTP_ERROR);

	uint8_t rc = MCTP_ERROR;
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(mctp_route_tbl); i++) {
		mctp_route_entry *p = mctp_route_tbl + i;
		if (!p) {
			return MCTP_ERROR;
		}
		if (p->endpoint == dest_endpoint) {
			*mctp_inst = find_mctp_by_i3cbus(p->bus);
			ext_params->type = MCTP_MEDIUM_TYPE_TARGET_I3C;
			ext_params->smbus_ext_params.addr = p->addr;
			rc = MCTP_SUCCESS;
			break;
		}
	}
	return rc;
}

static uint8_t mctp_msg_recv(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params ext_params)
{
	if (!mctp_p || !buf || !len)
		return MCTP_ERROR;

	/* first byte is message type and ic */
	uint8_t msg_type = (buf[0] & MCTP_MSG_TYPE_MASK) >> MCTP_MSG_TYPE_SHIFT;
	uint8_t ic = (buf[0] & MCTP_IC_MASK) >> MCTP_IC_SHIFT;
	(void)ic;

	switch (msg_type) {
	case MCTP_MSG_TYPE_CTRL:
		mctp_ctrl_cmd_handler(mctp_p, buf, len, ext_params);
		break;

	case MCTP_MSG_TYPE_PLDM:
		mctp_pldm_cmd_handler(mctp_p, buf, len, ext_params);
		break;

	default:
		LOG_WRN("Cannot find message receive function!!");
		return MCTP_ERROR;
	}

	return MCTP_SUCCESS;
}

static uint8_t get_mctp_route_info(uint8_t dest_endpoint, void **mctp_inst,
				   mctp_ext_params *ext_params)
{
	if (!mctp_inst || !ext_params)
		return MCTP_ERROR;

	uint8_t rc = MCTP_ERROR;
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(mctp_route_tbl); i++) {
		mctp_route_entry *p = mctp_route_tbl + i;
		if (p->endpoint == dest_endpoint) {
			*mctp_inst = find_mctp_by_i3cbus(p->bus);
			ext_params->type = MCTP_MEDIUM_TYPE_SMBUS;
			ext_params->smbus_ext_params.addr = p->addr;
			rc = MCTP_SUCCESS;
			break;
		}
	}

	return rc;
}

void plat_mctp_init(void)
{
	LOG_INF("plat_mctp_init");

	plat_init_set_eid();

	/* init the mctp/pldm instance */
	for (uint8_t i = 0; i < ARRAY_SIZE(i3c_port); i++) {
		mctp_port *p = i3c_port + i;
		LOG_DBG("i3c port %d", i);
		LOG_DBG("bus = %x, addr = %x", p->conf.i3c_conf.bus, p->conf.i3c_conf.addr);

		p->mctp_inst = mctp_init();
		if (!p->mctp_inst) {
			LOG_ERR("mctp_init failed!!");
			continue;
		}

		LOG_DBG("mctp_inst = %p", p->mctp_inst);
		uint8_t rc = mctp_set_medium_configure(p->mctp_inst, MCTP_MEDIUM_TYPE_TARGET_I3C,
						       p->conf);
		LOG_DBG("mctp_set_medium_configure %s",
			(rc == MCTP_SUCCESS) ? "success" : "failed");

		mctp_reg_endpoint_resolve_func(p->mctp_inst, get_mctp_route_info);

		mctp_reg_msg_rx_func(p->mctp_inst, mctp_msg_recv);

		mctp_start(p->mctp_inst);
	}
}

int load_mctp_support_types(uint8_t *type_len, uint8_t *types)
{
	*type_len = sizeof(MCTP_SUPPORTED_MESSAGES_TYPES);
	memcpy(types, MCTP_SUPPORTED_MESSAGES_TYPES, sizeof(MCTP_SUPPORTED_MESSAGES_TYPES));
	return MCTP_SUCCESS;
}

uint8_t plat_get_mctp_port_count()
{
	return ARRAY_SIZE(i3c_port);
}

mctp_port *plat_get_mctp_port(uint8_t index)
{
	return i3c_port + index;
}

void plat_init_set_eid()
{
	uint8_t slot_id = get_slot_id();

	if (slot_id >= MAX_SLOT) {
		LOG_ERR("Invalid slot ID (%d), fallback to slot 0 EID", slot_id);
		slot_id = 0;
	}

	const mmc_info_t *cfg = &mmc_info_table[slot_id];
	plat_set_eid(cfg->eid);

	LOG_INF("Set EID to 0x%02x for slot %d", cfg->eid, slot_id);
}

uint8_t plat_get_eid()
{
	return plat_eid;
}

void plat_set_eid(int slot_eid)
{
	plat_eid = slot_eid;
	return;
}