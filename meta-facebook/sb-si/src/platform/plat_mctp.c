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
#include "plat_hook.h"
#include "plat_mctp.h"
#include "plat_gpio.h"

LOG_MODULE_REGISTER(plat_mctp);

/* i2c 8 bit address */
#define I2C_ADDR_BIC 0x40
#define I2C_ADDR_BMC 0x20

/* i2c dev bus */
#define I2C_BUS_BMC I2C_BUS4

/* mctp endpoint */
#define MCTP_EID_BMC 0x08

uint8_t plat_eid = MCTP_DEFAULT_ENDPOINT;

static mctp_port smbus_port[] = {
	{ .conf.smbus_conf.addr = I2C_ADDR_BIC, .conf.smbus_conf.bus = I2C_BUS_BMC },
};

static mctp_port i3c_port[] = {
    { 
        .conf.i3c_conf.addr = 0x21,
        .conf.i3c_conf.bus = 4,
    }
};

mctp_route_entry mctp_route_tbl[] = {
	{ MCTP_EID_BMC, I2C_BUS_BMC, I2C_ADDR_BMC },
};

uint8_t MCTP_SUPPORTED_MESSAGES_TYPES[] = {
	TYPE_MCTP_CONTROL,
	TYPE_PLDM,
};

static mctp *find_mctp_by_smbus(uint8_t bus)
{
	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(smbus_port); i++) {
		mctp_port *p = smbus_port + i;

		if (bus == p->conf.smbus_conf.bus)
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
			*mctp_inst = find_mctp_by_smbus(p->bus);
			ext_params->type = MCTP_MEDIUM_TYPE_SMBUS;
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
	// uint8_t msg_type = (buf[0] & MCTP_MSG_TYPE_MASK) >> MCTP_MSG_TYPE_SHIFT;
	// uint8_t ic = (buf[0] & MCTP_IC_MASK) >> MCTP_IC_SHIFT;
	// (void)ic;

	for(int i=0;i<len; i++ ){
		printk("here %u\n",buf[i]);	
	}	

	k_msleep(10000);
	mctp *mctp_inst = (mctp *)mctp_p;

	mctp_tx_msg mctp_msg = { 0 };
	mctp_msg.is_bridge_packet = 0;
	mctp_msg.len = len;
	mctp_msg.buf = (uint8_t *)malloc(len);
	if (!mctp_msg.buf)
		goto error;
	memcpy(mctp_msg.buf, buf, len);

	mctp_msg.ext_params = ext_params;
	printk("to: %u\nmsg tag: %u\nep: %u\n", ext_params.tag_owner, ext_params.msg_tag, ext_params.ep);

	/* create msg queue for catching the return code from mctp_tx_task */
	uint8_t evt_msgq_buf = 0;
	struct k_msgq evt_msgq;

	k_msgq_init(&evt_msgq, &evt_msgq_buf, sizeof(uint8_t), 1);
	mctp_msg.evt_msgq = &evt_msgq;

	int ret = k_msgq_put(&mctp_inst->mctp_tx_queue, &mctp_msg, K_NO_WAIT);
	if (!ret) {
		uint8_t evt = MCTP_ERROR;
		if (k_msgq_get(&evt_msgq, &evt, K_FOREVER)) {
			LOG_WRN("failed to get status from msgq!");
			goto error;
		}

		printk("evt from msg recv : evt");
	}

error:
	if (mctp_msg.buf)
		free(mctp_msg.buf);

	return MCTP_ERROR;

	// switch (msg_type) {
	// case MCTP_MSG_TYPE_CTRL:
	// 	mctp_ctrl_cmd_handler(mctp_p, buf, len, ext_params);
	// 	break;

	// case MCTP_MSG_TYPE_PLDM:
	// 	mctp_pldm_cmd_handler(mctp_p, buf, len, ext_params);
	// 	break;

	// default:
	// 	LOG_WRN("Cannot find message receive function!!");
	// 	return MCTP_ERROR;
	// }

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
			*mctp_inst = find_mctp_by_smbus(p->bus);
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

		uint8_t rc;
		if (p->conf.i3c_conf.bus == 5) {
			// Master (i3c5)
			// rc = mctp_set_medium_configure(p->mctp_inst, MCTP_MEDIUM_TYPE_CONTROLLER_I3C, p->conf);
			// LOG_DBG("MCTP medium type for master: %s", (rc == MCTP_SUCCESS) ? "success" : "failed");
		} else if (p->conf.i3c_conf.bus == 4) {
			// Slave (i3c4)
			rc = mctp_set_medium_configure(p->mctp_inst, MCTP_MEDIUM_TYPE_TARGET_I3C, p->conf);
			LOG_DBG("MCTP medium type for slave: %s", (rc == MCTP_SUCCESS) ? "success" : "failed");
		} else {
			LOG_ERR("Unknown bus number!");
			continue;
		}

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

uint8_t plat_get_eid()
{
	return plat_eid;
}