/* Copyright (c) Meta Platforms, Inc. and affiliates. */
/* SPDX-License-Identifier: Apache-2.0 */

#include <logging/log.h>
#include <string.h>
#include "arke_smbus.h"
#include "hal_i2c.h"
#include "libutil.h"
#include "sensor.h"

LOG_MODULE_REGISTER(arke_smbus);

static bool arke_smbus_i2c_read(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG msg = { .bus = bus, .target_addr = addr, .tx_len = 1, .rx_len = len };
	msg.data[0] = reg;
	if (i2c_master_read(&msg, 5)) {
		LOG_ERR("Failed to read ARKE SMBus, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr,
			reg);
		return false;
	}
	memcpy(data, msg.data, len);
	return true;
}

static uint8_t arke_smbus_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	arke_smbus_priv_data_t *priv = (arke_smbus_priv_data_t *)cfg->priv_data;
	if (!priv)
		return SENSOR_UNSPECIFIED_ERROR;

	if (cfg->offset == ASIC_MONITOR_HBM_TEMP_REG) {
		if (!arke_smbus_i2c_read(cfg->port, cfg->target_addr, cfg->offset, priv->hbm_temp,
					 sizeof(priv->hbm_temp)))
			return SENSOR_UNSPECIFIED_ERROR;
		*reading = priv->hbm_temp[0];
	} else if (cfg->offset == ASIC_MONITOR_TEMP_REG) {
		if (!arke_smbus_i2c_read(cfg->port, cfg->target_addr, cfg->offset, priv->temp,
					 sizeof(priv->temp)))
			return SENSOR_UNSPECIFIED_ERROR;
		*reading = priv->temp[0];
	} else {
		return SENSOR_UNSPECIFIED_ERROR;
	}
	return SENSOR_READ_SUCCESS;
}

uint8_t arke_smbus_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	arke_smbus_priv_data_t *priv = k_malloc(sizeof(*priv));
	if (!priv)
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	memset(priv, 0, sizeof(*priv));
	cfg->priv_data = priv;
	cfg->read = arke_smbus_read;
	return SENSOR_INIT_SUCCESS;
}
