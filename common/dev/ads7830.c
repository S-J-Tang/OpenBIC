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

#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "ads7830.h"

LOG_MODULE_REGISTER(dev_ads7830);

static const uint8_t ads7830_cmd_table[8] = {
	ADC_CH0, ADC_CH1, ADC_CH2, ADC_CH3,
	ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7
};

uint8_t ads7830_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX || cfg->offset > 7) {
		return SENSOR_UNSPECIFIED_ERROR;
	}

	I2C_MSG msg;
	uint8_t retry = 3;

	// Step 1: Write Command Byte to select channel
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 0;
	msg.data[0] = ads7830_cmd_table[cfg->offset];

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("ADS7830 write failed");
		return SENSOR_FAIL_TO_ACCESS;
	}

	k_msleep(2); // Wait for ADC conversion

	// Step 2: Read 1 byte ADC result
	msg.tx_len = 0;
	msg.rx_len = 1;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("ADS7830 read failed");
		return SENSOR_FAIL_TO_ACCESS;
	}

	uint8_t raw_adc = msg.data[0];
	*reading = raw_adc;

	// Fetch reference voltage and resistors from init_args
	ads7830_init_args *init_args = (ads7830_init_args *)cfg->init_args;
	float reference_voltage = init_args->reference_voltage;
	float resistor1 = init_args->resistor1;
	float resistor2 = init_args->resistor2;

	// Calculate the voltage based on ADC value, resistors, and reference voltage
	float voltage = ((raw_adc / 255.0f) * reference_voltage) * (resistor2 / (resistor1 + resistor2));

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)voltage;
	sval->fraction = (voltage - sval->integer) * 1000;

	return SENSOR_READ_SUCCESS;
}

uint8_t ads7830_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = ads7830_read;
	return SENSOR_INIT_SUCCESS;
}
