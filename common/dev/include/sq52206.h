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

#ifndef __SQ52206__
#define __SQ52206__

#include <stdint.h>
#include "sensor.h"

typedef struct _sq52206_init_arg {
	bool is_init;
	// user defined
	double r_shunt; /* Shunt resistor value. Unit: Ohm. */
	uint8_t adc_range; /* IN+ and IN-, 0:±163.84 mV, 1:±81.92 mV, 2:±40.96 mV */
	uint8_t alert_latch; /*alert_latch, 0:Disable, 1:Enable */
	double i_max; /* Expected maximum current */
	// calculated data don't set
	double cur_lsb;
} sq52206_init_arg;

/* SQ52206 register map is a superset of INA238's for the offsets both parts
 * implement (CONFIG/ADC_CONFIG/SHUNT_CAL/VSHUNT/VBUS/DIETEMP/CURRENT/POWER/
 * DIAG_ALRT/thresholds); it additionally has SHUNT_TEMPCO/ENERGY/CHARGE/
 * PWR_PEAK and has no MANUFACTURER_ID/DEVICE_ID register.
 */
enum SQ52206_OFFSET {
	SQ52206_CFG_OFFSET = 0x00,
	SQ52206_ADC_CFG_OFFSET = 0x01,
	SQ52206_SHUNT_CAL_OFFSET = 0x02,
	SQ52206_SHUNT_TEMPCO_OFFSET = 0x03,
	SQ52206_VSHUNT_OFFSET = 0x04,
	SQ52206_VBUS_OFFSET = 0x05,
	SQ52206_DIETEMP_OFFSET = 0x06,
	SQ52206_CUR_OFFSET = 0x07,
	SQ52206_PWR_OFFSET = 0x08,
	SQ52206_ENERGY_OFFSET = 0x09,
	SQ52206_CHARGE_OFFSET = 0x0A,
	SQ52206_DIAG_ALRT_OFFSET = 0x0B,
	SQ52206_SOVL_OFFSET = 0x0C,
	SQ52206_SUVL_OFFSET = 0x0D,
	SQ52206_BOVL_OFFSET = 0x0E,
	SQ52206_BUVL_OFFSET = 0x0F,
	SQ52206_TEMP_LIMIT_OFFSET = 0x10,
	SQ52206_PWR_LIMIT_OFFSET = 0x11,
	SQ52206_PWR_PEAK_OFFSET = 0x20,
};

enum SQ52206_ADC_RANGE {
	/* IN+ and IN-, CONFIG[4:3] */
	SQ52206_ADC_RANGE_PN_163 = 0x00, // ±163.84 mV, 5uV/LSB
	SQ52206_ADC_RANGE_PN_81 = 0x01, // ±81.92 mV, 2.5uV/LSB
	SQ52206_ADC_RANGE_PN_40 = 0x02, // ±40.96 mV, 1.25uV/LSB
};

enum SQ52206_ALERT_LATCH {
	SQ52206_ALERT_LATCH_DISABLE,
	SQ52206_ALERT_LATCH_ENABLE,
};
#endif
