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
#include "libutil.h"
#include <logging/log.h>
#include "plat_hook.h"
#include "plat_gpio.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_user_setting);

bool post_vr_read(sensor_cfg *cfg, void *args, int *const reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);
	for (int i = 0; i < VR_RAIL_E_MAX; i++) {
		if (reading == NULL) {
			break;
		}

		if (((get_asic_board_id() != ASIC_BOARD_ID_EVB)) &&
		    (i == VR_RAIL_E_P3V3_OSFP_VOLT_V))
			continue; // skip osfp p3v3 on AEGIS BD

		if (cfg->num == vr_rail_table[i].sensor_id) {
			if (vr_rail_table[i].peak_value == 0xffffffff) {
				vr_rail_table[i].peak_value = *reading;
			} else {
				if (vr_rail_table[i].peak_value < *reading) {
					vr_rail_table[i].peak_value = *reading;
				}
			}
			break;
		}
	}

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;

	/* mutex unlock */
	if (pre_proc_args->mutex) {
		LOG_DBG("%x u %p", cfg->num, pre_proc_args->mutex);
		if (k_mutex_unlock(pre_proc_args->mutex)) {
			LOG_ERR("0x%02x post_vr_read, mutex unlock fail", cfg->num);
			return false;
		}
	}

	/* set reading val to 0 if reading val is negative */
	sensor_val tmp_reading;
	tmp_reading.integer = (int16_t)(*reading & 0xFFFF);
	tmp_reading.fraction = (int16_t)((*reading >> 16) & 0xFFFF);

	/* sensor_value = 1000 times of true value */
	int32_t sensor_value = tmp_reading.integer * 1000 + tmp_reading.fraction;

	if (sensor_value < 0) {
		LOG_DBG("Original sensor reading: integer = %d, fraction = %d (combined value * 1000: %d)",
			tmp_reading.integer, tmp_reading.fraction, sensor_value);
		*reading = 0;
		LOG_DBG("Negative sensor reading detected. Set reading to 0x%x", *reading);
	}
	post_sensor_reading_hook_func(cfg->num);

	// if (reading != NULL) {
	// 	float resolution = 0, offset = 0;
	// 	int cache_reading = 0;
	// 	int8_t unit_modifier = 0;
	// 	uint8_t sensor_operational_state = PLDM_SENSOR_STATUSUNKOWN;
	// 	pldm_sensor_get_info_via_sensor_id(cfg->num, &resolution, &offset, &unit_modifier,
	// 					   &cache_reading, &sensor_operational_state);
	// 	if (resolution == 0)
	// 		LOG_ERR("resolution is 0");

	// 	int16_t integer = *reading & 0xFFFF;
	// 	float fraction = (float)(*reading >> 16) / 1000.0;

	// 	if (integer < 0 && fraction > 0)
	// 		fraction = -fraction;

	// 	float tmp_reading_value = (float)integer + fraction;

	// 	if (tmp_reading_value < 0) {
	// 		tmp_reading_value = 0;
	// 		*reading = 0;
	// 		LOG_DBG("Original sensor reading: integer = %d, fraction = %f", integer,
	// 			fraction);
	// 		LOG_DBG("Negative sensor reading detected. Set reading to 0x%x", *reading);
	// 	}

	// 	int decoded_reading =
	// 		(int)((tmp_reading_value * power(10, -1 * unit_modifier) - offset) /
	// 		      resolution);

	// 	/* record power history */
	// 	for (int i = 0; i < UBC_VR_RAIL_E_MAX; i++) {
	// 		if ((get_asic_board_id() != ASIC_BOARD_ID_EVB) &&
	// 		    (i == UBC_VR_RAIL_E_P3V3_OSFP))
	// 			continue; // skip osfp p3v3
	// 		if (cfg->num == ubc_vr_power_table[i].sensor_id) {
	// 			ubc_vr_power_table[i].power_history[power_index[i]] =
	// 				decoded_reading;
	// 			power_index[i] = (power_index[i] + 1) % POWER_HISTORY_SIZE;
	// 			if (power_count[i] < POWER_HISTORY_SIZE) {
	// 				power_count[i]++;
	// 			}
	// 		}
	// 	}

	// 	/* TO_DO wait power capping add
	// 	if (cfg->num == VR_ASIC_P0V85_PVDD_PWR_W) {
	// 		update_plat_power_capping_table();
	// 		ath_vdd_power = (int)tmp_reading;
	// 		ath_vdd_polling_counter++;
	// 		// LOG_INF("counter:%d/%d", ath_vdd_polling_counter, comparator_counter_max);
	// 		if (ath_vdd_polling_counter >= comparator_counter_max) {
	// 			ath_vdd_polling_counter = 0;
	// 			power_capping_comparator_handler();
	// 		}
	// 	}
	// 	*/
	// }

	return true;
}

void user_settings_init(void)
{
	vr_vout_range_user_settings_init();
	bootstrap_default_settings_init();
	bootstrap_user_settings_init();
}