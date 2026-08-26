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

#include <kernel.h>
#include <stdlib.h>
#include <logging/log.h>
#include <libutil.h>
#include "plat_sensor_table.h"
#include "fru.h"
#include "plat_fru.h"
#include "plat_i2c.h"
#include "plat_log.h"
#include "plat_cpld.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_class.h"
#include "plat_pldm_sensor.h"
#include "pldm_oem.h"

LOG_MODULE_REGISTER(plat_log);

#define LOG_MAX_INDEX 0x0FFF // err_log_data[].index range: 0 ~ (LOG_MAX_INDEX - 1), then wraps
#define LOG_MAX_NUM 50 // total log amount: 50
#define FRU_LOG_START 0x0000 // log offset: 0KB
#define EEPROM_MAX_WRITE_TIME 5 // the BR24G512 eeprom max write time is 3.5 ms
#define CPLD_VR_VENDOR_TYPE_REG 0x1C
#define ERROR_CODE_TYPE_SHIFT 13

static plat_err_log_mapping err_log_data[LOG_MAX_NUM];
static uint16_t err_code_caches[200];
// codes currently asserting (not yet deasserted); extend if concurrent error types > 200
static uint16_t next_log_position; // next RAM/EEPROM slot to write, 0-based
static uint16_t next_index; // next value for err_log_data[].index, 0-based, wraps at LOG_MAX_INDEX
static uint8_t log_num; // Number of logs in EEPROM

typedef struct _vr_ubc_device_table_ {
	uint8_t index;
	uint8_t sensor_num_1;
	uint8_t sensor_num_2;
} vr_ubc_device_table;

vr_ubc_device_table vr_device_table[] = {
	// index, sensor_num_1(page 0), sensor_num_2(page 1)
	{ VR_INDEX_E_1, SENSOR_NUM_ASIC_P0V75_NUWA1_VDD_TEMP_C },

	{ VR_INDEX_E_2, SENSOR_NUM_ASIC_P0V75_NUWA0_VDD_TEMP_C },

	{ VR_INDEX_E_3, SENSOR_NUM_ASIC_P0V9_OWL_E_TRVDD_TEMP_C,
	  SENSOR_NUM_ASIC_P0V75_OWL_E_TRVDD_TEMP_C },

	{ VR_INDEX_E_4, SENSOR_NUM_ASIC_P0V75_MAX_M_VDD_TEMP_C,
	  SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM1357_TEMP_C },

	{ VR_INDEX_E_5, SENSOR_NUM_ASIC_P0V75_OWL_E_VDD_TEMP_C,
	  SENSOR_NUM_ASIC_P0V4_VDDQL_HBM1357_TEMP_C },

	{ VR_INDEX_E_6, SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_TEMP_C,
	  SENSOR_NUM_ASIC_P1V8_VPP_HBM1357_TEMP_C },

	{ VR_INDEX_E_7, SENSOR_NUM_ASIC_P1V05_VDDQC_HBM1357_TEMP_C,
	  SENSOR_NUM_ASIC_P0V85_HAMSA_VDD_TEMP_C },

	{ VR_INDEX_E_8, SENSOR_NUM_ASIC_P0V9_VDDQ_HBM1357_TEMP_C,
	  SENSOR_NUM_ASIC_P0V8_HAMSA_AVDD_PCIE_TEMP_C },

	{ VR_INDEX_E_9, SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_TEMP_C,
	  SENSOR_NUM_ASIC_P1V2_HAMSA_VDDHRXTX_PCIE_TEMP_C },

	{ VR_INDEX_E_10, SENSOR_NUM_ASIC_P1V05_VDDQC_HBM0246_TEMP_C,
	  SENSOR_NUM_ASIC_P1V8_VPP_HBM0246_TEMP_C },

	{ VR_INDEX_E_11, SENSOR_NUM_ASIC_P0V4_VDDQL_HBM0246_TEMP_C,
	  SENSOR_NUM_ASIC_P0V75_VDDPHY_HBM0246_TEMP_C },

	{ VR_INDEX_E_12, SENSOR_NUM_ASIC_P0V75_OWL_W_VDD_TEMP_C,
	  SENSOR_NUM_ASIC_P0V75_MAX_S_VDD_TEMP_C },

	{ VR_INDEX_E_13, SENSOR_NUM_ASIC_P0V9_OWL_W_TRVDD_TEMP_C,
	  SENSOR_NUM_ASIC_P0V75_OWL_W_TRVDD_TEMP_C },
};

typedef struct _vr_error_callback_info_ {
	uint8_t cpld_offset;
	uint8_t vr_status_word_access_map;
	uint8_t bit_mapping_vr_sensor_num[8];
} vr_error_callback_info;

vr_error_callback_info vr_error_callback_info_table[] = {
	// cpld_offset, reading mask, bit_mapping_vr_sensor_num
	{ VR_POWER_FAULT_1_REG,
	  0xFC,
	  { VR_INDEX_E_3, VR_INDEX_E_13, VR_INDEX_E_3, VR_INDEX_E_13, VR_INDEX_E_8, VR_INDEX_E_9,
	    0x00, 0x00 } },
	{ VR_POWER_FAULT_2_REG,
	  0xFF,
	  { VR_INDEX_E_1, VR_INDEX_E_2, VR_INDEX_E_5, VR_INDEX_E_12, VR_INDEX_E_9, VR_INDEX_E_12,
	    VR_INDEX_E_4, VR_INDEX_E_8 } },
	{ VR_POWER_FAULT_3_REG,
	  0xFF,
	  { VR_INDEX_E_11, VR_INDEX_E_10, VR_INDEX_E_10, VR_INDEX_E_11, VR_INDEX_E_5, VR_INDEX_E_6,
	    VR_INDEX_E_6, VR_INDEX_E_4 } },
	{ VR_POWER_FAULT_4_REG,
	  0,
	  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } }, // to_do not sure
	{ VR_POWER_FAULT_5_REG,
	  0,
	  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } }, // to_do not sure
};

void plat_log_read(uint8_t *log_data, uint8_t cmd_size, uint16_t order)
{
	CHECK_NULL_ARG(log_data);

	// Calculate the target log position based on next_log_position (0-based)
	uint16_t zero_base_log_position = (next_log_position + LOG_MAX_NUM - order) % LOG_MAX_NUM;

	uint16_t eeprom_address =
		FRU_LOG_START + zero_base_log_position * sizeof(plat_err_log_mapping);

	LOG_DBG("order: %d, log_position: %d, eeprom_address: 0x%X", order,
		(zero_base_log_position + 1),
		eeprom_address); //remove after all log function is ready

	plat_err_log_mapping log_entry;

	if (!plat_eeprom_read(eeprom_address, (uint8_t *)&log_entry,
			      sizeof(plat_err_log_mapping))) {
		LOG_ERR("Failed to read log from EEPROM at position %d (address: 0x%X)", order,
			eeprom_address);
		memset(log_data, 0x00, cmd_size);
		return;
	}

	memcpy(log_data, &log_entry, cmd_size);

	const plat_err_log_mapping *p = (plat_err_log_mapping *)log_data;

	LOG_HEXDUMP_DBG(log_data, cmd_size, "plat_log_read_before");

	if (p->index == 0xFFFF) {
		memset(log_data, 0x00, cmd_size);
	}

	LOG_HEXDUMP_DBG(log_data, cmd_size, "plat_log_read_after");
}

// Clear logs from memory and EEPROM with error handling
void plat_clear_log()
{
	memset(err_log_data, 0xFF, sizeof(err_log_data));
	memset(err_code_caches, 0, sizeof(err_code_caches));

	for (uint8_t i = 0; i < LOG_MAX_NUM; i++) {
		if (!plat_eeprom_write(FRU_LOG_START + sizeof(plat_err_log_mapping) * i,
				       (uint8_t *)err_log_data, sizeof(plat_err_log_mapping))) {
			LOG_ERR("Clear EEPROM Log failed at index %d", i);
		}
		k_msleep(EEPROM_MAX_WRITE_TIME);
	}
	log_num = 0;
	next_index = 0;
}

bool vr_fault_get_error_data(uint8_t sensor_id, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	// vr status word
	return get_raw_data_from_sensor_id(sensor_id, 0x79, data, 2);
}

bool get_error_data(uint16_t error_code, uint8_t *data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	uint8_t trigger_case = (error_code >> ERROR_CODE_TYPE_SHIFT) & 0x07;

	switch (trigger_case) {
	case AC_ON_TRIGGER_CAUSE:
	case DC_ON_TRIGGER_CAUSE: {
		data[0] = gpio_get(RST_ASTRID_PWR_ON_PLD_R1_N);
		return true;
	}
	}

	// Extract CPLD offset and bit position from the error code
	uint8_t cpld_offset = error_code & 0xFF;
	uint8_t bit_position = (error_code >> 8) & 0x07;
	LOG_DBG("cpld_offset: 0x%x, bit_position: 0x%x", cpld_offset, bit_position);

	// Initialize sensor number
	uint8_t sensor_num = 0x00;
	uint8_t device_id = 0x00;

	// Find the device_id associated with the error code
	for (size_t i = 0; i < ARRAY_SIZE(vr_error_callback_info_table); i++) {
		if (vr_error_callback_info_table[i].cpld_offset == cpld_offset) {
			device_id = vr_error_callback_info_table[i]
					    .bit_mapping_vr_sensor_num[bit_position];
			break;
		}
	}

	if (device_id == 0x00) {
		LOG_DBG("No valid device_id for error_code: 0x%x", error_code);
		return false;
	}

	// Find the sensor number associated with the device_id
	for (size_t i = 0; i < ARRAY_SIZE(vr_device_table); i++) {
		if (vr_device_table[i].index == device_id) {
			sensor_num = vr_device_table[i].sensor_num_1;
			break;
		}
	}

	// If no valid sensor number is found, skip further data retrieval
	if (sensor_num == 0x00) {
		LOG_DBG("No valid sensor_num for error_code: 0x%x", error_code);
		return false;
	}

	// Handle VR_FAULT_ASSERT errors and retrieve VR-specific data
	if (!vr_fault_get_error_data(sensor_num, data)) {
		LOG_ERR("Failed to retrieve VR fault data for sensor_num: 0x%x", sensor_num);
		return false;
	}

	return true;
}

// Find error_code in the active-fault cache. Returns the slot index, or -1 if not present.
static int16_t find_active_fault(uint16_t error_code)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(err_code_caches); i++) {
		if (err_code_caches[i] == error_code) {
			return i;
		}
	}
	return -1;
}

// Update the active-fault cache and decide whether a new log entry is needed.
// DEASSERT only clears the cache, it never produces a new log entry.
static bool update_active_fault_cache(uint16_t error_code, bool log_status)
{
	int16_t idx = find_active_fault(error_code);

	if (log_status == LOG_DEASSERT) {
		if (idx >= 0) {
			err_code_caches[idx] = 0;
			LOG_INF("Fault cleared, error_code: 0x%x", error_code);
		}
		return false;
	}

	// LOG_ASSERT
	if (idx >= 0) {
		LOG_INF("Duplicate error_code: 0x%x, already asserting", error_code);
		return false;
	}

	int16_t free_idx = find_active_fault(0);
	if (free_idx < 0) {
		LOG_ERR("err_code_caches full, cannot track error_code: 0x%x", error_code);
		return false;
	}
	err_code_caches[free_idx] = error_code;
	return true;
}

// Fill in one log entry: index, error code, timestamp, VR data and CPLD dump.
static void fill_log_entry(plat_err_log_mapping *entry, uint16_t error_code)
{
	entry->index = next_index;
	next_index = (next_index + 1) % LOG_MAX_INDEX;

	entry->err_code = error_code;
	entry->sys_time = k_uptime_get();

	if (!get_error_data(error_code, entry->error_data)) {
		// Clear error data if no valid data is found
		memset(entry->error_data, 0, sizeof(entry->error_data));
	}

	if (!plat_read_cpld(CPLD_REGISTER_1ST_PART_START_OFFSET, entry->cpld_dump,
			    CPLD_REGISTER_1ST_PART_NUM)) {
		LOG_ERR("Failed to dump 1st part CPLD data");
	}
}

// Persist one log entry to EEPROM and advance the ring buffer position.
static void store_log_entry(const plat_err_log_mapping *entry)
{
	uint16_t write_address = FRU_LOG_START + next_log_position * sizeof(plat_err_log_mapping);

	if (!plat_eeprom_write(write_address, (uint8_t *)entry, sizeof(plat_err_log_mapping))) {
		LOG_ERR("Write Log failed with Error code: %02x", entry->err_code);
	} else {
		k_msleep(EEPROM_MAX_WRITE_TIME); // wait 5ms to write EEPROM
	}

	next_log_position = (next_log_position + 1) % LOG_MAX_NUM;
	if (log_num < LOG_MAX_NUM) {
		log_num++;
	}
}

// Handle error log events and record them if necessary
void error_log_event(uint16_t error_code, bool log_status)
{
	if (!update_active_fault_cache(error_code, log_status)) {
		/* dessert or duplicate, nothing to do */
		return;
	}

	plat_err_log_mapping *entry = &err_log_data[next_log_position];
	fill_log_entry(entry, error_code);

	store_log_entry(entry);
}

void reset_error_log_event(uint8_t err_type)
{
	// Remove and DEASSERT error logs starting with the err_type
	for (uint8_t i = 0; i < ARRAY_SIZE(err_code_caches); i++) {
		uint16_t error_code = err_code_caches[i];
		uint8_t code_type = error_code >> ERROR_CODE_TYPE_SHIFT;
		if (code_type == err_type) {
			LOG_DBG("DEASSERT");
			error_log_event(error_code, LOG_DEASSERT);
			err_code_caches[i] = 0;
		}
	}
}

uint8_t plat_log_get_num(void)
{
	return log_num;
}

void find_last_log_position()
{
	uint16_t max_index = 0; // Highest valid index found
	uint16_t last_position = 0; // Position (0-based) of the highest valid index
	bool all_empty = true; // Flag to detect if all entries are empty
	plat_err_log_mapping log_entry;

	for (uint16_t i = 0; i < LOG_MAX_NUM; i++) {
		uint16_t eeprom_address = FRU_LOG_START + i * sizeof(plat_err_log_mapping);

		if (!plat_eeprom_read(eeprom_address, (uint8_t *)&log_entry,
				      sizeof(plat_err_log_mapping))) {
			LOG_ERR("Failed to read log at position %d (address: 0x%X)", i,
				eeprom_address);
			continue;
		}

		// Check if the entry is valid
		if (log_entry.index != 0xFFFF && log_entry.index < LOG_MAX_INDEX) {
			all_empty = false; // At least one entry is valid
			log_num++;
			if (log_entry.index > max_index) {
				max_index = log_entry.index; // Update max index
				last_position = i; // Update last position, 0-based
			}
		}
	}

	// All entries are empty
	if (all_empty) {
		LOG_INF("All entries are empty. Initializing next_log_position and next_index to 0.");
		next_log_position = 0;
		next_index = 0;
		return;
	}

	next_log_position = (last_position + 1) % LOG_MAX_NUM;
	next_index = (max_index + 1) % LOG_MAX_INDEX;
	LOG_INF("Next log position: %d, next index: %d", next_log_position, next_index);
}

// Load logs from EEPROM into memory during initialization
void init_load_eeprom_log(void)
{
	memset(err_log_data, 0xFF, sizeof(err_log_data));
	uint16_t log_len = sizeof(plat_err_log_mapping);
	for (uint8_t i = 0; i < LOG_MAX_NUM; i++) {
		if (!plat_eeprom_read(FRU_LOG_START + i * log_len, (uint8_t *)&err_log_data[i],
				      log_len)) {
			LOG_ERR("READ Event %d failed from EEPROM", i + 1);
		}
	}

	// Determine the next log position
	find_last_log_position();
}