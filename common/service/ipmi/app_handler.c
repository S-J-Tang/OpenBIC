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

#include "app_handler.h"

#include "fru.h"
#include "sdr.h"
#include "guid.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include "plat_def.h"
#include "util_sys.h"
#include <logging/log.h>
#include <libutil.h>

#ifdef ENABLE_EVENT_TO_BMC
#include "pldm_oem.h"
#endif

LOG_MODULE_DECLARE(ipmi);
#define FRB2_WDT_STACK 1024
K_KERNEL_STACK_MEMBER(frb2_wdt_stack, FRB2_WDT_STACK);
static struct k_thread frb2_wdt_thread;
k_tid_t frb2_wdt_tid;

/* variables for Set/Get Watchdog Timer command handler*/
static uint8_t timerUse = 0;
static uint8_t timerAction = 0;
static uint8_t preTimeoutInSec = 0;
static uint8_t timerUseExpireFlagsClear = 0;
static uint8_t timerUseExpireFlags = 0;
static uint8_t initCountdownValueLsb = 0;
static uint8_t initCountdownValueMsb = 0;
static uint16_t presentCountdownValue = 0;
static uint16_t setCountdownValue = 0;
static bool isWdtSet = false;

__weak void APP_GET_DEVICE_ID(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data[0] = DEVICE_ID;
	msg->data[1] = DEVICE_REVISION;
	msg->data[2] = FIRMWARE_REVISION_1;
	msg->data[3] = FIRMWARE_REVISION_2;
	msg->data[4] = IPMI_VERSION;
	msg->data[5] = ADDITIONAL_DEVICE_SUPPORT;
	msg->data[6] = (IANA_ID & 0xFF);
	msg->data[7] = (IANA_ID >> 8) & 0xFF;
	msg->data[8] = (IANA_ID >> 16) & 0xFF;
	msg->data[9] = (PRODUCT_ID & 0xFF);
	msg->data[10] = (PRODUCT_ID >> 8) & 0xFF;
	msg->data[11] = (AUXILIARY_FW_REVISION >> 24) & 0xFF;
	msg->data[12] = (AUXILIARY_FW_REVISION >> 16) & 0xFF;
	msg->data[13] = (AUXILIARY_FW_REVISION >> 8) & 0xFF;
	msg->data[14] = (AUXILIARY_FW_REVISION & 0xFF);
	msg->data_len = 15;
	msg->completion_code = CC_SUCCESS;

	return;
}

__weak void APP_COLD_RESET(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	submit_bic_cold_reset();

	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void APP_WARM_RESET(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	submit_bic_warm_reset();

	msg->completion_code = CC_SUCCESS;
	return;
}

__weak void APP_GET_SELFTEST_RESULTS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	EEPROM_ENTRY fru_entry;
	fru_entry.config.dev_id = BIC_FRU_DEV_ID;
	fru_entry.offset = 0;
	fru_entry.data_len = 8;
	FRU_read(&fru_entry);
	uint8_t checksum = 0;
	for (uint8_t i = 0; i < fru_entry.data_len; i++) {
		checksum += fru_entry.data[i];
	}

	SELF_TEST_RESULT res;

	res.result.opFwCorrupt = 0;
	res.result.updateFwCorrupt = 0;
	res.result.sdrRepoEmpty = is_sdr_not_init;
	res.result.ipmbLinesDead = 0;

	if (checksum == 0) {
		res.result.cannotAccessBmcFruDev = 0;
		res.result.internalCorrupt = 0;
	} else {
		res.result.cannotAccessBmcFruDev = 1;
		res.result.internalCorrupt = 1;
	}

	res.result.cannotAccessSdrRepo = is_sdr_not_init;
	res.result.cannotAccessSelDev = 0;

	memcpy(&msg->data[1], &res.result, 1);
	// 55h = No error, 57h = Corrupted or inaccessible data or devices
	msg->data[0] = (msg->data[1] == 0x00) ? 0x55 : 0x57;
	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;

	return;
}

__weak void APP_GET_SYSTEM_GUID(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	uint8_t status = get_system_guid(&msg->data_len, &msg->data[0]);
	switch (status) {
	case GUID_READ_SUCCESS:
		msg->completion_code = CC_SUCCESS;
		break;
	case GUID_INVALID_ID:
		msg->completion_code = CC_INVALID_PARAM;
		break;
	case GUID_OUT_OF_RANGE:
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		break;
	case GUID_FAIL_TO_ACCESS:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	default:
		msg->completion_code = CC_UNSPECIFIED_ERROR;
		break;
	}

	return;
}

__weak void APP_MASTER_WRITE_READ(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	uint8_t retry = 3;
	uint8_t bus_7bit;
	I2C_MSG i2c_msg;

	// at least include bus, addr, rx_len, offset
	if (msg->data_len < 4) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	// should ignore bit0, all bus public
	bus_7bit = msg->data[0] >> 1;
	if (bus_7bit >= I2C_BUS_MAX_NUM) {
		LOG_ERR("Accessing invalid bus with IPMI master write read");
		msg->completion_code = CC_PARAM_OUT_OF_RANGE;
		return;
	}

	i2c_msg.bus = bus_7bit;
	// 8 bit address to 7 bit
	i2c_msg.target_addr = msg->data[1] >> 1;
	i2c_msg.rx_len = msg->data[2];
	i2c_msg.tx_len = msg->data_len - 3;

	if (i2c_msg.tx_len == 0) {
		msg->completion_code = CC_INVALID_DATA_FIELD;
		return;
	}

	memcpy(&i2c_msg.data[0], &msg->data[3], i2c_msg.tx_len);
	msg->data_len = i2c_msg.rx_len;

	if (i2c_msg.rx_len == 0) {
		if (!i2c_master_write(&i2c_msg, retry)) {
			msg->completion_code = CC_SUCCESS;
		} else {
			msg->completion_code = CC_I2C_BUS_ERROR;
		}
	} else {
		if (!i2c_master_read(&i2c_msg, retry)) {
			memcpy(&msg->data[0], &i2c_msg.data, i2c_msg.rx_len);
			msg->completion_code = CC_SUCCESS;
		} else {
			msg->completion_code = CC_I2C_BUS_ERROR;
		}
	}

	return;
}

__weak void APP_GET_BMC_GLOBAL_ENABLES(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void APP_SET_ACPI_POWER(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void APP_GET_DEVICE_GUID(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void APP_CLEAR_MESSAGE_FLAGS(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void APP_GET_CAHNNEL_INFO(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	msg->data_len = 0;
	msg->completion_code = CC_INVALID_CMD;
	return;
}

__weak void frb2_wdt_timer_action(uint8_t action, uint8_t timer)
{
	LOG_INF("frb2 timer action not implement");
}

void frb2_wdt_handler(void *countdownValue, void *arug1, void *arug2)
{
	uint8_t timer = (timerUse & 0x07);
	presentCountdownValue = POINTER_TO_UINT(countdownValue);
	while (presentCountdownValue > 0) {
		k_sleep(K_MSEC(FRB2_WDT_DELAY_MS));
		if (presentCountdownValue < 100) {
			presentCountdownValue = 0;
			break;
		}
		presentCountdownValue = (presentCountdownValue - 100);
	}

	timerUseExpireFlags |= (uint8_t)(1 << (timerUse & 0x07));
	frb2_wdt_timer_action(timerAction, timer);
}

void init_frb2_wdt_thread(int16_t initCountdownValue)
{
	// Avoid re-create thread by checking thread status and thread id
	if (frb2_wdt_tid != NULL && strcmp(k_thread_state_str(frb2_wdt_tid), "dead") != 0) {
		// Restart from the initial countdown value
		presentCountdownValue = setCountdownValue;
		return;
	}

	frb2_wdt_tid = k_thread_create(&frb2_wdt_thread, frb2_wdt_stack,
				       K_THREAD_STACK_SIZEOF(frb2_wdt_stack), frb2_wdt_handler,
				       INT_TO_POINTER((int16_t)initCountdownValue), NULL, NULL,
				       CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&frb2_wdt_thread, "frb2_wdt_thread");

	LOG_INF("frb2 wdt start!");
}

void abort_frb2_wdt_thread()
{
	if (frb2_wdt_tid != NULL && strcmp(k_thread_state_str(frb2_wdt_tid), "dead") != 0) {
		k_thread_abort(frb2_wdt_tid);
		frb2_wdt_tid = NULL;
		setCountdownValue = 0;
		presentCountdownValue = 0;
		isWdtSet = false;

		LOG_INF("frb2 wdt stop!");
	}
}

__weak void APP_RESET_WATCHDOG_TIMER(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (isWdtSet) {
		init_frb2_wdt_thread(setCountdownValue);

		msg->data_len = 0;
		msg->completion_code = CC_SUCCESS;
	} else {
		LOG_INF("set frb2 wdt is not issued!");
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_PARAM;
	}

	return;
}

__weak void APP_SET_WATCHDOG_TIMER(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	/* Parsing Set Watchdog Timer Request data field */
	timerUse = msg->data[0];
	timerAction = msg->data[1];
	preTimeoutInSec = msg->data[2];
	timerUseExpireFlagsClear = msg->data[3];
	initCountdownValueLsb = msg->data[4];
	initCountdownValueMsb = msg->data[5];

	// Compared with Timer Use Expiration flags clear
	timerUseExpireFlags &= ~(timerUseExpireFlagsClear);

	setCountdownValue =
		(uint16_t)(initCountdownValueMsb) << 8 | (uint16_t)(initCountdownValueLsb);

	if (frb2_wdt_tid != NULL && strcmp(k_thread_state_str(frb2_wdt_tid), "dead") != 0) {
		abort_frb2_wdt_thread();
		isWdtSet = false;
	} else {
		presentCountdownValue = setCountdownValue;
		LOG_INF("frb2 wdt set!");
		isWdtSet = true;
	}

	msg->data_len = 0;
	msg->completion_code = CC_SUCCESS;

	return;
}

__weak void APP_GET_WATCHDOG_TIMER(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	/* Check FRB2 watchdog timer status */
	if (frb2_wdt_tid != NULL && strcmp(k_thread_state_str(frb2_wdt_tid), "dead") != 0) {
		msg->data[0] = timerUse | BIT(6);
	} else {
		msg->data[0] = timerUse;
	}

	/* Fill data from the request of Set Watchdog Timer command */
	msg->data[1] = timerAction;
	msg->data[2] = preTimeoutInSec;
	msg->data[3] = timerUseExpireFlags;
	msg->data[4] = initCountdownValueLsb;
	msg->data[5] = initCountdownValueMsb;

	/* Fill present countdown value */
	msg->data[6] = (uint8_t)(presentCountdownValue & 0x00FF); // LSB
	msg->data[7] = (uint8_t)((presentCountdownValue & 0xFF00) >> 8); // MSB

	msg->data_len = 8;
	msg->completion_code = CC_SUCCESS;

	return;
}

void IPMI_APP_handler(ipmi_msg *msg)
{
	CHECK_NULL_ARG(msg);

	switch (msg->cmd) {
	case CMD_APP_GET_DEVICE_ID:
		APP_GET_DEVICE_ID(msg);
		break;
	case CMD_APP_COLD_RESET:
		APP_COLD_RESET(msg);
		break;
	case CMD_APP_WARM_RESET:
		APP_WARM_RESET(msg);
		break;
	case CMD_APP_GET_SELFTEST_RESULTS:
		APP_GET_SELFTEST_RESULTS(msg);
		break;
	case CMD_APP_MASTER_WRITE_READ:
		APP_MASTER_WRITE_READ(msg);
		break;
	case CMD_APP_GET_SYSTEM_GUID:
		APP_GET_SYSTEM_GUID(msg);
		break;
	case CMD_APP_GET_BMC_GLOBAL_ENABLES:
		APP_GET_BMC_GLOBAL_ENABLES(msg);
		break;
	case CMD_APP_SET_ACPI_POWER:
		APP_SET_ACPI_POWER(msg);
		break;
	case CMD_APP_CLEAR_MESSAGE_FLAGS:
		APP_CLEAR_MESSAGE_FLAGS(msg);
		break;
	case CMD_APP_GET_CAHNNEL_INFO:
		APP_GET_CAHNNEL_INFO(msg);
		break;
	case CMD_APP_RESET_WATCHDOG_TIMER:
		APP_RESET_WATCHDOG_TIMER(msg);
		break;
	case CMD_APP_SET_WATCHDOG_TIMER:
		APP_SET_WATCHDOG_TIMER(msg);
		break;
	case CMD_APP_GET_WATCHDOG_TIMER:
		APP_GET_WATCHDOG_TIMER(msg);
		break;
	default:
		LOG_ERR("Invalid APP msg netfn: %x, cmd: %x", msg->netfn, msg->cmd);
		msg->data_len = 0;
		msg->completion_code = CC_INVALID_CMD;
		break;
	}

	return;
}
